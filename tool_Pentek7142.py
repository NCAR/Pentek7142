import os
import sys

import eol_scons

# Any environment setting will be the default, but it can be overridden by
# setting the configuration variable.

variables = eol_scons.GlobalVariables()

tools = Split("""
boost_thread
logx
ReadyFlow7142_428
doxygen
""")

env = Environment(tools = ['default'] + tools)

variables.Update(env)

libsources = Split("""
p7142.cpp
p7142Dn.cpp
p7142Up.cpp
p7142sd3c.cpp
p7142sd3cDn.cpp
FilterSpec.cpp
BuiltinFilters.cpp
BuiltinGaussian.cpp
BuiltinKaiser.cpp
SingleMutex.cpp
""")

headers = Split("""
p7142.h
p7142Dn.h
p7142Up.h
p7142sd3c.h
p7142sd3cDn.h
BuiltinFilters.h
BuiltinGaussian.h
BuiltinKaiser.h
FilterSpec.h
DDCregisters.h
SingleMutex.h
""")

libpentek = env.Library('Pentek7142', libsources)
Default(libpentek)

env['DOXYFILE_DICT'].update({'PROJECT_NAME':'Pentek7142'})
html = env.Apidocs(libsources + headers)
Default(html)

thisdir = env.Dir('.').srcnode().abspath
def Pentek7142(env):
    env.AppendUnique(CPPPATH = [thisdir])
    env.AppendLibrary('Pentek7142')
    env.AppendDoxref('Pentek7142')
    env.Require(tools)

Export('Pentek7142')

# Add build of the test programs before we exit...
SConscript("test/SConscript")
