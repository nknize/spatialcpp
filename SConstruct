# -*- mode: python; -*-

EnsureSConsVersion(0, 98, 4) # this is a common version known to work

import os
import sys
import imp
import types
import re
import shutil
import urllib
import urllib2
import stat

options = {}
options_topass = {}

def add_option( name, help , nargs , contibutesToVariantDir , dest=None ):

    if dest is None:
        dest = name

    AddOption( "--" + name ,
               dest=dest ,
               type="string" ,
               nargs=nargs ,
               action="store" ,
               help=help )

    options[name] = { "help" : help ,
                      "nargs" : nargs ,
                      "contibutesToVariantDir" : contibutesToVariantDir ,
                      "dest" : dest }

def get_option( name ):
    return GetOption( name )

def _has_option( name ):
    x = get_option( name )
    if x is None:
        return False
    if x == False:
        return False
    if x == "":
        return False
    return True

def has_option( name ):
    x = _has_option(name)
    options_topass[name] = x
    return x

add_option( "release", "release build", 0, True )
add_option( "static", "fully static build", 0, False )
add_option( "dynamic", "dynamic library build", 1, True )

add_option( "64", "whether to force 64 bit", 1, True, "force64" )
add_option( "32", "whether to force 32 bit", 0, True, "force32" )

add_option( "extrapath", "comma separated list of add'l paths  (--extrapath /opt/foo/,/foo) static linking" , 1 , True )
add_option( "extrapathdyn", "comma separated list of add'l paths  (--extrapath /opt/foo/,/foo) dynamic linking" , 1 , True )
add_option( "extralib", "comma separated list of libraries  (--extralib js_static,readline" , 1 , True )
add_option( "staticlib", "comma separated list of libs to link statically (--staticlib js_static,boost_program_options-mt,..." , 1 , True )
add_option( "staticlibpath", "comma separated list of dirs to search for staticlib arguments" , 1 , True )


add_option( "boost-compiler", "compiler used for boost (gcc41)", 1, True, "boostCompiler" )
add_option( "boost-version", "boost version for linking(1_38)", 1, True, "boostVersion" )

add_option( "cxx", "compiler to use", 1, True )
add_option( "cc", "compiler to use for c", 1, True)

add_option( "distcc", "use distcc for distributing builds", 0, False )
add_option( "clang", "use clang++ rather than g++ (experimental)", 0, True)

boostLibs = [ "geometry" , "program_options" ]

def removeIfInList( lst , thing ):
    if thing in lst:
        lst.remove( thing )

nix = False
linux = False
linux64 = False
darwin = False
windows = False
freebsd = False
openbsd = False
solaris = False

force64 = has_option( "force64" )
msarch = None
if force64:
    msarch = "amd64"

force32 = has_option( "force32" )
release = has_option( "release" )
static = has_option( "static" )
dynamic = has_option( "dynamic" )

env = Environment( MSVS_ARCH=msarch , tools = ["default"], toolpath = '.' )

if has_option( "cxx" ):
    env["CC"] = get_option( "cxx" )
    env["CXX"] = get_option( "cxx" )

if has_option( "cc" ):
    env["CC"] = get_option( "cc" )

boostCompiler = GetOption( "boostCompiler" )

if boostCompiler is None:
    boostCompiler = ""
else:
    boostCompiler = "-" + boostCompiler

boostVersion = GetOption( "boostVersion" )
if boostVersion is None:
    boostVersion = ""
else:
    boostVersion = "-" + boostVersion

# ----- SOURCE FILES ------
coreLibraryFiles = Split( "foundation/ellipsoid.cpp" )

def filterExists(paths):
    return filter(os.path.exists, paths)

if "darwin" == os.sys.platform:
    darwin = True
    platform = "osx" # prettier than darwin

    if env["CXX"] is None:
        if os.path.exists( "/usr/bin/g++-4.2" ):
            env["CXX"] = "g++-4.2"

    nix = True

    if force64:
        env.Append( CPPPATH=["/usr/64/include"] )
        env.Append( LIBPATH=["/usr/64/lib"] )
        if installDir == DEFAULT_INSTALL_DIR and not distBuild:
            installDir = "/usr/64/"
    else:
        env.Append( CPPPATH=filterExists(["/sw/include" , "/opt/local/include"]) )
        env.Append( LIBPATH=filterExists(["/sw/lib/", "/opt/local/lib"]) )

elif "linux2" == os.sys.platform or "linux3" == os.sys.platform:
    linux = True
    platform = "linux"

    if os.uname()[4] == "x86_64" and not force32:
        linux64 = True
        nixLibPrefix = "lib64"
        env.Append( LIBPATH=["/usr/lib64" , "/lib64" ] )

        force64 = False

    if force32:
        env.Append( LIBPATH=["/usr/lib32"] )

    nix = True

    if static:
        env.Append( LINKFLAGS=" -static " )
    else:
        env.Append( LINKFLAGS=" -shared " )

if nix:

    if has_option( "distcc" ):
        env["CSS"] = "distcc " + env["CXX"]

    env.Append( CPPFLAGS="-O0 -std=c++11 -rdynamic -fPIC -fno-omit-frame-pointer -fno-strict-aliasing -lm -ggdb -gdwarf-2 -Wall -Werror -Wsign-compare -Wno-unknown-pragmas -Winvalid-pch" )

    if linux:
        env.Append( CPPFLAGS=" -Werror " )

        env.Append( LINKFLAGS=" -fPIC -rdynamic" )
        env.Append( LIBS=[] )

        # color gcc friendly
        for key in ('HOME', 'TERM'):
            try:
                env['ENF'][key] = os.environ[key]
            except KeyError:
                pass

        if force64:
            env.Append( CFLAGS="-m64" )
            env.Append( CXXFLAGS="-m64" )
            env.Append( LINKFLAGS="-m64" )
        if darwin:
            env.Append( LINKFLAGS="-stdlib=libc++")

        if dynamic:
            env.Append( LINKFLAGS=" -shared " )

extraLibPlaces = []

def addExtraLibs( s ):
    for x in s.split(","):
        env.Append( CPPPATH=[ x + "/include" ] )
        env.Append( LIBPATH=[ x + "/lib" ] )
        env.Append( LIBPATH=[ x + "/lib64" ] )
        extraLibPlaces.append( x + "/lib" )

if has_option( "extrapath" ):
    addExtraLibs( GetOption( "extrapath" ) )
    release = True # this is so we force using .a

if has_option( "extrapathdyn" ):
    addExtraLibs( GetOption( "extrapathdyn" ) )

if has_option( "extralib" ):
    for x in GetOption( "extralib" ).split( "," ):
        env.Append( LIBS=[ x ] )

modules = []
moduleNames = []

#for x in os.listdir( "db/modules/" ):
#    if x.find( "." ) >= 0:
#        continue
#    print( "adding module: " + x )
#    moduleNames.append( x )
#    modRoot = "db/modules/" + x + "/"
#
#    modBuildFile = modRoot + "build.py"
#    myModule = None
#    if os.path.exists( modBuildFile ):
#        myModule = imp.load_module( "module_" + x , open( modBuildFile , "r" ) , modBuildFile , ( ".py" , "r" , imp.PY_SOURCE  ) )
#        modules.append( myModule )
#                                        
#    if myModule and "customIncludes" in dir(myModule) and myModule.customIncludes:
#        pass    

def doConfigure( myenv, shell=False ):
    conf = Configure(myenv)
    myenv["LINKFLAGS_CLEAN"] = list( myenv["LINKFLAGS"] )
    myenv["LIBS_CLEAN"] = list( myenv["LIBS"] )

    if 'CheckCXX' in dir( conf ):
        if not conf.CheckCXX():
            print( "c++ compiler not installed!" )
            Exit(1)
#    if nix and not shell:
#        if not darwin and not conf.CheckLib( "stdc++" ):
#            print( "can't find stdc++ library which is needed" );
#            Exit(1)

    def myCheckLib( poss , failIfNotFound=False , staticOnly=False):

        if type( poss ) != types.ListType :
            poss = [poss]

        allPlaces = [];
        allPlaces += extraLibPlaces
        if nix and release:
            allPlaces += myenv["LIBPATH"]
            if not force64:
                allPlaces += [ "/usr/lib" , "/usr/local/lib" ]

            for p in poss:
                for loc in allPlaces:
                    fullPath = loc + "/lib" + p + ".a"
                    if os.path.exists( fullPath ):
                        myenv['_LIBFLAGS']='${_stripixes(LIBLINKPREFIX, LIBS, LIBLINKSUFFIX, LIBPREFIXES, LIBSUFFIXES, __env__)} $SLIBS'
                        myenv.Append( SLIBS=" " + fullPath + " " )
                        return True

        if release and not windows and failIfNotFound:
            print( "ERROR: can't find static version of: " + str( poss ) + " in: " + str( allPlaces ) )
            Exit(1)

        res = not staticOnly and conf.CheckLib( poss )
        if res:
            return True

        if failIfNotFound:
            print( "can't find or link against library " + str( poss ) + " in " + str( myenv["LIBPATH"] ) )
            print( "see config.log for more information" )
        if windows:
            print( "use scons --64 when cl.exe is 64 bit compiler" )
            Exit(1)

        return False

    if not darwin and not conf.CheckCXXHeader( "boost/filesystem/operations.hpp" ):
        print( "can't find boost headers" )
        if shell:
            print( "\tshell might not compile" )
        else:
            Exit(1)

    # this will add it if it exists and works
    myCheckLib( [ "boost_system" + boostCompiler + "-mt" + boostVersion ,
                  "boost_system" + boostCompiler + boostVersion ] )
    
    for b in boostLibs:
        l = "boost_" + b
        myCheckLib( [ l + boostCompiler + "-mt" + boostVersion ,
                      l + boostCompiler + boostVersion ] ,
                    release or not shell)

    if not conf.CheckCXXHeader( "execinfo.h" ):
        myenv.Append( CPPDEFINES=[ "NOEXECINFO" ] )

    myenv["_HAVEPCAP"] = myCheckLib( ["pcap", "wpcap"] )
    removeIfInList( myenv["LIBS"] , "pcap" )
    removeIfInList( myenv["LIBS"] , "wpcap" )

    for m in modules:
        if "customIncludes" in dir(m) and m.customIncludes:
            m.configure( conf , myenv , serverOnlyFiles )
        else:
            m.configure( conf , myenv )

    staticlibfiles = []
    myenv.Append(LINKCOM=" $STATICFILES")
    myenv.Append(STATICFILES=staticlibfiles)

    return conf.Finish()
env.Append( LIBS=[] )
env = doConfigure( env )


# main library target
env.SharedLibrary( "spatialcpp", coreLibraryFiles )
