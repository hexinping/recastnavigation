--
-- Add by yaukey at 2019-02-19.
-- premake5 file to build NavMeshExporter.
-- http://premake.github.io/
--

local action = _ACTION or ""
local todir = "BuildNavMeshExporter/" .. action

solution "NavMeshExporter"
	configurations { 
		"Debug",
		"Release"
	}

	location (todir)

	floatingpoint "Fast"
	symbols "On"
	exceptionhandling "Off"
	rtti "Off"
	flags { "FatalCompileWarnings" }

	-- debug configs
	configuration "Debug*"
		defines { "DEBUG" }
		targetdir ( todir .. "/lib/Debug" )
 
 	-- release configs
	configuration "Release*"
		defines { "NDEBUG" }
		optimize "On"
		targetdir ( todir .. "/lib/Release" )

	configuration "not windows"
		warnings "Extra"

	-- windows specific
	configuration "windows"
		platforms { "x86", "x64" }
		defines { "WIN32", "_WINDOWS", "_CRT_SECURE_NO_WARNINGS", "_HAS_EXCEPTIONS=0", "NAVMESH_EXPORT_API" }
		-- warnings "Extra" uses /W4 which is too aggressive for us, so use W3 instead.
		-- Disable:
		-- * C4351: new behavior for array initialization
		buildoptions { "/W3", "/wd4351" }

	filter "platforms:x86"
		architecture "x86"

	filter "platforms:x64"
		architecture "x64"

project "NavMeshExporter"
    language "C++"
    kind "SharedLib"
	targetdir "Bin"

	includedirs
    {
        "../Detour/Include",
        "../Recast/Include",
        "../NavMeshExporter"
    }

    files
    {
        "../Detour/Include/*.h",
        "../Detour/Source/*.cpp",
        "../Recast/Include/*.h",
        "../Recast/Source/*.cpp",
        "../NavMeshExporter/*.h",
        "../NavMeshExporter/*.cpp",
    }
