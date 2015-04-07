
solution "libdrone"
    configurations {"Debug", "Release"}
    location "build"

project "drone"
    kind "SharedLib"
    language "C++"
    location "build/libdrone"
    
    includedirs {"include"}
    files {"src/**.h", "src/**.cpp"}