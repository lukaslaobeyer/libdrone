
solution "libdrone"
    configurations {"Debug", "Release"}
    location "build"

project "drone"
    kind "SharedLib"
    language "C++"
    location "build/libdrone"
    
    -- Enable C++11
    buildoptions { "-std=c++11" }
    
    -- Include third-party libraries
    includedirs {"/usr/include/eigen3"}
    
    -- Source files and library headers
    includedirs {"include"}
    files {"src/**.h", "src/**.cpp"}
    
    -- Link libraries
    links {"eigen3"}