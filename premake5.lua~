solution "libdrone"
    configurations {"Debug", "Release"}
    location "build"

    configuration "Debug"
        flags { "Symbols" }

project "drone"
    kind "SharedLib"
    language "C++"
    location "build/libdrone"

    -- Enable C++11
    buildoptions { "-std=c++11" }

    -- Include third-party libraries
    includedirs
    {
        "/opt/ffmpeg/include",
        "/opt/opencv/include",
        "/usr/include/eigen3"
    }

    -- Source files and library headers
    includedirs { "include" }
    files { "src/**.h", "src/**.cpp" }

    -- Link libraries
    libdirs
    {
        "/opt/ffmpeg/lib/",
        "/opt/opencv/lib/"
    }

    links
    {
        "boost_system", "boost_thread", "boost_timer",
        "opencv_core", "opencv_highgui", "opencv_imgcodecs",
        "avcodec", "avutil", "avformat", "swresample", "swscale"
    }

project "example-basic"
    kind "ConsoleApp"
    language "C++"
    location "build/examples/basic"

    -- Enable C++11
    buildoptions { "-std=c++11" }

    -- Include third-party libraries
    includedirs
    {
        "/opt/ffmpeg/include",
        "/opt/opencv/include",
        "/usr/include/eigen3"
    }

    -- Include libdrone
    includedirs { "include" }

    -- Source files
    files { "examples/basic/**.h", "examples/basic/**.cpp" }

    -- Link libraries
    libdirs
    {
        "/opt/ffmpeg/lib/",
        "/opt/opencv/lib/"
    }

    links
    {
        "boost_system", "boost_thread", "boost_timer",
        "opencv_core", "opencv_highgui", "opencv_imgcodecs",
        "avcodec", "avutil", "avformat", "swresample", "swscale",
        "drone"
    }
