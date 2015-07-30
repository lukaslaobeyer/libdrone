solution "libdrone"
    configurations { "Debug", "Release" }
    platforms { "linux", "mingw" }
    location "build"

    configuration "Debug"
        flags { "Symbols" }

project "drone"
    kind "SharedLib"
    language "C++"
    location "build/libdrone"

    -- Enable C++11
    buildoptions { "-std=c++11", "-DBOOST_LOG_DYN_LINK" }

    -- Source files and library headers
    includedirs { "include" }
    files { "src/**.h", "src/**.cpp" }

    -- For Linux
    configuration { "linux" }
        -- Include third-party libraries
        includedirs
        {
            "/opt/ffmpeg/include",
            "/opt/opencv/include",
            "/usr/include/eigen3"
        }

        -- Link libraries
        libdirs
        {
            "/opt/ffmpeg/lib/",
            "/opt/opencv/lib/"
        }

        links
        {
            "boost_system", "boost_thread", "boost_timer", "boost_filesystem", "boost_log", "boost_date_time",
            "opencv_core", "opencv_highgui", "opencv_imgcodecs", "opencv_imgproc",
            "avcodec", "avutil", "avformat", "swresample", "swscale"
        }

    -- For MinGW on Windows
    configuration { "mingw" }
        -- Include third-party libraries
        includedirs
        {
            "C:/boost/include/boost-1_58",
            "C:/ffmpeg/include",
            "C:/opencv/include",
            "C:/eigen3/include/eigen3"
        }

        -- Link libraries
        libdirs
        {
            "C:/boost/lib",
            "C:/ffmpeg/lib/",
            "C:/opencv/x86/mingw/lib"
        }

        links
        {
            "ws2_32",
            "boost_system-mgw49-mt-1_58", "boost_thread-mgw49-mt-1_58", "boost_timer-mgw49-mt-1_58", "boost_filesystem-mgw49-mt-1_58", "boost_chrono-mgw49-mt-1_58", "boost_log-mgw49-mt-1_58", "boost_date_time-mgw49-mt-1_58",
            "opencv_core300", "opencv_highgui300", "opencv_imgcodecs300", "opencv_imgproc300",
            "avcodec", "avutil", "avformat", "swresample", "swscale"
        }

project "example-basic"
    kind "ConsoleApp"
    language "C++"
    location "build/examples/basic"

    -- Enable C++11
    buildoptions { "-std=c++11", "-DBOOST_LOG_DYN_LINK" }

    -- Include libdrone
    includedirs { "include" }

    -- Source files
    files { "examples/basic/**.h", "examples/basic/**.cpp" }

    configuration { "linux" }
        -- Include third-party libraries
        includedirs
        {
            "/opt/ffmpeg/include",
            "/opt/opencv/include",
            "/usr/include/eigen3"
        }

        -- Link libraries
        libdirs
        {
            "/opt/ffmpeg/lib/",
            "/opt/opencv/lib/"
        }

        links
        {
            "boost_system", "boost_thread", "boost_timer", "boost_log", "boost_date_time",
            "opencv_core", "opencv_highgui", "opencv_imgcodecs", "opencv_imgproc",
            "avcodec", "avutil", "avformat", "swresample", "swscale",
            "drone"
        }

    -- For MinGW on Windows
    configuration { "mingw" }
        -- Include third-party libraries
        includedirs
        {
            "C:/boost/include/boost-1_58",
            "C:/ffmpeg/include",
            "C:/opencv/include",
            "C:/eigen3/include/eigen3"
        }

        -- Link libraries
        libdirs
        {
            "C:/boost/lib",
            "C:/ffmpeg/lib/",
            "C:/opencv/x86/mingw/lib"
        }

        links
        {
            "ws2_32",
            "boost_system-mgw49-mt-1_58", "boost_thread-mgw49-mt-1_58", "boost_timer-mgw49-mt-1_58", "boost_log-mgw49-mt-1_58", "boost_date_time-mgw49-mt-1_58",
            "opencv_core300", "opencv_highgui300", "opencv_imgcodecs300", "opencv_imgproc300",
            "avcodec", "avutil", "avformat", "swresample", "swscale",
            "drone"
        }
