SET(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:/usr/local/lib/pkgconfig:/opt/local/lib/pkgconfig:/usr/local/share/pkgconfig")

pods_install_pkg_config_file(rrtstar-standalone
    CFLAGS
    LIBS  -lrrtstar-standalone
    VERSION 0.0.1)



add_executable(rrtstar rrts_main.cpp system_single_integrator.cpp kdtree.c)

pods_use_pkg_config_packages(rrtstar-standalone)

pods_install_executables(rrtstar-standalone)