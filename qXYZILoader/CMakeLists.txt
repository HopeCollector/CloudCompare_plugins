option( PLUGIN_IO_XYZI "Install xyzi I/O plugin" OFF )

if( PLUGIN_IO_XYZI )
	project( XYZILoaderIOPlugin )

    AddPlugin( NAME ${PROJECT_NAME} TYPE io )
    
    include(FileGLOBBER)
endif()
