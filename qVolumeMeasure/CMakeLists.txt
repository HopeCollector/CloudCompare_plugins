# AND ADAPT THE CODE BELOW TO YOUR OWN NEEDS!

# Add an option to CMake to control whether we build this plugin or not
option( PLUGIN_STANDARD_QVOLUME_MEASURE "Install qVolumeMeasure plugin" OFF )

if ( PLUGIN_STANDARD_QVOLUME_MEASURE )
	project( QVOLUME_MEASURE )
	
	find_package(Qt5 COMPONENTS AxContainer REQUIRED)
	AddPlugin( NAME ${PROJECT_NAME} )
		
	include(FileGLOBBER)
	target_link_libraries( ${PROJECT_NAME} Qt5::AxContainer )
endif()
