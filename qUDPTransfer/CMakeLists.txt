# AND ADAPT THE CODE BELOW TO YOUR OWN NEEDS!

# Add an option to CMake to control whether we build this plugin or not
option( PLUGIN_STANDARD_QUDP_TRANSFER "Install qUDPTransfer plugin" OFF )

if ( PLUGIN_STANDARD_QUDP_TRANSFER )
	project( QUDP_TRANSFER )
	 
	find_package(Qt5 COMPONENTS Network REQUIRED)
	AddPlugin( NAME ${PROJECT_NAME} )
		
	include(FileGLOBBER)
	
	target_link_libraries( ${PROJECT_NAME} Qt5::Network )
endif()
