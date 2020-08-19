//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: qUDPTransfer                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                             COPYRIGHT: XXX                             #
//#                                                                        #
//##########################################################################

// First:
//	Replace all occurrences of 'qUDPTransfer' by your own plugin class name in this file.
//	This includes the resource path to info.json in the constructor.

// Second:
//	Open qUDPTransfer.qrc, change the "prefix" and the icon filename for your plugin.
//	Change the name of the file to <yourPluginName>.qrc

// Third:
//	Open the info.json file and fill in the information about the plugin.
//	 "type" should be one of: "Standard", "GL", or "I/O" (required)
//	 "name" is the name of the plugin (required)
//	 "icon" is the Qt resource path to the plugin's icon (from the .qrc file)
//	 "description" is used as a tootip if the plugin has actions and is displayed in the plugin dialog
//	 "authors", "maintainers", and "references" show up in the plugin dialog as well

#include <QtGui>

#include "qUDPTransfer.h"
#include "qUDPTransferDlg.h"

// Default constructor:
//	- pass the Qt resource path to the info.json file (from <yourPluginName>.qrc file) 
//  - constructor should mainly be used to initialize actions and other members
qUDPTransfer::qUDPTransfer( QObject *parent )
	: QObject( parent )
	, ccStdPluginInterface( ":/CC/plugin/qUDPTransfer/info.json" )
	, m_action( nullptr )
{
}

// This method returns all the 'actions' your plugin can perform.
// getActions() will be called only once, when plugin is loaded.
QList<QAction *> qUDPTransfer::getActions()
{
	// default action (if it has not been already created, this is the moment to do it)
	if ( !m_action )
	{
		// Here we use the default plugin name, description, and icon,
		// but each action should have its own.
		m_action = new QAction( getName(), this );
		m_action->setToolTip( getDescription() );
		m_action->setIcon( getIcon() );
		
		// Connect appropriate signal
		connect( m_action, &QAction::triggered, this, &qUDPTransfer::doAction);
	}

	return { m_action };
}

void qUDPTransfer::doAction()
{
	m_app->dispToConsole("udp enable!");

	qUDPTransferDlg udpTransferDlg(m_app);

	udpTransferDlg.show();
	QCoreApplication::processEvents();

	udpTransferDlg.exec();

	m_app->refreshAll();
}
