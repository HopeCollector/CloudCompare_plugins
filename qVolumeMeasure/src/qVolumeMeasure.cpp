//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: qVolumeMeasure                      #
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
//	Replace all occurrences of 'qVolumeMeasure' by your own plugin class name in this file.
//	This includes the resource path to info.json in the constructor.

// Second:
//	Open qVolumeMeasure.qrc, change the "prefix" and the icon filename for your plugin.
//	Change the name of the file to <yourPluginName>.qrc

// Third:
//	Open the info.json file and fill in the information about the plugin.
//	 "type" should be one of: "Standard", "GL", or "I/O" (required)
//	 "name" is the name of the plugin (required)
//	 "icon" is the Qt resource path to the plugin's icon (from the .qrc file)
//	 "description" is used as a tootip if the plugin has actions and is displayed in the plugin dialog
//	 "authors", "maintainers", and "references" show up in the plugin dialog as well

#include <QtGui>

#include "qVolumeMeasure.h"

// Default constructor:
//	- pass the Qt resource path to the info.json file (from <yourPluginName>.qrc file) 
//  - constructor should mainly be used to initialize actions and other members
qVolumeMeasure::qVolumeMeasure( QObject *parent )
	: QObject( parent )
	, ccStdPluginInterface( ":/CC/plugin/qVolumeMeasure/info.json" )
	, m_action( nullptr )
{
}

void qVolumeMeasure::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
	{
		//a single point cloud must be selected
		m_action->setEnabled(selectedEntities.size() == 1 && selectedEntities.front()->isA(CC_TYPES::POINT_CLOUD));
	}
}

// This method returns all the 'actions' your plugin can perform.
// getActions() will be called only once, when plugin is loaded.
QList<QAction *> qVolumeMeasure::getActions()
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
		connect( m_action, &QAction::triggered, this, &qVolumeMeasure::doAction);
	}

	return { m_action };
}

void qVolumeMeasure::doAction()
{
	ccPointCloud * cloud = dynamic_cast<ccPointCloud*>(m_app->getSelectedEntities().front());

	m_app->dispToConsole("measure volume v0.1");

	qVolumeMeasureDlg dlg(m_app);

	dlg.show();
	QCoreApplication::processEvents();

	m_app->setSelectedInDB(cloud, false);
	dlg.setCloud(cloud);
	dlg.exec();

	m_app->refreshAll();
}
