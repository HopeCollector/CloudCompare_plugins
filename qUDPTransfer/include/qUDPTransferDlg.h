#pragma once
#include "ui_qUDPTransfer.h"
#include "qUDPPointCloud.h"

#include <ccMainAppInterface.h>
#include <ccGLWindow.h>
#include <ccPointCloud.h>
#include <ccColorScale.h>
#include <ccHObject.h>

#include <QMainWindow>
#include <QFile>
#include <QNetworkDatagram>
#include <QUdpSocket>
#include <QtConcurrent>
#include <QColor>


class qUDPTransferDlg : public QDialog, public Ui::UDPTransferDialog
{
	Q_OBJECT

public:
	explicit qUDPTransferDlg(ccMainAppInterface* app = nullptr);
	~qUDPTransferDlg();
	void dataProcessor();

public slots:
	void dispToConsole(const QString& msg);
	void onStartStopButtonClick();
	void onUpdateAddressButtonClick();

private:
	ccMainAppInterface* m_app;
	ccHObject* m_container;
	qUDPPointCloud* m_pointCloud;
	QUdpSocket* m_socket;
	QHostAddress m_scannerIP;
	quint16 m_scannerPort;
	ccColorScale::Shared m_colorScale;
	bool isListening;
};