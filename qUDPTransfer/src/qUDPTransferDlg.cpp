#include "qUDPTransferDlg.h"

qUDPTransferDlg::qUDPTransferDlg(ccMainAppInterface* app)
	: QDialog(app ? app->getMainWindow() : nullptr, Qt::WindowCloseButtonHint)
	, Ui::UDPTransferDialog()
	, m_app(app)
	, m_pointCloud(new qUDPPointCloud)
	, m_container(new ccHObject)
	, m_socket(new QUdpSocket)
	, m_scannerIP("255.255.255.255")
	, m_scannerPort(7878)
	, m_colorScale(ccColorScale::Create("Temp Scale"))
	, isListening(false)
{
	setupUi(this);

	// config point cloud
	{
		m_container->setName(QString("UDP Receive (%1)")
			.arg(QDateTime::currentDateTime().toString()));
		m_pointCloud->setName((QString("Point Cloud (%1)")
			.arg(QTime::currentTime().toString())));
		m_pointCloud->setGlobalScale(25.0);
		m_container->addChild(m_pointCloud);
		m_app->addToDB(m_container);
	}

	// config socket
	{
		try
		{
			m_socket->bind(7878); // FIXME: AnyIpv4 have big problem!!!! app cannot receive data!!!!!!!
		}
		catch (const std::exception&)
		{
			dispToConsole("Invalid IP or Port");
			return;
		}
		IPInput->setText("255.255.255.255");
		PortInput->setText("7878");
	}

	// connect signals
	{
		connect(StartStopButton, &QPushButton::clicked,
			this, &qUDPTransferDlg::onStartStopButtonClick);
		connect(UpdateAddressButton, &QPushButton::clicked,
			this, &qUDPTransferDlg::onUpdateAddressButtonClick);
		connect(m_socket, &QUdpSocket::readyRead,
			this, &qUDPTransferDlg::dataProcessor);
	}

	// init color scale
	{
		m_colorScale->insert(ccColorScaleElement(0.0, QColor("black")));
		m_colorScale->insert(ccColorScaleElement(1.0, QColor("white")));
		m_colorScale->insert(ccColorScaleElement(0.1, QColor("red")));
		m_colorScale->insert(ccColorScaleElement(0.25, QColor("green")));
		m_colorScale->insert(ccColorScaleElement(0.75, QColor("blue")));
	}
}

qUDPTransferDlg::~qUDPTransferDlg()
{
	m_socket->disconnectFromHost();
	delete m_socket;

	if (m_pointCloud->size() <= 100)
	{
		m_app->removeFromDB(m_container);
		m_container = nullptr;
		m_pointCloud = nullptr;
	}

	/*m_pointCloud->showColors(true);
	m_pointCloud->setRGBColorByHeight(2, m_colorScale);*/
}

void qUDPTransferDlg::dataProcessor()
{
	while (isListening && m_socket->hasPendingDatagrams())
	{
		/*QtConcurrent::run(m_pointCloud,
			&qUDPPointCloud::addPoints,
			m_socket->receiveDatagram().data());*/
		auto datagram = m_socket->receiveDatagram();
		m_pointCloud->addPoints(datagram.data());
	}
	m_pointCloud->refreshDisplay();
	dispToConsole(QString("current point size: %1")
		.arg(m_pointCloud->size()));
}

void qUDPTransferDlg::dispToConsole(const QString& msg)
{
	m_app->dispToConsole(msg);
}

void qUDPTransferDlg::onStartStopButtonClick()
{
	 //m_pointCloud->addPoints(QStringLiteral("D:\\200_work\\cc\\pcData\\points.xyzi"));
	if (isListening)
	{
		isListening = false;
		StartStopButton->setText("Start");
		QByteArray ba("Start work");
		if (m_socket->writeDatagram(ba, QHostAddress(m_scannerIP), m_scannerPort) > 0)
		{
			m_app->dispToConsole("Command send success!");
		}
		
	}
	else
	{
		isListening = true;
		StartStopButton->setText("Stop");
		QByteArray ba("Stop work");
		if (m_socket->writeDatagram(ba, QHostAddress(m_scannerIP), m_scannerPort) > 0)
		{
			m_app->dispToConsole("Command send success!");
		}
	}
}

void qUDPTransferDlg::onUpdateAddressButtonClick()
{
	m_scannerIP = QHostAddress(IPInput->text());
	m_scannerPort = PortInput->text().toUInt();
	dispToConsole("Scanner address update success!");
}
