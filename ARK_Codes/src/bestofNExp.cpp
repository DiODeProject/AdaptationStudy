#include "bestofNExp.h"
#include "bestofNEnv.h"
#include <QDebug>
#include <QThread>
#include <QTime>
#include <time.h>

// widgets
#include <QPushButton>
#include <QSlider>
#include <QGroupBox>
#include <QFormLayout>
#include <QLabel>
#include <QFrame>
#include <QCheckBox>
#include <QtMath>
#include <QSpinBox>
#include <QDir>
#include <QFileDialog>
#include <QSettings>
#include <QTableWidgetItem>
#include <QPainter>
#include <QComboBox>
#include <QVector3D>


// return pointer to interface!
// mykilobotexperiment can and should be completely hidden from the application
extern "C" BESTOFNEXPSHARED_EXPORT KilobotExperiment *createExpt()
{
    return new mykilobotexperiment();
}

mykilobotexperiment::mykilobotexperiment()
{
    // setup the environments here
    connect(&m_optionsEnv,SIGNAL(transmitKiloState(kilobot_message)), this, SLOT(signalKilobotExpt(kilobot_message)));
    connect(&m_DataRetrieval,SIGNAL(broadcastMessage(kilobot_broadcast)),this,SLOT(SendDataRetrievalProcessMessage(kilobot_broadcast)));
    connect(&m_DataRetrieval,SIGNAL(DataRetrievalProcessEnded()),this,SLOT(toggleDataRetrievalProcessEnded()));
    connect(&m_DataRetrieval,SIGNAL(SeekInformationAboutKilobotResponses(std::vector<lightColour>&)),this,SLOT(InformDataRetrievalProcessAboutRobotsColors(std::vector<lightColour>&)));
    this->serviceInterval = 100; // timestep in ms?
}

// GUI creation function: Working properly

QWidget * mykilobotexperiment::createGUI() {
    QFrame * frame = new QFrame;
    QVBoxLayout * lay = new QVBoxLayout;
    frame->setLayout(lay);

    /** Eperiment recording check box */
    QCheckBox * saveImages_ckb = new QCheckBox("Record experiment");
    saveImages_ckb->setChecked(true);// make the box not checked by default
    toggleSaveImages(saveImages_ckb->isChecked());

    lay->addWidget(saveImages_ckb);

    /** Experiment logging check box */
    QCheckBox * logExp_ckb = new QCheckBox("Log experiment");
    logExp_ckb->setChecked(true); // make the box checked by default
    toggleLogExp(logExp_ckb->isChecked());

    lay->addWidget(logExp_ckb);

    /** Experiment index specification */
    // Add a field to specify the experiment no
    QGroupBox * formGroupExpno = new QGroupBox(tr("Expno:"));
    QFormLayout * layout2 = new QFormLayout;
    formGroupExpno->setLayout(layout2);
    QSpinBox* expno_spin = new QSpinBox();
    expno_spin->setMinimum(1);
    expno_spin->setMaximum(100);
    expno_spin->setValue(1);
    layout2->addRow(new QLabel(tr("Exp no:")), expno_spin);
    setExpNumber(expno_spin->value());

    lay->addWidget(formGroupExpno);

    /** Data retrieval checkbox and frequency specification */
    // Add fields to specify data retrieval params
    QGroupBox * formGroupDataRetrieval = new QGroupBox(tr("Data Retrieval:"));
    QFormLayout * layout3 = new QFormLayout;
    formGroupDataRetrieval->setLayout(layout3);

    // add a check box to enable/disable data retrieval
    QCheckBox * logDataRetrieval_ckb = new QCheckBox("Data retrieval");
    logDataRetrieval_ckb->setChecked(false); // make the box checked by default
    layout3->addWidget(logDataRetrieval_ckb);
    toggleDataRetrieval(logDataRetrieval_ckb->isChecked());

    // add a field to specify the data retrieval period
    QSpinBox* data_retrieval_spin = new QSpinBox();
    data_retrieval_spin->setMinimum(1);
    data_retrieval_spin->setMaximum(120);
    data_retrieval_spin->setValue(10);
    layout3->addRow(new QLabel(tr("DR-Frequency:")), data_retrieval_spin);
    setDataRetrievalFrequency(data_retrieval_spin->value());

    lay->addWidget(formGroupDataRetrieval);


    /** Experiment Parameters specification */
    // Add fields to specify experiment params
    QGroupBox * formGroupExpVariables = new QGroupBox(tr("Experiment params:"));
    QFormLayout * layout4 = new QFormLayout;
    formGroupExpVariables->setLayout(layout4);

    // number of options
    QSpinBox* NumberOfOptions_spin = new QSpinBox();
    NumberOfOptions_spin->setMinimum(2);
    NumberOfOptions_spin->setMaximum(10);
    NumberOfOptions_spin->setSingleStep(1);
    NumberOfOptions_spin->setValue(m_NumberOfOptions);
    layout4->addRow(new QLabel(tr("Number of options:")), NumberOfOptions_spin);
    setNumberOfOptions(NumberOfOptions_spin->value());

    // Radius of the options
    QDoubleSpinBox* options_radius_spin = new QDoubleSpinBox();
    options_radius_spin->setMinimum(0.1);
    options_radius_spin->setMaximum(2.0);
    options_radius_spin->setSingleStep(0.05);
    options_radius_spin->setValue(m_OptionsRadius);
    layout4->addRow(new QLabel(tr("Options radius")), options_radius_spin);
    setOptionsRadius(options_radius_spin->value());

    // X offset
    QSpinBox* Xoffset_spin = new QSpinBox();
    Xoffset_spin->setMinimum(-2000);
    Xoffset_spin->setMaximum(+2000);
    Xoffset_spin->setSingleStep(1);
    Xoffset_spin->setValue(m_Xoffset);
    layout4->addRow(new QLabel(tr("X offset")), Xoffset_spin);
    setXoffset(Xoffset_spin->value());


    // Y offset
    QSpinBox* Yoffset_spin = new QSpinBox();
    Yoffset_spin->setMinimum(-2000);
    Yoffset_spin->setMaximum(+2000);
    Yoffset_spin->setSingleStep(1);
    Yoffset_spin->setValue(m_Yoffset);
    layout4->addRow(new QLabel(tr("Y offset")), Yoffset_spin);
    setYoffset(Yoffset_spin->value());

    // Polygone radius
    QDoubleSpinBox* Polygone_radius_spin = new QDoubleSpinBox();
    Polygone_radius_spin->setMinimum(0.05);
    Polygone_radius_spin->setMaximum(2.0);
    Polygone_radius_spin->setSingleStep(0.05);
    Polygone_radius_spin->setValue(m_PolygoneRadius);
    layout4->addRow(new QLabel(tr("Polygone radius")), Polygone_radius_spin);
    setPolygoneRadius(Polygone_radius_spin->value());

    // Highest quality
    QDoubleSpinBox* highest_quality_spin = new QDoubleSpinBox();
    highest_quality_spin->setMinimum(1);
    highest_quality_spin->setMaximum(10.0);
    highest_quality_spin->setSingleStep(1);
    highest_quality_spin->setValue(m_HighestQuality);
    layout4->addRow(new QLabel(tr("Highest quality")), highest_quality_spin);
    setHighestQuality(highest_quality_spin->value());

    // Difficulty
    QDoubleSpinBox* difficulty_spin = new QDoubleSpinBox();
    difficulty_spin->setMinimum(0.1);
    difficulty_spin->setMaximum(1.0);
    difficulty_spin->setSingleStep(0.1);
    difficulty_spin->setValue(m_Difficulty);
    layout4->addRow(new QLabel(tr("Difficulty")), difficulty_spin);
    setDifficulty(difficulty_spin->value());

    // Quality-noise variance
    QDoubleSpinBox* quality_noise_variance_spin = new QDoubleSpinBox();
    quality_noise_variance_spin->setMinimum(0.0);
    quality_noise_variance_spin->setMaximum(5.0);
    quality_noise_variance_spin->setSingleStep(0.5);
    quality_noise_variance_spin->setValue(m_Quality_noise_variance);
    layout4->addRow(new QLabel(tr("Noise variance:")), quality_noise_variance_spin);
    setNoise(quality_noise_variance_spin->value());

    // Trobot
    QDoubleSpinBox* Trobot_spin = new QDoubleSpinBox();
    Trobot_spin->setMinimum(2.5);
    Trobot_spin->setMaximum(30.0);
    Trobot_spin->setSingleStep(2.5);
    Trobot_spin->setValue(Trobot);
    layout4->addRow(new QLabel(tr("T_robot:")), Trobot_spin);
    setTrobot(Trobot_spin->value());


    lay->addWidget(formGroupExpVariables);


    // signal-slot connections
    connect(saveImages_ckb, SIGNAL(toggled(bool)),this, SLOT(toggleSaveImages(bool)));
    connect(logExp_ckb, SIGNAL(toggled(bool)),this, SLOT(toggleLogExp(bool)));
    connect(logDataRetrieval_ckb,SIGNAL(toggled(bool)),this,SLOT(toggleDataRetrieval(bool)));
    connect(expno_spin,SIGNAL(valueChanged(int)),this,SLOT(setExpNumber(int)));
    connect(data_retrieval_spin,SIGNAL(valueChanged(int)),this,SLOT(setDataRetrievalFrequency(int)));
    connect(NumberOfOptions_spin,SIGNAL(valueChanged(int)),this,SLOT(setNumberOfOptions(int)));
    connect(options_radius_spin,SIGNAL(valueChanged(double)),this,SLOT(setOptionsRadius(double)));
    connect(highest_quality_spin,SIGNAL(valueChanged(double)),this,SLOT(setHighestQuality(double)));
    connect(difficulty_spin,SIGNAL(valueChanged(double)),this,SLOT(setDifficulty(double)));
    connect(quality_noise_variance_spin,SIGNAL(valueChanged(double)),this,SLOT(setNoise(double)));
    connect(Trobot_spin,SIGNAL(valueChanged(double)),this,SLOT(setTrobot(double)));
    connect(Xoffset_spin,SIGNAL(valueChanged(int)),this,SLOT(setXoffset(int)));
    connect(Yoffset_spin,SIGNAL(valueChanged(int)),this,SLOT(setYoffset(int)));
    connect(Polygone_radius_spin,SIGNAL(valueChanged(double)),this,SLOT(setPolygoneRadius(double)));


    lay->addStretch();

    connect(this,SIGNAL(destroyed(QObject*)), lay, SLOT(deleteLater()));

    return frame;
}


// Initialization function: Working properly
void mykilobotexperiment::initialise(bool isResume) {


    //qDebug() << "Entered initializatioN%%%%";
    // Initialize variables
    m_DataRetrieval_ON=true;
    m_log_period=1;
    m_optionsEnv.m_TimeForUpdatingVirtualSensors=false;
    m_optionsEnv.m_fCellLength=2000.0/m_optionsEnv.m_unGpsCells;
    qDebug() <<"Cell Length: " << m_optionsEnv.m_fCellLength;

    // Generate Environments:
    setupEnvironments();

    // Initialize Kilobot States:
    if (!isResume) {
        // init stuff
        emit getInitialKilobotStates();
    } else
    {

    }

    emit setTrackingType(POS | LED | ROT);

    QThread::currentThread()->setPriority(QThread::HighestPriority);

    // Init Log File operations
    if (logExp)
    {
        if (log_file.isOpen()){
            log_file.close(); // if it was open I close and re-open it (erasing old content!! )
        }

        QString log_foldername =(log_filename_prefix.arg(m_expno));
        QString log_filename = log_foldername+log_filename_suffix;
        if(!QDir(log_foldername).exists()){
            QDir().mkdir(log_foldername);
        }
        //        QString log_filename = log_filename_prefix + "_" + QDate::currentDate().toString("yyMMdd") + "_" + QTime::currentTime().toString("hhmmss") + ".txt";
        log_file.setFileName( log_filename );
        if ( !log_file.open(QIODevice::WriteOnly) ) { // open file
            qDebug() << "ERROR(!) in opening file" << log_filename;
        } else {
            qDebug () << "Log file" << log_file.fileName() << "opened.";
            log_stream.setDevice(&log_file);
        }
    }

    savedImagesCounter = 0;
    this->m_time = 0;
    ElapsedTime.start();

    //save initial image
    if (saveImages) {
        emit saveImage(im_filename_prefix.arg(m_expno,1)+im_filename_suffix.arg(savedImagesCounter++, 5,10, QChar('0')));
    }

    //log initial state of the robots
    if (logExp)
    {
        log_stream << this->m_time;
        for (int i = 0; i < allKiloIDs.size(); ++i){
            kilobot_id kID = allKiloIDs[i];
            log_stream << "\t" << kID << "\t" << allKilos[kID].colour << "\t" << allKilos[kID].position.x() << "\t" << allKilos[kID].position.y();
        }
        log_stream << endl;
    }


    clearDrawings();
}

// Stop function working properly
void mykilobotexperiment::stopExperiment() {
    if (log_file.isOpen()){
        qDebug() << "Closing file" << log_file.fileName();
        log_file.close();
    }
    m_optionsEnv.m_Options.clear();
    m_NumberOfConfigMsgsSent=0;
    RobotSpeaking=false;
    configurationMsgsSent=false;
    m_time=0;
    Trobot_last=0;
}

void mykilobotexperiment::run()
{
    // Update Broadcasting state
    UpdateBroadcastingState();

    // Send configuration messages
    if(m_NumberOfConfigMsgsSent < m_optionsEnv.m_Options.size() && !broadcasing)
    {

        kilobot_broadcast msg;
        msg.type=10; // Configuration message
        msg.data.resize(9);
        msg.data[0]= m_optionsEnv.m_Options[m_NumberOfConfigMsgsSent].ID;
        msg.data[1]= (uint8_t) m_optionsEnv.m_Options[m_NumberOfConfigMsgsSent].GPS_X;
        msg.data[2]= (uint8_t) m_optionsEnv.m_Options[m_NumberOfConfigMsgsSent].GPS_Y;
        msg.data[3]= (uint8_t) (m_Quality_noise_variance*10);
        msg.data[4]= 16;    // max cell
        msg.data[5]= 4;    // min distance
        msg.data[6]= 5;    // number of attacker


        qDebug() << "Sending Config Msg No." << m_NumberOfConfigMsgsSent;
        qDebug() << "data[0]=" << msg.data[0];
        qDebug() << "data[1]=" << msg.data[1];
        qDebug() << "data[2]=" << msg.data[2];
        qDebug() << "data[3]=" << msg.data[3];

        emit broadcastMessage(msg);

        broadcasing=true;
        t_since=1;

        m_NumberOfConfigMsgsSent++;
    }

    if(m_NumberOfConfigMsgsSent == m_optionsEnv.m_Options.size() && !broadcasing && !configurationMsgsSent )
    {

        qDebug() << "Asking robots to start speaking!";

        kilobot_broadcast msg;
        msg.type=6; // Comeback to speaking message
        emit broadcastMessage(msg);

        broadcasing=true;
        t_since=1;

        RobotSpeaking=true;
    }


    if(configurationMsgsSent)
    {

        if(!this->runtimeIdentificationLock){

            // Increment time
            if(!m_data_retrieval_running){
                this->m_time=ElapsedTime.elapsed()/1000.0; // 100 ms in sec
            }

            // Update Kilobot States:
            emit updateKilobotStates();

            //Save image and log data once every second
            if (qRound(m_time*10.0f) % qRound(m_log_period*10.0f) == 0)

            { // every 1s
                if (saveImages) {
                    emit saveImage(im_filename_prefix.arg(m_expno,1)+im_filename_suffix.arg(savedImagesCounter++, 5,10, QChar('0')));
                }
                if (logExp)
                {
                    log_stream << this->m_time;
                    for (int i = 0; i < allKiloIDs.size(); ++i){
                        kilobot_id kID = allKiloIDs[i];
                        log_stream << "\t" << kID << "\t" << allKilos[kID].colour << "\t" << allKilos[kID].position.x() << "\t" << allKilos[kID].position.y();
                    }
                    log_stream << endl;
                }
            }

            // Check if it is time retrieve data
            if( m_DataRetrieval_ON && ((qRound(m_time*10.0f) % qRound(m_data_retrieval_period*60*10.0f) == 0) || m_data_retrieval_running) )
            {
                if(m_data_retrieval_running)
                {
                    m_DataRetrieval.run();
                }
                else
                {
                    m_DataRetrieval.initialise(allKilos.size(),m_data_retrieval_method,1,3,true);
                    m_data_retrieval_running=true;
                }
            }

            if(!m_data_retrieval_running)
            {

//                qDebug() << "m_VirtualSensorsNeedUpdate=" <<m_optionsEnv.m_VirtualSensorsNeedUpdate;
//                qDebug() << "m_time-Trobot_last=" <<m_time-Trobot_last;
//                qDebug() << "broadcasing=" <<broadcasing;


                /* Check if the robots are required to stop speaking:  time to update virtual sensors */
                if ( m_time-Trobot_last >= Trobot)
                {

                    //                    qDebug() << "*********************************************";

                    Trobot_last=m_time;

                    if((m_optionsEnv.m_VirtualSensorsNeedUpdate) && !broadcasing
                            && !StopSpeakingMsgSent && !m_optionsEnv.m_TimeForUpdatingVirtualSensors )
                    { // if it is time to update virtual-sensors and there is things to update for at least one robot

                        //qDebug() << "Sending Stop speaking message";

                        kilobot_broadcast msg;
                        msg.type=5; // Stop speaking message
                        emit broadcastMessage(msg);

                        broadcasing=true;
                        t_since=1;

                        m_optionsEnv.m_VirtualSensorsNeedUpdate=false;

                        StopSpeakingMsgSent=true;
                        for(int i=0 ; i<m_optionsEnv.UpdateMessagingQueue.size(); i++){
                            m_optionsEnv.UpdateMessagingQueue[i]=true;
                        }
                    }
                }

                if(StopSpeakingMsgSent && !broadcasing)
                {// the robot are quite, now ARK can speakS
                    //qDebug() << "Stop speaking message sent... now ARK can speak!";
                    ThereIsMsgsToSend=true;
                    m_optionsEnv.m_TimeForUpdatingVirtualSensors=true;
                    StopSpeakingMsgSent=false;
                }


                if(m_optionsEnv.m_TimeForUpdatingVirtualSensors && !ThereIsMsgsToSend)
                {// the virtul-sensors of the robots has been updated

                    //qDebug() << "Sending come back speaking message";

                    // Ask the robots to resume speaking to each other
                    kilobot_broadcast msg;
                    msg.type=6; // Comeback to speaking message
                    emit broadcastMessage(msg);


                    broadcasing=true;
                    t_since=1;

                    m_optionsEnv.m_TimeForUpdatingVirtualSensors=false;

                    Trobot_last=m_time;
                }
            }


            clearDrawings();
            clearDrawingsOnRecordedImage();
            plotEnvironment();
        }
        else{
            clearDrawings();
            clearDrawingsOnRecordedImage();
        }
    }


}

// Setup the Initial Kilobot Environment:
//   This is run once for each kilobot after emitting getInitialKilobotStates() signal.
//   This assigns kilobots to an environment.
void mykilobotexperiment::setupInitialKilobotState(Kilobot kilobotCopy) {

    // Assigns all kilobots to environment pheroEnv:
    this->setCurrentKilobotEnvironment(&m_optionsEnv);

    kilobot_id kID = kilobotCopy.getID();

    // create a necessary lists and variables
    if (kID > allKilos.size()-1)
    {
        allKilos.resize(kID+1);
        m_optionsEnv.m_Single_Discovery.resize(kID+1);
        m_optionsEnv.m_prev_discovery.resize(kID+1);
        m_optionsEnv.m_wall_detected.resize(kID+1);
        m_optionsEnv.m_requireGPS.resize(kID+1);
        m_optionsEnv.MessagingQueue.resize(kID+1);
        m_optionsEnv.UpdateMessagingQueue.resize(kID+1);
    }

    KiloLog kLog(kID, kilobotCopy.getPosition(), 0, OFF);
    allKilos[kID] = kLog;
    if (!allKiloIDs.contains(kID)) allKiloIDs.append(kID);

}

// run once for each kilobot after emitting updateKilobotStates() signal
void mykilobotexperiment::updateKilobotState(Kilobot kilobotCopy) {

    kilobot_id kID = kilobotCopy.getID();
    QPointF kPos = kilobotCopy.getPosition();
    lightColour kColor=kilobotCopy.getLedColour();

    // update values for logging
    if (logExp){
        allKilos[kID].updateAllValues(kID, kPos, 0, kColor);
    }

}

// Setup Environment
void mykilobotexperiment::setupEnvironments( )
{

    qsrand(time(NULL));

    QVector<int> RandPosCopy,RandPos;

    for(int i=1; i<=m_NumberOfOptions ; i++)
    {
        RandPosCopy.push_back(i);
    }

    for(int i=1; i<=m_NumberOfOptions ; i++)
    {
        int rand=qrand()%m_NumberOfOptions;

        while (RandPosCopy[rand]==0)
        {
            rand=qrand()%m_NumberOfOptions;
        }

        RandPos.push_back(RandPosCopy[rand]);
        RandPosCopy[rand]=0;
    }


    option Op;

    for(int i=1; i<=m_NumberOfOptions ; i++)
    {

        Op.ID=i;

//        if( i == m_NumberOfOptions )
//        {
//            Op.quality=m_HighestQuality;
//        }
//        else
//        {
//            Op.quality=m_Difficulty*m_HighestQuality;
//        }

        if( i == 3 )
        {
            Op.quality=8.0;
            Op.AppearanceTime=0.0;
            Op.DisappearanceTime=1200.0;
            Op.QualityChangeTime=0.0;
            Op.QualityAfterChange=0.0;
            Op.color=QColor(255,0,0);

        }
        else if ( i == 2)
        {
            Op.quality=6.0;
            Op.AppearanceTime=0.0;
            Op.DisappearanceTime=0.0;
            Op.QualityChangeTime=2400.0;
            Op.QualityAfterChange=4.0;
            Op.color=QColor(0,255,0);
        }
        else if ( i == 1)
        {
            Op.quality=4.0;
            Op.AppearanceTime=0.0;
            Op.DisappearanceTime=0.0;
            Op.QualityChangeTime=2400.0;
            Op.QualityAfterChange=6.0;
            Op.color=QColor(0,0,255);
        }

        Op.rad=m_OptionsRadius*M_TO_PIXEL;

        Op.posX=(m_PolygoneRadius*qCos((m_NumberOfOptions-RandPos[i-1])*2*M_PI/m_NumberOfOptions)+1)*M_TO_PIXEL+m_Xoffset;
        Op.posY=(m_PolygoneRadius*qSin((m_NumberOfOptions-RandPos[i-1])*2*M_PI/m_NumberOfOptions)+1)*M_TO_PIXEL+m_Yoffset;
//        Op.color=QColor(255,255-Op.quality/m_HighestQuality*255,255-Op.quality/m_HighestQuality*255);

        // Set Option GPS coordinates
        QPoint GPS_cords=m_optionsEnv.PositionToGPS(QPointF(Op.posX,Op.posY));
        Op.GPS_X= (unsigned int) GPS_cords.x();
        Op.GPS_Y= 31-(unsigned int) GPS_cords.y();

        qDebug() << "Option=" << i << " Quality=" << Op.quality << " GPS.X="<<Op.GPS_X << " GPS.Y="<<Op.GPS_Y;

        // Add the options to the en
        m_optionsEnv.m_Options.push_back(Op);
    }

    plotEnvironment();
}

// Draw the options:
void mykilobotexperiment::plotEnvironment() {
    option Op;
    for(int i=0;i<m_optionsEnv.m_Options.size();i++){
        Op=m_optionsEnv.m_Options[i];
        drawCircle(QPointF(Op.posX, Op.posY),Op.rad, Op.color, -1,"",true);
    }
}

// Slot to inform the data retrieval process about the color of the kilobots
void mykilobotexperiment::InformDataRetrievalProcessAboutRobotsColors(std::vector<lightColour>& infovector)
{
    infovector.clear();
    for(int i=0;i<allKilos.size();i++){
        infovector.push_back(allKilos[i].colour);
    }
}



void mykilobotexperiment::UpdateBroadcastingState()
{
    if(broadcasing){
        if(t_since>0){
            t_since--;
        }
        else {
            // STOP sending message
            kilobot_broadcast msg;
            msg.type = 250;
            emit broadcastMessage(msg);
            // Update broadcasting flag
            broadcasing=false;

            if(m_NumberOfConfigMsgsSent == m_optionsEnv.m_Options.size() && !configurationMsgsSent && RobotSpeaking ){
                configurationMsgsSent=true;
                qDebug() << "Done sending config messages!";
                qDebug() << "*********************************************";
            }
        }
    }
}
