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
    m_DataRetrieval_ON=false;
    m_log_period=1;
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
            qDebug() << "ERROR(!) in opening file " << log_filename;
        } else {
            qDebug () << "Log file " << log_file.fileName() << " opened.";
            log_stream.setDevice(&log_file);
        }

        /* save the option positions and behaviour in another file */
        QFile log_opts_file;
        QString log_opts_filename = log_foldername+log_opts_filename_suffix;
        log_opts_file.setFileName( log_opts_filename );
        if ( !log_opts_file.open(QIODevice::WriteOnly) ) { // open file
            qDebug() << "ERROR(!) in opening file " << log_opts_filename;
        } else {
            QTextStream log_opts_stream;
            log_opts_stream.setDevice(&log_opts_file);
            log_opts_stream << m_optionsEnv.m_Options.size();
            for (int i = 0; i < m_optionsEnv.m_Options.size(); ++i){
                option op = m_optionsEnv.m_Options[i];
                log_opts_stream << "\t" << op.ID << "\t" << op.posX << "\t" << op.posY
                           << "\t" << op.GPS_X << "\t" << op.GPS_Y << "\t" << op.rad << "\t" << op.quality << "\t"
                           << op.AppearanceTime << "\t" << op.DisappearanceTime << "\t"
                           << op.QualityChangeTime << "\t" << op.QualityAfterChange;
            }
            log_opts_stream << endl;
            log_opts_file.close();
            qDebug() << "Saved option info in file " << log_opts_filename;
        }
    }

    savedImagesCounter = 0;
    this->m_time = 0;
    ElapsedTime.start();
    RTID_elapsedTime=0;
    RTID_wasActive=0;

    //save initial image
    if (saveImages) {
        emit saveImage(im_filename_prefix.arg(m_expno,1)+im_filename_suffix.arg(savedImagesCounter++, 5,10, QChar('0')));
    }

    //log initial state of the robots
    if (logExp) {
        m_last_log=m_time;
        log_stream << this->m_time;
        for (int i = 0; i < allKiloIDs.size(); ++i){
            kilobot_id kID = allKiloIDs[i];
            log_stream << "\t" << kID << "\t" << allKilos[kID].colour << "\t" << allKilos[kID].position.x() << "\t" << allKilos[kID].position.y() << "\t" << allKilos[kID].orientation << "\t" << m_optionsEnv.goingResamplingList[kID];
        }
        log_stream << endl;
    }


    int count=0;
    int groupId=0;
    QVector<kilobot_id> group;
    for (int i = 0; i < allKiloIDs.size(); ++i){
        group.append(allKiloIDs.at(i));
        count++;
        if (count>=10 || i==allKiloIDs.size()-1){
            m_optionsEnv.kiloGroupForMessaging.append(group);
            count=0;
            groupId++;
            group.clear();
        }
    }
    for (int i = 0; i < m_optionsEnv.kiloGroupForMessaging.size(); ++i){
        //qDebug() << "In group" << i << "there are the"<< m_optionsEnv.kiloGroupForMessaging.at(i).size()<<"ids:";
        for (int j = 0; j < m_optionsEnv.kiloGroupForMessaging.at(i).size(); ++j){
            //qDebug() << m_optionsEnv.kiloGroupForMessaging.at(i).at(j);
        }
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
    m_optionsEnv.msgGroup=0;
    m_optionsEnv.kiloGroupForMessaging.clear();
    m_optionsEnv.goingResamplingList.clear();
    previousColourList.clear();
    m_NumberOfConfigMsgsSent=0;
    RobotSpeaking=true;
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
        qDebug() << "*************************************** BROADCAST MSG 10 ***************************************";
        msg.data.resize(9);
        msg.data[0]= m_optionsEnv.m_Options[m_NumberOfConfigMsgsSent].ID; // OP ID
        msg.data[1]= (uint8_t) m_optionsEnv.m_Options[m_NumberOfConfigMsgsSent].GPS_X; // OP X
        msg.data[2]= (uint8_t) m_optionsEnv.m_Options[m_NumberOfConfigMsgsSent].GPS_Y; // OP Y
//        msg.data[3]= (uint8_t) m_optionsEnv.m_Options[m_NumberOfConfigMsgsSent].rad*100; // robot field of view to detect disappearance
        msg.data[3]= (uint8_t) (m_OptionsRadius*100); // robot field of view to detect disappearance
        msg.data[4]= 4; // min distance to go away

        if(initial_opinion!=0)
        {
            msg.data[5]= (uint8_t) initial_opinion;    // initial opinion
            msg.data[6]= m_optionsEnv.m_Options[initial_opinion-1].quality;  // initial opinion quality
            msg.data[7]= m_optionsEnv.m_Options[initial_opinion-1].GPS_X;    // initial opinion X
            msg.data[8]= m_optionsEnv.m_Options[initial_opinion-1].GPS_Y;    // initial opinion Y
        }

//        qDebug() << "Sending Config Msg No." << m_NumberOfConfigMsgsSent;
//        qDebug() << "data[0]=" << msg.data[0];
//        qDebug() << "data[1]=" << msg.data[1];
//        qDebug() << "data[2]=" << msg.data[2];
//        qDebug() << "data[3]=" << msg.data[3];
//        qDebug() << "data[4]=" << msg.data[4];
//        qDebug() << "data[5]=" << msg.data[5];
//        qDebug() << "data[6]=" << msg.data[6];
//        qDebug() << "data[7]=" << msg.data[7];

        emit broadcastMessage(msg);

        broadcasing=true;
        t_since=3;

        m_NumberOfConfigMsgsSent++;
    }

    if(m_NumberOfConfigMsgsSent == m_optionsEnv.m_Options.size() && !broadcasing && !configurationMsgsSent )
    {

        qDebug() << "Asking robots to stop speaking!";

        kilobot_broadcast msg;
        msg.type=5; // Stop speaking
        emit broadcastMessage(msg);

        broadcasing=true;
        t_since=3;

        RobotSpeaking=false;
    }


    if(configurationMsgsSent)
    {

        if(!this->runtimeIdentificationLock){
            // keep track of the time spent in RTID
            if (RTID_wasActive==RTID_periodOfStoppedActivityAfterActive) { // this means this is the first timestep after active RTID
                RTID_elapsedTime += RTID_timer.elapsed();
                qDebug() << "RTID was active for a total of " << RTID_elapsedTime << "ms in the whole experiment.";
            }

            // Increment time
            if(!m_data_retrieval_running){
                this->m_time=(ElapsedTime.elapsed() - RTID_elapsedTime)/1000.0; // time in seconds
                m_optionsEnv.m_time=this->m_time;
                if (qRound((m_time-m_last_log2)*10.0f) >= 60*10.0f) { // every minute
                    m_last_log2=m_time;
                    qDebug() << "Running time: " << this->m_time <<" at " << QLocale("en_GB").toString( QDateTime::currentDateTime(), "hh:mm:ss.zzz");
                }
            }

            if (RTID_wasActive==0) {
                // Reset population counters (which are counted in updateKilobotState()
                countRed=0;countGreen=0;countBlue=0;countOff=0;
                // Update Kilobot States
                emit updateKilobotStates();
            }

            //Save image and log data once every m_log_period seconds [we avoid to log data at the timestep that follows a RTID as it can contain spurious information]
            if (RTID_wasActive==0) {
                //if (qRound(m_time*10.0f) % qRound(m_log_period*10.0f) == 0)
                if (qRound((m_time-m_last_log)*10.0f) >= m_log_period*10.0f)
                { // every m_log_period
                    m_last_log=m_time;
                    if (saveImages) {
                        emit saveImage(im_filename_prefix.arg(m_expno,1)+im_filename_suffix.arg(savedImagesCounter++, 5,10, QChar('0')));
                    }
                    if (logExp)
                    {
                        log_stream << this->m_time;
                        for (int i = 0; i < allKiloIDs.size(); ++i){
                            kilobot_id kID = allKiloIDs[i];
                            log_stream << "\t" << kID << "\t" << allKilos[kID].colour << "\t" << allKilos[kID].position.x() << "\t" << allKilos[kID].position.y() << "\t" << allKilos[kID].orientation << "\t" << m_optionsEnv.goingResamplingList[kID];
                        }
                        log_stream << endl;
                    }
                }
            } else {
                RTID_wasActive -= 1;
            }

            // move to the next group when all previous messages have been sent
            //qDebug() << "**************** [" << m_time << "] msgToSend:" << ThereIsMsgsToSend << " groupID:"<< m_optionsEnv.msgGroup;
            if (!ThereIsMsgsToSend) {
                //m_optionsEnv.msgGroup = (m_optionsEnv.msgGroup+1)%m_optionsEnv.kiloGroupForMessaging.size();
                m_optionsEnv.msgGroup++;

                // when all groups have been messaged, broadcast virtual robot-robot global communication (population size)
                if (m_optionsEnv.msgGroup == m_optionsEnv.kiloGroupForMessaging.size()){
                    /** broadcast info about population **/
                    kilobot_broadcast msg;
                    msg.type=7; // Comeback to speaking message
                    QVector < uint8_t > data(9);
                    data[0] = countRed;
                    data[1] = countGreen;
                    data[2] = countBlue;
                    data[3] = qRound(m_optionsEnv.m_Options[0].quality*10); // red
                    data[4] = qRound(m_optionsEnv.m_Options[2].quality*10); // green
                    data[5] = qRound(m_optionsEnv.m_Options[1].quality*10); // blue
                    msg.data = data;
                    emit broadcastMessage(msg);
                    qDebug() << "Broadcasting (at time " << m_time << ") the global populations = [" << countRed << "," << countGreen << "," << countBlue << "]"  << " (" << countOff << "), with qualities [" << data[3] << "," << data[4] << "," << data[5] << "]";
                    broadcasing = true;
                    t_since=1;
                }
                // if all groups have been messaged, and broadcast has been completed, the loop on groups can restart
                if (m_optionsEnv.msgGroup > m_optionsEnv.kiloGroupForMessaging.size() && !broadcasing){
                    m_optionsEnv.msgGroup = 0;
                }
            }
            // block communication if the buffer is busy or if there are broadcast messages
            if (m_optionsEnv.msgGroup < m_optionsEnv.kiloGroupForMessaging.size()){
                m_optionsEnv.OhcBufferIsEmpty = !ThereIsMsgsToSend;
            } else {
                m_optionsEnv.OhcBufferIsEmpty = false;
            }

            // update the dynamic (virtual) environment
            m_optionsEnv.update();

            /* plot virtual environment */
            clearDrawings();
            clearDrawingsOnRecordedImage();
            plotEnvironment();
        }
        else{
            if (RTID_wasActive==0) { // it means this is the first timestep with active RTID
                RTID_timer.start();
            }
            RTID_wasActive=RTID_periodOfStoppedActivityAfterActive; // flag used to avoid to log spurious data after RTID. We wait for RTID_periodOfStoppedActivityAfterActive loops
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
        previousColourList.resize(kID+1);
        m_optionsEnv.goingResamplingList.resize(kID+1);
        m_optionsEnv.m_Single_Discovery.resize(kID+1);
        m_optionsEnv.m_optionStillThere.resize(kID+1);
        m_optionsEnv.m_wall_detected.resize(kID+1);
    }

    KiloLog kLog(kID, kilobotCopy.getPosition(), 0, OFF);
    allKilos[kID] = kLog;
    previousColourList[kID] = OFF;
    m_optionsEnv.goingResamplingList[kID] = false;
    if (!allKiloIDs.contains(kID)) allKiloIDs.append(kID);

}

// run once for each kilobot after emitting updateKilobotStates() signal
void mykilobotexperiment::updateKilobotState(Kilobot kilobotCopy) {

    kilobot_id kID = kilobotCopy.getID();
    QPointF kPos = kilobotCopy.getPosition();
    lightColour kColor=kilobotCopy.getLedColour();
    double kRot = qRadiansToDegrees(qAtan2(-kilobotCopy.getVelocity().y(), kilobotCopy.getVelocity().x()));

    // update values for logging
    if (logExp){
        allKilos[kID].updateAllValues(kID, kPos, kRot, kColor);
    }

    //qDebug()<<kID;
    // update counters used for vitualising global communication
    lightColour commitment = kColor;
    if (commitment==OFF){ // if the tracker detects an OFF led, we use the previous detected colour (as in the compare method there are no uncommitted)
        commitment = previousColourList[kID];
    } else {
        // if the robot changed opinion it needs resampling (except if it is already within the option)
        if ( previousColourList[kID] != commitment && commitment!=OFF && previousColourList[kID]!=OFF) { // I use previousColourList[kID]!=OFF to avoid to initially cast all robots as needed resampling
            /* check if already within option, in this case, (we assume) no resamplind would be needed */
            unsigned int oidx = m_optionsEnv.indexOptOfColour(commitment);
            int idx=-1;
            for (int i=0;i<m_optionsEnv.m_Options.size();i++ ) { if (m_optionsEnv.m_Options[i].ID==oidx) {idx=i;} }
            /* if not in the option area, set goingResamplingList[kID]=true */
            m_optionsEnv.goingResamplingList[kID] = !(idx>=0 && (pow(kPos.x()-m_optionsEnv.m_Options[idx].posX,2)+pow(kPos.y()-m_optionsEnv.m_Options[idx].posY,2)) < pow(m_optionsEnv.m_Options[idx].rad,2) );
        }
        previousColourList[kID] = commitment;
    }
    /* update the counter */
    if ( m_optionsEnv.goingResamplingList[kID] || commitment==OFF){
        countOff++;
    } else {
        if (commitment==RED){ countRed++; }
        if (commitment==GREEN){ countGreen++; }
        if (commitment==BLUE){ countBlue++; }
    }

}

// Setup Environment
void mykilobotexperiment::setupEnvironments( )
{

    qsrand(time(NULL));

    QVector<int> RandPosCopy,RandPos;

    for(size_t i=1; i<=m_NumberOfOptions ; i++)
    {
        RandPosCopy.push_back(i);
    }

    for(size_t i=1; i<=m_NumberOfOptions ; i++)
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

    for(unsigned int i=1; i<=m_NumberOfOptions ; i++)
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

        if( i == 1 )
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
            Op.color=QColor(0,0,255);
        }
        else if ( i == 3)
        {
            Op.quality=4.0;
            Op.AppearanceTime=0.0;
            Op.DisappearanceTime=0.0;
            Op.QualityChangeTime=2400.0;
            Op.QualityAfterChange=6.0;
            Op.color=QColor(0,255,0);
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
void mykilobotexperiment::plotEnvironment()
{
    option Op;
    for(int i=0;i<m_optionsEnv.m_Options.size();i++)
    {
        Op=m_optionsEnv.m_Options[i];

        if( Op.DisappearanceTime==0 || m_time < Op.DisappearanceTime)
        {
            drawCircle(QPointF(Op.posX, Op.posY),Op.rad, Op.color, 8,"",true);
        }
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

            if(m_NumberOfConfigMsgsSent == m_optionsEnv.m_Options.size() && !configurationMsgsSent && !RobotSpeaking ){
                configurationMsgsSent=true;
                qDebug() << "Done sending config messages!";
                qDebug() << "*********************************************";
            }
        }
    }
}
