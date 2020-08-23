#ifndef TESTLIB_H
#define TESTLIB_H

#include "global.h"

//
#include <QObject>
#include <QFile>
//#include <QTextStream>

// Project includes
#include "kilobot.h"
#include "kilobotexperiment.h"
#include "kilobotenvironment.h"
#include "bestofNEnv.h"

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <QTableWidget>
#include <QSpinBox>


using namespace cv;

class KiloLog {
public:
    // constructors
    KiloLog() {}
    KiloLog(kilobot_id id, QPointF pos, double rot, kilobot_colour col) : id(id), position(pos), orientation(rot), colour(col) {}

    // methods
    void updateAllValues(kilobot_id id, QPointF pos, double rot, kilobot_colour col){
        this->id = id;
        this->position = pos;
        this->orientation = rot;
        this->colour = col;
    }
    void setPos(QPointF pos){
        this->position = pos;
    }
    void setOrientation(double rot){
        this->orientation = rot;
    }
    void setCol(kilobot_colour col){
        this->colour = col;
    }

    // variables
    kilobot_id id;
    QPointF position;
    double orientation;
    kilobot_colour colour;
};

class BESTOFNEXPSHARED_EXPORT mykilobotexperiment : public KilobotExperiment
{
    Q_OBJECT
public:
    mykilobotexperiment();
    virtual ~mykilobotexperiment() {}

    QWidget * createGUI();

    //
    QVector < Kilobot * > mykilobot;

signals:
    void errorMessage(QString);
    //    void setExptImage(QPixmap);

public slots:
    void initialise(bool);
    void run();
    void stopExperiment();
    void UpdateBroadcastingState();

    //    void setupExperiment();
    void toggleSaveImages(bool toggle) { saveImages = toggle; }
    void toggleLogExp(bool toggle){ logExp = toggle; }
    void toggleDataRetrieval(bool toggle){ m_DataRetrieval_ON = toggle; }
    void setExpNumber(int value){m_expno=value;}
    void setNumberOfOptions(int value){m_NumberOfOptions=value;}
    void setHighestQuality(double value){m_HighestQuality=value;}
    void setDifficulty(double value){m_Difficulty=value;}
    void setOptionsRadius(double value){m_OptionsRadius=value;}
    void setNoise(double value){m_Quality_noise_variance=value;}
    void setTrobot(double value){Trobot=value;}
    void setXoffset(int value){m_Xoffset=value;}
    void setYoffset(int value){m_Yoffset=value;}
    void setPolygoneRadius(double value){m_PolygoneRadius=value;}
    void setDataRetrievalFrequency(int value){m_data_retrieval_period=value;}
    void InformDataRetrievalProcessAboutRobotsColors(std::vector<lightColour>& infovector);


private:
    void updateKilobotState(Kilobot kilobotCopy);
    void setupInitialKilobotState(Kilobot kilobotCopy);

    //
    void setupEnvironments();
    void plotEnvironment();

    //
    mykilobotenvironment m_optionsEnv;

    // Experiment number
    unsigned int m_expno=0;

    //Image saving variables
    bool saveImages;
    int savedImagesCounter = 0;
    QString im_filename_prefix="/home/salah/Francesco/run%1";
    QString im_filename_suffix="/bestof3_%1.jpg";

    //discovery update period in seconds
    float m_VirtualSensorUpdatePeriod;


    //Logging period in seconds
    float m_log_period;
    float m_last_log=0, m_last_log2=0;

    //Log variables
    bool logExp;
    QFile log_file;
    QString log_filename_prefix="/home/salah/Francesco/run%1";
    QString log_filename_suffix="/log_bestof3.txt";
    QString log_opts_filename_suffix="/log_optionPositionAndQuality.txt";
    QTextStream log_stream;
    QVector < kilobot_id >  allKiloIDs;
    QVector <KiloLog> allKilos;

    // GUI objects
    QTableWidget * tableofoptions;

    // Configuration messages
    bool configurationMsgsSent=false;
    unsigned int m_NumberOfConfigMsgsSent=0;
    bool RobotSpeaking=false;



    // broadcasting management
    bool broadcasing=false;
    int t_since;
    bool StopSpeakingMsgSent=false;


    // Env setup
    unsigned int m_NumberOfOptions=3;
    float m_Difficulty=0.6;
    float m_HighestQuality=8.0;
    float m_OptionsRadius=0.20;
    float m_Quality_noise_variance=1.0;
    int m_Xoffset=-550;
    int m_Yoffset=470;
    float m_PolygoneRadius=0.285;

    float Trobot=5.0;
    float Trobot_last=0.0;

    QTime ElapsedTime;
    QTime RTID_ElapsedTime;


    uint8_t initial_opinion=2;

};

#endif // TESTLIB_H
