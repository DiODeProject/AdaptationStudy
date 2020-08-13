#ifndef KILOBOTEXPERIMENT_H
#define KILOBOTEXPERIMENT_H

#include <QObject>
#include <QDebug>
#include <QLayout>
#include <kilobotenvironment.h>
#include <kilobot.h>
#include <QColor>
#include <opencv2/core/core.hpp>
#include <QElapsedTimer>


class KiloDataRetrieval: public QObject{
    Q_OBJECT
public:
    enum DataRetrievalStages {
        START,
        SEND,
        GETDIGITS,
        CONFIRM,
        RETRY,
        COMPLETE,
    };

    enum DataRetrievalMethods {
        BINARY,
        BASETHREE,
    };


    KiloDataRetrieval(){

    }

    ~KiloDataRetrieval() {}

    void initialise(unsigned int number_of_robots,DataRetrievalMethods method,unsigned int data_to_retrieve, unsigned int number_of_digits,bool confirmation)
    {
        /* Initialize the required variables */
        m_retrieved_data.resize(number_of_robots);
        m_robots_responses.resize(number_of_digits);
//        m_confirmation_responses.resize(number_of_digits);


        m_method=method;

        m_data_ID=data_to_retrieve;
        m_NumOfDigits=number_of_digits;
        m_confirmation=confirmation;

        /* reset time of the data retrieval process */
        time = 0.0;
        lastTime = 0.0;
        t_since=0.0;


        /* number of retrieved digits */
        m_retrieved_digits=0;

        /* We should retrieve the next digit */
        m_switchSegment=true;

        /* Set the current stage of the data retrieval process */
        m_current_stage=START;

        /* Inform the robots that we want to start a data retrieval process*/
        kilobot_broadcast msg;
        msg.type = 10;
        emit broadcastMessage(msg);
        /* set the start of the data retrieval process */
        qDebug() << "Data retrieval process started";
        t.start();

    }

    void run(){

        this->time += 0.1; // 100 ms in sec
        this->lastTime += 0.1;

        // stop broadcasting (replace)
        if (t_since > 0) {
            --t_since;
        } else {
            kilobot_broadcast msg;
            msg.type = 250;
            emit broadcastMessage(msg);
        }

        if (time > 2.0f) {

            switch (this->m_current_stage) {
            case START:
            {
                this->lastTime = 0;
                this->m_switchSegment = true;
                this->m_retrieved_digits = 0;

                // Specify to the robots which data we want to retrieve
                kilobot_broadcast msg;

                msg.type = 1;
                QVector<uint8_t> data;
                data.resize(9);
                data[0] = m_data_ID; // it can be either an internal variable of even to pick a random number
                data[1] = m_NumOfDigits; // how many digits are required to express the data to retrieve
                msg.data=data;
                emit broadcastMessage(msg);
                qDebug() << "Inform the robots about the required data and ask them to decompose it ";
                t_since = 2;


                this->m_current_stage = GETDIGITS;
                break;
            }

            case GETDIGITS:
            {

                if (this->m_switchSegment == true) {

                    this->m_switchSegment = false;

                    // when we have all segments
                    if (m_retrieved_digits > (m_NumOfDigits-1)) {

                        // compute retreived data;
                        for(int i=0;i<m_retrieved_data.size();i++){

                            m_retrieved_data[i]=0;

                            for(int j=0;j<m_robots_responses.size();j++){
                                if(m_robots_responses[j][i]!=OFF){
                                    int digit_value;
                                    if(m_robots_responses[j][i]==RED){
                                        digit_value=0;
                                    }
                                    if(m_robots_responses[j][i]==BLUE){
                                        digit_value=1;
                                    }
                                    /*
                                     * // Debugging the digits
                                    qDebug() << "Robot " << i << "digit:"<< j <<"="<< digit_value;
                                    */
                                    m_retrieved_data[i]+=digit_value*binaryMultipliers[j];
                                }
                                else{
                                    m_retrieved_data[i]=-1;
                                    break;
                                }
                            }
                            /*
                             * // Debugging retrieved data
                            if(m_retrieved_data[i]==-1){
                                qDebug() << "Could not trieve data from robot:" << i;
                            }
                            else{
                                qDebug() << "Robot " << i << "has data: "<< m_retrieved_data[i];
                            }*/

                        }

                        if(m_confirmation){
                            this->m_current_stage = CONFIRM;
                            qDebug() << "Starting confirmation stage!" ;
                        }
                        else{
                            this->m_current_stage = COMPLETE;
                        }
                    }

                    // get next seg
                    kilobot_broadcast msg;

                    msg.type = 4;
                    emit broadcastMessage(msg);
                    qDebug() << "Asking the robots to show new digit (t:" << lastTime << t.elapsed() <<")";
                    t_since = 2;
                }

                if (lastTime > 2.0f*float(m_retrieved_digits+1)+0.21f) {
                    emit SeekInformationAboutKilobotResponses(m_robots_responses[m_retrieved_digits]);
                    ++m_retrieved_digits;
                    this->m_switchSegment = true;
                }
                break;
            }


            case CONFIRM:
            {
                //            qDebug() << "CONFIRMING" << lastTime;

                if((this->confirmationrequestsent==false)){

                    // get the next tempID to confirm
                    while( ( this->m_retrieved_data[increment] == -1 ) && (increment<m_retrieved_data.size()-1)  )
                        increment++;


                    // send a confirmation request to the current tempID holder
                    QVector<uint8_t> data;
                    data.resize(9);
                    // current temp ID
                    data[0] = (increment >> 8)&0xFF;
                    data[1] = increment&0xFF;
                    data[2] = m_retrieved_data[increment];
                    kilobot_broadcast msg;
                    msg.type = 7;
                    msg.data = data;
                    emit broadcastMessage(msg);
                    t_since=40;
                    this->confirmationrequestsent=true;
                    qDebug() << "Confirmation message sent to robot: " << increment;
                }
                else {
                    // get the confirmation
                    if(t_since==20){
                        emit SeekInformationAboutKilobotResponses(m_confirmation_responses);
                    }
                    if(t_since==0){
                        if(m_confirmation_responses[increment]!=BLUE){
                            m_retrieved_data[increment]=-1;
                        }
                        this->confirmationrequestsent=false;
                        increment++;
                    }
                }

                if(increment>=m_retrieved_data.size()){
                    increment=0;
                    this->m_current_stage = COMPLETE;
                }

                break;
            }
                /*
            case SEND:
            {
                if (lastTime > 25.0f) {
                    //                  for (int i = 0; i < tempIDs.size(); ++i) {
                    //                      qDebug() << i << "[" << this->isAssigned[i] << "]:" << this->tempIDs[i];
                    //                  }

                    for (int id = 0; id < tempIDs.size(); ++id) {
                        //qDebug() << "SEND" << lastTime;
                        if (this->tempIDs[id] != DUPE && !this->isAssigned[id]) {
                            QVector<uint8_t> data;
                            data.resize(9);
                            // current temp ID
                            data[0] = (this->tempIDs[id] >> 8)&0xFF;
                            data[1] = this->tempIDs[id]&0xFF;
                            // UID to set
                            data[2] = (id >> 8)&0xFF;
                            data[3] = id&0xFF;
                            kilobot_broadcast msg;
                            msg.type = 2;
                            msg.data = data;
                            emit broadcastMessage(msg);
                            this->isAssigned[id] = true; // set as assigned
                        }
                    }

                    t_since = tempIDs.size() + 2;
                    if (dupesFound) {
                        this->stage = RETRY;
                        t_move=20;
                    } else {
                        this->stage = COMPLETE;
                    }
                }
                break;
            }
            case RETRY:
            {
                if (t_since <= 0) {
                    //qDebug() << "RETRY" << lastTime;
                    kilobot_broadcast msg;
                    msg.type = 3;
                    emit broadcastMessage(msg);
                    t_since = 2;
                    if(t_move>0){
                        t_move--;}
                    else{
                        this->stage = START;
                    }
                }
                break;
            }
*/
            case COMPLETE:
            {

                /*
                 * // Debugging retrieved data and failures
                for(int i=0;i<m_retrieved_data.size();i++){
                    if(m_retrieved_data[i]==-1){
                        qDebug() << "Could not trieve data from robot:" << i;
                    }
                    else{
                        qDebug() << "Robot " << i << "has data: "<< m_retrieved_data[i];
                    }
                }
                */

                /* Inform the robots about the end of data retrieval process*/
                kilobot_broadcast msg;
                msg.type = 11;
                emit broadcastMessage(msg);
                qDebug() << "Ending data retrieval process";
                emit DataRetrievalProcessEnded();
                break;
            }

            }

        }
    }



    const int baseFourMultipliers[6] = {1,4,16,64,256,1024};
    const int binaryMultipliers[11] = {1,2,4,8,16,32,64,128,256,512,1024};
    const int baseThreeMultipliers[8] = {1,3,9,27,81,243,729,2187};


    /* Define which data will be retrieve (there may be many data to be
     * retreived in the the same experiment), it should be defined when calling
     * the initialise() function */
    unsigned int m_data_ID;


    /* Define how many digits are required to express the data to retrieve,
     * it should be defined when calling the initialise() function */
    unsigned int m_NumOfDigits;

    /* Is it time to switch to the next digit */
    bool m_switchSegment;


    /* Current stage of the data retrieval process */
    DataRetrievalStages m_current_stage=COMPLETE;


    /* Current digit to be retrieved */
    unsigned int m_retrieved_digits;


    /* Define in which base the data will be expressed by
     * the robot it should be defined at the creation of
     * the class, at the begging of the experiment)    */
    DataRetrievalMethods m_method;

    /* Output of the data retrieval (a number for each kilobot)*/
    std::vector<int> m_retrieved_data;
    std::vector<lightColour> m_confirmation_responses;

    /* Output of the data retrieval (a number for each kilobot)*/
    std::vector< std::vector<lightColour> > m_robots_responses;

    /* Set if to do a confirmation of the retrieved data or not */
    bool m_confirmation;

    bool confirmationrequestsent;
    int increment=0;

    QElapsedTimer t;


    float time;
    float lastTime;
    int t_since;

signals:
    void DataRetrievalProcessEnded();
    void broadcastMessage(kilobot_broadcast);
    void SeekInformationAboutKilobotResponses(std::vector<lightColour>&);
};



class KilobotExperiment : public QObject
{
    Q_OBJECT
public:
    KilobotExperiment() {}
    virtual ~KilobotExperiment()
    {
    }

    int serviceInterval = 100; // ms

    virtual QWidget * createGUI() {return NULL;}

signals:
    void updateKilobotStates();
    void getInitialKilobotStates();
    void experimentComplete();
    void saveImage(QString);
    void saveVideoFrames(QString,unsigned int);
    void signalKilobot(kilobot_message);
    void broadcastMessage(kilobot_broadcast);
    void setTrackingType(int);

    // drawing
    void drawCircle(QPointF pos, float r, QColor col, int thickness, std::string text, bool transparent=false);
    void drawLine(std::vector<cv::Point> pos, QColor col, int thickness, std::string text, bool transparent);
    void clearDrawings();
    void drawCircleOnRecordedImage(QPointF pos, float r, QColor col, int thickness, std::string text);
    void clearDrawingsOnRecordedImage();


public slots:
    virtual void initialise(bool) = 0;
    virtual void stopExperiment() {}
    virtual void run() {}
    void setRuntimeIdentificationLock(bool lock) {this->runtimeIdentificationLock = lock;}
    void SendDataRetrievalProcessMessage(kilobot_broadcast msg) {emit broadcastMessage(msg);}
    void toggleDataRetrievalProcessEnded(){ m_data_retrieval_running = false;
                                            qDebug() << "Data retrieval process ended";
                                          }
    void InformDataRetrievalProcessAboutRobotsColors(std::vector<lightColour>&);
    void AllMsgsSent(){ ThereIsMsgsToSend=false;}


    /*!
     * \brief updateStateRequiredCode
     * \param kilobot
     * \param kilobotCopy
     *
     * Slot that makes sure that some code is run BEFORE the derived function
     */
    void updateStateRequiredCode(Kilobot* kilobot, Kilobot kilobotCopy)
    {
        //qDebug() << "pre set state 2";
        // store pointer for connecting
        this->currKilobot = kilobot;
        updateKilobotState(kilobotCopy);
    }

    /*!
     * \brief setupInitialStateRequiredCode
     * \param kilobot
     * \param kilobotCopy
     *
     * Slot that makes sure that some code is run BEFORE the derived function
     */
    void setupInitialStateRequiredCode(Kilobot* kilobot, Kilobot kilobotCopy)
    {
        //qDebug() << "pre set state";
        // store pointer for connecting
        this->currKilobot = kilobot;
        // switch the signal from setup to standard
        kilobot->disconnect(SIGNAL(sendUpdateToExperiment(Kilobot*,Kilobot)));
        connect(kilobot,SIGNAL(sendUpdateToExperiment(Kilobot*,Kilobot)), this, SLOT(updateStateRequiredCode(Kilobot*,Kilobot)));
        setupInitialKilobotState(kilobotCopy);
    }

    void signalKilobotExpt(kilobot_message msg)
    {
        emit signalKilobot(msg);
    }

protected:
    float m_time;
    bool runtimeIdentificationLock = false;
    bool ThereIsMsgsToSend=false;

    void setCurrentKilobotEnvironment(KilobotEnvironment * environment) {
        if (currKilobot != NULL && environment != NULL) {
            QObject::disconnect(currKilobot,SIGNAL(sendUpdateToHardware(Kilobot)), 0, 0);
            QObject::connect(currKilobot,SIGNAL(sendUpdateToHardware(Kilobot)), environment, SLOT(updateVirtualSensor(Kilobot)));
        }
    }

    virtual void updateKilobotState(Kilobot) {} // provided in derived class to implement experiment logic for Kilobot state updates
    virtual void setupInitialKilobotState(Kilobot) {}

    // Data Retrieval class
    KiloDataRetrieval m_DataRetrieval; // data retrieval class
    bool m_DataRetrieval_ON=false;
    float m_data_retrieval_period; //data retrieval period in MINUTES
    unsigned int m_data_to_retrieve_ID;
    unsigned int m_data_to_retrieve_numberofdigits;
    bool m_data_retrieval_running=false;
    KiloDataRetrieval::DataRetrievalMethods m_data_retrieval_method=KiloDataRetrieval::DataRetrievalMethods::BINARY;




private:
    Kilobot * currKilobot = NULL;

};



#endif // KILOBOTEXPERIMENT_H
