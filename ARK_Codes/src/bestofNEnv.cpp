#include "bestofNEnv.h"
#include <QVector>
#include <QLineF>
#include <QDebug>
#include <QtMath>

#include "kilobot.h"

mykilobotenvironment::mykilobotenvironment(QObject *parent) : KilobotEnvironment(parent) {
    qDebug() << "Environment is up and running. :-)";
}

// generate virtual sensor readings & send to KB
void mykilobotenvironment::updateVirtualSensor(Kilobot kilobot) {
    kilobot_id kID = kilobot.getID();
    QPointF kPos = kilobot.getPosition();

    //     qDebug() << "UpdateMessagingQueue: " << UpdateMessagingQueue[kID];

    if(UpdateMessagingQueue[kID])
    {

        UpdateMessagingQueue[kID]=false;

        kilobot_message msg;

        unsigned int m_message_type;

        if (m_requireGPS[kID]==true)
        { // Update virtual-GPS

            // Get robot's GPS coordinates
            QPoint GPS_cords=PositionToGPS(kPos);
            //                        qDebug() << kID << " X=" <<GPS_cords.x()<< " Y=" <<31-GPS_cords.y();

            // Set msg type
            m_message_type=0; // 0->GPS 1->Discovery

            // Get robot's orientation
            double orientDegrees = qRadiansToDegrees(qAtan2(-kilobot.getVelocity().y(),kilobot.getVelocity().x()));
            //            qDebug() << kID << orientDegrees;

            orientDegrees=desNormAngle(orientDegrees);
            //                        qDebug() << kID << orientDegrees;

            msg.id=kID<<2 | m_message_type<<1 | ( (uint8_t) GPS_cords.x() ) >> 4;
            msg.type= ( (uint8_t) GPS_cords.x() ) & 0x0F;
            msg.data= ( (uint8_t) ( 31-GPS_cords.y() )  ) << 5 | ( (uint8_t) (orientDegrees/12.0) );
            MessagingQueue[kID].push_back(msg);
            m_requireGPS[kID]=false;
        }

        if(m_Single_Discovery[kID]!=0)
        { // Update wall virtual-sensor
            m_message_type=1;
            msg.id=kID<<2 | m_message_type<<1 |( (uint8_t) m_Options[m_Single_Discovery[kID]-1].GPS_X )>>4 ;
            msg.type= ( (uint8_t) m_Options[m_Single_Discovery[kID]-1].GPS_X ) & 0x0F;
            msg.data= ( (uint8_t) m_Options[m_Single_Discovery[kID]-1].GPS_Y ) << 5 | ( (uint8_t) m_Options[m_Single_Discovery[kID]-1].quality );
            MessagingQueue[kID].push_back(msg);
            m_Single_Discovery[kID]=0;
        }

    }
    /*
        if (m_requireGPS[kID]==true)
        { // Update virtual-GPS

            m_message_type=2;
            // Get robot's GPS coordinates
            QPoint GPS_cords=PositionToGPS(kPos);

            // Get robot's orientation
            double orientDegrees = qRadiansToDegrees(qAtan2(-kilobot.getVelocity().y(), kilobot.getVelocity().x()));

            msg.id=kID<<2 | m_message_type;
            msg.type= ( (uint8_t) GPS_cords.x() ) >> 1;
            msg.data= ( (uint8_t) GPS_cords.x() ) << 9 | ( (uint8_t) GPS_cords.y() ) << 4 | ( (uint8_t) (orientDegrees/24.0) );
            MessagingQueue[kID].push_back(msg);
            m_requireGPS[kID]=false;
        }

        if(m_Single_Discovery[kID]!=0)
        { // Update wall virtual-sensor
            m_message_type=0;
            msg.id=kID<<2 | m_message_type;
            msg.type= ( (uint8_t) m_Options[m_Single_Discovery[kID]-1].GPS_X ) >> 1;
            msg.data= ( (uint8_t) m_Options[m_Single_Discovery[kID]-1].GPS_X ) << 9 | ( (uint8_t) m_Options[m_Single_Discovery[kID]-1].GPS_Y ) << 4 | ( (uint8_t) m_Options[m_Single_Discovery[kID]-1].quality );
            MessagingQueue[kID].push_back(msg);
            m_Single_Discovery[kID]=0;
        }

        if(m_wall_detected[kID]== true)
        {// Update wall virtual-sensor
            m_message_type=1;

            msg.id=kID<<2 | m_message_type;
            MessagingQueue[kID].push_back(msg);
            m_wall_detected[kID]=false;
        }

    }
    */


    if(m_TimeForUpdatingVirtualSensors && !MessagingQueue[kID].empty())
    {
        emit transmitKiloState(MessagingQueue[kID].front());
        MessagingQueue[kID].pop_front();
    }


    kilobot_colour kLed = kilobot.getLedColour();
    option Op;


    // Check discovery virtual-sensor
    for (int i=0;i<m_Options.size();i++ )
    {
        Op=m_Options[i];
        float rad=Op.rad;
        float x=Op.posX;
        float y=Op.posY;
        //Check if the robot is inside an option for discovery
        if((pow(kPos.x()-x,2)+pow(kPos.y()-y,2)) < pow(rad,2) )
        {
            m_Single_Discovery[kID]=Op.ID;
            m_VirtualSensorsNeedUpdate=true;
        }
    }

    /*
    //Check wall virtual-sensor
    if((kPos.x()<=100)||(kPos.x()>=1900)||(kPos.y()<=100)||(kPos.y()>=1900))
    {
        m_wall_detected[kID]=true;
        m_VirtualSensorsNeedUpdate=true;
    }
    */

    //Check GPS virtual-sensor


    //    if( kLed == BLUE )
    //    {
    m_requireGPS[kID]=true;
    m_VirtualSensorsNeedUpdate=true;
    //    }
}


QPoint mykilobotenvironment::PositionToGPS(QPointF position){
    return QPoint((unsigned int) std::ceil(position.x()/m_fCellLength )-1, (unsigned int) std::ceil(position.y()/m_fCellLength )-1);
}

float mykilobotenvironment::desNormAngle(float angle){
    while (angle > 360) angle = angle - 360;
    while (angle < 0) angle = 360 + angle;
    return angle;
}
