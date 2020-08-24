#include "bestofNEnv.h"
#include <QVector>
#include <QLineF>
#include <QDebug>
#include <QtMath>

#include "kilobot.h"

mykilobotenvironment::mykilobotenvironment(QObject *parent) : KilobotEnvironment(parent) {
    qDebug() << "Environment is up and running. :-)";
}

// Update the dynamic environment:
void mykilobotenvironment::update() {
    for(int i=0; i<m_Options.size(); ++i) {
        if (!m_Options[i].QualityChangeApplied && m_Options[i].QualityChangeTime != 0.0 && this->m_time >= m_Options[i].QualityChangeTime)
        {
            qDebug() << "**** QUALITY CHANGE **** Option " << i+1 << "had quality "<< m_Options[i].quality << ", now its quality is" << m_Options[i].QualityAfterChange;
            m_Options[i].quality=m_Options[i].QualityAfterChange;
            m_Options[i].QualityChangeApplied = true;
        }
    }
}

// return the option ID for a given kilobot colour
unsigned int mykilobotenvironment::indexOptOfColour(lightColour kColor) {
    switch(kColor){
    case RED:
        return 1;
    case GREEN:
        return 3;
    case BLUE:
        return 2;
    default:
        return 0;
    }
}

// generate virtual sensor readings & send to KB
void mykilobotenvironment::updateVirtualSensor(Kilobot kilobot) {
    kilobot_id kID = kilobot.getID();
    QPointF kPos = kilobot.getPosition();
    lightColour kColor=kilobot.getLedColour();
    
    //     qDebug() << "UpdateMessagingQueue: " << UpdateMessagingQueue[kID];

    // Check discovery virtual-sensor
    option Op;
    for (int i=0;i<m_Options.size();i++ ) {
        Op=m_Options[i];
        float rad=Op.rad;
        float x=Op.posX;
        float y=Op.posY;
        //Check if the robot is inside an option for discovery
        if((pow(kPos.x()-x,2)+pow(kPos.y()-y,2)) < pow(rad,2) )
        {
            if(this->m_time>=Op.AppearanceTime && (this->m_time<Op.DisappearanceTime||Op.DisappearanceTime<=0)){
                m_Single_Discovery[kID]=Op.ID;
                m_optionStillThere[kID]=true;
            }
            else
            {
                m_Single_Discovery[kID]=Op.ID;
                m_optionStillThere[kID]=false;
            }
            // if robot is within the option it wanted to resample, deactivate the resampling toggle
            if (Op.ID == indexOptOfColour(kColor)) {
                goingResamplingList[kID] = false;
            }
        }
    }
    if (OhcBufferIsEmpty && kiloGroupForMessaging.at(msgGroup).contains(kID)) { // only send messages when the kID is in the assigned group

        kilobot_message msg;
        unsigned int m_message_type;

        uint8_t on_option=0;

        /** Update virtual-GPS */

        // Get robot's GPS coordinates
        QPoint GPS_cords=PositionToGPS(kPos);

        // Set msg type
        m_message_type=0; // 0->GPS 1->Discovery

        // Get robot's orientation
        double orientDegrees = qRadiansToDegrees(qAtan2(-kilobot.getVelocity().y(),kilobot.getVelocity().x()));

        orientDegrees=desNormAngle(orientDegrees);

        if(m_Single_Discovery[kID]!=0 && m_optionStillThere[kID])
        {
            on_option = 1;
        }

        msg.id= kID<<2 | on_option <<1 |m_message_type;
        msg.type= ( (uint8_t) GPS_cords.x() ) & 0x0F;
        msg.data= ( (uint8_t) ( 31-GPS_cords.y() )  ) << 6 | ( (uint8_t) (orientDegrees/12.0) );
        emit transmitKiloState(msg);
        //qDebug() << "Sending GPS (" << GPS_cords.x() << "," << 31-GPS_cords.y() << ")[o:"<<on_option<<"] to robot " << kID;

        if(m_Single_Discovery[kID]!=0 && m_optionStillThere[kID]) {
            // Update option virtual-sensor
            m_message_type=1;

            on_option = 1;
            //qDebug() << "Sending discovery " << m_Single_Discovery[kID] << " to robot " << kID;
            msg.id= kID<<2 | on_option <<1 |m_message_type;
            msg.type= ( (uint8_t) m_Options[m_Single_Discovery[kID]-1].GPS_X ) & 0x0F;
            msg.data= ( (uint8_t) m_Options[m_Single_Discovery[kID]-1].GPS_Y ) << 6 | ( (uint8_t) m_Options[m_Single_Discovery[kID]-1].quality );
            emit transmitKiloState(msg);
            m_Single_Discovery[kID]=0;

        }
    }

}


QPoint mykilobotenvironment::PositionToGPS(QPointF position){
    return QPoint((unsigned int) std::ceil(position.x()/m_fCellLength )-1, (unsigned int) std::ceil(position.y()/m_fCellLength )-1);
}

float mykilobotenvironment::desNormAngle(float angle){
    while (angle > 360) angle = angle - 360;
    while (angle < 0) angle = 360 + angle;
    return angle;
}
