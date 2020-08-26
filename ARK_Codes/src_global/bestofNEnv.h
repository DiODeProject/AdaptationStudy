#ifndef MYKILOBOTENVIRONMENT_H
#define MYKILOBOTENVIRONMENT_H

#include <QObject>
#include <QPointF>
#include <QVector>
#include <kilobotenvironment.h>
#include <QTime>
#include <QColor>

static const double MM_TO_PIXEL = 2000.0/2000.0;
static const double PIXEL_TO_MM = 2000.0/2000.0;

static const double M_TO_PIXEL = 1000.0;

struct option {
    unsigned int ID;
    float quality;
    float posX; // in pixel
    float posY; // in pixel
    unsigned int GPS_X; // in cells
    unsigned int GPS_Y; // in cells
    float rad;  // in pixel
    QColor color;

    float AppearanceTime=0;
    float DisappearanceTime=0;

    float QualityChangeTime=0;
    float QualityAfterChange;
    bool QualityChangeApplied=false;
};

class mykilobotenvironment : public KilobotEnvironment
{
    Q_OBJECT
public:
    explicit mykilobotenvironment(QObject *parent = 0);
    QPoint PositionToGPS(QPointF position);
    float desNormAngle(float angle);

    // Current time (necessary to check about disappearance and quality swap)
    float m_time;

    // A vector containing the options
    QVector <option> m_Options;

    // Discovery variables
    QVector <unsigned int> m_Single_Discovery;
    QVector <unsigned int> m_optionStillThere;

    // Global communication variables
    QVector <bool> goingResamplingList;
    unsigned int indexOptOfColour(lightColour kColor);
    bool OhcBufferIsEmpty=true;
    int msgGroup;
    QVector<QVector<kilobot_id> > kiloGroupForMessaging;

    // GPS variables
    unsigned int m_unGpsCells=32;
    float m_fCellLength=0;
    int GPS_max_x=15; // max GPS value on x axis (to correct orientation of robot colliding with walls)
    int GPS_max_y=15; // max GPS value on y axis (to correct orientation of robot colliding with walls)

    // Wall avoidance variables
    QVector <bool> m_wall_detected;

signals:
    void errorMessage(QString);

public slots:
    void update();
    void updateVirtualSensor(Kilobot kilobot);

private:

};
#endif // MYKILOBOTENVIRONMENT_H
