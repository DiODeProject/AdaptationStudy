/**
 * @file <ark.h>
 *
 * @brief This is the source file of ALF, the ARK (Augmented Reality for Kilobots) loop function.
 *
 * @cite Reina, A., Cope, A. J., Nikolaidis, E., Marshall, J. A. R., & Sabo, C. (2017). ARK: Augmented Reality for Kilobots.
 * IEEE Robotics and Automation Letters, 2(3), 1755–1761. https://doi.org/10.1109/LRA.2017.2700059
 *
 * @cite Pinciroli, C., Talamali, M. S., Reina, A., Marshall, J. A. R., and Trianni, V. (2018). Simulating Kilobots within ARGoS: models and experimental validation.
 * In Swarm Intelligence (ANTS 2018), volume 11172 of LNCS, pages 176–187. Springer. https://doi.org/10.1007/978-3-030-00533-7_14
 *
 * @author Mohamed Salaheddine Talamali <mstalamali@gmail.com>
 */



#ifndef ARK_H
#define ARK_H

namespace argos {
class CSpace;
class CRay3;
class CFloorEntity;
class CSimulator;
}

#include <math.h>

#include <behaviours/agentCDCIlocal.h>

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>

#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_medium.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_default_actuator.h>
#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_controller.h>

//kilobot messaging
#include <argos3/plugins/robots/kilobot/control_interface/kilolib.h>
#include <argos3/plugins/robots/kilobot/control_interface/message_crc.h>
#include <argos3/plugins/robots/kilobot/control_interface/message.h>

#include <array>


using namespace argos;


class CArk : public CLoopFunctions
{

public:
    /************************************************/
    /*       Basic ARGoS Loop function methods      */
    /************************************************/
    CArk();
    virtual ~CArk();
    virtual void Init(TConfigurationNode& t_tree);
    virtual void Reset();
    virtual void Destroy();
    virtual void PreStep();
    virtual void PostStep();
    virtual CColor GetFloorColor(const CVector2& vec_position_on_plane); // Used to plot the Virtual environment on the floor

    /************************************************/
    /*          ARK Loop function methods           */
    /************************************************/
    /** Initialization related methods */
    /* Get a Vector of all the Kilobots in the space */
    void GetKilobotsEntities();

    /* Get debug infromation of all kilobots */
    void GetDebugInfo();

    /* Setup the initial state of the Kilobots in the space */
    void SetupInitialKilobotsStates();

    /* Setup the initial state of the kilobot pc_kilobot_entity */
    void SetupInitialKilobotState(CKilobotEntity& c_kilobot_entity);

    /** Experiment configuration methods (From .argos files) */
    /* Set tracking type for the experiments */
    void SetTrackingType(TConfigurationNode& t_tree);

    /* Setup virtual environment */
    void SetupVirtualEnvironments(TConfigurationNode& t_tree);

    /* Get experiment variables */
    void GetExperimentVariables(TConfigurationNode& t_tree);

    /** Virtual environment visualization updating */
    /* Plot the environment */
    void PlotEnvironment();


    /** Kilobots' states updating methods */
    /* Get the messages to send to the Kilobots according their positions */
    void UpdateKilobotsState();

    /* Get the message to send to a Kilobot according to its position */
    void UpdateKilobotState(CKilobotEntity& c_kilobot_entity);

    /** Virtual Sensors updating methods */
    /* Get the messages to send to the Kilobots according their positions */
    void UpdateVirtualSensors();

    /* Get the message to send to a Kilobot according to its position */
    void UpdateVirtualSensor(CKilobotEntity& c_kilobot_entity);

    /** Virtual Envirenment Updating methods */
    /* Update the virtual environment if dynamics */
    void UpdateVirtualEnvironments();

    /* Update the virtual environment if dynamics */
    void UpdatesVirtualEnvironmentsBasedOnKilobotState(CKilobotEntity& c_kilobot_entity);

    /** Kilobot Tracking methods */
    /* Get the position of a Kilobot */
    CVector2 GetKilobotPosition(CKilobotEntity& c_kilobot_entity);

    /* Get the orientation of a Kilobot */
    CRadians GetKilobotOrientation(CKilobotEntity& c_kilobot_entity);

    /* Get the kilobot ID */
    UInt16 GetKilobotId(CKilobotEntity& c_kilobot_entity);

    /* Get the kilobot LED color */
    CColor GetKilobotLedColor(CKilobotEntity& c_kilobot_entity);

    /* Create a circular arena */
    void CreateCircularArena(TConfigurationNode &t_tree);

    /* Add one option */
    void AddOption(TConfigurationNode& t_node);

    /* Convert a position in the arena to a GPS coordinates (grid) */
    CVector2 PositionToGPS( CVector2 t_position );

private:
    /************************************************/
    /*  Basic attributes of the ARK loop function   */
    /************************************************/

    /* Random number generator */
    CRandom::CRNG* m_pcRNG;

    /* List of the Kilobots in the space */
    typedef std::vector<CKilobotEntity*> TKilobotEntitiesVector;
    TKilobotEntitiesVector m_tKilobotsEntities;

    /* List of the messages sent by communication entities */
    typedef std::vector<message_t> TKilobotsMessagesVector;
    TKilobotsMessagesVector m_tMessages;

    /* ARK kilobot message*/
    typedef struct {
        UInt8 m_sType:4;
        UInt16 m_sID:10;
        UInt16 m_sData:10;
    } m_tArkKilobotMessage;

    /* Tracking Flags*/
    bool m_bPositionTracking;
    bool m_bOrientationTracking;
    bool m_bColorTracking;

    /************************************/
    /*  Virtual Environment variables   */
    /************************************/
    /* virtual environment struct*/
    struct m_sOption
    {
        UInt16 id;
        Real quality;
        CVector2 position;
        CVector2 GPS_position;
        Real radius;
        CColor color;
        Real AppearanceTime=0;
        Real DisappearanceTime=0;
        Real QualityChangeTime=0;
        Real qualityAfterChange;
    };

    /* options vector */
    typedef std::vector<m_sOption> TOptionsVector;
    TOptionsVector m_tOptions;

    /* Vector containing information of the virtual robot states */
    std::vector<Real> tLastTimeMessagedGPS;
    std::vector<Real> tLastTimeMessagedDiscovery;

    std::vector<bool> tRequiresGPS;
    std::vector< SInt16 > tEncounteredOption;

    /* Time for one kilobot message to be sent */
    Real m_fTimeForAMessage;

    /* Time for one kilobot message to be sent */
    Real MinTimeBetweenTwoMsg;

    /************************************/
    /*       Experiment variables       */
    /************************************/
    /* Experiment time in seconds */
    Real m_fTimeInSeconds;

    /* Counters for messages and data acquizition */
    UInt16 m_unMessagingCounter,m_unDataSavingCounter;

    /* Is the virtual environment dynamics? If yes update its plotting */
    bool m_bDynamicVirtualEnvironments;

    /* Environment plot update frequency in ticks */
    UInt16 m_unEnvironmentPlotUpdateFrequency;

    /* output file for data acquizition */
    std::ofstream m_cOutput;

    /* output file name*/
    std::string m_strOutputFileName;

    /* data acquisition frequency in ticks */
    UInt16 m_unDataFrequency;

    /* OHC broadcast frequency */
    UInt16 m_unOhcFrequency;

    /* quorum */
    Real m_fQuorum;

    /* quorum in robots number */
    Real m_fQuorumRobots;

    /* A flag for a kilbot needs a message to be sent or not */
    bool m_bQuorumReached;

    /* Number of GPS cells */
    UInt16 m_unGpsCells;

    /* GPS cell length in meters */
    Real m_fCellLength;

    /* Vector containing the commitement state of a robot */
    std::vector<unsigned int> m_tCommitmentState;

    // DEBUGGING INFORMATION
    //
    // This is an efficient way to store both controllers and debug information.
    std::vector<debug_info_t*> m_tKBs;


    /* Option's lookup table sending */
    bool m_bOptionsLookUpTableSent;
    UInt8 initial_opinion=2;
    SInt16 m_unOptionsSent;
    Real m_fvariance=0.0;

    /* Vector to store the times where the simulator need to update the virtual environment (to not do checks every time-step as it is expensive) */
    std::vector<Real> interestingTimes;

};


// There should be two functions


#endif
