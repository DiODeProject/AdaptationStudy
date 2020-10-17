/**
 * @file <ark.cpp>
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

#include "ark.h"


/** **************************************/
/**     Default LoopFunction Methods     */
/** **************************************/

CArk::CArk(): CLoopFunctions(),m_fTimeInSeconds(0),m_unMessagingCounter(0), m_unDataSavingCounter(1),m_bDynamicVirtualEnvironments(false),
    m_unEnvironmentPlotUpdateFrequency(10),m_unDataFrequency(100),m_unOhcFrequency(10),
    m_fQuorum(1.0),m_bQuorumReached(false),m_bOptionsLookUpTableSent(false),m_unOptionsSent(-1){}
CArk::~CArk(){
}

/****************************************/
/****************************************/

void CArk::Init(TConfigurationNode& t_node) {

    /* Create random number generator */
    m_pcRNG = CRandom::CreateRNG("argos");

    /* Create the circular arena */
    if(NodeExists(t_node,"circular_arena")){
        CreateCircularArena(t_node);
    }

    /* Set the tracking type from the .argos file*/
    SetTrackingType(t_node);

    /* Get experiment variables from the .argos file*/
    GetExperimentVariables(t_node);

    /* Get the virtual environment from the .argos file */
    SetupVirtualEnvironments(t_node);

    /* Get the initial kilobots' states */
    SetupInitialKilobotsStates();

    /* Get kilobots' debug info*/
    GetDebugInfo();

    /* Other initializations: Varibales, Log file opening... */

    // Open a log file
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);

    // write the log file header (it is not mendatory)
    m_cOutput << "time;";
    for(unsigned int i=0;i<m_tOptions.size();i++){
        m_cOutput << i << ";";
    }

    if(m_tOptions.size()>0){
        m_cOutput << m_tOptions.size()<< std::endl;;
    }

    // Intializing variables
    m_unMessagingCounter=m_unOhcFrequency+1;
    m_fQuorumRobots=m_fQuorum*m_tKilobotsEntities.size();
    m_tCommitmentState.resize(m_tOptions.size()+1);
}

/****************************************/
/****************************************/

void CArk::Reset() {

    /* Reset counters */
    m_unDataSavingCounter = m_unOhcFrequency+1;
    m_unMessagingCounter = 0;

    /* Close data file */
    m_cOutput.close();

    /* Reopen the file, erasing its contents */
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);
    // write the log file header (it is not mendatory)
    m_cOutput << "time;";
    for(unsigned int i=0;i<m_tOptions.size();i++){
        m_cOutput << i << ";";
    }
    if(m_tOptions.size()>0){
        m_cOutput << m_tOptions.size()<< std::endl;;
    }
}

/****************************************/
/****************************************/

void CArk::Destroy() {
    /* Close data file */
    m_cOutput.close();
}

/****************************************/
/****************************************/

void CArk::PreStep(){

    /* Update the time variable required for the experiment (in sec)*/
    m_fTimeInSeconds=GetSpace().GetSimulationClock()/CPhysicsEngine::GetInverseSimulationClockTick();

    /* Update the state of the kilobots in the space*/
    UpdateKilobotsState();

    /* Update the virtual sensor of the kilobots*/
    UpdateVirtualSensors();

    /* Update the virtual environment*/
    UpdateVirtualEnvironments();

    /* Update the virtual environment plot*/
    PlotEnvironment();
}

/****************************************/
/****************************************/

void CArk::PostStep(){

    /* Save experiment data to the specified log file*/
    std::fill(m_tCommitmentState.begin(), m_tCommitmentState.end(), 0);
    for(unsigned int i=0;i< m_tKilobotsEntities.size();i++)
    {
        m_tCommitmentState[((unsigned int) m_tKBs[i]->commitement)]++;
        if( (m_tKBs[i]->commitement!=0) && (m_fQuorumRobots>0) && (m_tCommitmentState[((unsigned int) m_tKBs[i]->commitement)]>=m_fQuorumRobots) ){
            m_bQuorumReached=true;
        }
    }

    if(  m_bQuorumReached || ((m_unDataSavingCounter%m_unDataFrequency == 0) && (m_unDataSavingCounter!=0)) ||
         ((GetSpace().GetSimulationClock() >= GetSimulator().GetMaxSimulationClock()) && (GetSimulator().GetMaxSimulationClock()>0)))
    {
        m_cOutput << GetSpace().GetSimulationClock();

        for(unsigned int i=0;i< m_tCommitmentState.size();i++){
            m_cOutput << ";" << m_tCommitmentState[i];
        }
        m_cOutput<< std::endl;
    }

    if(m_bQuorumReached==true){
        GetSimulator().Terminate();
    }

    m_unDataSavingCounter++;
}


/** **************************************/
/**      Kilobot Tracking Functions      */
/** **************************************/

CVector2 CArk::GetKilobotPosition(CKilobotEntity& c_kilobot_entity){
    CVector2 vecKilobotPosition(c_kilobot_entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),c_kilobot_entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
    return vecKilobotPosition;
}

/****************************************/
/****************************************/

CRadians CArk::GetKilobotOrientation(CKilobotEntity& c_kilobot_entity)
{

    CRadians cZAngle;
    CRadians cYAngle;
    CRadians cXAngle;

    //Calculate the orientations of the kilobot
    CQuaternion cRobotOrientations = c_kilobot_entity.GetEmbodiedEntity().GetOriginAnchor().Orientation;

    cRobotOrientations.ToEulerAngles(cZAngle,cYAngle, cXAngle);

    return cZAngle;
}

/****************************************/
/****************************************/

UInt16 CArk::GetKilobotId(CKilobotEntity& c_kilobot_entity){
    std::string strKilobotID((c_kilobot_entity).GetControllableEntity().GetController().GetId());
    return std::stoul(strKilobotID.substr(2));
}

/****************************************/
/****************************************/

CColor CArk::GetKilobotLedColor(CKilobotEntity &c_kilobot_entity){
    return c_kilobot_entity.GetLEDEquippedEntity().GetLED(0).GetColor();
}

/** ***************************************/
/**        Initialization functions       */
/** ***************************************/

void CArk::GetKilobotsEntities(){
    /*
     * Go through all the robots in the environment
     * and create a vector of pointers on their entities
     */

    /* Get the map of all kilobots from the space */
    CSpace::TMapPerType& mapKilobots=GetSpace().GetEntitiesByType("kilobot");
    /* Go through them */
    for(CSpace::TMapPerType::iterator it = mapKilobots.begin();
        it != mapKilobots.end();
        ++it) {
        m_tKilobotsEntities.push_back(any_cast<CKilobotEntity*>(it->second));
    }

}

/****************************************/
/****************************************/

void CArk::GetDebugInfo(){
    m_tKBs.clear();
    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        /* Check if there is a message to send to the kilobot "i" and get the message */

        /*
         * When the 'reset' method is called on the kilobot controller, the
         * kilobot state is destroyed and recreated. Thus, we need to
         * recreate the list of controllers and debugging info from scratch
         * as well.
         */
        /* Create a pointer to the current kilobot */
        CCI_KilobotController* pcKBC = &dynamic_cast<CCI_KilobotController&>(m_tKilobotsEntities[it]->GetControllableEntity().GetController());
        /* Create debug info for controller */
        debug_info_t* ptDebugInfo = pcKBC->DebugInfoCreate<debug_info_t>();
        /* Append to list */
        m_tKBs.push_back(ptDebugInfo);
    }
}

/****************************************/
/****************************************/

void CArk::SetupInitialKilobotsStates(){
    /* Get the Kilobots entities from the space.*/
    GetKilobotsEntities();

    /* Create Kilobots individual messages */
    m_tMessages=TKilobotsMessagesVector(m_tKilobotsEntities.size());

    tLastTimeMessagedGPS.resize(m_tKilobotsEntities.size());
    tLastTimeMessagedDiscovery.resize(m_tKilobotsEntities.size());
    tRequiresGPS.resize(m_tKilobotsEntities.size());
    tEncounteredOption.resize(m_tKilobotsEntities.size());

    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        /* Setup the virtual states of a kilobot(e.g. has food state)*/
        SetupInitialKilobotState(*m_tKilobotsEntities[it]);
    }
}

/****************************************/
/****************************************/

void CArk::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity){
    /* Initialize the individuals variables (such as states) */
    tLastTimeMessagedGPS[GetKilobotId(c_kilobot_entity)]=-1000;
    tLastTimeMessagedDiscovery[GetKilobotId(c_kilobot_entity)]=-1000;
    tRequiresGPS[GetKilobotId(c_kilobot_entity)]=false;
    tEncounteredOption[GetKilobotId(c_kilobot_entity)]=-1;
}

/****************************************/
/****************************************/

void CArk::SetTrackingType(TConfigurationNode& t_tree){

    TConfigurationNode& tTrackingNode=GetNode(t_tree,"tracking");

    GetNodeAttribute(tTrackingNode, "position", m_bPositionTracking);

    GetNodeAttribute(tTrackingNode, "orientation", m_bOrientationTracking);

    GetNodeAttribute(tTrackingNode, "color", m_bColorTracking);
}

/****************************************/
/****************************************/

void CArk::SetupVirtualEnvironments(TConfigurationNode& t_tree){

    TConfigurationNode& tVirtualEnvironmentsNode=GetNode(t_tree,"environments");
    std::string type;
    GetNodeAttribute(tVirtualEnvironmentsNode,"type",type);
    if(type=="manual"){
        TConfigurationNodeIterator itNodes;

        for(itNodes=itNodes.begin(&tVirtualEnvironmentsNode); itNodes!=itNodes.end(); ++itNodes)
        {
            if(itNodes->Value()=="option"){
                AddOption(*itNodes);
            }
        }

        std::vector<int> posShuffling={1,2,3};

        for (int k=0;k<m_tOptions.size();k++)
        {
            UInt32 pos=m_pcRNG->Uniform(CRange<UInt32>(0,posShuffling.size()));

            float polygonradius=0.285;
            CRadians angleoffset(0);
            UInt16 numberofoptions=m_tOptions.size();
            float Xoffset=-0.5;
            float Yoffset=-0.5;

            int i = posShuffling[pos];
            posShuffling.erase(posShuffling.begin()+pos);
            m_tOptions[k].position=CVector2(polygonradius*Cos(angleoffset+(numberofoptions-i)*CRadians::TWO_PI/numberofoptions)+1+Xoffset,polygonradius*Sin(angleoffset+(numberofoptions-i)*CRadians::TWO_PI/numberofoptions)+1+Yoffset);
            m_tOptions[k].GPS_position=PositionToGPS(m_tOptions[k].position);
        }
    }

    if(type=="automatic"){
        /*
         *  This part change based on how I want to automatically distribute my options in the space
         *  For example here I will place my options on a polygon with centre is the arena centre
         */

        UInt16 numberofoptions;
        Real polygonradius;
        Real bestquality;
        Real kappa;
        Real optionsradius;
        Real Xoffset=0;
        Real Yoffset=0;

        // Implementation of size bias
        Real optionsradiusbias=0;
        UInt16 numberofbiasedoptions=0;

        // Getting the environment variables from the .argos file
        GetNodeAttribute(tVirtualEnvironmentsNode,"numberofoptions",numberofoptions);
        GetNodeAttribute(tVirtualEnvironmentsNode,"polygonradius",polygonradius);
        GetNodeAttribute(tVirtualEnvironmentsNode,"bestquality",bestquality);
        GetNodeAttribute(tVirtualEnvironmentsNode,"kappa",kappa);
        GetNodeAttribute(tVirtualEnvironmentsNode,"optionsradius",optionsradius);

        GetNodeAttributeOrDefault(tVirtualEnvironmentsNode,"xoffset",Xoffset,Xoffset);
        GetNodeAttributeOrDefault(tVirtualEnvironmentsNode,"yoffset",Yoffset,Yoffset);

        GetNodeAttributeOrDefault(tVirtualEnvironmentsNode,"optionsradiusbias",optionsradiusbias,optionsradiusbias);
        GetNodeAttributeOrDefault(tVirtualEnvironmentsNode,"numberofbiasedoptions",numberofbiasedoptions,numberofbiasedoptions);

        bool angularoffset=false;
        GetNodeAttribute(tVirtualEnvironmentsNode,"angularoffset",angularoffset);

        CRadians angleoffset=CRadians(0);
        if(angularoffset){
            angleoffset=((Real)m_pcRNG->Uniform(CRange<UInt32>(0,numberofoptions))/(Real)numberofoptions)*CRadians::TWO_PI;
        }

        m_sOption sOption;

        for(unsigned int i=1; i<=numberofoptions;i++){
            sOption.id=i;
            if(i==numberofoptions){
                sOption.quality=bestquality;
                sOption.radius=optionsradius;
            }
            else{
                sOption.quality=bestquality*kappa;
                sOption.radius=optionsradius+(numberofbiasedoptions>0)*optionsradiusbias;
                if(sOption.radius!=optionsradius){
                    numberofbiasedoptions--;
                }
            }
            //            sOption.position=CVector2(polygonradius*Cos(angleoffset+(i-1)*CRadians::TWO_PI/numberofoptions),polygonradius*Sin(angleoffset+(i-1)*CRadians::TWO_PI/numberofoptions));
            sOption.position=CVector2(polygonradius*Cos(angleoffset+(numberofoptions-i)*CRadians::TWO_PI/numberofoptions)+1+Xoffset,polygonradius*Sin(angleoffset+(numberofoptions-i)*CRadians::TWO_PI/numberofoptions)+1+Yoffset);
            sOption.GPS_position=PositionToGPS(sOption.position);
            sOption.color=CColor(255,255-sOption.quality/bestquality*255,255-sOption.quality/bestquality*255);
            m_tOptions.push_back(sOption);
        }
    }

}

/****************************************/
/****************************************/

void CArk::GetExperimentVariables(TConfigurationNode& t_tree){

    TConfigurationNode& tExperimentVariablesNode = GetNode(t_tree,"variables");

    /* Get the output datafile name and open it */
    GetNodeAttribute(tExperimentVariablesNode, "datafilename", m_strOutputFileName);

    /* Get the frequency of data saving */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "frequency", m_unDataFrequency, m_unDataFrequency);

    /* Get the OHC broadcast frequency */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "ohcfrequency", m_unOhcFrequency, m_unOhcFrequency);

    /* Get the quorum value */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "quorum", m_fQuorum, m_fQuorum);

    /* Get if the virtual environments are dynamics or not */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "dynamic_virtual_environment", m_bDynamicVirtualEnvironments, m_bDynamicVirtualEnvironments);

    /* Get the time for one kilobot message */
    MinTimeBetweenTwoMsg=5.0; // ALF will send virtual sensors information to the robots every 5 sec (the same as ARK in the real robot experiments)

    /* Get the time for one kilobot message */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "gps_cells", m_unGpsCells, m_unGpsCells);
    m_fCellLength=GetSpace().GetArenaSize().GetX()/m_unGpsCells;
    LOG << "GPS cell length is: " << m_fCellLength << std::endl;

    /* Get noise variance value */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "quality_variance", m_fvariance, m_fvariance);

}

/** **************************************/
/**          Updating functions          */
/** **************************************/

void CArk::UpdateKilobotsState(){
    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        /* Update the virtual states and actuators of the kilobot*/
        UpdateKilobotState(*m_tKilobotsEntities[it]);
    }
}

/****************************************/
/****************************************/

void CArk::UpdateKilobotState(CKilobotEntity &c_kilobot_entity){

    /* Check if the kilobot required its GPS location */
    if(GetKilobotLedColor(c_kilobot_entity)==CColor::RED){
        tRequiresGPS[GetKilobotId(c_kilobot_entity)]=true;
    }
    else{
        tRequiresGPS[GetKilobotId(c_kilobot_entity)]=false;
    }

    /* Check if the robot encountered an options */
    std::vector <UInt16> vecInOptions;
    for(UInt16 it = 0;it < m_tOptions.size(); ++it) {
        if((pow(GetKilobotPosition(c_kilobot_entity).GetX()-m_tOptions[it].position.GetX(),2)+pow(GetKilobotPosition(c_kilobot_entity).GetY()-m_tOptions[it].position.GetY(),2))<pow(m_tOptions[it].radius,2)) {
            vecInOptions.push_back(it);
        }
    }
    if(vecInOptions.size() > 0){
        tEncounteredOption[GetKilobotId(c_kilobot_entity)]=vecInOptions[m_pcRNG->Uniform(CRange<UInt32>(0,vecInOptions.size()))];
        //        LOG << " ROBOT: " << GetKilobotId(c_kilobot_entity)<< "encountered options" <<  tEncounteredOption[GetKilobotId(c_kilobot_entity)] <<std::endl;
    }

}

/****************************************/
/****************************************/

void CArk::UpdateVirtualSensors(){

    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        /* Update the virtual sensor of a kilobot based on its current state */
        UpdateVirtualSensor(*m_tKilobotsEntities[it]);
    }

    /* Update flags */
    if(m_unOptionsSent < (SInt16) m_tOptions.size()){
        m_unOptionsSent++;
    }

    if(m_unOptionsSent== (SInt16)m_tOptions.size()){
        m_bOptionsLookUpTableSent=true;
    }
}

/****************************************/
/****************************************/

void CArk::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity){
    /* broadcast Options lookup table at the beggining */
    if(!m_bOptionsLookUpTableSent){

        if(m_unOptionsSent>=0)
        {
            /* Prepare the message */
            m_tMessages[GetKilobotId(c_kilobot_entity)].type=10;
            m_tMessages[GetKilobotId(c_kilobot_entity)].data[0]=(UInt8) m_tOptions[m_unOptionsSent].id;
            m_tMessages[GetKilobotId(c_kilobot_entity)].data[1]=(UInt8) m_tOptions[m_unOptionsSent].GPS_position.GetX();
            m_tMessages[GetKilobotId(c_kilobot_entity)].data[2]=(UInt8) m_tOptions[m_unOptionsSent].GPS_position.GetY();
            m_tMessages[GetKilobotId(c_kilobot_entity)].data[3]=(UInt8) (m_tOptions[m_unOptionsSent].radius*100);
            //        m_tMessages[GetKilobotId(c_kilobot_entity)].data[4]=16;
            m_tMessages[GetKilobotId(c_kilobot_entity)].data[4]=4;
            if(initial_opinion!=0)
            {
                m_tMessages[GetKilobotId(c_kilobot_entity)].data[5]=(UInt8) initial_opinion;
                m_tMessages[GetKilobotId(c_kilobot_entity)].data[6]=(UInt8) m_tOptions[initial_opinion-1].quality;
                m_tMessages[GetKilobotId(c_kilobot_entity)].data[7]=(UInt8) m_tOptions[initial_opinion-1].GPS_position.GetX();
                m_tMessages[GetKilobotId(c_kilobot_entity)].data[8]=(UInt8) m_tOptions[initial_opinion-1].GPS_position.GetY();
            }

            /* Debugging */
            /*        LOG << "SENDING MESSAGE: "
            << m_tMessages[GetKilobotId(c_kilobot_entity)].data[0]
            << m_tMessages[GetKilobotId(c_kilobot_entity)].data[1]
            << m_tMessages[GetKilobotId(c_kilobot_entity)].data[2]
            << "TO ROBOT: " << GetKilobotId(c_kilobot_entity) << std::endl;
        */

            /* Send the message */
            GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[GetKilobotId(c_kilobot_entity)]);
        }
    }
    else{
        /* check if enough time has passed from the last message otherwise*/
        //        if {
        //            GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,NULL);
        //            return; // if the time is too short, the kilobot cannot receive a message
        //        }

        /* Message type */
        UInt8 b_MessageType=0;

        /* Flag for existance of message to send*/
        bool bMessageToSend=false;

        /*Create ARK-type messages variables*/
        m_tArkKilobotMessage tKilobotMessage,tEmptyMessage,tMessage;


        /* Discovery Messages*/
        if( ( tEncounteredOption[GetKilobotId(c_kilobot_entity)] != -1) && (m_fTimeInSeconds - tLastTimeMessagedDiscovery[GetKilobotId(c_kilobot_entity)]> MinTimeBetweenTwoMsg ))
        {
            if(m_fTimeInSeconds>=m_tOptions[tEncounteredOption[GetKilobotId(c_kilobot_entity)]].AppearanceTime && (m_fTimeInSeconds<m_tOptions[tEncounteredOption[GetKilobotId(c_kilobot_entity)]].DisappearanceTime || m_tOptions[tEncounteredOption[GetKilobotId(c_kilobot_entity)]].DisappearanceTime<=0))
            {
                /* Set the message type to discovery */
                b_MessageType=1;

                /* On option?*/
                UInt8 on_option=1;


                /* Fill the message */
                tKilobotMessage.m_sID = GetKilobotId(c_kilobot_entity) << 2 | on_option << 1 | b_MessageType;
                tKilobotMessage.m_sType = ( (UInt8) m_tOptions[ tEncounteredOption[GetKilobotId(c_kilobot_entity)] ].GPS_position.GetX() ) & 0x0F;
                tKilobotMessage.m_sData = ( (UInt8) m_tOptions[ tEncounteredOption[GetKilobotId(c_kilobot_entity)] ].GPS_position.GetY() ) << 6 | (UInt8) m_tOptions[ tEncounteredOption[GetKilobotId(c_kilobot_entity)] ].quality ;

                /* Reset the state of the robots */
                tEncounteredOption[GetKilobotId(c_kilobot_entity)]=-1;


                /* Debugging messages */
                //        LOG << " The robot's goal is: " << m_tOptions[ vecInOptions[op] ].GPS_position << std::endl;
                //        LOG << " The robot's goal has quality: " << m_tOptions[ vecInOptions[op] ].quality << std::endl;


                /*  Set the message sending flag to True */
                bMessageToSend=true;

                tLastTimeMessagedDiscovery[GetKilobotId(c_kilobot_entity)] = m_fTimeInSeconds;
            }
        }



        /* GPS Messages*/
        //        if((!bMessageToSend)&&tRequiresGPS[GetKilobotId(c_kilobot_entity)]&& (m_fTimeInSeconds - tLastTimeMessagedGPS[GetKilobotId(c_kilobot_entity)]> MinTimeBetweenTwoMsg ) )
        if((!bMessageToSend)&& (m_fTimeInSeconds - tLastTimeMessagedGPS[GetKilobotId(c_kilobot_entity)]> MinTimeBetweenTwoMsg ) )
        {

            /* Set the message type to GPS */
            b_MessageType=0;

            /* On option?*/
            UInt8 on_option=0;

            if( tEncounteredOption[GetKilobotId(c_kilobot_entity)] != -1)
            {
                if(m_fTimeInSeconds>=m_tOptions[tEncounteredOption[GetKilobotId(c_kilobot_entity)]].AppearanceTime && (m_fTimeInSeconds<m_tOptions[tEncounteredOption[GetKilobotId(c_kilobot_entity)]].DisappearanceTime || m_tOptions[tEncounteredOption[GetKilobotId(c_kilobot_entity)]].DisappearanceTime<=0))
                {
                    on_option= 1;
                }
            }

            /* Fill the message */
            tKilobotMessage.m_sID = GetKilobotId(c_kilobot_entity) << 2 | on_option << 1 | b_MessageType ;
            tKilobotMessage.m_sType = ( (UInt8) PositionToGPS(GetKilobotPosition(c_kilobot_entity)).GetX() ) & 0x0F;

            CDegrees KB_Orientation = ToDegrees( GetKilobotOrientation(c_kilobot_entity) );
            KB_Orientation.UnsignedNormalize();

            tKilobotMessage.m_sData = ( (UInt8) PositionToGPS(GetKilobotPosition(c_kilobot_entity)).GetY() ) << 6 | ( (UInt8) (KB_Orientation.GetValue()/12.0) ) ;

            //            LOG << " GPS location is : " << PositionToGPS(GetKilobotPosition(c_kilobot_entity))<< std::endl;
            //            LOG << " Sent orientation : " << (UInt16) KB_Orientation.GetValue() << std::endl;

            tRequiresGPS[GetKilobotId(c_kilobot_entity)]=false;

            /*  Set the message sending flag to True */
            bMessageToSend=true;

            tLastTimeMessagedGPS[GetKilobotId(c_kilobot_entity)] = m_fTimeInSeconds;
        }

        /* Send the message to the kilobot using the ARK messaging protocol (addressing 3 kilobots per one standard kilobot message)*/
        if(bMessageToSend){

            for (int i = 0; i < 9; ++i) {
                m_tMessages[GetKilobotId(c_kilobot_entity)].data[i]=0;
            }
            m_tMessages[GetKilobotId(c_kilobot_entity)].type=0;

            // Prepare an empty ARK-type message to fill the gap in the full kilobot message
            tEmptyMessage.m_sID=1023;
            tEmptyMessage.m_sType=0;
            tEmptyMessage.m_sData=0;

            // Fill the kilobot message by the ARK-type messages

            for (int i = 0; i < 3; ++i) {

                if(i==0){
                    tMessage=tKilobotMessage;
                } else{
                    tMessage=tEmptyMessage;
                }

                m_tMessages[GetKilobotId(c_kilobot_entity)].data[i*3] = (tMessage.m_sID >> 2);
                m_tMessages[GetKilobotId(c_kilobot_entity)].data[1+i*3] = (tMessage.m_sID << 6);
                m_tMessages[GetKilobotId(c_kilobot_entity)].data[1+i*3] = m_tMessages[GetKilobotId(c_kilobot_entity)].data[1+i*3] | (tMessage.m_sType << 2);
                m_tMessages[GetKilobotId(c_kilobot_entity)].data[1+i*3] = m_tMessages[GetKilobotId(c_kilobot_entity)].data[1+i*3] | (tMessage.m_sData >> 8);
                m_tMessages[GetKilobotId(c_kilobot_entity)].data[2+i*3] = tMessage.m_sData;
            }

            //        LOG << " data[0]=" << m_tMessages[GetKilobotId(c_kilobot_entity)].data[0] << std::endl;
            //        LOG << " data[1]=" << m_tMessages[GetKilobotId(c_kilobot_entity)].data[1] << std::endl;
            //        LOG << " data[2]=" << m_tMessages[GetKilobotId(c_kilobot_entity)].data[2] << std::endl;

            /* Sending the message */
            GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[GetKilobotId(c_kilobot_entity)]);
        }
        else{
            GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,NULL);
        }
    }
}

/****************************************/
/****************************************/

void CArk::UpdateVirtualEnvironments(){

    /* Updates the virtual environments  based on the kilobots' states */
    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        /* Let a kilobot modify the virtual environment  */
        UpdatesVirtualEnvironmentsBasedOnKilobotState(*m_tKilobotsEntities[it]);
    }

    /* Apply change to the options' qualities */
    for(unsigned int i=0;i<m_tOptions.size();i++)
    {
        if(m_tOptions[i].QualityChangeTime!= 0 && m_fTimeInSeconds==m_tOptions[i].QualityChangeTime)
        {
            LOG << "Option " << i+1 << "had quality "<< m_tOptions[i].quality;

            m_tOptions[i].quality=m_tOptions[i].qualityAfterChange;

            LOG << "now its quality is" << m_tOptions[i].quality <<std::endl;
        }
    }

}

/****************************************/
/****************************************/

void CArk::UpdatesVirtualEnvironmentsBasedOnKilobotState(CKilobotEntity &c_kilobot_entity){
    /* Here the virtual environment are updated based on the kilobot "kilobot_entity" state */
}

/** ********************************************/
/**          Visualisation functions           */
/** ********************************************/

CColor CArk::GetFloorColor(const CVector2 &vec_position_on_plane) {

    CColor cColor=CColor::WHITE;
    std::vector <UInt16> vecInOptions;

    for(UInt16 it = 0;it < m_tOptions.size(); ++it) {
        if((pow(vec_position_on_plane.GetX()-m_tOptions[it].position.GetX(),2)+pow(vec_position_on_plane.GetY()-m_tOptions[it].position.GetY(),2))<pow(m_tOptions[it].radius,2)) vecInOptions.push_back(it);
    }

    if(vecInOptions.size() > 0){
        if(vecInOptions.size() == 1) {
            if(m_fTimeInSeconds>=m_tOptions[vecInOptions[0]].AppearanceTime && (m_fTimeInSeconds<m_tOptions[vecInOptions[0]].DisappearanceTime || m_tOptions[vecInOptions[0]].DisappearanceTime<=0))
            {
                //                LOG << "Time="<< (int) m_fTimeInSeconds << " Option="<< vecInOptions[0] << " AppearanceTime=" << m_tOptions[vecInOptions[0]].AppearanceTime << std::endl;
                cColor=m_tOptions[vecInOptions[0]].color;
            }
            else{
                cColor=CColor::WHITE;
            }
        }
        else{
            cColor=CColor::GRAY90;
        }
    }

    return cColor;
}

/****************************************/
/****************************************/

void CArk::PlotEnvironment(){
    if(m_bDynamicVirtualEnvironments){
        /* Update the Floor visualization of the virtual environment every m_unEnvironmentPlotUpdateFrequency ticks*/
        if (std::find(interestingTimes.begin(), interestingTimes.end(), m_fTimeInSeconds) != interestingTimes.end())
        {
            GetSpace().GetFloorEntity().SetChanged();
        }
    }
}

/** **************************************/
/**       Circular Arena Function        */
/** **************************************/
void CArk::CreateCircularArena(TConfigurationNode& t_tree){
    unsigned int m_unNumArenaWalls=100,m_fArenaRadius=1;
    TConfigurationNode& tCircularArenaNode=GetNode(t_tree,"circular_arena");
    GetNodeAttributeOrDefault(tCircularArenaNode,"number_walls",m_unNumArenaWalls,m_unNumArenaWalls);
    GetNodeAttributeOrDefault(tCircularArenaNode,"radius",m_fArenaRadius,m_fArenaRadius);

    CRadians wall_angle = CRadians::TWO_PI/m_unNumArenaWalls;
    CVector3 wall_size(0.01, 2.0*m_fArenaRadius*Tan(CRadians::PI/m_unNumArenaWalls), 0.1);
    std::ostringstream entity_id;
    for( UInt32 i = 0; i < m_unNumArenaWalls; i++ ) {
        entity_id.str("");
        entity_id << "wall_" << i;

        CRadians wall_rotation = wall_angle*i;
        CVector3 wall_position( m_fArenaRadius*Cos(wall_rotation), m_fArenaRadius*Sin(wall_rotation), 0 );
        CQuaternion wall_orientation;
        wall_orientation.FromEulerAngles( wall_rotation,  CRadians::ZERO, CRadians::ZERO );

        CBoxEntity* box = new CBoxEntity(entity_id.str(), wall_position, wall_orientation, false, wall_size, (Real)1.0 );
        GetSpace().AddEntity( *box );
        GetSpace().AddEntityToPhysicsEngine( (*box).GetEmbodiedEntity() );
    }
}



/** *****************************************/
/**    Here Goes the user created functions */
/** *****************************************/

void CArk::AddOption(TConfigurationNode& t_node)
{
    m_sOption sOption;

    GetNodeAttribute(t_node, "id", sOption.id);

    GetNodeAttribute(t_node, "quality", sOption.quality);

//    GetNodeAttribute(t_node, "position", sOption.position);

    GetNodeAttribute(t_node, "radius", sOption.radius);

    GetNodeAttribute(t_node, "color", sOption.color);

    GetNodeAttribute(t_node, "AppearanceTime", sOption.AppearanceTime);

    GetNodeAttribute(t_node, "DisappearanceTime", sOption.DisappearanceTime);

    GetNodeAttribute(t_node, "QualityChangeTime", sOption.QualityChangeTime);

    GetNodeAttribute(t_node, "qualityAfterChange", sOption.qualityAfterChange);


    if(sOption.AppearanceTime!=0)
        interestingTimes.push_back(sOption.AppearanceTime);

    if(sOption.DisappearanceTime!=0)
        interestingTimes.push_back(sOption.DisappearanceTime);

//    sOption.GPS_position=PositionToGPS(sOption.position);

    //    LOG << "OPTION " <<  sOption.id <<" IS LOCATED AT X=" << sOption.GPS_position.GetX() << " Y=" << sOption.GPS_position.GetY() << std::endl;

    m_tOptions.push_back(sOption);
}

/****************************************/
/****************************************/

CVector2 CArk::PositionToGPS( CVector2 t_position ) {
    return CVector2(Ceil(t_position.GetX()/m_fCellLength)-1,Ceil(t_position.GetY()/m_fCellLength)-1);
}



REGISTER_LOOP_FUNCTIONS(CArk, "ark_loop_functions")
