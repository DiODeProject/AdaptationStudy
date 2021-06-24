#!/bin/bash

#Author: 	Dr Mohamed Salaheddine Talamali - University College London (UCL)

#Directories paths
PROJECT_HOME="${HOME}/DeMaMAS_Adaptation" # Absolute path to the multi-agent simulator folder 
DATA_HOME="$(echo $PROJECT_HOME | sed 's/home/data/g')"
DATA_HOME=${HOME}/data/Fig7_compare # Absolute path to the the folder where data will be saved
mkdir -p ${DATA_HOME}

EXPERIMENT_TYPE=appearance # Environmental change type
EXEC_FILE="${PROJECT_HOME}/src/deMaMAS.py" # full path to the multi-agent simulator executable file 
RESULT_FOLDER_PATH="${DATA_HOME}/resultsJobArray" # full path to the results folder
RESULT_FOLDER_PATH_SUMMARY="${DATA_HOME}/resultsJobArray/"
CONF_FILE_SUMMARY="${RESULT_FOLDER_PATH_SUMMARY}summary.config"
SUMMARY_TEMPLATE="${PROJECT_HOME}/configuration/configuration.summary.template.config" 
TEMPLATE_SETTINGS="${PROJECT_HOME}/configuration/configuration.template.config" # full path to the configuration file template
mkdir -p ${RESULT_FOLDER_PATH}
mkdir -p ${RESULT_FOLDER_PATH_SUMMARY}

#USUALLY FIXED

#General parameters
COLORS=[gray,red,green,blue,yellow,violet,darkorange,navy,peru,turquoise,olive,cadetblue,indigo]
COMPOSITION=[1]
COMP_LABELS=[CI]
ENVIRONMENT_SIZE=1 # Environment size (1 x 1)
NUM_SIMULATION=1
NUM_START_P=1
NUM_STEP=60000 # Number of timesteps
NUM_TOTEM=[1]
OVERLAPPING_TOTEMS="false"
QUORUM=0 # Quorum threshold when experiment is abrupted (not used in this case)
STARTING_POINT="false"
TEST_NAME=DYNAMIC_ENV

#Behaviour parameters
DISCOVERY_METHOD=[probabilistic]
K_UNANIMITY_PARAMETER=2
MAX_QUALITY=1 
UPDATE_RULE="[random]" # opinion update model (here is the voter rule)
ZEALOT_OPINION=0
ZEALOT_QUALITY=0.5

#Message parameters
MSG_TYPE=simple
SEND_METHOD=[wv]
SEND_CONSTANT_INTERVAL=1
FILTER_MSG_PARAM=[0]

#Movement parameters
MOVE_DIMENSION=0.005
STRAIGHT_LENGTH=20

#Agent and totem characteristics parameters
START_WITH_OPINION="false" # Agents will start without opinion
START_WITH_GIVEN_OPINION=0 # Agents will start without opinion
TOTEM_RADIUS=0.2 # Agent's field of view
FIRST_ENTRY_ONLY_DISC="true"

#Decay parameters
DECAY_METHOD=[constant]
DECAY_STRENGTH=[0]

#Interaction function parameters
INTERACTION_FUNCTION=[constant]
PRE_STEP_SOCIAL_STRENGTH=[1]
POST_STEP_SOCIAL_STRENGTH=[1]
POST_STEP_SELF_STRENGTH=[1]
INTERACTIVE_PROBABILITY=1
INITIAL_TIME_VALUE=400

#Files parameters
COLOURS_FILE=[0,R,B,G,Y,V,D,N,P,T,O,C,I]

#Graphic and plot parameters
ANIMATION_INTERVAL=50
GRAPH_SIMULATION="false"
PLOT="false"

#USUALLY CHANGE
NUM_OP=3 # Number of options/sites
NUM_AGENT=100 # Number of agents
AGENT_RADIUS=0.3 # Communication range

UPDATE_MODEL=[direct]
PROBABILISTIC_DISCOVERY_PROPORTION=[compare] # compare exploration rule
QUALITY_VALUES_AFTER_CHANGE="[7.2,9,3.6]"
TOTEMS_APPEARANCE_TIMES=[10000,0] # Appearance time of the options/sites
TOTEMS_DISAPPEARANCE_TIMES=[-1] # Disappearance time of the options/sites
QUALITY_CHANGE_TIME=0
STANDARD_DEVIATION_VALUE=0.1 # Qualities estimation noise
PRE_STEP_SELF_STRENGTH=[0.05] # Value of the compare exploration rule parameter "epsilon"

# Qualities of the options/sites values set
QUALITY_VALUES_LIST=(
[0.9,0.6,0.1] [0.9,0.65,0.1] [0.9,0.7,0.1] [0.9,0.75,0.1] [0.9,0.8,0.1]
[0.8,0.5,0.1] [0.8,0.55,0.1] [0.8,0.6,0.1] [0.8,0.65,0.1] [0.8,0.7,0.1]
[0.7,0.4,0.1] [0.7,0.45,0.1] [0.7,0.5,0.1] [0.7,0.55,0.1] [0.7,0.6,0.1]
[0.6,0.3,0.1] [0.6,0.35,0.1] [0.6,0.4,0.1] [0.6,0.45,0.1] [0.6,0.5,0.1] 
[0.5,0.2,0.1] [0.5,0.25,0.1] [0.5,0.3,0.1] [0.5,0.35,0.1] [0.5,0.4,0.1]
)

NUMBERS_OF_HOURS=3                                                                                                        
NUMBERS_OF_MINS=15

sed -e "s|start|${1}|"                          \
    -e "s|end|${2}|"                            \
    -e "s|HOURS|${NUMBERS_OF_HOURS}|"   \
    -e "s|MINS|${NUMBERS_OF_MINS}|"   \
    -e "s|JOBNAME|Fig7_compare_${NUM_AGENT}_${AGENT_RADIUS}|"   \
    run_jobarray_template.sh > run_jobarray.sh


CONF_DIR="${DATA_HOME}/conf_cluster_JobArray/${NUM_AGENT}_${AGENT_RADIUS}/${EXPERIMENT_TYPE}"
mkdir -p ${CONF_DIR}


EXP_RESULT_FOLDER_PATH=${RESULT_FOLDER_PATH}/${NUM_AGENT}_${AGENT_RADIUS}/${EXPERIMENT_TYPE}/
FINAL_RESULT_FILENAME="null"
mkdir -p ${EXP_RESULT_FOLDER_PATH}

mkdir -p "${DATA_HOME}/configuration/${NUM_AGENT}_${AGENT_RADIUS}/${EXPERIMENT_TYPE}/"

SPECIAL_TEMPLATE_SETTINGS1="${DATA_HOME}/configuration/${NUM_AGENT}_${AGENT_RADIUS}/${EXPERIMENT_TYPE}/configuration.specialtemplate2.config"

# Create specialised template to make large runs submission quicker
sed -e "s|ENVIRONMENT_SIZE|${ENVIRONMENT_SIZE}|" \
	-e "s|MOVE_DIMENSION|${MOVE_DIMENSION}|" \
	-e "s|NUM_AGENT|${NUM_AGENT}|" \
	-e "s|AGENT_RADIUS|${AGENT_RADIUS}|" \
	-e "s|TOTEM_RADIUS|${TOTEM_RADIUS}|" \
	-e "s|FIRST_ENTRY_ONLY_DISC|${FIRST_ENTRY_ONLY_DISC}|" \
	-e "s|NUM_STEP|${NUM_STEP}|" \
	-e "s|STRAIGHT_LENGTH|${STRAIGHT_LENGTH}|" \
	-e "s|START_WITH_OPINION|${START_WITH_OPINION}|" \
	-e "s|START_WITH_GIVEN_OPINION|${START_WITH_GIVEN_OPINION}|" \
	-e "s|MAX_QUALITY|${MAX_QUALITY}|" \
	-e "s|COLORS|${COLORS}|" \
	-e "s|COMPOSITION|${COMPOSITION}|" \
	-e "s|COMP_LABELS|${COMP_LABELS}|" \
	-e "s|COLOURS_FILE|${COLOURS_FILE}|" \
	-e "s|NUM_TOTEM|${NUM_TOTEM}|" \
	-e "s|OVERLAPPING_TOTEMS|${OVERLAPPING_TOTEMS}|" \
	-e "s|STARTING_POINT|${STARTING_POINT}|" \
	-e "s|NUM_START_P|${NUM_START_P}|" \
	-e "s|UPDATE_MODEL|${UPDATE_MODEL}|" \
	-e "s|SEND_METHOD|${SEND_METHOD}|" \
	-e "s|SEND_CONSTANT_INTERVAL|${SEND_CONSTANT_INTERVAL}|" \
	-e "s|FILTER_MSG_PARAM|${FILTER_MSG_PARAM}|" \
	-e "s|MSG_TYPE|${MSG_TYPE}|" \
	-e "s|UPDATE_RULE|${UPDATE_RULE}|" \
	-e "s|ZEALOT_OPINION|${ZEALOT_OPINION}|" \
	-e "s|ZEALOT_QUALITY|${ZEALOT_QUALITY}|" \
	-e "s|K_UNANIMITY_PARAMETER|${K_UNANIMITY_PARAMETER}|" \
	-e "s|DISCOVERY_METHOD|${DISCOVERY_METHOD}|" \
	-e "s|ANIMATION_INTERVAL|${ANIMATION_INTERVAL}|" \
	-e "s|QUORUM|${QUORUM}|" \
	-e "s|RESULT_FOLDER_PATH|${EXP_RESULT_FOLDER_PATH}|" \
	-e "s|NUM_SIMULATION|${NUM_SIMULATION}|" \
	-e "s|GRAPH_SIMULATION|${GRAPH_SIMULATION}|" \
	-e "s|PLOT|${PLOT}|" \
	-e "s|DECAY_METHOD|${DECAY_METHOD}|" \
	-e "s|DECAY_STRENGTH|${DECAY_STRENGTH}|" \
	-e "s|STANDARD_DEVIATION_VALUE|${STANDARD_DEVIATION_VALUE}|" \
	-e "s|INITIAL_TIME_VALUE|${INITIAL_TIME_VALUE}|" \
	-e "s|INTERACTIVE_PROBABILITY|${INTERACTIVE_PROBABILITY}|" \
	-e "s|TEST_NAME|${TEST_NAME}|" \
	-e "s|TOTEMS_APPEARANCE_TIMES|${TOTEMS_APPEARANCE_TIMES}|" \
	-e "s|PROBABILISTIC_DISCOVERY_PROPORTION|${PROBABILISTIC_DISCOVERY_PROPORTION}|" \
	-e "s|INTERACTION_FUNCTION|${INTERACTION_FUNCTION}|"\
	-e "s|PRE_STEP_SOCIAL_STRENGTH|${PRE_STEP_SOCIAL_STRENGTH}|"\
	-e "s|POST_STEP_SOCIAL_STRENGTH|${POST_STEP_SOCIAL_STRENGTH}|"\
	-e "s|PRE_STEP_SELF_STRENGTH|${PRE_STEP_SELF_STRENGTH}|"\
	-e "s|POST_STEP_SELF_STRENGTH|${POST_STEP_SELF_STRENGTH}|"\
	-e "s|NUM_OP|${NUM_OP}|"\
	-e "s|QUALITYVALUES_AFTER_CHANGE|${QUALITY_VALUES_AFTER_CHANGE}|"\
	-e "s|TOTEMS_DISAPPEARANCE_TIMES|${TOTEMS_DISAPPEARANCE_TIMES}|"\
	-e "s|QUALITY_CHANGE_TIME|${QUALITY_CHANGE_TIME}|"\
	-e "s|FINAL_RESULT_FILENAME|${FINAL_RESULT_FILENAME}|"\
		${TEMPLATE_SETTINGS} > ${SPECIAL_TEMPLATE_SETTINGS1}

for QUALITY_VALUES in ${QUALITY_VALUES_LIST[*]}
do
	JOB_PARAM="t-${NUM_TOTEM}_agt-${NUM_AGENT}_stp-${NUM_STEP}_op-${NUM_OP}_maxQ-${MAX_QUALITY}_mdl-${UPDATE_MODEL}_dprop${PROBABILISTICDISCOVERYPROPORTION}_rle-${UPDATE_RULE}_dscv-${DISCOVERY_METHOD}_qrm-${QUORUM}_stdDv-${STANDARD_DEVIATION_VALUE}_intrct-${INTERACTION_FUNCTION}_iniT-${INITIAL_TIME_VALUE}_cmps-${COMPOSITION}_tstN-${TEST_NAME}_qlt${QUALITY_VALUES}_threshold${PRE_STEP_SELF_STRENGTH}"
	CONF_FILE_PRE="${CONF_DIR}/demamas_${JOB_PARAM}_"
	SPECIAL_TEMPLATE_SETTINGS2="${DATA_HOME}/configuration/${NUM_AGENT}_${AGENT_RADIUS}/${EXPERIMENT_TYPE}/${QUALITY_VALUES}.specialtemplate1.config"

	cp ${SPECIAL_TEMPLATE_SETTINGS1} ${SPECIAL_TEMPLATE_SETTINGS2}
	perl -i -p -e "s/QUALITY_VALUES/${QUALITY_VALUES}/" ${SPECIAL_TEMPLATE_SETTINGS2}


	for SIM_NO in `seq ${1} ${2}`;
	do
		SEED=$((7777+${SIM_NO}))

		echo -n "${SIM_NO} "

		# STATISTICS_FILENAME=statistics_${QUALITY_VALUES}_$((${SIM_NO}-1)).txt
		STATISTICS_FILENAME=statistics_${UPDATE_MODEL}_${PRE_STEP_SELF_STRENGTH}_${QUALITY_VALUES}__${STANDARD_DEVIATION_VALUE}_$((${SIM_NO}-1)).txt
		# echo ${STATISTICS_FILENAME}
		CONF_FILE="${CONF_FILE_PRE}${SIM_NO}.config"
		# cp ${SPECIAL_TEMPLATE_SETTINGS2} ${CONF_FILE}
		sed -e "s|SEED|${SEED}|" \
			-e "s|STATISTICS_FILENAME|${STATISTICS_FILENAME}|" \
			${SPECIAL_TEMPLATE_SETTINGS2} > ${CONF_FILE}

		# perl -i -p -e "s/SEED/${SEED}/ ; s/STATISTICS_FILENAME/${STATISTICS_FILENAME}/" ${CONF_FILE}
		COUNT=$((COUNT + 1))
	done

	COMMAND="qsub run_jobarray.sh ${EXEC_FILE} ${CONF_FILE_PRE}"
	${COMMAND}
	# echo "submitted n=${NUM_OP} std=${QUALITY_DISTRIBUTION_STD}" 
done

echo "Submitted " $COUNT " jobs"


