'''
    File name: agent.py
    Author: Mohamed S. Talamali and Andreagiovanni Reina
            University of Sheffield, UK
    Date created: October 2018 (By Francesco Cancianni)
    Date last modified: October 2020 (By Mohamed S. Talamali)
'''

import shutil
import numpy as np
import math
from copy import deepcopy

import matplotlib.pyplot as plt
import matplotlib.animation as animation


#Import of other classes
from agent import Agent, ActivityState
from totem import Totem
from parser import Parser
from errorController import ErrorController

import os
import sys

#Import configuration
import json
import configparser

print("\n---SIMULATION---\n")
#Global variable must be configured
environmentSize = None
seed = None
numberOfAgents = None
agentRadius = None
totemRadius = None
steps = None
straightLength = None
startWithOpinion = None
numberOfOpinion = None
numberOfTotems = None
totemsAppearanceTimes=None
totemsDisappearanceTimes=None
overlappingTotems = None
totalNumberOfTotems = None
maxQuality = None
colors = None
colorsFile = None
qualityValues = None
qualityValuesAfterChange = None
interval = None
quorumValue = None
resultFolderPath = None
statisticsFileName = None
finalResultFn = None
graphSimulation = None
numberOfSimulation = None
startingPoint = None
numberOfStartingPoints = None
plot = None
standardDeviation = None
composition = None
updateRule = None
updateModel = None
kParameter = None
sendInfoMethod = None
interactionFunction = None
preStepSelfStrength = None
postStepSelfStrength = None
preStepSocialStrength = None
postStepSocialStrength = None
decayMethod = None
decayStrength = None
discoveryMethod = None
moveDimension = None
initialTimeValue = None
msgType = None
sendConstant = None
zealotOpinion = None
zealotQuality = None
compositionLabels = None
firstEntryOnlyDiscovery = None
interactiveProb = None
filterMsgParam = None

conifgurationInput = None
stopFlag = False
firstTimeFlag = True
stopAnimation = False
listOfAgents = []
listOfTotems = []
plots = []
labels = []
handlers = []
totemPlots = []

def init():
    global listOfAgents
    global listOfTotems
    resetVariables()
    createTotems()
    createAgents()

def configuration(configurationFilePath):
    '''
    with open(configurationFilePath, 'r') as config:
        config_dict = json.load(config)
    '''
    config = configparser.ConfigParser()
    config.readfp(open(configurationFilePath))
    global environmentSize
    global seed
    global numberOfAgents
    global agentRadius
    global totemRadius
    global firstEntryOnlyDiscovery
    global steps
    global straightLength
    global startWithOpinion
    global numberOfOpinion
    global numberOfTotems
    global totemsAppearanceTimes
    global totemsDisappearanceTimes
    global totalNumberOfTotems
    global overlappingTotems
    global maxQuality
    global colors
    global colorsFile
    global qualityValues
    global qualityValuesAfterChange
    global interval
    global quorumValue
    global resultFolderPath
    global statisticsFileName
    global filterMsgParam
    global finalResultFn
    global graphSimulation
    global numberOfSimulation
    global startingPoint
    global numberOfStartingPoints
    global plot
    global standardDeviation
    global sendInfoMethod
    global interactionFunction
    global preStepSelfStrength
    global preStepSocialStrength
    global postStepSelfStrength
    global postStepSocialStrength
    global decayMethod
    global decayStrength
    global discoveryMethod
    global probabilisticDiscoveryProportion
    global composition
    global compositionLabels
    global updateRule
    global kParameter
    global powRadius
    global powTotemRadius
    global updateModel
    global initialTimeValue
    global moveDimension
    global msgType
    global sendConstant
    global interactiveProb
    global zealotOpinion
    global zealotQuality
    global qualityChangeTime

    #experiment parameters
    colors = config.get("experiment", "colors")
    colors = fromStringToArray(colors)
    composition = json.loads(config["experiment"]["composition"])
    compositionLabels = config.get("experiment", "compositionLabels")
    compositionLabels = fromStringToArray(compositionLabels)
    environmentSize = config.getfloat("experiment", "environmentSize")
    numberOfAgents = config.getint("experiment", "numberOfAgents")
    numberOfOpinion = config.getint("experiment", "numberOfOpinion")
    numberOfSimulation = config.getint("experiment", "numberOfSimulation")
    numberOfStartingPoints = config.getint("experiment", "numberOfStartingPoint")
    numberOfTotems = json.loads(config["experiment"]["numberOfTotems"])
    totemsAppearanceTimes= json.loads(config["experiment"]["totemsAppearanceTimes"])
    totemsDisappearanceTimes = json.loads(config["experiment"]["totemsDisappearanceTimes"])
    overlappingTotems = config.getboolean("experiment", "overlappingTotems")
    quorumValue = config.getfloat("experiment", "quorum")
    seed = config.getint("experiment", "seed")
    standardDeviation = config.getfloat("experiment", "noiseStandardDeviationValue")
    startingPoint = config.getboolean("experiment", "startingPoint")
    steps = config.getint("experiment", "numberOfSteps")
    qualityChangeTime = config.getint("experiment", "qualityChangeTime")

    #model_update parameters
    discoveryMethod = config.get("model_update", "discoveryMethod")
    discoveryMethod = fromStringToArray(discoveryMethod)
    probabilisticDiscoveryProportion = config.get("model_update", "probabilisticDiscoveryProportion")
    probabilisticDiscoveryProportion = fromStringToArray(probabilisticDiscoveryProportion)
    kParameter = config.getint("model_update", "kUnanimityParameter")
    maxQuality = config.getfloat("model_update", "maxQuality")
    qualityValues = json.loads(config["model_update"]["qualityValues"])
    qualityValuesAfterChange = json.loads(config["model_update"]["qualityValuesAfterChange"])
    updateModel = config.get("model_update", "updateModel")
    updateModel = fromStringToArray(updateModel)
    updateRule = config.get("model_update", "updateRule")
    updateRule = fromStringToArray(updateRule)
    zealotOpinion = config.getint("model_update", "zealotOpinion")
    zealotQuality = config.getfloat("model_update", "zealotQuality")

    #msg parameters
    msgType = config.get("msg", "msgType")
    sendConstant = config.getint("msg", "sendConstant")
    sendInfoMethod = config.get("msg", "sendInfoMethod")
    sendInfoMethod = fromStringToArray(sendInfoMethod)


    #movement parameters
    straightLength = config.getint("movement", "straightLength")
    moveDimension = config.getfloat("movement", "moveDimension")

    #agent_totem_characteristics parameters
    agentRadius = config.getfloat("agent_totem_characteristics", "agentRadius")
    startWithOpinion = config.getboolean("agent_totem_characteristics", "startWithOpinion")
    totemRadius = config.getfloat("agent_totem_characteristics", "totemRadius")
    firstEntryOnlyDiscovery = config.getboolean("agent_totem_characteristics", "firstEntryOnlyDiscovery")

    #decay parameters
    decayMethod = config.get("decay", "decayMethod")
    decayMethod = fromStringToArray(decayMethod)
    decayStrength = json.loads(config["decay"]["decayStrength"])

    #interaction parameters
    interactionFunction = config.get("interaction", "interactionFunction")
    interactionFunction = fromStringToArray(interactionFunction)
    initialTimeValue = config.getint("interaction", "initialTimeValue")
    interactiveProb = config.getfloat("interaction", "interactiveProb")
    filterMsgParam = json.loads(config["interaction"]["filterMsgParam"])

    postStepSelfStrength = json.loads(config["interaction"]["postStepSelfStrength"])
    postStepSocialStrength = json.loads(config["interaction"]["postStepSocialStrength"])
    preStepSelfStrength = json.loads(config["interaction"]["preStepSelfStrength"])
    preStepSocialStrength = json.loads(config["interaction"]["preStepSocialStrength"])

    #log_files parameters
    colorsFile = config.get("log_files", "colorsFile")
    colorsFile = fromStringToArray(colorsFile)
    resultFolderPath = config.get("log_files", "resultFolderPath")
    statisticsFileName = config.get("log_files", "statisticsFileName")
    finalResultFn = config.get("log_files", "finalResultFileName")

    #graphic_plot parameters
    interval = config.getint("graphic_plot", "animationInterval")
    graphSimulation = config.getboolean("graphic_plot", "graphSimulation")
    plot = config.getboolean("graphic_plot", "plot")

    #derivative parameters
    powRadius = agentRadius * agentRadius
    powTotemRadius = totemRadius * totemRadius


def configurationErrorCheck():
    errorController = ErrorController(configurationInput)
    errorController.errorHandler()

def resetVariables():
    global listOfAgents
    global listOfTotems
    listOfTotems = []
    listOfAgents = []

def fromStringToArray(string):
    #remove the [ ]
    string = string.replace("[", "")
    string = string.replace("]", "")
    #remove spaces " "
    string = string.replace(" ", "")
    #create the array from the string
    array = string.split(",")

    return array

def overlappingWithPreviousTotems(x, y, totemList, distSq):
    for totem in totemList:
        if distanceOnTorus(x, y, totem.x, totem.y) < distSq:
            return True
    return False

def overlappingWithPreviousTotemsOfOpinion(x, y, totemList, distSq, opinion):
    return overlappingWithPreviousTotems(x ,y, [t for t in totemList if t.opinion == opinion], distSq)

def createTotems():
    global totalNumberOfTotems
    global numberOfTotems
    if not startWithOpinion:
        opinionList = []
        for n in range(1, numberOfOpinion + 1):
            opinionList.append(n)
        t_id = 1

        lenQualityValues = len(qualityValues)
        lenTotemsAppearanceTimes = len(totemsAppearanceTimes)
        lenTotemsDisappearanceTimes = len(totemsDisappearanceTimes)
        totemDiameterSq = (totemRadius*2)**2
        #Set the correct opinion and quality for each totem as specified in the configuration explanation on gitHub
        if len(numberOfTotems) == 1:
            numberOfTotems = [numberOfTotems[0]]*numberOfOpinion
        for opinion, nTotem in enumerate(numberOfTotems):
            #Create a number "elem" of totem with the same opinion and quality.
            for _ in range(nTotem):
                x = np.random.uniform(0, environmentSize)
                y = np.random.uniform(0, environmentSize)
                attempt=1
                max_attempts = 1000000
                while (not overlappingTotems) and overlappingWithPreviousTotems(x, y, listOfTotems, totemDiameterSq):
                    if attempt > max_attempts:
                        print("ERROR! After " + str(max_attempts) + ", unable to place a totem without overlap...Either reduce the number of totems, their radius, or allow overlapping totems.")
                        sys.exit()
                    attempt += 1
                    x = np.random.uniform(0, environmentSize)
                    y = np.random.uniform(0, environmentSize)
                led = colors[opinion + 1]
                if lenQualityValues >= numberOfOpinion + 1:
                    quality = qualityValues[opinion]
                    qualityAfterChange = qualityValuesAfterChange[opinion]
                else:
                    if opinion <= lenQualityValues - 1:
                        quality = qualityValues[opinion]
                        qualityAfterChange = qualityValuesAfterChange[opinion]
                    else:
                        quality = qualityValues[lenQualityValues - 1]
                        qualityAfterChange = qualityValuesAfterChange[lenQualityValues - 1]

                if lenTotemsAppearanceTimes >= numberOfOpinion + 1:
                    appearanceTime = totemsAppearanceTimes[opinion]
                else:
                    if opinion <= lenTotemsAppearanceTimes - 1:
                        appearanceTime = totemsAppearanceTimes[opinion]
                    else:
                        appearanceTime = totemsAppearanceTimes[lenTotemsAppearanceTimes - 1]

                if lenTotemsDisappearanceTimes >= numberOfOpinion + 1:
                    disappearanceTime = totemsDisappearanceTimes[opinion]
                else:
                    if opinion <= lenTotemsDisappearanceTimes - 1:
                        disappearanceTime = totemsDisappearanceTimes[opinion]
                    else:
                        disappearanceTime = totemsDisappearanceTimes[lenTotemsDisappearanceTimes - 1]


                t_id = t_id + 1
                #opinion = 0 is uncommitted and a opinion for a totem must be > 0 (+1)
                newTotem = Totem(x, y, t_id, opinion + 1, led, quality,appearanceTime,disappearanceTime,qualityAfterChange)
                listOfTotems.append(newTotem)
                print("ID: " + str(newTotem.id) + " OP: " + str(newTotem.opinion) + " Q: " + str(newTotem.quality) + " COLOR: " + str(newTotem.led)+" Position: ("+str(newTotem.x)+","+str(newTotem.y)+")")
        totalNumberOfTotems = len(listOfTotems)

def createAgents():
    global updateRule
    if startingPoint:
        startsX, startsY = createStartingPoints()
    #Compute the composition of the swarm from the percentage specified in the global parameter composition
    compositionCardinality = swarmComposition()
    counter = 0
    #create the fraction of the subswarm of zealot
    zealotSubSwarm = zealotSubSwarmComposition(compositionCardinality)
    #create a list of possible opinion for zealot already shuffled
    listZealotOpinions = opinionsForZealot()
    print("LIST OF OP: " + str(listZealotOpinions))
    for pos, swarmFraction in enumerate(compositionCardinality):
        if pos != 0:
            counter = counter + compositionCardinality[pos-1]
        for n in range(swarmFraction):
            if startingPoint:
                pos = np.random.randint(numberOfStartingPoints)
                x = startsX[pos]
                y = startsY[pos]
            else:
                x = np.random.uniform(0, environmentSize)
                y = np.random.uniform(0, environmentSize)
            orientation = np.random.uniform(0, 360)
            a_id = counter + n
            #Opinion a priori
            if(startWithOpinion):
                opinion = np.random.randint(1, numberOfOpinion + 1)
                if standardDeviation != 0:
                    quality = qualityValues[opinion - 1] + np.random.normal(0, standardDeviation)
                    if quality > maxQuality:
                        quality = maxQuality
                    elif quality < 0:
                        quality = 0
                else:
                    quality = qualityValues[opinion - 1]
            #Opinion a Posteriori
            else:
                opinion = 0
                quality = 0
            led = colors[opinion]
            newAgent = Agent(a_id, colors, decayMethod[pos], decayStrength[pos], discoveryMethod[pos],probabilisticDiscoveryProportion[pos], environmentSize, filterMsgParam[pos], firstEntryOnlyDiscovery, initialTimeValue, interactionFunction[pos], interactiveProb, kParameter, led, maxQuality, moveDimension, msgType, numberOfOpinion, opinion, postStepSelfStrength[pos], postStepSocialStrength[pos], preStepSelfStrength[pos], preStepSocialStrength[pos], orientation, quality, sendConstant, sendInfoMethod[pos], standardDeviation, startWithOpinion, straightLength, updateModel[pos], updateRule[pos], x, y, zealotOpinion, zealotQuality)
            if(newAgent.updateRule == "zealot"):
                if zealotOpinion == 0:
                    #Assing option related to the subSwarm
                    for m in range(1, numberOfOpinion):
                        if n < (zealotSubSwarm * m):
                            newAgent.opinion = m + 1
                            break
                    #Assign the remaining outliers by random without repetitions
                    if newAgent.opinion == 0:
                        newAgent.opinion = listZealotOpinions[0]
                        listZealotOpinions.pop(0)
                    #Assign the position of the zealot opinion
                    if not startWithOpinion:
                        newAgent.totemPos = zealotTotemPosition(newAgent.opinion)
                else:
                    if not startWithOpinion:
                        newAgent.totemPos = zealotTotemPosition(zealotOpinion)
                print("OP: " + str(newAgent.opinion))
            newAgent.setColor
            listOfAgents.append(newAgent)
#Return the position of a totem for a zealot opinion
def zealotTotemPosition(opinion):
    selectedTotems = list(filter(lambda x: x.opinion == opinion, listOfTotems))
    np.random.shuffle(selectedTotems)
    return selectedTotems[0].msg.totemPos
#Count the cardinality for each zealot-subswarm
def zealotSubSwarmComposition(compositionCardinality):
    zealotSubSwarm = 0
    for pos, swarmFraction in enumerate(compositionCardinality):
        if updateRule[pos] == "zealot":
            zealotSubSwarm = math.floor(swarmFraction/(numberOfOpinion-1))
    return zealotSubSwarm
#create a list contained all possible opinions for zealots
def opinionsForZealot():
    listZealotOpinions = [n+1 for n in range(1, numberOfOpinion)]
    np.random.shuffle(listZealotOpinions)
    return listZealotOpinions
#OK
def createStartingPoints():
    startsX = []
    startsY = []
    for _p in range(numberOfStartingPoints):
        startsX.append(np.random.uniform(0, environmentSize))
        startsY.append(np.random.uniform(0, environmentSize))
    return startsX, startsY
#OK
def checkStatistics():
    counter = [0]*(numberOfOpinion+1)
    for agent in listOfAgents:
        counter[agent.opinion] += 1
    return counter
#OK
def quorum():
    stats = checkStatistics()
    #Not taking in account uncommitted for evaluate the quorum
    stats[0] = 0
    maxq = np.amax(stats)
    index = stats.index(maxq)
    if quorumValue == 0:
        return False
    if (stats[index] / numberOfAgents) >= quorumValue:
        return True
    else:
        return False

def swarmComposition():
    leftOvers = [0] * len(composition)
    compositionCardinality = [0] * len(composition)
    #initialise compositionCardinality by multiplying composition with numberOfAgents
    for pos,comp in enumerate(composition):
        pop = comp * numberOfAgents
        if not almostEqual(pop, math.floor(pop)):
            leftOvers[pos] = pop - math.floor(pop)
        compositionCardinality[pos] = math.floor(pop)
        # if approximations resulted in one agent less, it is added randomly (with probability proportional to the rounding quantities)
        sumAgents = sum(compositionCardinality)
        if sumAgents < numberOfAgents:
            rnd = np.random.rand() * sum(leftOvers)
            bottom = 0.0
            for pos, prob in enumerate(leftOvers):
                if rnd >= bottom and rnd < (bottom + prob):
                    compositionCardinality[pos] += 1
                    break
                bottom += prob
    print("SWARM COMPOSITION: " + str(compositionCardinality))
    print("SWARM METHOD: " + str(updateRule))
    return compositionCardinality

def almostEqual(x, y):
    epsilon = 0.0000001
    return abs(x - y) < epsilon

def writeStatistics(step, nSimulation):
    global stopFlag
    global firstTimeFlag
    if not stopFlag and statisticsFileName != "null":
        if firstTimeFlag:
            if not os.path.exists(resultFolderPath):
                os.makedirs(resultFolderPath, exist_ok=True)
            shutil.copy(configurationInput, resultFolderPath)

            f = open(resultFolderPath + "/" + statisticsFileName, "w")
            f.write("SEED: " + str(seed) + "\n")
            f.write("TS ")
            for n in range(numberOfOpinion + 1):
                f.write(colorsFile[n] + " ")
            f.write("Q \n")
            firstTimeFlag = False
        else:
            f = open(resultFolderPath + "/" + statisticsFileName, "a")
        f.write(str(step) + " ")
        counter = checkStatistics()
        for n in range(numberOfOpinion + 1):
            f.write(str(counter[n]) + " ")
        f.write(str(quorum()) + "\n")
        if quorum():
            stopFlag = True
            f.write("\n")
            f.close()
            if graphSimulation and finalResultFn == "null":
                exit()

    if finalResultFn != "null" and (quorum() or step == steps - 1):
        if not os.path.exists(resultFolderPath):
            os.makedirs(resultFolderPath, exist_ok=True)
        shutil.copy(configurationInput, resultFolderPath)
        f = open(resultFolderPath + finalResultFn, "a")
        f.write("SEED: " + str(seed) + "\n")
        f.write("TS ")
        for n in range(numberOfOpinion + 1):
            f.write(colorsFile[n] + " ")
        f.write("Q")
        f.write(" D")
        f.write(" S\n")
        firstTimeFlag = False
        f.write(str(step) + " ")
        counter = checkStatistics()
        for n in range(numberOfOpinion + 1):
            f.write(str(counter[n]) + " ")
        f.write(str(quorum())+ " ")
        counterForDiscoveryEvents = [0]*len(updateRule)
        counterForSocialEvents = [0]*len(updateRule)
        for agent in listOfAgents:
            for pos,model in enumerate(updateModel):
                if (agent.updateRule == updateRule[pos] and agent.updateModelParameter == updateModel[pos]):
                    counterForSocialEvents[pos] = counterForSocialEvents[pos] + agent.counterForSocialEvents
                    counterForDiscoveryEvents[pos] = counterForDiscoveryEvents[pos] + agent.counterForDiscoveryEvents
        f.write(str(counterForDiscoveryEvents) + " ")
        f.write(str(counterForSocialEvents) + "\n \n")
        print("DISCOVERY INTERACTIONS: " + str(counterForDiscoveryEvents))
        print("SOCIAL INTERACTIONS: " + str(counterForSocialEvents))
        f.close()

def performAction(step, nSimulation):
    listOfNeighbors = createNeighborsList()
    frozenList = deepcopy(listOfAgents)
    if step == 0:
        writeStatistics(step, nSimulation)

    # Update totems qualities in case of change
    for pos, totem in enumerate(listOfTotems):
        if(pos<=totalNumberOfTotems):
            if(qualityChangeTime!=0 and step==qualityChangeTime):
                listOfTotems[pos].quality=listOfTotems[pos].qualityAfterChange
                listOfTotems[pos].msg.quality=listOfTotems[pos].quality
        else:
            break

    for pos, agent in enumerate(listOfAgents):
        agent.oneStep(frozenList, listOfNeighbors[pos], listOfTotems, powTotemRadius, step)
    writeStatistics(step+1, nSimulation)

def createNeighborsList():
    #Initialize a list of empty lists per each agent in the swarm
    listOfNeighbors = [[] for i in range(numberOfAgents)]
    #Create a vector of vectors that represents neighbours for each agent
    for pos, elem in enumerate(listOfAgents):
        if pos + 1 < numberOfAgents:
            for i in range(pos + 1, numberOfAgents):
                if listOfAgents[i].activityState == ActivityState.LATENT: continue
                distance = distanceOnTorus(elem.x, elem.y, listOfAgents[i].x, listOfAgents[i].y)
                if distance <= powRadius:
                    listOfNeighbors[pos].append(i)
                    listOfNeighbors[i].append(pos)
    return listOfNeighbors

#Calculate the distance between two agents in the environment
def distanceOnTorus(x, y, x1, y1):
    result = ((min(abs(x - x1), environmentSize - abs(x - x1)))**2 +
     (min(abs(y - y1), environmentSize - abs(y - y1)))**2)
    return result

def initAnimation():
    global plots
    global handlers
    global labels
    tempLabels = []
    ax = plt.axes(xlim = (0, environmentSize), ylim = (0, environmentSize))
    ax.set_aspect('equal')
    plt.suptitle('DeMaMAS - Decision-Making in MultiAgent Systems')
    plt.title('Quorum: ' + str(quorumValue) + " Composition: " + str(composition))
    if not startWithOpinion:
        for totem in listOfTotems:
            circle = plt.Circle((totem.x, totem.y), totemRadius, color = colors[totem.opinion])
            if(totem.appearanceTime <=0):
                circle.set_alpha(0.6)
            else:
                circle.set_alpha(0)

            ax.add_artist(circle)
            plots.append(circle)

            if totem.x + totemRadius >= 1:
                xFallOut = -1 + totem.x
            elif totem.x - totemRadius <= 0:
                xFallOut = 1 + totem.x
            else:
                xFallOut = totem.x
            if totem.y + totemRadius >= 1:
                yFallOut = -1 + totem.y
            elif totem.y - totemRadius <= 0:
                yFallOut = 1 + totem.y
            else:
                yFallOut = totem.y
            if xFallOut != totem.x or yFallOut != totem.y:
                circle = plt.Circle((xFallOut, yFallOut), totemRadius, color = colors[totem.opinion])
                if(totem.appearanceTime <=0):
                    circle.set_alpha(0.6)
                else:
                    circle.set_alpha(0)
                ax.add_artist(circle)

    for pos,elem in enumerate(updateRule):
        labels.append(compositionLabels[pos])
        tempLabels.append(elem)
    for agent in listOfAgents:
        circle = plt.Circle((agent.x, agent.y), 0.01,
        color = colors[agent.opinion])
        if agent.updateRule == "majority":
            circle.set_linestyle('-.')
            circle.set_alpha(0.8)
            if agent.updateRule in tempLabels:
                tempLabels.remove("majority")
                handlers.append(circle)
        elif agent.updateRule == "kUnanimity":
            circle.set_linestyle('--')
            circle.set_alpha(0.8)
            if agent.updateRule in tempLabels:
                tempLabels.remove("kUnanimity")
                handlers.append(circle)
        elif agent.updateRule == "random":
            circle.set_linestyle("-")
            circle.set_alpha(0.8)
            if agent.updateRule in tempLabels:
                tempLabels.remove("random")
                handlers.append(circle)
        elif agent.updateRule == "minority":
            circle.set_linestyle("--")
            circle.set_edgecolor("black")
            if agent.updateRule in tempLabels:
                tempLabels.remove("minority")
                handlers.append(circle)
        elif agent.updateRule == "zealot":
            circle.set_linestyle("-")
            circle.set_edgecolor("black")
            if agent.updateRule in tempLabels:
                tempLabels.remove("zealot")
                handlers.append(circle)
        elif agent.updateRule == "mad":
            circle.set_linestyle("-.")
            circle.set_edgecolor("black")
            if agent.updateRule in tempLabels:
                tempLabels.remove("mad")
                handlers.append(circle)

        ax.add_artist(circle)
        plots.append(circle)
    if(len(updateRule) > 1):
        plt.legend(handlers, labels, title="Agent", bbox_to_anchor=(1, 1), loc='upper left', borderaxespad=0.3)
    return plots

def animate(i):
    global stopAnimation

    plt.xlabel("time= "+str(i),fontsize=18)

    if quorum() or stopAnimation:
        if plot:
            plotGraph()
        exit()
    performAction(i, 0)

    # Update totems visualisation
    for pos, totem in enumerate(listOfTotems):
        if(pos<=totalNumberOfTotems):

            if(totem.appearanceTime <=i and (totem.disappearanceTime <= 0 or totem.disappearanceTime > i) ):
                plots[pos].set_alpha(0.6)
            else:
                plots[pos].set_alpha(0)

        else:
            break

    # Update agents visualisation
    for pos, agent in enumerate(listOfAgents):
        plots[pos + totalNumberOfTotems].center = agent.x, agent.y
        plots[pos + totalNumberOfTotems].set_color(colors[agent.opinion])
        if agent.activityState == ActivityState.LATENT: #if agent.goingResampling:
            plots[pos + totalNumberOfTotems].set_alpha(0.5)
            plots[pos + totalNumberOfTotems].set_edgecolor(None)
            if agent.updateRule == "minority":
                plots[pos + totalNumberOfTotems].set_edgecolor("black")
        else:
            if agent.updateRule == "majority":
                plots[pos + totalNumberOfTotems].set_alpha(0.8)

            elif agent.updateRule == "kUnanimity":
                plots[pos + totalNumberOfTotems].set_alpha(0.8)
            elif agent.updateRule == "random" or agent.updateRule == "maxQuality":
                plots[pos + totalNumberOfTotems].set_alpha(1)
            elif agent.updateRule == "minority":
                plots[pos + totalNumberOfTotems].set_alpha(1)
                plots[pos + totalNumberOfTotems].set_edgecolor("black")
            elif agent.updateRule == "zealot":
                plots[pos + totalNumberOfTotems].set_linestyle("-")
                plots[pos + totalNumberOfTotems].set_edgecolor("black")
            elif agent.updateRule == "mad":
                plots[pos + totalNumberOfTotems].set_linestyle("-.")
                plots[pos + totalNumberOfTotems].set_edgecolor("black")

    if i == steps - 1 or quorum():
        writeStatistics(i, 0)
        stopAnimation = True

    return plots

def animationController():
    fig = plt.figure()
    _anim = animation.FuncAnimation(fig, animate, init_func = initAnimation, frames = steps, interval = interval, blit = False)
    plt.show()

def plotGraph():
    print("Choose a graph to plot:")
    typeOfGraph = input("digit 0 for BoxPlot and 1 for Population Evo \n")
    typeOfGraph = int(typeOfGraph)
    if typeOfGraph == 0:
        parser = Parser(resultFolderPath + "/" + finalResultFn,
         sys.argv[1], "BOX PLOT")
        data = parser.parseFile()
        parser.boxPlotTime(data)
    else:
        simNum = input("Select the number of the simulation, from 0 to " +
        str(numberOfSimulation - 1) + " :\n")
        simNum = int(simNum)
        parser = Parser(resultFolderPath + "/" + statisticsFileName, sys.argv[1], "POP EVO")
        data = parser.parseFile()
        parser.popEvoPlot(data)
        plotAnotherGraph = True
        while plotAnotherGraph:
            plotAgain = input("Plot another simulation? \n Y = 1, N = 0 \n")
            plotAgain = int(plotAgain)
            if plotAgain == 0:
                plotAnotherGraph = False
            else:
                simNum = input("Select the number of the simulation, from 0 to "
                 + str(numberOfSimulation - 1) + " :\n")
                simNum = int(simNum)
                parser = Parser(resultFolderPath + "/" + statisticsFileName, sys.argv[1], "POP EVO")
                data = parser.parseFile()
                parser.popEvoPlot(data)

def main():
    global configurationInput
    configurationInput = sys.argv[1]
    configurationErrorCheck()
    configuration(configurationInput)
    global firstTimeFlag
    global stopFlag
    global seed
    if seed == 0:
        seed = np.random.randint(4294967295)
        np.random.seed(seed)
    else:
        np.random.seed(seed)

    if(graphSimulation):
        init()
        animationController()
    else:
        for n in range(numberOfSimulation):
            init()
            step = 0
            while step < steps and not quorum():
                performAction(step, n)
                step = step + 1
            if(step == steps):
                print("Simulation number: " + str(n) + " quorum not reached")
            else:
                print("Simulation number: " + str(n) + " quorum reached")
            firstTimeFlag = True
            stopFlag = False
            seed = seed + 1;
            np.random.seed(seed)
    if plot:
        plotGraph()

if __name__ == "__main__":
    main()
