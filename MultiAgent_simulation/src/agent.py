'''
    File name: agent.py
    Author: Mohamed S. Talamali and Andreagiovanni Reina
            University of Sheffield, UK
    Date created: October 2018 (By Francesco Cancianni)
    Date last modified: October 2020 (By Mohamed S. Talamali)
'''

import numpy as np
import math
from msg import Msg
from enum import Enum

def sigmoid(x,S,x0,M):
  return M / (1 + math.exp(-S*(x-x0)))

class ActivityState(Enum):
    INTERACTIVE = 1
    LATENT = 2


class Agent:
    def __init__(self, a_id, colors, decayMethod, decayStrength, discoveryMethod, probabilisticDiscoveryProportion, environmentSize, filterMsgParam, firstEntryOnlyDiscovery, initialTimeValue, interactionFunction, interactiveProb, kParameter, led, maxQuality, moveDimension,msgType, numberOfOpinion, opinion, postStepSelfStrength, postStepSocialStrength, preStepSelfStrength, preStepSocialStrength, orientation, quality, sendConstant, sendInfoMethod, standardDeviation, startWithOpinion, straightLength, updateModel, updateRule, x, y, zealotOpinion, zealotQuality):
        self.activityState = ActivityState.INTERACTIVE # if 0.75 > np.random.uniform(0, 1) else ActivityState.LATENT
        self.id = a_id
        self.colors = colors
        self.counterForDiscoveryEvents = 0
        self.counterForSocialEvents = 0
        self.environmentSize = environmentSize
        self.decayMethod = decayMethod
        self.decayStrength = decayStrength
        self.discoveryMethod = discoveryMethod
        self.probabilisticDiscoveryProportion = probabilisticDiscoveryProportion
        self.goingResampling = False
        self.filterMsgParam = filterMsgParam
        self.firstEntryOnlyDiscovery = firstEntryOnlyDiscovery
        self.initialTimeValue = initialTimeValue
        self.interactionFunction = interactionFunction
        self.interactiveProb = interactiveProb
        self.kParameter = kParameter
        self.lastTotems = []
        self.led = led
        self.maxQuality = maxQuality
        self.moveDimension = moveDimension
        self.msgType = msgType
        self.neighbors = []
        self.neighborsT = []
        self.numberOfOpinion = numberOfOpinion
        self.opinion = opinion
        self.opinionDiscovered = False
        self.orientation = orientation
        self.quality = quality
        self.postStepSelfStrength = postStepSelfStrength
        self.postStepSocialStrength = postStepSocialStrength
        self.preStepSelfStrength = preStepSelfStrength
        self.preStepSocialStrength = preStepSocialStrength
        self.selfStrength = preStepSelfStrength
        self.sendConstant = sendConstant
        self.sendInfoMethod = sendInfoMethod
        self.socialStrength = preStepSocialStrength
        self.standardDeviation = standardDeviation
        self.startWithOpinion = startWithOpinion
        self.stepExecuted = False
        self.straightLength = straightLength
        self.straightLengthCounter = np.random.randint(0, self.straightLength)
        self.timestep = 0
        self.totemPos = [None, None]
        self.updateModelParameter = updateModel
        self.updateRule = updateRule
        self.x = x
        self.y = y
        self.zealotOpinion = zealotOpinion
        self.zealotQuality = zealotQuality
        self.msg = Msg(self.id, self.opinion, self.quality, self.totemPos,0)
        
    def oneStep(self, frozenList, listOfNeighbors, listOfTotems, powTotemRad, step):
        self.timestep = step
        self.resetVariables()
        self.getNeighborsMsg(frozenList, listOfNeighbors)
        if not self.startWithOpinion:
            self.getTotemMsg(listOfTotems, powTotemRad)
        self.updateInteractionRate()
        self.updateOpinion(listOfTotems)
        if self.activityState == ActivityState.LATENT:
            if self.goingResampling:
                self.goToTotem(powTotemRad, listOfTotems)
            else:
                self.probabilisticallyTurnInteractive()
        self.move()
        self.createMsg()


    def resetVariables(self):
        self.neighbors = []
        self.neighborsT = []
        if not self.firstEntryOnlyDiscovery: self.lastTotems.clear()
        self.opinionDiscovered = False

    def setColor(self):
        self.led = self.colors[self.opinion]

    def getNeighborsMsg(self, frozenList, listOfNeighbors):
        # create the neighbors list excluding myself and message with visible==False
        self.neighbors = [ frozenList[idx].msg for idx in listOfNeighbors if frozenList[idx].id != self.id and frozenList[idx].msg.visible ]

    def getTotemMsg(self, listOfTotems, powRadius):
        for totem in listOfTotems:
            dist = self.distanceOnTorus(totem.x, totem.y)
            if( totem.appearanceTime <=self.timestep):
                if dist <= powRadius:
                    if not totem.id in self.lastTotems:
                        self.lastTotems.append(totem.id)
                        self.neighborsT.append(totem.msg)
                else:
                    if totem.id in self.lastTotems:
                        self.lastTotems.remove(totem.id)

    def createMsg(self):
        if self.sendInfoMethod== "constant":
            #always update
            if self.timestep % self.sendConstant == 0 and self.activityState == ActivityState.INTERACTIVE:
                self.msg.visible = True
                self.msg.opinion = self.opinion
                self.msg.totemPos = self.totemPos
                self.msg.active = True
                if self.msgType == "super":
                    self.msg.quality = self.quality
            else:
                self.msg.visible = False

        elif self.sendInfoMethod== "wv":
            # probabilistically send the message with probability proportional to the quality, i.e. P=socialStrength * quality / maxQuality
            if self.activityState == ActivityState.INTERACTIVE:
                self.msg.visible = True
                self.msg.opinion = self.opinion
                self.msg.totemPos = self.totemPos
                if self.msgType == "super":
                    self.msg.quality = self.quality
                # with probability (self.socialStrength * self.quality / self.maxQuality) the message is active
                sendProbability = self.socialStrength * self.quality / self.maxQuality
                self.msg.active = sendProbability > np.random.uniform(0, 1)
            else:
                self.msg.visible = False

    def updateOpinion(self, listOfTotems):
        # Process Self Sensing
        self_newOp, self_newQuality, self_newTotemPos = self.discovery()

        # Process Social Sensing
        self.prefilterNeigbourMsgs()
        social_newOp, social_newQuality, social_newTotemPos = self.processSocialInput(listOfTotems)

        # Modulate Sensing
        if self_newOp!=None and social_newOp!=None: # Both infos are available
            if np.random.uniform(0, 1)>=0.5:
                self.opinionDiscovered = True # Set discovery flag
                self.updateModel(self_newOp, self_newQuality, self_newTotemPos)
            else:
                self.updateModel(social_newOp, social_newQuality, social_newTotemPos)

        elif self_newOp!=None: # Only self info available
            self.opinionDiscovered = True # Set discovery flag
            self.updateModel(self_newOp, self_newQuality, self_newTotemPos)

        elif social_newOp!=None: # Only social info available
            self.updateModel(social_newOp, social_newQuality, social_newTotemPos)

        else: # No info available
            self.updateModel(None, None, None)

        self.setColor()

    # Method to modify the message list before calling the processSocialInput() method, this is necessary for some updateModels (e.g. 'selfInhibition')
    def prefilterNeigbourMsgs(self):
        if self.updateModelParameter == "selfInhibition" and self.opinion !=0:
            ## recomputing the attribute active (True/False) for same-opinion messages (because self-inhibition strength should be different than recruitment's)
            prob = self.filterMsgParam
            for msg in self.neighbors:
                if msg.opinion == self.opinion:
                    msg.active = prob > np.random.uniform(0, 1)

    def discovery(self):
        if not self.goingResampling:
            #Agents that are zealot or mad cannot discover anything
            if self.updateRule == "zealot" or self.updateRule == "mad":
                return None, None, None
            # totems do no exist
            if self.startWithOpinion: return None, None, None
            # no totem encoutered
            if len(self.neighborsT) == 0: return None, None, None

            ## any encountered opinion is selected (if more than one are encountered, one at random is chosen)
            if self.discoveryMethod == "always":
                chosenMsgTotem = np.random.choice(self.neighborsT)
                newOp = chosenMsgTotem.opinion
                # apply random gaussian noise to the opinion quality (trimmed in the range [0,maxQuality])
                if self.standardDeviation == 0:
                    newQ = chosenMsgTotem.quality
                else:
                    newQ = chosenMsgTotem.quality + np.random.normal(0, self.standardDeviation)
                #set newTotemPos with the relative postion of the selected totem
                newTotemPos = chosenMsgTotem.totemPos
                if newQ > self.maxQuality:
                    newQ = self.maxQuality
                elif newQ < 0:
                    newQ = 0
                self.opinionDiscovered = True
                return newOp, newQ, newTotemPos

            ## probabilistic discovery triggered with probability function of the (noisy) normalised quality, i.e. P=(self.selfStrength * quality / self.maxQuality)
            elif self.discoveryMethod == "probabilistic":
                # apply random gaussian noise to quality values (trimmed in the range [0,maxQuality])
                if self.standardDeviation == 0:
                    noisedQualityValues = [ [msgT.opinion, min(self.maxQuality, max(0, msgT.quality)), msgT.totemPos, msgT.disappearanceTime] for msgT in self.neighborsT]
                else:
                    noisedQualityValues = [ [msgT.opinion, min(self.maxQuality, max(0, msgT.quality + np.random.normal(0, self.standardDeviation))), msgT.totemPos, msgT.disappearanceTime] for msgT in self.neighborsT]
                # shuffle the list to avoid any ordering bias
                np.random.shuffle(noisedQualityValues)
                # sequentially check if any opinion is probabilistically discovered
                for msgT in noisedQualityValues:
                    if(msgT[0] != self.opinion):
                        if( msgT[3]<=0 or msgT[3]>self.timestep):

                            if self.quality == 0:
                                if (msgT[1]/self.maxQuality) > np.random.uniform(0.0,1.0):
                                    newOp = msgT[0]
                                    newQ = msgT[1]
                                    newTotemPos = msgT[2]
                                    return newOp, newQ, newTotemPos
                            else:
                                if self.probabilisticDiscoveryProportion == "compare":
                                    if  msgT[1]/self.maxQuality > self.quality/self.maxQuality+self.selfStrength and msgT[1]/self.maxQuality > np.random.uniform(0.0,1.0):
                                        newOp = msgT[0]
                                        newQ = msgT[1]
                                        newTotemPos = msgT[2]
                                        return newOp, newQ, newTotemPos

                                elif self.probabilisticDiscoveryProportion == "resample":
                                    if (self.selfStrength*msgT[1]/self.maxQuality) > np.random.uniform(0.0,1.0):
                                        newOp = msgT[0]
                                        newQ = msgT[1]
                                        newTotemPos = msgT[2]
                                        return newOp, newQ, newTotemPos

                                elif self.probabilisticDiscoveryProportion == "value":
                                    if (msgT[1]/self.maxQuality) > np.random.uniform(0.0,1.0):
                                        newOp = msgT[0]
                                        newQ = msgT[1]
                                        newTotemPos = msgT[2]
                                        return newOp, newQ, newTotemPos

                                elif self.probabilisticDiscoveryProportion == "nothing":
                                     return None, None, None

                    else:
                        self.quality = msgT[1]

                        if(not(msgT[3]<=0 or msgT[3]>self.timestep)):
                            self.quality = 0
            return None, None, None
        else:
            return None, None, None

    # Update the interaction rate to postStep for Social and Self strength for certain interactionFunctions
    def updateInteractionRate(self):
        # update to postStep at a time proportional to the estimated quality
        if self.interactionFunction == "stepTimeValue":
            if not self.stepExecuted: # avoiding useless checks if the step has been already executed
                if self.quality != 0:
                    timeOfCommitment = self.initialTimeValue * (self.maxQuality / self.quality)
                    if self.timestep >= timeOfCommitment:
                        self.socialStrength = self.postStepSocialStrength
                        self.selfStrength = self.postStepSelfStrength
                        self.stepExecuted = True

        # update to postStep at a predefined time self.initialTimeValue
        elif self.interactionFunction == "stepTime":
            if self.timestep >= self.initialTimeValue:
                self.socialStrength = self.postStepSocialStrength
                self.selfStrength = self.postStepSelfStrength

    def processSocialInput(self, listOfTotems):
        #UpdateRules
        if self.updateRule == "random":
            self.neighbors = list(filter(lambda x: x.active, self.neighbors))
            if len(self.neighbors) != 0:
                chosenMsg = np.random.choice(self.neighbors)
                if chosenMsg.opinion != 0 and chosenMsg.active:
                    newOp = chosenMsg.opinion
                    newTotemPos = chosenMsg.totemPos
                    newQ = chosenMsg.quality if self.msgType =="super" else None
                    return newOp, newQ, newTotemPos
            return None, None, None

        #majority without use of quality
        elif self.updateRule == "majority":
            #init
            opinionSet = []
            opinionSum = []
            #add to his neighbors itself
            self.neighbors.append(self.msg)
            #filter msg with opinion = 0 (uncommitted)
            self.neighbors = list(filter(lambda x: x.opinion != 0 and x.active, self.neighbors))
            if len(self.neighbors) != 0:
                opinionSet = [msg.opinion for msg in self.neighbors]
                opinionSum = [opinionSet.count(x) for x in range(self.numberOfOpinion + 1)]
                maxOccur = np.amax(opinionSum)
                indexes = np.argwhere(opinionSum == maxOccur).flatten().tolist()
                newOp = np.random.choice(indexes)
                #filter the neighbors based on the newOp
                listOfMsgSameOp = list(filter(lambda x: x.opinion == newOp and x.active, self.neighbors))
                np.random.shuffle(listOfMsgSameOp)
                newTotemPos = listOfMsgSameOp[0].totemPos
                newQ = None
                if self.msgType =="super":
                    if newOp != self.opinion:
                        #computation of the avg of qualities
                        newQ = np.mean([msg.quality for msg in listOfMsgSameOp])
                    else:
                        newOp = self.opinion
                        newQ = self.quality
                        newTotemPos = self.totemPos
                return newOp, newQ, newTotemPos
            return None, None, None

        #Attacker method -> opposite of majority
        elif self.updateRule == "minority":
            #init
            opinionSet = []
            opinionSum = []
            #add to his neighbors itself
            self.neighbors.append(self.msg)
            #filter msg with opinion = 0 (uncommitted)
            self.neighbors = list(filter(lambda x: x.opinion != 0 and x.active, self.neighbors))
            if len(self.neighbors) != 0:
                opinionSet = [msg.opinion for msg in self.neighbors]
                # counting the message for each option and setting to high values the count 0
                opinionSum = [opinionSet.count(x) if opinionSet.count(x) >0 else len(self.neighbors)+1 for x in range(self.numberOfOpinion + 1)]
                minOccur = np.amin(opinionSum)
                indexes = np.argwhere(opinionSum == minOccur).flatten().tolist()
                newOp = np.random.choice(indexes)
                listOfMsgSameOp = list(filter(lambda x: x.opinion == newOp and x.active, self.neighbors))
                np.random.shuffle(listOfMsgSameOp)
                newTotemPos = listOfMsgSameOp[0].totemPos
                newQ = None
                if self.msgType =="super":
                    if newOp != self.opinion:
                        #computation of the avg of qualities
                        newQ = np.mean([msg.quality for msg in listOfMsgSameOp])
                    else:
                        newOp = self.opinion
                        newQ = self.quality
                        newTotemPos = self.totemPos
                return newOp, newQ, newTotemPos
            return None, None, None

        elif self.updateRule == "kUnanimity":
            #filter msg with opinion = 0 (uncommitted)
            self.neighbors = list(filter(lambda x: x.opinion != 0 and x.active, self.neighbors))
            if self.kParameter <= len(self.neighbors):
                np.random.shuffle(self.neighbors)
                self.neighbors = self.neighbors[len(self.neighbors) - self.kParameter : len(self.neighbors)]
                if all(x.opinion == self.neighbors[0].opinion for x in self.neighbors):
                    newOp = self.neighbors[0].opinion
                    newTotemPos = self.neighbors[0].totemPos
                    if self.msgType == "super":
                        newQ = (sum(map(lambda x : x.quality, self.neighbors)) / len(self.neighbors))
                    else:
                        newQ = None
                    return newOp, newQ, newTotemPos
            return None, None, None

        elif self.updateRule == "zealot":
            if self.opinion == 0:
                self.opinion = self.zealotOpinion;
                #We set the quality to the maximum in order to manage how much he can diffuse the message
                self.quality = self.zealotQuality;
            return None, None, None

        elif self.updateRule == "mad":
            self.opinion = np.random.randint(1, self.numberOfOpinion + 1)
            self.quality = np.random.uniform(0, self.maxQuality)
            filteredTotem = list(filter(lambda x: x.opinion == self.opinion, listOfTotems))
            np.random.shuffle(filteredTotem)
            self.totemPos = filteredTotem[0].msg.totemPos
            return None, None, None

        elif self.updateRule == "maxQuality":
            self.neighbors.append(self.msg)
            qualities = [neighbor.quality for neighbor in self.neighbors if neighbor.active]
            if len(qualities) == 0:
                return None, None, None
            tmpIndexes = np.argwhere(qualities == np.amax(qualities))
            indexes = tmpIndexes.flatten().tolist()
            newOp = self.neighbors[indexes[0]].opinion
            newQ = self.neighbors[indexes[0]].quality
            newTotemPos = self.neighbors[indexes[0]].totemPos
            return newOp, newQ, newTotemPos

    def decay(self):
        if self.activityState == ActivityState.INTERACTIVE:
            if self.decayMethod == "constant":
                if self.decayStrength > np.random.uniform(0, 1):
                    return True
            elif self.decayMethod == "probabilistic":
                if self.quality != 0:
                    # probabilistic abandonment function of quality
                    if self.decayStrength / self.quality > np.random.uniform(0, 1):
                        return True
        return False

    def updateModel(self, newOp, newQ, newTotemPos):
        #opinion changed should be set to True when newOp triggers a change; in case opinionChanged==True, decay() is not called, otherwise it is
        opinionChanged = False

        if not self.startWithOpinion and newOp != None:
            if self.updateModelParameter == "direct":
                # DISCOVERY / SWITCH: when self.opinion != newOp (and not from uncommitted), the agent switch to that opinion
                if self.opinion != newOp and newOp != 0:
                    self.setNewOpinion(newOp, newQ, newTotemPos)
                    opinionChanged = True
                    
            elif self.updateModelParameter == "crossInhibition":
                # CROSS-INHIBITION: when my opinion != 0 and I receive a different opinion, I get cross-inhibited
                if self.opinion != newOp and self.opinion != 0:
                    self.opinion = 0
                    self.totemPos = [None, None]
                    if self.msgType == "super":
                        self.quality = 0
                    opinionChanged = True
                # DISCOVERY / RECRUITMENT: when my opinion == 0 and I receive an opinion != 0, I get that opinion
                elif self.opinion != newOp and self.opinion == 0:
                    self.setNewOpinion(newOp, newQ, newTotemPos)
                    opinionChanged = True

            elif self.updateModelParameter == "selfInhibition":
                # if newOp is the same self opinion which is not 0 and not coming from a totem, then apply self.inhibition
                if self.opinion == newOp and self.opinion != 0 and not self.opinionDiscovered:
                    self.opinion = 0
                    self.totemPos = [None, None]
                    if self.msgType == "super":
                        self.quality = 0
                    opinionChanged = True
                    # to allow immediate discovery (which was ignored when committed) we need to clear the lastTotem list
                    # Note that totems cannot self-inhibit (differently from direct-switch and cross-inhibition)
                    if self.firstEntryOnlyDiscovery: self.lastTotems.clear()
                # if self.opinion == 0 apply discovery and recruitment
                elif self.opinion != newOp and self.opinion == 0:
                    self.setNewOpinion(newOp, newQ, newTotemPos)
                    opinionChanged = True

            elif self.updateModelParameter == "recruitmentOnly":
                # if self.opinion == 0 apply discovery and recruitment
                if self.opinion == 0 and self.opinion != newOp:
                    self.setNewOpinion(newOp, newQ, newTotemPos)
                    opinionChanged = True

        #Must be checked and corrected
        elif self.startWithOpinion and newOp != None:
            if self.opinion != newOp:
                if self.updateModelParameter == "direct" or self.opinion == 0:
                    self.opinion = newOp
                    self.totemPos = newTotemPos
                    if self.msgType == "super":
                        self.quality = newQ
                elif self.updateModelParameter == "crossInhibition":
                    self.opinion = 0
                    self.totemPos = [None, None]
                    if self.msgType == "super":
                        self.quality = 0
        #Update discovery and social interaction counter
        if opinionChanged:
            if self.opinionDiscovered:
                self.counterForDiscoveryEvents = self.counterForDiscoveryEvents + 1
            else:
                self.counterForSocialEvents = self.counterForSocialEvents + 1

        # if the opinion has not been just updated (i.e. opinionChanged==False), we apply decay()
        if not opinionChanged:
            if self.opinion != 0 and self.decay():
                self.opinion = 0
                self.totemPos = [None, None]
                if self.msgType == "super":
                    self.quality = 0
                opinionChanged = True

        # After every state change the agent goes into a LATENT state
        if opinionChanged:
            self.activityState = ActivityState.LATENT

    #Set the opinion to newOp and newQ, if necessary trigger the resampling behaviour
    def setNewOpinion(self, newOp, newQ, newTotemPos):
        # DISCOVERY: the agent gets opinion and quality from the totem and does not need to the resampling behaviour
        if self.opinionDiscovered:
            self.opinion = newOp
            self.quality = newQ
            self.goingResampling = False
            self.totemPos = newTotemPos

        # RECRUITMENT: the super agent receives the quality, instead the simple agent go resampling to estimate the quality
        else:
            if self.msgType == "super":
                self.opinion = newOp
                self.quality = newQ
                self.goingResampling = False
            else: # resampling is needed
                if newOp != self.opinion:
                    self.goingResampling = True
                    self.opinion = newOp
                    self.quality = 0
                    self.totemPos = newTotemPos
                    # compute the direction angle towards the totemPos
                    self.orientation = self.computeAngleForShortestPathToTotem()

    def _computeAngleToTotem(self):
        dx = self.totemPos[0] - self.x
        dy = self.totemPos[1] - self.y
        rads = math.atan2(dy, dx)
        deg = math.degrees(rads)
        if deg < 0:
            deg = 360 + deg
        return deg

    def computeAngleForShortestPathToTotem(self):
        tempX = self.totemPos[0]
        tempY = self.totemPos[1]
        if self.environmentSize - abs(self.x - self.totemPos[0]) < abs(self.x - self.totemPos[0]):
            #DX
            if self.environmentSize - self.x < self.x:
                tempX = tempX + self.environmentSize
            #SX
            else:
                tempX = tempX - self.environmentSize
        if self.environmentSize - abs(self.y - self.totemPos[1]) < abs(self.y - self.totemPos[1]):
            #UP
            if self.environmentSize - self.y < self.y:
                tempY = tempY + self.environmentSize
            #DOWN
            else:
                tempY = tempY - self.environmentSize
        dx = tempX - self.x
        dy = tempY - self.y
        rads = math.atan2(dy, dx)
        deg = math.degrees(rads)
        if deg < 0:
            deg = 360 + deg
        return deg

    # Probabilistically the agent changes from LATENT to INTERACTIVE state (when it is not goingToReample)
    def probabilisticallyTurnInteractive(self):
        if self.interactiveProb > np.random.uniform(0, 1):
            self.activityState = ActivityState.INTERACTIVE

    #Two totem same position will return the first one! Aggiungere ID
    def goToTotem(self, powRadius, listOfTotems):
        dist = self.distanceOnTorus(self.totemPos[0], self.totemPos[1])
        if dist <= powRadius:
            newQuality = [totem.quality for totem in listOfTotems if (totem.x == self.totemPos[0] and totem.y == self.totemPos[1])]
            disappearanceTime=[totem.disappearanceTime for totem in listOfTotems if (totem.x == self.totemPos[0] and totem.y == self.totemPos[1])]
            if( disappearanceTime[0]<=0 or disappearanceTime[0]>self.timestep):
                if self.standardDeviation == 0:
                    self.quality = newQuality[0]
                else:
                    self.quality = newQuality[0] + np.random.normal(0, self.standardDeviation)
                if self.quality > self.maxQuality:
                    self.quality = self.maxQuality
                elif self.quality < 0:
                    self.quality = 0
            else:
                self.quality = 0

            self.goingResampling = False
            self.activityState = ActivityState.INTERACTIVE
            #when it has reached the totem set a random orientation to improve exploration
            self.orientation = np.random.uniform(0, 360)

    def move(self):
        self.straightLengthCounter += 1;
        if not self.goingResampling:
            #it checks if the agent has to change its direction.
            if(self.straightLengthCounter % self.straightLength == 0):
                self.orientation = np.random.uniform(0, 360)
                self.straightLengthCounter = 0
        step_x = np.cos(self.orientation * np.pi / 180.) * self.moveDimension
        step_y = np.sin(self.orientation * np.pi / 180.) * self.moveDimension
        self.x += step_x
        self.y += step_y
        # check if agent is out of bounds
        if self.x > self.environmentSize:
            self.x = (self.x - self.environmentSize)
        elif self.x < 0:
            self.x = (self.x + self.environmentSize)

        if self.y > self.environmentSize:
            self.y = (self.y - self.environmentSize)
        elif self.y < 0:
            self.y = (self.y + self.environmentSize)

    def minDistanceFromTotems(self, selectedTotemsPos):
        distances = []
        for position in selectedTotemsPos:
            distances.append(self.distanceOnTorus(position[0], position[1]))
        minDistIndex = np.argmin(distances)
        return selectedTotemsPos[minDistIndex][0], selectedTotemsPos[minDistIndex][1]

    def distanceOnTorus(self, x1, y1):
        result = ((min(abs(self.x - x1), self.environmentSize - abs(self.x - x1)))**2 +
         (min(abs(self.y - y1), self.environmentSize - abs(self.y - y1)))**2)
        return result
