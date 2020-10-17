'''
    File name: agent.py
    Author: Mohamed S. Talamali and Andreagiovanni Reina
            University of Sheffield, UK
    Date created: October 2018 (By Francesco Cancianni)
    Date last modified: October 2020 (By Mohamed S. Talamali)
'''

import json
import configparser
import math

class ErrorController:
    def __init__(self, configurationFilePath):
        self.config = configparser.ConfigParser()
        try:
            self.config.readfp(open(configurationFilePath))
        except (configparser.MissingSectionHeaderError):
            print("MISSING A SECTION IN THE FILE")
            exit()

        self.errorFounded = False
        #List of different types of elements:
        self.paramsString = ["testName", "finalResultFileName", "statisticsFileName", "resultFolderPath"]
        self.paramsStringList = ["colors","compositionLabels", "discoveryMethod", "updateModel", "updateRule", "msgType", "sendInfoMethod", "decayMethod", "interactionFunction", "colorsFile"]
        self.paramsZeroToInfFloat = ["environmentSize", "maxQuality", "agentRadius", "totemRadius","noiseStandardDeviationValue"]
        self.paramsZeroToInfInt = ["seed", "straightLength", "initialTimeValue", "animationInterval"]
        self.paramsZeroToInfIntList = ["numberOfTotems"]
        self.paramsZeroToOneFloat = ["quorum","interactiveProb"]
        self.paramsZeroToOneFloatList = ["composition", "decayStrength", "preStepSelfStrength", "postStepSelfStrength", "preStepSocialStrength", "postStepSocialStrength", "filterMsgParam"]
        self.paramsOneToInfInt = ["numberOfAgents", "numberOfOpinion", "numberOfSimulation", "numberOfStartingPoint", "numberOfSteps", "sendConstant"]
        self.paramsOneToAgentCardinalityInt = ["kUnanimityParameter"]
        self.paramasOneToNumberOfOpInt = ["zealotOpinion"]
        self.paramsZeroToEnvironmentFloat = ["moveDimension"]
        self.paramsZeroToMaxQFloat = ["zealotQuality"]
        self.paramsZeroToMaxQFloatList = ["qualityValues"]
        self.paramsBoolean = ["startingPoint", "startWithOpinion", "graphSimulation", "plot", "overlappingTotems", "firstEntryOnlyDiscovery" ]

        #List of permitted values
        self.permittedValuesStringLists = {
            "colors": ["gray","red","blue","green","yellow","violet","darkorange","navy","peru","turquoise","olive","cadetblue","indigo"],
            "colorsFile": ["0","R","B","G","Y","V","D","N","P","T","O","C","I"],
            "decayMethod": ["constant", "probabilistic"],
            "discoveryMethod": ["always", "probabilistic"],
            "interactionFunction":["constant", "stepTime", "stepTimeValue"],
            "msgtype": ["simple", "super"],
            "sendInfoMethod": ["constant", "wv"],
            "updateModel": ["direct", "crossInhibition", "selfInhibition", "recruitmentOnly"],
            "updateRule": ["kUnanimity", "mad", "majority", "maxQuality", "minority", "random", "zealot"]
        }

        #List of element per section
        self.experimentSection = ["colors", "composition", "compositionLabels", "environmentSize", "noiseStandardDeviationValue", "numberOfAgents", "numberOfSimulation", "numberOfOpinion", "numberOfStartingPoint", "numberOfSteps", "numberOfTotems", "overlappingTotems", "quorum", "seed", "startingPoint"]
        self.modelUpdateSection = ["discoveryMethod", "kUnanimityParameter", "maxQuality", "qualityValues", "updateModel", "updateRule", "zealotOpinion", "zealotQuality"]
        self.msgSection = ["msgType", "sendConstant", "sendInfoMethod"]
        self.movementSection = ["moveDimension", "straightLength"]
        self.agentTotemCharacteristicsSection = ["agentRadius", "startWithOpinion", "totemRadius", "firstEntryOnlyDiscovery"]
        self.decaySection = ["decayMethod", "decayStrength"]
        self.interactionSection =["interactionFunction", "preStepSocialStrength", "postStepSocialStrength", "preStepSelfStrength", "postStepSelfStrength", "initialTimeValue", "interactiveProb", "filterMsgParam"]
        self.logFilesSection = ["colorsFile", "finalResultFileName", "resultFolderPath", "statisticsFileName", "testName"]
        self.grapicPlotSection =["animationInterval", "graphSimulation", "plot"]

        self.userValues = {
                    "agentRadius": 0,
                    "animationInterval": 0,
                    "colors": [],
                    "colorsFile": [],
                    "composition": [],
                    "compositionLabels": [],
                    "decayMethod": [],
                    "decayStrength": [],
                    "discoveryMethod": [],
                    "environmentSize": 0,
                    "filterMsgParam": [],
                    "finalResultFileName": "null",
                    "firstEntryOnlyDiscovery": False,
                    "graphSimulation": False,
                    "initialTimeValue": 0,
                    "interactionFunction": [],
                    "interactiveProb": 1,
                    "kUnanimityParameter": 0,
                    "maxQuality": 0,
                    "moveDimension": 0,
                    "msgType": [],
                    "noiseStandardDeviationValue": 0,
                    "numberOfAgents": 0,
                    "numberOfOpinion": 0,
                    "numberOfSimulation": 0,
                    "numberOfStartingPoint": 0,
                    "numberOfSteps": 0,
                    "numberOfTotems": [],
                    "overlappingTotems": True,
                    "plot": False,
                    "postStepSelfStrength": [],
                    "postStepSocialStrength": [],
                    "preStepSelfStrength": [],
                    "preStepSocialStrength": [],
                    "qualityValues": [],
                    "quorum": 0,
                    "resultFolderPath": "null",
                    "seed": 0,
                    "sendConstant": 0,
                    "sendInfoMethod": [],
                    "startingPoint": False,
                    "startWithOpinion": False,
                    "statisticsFileName": "null",
                    "straightLength": 0,
                    "testName": "null",
                    "totemRadius": 0,
                    "updateModel": [],
                    "updateRule": [],
                    "zealotOpinion": 0,
                    "zealotQuality": 0,
        }

    def fromStringToArray(self, string):
        #remove the [ ]
        string = string.replace("[", "")
        string = string.replace("]", "")
        #remove spaces " "
        string = string.replace(" ", "")
        #create the array from the string
        array = string.split(",")
        return array

    #initialise the value per each parameters contained in the configuration file
    #check also if each section and each parametes exists
    def initWithCheck(self):
        for key, _ in self.userValues.items():
            if key in self.paramsStringList:
                section = self.sectionDetection(key)
                try:
                    self.userValues[key] = self.config.get(section, key)
                    self.userValues[key] = self.fromStringToArray(self.userValues[key])
                    for elem in self.userValues[key]:
                        if not isinstance(elem, str):
                            print(key + " HAS TO BE A LIST OF STRING\n")
                            self.errorFounded = True
                            break
                except (ValueError):
                    print(key + " HAS TO BE A LIST OF STRING\n")
                    self.errorFounded = True
                except (configparser.NoOptionError):
                    print(key + " HAS TO BE INSERTED IN CONFIGURATION.config FILE UNDER SECTION " + section +"\n")
                    self.errorFounded = True
                except (configparser.NoSectionError):
                    print(section + " SECTION HAS TO BE INSERTED IN CONFIGURATION.config FILE\n")
                    self.errorFounded = True
                    break

            elif key in self.paramsString:
                section = self.sectionDetection(key)
                try:
                    self.userValues[key] = self.config.get(section, key)
                except (ValueError):
                    print(key + " HAS TO BE A STRING\n")
                    self.errorFounded = True
                except (configparser.NoOptionError):
                    print(key + " HAS TO BE INSERTED IN CONFIGURATION.config FILE UNDER SECTION " + section +"\n")
                    self.errorFounded = True
                except (configparser.NoSectionError):
                    print(section + " SECTION HAS TO BE INSERTED IN CONFIGURATION.config FILE\n")
                    self.errorFounded = True
                    break
            elif (key in self.paramsZeroToInfFloat or key in self.paramsZeroToOneFloat or
                key in self.paramsZeroToEnvironmentFloat or key in self.paramsZeroToMaxQFloat):
                section = self.sectionDetection(key)
                try:
                    self.userValues[key] = self.config.getfloat(section, key)
                except (ValueError):
                    print(key + " HAS TO BE A FLOAT\n")
                    self.errorFounded = True
                except (configparser.NoOptionError):
                    print(key + " HAS TO BE INSERTED IN CONFIGURATION.config FILE UNDER SECTION " + section +"\n")
                    self.errorFounded = True
                except (configparser.NoSectionError):
                    print(section + " SECTION HAS TO BE INSERTED IN CONFIGURATION.config FILE\n")
                    self.errorFounded = True
                    break
            elif (key in self.paramsZeroToInfInt or
                    key in self.paramsOneToInfInt or
                    key in self.paramsOneToAgentCardinalityInt or
                    key in self.paramasOneToNumberOfOpInt):
                section = self.sectionDetection(key)
                try:
                    self.userValues[key] = self.config.getint(section, key)
                except (ValueError):
                    print(key + " HAS TO BE A INT\n")
                    self.errorFounded = True
                except (configparser.NoOptionError):
                    print(key + " HAS TO BE INSERTED IN CONFIGURATION.config FILE UNDER SECTION " + section +"\n")
                    self.errorFounded = True
                except (configparser.NoSectionError):
                    print(section + " SECTION HAS TO BE INSERTED IN CONFIGURATION.config FILE\n")
                    self.errorFounded = True
                    break
            elif key in self.paramsZeroToInfIntList:
                section = self.sectionDetection(key)
                try:
                    _riseExceptionIfNotExists = self.config.get(section, key)
                    self.userValues[key] = json.loads(self.config[section][key])
                    for elem in self.userValues[key]:
                        if not isinstance(elem, int):
                            print(key + " HAS TO BE A LIST OF INT\n")
                            self.errorFounded = True
                            break
                except (ValueError):
                    print(key + " HAS TO BE A LIST OF INT\n")
                    self.errorFounded = True
                except (configparser.NoOptionError):
                    print(key + " HAS TO BE INSERTED IN CONFIGURATION.config FILE UNDER SECTION " + section +"\n")
                    self.errorFounded = True
                except (configparser.NoSectionError):
                    print(section + " SECTION HAS TO BE INSERTED IN CONFIGURATION.config FILE\n")
                    self.errorFounded = True
                    break
            elif (key in self.paramsZeroToOneFloatList or
                    key in self.paramsZeroToMaxQFloatList or
                    key in self.paramsZeroToOneFloatList):
                section = self.sectionDetection(key)
                try:
                    _riseExceptionIfNotExists = self.config.get(section, key)
                    self.userValues[key] = json.loads(self.config[section][key])
                    for elem in self.userValues[key]:
                        if not isinstance(elem, float):
                            if not isinstance(elem, int):
                                self.errorFounded = True
                                print(key + " HAS TO BE A LIST OF FLOAT\n")
                                break
                except (ValueError):
                    print(key + " HAS TO BE A LIST OF FLOAT")
                    self.errorFounded = True
                except (configparser.NoOptionError):
                    print(key + " HAS TO BE INSERTED IN CONFIGURATION.config FILE UNDER SECTION " + section +"\n")
                    self.errorFounded = True
                except (configparser.NoSectionError):
                    print(section + " SECTION HAS TO BE INSERTED IN CONFIGURATION.config FILE\n")
                    self.errorFounded = True
                    break
            elif key in self.paramsBoolean:
                section = self.sectionDetection(key)
                try:
                    self.userValues[key] = self.config.getboolean(section, key)
                except (ValueError):
                    print(key + " HAS TO BE A BOOLEAN")
                    self.errorFounded = True
                except (configparser.NoOptionError):
                    print(key + " HAS TO BE INSERTED IN CONFIGURATION.config FILE UNDER SECTION " + section +"\n")
                    self.errorFounded = True
                except (configparser.NoSectionError):
                    print(section + " SECTION HAS TO BE INSERTED IN CONFIGURATION.config FILE\n")
                    self.errorFounded = True
                    break

    #Detects the correct section per each key and returns the corresponding string
    def sectionDetection(self, key):
        if key in self.experimentSection:
            return "experiment"
        elif key in self.modelUpdateSection:
            return "model_update"
        elif key in self.msgSection:
            return "msg"
        elif key in self.movementSection:
            return "movement"
        elif key in self.agentTotemCharacteristicsSection:
            return "agent_totem_characteristics"
        elif key in self.decaySection:
            return "decay"
        elif key in self.interactionSection:
            return "interaction"
        elif key in self.logFilesSection:
            return "log_files"
        elif key in self.grapicPlotSection:
            return "graphic_plot"

    def errorHandler(self):

        #Check if all the parameters are of the correct type and name of sections are correct
        self.initWithCheck()
        #If an error is founded the execution of the program is ended
        if self.errorFounded:
            exit()

        for paramName, paramValue in self.userValues.items():
            if paramName in self.permittedValuesStringLists:
                for elem in paramValue:
                    if elem not in self.permittedValuesStringLists[paramName]:
                        print("ERROR - WRONG CONFIGURATION")
                        print("MISSED OR INVALID ESSENTIALS PARAMETERS: " + str(paramName))
                        print("THE FOLLOWING ELEMENT MUST BE DELETED: " + str(elem))
                        print("POSSIBLE CONFIGURATION: "+ str(paramName) + " = " + str(self.permittedValuesStringLists[paramName]) + "\n")
                        self.errorFounded = True
            elif paramName in self.paramsZeroToInfFloat:
                if paramValue < 0:
                    print("ERROR - WRONG CONFIGURATION")
                    print("MISSED OR INVALID ESSENTIALS PARAMETERS: " + str(paramName))
                    print("POSSIBLE CONFIGURATION: "+ str(paramName) + " = float [0, +inf]\n")
                    self.errorFounded = True
            elif paramName in self.paramsZeroToInfInt:
                if paramValue < 0:
                    print("ERROR - WRONG CONFIGURATION")
                    print("MISSED OR INVALID ESSENTIALS PARAMETERS: " + str(paramName))
                    print("POSSIBLE CONFIGURATION: "+ str(paramName) + " = int [0, +inf]\n")
                    self.errorFounded = True
            elif paramName in self.paramsZeroToOneFloat:
                if paramValue < 0 or paramValue > 1:
                    print("ERROR - WRONG CONFIGURATION")
                    print("MISSED OR INVALID ESSENTIALS PARAMETERS: " + str(paramName))
                    print("POSSIBLE CONFIGURATION: "+ str(paramName) + " = float [0, 1]\n")
                    self.errorFounded = True
            elif paramName in self.paramsOneToInfInt:
                if paramValue < 1:
                    print("ERROR - WRONG CONFIGURATION")
                    print("MISSED OR INVALID ESSENTIALS PARAMETERS: " + str(paramName))
                    print("POSSIBLE CONFIGURATION: "+ str(paramName) + " = int [1, +inf]\n")
                    self.errorFounded = True
            elif paramName in self.paramsBoolean:
                if paramValue != True and paramValue != False:
                    print("ERROR - WRONG CONFIGURATION")
                    print("MISSED OR INVALID ESSENTIALS PARAMETERS: " + str(paramName))
                    print("POSSIBLE CONFIGURATION: "+ str(paramName) + " = True or False\n")
                    self.errorFounded = True
            elif paramName in self.paramsOneToAgentCardinalityInt:
                if paramValue < 1 or paramValue > self.userValues["numberOfAgents"]:
                    print("ERROR - WRONG CONFIGURATION")
                    print("MISSED OR INVALID ESSENTIALS PARAMETERS: " + str(paramName))
                    print("POSSIBLE CONFIGURATION: "+ str(paramName) + " = int [1, NUMBER_OF_AGENTS]\n")
                    self.errorFounded = True
            elif paramName in self.paramasOneToNumberOfOpInt:
                if (paramValue != 0 and paramValue <= 1) or paramValue > self.userValues["numberOfOpinion"]:
                    print("ERROR - WRONG CONFIGURATION")
                    print("MISSED OR INVALID ESSENTIALS PARAMETERS: " + str(paramName))
                    print("POSSIBLE CONFIGURATION: "+ str(paramName) + " = int (1, NUMBER_OF_OPINION] or 0 if zealot are considered like individuals\n")
                    self.errorFounded = True
            elif paramName in self.paramsZeroToEnvironmentFloat:
                if paramValue < 0 and paramValue > self.userValues["environmentSize"]:
                    print("ERROR - WRONG CONFIGURATION")
                    print("MISSED OR INVALID ESSENTIALS PARAMETERS: " + str(paramName))
                    print("POSSIBLE CONFIGURATION: "+ str(paramName) + " = float [0, ENVIRONMENT_SIZE]\n")
                    self.errorFounded = True
            elif paramName in self.paramsZeroToMaxQFloat:
                if paramValue < 0 or paramValue > self.userValues["maxQuality"]:
                    print("ERROR - WRONG CONFIGURATION")
                    print("MISSED OR INVALID ESSENTIALS PARAMETERS: " + str(paramName))
                    print("POSSIBLE CONFIGURATION: "+ str(paramName) + " = float [0, MAX_QUALITY]\n")
                    self.errorFounded = True
            elif paramName in self.paramsZeroToMaxQFloatList:
                for elem in paramValue:
                    if elem < 0 or elem > self.userValues["maxQuality"]:
                        print("ERROR - WRONG CONFIGURATION")
                        print("MISSED OR INVALID ESSENTIALS PARAMETERS: " + str(paramName))
                        print("POSSIBLE CONFIGURATION: "+ str(paramName) + " = [LIST] float [0, MAX_QUALITY]\n")
                        self.errorFounded = True
            elif paramName in self.paramsZeroToOneFloatList:
                for elem in paramValue:
                    if elem < 0 or elem > 1:
                        print("ERROR - WRONG CONFIGURATION")
                        print("MISSED OR INVALID ESSENTIALS PARAMETERS: " + str(paramName))
                        print("POSSIBLE CONFIGURATION: "+ str(paramName) + " = [LIST] float [0, 1]\n")
                        self.errorFounded = True

        if self.errorFounded:
            exit()

        #Extra check
        if self.userValues["startWithOpinion"]:
            if len(self.userValues["numberOfTotems"]) != 1 or self.userValues["numberOfTotems"][0] != 0:
                print("ERROR - WRONG CONFIGURATION")
                print("INSERT A VALID CONFIGURATION FOR START_WITH_OPINION = TRUE")
                print("NUMBER OF TOTEM MUST BE EQUAL TO 0 AND len(NUMBER_OF_TOTEM = 0) -> NUMBER_OF_TOTEM = [0]\n")
                self.errorFounded = True

        if self.userValues["numberOfOpinion"] >= len(self.userValues["colors"]):
            print("ERROR - WRONG CONFIGURATION")
            print("MISSED OR INVALID ESSENTIALS PARAMETERS: COLORS")
            print("POSSIBLE CONFIGURATION: MORE COLORS ARE AVAILBALE HERE:")
            print("https://matplotlib.org/examples/color/named_colors.html\n")
            self.errorFounded= True

        if len(self.userValues["colorsFile"]) != len(self.userValues["colors"]):
            print("ERROR - WRONG CONFIGURATION")
            print("MISSED OR INVALID ESSENTIALS PARAMETERS: COLORS_FILE")
            print("POSSIBLE CONFIGURATION: MORE COLORS ARE AVAILBALE HERE:")
            print("https://matplotlib.org/examples/color/named_colors.html")
            print("len(colorsFile) MUST BE EQUAL len(colors)\n")
            self.errorFounded= True

        if len(self.userValues["numberOfTotems"]) != self.userValues["numberOfOpinion"] and len(self.userValues["numberOfTotems"]) != 1 and not self.userValues["startWithOpinion"]:
            print("ERROR - WRONG CONFIGURATION")
            print("INVALID ESSENTIALS PARAMETERS:: NUMBER_OF_TOTEMS")
            print("POSSIBLE CONFIGURATION: START_WITH_OPINION == FALSE -> len(NUMBER_OF_TOTEMS) = 1 or len(NUMBER_OF_TOTEMS) = NUMBER_OF_OPINION\n")
            self.errorFounded= True

        if self.userValues["numberOfTotems"] != None and not self.userValues["startWithOpinion"]:
            for elem in self.userValues["numberOfTotems"]:
                if elem < 0 or not isinstance(elem, int):
                    print("ERROR - WRONG CONFIGURATION")
                    print("INVALID ESSENTIALS PARAMETERS:: NUMBER_OF_TOTEMS")
                    print("POSSIBLE CONFIGURATION: START_WITH_OPINION == FALSE -> EACH ELEMENT IN NUMBER_OF_TOTEMS MUST BE AN INTEGER >= 1\n")
                    self.errorFounded= True
                    break

        if  ((self.userValues["statisticsFileName"] != "null" or self.userValues["finalResultFileName"] != "null") and
         self.userValues["resultFolderPath"] == "null"):
            print("ERROR - WRONG CONFIGURATION")
            print("MISSED ONE OF THE ESSENTIALS PARAMETERS: RESULT_FOLDER_PATH")
            print("STATISTICS_FILE_NAME != NONE OR FINAL_RESULT_FN != NONE -> RESULT_FOLDER_PATH != NONE\n")
            self.errorFounded= True

        if (self.userValues["statisticsFileName"] == "null" or self.userValues["finalResultFileName" ] == "null") and self.userValues["plot"]  == True:
            print("ERROR - WRONG CONFIGURATION")
            print("INVALID COMBO OF PARAMETERS: STATISTICS_FILE_NAME AND PLOT")
            print("STATISTICS_FILE_NAME == NONE OR  FINAL_RESULT_FN == NONE -> PLOT == NONE\n")
            self.errorFounded= True

        #MUST BE BETTER IMPLEMENTED
        sameLength = ["composition","compositionLabels", "sendInfoMethod","interactionFunction","preStepSelfStrength","postStepSelfStrength","preStepSocialStrength","postStepSocialStrength","decayMethod","decayStrength","discoveryMethod"]
        for elem in sameLength:
            if len(self.userValues["updateRule"]) != len(self.userValues[elem]):
                print("ERROR - WRONG CONFIGURATION")
                print("WRONG DIMENSION PARAMETER: " + str(elem))
                print("THE FOLLOWING PARAMETER MUST HAVE THE SAME DIMENSIONS:\nUPDATE_RULE\nCOMPOSITION\nCOMPOSITION_LABELS\nSEND_METHOD\nINTERACTION_FUNCTION\nPRE_STEP_\nPOST_STEP _SELF_STRENGTH\nPRE_STEP\nPOST_STEP _SOCIAL_STRENGTH\nDECAY_METHOD\nDECAY_VALUE\nDECAY_STRENGTH\nDISCOVERY_METHOD\n")
                self.errorFounded = True
                break

        if self.userValues["composition"] != None:
            compsum = math.fsum(self.userValues["composition"])
            if compsum != 1:
                print("ERROR - WRONG CONFIGURATION")
                print("WRONG ELEMENTS IN PARAMETER: COMPOSITION")
                print("CONSTRAINT: THE SUM OF THE ELEMENTS OF COMPOSITION MUST BE EQUAL TO 1\n")
                self.errorFounded= True

        if "zealot" in self.userValues["updateRule"]:
            if self.userValues["numberOfOpinion"] == 1:
                print("ERROR - WRONG CONFIGURATION")
                print("WRONG ELEMENTS IN PARAMETER: NUMBER_OF_OPINION")
                print("CONSTRAINT: IF ZEALOT IS USED IN UPDATE_RULE, THE NUMBER_OF_OPINION > 1\n")
                self.errorFounded = True

        if (self.userValues["msgType"] == "simple" and self.userValues["updateRule"] == "maxQuality"):
            print("ERROR - WRONG CONFIGURATION")
            print("INVALID ESSENTIALS PARAMETERS COMBO: MAXQUALITY -> MSG_TYPE = super\n")
            self.errorFounded= True

        if self.errorFounded:
            exit()
