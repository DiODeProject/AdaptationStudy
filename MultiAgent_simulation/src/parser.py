'''
    File name: parser.py
    Author: Francesco Canciani and Andreagiovanni Reina
            University of Sheffield, UK
    Date created: November 2018
    Date last modified:
'''

import numpy as np
import matplotlib.pyplot as plt
import re

import json
import configparser

class Parser:
    def __init__(self, filepath, configurationFilePath, typeOfGraph):
        self.filepath = filepath
        self.typeOfGraph = typeOfGraph
        self.configurationFilePath = configurationFilePath
        if self.typeOfGraph == "BOX PLOT":
            self.rx_dict = {
                'seed': re.compile(r'SEED: (?P<seed>\d+)\n'),
                'elem': re.compile(r'TS 0 (?P<elem>.*)\n'),
                'data': re.compile(r'(?P<data>\d+) ')
            }
        else:
            self.rx_dict = {
                'seed': re.compile(r'SEED: (?P<seed>\d+)\n'),
                'elem': re.compile(r'TS 0 (?P<elem>.*)\n'),
                'data': re.compile(r'(?P<ts>\d+) (?P<count>\d.+) (?P<quorum>True|False)\n')
            }

        if typeOfGraph == "BOX PLOT":
            config = configparser.ConfigParser()
            config.readfp(open(self.configurationFilePath))
            self.nSteps = config.getint("experiment", "numberOfSteps")

        elif typeOfGraph == "POP EVO":
            config = configparser.ConfigParser()
            config.readfp(open(self.configurationFilePath))
            self.colors = config.get("experiment", "colors")
            self.colors = self.fromStringToArray(self.colors)
            self.quorum = config.getfloat("experiment", "quorum")
            self.nAgent = config.getint("experiment", "numberOfAgents")
            self.simNum = config.getint("experiment", "numberOfSimulation")
            self.qualities = json.loads(config["model_update"]["qualityValues"])


    def parseLine(self, line):
        for key, rx in self.rx_dict.items():
            match = rx.search(line)
            if match:
                return key, match
        return None, None

    def parseFile(self):
        parsedData = {
        "seeds": [],
        "opinions": [],
        "timeSteps": [],
        "opinionsCardinalities": [],
        "quorums": []
        }
        with open(self.filepath, 'r') as file_object:
            line = file_object.readline()
            while line:
                key, match = self.parseLine(line)
                if key == 'seed':
                    seed = match.group('seed')
                    parsedData["seeds"].append(seed)
                if key == 'elem':
                    elems = match.group('elem')
                    opinionNumber = len(elems) - elems.count(' ') - elems.count('Q')
                    parsedData["opinions"].append(opinionNumber)

                if key == 'data':
                    if self.typeOfGraph == "BOX PLOT":
                        value = match.group('data')
                        value = int(value)
                        parsedData["opinionsCardinalities"].append(value)
                    elif self.typeOfGraph == "POP EVO" or self.typeOfGraph == "ACCURACY":
                        ts = match.group('ts')
                        ts = int(ts)
                        parsedData["timeSteps"].append(ts)
                        values = match.group('count')
                        values = [int(i) for i in values.split()]
                        parsedData["opinionsCardinalities"].append(values)
                        quorum = match.group('quorum')
                        parsedData["quorums"].append(quorum)
                line = file_object.readline()
        return parsedData

    def boxPlotTime(self, collectedData):
        fig = plt.figure()
        ax = plt.axes(xlim = (0, 6), ylim = (0, self.nSteps))
        ax.set(xlabel='opinions', ylabel='steps', title='BoxPlot analysis')
        max = np.amax(collectedData["opinions"])
        min = np.amin(collectedData["opinions"])
        dataOpinion = []
        opinionPosition = []
        for n in range(min, max + 1):
            tmp_data = []
            for m, elem in enumerate(collectedData["opinions"]):
                if elem == n:
                    tmp_data.append(collectedData["opinionsCardinalities"][m])
            dataOpinion.append(tmp_data)
            opinionPosition.append(n)
        ax.boxplot(dataOpinion, positions = opinionPosition)
        ax.grid()
        plt.show()

    def popEvoPlot(self, collectedData):
        opinions = collectedData["opinions"][0]
        maxTs = np.amax(collectedData["timeSteps"])
        fig = plt.figure()
        ax = plt.axes(xlim = (0, maxTs + 10), ylim = (0, 100))
        ax.set(xlabel = 'time steps', ylabel = '', title = 'Population evo')
        ax.spines['top'].set_visible(False)
        ax.spines['bottom'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['left'].set_visible(False)
        ax.axhline(y = self.quorum * 100, color = 'red', alpha = 0.4)
        ax.get_xaxis().tick_bottom()
        ax.get_yaxis().tick_left()
        ax.xaxis.set_major_formatter(plt.FuncFormatter('{:.0f}'.format))
        ax.yaxis.set_major_formatter(plt.FuncFormatter('{:.0f}%'.format))
        plt.grid(True, 'major', 'y', ls='--', lw=.5, c='k', alpha=.3)
        plt.tick_params(axis='both', which='both', bottom=False, top=False,
                labelbottom=True, left=False, right=False, labelleft=True)
        agentColors = [color for pos, color in enumerate(self.colors) if pos <= opinions]
        for rank, color in enumerate(agentColors):
            agentData = []
            for x in collectedData["opinionsCardinalities"]:
                agentData.append(x[rank]/self.nAgent * 100)
            if rank != 1 and rank != 0:
                line = plt.plot(collectedData["timeSteps"], agentData, lw=1.5,
                color=color, label = min(self.qualities))
            if rank == 1:
                line = plt.plot(collectedData["timeSteps"], agentData, lw=1.5,
                color=color, label = max(self.qualities))
            else:
                line = plt.plot(collectedData["timeSteps"], agentData, lw=1.5,
                color=color)

        plt.legend(title="Quality", bbox_to_anchor=(1, 1), loc='upper left', borderaxespad=0.3)
        plt.show()

    def fromStringToArray(self, string):
        #remove the [ ]
        string = string.replace("[", "")
        string = string.replace("]", "")
        #remove spaces " "
        string = string.replace(" ", "")
        #create the array from the string
        array = string.split(",")

        return array
