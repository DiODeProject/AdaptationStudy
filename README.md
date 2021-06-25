# AdaptationStudy
This repository contains the codes used in the study: "When less is more: robot swarms adapt better to changes with constrained communication" by Mohamed S. Talamali, Arindam Saha, James A. R. Marshall, and Andreagiovanni Reina (the University of Sheffield).

The code is organised as follows:

* **ARK_Codes:** code to run the real robot experiments with [ARK][ARK_Repo] (the system for Augmented Reality for Kilobots)
* **ARGoS_simulation:** code to run the [ARGoS][ARGoS_Repo] physics-based simulations of the [Kilobot][KilobotPlugin_Repo] robot.
* **Kilobot_Codes:** control code of the Kilobots (same code for simulated and real robots)
* **MultiAgent_simulation:** code of the [DeMaMAS][DeMaMAS_Repo] multi-agent simulations (it includes the simulator and configuration files).
* **ODE_Analysis:** python jupyter notebook to reproduce the stability and bifurcation analysis that we have done on the ODE model that describes the adaptation process.


[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)


   [ARGoS_Repo]: <https://github.com/mstalamali/argos3>
   [KilobotPlugin_Repo]: <https://github.com/mstalamali/argos3-kilobot>
   [ARK_Repo]: <https://github.com/DiODeProject/KilobotArena>
   [DeMaMAS_Repo]: <https://github.com/DiODeProject/DeMaMAS>
   