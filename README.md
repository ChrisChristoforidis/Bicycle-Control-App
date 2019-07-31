# Bicycle-Control-App

This app is used for simulating and animating the balance behaviour of a bicycle rider system. For the EOM (equations of motion) of the bicycle the linearized equations of motion based on the Whipple model derived by Meijjard et al. are used. The plant is extended by accounting for neuromusclular dynamics. The gains of the controller were estimated by trying to fit the ouput of a simple paramtric model to the bicycle riding dataset recently acquired through experiemnts with humans in TU Delft. K1, K2, K3, K4, K5  are the gains correcting for the roll rate, steer rate, roll angle, steer angle and yaw respectively. The basic bicycle EOM are extended with yaw to account for heading corrections that were present during the experiments.The experiments were done in four distinct speed levels (9km/h 13km/h 16km/h and 20km/h). The first three are located in the stable region of the bicycle while the last gives some insight into how humans control the bike when it is already self-stable. 

The user has the ability to mess around with lateral pull disturbances (magnitude and duration) and see how the rider would react. Disturbance options include white noise and impulse. The app also has the ability to adapt in online changes of bicycle design parameters such as its wheelbase (distance between wheel centers) and wheel radius. 

In order to build and run the app. Simply run the simulation_exported.m file in the MATLAB programming environement. No additional toolboxes are required. The plan is to create a standalone executable that works without a MATLAB license.

![Bike Simulation](https://i.imgur.com/VOr6l0M.gifv)
