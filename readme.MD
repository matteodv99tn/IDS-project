# Run the code

Performing simulation is computationally expensive.
It is suggested forstly to run `main_test` in order to generate the data that then can be loaded and plotted using the function `plot_results`. At the end a folder of data should be generated as follows:

```
Results/
├── test_1a/
│   ├── simulation_001.log
│   ├── simulation_001.mat
│   ├── simulation_002.log
│   ├── simulation_002.mat
│   └── ...
├── test_1b/
│   ├── simulation_001.log
│   ├── simulation_001.mat
│   └── ...
└── ...
```

Now to use the `plot_results` function to see the results of a given simulation:
```
plot_results('Results/test_1a/simulation_001.mat', true, false);
```
where `true` enables the dynamic plot of the moving robots while `false` is to avoid the automatic exporting of the associated figures.




# Bibliography and other useful references:

- [_Cooperative manipulation_](http://www.prisma.unina.it/courses/SHB_39.pdf): interesting book extract containing useful informations in order to work multiple robots;
- [_Control and Estimation for Cooperative Manipulator Tasks_](http://dspace.mit.edu/handle/1721.1/31217): technical report for a project whee they discuss the problem of handing a object from one robot to another;
- [_Real-Time Kinematic Control for Redundant Manipulators in a Time-Varying Environment: Multiple-Dynamic Obstacle Avoidance and Fast Tracking of a Movign Object_](https://ieeexplore.ieee.org/document/8716565): here they use principal component analysis to adapt the tracking capability of the system (the paper studies a mobile robot on top of which multiple manipulators are mounted);
