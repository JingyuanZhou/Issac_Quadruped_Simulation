### Quadruped Simulation Using Issac

```
- Quadruped,

- RobotRunner ->

  - LegController,

  - StateEstimator,

  - ControlFSM ->

    - FSM State RecoveryStand,
    
    - FSM State FrontJump->

        1. FrontJumpCtrl,

        2. DataReader,

      	3. DataReaderCtrl
        
    - FSM State Backflip->

      	1. BackFlipCtrl,

      	2. DataReader,

      	3. DataReaderCtrl

    - FSM State Locomotion ->

      - ConvexMPCLocomotion->

        	1. FootSwingTrajectory,

        	2. Gait,

        	3. MPC Solver in C
```
