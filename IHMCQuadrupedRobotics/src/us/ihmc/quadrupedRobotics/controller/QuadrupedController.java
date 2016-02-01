package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.quadrupedRobotics.controller.state.QuadrupedControllerState;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;

public abstract class QuadrupedController extends State<QuadrupedControllerState>
{
   public QuadrupedController(QuadrupedControllerState stateEnum)
   {
      super(stateEnum);
   }
   
   public abstract RobotMotionStatus getMotionStatus();
}
