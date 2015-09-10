package us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation.states.toroidManipulation.states;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.stateMachines.State;


public abstract class ToroidManipulationStateInterface<T extends Enum<T>> extends State<T>
{
   public ToroidManipulationStateInterface(T stateEnum)
   {
      super(stateEnum);
   }

   public abstract SpatialAccelerationVector getDesiredHandAcceleration(RobotSide robotSide);

   public abstract Wrench getHandExternalWrench(RobotSide robotSide);

   public abstract boolean isDone();
}
  