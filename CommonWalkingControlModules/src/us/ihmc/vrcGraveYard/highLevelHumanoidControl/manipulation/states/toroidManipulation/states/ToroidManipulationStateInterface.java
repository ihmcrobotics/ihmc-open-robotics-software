package us.ihmc.vrcGraveYard.highLevelHumanoidControl.manipulation.states.toroidManipulation.states;

import com.yobotics.simulationconstructionset.util.statemachines.State;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Wrench;

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
  