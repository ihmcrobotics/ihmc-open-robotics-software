package us.ihmc.robotModels;

import us.ihmc.robotics.robotSide.RobotSegment;

public interface FullLeggedRobotModelFactory<E extends Enum<E> & RobotSegment<E>> extends FullRobotModelFactory
{
   @Override
   default FullLeggedRobotModel<E> createFullRobotModel()
   {
      return createFullRobotModel(true);
   }

   @Override
   FullLeggedRobotModel<E> createFullRobotModel(boolean enforceUniqueReferenceFrames);
}
