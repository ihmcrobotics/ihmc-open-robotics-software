package us.ihmc.robotModels;

import us.ihmc.robotics.robotSide.RobotSegment;

public interface FullLeggedRobotModelFactory<E extends Enum<E> & RobotSegment<E>> extends FullRobotModelFactory
{
   @Override
   FullLeggedRobotModel<E> createFullRobotModel();
}
