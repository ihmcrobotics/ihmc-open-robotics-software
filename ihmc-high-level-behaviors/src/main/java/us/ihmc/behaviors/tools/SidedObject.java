package us.ihmc.behaviors.tools;

import us.ihmc.commons.robotics.robotSide.RobotSide;

/**
 * An interface for anything that has a {@link RobotSide}.
 */
public interface SidedObject
{
   RobotSide getSide();
}
