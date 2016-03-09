package us.ihmc.SdfLoader.models;

import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.RigidBody;

public interface FullQuadrupedRobotModel extends FullRobotModel
{

   /**
    * Returns the {@link RigidBody} describing the foot of the corresponding of quadrant from this robot.
    * A foot is considered as the end-effector (thus the last {@RigidBody}) of the leg to which it belongs to.
    * 
    * @param RobotQuadrant Refers to which quadrant the foot belongs to
    */
   public abstract RigidBody getFoot(RobotQuadrant robotQuadrant);

}