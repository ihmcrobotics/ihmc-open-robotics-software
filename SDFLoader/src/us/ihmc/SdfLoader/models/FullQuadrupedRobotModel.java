package us.ihmc.SdfLoader.models;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;
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

   /**
    * This methods returns the frame located right after the parent joint of this end-effector.
    */
   public abstract ReferenceFrame getEndEffectorFrame(RobotQuadrant robotQuadrant);

   /**
    * Returns the sole reference frame.
    * A sole frame is attached to a foot and is generally used to put the foot contact points.
    * Its origin is right in the middle of the bottom of the foot.
    * @param robotSide
    * @return
    */
   public abstract ReferenceFrame getSoleFrame(RobotQuadrant robotQuadrant);
   
   
}