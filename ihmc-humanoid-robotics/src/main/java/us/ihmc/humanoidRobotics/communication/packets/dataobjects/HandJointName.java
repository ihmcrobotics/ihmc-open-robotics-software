package us.ihmc.humanoidRobotics.communication.packets.dataobjects;

import us.ihmc.robotics.robotSide.RobotSide;

public interface HandJointName
{
   /**
    * Provides a side-dependent index for this joint that contained in [0, N[, where N is the number
    * of hand joints.
    * 
    * @param robotSide refers to which hand this joint belongs.
    * @return the index associated with this joint.
    */
   public int getIndex(RobotSide robotSide);

   /**
    * Retrieves the joint name as a {@code String} that corresponds to the joint name as defined in
    * the robot definition, i.e. the SDF/URDF model or the {@code RobotDescription}.
    * 
    * @param robotSide refers to which hand this joint belongs.
    * @return the {@code String} of this joint name.
    */
   public String getJointName(RobotSide robotSide);
}
