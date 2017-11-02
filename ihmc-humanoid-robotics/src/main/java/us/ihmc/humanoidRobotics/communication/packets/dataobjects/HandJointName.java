package us.ihmc.humanoidRobotics.communication.packets.dataobjects;

import us.ihmc.robotics.partNames.FingerName;
import us.ihmc.robotics.robotSide.RobotSide;

public interface HandJointName
{
   /**
    * Retrieves the name of the finger to which this joint belongs.
    * 
    * @param robotSide refers to which hand this joint belongs.
    * @return the finger name.
    */
   public FingerName getFinger(RobotSide robotSide);

   /**
    * Retrieves the position index of this joint on the finger.
    * <p>
    * For instance, if the finger to which this joint belongs has 3 knuckles, the returned index
    * should be contained in [0,3[.
    * </p>
    * 
    * @return index of the joint on the finger.
    */
   public int getHandJointAngleIndex();

   /**
    * Get the {@code HandJointName} for all the finger joints for one hand.
    * 
    * @return the array containing all the joint names.
    */
   public HandJointName[] getValues();

   /**
    * Retrieves the joint name as a {@code String} that corresponds to the joint name as defined in
    * the robot definition, i.e. the SDF/URDF model or the {@code RobotDescription}.
    * 
    * @param robotSide refers to which hand this joint belongs.
    * @return the {@code String} of this joint name.
    */
   public String getJointName(RobotSide robotSide);
}
