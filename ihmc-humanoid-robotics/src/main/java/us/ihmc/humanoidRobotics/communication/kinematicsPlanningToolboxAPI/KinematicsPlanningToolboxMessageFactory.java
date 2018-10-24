package us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI;

import controller_msgs.msg.dds.KinematicsPlanningToolboxRigidBodyMessage;

public class KinematicsPlanningToolboxMessageFactory
{
   public static final double DEFAULT_POSITION_DISPLACEMENT = 0.01;
   public static final double DEFAULT_ORIENTATION_DISPLACEMENT = 0.01;

   public static void setDefaultAllowableDisplacement(KinematicsPlanningToolboxRigidBodyMessage message, int numberOfKeyFrames)
   {
      for (int i = 0; i < numberOfKeyFrames; i++)
      {
         message.getAllowablePositionDisplacement().add(DEFAULT_POSITION_DISPLACEMENT);
         message.getAllowableOrientationDisplacement().add(DEFAULT_ORIENTATION_DISPLACEMENT);
      }
   }
}
