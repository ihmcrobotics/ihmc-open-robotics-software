package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.humanoidRobotics.communication.packets.AbstractJointspaceTrajectoryMessage;

public class SpineTrajectoryMessage extends AbstractJointspaceTrajectoryMessage<SpineTrajectoryMessage>
{
   public SpineTrajectoryMessage()
   {
      super();
   }

   public SpineTrajectoryMessage(int numberOfJoints, int waypoints)
   {
      super(numberOfJoints, waypoints);
   }

   public SpineTrajectoryMessage(int numberOfJoints)
   {
      super(numberOfJoints);
   }

   public SpineTrajectoryMessage(double trajectoryTime, double[] jointDesireds)
   {
      super(trajectoryTime, jointDesireds);
   }
}
