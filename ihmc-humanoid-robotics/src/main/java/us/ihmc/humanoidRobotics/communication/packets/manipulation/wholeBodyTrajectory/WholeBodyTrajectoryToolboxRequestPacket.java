package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.Packet;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;

public class WholeBodyTrajectoryToolboxRequestPacket extends Packet<WholeBodyTrajectoryToolboxRequestPacket>
{
   public int numberOfExpanding;

   public int numberOfFindInitialGuess;

   public int numerOfEndEffectorWayPoints;

   public KinematicsToolboxOutputStatus initialConfiguration;

   /*
    * ConstrainedEndEffectorTrajectory cannot be imported in IHMCHumanoidRobotics.
    */
   // public ConstrainedEndEffectorTrajectory constrainedEndEffectorTrajectory;

   public WholeBodyTrajectoryToolboxRequestPacket()
   {

   }

   public void setInitialRobotConfigration(FullHumanoidRobotModel fullRobotModel)
   {
      this.initialConfiguration = new KinematicsToolboxOutputStatus(fullRobotModel.getRootJoint(),
                                                                    FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel), false);
   }

   public void setNumberOfExpanding(int value)
   {
      this.numberOfExpanding = value;
   }

   public void setNumberOfFindInitialGuess(int value)
   {
      this.numberOfFindInitialGuess = value;
   }

   public void setNumberOfEndEffectorWayPoints(int value)
   {
      this.numerOfEndEffectorWayPoints = value;
   }

   @Override
   public boolean epsilonEquals(WholeBodyTrajectoryToolboxRequestPacket other, double epsilon)
   {
      return true;
   }
}