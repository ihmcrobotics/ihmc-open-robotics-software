package us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotModels.FullHumanoidRobotModel;

public class ConstrainedWholeBodyPlanningRequestPacket extends Packet<ConstrainedWholeBodyPlanningRequestPacket>
{
   public int numberOfExpanding;

   public int numberOfFindInitialGuess;

   public RobotKinematicsConfiguration initialConfiguration;

   /*
    * ConstrainedEndEffectorTrajectory cannot be imported in
    * IHMCHumanoidRobotics.
    */
   //public ConstrainedEndEffectorTrajectory constrainedEndEffectorTrajectory;

   public ConstrainedWholeBodyPlanningRequestPacket()
   {

   }

   public void setInitialRobotConfigration(FullHumanoidRobotModel fullRobotModel)
   {
      initialConfiguration = new RobotKinematicsConfiguration(fullRobotModel);
   }

   public void setNumberOfExpanding(int value)
   {
      this.numberOfExpanding = value;
   }

   public void setNumberOfFindInitialGuess(int value)
   {
      this.numberOfFindInitialGuess = value;
   }

   @Override
   public boolean epsilonEquals(ConstrainedWholeBodyPlanningRequestPacket other, double epsilon)
   {
      return true;
   }

}
