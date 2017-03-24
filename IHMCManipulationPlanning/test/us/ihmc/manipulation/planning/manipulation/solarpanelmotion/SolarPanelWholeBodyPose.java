package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;

public class SolarPanelWholeBodyPose
{
   private SolarPanelCleaningPose cleaningPose;
   private double desiredPelvisYaw;
   private double deisredPelvisHeight;
   
   public SolarPanelWholeBodyPose(SolarPanelCleaningPose cleaningPose, double pelvisYaw, double pelvisHeight)
   {
      this.cleaningPose = cleaningPose;      
      this.desiredPelvisYaw = pelvisYaw;
      this.deisredPelvisHeight = pelvisHeight;
   }
   
   public void getWholeBodyTrajectoryMessage(WholeBodyTrajectoryMessage wholeBodyMessage)
   {
      
   }
}
