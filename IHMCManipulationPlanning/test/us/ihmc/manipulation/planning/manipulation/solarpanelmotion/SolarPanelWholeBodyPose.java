package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

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
   
   public void getWholeBodyTrajectoryMessage(WholeBodyTrajectoryMessage wholeBodyMessage, double motionTime)
   {
      //HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, motionTime, cleaningPose.getDesiredHandPosition(), cleaningPose.getDesiredHandOrientation(), ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
      HandTrajectoryMessage handTrajectoryMessage = cleaningPose.getHandTrajectoryMessage(motionTime);
      
      //PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(motionTime, deisredPelvisHeight);
      
      Quaternion desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendYawRotation(desiredPelvisYaw);      
      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(motionTime, desiredChestOrientation, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
      
      wholeBodyMessage.setHandTrajectoryMessage(handTrajectoryMessage);
      wholeBodyMessage.setChestTrajectoryMessage(chestTrajectoryMessage);
      //wholeBodyMessage.setPelvisTrajectoryMessage(pelvisHeightTrajectoryMessage);      
   }
}
