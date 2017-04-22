package us.ihmc.humanoidBehaviors.behaviors.wholebodyValidityTester;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.manipulation.planning.robotcollisionmodel.CollisionModelBox;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelCleaningPose;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelPath;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class SolarPanelPoseValidityTester extends WholeBodyPoseValidityTester
{
   private SolarPanel solarPanel;
   
   public SolarPanelPoseValidityTester(FullHumanoidRobotModelFactory fullRobotModelFactory, CommunicationBridgeInterface outgoingCommunicationBridge, 
                                       FullHumanoidRobotModel fullRobotModel)
   {
      super(fullRobotModelFactory, outgoingCommunicationBridge, fullRobotModel);
   }
   
   public void setSolarPanel(SolarPanel solarPanel)
   {
      this.solarPanel = solarPanel;
      addEnvironmentCollisionModel();  
   }
   
   public void setWholeBodyPose(Pose desiredHandPose, double pelvisYaw, double pelvisHeight)
   {
      // Hand
      FramePoint desiredHandFramePoint = new FramePoint(ReferenceFrame.getWorldFrame(), desiredHandPose.getPoint());
      FrameOrientation desiredHandFrameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), desiredHandPose.getOrientation());
      
      FramePose desiredHandFramePose = new FramePose(desiredHandFramePoint, desiredHandFrameOrientation);
      
      this.setDesiredHandPose(RobotSide.RIGHT, desiredHandFramePose);
      
      // Pelvis Orientation
      this.holdCurrentPelvisOrientation();
      
      // Pelvis Height
      this.setDesiredPelvisHeight(pelvisHeight);
      
      // Chest Orientation
      Quaternion desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendYawRotation(pelvisYaw);
      FrameOrientation desiredChestFrameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), desiredChestOrientation);
      this.setDesiredChestOrientation(desiredChestFrameOrientation);
      
      isGoodIKSolution = getIKResult();
   }
   
   public void setWholeBodyPose(SolarPanelPath cleaningPath, double time, double pelvisYaw)
   {
      SolarPanelCleaningPose cleaningPose = cleaningPath.getCleaningPose(time);
            
      Pose aPose = new Pose(cleaningPose.getDesiredHandPosition(), cleaningPose.getDesiredHandOrientation());
      //PrintTools.info("IN "+cleaningPose.getDesiredHandPosition().getX()+" "+cleaningPose.getDesiredHandPosition().getY()+" "+cleaningPose.getDesiredHandPosition().getZ());
      setWholeBodyPose(aPose, pelvisYaw);
   }
   
   public void setWholeBodyPose(Pose desiredHandPose, double pelvisYaw)
   {
      // Hand
      FramePoint desiredHandFramePoint = new FramePoint(ReferenceFrame.getWorldFrame(), desiredHandPose.getPoint());
      FrameOrientation desiredHandFrameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), desiredHandPose.getOrientation());
      
      FramePose desiredHandFramePose = new FramePose(desiredHandFramePoint, desiredHandFrameOrientation);
      
      this.setDesiredHandPose(RobotSide.RIGHT, desiredHandFramePose);
      
      // Chest Orientation
      Quaternion desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendYawRotation(pelvisYaw);
      FrameOrientation desiredChestFrameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), desiredChestOrientation);
      this.setDesiredChestOrientation(desiredChestFrameOrientation);
      
      // Pelvis Orientation
      this.holdCurrentPelvisOrientation();
      
      // Pelvis Height
      this.holdCurrentPelvisHeight();
      
      isGoodIKSolution = getIKResult();
      cnt++;
   }

   public static int cnt = 0;
   
   @Override
   public void addEnvironmentCollisionModel()
   {  
      CollisionModelBox solarPanelCollisionModel;
      solarPanelCollisionModel = new CollisionModelBox(getRobotCollisionModel().getCollisionShapeFactory(), solarPanel.getRigidBodyTransform(),
                                                       solarPanel.getSizeX(), solarPanel.getSizeY(), solarPanel.getSizeZ());
      solarPanelCollisionModel.getCollisionShape().setCollisionGroup(0b11111111111111);
      solarPanelCollisionModel.getCollisionShape().setCollisionMask(0b11111111111111);      
   }
   

}
