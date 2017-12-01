package us.ihmc.humanoidBehaviors.behaviors.wholebodyValidityTester;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.robotcollisionmodel.CollisionModelBox;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelCleaningPose;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelPath;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.robotSide.RobotSide;

public class SolarPanelPoseValidityTester extends WholeBodyPoseValidityTester
{
   private SolarPanel solarPanel;   
   
   public SolarPanelPoseValidityTester(FullHumanoidRobotModelFactory fullRobotModelFactory, CommunicationBridgeInterface outgoingCommunicationBridge, 
                                       FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames)
   {
      super(fullRobotModelFactory, outgoingCommunicationBridge, fullRobotModel, referenceFrames);
      
   }
   
   public void setSolarPanel(SolarPanel solarPanel)
   {
      this.solarPanel = solarPanel;
      addEnvironmentCollisionModel();  
   }   

   public void setWholeBodyPose(SolarPanelPath cleaningPath, RRTNode node)
   {
      referenceFrames.updateFrames();
      midFeetFrame = referenceFrames.getMidFootZUpGroundFrame();
      
      SolarPanelCleaningPose cleaningPose = cleaningPath.getCleaningPose(node.getNodeData(0));
      
      Pose3D desiredHandPose = new Pose3D(cleaningPose.getDesiredHandPosition(), cleaningPose.getDesiredHandOrientation());
      
      if(node.getDimensionOfNodeData() == 2)
      {
         double chestYaw = node.getNodeData(1);
         setWholeBodyPose(desiredHandPose, chestYaw);
      }
      else if(node.getDimensionOfNodeData() == 4)
      {
         double pelvisHeight = node.getNodeData(1);
         double chestYaw = node.getNodeData(2);
         double chestPitch = node.getNodeData(3);
         
         setWholeBodyPose(desiredHandPose, pelvisHeight, chestYaw, chestPitch);
      }      
   }
   
   public void setWholeBodyPose(SolarPanelPath cleaningPath, double time, double pelvisYaw)
   {
      SolarPanelCleaningPose cleaningPose = cleaningPath.getCleaningPose(time);
            
      Pose3D aPose = new Pose3D(cleaningPose.getDesiredHandPosition(), cleaningPose.getDesiredHandOrientation());
      setWholeBodyPose(aPose, pelvisYaw);
   }
   
   public void setWholeBodyPose(Pose3D desiredHandPose, double chestYaw)
   {
      referenceFrames.updateFrames();
      midFeetFrame = referenceFrames.getMidFootZUpGroundFrame();
      
      // Hand
      FramePoint3D desiredHandFramePoint = new FramePoint3D(midFeetFrame, desiredHandPose.getPosition());
      FrameQuaternion desiredHandFrameOrientation = new FrameQuaternion(midFeetFrame, desiredHandPose.getOrientation());
      
      FramePose desiredHandFramePose = new FramePose(desiredHandFramePoint, desiredHandFrameOrientation);
      
      this.setDesiredHandPose(RobotSide.RIGHT, desiredHandFramePose);
      
      // Chest Orientation
      Quaternion desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendYawRotation(chestYaw);
      FrameQuaternion desiredChestFrameOrientation = new FrameQuaternion(midFeetFrame, desiredChestOrientation);
      this.setDesiredChestOrientation(desiredChestFrameOrientation);
      
      // Pelvis Orientation
      this.holdCurrentPelvisOrientation();
      
      // Pelvis Height
      this.holdCurrentPelvisHeight();
   }

   public void setWholeBodyPose(Pose3D desiredHandPose, double pelvisHeight, double chestYaw, double chestPitch)
   {
      referenceFrames.updateFrames();
      midFeetFrame = referenceFrames.getMidFootZUpGroundFrame();
      
      // Hand
      FramePoint3D desiredHandFramePoint = new FramePoint3D(midFeetFrame, desiredHandPose.getPosition());
      FrameQuaternion desiredHandFrameOrientation = new FrameQuaternion(midFeetFrame, desiredHandPose.getOrientation());
      
      FramePose desiredHandFramePose = new FramePose(desiredHandFramePoint, desiredHandFrameOrientation);
      
      this.setDesiredHandPose(RobotSide.RIGHT, desiredHandFramePose);
      
      // Chest Orientation
      Quaternion desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendYawRotation(chestYaw);
      desiredChestOrientation.appendPitchRotation(chestPitch);
      FrameQuaternion desiredChestFrameOrientation = new FrameQuaternion(midFeetFrame, desiredChestOrientation);
      this.setDesiredChestOrientation(desiredChestFrameOrientation);
      
      // Pelvis Orientation
      this.holdCurrentPelvisOrientation();
      
      // Pelvis Height
      this.setDesiredPelvisHeight(pelvisHeight);
   }
   
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
