package us.ihmc.humanoidBehaviors.behaviors.solarPanel;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.rrtPlanner.SolarPanelCleaningInfo;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelCleaningPose;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelPath;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public class SolarPanelWholeBodyTrajectoryMessageFacotry
{
   private WholeBodyTrajectoryMessage wholebodyTrajectoryMessage;
   private SolarPanelPath cleaningPath;
   
   private ReferenceFrame midFeetFrame;
   
   public SolarPanelWholeBodyTrajectoryMessageFacotry(FullHumanoidRobotModel fullRobotModel)
   {
      this.wholebodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
            
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      referenceFrames.updateFrames();
      midFeetFrame = referenceFrames.getMidFootZUpGroundFrame();
   }
   
   private void clearWholeBodyTrajectoryMessage()
   {
      this.wholebodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
   }
   
   public void setCleaningPath(SolarPanelPath cleaningPath)
   {
      this.cleaningPath = cleaningPath;
   }
   
   public WholeBodyTrajectoryMessage getWholeBodyTrajectoryMessage()
   {
      return wholebodyTrajectoryMessage;
   }
   
   public void setMessage(SolarPanelCleaningPose cleaningPose, double pelvisYaw, double pelvisHeight, double motionTime)
   {
      clearWholeBodyTrajectoryMessage();
            
      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, motionTime, cleaningPose.getDesiredHandPosition(), cleaningPose.getDesiredHandOrientation(), midFeetFrame);      
      
      Quaternion desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendYawRotation(pelvisYaw);      
      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(motionTime, desiredChestOrientation, midFeetFrame);
      
      wholebodyTrajectoryMessage.setHandTrajectoryMessage(handTrajectoryMessage);
      wholebodyTrajectoryMessage.setChestTrajectoryMessage(chestTrajectoryMessage);
   }
   
   public void setMessage(SolarPanelCleaningPose cleaningPose, double pelvisHeight, double chestYaw, double chestPitch, double motionTime)
   {
      clearWholeBodyTrajectoryMessage();
            
      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, motionTime, cleaningPose.getDesiredHandPosition(), cleaningPose.getDesiredHandOrientation(), midFeetFrame);      
      
      Quaternion desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendYawRotation(chestYaw);      
      desiredChestOrientation.appendPitchRotation(chestPitch);      
      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(motionTime, desiredChestOrientation, midFeetFrame);
      
      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage(motionTime, new Point3D(0, 0, pelvisHeight), new Quaternion());
      SelectionMatrix6D pelvisTrajectorySelectionMatrix = new SelectionMatrix6D();
      pelvisTrajectorySelectionMatrix.clearSelection();
      pelvisTrajectorySelectionMatrix.selectLinearZ(true);
      pelvisTrajectoryMessage.setSelectionMatrix(pelvisTrajectorySelectionMatrix);
      
      wholebodyTrajectoryMessage.setHandTrajectoryMessage(handTrajectoryMessage);
      wholebodyTrajectoryMessage.setChestTrajectoryMessage(chestTrajectoryMessage);
      wholebodyTrajectoryMessage.setPelvisTrajectoryMessage(pelvisTrajectoryMessage);
   }
   
   public void setMessage(ArrayList<RRTNode> rrtPath)
   {
      switch(SolarPanelCleaningInfo.getDegreesOfRedundancy())
      {
         case ONE:
         {
            setMessage1D(rrtPath);
            break;
         }
         case THREE:
         {
            PrintTools.info("set three ");
            setMessage3D(rrtPath);
            break;
         }
      }
   }
   
   public void setMessage1D(ArrayList<RRTNode> rrtPath)
   {
      if(cleaningPath == null)
      {
         PrintTools.warn("You must set cleaningPath");
      }
      else
      {
         clearWholeBodyTrajectoryMessage();
         int numberOfWayPoints = rrtPath.size()-1;
         
         PrintTools.info("numberOfWayPoints "+numberOfWayPoints);
         
         HandTrajectoryMessage handTrajectoryMessage;
         ChestTrajectoryMessage chestTrajectoryMessage;
         
         handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, numberOfWayPoints);
         chestTrajectoryMessage = new ChestTrajectoryMessage(numberOfWayPoints);
         
         handTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(midFeetFrame);
         handTrajectoryMessage.getFrameInformation().setDataReferenceFrame(midFeetFrame);
         
         chestTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(midFeetFrame);
         chestTrajectoryMessage.getFrameInformation().setDataReferenceFrame(midFeetFrame);
         
         for(int i=1;i<rrtPath.size();i++)
         {
            double time = rrtPath.get(i).getNodeData(0);
            SolarPanelCleaningPose cleaningPose = cleaningPath.getCleaningPose(time);
            
            handTrajectoryMessage.setTrajectoryPoint(i-1, time, cleaningPose.getDesiredHandPosition(), cleaningPose.getDesiredHandOrientation(), new Vector3D(), new Vector3D(), midFeetFrame);            
            
            Quaternion desiredChestOrientation = new Quaternion();
            desiredChestOrientation.appendYawRotation(rrtPath.get(i).getNodeData(1));
            chestTrajectoryMessage.setTrajectoryPoint(i-1, time, desiredChestOrientation, new Vector3D(), midFeetFrame);
         }
         
         wholebodyTrajectoryMessage.setHandTrajectoryMessage(handTrajectoryMessage);
         wholebodyTrajectoryMessage.setChestTrajectoryMessage(chestTrajectoryMessage);     
      }
   }
   
   public void setMessage3D(ArrayList<RRTNode> rrtPath)
   {
      if(cleaningPath == null)
      {
         PrintTools.warn("You must set cleaningPath");
      }
      else
      {
         clearWholeBodyTrajectoryMessage();
         int numberOfWayPoints = rrtPath.size()-1;
         
         PrintTools.info("numberOfWayPoints "+numberOfWayPoints);
         
         HandTrajectoryMessage handTrajectoryMessage;
         ChestTrajectoryMessage chestTrajectoryMessage;
         PelvisTrajectoryMessage pelvisTrajectoryMessage;
         
         handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, numberOfWayPoints);
         chestTrajectoryMessage = new ChestTrajectoryMessage(numberOfWayPoints);
         pelvisTrajectoryMessage = new PelvisTrajectoryMessage(numberOfWayPoints);
         
         handTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(midFeetFrame);
         handTrajectoryMessage.getFrameInformation().setDataReferenceFrame(midFeetFrame);
         
         chestTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(midFeetFrame);
         chestTrajectoryMessage.getFrameInformation().setDataReferenceFrame(midFeetFrame);
         
         pelvisTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(midFeetFrame);
         pelvisTrajectoryMessage.getFrameInformation().setDataReferenceFrame(midFeetFrame);
         
         SelectionMatrix6D pelvisTrajectorySelectionMatrix = new SelectionMatrix6D();
         pelvisTrajectorySelectionMatrix.clearSelection();
         pelvisTrajectorySelectionMatrix.selectLinearZ(true);
         pelvisTrajectoryMessage.setSelectionMatrix(pelvisTrajectorySelectionMatrix);
         
         for(int i=1;i<rrtPath.size();i++)
         {
            double time = rrtPath.get(i).getNodeData(0);
            SolarPanelCleaningPose cleaningPose = cleaningPath.getCleaningPose(time);
            
            handTrajectoryMessage.setTrajectoryPoint(i-1, time, cleaningPose.getDesiredHandPosition(), cleaningPose.getDesiredHandOrientation(), new Vector3D(), new Vector3D(), midFeetFrame);            
            
            Quaternion desiredChestOrientation = new Quaternion();
            desiredChestOrientation.appendYawRotation(rrtPath.get(i).getNodeData(2));
            desiredChestOrientation.appendPitchRotation(rrtPath.get(i).getNodeData(3));
            chestTrajectoryMessage.setTrajectoryPoint(i-1, time, desiredChestOrientation, new Vector3D(), midFeetFrame);
            
            pelvisTrajectoryMessage.setTrajectoryPoint(i-1, time, new Point3D(0,0, rrtPath.get(i).getNodeData(1)), new Quaternion(), new Vector3D(), new Vector3D(), midFeetFrame);
         }
         
         wholebodyTrajectoryMessage.setHandTrajectoryMessage(handTrajectoryMessage);
         wholebodyTrajectoryMessage.setChestTrajectoryMessage(chestTrajectoryMessage);
         wholebodyTrajectoryMessage.setPelvisTrajectoryMessage(pelvisTrajectoryMessage);
      }
   }
}
