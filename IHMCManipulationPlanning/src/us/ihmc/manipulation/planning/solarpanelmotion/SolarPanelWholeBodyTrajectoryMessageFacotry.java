package us.ihmc.manipulation.planning.solarpanelmotion;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class SolarPanelWholeBodyTrajectoryMessageFacotry
{
   private WholeBodyTrajectoryMessage wholebodyTrajectoryMessage;
   private SolarPanelPath cleaningPath;
   
   public SolarPanelWholeBodyTrajectoryMessageFacotry()
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
      this.wholebodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
      //HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, motionTime, cleaningPose.getDesiredHandPosition(), cleaningPose.getDesiredHandOrientation(), ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
      HandTrajectoryMessage handTrajectoryMessage = cleaningPose.getHandTrajectoryMessage(motionTime);
      
      //PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(motionTime, pelvisHeight);
      
      Quaternion desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendYawRotation(pelvisYaw);      
      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(motionTime, desiredChestOrientation, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
      
      wholebodyTrajectoryMessage.setHandTrajectoryMessage(handTrajectoryMessage);
      wholebodyTrajectoryMessage.setChestTrajectoryMessage(chestTrajectoryMessage);    
      //wholebodyTrajectoryMessage.setPelvisTrajectoryMessage(pelvisHeightTrajectoryMessage);  
  
   }
   
   public void setMessage(ArrayList<RRTNode> rrtPath)
   {
      if(cleaningPath == null)
      {
         PrintTools.warn("You must set cleaningPath");
      }
      else
      {
         this.wholebodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
         int numberOfWayPoints = rrtPath.size()-1;
         
         PrintTools.info("numberOfWayPoints "+numberOfWayPoints);
         
         HandTrajectoryMessage handTrajectoryMessage;
         ChestTrajectoryMessage chestTrajectoryMessage;
         
         handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, numberOfWayPoints);
         chestTrajectoryMessage = new ChestTrajectoryMessage(numberOfWayPoints);
         
         for(int i=1;i<rrtPath.size();i++)
         {
            double time = rrtPath.get(i).getNodeData(0);
            SolarPanelCleaningPose cleaningPose = cleaningPath.getCleaningPose(time);
            PrintTools.info("cleaningPose "+cleaningPose.getU() +" "+cleaningPose.getV() +" " + cleaningPose.getDesiredHandPosition().getY());
            PrintTools.info("cleaningPose "+cleaningPose.getDesiredHandPosition().getX() +" "+cleaningPose.getDesiredHandPosition().getY() +" " + cleaningPose.getDesiredHandPosition().getZ());
                        
            handTrajectoryMessage.setTrajectoryPoint(i-1, time, cleaningPose.getDesiredHandPosition(), cleaningPose.getDesiredHandOrientation(), new Vector3D(), new Vector3D(), ReferenceFrame.getWorldFrame());            
            
            Quaternion desiredChestOrientation = new Quaternion();
            desiredChestOrientation.appendYawRotation(rrtPath.get(i).getNodeData(1));
            chestTrajectoryMessage.setTrajectoryPoint(i-1, time, desiredChestOrientation, new Vector3D(), ReferenceFrame.getWorldFrame());
            PrintTools.info("rrtPath.get(i).getNodeData(0) "+rrtPath.get(i).getNodeData(0));
            PrintTools.info("rrtPath.get(i).getNodeData(1) "+rrtPath.get(i).getNodeData(1));
         }
//         double time = rrtPath.get(1).getNodeData(0);
//         SolarPanelCleaningPose cleaningPose = cleaningPath.getCleaningPose(time);
//         handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, time, cleaningPose.getDesiredHandPosition(), cleaningPose.getDesiredHandOrientation(), ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
//         PrintTools.info("cleaningPose "+cleaningPose.getDesiredHandPosition().getX() +" "+cleaningPose.getDesiredHandPosition().getY() +" " + cleaningPose.getDesiredHandPosition().getZ());
//         Quaternion desiredChestOrientation = new Quaternion();
//         desiredChestOrientation.appendYawRotation(rrtPath.get(1).getNodeData(1));
//         chestTrajectoryMessage = new ChestTrajectoryMessage(time, desiredChestOrientation, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
         
         wholebodyTrajectoryMessage.setHandTrajectoryMessage(handTrajectoryMessage);
         wholebodyTrajectoryMessage.setChestTrajectoryMessage(chestTrajectoryMessage);         
         
      }

   }
}
