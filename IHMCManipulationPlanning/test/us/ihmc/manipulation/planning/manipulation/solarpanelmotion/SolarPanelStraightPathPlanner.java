package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage.BaseForControl;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class SolarPanelStraightPathPlanner
{
   private ArrayList<SolarPanelStraightPath> straightPaths = new ArrayList<SolarPanelStraightPath>();
   private ArrayList<Double> zRotation = new ArrayList<Double>();
   private ArrayList<Double> pelvisYaw = new ArrayList<Double>();
   
   
   public SolarPanelStraightPathPlanner()
   {
            
   }
   
   public SolarPanelStraightPathPlanner(ArrayList<SolarPanelStraightPath> straightPaths)
   {
      this.straightPaths = straightPaths;      
   }
   
   public SolarPanelStraightPathPlanner(SolarPanelStraightPath straightPath)
   {
      this.straightPaths.add(straightPath);
   }
   
   public void addPath(SolarPanelStraightPath straightPath)
   {
      this.straightPaths.add(straightPath);
   }
   
   public ArrayList<SolarPanelStraightPath> getPaths()
   {
      return straightPaths;
   }
   
   // ******************** //
   // ******* SLIR ******* //
   // ******************** //
   public void temporarySolution()
   {
      // linear
      if(false)
      {
         PrintTools.info("a");
         zRotation.add(-Math.PI*0.2);
         zRotation.add(-Math.PI*0.3);
         pelvisYaw.add(Math.PI*0.05);
         pelvisYaw.add(Math.PI*0.3);   
      }
      else
      {
         PrintTools.info("b");
         zRotation.add(-Math.PI*0.2);
         zRotation.add(-Math.PI*0.3);
         zRotation.add(-Math.PI*0.3);
         zRotation.add(-Math.PI*0.2);
         zRotation.add(-Math.PI*0.2);
         zRotation.add(-Math.PI*0.3);
         zRotation.add(-Math.PI*0.3);
         zRotation.add(-Math.PI*0.2);
         zRotation.add(-Math.PI*0.2);
         zRotation.add(-Math.PI*0.3);
         
         pelvisYaw.add(Math.PI*0.05);
         pelvisYaw.add(Math.PI*0.3);
         pelvisYaw.add(Math.PI*0.3);
         pelvisYaw.add(Math.PI*0.05);
         pelvisYaw.add(Math.PI*0.05);
         pelvisYaw.add(Math.PI*0.3);
         pelvisYaw.add(Math.PI*0.3);
         pelvisYaw.add(Math.PI*0.05);
         pelvisYaw.add(Math.PI*0.05);
         pelvisYaw.add(Math.PI*0.3);
      }
      // whole
   }
   
   public HandTrajectoryMessage getHandTrajectoryTemporary()
   {      
      HandTrajectoryMessage retMaessage = new HandTrajectoryMessage(RobotSide.RIGHT, BaseForControl.WORLD, straightPaths.size()+1);
      
      Point3D desiredPosition;
      Vector3D desiredLinearVelocity;
      Quaternion desiredOrientation;      
      Vector3D desiredAngularVelocity;

      SolarPanelCleaningPose startPose = straightPaths.get(0).getStartPose();
      SolarPanelCleaningPose endPose = straightPaths.get(0).getEndPose();
      startPose.setZRotation(zRotation.get(0));
      desiredPosition = new Point3D(startPose.getDesiredHandPosition());
      desiredLinearVelocity = new Vector3D();
      desiredOrientation = new Quaternion(startPose.getDesiredHandOrientation());
      desiredAngularVelocity = new Vector3D();
      retMaessage.setTrajectoryPoint(0, 1.0, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
            
      endPose.setZRotation(zRotation.get(1));
      desiredPosition = new Point3D(endPose.getDesiredHandPosition());
      desiredLinearVelocity = new Vector3D();
      desiredOrientation = new Quaternion(endPose.getDesiredHandOrientation());
      desiredAngularVelocity = new Vector3D();
      
      retMaessage.setTrajectoryPoint(1, straightPaths.get(0).getMotionTime() - 1.0, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);

      return retMaessage;
   } 
   
   public HandTrajectoryMessage getHandTrajectoryTemporaryWhole()
   {      
      HandTrajectoryMessage retMaessage = new HandTrajectoryMessage(RobotSide.RIGHT, BaseForControl.WORLD, straightPaths.size()+1);
      
      Point3D desiredPosition;
      Vector3D desiredLinearVelocity;
      Quaternion desiredOrientation;      
      Vector3D desiredAngularVelocity;

      int index;
      
      index = 0;      
      straightPaths.get(index).getStartPose().setZRotation(zRotation.get(index));      
      desiredPosition = new Point3D(straightPaths.get(index).getStartPose().getDesiredHandPosition());
      desiredLinearVelocity = new Vector3D();
      desiredOrientation = new Quaternion(straightPaths.get(index).getStartPose().getDesiredHandOrientation());
      desiredAngularVelocity = new Vector3D();
      retMaessage.setTrajectoryPoint(index, 1.0, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
      
      index = 1;      
      straightPaths.get(index).getStartPose().setZRotation(zRotation.get(index));      
      desiredPosition = new Point3D(straightPaths.get(index).getStartPose().getDesiredHandPosition());
      desiredLinearVelocity = new Vector3D();
      desiredOrientation = new Quaternion(straightPaths.get(index).getStartPose().getDesiredHandOrientation());
      desiredAngularVelocity = new Vector3D();
      retMaessage.setTrajectoryPoint(index, 6.0, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
      
      index = 2;      
      straightPaths.get(index).getStartPose().setZRotation(zRotation.get(index));      
      desiredPosition = new Point3D(straightPaths.get(index).getStartPose().getDesiredHandPosition());
      desiredLinearVelocity = new Vector3D();
      desiredOrientation = new Quaternion(straightPaths.get(index).getStartPose().getDesiredHandOrientation());
      desiredAngularVelocity = new Vector3D();
      retMaessage.setTrajectoryPoint(index, 7.0, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
      
      index = 3;      
      straightPaths.get(index).getStartPose().setZRotation(zRotation.get(index));      
      desiredPosition = new Point3D(straightPaths.get(index).getStartPose().getDesiredHandPosition());
      desiredLinearVelocity = new Vector3D();
      desiredOrientation = new Quaternion(straightPaths.get(index).getStartPose().getDesiredHandOrientation());
      desiredAngularVelocity = new Vector3D();
      retMaessage.setTrajectoryPoint(index, 15.0, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
      
      index = 4;      
      straightPaths.get(index).getStartPose().setZRotation(zRotation.get(index));      
      desiredPosition = new Point3D(straightPaths.get(index).getStartPose().getDesiredHandPosition());
      desiredLinearVelocity = new Vector3D();
      desiredOrientation = new Quaternion(straightPaths.get(index).getStartPose().getDesiredHandOrientation());
      desiredAngularVelocity = new Vector3D();
      retMaessage.setTrajectoryPoint(index, 16.0, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
      
      index = 5;      
      straightPaths.get(index).getStartPose().setZRotation(zRotation.get(index));      
      desiredPosition = new Point3D(straightPaths.get(index).getStartPose().getDesiredHandPosition());
      desiredLinearVelocity = new Vector3D();
      desiredOrientation = new Quaternion(straightPaths.get(index).getStartPose().getDesiredHandOrientation());
      desiredAngularVelocity = new Vector3D();
      retMaessage.setTrajectoryPoint(index, 21.0, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
      
      index = 6;      
      straightPaths.get(index).getStartPose().setZRotation(zRotation.get(index));      
      desiredPosition = new Point3D(straightPaths.get(index).getStartPose().getDesiredHandPosition());
      desiredLinearVelocity = new Vector3D();
      desiredOrientation = new Quaternion(straightPaths.get(index).getStartPose().getDesiredHandOrientation());
      desiredAngularVelocity = new Vector3D();
      retMaessage.setTrajectoryPoint(index, 22.0, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
      
      index = 7;      
      straightPaths.get(index).getStartPose().setZRotation(zRotation.get(index));      
      desiredPosition = new Point3D(straightPaths.get(index).getStartPose().getDesiredHandPosition());
      desiredLinearVelocity = new Vector3D();
      desiredOrientation = new Quaternion(straightPaths.get(index).getStartPose().getDesiredHandOrientation());
      desiredAngularVelocity = new Vector3D();
      retMaessage.setTrajectoryPoint(index, 27.0, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
      
      index = 8;      
      straightPaths.get(index).getStartPose().setZRotation(zRotation.get(index));      
      desiredPosition = new Point3D(straightPaths.get(index).getStartPose().getDesiredHandPosition());
      desiredLinearVelocity = new Vector3D();
      desiredOrientation = new Quaternion(straightPaths.get(index).getStartPose().getDesiredHandOrientation());
      desiredAngularVelocity = new Vector3D();
      retMaessage.setTrajectoryPoint(index, 28.0, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
           
      straightPaths.get(index).getEndPose().setZRotation(zRotation.get(index+1));      
      desiredPosition = new Point3D(straightPaths.get(index).getEndPose().getDesiredHandPosition());
      desiredLinearVelocity = new Vector3D();
      desiredOrientation = new Quaternion(straightPaths.get(index).getEndPose().getDesiredHandOrientation());
      desiredAngularVelocity = new Vector3D();
      retMaessage.setTrajectoryPoint(index+1, 33.0, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
                  

      return retMaessage;
   } 

   public ChestTrajectoryMessage getChestTrajectoryTemporary()
   {
      ChestTrajectoryMessage retMaessage = new ChestTrajectoryMessage(2);

      Quaternion desiredOrientation;      
      Vector3D desiredAngularVelocity;
      
      desiredOrientation = new Quaternion();
      desiredOrientation.appendYawRotation(pelvisYaw.get(0));
      desiredAngularVelocity = new Vector3D();
      
      retMaessage.setTrajectoryPoint(0, 1.0, desiredOrientation, desiredAngularVelocity);
            
      desiredOrientation = new Quaternion();
      desiredOrientation.appendYawRotation(pelvisYaw.get(1));
      desiredAngularVelocity = new Vector3D();
      
      retMaessage.setTrajectoryPoint(1, straightPaths.get(0).getMotionTime() - 1.0, desiredOrientation, desiredAngularVelocity);

      return retMaessage;
   }
   
   public ChestTrajectoryMessage getChestTrajectoryTemporaryWhole()
   {
      ChestTrajectoryMessage retMaessage = new ChestTrajectoryMessage(straightPaths.size()+1);

      Quaternion desiredOrientation;      
      Vector3D desiredAngularVelocity;

      int index;

      index = 0;      
      desiredOrientation = new Quaternion();
      desiredOrientation.appendYawRotation(pelvisYaw.get(index));
      desiredAngularVelocity = new Vector3D();
      retMaessage.setTrajectoryPoint(index, 1.0, desiredOrientation, desiredAngularVelocity);
      
      index = 1;      
      desiredOrientation = new Quaternion();
      desiredOrientation.appendYawRotation(pelvisYaw.get(index));
      desiredAngularVelocity = new Vector3D();
      retMaessage.setTrajectoryPoint(index, 6.0, desiredOrientation, desiredAngularVelocity);

      index = 2;      
      desiredOrientation = new Quaternion();
      desiredOrientation.appendYawRotation(pelvisYaw.get(index));
      desiredAngularVelocity = new Vector3D();
      retMaessage.setTrajectoryPoint(index, 7.0, desiredOrientation, desiredAngularVelocity);

      index = 3;      
      desiredOrientation = new Quaternion();
      desiredOrientation.appendYawRotation(pelvisYaw.get(index));
      desiredAngularVelocity = new Vector3D();
      retMaessage.setTrajectoryPoint(index, 15.0, desiredOrientation, desiredAngularVelocity);
      
      index = 4;      
      desiredOrientation = new Quaternion();
      desiredOrientation.appendYawRotation(pelvisYaw.get(index));
      desiredAngularVelocity = new Vector3D();
      retMaessage.setTrajectoryPoint(index, 16.0, desiredOrientation, desiredAngularVelocity);

      index = 5;      
      desiredOrientation = new Quaternion();
      desiredOrientation.appendYawRotation(pelvisYaw.get(index));
      desiredAngularVelocity = new Vector3D();
      retMaessage.setTrajectoryPoint(index, 21.0, desiredOrientation, desiredAngularVelocity);
      
      index = 6;      
      desiredOrientation = new Quaternion();
      desiredOrientation.appendYawRotation(pelvisYaw.get(index));
      desiredAngularVelocity = new Vector3D();
      retMaessage.setTrajectoryPoint(index, 22.0, desiredOrientation, desiredAngularVelocity);
      
      index = 7;      
      desiredOrientation = new Quaternion();
      desiredOrientation.appendYawRotation(pelvisYaw.get(index));
      desiredAngularVelocity = new Vector3D();
      retMaessage.setTrajectoryPoint(index, 27.0, desiredOrientation, desiredAngularVelocity);
      index = 8;  
      desiredOrientation = new Quaternion();
      desiredOrientation.appendYawRotation(pelvisYaw.get(index));
      desiredAngularVelocity = new Vector3D();
      retMaessage.setTrajectoryPoint(index, 28.0, desiredOrientation, desiredAngularVelocity);
      
      index = 9;   
      desiredOrientation = new Quaternion();
      desiredOrientation.appendYawRotation(pelvisYaw.get(index));
      desiredAngularVelocity = new Vector3D();
      retMaessage.setTrajectoryPoint(index, 33.0, desiredOrientation, desiredAngularVelocity);
      
      return retMaessage;
   
   }
   
   // select random start yaw
   // select random end yaw
   // interpolate Linealy

   // set node, discrete time and yaw
}





















