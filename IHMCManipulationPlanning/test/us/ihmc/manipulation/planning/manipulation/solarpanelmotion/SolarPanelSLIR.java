package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import java.util.Random;

import us.ihmc.commons.PrintTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

/*
 * SLIR **************************************************************************************************** //  
   Simple Linear Interpolation for Randomly selected start pose and end pose. (pelvisYaw, zRotation only) ** //
   ********************************************************************************************************* //
 */

public class SolarPanelSLIR
{
   private SolarPanelStraightPath straightPath;
   private Random random = new Random(564574L);
   
   private static Point2D limitPelvisYaw = new Point2D(-Math.PI * 0.1, Math.PI * 0.4);
   private static Point2D limitPelvisHeight = new Point2D(-0.1, 0.1);
   private static Point2D limitZRotatio = new Point2D(-Math.PI * 0.5, Math.PI * 0.5);
      
   private double startPelvisYawSLIR;
   private double startPelvisHeightSLIR;
   private double startZRotationSLIR;
   
   private double endPelvisYawSLIR;
   private double endPelvisHeightSLIR;
   private double endZRotationSLIR;
   
   private double motionTime;
   
   public SolarPanelSLIR(SolarPanelStraightPath solarPanelStraightPath)
   {
      this.straightPath = solarPanelStraightPath;
      this.motionTime = this.straightPath.getMotionTime(); 
   }
   
   private double getRandomZRotation()
   {      
      return RandomNumbers.nextDouble(random, limitZRotatio.getX(), limitZRotatio.getY());
   }

   private double getRandomPelvisYaw()
   {
      return RandomNumbers.nextDouble(random, limitPelvisYaw.getX(), limitPelvisYaw.getY());
   }

   private double getRandomPelvisHeight()
   {
      return RandomNumbers.nextDouble(random, limitPelvisHeight.getX(), limitPelvisHeight.getY());
   }
   
   public void getOptimalRedundancyFor()
   {   
      startPelvisYawSLIR = getRandomPelvisYaw();
      startZRotationSLIR = getRandomZRotation();
      
      PrintTools.info("start (pelvisYaw) " + startPelvisYawSLIR + " (zRotation) " + startZRotationSLIR);
      
      endPelvisYawSLIR = getRandomPelvisYaw();
      endZRotationSLIR = getRandomZRotation();
      
      PrintTools.info("end   (pelvisYaw) " + endPelvisYawSLIR + " (zRotation) " + endZRotationSLIR);
      
      // ** temporary fixed ** //
      startPelvisYawSLIR = Math.PI*0.05;
      startZRotationSLIR = -Math.PI*0.2;
      
      endPelvisYawSLIR = Math.PI*0.3;
      endZRotationSLIR = -Math.PI*0.3;
   }

   public HandTrajectoryMessage getHandTrajectory()
   {
      HandTrajectoryMessage retMaessage = new HandTrajectoryMessage(RobotSide.RIGHT, 2);
      retMaessage.setTrajectoryReferenceFrameId(ReferenceFrame.getWorldFrame());

      Point3D desiredPosition;
      Vector3D desiredLinearVelocity;
      Quaternion desiredOrientation;      
      Vector3D desiredAngularVelocity;

      SolarPanelCleaningPose startPose = straightPath.getStartPose();
      SolarPanelCleaningPose endPose = straightPath.getEndPose();
      
      startPose.setZRotation(startZRotationSLIR);
      desiredPosition = new Point3D(startPose.getDesiredHandPosition());
      desiredLinearVelocity = new Vector3D();
      desiredOrientation = new Quaternion(startPose.getDesiredHandOrientation());
      desiredAngularVelocity = new Vector3D();
            
      retMaessage.setTrajectoryPoint(0, 1.0, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity, ReferenceFrame.getWorldFrame());
            
      endPose.setZRotation(endZRotationSLIR);
      desiredPosition = new Point3D(endPose.getDesiredHandPosition());
      desiredLinearVelocity = new Vector3D();
      desiredOrientation = new Quaternion(endPose.getDesiredHandOrientation());
      desiredAngularVelocity = new Vector3D();
      
      retMaessage.setTrajectoryPoint(1, motionTime - 1.0, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity, ReferenceFrame.getWorldFrame());

      return retMaessage;
   }   

   public ChestTrajectoryMessage getChestTrajectory()
   {
      ChestTrajectoryMessage retMaessage = new ChestTrajectoryMessage(2);

      Quaternion desiredOrientation;      
      Vector3D desiredAngularVelocity;
      
      desiredOrientation = new Quaternion();
      desiredOrientation.appendYawRotation(startPelvisYawSLIR);
      desiredAngularVelocity = new Vector3D();
      
      retMaessage.setTrajectoryPoint(0, 1.0, desiredOrientation, desiredAngularVelocity, ReferenceFrame.getWorldFrame());
            
      desiredOrientation = new Quaternion();
      desiredOrientation.appendYawRotation(endPelvisYawSLIR);
      desiredAngularVelocity = new Vector3D();
      
      retMaessage.setTrajectoryPoint(1, motionTime - 1.0, desiredOrientation, desiredAngularVelocity, ReferenceFrame.getWorldFrame());

      return retMaessage;
   }
}
