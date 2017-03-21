package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import java.util.Random;

import us.ihmc.commons.PrintTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage.BaseForControl;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class SolarPanelStraightPathPlanner
{
   private SolarPanelStraightPath straightPath;
   private Random random = new Random(564574L);
   /*
    * private HandTrajectoryMessage handTrajectoryMessage; // only yaw angle I
    * can private PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage;
    * // temporary constant private ChestTrajectoryMessage
    * chestTrajectoryMessage; // temporary constant
    */

   private double motionTime;

   private static double maxPelvisYaw = Math.PI * 0.4;
   private static double minPelvisYaw = -Math.PI * 0.1;

   private static double maxPelvisHeight = 0.1;
   private static double minPelvisHeight = -0.1;

   private static double maxZRotation = Math.PI * 0.5;
   private static double minZRotation = -Math.PI * 0.5;
   
   private double startPelvisYawSLIR;
   private double startPelvisHeightSLIR;
   private double startZRotationSLIR;
   
   private double endPelvisYawSLIR;
   private double endPelvisHeightSLIR;
   private double endZRotationSLIR;

   public SolarPanelStraightPathPlanner(SolarPanelStraightPath solarPanelStraightPath, double motionTime)
   {
      straightPath = solarPanelStraightPath;
      this.motionTime = motionTime;
      // pelvisheight
      // chest
   }

   // ********************************************************************************************************* //   
   // SLIR **************************************************************************************************** //  
   // Simple Linear Interpolation for Randomly selected start pose and end pose. (pelvisYaw, zRotation only) ** //
   // ********************************************************************************************************* //
   private double getRandomZRotation()
   {
      
      return RandomNumbers.nextDouble(random, minZRotation, maxZRotation);
   }

   private double getRandomPelvisYaw()
   {
      return RandomNumbers.nextDouble(random, minPelvisYaw, maxPelvisYaw);
   }

   private double getRandomPelvisHeight()
   {
      return RandomNumbers.nextDouble(random, minPelvisHeight, maxPelvisHeight);
   }
   
   public void getOptimalRedundancyForSLIR()
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

   public HandTrajectoryMessage getHandTrajectorySLIR()
   {
      HandTrajectoryMessage retMaessage = new HandTrajectoryMessage(RobotSide.RIGHT, BaseForControl.WORLD, 2);

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
            
      retMaessage.setTrajectoryPoint(0, 1.0, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
            
      endPose.setZRotation(endZRotationSLIR);
      desiredPosition = new Point3D(endPose.getDesiredHandPosition());
      desiredLinearVelocity = new Vector3D();
      desiredOrientation = new Quaternion(endPose.getDesiredHandOrientation());
      desiredAngularVelocity = new Vector3D();
      
      retMaessage.setTrajectoryPoint(1, motionTime - 1.0, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);

      return retMaessage;
   }   

   public ChestTrajectoryMessage getChestTrajectorySLIR()
   {
      ChestTrajectoryMessage retMaessage = new ChestTrajectoryMessage(2);

      Quaternion desiredOrientation;      
      Vector3D desiredAngularVelocity;
      
      desiredOrientation = new Quaternion();
      desiredOrientation.appendYawRotation(startPelvisYawSLIR);
      desiredAngularVelocity = new Vector3D();
      
      retMaessage.setTrajectoryPoint(0, 1.0, desiredOrientation, desiredAngularVelocity);
            
      desiredOrientation = new Quaternion();
      desiredOrientation.appendYawRotation(endPelvisYawSLIR);
      desiredAngularVelocity = new Vector3D();
      
      retMaessage.setTrajectoryPoint(1, motionTime - 1.0, desiredOrientation, desiredAngularVelocity);

      return retMaessage;
   }
   
   // select random start yaw
   // select random end yaw
   // interpolate Linealy

   // set node, discrete time and yaw
}





















