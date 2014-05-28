package us.ihmc.atlas.parameters;

import static us.ihmc.atlas.ros.AtlasOrderedJointMap.back_bky;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.back_bkz;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.jointNames;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.neck_ry;

import java.util.LinkedHashMap;
import java.util.Map;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.screwTheory.OneDoFJoint;


public class AtlasWalkingControllerParameters implements WalkingControllerParameters
{   
   private final boolean runningOnRealRobot;
   private final SideDependentList<Transform3D> handControlFramesWithRespectToFrameAfterWrist = new SideDependentList<Transform3D>();
   private final SideDependentList<Transform3D> handPosesWithRespectToChestFrame = new SideDependentList<Transform3D>();
   private final double minElbowRollAngle = 0.5;
   
   // Limits
   private final double neck_pitch_upper_limit = 1.14494; //0.83;    // true limit is = 1.134460, but pitching down more just looks at more robot chest
   private final double neck_pitch_lower_limit = -0.602139; //-0.610865;    // -math.pi/2.0;
   private final double head_yaw_limit = Math.PI / 4.0;
   private final double head_roll_limit = Math.PI / 4.0;
   private final double pelvis_pitch_upper_limit = 0.0;
   private final double pelvis_pitch_lower_limit = -0.35; //-math.pi / 6.0;

   private final double  min_leg_length_before_collapsing_single_support = 0.53; // corresponds to q_kny = 1.70 rad

   public AtlasWalkingControllerParameters()
   {
      this(false);
   }
   
   public AtlasWalkingControllerParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      for(RobotSide robotSide : RobotSide.values)
      {
         Transform3D rotationPart = new Transform3D();
         double yaw = robotSide.negateIfRightSide(Math.PI / 2.0);
         double pitch = 0.0;
         double roll = 0.0;
         rotationPart.setEuler(new Vector3d(roll, pitch, yaw));
   
         Transform3D toHand = new Transform3D();
//         double x = 0.2;
//         double y = robotSide.negateIfRightSide(-0.03);
//         double z = 0.04;
         double x = 0.1;
         double y = 0.0;
         double z = 0.0;
         toHand.setTranslation(new Vector3d(x, y, z));
   
         Transform3D transform = new Transform3D();
         transform.mul(rotationPart, toHand);
         
         handControlFramesWithRespectToFrameAfterWrist.put(robotSide, transform);
      }
      
      
      for (RobotSide robotSide : RobotSide.values)
      {
         Transform3D transform = new Transform3D();

         double x = 0.20;
         double y = robotSide.negateIfRightSide(0.35); //0.30);
         double z = -0.40;
         transform.setTranslation(new Vector3d(x, y, z));

         Matrix3d rotation = new Matrix3d();
         double yaw = 0.0;//robotSide.negateIfRightSide(-1.7);
         double pitch = 0.7;
         double roll = 0.0;//robotSide.negateIfRightSide(-0.8);
         RotationFunctions.setYawPitchRoll(rotation, yaw, pitch, roll);
         transform.setRotation(rotation);

         handPosesWithRespectToChestFrame.put(robotSide, transform);
      }
   }
   
   public boolean stayOnToes()
   {
      return false; // Not working for now
   }
   
   public boolean doToeOffIfPossible()
   {
      return true; 
   }

   public boolean doToeTouchdownIfPossible()
   {
      return false;
   }

   public boolean doHeelTouchdownIfPossible()
   {
      return false;
   }

   public String[] getDefaultHeadOrientationControlJointNames()
   {
      return new String[] {jointNames[back_bkz], jointNames[neck_ry]}; 
   }
   
   public String[] getDefaultChestOrientationControlJointNames()
   {
      return new String[]{};
   }

   public boolean checkOrbitalEnergyCondition()
   {
      return false;
   }

// USE THESE FOR Real Atlas Robot and sims when controlling pelvis height instead of CoM.
   private final double minimumHeightAboveGround = 0.595 + 0.03;                                       
   private double nominalHeightAboveGround = 0.675 + 0.03; 
   private final double maximumHeightAboveGround = 0.735 + 0.03;

// USE THESE FOR DRC Atlas Model TASK 2 UNTIL WALKING WORKS BETTER WITH OTHERS.
//   private final double minimumHeightAboveGround = 0.785;                                       
//   private double nominalHeightAboveGround = 0.865; 
//   private final double maximumHeightAboveGround = 0.925; 
   
//   // USE THESE FOR VRC Atlas Model TASK 2 UNTIL WALKING WORKS BETTER WITH OTHERS.
//   private double minimumHeightAboveGround = 0.68;                                       
//   private double nominalHeightAboveGround = 0.76; 
//   private double maximumHeightAboveGround = 0.82; 

//   // USE THESE FOR IMPROVING WALKING, BUT DONT CHECK THEM IN UNTIL IT IMPROVED WALKING THROUGH MUD.
//   private double minimumHeightAboveGround = 0.68;                                       
//   private double nominalHeightAboveGround = 0.80;  // NOTE: used to be 0.76, jojo        
//   private double maximumHeightAboveGround = 0.84;  // NOTE: used to be 0.82, jojo        
   
   public double minimumHeightAboveAnkle()
   {
      return minimumHeightAboveGround;
   }
   
   public double nominalHeightAboveAnkle()
   {
      return nominalHeightAboveGround;
   }
   
   public double maximumHeightAboveAnkle()
   {
      return maximumHeightAboveGround;
   }

   public void setNominalHeightAboveAnkle(double nominalHeightAboveAnkle)
   {
      this.nominalHeightAboveGround = nominalHeightAboveAnkle;
   }

   public double getGroundReactionWrenchBreakFrequencyHertz()
   {
      return 7.0;
   }

   public boolean resetDesiredICPToCurrentAtStartOfSwing()
   {
      return false;
   }

   public double getUpperNeckPitchLimit()
   {
      return neck_pitch_upper_limit;
   }

   public double getLowerNeckPitchLimit()
   {
      return neck_pitch_lower_limit;
   }

   public double getHeadYawLimit()
   {
      return head_yaw_limit;
   }

   public double getHeadRollLimit()
   {
      return head_roll_limit;
   }

   public String getJointNameForExtendedPitchRange()
   {
      return jointNames[back_bky];
   }

   public boolean finishSwingWhenTrajectoryDone()
   {
      return false;
   }

   public double getFootForwardOffset()
   {
      return AtlasPhysicalProperties.foot_forward;
   }
   
   public double getFootSwitchCoPThresholdFraction()
   {
	   return 0.02;
   }

   public double getFootBackwardOffset()
   {
      return AtlasPhysicalProperties.foot_back;
   }
   
   public double getAnkleHeight()
   {
      return AtlasPhysicalProperties.ankleHeight;
   }

   public double getLegLength()
   {
      return AtlasPhysicalProperties.shinLength + AtlasPhysicalProperties.thighLength;
   }
   
   public double getMinLegLengthBeforeCollapsingSingleSupport()
   {
      return min_leg_length_before_collapsing_single_support;
   }

   public double getFinalToeOffPitchAngularVelocity()
   {
      return 3.5;
   }

   public double getInPlaceWidth()
   {
      return 0.25;
   }

   public double getDesiredStepForward()
   {
      return 0.5; //0.35;
   }
  
   public double getMaxStepLength()
   {
       return 0.6; //0.5; //0.35;
   }

   public double getMinStepWidth()
   {
      return 0.15;
   }

   public double getMaxStepWidth()
   {
      return 0.6; //0.4;
   }

   public double getStepPitch()
   {
      return 0.0;
   }

   public double getCaptureKpParallelToMotion()
   {
      if (!runningOnRealRobot) return 1.0;
      return 1.0; 
   }

   public double getCaptureKpOrthogonalToMotion()
   {      
      if (!runningOnRealRobot) return 1.0; 
      return 1.0; 
   }
   
   public double getCaptureKi()
   {      
      if (!runningOnRealRobot) return 4.0;
      return 4.0; 
   }
   
   public double getCaptureKiBleedoff()
   {      
      return 0.9; 
   }
   
   public double getCaptureFilterBreakFrequencyInHz()
   {
      if (!runningOnRealRobot) return 16.0; //Double.POSITIVE_INFINITY;
      return 16.0;
   }
   
   public double getCMPRateLimit()
   {
      if (!runningOnRealRobot) return 60.0; 
      return 6.0; //3.0; //4.0; //3.0;
   }

   public double getCMPAccelerationLimit()
   {
      if (!runningOnRealRobot) return 2000.0;
      return 200.0; //80.0; //40.0;
   }
   
   public double getKpCoMHeight()
   {
      if (!runningOnRealRobot) return 40.0;
      return 40.0; //20.0; 
   }

   public double getZetaCoMHeight()
   {
      if (!runningOnRealRobot) return 0.8; //1.0;
      return 0.4;
   }
   
   public double getKpPelvisOrientation()
   {
      if (!runningOnRealRobot) return 80.0; //100.0;
      return 80.0; //30.0; 
   }

   public double getZetaPelvisOrientation()
   {
      if (!runningOnRealRobot) return 0.8; //1.0;
      return 0.25;
   }
   

   public double getMaxAccelerationPelvisOrientation()
   {
      if (!runningOnRealRobot) return 18.0;
      return 12.0; 
   }

   public double getMaxJerkPelvisOrientation()
   {
      if (!runningOnRealRobot) return 270.0;
      return 180.0; 
   }

   public double getKpHeadOrientation()
   {
      if (!runningOnRealRobot) return 40.0;
      return 40.0; 
   }

   public double getZetaHeadOrientation()
   {
      if (!runningOnRealRobot) return 0.8; //1.0;
      return 0.4;
   }

   public double getTrajectoryTimeHeadOrientation()
   {
      return 3.0;
   }

   public double getKpUpperBody()
   {
      if (!runningOnRealRobot) return 80.0; //100.0;
      return 80.0; //40.0;
   }

   public double getZetaUpperBody()
   {
      if (!runningOnRealRobot) return 0.8; //1.0;
      return 0.25;
   }
   
   public double getMaxAccelerationUpperBody()
   {
      if (!runningOnRealRobot) return 18.0; //100.0;
      return 6.0;
   }
   
   public double getMaxJerkUpperBody()
   {
      if (!runningOnRealRobot) return 270.0; //1000.0;
      return 60.0;
   }
   
   public double getSwingKpXY()
   {
      return 100.0;
   }
   
   public double getSwingHeightMaxForPushRecoveryTrajectory()
   {
      return 0.12;
   }
   
   public double getSwingKpZ()
   {
      return 200.0;
   }
   
   public double getSwingKpOrientation()
   {
      return 200.0;
   }
   
   public double getSwingZetaXYZ()
   {
      if (!runningOnRealRobot) return 0.7;
      return 0.25;
   }
   
   public double getSwingZetaOrientation()
   {
      if (!runningOnRealRobot) return 0.7;
      return 0.7; 
   }

   public double getHoldKpXY()
   {
      return 100.0;
   }
   
   public double getHoldKpOrientation()
   {
      if (!runningOnRealRobot) return 100.0;
      return 200.0;
   }
   
   public double getHoldZeta()
   {
      if (!runningOnRealRobot) return 1.0;
      return 0.2;
   }

   public double getSwingMaxPositionAcceleration()
   {
      if (!runningOnRealRobot) return Double.POSITIVE_INFINITY;
      return 10.0;
   }
   
   public double getSwingMaxPositionJerk()
   {
      if (!runningOnRealRobot) return Double.POSITIVE_INFINITY;
      return 150.0;
   }
   
   public double getSwingMaxOrientationAcceleration()
   {
      if (!runningOnRealRobot) return Double.POSITIVE_INFINITY;
      return 100.0;
   }
   
   public double getSwingMaxHeightForPushRecoveryTrajectory()
   {
	   return 0.15;
   }
   
   public double getSwingMaxOrientationJerk()
   {
      if (!runningOnRealRobot) return Double.POSITIVE_INFINITY;
      return 1500.0;
   }

   public double getSwingSingularityEscapeMultiplier()
   {
      return runningOnRealRobot ? 50.0 : 200.0;
   }

   public boolean doPrepareManipulationForLocomotion()
   {
      return DRCConfigParameters.HOLD_HANDS_IN_CHEST_FRAME_WHEN_WALKING;
   }

   public double getToeOffKpXY()
   {
      return 100.0;
   }

   public double getToeOffKpOrientation()
   {
      return 200.0;
   }

   public double getToeOffZeta()
   {
      return 0.4;
   }

   public boolean isRunningOnRealRobot()
   {
      return runningOnRealRobot;
   }

   public double getDefaultTransferTime()
   {
      return runningOnRealRobot ? 1.5 : 0.25;
   }

   public double getDefaultSwingTime()
   {
      return runningOnRealRobot ? 1.5 : 0.60;
   }

   public double getPelvisPitchUpperLimit()
   {
      return pelvis_pitch_upper_limit;
   }
   
   public double getPelvisPitchLowerLimit()
   {
      return pelvis_pitch_lower_limit;
   }

   public boolean isPelvisPitchReversed()
   {
      return false;
   }

   public double getFootWidth()
   {
      return AtlasPhysicalProperties.foot_width;
   }

   public double getToeWidth()
   {
      return AtlasPhysicalProperties.toe_width;
   }

   public double getFootLength()
   {
      return AtlasPhysicalProperties.foot_length;
   }

   public double getFoot_start_toetaper_from_back()
   {
      return AtlasPhysicalProperties.foot_start_toetaper_from_back;
   }

   public double getSideLengthOfBoundingBoxForFootstepHeight()
   {
      return (1 + 0.3) * 2 * Math.sqrt(getFootForwardOffset() * getFootForwardOffset()
            + 0.25 * getFootWidth() * getFootWidth());
   }
   
   public SideDependentList<Transform3D> getDesiredHandPosesWithRespectToChestFrame()
   {
      return handPosesWithRespectToChestFrame;
   }

   public Map<OneDoFJoint, Double> getDefaultArmJointPositions(FullRobotModel fullRobotModel, RobotSide robotSide)
   {
      Map<OneDoFJoint, Double> jointPositions = new LinkedHashMap<OneDoFJoint, Double>();

      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfRightSide(-1.30));
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_PITCH), 0.34);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_ROLL), robotSide.negateIfRightSide(1.18));
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH), 1.94);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.WRIST_PITCH), -0.19);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.WRIST_ROLL), robotSide.negateIfRightSide(-0.07));

//      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfRightSide(0.0));
//      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_PITCH), 0.0);
//      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_ROLL), robotSide.negateIfRightSide(0.0));
//      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH), 0.0);
//      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.WRIST_PITCH), 0.0);
//      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.WRIST_ROLL), robotSide.negateIfRightSide(0.0));

      return jointPositions;
   }


   public Map<OneDoFJoint, Double> getMinTaskspaceArmJointPositions(FullRobotModel fullRobotModel, RobotSide robotSide)
   {
      Map<OneDoFJoint, Double> ret = new LinkedHashMap<OneDoFJoint, Double>();
      if (robotSide == RobotSide.LEFT)
      {
         ret.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_ROLL), minElbowRollAngle);
      }
      return ret;
   }

   public Map<OneDoFJoint, Double> getMaxTaskspaceArmJointPositions(FullRobotModel fullRobotModel, RobotSide robotSide)
   {
      Map<OneDoFJoint, Double> ret = new LinkedHashMap<OneDoFJoint, Double>();
      if (robotSide == RobotSide.RIGHT)
      {
         ret.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_ROLL), -minElbowRollAngle);
      }
      return ret;
   }
   
   public SideDependentList<Transform3D> getHandControlFramesWithRespectToFrameAfterWrist()
   {
      return handControlFramesWithRespectToFrameAfterWrist;
   }

   public double getDesiredTouchdownVelocity()
   {
      return -0.3;
   }
}

