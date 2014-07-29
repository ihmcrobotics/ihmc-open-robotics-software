package us.ihmc.valkyrie.paramaters;

import javax.media.j3d.Transform3D;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.humanoidRobot.partNames.NeckJointName;
import us.ihmc.utilities.humanoidRobot.partNames.SpineJointName;

public class ValkyrieWalkingControllerParameters implements WalkingControllerParameters
{
   private final boolean runningOnRealRobot;

   private final SideDependentList<Transform3D> handControlFramesWithRespectToFrameAfterWrist = new SideDependentList<Transform3D>();
   private final SideDependentList<Transform3D> handPosesWithRespectToChestFrame = new SideDependentList<Transform3D>();

   private final double upperNeckExtensorUpperLimit = 0.785398;
   private final double upperNeckExtensorLowerLimit = -0.0872665;
   private final double lowerNeckExtensorUpperLimit = 0.0;
   private final double lowerNeckExtensorLowerLimit = -1.5708;

   private final  DRCRobotJointMap jointMap;
   
   public ValkyrieWalkingControllerParameters(DRCRobotJointMap jointMap)
   {
      this(jointMap, false);
   }
   
   public ValkyrieWalkingControllerParameters(DRCRobotJointMap jointMap, boolean runningOnRealRobot)
   {
      this.jointMap = jointMap;
      this.runningOnRealRobot = runningOnRealRobot;

      for (RobotSide robotSide : RobotSide.values)
      {
         handControlFramesWithRespectToFrameAfterWrist.put(robotSide, new Transform3D());
      }

      
      // Genreated using ValkyrieFullRobotModelVisualizer
      Transform3D leftHandLocation = new Transform3D(new double[] {0.8772111323383822, -0.47056204413925823, 0.09524700476706424, 0.11738015536007923,
            1.5892231999088989E-4, 0.1986725292086453, 0.980065916600275, 0.3166524835978034,
            -0.48010478444326166, -0.8597095955922112, 0.1743525371234003, -0.13686311108389013,
            0.0, 0.0, 0.0, 1.0});
      
      Transform3D rightHandLocation = new Transform3D(new double[] {0.8772107606751612, -0.47056267784177724, -0.09524729695945025, 0.11738015535642271,
            -1.5509783447718197E-4, -0.19866600827375044, 0.9800672390715021, -0.3166524835989298,
            -0.48010546476828164, -0.8597107556492186, -0.17434494349043353, -0.13686311108617974,
            0.0, 0.0, 0.0, 1.0});
      
      handPosesWithRespectToChestFrame.put(RobotSide.LEFT, leftHandLocation);
      handPosesWithRespectToChestFrame.put(RobotSide.RIGHT, rightHandLocation);
      
      
   }

   @Override
   public SideDependentList<Transform3D> getDesiredHandPosesWithRespectToChestFrame()
   {
      return handPosesWithRespectToChestFrame;
   }

   @Override
   public boolean stayOnToes()
   {
      return false; // Not working for now
   }

   @Override
   public boolean doToeOffIfPossible()
   {
      return !runningOnRealRobot;
   }

   @Override
   public double getMaximumToeOffAngle()
   {
      return Math.toRadians(45.0);
   }

   @Override
   public boolean doToeTouchdownIfPossible()
   {
      return false;
   }

   @Override
   public double getToeTouchdownAngle()
   {
      return Math.toRadians(20.0);
   }

   @Override
   public boolean doHeelTouchdownIfPossible()
   {
      return false;
   }

   @Override
   public double getHeelTouchdownAngle()
   {
      return Math.toRadians(-20.0);
   }

   @Override
   public String[] getDefaultHeadOrientationControlJointNames()
   {
     
      String[] defaultHeadOrientationControlJointNames = new String[] {
            jointMap.getSpineJointName(SpineJointName.SPINE_YAW),
            jointMap.getNeckJointName(NeckJointName.LOWER_NECK_PITCH)
            };

      return defaultHeadOrientationControlJointNames;
   }

   @Override
   public String[] getDefaultChestOrientationControlJointNames()
   {
      return new String[] {};
   }

   @Override
   public String getJointNameForExtendedPitchRange()
   {
      return jointMap.getSpineJointName(SpineJointName.SPINE_PITCH);
   }

   @Override
   public double getUpperNeckPitchLimit()
   {
      return upperNeckExtensorUpperLimit + lowerNeckExtensorUpperLimit;//1.14494;
   }

   @Override
   public double getLowerNeckPitchLimit()
   {
      return upperNeckExtensorLowerLimit + lowerNeckExtensorLowerLimit;//-0.602139;
   }

   @Override
   public double getHeadYawLimit()
   {
      return Math.PI / 4.0;
   }

   @Override
   public double getHeadRollLimit()
   {
      return Math.PI / 4.0;
   }

   @Override
   public boolean checkOrbitalEnergyCondition()
   {
      return false;
   }

   // USE THESE FOR Real Atlas Robot and sims when controlling pelvis height instead of CoM.
   private final double minimumHeightAboveGround = 0.595 + 0.23;
   private double nominalHeightAboveGround = 0.675 + 0.23;
   private final double maximumHeightAboveGround = 0.735 + 0.23;

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

   @Override
   public double minimumHeightAboveAnkle()
   {
      return minimumHeightAboveGround;
   }

   @Override
   public double nominalHeightAboveAnkle()
   {
      return nominalHeightAboveGround;
   }

   @Override
   public double maximumHeightAboveAnkle()
   {
      return maximumHeightAboveGround;
   }

   @Override
   public double getGroundReactionWrenchBreakFrequencyHertz()
   {
      return 7.0;
   }

   @Override
   public boolean resetDesiredICPToCurrentAtStartOfSwing()
   {
      return false;
   }

   @Override
   public SideDependentList<Transform3D> getHandControlFramesWithRespectToFrameAfterWrist()
   {
      return handControlFramesWithRespectToFrameAfterWrist;
   }
      
   @Override
   public boolean finishSwingWhenTrajectoryDone()
   {
      return false;
   }

   @Override
   public double getFootForwardOffset()
   {
      return ValkyriePhysicalProperties.footForward;
   }

   @Override
   public double getFootBackwardOffset()
   {
      return ValkyriePhysicalProperties.footBack;
   }
   
   @Override
   public double getFootSwitchCoPThresholdFraction()
   {
	   return 0.02;
   }

   @Override
   public double getAnkleHeight()
   {
      return ValkyriePhysicalProperties.ankleHeight;
   }

   @Override
   public double getLegLength()
   {
      return ValkyriePhysicalProperties.thighLength + ValkyriePhysicalProperties.shinLength;
   }

   @Override
   public double getMinLegLengthBeforeCollapsingSingleSupport()
   {
      //TODO: Useful values
      return 0.1;
   }
   
   @Override
   public double getSwingHeightMaxForPushRecoveryTrajectory()
   {
      return 0.1;
   }

   @Override
   public double getFinalToeOffPitchAngularVelocity()
   {
      return 3.5;
   }

   @Override
   public double getInPlaceWidth()
   {
      return 0.25;
   }

   @Override
   public double getDesiredStepForward()
   {
      return 0.5; //0.35;
   }

   @Override
   public double getMaxStepLength()
   {
      if(!runningOnRealRobot) return 0.6; //0.5; //0.35;
	  return 0.6;
      
   }

   @Override
   public double getMinStepWidth()
   {
      return 0.15;
   }

   @Override
   public double getMaxStepWidth()
   {
      return 0.6; //0.4;
   }

   @Override
   public double getStepPitch()
   {
      return 0.0;
   }

   @Override
   public double getCaptureKpParallelToMotion()
   {
      if (!runningOnRealRobot) return 1.0;
      return 1.0; 
   }

   @Override
   public double getCaptureKpOrthogonalToMotion()
   {
      if (!runningOnRealRobot) return 1.0; 
      return 1.0;
   }

   @Override
   public double getCaptureKi()
   {
      if (!runningOnRealRobot) return 4.0;
      return 4.0;
   }

   @Override
   public double getCaptureKiBleedoff()
   {
      return 0.9;
   }

   @Override
   public double getCaptureFilterBreakFrequencyInHz()
   {
      if (!runningOnRealRobot) return 16.0;
      return 16.0; //20.0;//16.0;
   }

   @Override
   public double getCMPRateLimit()
   {
      if (!runningOnRealRobot) return 60.0;
      return 6.0; //12.0;//60.0; //6.0;
   }

   @Override
   public double getCMPAccelerationLimit()
   {
      if (!runningOnRealRobot) return 2000.0;
      return 200.0; //400.0;//2000.0; //200.0;
   }
   
   @Override
   public double getKpCoMHeight()
   {
      if (!runningOnRealRobot) return 50.0;
      return 40.0;
   }

   @Override
   public double getZetaCoMHeight()
   {
      if (!runningOnRealRobot) return 1.0;
      return 0.4;
   }

   @Override
   public double getDefaultDesiredPelvisPitch()
   {
      return 0.0;
   }

   @Override
   public double getKpPelvisOrientation()
   {
      if (!runningOnRealRobot) return 100.0;
      return 80.0;
   }

   @Override
   public double getZetaPelvisOrientation()
   {
      if (!runningOnRealRobot) return 0.8;
      return 0.25;
   }
   

   @Override
   public double getMaxAccelerationPelvisOrientation()
   {
      if (!runningOnRealRobot) return 18.0;
      return 36.0;//24.0; //12.0; 
   }

   @Override
   public double getMaxJerkPelvisOrientation()
   {
      if (!runningOnRealRobot) return 270.0;
      return 540.0;//360.0; //180.0;
   }

   @Override
   public double getKpHeadOrientation()
   {
      if (!runningOnRealRobot) return 40.0;
      return 40.0; 
   }

   @Override
   public double getZetaHeadOrientation()
   {
      if (!runningOnRealRobot) return 0.8;
      return 0.4;
   }

   @Override
   public double getTrajectoryTimeHeadOrientation()
   {
      return 3.0;
   }

   @Override
   public double getKpUpperBody()
   {
      if (!runningOnRealRobot) return 100.0;
      return 80.0;
   }

   @Override
   public double getZetaUpperBody()
   {
      if (!runningOnRealRobot) return 0.8;
      return 0.25;
   }

   @Override
   public double getMaxAccelerationUpperBody()
   {
      if (!runningOnRealRobot) return 18.0;
      return 18.0;//24.0;//12.0; //6.0;
   }

   @Override
   public double getMaxJerkUpperBody()
   {
      if (!runningOnRealRobot) return 270.0;
      return 270.0; //360;//180.0; //60.0;
   }

   @Override
   public double getSwingKpXY()
   {
      if (!runningOnRealRobot) return 100.0;
      return 100.0;
   }

   @Override
   public double getSwingKpZ()
   {
      if (!runningOnRealRobot) return 200.0;
      return 100.0;
   }

   @Override
   public double getSwingKpOrientation()
   {
      if (!runningOnRealRobot) return 200.0;
      return 40.0;
   }

   @Override
   public double getSwingZetaXYZ()
   {
      if (!runningOnRealRobot) return 0.7;
      return 0.3;
   }

   @Override
   public double getSwingZetaOrientation()
   {
      if (!runningOnRealRobot) return 0.7;
      return 0.3; 
   }

   @Override
   public double getHoldKpXY()
   {
      return 100.0;
   }

   @Override
   public double getHoldKpOrientation()
   {
      if (!runningOnRealRobot) return 100.0;
      return 40.0;
   }

   @Override
   public double getHoldZeta()
   {
      if (!runningOnRealRobot) return 1.0;
      return 0.2;
   }

   @Override
   public double getSwingMaxPositionAcceleration()
   {
      if (!runningOnRealRobot) return Double.POSITIVE_INFINITY;
      return 10.0;
   }

   @Override
   public double getSwingMaxPositionJerk()
   {
      if (!runningOnRealRobot) return Double.POSITIVE_INFINITY;
      return 150.0;
   }

   @Override
   public double getSwingMaxOrientationAcceleration()
   {
      if (!runningOnRealRobot) return Double.POSITIVE_INFINITY;
      return 100.0;
   }

   @Override
   public double getSwingMaxOrientationJerk()
   {
      if (!runningOnRealRobot) return Double.POSITIVE_INFINITY;
      return 1500.0;
   }

   @Override
   public double getSupportSingularityEscapeMultiplier()
   {
      return -30; //negative as knee axis are -y direction
   }

   @Override
   public double getSwingSingularityEscapeMultiplier()
   {
      return -(runningOnRealRobot ? 50.0 : 200.0); //negative as knee axis are -y direction
   }

   @Override
   public boolean doPrepareManipulationForLocomotion()
   {
      return true;
   }

   @Override
   public double getToeOffKpXY()
   {
      return 100.0;
   }

   @Override
   public double getToeOffKpOrientation()
   {
      return 200.0;
   }

   @Override
   public double getToeOffZeta()
   {
      return 0.4;
   }

   @Override
   public boolean isRunningOnRealRobot()
   {
      return runningOnRealRobot;
   }

   @Override
   public double getDefaultTransferTime()
   {
      return runningOnRealRobot ? 1.0 : 0.25;
   }

   @Override
   public double getDefaultSwingTime()
   {
      return runningOnRealRobot ? 1.0 : 0.60;
   }

   @Override
   public double getPelvisPitchUpperLimit()
   {
      return 0.0872665;
   }

   @Override
   public double getPelvisPitchLowerLimit()
   {
      return -0.698132;
   }

   @Override
   public boolean isPelvisPitchReversed()
   {
      return true;
   }

   @Override
   public double getFootWidth()
   {
      return ValkyriePhysicalProperties.footWidth;
   }

   @Override
   public double getToeWidth()
   {
      return ValkyriePhysicalProperties.footWidth;
   }

   @Override
   public double getFootLength()
   {
      return ValkyriePhysicalProperties.footBack + ValkyriePhysicalProperties.footForward;
   }

   @Override
   public double getFoot_start_toetaper_from_back()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public double getSideLengthOfBoundingBoxForFootstepHeight()
   {
      return (1 + 0.3) * 2 * Math.sqrt(getFootForwardOffset() * getFootForwardOffset()
            + 0.25 * getFootWidth() * getFootWidth());
   }
   
   @Override
   public double getDesiredTouchdownVelocity()
   {
      return -0.3;
   }

   @Override
   public double getContactThresholdForce()
   {
      return runningOnRealRobot ? 80.0 : 5.0;
   }
}
