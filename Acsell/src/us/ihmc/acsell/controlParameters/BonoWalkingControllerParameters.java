package us.ihmc.acsell.controlParameters;

import java.util.LinkedHashMap;
import java.util.Map;

import javax.media.j3d.Transform3D;

import us.ihmc.acsell.parameters.BonoPhysicalProperties;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

/**
 * Created by dstephen on 2/14/14.
 */
public class BonoWalkingControllerParameters implements WalkingControllerParameters
{

   private final SideDependentList<Transform3D> handControlFramesWithRespectToFrameAfterWrist = new SideDependentList<Transform3D>();
   private final SideDependentList<Transform3D> handPosesWithRespectToChestFrame = new SideDependentList<Transform3D>();

   private final boolean runningOnRealRobot;
   
   public BonoWalkingControllerParameters()
   {
      this(false);
   }
   
   public BonoWalkingControllerParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      
      for(RobotSide robotSide : RobotSide.values())
      {
         handControlFramesWithRespectToFrameAfterWrist.put(robotSide, new Transform3D());
         handPosesWithRespectToChestFrame.put(robotSide, new Transform3D());
      }
   }

   public SideDependentList<Transform3D> getDesiredHandPosesWithRespectToChestFrame()
   {
      return handPosesWithRespectToChestFrame;
   }

   public Map<OneDoFJoint, Double> getDefaultArmJointPositions(FullRobotModel fullRobotModel, RobotSide robotSide)
   {
      return new LinkedHashMap<OneDoFJoint, Double>();
   }

   public Map<OneDoFJoint, Double> getMinTaskspaceArmJointPositions(FullRobotModel fullRobotModel, RobotSide robotSide)
   {
      return new LinkedHashMap<OneDoFJoint, Double>();
   }

   public Map<OneDoFJoint, Double> getMaxTaskspaceArmJointPositions(FullRobotModel fullRobotModel, RobotSide robotSide)
   {
      return new LinkedHashMap<OneDoFJoint, Double>();
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
      return new String[0];
   }

   public String[] getAllowableHeadOrientationControlJointNames()
   {
      return new String[0];
   }

   public String[] getDefaultChestOrientationControlJointNames()
   {
      return new String[0];
   }

   public String[] getAllowableChestOrientationControlJointNames()
   {
      return new String[] {"back_mby"};
   }

   public boolean checkOrbitalEnergyCondition()
   {
      return false;
   }

   private final double minimumHeightAboveGround = 0.595 + 0.03;
   private double nominalHeightAboveGround = 0.675 + 0.03;
   private final double maximumHeightAboveGround = 0.735 + 0.03;
   private final double additionalOffsetHeightBono = 0.05;

   public double minimumHeightAboveAnkle()
   {
      return minimumHeightAboveGround + additionalOffsetHeightBono;
   }

   public double nominalHeightAboveAnkle()
   {
      return nominalHeightAboveGround + additionalOffsetHeightBono;
   }

   public double maximumHeightAboveAnkle()
   {
      return maximumHeightAboveGround + additionalOffsetHeightBono;
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
      return 0.0;
   }

   public double getLowerNeckPitchLimit()
   {
      return 0.0;
   }

   public double getHeadYawLimit()
   {
      return 0.0;
   }

   public double getHeadRollLimit()
   {
      return 0.0;
   }

   public String getJointNameForExtendedPitchRange()
   {
      return null;
   }

   public SideDependentList<Transform3D> getHandControlFramesWithRespectToFrameAfterWrist()
   {
      return handControlFramesWithRespectToFrameAfterWrist;
   }

   public boolean finishSwingWhenTrajectoryDone()
   {
      return false;
   }

   public double getFootForwardOffset()
   {
      return BonoPhysicalProperties.footForward;
   }

   public double getFootBackwardOffset()
   {
      return BonoPhysicalProperties.footBack;
   }

   public double getAnkleHeight()
   {
      return BonoPhysicalProperties.ankleHeight;
   }

   public double getLegLength()
   {
      return BonoPhysicalProperties.legLength;
   }

   public double getMinLegLengthBeforeCollapsingSingleSupport()
   {
      //TODO: Useful values
      return 0.1;
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
      return 0.3; //0.5; //0.35;
   }

   public double getMaxStepLength()
   {
      return 0.4; //0.6; //0.5; //0.35;
   }

   public double getMinStepWidth()
   {
      return 0.15;
   }

   public double getMaxStepWidth()
   {
      return 0.4; //0.5; //0.4;
   }

   public double getStepPitch()
   {
      return 0.0;
   }

   public double getCaptureKpParallelToMotion()
   {
      return 1.0;
   }

   public double getCaptureKpOrthogonalToMotion()
   {
      return 1.0;
   }

   public double getCaptureKi()
   {
      return 4.0;
   }

   public double getCaptureKiBleedoff()
   {
      return 0.9;
   }

   public double getCaptureFilterBreakFrequencyInHz()
   {
      return 16.0; //Double.POSITIVE_INFINITY;
   }

   public double getCMPRateLimit()
   {
      return 60.0;
   }

   public double getCMPAccelerationLimit()
   {
      return 2000.0;
   }

   public double getKpCoMHeight()
   {
      return 50.0;
   }

   public double getZetaCoMHeight()
   {
      return 1.0;
   }

   public double getKpPelvisOrientation()
   {
      return 100.0;
   }

   public double getZetaPelvisOrientation()
   {
      return 0.8; //1.0;
   }

   public double getMaxAccelerationPelvisOrientation()
   {
      return Double.POSITIVE_INFINITY;
   }

   public double getMaxJerkPelvisOrientation()
   {
      return Double.POSITIVE_INFINITY;
   }

   public double getKpHeadOrientation()
   {
      return 40.0;
   }

   public double getZetaHeadOrientation()
   {
      return 0.8; //1.0;
   }

   public double getTrajectoryTimeHeadOrientation()
   {
      return 3.0;
   }

   public double getKpUpperBody()
   {
      return 100.0;
   }

   public double getZetaUpperBody()
   {
      return 0.8; //1.0;
   }

   public double getMaxAccelerationUpperBody()
   {
      return Double.POSITIVE_INFINITY; //100.0;
   }

   public double getMaxJerkUpperBody()
   {
      return Double.POSITIVE_INFINITY;//270.0; //1000.0;
   }

   public double getSwingKpXY()
   {
      return 100.0;
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
      return 0.7;
   }

   public double getSwingZetaOrientation()
   {
      return 0.7;
   }

   public double getHoldKpXY()
   {
      return 100.0;
   }

   public double getHoldKpOrientation()
   {
      return 100.0;
   }

   public double getHoldZeta()
   {
      return 1.0;
   }

   public double getSwingMaxPositionAcceleration()
   {
      return Double.POSITIVE_INFINITY;
   }

   public double getSwingMaxPositionJerk()
   {
      return Double.POSITIVE_INFINITY;
   }

   public double getSwingMaxOrientationAcceleration()
   {
      return Double.POSITIVE_INFINITY;
   }

   public double getswingMaxOrientationJerk()
   {
      return Double.POSITIVE_INFINITY;
   }

   public boolean doPrepareManipulationForLocomotion()
   {
      return true;
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
      return 0.25; // 1.5; //
   }

   public double getDefaultSwingTime()
   {
      return 0.6; // 1.5; //
   }

   public double getPelvisPitchUpperLimit()
   {
      return 0;
   }

   public double getPelvisPitchLowerLimit()
   {
      return 0;
   }

   public boolean isPelvisPitchReversed()
   {
      return false;
   }

   public double getFootWidth()
   {
      return BonoPhysicalProperties.footWidth;
   }

   public double getToeWidth()
   {
      return BonoPhysicalProperties.footWidth;
   }

   public double getFootLength()
   {
      return BonoPhysicalProperties.footForward + BonoPhysicalProperties.footBack;
   }

   public double getFoot_start_toetaper_from_back()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   public double getSideLengthOfBoundingBoxForFootstepHeight()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   public double getSwingHeightMaxForPushRecoveryTrajectory()
   {
	   return 0.15;
   }

   public double getDesiredTouchdownVelocity()
   {
      return -0.3;
   }
}
