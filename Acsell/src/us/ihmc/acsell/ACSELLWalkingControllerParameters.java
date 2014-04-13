package us.ihmc.acsell;

import java.util.LinkedHashMap;
import java.util.Map;

import javax.media.j3d.Transform3D;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

/**
 * Created by dstephen on 2/14/14.
 */
public class ACSELLWalkingControllerParameters implements WalkingControllerParameters
{

   private final SideDependentList<Transform3D> handControlFramesWithRespectToFrameAfterWrist = new SideDependentList<>();
   private final SideDependentList<Transform3D> handPosesWithRespectToChestFrame = new SideDependentList<>();

   private final boolean runningOnRealRobot;
   
   public ACSELLWalkingControllerParameters()
   {
      this(false);
   }
   
   public ACSELLWalkingControllerParameters(boolean runningOnRealRobot)
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
      return new LinkedHashMap<>();
   }

   public Map<OneDoFJoint, Double> getMinTaskspaceArmJointPositions(FullRobotModel fullRobotModel, RobotSide robotSide)
   {
      return new LinkedHashMap<>();
   }

   public Map<OneDoFJoint, Double> getMaxTaskspaceArmJointPositions(FullRobotModel fullRobotModel, RobotSide robotSide)
   {
      return new LinkedHashMap<>();
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
//      if(model == DRCRobotModel.BONO)
         return new String[] {"back_mby"};
//      else
//         return new String[0];
   }

   public boolean checkOrbitalEnergyCondition()
   {
      return false;
   }

   // USE THESE FOR Real Atlas Robot and sims when controlling pelvis height instead of CoM.
   private final double minimumHeightAboveGround = 0.595 + 0.03;
   private double nominalHeightAboveGround = 0.675 + 0.03;
   private final double maximumHeightAboveGround = 0.735 + 0.03;
   private final double additionalOffsetHeightBono = 0.05;

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
//      if(model == DRCRobotModel.BONO)
         return minimumHeightAboveGround + additionalOffsetHeightBono;
//      else
//         return minimumHeightAboveGround;
   }

   public double nominalHeightAboveAnkle()
   {
//      if(model == DRCRobotModel.BONO)
         return nominalHeightAboveGround + additionalOffsetHeightBono;
//      else
//         return nominalHeightAboveGround;
   }

   public double maximumHeightAboveAnkle()
   {
//      if(model == DRCRobotModel.BONO)
         return maximumHeightAboveGround + additionalOffsetHeightBono;
//      else
//         return maximumHeightAboveGround;
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
//      if (model == DRCRobotModel.AXL)
//         return AxlJointMap.footForward;
//      else if (model == DRCRobotModel.BONO)
         return BonoPhysicalProperties.footForward;
//      else
//         throw new RuntimeException("Should not get there");
   }

   public double getFootBackwardOffset()
   {
//      if (model == DRCRobotModel.AXL)
//         return AxlJointMap.footBack;
//      else if (model == DRCRobotModel.BONO)
         return BonoPhysicalProperties.footBack;
//      else
//         throw new RuntimeException("Should not get there");
   }

   public double getAnkleHeight()
   {
//      if (model == DRCRobotModel.AXL)
//         return AxlJointMap.ankleHeight;
//      else if (model == DRCRobotModel.BONO)
         return BonoPhysicalProperties.ankleHeight;
//      else
//         throw new RuntimeException("Should not get there");
   }

   public double getLegLength()
   {
//      if (model == DRCRobotModel.AXL)
//         return AxlJointMap.legLength;
//      else if (model == DRCRobotModel.BONO)
         return BonoPhysicalProperties.legLength;
//      else
//         throw new RuntimeException("Should not get there");
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
      return 0.5; //0.4;
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
      return 50.0;
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
      return 200.0; //100.0;
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
      return 300.0; //100.0;
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
      return false;
   }

   public double getDefaultTransferTime()
   {
      return 0.25;
   }

   public double getDefaultSwingTime()
   {
      return 0.6;
   }

   @Override
   public double getPelvisPitchUpperLimit()
   {
      return 0;
   }

   @Override
   public double getPelvisPitchLowerLimit()
   {
      return 0;
   }

   @Override
   public boolean isPelvisPitchReversed()
   {
      return false;
   }

   @Override
   public double getFootWidth()
   {
      // TODO Auto-generated method stub
      return BonoPhysicalProperties.footWidth;
   }

   @Override
   public double getToeWidth()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public double getFootLength()
   {
      // TODO Auto-generated method stub
      return BonoPhysicalProperties.footForward + BonoPhysicalProperties.footBack;
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
      // TODO Auto-generated method stub
      return 0;
   }
}
