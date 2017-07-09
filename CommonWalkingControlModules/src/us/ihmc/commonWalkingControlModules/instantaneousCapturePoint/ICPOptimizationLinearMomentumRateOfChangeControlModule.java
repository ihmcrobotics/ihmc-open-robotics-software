package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPAdjustmentOptimizationController;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationController;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPTimingOptimizationController;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.ReferenceFrames;

public class ICPOptimizationLinearMomentumRateOfChangeControlModule extends LinearMomentumRateOfChangeControlModule
{
   private final ICPOptimizationController icpOptimizationController;
   private final YoDouble yoTime;
   private final BipedSupportPolygons bipedSupportPolygons;
   
   private final SideDependentList<RigidBodyTransform> transformsFromAnkleToSole = new SideDependentList<>();

   public ICPOptimizationLinearMomentumRateOfChangeControlModule(ReferenceFrames referenceFrames, BipedSupportPolygons bipedSupportPolygons,
         SideDependentList<? extends ContactablePlaneBody> contactableFeet, CapturePointPlannerParameters icpPlannerParameters,
         ICPOptimizationParameters icpOptimizationParameters, WalkingControllerParameters walkingControllerParameters, YoDouble yoTime, double totalMass,
         double gravityZ, double controlDT, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(referenceFrames, bipedSupportPolygons, contactableFeet, icpPlannerParameters, icpOptimizationParameters, walkingControllerParameters, yoTime,
            totalMass, gravityZ, controlDT, parentRegistry, yoGraphicsListRegistry, true);
   }

   public ICPOptimizationLinearMomentumRateOfChangeControlModule(ReferenceFrames referenceFrames, BipedSupportPolygons bipedSupportPolygons,
         SideDependentList<? extends ContactablePlaneBody> contactableFeet, CapturePointPlannerParameters icpPlannerParameters,
         ICPOptimizationParameters icpOptimizationParameters, WalkingControllerParameters walkingControllerParameters,
         YoDouble yoTime, double totalMass, double gravityZ, double controlDT,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry, boolean use2DProjection)
   {
      super("", referenceFrames, gravityZ, totalMass, parentRegistry, yoGraphicsListRegistry, use2DProjection);
      this.bipedSupportPolygons = bipedSupportPolygons;

      this.yoTime = yoTime;

      MathTools.checkIntervalContains(gravityZ, 0.0, Double.POSITIVE_INFINITY);
      
      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody contactableFoot = contactableFeet.get(robotSide);
         ReferenceFrame ankleFrame = contactableFoot.getRigidBody().getParentJoint().getFrameAfterJoint();
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         RigidBodyTransform ankleToSole = new RigidBodyTransform();
         ankleFrame.getTransformToDesiredFrame(ankleToSole, soleFrame);
         transformsFromAnkleToSole.put(robotSide, ankleToSole);
      }

      if (icpOptimizationParameters.useTimingOptimization())
      {
         icpOptimizationController = new ICPTimingOptimizationController(icpPlannerParameters, icpOptimizationParameters, walkingControllerParameters,
               bipedSupportPolygons, contactableFeet, controlDT, registry, yoGraphicsListRegistry);
      }
      else
      {
         icpOptimizationController = new ICPAdjustmentOptimizationController(icpPlannerParameters, icpOptimizationParameters, walkingControllerParameters,
               bipedSupportPolygons, contactableFeet, controlDT, registry, yoGraphicsListRegistry);
      }
   }

   @Override
   public void clearPlan()
   {
      icpOptimizationController.clearPlan();
   }

   @Override
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      icpOptimizationController.addFootstepToPlan(footstep, timing);
   }

   @Override
   public void setFinalTransferDuration(double finalTransferDuration)
   {
      icpOptimizationController.setFinalTransferDuration(finalTransferDuration);
      icpOptimizationController.setFinalTransferSplitFractionToDefault();
   }

   @Override
   public void initializeForStanding()
   {
      icpOptimizationController.initializeForStanding(yoTime.getDoubleValue());
   }

   @Override
   public void initializeForSingleSupport()
   {
      icpOptimizationController.initializeForSingleSupport(yoTime.getDoubleValue(), supportSide, omega0);
   }

   @Override
   public void initializeForTransfer()
   {
      icpOptimizationController.initializeForTransfer(yoTime.getDoubleValue(), transferToSide, omega0);
   }

   @Override
   public void computeCMPInternal(FramePoint2d desiredCMPPreviousValue)
   {
      icpOptimizationController.compute(yoTime.getDoubleValue(), desiredCapturePoint, desiredCapturePointVelocity, capturePoint, omega0);
      icpOptimizationController.getDesiredCMP(desiredCMP);

      yoUnprojectedDesiredCMP.set(desiredCMP);

      // do projection here:
      if (!areaToProjectInto.isEmpty())
      {
         desiredCMPinSafeArea.set(safeArea.isPointInside(desiredCMP));
         if (safeArea.isPointInside(desiredCMP))
         {
            supportPolygon.setIncludingFrameAndUpdate(bipedSupportPolygons.getSupportPolygonInMidFeetZUp());
            areaToProjectInto.setIncludingFrameAndUpdate(supportPolygon);
         }

         if (!icpOptimizationController.useAngularMomentum())
            cmpProjector.projectCMPIntoSupportPolygonIfOutside(capturePoint, areaToProjectInto, finalDesiredCapturePoint, desiredCMP);
      }
   }

   private final FramePose footstepPose = new FramePose();
   private final FramePoint2d footstepPositionSolution = new FramePoint2d();

   @Override
   public boolean getUpcomingFootstepSolution(Footstep footstepToPack)
   {
      if (icpOptimizationController.getNumberOfFootstepsToConsider() > 0)
      {
         RigidBodyTransform ankleToSole = transformsFromAnkleToSole.get(footstepToPack.getRobotSide());
         
         footstepToPack.getAnklePose(footstepPose, ankleToSole);
         icpOptimizationController.getFootstepSolution(0, footstepPositionSolution);
         footstepPose.setXYFromPosition2d(footstepPositionSolution);
         footstepToPack.setFromAnklePose(footstepPose, ankleToSole);
      }

      return icpOptimizationController.wasFootstepAdjusted();
   }

   @Override
   public void submitRemainingTimeInSwingUnderDisturbance(double remainingTimeForSwing)
   {
      icpOptimizationController.submitRemainingTimeInSwingUnderDisturbance(remainingTimeForSwing);
   }

   @Override
   public ICPOptimizationController getICPOptimizationController()
   {
      return icpOptimizationController;
   }

   @Override
   public double getOptimizedTimeRemaining()
   {
      return icpOptimizationController.getOptimizedTimeRemaining();
   }

   @Override
   public void setReferenceICPVelocity(FrameVector2d referenceICPVelocity)
   {
      icpOptimizationController.setReferenceICPVelocity(referenceICPVelocity);
   }
}
