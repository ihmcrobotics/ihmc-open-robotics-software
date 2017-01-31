package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationController;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.ReferenceFrames;

public class ICPOptimizationLinearMomentumRateOfChangeControlModule extends LinearMomentumRateOfChangeControlModule
{
   private final ICPOptimizationController icpOptimizationController;
   private final DoubleYoVariable yoTime;
   private final BipedSupportPolygons bipedSupportPolygons;

   public ICPOptimizationLinearMomentumRateOfChangeControlModule(ReferenceFrames referenceFrames, BipedSupportPolygons bipedSupportPolygons,
         SideDependentList<? extends ContactablePlaneBody> contactableFeet, CapturePointPlannerParameters icpPlannerParameters,
         ICPOptimizationParameters icpOptimizationParameters, WalkingControllerParameters walkingControllerParameters, DoubleYoVariable yoTime, double totalMass,
         double gravityZ, double controlDT, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(referenceFrames, bipedSupportPolygons, contactableFeet, icpPlannerParameters, icpOptimizationParameters, walkingControllerParameters, yoTime,
            totalMass, gravityZ, controlDT, parentRegistry, yoGraphicsListRegistry, true);
   }

   public ICPOptimizationLinearMomentumRateOfChangeControlModule(ReferenceFrames referenceFrames, BipedSupportPolygons bipedSupportPolygons,
         SideDependentList<? extends ContactablePlaneBody> contactableFeet, CapturePointPlannerParameters icpPlannerParameters,
         ICPOptimizationParameters icpOptimizationParameters, WalkingControllerParameters walkingControllerParameters,
         DoubleYoVariable yoTime, double totalMass, double gravityZ, double controlDT,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry, boolean use2DProjection)
   {
      super("", referenceFrames, gravityZ, totalMass, parentRegistry, yoGraphicsListRegistry, use2DProjection);
      this.bipedSupportPolygons = bipedSupportPolygons;

      this.yoTime = yoTime;

      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      icpOptimizationController = new ICPOptimizationController(icpPlannerParameters, icpOptimizationParameters, walkingControllerParameters,
            bipedSupportPolygons, contactableFeet, controlDT, registry, yoGraphicsListRegistry);
   }

   public void setDoubleSupportDuration(double doubleSupportDuration)
   {
      icpOptimizationController.setDoubleSupportDuration(doubleSupportDuration);
   }

   public void setSingleSupportDuration(double singleSupportDuration)
   {
      icpOptimizationController.setSingleSupportDuration(singleSupportDuration);
   }

   public void clearPlan()
   {
      icpOptimizationController.clearPlan();
   }

   public void addFootstepToPlan(Footstep footstep)
   {
      icpOptimizationController.addFootstepToPlan(footstep);
   }

   public void initializeForStanding()
   {
      icpOptimizationController.initializeForStanding(yoTime.getDoubleValue());
   }

   public void initializeForSingleSupport()
   {
      icpOptimizationController.initializeForSingleSupport(yoTime.getDoubleValue(), supportSide, omega0);
   }

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

         cmpProjector.projectCMPIntoSupportPolygonIfOutside(capturePoint, areaToProjectInto, finalDesiredCapturePoint, desiredCMP);
      }
   }

   private final FramePose footstepPose = new FramePose();
   private final FramePoint2d footstepPositionSolution = new FramePoint2d();

   @Override public boolean getUpcomingFootstepSolution(Footstep footstepToPack)
   {
      if (icpOptimizationController.getNumberOfFootstepsToConsider() > 0)
      {
         footstepToPack.getPose(footstepPose);
         icpOptimizationController.getFootstepSolution(0, footstepPositionSolution);
         footstepPose.setXYFromPosition2d(footstepPositionSolution);
         footstepToPack.setPose(footstepPose);
      }

      return icpOptimizationController.wasFootstepAdjusted();
   }

   @Override
   public void submitRemainingTimeInSwingUnderDisturbance(double remainingTimeForSwing)
   {
      icpOptimizationController.submitRemainingTimeInSwingUnderDisturbance(remainingTimeForSwing);
   }
}
