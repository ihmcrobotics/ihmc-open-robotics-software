package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.*;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationController;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.ReferenceFrames;

public class ICPOptimizationLinearMomentumRateOfChangeControlModule extends LinearMomentumRateOfChangeControlModule
{
   private final ICPOptimizationControllerInterface icpOptimizationController;
   private final YoDouble yoTime;
   private final FrameVector2D perfectCMPDelta = new FrameVector2D();

   private RobotSide supportSide = null;
   private RobotSide transferToSide = null;
   private final YoEnum<RobotSide> supportLegPreviousTick;

   public ICPOptimizationLinearMomentumRateOfChangeControlModule(ReferenceFrames referenceFrames, BipedSupportPolygons bipedSupportPolygons,
                                                                 ICPControlPolygons icpControlPolygons, SideDependentList<ContactableFoot> contactableFeet,
                                                                 WalkingControllerParameters walkingControllerParameters, YoDouble yoTime, double totalMass,
                                                                 double gravityZ, double controlDT, YoVariableRegistry parentRegistry,
                                                                 YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super("", referenceFrames, gravityZ, totalMass, parentRegistry, yoGraphicsListRegistry);

      this.yoTime = yoTime;

      icpOptimizationController = new ICPOptimizationController(walkingControllerParameters, bipedSupportPolygons, icpControlPolygons,
                                                                contactableFeet, controlDT, registry, yoGraphicsListRegistry);

      supportLegPreviousTick = YoEnum.create("SupportLegPreviousTick", "", RobotSide.class, registry, true);
   }

   public void setSupportLeg(RobotSide newSupportSide)
   {
      supportSide = newSupportSide;
   }

   public void setTransferToSide(RobotSide transferToSide)
   {
      this.transferToSide = transferToSide;
   }

   public void setTransferFromSide(RobotSide robotSide)
   {
      if (robotSide != null)
         this.transferToSide = robotSide.getOppositeSide();
   }

   @Override
   public boolean compute(FramePoint2DReadOnly desiredCMPPreviousValue, FramePoint2D desiredCMPToPack)
   {
      boolean inputsAreOk = super.compute(desiredCMPPreviousValue, desiredCMPToPack);
      supportLegPreviousTick.set(supportSide);

      return inputsAreOk;
   }

   @Override
   public boolean compute(FramePoint2DReadOnly desiredCMPPreviousValue, FramePoint2D desiredCMPToPack, FramePoint2D desiredCoPToPack)
   {
      boolean inputsAreOk = super.compute(desiredCMPPreviousValue, desiredCMPToPack, desiredCoPToPack);
      supportLegPreviousTick.set(supportSide);

      return inputsAreOk;
   }

   public void clearPlan()
   {
      icpOptimizationController.clearPlan();
   }

   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      icpOptimizationController.addFootstepToPlan(footstep, timing);
   }

   public void setFinalTransferDuration(double finalTransferDuration)
   {
      icpOptimizationController.setFinalTransferDuration(finalTransferDuration);
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
      icpOptimizationController.initializeForTransfer(yoTime.getDoubleValue(), transferToSide);
   }

   @Override
   public void computeCMPInternal(FramePoint2DReadOnly desiredCMPPreviousValue)
   {
      if (perfectCoP.containsNaN())
      {
         perfectCMPDelta.setToZero();
         icpOptimizationController.compute(yoTime.getDoubleValue(), desiredCapturePoint, desiredCapturePointVelocity, perfectCMP, capturePoint,
                                           capturePointVelocity, omega0);
      }
      else
      {
         perfectCMPDelta.sub(perfectCMP, perfectCoP);
         icpOptimizationController.compute(yoTime.getDoubleValue(), desiredCapturePoint, desiredCapturePointVelocity, perfectCoP, perfectCMPDelta, capturePoint,
                                           capturePointVelocity, omega0);
      }

      icpOptimizationController.getDesiredCMP(desiredCMP);
      icpOptimizationController.getDesiredCoP(desiredCoP);

      yoUnprojectedDesiredCMP.set(desiredCMP);
   }

   public boolean getUpcomingFootstepSolution(Footstep footstepToPack)
   {
      if (icpOptimizationController.useStepAdjustment())
      {
         icpOptimizationController.getFootstepSolution(footstepToPack);
      }

      return icpOptimizationController.wasFootstepAdjusted();
   }

   public void submitRemainingTimeInSwingUnderDisturbance(double remainingTimeForSwing)
   {
      icpOptimizationController.submitRemainingTimeInSwingUnderDisturbance(remainingTimeForSwing);
   }

   public ICPOptimizationControllerInterface getICPOptimizationController()
   {
      return icpOptimizationController;
   }

   public void submitCurrentPlanarRegions(RecyclingArrayList<PlanarRegion> planarRegions)
   {
      icpOptimizationController.submitCurrentPlanarRegions(planarRegions);
   }

   @Override
   public void setKeepCoPInsideSupportPolygon(boolean keepCoPInsideSupportPolygon)
   {
      this.icpOptimizationController.setKeepCoPInsideSupportPolygon(keepCoPInsideSupportPolygon);
   }
}
