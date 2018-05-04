package us.ihmc.commonWalkingControlModules.capturePoint;


import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationControllerInterface;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class ICPBasedLinearMomentumRateOfChangeControlModule extends LeggedLinearMomentumRateOfChangeControlModule
{
   private final ICPProportionalController icpProportionalController;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final YoBoolean desiredCMPinSafeArea;
   
   private final FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();


   public ICPBasedLinearMomentumRateOfChangeControlModule(CommonHumanoidReferenceFrames referenceFrames, BipedSupportPolygons bipedSupportPolygons,
         double controlDT, double totalMass, double gravityZ, YoICPControlGains icpControlGains, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(referenceFrames, bipedSupportPolygons, controlDT, totalMass, gravityZ, icpControlGains, parentRegistry, yoGraphicsListRegistry, true);
   }

   public ICPBasedLinearMomentumRateOfChangeControlModule(CommonHumanoidReferenceFrames referenceFrames, BipedSupportPolygons bipedSupportPolygons,
         double controlDT, double totalMass, double gravityZ, YoICPControlGains icpControlGains, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry, boolean use2DProjection)
   {
      super("", referenceFrames, gravityZ, totalMass, parentRegistry, yoGraphicsListRegistry, use2DProjection);
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.desiredCMPinSafeArea = new YoBoolean("DesiredCMPinSafeArea", registry);


      icpProportionalController = new ICPProportionalController(icpControlGains, controlDT, registry);


   }

   public void computeCMPInternal(FramePoint2DReadOnly desiredCMPPreviousValue)
   {
      if (supportSide != supportLegPreviousTick.getEnumValue())
      {
         icpProportionalController.reset();
      }

      desiredCMP.set(icpProportionalController.doProportionalControl(desiredCMPPreviousValue, capturePoint, desiredCapturePoint,
            finalDesiredCapturePoint, desiredCapturePointVelocity, perfectCMP, omega0));

      yoUnprojectedDesiredCMP.set(desiredCMP);

      // do projection here:
      if (!areaToProjectInto.isEmpty())
      {
         desiredCMPinSafeArea.set(safeArea.isPointInside(desiredCMP));
         if (safeArea.isPointInside(desiredCMP))
         {
            supportPolygon.setIncludingFrame(bipedSupportPolygons.getSupportPolygonInMidFeetZUp());
            areaToProjectInto.setIncludingFrame(supportPolygon);
         }

         cmpProjector.projectCMPIntoSupportPolygonIfOutside(capturePoint, areaToProjectInto, finalDesiredCapturePoint, desiredCMP);
         if (cmpProjector.getWasCMPProjected())
            icpProportionalController.bleedOffIntegralTerm();
      }
   }

   @Override
   public void clearPlan(){}

   @Override
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing){}

   @Override
   public void setFinalTransferDuration(double finalTransferDuration){}

   @Override
   public void initializeForStanding(){}

   @Override
   public void initializeForSingleSupport(){}

   @Override
   public void initializeForTransfer(){}

   @Override
   public void submitRemainingTimeInSwingUnderDisturbance(double remainingTimeInSwing) {}

   @Override
   public boolean getUpcomingFootstepSolution(Footstep footstepToPack)
   {
      return false;
   }

   @Override
   public ICPOptimizationControllerInterface getICPOptimizationController()
   {
      return null;
   }

   @Override
   public void submitCurrentPlanarRegions(RecyclingArrayList<PlanarRegion> planarRegions)
   {}

   @Override
   public void setKeepCoPInsideSupportPolygon(boolean keepCoPInsideSupportPolygon)
   {}
}
