package us.ihmc.commonWalkingControlModules.capturePoint;


import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationControllerInterface;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;

public class ICPBasedLinearMomentumRateOfChangeControlModule extends LeggedLinearMomentumRateOfChangeControlModule
{
   private final ICPProportionalController icpProportionalController;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final YoBoolean desiredCMPinSafeArea;

   protected final FrameConvexPolygon2D areaToProjectInto = new FrameConvexPolygon2D();
   protected final FrameConvexPolygon2D safeArea = new FrameConvexPolygon2D();

   private final YoFrameConvexPolygon2D yoSafeAreaPolygon;
   private final YoFrameConvexPolygon2D yoProjectionPolygon;
   
   private final FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();
   private final CMPProjector cmpProjector;

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
      super("", referenceFrames, gravityZ, totalMass, parentRegistry, yoGraphicsListRegistry);
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.desiredCMPinSafeArea = new YoBoolean("DesiredCMPinSafeArea", registry);

      yoSafeAreaPolygon = new YoFrameConvexPolygon2D("yoSafeAreaPolygon", worldFrame, 10, registry);
      yoProjectionPolygon = new YoFrameConvexPolygon2D("yoProjectionPolygon", worldFrame, 10, registry);


      icpProportionalController = new ICPProportionalController(icpControlGains, controlDT, registry);

      if (use2DProjection)
         cmpProjector = new SmartCMPProjector(yoGraphicsListRegistry, registry);
      else
         cmpProjector = new SmartCMPPlanarProjector(registry);
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

      // we don't have any knowledge of a feedback CoP, so we're going to set this value to the perfect CoP.
      desiredCoP.set(perfectCoP);
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

   @Override
   public void setCMPProjectionArea(FrameConvexPolygon2DReadOnly areaToProjectInto, FrameConvexPolygon2DReadOnly safeArea)
   {
      this.areaToProjectInto.setIncludingFrame(areaToProjectInto);
      this.safeArea.setIncludingFrame(safeArea);

      yoSafeAreaPolygon.set(safeArea);
      yoProjectionPolygon.set(areaToProjectInto);
   }
}
