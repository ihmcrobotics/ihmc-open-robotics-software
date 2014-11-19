package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.CapturabilityBasedDesiredCoPVisualizer;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointTrajectoryData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchDistributorTools;
import us.ihmc.controlFlow.AbstractControlFlowElement;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.utilities.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class ICPBasedLinearMomentumRateOfChangeControlModule extends AbstractControlFlowElement implements ICPBasedMomentumRateOfChangeControlModule
{
   private final ControlFlowInputPort<Double> desiredCenterOfMassHeightAccelerationInputPort = createInputPort("desiredCenterOfMassHeightAccelerationInputPort");
   private final ControlFlowInputPort<CapturePointTrajectoryData> desiredCapturePointTrajectoryInputPort = createInputPort("desiredCapturePointTrajectoryInputPort");

   private final MomentumRateOfChangeData momentumRateOfChangeData;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ICPProportionalController icpProportionalController;
   private final CapturabilityBasedDesiredCoPVisualizer visualizer;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame centerOfMassFrame;
   private final SideDependentList<ReferenceFrame> soleFrames;

   private final YoFramePoint2d controlledCMP = new YoFramePoint2d("controlledCMP", "", worldFrame, registry);
   private final YoFrameVector controlledCoMAcceleration;

   private final double totalMass;
   private final FramePoint centerOfMass;
   private final double gravityZ;

   private final BooleanYoVariable keepCMPInsideSupportPolygon = new BooleanYoVariable("keepCMPInsideSupportPolygon", registry);

   private final EnumYoVariable<RobotSide> supportLegPreviousTick = EnumYoVariable.create("supportLegPreviousTick", "", RobotSide.class, registry, true);

   private final BipedSupportPolygons bipedSupportPolygons;
   private final FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d();
   private RobotSide supportSide = null;

   private final CapturePointData capturePointData = new CapturePointData();
   
   public ICPBasedLinearMomentumRateOfChangeControlModule(CommonHumanoidReferenceFrames referenceFrames, BipedSupportPolygons bipedSupportPolygons,
         double controlDT, double totalMass, double gravityZ, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      icpProportionalController = new ICPProportionalController(controlDT, registry, yoGraphicsListRegistry);
      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      soleFrames = referenceFrames.getSoleFrames();

      this.bipedSupportPolygons = bipedSupportPolygons;

      visualizer = new CapturabilityBasedDesiredCoPVisualizer(registry, yoGraphicsListRegistry);
      this.totalMass = totalMass;
      centerOfMass = new FramePoint(centerOfMassFrame);
      this.gravityZ = gravityZ;
      momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
      parentRegistry.addChild(registry);

      // hide CoP since we won't be calculating it explicitly in this class
      visualizer.setDesiredCoP(new FramePoint2d(controlledCMP.getReferenceFrame(), Double.NaN, Double.NaN));

      controlledCoMAcceleration = new YoFrameVector("controlledCoMAcceleration", "", centerOfMassFrame, registry);

   }

   @Override
   public void startComputation()
   {
      if (supportSide != supportLegPreviousTick.getEnumValue())
      {
         icpProportionalController.reset();
      }

      CapturePointTrajectoryData desiredCapturePointTrajectory = desiredCapturePointTrajectoryInputPort.getData();
      FramePoint2d finalDesiredCapturePoint = desiredCapturePointTrajectory.getFinalDesiredCapturePoint();
      FramePoint2d desiredCapturePoint = desiredCapturePointTrajectory.getDesiredCapturePoint();
      FramePoint2d capturePoint = capturePointData.getCapturePoint();
      supportPolygon.setIncludingFrameAndUpdate(bipedSupportPolygons.getSupportPolygonInMidFeetZUp());
      boolean keepCmpInsideSupportPolygon = desiredCapturePointTrajectory.isProjectCMPIntoSupportPolygon();
      keepCMPInsideSupportPolygon.set(keepCmpInsideSupportPolygon);

      ReferenceFrame swingSoleFrame = null;
      if (supportSide != null)
         swingSoleFrame = soleFrames.get(supportSide.getOppositeSide());

      FramePoint2d desiredCMP = icpProportionalController.doProportionalControl(capturePoint, desiredCapturePoint, finalDesiredCapturePoint,
            desiredCapturePointTrajectory.getDesiredCapturePointVelocity(), capturePointData.getOmega0(), keepCmpInsideSupportPolygon, supportPolygon,
            swingSoleFrame);

      desiredCMP.changeFrame(controlledCMP.getReferenceFrame());
      controlledCMP.set(desiredCMP);

      visualizer.setDesiredCapturePoint(desiredCapturePoint);
      visualizer.setDesiredCMP(desiredCMP);
      visualizer.setFinalDesiredCapturePoint(finalDesiredCapturePoint);
      centerOfMass.setToZero(centerOfMassFrame);
      visualizer.setCenterOfMass(centerOfMass);

      supportLegPreviousTick.set(supportSide);

      double fZ = WrenchDistributorTools.computeFz(totalMass, gravityZ, desiredCenterOfMassHeightAccelerationInputPort.getData());
      FrameVector linearMomentumRateOfChange = computeGroundReactionForce(desiredCMP, fZ);
      linearMomentumRateOfChange.changeFrame(centerOfMassFrame);
      linearMomentumRateOfChange.setZ(linearMomentumRateOfChange.getZ() - totalMass * gravityZ);

      if (linearMomentumRateOfChange.containsNaN())
         throw new RuntimeException("linearMomentumRateOfChange = " + linearMomentumRateOfChange);

      controlledCoMAcceleration.set(linearMomentumRateOfChange);
      controlledCoMAcceleration.scale(1.0 / totalMass);
      momentumRateOfChangeData.setLinearMomentumRateOfChange(linearMomentumRateOfChange);
   }

   private final FramePoint cmp3d = new FramePoint();
   private final FrameVector groundReactionForce = new FrameVector();

   private FrameVector computeGroundReactionForce(FramePoint2d cmp2d, double fZ)
   {
      centerOfMass.setToZero(centerOfMassFrame);
      WrenchDistributorTools.computePseudoCMP3d(cmp3d, centerOfMass, cmp2d, fZ, totalMass, capturePointData.getOmega0());

      visualizer.setPseudoCMP(cmp3d);

      centerOfMass.setToZero(centerOfMassFrame);
      WrenchDistributorTools.computeForce(groundReactionForce, centerOfMass, cmp3d, fZ);
      groundReactionForce.changeFrame(centerOfMassFrame);

      return groundReactionForce;
   }

   public void setGains(double captureKpParallelToMotion, double captureKpOrthogonalToMotion, double captureKi, double captureKiBleedoff,
         double filterBreakFrequencyHertz, double rateLimitCMP, double accelerationLimitCMP)
   {
      icpProportionalController.setGains(captureKpParallelToMotion, captureKpOrthogonalToMotion, captureKi, captureKiBleedoff, filterBreakFrequencyHertz,
            rateLimitCMP, accelerationLimitCMP);
   }

   @Override
   public void waitUntilComputationIsDone()
   {
      // empty
   }

   @Override
   public void getDesiredCMP(FramePoint2d desiredCMPToPack)
   {
      controlledCMP.getFrameTuple2dIncludingFrame(desiredCMPToPack);
   }

   @Override
   public void getMomentumRateOfChange(MomentumRateOfChangeData momentumRateOfChangeDataToPack)
   {
      momentumRateOfChangeDataToPack.set(momentumRateOfChangeData);
   }

   @Override
   public void setSupportLeg(RobotSide newSupportSide)
   {
      supportSide = newSupportSide;
   }

   @Override
   public void setCapturePointData(CapturePointData newCapturePointData)
   {
      capturePointData.set(newCapturePointData);
   }

   @Override
   public ControlFlowInputPort<CapturePointTrajectoryData> getDesiredCapturePointTrajectoryInputPort()
   {
      return desiredCapturePointTrajectoryInputPort;
   }

   @Override
   public ControlFlowInputPort<Double> getDesiredCenterOfMassHeightAccelerationInputPort()
   {
      return desiredCenterOfMassHeightAccelerationInputPort;
   }

   @Override
   public void initialize()
   {
      //    empty
   }
}
