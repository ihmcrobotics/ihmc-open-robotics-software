package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.CapturabilityBasedDesiredCoPVisualizer;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointTrajectoryData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchDistributorTools;
import us.ihmc.controlFlow.AbstractControlFlowElement;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;

public class ICPBasedLinearMomentumRateOfChangeControlModule extends AbstractControlFlowElement implements ICPBasedMomentumRateOfChangeControlModule
{
   private static final boolean KEEP_CMP_INSIDE_SUPPORT_POLYGON = true; //false;
   
   private final ControlFlowInputPort<Double> desiredCenterOfMassHeightAccelerationInputPort = createInputPort("desiredCenterOfMassHeightAccelerationInputPort");
   private final ControlFlowInputPort<BipedSupportPolygons> bipedSupportPolygonsInputPort = createInputPort("bipedSupportPolygonsInputPort");
   private final ControlFlowInputPort<RobotSide> supportLegInputPort = createInputPort("supportLegInputPort");
   private final ControlFlowInputPort<CapturePointData> capturePointInputPort = createInputPort("capturePointInputPort");
   private final ControlFlowInputPort<CapturePointTrajectoryData> desiredCapturePointTrajectoryInputPort = createInputPort("desiredCapturePointTrajectoryInputPort");

   private final ControlFlowOutputPort<MomentumRateOfChangeData> momentumRateOfChangeOutputPort = createOutputPort("momentumRateOfChangeOutputPort");
   private final MomentumRateOfChangeData momentumRateOfChangeData;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ICPProportionalController icpProportionalController;
   private final CapturabilityBasedDesiredCoPVisualizer visualizer;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame centerOfMassFrame;

   private final YoFramePoint2d controlledCMP = new YoFramePoint2d("controlledCMP", "", worldFrame, registry);

   private final double totalMass;
   private final FramePoint centerOfMass;
   private final double gravityZ;

   private final EnumYoVariable<RobotSide> supportLegPreviousTick = EnumYoVariable.create("supportLegPreviousTick", "", RobotSide.class, registry, true);


   public ICPBasedLinearMomentumRateOfChangeControlModule(ReferenceFrame centerOfMassFrame,
           double controlDT, double totalMass, double gravityZ, YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      this.icpProportionalController = new ICPProportionalController(controlDT, registry, dynamicGraphicObjectsListRegistry);
      this.centerOfMassFrame = centerOfMassFrame;
      this.visualizer = new CapturabilityBasedDesiredCoPVisualizer(registry, dynamicGraphicObjectsListRegistry);
      this.totalMass = totalMass;
      this.centerOfMass = new FramePoint(centerOfMassFrame);
      this.gravityZ = gravityZ;
      this.momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
      parentRegistry.addChild(registry);
      momentumRateOfChangeOutputPort.setData(momentumRateOfChangeData);

      // hide CoP since we won't be calculating it explicitly in this class
      visualizer.setDesiredCoP(new FramePoint2d(controlledCMP.getReferenceFrame(), Double.NaN, Double.NaN));
   }

      
   public void startComputation()
   {
      if (supportLegInputPort.getData() != supportLegPreviousTick.getEnumValue())
      {
         icpProportionalController.reset();
      }

      CapturePointData capturePointData = capturePointInputPort.getData();
      CapturePointTrajectoryData desiredCapturePointTrajectory = desiredCapturePointTrajectoryInputPort.getData();
      FramePoint2d finalDesiredCapturePoint = desiredCapturePointTrajectory.getFinalDesiredCapturePoint();
      FramePoint2d desiredCapturePoint = desiredCapturePointTrajectory.getDesiredCapturePoint();
      FramePoint2d capturePoint = capturePointData.getCapturePoint();
      FrameConvexPolygon2d supportPolygon = bipedSupportPolygonsInputPort.getData().getSupportPolygonInMidFeetZUp();

      FramePoint2d desiredCMP = icpProportionalController.doProportionalControl(capturePoint,
                                   desiredCapturePoint, finalDesiredCapturePoint, desiredCapturePointTrajectory.getDesiredCapturePointVelocity(),
                                   capturePointData.getOmega0(), KEEP_CMP_INSIDE_SUPPORT_POLYGON, supportPolygon);
      

      desiredCMP.changeFrame(this.controlledCMP.getReferenceFrame());
      this.controlledCMP.set(desiredCMP);

      visualizer.setDesiredCapturePoint(desiredCapturePoint);
      visualizer.setDesiredCMP(desiredCMP);
      visualizer.setFinalDesiredCapturePoint(finalDesiredCapturePoint);

      supportLegPreviousTick.set(supportLegInputPort.getData());

      double fZ = WrenchDistributorTools.computeFz(totalMass, gravityZ, desiredCenterOfMassHeightAccelerationInputPort.getData());
      FrameVector linearMomentumRateOfChange = computeGroundReactionForce(desiredCMP, fZ);
      linearMomentumRateOfChange.changeFrame(centerOfMassFrame);
      linearMomentumRateOfChange.setZ(linearMomentumRateOfChange.getZ() - totalMass * gravityZ);
      
      if (linearMomentumRateOfChange.containsNaN())
         throw new RuntimeException("linearMomentumRateOfChange = " + linearMomentumRateOfChange);

      momentumRateOfChangeData.setLinearMomentumRateOfChange(linearMomentumRateOfChange);
   }

   private final FramePoint cmp3d = new FramePoint();
   private final FrameVector groundReactionForce = new FrameVector();
   
   private FrameVector computeGroundReactionForce(FramePoint2d cmp2d, double fZ)
   {
      centerOfMass.setToZero(centerOfMassFrame);
      WrenchDistributorTools.computePseudoCMP3d(cmp3d, centerOfMass, cmp2d, fZ, totalMass, capturePointInputPort.getData().getOmega0());
      
      visualizer.setPseudoCMP(cmp3d);
      
      centerOfMass.setToZero(centerOfMassFrame);
      WrenchDistributorTools.computeForce(groundReactionForce, centerOfMass, cmp3d, fZ);
      groundReactionForce.changeFrame(centerOfMassFrame);

      return groundReactionForce;
   }

   public void setGains(double captureKpParallelToMotion, double captureKpOrthogonalToMotion, double captureKi, double captureKiBleedoff,
         double filterBreakFrequencyHertz, double rateLimitCMP, double accelerationLimitCMP)
   {
      this.icpProportionalController.setGains(captureKpParallelToMotion, captureKpOrthogonalToMotion, captureKi, captureKiBleedoff, filterBreakFrequencyHertz, rateLimitCMP, accelerationLimitCMP);
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }
   
   public void getDesiredCMP(FramePoint2d desiredCMPToPack)
   {
      controlledCMP.getFrameTuple2dIncludingFrame(desiredCMPToPack);
   }

   public ControlFlowOutputPort<MomentumRateOfChangeData> getMomentumRateOfChangeOutputPort()
   {
      return momentumRateOfChangeOutputPort;
   }

   public ControlFlowInputPort<BipedSupportPolygons> getBipedSupportPolygonsInputPort()
   {
      return bipedSupportPolygonsInputPort;
   }

   public ControlFlowInputPort<RobotSide> getSupportLegInputPort()
   {
      return supportLegInputPort;
   }

   public ControlFlowInputPort<CapturePointData> getCapturePointInputPort()
   {
      return capturePointInputPort;
   }

   public ControlFlowInputPort<CapturePointTrajectoryData> getDesiredCapturePointTrajectoryInputPort()
   {
      return desiredCapturePointTrajectoryInputPort;
   }

   public ControlFlowInputPort<Double> getDesiredCenterOfMassHeightAccelerationInputPort()
   {
      return desiredCenterOfMassHeightAccelerationInputPort;
   }

   public void initialize()
   {
//    empty
   }
}
