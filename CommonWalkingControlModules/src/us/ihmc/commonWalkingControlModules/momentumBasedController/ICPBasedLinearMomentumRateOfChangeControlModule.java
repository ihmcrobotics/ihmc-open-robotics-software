package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.commonWalkingControlModules.WrenchDistributorTools;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.CapturabilityBasedDesiredCoPVisualizer;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;

public class ICPBasedLinearMomentumRateOfChangeControlModule implements ICPBasedMomentumRateOfChangeControlModule
{
   private final ControlFlowInputPort<Double> desiredCenterOfMassHeightAccelerationInputPort = new ControlFlowInputPort<Double>(this);
   private final ControlFlowInputPort<BipedSupportPolygons> bipedSupportPolygonsInputPort = new ControlFlowInputPort<BipedSupportPolygons>(this);
   private final ControlFlowInputPort<RobotSide> supportLegInputPort = new ControlFlowInputPort<RobotSide>(this);
   private final ControlFlowInputPort<CapturePointData> capturePointInputPort = new ControlFlowInputPort<CapturePointData>(this);
   private final ControlFlowInputPort<CapturePointTrajectoryData> desiredCapturePointTrajectoryInputPort =
      new ControlFlowInputPort<CapturePointTrajectoryData>(this);

   private final ControlFlowOutputPort<MomentumRateOfChangeData> momentumRateOfChangeOutputPort = new ControlFlowOutputPort<MomentumRateOfChangeData>(this);
   private final MomentumRateOfChangeData momentumRateOfChangeData;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ICPProportionalController icpProportionalController;
   private final CapturabilityBasedDesiredCoPVisualizer visualizer;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame centerOfMassFrame;

   private final YoFramePoint2d desiredCMP = new YoFramePoint2d("desiredCMP", "", worldFrame, registry);

   private final BooleanYoVariable cmpProjected = new BooleanYoVariable("cmpProjected", registry);
   private final double totalMass;
   private final double gravityZ;

   private final EnumYoVariable<RobotSide> supportLegPreviousTick = EnumYoVariable.create("supportLegPreviousTick", "", RobotSide.class, registry, true);

   public ICPBasedLinearMomentumRateOfChangeControlModule(ReferenceFrame centerOfMassFrame,
           double controlDT, double totalMass, double gravityZ, YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      this.icpProportionalController = new ICPProportionalController(controlDT, registry);
      this.centerOfMassFrame = centerOfMassFrame;
      this.visualizer = new CapturabilityBasedDesiredCoPVisualizer(registry, dynamicGraphicObjectsListRegistry);
      this.totalMass = totalMass;
      this.gravityZ = gravityZ;
      this.momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
      parentRegistry.addChild(registry);
      momentumRateOfChangeOutputPort.setData(momentumRateOfChangeData);

      // hide CoP since we won't be calculating it explicitly in this class
      visualizer.setDesiredCoP(new FramePoint2d(desiredCMP.getReferenceFrame(), Double.NaN, Double.NaN));
   }

   public void startComputation()
   {
      if (supportLegInputPort.getData() != supportLegPreviousTick.getEnumValue())
      {
         icpProportionalController.reset();
      }

      CapturePointData capturePointData = capturePointInputPort.getData();
      CapturePointTrajectoryData desiredCapturePointTrajectory = desiredCapturePointTrajectoryInputPort.getData();
      FramePoint2d desiredCMP = icpProportionalController.doProportionalControl(capturePointData.getCapturePoint(),
                                   desiredCapturePointTrajectory.getDesiredCapturePoint(), desiredCapturePointTrajectory.getDesiredCapturePointVelocity(),
                                   capturePointData.getOmega0());
      FrameConvexPolygon2d supportPolygon = bipedSupportPolygonsInputPort.getData().getSupportPolygonInMidFeetZUp();
      desiredCMP.changeFrame(supportPolygon.getReferenceFrame());

      if (supportPolygon.isPointInside(desiredCMP))
      {
         cmpProjected.set(false);
      }
      else
      {
         supportPolygon.orthogonalProjection(desiredCMP);
         cmpProjected.set(true);
      }

      desiredCMP.changeFrame(this.desiredCMP.getReferenceFrame());
      this.desiredCMP.set(desiredCMP);

      visualizer.setDesiredCapturePoint(desiredCapturePointTrajectory.getDesiredCapturePoint());
      visualizer.setDesiredCMP(desiredCMP);

      supportLegPreviousTick.set(supportLegInputPort.getData());

      double fZ = WrenchDistributorTools.computeFz(totalMass, gravityZ, desiredCenterOfMassHeightAccelerationInputPort.getData());
      FrameVector linearMomentumRateOfChange = computeGroundReactionForce(desiredCMP, fZ);
      linearMomentumRateOfChange.changeFrame(centerOfMassFrame);
      linearMomentumRateOfChange.setZ(linearMomentumRateOfChange.getZ() - totalMass * gravityZ);

      momentumRateOfChangeData.setLinearMomentumRateOfChange(linearMomentumRateOfChange);
   }

   private FrameVector computeGroundReactionForce(FramePoint2d cmp2d, double fZ)
   {
      FramePoint centerOfMass = new FramePoint(centerOfMassFrame);
      FramePoint cmp3d = WrenchDistributorTools.computePseudoCMP3d(centerOfMass, cmp2d, fZ, totalMass, capturePointInputPort.getData().getOmega0());
      FrameVector ret = WrenchDistributorTools.computeForce(centerOfMass, cmp3d, fZ);
      ret.changeFrame(centerOfMassFrame);

      return ret;
   }

   public void setGains(double captureKpParallelToMotion, double captureKpOrthogonalToMotion, double filterBreakFrequencyHertz)
   {
      this.icpProportionalController.setGains(captureKpParallelToMotion, captureKpOrthogonalToMotion, filterBreakFrequencyHertz);
   }

   public void waitUntilComputationIsDone()
   {
      // empty
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
}
