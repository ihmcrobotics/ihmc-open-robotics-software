package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.WrenchDistributorTools;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.CapturabilityBasedDesiredCoPVisualizer;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointTrajectoryData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
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

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;

public class ICPBasedLinearMomentumRateOfChangeControlModule extends AbstractControlFlowElement implements ICPBasedMomentumRateOfChangeControlModule
{
   private static final boolean KEEP_CMP_INSIDE_SUPPORT_POLYGON = false;
   
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
      FramePoint2d desiredCMP = icpProportionalController.doProportionalControl(capturePointData.getCapturePoint(),
                                   desiredCapturePointTrajectory.getDesiredCapturePoint(), desiredCapturePointTrajectory.getDesiredCapturePointVelocity(),
                                   capturePointData.getOmega0());
      FrameConvexPolygon2d supportPolygon = bipedSupportPolygonsInputPort.getData().getSupportPolygonInMidFeetZUp();
      desiredCMP.changeFrame(supportPolygon.getReferenceFrame());

      if (KEEP_CMP_INSIDE_SUPPORT_POLYGON)
      {
    	  if (supportPolygon.isPointInside(desiredCMP))
    	  {
    		  cmpProjected.set(false);
    	  }
    	  else
    	  {
    		  supportPolygon.orthogonalProjection(desiredCMP);
    		  cmpProjected.set(true);
    	  }
      }
      
      desiredCMP.changeFrame(this.controlledCMP.getReferenceFrame());
      this.controlledCMP.set(desiredCMP);

      visualizer.setDesiredCapturePoint(desiredCapturePointTrajectory.getDesiredCapturePoint());
      visualizer.setDesiredCMP(desiredCMP);

      supportLegPreviousTick.set(supportLegInputPort.getData());

      double fZ = WrenchDistributorTools.computeFz(totalMass, gravityZ, desiredCenterOfMassHeightAccelerationInputPort.getData());
      FrameVector linearMomentumRateOfChange = computeGroundReactionForce(desiredCMP, fZ);
      linearMomentumRateOfChange.changeFrame(centerOfMassFrame);
      linearMomentumRateOfChange.setZ(linearMomentumRateOfChange.getZ() - totalMass * gravityZ);
      
      if (linearMomentumRateOfChange.containsNaN())
         throw new RuntimeException("linearMomentumRateOfChange = " + linearMomentumRateOfChange);

      momentumRateOfChangeData.setLinearMomentumRateOfChange(linearMomentumRateOfChange);
   }

   private FrameVector computeGroundReactionForce(FramePoint2d cmp2d, double fZ)
   {
      FramePoint centerOfMass = new FramePoint(centerOfMassFrame);
      FramePoint cmp3d = WrenchDistributorTools.computePseudoCMP3d(centerOfMass, cmp2d, fZ, totalMass, capturePointInputPort.getData().getOmega0());
      
      visualizer.setPseudoCMP(cmp3d);
      
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
