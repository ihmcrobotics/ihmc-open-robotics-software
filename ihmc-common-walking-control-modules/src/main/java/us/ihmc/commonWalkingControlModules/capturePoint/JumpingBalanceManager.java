package us.ihmc.commonWalkingControlModules.capturePoint;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.*;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CornerPointViewer;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.OptimizedCoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.JumpingControllerToolbox;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.*;

public class JumpingBalanceManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final BipedSupportPolygons bipedSupportPolygons;

   private final JumpingMomentumRateControlModuleInput jumpingMomentumRateControlModuleInput = new JumpingMomentumRateControlModuleInput();

   private final JumpingControllerToolbox controllerToolbox;

   private final YoFramePoint2D yoDesiredCapturePoint = new YoFramePoint2D("desiredICP", worldFrame, registry);
   private final YoFrameVector2D yoDesiredICPVelocity = new YoFrameVector2D("desiredICPVelocity", worldFrame, registry);
   private final YoFramePoint3D yoDesiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", worldFrame, registry);
   private final YoFrameVector3D yoDesiredCoMVelocity = new YoFrameVector3D("desiredCoMVelocity", worldFrame, registry);

   /** CMP position according to the ICP planner */
   private final YoFramePoint2D yoPerfectCMP = new YoFramePoint2D("perfectCMP", worldFrame, registry);

   private final ReferenceFrame centerOfMassFrame;

   private final FramePoint3D centerOfMassPosition = new FramePoint3D();
   private final FramePoint2D centerOfMassPosition2d = new FramePoint2D();

   private final FramePoint2D capturePoint2d = new FramePoint2D();
   private final FramePoint2D desiredCapturePoint2d = new FramePoint2D();
   private final FramePoint2D desiredCoM2d = new FramePoint2D();
   private final FrameVector2D desiredCapturePointVelocity2d = new FrameVector2D();
   private final FramePoint2D perfectCoP2d = new FramePoint2D();

   private final ConvexPolygonScaler convexPolygonShrinker = new ConvexPolygonScaler();
   private final FrameConvexPolygon2D shrunkSupportPolygon = new FrameConvexPolygon2D();

   private final YoDouble distanceToShrinkSupportPolygonWhenHoldingCurrent = new YoDouble("distanceToShrinkSupportPolygonWhenHoldingCurrent", registry);

   private final YoBoolean holdICPToCurrentCoMLocationInNextDoubleSupport = new YoBoolean("holdICPToCurrentCoMLocationInNextDoubleSupport", registry);

   private final CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();

   private final YoBoolean icpPlannerDone = new YoBoolean("ICPPlannerDone", registry);
   private final ExecutionTimer plannerTimer = new ExecutionTimer("icpPlannerTimer", registry);

   private final SideDependentList<PlaneContactStateCommand> contactStateCommands = new SideDependentList<>(new PlaneContactStateCommand(),
                                                                                                            new PlaneContactStateCommand());
   private final SideDependentList<? extends ReferenceFrame> soleFrames;

   private final List<Footstep> footsteps = new ArrayList<>();
   private final List<FootstepTiming> footstepTimings = new ArrayList<>();

   private final YoDouble currentStateDuration = new YoDouble("CurrentStateDuration", registry);
   private final YoDouble totalStateDuration = new YoDouble("totalStateDuration", registry);
   private final FootstepTiming currentTiming = new FootstepTiming();
   private final YoDouble timeInSupportSequence = new YoDouble("TimeInSupportSequence", registry);
   private final CoPTrajectoryGeneratorState copTrajectoryState;
   private final WalkingCoPTrajectoryGenerator copTrajectory;
   private final OptimizedCoMTrajectoryPlanner comTrajectoryPlanner;

   public JumpingBalanceManager(JumpingControllerToolbox controllerToolbox,
                                CoPTrajectoryParameters copTrajectoryParameters,
                                YoRegistry parentRegistry)
   {
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();

      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();

      this.controllerToolbox = controllerToolbox;

      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      bipedSupportPolygons = controllerToolbox.getBipedSupportPolygons();

      distanceToShrinkSupportPolygonWhenHoldingCurrent.set(0.08);

      FrameConvexPolygon2D defaultSupportPolygon = controllerToolbox.getDefaultFootPolygons().get(RobotSide.LEFT);
      soleFrames = controllerToolbox.getReferenceFrames().getSoleFrames();
      registry.addChild(copTrajectoryParameters.getRegistry());
      comTrajectoryPlanner = new OptimizedCoMTrajectoryPlanner(controllerToolbox.getGravityZ(), controllerToolbox.getOmega0Provider(), registry);
      copTrajectoryState = new CoPTrajectoryGeneratorState(registry);
      copTrajectoryState.registerStateToSave(copTrajectoryParameters);
      copTrajectory = new WalkingCoPTrajectoryGenerator(copTrajectoryParameters, defaultSupportPolygon, registry);
      copTrajectory.registerState(copTrajectoryState);

      String graphicListName = getClass().getSimpleName();

      if (yoGraphicsListRegistry != null)
      {
         comTrajectoryPlanner.setCornerPointViewer(new CornerPointViewer(true, false, registry, yoGraphicsListRegistry));
//         copTrajectory.setWaypointViewer(new WaypointViewer(registry, yoGraphicsListRegistry));

         YoGraphicPosition desiredCapturePointViz = new YoGraphicPosition("Desired Capture Point", yoDesiredCapturePoint, 0.01, Yellow(), GraphicType.BALL_WITH_ROTATED_CROSS);
         YoGraphicPosition perfectCMPViz = new YoGraphicPosition("Perfect CMP", yoPerfectCMP, 0.002, BlueViolet());

         yoGraphicsListRegistry.registerArtifact(graphicListName, desiredCapturePointViz.createArtifact());
         YoArtifactPosition perfectCMPArtifact = perfectCMPViz.createArtifact();
         perfectCMPArtifact.setVisible(false);
         yoGraphicsListRegistry.registerArtifact(graphicListName, perfectCMPArtifact);
      }
      yoDesiredCapturePoint.setToNaN();
      yoPerfectCMP.setToNaN();

      parentRegistry.addChild(registry);
   }

   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      copTrajectoryState.addFootstep(footstep);
      copTrajectoryState.addFootstepTiming(timing);
      footsteps.add(footstep);
      footstepTimings.add(timing);
   }

   public void clearICPPlan()
   {
      copTrajectoryState.clear();
      footsteps.clear();
      footstepTimings.clear();
   }

   public void compute()
   {
      desiredCapturePoint2d.set(comTrajectoryPlanner.getDesiredDCMPosition());
      desiredCapturePointVelocity2d.set(comTrajectoryPlanner.getDesiredDCMVelocity());
      perfectCoP2d.set(comTrajectoryPlanner.getDesiredECMPPosition());
      desiredCoM2d.set(comTrajectoryPlanner.getDesiredCoMPosition());
      yoDesiredCoMVelocity.set(comTrajectoryPlanner.getDesiredCoMVelocity());

      capturePoint2d.setIncludingFrame(controllerToolbox.getCapturePoint());

      double omega0 = controllerToolbox.getOmega0();
      if (Double.isNaN(omega0))
         throw new RuntimeException("omega0 is NaN");

      yoDesiredCapturePoint.set(desiredCapturePoint2d);
      yoDesiredICPVelocity.set(desiredCapturePointVelocity2d);
      yoDesiredCoMPosition.set(desiredCoM2d);

      CapturePointTools.computeCentroidalMomentumPivot(yoDesiredCapturePoint, yoDesiredICPVelocity, omega0, yoPerfectCMP);

      for (RobotSide robotSide : RobotSide.values)
      {
         YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
         contactState.getPlaneContactStateCommand(contactStateCommands.get(robotSide));
      }

      jumpingMomentumRateControlModuleInput.setOmega0(omega0);
      jumpingMomentumRateControlModuleInput.setTimeInState(timeInSupportSequence.getDoubleValue());
      jumpingMomentumRateControlModuleInput.setVrpTrajectories(comTrajectoryPlanner.getVRPTrajectories());
   }

   public void computeICPPlan()
   {
     computeICPPlanInternal(copTrajectory);
   }

   private void computeICPPlanInternal(CoPTrajectoryGenerator copTrajectory)
   {
      plannerTimer.startMeasurement();

      // update online to account for foot slip
      for (RobotSide robotSide : RobotSide.values)
      {
         if (controllerToolbox.getFootContactState(robotSide).inContact())
            copTrajectoryState.initializeStance(robotSide, bipedSupportPolygons.getFootPolygonsInSoleZUpFrame().get(robotSide), soleFrames.get(robotSide));
      }
      copTrajectory.compute(copTrajectoryState);

      comTrajectoryPlanner.solveForTrajectory(copTrajectory.getContactStateProviders());
      comTrajectoryPlanner.compute(timeInSupportSequence.getDoubleValue());

      // If this condition is false we are experiencing a late touchdown or a delayed liftoff. Do not advance the time in support sequence!
      if (footsteps.isEmpty() || !icpPlannerDone.getValue())
         timeInSupportSequence.add(controllerToolbox.getControlDT());

      icpPlannerDone.set(timeInSupportSequence.getValue() >= currentStateDuration.getValue());

      plannerTimer.stopMeasurement();
   }

   public FramePoint2DReadOnly getDesiredICP()
   {
      return yoDesiredCapturePoint;
   }

   public FrameVector2DReadOnly getDesiredICPVelocity()
   {
      return yoDesiredICPVelocity;
   }

   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return yoDesiredCoMVelocity;
   }

   public double getTimeRemainingInCurrentState()
   {
      if (footstepTimings.isEmpty())
         return 0.0;
      return currentTiming.getStepTime() - timeInSupportSequence.getValue();
   }

   public void initialize()
   {
      yoDesiredCapturePoint.set(controllerToolbox.getCapturePoint());
      yoDesiredCoMPosition.setFromReferenceFrame(controllerToolbox.getCenterOfMassFrame());
      yoDesiredCoMVelocity.setToZero();

      yoPerfectCMP.set(bipedSupportPolygons.getSupportPolygonInWorld().getCentroid());
      copTrajectoryState.setInitialCoP(bipedSupportPolygons.getSupportPolygonInWorld().getCentroid());
      copTrajectoryState.initializeStance(bipedSupportPolygons.getFootPolygonsInSoleZUpFrame(), soleFrames);
      comTrajectoryPlanner.setInitialCenterOfMassState(yoDesiredCoMPosition, yoDesiredCoMVelocity);
      timeInSupportSequence.set(0.0);
      currentStateDuration.set(Double.NaN);
      totalStateDuration.set(Double.NaN);

      comTrajectoryPlanner.setMaintainInitialCoMVelocityContinuity(false);
   }

   public void initializeICPPlanForStanding()
   {
      if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
      {
         requestICPPlannerToHoldCurrentCoM();
         holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
      }
      copTrajectoryState.setInitialCoP(yoPerfectCMP);
      copTrajectoryState.initializeStance(bipedSupportPolygons.getFootPolygonsInSoleZUpFrame(), soleFrames);
      comTrajectoryPlanner.setInitialCenterOfMassState(yoDesiredCoMPosition, yoDesiredCoMVelocity);

      timeInSupportSequence.set(0.0);
      currentStateDuration.set(Double.POSITIVE_INFINITY);
      totalStateDuration.set(Double.POSITIVE_INFINITY);

      comTrajectoryPlanner.setMaintainInitialCoMVelocityContinuity(true);

      icpPlannerDone.set(false);
   }

   public void initializeICPPlanForTransferToStanding()
   {
      if (holdICPToCurrentCoMLocationInNextDoubleSupport.getBooleanValue())
      {
         requestICPPlannerToHoldCurrentCoM();
         holdICPToCurrentCoMLocationInNextDoubleSupport.set(false);
      }
      copTrajectoryState.setInitialCoP(yoPerfectCMP);
      copTrajectoryState.initializeStance(bipedSupportPolygons.getFootPolygonsInSoleZUpFrame(), soleFrames);
      comTrajectoryPlanner.setInitialCenterOfMassState(yoDesiredCoMPosition, yoDesiredCoMVelocity);

      timeInSupportSequence.set(0.0);
      currentStateDuration.set(copTrajectoryState.getFinalTransferDuration());
      totalStateDuration.set(copTrajectoryState.getFinalTransferDuration());

      comTrajectoryPlanner.setMaintainInitialCoMVelocityContinuity(true);

      icpPlannerDone.set(false);
   }


   public boolean isICPPlanDone()
   {
      return icpPlannerDone.getValue();
   }

   public void requestICPPlannerToHoldCurrentCoMInNextDoubleSupport()
   {
      holdICPToCurrentCoMLocationInNextDoubleSupport.set(true);
   }

   public void requestICPPlannerToHoldCurrentCoM()
   {
      centerOfMassPosition.setToZero(centerOfMassFrame);

      FrameConvexPolygon2DReadOnly supportPolygonInMidFeetZUp = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
      convexPolygonShrinker.scaleConvexPolygon(supportPolygonInMidFeetZUp, distanceToShrinkSupportPolygonWhenHoldingCurrent.getDoubleValue(), shrunkSupportPolygon);

      centerOfMassPosition.changeFrame(shrunkSupportPolygon.getReferenceFrame());
      centerOfMassPosition2d.setIncludingFrame(centerOfMassPosition);
      shrunkSupportPolygon.orthogonalProjection(centerOfMassPosition2d);
      centerOfMassPosition.set(centerOfMassPosition2d, 0.0);

      centerOfMassPosition.changeFrame(worldFrame);

      // This tricks it to the current value.
      copTrajectoryState.setInitialCoP(centerOfMassPosition);
   }

   public void setFinalTransferTime(double finalTransferDuration)
   {
      copTrajectoryState.setFinalTransferDuration(finalTransferDuration);
   }

   public CapturabilityBasedStatus updateAndReturnCapturabilityBasedStatus()
   {
      centerOfMassPosition.setToZero(centerOfMassFrame);
      centerOfMassPosition.changeFrame(worldFrame);

      capturabilityBasedStatus.setOmega(controllerToolbox.getOmega0());
      capturabilityBasedStatus.getCapturePoint2d().set(controllerToolbox.getCapturePoint());
      capturabilityBasedStatus.getDesiredCapturePoint2d().set(yoDesiredCapturePoint);
      capturabilityBasedStatus.getCenterOfMass3d().set(centerOfMassPosition);
      for (RobotSide robotSide : RobotSide.values)
      {
         HumanoidMessageTools.packFootSupportPolygon(robotSide, bipedSupportPolygons.getFootPolygonInSoleFrame(robotSide), capturabilityBasedStatus);
      }

      return capturabilityBasedStatus;
   }

   public FramePoint3DReadOnly getCapturePoint()
   {
      return controllerToolbox.getCapturePoint();
   }


   public JumpingMomentumRateControlModuleInput getJumpingMomentumRateControlModuleInput()
   {
      return jumpingMomentumRateControlModuleInput;
   }

}
