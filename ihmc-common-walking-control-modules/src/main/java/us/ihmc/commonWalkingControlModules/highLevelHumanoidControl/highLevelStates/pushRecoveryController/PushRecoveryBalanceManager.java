package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Beige;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Black;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.BlueViolet;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Yellow;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModuleInput;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModuleOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.PushRecoveryCoPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.PushRecoveryState;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMContinuousContinuityCalculator;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class PushRecoveryBalanceManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean viewCoPHistory = false;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final BipedSupportPolygons bipedSupportPolygons;

   private final LinearMomentumRateControlModuleInput linearMomentumRateControlModuleInput = new LinearMomentumRateControlModuleInput();

   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final YoFramePoint2D yoDesiredCapturePoint = new YoFramePoint2D("desiredRecoveryICP", worldFrame, registry);
   private final YoFrameVector2D yoDesiredICPVelocity = new YoFrameVector2D("desiredRecoveryICPVelocity", worldFrame, registry);
   private final YoFramePoint3D yoDesiredCoMPosition = new YoFramePoint3D("desiredRecoveryCoMPosition", worldFrame, registry);
   private final YoFrameVector3D yoDesiredCoMVelocity = new YoFrameVector3D("desiredRecoveryCoMVelocity", worldFrame, registry);
   private final YoFramePoint2D yoFinalDesiredICP = new YoFramePoint2D("finalDesiredICP", worldFrame, registry);
   private final YoFramePoint3D yoFinalDesiredCoM = new YoFramePoint3D("finalDesiredCoM", worldFrame, registry);
   private final YoFrameVector3D yoFinalDesiredCoMVelocity = new YoFrameVector3D("finalDesiredCoMVelocity", worldFrame, registry);
   private final YoFrameVector3D yoFinalDesiredCoMAcceleration = new YoFrameVector3D("finalDesiredCoMAcceleration", worldFrame, registry);
   private final FrameVector2D icpError2d = new FrameVector2D();

   private final YoDouble normalizedICPError = new YoDouble("normalizedICPError", registry);
   private final DoubleProvider maxICPErrorBeforeSingleSupportForwardX;
   private final DoubleProvider maxICPErrorBeforeSingleSupportBackwardX;
   private final DoubleProvider maxICPErrorBeforeSingleSupportInnerY;
   private final DoubleProvider maxICPErrorBeforeSingleSupportOuterY;

   /** CMP position according to the ICP planner */
   private final YoFramePoint3D yoPerfectCMP = new YoFramePoint3D("perfectCMP", worldFrame, registry);

   private final YoDouble timeAtStartOfState = new YoDouble("timeAtStartOfPushRecoveryState", registry);

   private final BagOfBalls perfectCMPTrajectory;

   private final ReferenceFrame centerOfMassFrame;

   private final FramePoint3D centerOfMassPosition = new FramePoint3D();

   private final FramePoint2D perfectCMP2d = new FramePoint2D();
   private final FramePoint2D perfectCoP2d = new FramePoint2D();

   private final DoubleProvider icpDistanceOutsideSupportForStep = new DoubleParameter("icpDistanceOutsideSupportForStep", registry, 0.03);

   /**
    * Duration parameter used to linearly decrease the desired ICP velocity once the current state is
    * done.
    * <p>
    * This reduction in desired ICP velocity is particularly useful to reduce the ICP tracking error
    * when the robot is getting stuck at the end of transfer.
    * </p>
    */
   private final DoubleParameter icpVelocityDecayDurationWhenDone = new DoubleParameter("ICPVelocityDecayDurationWhenDone", registry, Double.NaN);
   /**
    * Output of the linear reduction being applied on the desired ICP velocity when the current state
    * is done.
    * <p>
    * This reduction in desired ICP velocity is particularly useful to reduce the ICP tracking error
    * when the robot is getting stuck at the end of transfer. true*
    * </p>
    */
   private final YoDouble icpVelocityReductionFactor = new YoDouble("ICPVelocityReductionFactor", registry);

   private final CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();

   private final YoBoolean icpPlannerDone = new YoBoolean("ICPPlannerDone", registry);
   private final ExecutionTimer plannerTimer = new ExecutionTimer("icpPlannerTimer", registry);

   private boolean initializeOnStateChange = false;
   private final boolean minimizeAngularMomentumRateZ = false;
   private final FixedFramePoint2DBasics desiredCMP = new FramePoint2D();
   private final SimpleFootstep currentFootstep = new SimpleFootstep();
   private final SideDependentList<PlaneContactStateCommand> contactStateCommands = new SideDependentList<>(new PlaneContactStateCommand(),
                                                                                                            new PlaneContactStateCommand());
   private final SideDependentList<? extends ReferenceFrame> soleFrames;

   private final List<Footstep> footsteps = new ArrayList<>();
   private final List<FootstepTiming> footstepTimings = new ArrayList<>();

   private final YoBoolean inSingleSupport = new YoBoolean("InSingleSupport", registry);
   private final YoDouble currentStateDuration = new YoDouble("CurrentStateDuration", registry);
   private final YoDouble totalStateDuration = new YoDouble("totalStateDuration", registry);
   private final FootstepTiming currentTiming = new FootstepTiming();
   private final YoDouble timeInSupportSequence = new YoDouble("TimeInSupportSequence", registry);

   private final PushRecoveryState copTrajectoryState;
   private final PushRecoveryCoPTrajectoryGenerator copTrajectory;

   private final CoMTrajectoryPlanner comTrajectoryPlanner;
   private final int maxNumberOfStepsToConsider;
   private final BooleanProvider maintainInitialCoMVelocityContinuitySingleSupport;
   private final BooleanProvider maintainInitialCoMVelocityContinuityTransfer;

   public PushRecoveryBalanceManager(HighLevelHumanoidControllerToolbox controllerToolbox,
                                     PushRecoveryControllerParameters pushRecoveryControllerParameters,
                                     CoPTrajectoryParameters copTrajectoryParameters,
                                     YoRegistry parentRegistry)
   {
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();

      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();

      this.controllerToolbox = controllerToolbox;

      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      bipedSupportPolygons = controllerToolbox.getBipedSupportPolygons();

      FrameConvexPolygon2D defaultSupportPolygon = controllerToolbox.getDefaultFootPolygons().get(RobotSide.LEFT);
      soleFrames = controllerToolbox.getReferenceFrames().getSoleFrames();
      maxNumberOfStepsToConsider = copTrajectoryParameters.getMaxNumberOfStepsToConsider();

      maxICPErrorBeforeSingleSupportForwardX = new DoubleParameter("maxICPErrorBeforeSingleSupportForwardX", registry, pushRecoveryControllerParameters.getMaxICPErrorBeforeSingleSupportForwardX());
      maxICPErrorBeforeSingleSupportBackwardX = new DoubleParameter("maxICPErrorBeforeSingleSupportBackwardX", registry, pushRecoveryControllerParameters.getMaxICPErrorBeforeSingleSupportBackwardX());
      maxICPErrorBeforeSingleSupportInnerY = new DoubleParameter("maxICPErrorBeforeSingleSupportInnerY", registry, pushRecoveryControllerParameters.getMaxICPErrorBeforeSingleSupportInnerY());
      maxICPErrorBeforeSingleSupportOuterY = new DoubleParameter("maxICPErrorBeforeSingleSupportOuterY", registry, pushRecoveryControllerParameters.getMaxICPErrorBeforeSingleSupportOuterY());

      maintainInitialCoMVelocityContinuitySingleSupport = new BooleanParameter("maintainInitialCoMVelocityContinuitySingleSupport", registry, false);
      maintainInitialCoMVelocityContinuityTransfer = new BooleanParameter("maintainInitialCoMVelocityContinuityTransfer", registry, false);

      comTrajectoryPlanner = new CoMTrajectoryPlanner(controllerToolbox.getGravityZ(), controllerToolbox.getOmega0Provider(), registry);
      comTrajectoryPlanner.setComContinuityCalculator(new CoMContinuousContinuityCalculator(controllerToolbox.getGravityZ(), controllerToolbox.getOmega0Provider(), registry));

      copTrajectoryState = new PushRecoveryState(registry, maxNumberOfStepsToConsider);
      copTrajectory = new PushRecoveryCoPTrajectoryGenerator(defaultSupportPolygon, registry);
      copTrajectory.registerState(copTrajectoryState);

      String graphicListName = getClass().getSimpleName();

      if (yoGraphicsListRegistry != null)
      {
         if (viewCoPHistory)
         {
            perfectCMPTrajectory = new BagOfBalls(150, 0.002, "perfectCMP", BlueViolet(), GraphicType.BALL, registry, yoGraphicsListRegistry);
         }
         else
         {
            perfectCMPTrajectory = null;
         }

//         comTrajectoryPlanner.setCornerPointViewer(new CornerPointViewer(true, false, registry, yoGraphicsListRegistry));
//         copTrajectory.setWaypointViewer(new CoPPointViewer(registry, yoGraphicsListRegistry));

         YoGraphicPosition desiredCapturePointViz = new YoGraphicPosition("Desired Recovery Capture Point", yoDesiredCapturePoint, 0.01, Yellow(), GraphicType.BALL_WITH_ROTATED_CROSS);
         YoGraphicPosition finalDesiredCapturePointViz = new YoGraphicPosition("Final Desired Recovery Capture Point", yoFinalDesiredICP, 0.01, Beige(), GraphicType.BALL_WITH_ROTATED_CROSS);
         YoGraphicPosition finalDesiredCoMViz = new YoGraphicPosition("Final Desired Recovery CoM", yoFinalDesiredCoM, 0.01, Black(), GraphicType.BALL_WITH_ROTATED_CROSS);
         YoGraphicPosition perfectCMPViz = new YoGraphicPosition("Perfect Recovery CMP", yoPerfectCMP, 0.002, BlueViolet());

         yoGraphicsListRegistry.registerArtifact(graphicListName, desiredCapturePointViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact(graphicListName, finalDesiredCapturePointViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact(graphicListName, finalDesiredCoMViz.createArtifact());
         YoArtifactPosition perfectCMPArtifact = perfectCMPViz.createArtifact();
         perfectCMPArtifact.setVisible(false);
         yoGraphicsListRegistry.registerArtifact(graphicListName, perfectCMPArtifact);
      }
      else
      {
         perfectCMPTrajectory = null;
      }
      yoDesiredCapturePoint.setToNaN();
      yoFinalDesiredICP.setToNaN();
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

   public void compute(FeedbackControlCommand<?> heightControlCommand, boolean keepCoPInsideSupportPolygon, boolean controlHeightWithMomentum)
   {
      yoDesiredCapturePoint.set(comTrajectoryPlanner.getDesiredDCMPosition());
      yoDesiredICPVelocity.set(comTrajectoryPlanner.getDesiredDCMVelocity());
      if (!icpVelocityReductionFactor.isNaN())
         yoDesiredICPVelocity.scale(icpVelocityReductionFactor.getValue());
      yoPerfectCMP.set(comTrajectoryPlanner.getDesiredECMPPosition());

      yoDesiredCoMPosition.set(comTrajectoryPlanner.getDesiredCoMPosition());
      yoDesiredCoMVelocity.set(comTrajectoryPlanner.getDesiredCoMVelocity());

      double omega0 = controllerToolbox.getOmega0();
      if (Double.isNaN(omega0))
         throw new RuntimeException("omega0 is NaN");

      for (RobotSide robotSide : RobotSide.values)
      {
         YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
         contactState.getPlaneContactStateCommand(contactStateCommands.get(robotSide));
      }

      if (heightControlCommand.getCommandType() == ControllerCoreCommandType.POINT)
      {
         linearMomentumRateControlModuleInput.setUsePelvisHeightCommand(true);
         linearMomentumRateControlModuleInput.setPelvisHeightControlCommand((PointFeedbackControlCommand) heightControlCommand);
      }
      else if (heightControlCommand.getCommandType() == ControllerCoreCommandType.MOMENTUM)
      {
         linearMomentumRateControlModuleInput.setUsePelvisHeightCommand(false);
         linearMomentumRateControlModuleInput.setCenterOfMassHeightControlCommand((CenterOfMassFeedbackControlCommand) heightControlCommand);
      }
      else
      {
         throw new IllegalArgumentException("Invalid height control type.");
      }

      if (perfectCMPTrajectory != null)
      {
         perfectCMPTrajectory.setBallLoop(yoPerfectCMP);
      }

      perfectCMP2d.setIncludingFrame(yoPerfectCMP);
      perfectCoP2d.setIncludingFrame(yoPerfectCMP);
      linearMomentumRateControlModuleInput.setInitializeOnStateChange(initializeOnStateChange);
      linearMomentumRateControlModuleInput.setKeepCoPInsideSupportPolygon(keepCoPInsideSupportPolygon);
      linearMomentumRateControlModuleInput.setControlHeightWithMomentum(controlHeightWithMomentum);
      linearMomentumRateControlModuleInput.setOmega0(omega0);
      linearMomentumRateControlModuleInput.setUseMomentumRecoveryMode(true); // TODO
      linearMomentumRateControlModuleInput.setDesiredCapturePoint(yoDesiredCapturePoint);
      linearMomentumRateControlModuleInput.setDesiredCapturePointVelocity(yoDesiredICPVelocity);
      linearMomentumRateControlModuleInput.setDesiredCapturePointAtEndOfState(yoFinalDesiredICP);
      linearMomentumRateControlModuleInput.setPerfectCMP(perfectCMP2d);
      linearMomentumRateControlModuleInput.setPerfectCoP(perfectCoP2d);
      linearMomentumRateControlModuleInput.setMinimizeAngularMomentumRateZ(minimizeAngularMomentumRateZ);
      linearMomentumRateControlModuleInput.setContactStateCommand(contactStateCommands);

      initializeOnStateChange = false;
   }

   public void adjustFootstep(Footstep footstep)
   {
      copTrajectoryState.getFootstep(0).set(footstep);
   }

   public void computeICPPlan()
   {
      plannerTimer.startMeasurement();

      // update online to account for foot slip
      for (RobotSide robotSide : RobotSide.values)
      {
         if (controllerToolbox.getFootContactState(robotSide).inContact())
            copTrajectoryState.initializeStance(robotSide, bipedSupportPolygons.getFootPolygonInSoleFrame(robotSide), soleFrames.get(robotSide));
      }

      copTrajectory.compute(copTrajectoryState);

      List<SettableContactStateProvider> contactStateProviders = copTrajectory.getContactStateProviders();
      comTrajectoryPlanner.reset();

      comTrajectoryPlanner.solveForTrajectory(contactStateProviders);
      comTrajectoryPlanner.compute(totalStateDuration.getDoubleValue());

      yoFinalDesiredCoM.set(comTrajectoryPlanner.getDesiredCoMPosition());
      yoFinalDesiredCoMVelocity.set(comTrajectoryPlanner.getDesiredCoMVelocity());
      yoFinalDesiredCoMAcceleration.set(comTrajectoryPlanner.getDesiredCoMAcceleration());
      yoFinalDesiredICP.set(comTrajectoryPlanner.getDesiredDCMPosition());

      timeInSupportSequence.set(controllerToolbox.getYoTime().getValue() - timeAtStartOfState.getDoubleValue());
      icpPlannerDone.set(timeInSupportSequence.getValue() >= currentStateDuration.getValue());

      if (icpPlannerDone.getValue())
         timeInSupportSequence.set(currentStateDuration.getDoubleValue());

      comTrajectoryPlanner.compute(timeInSupportSequence.getDoubleValue());

      if (footstepTimings.isEmpty())
      {
         yoFinalDesiredICP.setToNaN();
         yoFinalDesiredCoM.setToNaN();
         yoFinalDesiredCoMVelocity.setToNaN();
         yoFinalDesiredCoMAcceleration.setToNaN();
      }

      decayDesiredICPVelocity();

      plannerTimer.stopMeasurement();
   }

   /**
    * Time-based decay of the desired ICP velocity activated when a double support is going over the
    * planner state duration.
    */
   private void decayDesiredICPVelocity()
   {
      if (Double.isNaN(icpVelocityDecayDurationWhenDone.getValue()))
      {
         icpVelocityReductionFactor.set(Double.NaN);
         return;
      }

      if (inSingleSupport.getValue() || !icpPlannerDone.getValue())
      {
         icpVelocityReductionFactor.set(Double.NaN);
         return;
      }

      if (icpVelocityReductionFactor.isNaN())
      {
         icpVelocityReductionFactor.set(1.0);
         return;
      }

      double scaleUpdated = icpVelocityReductionFactor.getValue() - controllerToolbox.getControlDT() / icpVelocityDecayDurationWhenDone.getValue();
      icpVelocityReductionFactor.set(Math.max(0.0, scaleUpdated));
   }

   public int getMaxNumberOfStepsToConsider()
   {
      return maxNumberOfStepsToConsider;
   }

   public FramePoint2DReadOnly getDesiredCMP()
   {
      return desiredCMP;
   }

   public FramePoint2DReadOnly getFinalDesiredICP()
   {
      return yoFinalDesiredICP;
   }

   public FramePoint2DReadOnly getDesiredICP()
   {
      return yoDesiredCapturePoint;
   }

   public FrameVector2DReadOnly getDesiredICPVelocity()
   {
      return yoDesiredICPVelocity;
   }

   public FramePoint3DReadOnly getFinalDesiredCoMPosition()
   {
      return yoFinalDesiredCoM;
   }

   public FrameVector3DReadOnly getFinalDesiredCoMVelocity()
   {
      return yoFinalDesiredCoMVelocity;
   }
   
   public FrameVector3DReadOnly getFinalDesiredCoMAcceleration()
   {
      return yoFinalDesiredCoMAcceleration;
   }

   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return yoDesiredCoMVelocity;
   }

   public FramePoint3DReadOnly getFirstExitCMPForToeOff(boolean isInTransfer)
   {
      if (isInTransfer)
         return copTrajectory.getContactStateProviders().get(0).getECMPStartPosition();
      else
         return copTrajectory.getContactStateProviders().get(4).getECMPEndPosition();
   }

   public double getTimeRemainingInCurrentState()
   {
      if (footstepTimings.isEmpty())
         return 0.0;
      return currentTiming.getStepTime() - timeInSupportSequence.getValue();
   }

   public void initialize()
   {
      yoFinalDesiredICP.setToNaN();
      yoFinalDesiredCoM.setToNaN();
      yoFinalDesiredCoMVelocity.setToNaN();
      yoFinalDesiredCoMAcceleration.setToNaN();
      yoDesiredCapturePoint.set(controllerToolbox.getCapturePoint());

      yoDesiredCoMPosition.setFromReferenceFrame(controllerToolbox.getCenterOfMassFrame());
      yoDesiredCoMVelocity.set(controllerToolbox.getCenterOfMassVelocity());

      copTrajectoryState.setIcpAtStartOfState(controllerToolbox.getCapturePoint());
      copTrajectoryState.initializeStance(bipedSupportPolygons.getFootPolygonsInSoleZUpFrame(), soleFrames);
      comTrajectoryPlanner.setInitialCenterOfMassState(yoDesiredCoMPosition, yoDesiredCoMVelocity);

      timeAtStartOfState.set(controllerToolbox.getYoTime().getValue());
      timeInSupportSequence.set(0.0);

      inSingleSupport.set(false);
      currentStateDuration.set(Double.NaN);
      totalStateDuration.set(Double.NaN);

      comTrajectoryPlanner.setMaintainInitialCoMVelocityContinuity(false);

      initializeOnStateChange = true;
      comTrajectoryPlanner.reset();
   }

   public void initializeICPPlanForSingleSupport()
   {
      inSingleSupport.set(true);
      currentTiming.set(footstepTimings.get(0));
      currentFootstep.set(footsteps.get(0));

      copTrajectoryState.setIcpAtStartOfState(controllerToolbox.getCapturePoint());
      copTrajectoryState.initializeStance(bipedSupportPolygons.getFootPolygonsInSoleZUpFrame(), soleFrames);
      comTrajectoryPlanner.setInitialCenterOfMassState(yoDesiredCoMPosition, yoDesiredCoMVelocity);

      timeAtStartOfState.set(controllerToolbox.getYoTime().getValue());

      currentStateDuration.set(currentTiming.getSwingTime());
      totalStateDuration.set(currentTiming.getStepTime());

      comTrajectoryPlanner.setMaintainInitialCoMVelocityContinuity(maintainInitialCoMVelocityContinuitySingleSupport.getValue());
      initializeOnStateChange = true;
      icpPlannerDone.set(false);
   }

   public void initializeICPPlanForTransfer()
   {
      comTrajectoryPlanner.removeCompletedSegments(totalStateDuration.getDoubleValue());

      copTrajectoryState.setIcpAtStartOfState(controllerToolbox.getCapturePoint());
      copTrajectoryState.initializeStance(bipedSupportPolygons.getFootPolygonsInSoleZUpFrame(), soleFrames);
      comTrajectoryPlanner.setInitialCenterOfMassState(yoDesiredCoMPosition, yoDesiredCoMVelocity);

      currentStateDuration.set(currentTiming.getStepTime());
      totalStateDuration.set(currentTiming.getStepTime());

      inSingleSupport.set(false);
      comTrajectoryPlanner.setMaintainInitialCoMVelocityContinuity(maintainInitialCoMVelocityContinuityTransfer.getValue());

      initializeOnStateChange = true;
      icpPlannerDone.set(false);
   }

   public void initializeICPPlanForTransferToStanding()
   {
      comTrajectoryPlanner.removeCompletedSegments(totalStateDuration.getDoubleValue());

      copTrajectoryState.setIcpAtStartOfState(controllerToolbox.getCapturePoint());
      copTrajectoryState.initializeStance(bipedSupportPolygons.getFootPolygonsInSoleZUpFrame(), soleFrames);
      comTrajectoryPlanner.setInitialCenterOfMassState(yoDesiredCoMPosition, yoDesiredCoMVelocity);

      timeAtStartOfState.set(controllerToolbox.getYoTime().getValue());

      timeInSupportSequence.set(0.0);
      currentStateDuration.set(copTrajectoryState.getFinalTransferDuration());
      totalStateDuration.set(copTrajectoryState.getFinalTransferDuration());

      inSingleSupport.set(false);
      initializeOnStateChange = true;
      comTrajectoryPlanner.setMaintainInitialCoMVelocityContinuity(true);

      icpPlannerDone.set(false);
   }

   public double getICPDistanceOutsideSupportForStep()
   {
      return icpDistanceOutsideSupportForStep.getValue();
   }

   public double getICPErrorMagnitude()
   {
      return controllerToolbox.getCapturePoint().distanceXY(yoDesiredCapturePoint);
   }

   public void computeNormalizedEllipticICPError(RobotSide transferToSide)
   {
      getICPError(icpError2d);
      ReferenceFrame leadingSoleZUpFrame = controllerToolbox.getReferenceFrames().getSoleZUpFrame(transferToSide);
      icpError2d.changeFrame(leadingSoleZUpFrame);
      boolean isICPErrorToTheInside = transferToSide == RobotSide.RIGHT ? icpError2d.getY() > 0.0 : icpError2d.getY() < 0.0;
      double maxICPErrorBeforeSingleSupportX = icpError2d.getX() > 0.0 ? maxICPErrorBeforeSingleSupportForwardX.getValue() : maxICPErrorBeforeSingleSupportBackwardX.getValue();
      double maxICPErrorBeforeSingleSupportY = isICPErrorToTheInside ? maxICPErrorBeforeSingleSupportInnerY.getValue() : maxICPErrorBeforeSingleSupportOuterY.getValue();
      normalizedICPError.set(MathTools.square(icpError2d.getX() / maxICPErrorBeforeSingleSupportX) + MathTools.square(icpError2d.getY() / maxICPErrorBeforeSingleSupportY));
   }

   public double getNormalizedEllipticICPError()
   {
      return normalizedICPError.getValue();
   }

   public void getICPError(FrameVector2D icpErrorToPack)
   {
      icpErrorToPack.setIncludingFrame(yoDesiredCapturePoint);
      icpErrorToPack.checkReferenceFrameMatch(controllerToolbox.getCapturePoint());
      icpErrorToPack.sub(controllerToolbox.getCapturePoint().getX(), controllerToolbox.getCapturePoint().getY());
   }

   public boolean isICPPlanDone()
   {
      return icpPlannerDone.getValue();
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

   public LinearMomentumRateControlModuleInput getLinearMomentumRateControlModuleInput()
   {
      return linearMomentumRateControlModuleInput;
   }

   public void setLinearMomentumRateControlModuleOutput(LinearMomentumRateControlModuleOutput output)
   {
      desiredCMP.set(output.getDesiredCMP());
   }
}
