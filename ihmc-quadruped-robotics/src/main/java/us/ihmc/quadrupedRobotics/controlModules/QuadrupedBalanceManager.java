package us.ihmc.quadrupedRobotics.controlModules;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Beige;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Blue;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.DarkRed;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Yellow;
import static us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType.BALL_WITH_CROSS;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableInt;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedBodyHeightCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedBodyTrajectoryCommand;
import us.ihmc.quadrupedBasics.gait.QuadrupedStep;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.toolbox.LinearInvertedPendulumModel;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.planning.trajectory.ContinuousDCMPlanner;
import us.ihmc.quadrupedRobotics.planning.trajectory.DCMPlannerInterface;
import us.ihmc.quadrupedRobotics.util.YoQuadrupedTimedStep;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class QuadrupedBalanceManager
{
   private static final boolean debug = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int NUMBER_OF_STEPS_TO_CONSIDER = 8;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoDouble robotTimestamp;

   private final LinearInvertedPendulumModel linearInvertedPendulumModel;

   private final QuadrupedCenterOfMassHeightManager centerOfMassHeightManager;
   private final QuadrupedMomentumRateOfChangeModule momentumRateOfChangeModule;
   private final QuadrupedStepAdjustmentController stepAdjustmentController;
   private final QuadrupedSwingSpeedUpCalculator swingSpeedUpCalculator;

   private final QuadrupedBodyICPBasedTranslationManager bodyICPBasedTranslationManager;

   private final boolean useCustomCoMPlanner;
   private final DCMPlannerInterface comPlanner;
   private final DCMPlannerInterface dcmPlanner;

   private final YoFramePoint3D yoDesiredDCMPosition = new YoFramePoint3D("desiredDCMPosition", worldFrame, registry);
   private final YoFrameVector3D yoDesiredDCMVelocity = new YoFrameVector3D("desiredDCMVelocity", worldFrame, registry);
   private final YoFramePoint3D yoFinalDesiredDCM = new YoFramePoint3D("finalDesiredDCMPosition", worldFrame, registry);
   private final YoFramePoint3D yoVrpPositionSetpoint = new YoFramePoint3D("vrpPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D yoDesiredECMP = new YoFramePoint3D("desiredECMP", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D yoAchievedECMP = new YoFramePoint3D("achievedECMP", worldFrame, registry);
   private final YoFramePoint3D yoPerfectECMP = new YoFramePoint3D("perfectECMP", worldFrame, registry);

   private final YoInteger numberOfStepsToConsider = new YoInteger("numberOfStepsToConsider", registry);

   private final DoubleProvider maxDcmErrorBeforeLiftOffX;
   private final DoubleProvider maxDcmErrorBeforeLiftOffY;

   private final BooleanProvider updateLipmHeightFromDesireds = new BooleanParameter("updateLipmHeightFromDesireds", registry, true);


   private final YoDouble normalizedDcmErrorForDelayedLiftOff = new YoDouble("normalizedDcmErrorForDelayedLiftOff", registry);

   private final YoDouble normalizedDcmErrorForSwingSpeedUp = new YoDouble("normalizedDcmErrorForSpeedUp", registry);
   private final DoubleProvider maxDcmErrorForSpeedUpX = new DoubleParameter("maxDcmErrorForSpeedUpX", registry, 0.08);
   private final DoubleProvider maxDcmErrorForSpeedUpY = new DoubleParameter("maxDcmErrorForSpeedUpY", registry, 0.06);

   private final ReferenceFrame supportFrame;

   private final QuadrupedControllerToolbox controllerToolbox;

   private final RecyclingArrayList<QuadrupedStep> adjustedActiveSteps;

   private final List<QuadrupedTimedStep> activeSteps = new ArrayList<>();

   private final List<QuadrupedTimedStep> stepSequence = new ArrayList<>();

   private final FeetInContactForPlanner feetInContactForPlanner;

   // footstep graphics
   private static final int maxNumberOfFootstepGraphicsPerQuadrant = 4;
   private final FramePoint3D stepSequenceVisualizationPosition = new FramePoint3D();
   private final QuadrantDependentList<BagOfBalls> stepSequenceVisualization = new QuadrantDependentList<>();
   private final QuadrantDependentList<MutableInt> stepVisualizationCounter = new QuadrantDependentList<>(new MutableInt(0), new MutableInt(0),
                                                                                                          new MutableInt(0), new MutableInt(0));
   private static final QuadrantDependentList<AppearanceDefinition> stepSequenceAppearance = new QuadrantDependentList<>(YoAppearance.Red(),
                                                                                                                         YoAppearance.Blue(),
                                                                                                                         YoAppearance.RGBColor(1, 0.5, 0.0),
                                                                                                                         YoAppearance.RGBColor(0.0, 0.5, 1.0));

   private final DoubleProvider durationToAllowEarlyTouchdown = new DoubleParameter("durationToAllowEarlyTouchdown", registry, 5.0e-2);

   public QuadrupedBalanceManager(QuadrupedControllerToolbox controllerToolbox, QuadrupedPhysicalProperties physicalProperties,
                                  YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.controllerToolbox = controllerToolbox;

      feetInContactForPlanner = new FeetInContactForPlanner(controllerToolbox, durationToAllowEarlyTouchdown, registry);

      numberOfStepsToConsider.set(NUMBER_OF_STEPS_TO_CONSIDER);

      QuadrupedReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      QuadrupedRuntimeEnvironment runtimeEnvironment = controllerToolbox.getRuntimeEnvironment();
      supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      robotTimestamp = runtimeEnvironment.getRobotTimestamp();

      ReferenceFrame supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      linearInvertedPendulumModel = controllerToolbox.getLinearInvertedPendulumModel();
      if (runtimeEnvironment.getCoMTrajectoryPlanner() == null)
      {
         comPlanner = null;
         dcmPlanner = new ContinuousDCMPlanner(runtimeEnvironment.getDCMPlannerParameters(), linearInvertedPendulumModel.getLipmHeight(),
                                               runtimeEnvironment.getGravity(), supportFrame, referenceFrames.getSoleFrames(), registry,
                                               yoGraphicsListRegistry);
         useCustomCoMPlanner = false;
      }
      else
      {
         comPlanner = runtimeEnvironment.getCoMTrajectoryPlanner();
         dcmPlanner = null;
         useCustomCoMPlanner = true;
      }

      bodyICPBasedTranslationManager = new QuadrupedBodyICPBasedTranslationManager(controllerToolbox, 0.05, registry);
      maxDcmErrorBeforeLiftOffX = new DoubleParameter("maxDcmErrorBeforeLiftOffX", registry, 0.06);
      maxDcmErrorBeforeLiftOffY = new DoubleParameter("maxDcmErrorBeforeLiftOffY", registry, 0.04);

      centerOfMassHeightManager = new QuadrupedCenterOfMassHeightManager(controllerToolbox, physicalProperties, parentRegistry);
      momentumRateOfChangeModule = new QuadrupedMomentumRateOfChangeModule(controllerToolbox, registry);
      stepAdjustmentController = new QuadrupedStepAdjustmentController(controllerToolbox, registry);
      swingSpeedUpCalculator = new QuadrupedSwingSpeedUpCalculator(controllerToolbox, registry);

      adjustedActiveSteps = new RecyclingArrayList<>(10, QuadrupedStep::new);
      adjustedActiveSteps.clear();

      if (yoGraphicsListRegistry != null)
         setupGraphics(yoGraphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   private void setupGraphics(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      String graphicsListName = getClass().getSimpleName();

      YoGraphicsList graphicsList = new YoGraphicsList(graphicsListName);
      ArtifactList artifactList = new ArtifactList(graphicsListName);

      YoGraphicPosition desiredDCMViz = new YoGraphicPosition("Desired DCM", yoDesiredDCMPosition, 0.01, Yellow(),
                                                              YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition finalDesiredDCMViz = new YoGraphicPosition("Final Desired DCM", yoFinalDesiredDCM, 0.01, Beige(),
                                                                   YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition yoCmpPositionSetpointViz = new YoGraphicPosition("Desired eCMP", yoDesiredECMP, 0.012, YoAppearance.Purple(), BALL_WITH_CROSS);
      YoGraphicPosition yoVRPPositionSetpointViz = new YoGraphicPosition("Desired VRP", yoVrpPositionSetpoint, 0.012, YoAppearance.Purple());
      YoGraphicPosition achievedCMPViz = new YoGraphicPosition("Achieved eCMP", yoAchievedECMP, 0.005, DarkRed(),
                                                               YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition perfectDCMViz = new YoGraphicPosition("Perfect ECMP", yoPerfectECMP, 0.005, Blue(),
                                                            GraphicType.SOLID_BALL);

      graphicsList.add(desiredDCMViz);
      graphicsList.add(perfectDCMViz);
      graphicsList.add(finalDesiredDCMViz);
      graphicsList.add(yoCmpPositionSetpointViz);
      graphicsList.add(yoVRPPositionSetpointViz);

      artifactList.add(perfectDCMViz.createArtifact());
      artifactList.add(desiredDCMViz.createArtifact());
      artifactList.add(finalDesiredDCMViz.createArtifact());
      artifactList.add(yoCmpPositionSetpointViz.createArtifact());
      artifactList.add(achievedCMPViz.createArtifact());

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         AppearanceDefinition appearance = stepSequenceAppearance.get(robotQuadrant);
         String prefix = "timedStepController" + robotQuadrant.getPascalCaseName() + "GoalPositions";
         stepSequenceVisualization
               .set(robotQuadrant, new BagOfBalls(maxNumberOfFootstepGraphicsPerQuadrant, 0.015, prefix, appearance, registry, yoGraphicsListRegistry));
      }

      yoGraphicsListRegistry.registerYoGraphicsList(graphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   private void updateFootstepGraphics(List<? extends QuadrupedTimedStep> steps)
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         stepSequenceVisualization.get(quadrant).hideAll();
         stepVisualizationCounter.get(quadrant).setValue(0);
      }

      for (int i = 0; i < Math.min(steps.size(), numberOfStepsToConsider.getValue()); i++)
      {
         QuadrupedTimedStep step = steps.get(i);
         RobotQuadrant quadrant = step.getRobotQuadrant();
         int count = stepVisualizationCounter.get(quadrant).getValue();
         if (count == maxNumberOfFootstepGraphicsPerQuadrant)
            continue;

         stepSequenceVisualizationPosition.set(step.getReferenceFrame(), step.getGoalPosition());
         stepSequenceVisualization.get(quadrant).setBall(stepSequenceVisualizationPosition, count);
         stepVisualizationCounter.get(quadrant).setValue(count + 1);
      }
   }

   private void adjustActiveFootstepGraphics(List<? extends QuadrupedTimedStep> steps)
   {
      // should be at most two steps
      for (int i = 0; i < steps.size(); i++)
      {
         QuadrupedTimedStep step = steps.get(i);
         RobotQuadrant quadrant = step.getRobotQuadrant();
         stepSequenceVisualizationPosition.set(step.getReferenceFrame(), step.getGoalPosition());
         stepSequenceVisualization.get(quadrant).setBall(stepSequenceVisualizationPosition, 0);
      }
   }

   public void handleBodyHeightCommand(QuadrupedBodyHeightCommand command)
   {
      centerOfMassHeightManager.handleBodyHeightCommand(command);
   }

   public void handleBodyTrajectoryCommand(QuadrupedBodyTrajectoryCommand command)
   {
      bodyICPBasedTranslationManager.handleBodyTrajectoryCommand(command);
      centerOfMassHeightManager.handleBodyTrajectoryCommand(command);
   }

   public void handlePlanarRegionsListCommand(PlanarRegionsListCommand command)
   {
      stepAdjustmentController.handlePlanarRegionsListCommand(command);
   }

   public void clearStepSequence()
   {
      stepSequence.clear();
   }

   public void addStepsToSequence(List<? extends QuadrupedTimedStep> steps)
   {
      for (int i = 0; i < Math.min(steps.size(), numberOfStepsToConsider.getIntegerValue()); i++)
         stepSequence.add(steps.get(i));

      updateFootstepGraphics(steps);

      updateActiveSteps(steps);
      centerOfMassHeightManager.setActiveSteps(activeSteps);
   }

   private void updateActiveSteps(List<? extends QuadrupedTimedStep> steps)
   {
      activeSteps.clear();

      for (int i = 0; i < steps.size(); i++)
      {
         double currentTime = robotTimestamp.getDoubleValue();
         double startTime = steps.get(i).getTimeInterval().getStartTime();
         double endTime = steps.get(i).getTimeInterval().getEndTime();

         if (MathTools.intervalContains(currentTime, startTime, endTime))
         {
            activeSteps.add(steps.get(i));
         }
      }
   }

   public void initialize()
   {
      centerOfMassHeightManager.initialize();

      // update model
      centerOfMassHeightManager.update();
      if (updateLipmHeightFromDesireds.getValue())
         linearInvertedPendulumModel.setLipmHeight(centerOfMassHeightManager.getDesiredHeight(supportFrame));

      yoDesiredDCMPosition.set(controllerToolbox.getDCMPositionEstimate());
      yoDesiredDCMVelocity.setToZero();

      momentumRateOfChangeModule.initialize();
   }

   private final FramePoint3D tempComPosition = new FramePoint3D();
   private final FramePoint3D tempEcmpPosition = new FramePoint3D();
   private final FrameVector3D tempComVelocity = new FrameVector3D();

   public void initializeForStanding()
   {
      centerOfMassHeightManager.initialize();

      // update model
      centerOfMassHeightManager.update();
      if (updateLipmHeightFromDesireds.getValue())
         linearInvertedPendulumModel.setLipmHeight(centerOfMassHeightManager.getDesiredHeight(supportFrame));

      if (useCustomCoMPlanner)
      {
         tempComPosition.setToZero(supportFrame);
         tempComPosition.setZ(centerOfMassHeightManager.getDesiredHeight(supportFrame));
         tempComPosition.changeFrame(worldFrame);
         tempEcmpPosition.setToZero(supportFrame);
         tempEcmpPosition.changeFrame(worldFrame);

         tempComVelocity.setToZero();

         comPlanner.setInitialState(robotTimestamp.getDoubleValue(), tempComPosition, tempComVelocity, tempEcmpPosition);
         comPlanner.computeSetpoints(robotTimestamp.getDoubleValue(), stepSequence, controllerToolbox.getFeetInContact());
      }

      momentumRateOfChangeModule.initialize();
   }

   public void initializeForStepping()
   {
      // update model
      centerOfMassHeightManager.update();
      if (updateLipmHeightFromDesireds.getValue())
         linearInvertedPendulumModel.setLipmHeight(centerOfMassHeightManager.getDesiredHeight(supportFrame));

      DCMPlannerInterface planner;
      if (useCustomCoMPlanner)
      {
         comPlanner.setInitialState(robotTimestamp.getDoubleValue(), comPlanner.getDesiredCoMPosition(), comPlanner.getDesiredCoMVelocity(), yoPerfectECMP);
         planner = comPlanner;
      }
      else
      {
         dcmPlanner.setInitialState(robotTimestamp.getDoubleValue(), yoDesiredDCMPosition, yoDesiredDCMVelocity, yoPerfectECMP);
         planner = dcmPlanner;
      }
      planner.computeSetpoints(robotTimestamp.getDoubleValue(), stepSequence, controllerToolbox.getFeetInContact());
   }

   public void beganStep(RobotQuadrant robotQuadrant, FramePoint3DReadOnly goalPosition, TimeIntervalReadOnly step)
   {
      stepAdjustmentController.beganStep(robotQuadrant, goalPosition);
      feetInContactForPlanner.beganStep(robotQuadrant, step);
      if (useCustomCoMPlanner)
         comPlanner.setInitialState(robotTimestamp.getDoubleValue(), comPlanner.getDesiredCoMPosition(), comPlanner.getDesiredCoMVelocity(), yoPerfectECMP);
      else
         dcmPlanner.setInitialState(robotTimestamp.getDoubleValue(), yoDesiredDCMPosition, yoDesiredDCMVelocity, yoPerfectECMP);
   }

   public void completedStep(RobotQuadrant robotQuadrant)
   {
      if (useCustomCoMPlanner)
         comPlanner.setInitialState(robotTimestamp.getDoubleValue(), comPlanner.getDesiredCoMPosition(), comPlanner.getDesiredCoMVelocity(), yoPerfectECMP);
      else
         dcmPlanner.setInitialState(robotTimestamp.getDoubleValue(), yoDesiredDCMPosition, yoDesiredDCMVelocity, yoPerfectECMP);
      stepAdjustmentController.completedStep(robotQuadrant);
      feetInContactForPlanner.completedStep(robotQuadrant);
   }

   public void setHoldCurrentDesiredPosition(boolean holdPosition)
   {
      if (useCustomCoMPlanner)
         comPlanner.setHoldCurrentDesiredPosition(holdPosition);
      else
         dcmPlanner.setHoldCurrentDesiredPosition(holdPosition);
   }

   public void compute()
   {
      centerOfMassHeightManager.update();
      if (updateLipmHeightFromDesireds.getValue())
         linearInvertedPendulumModel.setLipmHeight(centerOfMassHeightManager.getDesiredHeight(supportFrame));
      feetInContactForPlanner.update();

      DCMPlannerInterface planner;
      if (useCustomCoMPlanner)
         planner = comPlanner;
      else
         planner = dcmPlanner;

      planner.computeSetpoints(robotTimestamp.getDoubleValue(), stepSequence, feetInContactForPlanner.getFeetInContactForPlanner());
      yoFinalDesiredDCM.set(planner.getFinalDCMPosition());

      yoDesiredDCMPosition.set(planner.getDesiredDCMPosition());
      yoDesiredDCMVelocity.set(planner.getDesiredDCMVelocity());
      yoPerfectECMP.set(planner.getDesiredECMPPosition());

      bodyICPBasedTranslationManager.compute(yoDesiredDCMPosition);
      bodyICPBasedTranslationManager.addDCMOffset(yoDesiredDCMPosition);

      if (debug)
         runDebugChecks();

      double desiredCenterOfMassHeightAcceleration = centerOfMassHeightManager.computeDesiredCenterOfMassHeightAcceleration();

      momentumRateOfChangeModule.setDCMEstimate(controllerToolbox.getDCMPositionEstimate());
      momentumRateOfChangeModule.setDCMSetpoints(yoDesiredDCMPosition, yoDesiredDCMVelocity);
      momentumRateOfChangeModule.setDesiredCenterOfMassHeightAcceleration(desiredCenterOfMassHeightAcceleration);
      momentumRateOfChangeModule.compute(yoVrpPositionSetpoint, yoDesiredECMP);
   }

   private void runDebugChecks()
   {
      if (yoDesiredDCMPosition.containsNaN())
         throw new IllegalArgumentException("Desired DCM Position contains NaN");

      if (yoDesiredDCMVelocity.containsNaN())
         throw new IllegalArgumentException("Desired DCM Velocity contains NaN");
   }

   public RecyclingArrayList<QuadrupedStep> computeStepAdjustment(ArrayList<YoQuadrupedTimedStep> activeSteps, boolean stepPlanIsAdjustable)
   {
      RecyclingArrayList<QuadrupedStep> adjustedActiveSteps = stepAdjustmentController
            .computeStepAdjustment(activeSteps, yoDesiredDCMPosition, stepPlanIsAdjustable);

      adjustActiveFootstepGraphics(activeSteps);
      return adjustedActiveSteps;
   }

   public double computeNormalizedEllipticDcmErrorForSpeedUp()
   {
      return computeNormalizedEllipticDcmError(maxDcmErrorForSpeedUpX.getValue(), maxDcmErrorForSpeedUpY.getValue(), normalizedDcmErrorForSwingSpeedUp);
   }

   public double computeNormalizedEllipticDcmErrorForDelayedLiftOff()
   {
      return computeNormalizedEllipticDcmError(maxDcmErrorBeforeLiftOffX.getValue(), maxDcmErrorBeforeLiftOffY.getValue(), normalizedDcmErrorForDelayedLiftOff);
   }

   private double computeNormalizedEllipticDcmError(double maxXError, double maxYError, YoDouble normalizedError)
   {
      FrameVector3DReadOnly dcmError = momentumRateOfChangeModule.getDcmError();
      dcmError.checkReferenceFrameMatch(controllerToolbox.getReferenceFrames().getCenterOfMassZUpFrame());
      normalizedError.set(MathTools.square(dcmError.getX() / maxXError) + MathTools.square(dcmError.getY() / maxYError));

      return normalizedError.getDoubleValue();
   }

   public FrameVector3DReadOnly getDcmError()
   {
      return momentumRateOfChangeModule.getDcmError();
   }

   public FramePoint3DReadOnly getDesiredDcmPosition()
   {
      return yoDesiredDCMPosition;
   }


   public boolean stepHasBeenAdjusted()
   {
      return stepAdjustmentController.stepHasBeenAdjusted();
   }

   private final FramePoint3D perfectCMP = new FramePoint3D();

   public double estimateSwingSpeedUpTimeUnderDisturbance()
   {
      DCMPlannerInterface planner;
      if (useCustomCoMPlanner)
         planner = comPlanner;
      else
         planner = dcmPlanner;
      perfectCMP.setIncludingFrame(planner.getDesiredECMPPosition());
      perfectCMP.changeFrame(worldFrame);
      return swingSpeedUpCalculator.estimateSwingSpeedUpTimeUnderDisturbance(activeSteps, computeNormalizedEllipticDcmErrorForSpeedUp(), yoDesiredDCMPosition,
                                                                             yoFinalDesiredDCM, controllerToolbox.getDCMPositionEstimate(),
                                                                             perfectCMP);
   }

   public void enableBodyXYControl()
   {
      bodyICPBasedTranslationManager.enable();
   }

   public void disableBodyXYControl()
   {
      bodyICPBasedTranslationManager.disable();
   }

   public void computeAchievedCMP(FrameVector3DReadOnly achievedLinearMomentumRate)
   {
      momentumRateOfChangeModule.computeAchievedECMP(achievedLinearMomentumRate, yoAchievedECMP);
   }

   public VirtualModelControlCommand<?> getVirtualModelControlCommand()
   {
      return momentumRateOfChangeModule.getMomentumRateCommand();
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return momentumRateOfChangeModule.getMomentumRateCommand();
   }

   public FrameVector3DReadOnly getStepAdjustment(RobotQuadrant robotQuadrant)
   {
      return stepAdjustmentController.getStepAdjustment(robotQuadrant);
   }
}
