package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.quicksterFootstepProvider;

import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.*;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingZUpFrame;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class QuicksterFootstepProvider
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final String variableNameSuffix = "QFP";

   private final FullHumanoidRobotModel robotModel;
   private final CommonHumanoidReferenceFrames referenceFrames;

   private final double updateDT;
   private final double gravity = 9.81;

   // Parameters
   private final QuicksterFootstepProviderParameters parameters;
   private boolean useAlternateTransferDuration = false;
   private double alternateTransferDuration = 0.01;

   // Estimates
   private final QuicksterFootstepProviderEstimates estimates;
   private final MovingZUpFrame centerOfMassControlZUpFrame;

   // Command/Desired output related stuff
   private final SideDependentList<QuicksterFootstepProviderTouchdownCalculator> touchdownCalculator = new SideDependentList<>();
   private final SideDependentList<RecyclingArrayList<FramePoint2D>> desiredTouchdownPositionsList = new SideDependentList<>();
   private final SideDependentList<RecyclingArrayList<FramePose2D>> desiredTouchdownPosesList = new SideDependentList<>();
   private final SideDependentList<YoFramePoint3D> desiredTouchdownPosition3DInWorld = new SideDependentList<>();

   // Desired inputs
   private final YoDouble desiredTurningVelocity = new YoDouble("desiredTurningVelocity" + variableNameSuffix, registry);
   private final YoVector2D desiredVelocity = new YoVector2D("desiredVelocity" + variableNameSuffix, registry);
   private final YoBoolean walk = new YoBoolean("walk" + variableNameSuffix, registry);
   private final YoFrameQuaternion desiredPelvisOrientation;
   private final FrameQuaternion chestOrientation = new FrameQuaternion();

   // Foot state information
   public enum FootState
   {SUPPORT, SWING}

   private final SideDependentList<YoEnum<FootState>> footStates = new SideDependentList<>();
   private final SideDependentList<StateMachine<FootState, State>> footStateMachines = new SideDependentList<>();
   private final YoEnum<RobotSide> newestSupportSide = new YoEnum<>("newestSupportSide" + variableNameSuffix, registry, RobotSide.class);
   private final YoBoolean inDoubleSupport = new YoBoolean("inDoubleSupport" + variableNameSuffix, registry);
   private RobotSide trailingSide = RobotSide.LEFT;

   // Pendulum base information
   private final SideDependentList<FramePoint2D> pendulumBase = new SideDependentList<>();
   private final FramePoint2D netPendulumBase = new FramePoint2D();
   private final SideDependentList<YoFramePoint3D> pendulumBase3DInWorld = new SideDependentList<>();
   private final YoFramePoint3D netPendulumBase3DInWorld = new YoFramePoint3D("netPendulumBase3DInWorld" + variableNameSuffix,
                                                                              ReferenceFrame.getWorldFrame(),
                                                                              registry);

   // Temp variables for changing frames and stuff
   private final FramePoint2DBasics tempPendulumBase = new FramePoint2D();
   private final FramePoint2DBasics tempNetPendulumBase = new FramePoint2D();
   private final FramePoint2D tempTouchdownPosition = new FramePoint2D();

   // Inputs
   private final static Vector2DReadOnly zero2D = new Vector2D();
   private BooleanProvider walkInputProvider;
   private DesiredVelocityProvider desiredVelocityProvider = () -> zero2D;
   private DesiredTurningVelocityProvider desiredTurningVelocityProvider = () -> 0.0;

   // ALIP footstep calculator
   private final ALIPCalculatorTools alipCalculatorTools = new ALIPCalculatorTools();

   // Visualizers
   private final YoFramePoint3D nextStanceFootPosition = new YoFramePoint3D("nextStanceFootPositionQFP", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D nextCoMPosition = new YoFramePoint3D("nextCoMPositionQFP", ReferenceFrame.getWorldFrame(), registry);

   public QuicksterFootstepProvider(FullHumanoidRobotModel robotModel,
                                    CommonHumanoidReferenceFrames referenceFrames,
                                    double updateDT,
                                    YoRegistry parentRegistry,
                                    YoGraphicsListRegistry yoGraphicsListRegistry,
                                    DoubleProvider yoTime)
   {
      this.robotModel = robotModel;
      this.referenceFrames = referenceFrames;
      this.updateDT = updateDT;

      desiredPelvisOrientation = new YoFrameQuaternion("pelvisDesiredOrientation" + variableNameSuffix, ReferenceFrame.getWorldFrame(), registry);

      // Parameters
      parameters = new QuicksterFootstepProviderParameters(gravity, registry);

      // Estimates
      estimates = new QuicksterFootstepProviderEstimates(robotModel,
                                                         referenceFrames,
                                                         desiredPelvisOrientation,
                                                         parameters,
                                                         updateDT,
                                                         variableNameSuffix,
                                                         registry,
                                                         yoGraphicsListRegistry);

      centerOfMassControlZUpFrame = estimates.getCenterOfMassControlZUpFrame();

      // Side-dependant stuff, most of the desired-related things
      for (RobotSide robotSide : RobotSide.values)
      {
         footStates.put(robotSide, new YoEnum<>(robotSide.getLowerCaseName() + "FootStates" + variableNameSuffix, registry, FootState.class));

         pendulumBase.put(robotSide, new FramePoint2D(centerOfMassControlZUpFrame));

         pendulumBase3DInWorld.put(robotSide,
                                   new YoFramePoint3D(robotSide.getLowerCaseName() + "PendulumBase3DInWorld" + variableNameSuffix,
                                                      ReferenceFrame.getWorldFrame(),
                                                      registry));

         touchdownCalculator.put(robotSide,
                                 new QuicksterFootstepProviderTouchdownCalculator(robotSide,
                                                                                  centerOfMassControlZUpFrame,
                                                                                  estimates.getCenterOfMassVelocity(),
                                                                                  estimates.getCenterOfMassAngularMomentum(),
                                                                                  robotModel,
                                                                                  parameters,
                                                                                  gravity,
                                                                                  desiredVelocity,
                                                                                  registry));

         desiredTouchdownPositionsList.put(robotSide, new RecyclingArrayList<>(FramePoint2D.class));
         desiredTouchdownPosesList.put(robotSide, new RecyclingArrayList<>(FramePose2D.class));

         desiredTouchdownPosition3DInWorld.put(robotSide,
                                               new YoFramePoint3D(robotSide.getLowerCaseName() + "DesiredTouchdownPosition3DInWorld" + variableNameSuffix,
                                                                  ReferenceFrame.getWorldFrame(),
                                                                  registry));

         StateMachineFactory<FootState, State> stateMachineFactory = new StateMachineFactory<>(FootState.class);
         stateMachineFactory.setRegistry(registry).setNamePrefix(robotSide.getLowerCaseName() + variableNameSuffix).buildYoClock(yoTime);

         stateMachineFactory.addState(FootState.SUPPORT, new SupportFootState(robotSide));
         stateMachineFactory.addState(FootState.SWING, new SwingFootState(robotSide));

         stateMachineFactory.addDoneTransition(FootState.SUPPORT, FootState.SWING);

         footStateMachines.put(robotSide, stateMachineFactory.build(FootState.SUPPORT));
//         footStateMachines.get(robotSide).resetToInitialState();

         if (yoGraphicsListRegistry != null)
         {
            AppearanceDefinition pendulumBaseVizColor = robotSide == RobotSide.LEFT ? YoAppearance.Magenta() : YoAppearance.Green();
            YoGraphicPosition pendulumBaseViz = new YoGraphicPosition(robotSide.getLowerCaseName() + "PendulumBase" + variableNameSuffix,
                                                                      pendulumBase3DInWorld.get(robotSide),
                                                                      0.015,
                                                                      pendulumBaseVizColor,
                                                                      YoGraphicPosition.GraphicType.SQUARE);

            yoGraphicsListRegistry.registerYoGraphic(variableNameSuffix, pendulumBaseViz);
            yoGraphicsListRegistry.registerArtifact(variableNameSuffix, pendulumBaseViz.createArtifact());

            AppearanceDefinition touchdownVizColor = robotSide == RobotSide.LEFT ? YoAppearance.Magenta() : YoAppearance.Green();
            YoGraphicPosition desiredTouchdownPositionViz = new YoGraphicPosition(
                  robotSide.getLowerCaseName() + "DesiredTouchdownPosition" + variableNameSuffix,
                  desiredTouchdownPosition3DInWorld.get(robotSide),
                  0.015,
                  touchdownVizColor,
                  YoGraphicPosition.GraphicType.DIAMOND_WITH_CROSS);

            yoGraphicsListRegistry.registerYoGraphic(variableNameSuffix, desiredTouchdownPositionViz);
            yoGraphicsListRegistry.registerArtifact(variableNameSuffix, desiredTouchdownPositionViz.createArtifact());
         }
      }

      if (yoGraphicsListRegistry != null)
      {
         YoGraphicPosition nextStanceFootGraphic = new YoGraphicPosition("nextStanceFootGraphic" + variableNameSuffix,
               nextStanceFootPosition,
               0.09,
               YoAppearance.Purple());
         yoGraphicsListRegistry.registerYoGraphic(variableNameSuffix, nextStanceFootGraphic);
         yoGraphicsListRegistry.registerArtifact(variableNameSuffix, nextStanceFootGraphic.createArtifact());

         YoGraphicPosition nextCoMGraphic = new YoGraphicPosition("nextCoMGraphic" + variableNameSuffix,
               nextCoMPosition,
               0.09,
               YoAppearance.Red());
         yoGraphicsListRegistry.registerYoGraphic(variableNameSuffix, nextCoMGraphic);
         yoGraphicsListRegistry.registerArtifact(variableNameSuffix, nextCoMGraphic.createArtifact());

         YoGraphicPosition netPendulumBaseViz = new YoGraphicPosition("netPendulumBase" + variableNameSuffix,
                                                                      netPendulumBase3DInWorld,
                                                                      0.015,
                                                                      YoAppearance.Red(),
                                                                      YoGraphicPosition.GraphicType.SQUARE);
         yoGraphicsListRegistry.registerYoGraphic(variableNameSuffix, netPendulumBaseViz);
         yoGraphicsListRegistry.registerArtifact(variableNameSuffix, netPendulumBaseViz.createArtifact());
      }

      parentRegistry.addChild(registry);
   }

   private boolean firstTick = true;

   public void initialize()
   {
      chestOrientation.setToZero(robotModel.getPelvis().getBodyFixedFrame());
      chestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      desiredPelvisOrientation.setToZero();
      desiredPelvisOrientation.setToYawOrientation(chestOrientation.getYaw());

      firstTick = false;
   }

   private int numberOfFootstepsToPlan;

   public void update(int numberOfFootstepsToPlan)
   {
      if (firstTick)
         initialize();

      this.numberOfFootstepsToPlan = numberOfFootstepsToPlan;

      updateEstimates();

      updateDesireds();
   }

   private void updateEstimates()
   {
      initialize();
      desiredPelvisOrientation.appendYawRotation(desiredTurningVelocity.getDoubleValue() * 1 * updateDT);

      inDoubleSupport.set(footStates.get(RobotSide.LEFT).getEnumValue() == FootState.SUPPORT &&
                          footStates.get(RobotSide.RIGHT).getEnumValue() == FootState.SUPPORT);

      calculateNetPendulumBase();

      estimates.update(getTrailingSide().getOppositeSide());
   }

   private void updateDesireds()
   {
      handleDesiredsFromProviders();

      for (RobotSide robotSide : RobotSide.values)
      {
         footStateMachines.get(robotSide).doActionAndTransition();

         pendulumBase3DInWorld.get(robotSide).setMatchingFrame(pendulumBase.get(robotSide), 0.0);
      }

      netPendulumBase3DInWorld.setMatchingFrame(netPendulumBase, 0.0);
   }

   private final FramePose3D tempCurrentCoMPose = new FramePose3D();
   private final FrameVector3D tempCurrentContactPointAngularMomentum = new FrameVector3D();
   private final FramePoint3D tempCurrentStanceFootPosition = new FramePoint3D();
   private final PoseReferenceFrame tempControlFrame = new PoseReferenceFrame("tempControlFrameQFP", ReferenceFrame.getWorldFrame());

   public void calculateTouchdownPosition(RobotSide currentSwingSide, double timeRemainingInCurrentStep, boolean inDoubleSupport)
   {
      desiredTouchdownPositionsList.get(currentSwingSide).clear();
      desiredTouchdownPositionsList.get(currentSwingSide.getOppositeSide()).clear();
      desiredTouchdownPosesList.get(currentSwingSide).clear();
      desiredTouchdownPosesList.get(currentSwingSide.getOppositeSide()).clear();

      for (int i = 0; i < numberOfFootstepsToPlan; i++)
      {
         double timeToReachGoal;
         double stepDuration;
         double transferDuration;
         RobotSide swingSide;
         boolean useFutureCoM;

         if (i == 0)
         {
            tempCurrentCoMPose.setFromReferenceFrame(centerOfMassControlZUpFrame);
            tempCurrentContactPointAngularMomentum.setMatchingFrame(estimates.getContactPointAngularMomentum());
            tempCurrentStanceFootPosition.setMatchingFrame(netPendulumBase3DInWorld);//setFromReferenceFrame(referenceFrames.getSoleZUpFrame(currentSwingSide.getOppositeSide()));//
            tempControlFrame.setTransformAndUpdate(centerOfMassControlZUpFrame.getTransformToDesiredFrame(tempControlFrame.getParent()));

            swingSide = currentSwingSide;
            timeToReachGoal = timeRemainingInCurrentStep;
            stepDuration = useAlternateTransferDuration ? getSwingDuration(swingSide) + getAlternateTransferDuration() : getStepDuration(swingSide);
            transferDuration = useAlternateTransferDuration ? getAlternateTransferDuration() : getTransferDuration(swingSide);
            useFutureCoM = false;
         }
         else if (i % 2 == 0)
         {
            swingSide = currentSwingSide;
            timeToReachGoal = getStepDuration(swingSide);
            stepDuration = getStepDuration(swingSide);
            transferDuration = getTransferDuration(swingSide);
            useFutureCoM = true;
         }
         else
         {
            swingSide = currentSwingSide.getOppositeSide();
            timeToReachGoal = getStepDuration(swingSide);
            stepDuration = getStepDuration(swingSide);
            transferDuration = getTransferDuration(swingSide);
            useFutureCoM = true;
         }

         FramePoint2D desiredTouchdownPositionToPack = desiredTouchdownPositionsList.get(swingSide).add();
         FramePose2D desiredTouchdownPose = desiredTouchdownPosesList.get(swingSide).add();

         alipCalculatorTools.computeTouchdownPositionUsingRaibertHeuristicAndPolePlacement(tempCurrentCoMPose,
                                                                                           tempCurrentContactPointAngularMomentum,
                                                                                           tempCurrentStanceFootPosition,
                                                                                           desiredTouchdownPositionToPack,
                                                                                           swingSide,
                                                                                           tempControlFrame,
                                                                                           desiredVelocity.getX(),
                                                                                           desiredVelocity.getY(),
                                                                                           desiredTurningVelocity.getDoubleValue(),
                                                                                           parameters.getStanceWidth(swingSide).getDoubleValue(),
                                                                                           timeToReachGoal,
                                                                                           stepDuration,
                                                                                           transferDuration,
                                                                                           robotModel.getTotalMass(),
                                                                                           parameters.getDesiredCoMHeight(swingSide).getDoubleValue(),
                                                                                           parameters.getPole(swingSide).getDoubleValue(),
                                                                                           useFutureCoM,
                                                                                           updateDT);
         alipCalculatorTools.computeFutureStateUsingALIP(tempCurrentCoMPose,
                                                         tempCurrentContactPointAngularMomentum,
                                                         tempCurrentStanceFootPosition,
                                                         tempCurrentCoMPose,
                                                         tempCurrentContactPointAngularMomentum,
                                                         tempControlFrame,
                                                         timeToReachGoal,
                                                         robotModel.getTotalMass(),
                                                         parameters.getDesiredCoMHeight(swingSide).getDoubleValue(),
                                                         desiredTurningVelocity.getDoubleValue(),
                                                         updateDT);

         tempCurrentCoMPose.changeFrame(tempControlFrame.getParent());
         tempControlFrame.setPoseAndUpdate(tempCurrentCoMPose);

         desiredTouchdownPose.getPosition().set(desiredTouchdownPositionToPack);
         desiredTouchdownPose.getOrientation().setFromReferenceFrame(tempControlFrame);

         tempCurrentStanceFootPosition.setMatchingFrame(desiredTouchdownPositionToPack, 0.0);

         if (i == 0 && !useFutureCoM)
         {
            tempCurrentStanceFootPosition.changeFrame(centerOfMassControlZUpFrame);
            double stanceFootX = tempCurrentStanceFootPosition.getX();
            double stanceFootY = tempCurrentStanceFootPosition.getY();
            double stanceFootZ = tempCurrentStanceFootPosition.getZ();

            tempCurrentStanceFootPosition.changeFrame(tempControlFrame);
            tempCurrentStanceFootPosition.set(stanceFootX, stanceFootY, stanceFootZ);
            tempCurrentStanceFootPosition.changeFrame(tempControlFrame.getParent());

            nextStanceFootPosition.setMatchingFrame(tempCurrentStanceFootPosition);
            nextCoMPosition.setMatchingFrame(tempCurrentCoMPose.getPosition());
         }
      }

      if (desiredTouchdownPositionsList.get(currentSwingSide).size() > 0)
         desiredTouchdownPosition3DInWorld.get(currentSwingSide).setMatchingFrame(desiredTouchdownPositionsList.get(currentSwingSide).get(0), 0.0);
      else
         LogTools.error("Something may be wrong in QFP");
   }

   private void calculateNetPendulumBase()
   {
      // if only one foot is in contact, get the pendulum base directly from the support state
      if (!inDoubleSupport.getBooleanValue())
      {
         if (footStates.get(RobotSide.LEFT).getEnumValue() == FootState.SUPPORT)
         {
            netPendulumBase.setIncludingFrame(pendulumBase.get(RobotSide.LEFT));
            netPendulumBase.changeFrameAndProjectToXYPlane(referenceFrames.getSoleZUpFrame(RobotSide.LEFT));
            setTrailingSide(RobotSide.LEFT);
         }
         else
         {
            netPendulumBase.setIncludingFrame(pendulumBase.get(RobotSide.RIGHT));
            netPendulumBase.changeFrameAndProjectToXYPlane(referenceFrames.getSoleZUpFrame(RobotSide.RIGHT));
            setTrailingSide(RobotSide.RIGHT);
         }

         return;
      }

      // Figure out the side that you're shifting the weight away from based on the clock
      RobotSide trailingSide;
      if (newestSupportSide.getEnumValue() == RobotSide.RIGHT)
         trailingSide = RobotSide.LEFT;
      else
         trailingSide = RobotSide.RIGHT;

      setTrailingSide(trailingSide);

      // compute the double support duration
      double doubleSupportDuration = useAlternateTransferDuration ? getAlternateTransferDuration() : getTransferDuration(trailingSide.getOppositeSide());

      // compute the fraction through the double support state that we are at the current time
      double alpha = 1.0;
      if (doubleSupportDuration > 0.0)
         alpha = MathTools.clamp(footStateMachines.get(trailingSide.getOppositeSide()).getTimeInCurrentState() / doubleSupportDuration, 0.0, 1.0);

      // interpolate between the two pendulum bases based on the percentage through double support
      netPendulumBase.changeFrameAndProjectToXYPlane(referenceFrames.getSoleZUpFrame(trailingSide));
      pendulumBase.get(trailingSide).changeFrameAndProjectToXYPlane(referenceFrames.getSoleZUpFrame(trailingSide));
      pendulumBase.get(trailingSide.getOppositeSide()).changeFrameAndProjectToXYPlane(referenceFrames.getSoleZUpFrame(trailingSide));

      netPendulumBase.interpolate(pendulumBase.get(trailingSide), pendulumBase.get(trailingSide.getOppositeSide()), alpha);

      pendulumBase.get(trailingSide).changeFrameAndProjectToXYPlane(centerOfMassControlZUpFrame);
      pendulumBase.get(trailingSide.getOppositeSide()).changeFrameAndProjectToXYPlane(centerOfMassControlZUpFrame);
   }

   private void handleDesiredsFromProviders()
   {
      Vector2DReadOnly desiredVelocity = desiredVelocityProvider.getDesiredVelocity();
      double desiredVelocityX = desiredVelocity.getX();
      double desiredVelocityY = desiredVelocity.getY();
      double turningVelocity = desiredTurningVelocityProvider.getTurningVelocity();

      if (walkInputProvider != null)
         this.walk.set(walkInputProvider.getValue());
      this.desiredVelocity.set(desiredVelocityX, desiredVelocityY);
      this.desiredTurningVelocity.set(turningVelocity);
   }

   /**
    * Attaches a listener for {@code FootstepStatusMessage} to the manager.
    * <p>
    * This listener will automatically call {@link #notifyFootstepStarted(RobotSide)} and
    * {@link #notifyFootstepCompleted(RobotSide)}.
    * </p>
    *
    * @param statusMessageOutputManager the output API of the controller.
    */
   public void setFootstepStatusListener(StatusMessageOutputManager statusMessageOutputManager)
   {
      statusMessageOutputManager.attachStatusMessageListener(FootstepStatusMessage.class, this::consumeFootstepStatus);
   }

   /**
    * Consumes a newly received message and calls {@link #notifyFootstepStarted(RobotSide)} or
    * {@link #notifyFootstepCompleted(RobotSide)} according to the status.
    *
    * @param statusMessage the newly received footstep status.
    */
   public void consumeFootstepStatus(FootstepStatusMessage statusMessage)
   {
      FootstepStatus status = FootstepStatus.fromByte(statusMessage.getFootstepStatus());
      if (status == FootstepStatus.COMPLETED)
         notifyFootstepCompleted(RobotSide.fromByte(statusMessage.getRobotSide()));
      else if (status == FootstepStatus.STARTED)
         notifyFootstepStarted(RobotSide.fromByte(statusMessage.getRobotSide()));
   }

   /**
    * Notifies this generator that a footstep has been completed.
    * <p>
    * It is used internally to keep track of the support foot from which footsteps should be generated.
    * </p>
    *
    * @param robotSide the side corresponding to the foot that just completed a footstep.
    */
   public void notifyFootstepCompleted(RobotSide robotSide)
   {
      footStateMachines.get(robotSide).performTransition(FootState.SUPPORT);
   }

   /**
    * Notifies this generator that a footstep has been started, i.e. the foot started swinging.
    * <p>
    * It is used internally to mark the first generated footstep as unmodifiable so it does not change
    * while the swing foot is targeting it.
    * </p>
    */
   public void notifyFootstepStarted(RobotSide robotSide)
   {
      if (!footStateMachines.get(robotSide).getCurrentStateKey().equals(FootState.SWING))
         footStateMachines.get(robotSide).performTransition(FootState.SWING);

      if (!footStateMachines.get(robotSide.getOppositeSide()).getCurrentStateKey().equals(FootState.SUPPORT))
         footStateMachines.get(robotSide.getOppositeSide()).performTransition(FootState.SUPPORT);
   }

   /**
    * Sets the provider to use for getting every tick the desired turning velocity.
    *
    * @param desiredTurningVelocityProvider provider for obtaining the desired turning velocity.
    */
   public void setDesiredTurningVelocityProvider(DesiredTurningVelocityProvider desiredTurningVelocityProvider)
   {
      this.desiredTurningVelocityProvider = desiredTurningVelocityProvider;
   }

   /**
    * Sets the provider to use for getting every tick the desired forward/lateral velocity.
    *
    * @param desiredVelocityProvider provider for obtaining the desired forward/lateral velocity.
    */
   public void setDesiredVelocityProvider(DesiredVelocityProvider desiredVelocityProvider)
   {
      this.desiredVelocityProvider = desiredVelocityProvider;
   }

   /**
    * Sets a provider that is to be used to update the state of {@link #walk} internally on each call
    * to {@link #update(int)}.
    *
    * @param walkInputProvider the provider used to determine whether to walk or not walk.
    */
   public void setWalkInputProvider(BooleanProvider walkInputProvider)
   {
      this.walkInputProvider = walkInputProvider;
   }

   private void setTrailingSide(RobotSide robotSide)
   {
      trailingSide = robotSide;
   }

   private RobotSide getTrailingSide()
   {
      return trailingSide;
   }

   public void getDesiredTouchdownPose(int footstepIndex, RobotSide robotSide, FramePose2D touchdownPoseToPack)
   {
      // TODO 3D transform between two 2D poses?
      if (desiredTouchdownPosesList.get(robotSide).size() < 1)
         desiredTouchdownPosesList.get(robotSide).add();

      if (footstepIndex < desiredTouchdownPosesList.get(robotSide).size())
         touchdownPoseToPack.setMatchingFrame(desiredTouchdownPosesList.get(robotSide).get(footstepIndex));
      else
         touchdownPoseToPack.setMatchingFrame(desiredTouchdownPosesList.get(robotSide).getLast());
   }

   public double getStepDuration(RobotSide swingSide)
   {
      return getSwingDuration(swingSide) + getTransferDuration(swingSide);
   }

   public double getSwingDuration(RobotSide swingSide)
   {
      return parameters.getSwingDuration(swingSide).getDoubleValue();
   }

   public double getTransferDuration(RobotSide swingSide)
   {
      return parameters.getSwingDuration(swingSide).getDoubleValue() * parameters.getDoubleSupportFraction(swingSide).getDoubleValue();
   }

   public double getAlternateTransferDuration()
   {
      return alternateTransferDuration;
   }

   public void setAlternateTransferDuration(double alternateTransferDuration)
   {
      this.alternateTransferDuration = alternateTransferDuration;
   }

   public void setUseAlternateTransferDuration(boolean useAlternateTransferDuration)
   {
      this.useAlternateTransferDuration = useAlternateTransferDuration;
   }

   public double getSwingHeight(RobotSide swingSide)
   {
      return parameters.getSwingHeight(swingSide).getDoubleValue();
   }

   public RobotSide getCurrentSwingSide()
   {
      return newestSupportSide.getEnumValue().getOppositeSide();
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   private class SupportFootState implements State
   {
      private final RobotSide robotSide;

      private SupportFootState(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      @Override
      public void onEntry()
      {
         newestSupportSide.set(robotSide);
         footStates.get(robotSide).set(FootState.SUPPORT);

         pendulumBase.get(robotSide).setToZero(referenceFrames.getSoleZUpFrame(robotSide));
         pendulumBase.get(robotSide).changeFrameAndProjectToXYPlane(centerOfMassControlZUpFrame);

         double stepDuration = useAlternateTransferDuration ? getSwingDuration(robotSide) + getAlternateTransferDuration() : getStepDuration(robotSide);
         if (robotSide != newestSupportSide.getEnumValue() && inDoubleSupport.getValue())
            calculateTouchdownPosition(robotSide, stepDuration, true);
      }

      @Override
      public void doAction(double timeInState)
      {
         double stepDuration = useAlternateTransferDuration ? getSwingDuration(robotSide) + getAlternateTransferDuration() : getStepDuration(robotSide);
         double timeToReachGoal = stepDuration - footStateMachines.get(robotSide.getOppositeSide()).getTimeInCurrentState();
         timeToReachGoal = MathTools.clamp(timeToReachGoal, 0.0, stepDuration);

         pendulumBase.get(robotSide).setToZero(referenceFrames.getSoleZUpFrame(robotSide));
         pendulumBase.get(robotSide).changeFrameAndProjectToXYPlane(centerOfMassControlZUpFrame);

         if (robotSide != newestSupportSide.getEnumValue() && footStateMachines.get(robotSide.getOppositeSide()).getCurrentStateKey().equals(FootState.SUPPORT) && inDoubleSupport.getValue())
            calculateTouchdownPosition(robotSide, timeToReachGoal, true);
      }

      @Override
      public void onExit(double timeInState)
      {

      }

      @Override
      public boolean isDone(double timeInState)
      {
         double transferDuration = useAlternateTransferDuration ? getAlternateTransferDuration() : getTransferDuration(robotSide.getOppositeSide());

         return inDoubleSupport.getValue()
                && robotSide.getOppositeSide().equals(newestSupportSide.getEnumValue())
                && footStateMachines.get(robotSide.getOppositeSide()).getTimeInCurrentState() > transferDuration;
      }
   }

   private class SwingFootState implements State
   {
      private final RobotSide robotSide;
      private final YoDouble timeToReachGoal;

      private SwingFootState(RobotSide robotSide)
      {
         this.robotSide = robotSide;
         this.timeToReachGoal = new YoDouble("timeToReachGoal" + robotSide + "_QFP", registry);
      }

      @Override
      public void onEntry()
      {
         footStates.get(robotSide).set(FootState.SWING);

         calculateTouchdownPosition(robotSide, getSwingDuration(robotSide), false);

         parameters.setParametersForUpcomingSwing(robotSide);
      }

      @Override
      public void doAction(double timeInState)
      {
         double timeToReachGoal = getSwingDuration(robotSide) - timeInState;
         timeToReachGoal = MathTools.clamp(timeToReachGoal, 0.0, getSwingDuration(robotSide));
         this.timeToReachGoal.set(timeToReachGoal);

         calculateTouchdownPosition(robotSide, timeToReachGoal, false);
      }

      @Override
      public void onExit(double timeInState)
      {
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return false;
      }
   }
}