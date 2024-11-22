package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.dyanmicsBasedFootstepGenerator;

import controller_msgs.msg.dds.FootstepStatusMessage;
import org.apache.commons.lang3.mutable.MutableObject;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.HumanoidSteppingPlugin;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.AngularExcursionCalculator;
import us.ihmc.robotics.screwTheory.MovingZUpFrame;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class DynamicsBasedFootstepPlugin implements HumanoidSteppingPlugin
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final DynamicsBasedFootstepParameters parameters;

   private final String variableNameSuffix = "QFP";

   private final CommonHumanoidReferenceFrames referenceFrames;

   private final double updateDT;
   private final double gravity = 9.81;

   // Estimates
   private final YoFramePoint3D currentCoMPosition = new YoFramePoint3D("currentCoMPosition" + variableNameSuffix, ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D currentCoMVelocity = new YoFrameVector3D("currentCoMVelocity" + variableNameSuffix, ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D currentCentroidalLinearMomentum = new YoFrameVector3D("currentCentroidalLinearMomentum" + variableNameSuffix, ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D currentCentroidalAngularMomentum = new YoFrameVector3D("currentCentroidalAngularMomentum" + variableNameSuffix, ReferenceFrame.getWorldFrame(), registry);
   private final AngularExcursionCalculator angularExcursionCalculator;
   private final MovingReferenceFrame centerOfMassControlFrame;
   private final MovingZUpFrame centerOfMassControlZUpFrame;
   private final MovingReferenceFrame centerOfMassFrame;
//   private final FootSoleBasedGroundPlaneEstimator groundPlaneEstimator;

   // Command/Desired output related stuff
   private final SideDependentList<DynamicsBasedFootstepTouchdownCalculator> touchdownCalculator = new SideDependentList<>();
   private final SideDependentList<FramePoint2D> desiredTouchdownPositions = new SideDependentList<>();

   // Desired inputs
   private final YoDouble desiredTurningVelocity = new YoDouble("desiredTurningVelocity" + variableNameSuffix, registry);
   private final YoVector2D desiredVelocity = new YoVector2D("desiredVelocity" + variableNameSuffix, registry);
   private final YoBoolean ignoreWalkInputProvider = new YoBoolean("ignoreWalkInputProvider" + variableNameSuffix, registry);
   private final YoBoolean walk = new YoBoolean("walk" + variableNameSuffix, registry);
   private final YoBoolean walkPreviousValue = new YoBoolean("walkPreviousValue" + variableNameSuffix, registry);
   private final YoFrameQuaternion desiredPelvisOrientation;

   // Foot state information
   public enum FootState {SUPPORT, SWING}
   private final SideDependentList<YoEnum<FootState>> footStates = new SideDependentList<>();
   private final SideDependentList<StateMachine<FootState, State>> footStateMachines = new SideDependentList<>();
   private final YoEnum<RobotSide> newestSupportSide = new YoEnum<>("newestSupportSide" + variableNameSuffix, registry, RobotSide.class);
   private final MutableObject<FootstepStatus> latestFootstepStatusReceived = new MutableObject<>(null);
   private final MutableObject<RobotSide> footstepCompletionSide = new MutableObject<>(null);
   private final YoBoolean inDoubleSupport = new YoBoolean("inDoubleSupport" + variableNameSuffix, registry);
   private RobotSide trailingSide = RobotSide.LEFT;

   //
   private final SideDependentList<FramePoint2D> pendulumBase = new SideDependentList<>();
   private final FramePoint2D netPendulumBase = new FramePoint2D();
   private final SideDependentList<YoFramePoint3D> pendulumBase3DInWorld = new SideDependentList<>();
   private final YoFramePoint3D netPendulumBase3DInWorld = new YoFramePoint3D("netPendulumBase3DInWorld" + variableNameSuffix, ReferenceFrame.getWorldFrame(), registry);
   private final SideDependentList<YoGraphicPosition> pendulumBaseViz = new SideDependentList<>();
   private final YoGraphicPosition netPendulumBaseViz;


   //
   private final static Vector2DReadOnly zero2D = new Vector2D();
   private BooleanProvider walkInputProvider;
   private DoubleProvider swingHeightInputProvider;
   private DesiredVelocityProvider desiredVelocityProvider = () -> zero2D;
   private DesiredTurningVelocityProvider desiredTurningVelocityProvider = () -> 0.0;
   private DirectionalControlMessenger directionalControlMessenger;
   private StopWalkingMessenger stopWalkingMessenger;
   private StartWalkingMessenger startWalkingMessenger;
   private FootstepMessenger footstepMessenger;

   public DynamicsBasedFootstepPlugin(FullHumanoidRobotModel robotModel, CommonHumanoidReferenceFrames referenceFrames, double updateDT, YoGraphicsListRegistry yoGraphicsListRegistry, DoubleProvider yoTime)
   {
      this.updateDT = updateDT;
      this.referenceFrames = referenceFrames;

      // Parameters
      parameters = new DynamicsBasedFootstepParameters(gravity, registry);

      // All things estimates
      centerOfMassFrame = (MovingReferenceFrame) referenceFrames.getCenterOfMassFrame();

      angularExcursionCalculator = new AngularExcursionCalculator(centerOfMassFrame, robotModel.getElevator(), updateDT, registry, null);

      desiredPelvisOrientation = new YoFrameQuaternion("pelvisDesiredOrientation" + variableNameSuffix, ReferenceFrame.getWorldFrame(), registry);

      centerOfMassControlFrame = new MovingReferenceFrame("centerOfMassControlFrame" + variableNameSuffix, ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
         {
            //            pelvisFrame.getTwistRelativeToOther(ReferenceFrame.getWorldFrame(), pelvisTwist);
            //            pelvisTwist.changeFrame(centerOfMassFrame); // FIXME we really want the rotation about the center of mass, relative to the world.

            twistRelativeToParentToPack.getLinearPart().setMatchingFrame(getCenterOfMassVelocity());
            //            twistRelativeToParentToPack.getAngularPart().setMatchingFrame(pelvisTwist.getAngularPart());
         }

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.getTranslation().set(currentCoMPosition);
            transformToParent.getRotation().set(desiredPelvisOrientation);
         }
      };

      centerOfMassControlZUpFrame = new MovingZUpFrame(centerOfMassControlFrame, "centerOfMassControlZUpFrame" + variableNameSuffix);

      netPendulumBaseViz = new YoGraphicPosition("netPendulumBase" + variableNameSuffix,
                                                 netPendulumBase3DInWorld,
                                                 0.015,
                                                 YoAppearance.Red(),
                                                 YoGraphicPosition.GraphicType.SQUARE);


      //groundPlaneEstimator = new FootSoleBasedGroundPlaneEstimator(centerOfMassControlZUpFrame, referenceFrames, yoGraphicsListRegistry, registry);

      // Side-dependant stuff, most of the desired-related things
      for (RobotSide robotSide : RobotSide.values)
      {
         footStates.put(robotSide, new YoEnum<>(robotSide.getLowerCaseName() + "FootStates" + variableNameSuffix, registry, FootState.class));

         pendulumBase.put(robotSide, new FramePoint2D(centerOfMassControlZUpFrame));

         pendulumBase3DInWorld.put(robotSide, new YoFramePoint3D(robotSide.getLowerCaseName() + "PendulumBase3DInWorld" + variableNameSuffix,
                                                                 ReferenceFrame.getWorldFrame(),
                                                                 registry));

         pendulumBaseViz.put(robotSide, new YoGraphicPosition(robotSide.getLowerCaseName() + "PendulumBase" + variableNameSuffix,
                                                              pendulumBase3DInWorld.get(robotSide),
                                                              0.015,
                                                              YoAppearance.Black(),
                                                              YoGraphicPosition.GraphicType.SQUARE));

         yoGraphicsListRegistry.registerYoGraphic(variableNameSuffix, pendulumBaseViz.get(robotSide));
         yoGraphicsListRegistry.registerArtifact(variableNameSuffix, pendulumBaseViz.get(robotSide).createArtifact());

         touchdownCalculator.put(robotSide, new DynamicsBasedFootstepTouchdownCalculator(robotSide,
                                                                                         centerOfMassControlZUpFrame,
                                                                                         getCenterOfMassVelocity(),
                                                                                         currentCentroidalAngularMomentum,
                                                                                         robotModel,
                                                                                         parameters,
                                                                                         gravity,
                                                                                         desiredVelocity,
                                                                                         registry));

         desiredTouchdownPositions.put(robotSide, new FramePoint2D());

         StateMachineFactory<FootState, State> stateMachineFactory = new StateMachineFactory<>(FootState.class);
         stateMachineFactory.setRegistry(registry).setNamePrefix(robotSide.getLowerCaseName() + variableNameSuffix).buildYoClock(yoTime);

         stateMachineFactory.addState(FootState.SUPPORT, new SupportFootState(robotSide));
         stateMachineFactory.addState(FootState.SWING, new SwingFootState(robotSide));

         stateMachineFactory.addDoneTransition(FootState.SUPPORT, FootState.SWING);
         stateMachineFactory.addDoneTransition(FootState.SWING, FootState.SUPPORT);

         footStateMachines.put(robotSide, stateMachineFactory.build(FootState.SUPPORT));
         footStateMachines.get(robotSide).resetToInitialState();
      }

      yoGraphicsListRegistry.registerYoGraphic(variableNameSuffix, netPendulumBaseViz);
      yoGraphicsListRegistry.registerArtifact(variableNameSuffix, netPendulumBaseViz.createArtifact());
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
         pendulumBase.get(robotSide).setToZero(referenceFrames.getSoleZUpFrame(robotSide));
         pendulumBase.get(robotSide).changeFrameAndProjectToXYPlane(centerOfMassControlZUpFrame);
         calculateNetPendulumBase();

         if (inDoubleSupport.getBooleanValue())
            calculate(touchdownCalculator.get(robotSide),
                      robotSide,
                      getTransferDuration(robotSide),
                      pendulumBase.get(robotSide.getOppositeSide()),
                      netPendulumBase,
                      inDoubleSupport.getBooleanValue(),
                      desiredTouchdownPositions.get(robotSide));
      }

      @Override
      public void doAction(double timeInState)
      {
         pendulumBase.get(robotSide).setToZero(referenceFrames.getSoleZUpFrame(robotSide));
         pendulumBase.get(robotSide).changeFrameAndProjectToXYPlane(centerOfMassControlZUpFrame);
         calculateNetPendulumBase();

         double timeToReachGoal = getTransferDuration(robotSide) - timeInState;
         timeToReachGoal = MathTools.clamp(timeToReachGoal, 0.0, getTransferDuration(robotSide));

         if (inDoubleSupport.getBooleanValue() && footStateMachines.get(robotSide.getOppositeSide()).getTimeInCurrentState() > timeInState)
            calculate(touchdownCalculator.get(robotSide),
                      robotSide,
                      timeToReachGoal,
                      pendulumBase.get(robotSide.getOppositeSide()),
                      netPendulumBase,
                      inDoubleSupport.getBooleanValue(),
                      desiredTouchdownPositions.get(robotSide));
      }

      @Override
      public void onExit(double timeInState)
      {

      }

      @Override
      public boolean isDone(double timeInState)
      {
         return footStates.get(robotSide).getEnumValue() == FootState.SWING;
      }
   }

   private class SwingFootState implements State
   {
      private final RobotSide robotSide;

      private SwingFootState(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      @Override
      public void onEntry()
      {
         calculate(touchdownCalculator.get(robotSide),
                   robotSide,
                   getStepDuration(robotSide),
                   pendulumBase.get(robotSide.getOppositeSide()),
                   netPendulumBase,
                   inDoubleSupport.getBooleanValue(),
                   desiredTouchdownPositions.get(robotSide));
      }

      @Override
      public void doAction(double timeInState)
      {
         double timeToReachGoal = getStepDuration(robotSide) - timeInState;
         timeToReachGoal = MathTools.clamp(timeToReachGoal, 0.0, getStepDuration(robotSide));

         calculate(touchdownCalculator.get(robotSide),
                   robotSide,
                   timeToReachGoal,
                   pendulumBase.get(robotSide.getOppositeSide()),
                   netPendulumBase,
                   inDoubleSupport.getBooleanValue(),
                   desiredTouchdownPositions.get(robotSide));
      }

      @Override
      public void onExit(double timeInState)
      {
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return footStates.get(robotSide).getEnumValue() == FootState.SUPPORT;
      }
   }

   @Override
   public void update(double time)
   {
//      groundPlaneEstimator.update();

      currentCoMPosition.setFromReferenceFrame(centerOfMassFrame);

      angularExcursionCalculator.compute();
      currentCentroidalLinearMomentum.setMatchingFrame(angularExcursionCalculator.getLinearMomentum());
      currentCentroidalAngularMomentum.setMatchingFrame(angularExcursionCalculator.getAngularMomentum());

      // TODO maybe wrong?
      desiredPelvisOrientation.setToYawOrientation(referenceFrames.getChestFrame().getTransformToWorldFrame().getRotation().getYaw());
      desiredPelvisOrientation.appendYawRotation(desiredTurningVelocity.getDoubleValue() * updateDT);

      centerOfMassControlFrame.update();
      centerOfMassControlZUpFrame.update();

      currentCoMVelocity.setMatchingFrame(centerOfMassFrame.getTwistOfFrame().getLinearPart());

      handleDesiredsFromProviders();

      inDoubleSupport.set(footStates.get(RobotSide.LEFT).getEnumValue() == FootState.SUPPORT &&
                          footStates.get(RobotSide.RIGHT).getEnumValue() == FootState.SUPPORT);

      calculateNetPendulumBase();

      for (RobotSide robotSide : RobotSide.values)
      {
         footStateMachines.get(robotSide).doActionAndTransition();
         pendulumBase3DInWorld.get(robotSide).setMatchingFrame(pendulumBase.get(robotSide), 0.0);
      }

      netPendulumBase3DInWorld.setMatchingFrame(netPendulumBase, 0.0);
   }

   public void calculate(RobotSide swingSide,
                         double timeToReachGoal,
                         FramePose2DReadOnly pendulumBase,
                         FramePose2DReadOnly netPendulumBase,
                         FramePose2DBasics touchdownPositionToPack)
   {
      calculate(touchdownCalculator.get(swingSide),
                swingSide, timeToReachGoal,
                pendulumBase.getPosition(),
                netPendulumBase.getPosition(),
                inDoubleSupport.getBooleanValue(),
                touchdownPositionToPack.getPosition());
   }

   private final FramePoint2DBasics tempPendulumBase = new FramePoint2D();
   private final FramePoint2DBasics tempNetPendulumBase = new FramePoint2D();

   public void calculate(DynamicsBasedFootstepTouchdownCalculator touchdownCalculator,
                                RobotSide swingSide,
                                double timeToReachGoal,
                                FramePoint2DReadOnly pendulumBase,
                                FramePoint2DReadOnly netPendulumBase,
                                boolean isInDoubleSupport,
                                FixedFramePoint2DBasics touchdownPositionToPack)
   {
      tempPendulumBase.setIncludingFrame(pendulumBase);
      tempPendulumBase.changeFrame(centerOfMassControlZUpFrame);

      tempNetPendulumBase.setIncludingFrame(netPendulumBase);
      tempNetPendulumBase.changeFrame(referenceFrames.getSoleZUpFrame(getTrailingSide()));

      touchdownCalculator.computeDesiredTouchdownPosition(swingSide.getOppositeSide(), timeToReachGoal, tempPendulumBase, tempNetPendulumBase, isInDoubleSupport);
      touchdownPositionToPack.setMatchingFrame(touchdownCalculator.getDesiredTouchdownPosition2D());
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
      if (footStateMachines.get(RobotSide.LEFT).getTimeInCurrentState() > footStateMachines.get(RobotSide.RIGHT).getTimeInCurrentState())
         trailingSide = RobotSide.LEFT;
      else
         trailingSide = RobotSide.RIGHT;

      setTrailingSide(trailingSide);

      // compute the double support duration
      double doubleSupportDuration = getTransferDuration(trailingSide.getOppositeSide());

      // compute the fraction through the double support state that we are at the current time
      double alpha = 1.0;
      if (doubleSupportDuration > 0.0)
         alpha = MathTools.clamp(footStateMachines.get(trailingSide.getOppositeSide()).getTimeInCurrentState() / doubleSupportDuration, 0.0, 1.0);

      // interpolate between the two pendulum bases based on the percentage through double support
      netPendulumBase.changeFrameAndProjectToXYPlane(pendulumBase.get(trailingSide).getReferenceFrame());
      netPendulumBase.interpolate(pendulumBase.get(trailingSide), pendulumBase.get(trailingSide.getOppositeSide()), alpha);
      netPendulumBase.changeFrameAndProjectToXYPlane(referenceFrames.getSoleZUpFrame(trailingSide));
   }

   private void handleDesiredsFromProviders()
   {
      Vector2DReadOnly desiredVelocity = desiredVelocityProvider.getDesiredVelocity();
      double desiredVelocityX = desiredVelocity.getX();
      double desiredVelocityY = desiredVelocity.getY();
      double turningVelocity = desiredTurningVelocityProvider.getTurningVelocity();

//      if (desiredVelocityProvider.isUnitVelocity())
//      {
//         double minMaxVelocityX = maxStepLength / stepTime.getValue();
//         double minMaxVelocityY = maxStepWidth / stepTime.getValue();
//         desiredVelocityX = minMaxVelocityX * MathTools.clamp(desiredVelocityX, 1.0);
//         desiredVelocityY = minMaxVelocityY * MathTools.clamp(desiredVelocityY, 1.0);
//      }
//
//      if (desiredTurningVelocityProvider.isUnitVelocity())
//      {
//         double minMaxVelocityTurn = (turnMaxAngleOutward - turnMaxAngleInward) / stepTime.getValue();
//         turningVelocity = minMaxVelocityTurn * MathTools.clamp(turningVelocity, 1.0);
//      }

//      this.desiredVelocity.set(desiredVelocityX, desiredVelocityY);
//      this.desiredTurningVelocity.set(turningVelocity);
   }

   /**
    * Sets the protocol for sending footsteps to the controller.
    *
    * @param footstepMessenger the callback used to send footsteps.
    */
   public void setFootstepMessenger(FootstepMessenger footstepMessenger)
   {
      this.footstepMessenger = footstepMessenger;
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
      latestFootstepStatusReceived.setValue(FootstepStatus.COMPLETED);
      footstepCompletionSide.setValue(robotSide);
      footStates.get(robotSide).set(FootState.SUPPORT);
      newestSupportSide.set(robotSide);
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
      latestFootstepStatusReceived.setValue(FootstepStatus.STARTED);
      footstepCompletionSide.setValue(null);
      footStates.get(robotSide).set(FootState.SWING);
   }

   /**
    * Sets a provider that is to be used to update the state of {@link #walk} internally on each call
    * to {@link #update(double)}.
    *
    * @param walkInputProvider the provider used to determine whether to walk or not walk.
    */
   public void setWalkInputProvider(BooleanProvider walkInputProvider)
   {
      this.walkInputProvider = walkInputProvider;
   }

   /**
    * Sets a provider that is to be used to update the desired swing height of each foot internally
    * on each call to {@link #update(double)}
    *
    * @param swingHeightInputProvider the provider used to set the swing height
    */
   public void setSwingHeightInputProvider(DoubleProvider swingHeightInputProvider)
   {
      this.swingHeightInputProvider = swingHeightInputProvider;
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
    * Sets the protocol for stop walking requests to the controller.
    *
    * @param stopWalkingMessenger the callback used to send requests.
    */
   public void setStopWalkingMessenger(StopWalkingMessenger stopWalkingMessenger)
   {
      this.stopWalkingMessenger = stopWalkingMessenger;
   }

   /**
    * Sets the protocol for start walking requests to the controller.
    *
    * @param startWalkingMessenger the callback used to send requests.
    */
   public void setStartWalkingMessenger(StartWalkingMessenger startWalkingMessenger)
   {
      this.startWalkingMessenger = startWalkingMessenger;
   }

   @Override
   public void setFootstepAdjustment(FootstepAdjustment footstepAdjustment)
   {

   }

   private void setTrailingSide(RobotSide robotSide)
   {
      trailingSide = robotSide;
   }

   private RobotSide getTrailingSide()
   {
      return trailingSide;
   }

   private final FramePoint2D tempTouchdownPosition = new FramePoint2D();
   public void getDesiredTouchdownPosition2D(RobotSide robotSide, FixedFramePoint2DBasics touchdownPositionToPack)
   {
      tempTouchdownPosition.setIncludingFrame(getDesiredTouchdownPosition2D(robotSide));
      tempTouchdownPosition.changeFrameAndProjectToXYPlane(touchdownPositionToPack.getReferenceFrame());
      touchdownPositionToPack.set(tempTouchdownPosition);
   }

   public FramePoint2DReadOnly getDesiredTouchdownPosition2D(RobotSide robotSide)
   {
      if (desiredTouchdownPositions.get(robotSide).containsNaN())
      {
         footStates.get(robotSide).set(FootState.SWING);
         footStateMachines.get(robotSide).performTransition(FootState.SWING);
         footStateMachines.get(robotSide).doAction();
      }

      return desiredTouchdownPositions.get(robotSide);
   }

   public FrameVector3DReadOnly getCenterOfMassVelocity()
   {
      return currentCoMVelocity;
   }

   public double getSwingDuration(RobotSide swingSide)
   {
      return parameters.getSwingDuration(swingSide).getDoubleValue();
   }

   public double getTransferDuration(RobotSide swingSide)
   {
      return parameters.getSwingDuration(swingSide).getDoubleValue() * parameters.getDoubleSupportFraction(swingSide).getDoubleValue();
   }

   public double getStepDuration(RobotSide swingSide)
   {
      return getSwingDuration(swingSide) + getTransferDuration(swingSide);
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }
}