package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import org.apache.commons.lang3.mutable.MutableObject;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepVisualizer;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.contactable.ContactableBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * Use this class to generate a stream of footsteps which path can be controlled given desired
 * forward, lateral, and turning velocities.
 * <p>
 * To setup a continuous step generator the following items should be configured:
 * <ul>
 * <li><u>Step timing and step reach:</u><br>
 * They can all be set by calling: {@link #configureWith(WalkingControllerParameters)}, otherwise
 * they have to be set via {@link #setFootstepTiming(double, double)},
 * {@link #setStepWidths(double, double, double)}, {@link #setMaxStepLength(double)}, and
 * {@link #setStepTurningLimits(double, double)}. <b>If the step timing is not provided, the
 * generated footsteps will only be in place</b>.
 * <li><u>Protocol to access the current robot foot position and orientation:</u><br>
 * When the sole frames are available, this can be set via
 * {@link #setFrameBasedFootPoseProvider(SideDependentList)}. Otherwise, use
 * {@link #setFootPoseProvider(FootPoseProvider)}.
 * <li><u>This generator needs to be informed when footsteps are started/completed:</u><br>
 * The can be handle automatically by setting it up with
 * {@link #setFootstepStatusListener(StatusMessageOutputManager)}. Otherwise, the notifications
 * should be done <b>every step</b> via {@link #notifyFootstepStarted()} and
 * {@link #notifyFootstepCompleted(RobotSide)}, or
 * {@link #consumeFootstepStatus(FootstepStatusMessage)}.
 * <li><u>Protocol for submitting footsteps to the controller:</u><br>
 * This can be done via {@link #setFootstepMessenger(FootstepMessenger)}.
 * <li><u>Protocol to obtain the desired forward/lateral velocity and desired turning
 * velocity:</u><br>
 * To control them via {@code YoVariable}s, use {@link #setYoComponentProviders()}, otherwise, use
 * {@link #setDesiredVelocityProvider(DesiredVelocityProvider)} and
 * {@link #setDesiredTurningVelocityProvider(DesiredTurningVelocityProvider)}.
 * </ul>
 * Once configured, the method {@link #update(double)} is to be called on regular basis. This
 * generator then starts generating footsteps when {@link #walk} is {@code true}. {@link #walk} can
 * be changed via {@link #startWalking()}, {@link #stopWalking()}, and {@link #toggleWalking()}.
 * </p>
 * <p>
 * Additional <u>optional</u> settings can be configured:
 * <ul>
 * <li><u>Footstep visualization:</u><br>
 * See {@link #setupVisualization(List, List, YoGraphicsListRegistry)}.
 * <li><u>Number of footsteps to generate per tick:</u><br>
 * See {@link #setNumberOfFootstepsToPlan(int)}.
 * <li><u>Method for adjusting footstep height, pitch, and roll:</u><br>
 * Default is behavior is to adjust footsteps based on the current support foot pose. To change this
 * behavior, see {@link #setFootstepAdjustment(FootstepAdjustment)} and
 * {@link #setSupportFootBasedFootstepAdjustment(boolean)}.
 * <li><u>Method for indicating step validity and providing alternative steps for invalid
 * ones:</u><br>
 * See {@link #addFootstepValidityIndicator(FootstepValidityIndicator)}} to set method for
 * indicating invalid steps. By default, no steps are deemed invalid. See
 * {@link #setAlternateStepChooser(AlternateStepChooser)} to set how an alternative step is chosen
 * if a step is invalid. The default is to do a square-up step, see {@link #calculateSquareUpStep}.
 * <li><u>Frequency at which footsteps are sent to the controller:</u><br>
 * See {@link #setNumberOfTicksBeforeSubmittingFootsteps(int)}.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class ContinuousStepGenerator implements Updatable, SCS2YoGraphicHolder
{
   private static final int MAX_NUMBER_OF_FOOTSTEP_TO_VISUALIZE_PER_SIDE = 3;
   public static final Color defaultLeftColor = new Color(28, 134, 238); // dodgerblue 2, from: http://cloford.com/resources/colours/500col.htm
   public static final Color defaultRightColor = new Color(205, 133, 0); // orange 3, from: http://cloford.com/resources/colours/500col.htm
   public static final SideDependentList<Color> defaultFeetColors = new SideDependentList<>(defaultLeftColor, defaultRightColor);
   private final static Vector2DReadOnly zero2D = new Vector2D();

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final String variableNameSuffix = "CSG";

   private BooleanProvider walkInputProvider;
   private DoubleProvider swingHeightInputProvider;
   private final YoBoolean ignoreWalkInputProvider = new YoBoolean("ignoreWalkInputProvider" + variableNameSuffix, registry);
   private final YoBoolean walk = new YoBoolean("walk" + variableNameSuffix, registry);
   private final YoBoolean walkPreviousValue = new YoBoolean("walkPreviousValue" + variableNameSuffix, registry);

   private final YoEnum<RobotSide> currentSupportSide = new YoEnum<>("currentSupportSide" + variableNameSuffix, registry, RobotSide.class);
   private final YoFramePose3D currentSupportFootPose = new YoFramePose3D("currentSupportFootPose" + variableNameSuffix, worldFrame, registry);

   private final YoContinuousStepGeneratorParameters parameters = new YoContinuousStepGeneratorParameters(variableNameSuffix, registry);
   private final DoubleProvider stepTime = () -> parameters.getSwingDuration() + parameters.getTransferDuration();
   private final YoLong currentFootstepDataListCommandID = new YoLong("currentFootstepDataListCommandID_" + variableNameSuffix, registry);

   private final YoInteger numberOfTicksBeforeSubmittingFootsteps = new YoInteger("numberOfTicksBeforeSubmittingFootsteps" + variableNameSuffix, registry);

   private FootPoseProvider footPoseProvider;
   private DesiredVelocityProvider desiredVelocityProvider = () -> zero2D;
   private DesiredTurningVelocityProvider desiredTurningVelocityProvider = () -> 0.0;
   private FootstepMessenger footstepMessenger;
   private StopWalkingMessenger stopWalkingMessenger;
   private StartWalkingMessenger startWalkingMessenger;
   private List<FootstepAdjustment> footstepAdjustments = new ArrayList<>();
   private FootstepPlanAdjustment footstepPlanAdjustment;
   private List<FootstepValidityIndicator> footstepValidityIndicators = new ArrayList<>();
   private AlternateStepChooser alternateStepChooser = this::calculateSquareUpStep;

   private final YoDouble desiredTurningVelocity = new YoDouble("desiredTurningVelocity" + variableNameSuffix, registry);
   private final YoVector2D desiredVelocity = new YoVector2D("desiredVelocity" + variableNameSuffix, registry);
   private final ExecutionTimer stepGeneratorTimer = new ExecutionTimer("stepGeneratorTimer" + variableNameSuffix, registry);

   private final FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
   private final RecyclingArrayList<FootstepDataMessage> footsteps = footstepDataListMessage.getFootstepDataList();

   private final SideDependentList<List<FootstepVisualizer>> footstepSideDependentVisualizers = new SideDependentList<>(new ArrayList<>(), new ArrayList<>());

   private final MutableObject<FootstepStatus> latestStatusReceived = new MutableObject<>(null);
   private final MutableObject<RobotSide> footstepCompletionSide = new MutableObject<>(null);

   /**
    * Creates a new step generator, its {@code YoVariable}s will not be attached to any registry.
    */
   public ContinuousStepGenerator()
   {
      this(null);
   }

   /**
    * Creates a new step generator.
    *
    * @param parentRegistry the registry to attach this generator's registry.
    */
   public ContinuousStepGenerator(YoRegistry parentRegistry)
   {
      if (parentRegistry != null)
         parentRegistry.addChild(registry);

      parameters.clear();
      numberOfTicksBeforeSubmittingFootsteps.set(2);
      currentFootstepDataListCommandID.set(new Random().nextLong(0, Long.MAX_VALUE / 2)); // To make this command ID unique 

      setSupportFootBasedFootstepAdjustment(true);
   }

   private final FramePose2D footstepPose2D = new FramePose2D();
   private final FramePose2D nextFootstepPose2D = new FramePose2D();
   private final FramePose3D nextFootstepPose3D = new FramePose3D();
   private final FramePose3D previousFootstepPose = new FramePose3D();
   private final FramePose3D nextFootstepPose3DViz = new FramePose3D();

   private int counter = 0;

   /**
    * Process the current desired velocities to generate a new footstep plan.
    */
   @Override
   public void update(double time)
   {
      stepGeneratorTimer.startMeasurement();

      footstepDataListMessage.setDefaultSwingDuration(parameters.getSwingDuration());
      footstepDataListMessage.setDefaultTransferDuration(parameters.getTransferDuration());
      footstepDataListMessage.setFinalTransferDuration(parameters.getTransferDuration());
      footstepDataListMessage.setAreFootstepsAdjustable(parameters.getStepsAreAdjustable());
      footstepDataListMessage.setOffsetFootstepsWithExecutionError(parameters.getShiftUpcomingStepsWithTouchdown());

      if (!ignoreWalkInputProvider.getBooleanValue() && walkInputProvider != null)
         walk.set(walkInputProvider.getValue());

      if (!walk.getValue())
      {
         footsteps.clear();

         if (stopWalkingMessenger != null && walk.getValue() != walkPreviousValue.getValue())
         {
            stopWalkingMessenger.submitStopWalkingRequest();
         }

         walkPreviousValue.set(false);
         return;
      }
      else if (startWalkingMessenger != null && walk.getValue() != walkPreviousValue.getValue())
      {
         startWalkingMessenger.submitStartWalkingRequest();
      }

      if (walk.getValue() != walkPreviousValue.getValue())
      {
         currentSupportFootPose.setMatchingFrame(footPoseProvider.getCurrentFootPose(currentSupportSide.getValue()));
         counter = numberOfTicksBeforeSubmittingFootsteps.getValue(); // To make footsteps being sent right away.
      }

      { // Processing footstep status
         FootstepStatus statusToProcess = latestStatusReceived.getValue();

         if (statusToProcess != null)
         {
            if (statusToProcess == FootstepStatus.STARTED)
            {
               if (!footsteps.isEmpty())
                  footsteps.remove(0);
            }
            else if (statusToProcess == FootstepStatus.COMPLETED)
            {
               currentSupportSide.set(footstepCompletionSide.getValue());
               currentSupportFootPose.setMatchingFrame(footPoseProvider.getCurrentFootPose(currentSupportSide.getValue()));
               if (parameters.getNumberOfFixedFootsteps() == 0)
                  footsteps.clear();
            }
         }

         latestStatusReceived.setValue(null);
         footstepCompletionSide.setValue(null);
      }

      RobotSide swingSide;

      if (footsteps.isEmpty())
      {
         footstepPose2D.set(currentSupportFootPose);
         swingSide = currentSupportSide.getEnumValue().getOppositeSide();
         previousFootstepPose.set(currentSupportFootPose);
      }
      else
      {
         while (footsteps.size() > Math.max(1, parameters.getNumberOfFixedFootsteps()))
            footsteps.fastRemove(footsteps.size() - 1);

         FootstepDataMessage previousFootstep = footsteps.get(footsteps.size() - 1);
         footstepPose2D.getPosition().set(previousFootstep.getLocation());
         footstepPose2D.getOrientation().set(previousFootstep.getOrientation());
         swingSide = RobotSide.fromByte(previousFootstep.getRobotSide()).getOppositeSide();

         previousFootstepPose.set(previousFootstep.getLocation(), previousFootstep.getOrientation());
      }

      double maxStepLength = parameters.getMaxStepLength();
      double maxStepWidth = parameters.getMaxStepWidth();
      double minStepWidth = parameters.getMinStepWidth();
      double defaultStepWidth = parameters.getDefaultStepWidth();
      double turnMaxAngleInward = parameters.getTurnMaxAngleInward();
      double turnMaxAngleOutward = parameters.getTurnMaxAngleOutward();

      Vector2DReadOnly desiredVelocity = desiredVelocityProvider.getDesiredVelocity();
      double desiredVelocityX = desiredVelocity.getX();
      double desiredVelocityY = desiredVelocity.getY();
      double turningVelocity = desiredTurningVelocityProvider.getTurningVelocity();

      if (desiredVelocityProvider.isUnitVelocity())
      {
         double minMaxVelocityX = maxStepLength / stepTime.getValue();
         double minMaxVelocityY = maxStepWidth / stepTime.getValue();
         desiredVelocityX = minMaxVelocityX * MathTools.clamp(desiredVelocityX, 1.0);
         desiredVelocityY = minMaxVelocityY * MathTools.clamp(desiredVelocityY, 1.0);
      }

      if (desiredTurningVelocityProvider.isUnitVelocity())
      {
         double minMaxVelocityTurn = (turnMaxAngleOutward - turnMaxAngleInward) / stepTime.getValue();
         turningVelocity = minMaxVelocityTurn * MathTools.clamp(turningVelocity, 1.0);
      }

      this.desiredVelocity.set(desiredVelocityX, desiredVelocityY);
      this.desiredTurningVelocity.set(turningVelocity);

      int startingIndexToAdjust = footsteps.size();

      for (int i = startingIndexToAdjust; i < parameters.getNumberOfFootstepsToPlan(); i++)
      {
         double xDisplacement = MathTools.clamp(stepTime.getValue() * desiredVelocityX, maxStepLength);
         double yDisplacement = stepTime.getValue() * desiredVelocityY + swingSide.negateIfRightSide(defaultStepWidth);
         double headingDisplacement = stepTime.getValue() * turningVelocity;

         if (swingSide == RobotSide.LEFT)
         {
            yDisplacement = MathTools.clamp(yDisplacement, minStepWidth, maxStepWidth);
            headingDisplacement = MathTools.clamp(headingDisplacement, turnMaxAngleInward, turnMaxAngleOutward);
         }
         else
         {
            yDisplacement = MathTools.clamp(yDisplacement, -maxStepWidth, -minStepWidth);
            headingDisplacement = MathTools.clamp(headingDisplacement, -turnMaxAngleOutward, -turnMaxAngleInward);
         }

         double halfInPlaceWidth = 0.5 * swingSide.negateIfRightSide(defaultStepWidth);
         nextFootstepPose2D.set(footstepPose2D);
         // Applying the translation before the rotation allows the rotation to be centered in between the feet.
         // This ordering seems to provide the most natural footsteps.
         nextFootstepPose2D.appendTranslation(0.0, halfInPlaceWidth);
         nextFootstepPose2D.appendRotation(headingDisplacement);
         nextFootstepPose2D.appendTranslation(0.0, -halfInPlaceWidth);
         nextFootstepPose2D.appendTranslation(xDisplacement, yDisplacement);

         nextFootstepPose3D.set(nextFootstepPose2D);
         FootstepDataMessage footstep = footsteps.add();

         for (int adjustorIndex = 0; adjustorIndex < footstepAdjustments.size(); adjustorIndex++)
            footstepAdjustments.get(adjustorIndex).adjustFootstep(currentSupportFootPose, nextFootstepPose2D, swingSide, footstep);

         //         if (!isStepValid(nextFootstepPose3D, previousFootstepPose, swingSide))
         //         {
         //            alternateStepChooser.computeStep(footstepPose2D, nextFootstepPose2D, swingSide, nextFootstepPose3D);
         //            nextFootstepPose2D.set(nextFootstepPose3D);
         //         }

         footstep.setRobotSide(swingSide.toByte());
         if (swingHeightInputProvider == null)
            footstep.setSwingHeight(parameters.getSwingHeight());
         else
            footstep.setSwingHeight(swingHeightInputProvider.getValue());

         footstepPose2D.set(nextFootstepPose2D);
         swingSide = swingSide.getOppositeSide();
         previousFootstepPose.getPosition().set(footstep.getLocation());
         previousFootstepPose.getOrientation().set(footstep.getOrientation());
      }

      // adjust the whole footstep plan for the environment
      if (footstepPlanAdjustment != null)
      {
         footstepPlanAdjustment.adjustFootstepPlan(currentSupportFootPose, startingIndexToAdjust, footstepDataListMessage);

         if (startingIndexToAdjust == 0)
         {
            previousFootstepPose.set(currentSupportFootPose);
            footstepPose2D.set(currentSupportFootPose);
         }
         else
         {
            FootstepDataMessage footstepData = footstepDataListMessage.getFootstepDataList().get(startingIndexToAdjust - 1);
            previousFootstepPose.getPosition().set(footstepData.getLocation());
            previousFootstepPose.getOrientation().set(footstepData.getOrientation());
            footstepPose2D.set(previousFootstepPose);
         }
      }

      // run through and make sure these adjusted steps are valid.
      for (int i = startingIndexToAdjust; i < footstepDataListMessage.getFootstepDataList().size(); i++)
      {
         FootstepDataMessage footstepData = footstepDataListMessage.getFootstepDataList().get(i);
         nextFootstepPose2D.getPosition().set(footstepData.getLocation());
         nextFootstepPose2D.getOrientation().set(footstepData.getOrientation());
         nextFootstepPose3D.getPosition().set(footstepData.getLocation());
         nextFootstepPose3D.getOrientation().set(footstepData.getOrientation());
         swingSide = RobotSide.fromByte(footstepData.getRobotSide());

         if (!isStepValid(nextFootstepPose3D, previousFootstepPose, swingSide))
         {
            alternateStepChooser.computeStep(footstepPose2D, nextFootstepPose2D, swingSide, footstepData);

            // remove all the other steps after the invalid one.
            while (footstepDataListMessage.getFootstepDataList().size() > i + 1)
            {
               footstepDataListMessage.getFootstepDataList().remove(i + 1);
            }
         }

         previousFootstepPose.getPosition().set(footstepData.getLocation());
         previousFootstepPose.getOrientation().set(footstepData.getOrientation());
         footstepPose2D.set(previousFootstepPose);
      }

      // Update the visualizers
      for (int i = startingIndexToAdjust; i < footstepDataListMessage.getFootstepDataList().size(); i++)
      {
         int visualizerIndex = i / 2;
         List<FootstepVisualizer> footstepVisualizers = footstepSideDependentVisualizers.get(swingSide);

         if (visualizerIndex < footstepVisualizers.size())
         {
            FootstepDataMessage footstepData = footstepDataListMessage.getFootstepDataList().get(i);

            nextFootstepPose3D.getPosition().set(footstepData.getLocation());
            nextFootstepPose3D.getOrientation().set(footstepData.getOrientation());

            FootstepVisualizer footstepVisualizer = footstepVisualizers.get(visualizerIndex);
            nextFootstepPose3DViz.setIncludingFrame(nextFootstepPose3D);
            nextFootstepPose3DViz.appendTranslation(0.0, 0.0, -0.005); // Sink the viz slightly so it is below the controller footstep viz.
            footstepVisualizer.update(nextFootstepPose3DViz);
         }
      }

      if (walk.getValue() && footstepMessenger != null)
      {
         if (counter >= numberOfTicksBeforeSubmittingFootsteps.getValue())
         {
            currentFootstepDataListCommandID.increment();
            footstepDataListMessage.setSequenceId(currentFootstepDataListCommandID.getValue());
            footstepDataListMessage.getQueueingProperties().setSequenceId(currentFootstepDataListCommandID.getValue());
            footstepDataListMessage.getQueueingProperties().setMessageId(currentFootstepDataListCommandID.getValue());
            footstepMessenger.submitFootsteps(footstepDataListMessage);
            counter = 0;
         }
         else
         {
            counter++;
         }
      }

      walkPreviousValue.set(walk.getValue());
      stepGeneratorTimer.stopMeasurement();
   }

   /**
    * Sets the number of footsteps that are to be planned every tick.
    *
    * @param number the number of footsteps to plan.
    */
   public void setNumberOfFootstepsToPlan(int number)
   {
      parameters.setNumberOfFootstepsToPlan(number);
   }

   /**
    * Sets the number of footsteps which once planned cannot be modified.
    * <p>
    * A small value provides more responsive behavior to rapidly changing inputs, while a higher value
    * improves robustness to delays in the communication with the controller.
    * </p>
    * <p>
    * For walking gaits with short transfer duration and with the presence of communication delays with
    * the controller, it is recommended to set this parameter to &geq; 1.
    * </p>
    *
    * @param number the number of unmodifiable footsteps. Default value {@code 0}.
    */
   public void setNumberOfFixedFootsteps(int number)
   {
      parameters.setNumberOfFixedFootsteps(MathTools.clamp(number, 0, parameters.getNumberOfFootstepsToPlan() - 1));
   }

   /**
    * Sets the number of ticks to regulate the rate at which footsteps are sent to the controller.
    *
    * @param numberOfTicks number of ticks before sending footsteps to the controller, should be in [0,
    *                      &infin;[.
    */
   public void setNumberOfTicksBeforeSubmittingFootsteps(int numberOfTicks)
   {
      numberOfTicksBeforeSubmittingFootsteps.set(numberOfTicks);
   }

   /**
    * Sets the desired durations for the swing and the transfer.
    *
    * @param swingTime    desired duration of the swing, should be strictly positive.
    * @param transferTime desired duration of the transfer, should be strictly positive.
    */
   public void setFootstepTiming(double swingTime, double transferTime)
   {
      parameters.setSwingDuration(swingTime);
      parameters.setTransferDuration(transferTime);
   }

   public void setFootstepsAreAdjustable(boolean footstepsAreAdjustable)
   {
      parameters.setStepsAreAdjustable(footstepsAreAdjustable);
   }

   public void setSwingHeight(double swingHeight)
   {
      parameters.setSwingHeight(swingHeight);
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
    * Creates a {@code DesiredTurningVelocityProvider} and a {@code DesiredVelocityProvider} that are
    * baked with {@code YoVariable}s.
    */
   public void setYoComponentProviders()
   {
      setDesiredTurningVelocityProvider(desiredTurningVelocity::getValue);
      setDesiredVelocityProvider(() -> desiredVelocity);
   }

   /**
    * Sets the provider to use for getting the current pose of the support foot.
    *
    * @param footPoseProvider provider for obtaining the pose of the foot soles.
    */
   public void setFootPoseProvider(FootPoseProvider footPoseProvider)
   {
      this.footPoseProvider = footPoseProvider;
   }

   /**
    * Creates a {@code FootPoseProvider} that is baked by the given reference frames.
    *
    * @param soleFrames the sole frames that are assumed to be updated regularly.
    */
   public void setFrameBasedFootPoseProvider(SideDependentList<? extends ReferenceFrame> soleFrames)
   {
      setFootPoseProvider(new FootPoseProvider()
      {
         private final FramePose3D currentFootPose = new FramePose3D();

         @Override
         public FramePose3DReadOnly getCurrentFootPose(RobotSide robotSide)
         {
            currentFootPose.setFromReferenceFrame(soleFrames.get(robotSide));
            return currentFootPose;
         }
      });
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
      latestStatusReceived.setValue(FootstepStatus.COMPLETED);
      footstepCompletionSide.setValue(robotSide);
   }

   /**
    * Notifies this generator that a footstep has been started, i.e. the foot started swinging.
    * <p>
    * It is used internally to mark the first generated footstep as unmodifiable so it does not change
    * while the swing foot is targeting it.
    * </p>
    */
   public void notifyFootstepStarted()
   {
      latestStatusReceived.setValue(FootstepStatus.STARTED);
      footstepCompletionSide.setValue(null);
   }

   /**
    * Attaches a listener for {@code FootstepStatusMessage} to the manager.
    * <p>
    * This listener will automatically call {@link #notifyFootstepStarted()} and
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
    * Consumes a newly received message and calls {@link #notifyFootstepStarted()} or
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
         notifyFootstepStarted();
   }

   /**
    * Configures internal parameters for the step durations and step reach.
    *
    * @param walkingControllerParameters the parameters to use.
    */
   public void configureWith(WalkingControllerParameters walkingControllerParameters)
   {
      parameters.set(walkingControllerParameters);
   }

   /**
    * Configure parameters related to the width of the footsteps.
    *
    * @param inPlace desired step width when stepping in-place, forward, or backward.
    * @param min     minimum step width.
    * @param max     maximum step width.
    */
   public void setStepWidths(double inPlace, double min, double max)
   {
      parameters.setDefaultStepWidth(inPlace);
      parameters.setMinStepWidth(min);
      parameters.setMaxStepWidth(max);
   }

   /**
    * Maximum step length for both stepping forward and backward.
    *
    * @param max maximum step length.
    */
   public void setMaxStepLength(double max)
   {
      parameters.setMaxStepLength(max);
   }

   /**
    * Sets the rotation limits for turning.
    * <p>
    * These parameters are strongly related to the hip yaw range of motion.
    * </p>
    *
    * @param maxInwards  maximum reach when rotating the hip inwards. A positive angle is more
    *                    restrictive than a negative angle.
    * @param maxOutwards maximum reach when rotating the hip outwards. Should be positive.
    */
   public void setStepTurningLimits(double maxInwards, double maxOutwards)
   {
      parameters.setTurnMaxAngleInward(maxInwards);
      parameters.setTurnMaxAngleOutward(maxOutwards);
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

   /**
    * Sets the method for adjusting height, pitch, and roll of the generated footsteps.
    *
    * @param footstepAdjustment the adjustment method.
    */
   public void setFootstepAdjustment(FootstepAdjustment footstepAdjustment)
   {
      clearFootstepAdjustments();
      addFootstepAdjustment(footstepAdjustment);
   }

   /**
    * Sets the method for adjusting height, pitch, and roll of the generated footstep plan.
    *
    * @param footstepPlanAdjustment the plan adjustment method.
    */
   public void setFootstepPlanAdjustment(FootstepPlanAdjustment footstepPlanAdjustment)
   {
      this.footstepPlanAdjustment = footstepPlanAdjustment;
   }

   public void addFootstepAdjustment(FootstepAdjustment footstepAdjustment)
   {
      this.footstepAdjustments.add(footstepAdjustment);
   }

   public void clearFootstepAdjustments()
   {
      this.footstepAdjustments.clear();
   }

   public List<FootstepAdjustment> getFootstepAdjustments()
   {
      return footstepAdjustments;
   }

   /**
    * Adds a method of indicating step validity. If a step is not valid, the step generator calculates
    * an alternate step using the {@link #alternateStepChooser}
    *
    * @param footstepValidityIndicator method for checking step validity
    */
   public void addFootstepValidityIndicator(FootstepValidityIndicator footstepValidityIndicator)
   {
      footstepValidityIndicators.add(footstepValidityIndicator);
   }

   /**
    * Sets a method of calculating a new step pose if an element of {@link #footstepValidityIndicators}
    * indicates an invalid step.
    *
    * @param alternateStepChooser method for calculating alternate step
    */
   public void setAlternateStepChooser(AlternateStepChooser alternateStepChooser)
   {
      this.alternateStepChooser = alternateStepChooser;
   }

   /**
    * Sets a footstep adjustment that uses the current support foot pose to adjust the generated
    * footsteps.
    *
    * @param adjustPitchAndRoll whether the pitch and roll should be adjusted along with the height.
    */
   public void setSupportFootBasedFootstepAdjustment(boolean adjustPitchAndRoll)
   {
      setFootstepAdjustment(createSupportFootBasedFootstepAdjustment(adjustPitchAndRoll));
   }

   public FootstepAdjustment createSupportFootBasedFootstepAdjustment(boolean adjustPitchAndRoll)
   {
      return new FootstepAdjustment()
      {
         private final YawPitchRoll yawPitchRoll = new YawPitchRoll();

         @Override
         public boolean adjustFootstep(FramePose3DReadOnly supportFootPose,
                                       FramePose2DReadOnly footstepPose,
                                       RobotSide footSide,
                                       FootstepDataMessage adjustedPose)
         {
            adjustedPose.getLocation().set(footstepPose.getPosition());
            adjustedPose.getLocation().setZ(supportFootPose.getZ());
            if (adjustPitchAndRoll)
            {
               yawPitchRoll.set(supportFootPose.getOrientation());
               yawPitchRoll.setYaw(footstepPose.getYaw());
               adjustedPose.getOrientation().set(yawPitchRoll);
            }
            else
            {
               adjustedPose.getOrientation().set(footstepPose.getOrientation());
            }
            return true;
         }
      };
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
    * Sets a provider that is to be used to update the desired swing height of each foot internally on
    * each call to {@link #update(double)}
    *
    * @param swingHeightInputProvider the provider used to set the swing height
    */
   public void setSwingHeightInputProvider(DoubleProvider swingHeightInputProvider)
   {
      this.swingHeightInputProvider = swingHeightInputProvider;
   }

   /**
    * Toggle the walking state.
    */
   public void toggleWalking()
   {
      walk.set(!walk.getValue());
   }

   /**
    * Starts walking, i.e. this generator then starts generating footsteps.
    */
   public void startWalking()
   {
      walk.set(true);
   }

   /**
    * Stops walking, i.e. this generator then stops generating footsteps.
    */
   public void stopWalking()
   {
      walk.set(false);
   }

   /**
    * Whether this generator is currently generating footsteps.
    *
    * @return the walking state of this generator.
    */
   public boolean isWalking()
   {
      return walk.getBooleanValue();
   }

   /**
    * Creates a footstep visualization with a default size for the feet.
    *
    * @param yoGraphicsListRegistry the registry to attach the {@code YoGraphic}s to.
    */
   public void setupVisualization(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      setupVisualization(FootstepVisualizer.createTrapezoidalFootPolygon(0.12, 0.15, 0.25), yoGraphicsListRegistry);
   }

   /**
    * Creates a footstep visualization.
    *
    * @param footPolygon            the foot corner points (in clockwise order) to use for the
    *                               visualization of both feet.
    * @param yoGraphicsListRegistry the registry to attach the {@code YoGraphic}s to.
    */
   public void setupVisualization(List<? extends Point2DReadOnly> footPolygon, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      setupVisualization(footPolygon, footPolygon, yoGraphicsListRegistry);
   }

   /**
    * Creates a footstep visualization.
    *
    * @param contactableFeet        used to generate the foot polygons.
    * @param yoGraphicsListRegistry the registry to attach the {@code YoGraphic}s to.
    */
   public void setupVisualization(SideDependentList<? extends ContactableBody> contactableFeet, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      List<Point2D> leftFoot = contactableFeet.get(RobotSide.LEFT).getContactPointsCopy().stream().map(Point2D::new).collect(Collectors.toList());
      List<Point2D> rightFoot = contactableFeet.get(RobotSide.RIGHT).getContactPointsCopy().stream().map(Point2D::new).collect(Collectors.toList());
      setupVisualization(leftFoot, rightFoot, yoGraphicsListRegistry);
   }

   /**
    * Creates a footstep visualization.
    *
    * @param leftFootPolygon        the foot corner points (in clockwise order) to use for the
    *                               visualization of the left foot.
    * @param rightFootPolygon       the foot corner points (in clockwise order) to use for the
    *                               visualization of the right foot.
    * @param yoGraphicsListRegistry the registry to attach the {@code YoGraphic}s to.
    */
   public void setupVisualization(List<? extends Point2DReadOnly> leftFootPolygon,
                                  List<? extends Point2DReadOnly> rightFootPolygon,
                                  YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      String graphicListName = "FootstepGenerator";

      SideDependentList<List<? extends Point2DReadOnly>> footPolygons = new SideDependentList<>(leftFootPolygon, rightFootPolygon);

      for (RobotSide robotSide : RobotSide.values)
      {
         List<FootstepVisualizer> visualizers = new ArrayList<>();

         for (int i = 0; i < MAX_NUMBER_OF_FOOTSTEP_TO_VISUALIZE_PER_SIDE; i++)
         {
            String name = robotSide.getCamelCaseNameForStartOfExpression() + "PlannedFootstep" + i;
            visualizers.add(new FootstepVisualizer(name,
                                                   graphicListName,
                                                   robotSide,
                                                   footPolygons.get(robotSide),
                                                   defaultFeetColors.get(robotSide),
                                                   yoGraphicsListRegistry,
                                                   registry));
         }

         footstepSideDependentVisualizers.put(robotSide, visualizers);
      }
   }

   /**
    * Gets the read-only reference to the pose of the current support foot.
    *
    * @return the current foot pose.
    */
   public FramePose3DReadOnly getCurrentSupportFootPose()
   {
      return currentSupportFootPose;
   }

   /**
    * Gets the side of the current support foot.
    *
    * @return the support foot side.
    */
   public RobotSide getCurrentSupportSide()
   {
      return currentSupportSide.getEnumValue();
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   private boolean isStepValid(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      for (int i = 0; i < footstepValidityIndicators.size(); i++)
      {
         if (!footstepValidityIndicators.get(i).isFootstepValid(touchdownPose, stancePose, swingSide))
         {
            return false;
         }
      }

      return true;
   }

   private final FramePose2D alternateStepPose2D = new FramePose2D();

   /**
    * Calculates a square-up step given a 2D stance foot pose, to be used as an implementation of
    * {@link AlternateStepChooser}
    */
   private void calculateSquareUpStep(FramePose2DReadOnly stanceFootPose,
                                      FramePose2DReadOnly defaultTouchdownPose,
                                      RobotSide swingSide,
                                      FootstepDataMessage touchdownPoseToPack)
   {
      alternateStepPose2D.set(stanceFootPose);
      alternateStepPose2D.appendTranslation(0.0, swingSide.negateIfRightSide(parameters.getDefaultStepWidth()));
      for (int adjustorIndex = 0; adjustorIndex < footstepAdjustments.size(); adjustorIndex++)
         footstepAdjustments.get(adjustorIndex).adjustFootstep(currentSupportFootPose, alternateStepPose2D, swingSide, touchdownPoseToPack);
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());

      for (RobotSide robotSide : RobotSide.values)
      {
         List<FootstepVisualizer> visualizers = footstepSideDependentVisualizers.get(robotSide);
         if (visualizers == null || visualizers.isEmpty())
            return null;

         for (int i = 0; i < visualizers.size(); i++)
         {
            group.addChild(visualizers.get(i).getSCS2YoGraphics());
         }
      }

      return group;
   }
}
