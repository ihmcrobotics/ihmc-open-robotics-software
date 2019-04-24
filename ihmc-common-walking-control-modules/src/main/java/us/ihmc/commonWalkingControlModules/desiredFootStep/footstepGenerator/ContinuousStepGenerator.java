package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.apache.commons.lang3.mutable.MutableObject;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
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
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.robotics.contactable.ContactableBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePose3D;
import us.ihmc.yoVariables.variable.YoFrameVector2D;
import us.ihmc.yoVariables.variable.YoInteger;

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
 * behavior, see {@link #setFootstepAdjustment(FootstepAdjustment)},
 * {@link #setHeightMapBasedFootstepAdjustment(HeightMap)}, and
 * {@link #setSupportFootBasedFootstepAdjustment(boolean)}.
 * <li><u>Frequency at which footsteps are sent to the controller:</u><br>
 * See {@link #setNumberOfTicksBeforeSubmittingFootsteps(int)}.
 * </ul>
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class ContinuousStepGenerator implements Updatable
{
   private static final int MAX_NUMBER_OF_FOOTSTEP_TO_VISUALIZE_PER_SIDE = 3;
   public static final Color defaultLeftColor = new Color(28, 134, 238); // dodgerblue 2, from: http://cloford.com/resources/colours/500col.htm
   public static final Color defaultRightColor = new Color(205, 133, 0); // orange 3, from: http://cloford.com/resources/colours/500col.htm
   public static final SideDependentList<Color> defaultFeetColors = new SideDependentList<Color>(defaultLeftColor, defaultRightColor);
   private final static Vector2DReadOnly zero2D = new Vector2D();

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final String variableNameSuffix = "CSG";

   private final YoBoolean walk = new YoBoolean("walk" + variableNameSuffix, registry);
   private final YoBoolean walkPreviousValue = new YoBoolean("walkPreviousValue" + variableNameSuffix, registry);

   private final YoEnum<RobotSide> currentSupportSide = new YoEnum<>("currentSupportSide" + variableNameSuffix, registry, RobotSide.class);
   private final YoFramePose3D currentSupportFootPose = new YoFramePose3D("currentSupportFootPose" + variableNameSuffix, worldFrame, registry);

   private final YoInteger numberOfFootstepsToPlan = new YoInteger("numberOfFootstepsToPlan" + variableNameSuffix, registry);

   private final YoDouble inPlaceWidth = new YoDouble("inPlaceWidth" + variableNameSuffix, registry);
   private final YoDouble minStepWidth = new YoDouble("minStepWidth" + variableNameSuffix, registry);
   private final YoDouble maxStepWidth = new YoDouble("maxStepWidth" + variableNameSuffix, registry);
   private final YoDouble maxStepLength = new YoDouble("maxStepLength" + variableNameSuffix, registry);
   private final YoDouble maxAngleTurnOutwards = new YoDouble("maxAngleTurnOutwards" + variableNameSuffix, registry);
   private final YoDouble maxAngleTurnInwards = new YoDouble("maxAngleTurnInwards" + variableNameSuffix, registry);

   private final YoDouble swingTime = new YoDouble("swingTime" + variableNameSuffix, registry);
   private final YoDouble transferTime = new YoDouble("transferTime" + variableNameSuffix, registry);
   private final DoubleProvider stepTime = () -> swingTime.getDoubleValue() + transferTime.getDoubleValue();

   private final YoInteger numberOfTicksBeforeSubmittingFootsteps = new YoInteger("numberOfTicksBeforeSubmittingFootsteps" + variableNameSuffix, registry);

   private FootPoseProvider footPoseProvider;
   private DesiredVelocityProvider desiredVelocityProvider = () -> zero2D;
   private DesiredTurningVelocityProvider desiredTurningVelocityProvider = () -> 0.0;
   private FootstepMessenger footstepMessenger;
   private FootstepAdjustment footstepAdjustment;
   private FootstepValidityIndicator footstepValidityIndicator;

   private final FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
   private final RecyclingArrayList<FootstepDataMessage> footsteps = footstepDataListMessage.getFootstepDataList();
   private final FootstepDataMessage firstFootstep = new FootstepDataMessage();

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
   public ContinuousStepGenerator(YoVariableRegistry parentRegistry)
   {
      if (parentRegistry != null)
         parentRegistry.addChild(registry);

      numberOfFootstepsToPlan.set(4);
      minStepWidth.set(0.0);
      maxStepWidth.set(Double.POSITIVE_INFINITY);
      maxStepLength.set(Double.POSITIVE_INFINITY);
      maxAngleTurnOutwards.set(Math.PI / 2.0);
      maxAngleTurnInwards.set(-Math.PI / 2.0);
      numberOfTicksBeforeSubmittingFootsteps.set(2);

      setSupportFootBasedFootstepAdjustment(true);
   }

   private final FramePose2D footstepPose2D = new FramePose2D();
   private final FramePose2D nextFootstepPose2D = new FramePose2D();
   private final FramePose3D nextFootstepPose3D = new FramePose3D();
   private final FramePose3D nextFootstepPose3DViz = new FramePose3D();

   private boolean updateFirstFootstep = true;
   private int counter = 0;

   /**
    * Process the current desired velocities to generate a new footstep plan.
    */
   @Override
   public void update(double time)
   {
      if (!walk.getValue())
      {
         updateFirstFootstep = true;
         walkPreviousValue.set(false);
         return;
      }

      if (walk.getValue() != walkPreviousValue.getValue())
      {
         currentSupportFootPose.setMatchingFrame(footPoseProvider.getCurrentFootPose(currentSupportSide.getEnumValue()));
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
               if (!footsteps.isEmpty())
                  firstFootstep.set(footsteps.get(0));
            }
            else if (statusToProcess == FootstepStatus.COMPLETED)
            {
               updateFirstFootstep = true;
               currentSupportSide.set(footstepCompletionSide.getValue());
               currentSupportFootPose.setMatchingFrame(footPoseProvider.getCurrentFootPose(currentSupportSide.getEnumValue()));
            }
         }

         latestStatusReceived.setValue(null);
         footstepCompletionSide.setValue(null);
      }

      footstepDataListMessage.setDefaultSwingDuration(swingTime.getValue());
      footstepDataListMessage.setDefaultTransferDuration(transferTime.getValue());
      footstepDataListMessage.setFinalTransferDuration(transferTime.getValue());

      int startIndex = updateFirstFootstep ? 0 : 1;

      RobotSide swingSide;

      if (updateFirstFootstep)
      {
         footsteps.clear();
         footstepPose2D.set(currentSupportFootPose);
         swingSide = currentSupportSide.getEnumValue().getOppositeSide();
      }
      else
      {
         while (footsteps.size() > 1)
            footsteps.fastRemove(footsteps.size() - 1);

         footstepPose2D.getPosition().set(firstFootstep.getLocation());
         footstepPose2D.getOrientation().set(firstFootstep.getOrientation());
         swingSide = RobotSide.fromByte(firstFootstep.getRobotSide()).getOppositeSide();
      }

      for (int i = startIndex; i < numberOfFootstepsToPlan.getValue(); i++)
      {
         Vector2DReadOnly desiredVelocity = desiredVelocityProvider.getDesiredVelocity();

         double xDisplacement = MathTools.clamp(stepTime.getValue() * desiredVelocity.getX(), maxStepLength.getValue());
         double yDisplacement = stepTime.getValue() * desiredVelocity.getY() + swingSide.negateIfRightSide(inPlaceWidth.getValue());
         double headingDisplacement = stepTime.getValue() * desiredTurningVelocityProvider.getTurningVelocity();

         if (swingSide == RobotSide.LEFT)
         {
            yDisplacement = MathTools.clamp(yDisplacement, minStepWidth.getValue(), maxStepWidth.getValue());
            headingDisplacement = MathTools.clamp(headingDisplacement, maxAngleTurnInwards.getValue(), maxAngleTurnOutwards.getValue());
         }
         else
         {
            yDisplacement = MathTools.clamp(yDisplacement, -maxStepWidth.getValue(), -minStepWidth.getValue());
            headingDisplacement = MathTools.clamp(headingDisplacement, -maxAngleTurnOutwards.getValue(), -maxAngleTurnInwards.getValue());
         }

         double halfInPlaceWidth = 0.5 * swingSide.negateIfRightSide(inPlaceWidth.getValue());
         nextFootstepPose2D.set(footstepPose2D);
         // Applying the translation before the rotation allows the rotation to be centered in between the feet.
         // This ordering seems to provide the most natural footsteps.
         nextFootstepPose2D.appendTranslation(0.0, halfInPlaceWidth);
         nextFootstepPose2D.appendRotation(headingDisplacement);
         nextFootstepPose2D.appendTranslation(0.0, -halfInPlaceWidth);
         nextFootstepPose2D.appendTranslation(xDisplacement, yDisplacement);

         nextFootstepPose3D.set(footstepAdjustment.adjustFootstep(nextFootstepPose2D, swingSide));
         if (footstepValidityIndicator != null && !footstepValidityIndicator.isFootstepValid(nextFootstepPose3D))
         {
            nextFootstepPose2D.set(footstepPose2D);
            nextFootstepPose2D.appendTranslation(0.0, swingSide.negateIfRightSide(inPlaceWidth.getValue()));
            nextFootstepPose3D.set(footstepAdjustment.adjustFootstep(nextFootstepPose2D, swingSide));
         }
         
         int vizualizerIndex = i / 2;
         List<FootstepVisualizer> footstepVisualizers = footstepSideDependentVisualizers.get(swingSide);

         if (vizualizerIndex < footstepVisualizers.size())
         {
            FootstepVisualizer footstepVisualizer = footstepVisualizers.get(vizualizerIndex);
            nextFootstepPose3DViz.setIncludingFrame(nextFootstepPose3D);
            nextFootstepPose3DViz.appendTranslation(0.0, 0.0, -0.005); // Sink the viz slightly so it is below the controller footstep viz.
            footstepVisualizer.update(nextFootstepPose3DViz);
         }

         FootstepDataMessage footstep = footsteps.add();
         footstep.setRobotSide(swingSide.toByte());
         footstep.getLocation().set(nextFootstepPose3D.getPosition());
         footstep.getOrientation().set(nextFootstepPose3D.getOrientation());

         footstepPose2D.set(nextFootstepPose2D);
         swingSide = swingSide.getOppositeSide();
      }

      if (updateFirstFootstep)
      {
         firstFootstep.set(footsteps.get(0));
         updateFirstFootstep = false;
      }

      if (footstepMessenger != null)
      {
         if (counter >= numberOfTicksBeforeSubmittingFootsteps.getValue())
         {
            footstepMessenger.submitFootsteps(footstepDataListMessage);
            counter = 0;
         }
         else
         {
            counter++;
         }
      }

      walkPreviousValue.set(walk.getValue());
   }

   /**
    * Sets the number of footsteps that are to be planned every tick.
    * 
    * @param number the number of footsteps to plan.
    */
   public void setNumberOfFootstepsToPlan(int number)
   {
      numberOfFootstepsToPlan.set(number);
   }

   /**
    * Sets the number of ticks to regulate the rate at which footsteps are sent to the controller.
    * 
    * @param numberOfTicks number of ticks before sending footsteps to the controller, should be in
    *           [0, &infin;[.
    */
   public void setNumberOfTicksBeforeSubmittingFootsteps(int numberOfTicks)
   {
      numberOfTicksBeforeSubmittingFootsteps.set(numberOfTicks);
   }

   /**
    * Sets the desired durations for the swing and the transfer.
    * 
    * @param swingTime desired duration of the swing, should be strictly positive.
    * @param transferTime desired duration of the transfer, should be strictly positive.
    */
   public void setFootstepTiming(double swingTime, double transferTime)
   {
      this.swingTime.set(swingTime);
      this.transferTime.set(transferTime);
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
    * Creates a {@code DesiredTurningVelocityProvider} and a {@code DesiredVelocityProvider} that
    * are baked with {@code YoVariable}s.
    */
   public void setYoComponentProviders()
   {
      DoubleParameter desiredTurningVelocity = new DoubleParameter("desiredTurningVelocity" + variableNameSuffix, registry, 0.0);
      YoFrameVector2D desiredVelocity = new YoFrameVector2D("desiredVelocity" + variableNameSuffix, worldFrame, registry);
      setDesiredTurningVelocityProvider(() -> desiredTurningVelocity.getValue());
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
    * It is used internally to keep track of the support foot from which footsteps should be
    * generated.
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
    * It is used internally to mark the first generated footstep as unmodifiable so it does not
    * change while the swing foot is targeting it.
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
      setFootstepTiming(walkingControllerParameters.getDefaultSwingTime(), walkingControllerParameters.getDefaultFinalTransferTime());
      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();

      setStepWidths(steppingParameters.getInPlaceWidth(), steppingParameters.getMinStepWidth(), steppingParameters.getMaxStepWidth());
      setMaxStepLength(steppingParameters.getMaxStepLength());
      setStepTurningLimits(steppingParameters.getMaxAngleTurnInwards(), steppingParameters.getMaxAngleTurnOutwards());
   }

   /**
    * Configure parameters related to the width of the footsteps.
    * 
    * @param inPlace desired step width when stepping in-place, forward, or backward.
    * @param min minimum step width.
    * @param max maximum step width.
    */
   public void setStepWidths(double inPlace, double min, double max)
   {
      inPlaceWidth.set(inPlace);
      minStepWidth.set(min);
      maxStepWidth.set(max);
   }

   /**
    * Maximum step length for both stepping forward and backward.
    * 
    * @param max maximum step length.
    */
   public void setMaxStepLength(double max)
   {
      maxStepLength.set(max);
   }

   /**
    * Sets the rotation limits for turning.
    * <p>
    * These parameters are strongly related to the hip yaw range of motion.
    * </p>
    * 
    * @param maxInwards maximum reach when rotating the hip inwards. A positive angle is more
    *           restrictive than a negative angle.
    * @param maxOutwards maximum reach when rotating the hip outwards. Should be positive.
    */
   public void setStepTurningLimits(double maxInwards, double maxOutwards)
   {
      maxAngleTurnInwards.set(maxInwards);
      maxAngleTurnOutwards.set(maxOutwards);
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
    * Sets the method for adjusting height, pitch, and roll of the generated footsteps.
    * 
    * @param footstepAdjustment the adjustment method.
    */
   public void setFootstepAdjustment(FootstepAdjustment footstepAdjustment)
   {
      this.footstepAdjustment = footstepAdjustment;
   }

   /**
    * Sets a checker to indicate step validity. If a step is not valid, the step generator
    * replaces it with a square-up step
    *
    * @param footstepValidityIndicator method for checking step validity
    */
   public void setFootstepValidityIndicator(FootstepValidityIndicator footstepValidityIndicator)
   {
      this.footstepValidityIndicator = footstepValidityIndicator;
   }

   /**
    * Sets a footstep adjustment that uses the current support foot pose to adjust the generated
    * footsteps.
    * 
    * @param adjustPitchAndRoll whether the pitch and roll should be adjusted along with the height.
    */
   public void setSupportFootBasedFootstepAdjustment(boolean adjustPitchAndRoll)
   {
      setFootstepAdjustment(new FootstepAdjustment()
      {
         private final double[] yawPitchRoll = new double[3];
         private final FramePose3D adjustedPose = new FramePose3D();

         @Override
         public FramePose3DReadOnly adjustFootstep(FramePose2DReadOnly footstepPose, RobotSide footSide)
         {
            adjustedPose.getPosition().set(footstepPose.getPosition());
            adjustedPose.setZ(currentSupportFootPose.getZ());
            if (adjustPitchAndRoll)
            {
               currentSupportFootPose.getOrientationYawPitchRoll(yawPitchRoll);
               yawPitchRoll[0] = footstepPose.getYaw();
               adjustedPose.setOrientationYawPitchRoll(yawPitchRoll);
            }
            else
            {
               adjustedPose.setOrientation(footstepPose.getOrientation());
            }
            return adjustedPose;
         }
      });
   }

   /**
    * Sets a footstep adjustment that uses the height map to adjust the footsteps height and the
    * current support foot pitch/roll to adjust the footsteps pitch/roll.
    * 
    * @param heightMap the height map used to adjust footsteps height.
    */
   public void setHeightMapBasedFootstepAdjustment(HeightMap heightMap)
   {
      setFootstepAdjustment(new FootstepAdjustment()
      {
         private final double[] yawPitchRoll = new double[3];
         private final FramePose3D adjustedPose = new FramePose3D();

         @Override
         public FramePose3DReadOnly adjustFootstep(FramePose2DReadOnly footstepPose, RobotSide footSide)
         {
            adjustedPose.getPosition().set(footstepPose.getPosition());
            adjustedPose.setZ(heightMap.heightAt(footstepPose.getX(), footstepPose.getY(), 0.0));
            currentSupportFootPose.getOrientationYawPitchRoll(yawPitchRoll);
            yawPitchRoll[0] = footstepPose.getYaw();
            adjustedPose.setOrientationYawPitchRoll(yawPitchRoll);
            return adjustedPose;
         }
      });
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
    * @param footPolygon the foot corner points (in clockwise order) to use for the visualization of
    *           both feet.
    * @param yoGraphicsListRegistry the registry to attach the {@code YoGraphic}s to.
    */
   public void setupVisualization(List<? extends Point2DReadOnly> footPolygon, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      setupVisualization(footPolygon, footPolygon, yoGraphicsListRegistry);
   }

   /**
    * Creates a footstep visualization.
    * 
    * @param contactableFeet used to generate the foot polygons.
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
    * @param leftFootPolygon the foot corner points (in clockwise order) to use for the
    *           visualization of the left foot.
    * @param rightFootPolygon the foot corner points (in clockwise order) to use for the
    *           visualization of the right foot.
    * @param yoGraphicsListRegistry the registry to attach the {@code YoGraphic}s to.
    */
   public void setupVisualization(List<? extends Point2DReadOnly> leftFootPolygon, List<? extends Point2DReadOnly> rightFootPolygon,
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
            AppearanceDefinition footstepColor = new YoAppearanceRGBColor(defaultFeetColors.get(robotSide), 0.0);
            visualizers.add(new FootstepVisualizer(name, graphicListName, robotSide, footPolygons.get(robotSide), footstepColor, yoGraphicsListRegistry,
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
}
