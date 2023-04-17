package us.ihmc.avatar.networkProcessor.directionalControlToolboxModule;

import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.stream.Collectors;

import controller_msgs.msg.dds.*;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import us.ihmc.avatar.joystickBasedJavaFXController.JoystickStepParametersProperty.JoystickStepParameters;
import us.ihmc.avatar.joystickBasedJavaFXController.UserProfileManager;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.footstepPlanning.graphSearch.collision.BoundingBoxCollisionDetector;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStep;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStep.SnappingFailedException;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStepParameters;
import us.ihmc.humanoidRobotics.communication.directionalControlToolboxAPI.DirectionalControlConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.directionalControlToolboxAPI.DirectionalControlInputCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Controller for walking the robot using directional control messages.
 * <p>
 * DirectionalControlController is mainly intended to be used in conjunction with the
 * DirectionalControlToolbox. It receives two primary types of messages:
 * <ul>
 * <li>DirectionalControlConfigurationCommand messages to enable/disable walking, and otherwise
 * configure the controller
 * <li>DirectionalControlInputCommand messages to specify directional velocities. Velocities are
 * represented in the message as a set of three parameters (forward, right, clockwise), each with
 * values in the range [-1.0, 1.0].
 * </ul>
 * <p>
 * If using in conjunction with the toolbox, the messaging would be as follows:
 * <ul>
 * <li>Send a wake-up message to the DirectionalControlToolbox to enable it
 * <li>Within the toolbox timeout (currently 0.5 seconds), start streaming
 * DirectionalControlInputCommand messages
 * <li>Use DirectionalControlConfigurationCommand to enable/disable walking or load a different
 * walking profile.
 * <li>Send a sleep message to the DirectionalControlToolbox to disable it
 * </ul>
 * <p>
 * If using in a standalone capacity (see the toolboxBypass parameter in the constructor), the
 * controller is always ready to receive messages, and directional inputs are persistent, so only
 * changes need to be sent (however, streaming still works fine).
 * <p>
 * Bear in mind that this class controls step generation. The long pole in terms of walking is often
 * physically taking the step. It might not make sense to stream directional inputs at 100Hz if steps
 * take 1 second to complete; this drives fruitless re-computation. At the other extreme, the
 * toolbox will timeout if it does not receive a command within the timeout interval, so a rate of
 * at least 3Hz is needed.
 * <p>
 * In addition to generating footstep, the controller performs several types of footstep validation
 * and adjustment:
 * <ul>
 * <li>validation that the step is not too high/low to achieve
 * <li>validation that the step will not result in a collision
 * <li>snapping to a planar region (this requires REA)
 * </ul>
 * 
 * @author Mark Paterson (heavily based on existing hardware joystick walking code by Sylvain
 *         Bertand)
 */

public class DirectionalControlController extends ToolboxController
{

   /* Class constants */
   private final int MAIN_TASK_RATE_MS = 500; // How often to consider publishing footsteps to the controller in milliseconds.
                                              // XBox controller uses 500ms for the equivalent value.

   /*
    * Desired rate of travel, if walking is enabled. These properties correspond to how the footstep
    * generator sees the world. The incoming directional messages have directions: forward, right and
    * clockwise. These are mapped to the footstep directions in updateDirectionalInputs
    */
   private final DoubleProperty turningVelocityProperty = new SimpleDoubleProperty(this, "turningVelocityProperty", 0.0);
   private final DoubleProperty forwardVelocityProperty = new SimpleDoubleProperty(this, "forwardVelocityProperty", 0.0);
   private final DoubleProperty lateralVelocityProperty = new SimpleDoubleProperty(this, "lateralVelocityProperty", 0.0);

   // Storage space for messages
   private final AtomicReference<PlanarRegionsListMessage> planarRegionsListMessage = new AtomicReference<>(null);
   private final AtomicReference<FootstepDataListMessage> footstepsToSendReference = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsList> planarRegionsList = new AtomicReference<>(null);
   private final AtomicReference<DirectionalControlInputCommand> controlInputCommand = new AtomicReference<>(null);
   private final AtomicReference<DirectionalControlConfigurationCommand> controlConfigurationCommand = new AtomicReference<>(null);

   // Whether walking is enabled in the controller
   private final AtomicBoolean isWalking = new AtomicBoolean(false);

   // Whether robot has stopped walking on request. Needed because sometimes request messages are lost.
   private final AtomicBoolean hasSuccessfullyStoppedWalking = new AtomicBoolean(false);
   
   // Whether to ignore planar region information. This is on by default because it tends to block
   // side and rear stepping in a non-obvious way.
   private final AtomicBoolean ignorePlanarRegions = new AtomicBoolean(true);

   // Support for "profiles" that determine how aggressive or conservative the steps will be	
   private JoystickStepParameters controlParameters;
   private UserProfileManager<JoystickStepParameters> userProfileManager;
   private AtomicReference<JoystickStepParameters> stepParametersReference;

   // Robot properties 
   private final SteppingParameters steppingParameters;
   private final SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons;

   private final DoubleProvider stepTime;
   private final BoundingBoxCollisionDetector collisionDetector;

   // Keep the robotModel up-to-date with configuration changes
   private final RobotModelUpdater robotUpdater;

   // Publishers 
   private Consumer<FootstepDataListMessage> footstepPublisher = m ->
   {
   }; // publish  the footsteps from the step generator to the controller
   private Consumer<FootstepDataListMessage> footstepVisualizationPublisher = m ->
   {
   }; // publish  the footsteps from the step generator for visualization
   private Consumer<PauseWalkingMessage> pauseWalkingPublisher = m ->
   {
   }; // pause walking immediately

   // Footstep generation, validation and processing
   private final ContinuousStepGenerator continuousStepGenerator = new ContinuousStepGenerator();
   private final SnapAndWiggleSingleStep snapAndWiggleSingleStep;
   private final SnapAndWiggleSingleStepParameters snapAndWiggleParameters = new SnapAndWiggleSingleStepParameters();
   private final ConvexPolygon2D footPolygonToWiggle = new ConvexPolygon2D();
   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final PlanarRegion tempRegion = new PlanarRegion();
   private final SideDependentList<FramePose3D> lastSupportFootPoses = new SideDependentList<>(null, null);
   private final ConcurrentLinkedQueue<Runnable> queuedTasksToProcess = new ConcurrentLinkedQueue<>();
   private final AtomicBoolean isLeftFootInSupport = new AtomicBoolean(false);
   private final AtomicBoolean isRightFootInSupport = new AtomicBoolean(false);
   private final SideDependentList<AtomicBoolean> isFootInSupport = new SideDependentList<>(isLeftFootInSupport, isRightFootInSupport);
   private final BooleanProvider isInDoubleSupport = () -> isLeftFootInSupport.get() && isRightFootInSupport.get();
   private boolean supportFootPosesInitialized = false;

   private FootstepDataListMessage lastPublishedFootstepPlan = null; // the last footstep plan published

   // Scheduler for sub-tasks
   private final ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(4);

   /**
    * Main constructor.
    * 
    * @param robotModel                  -- model of the robot joints
    * @param walkingControllerParameters -- parameters for walking, stepping and more
    * @param statusOutputManager         -- manages toolbox output status messages. Currently this
    *                                    controller does not have any.
    * @param registry                    -- YoVariable registry for logging variables
    */
   public DirectionalControlController(FullHumanoidRobotModel robotModel,
                                       WalkingControllerParameters walkingControllerParameters,
                                       StatusMessageOutputManager statusOutputManager,
                                       YoRegistry registry)
   {
      super(statusOutputManager, registry);

      this.footPolygons = getFootPolygons(walkingControllerParameters);
      this.robotUpdater = new RobotModelUpdater(robotModel);

      steppingParameters = walkingControllerParameters.getSteppingParameters();
      stepTime = () -> getSwingDuration() + getTransferDuration();

      controlParameters = new JoystickStepParameters(walkingControllerParameters);

      userProfileManager = new UserProfileManager<JoystickStepParameters>(null,
                                                                          controlParameters,
                                                                          JoystickStepParameters::parseFromPropertyMap,
                                                                          JoystickStepParameters::exportToPropertyMap);
      controlParameters = userProfileManager.loadProfile("default");
      stepParametersReference = new AtomicReference<JoystickStepParameters>(controlParameters);

      snapAndWiggleParameters.setFootLength(walkingControllerParameters.getSteppingParameters().getFootLength());
      snapAndWiggleSingleStep = new SnapAndWiggleSingleStep(snapAndWiggleParameters);

      continuousStepGenerator.setNumberOfFootstepsToPlan(10);
      continuousStepGenerator.setDesiredTurningVelocityProvider(() -> turningVelocityProperty.get());
      continuousStepGenerator.setDesiredVelocityProvider(() -> new Vector2D(forwardVelocityProperty.get(), lateralVelocityProperty.get()));
      continuousStepGenerator.configureWith(walkingControllerParameters);

      continuousStepGenerator.addFootstepAdjustment(this::adjustFootstep);
      continuousStepGenerator.setFootstepMessenger(this::prepareFootsteps);
      continuousStepGenerator.setFootPoseProvider(robotSide -> new FramePose3D(getSoleFrame(robotSide)));
      continuousStepGenerator.addFootstepValidityIndicator(this::isStepSnappable);
      continuousStepGenerator.addFootstepValidityIndicator(this::isSafeDistanceFromObstacle);
      continuousStepGenerator.addFootstepValidityIndicator(this::isSafeStepHeight);

      // TODO: Collision box parameters taken from StepGeneratorJavaFXController.java
      // These are specific to the robot. To some degree, they should be derived from
      // some combination of:
      // - walking parameters
      // - physical properties
      // - step planning values
      double collisionBoxDepth = 0.65;
      double collisionBoxWidth = 1.15;
      double collisionBoxHeight = 1.0;
      collisionDetector = new BoundingBoxCollisionDetector();
      collisionDetector.setBoxDimensions(collisionBoxDepth, collisionBoxWidth, collisionBoxHeight);

      setupLowBandwidthTasks();
      LogTools.info("Ignoring REA planes is " + ignorePlanarRegions.toString());
   }

   public void setFootstepPublisher(Consumer<FootstepDataListMessage> footstepPublisher)
   {
      this.footstepPublisher = footstepPublisher;
   }

   public void setFootstepVisualizationPublisher(Consumer<FootstepDataListMessage> footstepVisualizationPublisher)
   {
      this.footstepVisualizationPublisher = footstepVisualizationPublisher;
   }

   public void setPauseWalkingPublisher(Consumer<PauseWalkingMessage> pauseWalkingPublisher)
   {
      this.pauseWalkingPublisher = pauseWalkingPublisher;
   }

   /**
    * Handler for robot configuration data updates
    * 
    * @param message
    */
   public void updateRobotConfigurationData(RobotConfigurationData message)
   {
      // Always update the robot configuration
      robotUpdater.updateConfiguration(message);

      RobotMotionStatus newStatus = RobotMotionStatus.fromByte(message.getRobotMotionStatus());
      // We only want to verify that the last PauseWalking sent has been successfully executed once.
      // Considering that the user may use a separate app to get the robot to walk, we do not want to interfere with the other app.
      if (hasSuccessfullyStoppedWalking.get() || isWalking.get())
         return;
      if (newStatus == null)
         return;
      if (newStatus == RobotMotionStatus.STANDING)
         hasSuccessfullyStoppedWalking.set(true);
   }

   // Status on whether a footstep started/completed
   public void updateFootstepStatusMessage(FootstepStatusMessage message)
   {
      queuedTasksToProcess.add(() ->
      {
         continuousStepGenerator.consumeFootstepStatus(message);

         if (message.getFootstepStatus() == FootstepStatus.COMPLETED.toByte())
         {
            lastSupportFootPoses.put(RobotSide.fromByte(message.getRobotSide()),
                                     new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                     message.getActualFootPositionInWorld(),
                                                     message.getActualFootOrientationInWorld()));
         }
      });
   }

   public void updatePlanarRegionsListMessage(PlanarRegionsListMessage message)
   {
      planarRegionsListMessage.set(message);
   }

   /**
    * Inform if a controller failure occurs so we can ensure we stop walking
    */
   public void updateWalkingControllerFailureStatusMessage(WalkingControllerFailureStatusMessage message)
   {
      stopWalking();
   }

   public void updateCapturabilityBasedStatus(CapturabilityBasedStatus message)
   {
      queuedTasksToProcess.add(() ->
      {
         isLeftFootInSupport.set(!message.getLeftFootSupportPolygon3d().isEmpty());
         isRightFootInSupport.set(!message.getRightFootSupportPolygon3d().isEmpty());
      });
   }

   /*
    * There need to be two sets of update functions, one for messages and one for commands. When using
    * this class as part of a toolbox, commands will be used. When using it as a standalone class,
    * messages will be used.
    */
   public void updateConfiguration(DirectionalControlConfigurationCommand command)
   {
      controlConfigurationCommand.set(command);
      LogTools.info("Config is now " + controlConfigurationCommand.toString());
   }

   public void updateConfiguration(DirectionalControlConfigurationMessage message)
   {
      DirectionalControlConfigurationCommand command = new DirectionalControlConfigurationCommand();
      command.setFromMessage(message);
      updateConfiguration(command);
   }

   public void updateInputs(DirectionalControlInputCommand command)
   {
      controlInputCommand.set(command);
      // LogTools.info("Input is now " + controlInputCommand.toString());
   }

   public void updateInputs(DirectionalControlInputMessage message)
   {
      DirectionalControlInputCommand command = new DirectionalControlInputCommand();
      command.setFromMessage(message);
      updateInputs(command);
   }

   /**
    * Set up to send footsteps to controller every MAIN_TASK_RATE_MS milliseconds
    */
   private void setupLowBandwidthTasks()
   {
      final Runnable task = new Runnable()
      {
         @Override
         public void run()
         {
            try
            {
               DirectionalControlController.this.publishFootsteps();
            }
            catch (Exception e)
            {
               // TODO Auto-generated catch block
               e.printStackTrace();
            }
         }

      };

      // Scheduled task to pick up any exceptions generated by the timer
      final ScheduledFuture<?> future = scheduler.scheduleAtFixedRate(task, 0, MAIN_TASK_RATE_MS, TimeUnit.MILLISECONDS);
      scheduler.execute(new Runnable()
      {
         @Override
         public void run()
         {
            try
            {
               future.get();
            }
            catch (InterruptedException | ExecutionException e)
            {
               e.printStackTrace();
               LogTools.error("Caught exception in main task: " + e);
               future.cancel(true);
            }
         }
      });
   }

   /**
    * Create polygons for the right and left feet. The polygons here are rectangles of the same size..
    * 
    * @param walkingControllerParameters
    * @return
    */
   private SideDependentList<ConvexPolygon2D> getFootPolygons(WalkingControllerParameters walkingControllerParameters)
   {
      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();
      double footLength = steppingParameters.getFootLength();
      double footWidth = steppingParameters.getFootWidth();
      ConvexPolygon2D footPolygon = new ConvexPolygon2D();
      footPolygon.addVertex(footLength / 2.0, footWidth / 2.0);
      footPolygon.addVertex(footLength / 2.0, -footWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, -footWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, footWidth / 2.0);
      footPolygon.update();

      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>(footPolygon, footPolygon);
      return footPolygons;
   }

   private FullHumanoidRobotModel getFullRobotModel()
   {
      return robotUpdater.getRobot();
   }

   private MovingReferenceFrame getSoleFrame(RobotSide robotSide)
   {
      return getFullRobotModel().getSoleFrame(robotSide);
   }

   public double getSwingDuration()
   {
      return stepParametersReference.get().getSwingDuration();
   }

   public double getTransferDuration()
   {
      return stepParametersReference.get().getTransferDuration();
   }

   /**
    * Turn the 2D desired foot pose from the step generator into a full 3D pose, allowing for planar
    * regions.
    * 
    * @param footstepPose -- incoming 2D foot pose
    * @param footSide     -- left or right
    * @return 3D pose for the input step
    */
   private boolean adjustFootstep(FramePose3DReadOnly stancePose, FramePose2DReadOnly footstepPose, RobotSide footSide, FootstepDataMessage adjustedFootstep)
   {
      FramePose3D adjustedBasedOnStanceFoot = new FramePose3D();
      adjustedBasedOnStanceFoot.getPosition().set(footstepPose.getPosition());
      // Initial Z position matches the stance foot
      adjustedBasedOnStanceFoot.setZ(continuousStepGenerator.getCurrentSupportFootPose().getZ());
      adjustedBasedOnStanceFoot.getOrientation().set(footstepPose.getOrientation());

      // If there are planar regions, attempt to modify the pose such that the foot
      // fits on a plane.
      if (!ignorePlanarRegions.get() && planarRegionsList.get() != null && planarRegionsList.get().getNumberOfPlanarRegions() > 0)
      {
         FramePose3D wiggledPose = new FramePose3D(adjustedBasedOnStanceFoot);
         footPolygonToWiggle.set(footPolygons.get(footSide));
         try
         {
            snapAndWiggleSingleStep.snapAndWiggle(wiggledPose, footPolygonToWiggle, forwardVelocityProperty.get() > 0.0);
            if (wiggledPose.containsNaN())
            {
               adjustedFootstep.getLocation().set(adjustedBasedOnStanceFoot.getPosition());
               adjustedFootstep.getOrientation().set(adjustedBasedOnStanceFoot.getOrientation());
               return true;
            }
         }
         catch (SnappingFailedException e)
         {
            /*
             * It's fine if the snap & wiggle fails, can be because there no planar regions around the footstep.
             * Let's just keep the adjusted footstep based on the pose of the current stance foot.
             */
         }
         adjustedFootstep.getLocation().set(wiggledPose.getPosition());
         adjustedFootstep.getOrientation().set(wiggledPose.getOrientation());
         return true;
      }
      else
      {
         adjustedFootstep.getLocation().set(adjustedBasedOnStanceFoot.getPosition());
         adjustedFootstep.getOrientation().set(adjustedBasedOnStanceFoot.getOrientation());
         return true;
      }
   }

   private void updateForwardVelocity(double alpha)
   {
      double minMaxVelocity = stepParametersReference.get().getMaxStepLength() / stepTime.getValue();
      forwardVelocityProperty.set(minMaxVelocity * MathTools.clamp(alpha, 1.0));
   }

   private void updateLateralVelocity(double alpha)
   {
      double minMaxVelocity = stepParametersReference.get().getMaxStepWidth() / stepTime.getValue();
      lateralVelocityProperty.set(minMaxVelocity * MathTools.clamp(alpha, 1.0));
   }

   private void updateTurningVelocity(double alpha)
   {
      double minMaxVelocity = (stepParametersReference.get().getTurnMaxAngleOutward() - stepParametersReference.get().getTurnMaxAngleInward())
            / stepTime.getValue();
      if (forwardVelocityProperty.get() < -1.0e-10)
         alpha = -alpha;
      turningVelocityProperty.set(minMaxVelocity * MathTools.clamp(alpha, 1.0));
   }

   /**
    * Configure robot to start walking
    */
   public void startWalking()
   {
      isWalking.set(true);
      continuousStepGenerator.startWalking();
      hasSuccessfullyStoppedWalking.set(false);
   }

   /**
    * Configure robot to stop walking
    */
   public void stopWalking()
   {
      isWalking.set(false);
      footstepsToSendReference.set(null);
      continuousStepGenerator.stopWalking();
      sendPauseMessage();
      lastPublishedFootstepPlan = null;
   }

   private boolean isNotYetWalking()
   {
      return lastPublishedFootstepPlan == null;
   }

   /**
    * Send a pause walking message to the walking controller
    */
   private void sendPauseMessage()
   {
      PauseWalkingMessage pauseWalkingMessage = new PauseWalkingMessage();
      pauseWalkingMessage.setPause(true);
      pauseWalkingPublisher.accept(pauseWalkingMessage);
   }

   /**
    * Take the footsteps generated by the continuous step generator and adjust according to our walking
    * parameters.
    * 
    * @param footstepDataListMessage
    */
   private void prepareFootsteps(FootstepDataListMessage footstepDataListMessage)
   {
      for (int i = 0; i < footstepDataListMessage.getFootstepDataList().size(); i++)
      {
         FootstepDataMessage footstepDataMessage = footstepDataListMessage.getFootstepDataList().get(i);
         footstepDataMessage.setSwingHeight(stepParametersReference.get().getSwingHeight());
      }
      footstepsToSendReference.set(new FootstepDataListMessage(footstepDataListMessage));
      footstepVisualizationPublisher.accept(footstepDataListMessage);
   }

   /**
    * Publish the footsteps to the walking controller, if walking is active.
    */
   private void publishFootsteps()
   {
      FootstepDataListMessage footstepsToSend = footstepsToSendReference.getAndSet(null);
      if (footstepsToSend != null)
      {
         if (isWalking.get())
         {
           long millis = System.currentTimeMillis();
           LogTools.info(String.format("%d.%d: Publishing Footsteps", millis / 1000, millis % 1000));
           footstepPublisher.accept(footstepsToSend);
           lastPublishedFootstepPlan = footstepsToSend;
         }
      }

      // Only send pause request if we think the command has not been executed yet. This is to be more robust in case packets are dropped.
      if (!isWalking.get())
      {
         if (!hasSuccessfullyStoppedWalking.get())
            sendPauseMessage();
      }
   }

   /**
    * Determine whether this step is stepping up or down too far to be safe
    * 
    * @param touchdownPose -- expected pose of the swing foot when it touches down
    * @param stancePose    -- pose of the current support foot
    * @param swingSide     -- which foot will be taking the step
    * @return -- True if the the step is safe; false otherwise
    */
   private boolean isSafeStepHeight(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      double heightChange = touchdownPose.getZ() - stancePose.getZ();
      return heightChange < steppingParameters.getMaxStepUp() && heightChange > -steppingParameters.getMaxStepDown();
   }

   /**
    * Determine whether the footstep places the robot too close to an obstacle
    * 
    * @param touchdownPose -- expected pose of the swing foot when it touches down
    * @param stancePose    -- pose of the current support foot
    * @param swingSide     -- which foot will be taking the step
    * @return -- True if the step is safe; false otherwise
    */
   private boolean isSafeDistanceFromObstacle(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      if (planarRegionsList.get() == null)
         return true;

      double halfStanceWidth = 0.5 * steppingParameters.getInPlaceWidth();

      /**
       * Shift box vertically by max step up, regions below this could be steppable
       */
      double heightOffset = steppingParameters.getMaxStepUp();

      double soleYaw = touchdownPose.getYaw();
      double lateralOffset = swingSide.negateIfLeftSide(halfStanceWidth);
      double offsetX = -lateralOffset * Math.sin(soleYaw);
      double offsetY = lateralOffset * Math.cos(soleYaw);
      collisionDetector.setBoxPose(touchdownPose.getX() + offsetX, touchdownPose.getY() + offsetY, touchdownPose.getZ() + heightOffset, soleYaw);

      return !collisionDetector.checkForCollision().isCollisionDetected();
   }

   /**
    * Determine whether the step can be snapped to a planar region
    * 
    * @param touchdownPose -- expected pose of the foot when it touches down
    * @param stancePose    -- pose of the current planted foot
    * @param swingSide     -- which foot will be taking the step
    * @return -- True if the step can be snapped; false otherwise
    */
   private boolean isStepSnappable(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      if (planarRegionsList.get() == null)
         return true;

      tempTransform.getTranslation().set(touchdownPose.getPosition().getX(), touchdownPose.getPosition().getY(), 0.0);
      tempTransform.getRotation().setToYawOrientation(touchdownPose.getYaw());

      footPolygon.set(footPolygons.get(swingSide));
      footPolygon.applyTransform(tempTransform, false);

      PlanarRegionsList planarRegionsList = this.planarRegionsList.get();
      return PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon, planarRegionsList, Double.POSITIVE_INFINITY, tempRegion) != null;
   }

   /**
    * Handle incoming changes to direction inputs. Note that the input message values need to be mapped
    * to how the footstep generator sees the world.
    */
   protected void updateDirectionalInputs()
   {
      // Handle directional inputs
      DirectionalControlInputCommand inputMessage = controlInputCommand.getAndSet(null);
      if (inputMessage != null)
      {
         updateForwardVelocity(inputMessage.getForward());
         updateLateralVelocity(-inputMessage.getRight());
         updateTurningVelocity(-inputMessage.getClockwise());
      }
   }

   /**
    * Handle incoming changes to controller state
    */
   protected void updateStateInputs()
   {
      // Handle configuration inputs
      DirectionalControlConfigurationCommand controlMessage = controlConfigurationCommand.getAndSet(null);
      if (controlMessage != null)
      {
         String profile = controlMessage.getProfileName();
         if (profile.length() > 0)
         {
            try
            {
               controlParameters = userProfileManager.loadProfile(profile);
               stepParametersReference = new AtomicReference<JoystickStepParameters>(controlParameters);
               LogTools.info("Switched profile to " + profile);
            }
            catch (RuntimeException e)
            {
               System.out.printf("Unable to load profile %s: %s\n", profile, e);
            }
         }
         if (isWalking.get() && !controlMessage.getEnableWalking())
         {
            stopWalking();
         }
         else if (!isWalking.get() && controlMessage.getEnableWalking())
         {
            startWalking();
         }
      }
   }

   /**
    * Process configuration and control messages
    */
   protected void handleIncomingMessages()
   {
      updateDirectionalInputs();
      updateStateInputs();
   }

   /**
    * Shut down all activity and support tasks
    */
   public void shutdown()
   {
      scheduler.shutdownNow();
      PauseWalkingMessage pauseWalkingMessage = new PauseWalkingMessage();
      pauseWalkingMessage.setPause(true);
      pauseWalkingPublisher.accept(pauseWalkingMessage);
   }

   @Override
   public boolean initialize()
   {
      // Should return true when the controller is initialized.
      // This controller is born ready.
      return true;
   }

   /**
    * Main loop update function called from the toolbox or from a scheduled task if bypassing the
    * toolbox. This function handles: 
    * - changing walking parameters based on input messages 
    * - updating the planar regions list 
    * - updating step generator control parameters and getting a new footstep plan 
    */
   @Override
   public void updateInternal() throws Exception
   {

      try
      {
         while (!queuedTasksToProcess.isEmpty())
            queuedTasksToProcess.poll().run();

         if (!supportFootPosesInitialized)
         {
            if (isLeftFootInSupport.get() && isRightFootInSupport.get())
            {
               for (RobotSide robotSide : RobotSide.values)
               {
                  lastSupportFootPoses.put(robotSide, new FramePose3D(getSoleFrame(robotSide)));
               }

               supportFootPosesInitialized = true;
            }
            else
            {
               for (RobotSide robotSide : RobotSide.values)
               {
                  if (!isFootInSupport.get(robotSide).get())
                     LogTools.warn(robotSide.getPascalCaseName() + " foot is not in support, cannot initialize foot poses.");
               }
            }
         }

         if (!supportFootPosesInitialized)
            return;

         for (RobotSide robotSide : RobotSide.values)
         {
            if (isFootInSupport.get(robotSide).get())
            { // Touchdown may not have been made with the foot properly settled, so we update the support foot pose if its current pose is lower.
               MovingReferenceFrame soleFrame = getSoleFrame(robotSide);
               double currentHeight = soleFrame.getTransformToWorldFrame().getTranslationZ();
               if (currentHeight < lastSupportFootPoses.get(robotSide).getZ())
                  lastSupportFootPoses.put(robotSide, new FramePose3D(soleFrame));
            }
         }

         DirectionalControlController.this.handleIncomingMessages();
         PlanarRegionsListMessage latestMessage = planarRegionsListMessage.getAndSet(null);
         if (latestMessage != null)
         {
            PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(latestMessage);
            snapAndWiggleSingleStep.setPlanarRegions(planarRegionsList);
            collisionDetector.setPlanarRegionsList(new PlanarRegionsList(planarRegionsList.getPlanarRegionsAsList().stream()
                                                                                          .filter(region -> region.getConvexHull()
                                                                                                                  .getArea() >= snapAndWiggleParameters.getMinPlanarRegionArea())
                                                                                          .collect(Collectors.toList())));
            DirectionalControlController.this.planarRegionsList.set(planarRegionsList);
         }

         JoystickStepParameters stepParameters = stepParametersReference.get();
         continuousStepGenerator.setFootstepTiming(stepParameters.getSwingDuration(), stepParameters.getTransferDuration());
         continuousStepGenerator.update(Double.NaN);

      }
      catch (Throwable e)
      {
         e.printStackTrace();
         LogTools.error("Caught exception, stopping timer.");
      }
   }

   /**
    * This function allows the controller to indicate that it has no active task to work on, and the
    * toolbox may sleep. This capability is not used here. Instead, we rely on the natural message
    * flow.
    */
   @Override
   public boolean isDone()
   {
      return false;
   }

   /**
    * Receive notifications when the state of this toolbox is changing. We use this to determine that
    * the toolbox has decided to sleep. In this case, we stop walking immediately rather than allowing
    * the current plan to complete.
    * 
    * @param newState the new state this toolbox is about to enter.
    */
   @Override
   public void notifyToolboxStateChange(ToolboxState newState)
   {
      if (newState == ToolboxState.SLEEP)
      {
         stopWalking();
      }
      LogTools.info("Directional controller state is now " + newState.toString());
   }

}
