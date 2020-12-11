package us.ihmc.avatar.networkProcessor.directionalControlToolboxModule;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import controller_msgs.msg.dds.DirectionalControlConfigurationMessage;
import controller_msgs.msg.dds.DirectionalControlInputMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import us.ihmc.avatar.joystickBasedJavaFXController.UserProfileManager;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.joystickBasedJavaFXController.JoystickStepParametersProperty.JoystickStepParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.footstepPlanning.graphSearch.collision.BoundingBoxCollisionDetector;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStep;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStepParameters;
import us.ihmc.humanoidRobotics.communication.directionalControlToolboxAPI.DirectionalControlConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.directionalControlToolboxAPI.DirectionalControlInputCommand;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStep.SnappingFailedException;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.concurrent.TimeUnit;

/**
 * Controller for walking the robot using directional control messages.
 * 
 * DirectionalControlController is mainly intended to be used in conjunction with the DirectionalControlToolbox. It 
 * receives two primary types of messages:
 * - DirectionalControlConfigurationCommand messages to enable/disable walking, and otherwise configure the controller
 * - DirectionalControlInputCommand messages to specify directional velocities. Velocities are represented in the
 * message as a set of three parameters (forward, right, clockwise), each with values in the range [-1.0, 1.0]. The
 * 
 * If using in conjunction with the toolbox, the messaging would be as follows:
 * - Send a wake-up message to the DirectionalControlToolbox to enable it
 * - Within the toolbox timeout (currently 0.5 seconds), start streaming DirectionalControlInputCommand messages
 * - Use DirectionalControlConfigurationCommand to enable/disable walking or load a different walking profile.
 * - Send a sleep message to the DirectionalControlToolbox to disable it
 * 
 * If using in a standalone capacity (see the toolboxBypass parameter in the constructor), the controller is always
 * ready to receive messages, and directional inputs are persistent, so only changes need to be sent (however,
 * streaming still works fine).
 * 
 * Bear in mind that this class controls step generation. The long pole in terms of walking is often
 * physically taking the step. It does not make sense to stream directional inputs at 100Hz if steps take
 * 1 second to complete; this drives fruitless re-computation. At the other extreme, the toolbox will
 * timeout if it does not receive a command within the timeout interval, so a rate of at least 3Hz is needed.
 * 
 * In addition to generating footstep, the controller performs several types of footstep validation and adjustment:
 * - validation that the step is not too high/low to achieve
 * - validation that the step will not result in a collision
 * - snapping to a planar region (this requires REA)
 * 
 * @author Mark Paterson (heavily based on existing hardware joystick walking code by Sylvain Bertand)
 *
 */
@SuppressWarnings("restriction")
public class DirectionalControlController extends ToolboxController {

	/* Class constants */
	private final int MAIN_TASK_RATE_MS = 1000; // How often to check tasks for exceptions (milliseconds)
	
	/* 
	 * Desired rate of travel, if walking is enabled. These properties correspond to how the footstep generator sees the world.
	 * The incoming directional messages have directions: forward, right and clockwise. These are mapped to the 
	 * footstep directions in updateDirectionalInputs  
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

    // Support for "profiles" that determine how aggressive or conservative the steps will be	
	private JoystickStepParameters controlParameters;
	private UserProfileManager<JoystickStepParameters> userProfileManager;
	private AtomicReference<JoystickStepParameters> stepParametersReference;	
	
	// Robot properties 
	private final WalkingControllerParameters walkingControllerParameters;
	private final String robotName;
	private final SteppingParameters steppingParameters;
	private final SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons;
	
	private final DoubleProvider stepTime;
	private final BoundingBoxCollisionDetector collisionDetector;
	
	// Keep the robotModel up-to-date with configuration changes
	private final RobotModelUpdater robotUpdater; 
	
	// Publishers 
	private final IHMCROS2Publisher<FootstepDataListMessage> footstepPublisher; // publish  the footsteps from the step generator to the controller
	private final IHMCROS2Publisher<FootstepDataListMessage> footstepVisualizationPublisher; // publish  the footsteps from the step generator
	                                                                                         // for visualization
	private final IHMCROS2Publisher<PauseWalkingMessage> pauseWalkingPublisher; // pause walking immediately

	// Footstep generation, validation and processing
	private final ContinuousStepGenerator continuousStepGenerator = new ContinuousStepGenerator();
	private final SnapAndWiggleSingleStep snapAndWiggleSingleStep;
	private final SnapAndWiggleSingleStepParameters snapAndWiggleParameters = new SnapAndWiggleSingleStepParameters();	
	private final ConvexPolygon2D footPolygonToWiggle = new ConvexPolygon2D();
	private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();
	private final RigidBodyTransform tempTransform = new RigidBodyTransform();
	private final PlanarRegion tempRegion = new PlanarRegion();
	
	// Scheduler for sub-tasks
	private final ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(4);

	// Comm to other nodes
	private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "ihmc_directional_control");

	public DirectionalControlController(FullHumanoidRobotModel robotModel, 
			                            DRCRobotModel robot,
			                            StatusMessageOutputManager statusOutputManager, 
			                            YoVariableRegistry registry) 
	{
		this(robotModel, robot, statusOutputManager, registry, false);
	}

	/**
	 * Main constructor.
	 * @param robotModel -- model of the robot joints
	 * @param robot -- behavioral model of the robot with parameters for walking, stepping and more
	 * @param statusOutputManager -- manages toolbox output status messages. Currently this controller does not have any.
	 * @param registry -- YoVariable registry for logging variables
	 * @param toolboxBypass -- Debug option to bypass toolbox control and communicate with the controller directly.
	 *                         Note that in this case, the topic names will be under /ihmc/valkyrie/humanoid_control/.
	 */
	public DirectionalControlController(FullHumanoidRobotModel robotModel, 
			                            DRCRobotModel robot,
			                            StatusMessageOutputManager statusOutputManager, 
			                            YoVariableRegistry registry,
			                            boolean toolboxBypass) 
	{
		super(statusOutputManager, registry);

		this.walkingControllerParameters = robot.getWalkingControllerParameters();
		this.robotName = robot.getSimpleRobotName();
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
		continuousStepGenerator.setDesiredVelocityProvider(
				() -> new Vector2D(forwardVelocityProperty.get(), lateralVelocityProperty.get()));
		continuousStepGenerator.configureWith(walkingControllerParameters);

		continuousStepGenerator.setFootstepAdjustment(this::adjustFootstep);
		continuousStepGenerator.setFootstepMessenger(this::prepareFootsteps);
		continuousStepGenerator.setFootPoseProvider(robotSide -> new FramePose3D(getSoleFrame(robotSide)));
		continuousStepGenerator.addFootstepValidityIndicator(this::isStepSnappable);
		continuousStepGenerator.addFootstepValidityIndicator(this::isSafeDistanceFromObstacle);
		continuousStepGenerator.addFootstepValidityIndicator(this::isSafeStepHeight);

		ROS2Tools.MessageTopicNameGenerator controllerPubGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
		ROS2Tools.MessageTopicNameGenerator controllerSubGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);

		// Status on whether a footstep started/completed
		ROS2Tools.createCallbackSubscription(ros2Node, 
				                             FootstepStatusMessage.class, controllerPubGenerator,
				                             s -> continuousStepGenerator.consumeFootstepStatus(s.takeNextData()));

		// REA planes
		ROS2Tools.createCallbackSubscription(ros2Node, 
				                             PlanarRegionsListMessage.class,
				                             REACommunicationProperties.publisherTopicNameGenerator,
				                             s -> planarRegionsListMessage.set(s.takeNextData()));

		// Inform if a controller failure occurs so we can ensure we stop walking 
		ROS2Tools.createCallbackSubscription(ros2Node, 
				                             WalkingControllerFailureStatusMessage.class,
				                             controllerPubGenerator, s -> stopWalking());
		
		// Robot configuration data updates (e.g. joint information)
		ROS2Tools.createCallbackSubscription(ros2Node, 
				                             RobotConfigurationData.class, 
				                             controllerPubGenerator,
				                             s -> updateRobotConfigurationData(s.takeNextData()));

		pauseWalkingPublisher = ROS2Tools.createPublisher(ros2Node, PauseWalkingMessage.class, controllerSubGenerator);
		footstepPublisher = ROS2Tools.createPublisher(ros2Node, FootstepDataListMessage.class, controllerSubGenerator);
		footstepVisualizationPublisher = ROS2Tools.createPublisher(ros2Node, FootstepDataListMessage.class, DirectionalControlModule.getPublisherTopicNameGenerator(robotName));

		// TODO: Collision box parameters taken from StepGeneratorJavaFXController.java
		// These are specific to the robot. To some degree, they should be derived from
		// some combination of:
		// - walking parameters
		// - physical properties
		// - step planning values
		double collisionBoxDepth = 0.65;
		double collisionBoxWidth = 1.15;
		double collisionBoxHeight = 1.0;
		double collisionXYProximityCheck = 0.01;
		collisionDetector = new BoundingBoxCollisionDetector();
		collisionDetector.setBoxDimensions(collisionBoxDepth, collisionBoxWidth, collisionBoxHeight,
				collisionXYProximityCheck);

		// Option to bypass toolbox control and accept messages directly sent to the controller
		if (toolboxBypass) {
			// Incoming directional input messages
			ROS2Tools.createCallbackSubscription(ros2Node, 
					                             DirectionalControlInputMessage.class, controllerSubGenerator,
					                             s -> updateInputs(s.takeNextData()));
			
			// Incoming directional configuration messages
			ROS2Tools.createCallbackSubscription(ros2Node, 
					                             DirectionalControlConfigurationMessage.class, 
					                             controllerSubGenerator,
					                             s -> updateConfiguration(s.takeNextData()));
			

			setupTasks();			
		}

	}
	
	/**
	 * Handler for robot configuration data updates
	 * @param message
	 */
	public void updateRobotConfigurationData(RobotConfigurationData message)
	{
		robotUpdater.updateConfiguration(message);
	}	

	/* There need to be two sets of update functions, one for messages and one for commands.
	 * When using this class as part of a toolbox, commands will be used. 
	 * When using it as a standalone class, messages will be used.
	 */
	public void updateConfiguration(DirectionalControlConfigurationCommand command) {
		controlConfigurationCommand.set(command);
		LogTools.info("Config is now " + controlConfigurationCommand.toString());
	}
	
	public void updateConfiguration(DirectionalControlConfigurationMessage message) {
		DirectionalControlConfigurationCommand command = new DirectionalControlConfigurationCommand();
		command.setFromMessage(message);
		updateConfiguration(command);
	}

	public void updateInputs(DirectionalControlInputCommand command) {
		controlInputCommand.set(command);
		LogTools.info("Input is now " + controlInputCommand.toString());
	}
	
	public void updateInputs(DirectionalControlInputMessage message) {
		DirectionalControlInputCommand command = new DirectionalControlInputCommand();
		command.setFromMessage(message);
		updateInputs(command);
	}

	/**
	 * Helper function to set up direct subscription when not going through the toolbox.
	 */
	private void setupTasks() {
		/*
		 * Main update task. Duties include: - handling incoming joystick control
		 * messages to control walking speed and direction - handling planar regions
		 * messages to guide footstep planning over terrain - calling the footstep
		 * generator to generate a new set of footsteps based on current parameters
		 */
		final Runnable task = new Runnable() {
			@Override
			public void run() {
				try {
					DirectionalControlController.this.updateInternal();
				} catch (Exception e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}

		};

		// Scheduled task to pick up any exceptions generated by the timer
		final ScheduledFuture<?> future = scheduler.scheduleAtFixedRate(task, 0, MAIN_TASK_RATE_MS,
				TimeUnit.MILLISECONDS);
		scheduler.execute(new Runnable() {
			@Override
			public void run() {
				try {
					future.get();
				} catch (InterruptedException | ExecutionException e) {
					e.printStackTrace();
					LogTools.error("Caught exception in main task: " + e);
					future.cancel(true);
				}
			}
		});
	}

	/**
	 * Create polygons for the right and left feet. The polygons here are rectangles of the same size.. 
	 * @param walkingControllerParameters
	 * @return
	 */
	private SideDependentList<ConvexPolygon2D> getFootPolygons(
			WalkingControllerParameters walkingControllerParameters) {
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

	private FullHumanoidRobotModel getFullRobotModel() {
		return robotUpdater.getRobot();
	}

	private MovingReferenceFrame getSoleFrame(RobotSide robotSide) {
		return getFullRobotModel().getSoleFrame(robotSide);
	}

	public double getSwingDuration() {
		return stepParametersReference.get().getSwingDuration();
	}

	public double getTransferDuration() {
		return stepParametersReference.get().getTransferDuration();
	}

	/**
	 * Turn the 2D desired foot pose from the step generator into a full 3D pose,
	 * allowing for planar regions.
	 * 
	 * @param footstepPose -- incoming 2D foot pose
	 * @param footSide     -- left or right
	 * @return 3D pose for the input step
	 */
	private FramePose3DReadOnly adjustFootstep(FramePose2DReadOnly footstepPose, RobotSide footSide) {
		FramePose3D adjustedBasedOnStanceFoot = new FramePose3D();
		adjustedBasedOnStanceFoot.getPosition().set(footstepPose.getPosition());
		// Initial Z position matches the stance foot
		adjustedBasedOnStanceFoot.setZ(continuousStepGenerator.getCurrentSupportFootPose().getZ());
		adjustedBasedOnStanceFoot.setOrientation(footstepPose.getOrientation());

		// If there are planar regions, attempt to modify the pose such that the foot
		// fits on a plane.
		if (planarRegionsList.get() != null && planarRegionsList.get().getNumberOfPlanarRegions() > 0) {
			FramePose3D wiggledPose = new FramePose3D(adjustedBasedOnStanceFoot);
			footPolygonToWiggle.set(footPolygons.get(footSide));
			try {
				snapAndWiggleSingleStep.snapAndWiggle(wiggledPose, footPolygonToWiggle,
						forwardVelocityProperty.get() > 0.0);
				if (wiggledPose.containsNaN())
					return adjustedBasedOnStanceFoot;
			} catch (SnappingFailedException e) {
				/*
				 * It's fine if the snap & wiggle fails, can be because there no planar regions
				 * around the footstep. Let's just keep the adjusted footstep based on the pose
				 * of the current stance foot.
				 */
			}
			return wiggledPose;
		} else {
			return adjustedBasedOnStanceFoot;
		}
	}

	private void updateForwardVelocity(double alpha) {
		double minMaxVelocity = stepParametersReference.get().getMaxStepLength() / stepTime.getValue();
		forwardVelocityProperty.set(minMaxVelocity * MathTools.clamp(alpha, 1.0));
	}

	private void updateLateralVelocity(double alpha) {
		double minMaxVelocity = stepParametersReference.get().getMaxStepWidth() / stepTime.getValue();
		lateralVelocityProperty.set(minMaxVelocity * MathTools.clamp(alpha, 1.0));
	}

	private void updateTurningVelocity(double alpha) {
		double minMaxVelocity = (stepParametersReference.get().getTurnMaxAngleOutward()
				- stepParametersReference.get().getTurnMaxAngleInward()) / stepTime.getValue();
		if (forwardVelocityProperty.get() < -1.0e-10)
			alpha = -alpha;
		turningVelocityProperty.set(minMaxVelocity * MathTools.clamp(alpha, 1.0));
	}

	/**
	 * Configure robot to start walking
	 */
	public void startWalking() {
		isWalking.set(true);
		continuousStepGenerator.startWalking();
	}

	/**
	 * Configure robot to stop walking
	 */
	public void stopWalking() {
		isWalking.set(false);
		footstepsToSendReference.set(null);
		continuousStepGenerator.stopWalking();
		sendPauseMessage();
	}

	/**
	 * Send a pause walking message to the walking controller
	 */
	private void sendPauseMessage() {
		PauseWalkingMessage pauseWalkingMessage = new PauseWalkingMessage();
		pauseWalkingMessage.setPause(true);
		pauseWalkingPublisher.publish(pauseWalkingMessage);
	}

	/**
	 * Take the footsteps generated by the continuous step generator and adjust according to 
	 * our walking parameters.
	 * @param footstepDataListMessage
	 */
	private void prepareFootsteps(FootstepDataListMessage footstepDataListMessage) {
		for (int i = 0; i < footstepDataListMessage.getFootstepDataList().size(); i++) {
			FootstepDataMessage footstepDataMessage = footstepDataListMessage.getFootstepDataList().get(i);
			footstepDataMessage.setSwingHeight(stepParametersReference.get().getSwingHeight());
		}
		footstepsToSendReference.set(new FootstepDataListMessage(footstepDataListMessage));
	}

	/**
	 * Publish the footsteps to the walking controller, if walking is active.
	 */
	private void publishFootsteps() {
		FootstepDataListMessage footstepsToSend = footstepsToSendReference.getAndSet(null);
		if (footstepsToSend != null) {
		   if (isWalking.get()) {
			footstepPublisher.publish(footstepsToSend);
		   }
		   footstepVisualizationPublisher.publish(footstepsToSend);
		}
		if (!isWalking.get())
			sendPauseMessage();
	}

	/**
	 * Determine whether this step is stepping up or down too far to be safe
	 * @param touchdownPose -- expected pose of the foot when it touches down
	 * @param stancePose -- pose of the current planted foot
	 * @param swingSide -- which foot will be taking the step
	 * @return -- True if the the step is safe; false otherwise
	 */
	private boolean isSafeStepHeight(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose,
			RobotSide swingSide) {
		double heightChange = touchdownPose.getZ() - stancePose.getZ();
		return heightChange < steppingParameters.getMaxStepUp() && heightChange > -steppingParameters.getMaxStepDown();
	}

	/**
	 * Determine whether the footstep places the robot too close to an obstacle
	 * @param touchdownPose -- expected pose of the foot when it touches down
	 * @param stancePose -- pose of the current planted foot
	 * @param swingSide -- which foot will be taking the step
	 * @return -- True if the step is safe; false otherwise
	 */
	private boolean isSafeDistanceFromObstacle(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose,
			RobotSide swingSide) {
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
		collisionDetector.setBoxPose(touchdownPose.getX() + offsetX, touchdownPose.getY() + offsetY,
				touchdownPose.getZ() + heightOffset, soleYaw);

		return !collisionDetector.checkForCollision().isCollisionDetected();
	}

	/**
	 * Determine whether the step can be snapped to a planar region
	 * @param touchdownPose -- expected pose of the foot when it touches down
	 * @param stancePose -- pose of the current planted foot
	 * @param swingSide -- which foot will be taking the step
	 * @return -- True if the step can be snapped; false otherwise
	 */
	private boolean isStepSnappable(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose,
			RobotSide swingSide) {
		if (planarRegionsList.get() == null)
			return true;

		tempTransform.setTranslation(touchdownPose.getPosition().getX(), touchdownPose.getPosition().getY(), 0.0);
		tempTransform.setRotationYaw(touchdownPose.getYaw());

		footPolygon.set(footPolygons.get(swingSide));
		footPolygon.applyTransform(tempTransform, false);

		PlanarRegionsList planarRegionsList = this.planarRegionsList.get();
		return PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon, planarRegionsList,
				tempRegion) != null;
	}
	
	/**
	 * Handle incoming changes to direction inputs. Note that the input message values need to be mapped to
	 * how the footstep generator sees the world.
	 */
	protected void updateDirectionalInputs() {
		// Handle directional inputs
		DirectionalControlInputCommand inputMessage = controlInputCommand.getAndSet(null);
		if (inputMessage != null) {
			updateForwardVelocity(inputMessage.getForward());
			updateLateralVelocity(-inputMessage.getRight());
			updateTurningVelocity(-inputMessage.getClockwise());
		}		
	}
	
	/**
	 * Handle incoming changes to controller state
	 */
	protected void updateStateInputs() {
		// Handle configuration inputs
		DirectionalControlConfigurationCommand controlMessage = controlConfigurationCommand.getAndSet(null);
		if (controlMessage != null) {
			String profile = controlMessage.getProfileName();
			if (profile.length() > 0) {
				try {
					controlParameters = userProfileManager.loadProfile(profile);
					stepParametersReference = new AtomicReference<JoystickStepParameters>(controlParameters);
					LogTools.info("Switched profile to " + profile);
				} catch (RuntimeException e) {
					System.out.printf("Unable to load profile %s: %s\n", profile, e);
				}
			}
			if (isWalking.get() && !controlMessage.getEnableWalking()) {
				stopWalking();
			} else if (!isWalking.get() && controlMessage.getEnableWalking()) {
				startWalking();
			}
		}		
	}

	protected void handleIncomingMessages() {
		updateDirectionalInputs();
		updateStateInputs();
	}

	/**
	 * Shut down all activity and support tasks
	 */
	public void shutdown() {
		scheduler.shutdownNow();
		PauseWalkingMessage pauseWalkingMessage = new PauseWalkingMessage();
		pauseWalkingMessage.setPause(true);
		pauseWalkingPublisher.publish(pauseWalkingMessage);
		ros2Node.destroy();
	}

	@Override
	public boolean initialize() {
		// Should return true when the controller is initialized.
		// This controller is born ready.
		return true;
	}

	/**
	 * Main loop update function called from the toolbox or from a scheduled task if bypassing the toolbox.
	 * This function handles:
	 * - changing walking parameters based on input messages
	 * - updating our planar regions list
	 * - updating step generator control parameters and getting a new footstep plan
	 * - publishing the new footstep plan
	 */
	@Override
	public void updateInternal() throws Exception {
		
		try {
			DirectionalControlController.this.handleIncomingMessages();
			PlanarRegionsListMessage latestMessage = planarRegionsListMessage.getAndSet(null);
			if (latestMessage != null) {
				PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(latestMessage);
				snapAndWiggleSingleStep.setPlanarRegions(planarRegionsList);
				collisionDetector.setPlanarRegionsList(
						new PlanarRegionsList(planarRegionsList.getPlanarRegionsAsList().stream().filter(
								region -> region.getConvexHull().getArea() >= snapAndWiggleParameters.getMinPlanarRegionArea()).collect(Collectors.toList())));
				DirectionalControlController.this.planarRegionsList.set(planarRegionsList);
			}

			JoystickStepParameters stepParameters = stepParametersReference.get();
			continuousStepGenerator.setFootstepTiming(stepParameters.getSwingDuration(), stepParameters.getTransferDuration());
			continuousStepGenerator.update(Double.NaN);
			publishFootsteps();

		} catch (Throwable e) {
			e.printStackTrace();
			LogTools.error("Caught exception, stopping timer.");
		}
	}

	/**
	 * This function allows the controller to indicate that it has no active task to work on, and the toolbox may sleep.
	 * This capability is not used here. Instead, we rely on the natural message flow.   
	 */
	@Override
	public boolean isDone() {
		return false;
	}


	/**
	 * Receive notifications when the state of this toolbox is changing.
	 * We use this to determine that the toolbox has decided to sleep. In this case, we stop walking
	 * immediately rather than allowing the current plan to complete.
	 * 
	 * @param newState the new state this toolbox is about to enter.
	 */
	@Override
	public void notifyToolboxStateChange(ToolboxState newState) {
		if (newState == ToolboxState.SLEEP) {
			stopWalking();
		}
		LogTools.info("Directional controller state is now " + newState.toString());
	}

}
