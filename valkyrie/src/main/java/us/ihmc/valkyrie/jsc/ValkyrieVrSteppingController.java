package us.ihmc.valkyrie.jsc;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import controller_msgs.msg.dds.JoystickControl;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import us.ihmc.avatar.joystickBasedJavaFXController.UserProfileManager;
import us.ihmc.avatar.joystickBasedJavaFXController.JoystickStepParametersProperty.JoystickStepParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.footstepPlanning.graphSearch.collision.BoundingBoxCollisionDetector;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStep;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStepParameters;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStep.SnappingFailedException;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.yoVariables.providers.DoubleProvider;
import java.util.concurrent.TimeUnit;

/**
 * Controller for walking the robot using virtual joystick messages over a topic.
 * 
 * @author Mark Paterson (heavily based on existing hardware joystick walking code)
 *
 */
@SuppressWarnings("restriction")
public class ValkyrieVrSteppingController {

	private final ContinuousStepGenerator continuousStepGenerator = new ContinuousStepGenerator();
	private final DoubleProperty turningVelocityProperty = new SimpleDoubleProperty(this, "turningVelocityProperty",
			0.0);
	private final DoubleProperty forwardVelocityProperty = new SimpleDoubleProperty(this, "forwardVelocityProperty",
			0.0);
	private final DoubleProperty lateralVelocityProperty = new SimpleDoubleProperty(this, "lateralVelocityProperty",
			0.0);

	private final AtomicReference<PlanarRegionsListMessage> planarRegionsListMessage = new AtomicReference<>(null);
	private final AtomicReference<FootstepDataListMessage> footstepsToSendReference = new AtomicReference<>(null);
	private final AtomicReference<JoystickStepParameters> stepParametersReference;
	private final AtomicBoolean isWalking = new AtomicBoolean(false);
	private final AtomicReference<PlanarRegionsList> planarRegionsList = new AtomicReference<>(null);
	
	private final DoubleProvider stepTime;
	private final BoundingBoxCollisionDetector collisionDetector;
	private final SteppingParameters steppingParameters;
	private final IHMCROS2Publisher<FootstepDataListMessage> footstepPublisher;
	private final IHMCROS2Publisher<PauseWalkingMessage> pauseWalkingPublisher;

	private final SnapAndWiggleSingleStep snapAndWiggleSingleStep;
	private final SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons;
	private final ConvexPolygon2D footPolygonToWiggle = new ConvexPolygon2D();
	private final ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(4);

	private JoystickControl joystickControlMessage = new JoystickControl();
	private JoystickStepParameters joystickParameters;
	private UserProfileManager<JoystickStepParameters> userProfileManager;
	private final ValkyrieRobotModelUpdater robotUpdater; // Keeps the robotModel up-to-date with configuration changes
	private final int SEND_FOOTSTEP_RATE_MS = 1000; // How often to send footstep (milliseconds)
	private final int MAIN_TASK_RATE_MS = 1000; // How often to check tasks for exceptions (milliseconds)
	
	private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();
	private final RigidBodyTransform tempTransform = new RigidBodyTransform();
	private final PlanarRegion tempRegion = new PlanarRegion();	

	public ValkyrieVrSteppingController(ValkyrieRobotModel robotModel, String robotName,
			WalkingControllerParameters walkingControllerParameters, Ros2Node ros2Node,
			SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons) {
		// trajectoryDuration = messager.createInput(WalkingTrajectoryDuration, 1.0);

		this.footPolygons = footPolygons;
		this.robotUpdater = new ValkyrieRobotModelUpdater(robotModel);
		Double trajectoryDurationInSecs = 1.0;
		new AtomicReference<Double>(trajectoryDurationInSecs);

		steppingParameters = walkingControllerParameters.getSteppingParameters();
		stepTime = () -> getSwingDuration() + getTransferDuration();

		joystickParameters = new JoystickStepParameters(walkingControllerParameters);

		userProfileManager = new UserProfileManager<JoystickStepParameters>(null, joystickParameters,
				JoystickStepParameters::parseFromPropertyMap, JoystickStepParameters::exportToPropertyMap);
		joystickParameters = userProfileManager.loadProfile("default");
		stepParametersReference = new AtomicReference<JoystickStepParameters>(joystickParameters);

		SnapAndWiggleSingleStepParameters parameters = new SnapAndWiggleSingleStepParameters();
		parameters.setFootLength(walkingControllerParameters.getSteppingParameters().getFootLength());
		snapAndWiggleSingleStep = new SnapAndWiggleSingleStep(parameters);
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

		ROS2Tools.MessageTopicNameGenerator controllerPubGenerator = ControllerAPIDefinition
				.getPublisherTopicNameGenerator(robotName);
		ROS2Tools.MessageTopicNameGenerator controllerSubGenerator = ControllerAPIDefinition
				.getSubscriberTopicNameGenerator(robotName);

		// Subscribe to robot configuration updates so we can update the local robot model
		ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class,
				ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
				s -> robotUpdater.updateConfiguration(s.takeNextData()));

		// Status on whether a footstep started/completed
		ROS2Tools.createCallbackSubscription(ros2Node, FootstepStatusMessage.class, controllerPubGenerator,
				s -> continuousStepGenerator.consumeFootstepStatus(s.takeNextData()));

		// Incoming control messages
		ROS2Tools.createCallbackSubscription(ros2Node, JoystickControl.class, controllerSubGenerator,
				s -> joystickControlMessage.set(s.takeNextData()));

		// REA planes
		ROS2Tools.createCallbackSubscription(ros2Node, PlanarRegionsListMessage.class,
				REACommunicationProperties.publisherTopicNameGenerator,
				s -> planarRegionsListMessage.set(s.takeNextData()));

		// Ensure we stop walking if a controller failure occurs
		ROS2Tools.createCallbackSubscription(ros2Node, WalkingControllerFailureStatusMessage.class,
				controllerPubGenerator, s -> stopWalking(true));

		pauseWalkingPublisher = ROS2Tools.createPublisher(ros2Node, PauseWalkingMessage.class, controllerSubGenerator);
		footstepPublisher = ROS2Tools.createPublisher(ros2Node, FootstepDataListMessage.class, controllerSubGenerator);

		double collisionBoxDepth = 0.65;
		double collisionBoxWidth = 1.15;
		double collisionBoxHeight = 1.0;
		double collisionXYProximityCheck = 0.01;
		collisionDetector = new BoundingBoxCollisionDetector();
		collisionDetector.setBoxDimensions(collisionBoxDepth, collisionBoxWidth, collisionBoxHeight,
				collisionXYProximityCheck);

		/* Main update task. Duties include:
		 * - handling incoming joystick control messages to control walking speed and direction
		 * - handling planar regions messages to guide footstep planning over terrain
		 * - calling the footstep generator to generate a new set of footsteps based on current parameters
		 */
		final Runnable task = new Runnable() {
			@Override
			public void run() {
				try {
					ValkyrieVrSteppingController.this.handleJoystickMessage();
					PlanarRegionsListMessage latestMessage = planarRegionsListMessage.getAndSet(null);
					if (latestMessage != null) {
						PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter
								.convertToPlanarRegionsList(latestMessage);
						snapAndWiggleSingleStep.setPlanarRegions(planarRegionsList);
						collisionDetector.setPlanarRegionsList(new PlanarRegionsList(planarRegionsList
								.getPlanarRegionsAsList().stream().filter(region -> region.getConvexHull()
										.getArea() >= parameters.getMinPlanarRegionArea())
								.collect(Collectors.toList())));
						ValkyrieVrSteppingController.this.planarRegionsList.set(planarRegionsList);
					}

					JoystickStepParameters stepParameters = stepParametersReference.get();
					continuousStepGenerator.setFootstepTiming(stepParameters.getSwingDuration(),
							stepParameters.getTransferDuration());
					continuousStepGenerator.update(Double.NaN);

				} catch (Throwable e) {
					e.printStackTrace();
					LogTools.error("Caught exception, stopping timer.");
				}
			}

		};

		// Scheduled task to pick up any exceptions generated by the timer
		final ScheduledFuture<?> future = scheduler.scheduleAtFixedRate(task, 0, MAIN_TASK_RATE_MS, TimeUnit.MILLISECONDS);
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

		// Send current footsteps at a fixed rate, once enabled by startWalking()
		scheduler.scheduleAtFixedRate(this::publishFootsteps, 0, SEND_FOOTSTEP_RATE_MS, TimeUnit.MILLISECONDS);

		startWalking(true);
		System.out.println("End of controller constructor");
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
	 * Turn the 2D desired foot pose from the step generator into a full 3D pose, allowing for
	 * planar regions.
	 * @param footstepPose -- incoming 2D foot pose
	 * @param footSide -- left or right
	 * @return 3D pose for the input step
	 */
	private FramePose3DReadOnly adjustFootstep(FramePose2DReadOnly footstepPose, RobotSide footSide) {
		FramePose3D adjustedBasedOnStanceFoot = new FramePose3D();
		adjustedBasedOnStanceFoot.getPosition().set(footstepPose.getPosition());
		// Initial Z position matches the stance foot
		adjustedBasedOnStanceFoot.setZ(continuousStepGenerator.getCurrentSupportFootPose().getZ());
		adjustedBasedOnStanceFoot.setOrientation(footstepPose.getOrientation());

		// If there are planar regions, attempt to modify the pose such that the foot fits on a plane.
		if (planarRegionsList.get() != null && planarRegionsList.get().getNumberOfPlanarRegions() > 0) {
			FramePose3D wiggledPose = new FramePose3D(adjustedBasedOnStanceFoot);
			footPolygonToWiggle.set(footPolygons.get(footSide));
			try {
				snapAndWiggleSingleStep.snapAndWiggle(wiggledPose, footPolygonToWiggle,
						forwardVelocityProperty.get() > 0.0);
				if (wiggledPose.containsNaN()) return adjustedBasedOnStanceFoot;
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

	private void startWalking(boolean confirm) {
		if (confirm) {
			isWalking.set(true);
			continuousStepGenerator.startWalking();
		}
	}

	private void stopWalking(boolean confirm) {
		if (confirm) {
			isWalking.set(false);
			footstepsToSendReference.set(null);
			continuousStepGenerator.stopWalking();
			sendPauseMessage();
		}
	}

	private void sendPauseMessage() {
		PauseWalkingMessage pauseWalkingMessage = new PauseWalkingMessage();
		pauseWalkingMessage.setPause(true);
		pauseWalkingPublisher.publish(pauseWalkingMessage);
	}

	private void prepareFootsteps(FootstepDataListMessage footstepDataListMessage) {
		for (int i = 0; i < footstepDataListMessage.getFootstepDataList().size(); i++) {
			FootstepDataMessage footstepDataMessage = footstepDataListMessage.getFootstepDataList().get(i);
			footstepDataMessage.setSwingHeight(stepParametersReference.get().getSwingHeight());
		}
		footstepsToSendReference.set(new FootstepDataListMessage(footstepDataListMessage));
	}

	/**
	 * Publish the footsteps, if walking is active.
	 */
	private void publishFootsteps() {
		FootstepDataListMessage footstepsToSend = footstepsToSendReference.getAndSet(null);
		if (footstepsToSend != null && isWalking.get()) {
			footstepPublisher.publish(footstepsToSend);
		}
		if (!isWalking.get())
			sendPauseMessage();
	}

	// Is this step too far down?
	private boolean isSafeStepHeight(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose,
			RobotSide swingSide) {
		double heightChange = touchdownPose.getZ() - stancePose.getZ();
		return heightChange < steppingParameters.getMaxStepUp() && heightChange > -steppingParameters.getMaxStepDown();
	}

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

	protected void handleJoystickMessage() {
		updateForwardVelocity(joystickControlMessage.forward_);
		updateLateralVelocity(joystickControlMessage.right_);
		updateTurningVelocity(joystickControlMessage.turn_);
	}

	/**
	 * Shut down all activity and support tasks
	 */
	public void shutdown() {
		scheduler.shutdownNow();
		PauseWalkingMessage pauseWalkingMessage = new PauseWalkingMessage();
		pauseWalkingMessage.setPause(true);
		pauseWalkingPublisher.publish(pauseWalkingMessage);
	}

}
