package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import static controller_msgs.msg.dds.KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_INITIALIZE_FAILURE_MISSING_RCD;
import static controller_msgs.msg.dds.KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_INITIALIZE_SUCCESSFUL;
import static controller_msgs.msg.dds.KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_RUNNING;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage;
import controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import gnu.trove.map.hash.TIntObjectHashMap;
import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision.KinematicsCollidable;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision.KinematicsCollisionResult;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPIDSE3Gains;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.time.ThreadTimer;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePose3D;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * {@code KinematicsToolboxController} is used as a whole-body inverse kinematics solver.
 * <p>
 * The interaction with this solver is achieved over the network using message that define the API,
 * see {@link KinematicsToolboxModule}.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class KinematicsToolboxController extends ToolboxController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double DEFAULT_PRIVILEGED_CONFIGURATION_WEIGHT = 0.025;
   private static final double DEFAULT_PRIVILEGED_CONFIGURATION_GAIN = 50.0;

   /**
    * This name is used with the {@code Map} {@link #userFeedbackCommands} to keep track of the active
    * set of commands provided by the caller.
    */
   private static final String centerOfMassName = "CenterOfMass";
   /**
    * Indicates the duration of a control tick. It should match the thread period in
    * {@link ToolboxModule}.
    */
   protected final double updateDT;

   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   /** Reference to the desired robot's root body. */
   protected final RigidBodyBasics rootBody;
   /** Reference to the desired robot's floating joint. */
   protected final FloatingJointBasics rootJoint;
   /**
    * Array containing all the one degree-of-freedom joints of the desired robot except for the finger
    * joints that are not handled by this solver.
    */
   private final OneDoFJointBasics[] oneDoFJoints;
   private final TIntObjectHashMap<OneDoFJointBasics> jointHashCodeMap = new TIntObjectHashMap<>();

   /**
    * Reference frame centered at the robot's center of mass. It is used to hold the initial center of
    * mass position when requested.
    */
   protected final ReferenceFrame centerOfMassFrame;

   /** The same set of gains is used for controlling any part of the desired robot body. */
   private final YoPIDSE3Gains gains = new DefaultYoPIDSE3Gains("GenericGains", GainCoupling.XYZ, false, registry);
   /**
    * Default settings for the solver. The joint velocity/acceleration weights can be modified at
    * runtime using {@link KinematicsToolboxConfigurationMessage}.
    */
   private final KinematicsToolboxOptimizationSettings optimizationSettings = new KinematicsToolboxOptimizationSettings();
   /** Command carrying the current optimization settings. */
   private final InverseKinematicsOptimizationSettingsCommand activeOptimizationSettings = new InverseKinematicsOptimizationSettingsCommand();
   /**
    * The controller core command is the single object used to pass the desired inputs to the
    * controller core.
    */
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_KINEMATICS);
   /**
    * This is where the magic is happening. The controller is responsible for performing feedback
    * control to reduce the difference between the desireds sent to the
    * {@code KinematicsToolboxController} and the actual robot pose. It is also responsible for
    * gathering the entire set of desired inputs and formulate the adequate optimization problem to be
    * solved for every control tick. The output of the controller core provides every tick a new robot
    * joint configurations and velocities that are one step closer to the desireds. The output is used
    * to update the state of the {@link #desiredFullRobotModel} such that it progresses towards the
    * desired user inputs over time.
    */
   private final WholeBodyControllerCore controllerCore;
   /**
    * This holds onto the data from the feedback controllers running inside the controller core.
    * {@link #feedbackControllerDataHolder} is used here to compute the solution quality every tick
    * from the tracking error for each end-effector being controlled.
    */
   private final FeedbackControllerDataReadOnly feedbackControllerDataHolder;

   /**
    * This is the output of the {@code KinematicsToolboxController}. It is filled with the robot
    * configuration obtained from {@link #desiredFullRobotModel} and also with the solution quality
    * which can be used to quickly see if the solution is viable. It is sent back to the caller only.
    */
   private final KinematicsToolboxOutputStatus inverseKinematicsSolution;
   /** Variable to keep track of when the last solution was published. */
   private final YoDouble timeSinceLastSolutionPublished = new YoDouble("timeSinceLastSolutionPublished", registry);
   /** Specifies time interval for publishing the solution. */
   private final YoDouble publishSolutionPeriod = new YoDouble("publishSolutionPeriod", registry);
   /**
    * This is the current estimate of the solution quality that is calculated based on the tracking
    * error for the end-effectors (center of mass included) being actively controlled.
    */
   private final YoDouble solutionQuality = new YoDouble("solutionQuality", registry);

   /**
    * Weight indicating the priority for getting closer to the current privileged configuration. The
    * current privileged configuration can be changed at any time by sending a
    * {@link HumanoidKinematicsToolboxConfigurationMessage}.
    */
   private final YoDouble privilegedWeight = new YoDouble("privilegedWeight", registry);
   /**
    * To make the robot get closer to the privileged configuration, a feedback control is used to
    * compute for each joint a privileged velocity based on the difference between the privileged angle
    * and the current angle. These privileged joint velocities are then used to complete the
    * optimization problem in such way that they don't interfere with the user commands.
    */
   private final YoDouble privilegedConfigurationGain = new YoDouble("privilegedConfigurationGain", registry);
   /**
    * Cap used to limit the magnitude of the privileged joint velocities computed in the controller
    * core. Should probably remain equal to {@link Double#POSITIVE_INFINITY} so the solution converges
    * quicker.
    */
   private final YoDouble privilegedMaxVelocity = new YoDouble("privilegedMaxVelocity", registry);
   private TObjectDoubleHashMap<OneDoFJointBasics> defaultPrivilegedConfigurationMap = null;
   /**
    * This reference to {@link PrivilegedConfigurationCommand} is used internally only to figure out if
    * the current privileged configuration used in the controller core is to be updated or not. It is
    * usually updated once right after the initialization phase.
    */
   private final AtomicReference<PrivilegedConfigurationCommand> privilegedConfigurationCommandReference = new AtomicReference<>(null);
   /**
    * The {@link #commandInputManager} is used as a 'thread-barrier'. When receiving a new user input,
    * this manager automatically copies the data in the corresponding command that can then be used
    * here safely.
    */
   protected final CommandInputManager commandInputManager;
   /**
    * Visualization of the desired end-effector poses seen as coordinate systems in the
    * {@code SCSVisualizer}. They are only visible when the end-effector is being actively controlled.
    */
   private final Map<RigidBodyBasics, YoGraphicCoordinateSystem> desiredCoodinateSystems = new HashMap<>();
   /**
    * Visualization of the current end-effector poses seen as coordinate systems in the
    * {@code SCSVisualizer}. They are only visible when the end-effector is being actively controlled.
    */
   private final Map<RigidBodyBasics, YoGraphicCoordinateSystem> currentCoodinateSystems = new HashMap<>();

   /**
    * Reference to the most recent robot configuration received from the controller. It is used for
    * initializing the {@link #desiredFullRobotModel} before starting the optimization process.
    */
   private final AtomicReference<RobotConfigurationData> latestRobotConfigurationDataReference = new AtomicReference<>(null);

   /**
    * Map of the commands requested during two initialization phases by the user. It used to memorize
    * active commands such that the caller does not have to send the same commands every control tick
    * of this solver.
    */
   private final Map<String, FeedbackControlCommand<?>> userFeedbackCommands = new HashMap<>();

   /**
    * This is mostly for visualization to be able to keep track of the number of commands that the user
    * submitted.
    */
   private final YoInteger numberOfActiveCommands = new YoInteger("numberOfActiveCommands", registry);
   /**
    * When {@code true}, a user command will persist over time until the method {@link #initialize()}
    * is called.
    */
   private final YoBoolean preserveUserCommandHistory = new YoBoolean("preserveUserCommandHistory", registry);

   /** Represents the collision model of the robot. */
   private final List<KinematicsCollidable> robotCollidables = new ArrayList<>();
   /**
    * User parameter updated via {@link KinematicsToolboxConfigurationMessage}. Collision is only
    * handled when this is set to {@code true}.
    */
   private final YoBoolean enableCollisionAvoidance = new YoBoolean("enableCollisionAvoidance", registry);
   /**
    * Threshold for activating collision response, i.e. when 2 collidables are within a distance that
    * is less than this value, only then the solver handles it. This is for reducing computational
    * burden.
    */
   private final YoDouble collisionActivationDistanceThreshold = new YoDouble("collisionActivationDistanceThreshold", registry);
   /** Sets the maximum number of collisions to create YoVariables for. */
   private final int numberOfCollisionsToVisualize = 20;
   /** Debug variable. */
   private final YoDouble[] collisionDistances = new YoDouble[numberOfCollisionsToVisualize];
   /** Debug variable. */
   private final YoFramePoint3D[] collisionPointAs = new YoFramePoint3D[numberOfCollisionsToVisualize];
   /** Debug variable. */
   private final YoFramePoint3D[] collisionPointBs = new YoFramePoint3D[numberOfCollisionsToVisualize];
   /** Debug variable. */
   private final YoFramePose3D[] collisionFramePoses = new YoFramePose3D[numberOfCollisionsToVisualize];
   /** Timer to debug computational load. */
   private final ThreadTimer threadTimer;

   /**
    * @param commandInputManager     the message/command barrier used by this controller. Submit
    *                                messages or commands to be processed to the
    *                                {@code commandInputManager} from outside the controller.
    * @param statusOutputManager     the output interface used by this controller.
    * @param rootJoint               the underactuated floating root joint of the multi-body system.
    *                                Can be {@code null} in the case all the joints are actuated.
    * @param oneDoFJoints            the actuated joints of the system. The inverse kinematics will
    *                                only use these joints during the optimization.
    * @param controllableRigidBodies the sublist of rigid-bodies that can be controlled by the user.
    *                                Can be {@code null} in the case all rigid-body should be
    *                                controllable.
    * @param updateDT                the period of one optimization tick.
    * @param yoGraphicsListRegistry  registry to register visualization to.
    * @param parentRegistry          registry to attach {@code YoVariable}s to.
    */
   public KinematicsToolboxController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager, FloatingJointBasics rootJoint,
                                      OneDoFJointBasics[] oneDoFJoints, Collection<? extends RigidBodyBasics> controllableRigidBodies, double updateDT,
                                      YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);
      this.commandInputManager = commandInputManager;
      this.rootJoint = rootJoint;
      this.oneDoFJoints = oneDoFJoints;
      this.updateDT = updateDT;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      // This will find the root body without using rootJoint so it can be null.
      rootBody = MultiBodySystemTools.getRootBody(oneDoFJoints[0].getPredecessor());

      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", worldFrame, rootBody);

      Arrays.stream(oneDoFJoints).forEach(joint -> jointHashCodeMap.put(joint.hashCode(), joint));

      controllerCore = createControllerCore(controllableRigidBodies);
      feedbackControllerDataHolder = controllerCore.getWholeBodyFeedbackControllerDataHolder();

      inverseKinematicsSolution = MessageTools.createKinematicsToolboxOutputStatus(oneDoFJoints);
      inverseKinematicsSolution.setDestination(-1);

      gains.setPositionProportionalGains(1200.0); // Gains used for everything. It is as high as possible to reduce the convergence time.
      gains.setPositionMaxFeedbackAndFeedbackRate(1500.0, Double.POSITIVE_INFINITY);
      gains.setOrientationProportionalGains(1200.0); // Gains used for everything. It is as high as possible to reduce the convergence time.
      gains.setOrientationMaxFeedbackAndFeedbackRate(1500.0, Double.POSITIVE_INFINITY);

      privilegedWeight.set(DEFAULT_PRIVILEGED_CONFIGURATION_WEIGHT);
      privilegedConfigurationGain.set(DEFAULT_PRIVILEGED_CONFIGURATION_GAIN);
      privilegedMaxVelocity.set(Double.POSITIVE_INFINITY);

      publishSolutionPeriod.set(0.01);
      preserveUserCommandHistory.set(true);

      threadTimer = new ThreadTimer("timer", updateDT, registry);

      enableCollisionAvoidance.set(true);
      collisionActivationDistanceThreshold.set(0.10);
      setupCollisionVisualization();
   }

   /**
    * Creates the debug variables and graphics for the collisions.
    */
   private void setupCollisionVisualization()
   {
      Random random = new Random();

      for (int i = 0; i < numberOfCollisionsToVisualize; i++)
      {
         YoDouble collisionDistance = new YoDouble("collision_" + i + "_distance", registry);
         YoFramePoint3D collisionPointA = new YoFramePoint3D("collision_" + i + "_pointA" + i, worldFrame, registry);
         YoFramePoint3D collisionPointB = new YoFramePoint3D("collision_" + i + "_pointB" + i, worldFrame, registry);
         YoFramePose3D collisionFramePose = new YoFramePose3D("collision_" + i + "_frame", worldFrame, registry);

         AppearanceDefinition appearance = new YoAppearanceRGBColor(new Color(random.nextInt()), 0.7);
         yoGraphicsListRegistry.registerYoGraphic("Collisions", new YoGraphicPosition("collision_" + i + "_pointA", collisionPointA, 0.01, appearance));
         yoGraphicsListRegistry.registerYoGraphic("Collisions", new YoGraphicPosition("collision_" + i + "_pointB", collisionPointB, 0.01, appearance));
         yoGraphicsListRegistry.registerYoGraphic("Collisions",
                                                  new YoGraphicCoordinateSystem("collision_" + i + "_frame", collisionFramePose, 0.1, appearance));

         collisionDistances[i] = collisionDistance;
         collisionPointAs[i] = collisionPointA;
         collisionPointBs[i] = collisionPointB;
         collisionFramePoses[i] = collisionFramePose;
      }
   }

   public void setDefaultPrivilegedConfiguration(TObjectDoubleHashMap<OneDoFJointBasics> defaultPrivilegedConfigurationMap)
   {
      this.defaultPrivilegedConfigurationMap = defaultPrivilegedConfigurationMap;
   }

   /**
    * Registers a new collidable to be used with this solver for preventing collisions.
    * 
    * @param collidable the new collidable to consider.
    */
   public void registerCollidable(KinematicsCollidable collidable)
   {
      robotCollidables.add(collidable);
   }

   /**
    * Registers new collidables to be used with this solver for preventing collisions.
    * 
    * @param collidables the new collidables to consider.
    */
   public void registerCollidables(KinematicsCollidable... collidables)
   {
      for (KinematicsCollidable collidable : collidables)
         robotCollidables.add(collidable);
   }

   /**
    * Registers new collidables to be used with this solver for preventing collisions.
    * 
    * @param collidables the new collidables to consider.
    */
   public void registerCollidables(Iterable<? extends KinematicsCollidable> collidables)
   {
      for (KinematicsCollidable collidable : collidables)
         robotCollidables.add(collidable);
   }

   /**
    * This is where the end-effectors needing a visualization are registered, if you need more, add it
    * there.
    * 
    * @param rigidBodies all the rigid bodies for which the desired and actual pose will be displayed
    *                    using graphical coordinate systems.
    */
   public void setupVisualization(RigidBodyBasics... rigidBodies)
   {
      AppearanceDefinition desiredAppearance = YoAppearance.Red();
      AppearanceDefinition currentAppearance = YoAppearance.Blue();

      for (RigidBodyBasics rigidBody : rigidBodies)
      {
         YoGraphicCoordinateSystem desiredCoodinateSystem = createCoodinateSystem(rigidBody, Type.DESIRED, desiredAppearance);
         YoGraphicCoordinateSystem currentCoodinateSystem = createCoodinateSystem(rigidBody, Type.CURRENT, currentAppearance);

         desiredCoodinateSystems.put(rigidBody, desiredCoodinateSystem);
         currentCoodinateSystems.put(rigidBody, currentCoodinateSystem);

         yoGraphicsListRegistry.registerYoGraphic("CoordinateSystems", desiredCoodinateSystem);
         yoGraphicsListRegistry.registerYoGraphic("CoordinateSystems", currentCoodinateSystem);
      }
   }

   /**
    * Convenience method that should only be used for setting up the visualization.
    * <p>
    * Simply creates a graphic coordinate system visualizable in {@code SCSVisualizer}.
    * </p>
    *
    * @param endEffector          used to create a name prefix required for creating a
    *                             {@link YoGraphicCoordinateSystem}.
    * @param type                 used to create a name prefix required for creating a
    *                             {@link YoGraphicCoordinateSystem}.
    * @param appearanceDefinition the appearance of the coordinate system's arrows.
    * @return the graphic with a good name for the given end-effector.
    */
   private YoGraphicCoordinateSystem createCoodinateSystem(RigidBodyBasics endEffector, Type type, AppearanceDefinition appearanceDefinition)
   {
      String namePrefix = endEffector.getName() + type.getName();
      return new YoGraphicCoordinateSystem(namePrefix, "", registry, false, 0.2, appearanceDefinition);
   }

   /**
    * Creating the controller core which is the main piece of this solver.
    * 
    * @param controllableRigidBodies
    * @return the controller core that will run for the desired robot {@link #desiredFullRobotModel}.
    */
   private WholeBodyControllerCore createControllerCore(Collection<? extends RigidBodyBasics> controllableRigidBodies)
   {
      JointBasics[] controlledJoints;
      if (rootJoint != null)
      {
         controlledJoints = new JointBasics[oneDoFJoints.length + 1];
         controlledJoints[0] = rootJoint;
         System.arraycopy(oneDoFJoints, 0, controlledJoints, 1, oneDoFJoints.length);
      }
      else
      {
         controlledJoints = oneDoFJoints;
      }
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(updateDT,
                                                                            0.0,
                                                                            rootJoint,
                                                                            controlledJoints,
                                                                            centerOfMassFrame,
                                                                            optimizationSettings,
                                                                            null,
                                                                            registry);
      toolbox.setJointPrivilegedConfigurationParameters(new JointPrivilegedConfigurationParameters());
      toolbox.setupForInverseKinematicsSolver();
      FeedbackControlCommandList controllerCoreTemplate = createControllerCoreTemplate(controllableRigidBodies);
      controllerCoreTemplate.addCommand(new CenterOfMassFeedbackControlCommand());
      JointDesiredOutputList lowLevelControllerOutput = new JointDesiredOutputList(oneDoFJoints);
      return new WholeBodyControllerCore(toolbox, controllerCoreTemplate, lowLevelControllerOutput, registry);
   }

   /**
    * Convenience method to create the template necessary for the controller core to create all the
    * necessary feedback controllers.
    * 
    * @param controllableRigidBodies the collection of all the rigid-bodies that will be controllable
    *                                by the user. If it is {@code null}, then all the rigid-bodies of
    *                                the robot will be controllable.
    * @return the template for the controller core.
    */
   private FeedbackControlCommandList createControllerCoreTemplate(Collection<? extends RigidBodyBasics> controllableRigidBodies)
   {
      FeedbackControlCommandList template = new FeedbackControlCommandList();
      Collection<? extends RigidBodyBasics> rigidBodies;

      if (controllableRigidBodies != null)
         rigidBodies = controllableRigidBodies;
      else
         rigidBodies = rootBody.subtreeList();

      rigidBodies.stream().map(this::createFeedbackControlCommand).forEach(template::addCommand);
      return template;
   }

   /**
    * Convenience method for pure laziness. Should only be used for
    * {@link #createControllerCoreTemplate()}.
    */
   private SpatialFeedbackControlCommand createFeedbackControlCommand(RigidBodyBasics endEffector)
   {
      SpatialFeedbackControlCommand command = new SpatialFeedbackControlCommand();
      command.set(rootBody, endEffector);
      return command;
   }

   /**
    * This marks the initialization phase. It is either called once when this toolbox wakes up or when
    * it is reinitialized.
    * <p>
    * It snaps {@code rootJoint} and {@code oneDoFJoints} to the most recent robot configuration
    * received from the walking controller. It also initializes the information needed to hold the
    * center of mass and support foot/feet in place.
    * </p>
    *
    * @return {@code true} if this controller is good to go and solve a new problem. It needs to have
    *         received at least once a robot configuration from the controller, otherwise this will
    *         fail and prevent the user from using this toolbox.
    */
   @Override
   public boolean initialize()
   {
      boolean success = initializeInternal();

      KinematicsToolboxOutputStatus status = new KinematicsToolboxOutputStatus();
      status.setJointNameHash(-1);
      status.setSolutionQuality(Double.NaN);
      status.setCurrentToolboxState(success ? CURRENT_TOOLBOX_STATE_INITIALIZE_SUCCESSFUL : CURRENT_TOOLBOX_STATE_INITIALIZE_FAILURE_MISSING_RCD);
      reportMessage(status);

      return success;
   }

   protected boolean initializeInternal()
   {
      userFeedbackCommands.clear();

      RobotConfigurationData robotConfigurationData = latestRobotConfigurationDataReference.get();

      if (robotConfigurationData == null)
      {
         commandInputManager.clearAllCommands();
         return false;
      }

      // Initializes this desired robot to the most recent robot configuration data received from the walking controller.
      KinematicsToolboxHelper.setRobotStateFromRobotConfigurationData(robotConfigurationData, rootJoint, oneDoFJoints);
      if (defaultPrivilegedConfigurationMap != null)
      {
         defaultPrivilegedConfigurationMap.forEachEntry((joint, q_priv) ->
         {
            joint.setQ(q_priv);
            return true;
         });
      }

      // Sets the privileged configuration to match the current robot configuration such that the solution will be as close as possible to the current robot configuration.
      snapPrivilegedConfigurationToCurrent();
      privilegedWeight.set(DEFAULT_PRIVILEGED_CONFIGURATION_WEIGHT);
      privilegedConfigurationGain.set(DEFAULT_PRIVILEGED_CONFIGURATION_GAIN);
      // It is required to update the tools now as it is only done at the end of each iteration.
      updateTools();

      return true;
   }

   private List<KinematicsCollisionResult> collisions = Collections.emptyList();

   /**
    * This is the control loop called periodically only when the user requested a solution for a
    * desired set of inputs.
    * <p>
    * Each time this method is called, the multi-body system gets closer to the user inputs.
    * </p>
    */
   @Override
   public void updateInternal()
   {
      threadTimer.start();
      // Updating the reference frames and twist calculator.

      // Compiling all the commands to be submitted to the controller core.
      controllerCoreCommand.clear();

      // Clear the map to forget the user commands previously submitted.
      if (!preserveUserCommandHistory.getValue())
         userFeedbackCommands.clear();

      FeedbackControlCommandList userCommands = consumeCommands();
      numberOfActiveCommands.set(userCommands.getNumberOfCommands());
      controllerCoreCommand.addFeedbackControlCommand(userCommands);
      controllerCoreCommand.addFeedbackControlCommand(getAdditionalFeedbackControlCommands());

      controllerCoreCommand.addInverseKinematicsCommand(activeOptimizationSettings);
      controllerCoreCommand.addInverseKinematicsCommand(privilegedConfigurationCommandReference.getAndSet(null));
      controllerCoreCommand.addInverseKinematicsCommand(getAdditionalInverseKinematicsCommands());
      controllerCoreCommand.addInverseKinematicsCommand(computeCollisionCommands(collisions));

      // Save all commands used for this control tick for computing the solution quality.
      FeedbackControlCommandList allFeedbackControlCommands = new FeedbackControlCommandList(controllerCoreCommand.getFeedbackControlCommandList());

      /*
       * Submitting and requesting the controller core to run the feedback controllers, formulate and
       * solve the optimization problem for this control tick.
       */
      controllerCore.reset();
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();

      // Calculating the solution quality based on sum of all the commands' tracking error.
      solutionQuality.set(KinematicsToolboxHelper.calculateSolutionQuality(allFeedbackControlCommands, feedbackControllerDataHolder));

      // Updating the the robot state from the current solution, initializing the next control tick.
      KinematicsToolboxHelper.setRobotStateFromControllerCoreOutput(controllerCore.getControllerCoreOutput(), rootJoint, oneDoFJoints);
      updateVisualization();

      inverseKinematicsSolution.setCurrentToolboxState(CURRENT_TOOLBOX_STATE_RUNNING);
      MessageTools.packDesiredJointState(inverseKinematicsSolution, rootJoint, oneDoFJoints);
      inverseKinematicsSolution.setSolutionQuality(solutionQuality.getDoubleValue());
      /*
       * Update tools for the next iteration. Only need to do it 1 per iteration and since it is updated
       * in the initialization method, it can be done at the end of the control tick. By doing this at the
       * end and computing collisions at the end, when visualizing the collisions they are exactly in sync
       * with the robot configuration.
       */
      updateTools();
      collisions = computeCollisions();

      timeSinceLastSolutionPublished.add(updateDT);

      if (timeSinceLastSolutionPublished.getValue() >= publishSolutionPeriod.getValue())
      {
         reportMessage(inverseKinematicsSolution);
         timeSinceLastSolutionPublished.set(0.0);
      }
      threadTimer.stop();
   }

   /**
    * Updates all the reference frames and the twist calculator. This method needs to be called at the
    * beginning of each control tick.
    */
   protected void updateTools()
   {
      rootBody.updateFramesRecursively();
      centerOfMassFrame.update();
   }

   /**
    * Checking if there is any new command available, in which case they polled from the
    * {@link #commandInputManager} and processed to update the state of the current optimization run.
    *
    * @return the new list of feedback control command to be executed for this control tick.
    */
   private FeedbackControlCommandList consumeCommands()
   {
      if (commandInputManager.isNewCommandAvailable(KinematicsToolboxConfigurationCommand.class))
      {
         KinematicsToolboxConfigurationCommand command = commandInputManager.pollNewestCommand(KinematicsToolboxConfigurationCommand.class);

         /*
          * If there is a new privileged configuration, the desired robot state is updated alongside with the
          * privileged configuration and the initial center of mass position and foot poses.
          */
         KinematicsToolboxHelper.setRobotStateFromPrivilegedConfigurationData(command, rootJoint, jointHashCodeMap);
         if (command.hasPrivilegedJointAngles() || command.hasPrivilegedRootJointPosition() || command.hasPrivilegedRootJointOrientation())
            robotConfigurationReinitialized();
         if (command.hasPrivilegedJointAngles())
            snapPrivilegedConfigurationToCurrent();
         if (command.getPrivilegedWeight() < 0.0)
            privilegedWeight.set(DEFAULT_PRIVILEGED_CONFIGURATION_WEIGHT);
         else
            privilegedWeight.set(command.getPrivilegedWeight());
         if (command.getPrivilegedGain() < 0.0)
            privilegedConfigurationGain.set(DEFAULT_PRIVILEGED_CONFIGURATION_GAIN);
         else
            privilegedConfigurationGain.set(command.getPrivilegedGain());

         if (command.getJointVelocityWeight() <= 0.0)
            activeOptimizationSettings.setJointVelocityWeight(optimizationSettings.getJointVelocityWeight());
         else
            activeOptimizationSettings.setJointVelocityWeight(command.getJointVelocityWeight());
         if (command.getJointAccelerationWeight() < 0.0)
            activeOptimizationSettings.setJointAccelerationWeight(optimizationSettings.getJointAccelerationWeight());
         else
            activeOptimizationSettings.setJointAccelerationWeight(command.getJointAccelerationWeight());

         if (command.getDisableCollisionAvoidance())
            enableCollisionAvoidance.set(false);
         if (command.getEnableCollisionAvoidance())
            enableCollisionAvoidance.set(true);
      }

      if (commandInputManager.isNewCommandAvailable(KinematicsToolboxCenterOfMassCommand.class))
      {
         KinematicsToolboxCenterOfMassCommand command = commandInputManager.pollNewestCommand(KinematicsToolboxCenterOfMassCommand.class);
         userFeedbackCommands.put(centerOfMassName, KinematicsToolboxHelper.consumeCenterOfMassCommand(command, gains.getPositionGains()));
      }

      if (commandInputManager.isNewCommandAvailable(KinematicsToolboxRigidBodyCommand.class))
      {
         List<KinematicsToolboxRigidBodyCommand> commands = commandInputManager.pollNewCommands(KinematicsToolboxRigidBodyCommand.class);
         for (int i = 0; i < commands.size(); i++)
         {
            SpatialFeedbackControlCommand rigidBodyCommand = KinematicsToolboxHelper.consumeRigidBodyCommand(commands.get(i), rootBody, gains);
            rigidBodyCommand.setPrimaryBase(getEndEffectorPrimaryBase(rigidBodyCommand.getEndEffector()));
            String endEffectorName = rigidBodyCommand.getEndEffector().getName();
            userFeedbackCommands.put(endEffectorName, rigidBodyCommand);
         }
      }

      FeedbackControlCommandList inputs = new FeedbackControlCommandList();
      /*
       * By using the map, we ensure that there is only one command per end-effector (including the center
       * of mass). The map is also useful for remembering commands received during the previous control
       * ticks of the same run.
       */
      userFeedbackCommands.values().forEach(inputs::addCommand);
      return inputs;
   }

   /**
    * Evaluates the collision between each possible pair of collidables that can collide.
    * 
    * @return the result of the evaluation.
    */
   private List<KinematicsCollisionResult> computeCollisions()
   {
      if (robotCollidables.isEmpty() || !enableCollisionAvoidance.getValue())
         return Collections.emptyList();

      int collisionIndex = 0;
      List<KinematicsCollisionResult> collisions = new ArrayList<>();

      for (int collidableAIndex = 0; collidableAIndex < robotCollidables.size(); collidableAIndex++)
      {
         KinematicsCollidable collidableA = robotCollidables.get(collidableAIndex);

         for (int collidableBIndex = collidableAIndex + 1; collidableBIndex < robotCollidables.size(); collidableBIndex++)
         {
            KinematicsCollidable collidableB = robotCollidables.get(collidableBIndex);

            if (!collidableA.isCollidableWith(collidableB))
               continue;

            KinematicsCollisionResult collisionResult = collidableA.evaluateCollision(collidableB);

            if (collisionResult.getSignedDistance() > collisionActivationDistanceThreshold.getValue())
               continue;

            if (collisionIndex < numberOfCollisionsToVisualize)
            {
               collisionDistances[collisionIndex].set(collisionResult.getSignedDistance());
               collisionPointAs[collisionIndex].setMatchingFrame(collisionResult.getPointOnA());
               collisionPointBs[collisionIndex].setMatchingFrame(collisionResult.getPointOnB());
            }

            collisions.add(collisionResult);
            collisionIndex++;
         }
      }

      return collisions;
   }

   /**
    * Calculates and sets up the list of constraints to submit to the solver to prevent collisions.
    * 
    * @param collisions the previously computed collisions.
    * @return the constraints to submit to the controller core.
    */
   public InverseKinematicsCommand<?> computeCollisionCommands(List<KinematicsCollisionResult> collisions)
   {
      if (collisions.isEmpty() || !enableCollisionAvoidance.getValue())
         return null;

      int collisionIndex = 0;
      InverseKinematicsCommandList commandList = new InverseKinematicsCommandList();

      for (int i = 0; i < collisions.size(); i++)
      {
         KinematicsCollisionResult collision = collisions.get(i);
         KinematicsCollidable collidableA = collision.getCollidableA();
         KinematicsCollidable collidableB = collision.getCollidableB();

         RigidBodyBasics bodyA = collidableA.getRigidBody();

         double sigma = -collision.getSignedDistance();
         double sigmaDot = sigma / updateDT;

         ReferenceFrame collisionFrame = KinematicsToolboxHelper.collisionFrame(collision, true, "collisionFrame" + collisionIndex);
         if (collisionIndex < numberOfCollisionsToVisualize)
            collisionFramePoses[collisionIndex].setFromReferenceFrame(collisionFrame);

         SpatialVelocityCommand command = new SpatialVelocityCommand();
         command.set(bodyA, collidableB.getRigidBody());
         command.getControlFramePose().setFromReferenceFrame(collisionFrame);
         SelectionMatrix6D selectionMatrix = command.getSelectionMatrix();
         selectionMatrix.clearSelection();
         selectionMatrix.selectLinearZ(true);
         selectionMatrix.setSelectionFrames(null, collisionFrame);

         command.setConstraintType(ConstraintType.GEQ_INEQUALITY);
         command.getDesiredLinearVelocity().setZ(sigmaDot);
         commandList.addCommand(command);

         collisionIndex++;
      }

      return commandList;
   }

   /**
    * Notifies when the user has sent a command that reinitializes the configuration of the robot.
    */
   protected void robotConfigurationReinitialized()
   {
      // Do nothing here
   }

   protected RigidBodyBasics getEndEffectorPrimaryBase(RigidBodyBasics endEffector)
   {
      return null;
   }

   /**
    * Updates the graphic coordinate systems for the end-effectors that are actively controlled during
    * this control tick.
    */
   private void updateVisualization()
   {
      boolean hasData;
      FramePoint3D position = new FramePoint3D();
      FrameQuaternion orientation = new FrameQuaternion();

      for (RigidBodyBasics endEffector : desiredCoodinateSystems.keySet())
      {
         YoGraphicCoordinateSystem coordinateSystem = desiredCoodinateSystems.get(endEffector);
         hasData = feedbackControllerDataHolder.getPositionData(endEffector, position, Type.DESIRED);
         if (!hasData)
            coordinateSystem.hide();
         else
            coordinateSystem.setPosition(position);

         hasData = feedbackControllerDataHolder.getOrientationData(endEffector, orientation, Type.DESIRED);
         if (!hasData)
            coordinateSystem.hide();
         else
            coordinateSystem.setOrientation(orientation);
      }

      for (RigidBodyBasics endEffector : currentCoodinateSystems.keySet())
      {
         YoGraphicCoordinateSystem coordinateSystem = currentCoodinateSystems.get(endEffector);
         hasData = feedbackControllerDataHolder.getPositionData(endEffector, position, Type.CURRENT);
         if (!hasData)
            coordinateSystem.hide();
         else
            coordinateSystem.setPosition(position);

         hasData = feedbackControllerDataHolder.getOrientationData(endEffector, orientation, Type.CURRENT);
         if (!hasData)
            coordinateSystem.hide();
         else
            coordinateSystem.setOrientation(orientation);
      }
   }

   /**
    * Creates a {@code PrivilegedConfigurationCommand} to update the privileged joint angles to match
    * the current state of {@link #desiredFullRobotModel}.
    */
   private void snapPrivilegedConfigurationToCurrent()
   {
      PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
      privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationOption.AT_CURRENT);
      privilegedConfigurationCommand.setDefaultWeight(privilegedWeight.getDoubleValue());
      privilegedConfigurationCommand.setDefaultConfigurationGain(privilegedConfigurationGain.getDoubleValue());
      privilegedConfigurationCommand.setDefaultMaxVelocity(privilegedMaxVelocity.getDoubleValue());
      privilegedConfigurationCommandReference.set(privilegedConfigurationCommand);
   }

   public void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      latestRobotConfigurationDataReference.set(newConfigurationData);
   }

   public boolean isUserControllingRigidBody(RigidBodyBasics rigidBody)
   {
      return isUserControllingRigidBody(rigidBody.getName());
   }

   public boolean isUserControllingRigidBody(String rigidBodyName)
   {
      return userFeedbackCommands.containsKey(rigidBodyName);
   }

   public boolean isUserControllingCenterOfMass()
   {
      return userFeedbackCommands.containsKey(centerOfMassName);
   }

   public YoPIDSE3Gains getDefaultGains()
   {
      return gains;
   }

   protected FeedbackControlCommandList getAdditionalFeedbackControlCommands()
   {
      return null;
   }

   protected InverseKinematicsCommandList getAdditionalInverseKinematicsCommands()
   {
      return null;
   }

   @Override
   public boolean isDone()
   {
      // This toolbox should run until if falls asleep.
      return false;
   }

   public FloatingJointBasics getDesiredRootJoint()
   {
      return rootJoint;
   }

   public OneDoFJointBasics[] getDesiredOneDoFJoint()
   {
      return oneDoFJoints;
   }

   public KinematicsToolboxOutputStatus getSolution()
   {
      return inverseKinematicsSolution;
   }

   public FeedbackControllerDataReadOnly getFeedbackControllerDataHolder()
   {
      return feedbackControllerDataHolder;
   }

   public void setPublishingSolutionPeriod(double periodInSeconds)
   {
      publishSolutionPeriod.set(periodInSeconds);
   }

   public void setPreserveUserCommandHistory(boolean value)
   {
      preserveUserCommandHistory.set(value);
   }

   public double getUpdateDT()
   {
      return updateDT;
   }
}