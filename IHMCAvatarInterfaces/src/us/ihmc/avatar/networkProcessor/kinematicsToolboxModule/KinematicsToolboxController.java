package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.KinematicsToolboxConfigurationMessage;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.controllers.SE3PIDGainsInterface;
import us.ihmc.robotics.controllers.YoSymmetricSE3PIDGains;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoseUsingQuaternions;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

/**
 * {@code KinematicsToolboxController} is used as a whole-body inverse kinematics solver.
 * <p>
 * The interaction with this solver is achieved over the network using message that define the API,
 * see {@link KinematicsToolboxModule}.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public class KinematicsToolboxController extends ToolboxController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   /**
    * Indicates the duration of a control tick. It should match the thread period in
    * {@link ToolboxModule}.
    */
   private static final double updateDT = 1.0e-3;
   /**
    * Specifies how many control ticks have to be performed before sending a solution to the caller.
    */
   private static final int numberOfTicksToSendSolution = 10;
   /**
    * This name is used with the {@code Map} {@link #userFeedbackCommands} to keep track of the
    * active set of commands provided by the caller.
    */
   private static final String centerOfMassName = "CenterOfMass";

   /**
    * This is the model of the robot that is constantly updated to represent the most recent
    * solution obtained. The {@link WholeBodyControllerCore} works on this robot to perform the
    * feedback controllers.
    */
   private final FullHumanoidRobotModel desiredFullRobotModel;
   /** Reference to the desired robot's root body. */
   private final RigidBody rootBody;
   /** Reference to the desired robot's floating joint. */
   private final FloatingInverseDynamicsJoint rootJoint;
   /**
    * Array containing all the one degree-of-freedom joints of the desired robot except for the
    * finger joints that are not handled by this solver.
    */
   private final OneDoFJoint[] oneDoFJoints;
   private final Map<Long, OneDoFJoint> jointNameBasedHashCodeMap = new HashMap<>();
   /**
    * Mostly used here for obtaining the center of mass frame. This holds on various useful frames
    * such as the center of frame.
    */
   private final CommonHumanoidReferenceFrames referenceFrames;

   /** The same set of gains is used for controlling any part of the desired robot body. */
   private final YoSymmetricSE3PIDGains gains = new YoSymmetricSE3PIDGains("genericGains", registry);
   /**
    * The controller core command is the single object used to pass the desired inputs to the
    * controller core.
    */
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_KINEMATICS);
   /**
    * This is where the magic is happening. The controller is responsible for performing feedback
    * control to reduce the difference between the desireds sent to the
    * {@code KinematicsToolboxController} and the actual robot pose. It is also responsible for
    * gathering the entire set of desired inputs and formulate the adequate optimization problem to
    * be solved for every control tick. The output of the controller core provides every tick a new
    * robot joint configurations and velocities that are one step closer to the desireds. The output
    * is used to update the state of the {@link #desiredFullRobotModel} such that it progresses
    * towards the desired user inputs over time.
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
    * which can be used to quickly see if the solution is viable. It is sent back to the caller
    * only.
    */
   private final KinematicsToolboxOutputStatus inverseKinematicsSolution;
   /**
    * This is the current estimate of the solution quality that is calculated based on the tracking
    * error for the end-effectors (center of mass included) being actively controlled.
    */
   private final YoDouble solutionQuality = new YoDouble("solutionQuality", registry);

   /**
    * Updated during the initialization phase, this set of two {@link YoBoolean}s is used to
    * know which foot is currently used for support in the walking controller.
    */
   private final SideDependentList<YoBoolean> isFootInSupport = new SideDependentList<>();
   /**
    * Updated during the initialization phase, this is where the poses of the feet are stored so
    * they can be held in place during the optimization process such that the solution will be
    * statically reachable.
    */
   private final SideDependentList<YoFramePoseUsingQuaternions> initialFootPoses = new SideDependentList<>();
   /**
    * Updated during the initialization phase, this is where the robot's center of mass position is
    * stored so it can be held in place during the optimization process such that the solution will
    * be statically reachable.
    */
   private final YoFramePoint initialCenterOfMassPosition = new YoFramePoint("initialCenterOfMass", worldFrame, registry);

   /**
    * Indicates whether the support foot/feet should be held in place for this run. It is
    * {@code true} by default but can be disabled using the message
    * {@link KinematicsToolboxConfigurationMessage}.
    */
   private final YoBoolean holdSupportFootPose = new YoBoolean("holdSupportFootPose", registry);
   /**
    * Indicates whether the center of mass x and y coordinates should be held in place for this run.
    * It is {@code true} by default but can be disabled using the message
    * {@link KinematicsToolboxConfigurationMessage}.
    */
   private final YoBoolean holdCenterOfMassXYPosition = new YoBoolean("holdCenterOfMassXYPosition", registry);

   /**
    * Weight indicating the priority for getting closer to the current privileged configuration. The
    * current privileged configuration can be changed at any time by sending a
    * {@link KinematicsToolboxConfigurationMessage}.
    */
   private final YoDouble privilegedWeight = new YoDouble("privilegedWeight", registry);
   /**
    * To make the robot get closer to the privileged configuration, a feedback control is used to
    * compute for each joint a privileged velocity based on the difference between the privileged
    * angle and the current angle. These privileged joint velocities are then used to complete the
    * optimization problem in such way that they don't interfere with the user commands.
    */
   private final YoDouble privilegedConfigurationGain = new YoDouble("privilegedConfigurationGain", registry);
   /**
    * Cap used to limit the magnitude of the privileged joint velocities computed in the controller
    * core. Should probably remain equal to {@link Double#POSITIVE_INFINITY} so the solution
    * converges quicker.
    */
   private final YoDouble privilegedMaxVelocity = new YoDouble("privilegedMaxVelocity", registry);
   /**
    * This reference to {@link PrivilegedConfigurationCommand} is used internally only to figure out
    * if the current privileged configuration used in the controller core is to be updated or not.
    * It is usually updated once right after the initialization phase.
    */
   private final AtomicReference<PrivilegedConfigurationCommand> privilegedConfigurationCommandReference = new AtomicReference<>(null);

   /**
    * Default weight used when holding the support foot/feet in place. It is rather high such that
    * they do not deviate much from their initial poses.
    */
   private final YoDouble footWeight = new YoDouble("footWeight", registry);
   /**
    * Default weight used when holding the center of mass in place. It is rather high such that it
    * does not deviate much from its initial position.
    */
   private final YoDouble momentumWeight = new YoDouble("momentumWeight", registry);

   /**
    * The {@link #commandInputManager} is used as a 'thread-barrier'. When receiving a new user
    * input, this manager automatically copies the data in the corresponding command that can then
    * be used here safely.
    */
   private final CommandInputManager commandInputManager;
   /**
    * Counter used alongside {@link #numberOfTicksToSendSolution} to reduce the frequency at which
    * the solution is sent back to the caller.
    */
   private int tickCount = 0;

   /**
    * This joint reduction factors are used to limit the range of motion for each joint in the
    * controller core. The formulated based on the percentage of the actual range of motion for each
    * joint such that a factor of 0.05 for the hip yaw will effectively reduce the allowed range of
    * motion by 2.5% on the upper and lower end of the joint.
    */
   private final EnumMap<LegJointName, YoDouble> legJointLimitReductionFactors = new EnumMap<>(LegJointName.class);

   /**
    * This is the list of all the rigid-bodies that can ever be controlled and it is initialized in
    * {@link #populateListOfControllableRigidBodies()}. If there is need for controlling a
    * rigid-body that is not in this list, it has to be added in the code, it cannot be changed on
    * the fly.
    */
   private final List<RigidBody> listOfControllableRigidBodies = new ArrayList<>();

   /**
    * Visualization of the desired end-effector poses seen as coordinate systems in the
    * {@code SCSVisualizer}. They are only visible when the end-effector is being actively
    * controlled.
    */
   private final Map<RigidBody, YoGraphicCoordinateSystem> desiredCoodinateSystems = new HashMap<>();
   /**
    * Visualization of the current end-effector poses seen as coordinate systems in the
    * {@code SCSVisualizer}. They are only visible when the end-effector is being actively
    * controlled.
    */
   private final Map<RigidBody, YoGraphicCoordinateSystem> currentCoodinateSystems = new HashMap<>();

   /**
    * Reference to the most recent robot configuration received from the controller. It is used for
    * initializing the {@link #desiredFullRobotModel} before starting the optimization process.
    */
   private final AtomicReference<RobotConfigurationData> latestRobotConfigurationDataReference = new AtomicReference<>(null);
   /**
    * Reference to the most recent data received from the controller relative to the balance
    * control. It is used for identifying which foot is in support and thus which foot should be
    * held in place.
    */
   private final AtomicReference<CapturabilityBasedStatus> latestCapturabilityBasedStatusReference = new AtomicReference<>(null);

   /**
    * Map of the commands requested during two initialization phases by the user. It used to
    * memorize active commands such that the caller does not have to send the same commands every
    * control tick of this solver.
    */
   private final Map<String, FeedbackControlCommand<?>> userFeedbackCommands = new HashMap<>();

   /**
    * This is mostly for visualization to be able to keep track of the number of commands that the
    * user submitted.
    */
   private final YoInteger numberOfActiveCommands = new YoInteger("numberOfActiveCommands", registry);

   public KinematicsToolboxController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                      FullHumanoidRobotModel desiredFullRobotModel, YoGraphicsListRegistry yoGraphicsListRegistry,
                                      YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);
      this.commandInputManager = commandInputManager;

      this.desiredFullRobotModel = desiredFullRobotModel;

      referenceFrames = new HumanoidReferenceFrames(desiredFullRobotModel);
      rootBody = desiredFullRobotModel.getElevator();
      rootJoint = desiredFullRobotModel.getRootJoint();

      populateJointLimitReductionFactors();
      populateListOfControllableRigidBodies();

      controllerCore = createControllerCore();
      feedbackControllerDataHolder = controllerCore.getWholeBodyFeedbackControllerDataHolder();

      oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(desiredFullRobotModel);
      Arrays.stream(oneDoFJoints).forEach(joint -> jointNameBasedHashCodeMap.put(joint.getNameBasedHashCode(), joint));

      inverseKinematicsSolution = new KinematicsToolboxOutputStatus(oneDoFJoints);
      inverseKinematicsSolution.setDestination(-1);

      gains.setProportionalGain(800.0); // Gains used for everything. It is as high as possible to reduce the convergence time.

      footWeight.set(200.0);
      momentumWeight.set(1.0);
      privilegedWeight.set(1.0);
      privilegedConfigurationGain.set(50.0);
      privilegedMaxVelocity.set(Double.POSITIVE_INFINITY);

      for (RobotSide robotSide : RobotSide.values)
      {
         String side = robotSide.getCamelCaseNameForMiddleOfExpression();
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         isFootInSupport.put(robotSide, new YoBoolean("is" + side + "FootInSupport", registry));
         initialFootPoses.put(robotSide, new YoFramePoseUsingQuaternions(sidePrefix + "FootInitial", worldFrame, registry));
      }

      setupVisualization(yoGraphicsListRegistry);
   }

   /**
    * This is where the end-effectors needing a visualization are registered, if you need more, add
    * it there.
    * 
    * @param yoGraphicsListRegistry the main registry of this module, it is used to register the
    *           different graphics that are to be displayed in {@code SCSVisualizer}.
    */
   public void setupVisualization(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = desiredFullRobotModel.getFoot(robotSide);
         RigidBody hand = desiredFullRobotModel.getHand(robotSide);
         AppearanceDefinition desiredAppearance = YoAppearance.Red();
         AppearanceDefinition currentAppearance = YoAppearance.Blue();

         desiredCoodinateSystems.put(foot, createCoodinateSystem(foot, Type.DESIRED, desiredAppearance));
         desiredCoodinateSystems.put(hand, createCoodinateSystem(hand, Type.DESIRED, desiredAppearance));
         currentCoodinateSystems.put(foot, createCoodinateSystem(foot, Type.CURRENT, currentAppearance));
         currentCoodinateSystems.put(hand, createCoodinateSystem(hand, Type.CURRENT, currentAppearance));
      }

      desiredCoodinateSystems.forEach((k, v) -> yoGraphicsListRegistry.registerYoGraphic("CoordinateSystems", v));
      currentCoodinateSystems.forEach((k, v) -> yoGraphicsListRegistry.registerYoGraphic("CoordinateSystems", v));
   }

   /**
    * Convenience method that should only be used for setting up the visualization.
    * <p>
    * Simply creates a graphic coordinate system visualizable in {@code SCSVisualizer}.
    * </p>
    * 
    * @param endEffector used to create a name prefix required for creating a
    *           {@link YoGraphicCoordinateSystem}.
    * @param type used to create a name prefix required for creating a
    *           {@link YoGraphicCoordinateSystem}.
    * @param appearanceDefinition the appearance of the coordinate system's arrows.
    * @return the graphic with a good name for the given end-effector.
    */
   private YoGraphicCoordinateSystem createCoodinateSystem(RigidBody endEffector, Type type, AppearanceDefinition appearanceDefinition)
   {
      String namePrefix = endEffector.getName() + type.getName();
      return new YoGraphicCoordinateSystem(namePrefix, "", registry, 0.2, appearanceDefinition);
   }

   /**
    * Creating the controller core which is the main piece of this solver.
    * 
    * @return the controller core that will run for the desired robot
    *         {@link #desiredFullRobotModel}.
    */
   public WholeBodyControllerCore createControllerCore()
   {
      InverseDynamicsJoint[] controlledJoints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(desiredFullRobotModel);
      ReferenceFrame centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      KinematicsToolboxOptimizationSettings optimizationSettings = new KinematicsToolboxOptimizationSettings();
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(updateDT, 0.0, rootJoint, controlledJoints, centerOfMassFrame, optimizationSettings,
                                                                            null, registry);
      toolbox.setJointPrivilegedConfigurationParameters(new JointPrivilegedConfigurationParameters());
      toolbox.setupForInverseKinematicsSolver();
      FeedbackControlCommandList controllerCoreTemplate = createControllerCoreTemplate();
      controllerCoreTemplate.addCommand(new CenterOfMassFeedbackControlCommand());
      return new WholeBodyControllerCore(toolbox, controllerCoreTemplate, registry);
   }

   /**
    * Setting up the map holding the joint limit reduction factors. If more reduction is needed, add
    * it there. If it has to be updated on the fly, it should then be added this toolbox API,
    * probably added to the message {@link KinematicsToolboxConfigurationMessage}.
    */
   public void populateJointLimitReductionFactors()
   {
      YoDouble hipReductionFactor = new YoDouble("hipLimitReductionFactor", registry);
      YoDouble kneeReductionFactor = new YoDouble("kneeLimitReductionFactor", registry);
      YoDouble ankleReductionFactor = new YoDouble("ankleLimitReductionFactor", registry);
      hipReductionFactor.set(0.05);

      legJointLimitReductionFactors.put(LegJointName.HIP_PITCH, hipReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.HIP_ROLL, hipReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.HIP_YAW, hipReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.KNEE_PITCH, kneeReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.ANKLE_PITCH, ankleReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.ANKLE_ROLL, ankleReductionFactor);
   }

   /**
    * Registering all the rigid-bodies that will ever be available for control. This is not
    * changeable on the fly as the controller core use this list at construction time to create the
    * required feedback controllers.
    */
   private void populateListOfControllableRigidBodies()
   {
      listOfControllableRigidBodies.add(desiredFullRobotModel.getChest());
      listOfControllableRigidBodies.add(desiredFullRobotModel.getPelvis());

      for (RobotSide robotSide : RobotSide.values)
      {
         listOfControllableRigidBodies.add(desiredFullRobotModel.getHand(robotSide));
         listOfControllableRigidBodies.add(desiredFullRobotModel.getFoot(robotSide));
      }
   }

   /**
    * Convenience method to create the template necessary for the controller core to create all the
    * necessary feedback controllers.
    * 
    * @return the template for the controller core.
    */
   private FeedbackControlCommandList createControllerCoreTemplate()
   {
      FeedbackControlCommandList template = new FeedbackControlCommandList();
      listOfControllableRigidBodies.stream().map(this::createFeedbackControlCommand).forEach(template::addCommand);
      return template;
   }

   /**
    * Convenience method for pure laziness. Should only be used for
    * {@link #createControllerCoreTemplate()}.
    */
   private SpatialFeedbackControlCommand createFeedbackControlCommand(RigidBody endEffector)
   {
      SpatialFeedbackControlCommand command = new SpatialFeedbackControlCommand();
      command.set(rootBody, endEffector);
      return command;
   }

   /**
    * This marks the initialization phase. It is either called once when this toolbox wakes up or
    * when it is reinitialized.
    * <p>
    * It snaps this {@link #desiredFullRobotModel} to the most recent robot configuration received
    * from the walking controller. It also initializes the information needed to hold the center of
    * mass and support foot/feet in place.
    * </p>
    * 
    * @return {@code true} if this controller is good to go and solve a new problem. It needs to
    *         have received at least once a robot configuration from the controller, otherwise this
    *         will fail and prevent the user from using this toolbox.
    */
   @Override
   protected boolean initialize()
   {
      userFeedbackCommands.clear();

      RobotConfigurationData robotConfigurationData = latestRobotConfigurationDataReference.get();

      if (robotConfigurationData == null)
         return false;

      // Initializes this desired robot to the most recent robot configuration data received from the walking controller.
      KinematicsToolboxHelper.setRobotStateFromRobotConfigurationData(robotConfigurationData, rootJoint, oneDoFJoints);

      // Using the most recent CapturabilityBasedStatus received from the walking controller to figure out which foot is in support.
      CapturabilityBasedStatus capturabilityBasedStatus = latestCapturabilityBasedStatusReference.get();

      if (capturabilityBasedStatus == null)
      {
         for (RobotSide robotSide : RobotSide.values)
            isFootInSupport.get(robotSide).set(true);
      }
      else
      {
         for (RobotSide robotside : RobotSide.values)
            isFootInSupport.get(robotside).set(capturabilityBasedStatus.isSupportFoot(robotside));
      }

      // Initialize the initialCenterOfMassPosition and initialFootPoses to match the current state of the robot.
      updateCoMPositionAndFootPoses();

      // By default, always hold the support foot/feet and center of mass in place. This can be changed on the fly by sending a KinematicsToolboxConfigurationMessage.
      holdSupportFootPose.set(true);
      holdCenterOfMassXYPosition.set(true);

      // Sets the privileged configuration to match the current robot configuration such that the solution will be as close as possible to the current robot configuration.
      snapPrivilegedConfigurationToCurrent();

      return true;
   }

   /**
    * This is the control loop called periodically only when the user requested a solution for a
    * desired set of inputs.
    * <p>
    * Each time this method is called, the {@link #desiredFullRobotModel} gets closer to the user
    * inputs.
    * </p>
    */
   @Override
   protected void updateInternal()
   {
      // Updating the reference frames and twist calculator.
      updateTools();

      // Compiling all the commands to be submitted to the controller core.
      controllerCoreCommand.clear();
      controllerCoreCommand.addFeedbackControlCommand(createHoldCenterOfMassXYCommand());
      controllerCoreCommand.addFeedbackControlCommand(createHoldSupportFootCommands());
      FeedbackControlCommandList userCommands = consumeCommands();
      numberOfActiveCommands.set(userCommands.getNumberOfCommands());
      controllerCoreCommand.addFeedbackControlCommand(userCommands);

      controllerCoreCommand.addInverseKinematicsCommand(createJointLimitReductionCommand());
      controllerCoreCommand.addInverseKinematicsCommand(privilegedConfigurationCommandReference.getAndSet(null));

      // Save all commands used for this control tick for computing the solution quality.
      FeedbackControlCommandList allFeedbackControlCommands = new FeedbackControlCommandList(controllerCoreCommand.getFeedbackControlCommandList());

      /*
       * Submitting and requesting the controller core to run the feedback controllers, formulate
       * and solve the optimization problem for this control tick.
       */
      controllerCore.reset();
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();

      // Calculating the solution quality based on sum of all the commands' tracking error.
      solutionQuality.set(KinematicsToolboxHelper.calculateSolutionQuality(allFeedbackControlCommands, feedbackControllerDataHolder));

      // Updating the the robot state from the current solution, initializing the next control tick.
      KinematicsToolboxHelper.setRobotStateFromControllerCoreOutput(controllerCore.getControllerCoreOutput(), rootJoint, oneDoFJoints);
      updateVisualization();

      if (tickCount++ == numberOfTicksToSendSolution)
      { // Packing and sending the solution every N control ticks, with N = numberOfTicksToSendSolution.
         inverseKinematicsSolution.setDesiredJointState(rootJoint, oneDoFJoints);
         inverseKinematicsSolution.setSolutionQuality(solutionQuality.getDoubleValue());
         reportMessage(inverseKinematicsSolution);
         tickCount = 0;
      }
   }

   /**
    * Updates all the reference frames and the twist calculator. This method needs to be called at
    * the beginning of each control tick.
    */
   public void updateTools()
   {
      desiredFullRobotModel.updateFrames();
      referenceFrames.updateFrames();
   }

   /**
    * Checking if there is any new command available, in which case they polled from the
    * {@link #commandInputManager} and processed to update the state of the current optimization
    * run.
    * 
    * @return the new list of feedback control command to be executed for this control tick.
    */
   private FeedbackControlCommandList consumeCommands()
   {
      if (commandInputManager.isNewCommandAvailable(KinematicsToolboxConfigurationCommand.class))
      {
         KinematicsToolboxConfigurationCommand command = commandInputManager.pollNewestCommand(KinematicsToolboxConfigurationCommand.class);

         holdCenterOfMassXYPosition.set(command.holdCurrentCenterOfMassXYPosition());
         holdSupportFootPose.set(command.holdSupportFootPositions());

         /*
          * If there is a new privileged configuration, the desired robot state is updated alongside
          * with the privileged configuration and the initial center of mass position and foot
          * poses.
          */
         KinematicsToolboxHelper.setRobotStateFromPrivilegedConfigurationData(command, rootJoint, jointNameBasedHashCodeMap);
         if (command.hasPrivilegedJointAngles() || command.hasPrivilegedRootJointPosition() || command.hasPrivilegedRootJointOrientation())
            updateCoMPositionAndFootPoses();
         if (command.hasPrivilegedJointAngles())
            snapPrivilegedConfigurationToCurrent();
      }

      if (commandInputManager.isNewCommandAvailable(KinematicsToolboxCenterOfMassCommand.class))
      {
         KinematicsToolboxCenterOfMassCommand command = commandInputManager.pollNewestCommand(KinematicsToolboxCenterOfMassCommand.class);
         userFeedbackCommands.put(centerOfMassName, KinematicsToolboxHelper.consumeCenterOfMassCommand(command, gains));
      }

      if (commandInputManager.isNewCommandAvailable(KinematicsToolboxRigidBodyCommand.class))
      {
         List<KinematicsToolboxRigidBodyCommand> commands = commandInputManager.pollNewCommands(KinematicsToolboxRigidBodyCommand.class);
         for (int i = 0; i < commands.size(); i++)
         {
            SpatialFeedbackControlCommand rigidBodyCommand = KinematicsToolboxHelper.consumeRigidBodyCommand(commands.get(i), rootBody, gains);
            String endEffectorName = rigidBodyCommand.getEndEffector().getName();
            userFeedbackCommands.put(endEffectorName, rigidBodyCommand);
         }
      }

      FeedbackControlCommandList inputs = new FeedbackControlCommandList();
      /*
       * By using the map, we ensure that there is only one command per end-effector (including the
       * center of mass). The map is also useful for remembering commands received during the
       * previous control ticks of the same run.
       */
      userFeedbackCommands.values().forEach(inputs::addCommand);
      return inputs;
   }

   /**
    * Creates and sets up the feedback control commands for holding the support foot/feet in place.
    * If {@link #holdSupportFootPose} is {@code false}, this methods returns {@code null}.
    * <p>
    * Also note that if a user command has been received for a support foot, the command for this
    * foot is not created.
    * </p>
    * 
    * @return the commands for holding the support foot/feet in place.
    */
   private FeedbackControlCommand<?> createHoldSupportFootCommands()
   {
      if (!holdSupportFootPose.getBooleanValue())
         return null;

      FeedbackControlCommandList inputs = new FeedbackControlCommandList();

      for (RobotSide robotSide : RobotSide.values)
      {
         if (!isFootInSupport.get(robotSide).getBooleanValue())
            continue;

         RigidBody foot = desiredFullRobotModel.getFoot(robotSide);

         // Do not hold the foot position if the user is already controlling it.
         if (userFeedbackCommands.containsKey(foot.getName()))
            continue;

         FramePose poseToHold = new FramePose();
         initialFootPoses.get(robotSide).getFramePoseIncludingFrame(poseToHold);

         SpatialFeedbackControlCommand feedbackControlCommand = new SpatialFeedbackControlCommand();
         feedbackControlCommand.set(rootBody, foot);
         feedbackControlCommand.setGains((SE3PIDGainsInterface) gains);
         feedbackControlCommand.setWeightForSolver(footWeight.getDoubleValue());
         feedbackControlCommand.set(poseToHold);
         inputs.addCommand(feedbackControlCommand);
      }
      return inputs;
   }

   /**
    * Creates and sets up the feedback control command for holding the center of mass x and y
    * coordinates in place. If {@link #holdCenterOfMassXYPosition} is {@code false}, this methods
    * returns {@code null}.
    * <p>
    * Also note that if a user command has been received for the center of mass, this methods
    * returns {@code null}.
    * </p>
    * 
    * @return the commands for holding the center of mass x and y coordinates in place.
    */
   private FeedbackControlCommand<?> createHoldCenterOfMassXYCommand()
   {
      if (!holdCenterOfMassXYPosition.getBooleanValue())
         return null;

      // Do not hold the CoM position if the user is already controlling it.
      if (userFeedbackCommands.containsKey(centerOfMassName))
      {
         holdCenterOfMassXYPosition.set(false);
         return null;
      }

      FramePoint positionToHold = new FramePoint();
      initialCenterOfMassPosition.getFrameTupleIncludingFrame(positionToHold);

      CenterOfMassFeedbackControlCommand feedbackControlCommand = new CenterOfMassFeedbackControlCommand();
      feedbackControlCommand.setGains(gains);
      feedbackControlCommand.setWeightForSolver(momentumWeight.getDoubleValue());
      feedbackControlCommand.setSelectionMatrixForLinearXYControl();
      feedbackControlCommand.set(positionToHold);
      return feedbackControlCommand;
   }

   /**
    * Creates and sets up the {@code JointLimitReductionCommand} from the map
    * {@link #legJointLimitReductionFactors}.
    * 
    * @return the command for reducing the allowed range of motion of the leg joints.
    */
   private JointLimitReductionCommand createJointLimitReductionCommand()
   {
      JointLimitReductionCommand jointLimitReductionCommand = new JointLimitReductionCommand();
      for (RobotSide robotSide : RobotSide.values)
      {
         for (LegJointName legJointName : desiredFullRobotModel.getRobotSpecificJointNames().getLegJointNames())
         {
            OneDoFJoint joint = desiredFullRobotModel.getLegJoint(robotSide, legJointName);
            double reductionFactor = legJointLimitReductionFactors.get(legJointName).getDoubleValue();
            jointLimitReductionCommand.addReductionFactor(joint, reductionFactor);
         }
      }
      return jointLimitReductionCommand;
   }

   /**
    * Updates the graphic coordinate systems for the end-effectors that are actively controlled
    * during this control tick.
    */
   private void updateVisualization()
   {
      boolean hasData;
      FramePoint position = new FramePoint();
      FrameOrientation orientation = new FrameOrientation();

      for (RigidBody endEffector : desiredCoodinateSystems.keySet())
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

      for (RigidBody endEffector : currentCoodinateSystems.keySet())
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
    * Sets the {@link #initialCenterOfMassPosition} and {@link #initialFootPoses} to match the
    * current state of {@link #desiredFullRobotModel}.
    */
   private void updateCoMPositionAndFootPoses()
   {
      updateTools();

      initialCenterOfMassPosition.setFromReferenceFrame(referenceFrames.getCenterOfMassFrame());

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = desiredFullRobotModel.getFoot(robotSide);
         initialFootPoses.get(robotSide).setFromReferenceFrame(foot.getBodyFixedFrame());
      }
   }

   /**
    * Creates a {@code PrivilegedConfigurationCommand} to update the privileged joint angles to
    * match the current state of {@link #desiredFullRobotModel}.
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

   void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      latestRobotConfigurationDataReference.set(newConfigurationData);
   }

   void updateCapturabilityBasedStatus(CapturabilityBasedStatus newStatus)
   {
      latestCapturabilityBasedStatusReference.set(newStatus);
   }

   public FullHumanoidRobotModel getDesiredFullRobotModel()
   {
      return desiredFullRobotModel;
   }

   @Override
   protected boolean isDone()
   {
      // This toolbox should run until if falls asleep.
      return false;
   }

   public KinematicsToolboxOutputStatus getSolution()
   {
      return inverseKinematicsSolution;
   }
}