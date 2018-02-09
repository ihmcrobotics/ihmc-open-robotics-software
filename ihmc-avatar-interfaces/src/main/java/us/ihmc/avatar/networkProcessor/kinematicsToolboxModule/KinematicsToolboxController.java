package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.util.Arrays;
import java.util.Collection;
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
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.HumanoidKinematicsToolboxConfigurationMessage;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.SymmetricYoPIDSE3Gains;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

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

   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   /** Reference to the desired robot's root body. */
   protected final RigidBody rootBody;
   /** Reference to the desired robot's floating joint. */
   protected final FloatingInverseDynamicsJoint rootJoint;
   /**
    * Array containing all the one degree-of-freedom joints of the desired robot except for the
    * finger joints that are not handled by this solver.
    */
   private final OneDoFJoint[] oneDoFJoints;
   private final Map<Long, OneDoFJoint> jointNameBasedHashCodeMap = new HashMap<>();

   /**
    * Reference frame centered at the robot's center of mass. It is used to hold the initial center
    * of mass position when requested.
    */
   protected final ReferenceFrame centerOfMassFrame;

   /** The same set of gains is used for controlling any part of the desired robot body. */
   private final SymmetricYoPIDSE3Gains gains = new SymmetricYoPIDSE3Gains("genericGains", registry);
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
    * Weight indicating the priority for getting closer to the current privileged configuration. The
    * current privileged configuration can be changed at any time by sending a
    * {@link HumanoidKinematicsToolboxConfigurationMessage}.
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
    * The {@link #commandInputManager} is used as a 'thread-barrier'. When receiving a new user
    * input, this manager automatically copies the data in the corresponding command that can then
    * be used here safely.
    */
   protected final CommandInputManager commandInputManager;
   /**
    * Counter used alongside {@link #numberOfTicksToSendSolution} to reduce the frequency at which
    * the solution is sent back to the caller.
    */
   private int tickCount = 0;
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
                                      FloatingInverseDynamicsJoint rootJoint, OneDoFJoint[] oneDoFJoints, YoGraphicsListRegistry yoGraphicsListRegistry,
                                      YoVariableRegistry parentRegistry)
   {
      this(commandInputManager, statusOutputManager, rootJoint, oneDoFJoints, null, yoGraphicsListRegistry, parentRegistry);
   }

   public KinematicsToolboxController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                      FloatingInverseDynamicsJoint rootJoint, OneDoFJoint[] oneDoFJoints, Collection<RigidBody> controllableRigidBodies,
                                      YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);
      this.commandInputManager = commandInputManager;
      this.rootJoint = rootJoint;
      this.oneDoFJoints = oneDoFJoints;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      // This will find the root body without using rootJoint so it can be null.
      rootBody = ScrewTools.getRootBody(oneDoFJoints[0].getPredecessor());

      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", worldFrame, rootBody);

      Arrays.stream(oneDoFJoints).forEach(joint -> jointNameBasedHashCodeMap.put(joint.getNameBasedHashCode(), joint));

      controllerCore = createControllerCore(controllableRigidBodies);
      feedbackControllerDataHolder = controllerCore.getWholeBodyFeedbackControllerDataHolder();

      inverseKinematicsSolution = new KinematicsToolboxOutputStatus(oneDoFJoints);
      inverseKinematicsSolution.setDestination(-1);

      gains.setProportionalGains(1200.0); // Gains used for everything. It is as high as possible to reduce the convergence time.
      gains.setMaxFeedbackAndFeedbackRate(1500.0, Double.POSITIVE_INFINITY);

      privilegedWeight.set(1.0);
      privilegedConfigurationGain.set(50.0);
      privilegedMaxVelocity.set(Double.POSITIVE_INFINITY);
   }

   /**
    * This is where the end-effectors needing a visualization are registered, if you need more, add
    * it there.
    * 
    * @param rigidBodies all the rigid bodies for which the desired and actual pose will be
    *           displayed using graphical coordinate systems.
    */
   public void setupVisualization(RigidBody... rigidBodies)
   {
      AppearanceDefinition desiredAppearance = YoAppearance.Red();
      AppearanceDefinition currentAppearance = YoAppearance.Blue();

      for (RigidBody rigidBody : rigidBodies)
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
    * @param controllableRigidBodies
    *
    * @return the controller core that will run for the desired robot
    *         {@link #desiredFullRobotModel}.
    */
   private WholeBodyControllerCore createControllerCore(Collection<RigidBody> controllableRigidBodies)
   {
      KinematicsToolboxOptimizationSettings optimizationSettings = new KinematicsToolboxOptimizationSettings();
      InverseDynamicsJoint[] controlledJoints;
      if (rootJoint != null)
      {
         controlledJoints = new InverseDynamicsJoint[oneDoFJoints.length + 1];
         controlledJoints[0] = rootJoint;
         System.arraycopy(oneDoFJoints, 0, controlledJoints, 1, oneDoFJoints.length);
      }
      else
      {
         controlledJoints = oneDoFJoints;
      }
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(getClass().getSimpleName(), updateDT, 0.0, rootJoint, controlledJoints, centerOfMassFrame, optimizationSettings,
                                                                            null, registry);
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
    * @param controllableRigidBodies the collection of all the rigid-bodies that will be
    *           controllable by the user. If it is {@code null}, then all the rigid-bodies of the
    *           robot will be controllable.
    * @return the template for the controller core.
    */
   private FeedbackControlCommandList createControllerCoreTemplate(Collection<RigidBody> controllableRigidBodies)
   {
      FeedbackControlCommandList template = new FeedbackControlCommandList();
      Collection<RigidBody> rigidBodies;

      if (controllableRigidBodies != null)
         rigidBodies = controllableRigidBodies;
      else
         rigidBodies = Arrays.asList(ScrewTools.computeSupportAndSubtreeSuccessors(rootBody));

      rigidBodies.stream().map(this::createFeedbackControlCommand).forEach(template::addCommand);
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
      FeedbackControlCommandList userCommands = consumeCommands();
      numberOfActiveCommands.set(userCommands.getNumberOfCommands());
      controllerCoreCommand.addFeedbackControlCommand(userCommands);
      controllerCoreCommand.addFeedbackControlCommand(getAdditionalFeedbackControlCommands());

      controllerCoreCommand.addInverseKinematicsCommand(privilegedConfigurationCommandReference.getAndSet(null));
      controllerCoreCommand.addInverseKinematicsCommand(getAdditionalInverseKinematicsCommands());

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

      inverseKinematicsSolution.setDesiredJointState(rootJoint, oneDoFJoints, false);
      inverseKinematicsSolution.setSolutionQuality(solutionQuality.getDoubleValue());

      if (tickCount++ == numberOfTicksToSendSolution)
      { // Packing and sending the solution every N control ticks, with N = numberOfTicksToSendSolution.
         reportMessage(inverseKinematicsSolution);
         tickCount = 0;
      }
   }

   /**
    * Updates all the reference frames and the twist calculator. This method needs to be called at
    * the beginning of each control tick.
    */
   protected void updateTools()
   {
      rootBody.updateFramesRecursively();
      centerOfMassFrame.update();
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

         /*
          * If there is a new privileged configuration, the desired robot state is updated alongside
          * with the privileged configuration and the initial center of mass position and foot
          * poses.
          */
         KinematicsToolboxHelper.setRobotStateFromPrivilegedConfigurationData(command, rootJoint, jointNameBasedHashCodeMap);
         if (command.hasPrivilegedJointAngles() || command.hasPrivilegedRootJointPosition() || command.hasPrivilegedRootJointOrientation())
            robotConfigurationReinitialized();
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
    * Notifies when the user has sent a command that reinitializes the configuration of the robot.
    */
   protected void robotConfigurationReinitialized()
   {
      // Do nothing here
   }

   /**
    * Updates the graphic coordinate systems for the end-effectors that are actively controlled
    * during this control tick.
    */
   private void updateVisualization()
   {
      boolean hasData;
      FramePoint3D position = new FramePoint3D();
      FrameQuaternion orientation = new FrameQuaternion();

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

   public void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      latestRobotConfigurationDataReference.set(newConfigurationData);
   }

   public boolean isUserControllingRigidBody(RigidBody rigidBody)
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

   protected PIDSE3Gains getDefaultGains()
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
   protected boolean isDone()
   {
      // This toolbox should run until if falls asleep.
      return false;
   }

   public FloatingInverseDynamicsJoint getDesiredRootJoint()
   {
      return rootJoint;
   }

   public OneDoFJoint[] getDesiredOneDoFJoint()
   {
      return oneDoFJoints;
   }

   public KinematicsToolboxOutputStatus getSolution()
   {
      return inverseKinematicsSolution;
   }
}