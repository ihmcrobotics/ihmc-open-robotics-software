package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController.RobotConfigurationDataBasedUpdater;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Provides whole body inverse kinematics solutions in the local process without ROS 2 communication.
 */
public class HumanoidKinematicsSolver
{
   private static final int DEFAULT_MAX_NUMBER_OF_ITERATIONS = 200;
   private static final double DEFAULT_QUALITY_THRESHOLD = 0.005;
   private static final double DEFAULT_STABILITY_THRESHOLD = 0.00002;
   private static final double DEFAULT_MIN_PROGRESSION = 0.005;

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final HumanoidKinematicsToolboxController controller;
   private final CommandInputManager commandInputManager = new CommandInputManager(name, KinematicsToolboxModule.supportedCommands());

   private final YoDouble solutionQualityThreshold = new YoDouble("solutionQualityThreshold", registry);
   private final YoDouble solutionStabilityThreshold = new YoDouble("solutionStabilityThreshold", registry);
   private final YoDouble solutionMinimumProgression = new YoDouble("solutionProgressionThreshold", registry);

   private final YoInteger numberOfIterations = new YoInteger("numberOfIterations", registry);
   private final YoInteger maximumNumberOfIterations = new YoInteger("maximumNumberOfIterations", registry);

   private final YoBoolean hasConverged = new YoBoolean("hasConverged", registry);
   private final YoDouble computationTime = new YoDouble("computationTime", registry);
   private final YoDouble solutionQuality = new YoDouble("solutionQuality", registry);

   private final RobotConfigurationDataBasedUpdater desiredRobotStateUpdater = new RobotConfigurationDataBasedUpdater();

   public HumanoidKinematicsSolver(FullHumanoidRobotModelFactory fullRobotModelFactory,
                                   YoGraphicsListRegistry yoGraphicsListRegistry,
                                   YoRegistry parentRegistry)
   {
      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(KinematicsToolboxModule.supportedStatus());
      FullHumanoidRobotModel desiredFullRobotModel = fullRobotModelFactory.createFullRobotModel();

      double updateDT = 1.0e-3;
      controller = new HumanoidKinematicsToolboxController(commandInputManager,
                                                           statusOutputManager,
                                                           desiredFullRobotModel,
                                                           updateDT,
                                                           yoGraphicsListRegistry,
                                                           registry);
      controller.setDesiredRobotStateUpdater(desiredRobotStateUpdater);

      commandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(desiredFullRobotModel, controller.getDesiredReferenceFrames()));

      maximumNumberOfIterations.set(DEFAULT_MAX_NUMBER_OF_ITERATIONS);
      solutionQualityThreshold.set(DEFAULT_QUALITY_THRESHOLD);
      solutionStabilityThreshold.set(DEFAULT_STABILITY_THRESHOLD);
      solutionMinimumProgression.set(DEFAULT_MIN_PROGRESSION);

      parentRegistry.addChild(registry);
   }

   public void setInitialConfiguration(KinematicsToolboxOutputStatus kinematicsToolboxOutputStatus)
   {
      RobotConfigurationData configurationData = new RobotConfigurationData();
      configurationData.setJointNameHash(kinematicsToolboxOutputStatus.getJointNameHash());
      MessageTools.copyData(kinematicsToolboxOutputStatus.getDesiredJointAngles(), configurationData.getJointAngles());
      configurationData.getRootPosition().set(new Vector3D32(kinematicsToolboxOutputStatus.getDesiredRootPosition()));
      configurationData.getRootOrientation().set(new Quaternion32(kinematicsToolboxOutputStatus.getDesiredRootOrientation()));
      setInitialConfiguration(configurationData);
   }

   public void setInitialConfiguration(RobotConfigurationData robotConfigurationData)
   {
      desiredRobotStateUpdater.setRobotConfigurationData(robotConfigurationData);
   }

   public void setCapturabilityBasedStatus(CapturabilityBasedStatus capturabilityBasedStatus)
   {
      controller.updateCapturabilityBasedStatus(capturabilityBasedStatus);
   }

   public void setAsDoubleSupport()
   {
      controller.updateFootSupportState(true, true);
   }

   public void submit(Iterable<KinematicsToolboxRigidBodyMessage> rigidBodyMessages)
   {
      rigidBodyMessages.forEach(this::submit);
   }

   public void submit(KinematicsToolboxRigidBodyMessage rigidBodyMessage)
   {
      commandInputManager.submitMessage(rigidBodyMessage);
   }

   public void submit(KinematicsToolboxCenterOfMassMessage centerOfMassMessage)
   {
      commandInputManager.submitMessage(centerOfMassMessage);
   }

   public void submit(KinematicsToolboxRigidBodyCommand rigidBodyCommand)
   {
      commandInputManager.submitCommand(rigidBodyCommand);
   }

   public void submit(KinematicsToolboxCenterOfMassCommand centerOfMassCommand)
   {
      commandInputManager.submitCommand(centerOfMassCommand);
   }

   public void initialize()
   {
      boolean initialized = controller.initialize();

      if (!initialized)
      {
         throw new RuntimeException("Could not initialize the " + KinematicsToolboxController.class.getSimpleName());
      }
   }

   public boolean solve()
   {
      long startTime = System.nanoTime();

      boolean isSolutionGood = false;
      boolean isSolverStuck = false;
      solutionQuality.set(Double.NaN);
      double solutionQualityLast = Double.NaN;
      double solutionQualityBeforeLast = Double.NaN;
      int iteration = 0;

      while (!isSolutionGood && iteration < maximumNumberOfIterations.getIntegerValue())
      {
         controller.updateInternal();

         KinematicsToolboxOutputStatus solution = controller.getSolution();
         solutionQuality.set(solution.getSolutionQuality());

         if (!Double.isNaN(solutionQualityLast))
         {
            double deltaSolutionQualityLast = Math.abs(solutionQuality.getDoubleValue() - solutionQualityLast);
            double deltaSolutionQualityBeforeLast = Math.abs(solutionQuality.getDoubleValue() - solutionQualityBeforeLast);

            boolean isSolutionStable = deltaSolutionQualityLast < solutionStabilityThreshold.getDoubleValue();
            boolean isSolutionQualityHigh = solutionQuality.getDoubleValue() < solutionQualityThreshold.getDoubleValue();

            isSolutionGood = isSolutionStable && isSolutionQualityHigh;

            if (!isSolutionQualityHigh)
            {
               // current solution quality should be compared with not only the last value but also the value before the last.
               boolean stuckLast = (deltaSolutionQualityLast / solutionQuality.getDoubleValue()) < solutionMinimumProgression.getDoubleValue();
               boolean stuckBeforeLast = (deltaSolutionQualityBeforeLast / solutionQuality.getDoubleValue()) < solutionMinimumProgression.getDoubleValue();

               isSolverStuck = stuckLast || stuckBeforeLast;
            }
         }

         solutionQualityBeforeLast = solutionQualityLast;
         solutionQualityLast = solutionQuality.getDoubleValue();

         iteration++;

         if (isSolverStuck)
            break;
      }

      numberOfIterations.set(iteration);
      hasConverged.set(isSolutionGood);

      long endTime = System.nanoTime();

      computationTime.set(Conversions.nanosecondsToSeconds(endTime - startTime));

      return isSolutionGood;
   }

   public FullHumanoidRobotModel getDesiredFullRobotModel()
   {
      return controller.getDesiredFullRobotModel();
   }

   public KinematicsToolboxOutputStatus getSolution()
   {
      return controller.getSolution();
   }

   public OneDoFJointBasics[] getDesiredOneDoFJoints()
   {
      return controller.getDesiredOneDoFJoints();
   }
}
