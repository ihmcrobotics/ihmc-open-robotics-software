package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.KinematicsToolboxCenterOfMassMessage;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.KinematicsToolboxRigidBodyMessage;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class HumanoidKinematicsSolver
{
   private static final int DEFAULT_MAX_NUMBER_OF_ITERATIONS = 200;
   private static final double DEFAULT_QUALITY_THRESHOLD = 0.005;
   private static final double DEFAULT_STABILITY_THRESHOLD = 0.00001;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final HumanoidKinematicsToolboxController controller;
   private final CommandInputManager commandInputManager = new CommandInputManager(name, KinematicsToolboxModule.supportedCommands());

   private final YoDouble solutionQualityPrevious = new YoDouble("solutionQualityPrevious", registry);
   private final YoDouble solutionQualityThreshold = new YoDouble("solutionQualityThreshold", registry);
   private final YoDouble solutionStabilityThreshold = new YoDouble("solutionStabilityThreshold", registry);

   private final YoInteger numberOfIterations = new YoInteger("numberOfIterations", registry);
   private final YoInteger maximumNumberOfIterations = new YoInteger("maximumNumberOfIterations", registry);

   public HumanoidKinematicsSolver(FullHumanoidRobotModelFactory fullRobotModelFactory, YoGraphicsListRegistry yoGraphicsListRegistry,
                                   YoVariableRegistry parentRegistry)
   {
      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(KinematicsToolboxModule.supportedStatus());
      FullHumanoidRobotModel desiredFullRobotModel = fullRobotModelFactory.createFullRobotModel();

      controller = new HumanoidKinematicsToolboxController(commandInputManager, statusOutputManager, desiredFullRobotModel, yoGraphicsListRegistry, registry);

      commandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(desiredFullRobotModel));

      maximumNumberOfIterations.set(DEFAULT_MAX_NUMBER_OF_ITERATIONS);
      solutionQualityThreshold.set(DEFAULT_QUALITY_THRESHOLD);
      solutionStabilityThreshold.set(DEFAULT_STABILITY_THRESHOLD);

      parentRegistry.addChild(registry);
   }

   public void setInitialConfiguration(KinematicsToolboxOutputStatus kinematicsToolboxOutputStatus)
   {
      RobotConfigurationData configurationData = new RobotConfigurationData();
      configurationData.jointNameHash = kinematicsToolboxOutputStatus.jointNameHash;
      configurationData.jointAngles = kinematicsToolboxOutputStatus.desiredJointAngles.clone();
      configurationData.rootTranslation = new Vector3D32(kinematicsToolboxOutputStatus.getPelvisTranslation());
      configurationData.rootOrientation = new Quaternion32(kinematicsToolboxOutputStatus.getPelvisOrientation());
      setInitialConfiguration(configurationData);
   }

   public void setInitialConfiguration(RobotConfigurationData robotConfigurationData)
   {
      controller.updateRobotConfigurationData(robotConfigurationData);
   }

   public void submit(KinematicsToolboxRigidBodyMessage rigidBodyMessage)
   {
      commandInputManager.submitMessage(rigidBodyMessage);
   }

   public void submit(KinematicsToolboxCenterOfMassMessage centerOfMassMessage)
   {
      commandInputManager.submitMessage(centerOfMassMessage);
   }

   public void initialize()
   {
      solutionQualityPrevious.setToNaN();
      numberOfIterations.set(0);

      controller.updateFootSupportState(true, true);

      boolean initialized = controller.initialize();

      if (!initialized)
      {
         throw new RuntimeException("Could not initialize the " + KinematicsToolboxController.class.getSimpleName());
      }
   }

   public boolean solve()
   {
      boolean isSolutionGood = false;

      while (!isSolutionGood && numberOfIterations.getIntegerValue() < maximumNumberOfIterations.getIntegerValue())
      {
         controller.updateInternal();

         KinematicsToolboxOutputStatus solution = controller.getSolution();
         double solutionQualityCurrent = solution.getSolutionQuality();

         if (!solutionQualityPrevious.isNaN())
         {
            double deltaSolutionQuality = solutionQualityCurrent - solutionQualityPrevious.getDoubleValue();
            boolean isSolutionStable = Math.abs(deltaSolutionQuality) < solutionStabilityThreshold.getDoubleValue();
            boolean isSolutionQualityHigh = solutionQualityCurrent < solutionQualityThreshold.getDoubleValue();
            isSolutionGood = isSolutionStable && isSolutionQualityHigh;
         }

         solutionQualityPrevious.set(solutionQualityCurrent);
         numberOfIterations.increment();
      }

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
}
