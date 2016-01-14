package us.ihmc.valkyrieRosControl.automatedDiagnosticControl;

import static us.ihmc.valkyrieRosControl.ValkyrieRosControlController.forceTorqueSensorModelNames;
import static us.ihmc.valkyrieRosControl.ValkyrieRosControlController.readForceTorqueSensors;
import static us.ihmc.valkyrieRosControl.ValkyrieRosControlController.readIMUs;

import java.io.InputStream;
import java.util.Arrays;
import java.util.HashMap;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitch;
import us.ihmc.darpaRoboticsChallenge.diagnostics.AutomatedDiagnosticConfiguration;
import us.ihmc.darpaRoboticsChallenge.diagnostics.logging.DiagnosticLoggerConfiguration;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.ContactSensorHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.rosControl.JointHandle;
import us.ihmc.rosControl.valkyrie.ForceTorqueSensorHandle;
import us.ihmc.rosControl.valkyrie.IHMCValkyrieControlJavaBridge;
import us.ihmc.rosControl.valkyrie.IMUHandle;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters.DiagnosticEnvironment;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticSensorProcessingConfiguration;
import us.ihmc.sensorProcessing.model.DesiredJointDataHolder;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.DRCKinematicsBasedStateEstimator;
import us.ihmc.tools.SettableTimestampProvider;
import us.ihmc.util.PeriodicRealtimeThreadScheduler;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieConfigurationRoot;
import us.ihmc.valkyrie.diagnostic.ValkyrieDiagnosticParameters;
import us.ihmc.valkyrie.parameters.ValkyrieSensorInformation;
import us.ihmc.valkyrieRosControl.ValkyriePriorityParameters;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlSensorReader;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlSensorReaderFactory;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.diagnostics.AutomatedDiagnosticAnalysisController;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticControllerToolbox;

public class ValkyrieAutomatedDiagnosticController extends IHMCValkyrieControlJavaBridge
{
   private static final String[] controlledJoints = { "leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll",
         "rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll", "torsoYaw", "torsoPitch", "torsoRoll",
         "leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch", "rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw",
         "rightElbowPitch" };

   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(DRCRobotModel.RobotTarget.REAL_ROBOT, true);

   private YoVariableServer yoVariableServer;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private final SettableTimestampProvider timestampProvider = new SettableTimestampProvider();
   private final DoubleYoVariable diagnosticControllerTime = new DoubleYoVariable("diagnosticControllerTime", registry);
   private final ExecutionTimer diagnosticControllerTimer = new ExecutionTimer("diagnosticControllerTimer", 10.0, registry);
   private final LongYoVariable startTime = new LongYoVariable("startTime", registry);
   private final BooleanYoVariable startController = new BooleanYoVariable("startController", registry);

   private final String diagnosticGainsFilePath = "diagnostic/realRobotPDGains.yaml";
   private final String diagnosticSetPointsFilePath = "diagnostic/diagnosticSetPoints.yaml";

   private ValkyrieRosControlSensorReader sensorReader;
   private DRCKinematicsBasedStateEstimator stateEstimator;
   private AutomatedDiagnosticAnalysisController diagnosticController;

   public ValkyrieAutomatedDiagnosticController()
   {
      DiagnosticLoggerConfiguration.setupLogging(diagnosticControllerTime, getClass(), robotModel.getSimpleRobotName(), true);
   }

   @Override
   protected void init()
   {
      long maxMemory = Runtime.getRuntime().maxMemory();

      System.out.println("Partying hard with max memory of: " + maxMemory);
      /*
       * Create joints
       */

      HashMap<String, JointHandle> jointHandles = new HashMap<>();
      for (String joint : controlledJoints)
      {
         jointHandles.put(joint, createJointHandle(joint));
      }

      HashMap<String, IMUHandle> imuHandles = new HashMap<>();
      for (String imu : readIMUs)
      {
         imuHandles.put(imu, createIMUHandle(imu));
      }

      HashMap<String, ForceTorqueSensorHandle> forceTorqueSensorHandles = new HashMap<>();
      for (int i = 0; i < readForceTorqueSensors.length; i++)
      {

         String forceTorqueSensor = readForceTorqueSensors[i];
         String modelName = forceTorqueSensorModelNames[i];
         forceTorqueSensorHandles.put(modelName, createForceTorqueSensorHandle(forceTorqueSensor));
      }

      /*
       * Create registries
       */

      ValkyrieSensorInformation sensorInformation = robotModel.getSensorInformation();

      /*
       * Create network servers/clients
       */
      double diagnosticControllerDT = robotModel.getEstimatorDT();
      yoVariableServer = new YoVariableServer(getClass(), new PeriodicRealtimeThreadScheduler(ValkyriePriorityParameters.LOGGER_PRIORITY),
            robotModel.getLogModelProvider(), robotModel.getLogSettings(ValkyrieConfigurationRoot.USE_CAMERAS_FOR_LOGGING), diagnosticControllerDT);

      /*
       * Create sensor reader
       */
      StateEstimatorParameters stateEstimatorParameters = robotModel.getStateEstimatorParameters();
      SDFFullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

      ValkyrieDiagnosticParameters diagnosticParameters = new ValkyrieDiagnosticParameters(DiagnosticEnvironment.RUNTIME_CONTROLLER, robotModel, true);
      DiagnosticSensorProcessingConfiguration diagnosticSensorProcessingConfiguration = new DiagnosticSensorProcessingConfiguration(diagnosticParameters,
            stateEstimatorParameters);

      ValkyrieRosControlSensorReaderFactory sensorReaderFactory = new ValkyrieRosControlSensorReaderFactory(timestampProvider,
            diagnosticSensorProcessingConfiguration, jointHandles, imuHandles, forceTorqueSensorHandles, sensorInformation);

      SixDoFJoint rootJoint = fullRobotModel.getRootJoint();
      IMUDefinition[] imuDefinitions = fullRobotModel.getIMUDefinitions();
      ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();
      ContactSensorHolder contactSensorHolder = new ContactSensorHolder(Arrays.asList(fullRobotModel.getContactSensorDefinitions()));
      RawJointSensorDataHolderMap rawJointSensorDataHolderMap = new RawJointSensorDataHolderMap(fullRobotModel);
      DesiredJointDataHolder estimatorDesiredJointDataHolder = new DesiredJointDataHolder(fullRobotModel.getOneDoFJoints());
      sensorReaderFactory.build(rootJoint, imuDefinitions, forceSensorDefinitions, contactSensorHolder, rawJointSensorDataHolderMap,
            estimatorDesiredJointDataHolder, registry);
      sensorReader = sensorReaderFactory.getSensorReader();
      SensorOutputMapReadOnly sensorOutputMap = sensorReader.getSensorOutputMapReadOnly();

      /*
       * Create state estimator
       */
      double gravity = ValkyrieRosControlController.gravity;
      stateEstimator = createStateEstimator(robotModel, gravity, sensorOutputMap, fullRobotModel);

      /*
       * Create diagnostic controller
       */
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      DiagnosticControllerToolbox toolbox = new DiagnosticControllerToolbox(fullRobotModel, sensorOutputMap, diagnosticParameters, walkingControllerParameters,
            diagnosticControllerTime, diagnosticControllerDT, diagnosticSensorProcessingConfiguration, registry);

      InputStream gainStream = getClass().getResourceAsStream(diagnosticGainsFilePath);
      InputStream setpointStream = getClass().getResourceAsStream(diagnosticSetPointsFilePath);
      diagnosticController = new AutomatedDiagnosticAnalysisController(toolbox, gainStream, setpointStream, registry);
      AutomatedDiagnosticConfiguration automatedDiagnosticConfiguration = new AutomatedDiagnosticConfiguration(toolbox, diagnosticController);
      automatedDiagnosticConfiguration.addWait(1.0);
      automatedDiagnosticConfiguration.addJointCheckUpDiagnostic();
      automatedDiagnosticConfiguration.addPelvisIMUCheckUpDiagnostic();

      yoVariableServer.setMainRegistry(registry, fullRobotModel, yoGraphicsListRegistry);
      yoVariableServer.start();
   }

   private boolean firstEstimatorTick = true;
   private boolean firstDiagnosticControlTick = true;

   @Override
   protected void doControl(long time, long duration)
   {
      diagnosticControllerTimer.startMeasurement();
      timestampProvider.setTimestamp(time);
      sensorReader.read();

      if (firstEstimatorTick)
      {
         stateEstimator.initialize();
         firstEstimatorTick = false;
      }

      stateEstimator.doControl();

      if (!startController.getBooleanValue())
      {
         diagnosticControllerTime.set(0.0);
         firstDiagnosticControlTick = true;
         diagnosticControllerTimer.stopMeasurement();
         return;
      }

      if (firstDiagnosticControlTick)
      {
         startTime.set(time);
         diagnosticController.setRobotIsAlive(true);
         diagnosticController.initialize();
         firstDiagnosticControlTick = false;
      }

      diagnosticController.doControl();

      diagnosticControllerTime.set(TimeTools.nanoSecondstoSeconds(time - startTime.getLongValue()));

      yoVariableServer.update(time);

      diagnosticControllerTimer.stopMeasurement();
   }

   private DRCKinematicsBasedStateEstimator createStateEstimator(DRCRobotModel robotModel, double gravity, SensorOutputMapReadOnly sensorOutputMapReadOnly,
         FullHumanoidRobotModel fullRobotModel)
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = DRCControllerThread.createInverseDynamicsStructure(fullRobotModel);
      RobotContactPointParameters contactPointParameters = robotModel.getContactPointParameters();
      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      StateEstimatorParameters stateEstimatorParameters = robotModel.getStateEstimatorParameters();

      HumanoidReferenceFrames estimatorReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ContactableBodiesFactory contactableBodiesFactory = contactPointParameters.getContactableBodiesFactory();
      SideDependentList<ContactablePlaneBody> bipedFeet = contactableBodiesFactory.createFootContactableBodies(fullRobotModel, estimatorReferenceFrames);

      double gravityMagnitude = Math.abs(gravity);
      double totalRobotWeight = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator()) * gravityMagnitude;

      SideDependentList<FootSwitchInterface> footSwitches = new SideDependentList<>();

      ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();
      ForceSensorDataHolder forceSensorDataHolderToUpdate = new ForceSensorDataHolder(Arrays.asList(forceSensorDefinitions));

      for (RobotSide robotSide : RobotSide.values)
      {
         String footForceSensorName = sensorInformation.getFeetForceSensorNames().get(robotSide);
         ForceSensorDataReadOnly footForceSensorForEstimator = forceSensorDataHolderToUpdate.getByName(footForceSensorName);
         String namePrefix = bipedFeet.get(robotSide).getName() + "StateEstimator";

         double footSwitchCoPThresholdFraction = stateEstimatorParameters.getFootSwitchCoPThresholdFraction();
         double contactThresholdForce = stateEstimatorParameters.getContactThresholdForce();

         WrenchBasedFootSwitch wrenchBasedFootSwitch = new WrenchBasedFootSwitch(namePrefix, footForceSensorForEstimator, footSwitchCoPThresholdFraction,
               totalRobotWeight, bipedFeet.get(robotSide), null, contactThresholdForce, registry);
         footSwitches.put(robotSide, wrenchBasedFootSwitch);

      }

      String[] imuSensorsToUseInStateEstimator = sensorInformation.getIMUSensorsToUseInStateEstimator();

      // Create the sensor readers and state estimator here:
      DRCKinematicsBasedStateEstimator stateEstimator = new DRCKinematicsBasedStateEstimator(inverseDynamicsStructure, stateEstimatorParameters,
            sensorOutputMapReadOnly, forceSensorDataHolderToUpdate, imuSensorsToUseInStateEstimator, gravityMagnitude, footSwitches, null,
            new RobotMotionStatusHolder(), bipedFeet, yoGraphicsListRegistry);

      registry.addChild(stateEstimator.getYoVariableRegistry());

      return stateEstimator;
   }
}
