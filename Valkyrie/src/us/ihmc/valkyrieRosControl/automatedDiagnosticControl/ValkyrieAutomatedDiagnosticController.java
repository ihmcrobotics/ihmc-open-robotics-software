package us.ihmc.valkyrieRosControl.automatedDiagnosticControl;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.avatar.diagnostics.AutomatedDiagnosticConfiguration;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitch;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.*;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.rosControl.EffortJointHandle;
import us.ihmc.rosControl.wholeRobot.*;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters.DiagnosticEnvironment;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticSensorProcessingConfiguration;
import us.ihmc.sensorProcessing.model.DesiredJointDataHolder;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
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
import us.ihmc.wholeBodyController.diagnostics.logging.DiagnosticLoggerConfiguration;

import java.io.InputStream;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import static us.ihmc.valkyrieRosControl.ValkyrieRosControlController.*;

public class ValkyrieAutomatedDiagnosticController extends IHMCWholeRobotControlJavaBridge
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

   private DesiredJointDataHolder estimatorDesiredJointDataHolder;
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

      HashMap<String, EffortJointHandle> jointHandles = new HashMap<>();
      for (String joint : controlledJoints)
      {
         jointHandles.put(joint, createEffortJointHandle(joint));
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
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

      ValkyrieDiagnosticParameters diagnosticParameters = new ValkyrieDiagnosticParameters(DiagnosticEnvironment.RUNTIME_CONTROLLER, robotModel, true);
      DiagnosticSensorProcessingConfiguration diagnosticSensorProcessingConfiguration = new DiagnosticSensorProcessingConfiguration(diagnosticParameters,
            stateEstimatorParameters);

      HashMap<String, PositionJointHandle> emptyPositionJointHandles = new HashMap<>();
      HashMap<String, JointStateHandle> emptyJointStateHandles = new HashMap<>();
      ValkyrieRosControlSensorReaderFactory sensorReaderFactory = new ValkyrieRosControlSensorReaderFactory(timestampProvider,
            diagnosticSensorProcessingConfiguration, jointHandles, emptyPositionJointHandles, emptyJointStateHandles, imuHandles, forceTorqueSensorHandles, sensorInformation);

      FloatingInverseDynamicsJoint rootJoint = fullRobotModel.getRootJoint();
      IMUDefinition[] imuDefinitions = fullRobotModel.getIMUDefinitions();
      ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();
      ContactSensorHolder contactSensorHolder = new ContactSensorHolder(Arrays.asList(fullRobotModel.getContactSensorDefinitions()));
      RawJointSensorDataHolderMap rawJointSensorDataHolderMap = new RawJointSensorDataHolderMap(fullRobotModel);
      estimatorDesiredJointDataHolder = new DesiredJointDataHolder(fullRobotModel.getOneDoFJoints());
      sensorReaderFactory.build(rootJoint, imuDefinitions, forceSensorDefinitions, contactSensorHolder, rawJointSensorDataHolderMap,
            estimatorDesiredJointDataHolder, registry);
      sensorReader = sensorReaderFactory.getSensorReader();
      sensorReader.setDoIHMCControlRatio(1.0);
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

      InputStream gainStream = getClass().getClassLoader().getResourceAsStream(diagnosticGainsFilePath);
      InputStream setpointStream = getClass().getClassLoader().getResourceAsStream(diagnosticSetPointsFilePath);
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
      sensorReader.readSensors();

      if (firstEstimatorTick)
      {
         startTime.set(time);
         stateEstimator.initialize();
         firstEstimatorTick = false;
      }

      stateEstimator.doControl();

      if (!startController.getBooleanValue())
      {
         firstDiagnosticControlTick = true;
         diagnosticController.setRobotIsAlive(false);
      }
      else if (firstDiagnosticControlTick)
      {
         diagnosticController.setRobotIsAlive(true);
         diagnosticController.initialize();
         firstDiagnosticControlTick = false;
      }

      diagnosticController.doControl();
      estimatorDesiredJointDataHolder.updateFromModel();
      sensorReader.writeCommandsToRobot();
      
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
      SideDependentList<? extends ContactablePlaneBody> bipedFeet = contactableBodiesFactory.createFootContactableBodies(fullRobotModel, estimatorReferenceFrames);

      double gravityMagnitude = Math.abs(gravity);
      double totalRobotWeight = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator()) * gravityMagnitude;

      ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();
      ForceSensorDataHolder forceSensorDataHolderToUpdate = new ForceSensorDataHolder(Arrays.asList(forceSensorDefinitions));
      CenterOfMassDataHolder centerOfMassDataHolderToUpdate = new CenterOfMassDataHolder();

      Map<RigidBody, FootSwitchInterface> footSwitchMap = new LinkedHashMap<RigidBody, FootSwitchInterface>();
      Map<RigidBody, ContactablePlaneBody> bipedFeetMap = new LinkedHashMap<RigidBody, ContactablePlaneBody>();

      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody contactablePlaneBody = bipedFeet.get(robotSide);
         RigidBody rigidBody = contactablePlaneBody.getRigidBody();
         bipedFeetMap.put(rigidBody, contactablePlaneBody);
         
         String footForceSensorName = sensorInformation.getFeetForceSensorNames().get(robotSide);
         ForceSensorDataReadOnly footForceSensorForEstimator = forceSensorDataHolderToUpdate.getByName(footForceSensorName);
         String namePrefix = bipedFeet.get(robotSide).getName() + "StateEstimator";

         double footSwitchCoPThresholdFraction = stateEstimatorParameters.getFootSwitchCoPThresholdFraction();
         double contactThresholdForce = stateEstimatorParameters.getContactThresholdForce();

         WrenchBasedFootSwitch wrenchBasedFootSwitch = new WrenchBasedFootSwitch(namePrefix, footForceSensorForEstimator, footSwitchCoPThresholdFraction,
               totalRobotWeight, bipedFeet.get(robotSide), null, contactThresholdForce, registry);
         footSwitchMap.put(rigidBody, wrenchBasedFootSwitch);

      }

      String[] imuSensorsToUseInStateEstimator = sensorInformation.getIMUSensorsToUseInStateEstimator();

      // Create the sensor readers and state estimator here:
      DRCKinematicsBasedStateEstimator stateEstimator = new DRCKinematicsBasedStateEstimator(inverseDynamicsStructure, stateEstimatorParameters,
            sensorOutputMapReadOnly, forceSensorDataHolderToUpdate, centerOfMassDataHolderToUpdate,
            imuSensorsToUseInStateEstimator, gravityMagnitude, footSwitchMap, null,
            new RobotMotionStatusHolder(), bipedFeetMap, yoGraphicsListRegistry);

      registry.addChild(stateEstimator.getYoVariableRegistry());

      return stateEstimator;
   }
}
