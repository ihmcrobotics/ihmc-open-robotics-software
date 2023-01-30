package us.ihmc.valkyrieRosControl.automatedDiagnosticControl;

import static us.ihmc.valkyrieRosControl.ValkyrieRosControlController.readIMUs;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commons.Conversions;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.rosControl.EffortJointHandle;
import us.ihmc.rosControl.wholeRobot.ForceTorqueSensorHandle;
import us.ihmc.rosControl.wholeRobot.IHMCWholeRobotControlJavaBridge;
import us.ihmc.rosControl.wholeRobot.IMUHandle;
import us.ihmc.rosControl.wholeRobot.JointStateHandle;
import us.ihmc.rosControl.wholeRobot.PositionJointHandle;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.DRCKinematicsBasedStateEstimator;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ForceSensorStateUpdater;
import us.ihmc.tools.SettableTimestampProvider;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.diagnostic.ValkyrieDiagnosticParameters;
import us.ihmc.valkyrie.parameters.ValkyrieSensorInformation;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlSensorReader;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlSensorReaderFactory;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.diagnostics.AutomatedDiagnosticAnalysisController;
import us.ihmc.wholeBodyController.diagnostics.AutomatedDiagnosticConfiguration;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticControllerToolbox;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticParameters;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticSensorProcessingConfiguration;
import us.ihmc.wholeBodyController.diagnostics.logging.DiagnosticLoggerConfiguration;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class ValkyrieAutomatedDiagnosticController extends IHMCWholeRobotControlJavaBridge
{
   private static final String[] controlledJoints = {"leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll",
         "rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll", "torsoYaw", "torsoPitch", "torsoRoll",
         "leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch", "rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw",
         "rightElbowPitch"};

   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT);

   private YoVariableServer yoVariableServer;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private final SettableTimestampProvider wallTimeProvider = new SettableTimestampProvider();
   private final TimestampProvider monotonicTimeProvider = () -> RealtimeThread.getCurrentMonotonicClockTime();
   private final YoDouble diagnosticControllerTime = new YoDouble("diagnosticControllerTime", registry);
   private final ExecutionTimer diagnosticControllerTimer = new ExecutionTimer("diagnosticControllerTimer", 10.0, registry);
   private final YoLong startTime = new YoLong("startTime", registry);
   private final YoBoolean startController = new YoBoolean("startController", registry);

   private JointDesiredOutputList estimatorDesiredJointDataHolder;
   private ValkyrieRosControlSensorReader sensorReader;
   private StateEstimatorController stateEstimator;
   private ForceSensorStateUpdater forceSensorStateUpdater;
   private AutomatedDiagnosticAnalysisController diagnosticController;

   public ValkyrieAutomatedDiagnosticController()
   {
      DiagnosticLoggerConfiguration.setupLogging(diagnosticControllerTime, getClass(), robotModel.getSimpleRobotName(), true);
   }

   @Override
   protected void init()
   {
      long maxMemory = Runtime.getRuntime().maxMemory();

      ValkyrieSensorInformation sensorInformation = robotModel.getSensorInformation();

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
      for (String ftSensorName : sensorInformation.getForceSensorNames())
      {
         forceTorqueSensorHandles.put(ftSensorName, createForceTorqueSensorHandle(ftSensorName));
      }

      /*
       * Create registries
       */

      /*
       * Create network servers/clients
       */
      double diagnosticControllerDT = robotModel.getEstimatorDT();
      yoVariableServer = new YoVariableServer(getClass(), robotModel.getLogModelProvider(), robotModel.getLogSettings(), diagnosticControllerDT);

      /*
       * Create sensor reader
       */
      StateEstimatorParameters stateEstimatorParameters = robotModel.getStateEstimatorParameters();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      estimatorDesiredJointDataHolder = new JointDesiredOutputList(fullRobotModel.getOneDoFJoints());

      DiagnosticParameters diagnosticParameters = robotModel.getDiagnoticParameters();
      DiagnosticSensorProcessingConfiguration diagnosticSensorProcessingConfiguration = diagnosticParameters.getOrCreateSensorProcessingConfiguration(stateEstimatorParameters,
                                                                                                                                                      estimatorDesiredJointDataHolder);

      HashMap<String, PositionJointHandle> emptyPositionJointHandles = new HashMap<>();
      HashMap<String, JointStateHandle> emptyJointStateHandles = new HashMap<>();
      ValkyrieRosControlSensorReaderFactory sensorReaderFactory = new ValkyrieRosControlSensorReaderFactory(wallTimeProvider,
                                                                                                            monotonicTimeProvider,
                                                                                                            diagnosticSensorProcessingConfiguration,
                                                                                                            jointHandles,
                                                                                                            emptyPositionJointHandles,
                                                                                                            emptyJointStateHandles,
                                                                                                            imuHandles,
                                                                                                            forceTorqueSensorHandles,
                                                                                                            robotModel.getJointMap(),
                                                                                                            sensorInformation);

      FloatingJointBasics rootJoint = fullRobotModel.getRootJoint();
      IMUDefinition[] imuDefinitions = fullRobotModel.getIMUDefinitions();
      ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();
      sensorReaderFactory.build(rootJoint, imuDefinitions, forceSensorDefinitions, estimatorDesiredJointDataHolder, registry);
      sensorReader = sensorReaderFactory.getSensorReader();
      SensorOutputMapReadOnly processedSensorOutputMap = sensorReader.getProcessedSensorOutputMap();

      /*
       * Create state estimator
       */
      double gravity = ValkyrieRosControlController.gravity;
      stateEstimator = createStateEstimator(robotModel, gravity, processedSensorOutputMap, fullRobotModel);

      /*
       * Create diagnostic controller
       */
      DiagnosticControllerToolbox toolbox = new DiagnosticControllerToolbox(fullRobotModel.getElevator(),
                                                                            fullRobotModel.getRootJoint(),
                                                                            estimatorDesiredJointDataHolder,
                                                                            processedSensorOutputMap,
                                                                            diagnosticParameters,
                                                                            diagnosticControllerTime,
                                                                            registry);

      diagnosticController = new AutomatedDiagnosticAnalysisController(toolbox);
      AutomatedDiagnosticConfiguration automatedDiagnosticConfiguration = new AutomatedDiagnosticConfiguration(toolbox, diagnosticController);
      automatedDiagnosticConfiguration.addWait(1.0);
      automatedDiagnosticConfiguration.addJointCheckUps(ValkyrieDiagnosticParameters.defaultJointCheckUpConfiguration(robotModel.getJointMap()));
      automatedDiagnosticConfiguration.addPelvisIMUCheckUpDiagnostic(DiagnosticParameters.defaultPelvisIMUCheckUp(robotModel.getSensorInformation(),
                                                                                                                  robotModel.getJointMap()));

      yoVariableServer.setMainRegistry(registry, fullRobotModel.getElevator(), yoGraphicsListRegistry);
      yoVariableServer.start();
   }

   private boolean firstEstimatorTick = true;
   private boolean firstDiagnosticControlTick = true;

   @Override
   protected void doControl(long rosTime, long duration)
   {
      diagnosticControllerTimer.startMeasurement();
      wallTimeProvider.setTimestamp(rosTime);
      sensorReader.readSensors();

      if (firstEstimatorTick)
      {
         startTime.set(rosTime);
         stateEstimator.initialize();
         forceSensorStateUpdater.initialize();
         firstEstimatorTick = false;
      }

      stateEstimator.doControl();
      forceSensorStateUpdater.updateForceSensorState();

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
      sensorReader.writeCommandsToRobot();

      diagnosticControllerTime.set(Conversions.nanosecondsToSeconds(rosTime - startTime.getLongValue()));

      yoVariableServer.update(rosTime);

      diagnosticControllerTimer.stopMeasurement();
   }

   private StateEstimatorController createStateEstimator(DRCRobotModel robotModel, double gravity, SensorOutputMapReadOnly sensorOutputMapReadOnly,
                                                         FullHumanoidRobotModel fullRobotModel)
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = DRCControllerThread.createInverseDynamicsStructure(fullRobotModel);
      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      HumanoidRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      StateEstimatorParameters stateEstimatorParameters = robotModel.getStateEstimatorParameters();

      HumanoidReferenceFrames estimatorReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
      contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(),
                                                       contactPointParameters.getControllerToeContactLines());
      contactableBodiesFactory.setFullRobotModel(fullRobotModel);
      contactableBodiesFactory.setReferenceFrames(estimatorReferenceFrames);
      SideDependentList<ContactableFoot> bipedFeet = new SideDependentList<>(contactableBodiesFactory.createFootContactableFeet());
      contactableBodiesFactory.disposeFactory();

      double gravityMagnitude = Math.abs(gravity);
      double totalRobotWeight = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator()) * gravityMagnitude;

      ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();
      ForceSensorDataHolder forceSensorDataHolderToUpdate = new ForceSensorDataHolder(Arrays.asList(forceSensorDefinitions));
      CenterOfMassDataHolder centerOfMassDataHolderToUpdate = new CenterOfMassDataHolder();

      Map<RigidBodyBasics, FootSwitchInterface> footSwitchMap = new LinkedHashMap<RigidBodyBasics, FootSwitchInterface>();
      Map<RigidBodyBasics, ContactablePlaneBody> bipedFeetMap = new LinkedHashMap<RigidBodyBasics, ContactablePlaneBody>();

      FootSwitchFactory footSwitchFactory = stateEstimatorParameters.getFootSwitchFactory();

      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody contactablePlaneBody = bipedFeet.get(robotSide);
         RigidBodyBasics rigidBody = contactablePlaneBody.getRigidBody();
         bipedFeetMap.put(rigidBody, contactablePlaneBody);

         String footForceSensorName = sensorInformation.getFeetForceSensorNames().get(robotSide);
         ForceSensorDataReadOnly footForceSensorForEstimator = forceSensorDataHolderToUpdate.getByName(footForceSensorName);
         String namePrefix = bipedFeet.get(robotSide).getName() + "StateEstimator";

         FootSwitchInterface footSwitchInterface = footSwitchFactory.newFootSwitch(namePrefix,
                                                                                   contactablePlaneBody,
                                                                                   Collections.singleton(bipedFeet.get(robotSide.getOppositeSide())),
                                                                                   footForceSensorForEstimator,
                                                                                   totalRobotWeight,
                                                                                   null,
                                                                                   registry);
         footSwitchMap.put(rigidBody, footSwitchInterface);

      }

      String[] imuSensorsToUseInStateEstimator = sensorInformation.getIMUSensorsToUseInStateEstimator();

      // Create the sensor readers and state estimator here:
      RobotMotionStatusHolder robotMotionStatusHolder = new RobotMotionStatusHolder();
      robotMotionStatusHolder.setCurrentRobotMotionStatus(RobotMotionStatus.UNKNOWN);
      StateEstimatorController stateEstimator = new DRCKinematicsBasedStateEstimator(inverseDynamicsStructure,
                                                                                     stateEstimatorParameters,
                                                                                     sensorOutputMapReadOnly,
                                                                                     centerOfMassDataHolderToUpdate,
                                                                                     imuSensorsToUseInStateEstimator,
                                                                                     gravityMagnitude,
                                                                                     footSwitchMap,
                                                                                     null,
                                                                                     robotMotionStatusHolder,
                                                                                     bipedFeetMap,
                                                                                     yoGraphicsListRegistry);

      registry.addChild(stateEstimator.getYoRegistry());

      forceSensorStateUpdater = new ForceSensorStateUpdater(fullRobotModel.getRootJoint(),
                                                            sensorOutputMapReadOnly,
                                                            forceSensorDataHolderToUpdate,
                                                            stateEstimatorParameters,
                                                            gravityMagnitude,
                                                            robotMotionStatusHolder,
                                                            yoGraphicsListRegistry,
                                                            registry);

      return stateEstimator;
   }
}
