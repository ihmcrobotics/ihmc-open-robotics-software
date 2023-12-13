package us.ihmc.avatar.diagnostics;

import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.avatar.SimulatedLowLevelOutputWriter;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
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
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedSensorHolderAndReaderFromRobotFactory;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.DRCKinematicsBasedStateEstimator;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.KinematicsBasedStateEstimatorFactory;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.diagnostics.AutomatedDiagnosticAnalysisController;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticControllerToolbox;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticParameters;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticSensorProcessingConfiguration;
import us.ihmc.wholeBodyController.diagnostics.logging.DiagnosticLoggerConfiguration;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class AutomatedDiagnosticSimulationFactory implements RobotController
{
   private final DRCRobotModel robotModel;
   private RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup;
   private final YoRegistry simulationRegistry = new YoRegistry("AutomatedDiagnosticSimulation");
   private final YoDouble controllerTime = new YoDouble("controllerTime", simulationRegistry);
   private final AlphaFilteredYoVariable averageControllerTime = new AlphaFilteredYoVariable("averageControllerTime", simulationRegistry, 0.99, controllerTime);
   private SensorReader sensorReader;
   private AutomatedDiagnosticAnalysisController automatedDiagnosticAnalysisController;
   private JointDesiredOutputWriter lowLevelOutputWriter;

   private final Point3D scsCameraPosition = new Point3D(0.0, -8.0, 1.8);
   private final Point3D scsCameraFix = new Point3D(0.0, 0.0, 1.35);

   private HumanoidFloatingRootJointRobot simulatedRobot;
   private HumanoidReferenceFrames humanoidReferenceFrames;
   private StateEstimatorController stateEstimator;

   private final SensorDataContext sensorDataContext = new SensorDataContext();

   public AutomatedDiagnosticSimulationFactory(DRCRobotModel robotModel)
   {
      this.robotModel = robotModel;
   }

   public void startSimulation()
   {
      SimulationConstructionSetParameters simulationParameters = new SimulationConstructionSetParameters(true, 16000);
      SimulationConstructionSet scs = new SimulationConstructionSet(simulatedRobot, simulationParameters);
      scs.setDT(robotModel.getSimulateDT(), 10);
      scs.setCameraPosition(scsCameraPosition.getX(), scsCameraPosition.getY(), scsCameraPosition.getZ());
      scs.setCameraFix(scsCameraFix.getX(), scsCameraFix.getY(), scsCameraFix.getZ());
      new DefaultParameterReader().readParametersInRegistry(scs.getRootRegistry());
      scs.startOnAThread();
   }

   public void createDiagnosticController(boolean startWithRobotAlive)
   {
      simulatedRobot = robotModel.createHumanoidFloatingRootJointRobot(false);
      DiagnosticLoggerConfiguration.setupLogging(simulatedRobot.getYoTime(), getClass(), robotModel.getSimpleRobotName());

      if (robotInitialSetup == null)
         robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);
      robotInitialSetup.initializeRobot(simulatedRobot);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      YoDouble yoTime = simulatedRobot.getYoTime();

      StateEstimatorParameters stateEstimatorParameters = robotModel.getStateEstimatorParameters();

      DiagnosticParameters diagnosticParameters = robotModel.getDiagnoticParameters();

      JointDesiredOutputList lowLevelOutput = new JointDesiredOutputList(fullRobotModel.getOneDoFJoints());
      DiagnosticSensorProcessingConfiguration sensorProcessingConfiguration = diagnosticParameters.getOrCreateSensorProcessingConfiguration(stateEstimatorParameters,
                                                                                                                                            lowLevelOutput);

      SensorOutputMapReadOnly sensorOutputMap = createStateEstimator(fullRobotModel, stateEstimatorParameters, sensorProcessingConfiguration);

      DiagnosticControllerToolbox diagnosticControllerToolbox = new DiagnosticControllerToolbox(fullRobotModel.getElevator(),
                                                                                                fullRobotModel.getRootJoint(),
                                                                                                lowLevelOutput,
                                                                                                sensorOutputMap,
                                                                                                diagnosticParameters,
                                                                                                yoTime,
                                                                                                simulationRegistry);
      automatedDiagnosticAnalysisController = new AutomatedDiagnosticAnalysisController(diagnosticControllerToolbox);
      automatedDiagnosticAnalysisController.setRobotIsAlive(startWithRobotAlive);

      lowLevelOutputWriter = new SimulatedLowLevelOutputWriter(simulatedRobot, false);
      lowLevelOutputWriter.setJointDesiredOutputList(lowLevelOutput);

      int simulationTicksPerControlTick = (int) (robotModel.getEstimatorDT() / robotModel.getSimulateDT());
      simulatedRobot.setController(this, simulationTicksPerControlTick);
   }

   private SensorOutputMapReadOnly createStateEstimator(FullHumanoidRobotModel fullRobotModel, StateEstimatorParameters stateEstimatorParameters,
                                                        DiagnosticSensorProcessingConfiguration sensorProcessingConfiguration)
   {
      FloatingJointBasics rootJoint = fullRobotModel.getRootJoint();
      IMUDefinition[] imuDefinitions = fullRobotModel.getIMUDefinitions();
      ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();
      JointDesiredOutputList estimatorDesiredJointDataHolder = null;

      ForceSensorDataHolder forceSensorDataHolderToUpdate = new ForceSensorDataHolder(Arrays.asList(forceSensorDefinitions));
      CenterOfMassDataHolder centerOfMassDataHolderToUpdate = new CenterOfMassDataHolder();

      SimulatedSensorHolderAndReaderFromRobotFactory sensorReaderFactory = new SimulatedSensorHolderAndReaderFromRobotFactory(simulatedRobot,
                                                                                                                              sensorProcessingConfiguration);
      sensorReaderFactory.build(rootJoint, imuDefinitions, forceSensorDefinitions, estimatorDesiredJointDataHolder, simulationRegistry);
      sensorReader = sensorReaderFactory.getSensorReader();

      FullInverseDynamicsStructure inverseDynamicsStructure = KinematicsBasedStateEstimatorFactory.createFullInverseDynamicsStructure(fullRobotModel);
      SensorOutputMapReadOnly processedSensorOutputMap = sensorReader.getProcessedSensorOutputMap();
      HumanoidRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      String[] imuSensorsToUseInStateEstimator = sensorInformation.getIMUSensorsToUseInStateEstimator();
      double gravitationalAcceleration = 9.81;
      double totalRobotWeight = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator()) * gravitationalAcceleration;

      humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
      contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(),
                                                       contactPointParameters.getControllerToeContactLines());
      contactableBodiesFactory.setFullRobotModel(fullRobotModel);
      contactableBodiesFactory.setReferenceFrames(humanoidReferenceFrames);
      SideDependentList<ContactableFoot> bipedFeet = new SideDependentList<>(contactableBodiesFactory.createFootContactableFeet());
      contactableBodiesFactory.disposeFactory();

      Map<RigidBodyBasics, FootSwitchInterface> footSwitchMap = new LinkedHashMap<RigidBodyBasics, FootSwitchInterface>();
      Map<RigidBodyBasics, ContactablePlaneBody> bipedFeetMap = new LinkedHashMap<RigidBodyBasics, ContactablePlaneBody>();

      SideDependentList<FootSwitchFactory> footSwitchFactories = stateEstimatorParameters.getFootSwitchFactories();

      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody contactablePlaneBody = bipedFeet.get(robotSide);
         RigidBodyBasics rigidBody = contactablePlaneBody.getRigidBody();
         bipedFeetMap.put(rigidBody, contactablePlaneBody);

         String footForceSensorName = sensorInformation.getFeetForceSensorNames().get(robotSide);
         ForceSensorDataReadOnly footForceSensorForEstimator = forceSensorDataHolderToUpdate.getData(footForceSensorName);
         String namePrefix = bipedFeet.get(robotSide).getName() + "StateEstimator";

         FootSwitchFactory footSwitchFactory = footSwitchFactories.get(robotSide);
         FootSwitchInterface footSwitchInterface = footSwitchFactory.newFootSwitch(namePrefix,
                                                                                   contactablePlaneBody,
                                                                                   Collections.singleton(bipedFeet.get(robotSide.getOppositeSide())),
                                                                                   fullRobotModel.getRootBody(),
                                                                                   footForceSensorForEstimator,
                                                                                   totalRobotWeight,
                                                                                   null,
                                                                                   simulationRegistry);
         footSwitchMap.put(rigidBody, footSwitchInterface);
      }

      RobotMotionStatusHolder robotMotionStatusHolder = new RobotMotionStatusHolder();
      robotMotionStatusHolder.setCurrentRobotMotionStatus(RobotMotionStatus.UNKNOWN);
      stateEstimator = new DRCKinematicsBasedStateEstimator(inverseDynamicsStructure,
                                                            stateEstimatorParameters,
                                                            processedSensorOutputMap,
                                                            centerOfMassDataHolderToUpdate,
                                                            imuSensorsToUseInStateEstimator,
                                                            gravitationalAcceleration,
                                                            footSwitchMap,
                                                            null,
                                                            robotMotionStatusHolder,
                                                            bipedFeetMap,
                                                            forceSensorDataHolderToUpdate,
                                                            null);
      simulationRegistry.addChild(stateEstimator.getYoRegistry());

      return sensorReader.getProcessedSensorOutputMap();
   }

   public void setRobotInitialSetup(RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup)
   {
      this.robotInitialSetup = robotInitialSetup;
   }

   public void setRobotInitialSetup(double groundHeight, double initialYaw)
   {
      robotInitialSetup = robotModel.getDefaultRobotInitialSetup(groundHeight, initialYaw);
   }

   /**
    * Sets the initial SCS camera position.
    * 
    * @param positionX
    * @param positionY
    * @param positionZ
    */
   public void setSCSCameraPosition(double positionX, double positionY, double positionZ)
   {
      scsCameraPosition.set(positionX, positionY, positionZ);
   }

   /**
    * Sets the initial fix point that the SCS camera looks at.
    * 
    * @param fixX
    * @param fixY
    * @param fixZ
    */
   public void setSCSCameraFix(double fixX, double fixY, double fixZ)
   {
      scsCameraFix.set(fixX, fixY, fixZ);
   }

   @Override
   public void initialize()
   {
   }

   private boolean firstControlTick = true;

   @Override
   public void doControl()
   {
      long startTime = System.nanoTime();

      lowLevelOutputWriter.writeBefore(startTime);
      long timestamp = sensorReader.read(sensorDataContext);
      sensorReader.compute(timestamp, sensorDataContext);
      humanoidReferenceFrames.updateFrames();

      if (firstControlTick)
      {
         stateEstimator.initialize();
         automatedDiagnosticAnalysisController.initialize();
         firstControlTick = false;
      }
      else
      {
         stateEstimator.doControl();
         automatedDiagnosticAnalysisController.doControl();
      }

      lowLevelOutputWriter.writeAfter();
      long endTime = System.nanoTime();
      controllerTime.set(Conversions.nanosecondsToSeconds(endTime - startTime));
      averageControllerTime.update();
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return simulationRegistry;
   }

   @Override
   public String getName()
   {
      return simulationRegistry.getName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }
}
