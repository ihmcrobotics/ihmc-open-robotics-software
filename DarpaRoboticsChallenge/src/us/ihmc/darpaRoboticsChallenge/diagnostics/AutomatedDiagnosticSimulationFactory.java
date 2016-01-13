package us.ihmc.darpaRoboticsChallenge.diagnostics;

import java.io.InputStream;
import java.util.Arrays;

import com.github.quickhull3d.Point3d;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitch;
import us.ihmc.darpaRoboticsChallenge.diagnostics.logging.DiagnosticLoggerConfiguration;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.ContactSensorHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters.DiagnosticEnvironment;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticSensorProcessingConfiguration;
import us.ihmc.sensorProcessing.model.DesiredJointDataHolder;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedSensorHolderAndReaderFromRobotFactory;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.DRCKinematicsBasedStateEstimator;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.DRCOutputWriter;
import us.ihmc.wholeBodyController.DRCSimulationOutputWriter;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.diagnostics.AutomatedDiagnosticAnalysisController;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticControllerToolbox;

public class AutomatedDiagnosticSimulationFactory implements RobotController
{
   private final DRCRobotModel robotModel;
   private DRCRobotInitialSetup<SDFHumanoidRobot> robotInitialSetup;
   private InputStream gainStream;
   private InputStream setpointStream;
   private final YoVariableRegistry simulationRegistry = new YoVariableRegistry("AutomatedDiagnosticSimulation");
   private final DoubleYoVariable controllerTime = new DoubleYoVariable("controllerTime", simulationRegistry);
   private final AlphaFilteredYoVariable averageControllerTime = new AlphaFilteredYoVariable("averageControllerTime", simulationRegistry, 0.99, controllerTime);
   private SensorReader sensorReader;
   private DiagnosticParameters diagnosticParameters;
   private AutomatedDiagnosticAnalysisController automatedDiagnosticAnalysisController;
   private DRCOutputWriter outputWriter;

   private final Point3d scsCameraPosition = new Point3d(0.0, -8.0, 1.8);
   private final Point3d scsCameraFix = new Point3d(0.0, 0.0, 1.35);

   private AutomatedDiagnosticConfiguration automatedDiagnosticConfiguration;
   private SDFHumanoidRobot simulatedRobot;
   private HumanoidReferenceFrames humanoidReferenceFrames;
   private DRCKinematicsBasedStateEstimator stateEstimator;

   public AutomatedDiagnosticSimulationFactory(DRCRobotModel robotModel)
   {
      this.robotModel = robotModel;
   }

   public void startSimulation()
   {
      SimulationConstructionSetParameters simulationParameters = new SimulationConstructionSetParameters(true, 16000);
      SimulationConstructionSet scs = new SimulationConstructionSet(simulatedRobot, simulationParameters);
      scs.setDT(robotModel.getSimulateDT(), 10);
      scs.setCameraPosition(scsCameraPosition.x, scsCameraPosition.y, scsCameraPosition.z);
      scs.setCameraFix(scsCameraFix.x, scsCameraFix.y, scsCameraFix.z);
      scs.startOnAThread();
   }

   public AutomatedDiagnosticConfiguration createDiagnosticController(boolean startWithRobotAlive)
   {
      simulatedRobot = robotModel.createSdfRobot(false);
      DiagnosticLoggerConfiguration.setupLogging(simulatedRobot.getYoTime(), getClass(), robotModel.getSimpleRobotName());

      if (robotInitialSetup == null)
         robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);
      robotInitialSetup.initializeRobot(simulatedRobot, robotModel.getJointMap());

      SDFFullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      DoubleYoVariable yoTime = simulatedRobot.getYoTime();
      double dt = robotModel.getEstimatorDT();

      StateEstimatorParameters stateEstimatorParameters = robotModel.getStateEstimatorParameters();
      
      if (diagnosticParameters == null)
         diagnosticParameters = new DiagnosticParameters(DiagnosticEnvironment.RUNTIME_CONTROLLER);

      DiagnosticSensorProcessingConfiguration sensorProcessingConfiguration = new DiagnosticSensorProcessingConfiguration(diagnosticParameters, stateEstimatorParameters);

      SensorOutputMapReadOnly sensorOutputMap = createStateEstimator(fullRobotModel, stateEstimatorParameters, sensorProcessingConfiguration);

      DiagnosticControllerToolbox diagnosticControllerToolbox = new DiagnosticControllerToolbox(fullRobotModel, sensorOutputMap, diagnosticParameters, walkingControllerParameters, yoTime, dt,
            sensorProcessingConfiguration, simulationRegistry);
      automatedDiagnosticAnalysisController = new AutomatedDiagnosticAnalysisController(diagnosticControllerToolbox, gainStream, setpointStream,
            simulationRegistry);
      automatedDiagnosticAnalysisController.setRobotIsAlive(startWithRobotAlive);
      automatedDiagnosticConfiguration = new AutomatedDiagnosticConfiguration(diagnosticControllerToolbox, automatedDiagnosticAnalysisController);

      outputWriter = new DRCSimulationOutputWriter(simulatedRobot);
      outputWriter.setFullRobotModel(fullRobotModel, null);

      int simulationTicksPerControlTick = (int) (robotModel.getEstimatorDT() / robotModel.getSimulateDT());
      simulatedRobot.setController(this, simulationTicksPerControlTick);

      return automatedDiagnosticConfiguration;
   }

   private SensorOutputMapReadOnly createStateEstimator(SDFFullHumanoidRobotModel fullRobotModel, StateEstimatorParameters stateEstimatorParameters,
         DiagnosticSensorProcessingConfiguration sensorProcessingConfiguration)
   {
      SixDoFJoint rootJoint = fullRobotModel.getRootJoint();
      IMUDefinition[] imuDefinitions = fullRobotModel.getIMUDefinitions();
      ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();
      ContactSensorHolder contactSensorHolder = null;
      RawJointSensorDataHolderMap rawJointSensorDataHolderMap = null;
      DesiredJointDataHolder estimatorDesiredJointDataHolder = null;

      ForceSensorDataHolder forceSensorDataHolderToUpdate = new ForceSensorDataHolder(Arrays.asList(forceSensorDefinitions));

      SimulatedSensorHolderAndReaderFromRobotFactory sensorReaderFactory = new SimulatedSensorHolderAndReaderFromRobotFactory(simulatedRobot,
            sensorProcessingConfiguration);
      sensorReaderFactory.build(rootJoint, imuDefinitions, forceSensorDefinitions, contactSensorHolder, rawJointSensorDataHolderMap,
            estimatorDesiredJointDataHolder, simulationRegistry);
      sensorReader = sensorReaderFactory.getSensorReader();

      FullInverseDynamicsStructure inverseDynamicsStructure = DRCControllerThread.createInverseDynamicsStructure(fullRobotModel);
      SensorOutputMapReadOnly sensorOutputMapReadOnly = sensorReader.getSensorOutputMapReadOnly();
      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      String[] imuSensorsToUseInStateEstimator = sensorInformation.getIMUSensorsToUseInStateEstimator();
      double gravitationalAcceleration = 9.81;
      double totalRobotWeight = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator()) * gravitationalAcceleration;

      humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      RobotContactPointParameters contactPointParameters = robotModel.getContactPointParameters();
      ContactableBodiesFactory contactableBodiesFactory = contactPointParameters.getContactableBodiesFactory();
      SideDependentList<ContactablePlaneBody> bipedFeet = contactableBodiesFactory.createFootContactableBodies(fullRobotModel, humanoidReferenceFrames);

      SideDependentList<FootSwitchInterface> footSwitches = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         String footForceSensorName = sensorInformation.getFeetForceSensorNames().get(robotSide);
         ForceSensorDataReadOnly footForceSensorForEstimator = forceSensorDataHolderToUpdate.getByName(footForceSensorName);
         String namePrefix = bipedFeet.get(robotSide).getName() + "StateEstimator";

         double footSwitchCoPThresholdFraction = stateEstimatorParameters.getFootSwitchCoPThresholdFraction();
         double contactThresholdForce = stateEstimatorParameters.getContactThresholdForce();

         WrenchBasedFootSwitch wrenchBasedFootSwitchForEstimator = new WrenchBasedFootSwitch(namePrefix, footForceSensorForEstimator,
               footSwitchCoPThresholdFraction, totalRobotWeight, bipedFeet.get(robotSide), null, contactThresholdForce, simulationRegistry);
         footSwitches.put(robotSide, wrenchBasedFootSwitchForEstimator);
      }

      stateEstimator = new DRCKinematicsBasedStateEstimator(inverseDynamicsStructure, stateEstimatorParameters, sensorOutputMapReadOnly,
            forceSensorDataHolderToUpdate, imuSensorsToUseInStateEstimator, gravitationalAcceleration, footSwitches, null, new RobotMotionStatusHolder(),
            bipedFeet, null);
      simulationRegistry.addChild(stateEstimator.getYoVariableRegistry());

      return sensorReader.getSensorOutputMapReadOnly();
   }

   public void setDiagnosticParameters(DiagnosticParameters diagnosticParameters)
   {
      this.diagnosticParameters = diagnosticParameters;
   }

   public void setRobotInitialSetup(DRCRobotInitialSetup<SDFHumanoidRobot> robotInitialSetup)
   {
      this.robotInitialSetup = robotInitialSetup;
   }

   public void setRobotInitialSetup(double groundHeight, double initialYaw)
   {
      robotInitialSetup = robotModel.getDefaultRobotInitialSetup(groundHeight, initialYaw);
   }

   public void setGainStream(InputStream gainStream)
   {
      this.gainStream = gainStream;
   }

   public void setSetpointStream(InputStream setpointStream)
   {
      this.setpointStream = setpointStream;
   }

   /**
    * Sets the initial SCS camera position.
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
      sensorReader.read();
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

      outputWriter.writeAfterController(0);
      long endTime = System.nanoTime();
      controllerTime.set(TimeTools.nanoSecondstoSeconds(endTime - startTime));
      averageControllerTime.update();
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
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
