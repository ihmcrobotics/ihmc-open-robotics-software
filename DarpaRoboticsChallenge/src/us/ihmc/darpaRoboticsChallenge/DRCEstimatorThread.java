package us.ihmc.darpaRoboticsChallenge;

import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.sensors.WrenchBasedFootSwitch;
import us.ihmc.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.darpaRoboticsChallenge.controllers.concurrent.ThreadDataSynchronizer;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotContactPointParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.darpaRoboticsChallenge.networking.dataProducers.JointConfigurationGatherer;
import us.ihmc.darpaRoboticsChallenge.sensors.RobotJointLimitWatcher;
import us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator.DRCKinematicsBasedStateEstimator;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.simulationconstructionset.robotController.ModularRobotController;
import us.ihmc.simulationconstructionset.robotController.MultiThreadedRobotControlElement;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorData;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.humanoidRobot.visualizer.RobotVisualizer;
import us.ihmc.yoUtilities.time.ExecutionTimer;

public class DRCEstimatorThread implements MultiThreadedRobotControlElement
{
   private final YoVariableRegistry estimatorRegistry = new YoVariableRegistry("DRCEstimatorThread");
   private final RobotVisualizer robotVisualizer;
   private final SDFFullRobotModel estimatorFullRobotModel;
   private final ForceSensorDataHolder forceSensorDataHolderForEstimator;
   private final ModularRobotController estimatorController;
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final DRCKinematicsBasedStateEstimator drcStateEstimator;

   private final ThreadDataSynchronizer threadDataSynchronizer;
   private final SensorReader sensorReader;

   private final LongYoVariable estimatorTime = new LongYoVariable("estimatorTime", estimatorRegistry);
   private final LongYoVariable estimatorTick = new LongYoVariable("estimatorTick", estimatorRegistry);
   private final BooleanYoVariable firstTick = new BooleanYoVariable("firstTick", estimatorRegistry);
   
   private final LongYoVariable startClockTime = new LongYoVariable("startTime", estimatorRegistry);
   private final ExecutionTimer estimatorTimer = new ExecutionTimer("estimatorTimer", 10.0, estimatorRegistry);
   
   private final LongYoVariable actualEstimatorDT = new LongYoVariable("actualEstimatorDT", estimatorRegistry);
   
   private final SensorOutputMapReadOnly sensorOutputMapReadOnly;
   
   private final DRCRobotSensorInformation sensorInformation;
   
   public DRCEstimatorThread(DRCRobotModel robotModel, SensorReaderFactory sensorReaderFactory, ThreadDataSynchronizer threadDataSynchronizer,
         GlobalDataProducer dataProducer, RobotVisualizer robotVisualizer, double gravity)
   {
      this.threadDataSynchronizer = threadDataSynchronizer;
      this.robotVisualizer = robotVisualizer;
      estimatorFullRobotModel = threadDataSynchronizer.getEstimatorFullRobotModel();
      forceSensorDataHolderForEstimator = threadDataSynchronizer.getEstimatorForceSensorDataHolder();
      
      sensorInformation = robotModel.getSensorInformation();
      DRCRobotContactPointParameters contactPointParamaters = robotModel.getContactPointParameters();
      StateEstimatorParameters stateEstimatorParameters = robotModel.getStateEstimatorParameters();

      sensorReaderFactory.build(estimatorFullRobotModel.getRootJoint(), estimatorFullRobotModel.getIMUDefinitions(), estimatorFullRobotModel.getForceSensorDefinitions(),
            forceSensorDataHolderForEstimator, threadDataSynchronizer.getEstimatorRawJointSensorDataHolderMap(), estimatorRegistry);
      sensorReader = sensorReaderFactory.getSensorReader();
      
      estimatorController = new ModularRobotController("EstimatorController");
      
      sensorOutputMapReadOnly = sensorReader.getSensorOutputMapReadOnly();
      if (sensorReaderFactory.useStateEstimator())
      {
         drcStateEstimator = createStateEstimator(estimatorFullRobotModel, robotModel, sensorOutputMapReadOnly, gravity, stateEstimatorParameters,
               contactPointParamaters, forceSensorDataHolderForEstimator, yoGraphicsListRegistry, estimatorRegistry);
         estimatorController.addRobotController(drcStateEstimator);
      }
      else
      {
         drcStateEstimator = null;
      }

      RobotJointLimitWatcher robotJointLimitWatcher = new RobotJointLimitWatcher(estimatorFullRobotModel.getOneDoFJoints());
      estimatorController.addRobotController(robotJointLimitWatcher);

      for(ForceSensorDefinition forceSensorDefinition:forceSensorDataHolderForEstimator.getForceSensorDefinitions())
      {
        ForceSensorToJointTorqueProjector footSensorToJointTorqueProjector = new ForceSensorToJointTorqueProjector(
              forceSensorDefinition.getSensorName(), 
              forceSensorDataHolderForEstimator.get(forceSensorDefinition), 
              forceSensorDefinition.getRigidBody());
        estimatorController.addRobotController(footSensorToJointTorqueProjector);
      }
            
      
      if (dataProducer != null)
      {
         ObjectCommunicator objectCommunicator = dataProducer.getObjectCommunicator();
         JointConfigurationGatherer jointConfigurationGathererAndProducer = new JointConfigurationGatherer(estimatorFullRobotModel, forceSensorDataHolderForEstimator);
         
         estimatorController.setRawOutputWriter(new DRCPoseCommunicator(estimatorFullRobotModel, jointConfigurationGathererAndProducer,
               objectCommunicator, sensorOutputMapReadOnly, sensorInformation));
      }
      
      firstTick.set(true);
      estimatorRegistry.addChild(estimatorController.getYoVariableRegistry());
      
      if(robotVisualizer != null)
      {
         robotVisualizer.setMainRegistry(estimatorRegistry, estimatorFullRobotModel, yoGraphicsListRegistry);
      }
   }
   
   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return estimatorRegistry;
   }

   @Override
   public String getName()
   {
      return estimatorRegistry.getName();
   }

   @Override
   public void read(long currentClockTime)
   {
      actualEstimatorDT.set(currentClockTime - startClockTime.getLongValue());
      startClockTime.set(currentClockTime);
      sensorReader.read();
      estimatorTime.set(sensorOutputMapReadOnly.getTimestamp());
   }

   @Override
   public void run()
   {
      if(firstTick.getBooleanValue())
      {
         estimatorController.initialize();
         firstTick.set(false);
      }
      
      estimatorTimer.startMeasurement();
      estimatorController.doControl();
      estimatorTimer.stopMeasurement();
   }

   @Override
   public void write(long timestamp)
   {
      long startTimestamp = estimatorTime.getLongValue();
      threadDataSynchronizer.publishEstimatorState(startTimestamp, estimatorTick.getLongValue(), startClockTime.getLongValue());
      if(robotVisualizer != null)
      {
         robotVisualizer.update(startTimestamp, estimatorRegistry);
      }
      estimatorTick.increment();
   }

   public static DRCKinematicsBasedStateEstimator createStateEstimator(SDFFullRobotModel estimatorFullRobotModel, DRCRobotModel drcRobotModel,
         SensorOutputMapReadOnly sensorOutputMapReadOnly, double gravity, StateEstimatorParameters stateEstimatorParameters,
         DRCRobotContactPointParameters contactPointParamaters, ForceSensorDataHolder forceSensorDataHolderForEstimator,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry)
   {
      DRCRobotJointMap jointMap = drcRobotModel.getJointMap();
      FullInverseDynamicsStructure inverseDynamicsStructure = DRCControllerThread.createInverseDynamicsStructure(estimatorFullRobotModel);

      ReferenceFrames estimatorReferenceFrames = new ReferenceFrames(estimatorFullRobotModel);
      ContactableBodiesFactory contactableBodiesFactory = jointMap.getContactPointParameters().getContactableBodiesFactory();
      SideDependentList<ContactablePlaneBody> bipedFeet = contactableBodiesFactory.createFootContactableBodies(estimatorFullRobotModel, estimatorReferenceFrames);

      double gravityMagnitude = Math.abs(gravity);
      double totalRobotWeight = TotalMassCalculator.computeSubTreeMass(estimatorFullRobotModel.getElevator()) * gravityMagnitude;

      SideDependentList<WrenchBasedFootSwitch> footSwitchesForEstimator = new SideDependentList<WrenchBasedFootSwitch>();
      for (RobotSide robotSide : RobotSide.values)
      {
         String footForceSensorName = drcRobotModel.getSensorInformation().getFeetForceSensorNames().get(robotSide);
         ForceSensorData footForceSensorForEstimator = forceSensorDataHolderForEstimator.getByName(footForceSensorName);
         String namePrefix = bipedFeet.get(robotSide).getName() + "StateEstimator";

         //         double footSwitchCoPThresholdFraction = 0.01;
         double footSwitchCoPThresholdFraction = stateEstimatorParameters.getFootSwitchCoPThresholdFraction();
         double contactTresholdForce = stateEstimatorParameters.getContactThresholdForce();
         WrenchBasedFootSwitch wrenchBasedFootSwitchForEstimator = new WrenchBasedFootSwitch(namePrefix, footForceSensorForEstimator,
               footSwitchCoPThresholdFraction, totalRobotWeight, bipedFeet.get(robotSide), null, contactTresholdForce, registry);
         footSwitchesForEstimator.put(robotSide, wrenchBasedFootSwitchForEstimator);
      }
      
      String[] imuSensorsToUseInStateEstimator = drcRobotModel.getSensorInformation().getIMUSensorsToUseInStateEstimator();
      
      // Create the sensor readers and state estimator here:
      DRCKinematicsBasedStateEstimator drcStateEstimator = new DRCKinematicsBasedStateEstimator(inverseDynamicsStructure, stateEstimatorParameters,
            sensorOutputMapReadOnly, imuSensorsToUseInStateEstimator, gravityMagnitude, footSwitchesForEstimator, bipedFeet, yoGraphicsListRegistry);

      return drcStateEstimator;
   }

   @Override
   public YoGraphicsListRegistry getDynamicGraphicObjectsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   @Override
   public long nextWakeupTime()
   {
      throw new RuntimeException("Estimator thread should not wake up based on clock");
   }

   public void initializeEstimatorToActual(Point3d initialCoMPosition, Quat4d initialEstimationLinkOrientation)
   {
      drcStateEstimator.initializeEstimatorToActual(initialCoMPosition, initialEstimationLinkOrientation);
   }
   
   
   public void setExternelPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      drcStateEstimator.setExternelPelvisCorrectorSubscriber(externalPelvisPoseSubscriber);
   }
   
   public  List<? extends IMUSensorReadOnly> getSimulatedIMUOutput()
   {
	   return sensorOutputMapReadOnly.getIMUProcessedOutputs();
   }
   
}
