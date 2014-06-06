package us.ihmc.darpaRoboticsChallenge;

import java.util.Arrays;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.referenceFrames.ReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.WrenchBasedFootSwitch;
import us.ihmc.commonWalkingControlModules.visualizer.RobotVisualizer;
import us.ihmc.darpaRoboticsChallenge.controllers.concurrent.ThreadDataSynchronizer;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotContactPointParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.darpaRoboticsChallenge.networking.dataProducers.JointConfigurationGatherer;
import us.ihmc.darpaRoboticsChallenge.sensors.RobotJointLimitWatcher;
import us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator.DRCKinematicsBasedStateEstimator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensors.ForceSensorData;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.utilities.ForceSensorDefinition;
import us.ihmc.utilities.IMUDefinition;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.AtomicSettableTimestampProvider;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.LongYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.ModularRobotController;
import com.yobotics.simulationconstructionset.robotController.MultiThreadedRobotControlElement;
import com.yobotics.simulationconstructionset.time.ExecutionTimer;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class DRCEstimatorThread implements MultiThreadedRobotControlElement
{
   private final YoVariableRegistry estimatorRegistry = new YoVariableRegistry("DRCEstimatorThread");
   private final RobotVisualizer robotVisualizer;
   private final SDFFullRobotModel estimatorFullRobotModel;
   private final ForceSensorDataHolder forceSensorDataHolderForEstimator;
   private final ModularRobotController estimatorController;
   private final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
   private final DRCKinematicsBasedStateEstimator drcStateEstimator;

   private final ThreadDataSynchronizer threadDataSynchronizer;
   private final SensorReader sensorReader;

   private final DoubleYoVariable estimatorTime = new DoubleYoVariable("estimatorTime", estimatorRegistry);
   private final LongYoVariable estimatorTick = new LongYoVariable("estimatorTick", estimatorRegistry);
   private final BooleanYoVariable firstTick = new BooleanYoVariable("firstTick", estimatorRegistry);
   
   private final LongYoVariable startClockTime = new LongYoVariable("startTime", estimatorRegistry);
   private final ExecutionTimer estimatorTimer = new ExecutionTimer("estimatorTimer", 10.0, estimatorRegistry);
   
   private final LongYoVariable actualEstimatorDT = new LongYoVariable("actualEstimatorDT", estimatorRegistry);
   
   private final AtomicSettableTimestampProvider timestampProvider = new AtomicSettableTimestampProvider();

   public DRCEstimatorThread(DRCRobotModel robotModel, SensorReaderFactory sensorReaderFactory, ThreadDataSynchronizer threadDataSynchronizer,
         GlobalDataProducer dataProducer, RobotVisualizer robotVisualizer, double estimatorDT, double gravity)
   {
      this.threadDataSynchronizer = threadDataSynchronizer;
      this.robotVisualizer = robotVisualizer;
      estimatorFullRobotModel = threadDataSynchronizer.getEstimatorFullRobotModel();
      forceSensorDataHolderForEstimator = threadDataSynchronizer.getEstimatorForceSensorDataHolder();
      
      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      DRCRobotContactPointParameters contactPointParamaters = robotModel.getContactPointParamaters(false, false);
      StateEstimatorParameters stateEstimatorParameters = robotModel.getStateEstimatorParameters();

      final List<String> imuSensorsToUse = Arrays.asList(sensorInformation.getIMUSensorsToUse());
      IMUDefinition[] imuDefinitions = new IMUDefinition[imuSensorsToUse.size()];
      int index = 0;
      for (IMUDefinition imuDefinition : estimatorFullRobotModel.getIMUDefinitions())
      {
         if (imuSensorsToUse.contains(imuDefinition.getName()))
         {
            imuDefinitions[index++] = imuDefinition;
         }
      }

      sensorReaderFactory.build(estimatorFullRobotModel.getRootJoint(), imuDefinitions, estimatorFullRobotModel.getForceSensorDefinitions(),
            forceSensorDataHolderForEstimator, threadDataSynchronizer.getEstimatorRawJointSensorDataHolderMap(), estimatorRegistry);
      sensorReader = sensorReaderFactory.getSensorReader();
      
      estimatorController = new ModularRobotController("EstimatorController");
      if (sensorReaderFactory.useStateEstimator())
      {
         SensorOutputMapReadOnly sensorOutputMapReadOnly = sensorReader.getSensorOutputMapReadOnly();
         drcStateEstimator = createStateEstimator(estimatorFullRobotModel, robotModel, sensorOutputMapReadOnly, gravity, stateEstimatorParameters,
               contactPointParamaters, forceSensorDataHolderForEstimator, dynamicGraphicObjectsListRegistry, estimatorRegistry);
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
         JointConfigurationGatherer jointConfigurationGathererAndProducer = new JointConfigurationGatherer(estimatorFullRobotModel, estimatorRegistry, robotModel.getJointMap());

         estimatorController.setRawOutputWriter(new DRCPoseCommunicator(estimatorFullRobotModel, jointConfigurationGathererAndProducer,
                 dataProducer.getObjectCommunicator(), timestampProvider, robotModel));
      }
      
      firstTick.set(true);
      estimatorRegistry.addChild(estimatorController.getYoVariableRegistry());
      
      if(robotVisualizer != null)
      {
         robotVisualizer.setMainRegistry(estimatorRegistry, estimatorFullRobotModel, dynamicGraphicObjectsListRegistry);
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
   public void read(double time, long currentClockTime, long sensorTime)
   {
      actualEstimatorDT.set(currentClockTime - startClockTime.getLongValue());
      estimatorTime.set(time);
      startClockTime.set(currentClockTime);
      sensorReader.read();
      timestampProvider.set(sensorTime);
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
      long startTimestamp = TimeTools.secondsToNanoSeconds(estimatorTime.getDoubleValue());
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
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry registry)
   {
      DRCRobotPhysicalProperties physicalProperties = drcRobotModel.getPhysicalProperties();
      FullInverseDynamicsStructure inverseDynamicsStructure = DRCControllerThread.createInverseDynamicsStructure(estimatorFullRobotModel);

      SideDependentList<ContactablePlaneBody> bipedFeet = new SideDependentList<ContactablePlaneBody>();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody footBody = estimatorFullRobotModel.getFoot(robotSide);
         ReferenceFrames estimatorReferenceFrames = new ReferenceFrames(estimatorFullRobotModel, drcRobotModel.getJointMap(),
               physicalProperties.getAnkleHeight());
         ReferenceFrame soleFrame = estimatorReferenceFrames.getSoleFrame(robotSide);
         ListOfPointsContactablePlaneBody foot = new ListOfPointsContactablePlaneBody(footBody, soleFrame, contactPointParamaters
               .getFootGroundContactPointsInSoleFrameForController().get(robotSide));

         bipedFeet.put(robotSide, foot);
      }

      double gravityMagnitude = Math.abs(gravity);
      double totalRobotWeight = TotalMassCalculator.computeSubTreeMass(estimatorFullRobotModel.getElevator()) * gravityMagnitude;

      SideDependentList<WrenchBasedFootSwitch> footSwitchesForEstimator = new SideDependentList<WrenchBasedFootSwitch>();
      for (RobotSide robotSide : RobotSide.values)
      {
         String footForceSensorName = drcRobotModel.getSensorInformation().getFeetForceSensorNames().get(robotSide);
         ForceSensorData footForceSensorForEstimator = forceSensorDataHolderForEstimator.getByName(footForceSensorName);
         String namePrefix = bipedFeet.get(robotSide).getName() + "StateEstimator";

         //         double footSwitchCoPThresholdFraction = 0.01;
         WalkingControllerParameters walkingControllerParameters = drcRobotModel.getWalkingControlParameters();
         double footSwitchCoPThresholdFraction = walkingControllerParameters.getFootSwitchCoPThresholdFraction();
         double contactTresholdForce = DRCConfigParameters.contactTresholdForceForGazebo;
         WrenchBasedFootSwitch wrenchBasedFootSwitchForEstimator = new WrenchBasedFootSwitch(namePrefix, footForceSensorForEstimator,
               footSwitchCoPThresholdFraction, totalRobotWeight, bipedFeet.get(robotSide), null, contactTresholdForce, registry);
         footSwitchesForEstimator.put(robotSide, wrenchBasedFootSwitchForEstimator);
      }

      // Create the sensor readers and state estimator here:
      DRCKinematicsBasedStateEstimator drcStateEstimator = new DRCKinematicsBasedStateEstimator(inverseDynamicsStructure, stateEstimatorParameters,
            sensorOutputMapReadOnly, gravityMagnitude, footSwitchesForEstimator, bipedFeet, dynamicGraphicObjectsListRegistry);

      return drcStateEstimator;
   }

   public DRCKinematicsBasedStateEstimator getDRCStateEstimator()
   {
      return drcStateEstimator;
   }

   @Override
   public DynamicGraphicObjectsListRegistry getDynamicGraphicObjectsListRegistry()
   {
      return dynamicGraphicObjectsListRegistry;
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

}
