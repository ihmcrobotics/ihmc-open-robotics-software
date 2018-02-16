package us.ihmc.avatar;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.ForceSensorToJointTorqueProjector;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.KinematicsBasedFootSwitch;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchAndContactSensorFusedFootSwitch;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitch;
import us.ihmc.communication.packets.ControllerCrashNotificationPacket.CrashLocation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.sensing.RequestWristForceSensorCalibrationPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorModePacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorModePacket.StateEstimatorMode;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.humanoidRobotics.communication.subscribers.RequestWristForceSensorCalibrationSubscriber;
import us.ihmc.humanoidRobotics.communication.subscribers.StateEstimatorModeSubscriber;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotDataLogger.RobotVisualizer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotController.ModularRobotController;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.ContactSensorHolder;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorData;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.communication.producers.DRCPoseCommunicator;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorData.JointConfigurationGatherer;
import us.ihmc.sensorProcessing.sensorProcessors.RobotJointLimitWatcher;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControlElement;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.DRCKinematicsBasedStateEstimator;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ForceSensorCalibrationModule;
import us.ihmc.util.PeriodicThreadScheduler;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoLong;

public class DRCEstimatorThread implements MultiThreadedRobotControlElement
{
   private static final boolean USE_FORCE_SENSOR_TO_JOINT_TORQUE_PROJECTOR = false;

   private final YoVariableRegistry estimatorRegistry = new YoVariableRegistry("DRCEstimatorThread");
   private final RobotVisualizer robotVisualizer;
   private final FullHumanoidRobotModel estimatorFullRobotModel;
   private final ForceSensorDataHolder forceSensorDataHolderForEstimator;
   private final CenterOfMassDataHolder centerOfMassDataHolderForEstimator;
   private final ContactSensorHolder contactSensorHolder;
   private final ModularRobotController estimatorController;
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final DRCKinematicsBasedStateEstimator drcStateEstimator;

   private final ThreadDataSynchronizerInterface threadDataSynchronizer;
   private final SensorReader sensorReader;

   private final YoLong estimatorTime = new YoLong("estimatorTime", estimatorRegistry);
   private final YoLong estimatorTick = new YoLong("estimatorTick", estimatorRegistry);
   private final YoBoolean firstTick = new YoBoolean("firstTick", estimatorRegistry);
   private final YoBoolean outputWriterInitialized = new YoBoolean("outputWriterInitialized", estimatorRegistry);
   private final YoBoolean controllerDataValid = new YoBoolean("controllerDataValid", estimatorRegistry);

   private final YoLong startClockTime = new YoLong("startTime", estimatorRegistry);
   private final ExecutionTimer estimatorTimer = new ExecutionTimer("estimatorTimer", 10.0, estimatorRegistry);

   private final YoLong actualEstimatorDT = new YoLong("actualEstimatorDT", estimatorRegistry);

   private final SensorOutputMapReadOnly sensorOutputMapReadOnly;
   private final SensorRawOutputMapReadOnly sensorRawOutputMapReadOnly;
   private final CenterOfPressureDataHolder centerOfPressureDataHolderFromController;
   private final RobotMotionStatusHolder robotMotionStatusFromController;
   private final DRCPoseCommunicator poseCommunicator;

   private final HumanoidGlobalDataProducer globalDataProducer;

   private final RigidBodyTransform rootToWorldTransform = new RigidBodyTransform();
   private final ReferenceFrame rootFrame;

   private final JointDesiredOutputWriter outputWriter;

   public DRCEstimatorThread(DRCRobotSensorInformation sensorInformation, RobotContactPointParameters contactPointParameters, WholeBodyControllerParameters wholeBodyControllerParameters,
         StateEstimatorParameters stateEstimatorParameters, SensorReaderFactory sensorReaderFactory, ThreadDataSynchronizerInterface threadDataSynchronizer,
         PeriodicThreadScheduler poseCommunicatorScheduler, HumanoidGlobalDataProducer dataProducer, JointDesiredOutputWriter outputWriter, RobotVisualizer robotVisualizer, double gravity)
   {
      this.threadDataSynchronizer = threadDataSynchronizer;
      this.robotVisualizer = robotVisualizer;
      this.globalDataProducer = dataProducer;
      estimatorFullRobotModel = threadDataSynchronizer.getEstimatorFullRobotModel();
      FloatingInverseDynamicsJoint rootJoint = estimatorFullRobotModel.getRootJoint();
      rootFrame = rootJoint.getFrameAfterJoint();

      forceSensorDataHolderForEstimator = threadDataSynchronizer.getEstimatorForceSensorDataHolder();
      centerOfMassDataHolderForEstimator = threadDataSynchronizer.getEstimatorCenterOfMassDataHolder();
      ContactSensorHolder estimatorContactSensorHolder = threadDataSynchronizer.getEstimatorContactSensorHolder();
      contactSensorHolder = estimatorContactSensorHolder;

      IMUDefinition[] imuDefinitions = estimatorFullRobotModel.getIMUDefinitions();
      ForceSensorDefinition[] forceSensorDefinitions = estimatorFullRobotModel.getForceSensorDefinitions();
      RawJointSensorDataHolderMap estimatorRawJointSensorDataHolderMap = threadDataSynchronizer.getEstimatorRawJointSensorDataHolderMap();
      JointDesiredOutputList estimatorDesiredJointDataHolder = threadDataSynchronizer.getEstimatorDesiredJointDataHolder();

      sensorReaderFactory.build(rootJoint, imuDefinitions, forceSensorDefinitions, estimatorContactSensorHolder, estimatorRawJointSensorDataHolderMap,
            estimatorDesiredJointDataHolder, estimatorRegistry);

      sensorReader = sensorReaderFactory.getSensorReader();

      estimatorController = new ModularRobotController("EstimatorController");

      centerOfPressureDataHolderFromController = threadDataSynchronizer.getEstimatorCenterOfPressureDataHolder();
      robotMotionStatusFromController = threadDataSynchronizer.getEstimatorRobotMotionStatusHolder();

      sensorOutputMapReadOnly = sensorReader.getSensorOutputMapReadOnly();
      sensorRawOutputMapReadOnly = sensorReader.getSensorRawOutputMapReadOnly();

      if (sensorReaderFactory.useStateEstimator())
      {
         drcStateEstimator = createStateEstimator(estimatorFullRobotModel, sensorInformation, sensorOutputMapReadOnly, gravity, stateEstimatorParameters,
               contactPointParameters, forceSensorDataHolderForEstimator,
               centerOfMassDataHolderForEstimator, contactSensorHolder,
               centerOfPressureDataHolderFromController,
               robotMotionStatusFromController, yoGraphicsListRegistry, estimatorRegistry);

         if (globalDataProducer != null)
         {
            StateEstimatorModeSubscriber stateEstimatorModeSubscriber = new StateEstimatorModeSubscriber();
            RequestWristForceSensorCalibrationSubscriber requestWristForceSensorCalibrationSubscriber = new RequestWristForceSensorCalibrationSubscriber();
            globalDataProducer.attachListener(StateEstimatorModePacket.class, stateEstimatorModeSubscriber);
            dataProducer.attachListener(RequestWristForceSensorCalibrationPacket.class, requestWristForceSensorCalibrationSubscriber);

            drcStateEstimator.setOperatingModeSubscriber(stateEstimatorModeSubscriber);
            drcStateEstimator.setRequestWristForceSensorCalibrationSubscriber(requestWristForceSensorCalibrationSubscriber);
         }

         estimatorController.addRobotController(drcStateEstimator);
      }
      else
      {
         drcStateEstimator = null;
      }

      RobotJointLimitWatcher robotJointLimitWatcher = new RobotJointLimitWatcher(estimatorFullRobotModel.getOneDoFJoints(), sensorRawOutputMapReadOnly);
      estimatorController.addRobotController(robotJointLimitWatcher);

      if (USE_FORCE_SENSOR_TO_JOINT_TORQUE_PROJECTOR)
      {
         for (ForceSensorDefinition forceSensorDefinition : forceSensorDataHolderForEstimator.getForceSensorDefinitions())
         {
            String sensorName = forceSensorDefinition.getSensorName();
            RigidBody sensorLink = forceSensorDefinition.getRigidBody();
            ForceSensorData forceSensorData = forceSensorDataHolderForEstimator.get(forceSensorDefinition);
            ForceSensorToJointTorqueProjector footSensorToJointTorqueProjector = new ForceSensorToJointTorqueProjector(sensorName, forceSensorData, sensorLink);
            estimatorController.addRobotController(footSensorToJointTorqueProjector);
         }
      }

      if (dataProducer != null)
      {
         ForceSensorDataHolderReadOnly forceSensorDataHolderToSend;
         if (drcStateEstimator != null && drcStateEstimator.getForceSensorOutputWithGravityCancelled() != null)
            forceSensorDataHolderToSend = drcStateEstimator.getForceSensorOutputWithGravityCancelled();
         else
            forceSensorDataHolderToSend = forceSensorDataHolderForEstimator;

         JointConfigurationGatherer jointConfigurationGathererAndProducer = new JointConfigurationGatherer(estimatorFullRobotModel,
               forceSensorDataHolderToSend);

         poseCommunicator = new DRCPoseCommunicator(estimatorFullRobotModel, jointConfigurationGathererAndProducer, sensorReader, dataProducer,
               sensorOutputMapReadOnly, sensorRawOutputMapReadOnly, robotMotionStatusFromController, sensorInformation, poseCommunicatorScheduler, new IHMCCommunicationKryoNetClassList());
         estimatorController.setRawOutputWriter(poseCommunicator);
      }
      else
      {
         poseCommunicator = null;
      }


      firstTick.set(true);
      outputWriterInitialized.set(false);
      controllerDataValid.set(false);

      estimatorRegistry.addChild(estimatorController.getYoVariableRegistry());

      this.outputWriter = outputWriter;
      if(this.outputWriter != null)
      {
         this.outputWriter.setForceSensorDataHolder(forceSensorDataHolderForEstimator);
         this.outputWriter.setJointDesiredOutputList(estimatorDesiredJointDataHolder);
         if(this.outputWriter.getYoVariableRegistry() != null)
         {
            estimatorRegistry.addChild(this.outputWriter.getYoVariableRegistry());
         }

      }

      ParameterLoaderHelper.loadParameters(this, wholeBodyControllerParameters, estimatorRegistry);

      if (robotVisualizer != null)
      {
         robotVisualizer.setMainRegistry(estimatorRegistry, estimatorFullRobotModel.getElevator(), yoGraphicsListRegistry);
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
      try
      {
         actualEstimatorDT.set(currentClockTime - startClockTime.getLongValue());
         startClockTime.set(currentClockTime);

         controllerDataValid.set(threadDataSynchronizer.receiveControllerDataForEstimator());

         if(outputWriter != null)
         {
            if(controllerDataValid.getBooleanValue())
            {
               if(!outputWriterInitialized.getBooleanValue())
               {
                  outputWriter.initialize();
                  outputWriterInitialized.set(true);
               }

               outputWriter.writeBefore(currentClockTime);
            }
         }

         sensorReader.read();

         estimatorTime.set(sensorOutputMapReadOnly.getTimestamp());

         if(globalDataProducer != null)
         {
            globalDataProducer.setRobotTime(estimatorTime.getLongValue());
         }
      }
      catch (Throwable e)
      {
         if(globalDataProducer != null)
         {
            globalDataProducer.notifyControllerCrash(CrashLocation.ESTIMATOR_READ, e.getMessage());
         }
         throw new RuntimeException(e);
      }
   }

   @Override
   public void run()
   {
      try
      {
         if (firstTick.getBooleanValue())
         {
            estimatorController.initialize();
            firstTick.set(false);
         }

         estimatorTimer.startMeasurement();
         estimatorController.doControl();
         estimatorTimer.stopMeasurement();
      }
      catch (Throwable e)
      {
         if (globalDataProducer != null)
         {
            globalDataProducer.notifyControllerCrash(CrashLocation.ESTIMATOR_RUN, e.getMessage());
         }
         throw new RuntimeException(e);
      }
   }

   @Override
   public void write(long timestamp)
   {
      try
      {
         if(outputWriter != null)
         {
            if(controllerDataValid.getBooleanValue())
            {
               outputWriter.writeAfter();
            }
         }


         long startTimestamp = estimatorTime.getLongValue();
         threadDataSynchronizer.publishEstimatorState(startTimestamp, estimatorTick.getLongValue(), startClockTime.getLongValue());
         if (robotVisualizer != null)
         {
            robotVisualizer.update(startTimestamp);
         }
         estimatorTick.increment();

         rootFrame.getTransformToDesiredFrame(rootToWorldTransform, ReferenceFrame.getWorldFrame());
         yoGraphicsListRegistry.setControllerTransformToWorld(rootToWorldTransform);
      }
      catch (Throwable e)
      {
         if (globalDataProducer != null)
         {
            globalDataProducer.notifyControllerCrash(CrashLocation.ESTIMATOR_WRITE, e.getMessage());
         }
         throw new RuntimeException(e);
      }
   }

   private DRCKinematicsBasedStateEstimator createStateEstimator(FullHumanoidRobotModel estimatorFullRobotModel, DRCRobotSensorInformation sensorInformation,
         SensorOutputMapReadOnly sensorOutputMapReadOnly, double gravity, StateEstimatorParameters stateEstimatorParameters,
         RobotContactPointParameters contactPointParamaters, ForceSensorDataHolder estimatorForceSensorDataHolderToUpdate,
         CenterOfMassDataHolder estimatorCenterOfMassDataHolderToUpdate,
         ContactSensorHolder contactSensorHolder,
         CenterOfPressureDataHolder centerOfPressureDataHolderFromController, RobotMotionStatusHolder robotMotionStatusFromController,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry)
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = DRCControllerThread.createInverseDynamicsStructure(estimatorFullRobotModel);

      HumanoidReferenceFrames estimatorReferenceFrames = new HumanoidReferenceFrames(estimatorFullRobotModel);
      ContactableBodiesFactory contactableBodiesFactory = contactPointParamaters.getContactableBodiesFactory();
      SideDependentList<? extends ContactablePlaneBody> bipedFeet = contactableBodiesFactory.createFootContactableBodies(estimatorFullRobotModel,
            estimatorReferenceFrames);

      double gravityMagnitude = Math.abs(gravity);
      double totalRobotWeight = TotalMassCalculator.computeSubTreeMass(estimatorFullRobotModel.getElevator()) * gravityMagnitude;

//      SideDependentList<FootSwitchInterface> footSwitchesForEstimator = new SideDependentList<>();

      Map<RigidBody, FootSwitchInterface> footSwitchMap = new LinkedHashMap<RigidBody, FootSwitchInterface>();
      Map<RigidBody, ContactablePlaneBody> bipedFeetMap = new LinkedHashMap<RigidBody, ContactablePlaneBody>();

      for (RobotSide robotSide : RobotSide.values)
      {
         String footForceSensorName = sensorInformation.getFeetForceSensorNames().get(robotSide);
         String footContactSensorName = sensorInformation.getFeetContactSensorNames().get(robotSide);
         ForceSensorDataReadOnly footForceSensorForEstimator = estimatorForceSensorDataHolderToUpdate.getByName(footForceSensorName);
         String namePrefix = bipedFeet.get(robotSide).getName() + "StateEstimator";

         //         double footSwitchCoPThresholdFraction = 0.01;
         double footSwitchCoPThresholdFraction = stateEstimatorParameters.getFootSwitchCoPThresholdFraction();
         double contactThresholdForce = stateEstimatorParameters.getContactThresholdForce();

         RigidBody foot = bipedFeet.get(robotSide).getRigidBody();
         bipedFeetMap.put(foot, bipedFeet.get(robotSide));

         switch (stateEstimatorParameters.getFootSwitchType())
         {
            case KinematicBased:

               KinematicsBasedFootSwitch footSwitch = new KinematicsBasedFootSwitch(namePrefix, bipedFeet,
                     stateEstimatorParameters.getContactThresholdHeight(), totalRobotWeight, robotSide, registry);
               footSwitchMap.put(foot, footSwitch);
               break;
            case WrenchBased:
               WrenchBasedFootSwitch wrenchBasedFootSwitchForEstimator = new WrenchBasedFootSwitch(namePrefix, footForceSensorForEstimator,
                     footSwitchCoPThresholdFraction, totalRobotWeight, bipedFeet.get(robotSide), null, contactThresholdForce, registry);
               footSwitchMap.put(foot, wrenchBasedFootSwitchForEstimator);
               break;

            case WrenchAndContactSensorFused:
               WrenchAndContactSensorFusedFootSwitch wrenchAndContactSensorBasedFootswitch = new WrenchAndContactSensorFusedFootSwitch(namePrefix,
                     footForceSensorForEstimator, contactSensorHolder.getByName(footContactSensorName), footSwitchCoPThresholdFraction, totalRobotWeight,
                     bipedFeet.get(robotSide), null, contactThresholdForce, registry);
               footSwitchMap.put(foot, wrenchAndContactSensorBasedFootswitch);
               break;
            default:
               throw new Error("unknown foot switch type");
         }

      }

      String[] imuSensorsToUseInStateEstimator = sensorInformation.getIMUSensorsToUseInStateEstimator();

      // Create the sensor readers and state estimator here:
      DRCKinematicsBasedStateEstimator drcStateEstimator = new DRCKinematicsBasedStateEstimator(inverseDynamicsStructure, stateEstimatorParameters,
            sensorOutputMapReadOnly, estimatorForceSensorDataHolderToUpdate, estimatorCenterOfMassDataHolderToUpdate,
            imuSensorsToUseInStateEstimator, gravityMagnitude, footSwitchMap,
            centerOfPressureDataHolderFromController, robotMotionStatusFromController, bipedFeetMap, yoGraphicsListRegistry);

      return drcStateEstimator;
   }

   @Override
   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   public ForceSensorCalibrationModule getForceSensorCalibrationModule()
   {
      return drcStateEstimator.getForceSensorCalibrationModule();
   }

   @Override
   public long nextWakeupTime()
   {
      throw new RuntimeException("Estimator thread should not wake up based on clock");
   }

   public void initializeEstimatorToActual(Point3D initialCoMPosition, Quaternion initialEstimationLinkOrientation)
   {
      if (drcStateEstimator != null)
         drcStateEstimator.initializeEstimatorToActual(initialCoMPosition, initialEstimationLinkOrientation);
   }

   public void initializeEstimatorToActual(Point3D initialCoMPosition, Quaternion initialEstimationLinkOrientation, Vector3D initialCoMVelocity, Vector3D initialAngularVelocity)
   {
      if (drcStateEstimator != null)
         drcStateEstimator.initializeEstimatorToActual(initialCoMPosition, initialCoMVelocity, initialEstimationLinkOrientation, initialAngularVelocity);
   }

   public void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      if (drcStateEstimator != null)
         drcStateEstimator.setExternalPelvisCorrectorSubscriber(externalPelvisPoseSubscriber);
   }

   public void requestStateEstimatorMode(StateEstimatorMode stateEstimatorMode)
   {
      if (drcStateEstimator != null)
         drcStateEstimator.requestStateEstimatorMode(stateEstimatorMode);
   }

   public List<? extends IMUSensorReadOnly> getSimulatedIMUOutput()
   {
      return sensorOutputMapReadOnly.getIMUProcessedOutputs();
   }

   public void dispose()
   {
      if (poseCommunicator != null)
      {
         poseCommunicator.stop();
      }
   }

   /**
    * used primarily for unit tests, but could be useful.
    */
   public void addRobotController(RobotController controller)
   {
      estimatorController.addRobotController(controller);
   }
}
