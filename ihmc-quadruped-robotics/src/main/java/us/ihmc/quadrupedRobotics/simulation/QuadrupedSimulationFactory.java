package us.ihmc.quadrupedRobotics.simulation;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.CommunicationOptions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.quadrupedRobotics.communication.QuadrupedGlobalDataProducer;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerEnum;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerManager;
import us.ihmc.quadrupedRobotics.controller.QuadrupedSimulationController;
import us.ihmc.quadrupedRobotics.controller.states.QuadrupedPositionBasedCrawlControllerParameters;
import us.ihmc.quadrupedRobotics.estimator.SimulatedQuadrupedFootSwitchFactory;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.estimator.sensorProcessing.simulatedSensors.SDFQuadrupedPerfectSimulatedSensor;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedSensorInformation;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedStateEstimatorFactory;
import us.ihmc.quadrupedRobotics.factories.QuadrupedRobotControllerFactory;
import us.ihmc.quadrupedRobotics.mechanics.inverseKinematics.QuadrupedInverseKinematicsCalculators;
import us.ihmc.quadrupedRobotics.mechanics.inverseKinematics.QuadrupedLegInverseKinematicsCalculator;
import us.ihmc.quadrupedRobotics.model.*;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.OutputWriter;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.sensors.ContactSensorHolder;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.communication.producers.DRCPoseCommunicator;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.sensorData.JointConfigurationGatherer;
import us.ihmc.sensorProcessing.sensorProcessors.SensorTimestampHolder;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedSensorHolderAndReaderFromRobotFactory;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationToolkit.controllers.SpringJointOutputWriter;
import us.ihmc.simulationToolkit.parameters.SimulatedElasticityParameters;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.*;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.DRCKinematicsBasedStateEstimator;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.util.PeriodicThreadScheduler;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;

import java.io.IOException;
import java.io.InputStream;
import java.net.BindException;
import java.util.ArrayList;
import java.util.List;

public class QuadrupedSimulationFactory
{
   private final RequiredFactoryField<FullQuadrupedRobotModel> fullRobotModel = new RequiredFactoryField<>("fullRobotModel");
   private final RequiredFactoryField<ControllerCoreOptimizationSettings> controllerCoreOptimizationSettings = new RequiredFactoryField<>(
         "controllerCoreOptimizationSettings");
   private final RequiredFactoryField<QuadrupedPhysicalProperties> physicalProperties = new RequiredFactoryField<>("physicalProperties");
   private final RequiredFactoryField<QuadrupedControlMode> controlMode = new RequiredFactoryField<>("controlMode");
   private final RequiredFactoryField<FloatingRootJointRobot> sdfRobot = new RequiredFactoryField<>("sdfRobot");
   private final RequiredFactoryField<Double> simulationDT = new RequiredFactoryField<>("simulationDT");
   private final RequiredFactoryField<Double> controlDT = new RequiredFactoryField<>("controlDT");
   private final RequiredFactoryField<Double> gravity = new RequiredFactoryField<>("gravity");
   private final RequiredFactoryField<Integer> recordFrequency = new RequiredFactoryField<>("recordFrequency");
   private final RequiredFactoryField<Boolean> useTrackAndDolly = new RequiredFactoryField<>("useTrackAndDolly");
   private final RequiredFactoryField<Boolean> showPlotter = new RequiredFactoryField<>("showPlotter");
   private final RequiredFactoryField<QuadrupedModelFactory> modelFactory = new RequiredFactoryField<>("modelFactory");
   private final RequiredFactoryField<SimulationConstructionSetParameters> scsParameters = new RequiredFactoryField<>("scsParameters");
   private final RequiredFactoryField<GroundContactParameters> groundContactParameters = new RequiredFactoryField<>("groundContactParameters");
   private final RequiredFactoryField<QuadrupedInitialPositionParameters> initialPositionParameters = new RequiredFactoryField<>("initialPositionParameters");
   private final RequiredFactoryField<QuadrupedInitialOffsetAndYaw> initialOffset = new RequiredFactoryField<>("initialOffset");
   private final RequiredFactoryField<OutputWriter> outputWriter = new RequiredFactoryField<>("outputWriter");
   private final RequiredFactoryField<Boolean> useNetworking = new RequiredFactoryField<>("useNetworking");
   private final RequiredFactoryField<NetClassList> netClassList = new RequiredFactoryField<>("netClassList");
   private final RequiredFactoryField<SensorTimestampHolder> timestampProvider = new RequiredFactoryField<>("timestampProvider");
   private final RequiredFactoryField<Boolean> useStateEstimator = new RequiredFactoryField<>("useStateEstimator");
   private final RequiredFactoryField<QuadrupedSensorInformation> sensorInformation = new RequiredFactoryField<>("sensorInformation");
   private final RequiredFactoryField<StateEstimatorParameters> stateEstimatorParameters = new RequiredFactoryField<>("stateEstimatorParameters");
   private final RequiredFactoryField<QuadrupedReferenceFrames> referenceFrames = new RequiredFactoryField<>("referenceFrames");
   private final RequiredFactoryField<QuadrupedPositionBasedCrawlControllerParameters> positionBasedCrawlControllerParameters = new RequiredFactoryField<>(
         "positionBasedCrawlControllerParameters");
   private final RequiredFactoryField<JointDesiredOutputList> jointDesiredOutputList = new RequiredFactoryField<>("jointDesiredOutputList");

   private final OptionalFactoryField<SimulatedElasticityParameters> simulatedElasticityParameters = new OptionalFactoryField<>(
         "simulatedElasticityParameters");
   private final OptionalFactoryField<QuadrupedGroundContactModelType> groundContactModelType = new OptionalFactoryField<>("groundContactModelType");
   private final OptionalFactoryField<QuadrupedRobotControllerFactory> headControllerFactory = new OptionalFactoryField<>("headControllerFactory");
   private final OptionalFactoryField<GroundProfile3D> providedGroundProfile3D = new OptionalFactoryField<>("providedGroundProfile3D");
   private final OptionalFactoryField<TerrainObject3D> providedTerrainObject3D = new OptionalFactoryField<>("providedTerrainObject3D");
   private final OptionalFactoryField<Boolean> usePushRobotController = new OptionalFactoryField<>("usePushRobotController");
   private final OptionalFactoryField<FootSwitchType> footSwitchType = new OptionalFactoryField<>("footSwitchType");
   private final OptionalFactoryField<Integer> scsBufferSize = new OptionalFactoryField<>("scsBufferSize");
   private final OptionalFactoryField<QuadrupedControllerEnum> initialForceControlState = new OptionalFactoryField<>("initialForceControlState");
   private final OptionalFactoryField<Boolean> useLocalCommunicator = new OptionalFactoryField<>("useLocalCommunicator");

   // TO CONSTRUCT
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistryForDetachedOverhead;
   private SensorReader sensorReader;
   private QuadrantDependentList<ContactablePlaneBody> contactableFeet;
   private List<ContactablePlaneBody> contactablePlaneBodies;
   private QuadrantDependentList<FootSwitchInterface> footSwitches;
   private DRCKinematicsBasedStateEstimator stateEstimator;
   private PacketCommunicator packetCommunicator;
   private RealtimeRos2Node realtimeRos2Node;
   private GlobalDataProducer globalDataProducer;
   private RobotController headController;
   private QuadrupedControllerManager controllerManager;
   private DRCPoseCommunicator poseCommunicator;
   private GroundProfile3D groundProfile3D;
   private LinearGroundContactModel groundContactModel;
   private QuadrupedSimulationController simulationController;
   private QuadrupedLegInverseKinematicsCalculator legInverseKinematicsCalculator;

   /**
    * The PacketCommunicator used as input of the controller is either equal to the output PacketCommunicator of the network processor or the behavior module if any.
    * It is bidirectional meaning that it carries commands to be executed by the controller and that the controller is able to send feedback the other way to whoever is listening to the PacketCommunicator.
    */
   private PacketCommunicator controllerPacketCommunicator;

   // CREATION

   private void setupYoRegistries()
   {
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(true);
      yoGraphicsListRegistryForDetachedOverhead = new YoGraphicsListRegistry();
   }

   private void createPushRobotController()
   {
      if (usePushRobotController.get())
      {
         FloatingRootJointRobot pushableRobot = sdfRobot.get();
         String rootJointName = pushableRobot.getRootJoint().getName();

         PushRobotController bodyPushRobotController = new PushRobotController(pushableRobot, rootJointName, new Vector3D(0.0, -0.00057633, 0.0383928));
         yoGraphicsListRegistry.registerYoGraphic("PushRobotControllers", bodyPushRobotController.getForceVisualizer());

         for (QuadrupedJointName quadrupedJointName : modelFactory.get().getQuadrupedJointNames())
         {
            String jointName = modelFactory.get().getSDFNameForJointName(quadrupedJointName);
            PushRobotController jointPushRobotController = new PushRobotController(sdfRobot.get(), jointName, new Vector3D(0.0, 0.0, 0.0));
            yoGraphicsListRegistry.registerYoGraphic("PushRobotControllers", jointPushRobotController.getForceVisualizer());
         }
      }
   }

   private void createSensorReader()
   {
      if (useStateEstimator.get())
      {

         FloatingInverseDynamicsJoint rootInverseDynamicsJoint = fullRobotModel.get().getRootJoint();
         IMUDefinition[] imuDefinitions = fullRobotModel.get().getIMUDefinitions();
         ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.get().getForceSensorDefinitions();
         ContactSensorHolder contactSensorHolder = null;
         RawJointSensorDataHolderMap rawJointSensorDataHolderMap = null;
         JointDesiredOutputList estimatorDesiredJointDataHolder = null;

         SimulatedSensorHolderAndReaderFromRobotFactory sensorReaderFactory;
         sensorReaderFactory = new SimulatedSensorHolderAndReaderFromRobotFactory(sdfRobot.get(), stateEstimatorParameters.get());
         sensorReaderFactory.build(rootInverseDynamicsJoint, imuDefinitions, forceSensorDefinitions, contactSensorHolder, rawJointSensorDataHolderMap,
                                   estimatorDesiredJointDataHolder, sdfRobot.get().getRobotsYoVariableRegistry());

         sensorReader = sensorReaderFactory.getSensorReader();
      }
      else
      {
         sensorReader = new SDFQuadrupedPerfectSimulatedSensor(sdfRobot.get(), fullRobotModel.get(), referenceFrames.get());
      }
   }

   private void createContactableFeet()
   {
      ContactableBodiesFactory<RobotQuadrant> footContactableBodiesFactory = new ContactableBodiesFactory<>();
      footContactableBodiesFactory.setFullRobotModel(fullRobotModel.get());
      footContactableBodiesFactory.setFootContactPoints(physicalProperties.get().getFeetGroundContactPoints());
      footContactableBodiesFactory.setReferenceFrames(referenceFrames.get());
      contactableFeet = new QuadrantDependentList<>(footContactableBodiesFactory.createFootContactablePlaneBodies());
      footContactableBodiesFactory.disposeFactory();
   }

   private void createContactablePlaneBodies()
   {
      contactablePlaneBodies = new ArrayList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         contactablePlaneBodies.add(contactableFeet.get(robotQuadrant));
   }

   private void createFootSwitches()
   {
      SimulatedQuadrupedFootSwitchFactory footSwitchFactory = new SimulatedQuadrupedFootSwitchFactory();
      footSwitchFactory.setFootContactableBodies(contactableFeet);
      footSwitchFactory.setFullRobotModel(fullRobotModel.get());
      footSwitchFactory.setGravity(gravity.get());
      footSwitchFactory.setSimulatedRobot(sdfRobot.get());
      footSwitchFactory.setYoVariableRegistry(sdfRobot.get().getRobotsYoVariableRegistry());
      footSwitchFactory.setFootSwitchType(footSwitchType.get());
      footSwitches = footSwitchFactory.createFootSwitches();
   }

   private void createStateEstimator()
   {
      if (useStateEstimator.get())
      {
         QuadrupedStateEstimatorFactory stateEstimatorFactory = new QuadrupedStateEstimatorFactory();
         stateEstimatorFactory.setEstimatorDT(controlDT.get());
         stateEstimatorFactory.setFootContactableBodies(contactableFeet);
         stateEstimatorFactory.setFootSwitches(footSwitches);
         stateEstimatorFactory.setFullRobotModel(fullRobotModel.get());
         stateEstimatorFactory.setGravity(gravity.get());
         stateEstimatorFactory.setSensorInformation(sensorInformation.get());
         stateEstimatorFactory.setSensorOutputMapReadOnly(sensorReader.getSensorOutputMapReadOnly());
         stateEstimatorFactory.setStateEstimatorParameters(stateEstimatorParameters.get());
         stateEstimatorFactory.setYoGraphicsListRegistry(yoGraphicsListRegistry);
         stateEstimatorFactory.setYoVariableRegistry(sdfRobot.get().getRobotsYoVariableRegistry());
         stateEstimator = stateEstimatorFactory.createStateEstimator();
      }
      else
      {
         stateEstimator = null;
      }
   }

   private void createPacketCommunicator() throws IOException
   {
      if (useNetworking.get())
      {
         try
         {
            if (useLocalCommunicator.get())
            {
               if (CommunicationOptions.USE_KRYO)
                  packetCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT, netClassList.get());
               if (CommunicationOptions.USE_ROS2)
                  realtimeRos2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.INTRAPROCESS, getClass().getSimpleName(), ROS2Tools.RUNTIME_EXCEPTION);
            }
            else
            {
               if (CommunicationOptions.USE_KRYO)
                  packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.CONTROLLER_PORT, netClassList.get());
               if (CommunicationOptions.USE_ROS2)
                  realtimeRos2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.FAST_RTPS, getClass().getSimpleName(), ROS2Tools.RUNTIME_EXCEPTION);
            }

            if (CommunicationOptions.USE_KRYO)
               packetCommunicator.connect();
         }
         catch (BindException bindException)
         {
            PrintTools.error(this, bindException.getMessage());
            PrintTools.warn(this, "Continuing without networking");
            useNetworking.set(false);
         }
      }
   }

   private void createGlobalDataProducer()
   {
      if (useNetworking.get())
      {
         globalDataProducer = new QuadrupedGlobalDataProducer(packetCommunicator);
      }
   }

   private void createHeadController()
   {
      if (headControllerFactory.hasValue())
      {
         headControllerFactory.get().setControlDt(controlDT.get());
         headControllerFactory.get().setFullRobotModel(fullRobotModel.get());
         headControllerFactory.get().setGlobalDataProducer(globalDataProducer);

         headController = headControllerFactory.get().createRobotController();
      }
   }

   private void createInverseKinematicsCalculator()
   {
      if (controlMode.get() == QuadrupedControlMode.POSITION)
      {
         legInverseKinematicsCalculator = new QuadrupedInverseKinematicsCalculators(modelFactory.get(), jointDesiredOutputList.get(), physicalProperties.get(),
                                                                                    fullRobotModel.get(), referenceFrames.get(),
                                                                                    sdfRobot.get().getRobotsYoVariableRegistry(), yoGraphicsListRegistry);
      }
   }

   public void createControllerManager() throws IOException
   {
      QuadrupedRuntimeEnvironment runtimeEnvironment = new QuadrupedRuntimeEnvironment(controlDT.get(), sdfRobot.get().getYoTime(), fullRobotModel.get(),
                                                                                       controllerCoreOptimizationSettings.get(), jointDesiredOutputList.get(),
                                                                                       sdfRobot.get().getRobotsYoVariableRegistry(), yoGraphicsListRegistry,
                                                                                       yoGraphicsListRegistryForDetachedOverhead, globalDataProducer,
                                                                                       contactableFeet, contactablePlaneBodies, footSwitches, gravity.get());
      switch (controlMode.get())
      {
      case FORCE:
         if (initialForceControlState.hasValue())
            controllerManager = new QuadrupedControllerManager(runtimeEnvironment, physicalProperties.get(), initialPositionParameters.get(),
                                                               initialForceControlState.get());
         else
            controllerManager = new QuadrupedControllerManager(runtimeEnvironment, physicalProperties.get(), initialPositionParameters.get());
         break;
      case POSITION:
         if (initialForceControlState.hasValue())
            controllerManager = new QuadrupedControllerManager(runtimeEnvironment, modelFactory.get(), physicalProperties.get(),
                                                               positionBasedCrawlControllerParameters.get(), initialPositionParameters.get(),
                                                               controlMode.get());
         else
            controllerManager = new QuadrupedControllerManager(runtimeEnvironment, modelFactory.get(), physicalProperties.get(),
                                                               positionBasedCrawlControllerParameters.get(), initialPositionParameters.get(),
                                                               initialForceControlState.get(), controlMode.get());
         break;
      default:
         controllerManager = null;
         break;
      }
   }

   private void createPoseCommunicator()
   {
      if (useNetworking.get())
      {
         JointConfigurationGatherer jointConfigurationGathererAndProducer = new JointConfigurationGatherer(fullRobotModel.get());
         PeriodicThreadScheduler scheduler = new PeriodicNonRealtimeThreadScheduler("PoseCommunicator");
         poseCommunicator = new DRCPoseCommunicator(fullRobotModel.get(), jointConfigurationGathererAndProducer, null, globalDataProducer,
                                                    timestampProvider.get(), sensorReader.getSensorRawOutputMapReadOnly(),
                                                    controllerManager.getMotionStatusHolder(), null, scheduler, netClassList.get());
      }
      else
      {
         poseCommunicator = null;
      }
   }

   private void createControllerNetworkSubscriber()
   {
      if (useNetworking.get())
         controllerManager.createControllerNetworkSubscriber(new PeriodicNonRealtimeThreadScheduler("controllerNetworkSubscriber"), packetCommunicator, realtimeRos2Node);
   }

   private void createGroundContactModel()
   {
      if (!providedGroundProfile3D.hasValue() && !providedTerrainObject3D.hasValue())
      {
         switch (groundContactModelType.get())
         {
         case FLAT:
            groundProfile3D = new FlatGroundProfile(0.0);
            break;
         case ROLLING_HILLS:
            groundProfile3D = new RollingGroundProfile(0.025, 1.0, 0.0, -20.0, 20.0, -20.0, 20.0);
            break;
         case ROTATABLE:
            groundProfile3D = new RotatablePlaneTerrainProfile(new Point3D(), sdfRobot.get(), yoGraphicsListRegistry, controlDT.get());
            break;
         case SLOPES:
            double xMin = -5.0, xMax = 40.0;
            double yMin = -5.0, yMax = 5.0;
            double[][] xSlopePairs = new double[][] {{1.0, 0.0}, {3.0, 0.1}};
            groundProfile3D = new AlternatingSlopesGroundProfile(xSlopePairs, xMin, xMax, yMin, yMax);
            break;
         case STEPPED:
            groundProfile3D = new VaryingStairGroundProfile(0.0, 0.0, new double[] {1.5, 1.0, 1.0, 0.5}, new double[] {0.0, 0.05, -0.1, 0.05, 0.05});
            break;
         default:
            groundProfile3D = null;
            break;
         }
      }
      else if (providedTerrainObject3D.hasValue())
      {
         if (providedGroundProfile3D.hasValue())
            throw new RuntimeException("You can only set either a terrain object or a ground profile.");

         groundProfile3D = providedTerrainObject3D.get();
      }
      else
      {
         groundProfile3D = providedGroundProfile3D.get();
      }

      groundContactModel = new LinearGroundContactModel(sdfRobot.get(), sdfRobot.get().getRobotsYoVariableRegistry());
      groundContactModel.setZStiffness(groundContactParameters.get().getZStiffness());
      groundContactModel.setZDamping(groundContactParameters.get().getZDamping());
      groundContactModel.setXYStiffness(groundContactParameters.get().getXYStiffness());
      groundContactModel.setXYDamping(groundContactParameters.get().getXYDamping());
      groundContactModel.setGroundProfile3D(groundProfile3D);
   }

   private void createSimulationController()
   {
      simulationController = new QuadrupedSimulationController(sdfRobot.get(), sensorReader, outputWriter.get(), controllerManager, stateEstimator,
                                                               poseCommunicator, headController);
   }

   private void setupSDFRobot()
   {
      initialPositionParameters.get().offsetInitialConfiguration(initialOffset.get());

      int simulationTicksPerControllerTick = (int) Math.round(controlDT.get() / simulationDT.get());
      sdfRobot.get().setController(simulationController, simulationTicksPerControllerTick);
      sdfRobot.get().setPositionInWorld(initialPositionParameters.get().getInitialBodyPosition());
      sdfRobot.get().setOrientation(initialPositionParameters.get().getInitialBodyOrientation());

      for (QuadrupedJointName quadrupedJointName : modelFactory.get().getQuadrupedJointNames())
      {
         OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint = sdfRobot.get()
                                                                   .getOneDegreeOfFreedomJoint(modelFactory.get().getSDFNameForJointName(quadrupedJointName));
         if (oneDegreeOfFreedomJoint != null)
         {
            oneDegreeOfFreedomJoint.setQ(initialPositionParameters.get().getInitialJointPosition(quadrupedJointName));
         }
      }
      try
      {
         sdfRobot.get().update();
         sdfRobot.get().doDynamicsButDoNotIntegrate();
         sdfRobot.get().update();
      }
      catch (UnreasonableAccelerationException unreasonableAccelerationException)
      {
         throw new RuntimeException("UnreasonableAccelerationException");
      }

      Point3D initialCoMPosition = new Point3D();
      double totalMass = sdfRobot.get().computeCenterOfMass(initialCoMPosition);

      if (useStateEstimator.get())
      {
         Quaternion initialEstimationLinkOrientation = new Quaternion();
         sdfRobot.get().getRootJoint().getJointTransform3D().getRotation(initialEstimationLinkOrientation);
         stateEstimator.initializeEstimatorToActual(initialCoMPosition, initialEstimationLinkOrientation);
      }

      sdfRobot.get().setGravity(gravity.get());
      sdfRobot.get().setGroundContactModel(groundContactModel);
      PrintTools.info(this, sdfRobot.get().getName() + " total mass: " + totalMass);
   }

   public SimulationConstructionSet createSimulation() throws IOException
   {
      groundContactModelType.setDefaultValue(QuadrupedGroundContactModelType.FLAT);
      usePushRobotController.setDefaultValue(false);
      footSwitchType.setDefaultValue(FootSwitchType.TouchdownBased);
      useLocalCommunicator.setDefaultValue(false);

      FactoryTools.checkAllFactoryFieldsAreSet(this);

      setupYoRegistries();
      createPushRobotController();
      createSensorReader();
      createContactableFeet();
      createContactablePlaneBodies();
      createFootSwitches();
      createStateEstimator();
      createPacketCommunicator();
      createGlobalDataProducer();
      createHeadController();
      createInverseKinematicsCalculator();
      createControllerManager();
      createControllerNetworkSubscriber();
      createPoseCommunicator();

      createGroundContactModel();
      createSimulationController();
      setupSDFRobot();
      setupJointElasticity();

      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot.get(), scsParameters.get());
      if (groundContactModelType.get() == QuadrupedGroundContactModelType.ROTATABLE || providedTerrainObject3D.hasValue())
      {
         scs.setGroundVisible(false);
      }
      if (scsBufferSize.hasValue())
      {
         scs.setMaxBufferSize(scsBufferSize.get());
      }

      if (providedTerrainObject3D.hasValue())
      {
         scs.addStaticLinkGraphics(providedTerrainObject3D.get().getLinkGraphics());
      }

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setDT(simulationDT.get(), recordFrequency.get());
      if (scs.getSimulationConstructionSetParameters().getCreateGUI())
      {
         scs.setCameraTrackingVars("q_x", "q_y", "q_z");
         scs.setCameraDollyVars("q_x", "q_y", "q_z");
         scs.setCameraTracking(useTrackAndDolly.get(), useTrackAndDolly.get(), useTrackAndDolly.get(), useTrackAndDolly.get());
         scs.setCameraDolly(useTrackAndDolly.get(), useTrackAndDolly.get(), useTrackAndDolly.get(), false);
         scs.setCameraDollyOffsets(4.0, 4.0, 1.0);
         SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
         simulationOverheadPlotterFactory.setVariableNameToTrack("centerOfMass");
         simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
         simulationOverheadPlotterFactory.setShowOnStart(showPlotter.get());
         simulationOverheadPlotterFactory.createOverheadPlotter();
      }

      InputStream parameterFile = getClass().getResourceAsStream(modelFactory.get().getParameterResourceName(controlMode.get()));
      ParameterLoaderHelper.loadParameters(this, parameterFile, simulationController.getYoVariableRegistry());
      scs.setParameterRootPath(simulationController.getYoVariableRegistry().getParent());

      FactoryTools.disposeFactory(this);

      return scs;
   }

   // OPTIONS

   private void setupJointElasticity()
   {
      if (simulatedElasticityParameters.hasValue())
      {
         FloatingRootJointRobot floatingRootJointRobot = sdfRobot.get();
         SpringJointOutputWriter springJointOutputWriter = new SpringJointOutputWriter(floatingRootJointRobot, simulatedElasticityParameters.get(),
                                                                                       simulationDT.get());
         floatingRootJointRobot.setController(springJointOutputWriter, 1);
      }
   }

   public void setSimulationDT(double simulationDT)
   {
      this.simulationDT.set(simulationDT);
   }

   public void setControlDT(double controlDT)
   {
      this.controlDT.set(controlDT);
   }

   public void setJointDesiredOutputList(JointDesiredOutputList jointDesiredOutputList)
   {
      this.jointDesiredOutputList.set(jointDesiredOutputList);
   }

   public void setGravity(double gravity)
   {
      this.gravity.set(gravity);
   }

   public void setRecordFrequency(int recordFrequency)
   {
      this.recordFrequency.set(recordFrequency);
   }

   public void setUseTrackAndDolly(boolean useTrackAndDolly)
   {
      this.useTrackAndDolly.set(useTrackAndDolly);
   }

   public void setShowPlotter(boolean showPlotter)
   {
      this.showPlotter.set(showPlotter);
   }

   public void setModelFactory(QuadrupedModelFactory modelFactory)
   {
      this.modelFactory.set(modelFactory);
   }

   public void setSCSParameters(SimulationConstructionSetParameters scsParameters)
   {
      this.scsParameters.set(scsParameters);
   }

   public void setGroundContactModelType(QuadrupedGroundContactModelType groundContactModelType)
   {
      this.groundContactModelType.set(groundContactModelType);
   }

   public void setSimulatedElasticityParameters(SimulatedElasticityParameters simulatedElasticityParameters)
   {
      this.simulatedElasticityParameters.set(simulatedElasticityParameters);
   }

   public void setGroundContactParameters(GroundContactParameters groundContactParameters)
   {
      this.groundContactParameters.set(groundContactParameters);
   }

   public void setHeadControllerFactory(QuadrupedRobotControllerFactory headControllerFactory)
   {
      this.headControllerFactory.set(headControllerFactory);
   }

   public void setOutputWriter(OutputWriter outputWriter)
   {
      this.outputWriter.set(outputWriter);
   }

   public void setInitialPositionParameters(QuadrupedInitialPositionParameters initialPositionParameters)
   {
      this.initialPositionParameters.set(initialPositionParameters);
   }

   public void setInitialOffset(QuadrupedInitialOffsetAndYaw initialOffset)
   {
      this.initialOffset.set(initialOffset);
   }

   public void setPhysicalProperties(QuadrupedPhysicalProperties physicalProperties)
   {
      this.physicalProperties.set(physicalProperties);
   }

   public void setControlMode(QuadrupedControlMode controlMode)
   {
      this.controlMode.set(controlMode);
   }

   public void setFullRobotModel(FullQuadrupedRobotModel fullRobotModel)
   {
      this.fullRobotModel.set(fullRobotModel);
   }

   public void setControllerCoreOptimizationSettings(ControllerCoreOptimizationSettings controllerCoreOptimizationSettings)
   {
      this.controllerCoreOptimizationSettings.set(controllerCoreOptimizationSettings);
   }

   public void setUseNetworking(boolean useNetworking)
   {
      this.useNetworking.set(useNetworking);
   }

   public void setNetClassList(NetClassList netClassList)
   {
      this.netClassList.set(netClassList);
   }

   public void setTimestampHolder(SensorTimestampHolder timestampProvider)
   {
      this.timestampProvider.set(timestampProvider);
   }

   public void setSDFRobot(FloatingRootJointRobot sdfRobot)
   {
      this.sdfRobot.set(sdfRobot);
   }

   public void setUseStateEstimator(boolean useStateEstimator)
   {
      this.useStateEstimator.set(useStateEstimator);
   }

   public void setSensorInformation(QuadrupedSensorInformation sensorInformation)
   {
      this.sensorInformation.set(sensorInformation);
   }

   public void setStateEstimatorParameters(StateEstimatorParameters stateEstimatorParameters)
   {
      this.stateEstimatorParameters.set(stateEstimatorParameters);
   }

   public void setReferenceFrames(QuadrupedReferenceFrames referenceFrames)
   {
      this.referenceFrames.set(referenceFrames);
   }

   public void setPositionBasedCrawlControllerParameters(QuadrupedPositionBasedCrawlControllerParameters positionBasedCrawlControllerParameters)
   {
      this.positionBasedCrawlControllerParameters.set(positionBasedCrawlControllerParameters);
   }

   public void setGroundProfile3D(GroundProfile3D groundProfile3D)
   {
      providedGroundProfile3D.set(groundProfile3D);
   }

   public void setTerrainObject3D(TerrainObject3D terrainObject3D)
   {
      providedTerrainObject3D.set(terrainObject3D);
   }

   public void setUsePushRobotController(boolean usePushRobotController)
   {
      this.usePushRobotController.set(usePushRobotController);
   }

   public void setFootSwitchType(FootSwitchType footSwitchType)
   {
      this.footSwitchType.set(footSwitchType);
   }

   public void setScsBufferSize(int scsBufferSize)
   {
      this.scsBufferSize.set(scsBufferSize);
   }

   public void setInitialForceControlState(QuadrupedControllerEnum initialForceControlState)
   {
      this.initialForceControlState.set(initialForceControlState);
   }

   public void setUseLocalCommunicator(boolean useLocalCommunicator)
   {
      this.useLocalCommunicator.set(useLocalCommunicator);
   }
}
