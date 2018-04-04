package us.ihmc.exampleSimulations.genericQuadruped;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedModelFactory;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedPhysicalProperties;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedSensorInformation;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedControllerCoreOptimizationSettings;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedStateEstimatorParameters;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedXGaitSettings;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.quadrupedRobotics.communication.QuadrupedGlobalDataProducer;
import us.ihmc.quadrupedRobotics.communication.QuadrupedNetClassList;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerEnum;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerManager;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.JointsOnlyStateEstimator;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedFootSwitchFactory;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedStateEstimatorFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.stateEstimation.humanoid.DRCStateEstimatorInterface;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.JointStateUpdater;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class GenericQuadrupedControllerFactoryDummyOutputDemo
{
   private static final boolean USE_KINEMATICS_BASED_STATE_ESTIMATOR = true;
   private static final double GRAVITY = 9.81;

   private static final double DT = 0.001;
   private static final int TEST_ITERATIONS = 10000;

   private QuadrupedForceControllerManager controllerManager;
   private DRCStateEstimatorInterface simulationStateEstimator;
   private QuadrupedRuntimeEnvironment runtimeEnvironment;

   public GenericQuadrupedControllerFactoryDummyOutputDemo() throws IOException
   {
      /*
       * Create GenericQuadruped model
       */
      QuadrupedPhysicalProperties physicalProperties = new GenericQuadrupedPhysicalProperties();
      GenericQuadrupedModelFactory modelFactory = new GenericQuadrupedModelFactory();
      FullQuadrupedRobotModel fullRobotModel = modelFactory.createFullRobotModel();
      GenericQuadrupedControllerCoreOptimizationSettings controllerCoreOptimizationSettings = new GenericQuadrupedControllerCoreOptimizationSettings(
            fullRobotModel.getTotalMass());

      /*
       * Create registries for every thread
       */
      YoVariableRegistry registry = new YoVariableRegistry("GenericQuadruped");
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoDouble controllerTime = new YoDouble("controllerTime", registry);

      /*
       * Create packet communicators
       */
      PacketCommunicator networkProcessorCommunicator = PacketCommunicator
            .createTCPPacketCommunicatorServer(NetworkPorts.CONTROLLER_PORT, 2097152, 2097152, new QuadrupedNetClassList());
      QuadrupedGlobalDataProducer dataProducer = new QuadrupedGlobalDataProducer(networkProcessorCommunicator);

      /*
       * Create EtherCAT loop
       */
      GenericQuadrupedStateEstimatorParameters stateEstimatorParameters = new GenericQuadrupedStateEstimatorParameters();

      /*
       * Create controller
       */
      QuadrupedReferenceFrames referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, physicalProperties);

      QuadrupedPhysicalProperties quadrupedPhysicalProperties = new GenericQuadrupedPhysicalProperties();

      ContactableBodiesFactory<RobotQuadrant> footContactableBodiesFactory = new ContactableBodiesFactory<>();
      footContactableBodiesFactory.setFullRobotModel(fullRobotModel);
      footContactableBodiesFactory.setFootContactPoints(quadrupedPhysicalProperties.getFeetGroundContactPoints());
      footContactableBodiesFactory.setReferenceFrames(referenceFrames);
      QuadrantDependentList<ContactablePlaneBody> contactableFeet = new QuadrantDependentList<>(
            footContactableBodiesFactory.createFootContactablePlaneBodies());
      footContactableBodiesFactory.disposeFactory();

      QuadrupedFootSwitchFactory footSwitchFactory = new QuadrupedFootSwitchFactory();
      footSwitchFactory.setFootContactableBodies(contactableFeet);
      footSwitchFactory.setFullRobotModel(fullRobotModel);
      footSwitchFactory.setGravity(GRAVITY);
      footSwitchFactory.setYoVariableRegistry(registry);
      footSwitchFactory.setFootSwitchType(FootSwitchType.TouchdownBased);
      QuadrantDependentList<FootSwitchInterface> footSwitches = footSwitchFactory.createFootSwitches();

      List<ContactablePlaneBody> contactablePlaneBodies = new ArrayList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         contactablePlaneBodies.add(contactableFeet.get(robotQuadrant));

      SensorOutputMapReadOnly sensorOutputMapReadOnly = createSensorProcessing(fullRobotModel, stateEstimatorParameters, registry);
      if (USE_KINEMATICS_BASED_STATE_ESTIMATOR)
      {
         GenericQuadrupedSensorInformation sensorInformation = new GenericQuadrupedSensorInformation();

         QuadrupedStateEstimatorFactory stateEstimatorFactory = new QuadrupedStateEstimatorFactory();
         stateEstimatorFactory.setEstimatorDT(DT);
         stateEstimatorFactory.setFootContactableBodies(contactableFeet);
         stateEstimatorFactory.setFootSwitches(footSwitches);
         stateEstimatorFactory.setFullRobotModel(fullRobotModel);
         stateEstimatorFactory.setGravity(GRAVITY);
         stateEstimatorFactory.setSensorInformation(sensorInformation);
         stateEstimatorFactory.setSensorOutputMapReadOnly(sensorOutputMapReadOnly);
         stateEstimatorFactory.setStateEstimatorParameters(stateEstimatorParameters);
         stateEstimatorFactory.setYoGraphicsListRegistry(yoGraphicsListRegistry);
         stateEstimatorFactory.setYoVariableRegistry(registry);
         simulationStateEstimator = stateEstimatorFactory.createStateEstimator();
      }
      else
      {
         FullInverseDynamicsStructure inverseDynamicsStructure = FullInverseDynamicsStructure.createInverseDynamicStructure(fullRobotModel);
         JointStateUpdater jointStateUpdater = new JointStateUpdater(inverseDynamicsStructure, sensorOutputMapReadOnly, null, registry);
         simulationStateEstimator = new JointsOnlyStateEstimator(fullRobotModel, sensorOutputMapReadOnly, jointStateUpdater);
      }

      JointDesiredOutputList jointDesiredOutputList = new JointDesiredOutputList(fullRobotModel.getOneDoFJoints());
      YoGraphicsListRegistry ignoredYoGraphicsListRegistry = new YoGraphicsListRegistry();
      GenericQuadrupedXGaitSettings xGaitSettings = new GenericQuadrupedXGaitSettings();

      runtimeEnvironment = new QuadrupedRuntimeEnvironment(DT, controllerTime, fullRobotModel, controllerCoreOptimizationSettings, jointDesiredOutputList,
                                                           registry, yoGraphicsListRegistry, ignoredYoGraphicsListRegistry, dataProducer, contactableFeet,
                                                           contactablePlaneBodies, xGaitSettings, footSwitches, GRAVITY);
      controllerManager = new QuadrupedForceControllerManager(runtimeEnvironment, physicalProperties);

      InputStream resourceAsStream = getClass().getResourceAsStream(modelFactory.getParameterResourceName(QuadrupedControlMode.FORCE));
      ParameterLoaderHelper.loadParameters(this, resourceAsStream, registry);

      //      PrintTools.debug(this, "Warming up JIT compiler.");
      //      controllerManager.warmup(2000);  // Compile threshold == 1000
      //      PrintTools.debug(this, "Coffee time.");

   }

   private SensorProcessing createSensorProcessing(FullRobotModel fullRobotModel, SensorProcessingConfiguration sensorProcessingConfiguration,
                                                   YoVariableRegistry parentRegistry)
   {

      StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();
      for (OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         stateEstimatorSensorDefinitions.addJointSensorDefinition(joint);
      }

      IMUDefinition[] imuDefinitions = fullRobotModel.getIMUDefinitions();

      stateEstimatorSensorDefinitions.addIMUSensorDefinition(imuDefinitions);

      return new SensorProcessing(stateEstimatorSensorDefinitions, sensorProcessingConfiguration, parentRegistry);
   }

   public void run()
   {

      YoDouble robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      double robotTimeBeforeWarmUp = robotTimestamp.getDoubleValue();
      for (QuadrupedForceControllerEnum state : QuadrupedForceControllerEnum.values)
      {
         QuadrupedController stateImpl = controllerManager.getState(state);

         stateImpl.onEntry();

         long min = Long.MAX_VALUE;
         long max = 0;

         long startTime = System.nanoTime();
         for (int i = 0; i < TEST_ITERATIONS; i++)
         {
            long itStart = System.nanoTime();
            simulationStateEstimator.doControl();
            robotTimestamp.add(Conversions.millisecondsToSeconds(1));
            stateImpl.doAction(Double.NaN);

            long itTime = System.nanoTime() - itStart;

            if (itTime < min)
            {
               min = itTime;
            }
            if (itTime > max)
            {
               max = itTime;
            }
         }
         long endTime = System.nanoTime() - startTime;

         stateImpl.onExit();

         System.out.println(
               state + ": " + Conversions.nanosecondsToSeconds(endTime / TEST_ITERATIONS) + "s/it. Min: " + Conversions.nanosecondsToSeconds(min) + "s ; max: "
                     + Conversions.nanosecondsToSeconds(max) + "s");

      }
      robotTimestamp.set(robotTimeBeforeWarmUp);

   }

   public static void main(String[] args) throws IOException
   {
      GenericQuadrupedControllerFactoryDummyOutputDemo genericQuadrupedControllerFactoryDummyOutputDemo = new GenericQuadrupedControllerFactoryDummyOutputDemo();

      genericQuadrupedControllerFactoryDummyOutputDemo.run();

      System.exit(0);
   }

}
