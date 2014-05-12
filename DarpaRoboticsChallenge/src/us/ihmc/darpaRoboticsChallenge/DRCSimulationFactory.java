package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.SdfLoader.SDFFullRobotModelFactory;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonAvatarInterfaces.CommonAvatarEnvironmentInterface;
import us.ihmc.commonWalkingControlModules.automaticSimulationRunner.AutomaticSimulationRunner;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.controllers.PIDLidarTorqueController;
import us.ihmc.commonWalkingControlModules.visualizer.RobotVisualizer;
import us.ihmc.darpaRoboticsChallenge.controllers.EstimationLinkHolder;
import us.ihmc.darpaRoboticsChallenge.controllers.concurrent.BlockingThreadSynchronizer;
import us.ihmc.darpaRoboticsChallenge.controllers.concurrent.ThreadSynchronizer;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.darpaRoboticsChallenge.drcRobot.SimulatedDRCRobotTimeProvider;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.initialSetup.ScsInitialSetup;
import us.ihmc.darpaRoboticsChallenge.outputs.DRCOutputWriter;
import us.ihmc.darpaRoboticsChallenge.outputs.DRCSimulationOutputWriter;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCPerfectSensorReaderFactory;
import us.ihmc.darpaRoboticsChallenge.stateEstimation.DRCSimulatedSensorNoiseParameters;
import us.ihmc.darpaRoboticsChallenge.stateEstimation.DRCStateEstimatorInterface;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedSensorHolderAndReaderFromRobotFactory;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimator;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.RunnableRunnerController;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.StateEstimatorErrorCalculatorController;
import us.ihmc.util.NonRealtimeThreadFactory;
import us.ihmc.util.ThreadFactory;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;
import us.ihmc.utilities.net.TimestampProvider;

import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.UnreasonableAccelerationException;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.gui.GUISetterUpperRegistry;
import com.yobotics.simulationconstructionset.physics.ScsCollisionConfigure;
import com.yobotics.simulationconstructionset.robotController.ModularRobotController;
import com.yobotics.simulationconstructionset.util.environments.SelectableObjectListener;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.ground.TerrainObject;

public class DRCSimulationFactory
{
   private static final boolean COMPUTE_ESTIMATOR_ERROR = true;

   private final HumanoidRobotSimulation<SDFRobot> sim;
   private final DRCController controller;
   
   private static SDFRobot simulatedRobot;
   private static SimulatedDRCRobotTimeProvider timestampProvider;
   
   public DRCSimulationFactory(DRCRobotModel drcRobotModel, ControllerFactory controllerFactory,
         final TerrainObject environmentTerrain, DRCRobotInitialSetup<SDFRobot> robotInitialSetup, ScsInitialSetup scsInitialSetup,
         DRCGuiInitialSetup guiInitialSetup, GlobalDataProducer globalDataProducer, RobotVisualizer robotVisualizer)
   {
      CommonAvatarEnvironmentInterface env;
      
      if (environmentTerrain != null)
      {
         env = new CommonAvatarEnvironmentInterface()

         {

            @Override
            public TerrainObject getTerrainObject()
            {
               // TODO Auto-generated method stub
               return environmentTerrain;
            }

            @Override
            public List<Robot> getEnvironmentRobots()
            {
               return new ArrayList<Robot>();
            }

            @Override
            public void createAndSetContactControllerToARobot()
            {
            }

            @Override
            public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
            {
            }

            @Override
            public void addContactPoints(ExternalForcePoint[] externalForcePoints)
            {
            }
         };
      }
      else
      {
         env = null;
      }
      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry(false);
      Pair<HumanoidRobotSimulation<SDFRobot>, DRCController> simAndController = createSimulation(controllerFactory, env, robotInitialSetup, scsInitialSetup, guiInitialSetup, globalDataProducer, robotVisualizer, dynamicGraphicObjectsListRegistry, false, drcRobotModel);
      sim = simAndController.first();
      controller = simAndController.second();
   }
   
   public static Pair<HumanoidRobotSimulation<SDFRobot>, DRCController> createSimulation(ControllerFactory controllerFactory,
         CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface, DRCRobotInitialSetup<SDFRobot> robotInitialSetup,
         ScsInitialSetup scsInitialSetup, DRCGuiInitialSetup guiInitialSetup, GlobalDataProducer dataProducer, RobotVisualizer robotVisualizer,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, boolean useNewPhysics, DRCRobotModel drcRobotModel)
   {
      GUISetterUpperRegistry guiSetterUpperRegistry = new GUISetterUpperRegistry();

      
      DRCRobotSensorInformation sensorInformation = drcRobotModel.getSensorInformation();

      double estimateDT = drcRobotModel.getEstimatorDT();
      double simulateDT = drcRobotModel.getSimulateDT();
      double controlDT = drcRobotModel.getControllerDT();
      StateEstimatorParameters stateEstimatorParameters = drcRobotModel.getStateEstimatorParameters();
      
      int estimationTicksPerControlTick = (int) (estimateDT / simulateDT);

      simulatedRobot = drcRobotModel.createSdfRobot(false);
      YoVariableRegistry estimatorRegistry = new YoVariableRegistry("Estimator");
      YoVariableRegistry controllerRegistry = new YoVariableRegistry("controllerRegistry");
//      YoVariableThreadAccessValidator estimatorThreadValidator = new YoVariableThreadAccessValidator(estimatorRegistry);
//      YoVariableThreadAccessValidator controllerThreadValidator = new YoVariableThreadAccessValidator(controllerRegistry);
      
      YoVariableRegistry simulationRegistry = simulatedRobot.getRobotsYoVariableRegistry();
      simulationRegistry.addChild(estimatorRegistry);
      simulationRegistry.addChild(controllerRegistry);
      
      setupJointDamping(simulatedRobot, drcRobotModel);

      DRCOutputWriter drcOutputWriter = new DRCSimulationOutputWriter(simulatedRobot, dynamicGraphicObjectsListRegistry, robotVisualizer, simulationRegistry);
      if (DRCLocalConfigParameters.INTEGRATE_ACCELERATIONS_AND_CONTROL_VELOCITIES)
      {
         drcOutputWriter = drcRobotModel.getOutputWriterWithAccelerationIntegration(drcOutputWriter, false);
      }
      
      if (DRCConfigParameters.LIMIT_CONTROLLER_OUTPUT_TORQUES)
      {
//         drcOutputWriter = new DRCOutputWriterWithTorqueLimits(drcOutputWriter);
         throw new RuntimeException("Atlas torque limits are under NDA");
      }
      
      // TODO: Build LIDAR here
      LidarControllerInterface lidarControllerInterface;

      SensorNoiseParameters sensorNoiseParameters;

      lidarControllerInterface = new PIDLidarTorqueController(DRCConfigParameters.LIDAR_SPINDLE_VELOCITY, controlDT);
      sensorNoiseParameters = DRCSimulatedSensorNoiseParameters.createSensorNoiseParametersZeroNoise();
//         sensorNoiseParameters = DRCSimulatedSensorNoiseParameters.createSensorNoiseParametersALittleNoise();
//         sensorNoiseParameters = DRCSimulatedSensorNoiseParameters.createSensorNoiseParametersGazeboSDF();

      //    SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMapForEstimator;
      //    scsToInverseDynamicsJointMapForEstimator = robotInterface.getSCSToInverseDynamicsJointMap();


      ScsCollisionConfigure collisionConfigure = null;
      
      if( useNewPhysics ) {
         collisionConfigure = drcRobotModel.getPhysicsConfigure(simulatedRobot);
      }

      SensorReaderFactory sensorReaderFactory; // this is the connection between the ModularRobotController and the DRCController below
      SimulatedSensorHolderAndReaderFromRobotFactory simulatedSensorHolderAndReaderFromRobotFactory;
      ModularRobotController controller = new ModularRobotController("SensorReaders");
      if (DRCConfigParameters.USE_PERFECT_SENSORS)
      {
         DRCPerfectSensorReaderFactory drcPerfectSensorReaderFactory = new DRCPerfectSensorReaderFactory(simulatedRobot, estimateDT);
         controller.addRobotController(drcPerfectSensorReaderFactory.getSensorReader());
         sensorReaderFactory = drcPerfectSensorReaderFactory;
      }
      else
      {
         simulatedSensorHolderAndReaderFromRobotFactory = new SimulatedSensorHolderAndReaderFromRobotFactory(
               simulatedRobot, sensorNoiseParameters, stateEstimatorParameters.getSensorFilterParameters(), sensorInformation.getIMUSensorsToUse(), estimatorRegistry);
         sensorReaderFactory = simulatedSensorHolderAndReaderFromRobotFactory;
      }

//      SimulatedHandControllerDispatcher handControllerDispatcher = new SimulatedHandControllerDispatcher(simulatedRobot, robotInterface.getTimeStampProvider(),
//            estimationTicksPerControlTick);
//      controller.addRobotController(handControllerDispatcher);

      controller.addRobotController(lidarControllerInterface);

      double gravity = simulatedRobot.getGravityZ();
     
      ThreadFactory threadFactory = new NonRealtimeThreadFactory();
      ThreadSynchronizer threadSynchronizer = new BlockingThreadSynchronizer();
      
      timestampProvider = new SimulatedDRCRobotTimeProvider(drcRobotModel.getSimulateDT());
      simulatedRobot.setController(timestampProvider, estimationTicksPerControlTick);
      SDFFullRobotModelFactory fullRobotModelFactory = new SDFFullRobotModelFactory(drcRobotModel.getGeneralizedRobotModel(), drcRobotModel.getJointMap());
      DRCController robotController = new DRCController(fullRobotModelFactory, controllerFactory, sensorReaderFactory, drcOutputWriter,
            lidarControllerInterface, gravity, controlDT, dataProducer, timestampProvider,
            dynamicGraphicObjectsListRegistry, guiSetterUpperRegistry, estimatorRegistry, controllerRegistry, threadFactory, threadSynchronizer, 
            drcRobotModel, drcRobotModel.getContactPointParamaters(false, false), estimateDT);
      robotController.initialize();

      if (simulatedSensorHolderAndReaderFromRobotFactory != null)
         controller.addRobotController(new RunnableRunnerController(simulatedSensorHolderAndReaderFromRobotFactory.getSensorReader()));

      final HumanoidRobotSimulation<SDFRobot> humanoidRobotSimulation = new HumanoidRobotSimulation<SDFRobot>(simulatedRobot, controller,
            estimationTicksPerControlTick, commonAvatarEnvironmentInterface, simulatedRobot.getAllExternalForcePoints(), robotInitialSetup, scsInitialSetup,
            guiInitialSetup, guiSetterUpperRegistry, dynamicGraphicObjectsListRegistry, drcRobotModel,collisionConfigure);

      //TODO: Can only do this if we have a simulation...
      if (scsInitialSetup.getInitializeEstimatorToActual())
      {
         System.err.println("Warning! Initializing Estimator to Actual!");
         DRCStateEstimatorInterface drcStateEstimator = robotController.getDRCStateEstimator();
         initializeEstimatorToActual(drcStateEstimator, robotInitialSetup, simulatedRobot, drcRobotModel.getJointMap());
      }

      if (COMPUTE_ESTIMATOR_ERROR && robotController.getDRCStateEstimator() != null)
      {
         DRCStateEstimatorInterface drcStateEstimator = robotController.getDRCStateEstimator();
         StateEstimator stateEstimator = drcStateEstimator.getStateEstimator();

         Joint estimationJoint = getEstimationJoint(simulatedRobot,drcRobotModel.getJointMap());

         StateEstimatorErrorCalculatorController stateEstimatorErrorCalculatorController = new StateEstimatorErrorCalculatorController(stateEstimator,
               simulatedRobot, estimationJoint, stateEstimatorParameters.getAssumePerfectIMU(), stateEstimatorParameters.useKinematicsBasedStateEstimator());
         simulatedRobot.setController(stateEstimatorErrorCalculatorController, estimationTicksPerControlTick);
      }
      
//      estimatorThreadValidator.start();
//      controllerThreadValidator.start();
      
      return new Pair<HumanoidRobotSimulation<SDFRobot>, DRCController>(humanoidRobotSimulation, robotController);
   }

   private static Joint getEstimationJoint(SDFRobot simulatedRobot, DRCRobotJointMap jointMap)
   {
      Joint estimationJoint;
      if (EstimationLinkHolder.usingChestLink())
      {
         estimationJoint = simulatedRobot.getJoint(jointMap.getNameOfJointBeforeChest()); // used to be: AtlasOrderedJointMap.jointNames[AtlasOrderedJointMap.back_bkx]);
         if (estimationJoint == null)
            throw new RuntimeException("Couldn't find chest joint!");
      }
      else
      {
         estimationJoint = simulatedRobot.getPelvisJoint();
      }
      return estimationJoint;
   }

   private static void setupJointDamping(SDFRobot simulatedRobot, DRCRobotModel robotModel)
   {      
      robotModel.setJointDamping(simulatedRobot);
   }

   private static void initializeEstimatorToActual(DRCStateEstimatorInterface drcStateEstimator, DRCRobotInitialSetup<SDFRobot> robotInitialSetup, SDFRobot simulatedRobot, DRCRobotJointMap jointMap)
   {
      // The following is to get the initial CoM position from the robot. 
      // It is cheating for now, and we need to move to where the 
      // robot itself determines coordinates, and the sensors are all
      // in the robot-determined world coordinates..
      Point3d initialCoMPosition = new Point3d();
      robotInitialSetup.initializeRobot(simulatedRobot, jointMap);
      updateRobot(simulatedRobot);

      simulatedRobot.computeCenterOfMass(initialCoMPosition);

      Joint estimationJoint = getEstimationJoint(simulatedRobot,jointMap);

      Transform3D estimationLinkTransform3D = estimationJoint.getJointTransform3D();
      Quat4d initialEstimationLinkOrientation = new Quat4d();
      estimationLinkTransform3D.get(initialEstimationLinkOrientation);
      
      if (drcStateEstimator != null)
         drcStateEstimator.initializeEstimatorToActual(initialCoMPosition, initialEstimationLinkOrientation);
   }

   private static void updateRobot(Robot robot)
   {
      try
      {
         robot.update();
         robot.doDynamicsButDoNotIntegrate();
         robot.update();
      }
      catch (UnreasonableAccelerationException e)
      {
         throw new RuntimeException("UnreasonableAccelerationException");
      }
   }

   //   public static DesiredCoMAccelerationsFromRobotStealerController createAndAddDesiredCoMAccelerationFromRobotStealerController(ReferenceFrame estimationFrame,
   //         double controlDT, int simulationTicksPerControlTick, SDFRobot simulatedRobot, ArrayList<RobotControllerAndParameters> robotControllersAndParameters)
   //   {
   //      InverseDynamicsJointsFromSCSRobotGenerator generator = new InverseDynamicsJointsFromSCSRobotGenerator(simulatedRobot);
   //
   //      // TODO: Better way to get estimationJoint
   //      Joint estimationJoint = simulatedRobot.getRootJoints().get(0);
   //      double comAccelerationProcessNoiseStandardDeviation = Math.sqrt(1e-1);
   //      double angularAccelerationProcessNoiseStandardDeviation = Math.sqrt(1e-1);
   //
   //      DesiredCoMAccelerationsFromRobotStealerController desiredCoMAccelerationsFromRobotStealerController = new DesiredCoMAccelerationsFromRobotStealerController(
   //            estimationFrame, comAccelerationProcessNoiseStandardDeviation, angularAccelerationProcessNoiseStandardDeviation, generator, estimationJoint,
   //            controlDT);
   //
   //      RobotControllerAndParameters desiredCoMAccelerationsFromRobotStealerControllerAndParameters = new RobotControllerAndParameters(
   //            desiredCoMAccelerationsFromRobotStealerController, simulationTicksPerControlTick);
   //      robotControllersAndParameters.add(desiredCoMAccelerationsFromRobotStealerControllerAndParameters);
   //
   //      return desiredCoMAccelerationsFromRobotStealerController;
   //   }
   
   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return sim.getSimulationConstructionSet();
   }
   
   public void start(AutomaticSimulationRunner automaticSimulationRunner)
   {
      sim.start(automaticSimulationRunner);
   }

   public void dispose()
   {
      controller.dispose();
   }
   public SDFRobot getRobot()
   {
      return sim.getRobot();
   }
   
   @Deprecated
   public void addAdditionalYoVariableRegistriesToSCS(YoVariableRegistry registry)
   {
      sim.addAdditionalYoVariableRegistriesToSCS(registry);
   }


   public TimestampProvider getTimeStampProvider()
   {
      return timestampProvider;
   }

   
}
