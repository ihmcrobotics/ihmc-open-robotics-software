package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.GazeboStateCommunicator.GazeboRobot;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFNoisySimulatedSensorReader;
import us.ihmc.SdfLoader.SDFPerfectSimulatedOutputWriter;
import us.ihmc.SdfLoader.SDFPerfectSimulatedSensorReader;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonAvatarInterfaces.CommonAvatarEnvironmentInterface;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.controllers.NullLidarController;
import us.ihmc.commonWalkingControlModules.controllers.PIDLidarTorqueController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.HandStatePacket;
import us.ihmc.commonWalkingControlModules.referenceFrames.ReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.CenterOfMassJacobianUpdater;
import us.ihmc.commonWalkingControlModules.sensors.CommonWalkingReferenceFramesUpdater;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.ForceSensorInterface;
import us.ihmc.commonWalkingControlModules.sensors.TwistUpdater;
import us.ihmc.commonWalkingControlModules.visualizer.CommonInertiaElipsoidsVisualizer;
import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.darpaRoboticsChallenge.controllers.ConstrainedCenterOfMassJacobianEvaluator;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.handControl.SandiaHandModel;
import us.ihmc.darpaRoboticsChallenge.handControl.SimulatedUnderactuatedSandiaHandController;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCPerfectPoseEstimator;
import us.ihmc.projectM.R2Sim02.initialSetup.GuiInitialSetup;
import us.ihmc.projectM.R2Sim02.initialSetup.RobotInitialSetup;
import us.ihmc.projectM.R2Sim02.initialSetup.ScsInitialSetup;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.simulatedSensors.InverseDynamicsJointsFromSCSRobotGenerator;
import us.ihmc.sensorProcessing.simulatedSensors.SCSToInverseDynamicsJointMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorMapFromRobotFactory;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.DesiredCoMAccelerationsFromRobotStealerController;
import us.ihmc.sensorProcessing.stateEstimation.DesiredCoMAndAngularAccelerationDataSource;
import us.ihmc.sensorProcessing.stateEstimation.OrientationEstimatorWithPorts;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.ComposableStateEstimatorEvaluatorController;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.SensorAndEstimatorAssembler;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.KryoObjectServer;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.GroundContactPoint;
import com.yobotics.simulationconstructionset.IMUMount;
import com.yobotics.simulationconstructionset.InverseDynamicsMechanismReferenceFrameVisualizer;
import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.KinematicPoint;
import com.yobotics.simulationconstructionset.RobotControllerAndParameters;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.gui.GUISetterUpperRegistry;
import com.yobotics.simulationconstructionset.robotController.AbstractModularRobotController;
import com.yobotics.simulationconstructionset.robotController.DelayedThreadedModularRobotController;
import com.yobotics.simulationconstructionset.robotController.ModularRobotController;
import com.yobotics.simulationconstructionset.robotController.ModularSensorProcessor;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class DRCSimulationFactory
{
   public static boolean SHOW_INERTIA_ELLIPSOIDS = false;
   private static final boolean SHOW_REFERENCE_FRAMES = false;
   private static final boolean CREATE_DYNAMICALLY_CONSISTENT_NULLSPACE_EVALUATOR = false;
   private static final boolean CREATE_STATE_ESTIMATOR = false;
   private static final boolean USE_STATE_ESTIMATOR = false;
   private static final boolean STEAL_DESIRED_COM_ACCELERATIONS_FROM_ROBOT = false;
   private static final boolean VISUALIZE_CONTROL_FLOW_GRAPH = false;

   public static HumanoidRobotSimulation<SDFRobot> createSimulation(ControllerFactory controllerFactory,
           CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface, DRCRobotInterface robotInterface, RobotInitialSetup<SDFRobot> robotInitialSetup,
           ScsInitialSetup scsInitialSetup, GuiInitialSetup guiInitialSetup, KryoObjectServer networkServer, ObjectCommunicator networkProccesorCommunicator)
   {
      SensorNoiseParameters sensorNoiseParamters = DRCSimulatedSensorNoiseParameters.createSensorNoiseParametersGazeboSDF();
//    SensorNoiseParameters sensorNoiseParamters = DRCSimulatedSensorNoiseParameters.createSensorNoiseParametersALittleNoise();
//    SensorNoiseParameters sensorNoiseParamters = DRCSimulatedSensorNoiseParameters.createSensorNoiseParametersZeroNoise();

      GUISetterUpperRegistry guiSetterUpperRegistry = new GUISetterUpperRegistry();
      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();

      DRCRobotJointMap jointMap = robotInterface.getJointMap();
      SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap = robotInterface.getSCSToInverseDynamicsJointMap();

      double simulateDT = robotInterface.getSimulateDT();
      double controlDT = controllerFactory.getControlDT();
      int simulationTicksPerControlTick = (int) (controlDT / simulateDT);

      SDFRobot simulatedRobot = robotInterface.getRobot();
      YoVariableRegistry registry = simulatedRobot.getRobotsYoVariableRegistry();
      simulatedRobot.setDynamicIntegrationMethod(scsInitialSetup.getDynamicIntegrationMethod());

      SDFFullRobotModel fullRobotModelForSimulation = robotInterface.getFullRobotModel();
      SDFFullRobotModel fullRobotModelForController = fullRobotModelForSimulation;

      ArrayList<RobotControllerAndParameters> robotControllersAndParameters = new ArrayList<RobotControllerAndParameters>();
      FullInverseDynamicsStructure inverseDynamicsStructure = createInverseDynamicsStructure(fullRobotModelForController);

      DesiredCoMAccelerationsFromRobotStealerController desiredCoMAccelerationsFromRobotStealerController = null;
      if (STEAL_DESIRED_COM_ACCELERATIONS_FROM_ROBOT)
      {
         desiredCoMAccelerationsFromRobotStealerController = createAndAddDesiredCoMAccelerationFromRobotStealerController(
               inverseDynamicsStructure.getEstimationFrame(),
               sensorNoiseParamters, controlDT, 
               simulationTicksPerControlTick, simulatedRobot,
               robotControllersAndParameters);
      }


      TwistCalculator twistCalculator = inverseDynamicsStructure.getTwistCalculator();
      CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(fullRobotModelForController.getElevator());

      ReferenceFrames referenceFramesForController = new ReferenceFrames(fullRobotModelForSimulation, jointMap, jointMap.getAnkleHeight());

      OrientationEstimatorWithPorts orientationEstimator = null;
      if (CREATE_STATE_ESTIMATOR)
      {
         SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMapForEstimator;
         FullInverseDynamicsStructure inverseDynamicsStructureForEstimator;

         if (USE_STATE_ESTIMATOR)
         {
            scsToInverseDynamicsJointMapForEstimator = scsToInverseDynamicsJointMap;
            inverseDynamicsStructureForEstimator = inverseDynamicsStructure;   
         }
         else
         {
            InverseDynamicsJointsFromSCSRobotGenerator generator = new InverseDynamicsJointsFromSCSRobotGenerator(simulatedRobot);
            scsToInverseDynamicsJointMapForEstimator = generator.getSCSToInverseDynamicsJointMap();
            inverseDynamicsStructureForEstimator = generator.getInverseDynamicsStructure();
         }

         ArrayList<IMUMount> imuMounts = new ArrayList<IMUMount>();
         ArrayList<IMUMount> allIMUMounts = new ArrayList<IMUMount>();
         simulatedRobot.getIMUMounts(allIMUMounts);
         
         for (IMUMount imuMount : allIMUMounts)
         {
//            System.out.println("IMUMount: " + imuMount.getName());
            // Only add the main one now. Not the head one.
//            if (imuMount.getName().equals("head_imu_sensor")) imuMounts.add(imuMount);
            if (imuMount.getName().equals("imu_sensor")) imuMounts.add(imuMount);
         }
         
         
         ArrayList<KinematicPoint> velocityPoints = new ArrayList<KinematicPoint>();    
         //TODO: Get the velocity points
         ArrayList<GroundContactPoint> allKinematicPoints = simulatedRobot.getAllGroundContactPoints();
         
         for (KinematicPoint kinematicPoint : allKinematicPoints)
         {
//            System.out.println("kinematicPoint.getName() = " + kinematicPoint.getName());
            if (kinematicPoint.getName().equals("gc_l_leg_lax_0") || (kinematicPoint.getName().equals("gc_r_leg_lax_0")))
            {
               System.out.println("Adding VelocityPoint " + kinematicPoint.getName());

               velocityPoints.add(kinematicPoint);
            }
         }
         
         SensorMapFromRobotFactory sensorMapFromRobotFactory = new SensorMapFromRobotFactory(scsToInverseDynamicsJointMapForEstimator, 
               simulatedRobot, sensorNoiseParamters,
               controlDT, imuMounts, velocityPoints, registry);
         SensorMap sensorMap = sensorMapFromRobotFactory.getSensorMap();

         orientationEstimator = createStateEstimator(sensorMap, inverseDynamicsStructureForEstimator, controlDT, simulationTicksPerControlTick, simulatedRobot,
                 registry, robotControllersAndParameters);
              
         ControlFlowGraph controlFlowGraph = orientationEstimator.getControlFlowGraph();

         Joint estimationJoint = simulatedRobot.getRootJoints().get(0);
         ComposableStateEstimatorEvaluatorController composableStateEstimatorEvaluatorController =
            new ComposableStateEstimatorEvaluatorController(controlFlowGraph, orientationEstimator, simulatedRobot, estimationJoint, controlDT, sensorMap);

         RobotControllerAndParameters humanoidStateEstimatorControllerAndParameters =
            new RobotControllerAndParameters(composableStateEstimatorEvaluatorController, simulationTicksPerControlTick);
         robotControllersAndParameters.add(humanoidStateEstimatorControllerAndParameters);
         
         
         
         FramePoint estimatedCoMPosition = new FramePoint(orientationEstimator.getEstimatedCoMPosition());
         estimatedCoMPosition.setZ(estimatedCoMPosition.getZ() + 1.2);
         orientationEstimator.setEstimatedCoMPosition(estimatedCoMPosition);
         
         if (!USE_STATE_ESTIMATOR)
         {
            orientationEstimator = null;
         }
      }

      SideDependentList<FootSwitchInterface> footSwitches = robotInterface.getFootSwitches();

      SideDependentList<HandControllerInterface> handControllers = null;
      if (jointMap.getSelectedModel() == DRCRobotModel.ATLAS_SANDIA_HANDS)
      {
         handControllers = new SideDependentList<HandControllerInterface>();

         for (RobotSide robotSide : RobotSide.values())
         {
            ForceSensorInterface wristForceSensor = robotInterface.getWristForceSensor(robotSide);

            SandiaHandModel handModel = new SandiaHandModel(fullRobotModelForController, wristForceSensor, robotSide);
            SimulatedUnderactuatedSandiaHandController simulatedUnderactuatedSandiaHandController =
               new SimulatedUnderactuatedSandiaHandController(simulatedRobot.getYoTime(), handModel, controlDT);
            handControllers.put(robotSide, simulatedUnderactuatedSandiaHandController);

            if (networkServer != null)
            {
               networkServer.attachListener(HandStatePacket.class, simulatedUnderactuatedSandiaHandController);
            }

         }
      }

      SDFPerfectSimulatedSensorReader sensorReader;

      if (DRCConfigParameters.INTRODUCE_FILTERED_GAUSSIAN_POSITIONING_ERROR)
      {
         System.err.println("Using noisy fullrobotmodel");
         SDFNoisySimulatedSensorReader noisySimulatedSensorReaderAndWriter = new SDFNoisySimulatedSensorReader(simulatedRobot,
                                                                                         fullRobotModelForController, referenceFramesForController);
         noisySimulatedSensorReaderAndWriter.setNoiseFilterAlpha(DRCConfigParameters.NOISE_FILTER_ALPHA);
         noisySimulatedSensorReaderAndWriter.setPositionNoiseStd(DRCConfigParameters.POSITION_NOISE_STD);
         noisySimulatedSensorReaderAndWriter.setQuaternionNoiseStd(DRCConfigParameters.QUATERNION_NOISE_STD);
         sensorReader = noisySimulatedSensorReaderAndWriter;
      }
      else
      {
         sensorReader = new SDFPerfectSimulatedSensorReader(simulatedRobot, fullRobotModelForController, referenceFramesForController);
      }

      SDFPerfectSimulatedOutputWriter outputWriter = new SDFPerfectSimulatedOutputWriter(simulatedRobot, fullRobotModelForController);

      // PathTODO: Build LIDAR here
      LidarControllerInterface lidarControllerInterface;
      OneDoFJoint lidarJoint = fullRobotModelForController.getOneDoFJointByName(jointMap.getLidarJointName()); 
      
      if (simulatedRobot instanceof GazeboRobot)
      {
         lidarControllerInterface = new NullLidarController(lidarJoint);
      }
      else
      {
         lidarControllerInterface = new PIDLidarTorqueController(lidarJoint, DRCConfigParameters.LIDAR_SPINDLE_VELOCITY, controlDT, registry);
      }

      //TODO: Get rid of this type cast.
      MomentumBasedController robotController = (MomentumBasedController) controllerFactory.getController(inverseDynamicsStructure.getEstimationLink(),
            inverseDynamicsStructure.getEstimationFrame(), fullRobotModelForController, referenceFramesForController, controlDT,
            simulatedRobot.getYoTime(), dynamicGraphicObjectsListRegistry, guiSetterUpperRegistry, twistCalculator,
            centerOfMassJacobian, footSwitches, handControllers, lidarControllerInterface);

      
      if (CREATE_STATE_ESTIMATOR)
      {
         RigidBody estimationLink = inverseDynamicsStructure.getEstimationLink();
         ReferenceFrame estimationFrame = inverseDynamicsStructure.getEstimationFrame();
         DesiredCoMAndAngularAccelerationDataSource desiredCoMAndAngularAccelerationDataSource = new DesiredCoMAndAngularAccelerationDataSource(estimationLink, estimationFrame);
         
         if (STEAL_DESIRED_COM_ACCELERATIONS_FROM_ROBOT)
         {
            desiredCoMAccelerationsFromRobotStealerController.attachDesiredCoMAndAngularAccelerationDataSource(desiredCoMAndAngularAccelerationDataSource);
         }
         else
         {
            robotController.attachDesiredCoMAndAngularAccelerationDataSource(desiredCoMAndAngularAccelerationDataSource);
         }
         

         ControlFlowGraph controlFlowGraph = orientationEstimator.getControlFlowGraph();

         desiredCoMAndAngularAccelerationDataSource.connectDesiredAccelerationPorts(controlFlowGraph, orientationEstimator);

//         SensorAndEstimatorAssembler.connectDesiredAccelerationPorts(controlFlowGraph, orientationEstimator,
//               desiredCoMAndAngularAccelerationOutputPortsHolder);

         controlFlowGraph.initializeAfterConnections();

         if (VISUALIZE_CONTROL_FLOW_GRAPH)
         {
            controlFlowGraph.visualize();
         }
      }
 
      
      AbstractModularRobotController modularRobotController;


      if (robotInterface.simulateDelay())
      {
         System.err.println("INFO: Creating controller with simulated delay");
         modularRobotController = new DelayedThreadedModularRobotController("ModularRobotController");
      }
      else
      {
         modularRobotController = new ModularRobotController("ModularRobotController");
      }

      final ModularSensorProcessor sensorProcessor = createSensorProcessor(twistCalculator, centerOfMassJacobian, referenceFramesForController);
      
      if (!USE_STATE_ESTIMATOR)
      {
         modularRobotController.setRawSensorReader(sensorReader);
      }
      
      modularRobotController.setSensorProcessor(sensorProcessor);
      modularRobotController.addRobotController(robotController);

      if (SHOW_INERTIA_ELLIPSOIDS)
      {
         modularRobotController.addRobotController(new CommonInertiaElipsoidsVisualizer(fullRobotModelForSimulation.getElevator(),
                 dynamicGraphicObjectsListRegistry));
      }

      if (SHOW_REFERENCE_FRAMES)
      {
         modularRobotController.addRobotController(new InverseDynamicsMechanismReferenceFrameVisualizer(fullRobotModelForSimulation.getElevator(),
                 dynamicGraphicObjectsListRegistry, 0.1));
      }

      modularRobotController.setRawOutputWriter(outputWriter);

      RobotControllerAndParameters modularRobotControllerAndParameters = new RobotControllerAndParameters(modularRobotController,
                                                                            simulationTicksPerControlTick);
      robotControllersAndParameters.add(modularRobotControllerAndParameters);

      if (CREATE_DYNAMICALLY_CONSISTENT_NULLSPACE_EVALUATOR)
      {
         RobotController dynamicallyConsistentNullspaceEvaluator = new ConstrainedCenterOfMassJacobianEvaluator(fullRobotModelForController);
         robotControllersAndParameters.add(new RobotControllerAndParameters(dynamicallyConsistentNullspaceEvaluator, simulationTicksPerControlTick));
      }

      final HumanoidRobotSimulation<SDFRobot> humanoidRobotSimulation = new HumanoidRobotSimulation<SDFRobot>(simulatedRobot, robotControllersAndParameters,
                                                                           fullRobotModelForSimulation, commonAvatarEnvironmentInterface,
                                                                           simulatedRobot.getAllExternalForcePoints(), robotInitialSetup, scsInitialSetup,
                                                                           guiInitialSetup, guiSetterUpperRegistry, dynamicGraphicObjectsListRegistry);

      if (simulatedRobot instanceof GazeboRobot)
      {
         ((GazeboRobot) simulatedRobot).registerWithSCS(humanoidRobotSimulation.getSimulationConstructionSet());
      }


      if (networkProccesorCommunicator != null)
      {
         sensorProcessor.addSensorProcessor(new DRCPerfectPoseEstimator(simulatedRobot, networkProccesorCommunicator, robotInterface.getTimeStampProvider()));
      }

      return humanoidRobotSimulation;
   }


   private static FullInverseDynamicsStructure createInverseDynamicsStructure(SDFFullRobotModel fullRobotModelForController)
   {
      RigidBody elevator = fullRobotModelForController.getElevator();
      SixDoFJoint rootInverseDynamicsJoint = fullRobotModelForController.getRootJoint();
      RigidBody estimationLink = rootInverseDynamicsJoint.getSuccessor();

      FullInverseDynamicsStructure inverseDynamicsStructure = new FullInverseDynamicsStructure(elevator, estimationLink, rootInverseDynamicsJoint);

      return inverseDynamicsStructure;
   }


   public static OrientationEstimatorWithPorts createStateEstimator(SensorMap sensorMap, FullInverseDynamicsStructure inverseDynamicsStructure, double controlDT,
           int simulationTicksPerControlTick, SDFRobot simulatedRobot, YoVariableRegistry registry,
           ArrayList<RobotControllerAndParameters> robotControllersAndParameters)
   {
      Vector3d gravitationalAcceleration = new Vector3d();
      simulatedRobot.getGravity(gravitationalAcceleration);

      // The following few lines are what you need to do to get the state estimator working with a robot.
      // You also need to either add the controlFlowGraph to another one, or make sure to run it's startComputation method at the right time:
      SensorNoiseParameters sensorNoiseParametersForEstimator = DRCSimulatedSensorNoiseParameters.createNoiseParametersForEstimatorBasedOnGazeboSDF();
      
      SensorAndEstimatorAssembler sensorAndEstimatorAssembler = new SensorAndEstimatorAssembler(sensorNoiseParametersForEstimator, gravitationalAcceleration, inverseDynamicsStructure, controlDT,
                                                                   sensorMap, registry);

      OrientationEstimatorWithPorts orientationEstimator = sensorAndEstimatorAssembler.getOrientationEstimator();
      
      return orientationEstimator;
   }


   public static DesiredCoMAccelerationsFromRobotStealerController createAndAddDesiredCoMAccelerationFromRobotStealerController(
         ReferenceFrame estimationFrame,
         SensorNoiseParameters simulatedSensorNoiseParameters, 
         double controlDT,
         int simulationTicksPerControlTick, SDFRobot simulatedRobot, ArrayList<RobotControllerAndParameters> robotControllersAndParameters)
   {
      InverseDynamicsJointsFromSCSRobotGenerator generator = new InverseDynamicsJointsFromSCSRobotGenerator(simulatedRobot);

      // TODO: Better way to get estimationJoint
      Joint estimationJoint = simulatedRobot.getRootJoints().get(0);
      DesiredCoMAccelerationsFromRobotStealerController desiredCoMAccelerationsFromRobotStealerController =
         new DesiredCoMAccelerationsFromRobotStealerController(estimationFrame, simulatedSensorNoiseParameters, generator, estimationJoint, controlDT);

      RobotControllerAndParameters desiredCoMAccelerationsFromRobotStealerControllerAndParameters =
         new RobotControllerAndParameters(desiredCoMAccelerationsFromRobotStealerController, simulationTicksPerControlTick);
      robotControllersAndParameters.add(desiredCoMAccelerationsFromRobotStealerControllerAndParameters);

      return desiredCoMAccelerationsFromRobotStealerController;
   }


   private static ModularSensorProcessor createSensorProcessor(TwistCalculator twistCalculator, CenterOfMassJacobian centerOfMassJacobian,       
         ReferenceFrames referenceFrames)
   {
      ModularSensorProcessor modularSensorProcessor = new ModularSensorProcessor("ModularSensorProcessor", "");
      modularSensorProcessor.addSensorProcessor(new TwistUpdater(twistCalculator));
      modularSensorProcessor.addSensorProcessor(new CenterOfMassJacobianUpdater(centerOfMassJacobian));
      modularSensorProcessor.addSensorProcessor(new CommonWalkingReferenceFramesUpdater(referenceFrames));
      
      return modularSensorProcessor;
   }
}
