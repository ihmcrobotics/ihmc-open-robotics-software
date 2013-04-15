package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.GazeboStateCommunicator.GazeboRobot;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFNoisySimulatedSensorReaderAndWriter;
import us.ihmc.SdfLoader.SDFPerfectSimulatedSensorReaderAndWriter;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonAvatarInterfaces.CommonAvatarEnvironmentInterface;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.HandStatePacket;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.referenceFrames.ReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.CenterOfMassJacobianUpdater;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.ForceSensorInterface;
import us.ihmc.commonWalkingControlModules.sensors.TwistUpdater;
import us.ihmc.commonWalkingControlModules.visualizer.CommonInertiaElipsoidsVisualizer;
import us.ihmc.controlFlow.ControlFlowGraph;
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
import us.ihmc.sensorProcessing.stateEstimation.DesiredCoMAccelerationsFromRobotStealerController;
import us.ihmc.sensorProcessing.stateEstimation.DesiredCoMAndAngularAccelerationOutputPortsHolder;
import us.ihmc.sensorProcessing.stateEstimation.OrientationEstimator;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.ComposableStateEstimatorEvaluatorController;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.SensorAndEstimatorAssembler;
import us.ihmc.utilities.net.KryoObjectServer;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.TwistCalculator;

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
   private static final boolean SHOW_REFERENCE_FRAMES = false;
   public static boolean SHOW_INERTIA_ELLIPSOIDS = false;
   private static final boolean CREATE_STATE_ESTIMATOR = false;
   private static final boolean USE_STATE_ESTIMATOR = false;
   private static final boolean STEAL_DESIRED_COM_ACCELERATIONS_FROM_ROBOT = true;
   private static final boolean VISUALIZE_CONTROL_FLOW_GRAPH = false;

   public static HumanoidRobotSimulation<SDFRobot> createSimulation(ControllerFactory controllerFactory,
           CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface, DRCRobotInterface robotInterface, RobotInitialSetup<SDFRobot> robotInitialSetup,
           ScsInitialSetup scsInitialSetup, GuiInitialSetup guiInitialSetup, KryoObjectServer networkServer, ObjectCommunicator networkProccesorCommunicator)
   {
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

      DesiredCoMAndAngularAccelerationOutputPortsHolder desiredCoMAndAngularAccelerationOutputPortsHolder = null;
      if (STEAL_DESIRED_COM_ACCELERATIONS_FROM_ROBOT)
      {
         DesiredCoMAccelerationsFromRobotStealerController desiredCoMAccelerationsFromRobotStealerController =
            createAndAddDesiredCoMAccelerationFromRobotStealerController(controlDT, simulationTicksPerControlTick, simulatedRobot,
               robotControllersAndParameters);
         desiredCoMAndAngularAccelerationOutputPortsHolder = desiredCoMAccelerationsFromRobotStealerController;
      }

      FullInverseDynamicsStructure inverseDynamicsStructure = createInverseDynamicsStructure(fullRobotModelForController);

      TwistCalculator twistCalculator = inverseDynamicsStructure.getTwistCalculator();
      CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(fullRobotModelForController.getElevator());

      OrientationEstimator orientationEstimator = null;
      if (CREATE_STATE_ESTIMATOR)
      {
         // TODO: Get the Kinematic Points!
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
         simulatedRobot.getIMUMounts(imuMounts);
         ArrayList<KinematicPoint> velocityPoints = new ArrayList<KinematicPoint>();    
         //TODO: Get the velocity points
         // simulatedRobot.getVelocityPoints();

         SensorMapFromRobotFactory sensorMapFromRobotFactory = new SensorMapFromRobotFactory(scsToInverseDynamicsJointMapForEstimator, simulatedRobot,
                                                                  controlDT, imuMounts, velocityPoints, registry);
         SensorMap sensorMap = sensorMapFromRobotFactory.getSensorMap();

         orientationEstimator = createStateEstimator(sensorMap, inverseDynamicsStructureForEstimator, controlDT, simulationTicksPerControlTick, simulatedRobot,
                 registry, robotControllersAndParameters, desiredCoMAndAngularAccelerationOutputPortsHolder);

         if (!USE_STATE_ESTIMATOR)
         {
            orientationEstimator = null;
         }
      }

      CommonWalkingReferenceFrames referenceFramesForController = new ReferenceFrames(fullRobotModelForSimulation, jointMap, jointMap.getAnkleHeight());

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

      SDFPerfectSimulatedSensorReaderAndWriter sensorReaderAndOutputWriter;

      if (DRCConfigParameters.INTRODUCE_FILTERED_GAUSSIAN_POSITIONING_ERROR)
      {
         System.err.println("Using noisy fullrobotmodel");
         SDFNoisySimulatedSensorReaderAndWriter noisySimulatedSensorReaderAndWriter = new SDFNoisySimulatedSensorReaderAndWriter(simulatedRobot,
                                                                                         fullRobotModelForController, referenceFramesForController);
         noisySimulatedSensorReaderAndWriter.setNoiseFilterAlpha(DRCConfigParameters.NOISE_FILTER_ALPHA);
         noisySimulatedSensorReaderAndWriter.setPositionNoiseStd(DRCConfigParameters.POSITION_NOISE_STD);
         noisySimulatedSensorReaderAndWriter.setQuaternionNoiseStd(DRCConfigParameters.QUATERNION_NOISE_STD);
         sensorReaderAndOutputWriter = noisySimulatedSensorReaderAndWriter;
      }
      else
      {
         sensorReaderAndOutputWriter = new SDFPerfectSimulatedSensorReaderAndWriter(simulatedRobot, fullRobotModelForController, referenceFramesForController);
      }

      // PathTODO: Build LIDAR here
      OneDoFJoint lidarJoint;
      if (simulatedRobot instanceof GazeboRobot)
      {
         lidarJoint = null;
      }
      else
      {
         lidarJoint = fullRobotModelForController.getOneDoFJointByName(jointMap.getLidarJointName());
      }

      RobotController robotController = controllerFactory.getController(fullRobotModelForController, referenceFramesForController, controlDT,
                                           simulatedRobot.getYoTime(), dynamicGraphicObjectsListRegistry, guiSetterUpperRegistry, twistCalculator,
                                           centerOfMassJacobian, footSwitches, handControllers, lidarJoint);

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

      final ModularSensorProcessor sensorProcessor = createSensorProcessor(twistCalculator, centerOfMassJacobian);
      
//      if (!USE_STATE_ESTIMATOR)
      {
         modularRobotController.setRawSensorReader(sensorReaderAndOutputWriter);
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

      modularRobotController.setRawOutputWriter(sensorReaderAndOutputWriter);

      RobotControllerAndParameters modularRobotControllerAndParameters = new RobotControllerAndParameters(modularRobotController,
                                                                            simulationTicksPerControlTick);
      robotControllersAndParameters.add(modularRobotControllerAndParameters);

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


   public static OrientationEstimator createStateEstimator(SensorMap sensorMap, FullInverseDynamicsStructure inverseDynamicsStructure, double controlDT,
           int simulationTicksPerControlTick, SDFRobot simulatedRobot, YoVariableRegistry registry,
           ArrayList<RobotControllerAndParameters> robotControllersAndParameters,
           DesiredCoMAndAngularAccelerationOutputPortsHolder desiredCoMAndAngularAccelerationOutputPortsHolder)
   {
      Vector3d gravitationalAcceleration = new Vector3d();
      simulatedRobot.getGravity(gravitationalAcceleration);

      // The following few lines are what you need to do to get the state estimator working with a robot.
      // You also need to either add the controlFlowGraph to another one, or make sure to run it's startComputation method at the right time:
      SensorAndEstimatorAssembler sensorAndEstimatorAssembler = new SensorAndEstimatorAssembler(gravitationalAcceleration, inverseDynamicsStructure, controlDT,
                                                                   sensorMap, desiredCoMAndAngularAccelerationOutputPortsHolder, registry);

      ControlFlowGraph controlFlowGraph = sensorAndEstimatorAssembler.getControlFlowGraph();
      OrientationEstimator orientationEstimator = sensorAndEstimatorAssembler.getOrientationEstimator();

      if (VISUALIZE_CONTROL_FLOW_GRAPH)
      {
         controlFlowGraph.visualize();
      }
      
      Joint estimationJoint = simulatedRobot.getRootJoints().get(0);
      ComposableStateEstimatorEvaluatorController composableStateEstimatorEvaluatorController =
         new ComposableStateEstimatorEvaluatorController(controlFlowGraph, orientationEstimator, simulatedRobot, estimationJoint, controlDT, sensorMap,
            desiredCoMAndAngularAccelerationOutputPortsHolder);

      RobotControllerAndParameters humanoidStateEstimatorControllerAndParameters =
         new RobotControllerAndParameters(composableStateEstimatorEvaluatorController, simulationTicksPerControlTick);
      robotControllersAndParameters.add(humanoidStateEstimatorControllerAndParameters);

//    HumanoidStateEstimatorController humanoidStateEstimatorController = new HumanoidStateEstimatorController(simulationDT, desiredEstimatorDT);
//    RobotControllerAndParameters humanoidStateEstimatorControllerAndParameters = humanoidStateEstimatorController.createRobotControllerAndParameters();
//    robotControllersAndParameters.add(humanoidStateEstimatorControllerAndParameters);

      return orientationEstimator;
   }


   public static DesiredCoMAccelerationsFromRobotStealerController createAndAddDesiredCoMAccelerationFromRobotStealerController(double controlDT,
           int simulationTicksPerControlTick, SDFRobot simulatedRobot, ArrayList<RobotControllerAndParameters> robotControllersAndParameters)
   {
      InverseDynamicsJointsFromSCSRobotGenerator generator = new InverseDynamicsJointsFromSCSRobotGenerator(simulatedRobot);

      // TODO: Better way to get estimationJoint
      Joint estimationJoint = simulatedRobot.getRootJoints().get(0);
      DesiredCoMAccelerationsFromRobotStealerController desiredCoMAccelerationsFromRobotStealerController =
         new DesiredCoMAccelerationsFromRobotStealerController(generator, estimationJoint, controlDT);

      RobotControllerAndParameters desiredCoMAccelerationsFromRobotStealerControllerAndParameters =
         new RobotControllerAndParameters(desiredCoMAccelerationsFromRobotStealerController, simulationTicksPerControlTick);
      robotControllersAndParameters.add(desiredCoMAccelerationsFromRobotStealerControllerAndParameters);

      return desiredCoMAccelerationsFromRobotStealerController;
   }


   private static ModularSensorProcessor createSensorProcessor(TwistCalculator twistCalculator, CenterOfMassJacobian centerOfMassJacobian)
   {
      ModularSensorProcessor modularSensorProcessor = new ModularSensorProcessor("ModularSensorProcessor", "");
      modularSensorProcessor.addSensorProcessor(new TwistUpdater(twistCalculator));
      modularSensorProcessor.addSensorProcessor(new CenterOfMassJacobianUpdater(centerOfMassJacobian));

      return modularSensorProcessor;
   }
}
