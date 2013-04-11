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
import us.ihmc.sensorProcessing.simulatedSensors.SensorMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorMapFromRobotFactory;
import us.ihmc.sensorProcessing.stateEstimation.DesiredCoMAccelerationsFromRobotStealerController;
import us.ihmc.sensorProcessing.stateEstimation.DesiredCoMAndAngularAccelerationOutputPortsHolder;
import us.ihmc.sensorProcessing.stateEstimation.OrientationEstimator;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.ComposableStateEstimatorEvaluatorController;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.SensorAndEstimatorAssembler;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.StateEstimatorEvaluatorFullRobotModel;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
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
   private static final boolean CREATE_STATE_ESTIMATOR = true;
   
   public static HumanoidRobotSimulation<SDFRobot> createSimulation(ControllerFactory controllerFactory,
         CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface, DRCRobotInterface robotInterface, RobotInitialSetup<SDFRobot> robotInitialSetup,
         ScsInitialSetup scsInitialSetup, GuiInitialSetup guiInitialSetup, KryoObjectServer networkServer, ObjectCommunicator networkProccesorCommunicator)
   {
      GUISetterUpperRegistry guiSetterUpperRegistry = new GUISetterUpperRegistry();
      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();

      DRCRobotJointMap jointMap = robotInterface.getJointMap();

      double simulateDT = robotInterface.getSimulateDT();
      double controlDT = controllerFactory.getControlDT();
      int simulationTicksPerControlTick = (int) (controlDT / simulateDT);

      SDFRobot simulatedRobot = robotInterface.getRobot();
      YoVariableRegistry registry = simulatedRobot.getRobotsYoVariableRegistry();


      simulatedRobot.setDynamicIntegrationMethod(scsInitialSetup.getDynamicIntegrationMethod());

      SDFFullRobotModel fullRobotModelForSimulation = robotInterface.getFullRobotModel();


      SDFFullRobotModel fullRobotModelForController = fullRobotModelForSimulation;
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
            SimulatedUnderactuatedSandiaHandController simulatedUnderactuatedSandiaHandController = new SimulatedUnderactuatedSandiaHandController(simulatedRobot.getYoTime(), handModel, controlDT);
            handControllers.put(robotSide, simulatedUnderactuatedSandiaHandController);
            if (networkServer != null)
            {
               networkServer.attachListener(HandStatePacket.class, simulatedUnderactuatedSandiaHandController);
            }

         }
      }

      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), fullRobotModelForController.getElevator());
      CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(fullRobotModelForController.getElevator());

      
      SDFPerfectSimulatedSensorReaderAndWriter sensorReaderAndOutputWriter;
      
      if(DRCConfigParameters.INTRODUCE_FILTERED_GAUSSIAN_POSITIONING_ERROR)
      {
         System.err.println("Using noisy fullrobotmodel");
         SDFNoisySimulatedSensorReaderAndWriter noisySimulatedSensorReaderAndWriter = new SDFNoisySimulatedSensorReaderAndWriter(simulatedRobot, fullRobotModelForController, referenceFramesForController);
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
            simulatedRobot.getYoTime(), dynamicGraphicObjectsListRegistry, guiSetterUpperRegistry, twistCalculator, centerOfMassJacobian, footSwitches,
            handControllers, lidarJoint);

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
      modularRobotController.setRawSensorReader(sensorReaderAndOutputWriter);
      modularRobotController.setSensorProcessor(sensorProcessor);
      modularRobotController.addRobotController(robotController);

      if (SHOW_INERTIA_ELLIPSOIDS)
      {
         modularRobotController.addRobotController(new CommonInertiaElipsoidsVisualizer(fullRobotModelForSimulation.getElevator(), dynamicGraphicObjectsListRegistry));
      }

      if (SHOW_REFERENCE_FRAMES)
      {
         modularRobotController.addRobotController(new InverseDynamicsMechanismReferenceFrameVisualizer(fullRobotModelForSimulation.getElevator(), dynamicGraphicObjectsListRegistry, 0.1));
      }

      modularRobotController.setRawOutputWriter(sensorReaderAndOutputWriter);

      RobotControllerAndParameters modularRobotControllerAndParameters = new RobotControllerAndParameters(modularRobotController, simulationTicksPerControlTick);
      
      ArrayList<RobotControllerAndParameters> robotControllersAndParameters = new ArrayList<RobotControllerAndParameters>();
      robotControllersAndParameters.add(modularRobotControllerAndParameters);
      
      if (CREATE_STATE_ESTIMATOR)
      {
         double simulationDT = scsInitialSetup.getDT();
         double desiredEstimatorDT = 0.001;
         
         //TODO: Get the IMU Mounts and Kinematic Points!
         InverseDynamicsJointsFromSCSRobotGenerator generator = new InverseDynamicsJointsFromSCSRobotGenerator(simulatedRobot);
         
         ArrayList<IMUMount> imuMounts = new ArrayList<IMUMount>();
         simulatedRobot.getIMUMounts(imuMounts);
         ArrayList<KinematicPoint> velocityPoints = new ArrayList<KinematicPoint>(); //simulatedRobot.getVelocityPoints();
         
         StateEstimatorEvaluatorFullRobotModel perfectFullRobotModel = new StateEstimatorEvaluatorFullRobotModel(generator, simulatedRobot, imuMounts,
               velocityPoints);
         //TODO: Better way to get estimationJoint
         Joint estimationJoint = simulatedRobot.getRootJoints().get(0);
               
         SensorMapFromRobotFactory sensorMapFromRobotFactory = new SensorMapFromRobotFactory(generator, simulatedRobot, controlDT, imuMounts,
               velocityPoints, registry);
         SensorMap sensorMap = sensorMapFromRobotFactory.getSensorMap();

         DesiredCoMAccelerationsFromRobotStealerController desiredCoMAccelerationsFromRobotStealerController =
            new DesiredCoMAccelerationsFromRobotStealerController(perfectFullRobotModel, generator, estimationJoint, controlDT);

         RobotControllerAndParameters desiredCoMAccelerationsFromRobotStealerControllerAndParameters = new RobotControllerAndParameters(desiredCoMAccelerationsFromRobotStealerController, simulationTicksPerControlTick);
         robotControllersAndParameters.add(desiredCoMAccelerationsFromRobotStealerControllerAndParameters);
         
         RigidBody estimationLink = perfectFullRobotModel.getRootBody();
         RigidBody elevator = perfectFullRobotModel.getElevator();
         SixDoFJoint rootInverseDynamicsJoint = perfectFullRobotModel.getRootInverseDynamicsJoint();

         FullInverseDynamicsStructure inverseDynamicsStructure = new FullInverseDynamicsStructure(elevator, estimationLink, rootInverseDynamicsJoint);

         Vector3d gravitationalAcceleration = new Vector3d();
         simulatedRobot.getGravity(gravitationalAcceleration);
         DesiredCoMAndAngularAccelerationOutputPortsHolder desiredCoMAndAngularAccelerationOutputPortsHolder = desiredCoMAccelerationsFromRobotStealerController;
        
         // The following few lines are what you need to do to get the state estimator working with a robot.
         // You also need to either add the controlFlowGraph to another one, or make sure to run it's startComputation method at the right time:
         SensorAndEstimatorAssembler sensorAndEstimatorAssembler = new SensorAndEstimatorAssembler(gravitationalAcceleration, inverseDynamicsStructure, controlDT,
                                                                      sensorMap, desiredCoMAndAngularAccelerationOutputPortsHolder, registry);

         ControlFlowGraph controlFlowGraph = sensorAndEstimatorAssembler.getControlFlowGraph();
         OrientationEstimator orientationEstimator = sensorAndEstimatorAssembler.getOrientationEstimator();
         
         ComposableStateEstimatorEvaluatorController composableStateEstimatorEvaluatorController = new ComposableStateEstimatorEvaluatorController(controlFlowGraph, orientationEstimator, 
               simulatedRobot, 
               estimationJoint, controlDT, 
               sensorMap, desiredCoMAndAngularAccelerationOutputPortsHolder);
         
         RobotControllerAndParameters humanoidStateEstimatorControllerAndParameters = new RobotControllerAndParameters(composableStateEstimatorEvaluatorController, simulationTicksPerControlTick);
         robotControllersAndParameters.add(humanoidStateEstimatorControllerAndParameters);

//         HumanoidStateEstimatorController humanoidStateEstimatorController = new HumanoidStateEstimatorController(simulationDT, desiredEstimatorDT);
//         RobotControllerAndParameters humanoidStateEstimatorControllerAndParameters = humanoidStateEstimatorController.createRobotControllerAndParameters();
//         robotControllersAndParameters.add(humanoidStateEstimatorControllerAndParameters);
      }
      
      final HumanoidRobotSimulation<SDFRobot> humanoidRobotSimulation = new HumanoidRobotSimulation<SDFRobot>(simulatedRobot, robotControllersAndParameters, fullRobotModelForSimulation, commonAvatarEnvironmentInterface, simulatedRobot.getAllExternalForcePoints(), robotInitialSetup, scsInitialSetup, guiInitialSetup, guiSetterUpperRegistry, dynamicGraphicObjectsListRegistry);

      if (simulatedRobot instanceof GazeboRobot)
      {
         ((GazeboRobot) simulatedRobot).registerWithSCS(humanoidRobotSimulation.getSimulationConstructionSet());
      }
      
      
      if(networkProccesorCommunicator != null)
      {
         sensorProcessor.addSensorProcessor(new DRCPerfectPoseEstimator(simulatedRobot, networkProccesorCommunicator, robotInterface.getTimeStampProvider()));
      }
      return humanoidRobotSimulation;
   }


   private static ModularSensorProcessor createSensorProcessor(TwistCalculator twistCalculator, CenterOfMassJacobian centerOfMassJacobian)
   {
      ModularSensorProcessor modularSensorProcessor = new ModularSensorProcessor("ModularSensorProcessor", "");
      modularSensorProcessor.addSensorProcessor(new TwistUpdater(twistCalculator));
      modularSensorProcessor.addSensorProcessor(new CenterOfMassJacobianUpdater(centerOfMassJacobian));
      return modularSensorProcessor;
   }
}
