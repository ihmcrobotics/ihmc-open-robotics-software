package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.GazeboStateCommunicator.GazeboRobot;
import us.ihmc.SdfLoader.SDFPerfectSimulatedOutputWriter;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonAvatarInterfaces.CommonAvatarEnvironmentInterface;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.controllers.NullLidarController;
import us.ihmc.commonWalkingControlModules.controllers.PIDLidarTorqueController;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCPerfectPoseEstimator;
import us.ihmc.projectM.R2Sim02.initialSetup.GuiInitialSetup;
import us.ihmc.projectM.R2Sim02.initialSetup.RobotInitialSetup;
import us.ihmc.projectM.R2Sim02.initialSetup.ScsInitialSetup;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedSensorHolderAndReaderFromRobotFactory;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorWithPorts;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.StateEstimatorErrorCalculatorController;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.net.ObjectCommunicator;

import com.yobotics.simulationconstructionset.GroundContactPoint;
import com.yobotics.simulationconstructionset.IMUMount;
import com.yobotics.simulationconstructionset.KinematicPoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.UnreasonableAccelerationException;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.gui.GUISetterUpperRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class DRCSimulationFactory
{


   // private static final SensorNoiseParameters sensorNoiseParamters = DRCSimulatedSensorNoiseParameters.createSensorNoiseParametersGazeboSDF();
   private static final SensorNoiseParameters sensorNoiseParamters = DRCSimulatedSensorNoiseParameters.createSensorNoiseParametersALittleNoise();
   // private static final SensorNoiseParameters sensorNoiseParamters = DRCSimulatedSensorNoiseParameters.createSensorNoiseParametersZeroNoise();
   private static final boolean COMPUTE_ESTIMATOR_ERROR = true;
   
   public static HumanoidRobotSimulation<SDFRobot> createSimulation(ControllerFactory controllerFactory,
         CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface, DRCRobotInterface robotInterface, RobotInitialSetup<SDFRobot> robotInitialSetup,
         ScsInitialSetup scsInitialSetup, GuiInitialSetup guiInitialSetup, ObjectCommunicator networkServer, ObjectCommunicator networkProccesorCommunicator)
   {
      GUISetterUpperRegistry guiSetterUpperRegistry = new GUISetterUpperRegistry();
      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();

      DRCRobotJointMap jointMap = robotInterface.getJointMap();

      double estimateDT = DRCConfigParameters.ESTIMATE_DT;
      double simulateDT = robotInterface.getSimulateDT();
      double controlDT = controllerFactory.getControlDT();
      int estimationTicksPerControlTick = (int) (estimateDT / simulateDT);
      

      SDFRobot simulatedRobot = robotInterface.getRobot();
      YoVariableRegistry registry = simulatedRobot.getRobotsYoVariableRegistry();
      simulatedRobot.setDynamicIntegrationMethod(scsInitialSetup.getDynamicIntegrationMethod());

      SDFPerfectSimulatedOutputWriter outputWriter = new SDFPerfectSimulatedOutputWriter(simulatedRobot);

      // TODO: Build LIDAR here
      LidarControllerInterface lidarControllerInterface;

      if (simulatedRobot instanceof GazeboRobot)
      {
         lidarControllerInterface = new NullLidarController();
      }
      else
      {
         lidarControllerInterface = new PIDLidarTorqueController(DRCConfigParameters.LIDAR_SPINDLE_VELOCITY, controlDT, registry);
      }

      //    SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMapForEstimator;
      //    scsToInverseDynamicsJointMapForEstimator = robotInterface.getSCSToInverseDynamicsJointMap();

      ArrayList<IMUMount> imuMounts = new ArrayList<IMUMount>();
      ArrayList<IMUMount> allIMUMounts = new ArrayList<IMUMount>();
      simulatedRobot.getIMUMounts(allIMUMounts);

      for (IMUMount imuMount : allIMUMounts)
      {
         // Only add the main one now. Not the head one.
         //          if (imuMount.getName().equals("head_imu_sensor")) imuMounts.add(imuMount);
         if (imuMount.getName().equals("imu_sensor"))
            imuMounts.add(imuMount);
      }

      ArrayList<KinematicPoint> positionPoints = new ArrayList<KinematicPoint>();
      ArrayList<KinematicPoint> velocityPoints = new ArrayList<KinematicPoint>();

      //     TODO: Get the velocity points in a better way!
      ArrayList<GroundContactPoint> allKinematicPoints = simulatedRobot.getAllGroundContactPoints();

      for (KinematicPoint kinematicPoint : allKinematicPoints)
      {
         //          System.out.println("kinematicPoint.getName() = " + kinematicPoint.getName());
         if (kinematicPoint.getName().equals("gc_l_leg_lax_0") || (kinematicPoint.getName().equals("gc_r_leg_lax_0")))
         {
            System.out.println("Adding VelocityPoint " + kinematicPoint.getName());

            positionPoints.add(kinematicPoint);
            velocityPoints.add(kinematicPoint);
         }
      }


      
      SimulatedSensorHolderAndReaderFromRobotFactory sensorReaderFactory = new SimulatedSensorHolderAndReaderFromRobotFactory(simulatedRobot,
            sensorNoiseParamters, controlDT, allIMUMounts, positionPoints, velocityPoints, registry);

      
      Vector3d gravity = new Vector3d();
      robotInterface.getRobot().getGravity(gravity);
      
      //TODO: Can only do this if have a simulation...
      Pair<Point3d, Quat4d> initialCoMPositionAndEstimationLinkOrientation = getInitialCoMPositionAndEstimationLinkOrientation(robotInitialSetup, simulatedRobot);
      
      DRCController robotController = new DRCController(initialCoMPositionAndEstimationLinkOrientation, robotInterface.getFullRobotModelFactory(), controllerFactory,
            robotInterface.getFootSwitches(), robotInterface.getWristForceSensors(), sensorReaderFactory, outputWriter,
            jointMap, lidarControllerInterface, gravity, estimateDT, controlDT, networkServer, dynamicGraphicObjectsListRegistry,
            guiSetterUpperRegistry, registry);

      final HumanoidRobotSimulation<SDFRobot> humanoidRobotSimulation = new HumanoidRobotSimulation<SDFRobot>(simulatedRobot, robotController,
            estimationTicksPerControlTick, commonAvatarEnvironmentInterface, simulatedRobot.getAllExternalForcePoints(), robotInitialSetup, scsInitialSetup,
            guiInitialSetup, guiSetterUpperRegistry, dynamicGraphicObjectsListRegistry);

      if (COMPUTE_ESTIMATOR_ERROR)
      {
         DRCStateEstimator drcStateEstimator = robotController.getDRCStateEstimator();
         StateEstimatorWithPorts stateEstimator = drcStateEstimator.getStateEstimator();

         StateEstimatorErrorCalculatorController stateEstimatorErrorCalculatorController = new StateEstimatorErrorCalculatorController(stateEstimator, simulatedRobot, simulatedRobot.getRootJoint());
         simulatedRobot.setController(stateEstimatorErrorCalculatorController, estimationTicksPerControlTick);
      }
      
      if (simulatedRobot instanceof GazeboRobot)
      {
         ((GazeboRobot) simulatedRobot).registerWithSCS(humanoidRobotSimulation.getSimulationConstructionSet());
      }

      if (networkProccesorCommunicator != null)
      {
         simulatedRobot.setController(new DRCPerfectPoseEstimator(simulatedRobot, networkProccesorCommunicator, robotInterface.getTimeStampProvider()), estimationTicksPerControlTick);
      }

      return humanoidRobotSimulation;
   }

   private static Pair<Point3d, Quat4d> getInitialCoMPositionAndEstimationLinkOrientation(RobotInitialSetup<SDFRobot> robotInitialSetup, SDFRobot simulatedRobot)
   {
      // The following is to get the initial CoM position from the robot. 
      // It is cheating for now, and we need to move to where the 
      // robot itself determines coordinates, and the sensors are all
      // in the robot-determined world coordinates..
      Point3d initialCoMPosition = new Point3d();
      robotInitialSetup.initializeRobot(simulatedRobot);
      updateRobot(simulatedRobot);
      
      simulatedRobot.computeCenterOfMass(initialCoMPosition);

      Quat4d initialEstimationLinkOrientation = simulatedRobot.getRootJointToWorldRotationQuaternion();
      return new Pair<Point3d, Quat4d>(initialCoMPosition, initialEstimationLinkOrientation);
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

}
