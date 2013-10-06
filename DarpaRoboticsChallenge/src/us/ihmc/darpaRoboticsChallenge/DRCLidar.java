package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.net.TimestampProvider;
import us.ihmc.utilities.lidar.polarLidar.geometry.PolarLidarScanParameters;

import com.yobotics.simulationconstructionset.OneDegreeOfFreedomJoint;
import com.yobotics.simulationconstructionset.simulatedSensors.FastPolarRayCastLIDAR;
import com.yobotics.simulationconstructionset.simulatedSensors.SimulatedLIDARSensorNoiseParameters;
import com.yobotics.simulationconstructionset.simulatedSensors.SimulatedLIDARSensorUpdateParameters;

public class DRCLidar
{
   private static final boolean PRINT_ALL_POSSIBLE_JOINT_NAMES = false;

   static
   {
      if (DRCConfigParameters.STREAM_POLAR_LIDAR)
         System.out.println("DRCLidar: PolarLidar is ON. This lidar passes less data, and also includes the transforms from when the data was produced.");
   }

   public static FastPolarRayCastLIDAR getLidarSensor(SDFRobot robot)
   {
      ArrayList<FastPolarRayCastLIDAR> lidarSensors = robot.getSensors(FastPolarRayCastLIDAR.class);
      if (lidarSensors.size() == 0)
      {
         System.err.println("DRCLidar: No LIDAR units found on SDF Robot.");
         return null;
      }
      else
      {
         if (lidarSensors.size() >= 2)
            System.err.println("DRCLidar: Only one LIDAR unit is supported at this time. Found " + lidarSensors.size()
                  + " LIDAR units. Will use only the first.");
         return lidarSensors.get(0); // the only one.
      }
   }

   public static void setupDRCRobotLidar(HumanoidRobotSimulation<SDFRobot> sdfRobotSimulation, ObjectCommunicator objectCommunicator,
         TimestampProvider timestampProvider, boolean startLidar)
   {
      Graphics3DAdapter graphics3dAdapter = sdfRobotSimulation.getSimulationConstructionSet().getGraphics3dAdapter();
      SimulatedLIDARSensorUpdateParameters updateParameters = new SimulatedLIDARSensorUpdateParameters();
      updateParameters.setObjectCommunicator(objectCommunicator);
      if (DRCConfigParameters.STREAM_POLAR_LIDAR)
      {
         System.out.println("DRCLidar: Setting up lidar.");
         FastPolarRayCastLIDAR polarLidar = getLidarSensor(sdfRobotSimulation.getRobot());
         if (polarLidar != null)
         {
            polarLidar.setWorld(graphics3dAdapter);
            if (DRCConfigParameters.OVERRIDE_DRC_LIDAR_CONFIG)
            {
               PolarLidarScanParameters largeScan = new PolarLidarScanParameters(true, DRCConfigParameters.LIDAR_POINTS_PER_SWEEP,
                     DRCConfigParameters.LIDAR_SWEEPS_PER_SCAN, DRCConfigParameters.LIDAR_SWEEP_MAX_YAW, DRCConfigParameters.LIDAR_SWEEP_MIN_YAW,
                     DRCConfigParameters.LIDAR_ANGLE_INCREMENT, DRCConfigParameters.LIDAR_TIME_INCREMENT, DRCConfigParameters.LIDAR_SCAN_TIME,
                     DRCConfigParameters.LIDAR_SCAN_MAX_ROLL, DRCConfigParameters.LIDAR_SCAN_MIN_ROLL, DRCConfigParameters.LIDAR_MIN_DISTANCE, DRCConfigParameters.LIDAR_MAX_DISTANCE);
               polarLidar.setScan(largeScan);
               updateParameters.setUpdateRate(DRCConfigParameters.LIDAR_UPDATE_RATE_OVERRIDE);
            }
            if (PRINT_ALL_POSSIBLE_JOINT_NAMES)
               System.out.println("DRCLidar availiable joints: " + sdfRobotSimulation.getRobot().getOneDoFJoints());
            OneDegreeOfFreedomJoint neckJoint = sdfRobotSimulation.getRobot().getOneDoFJoint("neck_ry");
            polarLidar.setSimulationNeckJoint(neckJoint);
            polarLidar.setLidarDaemonParameters(updateParameters);
            SimulatedLIDARSensorNoiseParameters noiseParameters = new SimulatedLIDARSensorNoiseParameters();
            noiseParameters.setGaussianNoiseStandardDeviation(DRCConfigParameters.LIDAR_NOISE_LEVEL_OVERRIDE);
            polarLidar.setNoiseParameters(noiseParameters);
            if (startLidar)
            {
               System.out.println("Streaming Lidar");
               polarLidar.startLidarDaemonThread(timestampProvider);
            }
         }
      }

   }

}
