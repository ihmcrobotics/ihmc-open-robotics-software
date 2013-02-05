package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;


import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.utilities.polarLidarGeometry.LIDARScanDefinition;
import us.ihmc.utilities.polarLidarGeometry.PolarLidarScanDefinition;

import com.yobotics.simulationconstructionset.simulatedSensors.FastPolarRayCastLIDAR;
import com.yobotics.simulationconstructionset.simulatedSensors.RayTraceLIDARSensor;
import com.yobotics.simulationconstructionset.simulatedSensors.SimulatedLIDARSensorUpdateParameters;

public class DRCLidar
{
   static {
      if(DRCConfigParameters.STREAM_VANILLA_LIDAR)
         System.out.println("DRCLidar: Lidar is ON");
      if(DRCConfigParameters.STREAM_POLAR_LIDAR)
         System.out.println("DRCLidar: PolarLidar is ON. This lidar passes less data, and also includes the transforms from when the data was produced.");
   }

   static void setupDRCRobotLidar(HumanoidRobotSimulation<SDFRobot> sdfRobotSimulation)
   {
      if (DRCConfigParameters.STREAM_VANILLA_LIDAR)
      {
         System.out.println("Streaming Lidar");
         ArrayList<RayTraceLIDARSensor> lidarSensors = sdfRobotSimulation.getRobot().getSensors(RayTraceLIDARSensor.class);
         if (lidarSensors.size() == 0)
            System.err.println("DRCLidar: No LIDAR units found on SDF Robot.");
         else
         {
            if (lidarSensors.size() >=2)
               System.err.println("DRCLidar: Only one LIDAR unit is supported at this time. Found "+lidarSensors.size() +" LIDAR units. Will use only the first.");
            Graphics3DAdapter graphics3dAdapter = sdfRobotSimulation.getSimulationConstructionSet().getGraphics3dAdapter();
            SimulatedLIDARSensorUpdateParameters updateParameters = new SimulatedLIDARSensorUpdateParameters();
            updateParameters.setTransponderSettings(DRCConfigParameters.LIDAR_DATA_PORT_NUMBER, DRCConfigParameters.LIDAR_DATA_IDENTIFIER);
            RayTraceLIDARSensor rayTraceLIDARSensor = lidarSensors.get(0); // the only one.
            rayTraceLIDARSensor.setWorld(graphics3dAdapter);
            if (DRCConfigParameters.OVERRIDE_DRC_LIDAR_CONFIG)
            {
               LIDARScanDefinition largeScan = LIDARScanDefinition.defineSimplifiedNoddingLIDARScan(DRCConfigParameters.LIDAR_VERTICAL_SCAN_ANGLE, DRCConfigParameters.LIDAR_SWEEPS_PER_SCAN, DRCConfigParameters.LIDAR_HORIZONTAL_SCAN_ANGLE,
                     DRCConfigParameters.LIDAR_POINTS_PER_SWEEP, DRCConfigParameters.MIN_LIDAR_DISTANCE);
               rayTraceLIDARSensor.setScan(largeScan);
               updateParameters.setUpdateRate(DRCConfigParameters.LIDAR_UPDATE_RATE_OVERRIDE);
            }
            
            rayTraceLIDARSensor.setLidarDaemonParameters(updateParameters);            
            rayTraceLIDARSensor.startLidarDaemonThread();
         }
      }
      if (DRCConfigParameters.STREAM_POLAR_LIDAR)
      {
         System.out.println("Streaming Lidar");
         ArrayList<FastPolarRayCastLIDAR> lidarSensors = sdfRobotSimulation.getRobot().getSensors(FastPolarRayCastLIDAR.class);
         if (lidarSensors.size() == 0)
            System.err.println("DRCLidar: No LIDAR units found on SDF Robot.");
         else
         {
            if (lidarSensors.size() >=2)
               System.err.println("DRCLidar: Only one LIDAR unit is supported at this time. Found "+lidarSensors.size() +" LIDAR units. Will use only the first.");
            Graphics3DAdapter graphics3dAdapter = sdfRobotSimulation.getSimulationConstructionSet().getGraphics3dAdapter();
            SimulatedLIDARSensorUpdateParameters updateParameters = new SimulatedLIDARSensorUpdateParameters();
            updateParameters.setTransponderSettings(DRCConfigParameters.LIDAR_DATA_PORT_NUMBER, DRCConfigParameters.LIDAR_DATA_IDENTIFIER);
            FastPolarRayCastLIDAR polarLidar = lidarSensors.get(0); // the only one.
            polarLidar.setWorld(graphics3dAdapter);
            if (DRCConfigParameters.OVERRIDE_DRC_LIDAR_CONFIG)
            {
               PolarLidarScanDefinition largeScan = new PolarLidarScanDefinition(
                     DRCConfigParameters.LIDAR_POINTS_PER_SWEEP,
                     DRCConfigParameters.LIDAR_SWEEPS_PER_SCAN,
                     DRCConfigParameters.LIDAR_SWEEP_MAX_YAW,
                     DRCConfigParameters.LIDAR_SWEEP_MIN_YAW,
                     DRCConfigParameters.LIDAR_SCAN_MAX_PITCH,
                     DRCConfigParameters.LDIAR_SCAN_MIN_PITCH,
                     DRCConfigParameters.LIDAR_MIN_DISTANCE
                     );
               LIDARScanDefinition.defineSimplifiedNoddingLIDARScan(DRCConfigParameters.LIDAR_VERTICAL_SCAN_ANGLE, DRCConfigParameters.LIDAR_SWEEPS_PER_SCAN, DRCConfigParameters.LIDAR_HORIZONTAL_SCAN_ANGLE,
                     DRCConfigParameters.LIDAR_POINTS_PER_SWEEP, DRCConfigParameters.MIN_LIDAR_DISTANCE);
               polarLidar.setScan(largeScan);
               updateParameters.setUpdateRate(DRCConfigParameters.LIDAR_UPDATE_RATE_OVERRIDE);
            }
            
            polarLidar.setLidarDaemonParameters(updateParameters);            
            polarLidar.startLidarDaemonThread();
         }
      }
      
   }

}
