package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;


import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.graphics3DAdapter.Graphics3DAdapter;

import com.yobotics.simulationconstructionset.simulatedSensors.LIDARScanDefinition;
import com.yobotics.simulationconstructionset.simulatedSensors.RayTraceLIDARSensor;
import com.yobotics.simulationconstructionset.simulatedSensors.SimulatedLIDARSensorUpdateParameters;

public class DRCLidar
{
   private static final boolean DEBUG =false;
   static {
      if(DRCConfigParameters.STREAM_LIDAR)
         System.out.println("DRCLidar: Lidar is ON");
   }

   static void setupDRCRobotLidar(HumanoidRobotSimulation<SDFRobot> sdfRobotSimulation)
   {
      if (DRCConfigParameters.STREAM_LIDAR)
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
                     DRCConfigParameters.LIDAR_POINTS_PER_SWEEP);
               rayTraceLIDARSensor.setScan(largeScan);
               updateParameters.setUpdateRate(DRCConfigParameters.LIDAR_UPDATE_RATE_OVERRIDE);
            }
            
            rayTraceLIDARSensor.setLidarDaemonParameters(updateParameters);            
            rayTraceLIDARSensor.startLidarDaemonThread();
         }
      }
   }

}
