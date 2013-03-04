package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.polarLidarGeometry.PolarLidarScanDefinition;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

import com.yobotics.simulationconstructionset.simulatedSensors.FastPolarRayCastLIDAR;
import com.yobotics.simulationconstructionset.simulatedSensors.SimulatedLIDARSensorUpdateParameters;

public class DRCLidar
{
   static {
      if(DRCConfigParameters.STREAM_POLAR_LIDAR)
         System.out.println("DRCLidar: PolarLidar is ON. This lidar passes less data, and also includes the transforms from when the data was produced.");
   }

   static void setupDRCRobotLidar(HumanoidRobotSimulation<SDFRobot> sdfRobotSimulation, ObjectCommunicator objectCommunicator, OneDoFJoint lidarJoint)
   {
      Graphics3DAdapter graphics3dAdapter = sdfRobotSimulation.getSimulationConstructionSet().getGraphics3dAdapter();
      SimulatedLIDARSensorUpdateParameters updateParameters = new SimulatedLIDARSensorUpdateParameters();
      updateParameters.setObjectCommunicator(objectCommunicator);
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
            FastPolarRayCastLIDAR polarLidar = lidarSensors.get(0); // the only one.
            polarLidar.setWorld(graphics3dAdapter);
            if (DRCConfigParameters.OVERRIDE_DRC_LIDAR_CONFIG)
            {
               PolarLidarScanDefinition largeScan = new PolarLidarScanDefinition(
                     DRCConfigParameters.LIDAR_POINTS_PER_SWEEP,
                     DRCConfigParameters.LIDAR_SWEEPS_PER_SCAN,
                     DRCConfigParameters.LIDAR_SWEEP_MAX_YAW,
                     DRCConfigParameters.LIDAR_SWEEP_MIN_YAW,
                     DRCConfigParameters.LIDAR_SCAN_MAX_ROLL,
                     DRCConfigParameters.LDIAR_SCAN_MIN_ROLL,
                     DRCConfigParameters.LIDAR_MIN_DISTANCE
                     );
               polarLidar.setScan(largeScan);
               updateParameters.setUpdateRate(DRCConfigParameters.LIDAR_UPDATE_RATE_OVERRIDE);
            }
            polarLidar.setLidarJoint(lidarJoint);
            polarLidar.setLidarDaemonParameters(updateParameters);            
            polarLidar.startLidarDaemonThread();
         }
      }
      
   }

}
