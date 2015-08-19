package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;
import java.util.Arrays;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.communication.net.LocalObjectCommunicator;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.packets.SimulatedLidarScanPacket;
import us.ihmc.graphics3DAdapter.GPULidar;
import us.ihmc.graphics3DAdapter.GPULidarListener;
import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.simulationconstructionset.simulatedSensors.LidarMount;
import us.ihmc.utilities.TimestampProvider;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.time.TimeTools;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class DRCLidar
{
   private static LidarMount getSensor(SDFRobot robot, String sensorName)
   {
      ArrayList<LidarMount> lidarSensors = robot.getSensors(LidarMount.class);
      
      if(lidarSensors.size() == 0)
      {
         System.err.println("DRCLidar: No LIDAR units found on SDF Robot.");

         return null;
      }
      
      for(LidarMount lidarMount : lidarSensors)
      {
         if(lidarMount.getName().equals(sensorName))
         {
            return lidarMount;
         }
      }
      System.err.println("DRCLidar: Could Not find " + sensorName + "Using " + lidarSensors.get(0).getName() + "Instead! FIX THIS!");
      return lidarSensors.get(0);
   }

   public static void setupDRCRobotLidar(DRCSimulationFactory drcSimulation, LocalObjectCommunicator objectCommunicator, DRCRobotJointMap jointMap,
         DRCRobotLidarParameters lidarParams, TimestampProvider timestampProvider, boolean startLidar)
 {
    Graphics3DAdapter graphics3dAdapter = drcSimulation.getSimulationConstructionSet().getGraphics3dAdapter();
    if (graphics3dAdapter != null)
    {
       LidarMount lidarMount = getSensor(drcSimulation.getRobot(), lidarParams.getSensorNameInSdf());

       LidarScanParameters lidarScanParameters = lidarMount.getLidarScanParameters();
       int horizontalRays = lidarScanParameters.pointsPerSweep;
       float fov = lidarScanParameters.sweepYawMax - lidarScanParameters.sweepYawMin;
       float near = lidarScanParameters.minRange;
       float far = lidarScanParameters.maxRange;

       DRCLidarCallback callback = new DRCLidarCallback(objectCommunicator, lidarScanParameters, lidarParams.getSensorId());
       GPULidar lidar = graphics3dAdapter.createGPULidar(callback, horizontalRays, fov, near, far);
       lidarMount.setLidar(lidar);
    }
 }

   public static class DRCLidarCallback implements GPULidarListener
   {
      private final ObjectCommunicator objectCommunicator;
      private final LidarScanParameters lidarScanParameters;
      private final int lidarSensorId;

      public DRCLidarCallback(ObjectCommunicator objectCommunicator, LidarScanParameters lidarScanParameters, int lidarSensorId)
      {
         this.objectCommunicator = objectCommunicator;
         this.lidarScanParameters = lidarScanParameters;
         this.lidarSensorId = lidarSensorId;
      }

      @Override
      public void scan(float[] scan, RigidBodyTransform lidarTransform, double time)
      {
         final SimulatedLidarScanPacket lidarScan = new SimulatedLidarScanPacket(lidarSensorId, new LidarScanParameters(lidarScanParameters,
               TimeTools.secondsToNanoSeconds(time)), Arrays.copyOf(scan, scan.length));

         objectCommunicator.consumeObject(lidarScan);
      }
   }

}
