package us.ihmc.avatar;

import java.util.ArrayList;
import java.util.Arrays;

import controller_msgs.msg.dds.SimulatedLidarScanPacket;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.net.LocalObjectCommunicator;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.jMonkeyEngineToolkit.GPULidar;
import us.ihmc.jMonkeyEngineToolkit.GPULidarListener;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.robotics.robotDescription.LidarSensorDescription;
import us.ihmc.sensorProcessing.parameters.AvatarRobotLidarParameters;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.simulatedSensors.LidarMount;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class DRCLidar
{
   public static LidarMount getSensor(FloatingRootJointRobot robot, String sensorName)
   {
      ArrayList<LidarMount> lidarSensors = robot.getSensors(LidarMount.class);

      if (lidarSensors.size() == 0)
      {
         System.err.println("DRCLidar: No LIDAR units found on SDF Robot.");

         return null;
      }

      for (LidarMount lidarMount : lidarSensors)
      {
         if (lidarMount.getName().equals(sensorName))
         {
            return lidarMount;
         }
      }
      System.err.println("DRCLidar: Could Not find " + sensorName + "Using " + lidarSensors.get(0).getName() + "Instead! FIX THIS!");
      return lidarSensors.get(0);
   }

   public static void setupDRCRobotLidar(FloatingRootJointRobot robot, Graphics3DAdapter graphics3dAdapter, LocalObjectCommunicator objectCommunicator,
                                         DRCRobotJointMap jointMap, AvatarRobotLidarParameters lidarParams, TimestampProvider timestampProvider,
                                         boolean startLidar)
   {
      if (graphics3dAdapter != null)
      {
         LidarMount lidarMount = getSensor(robot, lidarParams.getSensorNameInSdf());

         LidarSensorDescription desciption = lidarMount.getDescription();
         int horizontalRays = desciption.getPointsPerSweep();
         int scanHeight = desciption.getScanHeight();
         float fov = (float) (desciption.getSweepYawMax() - desciption.getSweepYawMin());
         float near = (float) desciption.getMinRange();
         float far = (float) desciption.getMaxRange();

         LidarScanParameters lidarScanParameters = new LidarScanParameters(desciption.getPointsPerSweep(), desciption.getScanHeight(),
                                                                           (float) desciption.getSweepYawMin(), (float) desciption.getSweepYawMax(),
                                                                           (float) desciption.getHeightPitchMin(), (float) desciption.getHeightPitchMax(), 0,
                                                                           (float) desciption.getMinRange(), (float) desciption.getMaxRange(), 0, 0);

         DRCLidarCallback callback = new DRCLidarCallback(objectCommunicator, lidarScanParameters, lidarParams.getSensorId());
         GPULidar lidar = graphics3dAdapter.createGPULidar(callback, horizontalRays, scanHeight, fov, near, far);
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
         final SimulatedLidarScanPacket lidarScan = MessageTools.createSimulatedLidarScanPacket(lidarSensorId,
                                                                                                new LidarScanParameters(lidarScanParameters,
                                                                                                                        Conversions.secondsToNanoseconds(time)),
                                                                                                Arrays.copyOf(scan, scan.length));

         objectCommunicator.consumeObject(lidarScan);
      }
   }

}
