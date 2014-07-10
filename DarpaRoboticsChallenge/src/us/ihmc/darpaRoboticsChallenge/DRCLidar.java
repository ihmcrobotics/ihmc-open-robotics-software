package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.graphics3DAdapter.GPULidar;
import us.ihmc.graphics3DAdapter.GPULidarCallback;
import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.utilities.lidar.PointCloudPacket;
import us.ihmc.utilities.lidar.polarLidar.LidarScan;
import us.ihmc.utilities.lidar.polarLidar.geometry.LidarScanParameters;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.net.TimestampProvider;

import com.yobotics.simulationconstructionset.simulatedSensors.LidarMount;

public class DRCLidar
{
   public static LidarMount getLidarSensor(SDFRobot robot)
   {
      ArrayList<LidarMount> lidarSensors = robot.getSensors(LidarMount.class);
      if (lidarSensors.size() == 0)
      {
         System.err.println("DRCLidar: No LIDAR units found on SDF Robot.");

         return null;
      }
      else if (lidarSensors.size() < 3)
      {
         System.err.println("DRCLidar: Only one LIDAR unit is supported at this time. Found " + lidarSensors.size() + " LIDAR units. Will use only the first.");

         return lidarSensors.get(0);
      }
      else
      {
         System.err.println("DRCLidar: Only one LIDAR unit is supported at this time. Found " + lidarSensors.size()
                            + " LIDAR units. Assuming this is Valkyrie and using the third.");

         return lidarSensors.get(2);
      }
   }

   public static void setupDRCRobotLidar(DRCSimulationFactory drcSimulation, ObjectCommunicator objectCommunicator, DRCRobotJointMap jointMap,
           TimestampProvider timestampProvider, boolean startLidar)
   {
      Graphics3DAdapter graphics3dAdapter = drcSimulation.getSimulationConstructionSet().getGraphics3dAdapter();
      if (graphics3dAdapter != null)
      {
         LidarMount lidarMount = getLidarSensor(drcSimulation.getRobot());

         LidarScanParameters lidarScanParameters = lidarMount.getLidarScanParameters();
         int horizontalRays = lidarScanParameters.pointsPerSweep;
         float fov = lidarScanParameters.sweepYawMax - lidarScanParameters.sweepYawMin;
         float near = lidarScanParameters.minRange;
         float far = lidarScanParameters.maxRange;


         DRCLidarCallback callback = new DRCLidarCallback(objectCommunicator, lidarScanParameters);
         GPULidar lidar = graphics3dAdapter.createGPULidar(callback, horizontalRays, fov, near, far);
         lidarMount.setLidar(lidar);
      }
   }

   public static void setupDRCRobotPointCloud(DRCSimulationFactory drcSimulation, ObjectCommunicator objectCommunicator, DRCRobotJointMap jointMap,
           TimestampProvider timestampProvider, boolean startLidar)
   {
      Graphics3DAdapter graphics3dAdapter = drcSimulation.getSimulationConstructionSet().getGraphics3dAdapter();

      LidarMount lidarMount = getLidarSensor(drcSimulation.getRobot());

      LidarScanParameters lidarScanParameters = lidarMount.getLidarScanParameters();
      int horizontalRays = lidarScanParameters.pointsPerSweep;
      float fov = lidarScanParameters.sweepYawMax - lidarScanParameters.sweepYawMin;
      float near = lidarScanParameters.minRange;
      float far = lidarScanParameters.maxRange;


      DRCLidarToPointCloudCallback callback = new DRCLidarToPointCloudCallback(objectCommunicator, lidarScanParameters);
      GPULidar lidar = graphics3dAdapter.createGPULidar(callback, horizontalRays, fov, near, far);
      lidarMount.setLidar(lidar);
   }


   public static class DRCLidarToPointCloudCallback implements GPULidarCallback
   {
      private final Executor pool = Executors.newSingleThreadExecutor();

      private final ObjectCommunicator objectCommunicator;
      private final LidarScanParameters lidarScanParameters;

      public DRCLidarToPointCloudCallback(ObjectCommunicator objectCommunicator, LidarScanParameters lidarScanParameters)
      {
         this.objectCommunicator = objectCommunicator;
         this.lidarScanParameters = lidarScanParameters;
      }

      @Override
      public void scan(float[] scan, Transform3D lidarTransform, double time, GPULidar gpuLidar)
      {
         Transform3D transform = new Transform3D(lidarTransform);
         final LidarScan lidarScan = new LidarScan(new LidarScanParameters(lidarScanParameters, TimeTools.secondsToNanoSeconds(time)), transform, transform,
                                        Arrays.copyOf(scan, scan.length));

         ArrayList<Point3d> points = lidarScan.getAllPoints();
         Point3d[] pointsArray = new Point3d[points.size()];
         points.toArray(pointsArray);

         final PointCloudPacket pointCloud = new PointCloudPacket(pointsArray, TimeTools.secondsToNanoSeconds(time));

         pool.execute(new Runnable()
         {
            @Override
            public void run()
            {
               objectCommunicator.consumeObject(pointCloud);
            }
         });
      }

   }


   public static class DRCLidarCallback implements GPULidarCallback
   {
      private final Executor pool = Executors.newSingleThreadExecutor();

      private final ObjectCommunicator objectCommunicator;
      private final LidarScanParameters lidarScanParameters;

      public DRCLidarCallback(ObjectCommunicator objectCommunicator, LidarScanParameters lidarScanParameters)
      {
         this.objectCommunicator = objectCommunicator;
         this.lidarScanParameters = lidarScanParameters;
      }

      @Override
      public void scan(float[] scan, Transform3D lidarTransform, double time, GPULidar gpuLidar)
      {
         Transform3D transform = new Transform3D(lidarTransform);
         final LidarScan lidarScan = new LidarScan(new LidarScanParameters(lidarScanParameters, TimeTools.secondsToNanoSeconds(time)), transform, transform,
                                        Arrays.copyOf(scan, scan.length));

         pool.execute(new Runnable()
         {
            @Override
            public void run()
            {
               objectCommunicator.consumeObject(lidarScan);
            }
         });
      }
   }

}
