package us.ihmc.ihmcPerception.depthData;

import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Point3d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class RandomPointCloudGenerator implements Runnable
{
   private final Random random = new Random();

   private final double coneHeight = 1.0;
   private final double coneBaseRadius = 0.25;

   private final long[] timestamps = new long[1];

   private final PointCloudDataReceiverInterface pointCloudDataReceiver;

   private final ReferenceFrame scanFrame;
   private final ReferenceFrame lidarFrame;

   public RandomPointCloudGenerator(PointCloudDataReceiverInterface pointCloudDataReceiver, ReferenceFrame scanFrame, ReferenceFrame lidarFrame,
         long period, TimeUnit timeUnit)
   {
      this.pointCloudDataReceiver = pointCloudDataReceiver;
      this.scanFrame = scanFrame;
      this.lidarFrame = lidarFrame;

      ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
      executor.scheduleAtFixedRate(this, 0L, period, timeUnit);

      timestamps[0] = 0L;
   }

   @Override
   public void run()
   {
      timestamps[0] = System.nanoTime();
      Point3d[] pointCloud = new Point3d[70000];

      //TODO: move this to RandomTools
      for(int i = 0; i < pointCloud.length; i++)
      {
         double distanceFromCamera = RandomTools.generateRandomDouble(random, 0.0, coneHeight);
         double angle = RandomTools.generateRandomDouble(random, Math.PI);
         double distanceFromConeCenter = RandomTools.generateRandomDouble(random, distanceFromCamera * (coneBaseRadius / coneHeight));

         double coneY = distanceFromConeCenter * Math.sin(angle);
         double coneZ = distanceFromConeCenter * Math.cos(angle);

         pointCloud[i] = new Point3d(distanceFromCamera, coneY, coneZ);
      }

      ArrayList<Point3d> pointCloudArrayList = new ArrayList<>();
      pointCloudArrayList.addAll(Arrays.asList(pointCloud));
      pointCloudDataReceiver.receivedPointCloudData(scanFrame, lidarFrame, timestamps, pointCloudArrayList, PointCloudSource.NEARSCAN);
   }
}
