package us.ihmc.ihmcPerception.depthData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

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
      Point3D[] pointCloud = new Point3D[70000];

      //TODO: move this to RandomTools
      for(int i = 0; i < pointCloud.length; i++)
      {
         double distanceFromCamera = RandomNumbers.nextDouble(random, 0.0, coneHeight);
         double angle = RandomNumbers.nextDouble(random, Math.PI);
         double distanceFromConeCenter = RandomNumbers.nextDouble(random, distanceFromCamera * (coneBaseRadius / coneHeight));

         double coneY = distanceFromConeCenter * Math.sin(angle);
         double coneZ = distanceFromConeCenter * Math.cos(angle);

         pointCloud[i] = new Point3D(distanceFromCamera, coneY, coneZ);
      }

      ArrayList<Point3D> pointCloudArrayList = new ArrayList<>();
      pointCloudArrayList.addAll(Arrays.asList(pointCloud));
      pointCloudDataReceiver.receivedPointCloudData(scanFrame, lidarFrame, timestamps, pointCloudArrayList, PointCloudSource.NEARSCAN);
   }
}
