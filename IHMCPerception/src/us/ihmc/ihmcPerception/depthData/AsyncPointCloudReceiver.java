package us.ihmc.ihmcPerception.depthData;

import java.util.ArrayList;

import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class AsyncPointCloudReceiver implements PointCloudDataReceiverInterface
{
   private final boolean DEBUG = false;
   private Stopwatch timer = DEBUG ? new Stopwatch().start() : null;

   private volatile ArrayList<Point3D> pointsInWorldFrame;
   private volatile double groundHeight;
   private volatile long timestamp;

   @Override
   public void receivedPointCloudData(ReferenceFrame scanFrame, ReferenceFrame lidarFrame, long[] timestamps, ArrayList<Point3D> points,
         PointCloudSource... sources)
   {
      if(DEBUG)
      {
         System.out.println(getClass().getSimpleName() + ": Received point cloud in " + scanFrame.getName() + " frame @ "
               + 1.0 / timer.lap() + " FPS from " + Thread.currentThread().getName());
      }

      if(timestamps.length > 0)
      {
         lidarFrame.update();
         RigidBodyTransform lidarTransform = lidarFrame.getTransformToWorldFrame();


         double localGroundHeight = Double.MAX_VALUE;
         for(Point3D point : points)
         {
            lidarTransform.transform(point);
            if(point.getZ() < localGroundHeight) localGroundHeight = point.getZ();
         }

         synchronized(this)
         {
            pointsInWorldFrame = points;
            groundHeight = localGroundHeight;
            timestamp = timestamps[0];
         }
      }
   }

   public synchronized ArrayList<Point3D> getPointsInWorldFrame()
   {
      return pointsInWorldFrame;
   }

   public synchronized double getGroundHeight()
   {
      return groundHeight;
   }

   public synchronized long getTimestamp()
   {
      return timestamp;
   }
}
