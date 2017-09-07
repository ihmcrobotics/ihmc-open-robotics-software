package us.ihmc.utilities.ros.apps;

import java.awt.Color;
import java.net.URI;
import java.net.URISyntaxException;

import sensor_msgs.PointCloud2;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.publisher.RosPointCloudPublisher;
import us.ihmc.utilities.ros.publisher.RosTf2Publisher;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
import us.ihmc.utilities.ros.types.GrowablePointCloud;
import us.ihmc.utilities.ros.types.PointType;

public class RosPointCloudFilterRepublisher implements Runnable
{
   final Point3D origin = new Point3D();
   RosMainNode mainNode;
   RosPointCloudPublisher publisher;
   RosPointCloudSubscriber subscriber;
   RosTf2Publisher tfPublisher;


   public RosPointCloudFilterRepublisher()
   {
      try
      {
         mainNode = new RosMainNode(new URI("http://localhost:11311"), "RosPointCloudSubscriber");
      }
      catch (URISyntaxException e)
      {
         e.printStackTrace();
      }

      tfPublisher = new RosTf2Publisher(false);

      publisher = new RosPointCloudPublisher(PointType.XYZRGB, true);
      subscriber = new RosPointCloudSubscriber()
      {
         GrowablePointCloud growablePointCloud = new GrowablePointCloud();
         @Override
         public synchronized void onNewMessage(PointCloud2 pointCloud)
         {
            growablePointCloud.clear();
            UnpackedPointCloud unpackedCloud=unpackPointsAndIntensities(pointCloud);

            for (int i = 0; i < unpackedCloud.getPoints().length; i++)
            {
               switch (PointType.fromFromFieldNames(pointCloud.getFields()))
               {
                  case XYZI :
                     if (includePoint(unpackedCloud.getPoints()[i], unpackedCloud.getIntensities()[i]))
                        growablePointCloud.addPoint(unpackedCloud.getPoints()[i], unpackedCloud.getIntensities()[i]);

                     break;

                  case XYZRGB :
                     if (includePoint(unpackedCloud.getPoints()[i], unpackedCloud.getPointColors()[i]))
                        growablePointCloud.addPoint(unpackedCloud.getPoints()[i], unpackedCloud.getPointColors()[i]);

                     break;
               }
            }

            System.out.println("Publishing " + pointCloud.getHeader().getSeq() + " " + growablePointCloud.size() + " points");
            publisher.publish(growablePointCloud.getPoints(), growablePointCloud.getColors(), pointCloud.getHeader().getFrameId());

            /*
             * Transform3d pinkBlobTransform = new Transform3d();
             * pinkBlobTransform.setTranslation(new Vector3d(growablePointCloud.getMeanPoint()));
             * tfPublisher.publish(pinkBlobTransform, pointCloud.getHeader().getStamp().totalNsecs(), "/multisense/left_camera_optical_frame", "pinkBlob");
             */

         }
      };

   }

   protected boolean includePoint(Point3D point, float intensity)
   {
      return point.distance(origin) < 1.0;
   }

   float[] hsbvals = new float[3];
   float pinkHue = 0.9893f;

   protected boolean includePoint(Point3D point, Color color)
   {
      Color.RGBtoHSB(color.getRed(),  color.getGreen(), color.getBlue(), hsbvals);

      return point.distance(origin) < 1.0;

      // return ((point.distance(origin) < 1.0) && (Math.abs(hsbvals[0] - pinkHue) < 0.01));
   }

   public void run()
   {
      //mainNode.attachSubscriber("/multisense/image_points2_color", subscriber);
      mainNode.attachSubscriber("/multisense/lidar_points2", subscriber);
      mainNode.attachPublisher("/testCloud", publisher);
      mainNode.attachPublisher("/tf", tfPublisher);
      mainNode.execute();
   }


   public static void main(String[] arg) throws URISyntaxException
   {
      RosPointCloudFilterRepublisher republisher = new RosPointCloudFilterRepublisher();
      new Thread(republisher).start();
   }
}
