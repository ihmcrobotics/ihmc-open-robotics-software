package us.ihmc.ihmcPerception.linemod;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;

import sensor_msgs.PointCloud2;
import us.ihmc.utilities.math.UnitConversions;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber.UnpackedPointCloud;

public class RosLineModDetector extends LineModDetector
{
   RosPointCloudSubscriber rosPointCloudSubscriber;
   public RosLineModDetector(String modelPathInResource)
   {
      super(modelPathInResource);
      int posesForSynthesis = 24;
      trainModelFromRenderedImages(posesForSynthesis);
      setupRosSubscriber();
   }

   private void onNewPointcloud(UnpackedPointCloud unpackedPointCloud)
   {
      OrganizedPointCloud organizedPointCloud = new OrganizedPointCloud(unpackedPointCloud.getWidth(), unpackedPointCloud.getHeight(),unpackedPointCloud.getXYZRGB());
      ArrayList<LineModDetection> detections=new ArrayList<>();
      int bestDetectionIndex= detectObjectAndEstimatePose(organizedPointCloud, detections);
      float estimatedAngle = detections.get(bestDetectionIndex).yaw;
      System.out.println("Found drill of angle "+ estimatedAngle/UnitConversions.DEG_TO_RAD);
   }

   private void setupRosSubscriber()
   {
      rosPointCloudSubscriber = new RosPointCloudSubscriber()
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud)
         {
            onNewPointcloud(unpackPointsAndIntensities(pointCloud));
         }
      };

      new Thread()
      {
         @Override
         public void run()
         {
            try
            {
               URI rosMasterUri = new URI("http://localhost:11311/");
               String topic = "/cloud_pcd";
               RosMainNode mainNode = new RosMainNode(rosMasterUri, "linemod");
               mainNode.attachSubscriber(topic, rosPointCloudSubscriber);
               mainNode.execute();
            }
            catch (URISyntaxException e)
            {
               e.printStackTrace();
            }

         }

      }.start();
   }

   public static void main(String[] arg)
   {
      RosLineModDetector detector= new RosLineModDetector("examples/drill/drill.obj");
      detector.rosPointCloudSubscriber.wailTillRegistered();
      System.out.println("connected to rosmaster");
   }

}
