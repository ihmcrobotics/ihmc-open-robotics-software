package us.ihmc.ihmcPerception.linemod;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.File;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Collections;

import javax.swing.JFrame;
import us.ihmc.euclid.tuple3D.Point3D;

import boofcv.gui.image.ImagePanel;
import boofcv.gui.image.ShowImages;
import sensor_msgs.PointCloud2;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;

public class RosLineModDetector extends LineModDetector
{
   RosPointCloudSubscriber rosPointCloudSubscriber;

   JFrame frame = null;
   ImagePanel imagePanel = null;

   public RosLineModDetector(String modelPathInResource)
   {
      super(modelPathInResource);
      File featureFile = new File("FeatureSphare3x4.dat");
      if (featureFile.exists())
      {
         System.out.println("loading feature from disk");
         loadFeatures(featureFile);
         System.out.println(byteFeatures.size() + " templates loaded");
      }
      else
      {
         System.out.println("trainign new feature");
         trainModelFromRenderedImagesSphere(0);
         saveFeatures(featureFile);
      }

      imagePanel = new ImagePanel(100, 100);
      frame = ShowImages.showWindow(imagePanel, "Detection");
      setupRosSubscriber();
   }

   private synchronized void onNewPointcloud(RosPointCloudSubscriber.UnpackedPointCloud unpackedPointCloud)
   {

      OrganizedPointCloud organizedPointCloud = new OrganizedPointCloud(unpackedPointCloud.getWidth(), unpackedPointCloud.getHeight(),
            unpackedPointCloud.getXYZRGB());
      ArrayList<LineModDetection> detections = new ArrayList<>();
      LineModDetection bestDetection = detectObjectAndEstimatePose(organizedPointCloud, detections,false,false);
      System.out.println(bestDetection);

      if (bestDetection != null)
      {

         BufferedImage image = organizedPointCloud.getRGBImage();
         
         Collections.sort(detections);
         for(int i=0;i<Math.min(detections.size(),3);i++)
         {
            drawDetectionOnImage(detections.get(i), image);
         }

         int heightOffset = frame.getHeight() - frame.getContentPane().getHeight();
         int widthOffset = frame.getWidth() - frame.getContentPane().getWidth();
         frame.setSize(image.getWidth() + widthOffset, image.getHeight() + heightOffset);
         imagePanel.setBufferedImageSafe(image);
      }
   }

   private void setupRosSubscriber()
   {
      rosPointCloudSubscriber = new RosPointCloudSubscriber()
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud)
         {
            UnpackedPointCloud cloud = unpackPointsAndIntensities(pointCloud);
            if (cloud.getWidth() > 1200)
            {
               int ptr = 0;
               Point3D[] points = new Point3D[(cloud.getWidth() / 2) * (cloud.getHeight() / 2)];
               Color[] pointColors = new Color[(cloud.getWidth() / 2) * (cloud.getHeight() / 2)];
               for (int j = 0; j < cloud.getHeight(); j += 2)
                  for (int i = 0; i < cloud.getWidth(); i += 2)
                  {
                     points[ptr] = cloud.getPoints()[j * cloud.getWidth() + i];
                     pointColors[ptr] = cloud.getPointColors()[j * cloud.getWidth() + i];
                     ptr++;
                  }

               UnpackedPointCloud scaledCloud = new UnpackedPointCloud(cloud.getWidth() / 2, cloud.getHeight() / 2, cloud.getPointType(), points, pointColors);
               onNewPointcloud(scaledCloud);
               return;
            }
            else
            {
               onNewPointcloud(cloud);
               return;
            }
         }
      };

      new Thread()
      {
         @Override
         public void run()
         {
            try
            {
               URI rosMasterUri;
               String topic;
               boolean isLocal = true;
               if (isLocal)
               {
                  rosMasterUri = new URI("http://localhost:11311/");
                  topic = "/cloud_pcd";
               }
               else
               {
                  rosMasterUri = new URI("http://cpu0:11311/");
                  topic = "/multisense/organized_image_points2_color";
               }
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
      RosLineModDetector detector = new RosLineModDetector("examples/drill/drill.obj");
      detector.rosPointCloudSubscriber.wailTillRegistered();
      System.out.println("connected to rosmaster");
   }

}
