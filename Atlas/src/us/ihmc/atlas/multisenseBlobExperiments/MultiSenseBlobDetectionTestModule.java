package us.ihmc.atlas.multisenseBlobExperiments;

import boofcv.gui.image.ImagePanel;
import boofcv.gui.image.ShowImages;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.ihmcPerception.OpenCVTools;
import us.ihmc.ihmcPerception.vision.HSVValue;
import us.ihmc.ihmcPerception.vision.shapes.HSVRange;
import us.ihmc.ihmcPerception.vision.shapes.HoughCircleResult;
import us.ihmc.ihmcPerception.vision.shapes.OpenCVColoredCircularBlobDetector;
import us.ihmc.ihmcPerception.vision.shapes.OpenCVColoredCircularBlobDetectorFactory;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.publisher.RosImagePublisher;
import us.ihmc.utilities.ros.subscriber.RosImageSubscriber;

import javax.swing.*;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;

public class MultiSenseBlobDetectionTestModule
{
   private final PacketCommunicator packetCommunicator = PacketCommunicator
         .createTCPPacketCommunicatorServer(NetworkPorts.BEHAVIOUR_MODULE_PORT, new IHMCCommunicationKryoNetClassList());
   private final String stereoTopic = "/multisense/image_points2_color";
   private final String lidarTopic = "/assembled_cloud";
   private final String leftImageColorTopic = "/multisense/left/image_rect_color";

   private BufferedImage latestBufferedImage;

   public MultiSenseBlobDetectionTestModule(URI rosMasterURI)
   {
      RosMainNode rosMainNode = new RosMainNode(rosMasterURI, "multiSenseBlobDetectionTestModule", true);

      MultisenseBlobDetectionPointCloudReceiver stereoReceiver = new MultisenseBlobDetectionPointCloudReceiver(packetCommunicator);
      MultisenseBlobDetectionPointCloudReceiver lidarReceiver = new MultisenseBlobDetectionPointCloudReceiver(packetCommunicator);

      rosMainNode.attachSubscriber(stereoTopic, stereoReceiver);
      rosMainNode.attachSubscriber(lidarTopic, lidarReceiver);
      new BlobDetectionImageProvider(rosMainNode);

      rosMainNode.execute();
      connect();

      Thread blobDetectionThread = new Thread()
      {
         public void run()
         {
            OpenCVColoredCircularBlobDetectorFactory factory = new OpenCVColoredCircularBlobDetectorFactory();

            factory.setCaptureSource(OpenCVColoredCircularBlobDetector.CaptureSource.JAVA_BUFFERED_IMAGES);
            OpenCVColoredCircularBlobDetector openCVColoredCircularBlobDetector = factory.buildBlobDetector();

            HSVRange greenRange = new HSVRange(new HSVValue(55, 80, 80), new HSVValue(139, 255, 255));
//            HSVRange brightRedRange = new HSVRange(new HSVValue(120, 80, 80), new HSVValue(179, 255, 255));
//            HSVRange dullRedRange = new HSVRange(new HSVValue(3, 80, 80), new HSVValue(10, 255, 255));
            HSVRange yellowRange = new HSVRange(new HSVValue(25, 100, 100), new HSVValue(40, 255, 255));

            openCVColoredCircularBlobDetector.addHSVRange(greenRange);
//            openCVColoredCircularBlobDetector.addHSVRange(brightRedRange);
//            openCVColoredCircularBlobDetector.addHSVRange(dullRedRange);
            openCVColoredCircularBlobDetector.addHSVRange(yellowRange);

            while (latestBufferedImage == null)
               ThreadTools.sleep(10);

            ImagePanel imagePanel = ShowImages.showWindow(latestBufferedImage, "Circle Detector");
            JFrame frame = (JFrame) SwingUtilities.getWindowAncestor(imagePanel);

            Scalar circleColor = new Scalar(160, 0, 0);

            while (true)
            {
               if (!frame.isVisible())
               {
                  break;
               }

               openCVColoredCircularBlobDetector.updateFromBufferedImage(latestBufferedImage);

               ArrayList<HoughCircleResult> circles = openCVColoredCircularBlobDetector.getCircles();

               for (HoughCircleResult circle : circles)
               {
                  Point openCVPoint = new Point(circle.getCenter().x, circle.getCenter().y);
                  Imgproc.circle(openCVColoredCircularBlobDetector.getCurrentCameraFrameMatInBGR(), openCVPoint, (int) circle.getRadius(), circleColor, 1);
               }

               imagePanel.setBufferedImage(OpenCVTools.convertMatToBufferedImage(openCVColoredCircularBlobDetector.getCurrentCameraFrameMatInBGR()));
            }
         }
      };

      blobDetectionThread.run();
   }

   private void connect()
   {
      try
      {
         packetCommunicator.connect();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public static void main(String[] args) throws URISyntaxException
   {
      new MultiSenseBlobDetectionTestModule(new URI("http://localhost:11311"));
   }

   private class BlobDetectionImageProvider extends RosImageSubscriber
   {
      public BlobDetectionImageProvider(RosMainNode rosNode)
      {
         super();

         rosNode.attachSubscriber(leftImageColorTopic, this);
      }

      @Override protected void imageReceived(long timeStamp, BufferedImage image)
      {
         latestBufferedImage = image;
      }
   }
}
