package us.ihmc.ihmcPerception;

import georegression.metric.UtilAngle;
import georegression.struct.se.Se3_F64;

import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.net.URI;
import java.net.URISyntaxException;

import org.ros.RosCore;

import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosCameraInfoSubscriber;
import us.ihmc.utilities.ros.subscriber.RosImageSubscriber;
import boofcv.abst.calib.ConfigChessboard;
import boofcv.abst.fiducial.FiducialDetector;
import boofcv.core.image.ConvertBufferedImage;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.gui.fiducial.VisualizeFiducial;
import boofcv.gui.image.ImagePanel;
import boofcv.gui.image.ShowImages;
import boofcv.struct.calib.IntrinsicParameters;
import boofcv.struct.image.ImageFloat32;

public class RosFiducialDetector extends RosImageSubscriber
{
   private RosCameraInfoSubscriber cameraInfoSubscriber;

   ImageFloat32 gray = new ImageFloat32(640, 480);
   ImagePanel gui = new ImagePanel(640, 480);
   IntrinsicParameters param = new IntrinsicParameters();

   FiducialDetector<ImageFloat32> detector = FactoryFiducial.
   //       squareBinaryRobust(new ConfigFiducialBinary(0.1), 6, ImageFloat32.class);
         calibChessboard(new ConfigChessboard(5, 6), 0.01, ImageFloat32.class);

   //       calibSquareGrid(new ConfigSquareGrid(5,7), 0.03, ImageFloat32.class);

   public RosFiducialDetector(RosMainNode rosMainNode, String imageTopic, String cameraInfoTopic)
   {
      cameraInfoSubscriber = new RosCameraInfoSubscriber(rosMainNode, cameraInfoTopic);
      rosMainNode.attachSubscriber(imageTopic, this);
      ShowImages.showWindow(gui, "Fiducials");
   }

   private void determineIntrinsicParameters(int width, int height)
   {
      try
      {
         param = cameraInfoSubscriber.getIntrinsicParametersBlocking();
         throw new InterruptedException();
      }
      catch (InterruptedException e)
      {
         System.out.println("Use default intrinsic parameters");
         param.width = width;
         param.height = height;
         param.cx = width / 2;
         param.cy = height / 2;
         param.fx = param.cx / Math.tan(UtilAngle.degreeToRadian(30)); // assume 60 degree FOV
         param.fy = param.cx / Math.tan(UtilAngle.degreeToRadian(30));
         param.flipY = false;
      }
      detector.setIntrinsic(param);
   }

   @Override
   protected void imageReceived(long timeStamp, BufferedImage image)
   {
      determineIntrinsicParameters(image.getWidth(), image.getHeight());
      ConvertBufferedImage.convertFrom(image, gray);

      try
      {
         detector.detect(gray);
         // display the results
         Graphics2D g2 = image.createGraphics();
         Se3_F64 targetToSensor = new Se3_F64();
         for (int i = 0; i < detector.totalFound(); i++)
         {
            detector.getFiducialToWorld(i, targetToSensor);

            VisualizeFiducial.drawCube(targetToSensor, param, 0.1, g2);
         }

         gui.setBufferedImageSafe(image);
         gui.repaint();
      }
      catch (Exception e)
      {
           e.printStackTrace();
      }



   }

   public static void main(String[] arg) throws URISyntaxException
   {
      //install ros libuvc_camera driver first to try on usbwebcams
      String cameraPrefix = "camera";
      String imageTopic = cameraPrefix + "/image_raw";
      String cameraInfoTopic = cameraPrefix + "/camera_info";

      final boolean RUN_JAVA_ROSCORE = false;

      URI rosMasterURI;
      if (RUN_JAVA_ROSCORE)
      {
         RosCore rosCore = RosCore.newPublic();
         rosMasterURI = rosCore.getUri();
      }
      else
      {

         rosMasterURI = new URI("http://localhost:11311");

      }
      RosMainNode rosMainNode = new RosMainNode(rosMasterURI, "RosMainNode");
      new RosFiducialDetector(rosMainNode, imageTopic, cameraInfoTopic);
      rosMainNode.execute();
   }
}
