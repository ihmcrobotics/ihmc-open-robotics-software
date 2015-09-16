package us.ihmc.ihmcPerception;

import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.net.URI;
import java.net.URISyntaxException;

import javax.swing.JFrame;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ros.RosCore;

import boofcv.abst.calib.ConfigChessboard;
import boofcv.abst.fiducial.FiducialDetector;
import boofcv.core.image.ConvertBufferedImage;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.gui.fiducial.VisualizeFiducial;
import boofcv.gui.image.ImagePanel;
import boofcv.gui.image.ShowImages;
import boofcv.struct.calib.IntrinsicParameters;
import boofcv.struct.image.ImageFloat32;
import georegression.metric.UtilAngle;
import georegression.struct.se.Se3_F64;
import us.ihmc.robotics.geometry.Transform3d;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.publisher.RosTf2Publisher;
import us.ihmc.utilities.ros.subscriber.RosCameraInfoSubscriber;
import us.ihmc.utilities.ros.subscriber.RosImageSubscriber;

public class RosFiducialDetector extends RosImageSubscriber
{
   final boolean VISUALIZE = true;
   private RosCameraInfoSubscriber cameraInfoSubscriber;
   private RosTf2Publisher tfPublisher = new RosTf2Publisher(false);

   ImageFloat32 gray = new ImageFloat32(640, 480);
   ImagePanel imagePanel = null;
   JFrame frame = null;
   IntrinsicParameters intrinsicParameters = null;

   FiducialDetector<ImageFloat32> detector = FactoryFiducial.
   //       squareBinaryRobust(new ConfigFiducialBinary(0.1), 6, ImageFloat32.class);
         calibChessboard(new ConfigChessboard(5, 6), 0.09, ImageFloat32.class);

   //       calibSquareGrid(new ConfigSquareGrid(5,7), 0.03, ImageFloat32.class);

   public RosFiducialDetector(RosMainNode rosMainNode, String imageTopic, String cameraInfoTopic)
   {
      cameraInfoSubscriber = new RosCameraInfoSubscriber(rosMainNode, cameraInfoTopic);
      rosMainNode.attachPublisher("/tf", tfPublisher);
      rosMainNode.attachSubscriber(imageTopic, this);
      if(VISUALIZE)
      {
         imagePanel= new ImagePanel(100, 100);
         frame=ShowImages.showWindow(imagePanel, "Fiducials");
      }
   }

   private void determineIntrinsicParameters(int width, int height)
   {
      try
      {
         intrinsicParameters = cameraInfoSubscriber.getIntrinsicParametersBlocking();
      }
      catch (InterruptedException e)
      {
         System.out.println("Use default intrinsic parameters");
         intrinsicParameters = new IntrinsicParameters();
         intrinsicParameters.width = width;
         intrinsicParameters.height = height;
         intrinsicParameters.cx = width / 2;
         intrinsicParameters.cy = height / 2;
         intrinsicParameters.fx = intrinsicParameters.cx / Math.tan(UtilAngle.degreeToRadian(30)); // assume 60 degree FOV
         intrinsicParameters.fy = intrinsicParameters.cx / Math.tan(UtilAngle.degreeToRadian(30));
         intrinsicParameters.flipY = false;
      }
      detector.setIntrinsic(intrinsicParameters);
   }

   @Override
   protected void imageReceived(long timeStamp, BufferedImage image)
   {
      if(intrinsicParameters==null)
         determineIntrinsicParameters(image.getWidth(), image.getHeight());
      gray.reshape(image.getWidth(), image.getHeight());
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

            VisualizeFiducial.drawCube(targetToSensor, intrinsicParameters, 0.1, g2);
         }
         
         Matrix3d rotation = new Matrix3d(targetToSensor.getRotation().getData());
         Vector3d translation = new Vector3d(targetToSensor.getTranslation().getX(),targetToSensor.getTranslation().getY(),targetToSensor.getTranslation().getZ());
         Transform3d transform = new Transform3d(rotation, translation);
         tfPublisher.publish(transform, timeStamp, "fiducialTarget", "camera");

         if(VISUALIZE)
         {
            int heightOffset=frame.getHeight()-frame.getContentPane().getHeight();
            int widthOffset=frame.getWidth() - frame.getContentPane().getWidth();
            frame.setSize(image.getWidth()+widthOffset,image.getHeight()+heightOffset);
            imagePanel.setBufferedImageSafe(image);
         }
      }
      catch (Exception e)
      {
           e.printStackTrace();
      }



   }

   public static void main(String[] arg) throws URISyntaxException
   {
      //install ros libuvc_camera driver first to try on usbwebcams
      String cameraPrefix = "/multisense/left";
      String imageTopic = cameraPrefix + "/image_color";
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

         rosMasterURI = new URI("http://cpu0:11311");

      }
      RosMainNode rosMainNode = new RosMainNode(rosMasterURI, "RosMainNode");
      new RosFiducialDetector(rosMainNode, imageTopic, cameraInfoTopic);
      rosMainNode.execute();
   }
}
