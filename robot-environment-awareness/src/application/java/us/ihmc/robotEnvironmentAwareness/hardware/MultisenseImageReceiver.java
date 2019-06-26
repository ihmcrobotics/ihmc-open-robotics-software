package us.ihmc.robotEnvironmentAwareness.hardware;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Scanner;
import java.util.concurrent.atomic.AtomicReference;

import javax.imageio.ImageIO;

import org.jboss.netty.buffer.ChannelBuffer;

import controller_msgs.msg.dds.ImageMessage;
import sensor_msgs.CameraInfo;
import sensor_msgs.Image;
import std_msgs.Header;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class MultisenseImageReceiver extends AbstractRosTopicSubscriber<Image>
{
   //private static final String multisenseCartAddress = "http://10.7.4.100:11311";
   private static final String multisenseCartAddress = "http://10.6.192.14:11311";
   private static final String imageROSTopicName = "/multisense/left/image_rect_color";
   private static final String cameraInfoROSTopicName = "/multisense/left/camera_info";

   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "imagePublisherNode");

   private final IHMCROS2Publisher<ImageMessage> imagePublisher;

   private final MultisenseCameraInfoReceiver cameraInfoReceiver;

   private final Scanner commandScanner;
   private static final String commandToSaveImage = "s";
   private static final String commandToShowCameraInfo = "c";
   private int savingIndex = 0;

   private AtomicReference<Boolean> saveImage = new AtomicReference<Boolean>(false);
   private AtomicReference<Boolean> showCameraInfo = new AtomicReference<Boolean>(false);

   public MultisenseImageReceiver() throws URISyntaxException, IOException
   {
      super(Image._TYPE);
      commandScanner = new Scanner(System.in);
      URI masterURI = new URI(multisenseCartAddress);
      RosMainNode rosMainNode = new RosMainNode(masterURI, "ImagePublisher", true);
      rosMainNode.attachSubscriber(imageROSTopicName, this);
      rosMainNode.execute();

      imagePublisher = ROS2Tools.createPublisher(ros2Node, ImageMessage.class, ROS2Tools.getDefaultTopicNameGenerator());
      System.out.println(ROS2Tools.getDefaultTopicNameGenerator().generateTopicName(ImageMessage.class));

      cameraInfoReceiver = new MultisenseCameraInfoReceiver();

      Runnable inputReader = new Runnable()
      {
         @Override
         public void run()
         {
            while(true)
            {
               String command = commandScanner.next();

               if (command.contains(commandToSaveImage))
               {
                  saveImage.set(true);
                  System.out.println(commandToSaveImage + " pressed");
               }
               else if (command.contains(commandToShowCameraInfo))
               {
                  showCameraInfo.set(true);
                  System.out.println(commandToShowCameraInfo + " pressed");
               }
            }
         }
      };
      Thread inputHolder = new Thread(inputReader);
      inputHolder.start();
   }

   @Override
   public void onNewMessage(Image image)
   {
      int width = image.getWidth();
      int height = image.getHeight();

      ImageMessage message = new ImageMessage();
      BufferedImage bufferedImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);

      message.setHeight(height);
      message.setWidth(width);

      ChannelBuffer data = image.getData();
      byte[] array = data.array();
      int dataIndex = data.arrayOffset();
      for (int i = 0; i < height; i++)
      {
         for (int j = 0; j < width; j++)
         {
            int b = array[dataIndex];
            dataIndex++;
            int g = array[dataIndex];
            dataIndex++;
            int r = array[dataIndex];
            dataIndex++;

            int rgbColor = convertBGR2RGB(b, g, r);
            message.getRgbdata().add(rgbColor);
            bufferedImage.setRGB(j, i, rgbColor);
         }
      }

      imagePublisher.publish(message);

      if (saveImage.getAndSet(false))
      {
         File outputfile = new File("image_" + savingIndex + ".jpg");
         try
         {
            ImageIO.write(bufferedImage, "jpg", outputfile);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         savingIndex++;
      }
   }

   private static int convertBGR2RGB(int b, int g, int r)
   {
      int rgb = ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF);
      return rgb;
   }

   public static void main(String[] args) throws URISyntaxException, IOException
   {
      new MultisenseImageReceiver();
   }

   private class MultisenseCameraInfoReceiver extends AbstractRosTopicSubscriber<CameraInfo>
   {
      public MultisenseCameraInfoReceiver() throws URISyntaxException, IOException
      {
         super(CameraInfo._TYPE);
         URI masterURI = new URI(multisenseCartAddress);
         RosMainNode rosMainNode = new RosMainNode(masterURI, "CameraInfoPublisher", true);
         rosMainNode.attachSubscriber(cameraInfoROSTopicName, this);
         rosMainNode.execute();
      }

      @Override
      public void onNewMessage(CameraInfo cameraInfo)
      {
         if (showCameraInfo.getAndSet(false))
         {
            System.out.println("## CameraInfo");
            String distortionModel = cameraInfo.getDistortionModel();
            System.out.println(distortionModel);
            double[] d = cameraInfo.getD();
            double[] k = cameraInfo.getK();
            double[] p = cameraInfo.getP();
            double[] r = cameraInfo.getR();
            Header header = cameraInfo.getHeader();
            int height = cameraInfo.getHeight();
            int width = cameraInfo.getWidth();

            System.out.println("D");
            for (int i = 0; i < d.length; i++)
               System.out.println(d[i]);

            System.out.println("K");
            for (int i = 0; i < k.length; i++)
               System.out.println(k[i]);

            System.out.println("P");
            for (int i = 0; i < p.length; i++)
               System.out.println(p[i]);

            System.out.println("R");
            for (int i = 0; i < r.length; i++)
               System.out.println(r[i]);

            System.out.println(header.getStamp() + " " + header.getFrameId());
            System.out.println("height = " + height + " width = " + width);
         }
      }
   }
}
