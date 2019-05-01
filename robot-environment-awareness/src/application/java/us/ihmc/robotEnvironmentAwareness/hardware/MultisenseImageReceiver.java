package us.ihmc.robotEnvironmentAwareness.hardware;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Scanner;

import javax.imageio.ImageIO;

import org.jboss.netty.buffer.ChannelBuffer;

import sensor_msgs.msg.dds.Image;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.utilities.ros.RosMainNode;

public class MultisenseImageReceiver
{
   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "imagePublisherNode");

   private final IHMCROS2Publisher<ImageMessage> imagePublisher;

   private final boolean publishingAllImages = true;
   private final Scanner commandScanner;
   private static final String commandToReceiveNewImage = "s";
   private int savingIndex = 0;

   public MultisenseImageReceiver() throws URISyntaxException, IOException
   {
      super(Image._TYPE);
      commandScanner = new Scanner(System.in);
      URI masterURI = new URI("http://10.6.192.14:11311");
      RosMainNode rosMainNode = new RosMainNode(masterURI, "ImagePublisher", true);
      rosMainNode.attachSubscriber("/multisense/left/image_rect_color", this);
      rosMainNode.execute();

      imagePublisher = ROS2Tools.createPublisher(ros2Node, ImageMessage.class, ROS2Tools.getDefaultTopicNameGenerator());
      System.out.println(ROS2Tools.getDefaultTopicNameGenerator().generateTopicName(ImageMessage.class));
   }

   @Override
   public void onNewMessage(Image image)
   {
      if (publishingAllImages)
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
      }
      else
      {
         String command = commandScanner.next();

         if (command.contains(commandToReceiveNewImage))
         {
            int width = image.getWidth();
            int height = image.getHeight();

            System.out.println("received " + width + " " + height);
            System.out.println(image.getStep() + " " + image.getEncoding());
            System.out.println(image.getHeader() + " " + image.getIsBigendian() + " " + image.getData().arrayOffset());

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

            File outputfile = new File("image_" + savingIndex + ".png");
            try
            {
               ImageIO.write(bufferedImage, "png", outputfile);
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }
            savingIndex++;
         }
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
}
