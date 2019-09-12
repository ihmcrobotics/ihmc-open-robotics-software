package us.ihmc.robotEnvironmentAwareness.hardware;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Random;
import java.util.Scanner;
import java.util.concurrent.atomic.AtomicReference;

import javax.imageio.ImageIO;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import sensor_msgs.PointCloud2;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.PointCloudProjectionHelper;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber.UnpackedPointCloud;

public class MultisenseStereoVisionPointCloudROS1Bridge extends AbstractRosTopicSubscriber<PointCloud2>
{
   private static final MultisenseInformation multisense = MultisenseInformation.CART;

   private static final int MAX_NUMBER_OF_POINTS = 200000;

   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "stereoVisionPublisherNode");

   private final IHMCROS2Publisher<StereoVisionPointCloudMessage> stereoVisionPublisher;

   private static final int projectionWidth = 1024;
   private static final int projectionHeight = 544;

   private final Scanner commandScanner;
   private static final String commandToSaveStereoVisionPointCloudData = "s";
   private static final String commandToSaveProjectedData = "p";
   private int savingIndex = 0;

   private AtomicReference<Boolean> saveStereoVisionPointCloud = new AtomicReference<Boolean>(false);
   private AtomicReference<Boolean> saveProjectedData = new AtomicReference<Boolean>(false);

   public MultisenseStereoVisionPointCloudROS1Bridge() throws URISyntaxException
   {
      super(PointCloud2._TYPE);
      URI masterURI = new URI(multisense.getAddress());
      RosMainNode rosMainNode = new RosMainNode(masterURI, "StereoVisionPublisher", true);
      rosMainNode.attachSubscriber(MultisenseInformation.getStereoVisionPointCloudTopicName(), this);

      rosMainNode.execute();

      stereoVisionPublisher = ROS2Tools.createPublisher(ros2Node, StereoVisionPointCloudMessage.class, ROS2Tools.getDefaultTopicNameGenerator());

      commandScanner = new Scanner(System.in);
      Runnable inputReader = new Runnable()
      {
         @Override
         public void run()
         {
            while (true)
            {
               String command = commandScanner.next();

               if (command.contains(commandToSaveStereoVisionPointCloudData))
               {
                  saveStereoVisionPointCloud.set(true);
                  System.out.println(commandToSaveStereoVisionPointCloudData + " pressed");
               }
               else if (command.contains(commandToSaveProjectedData))
               {
                  saveProjectedData.set(true);
                  System.out.println(commandToSaveProjectedData + " pressed");
               }
            }
         }
      };
      Thread inputHolder = new Thread(inputReader);
      inputHolder.start();
   }

   @Override
   public void onNewMessage(PointCloud2 cloudHolder)
   {
      UnpackedPointCloud pointCloudData = RosPointCloudSubscriber.unpackPointsAndIntensities(cloudHolder);

      Point3D[] pointCloud = pointCloudData.getPoints();
      Color[] colors = pointCloudData.getPointColors();

      Random random = new Random();
      int numberOfPoints = pointCloud.length;

      while (numberOfPoints > MAX_NUMBER_OF_POINTS)
      {
         int indexToRemove = random.nextInt(numberOfPoints);
         int lastIndex = numberOfPoints - 1;

         pointCloud[indexToRemove] = pointCloud[lastIndex];
         colors[indexToRemove] = colors[lastIndex];

         numberOfPoints--;
      }

      long timestamp = cloudHolder.getHeader().getStamp().totalNsecs();
      float[] pointCloudBuffer = new float[3 * numberOfPoints];
      int[] colorsInteger = new int[numberOfPoints];

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D scanPoint = pointCloud[i];

         pointCloudBuffer[3 * i + 0] = (float) scanPoint.getX();
         pointCloudBuffer[3 * i + 1] = (float) scanPoint.getY();
         pointCloudBuffer[3 * i + 2] = (float) scanPoint.getZ();

         colorsInteger[i] = colors[i].getRGB();
      }

      StereoVisionPointCloudMessage stereoVisionMessage = MessageTools.createStereoVisionPointCloudMessage(timestamp, pointCloudBuffer, colorsInteger);

      stereoVisionPublisher.publish(stereoVisionMessage);

      if (saveStereoVisionPointCloud.getAndSet(false))
      {
         FileWriter fileWriter;
         try
         {
            fileWriter = new FileWriter("stereovision_pointcloud_" + savingIndex + ".txt");
            StringBuilder builder = new StringBuilder("");
            for (int i = 0; i < numberOfPoints; i++)
            {
               Point3D scanPoint = pointCloud[i];
               builder.append(i + "\t" + scanPoint.getX() + "\t" + scanPoint.getY() + "\t" + scanPoint.getZ() + "\t" + colors[i].getRGB() + "\n");
            }
            fileWriter.write(builder.toString());
            fileWriter.close();
            System.out.println("saving is done");
         }
         catch (IOException e1)
         {
            e1.printStackTrace();
         }
         savingIndex++;
      }

      if (saveProjectedData.getAndSet(false))
      {
         BufferedImage bufferedImage = new BufferedImage(projectionWidth, projectionHeight, BufferedImage.TYPE_INT_RGB);

         for (int i = 0; i < numberOfPoints; i++)
         {
            Point3D scanPoint = pointCloud[i];
            Point2D projectedPixel = new Point2D();
            int[] offset = multisense.getProjectionOffset();

            PointCloudProjectionHelper.projectMultisensePointCloudOnImage(scanPoint, projectedPixel, multisense.getIntrinsicParameters(), offset[0], offset[1]);

            boolean inImage = false;
            if (projectedPixel.getX() >= 0 && projectedPixel.getX() < projectionWidth)
               if (projectedPixel.getY() >= 0 && projectedPixel.getY() < projectionHeight)
                  inImage = true;

            if (inImage)
               bufferedImage.setRGB((int) projectedPixel.getX(), (int) projectedPixel.getY(), colors[i].getRGB());
         }

         File outputfile = new File("projected_image_" + savingIndex + ".png");
         try
         {
            ImageIO.write(bufferedImage, "png", outputfile);
            System.out.println("saving is done");
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         savingIndex++;
      }
   }

   public static void main(String[] args) throws URISyntaxException
   {
      new MultisenseStereoVisionPointCloudROS1Bridge();
   }
}
