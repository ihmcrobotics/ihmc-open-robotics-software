package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;

import javax.imageio.ImageIO;

import controller_msgs.msg.dds.VideoPacket;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.parameters.AvatarRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;

public class CollaborativeBehavior extends AbstractBehavior
{
   private ConcurrentListeningQueue<VideoPacket> cameraData = new ConcurrentListeningQueue<>(20);
   private boolean testImage = false;

   public CollaborativeBehavior(String robotName, Ros2Node ros2Node, HumanoidReferenceFrames referenceFrames,
                                FullHumanoidRobotModel fullHumanoidRobotModel, HumanoidRobotSensorInformation robotSensorInfo,
                                WalkingControllerParameters walkingControllerParameters, YoGraphicsListRegistry graphicsListRegistry)
   {
      super(robotName, ros2Node);
      createSubscriber(VideoPacket.class, ROS2Tools.getDefaultTopicNameGenerator(), cameraData::put);
      AvatarRobotCameraParameters[] robotCameraParameters = robotSensorInfo.getCameraParameters();
   }

   @Override
   public void doControl()
   {
      if (cameraData.isNewPacketAvailable())
      {
         VideoPacket vidPack = cameraData.getLatestPacket();
         //System.out.println(vidPack.videoSource.toString());
         if (!testImage)
         {
            try
            {
               InputStream in = new ByteArrayInputStream(vidPack.getData().toArray());
               ImageIO.write(ImageIO.read(in), "png", new java.io.File("testImage"));
            }
            catch (IOException e)
            {

            }
            testImage = true;
         }
      }
      else
      {
         //System.out.println("Nothing Works");
      }
   }

   @Override
   public void onBehaviorEntered()
   {

   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {

   }

   @Override
   public void onBehaviorResumed()
   {

   }

   @Override
   public void onBehaviorExited()
   {

   }

   @Override
   public boolean isDone()
   {
      return false;
   }

}
