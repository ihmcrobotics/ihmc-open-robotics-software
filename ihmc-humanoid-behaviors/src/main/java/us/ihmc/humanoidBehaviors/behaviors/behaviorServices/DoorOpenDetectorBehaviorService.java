package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import perception_msgs.msg.dds.DoorLocationPacket;
import ihmc_common_msgs.msg.dds.TextToSpeechPacket;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.kinematics.AverageQuaternionCalculator;
import us.ihmc.ros2.ROS2Node;

public class DoorOpenDetectorBehaviorService extends ThreadedBehaviorService//FiducialDetectorBehaviorService
{

   private int numberToAverage = 5;
   private ArrayList<FramePose3D> originPoses;
   private ArrayList<FramePose3D> doorPoses;

   public FramePose3D averageOrigin;
   private FramePose3D averageCurrentDoorLocation;
   public FramePose3D newPose = null;

   private DoorLocationPacket latestDoorLocationPacketRecieved;
   private boolean doorOpen = false;
   private float openAngle = 0.15f;
   private float closeAngle = 0.07f;
   private boolean run = false;
   
   private long lastUpdateTime = -1;

   private final ROS2PublisherBasics<TextToSpeechPacket> textToSpeechPublisher;

   
   protected final AtomicReference<DoorLocationPacket> doorLocationLatest = new AtomicReference<DoorLocationPacket>();

   public DoorOpenDetectorBehaviorService(String robotName, String ThreadName, ROS2Node ros2Node, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(robotName, ThreadName, ros2Node);
      textToSpeechPublisher = createPublisher(TextToSpeechPacket.class, ROS2Tools.IHMC_ROOT);

      createSubscriber(DoorLocationPacket.class, PerceptionAPI.OBJECT_DETECTOR_TOOLBOX.withRobot(robotName).withOutput(), doorLocationLatest::set);

      initialize();
   }

   public void run(boolean run)
   {
      this.run = run;
      lastUpdateTime = -1;
      LogTools.info(1, "Start door open detector service = "+run);

      textToSpeechPublisher.publish(MessageTools.createTextToSpeechPacket("Start door open detector service = "+run));

   }

   @Override
   public void initialize()
   {
      // super.initialize();
      originPoses = new ArrayList<>();
      doorPoses = new ArrayList<>();
      averageOrigin = null;
      averageCurrentDoorLocation = null;
      doorOpen = false;
      run = false;
   }

   public boolean isDoorOpen()
   {
      return doorOpen;
   }

   public boolean doorDetected()
   {
      return averageOrigin != null;
   }

   public void reset()
   {
      originPoses.clear();
      doorOpen = false;
      averageOrigin = null;
      lastUpdateTime = -1;
      run = false;
      doorPoses.clear();
   }

   @Override
   public void doThreadAction()
   {
	//TODO, dont know why this has to be here... but without it, the loop freezes.. looking into it next iteration. 20/07/08
      try
      {
         Thread.sleep(100);
      }
      catch (InterruptedException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      if (run)
      {
         latestDoorLocationPacketRecieved = doorLocationLatest.getAndSet(null);
         if (latestDoorLocationPacketRecieved != null)
         {

            newPose = new FramePose3D(ReferenceFrame.getWorldFrame(), latestDoorLocationPacketRecieved.getDoorTransformToWorld());
            // getReportedGoalPoseWorldFrame(newPose);

            //first find the average location of the door to begin with(the doors closed point)
            if (averageOrigin == null)
            {

               originPoses.add(newPose);

               if (latestDoorLocationPacketRecieved.getTrustedPosition())
               {
                  averageOrigin = newPose;
               }

               else if (originPoses.size() >= numberToAverage)
               {
                  averageOrigin = averageFramePoses(originPoses);
               }

            }
            //after the average location has been found, start tracking the door locationtrack the door for movements
            else
            {

               doorPoses.add(newPose);
               if (doorPoses.size() > numberToAverage)
                  doorPoses.remove(0);

               if (doorPoses.size() >= numberToAverage || latestDoorLocationPacketRecieved.getTrustedPosition())
               {

                  if (latestDoorLocationPacketRecieved.getTrustedPosition())
                  {
                     averageCurrentDoorLocation = newPose;
                  }
                  else
                  {
                     averageCurrentDoorLocation = averageFramePoses(doorPoses);
                  }

                  if (averageCurrentDoorLocation != null)
                  {
                     if (!doorOpen && Math.abs(averageOrigin.getYaw() - averageCurrentDoorLocation.getYaw()) > openAngle)
                     {
                        doorOpen = true;
                     }
                     else if (doorOpen && Math.abs(averageOrigin.getYaw() - averageCurrentDoorLocation.getYaw()) < closeAngle)
                     {
                        doorOpen = false;
                     }
                     lastUpdateTime = System.currentTimeMillis();

                  }
                  

               }
            }

            //super.initialize();
         }
      }
   }

   public long getLastupdateTime()
   {
      return lastUpdateTime;
   }
   
   private FramePose3D averageFramePoses(ArrayList<FramePose3D> poses)
   {
      if (poses.size() > 0)
      {
         AverageQuaternionCalculator averageQuaternionCalculator = new AverageQuaternionCalculator();

         averageQuaternionCalculator.reset();

         float numberOfPoses = poses.size();
         FramePose3D aveagedPose = new FramePose3D(ReferenceFrame.getWorldFrame());
         for (FramePose3D pose : poses)
         {

            aveagedPose.setX(aveagedPose.getX() + pose.getX());
            aveagedPose.setY(aveagedPose.getY() + pose.getY());
            aveagedPose.setZ(aveagedPose.getZ() + pose.getZ());
            averageQuaternionCalculator.queueQuaternion(new Quaternion(pose.getOrientation()));

         }

         aveagedPose.setX(aveagedPose.getX() / numberOfPoses);
         aveagedPose.setY(aveagedPose.getY() / numberOfPoses);
         aveagedPose.setZ(aveagedPose.getZ() / numberOfPoses);

         averageQuaternionCalculator.compute();
         Quaternion actualAverageQuat = new Quaternion();
         averageQuaternionCalculator.getAverageQuaternion(actualAverageQuat);
         aveagedPose.getOrientation().set(actualAverageQuat);
         return aveagedPose;
      }
      else
         return null;

   }

}
