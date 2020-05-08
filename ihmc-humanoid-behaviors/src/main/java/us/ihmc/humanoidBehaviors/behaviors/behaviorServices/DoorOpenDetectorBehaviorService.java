package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.DoorLocationPacket;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.ros2.Ros2Node;

public class DoorOpenDetectorBehaviorService extends ThreadedBehaviorService//FiducialDetectorBehaviorService
{

   private int numberToAverage = 10;
   private ArrayList<FramePose3D> originPoses;
   private ArrayList<FramePose3D> doorPoses;

   public FramePose3D averageOrigin;
   private FramePose3D averageCurrentDoorLocation;
   public FramePose3D newPose = null;

   private DoorLocationPacket latestDoorLocationPacketRecieved;
   private boolean doorOpen = false;
   private float openDistance = 0.0308f;
   private boolean run = false;

   protected final AtomicReference<DoorLocationPacket> doorLocationQueue = new AtomicReference<DoorLocationPacket>();

   public DoorOpenDetectorBehaviorService(String robotName, String ThreadName, Ros2Node ros2Node, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(robotName, ThreadName, ros2Node);//, yoGraphicsListRegistry);

      createSubscriber(DoorLocationPacket.class, IHMCHumanoidBehaviorManager.getSubscriberTopicNameGenerator(robotName), doorLocationQueue::set);

      initialize();
   }

   public void run(boolean run)
   {
      this.run = run;
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
      run = false;
      doorPoses.clear();

   }

   @Override
   public void doThreadAction()
   {

      //super.doThreadAction();
      if (true)
      {
         latestDoorLocationPacketRecieved = doorLocationQueue.getAndSet(null);
         if (latestDoorLocationPacketRecieved != null)
         {
            System.out.println("recieved pose");
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
                     if (averageCurrentDoorLocation != null && averageOrigin.getPositionDistance(averageCurrentDoorLocation) > openDistance)
                     {
                        doorOpen = true;
                     }
                     else
                     {
                        doorOpen = false;
                     }
                  
               }
            }

            //super.initialize();
         }
      }
   }

   private FramePose3D averageFramePoses(ArrayList<FramePose3D> poses)
   {
      if (poses.size() > 0)
      {
         float numberOfPoses = poses.size();
         FramePose3D aveagedPose = new FramePose3D(ReferenceFrame.getWorldFrame());
         for (FramePose3D pose : poses)
         {
            aveagedPose.setX(aveagedPose.getX() + pose.getX());
            aveagedPose.setY(aveagedPose.getY() + pose.getY());
            aveagedPose.setZ(aveagedPose.getZ() + pose.getZ());
         }
         aveagedPose.setX(aveagedPose.getX() / numberOfPoses);
         aveagedPose.setY(aveagedPose.getY() / numberOfPoses);
         aveagedPose.setZ(aveagedPose.getZ() / numberOfPoses);
         return aveagedPose;
      }
      else
         return null;

   }

}
