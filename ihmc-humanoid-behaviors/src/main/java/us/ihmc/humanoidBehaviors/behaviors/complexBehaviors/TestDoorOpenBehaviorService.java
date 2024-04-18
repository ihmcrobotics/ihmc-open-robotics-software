package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import perception_msgs.msg.dds.DoorLocationPacket;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.DoorOpenDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.ros2.ROS2Node;

public class TestDoorOpenBehaviorService extends AbstractBehavior
{
   boolean isDoorOpen = false;
   protected final ConcurrentListeningQueue<DoorLocationPacket> doorLocationQueue = new ConcurrentListeningQueue<DoorLocationPacket>(10);
   private final DoorOpenDetectorBehaviorService doorOpenDetectorBehaviorService;
 //  private final FiducialDetectorBehaviorService fiducialDetectorBehaviorService;

   private ROS2PublisherBasics<DoorLocationPacket> doorToBehaviorPublisher;
   private ROS2PublisherBasics<DoorLocationPacket> doorToUIPublisher;

   public TestDoorOpenBehaviorService(String robotName, String yoNamePrefix, ROS2Node ros2Node, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(robotName, yoNamePrefix, ros2Node);
      createBehaviorInputSubscriber(DoorLocationPacket.class, doorLocationQueue::put);
      doorOpenDetectorBehaviorService = new DoorOpenDetectorBehaviorService(robotName, yoNamePrefix + "DoorOpenService", ros2Node, yoGraphicsListRegistry);
 

      registry.addChild(doorOpenDetectorBehaviorService.getYoVariableRegistry());

      addBehaviorService(doorOpenDetectorBehaviorService);
     

        doorToBehaviorPublisher = createPublisher(DoorLocationPacket.class, behaviorInputTopic);
        doorToUIPublisher = createBehaviorOutputPublisher(DoorLocationPacket.class);
        
   }

   @Override
   public void onBehaviorEntered()
   {
      publishTextToSpeech("watching the door");
      doorOpenDetectorBehaviorService.initialize();
      doorOpenDetectorBehaviorService.reset();
      doorOpenDetectorBehaviorService.run(true);
   }

   @Override
   public void doControl()
   {
      if (doorOpenDetectorBehaviorService.newPose != null)
      {
         Point3D location = new Point3D();
         Quaternion orientation = new Quaternion();
         doorOpenDetectorBehaviorService.newPose.get(location, orientation);
         publishUIPositionCheckerPacket(location, orientation);
      }

     
      if (isDoorOpen != doorOpenDetectorBehaviorService.isDoorOpen())
      {
         isDoorOpen = doorOpenDetectorBehaviorService.isDoorOpen();
         if (isDoorOpen)
            publishTextToSpeech("Door is Open");
         else
            publishTextToSpeech("Door is Closed");
      }

   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   @Override
   public void onBehaviorExited()
   {
      isDoorOpen = false;
      doorOpenDetectorBehaviorService.destroy();
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

}
