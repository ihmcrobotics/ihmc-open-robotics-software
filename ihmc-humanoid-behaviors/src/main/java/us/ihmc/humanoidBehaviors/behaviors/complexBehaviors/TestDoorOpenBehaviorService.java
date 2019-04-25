package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.DoorLocationPacket;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.DoorOpenDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.FiducialDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.ros2.Ros2Node;

public class TestDoorOpenBehaviorService extends AbstractBehavior
{
   boolean isDoorOpen = false;
   protected final ConcurrentListeningQueue<DoorLocationPacket> doorLocationQueue = new ConcurrentListeningQueue<DoorLocationPacket>(10);
   private final DoorOpenDetectorBehaviorService doorOpenDetectorBehaviorService;

   public TestDoorOpenBehaviorService(String robotName, String yoNamePrefix, Ros2Node ros2Node, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(robotName, yoNamePrefix, ros2Node);
      createBehaviorInputSubscriber(DoorLocationPacket.class, doorLocationQueue::put);
      doorOpenDetectorBehaviorService = new DoorOpenDetectorBehaviorService(robotName, yoNamePrefix + "DoorOpenService", ros2Node, yoGraphicsListRegistry);
      doorOpenDetectorBehaviorService.setTargetIDToLocate(50);
      doorOpenDetectorBehaviorService.setExpectedFiducialSize(0.2032);

      registry.addChild(doorOpenDetectorBehaviorService.getYoVariableRegistry());

      addBehaviorService(doorOpenDetectorBehaviorService);
   }

   @Override
   public void onBehaviorEntered()
   {
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
