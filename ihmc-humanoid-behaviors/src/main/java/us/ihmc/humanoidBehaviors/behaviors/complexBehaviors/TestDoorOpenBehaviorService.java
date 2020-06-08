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
   private final FiducialDetectorBehaviorService fiducialDetectorBehaviorService;
   
   private IHMCROS2Publisher<DoorLocationPacket> doorToBehaviorPublisher;
   private IHMCROS2Publisher<DoorLocationPacket> doorToUIPublisher;

   public TestDoorOpenBehaviorService(String robotName, String yoNamePrefix, Ros2Node ros2Node, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(robotName, yoNamePrefix, ros2Node);
      createBehaviorInputSubscriber(DoorLocationPacket.class, doorLocationQueue::put);
      doorOpenDetectorBehaviorService = new DoorOpenDetectorBehaviorService(robotName, yoNamePrefix + "DoorOpenService", ros2Node, yoGraphicsListRegistry);
      //doorOpenDetectorBehaviorService.setTargetIDToLocate(50);
      //doorOpenDetectorBehaviorService.setExpectedFiducialSize(0.2032);

      registry.addChild(doorOpenDetectorBehaviorService.getYoVariableRegistry());

      addBehaviorService(doorOpenDetectorBehaviorService);
      
      fiducialDetectorBehaviorService = new FiducialDetectorBehaviorService(robotName, yoNamePrefix + "SearchForDoorFiducial1", ros2Node,
              yoGraphicsListRegistry);
      	fiducialDetectorBehaviorService.setTargetIDToLocate(50);
      	fiducialDetectorBehaviorService.setExpectedFiducialSize(0.2032);

      	registry.addChild(fiducialDetectorBehaviorService.getYoVariableRegistry());

      	addBehaviorService(fiducialDetectorBehaviorService);
      
//      fiducialDetectorBehaviorService = new FiducialDetectorBehaviorService(robotName, yoNamePrefix + "SearchForDoorFiducial1", ros2Node,
//                                                                            yoGraphicsListRegistry);
//      fiducialDetectorBehaviorService.setTargetIDToLocate(50);
//      fiducialDetectorBehaviorService.setExpectedFiducialSize(0.2032);
//
//      registry.addChild(fiducialDetectorBehaviorService.getYoVariableRegistry());
//
//      addBehaviorService(fiducialDetectorBehaviorService);
        doorToBehaviorPublisher = createPublisher(DoorLocationPacket.class, behaviorInputTopic);
        doorToUIPublisher = createBehaviorPublisher(DoorLocationPacket.class);
        
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
	   
	   
	   if (fiducialDetectorBehaviorService.getGoalHasBeenLocated())
	      {

	         FramePose3D tmpFP = new FramePose3D();
	         fiducialDetectorBehaviorService.getReportedGoalPoseWorldFrame(tmpFP);

	         tmpFP.appendPitchRotation(Math.toRadians(90));
	         tmpFP.appendYawRotation(0);
	         tmpFP.appendRollRotation(Math.toRadians(-90));

	         tmpFP.appendPitchRotation(-tmpFP.getPitch());

	         FramePose3D doorFrame = new FramePose3D(tmpFP);
	         doorFrame.appendTranslation(0.025875, 0.68183125, -1.1414125);

	         Pose3D pose = new Pose3D(doorFrame.getPosition(), doorFrame.getOrientation());

	         //publishTextToSpeech("Recieved Door Location From fiducial");
	         pose.appendYawRotation(Math.toRadians(-90));

	         Point3D location = new Point3D();
	         Quaternion orientation = new Quaternion();
	         pose.get(location, orientation);
	         publishUIPositionCheckerPacket(location, orientation);

	         doorToBehaviorPublisher.publish(HumanoidMessageTools.createDoorLocationPacket(pose));
	         doorToUIPublisher.publish(HumanoidMessageTools.createDoorLocationPacket(pose));
	      }
	   
	   
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
