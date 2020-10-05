package us.ihmc.avatar.networkProcessor.objectDetectorToolBox;

import controller_msgs.msg.dds.DetectedFiducialPacket;
import controller_msgs.msg.dds.DoorLocationPacket;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ObjectDetectorToolboxController extends ToolboxController
{

   private final ConcurrentListeningQueue<DetectedFiducialPacket> fiducialPacketQueue = new ConcurrentListeningQueue<DetectedFiducialPacket>(20);

   public ObjectDetectorToolboxController(FullHumanoidRobotModel fullRobotModel, StatusMessageOutputManager statusOutputManager,
                                          YoRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);
   }

   @Override
   public boolean initialize()
   {
      return true;
   }

   public void receivedPacket(DetectedFiducialPacket packet)
   {
      if (packet != null)
         fiducialPacketQueue.put(packet);
   }

   @Override
   public void updateInternal()
   {
      while (fiducialPacketQueue.isNewPacketAvailable())
      {
         DetectedFiducialPacket newPacket = fiducialPacketQueue.poll();
         if (newPacket != null)
         {
            decodeFiducial(newPacket);
         }
      }
   }

   private void decodeFiducial(DetectedFiducialPacket fiducial)
   {
      if (fiducial.fiducial_id_ == 50)
      {
         decodePushDoorFiducial(fiducial);
      }
      else if (fiducial.fiducial_id_ == 150)
      {
         decodePullDoorFiducial(fiducial);
      }
      else
      {
         LogTools.debug("fiducial " + fiducial.fiducial_id_ + " detected, but not implemented in ObejctDetetorToolboxController");
      }
   }

   private void decodePushDoorFiducial(DetectedFiducialPacket fiducial)
   {
      Pose3D tmpFP = fiducial.fiducial_transform_to_world_;

      tmpFP.appendPitchRotation(Math.toRadians(90));
      tmpFP.appendYawRotation(0);
      tmpFP.appendRollRotation(Math.toRadians(-90));

      tmpFP.appendPitchRotation(-tmpFP.getPitch());

      FramePose3D doorFrame = new FramePose3D(tmpFP);
      doorFrame.appendTranslation(0.025875, 0.68183125, -1.1414125);

      Pose3D pose = new Pose3D(doorFrame.getPosition(), doorFrame.getOrientation());

      pose.appendYawRotation(Math.toRadians(-90));

      reportMessage(HumanoidMessageTools.createDoorLocationPacket(pose, DoorLocationPacket.PUSH_HANDLE_RIGHT));

   }
  

   
   private void decodePullDoorFiducial(DetectedFiducialPacket fiducial)
   {
      Pose3D tmpFP = fiducial.fiducial_transform_to_world_;

      tmpFP.appendPitchRotation(Math.toRadians(90));
      tmpFP.appendYawRotation(0);
      tmpFP.appendRollRotation(Math.toRadians(-90));

      tmpFP.appendPitchRotation(-tmpFP.getPitch());

      FramePose3D doorFrame = new FramePose3D(tmpFP);
      doorFrame.appendTranslation(0.025875, -0.68183125, -1.1414125);

      Pose3D pose = new Pose3D(doorFrame.getPosition(), doorFrame.getOrientation());

      pose.appendYawRotation(Math.toRadians(90));

      reportMessage(HumanoidMessageTools.createDoorLocationPacket(pose, DoorLocationPacket.PULL_HANDLE_LEFT));

   }
   
 
   

   @Override
   public boolean isDone()
   {
      return false;
   }
}
