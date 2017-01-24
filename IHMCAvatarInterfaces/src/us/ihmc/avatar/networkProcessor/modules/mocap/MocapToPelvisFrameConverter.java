package us.ihmc.avatar.networkProcessor.modules.mocap;

import optiTrack.MocapRigidBody;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;

import java.util.HashMap;

public class MocapToPelvisFrameConverter
{
   HashMap<Integer, ReferenceFrame> mocapReferenceFrames = new HashMap<Integer, ReferenceFrame>();
   HashMap<Integer, RigidBodyTransform> mocapRigidBodyTransforms = new HashMap<Integer, RigidBodyTransform>();

   private final HumanoidRobotDataReceiver robotDataReceiver;

   private final ReferenceFrame robotPelvisFrame;
   private final ReferenceFrame mocapPelvisFrame;

   /** in world frame, uses mocapHeadPoseInZUp as it's transform to parent */
   private final ReferenceFrame mocapOffsetFrame;

   /** the transform from mocap head in z-up to mocap origin */
   private final RigidBodyTransform mocapPelvisPoseInZUp = new RigidBodyTransform();

   /** the change in frame between pelvis in mocap to pelvis in robot world */
   private final RigidBodyTransform transformFromMocapPelvisToRobotPelvis = new RigidBodyTransform();

   /** the transform from the mocap centroid to robot pelvis */
   private final RigidBodyTransform transformFromMocapHeadCentroidToHeadRoot = new RigidBodyTransform();

   /** the calibration transform found by aligning the point cloud with the mocap objects*/
   private final RigidBodyTransform mocapJigCalibrationTransform = new RigidBodyTransform();

   /** used to pack the mocap pose and multiply the calibration and transform to headroot */
   private final RigidBodyTransform workingRigidBodyTransform = new RigidBodyTransform();

   private boolean enableMocapUpdates = false;

   private ReferenceFrame mocapOrigin = new ReferenceFrame("mocapOrigin", ReferenceFrame.getWorldFrame())
   {

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.setIdentity();
      }
   };


   public MocapToPelvisFrameConverter(DRCRobotModel robotModel, PacketCommunicator mocapModulePacketCommunicator)
   {
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      robotDataReceiver = new HumanoidRobotDataReceiver(fullRobotModel, null);
      HumanoidReferenceFrames referenceFrames = robotDataReceiver.getReferenceFrames();
      robotPelvisFrame = referenceFrames.getPelvisFrame();

      mocapPelvisFrame = new ReferenceFrame("pelvisInMocapFrame", mocapOrigin)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(mocapPelvisPoseInZUp);
         }
      };

      mocapOffsetFrame = new ReferenceFrame("mocapOffsetFrame", mocapOrigin)
      {

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(transformFromMocapPelvisToRobotPelvis);
         }
      };

      mocapModulePacketCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);
   }

   public RigidBodyTransform convertMocapPoseToRobotFrame(MocapRigidBody mocapRigidBody)
   {
      int id = mocapRigidBody.getId();

      if(!mocapReferenceFrames.containsKey(id))
      {
         ReferenceFrame mocapObjectFrame = createReferenceFrameForMocapObject(id);
         mocapReferenceFrames.put(id, mocapObjectFrame);
      }

      mocapRigidBody.packPose(mocapRigidBodyTransforms.get(id));

      ReferenceFrame referenceFrame = mocapReferenceFrames.get(id);
      referenceFrame.update();

      return referenceFrame.getTransformToDesiredFrame(mocapOffsetFrame);
   }

   private ReferenceFrame createReferenceFrameForMocapObject(int id)
   {
      final RigidBodyTransform mocapRigidBodyTransform = new RigidBodyTransform();
      mocapRigidBodyTransforms.put(id, mocapRigidBodyTransform);
      ReferenceFrame mocapObjectFrame = new ReferenceFrame("mocapObject" + id, mocapOrigin )
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(mocapRigidBodyTransform);
         }
      };
      return mocapObjectFrame;
   }
}
