package us.ihmc.avatar.networkProcessor.modules.mocap;

import java.util.HashMap;

import optiTrack.MocapRigidBody;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;


/**
 * This module converts MocapRigidBodies to robot world and sends them as packets the UI understands
 * This has an Atlas specific assumption!!!!!!!! 
 * This module uses the neck_ry joint as the bridge between mocap and robot
 */
public class MocapToHeadFrameConverter
{
   HashMap<Integer, ReferenceFrame> mocapReferenceFrames = new HashMap<Integer, ReferenceFrame>();
   HashMap<Integer, RigidBodyTransform> mocapRigidBodyTransforms = new HashMap<Integer, RigidBodyTransform>();
   
   /** Get robot configuration from controller **/
   private final HumanoidRobotDataReceiver robotDataReceiver;

   /** Atlas specific assumption!!! head frame for atlas coincides with neck_ry joint */
   private final ReferenceFrame robotHeadFrame;
   
   /** in world frame, uses mocapHeadPoseInZUp as it's transform to parent */
   private final ReferenceFrame mocapHeadFrame;
   
   /** in world frame, uses mocapHeadPoseInZUp as it's transform to parent */
   private final ReferenceFrame mocapOffsetFrame;
   
   /** the transform from mocap head in z-up to mocap origin */
   private final RigidBodyTransform mocapHeadPoseInZUp = new RigidBodyTransform();
   
   /** the change in frame between head in mocap to head in robot world */
   private final RigidBodyTransform transformFromMocapHeadToRobotHead = new RigidBodyTransform();
   
   /** the transform from the mocap centroid to robot head root */
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
   
   public MocapToHeadFrameConverter(DRCRobotModel robotModel, PacketCommunicator mocapModulePacketCommunicator)
   {
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      robotDataReceiver = new HumanoidRobotDataReceiver(fullRobotModel, null);
      HumanoidReferenceFrames referenceFrames = robotDataReceiver.getReferenceFrames();
      robotHeadFrame = referenceFrames.getNeckFrame(NeckJointName.PROXIMAL_NECK_PITCH);
      

      mocapHeadFrame = new ReferenceFrame("headInMocapFrame", mocapOrigin)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(mocapHeadPoseInZUp);
         }
      };
      
      mocapOffsetFrame = new ReferenceFrame("mocapOffsetFrame", mocapOrigin)
      {
         
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(transformFromMocapHeadToRobotHead);
            
         }
      };
      
      mocapModulePacketCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);
   }
   
   public void setTransformFromMocapCentroidToHeadRoot(RigidBodyTransform transformFromMocapCentroidToHeadRoot)
   {
      this.transformFromMocapHeadCentroidToHeadRoot.set(transformFromMocapCentroidToHeadRoot);
   }
   
   public void setMocapJigCalibrationTransform(RigidBodyTransform headJigCalibrationTransform)
   {
      this.mocapJigCalibrationTransform.set(headJigCalibrationTransform);
   }
   
   public void update(MocapRigidBody mocapObject)
   {
      if(enableMocapUpdates)
      {
         mocapObject.packPose(workingRigidBodyTransform);
         workingRigidBodyTransform.multiply(mocapJigCalibrationTransform);
         workingRigidBodyTransform.multiply(transformFromMocapHeadCentroidToHeadRoot);
         mocapHeadPoseInZUp.set(workingRigidBodyTransform);
         
         mocapHeadFrame.update();
         robotDataReceiver.updateRobotModel();
         mocapHeadFrame.getTransformToDesiredFrame(transformFromMocapHeadToRobotHead , robotHeadFrame);
         mocapOffsetFrame.update();
      }
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

   public void enableMocapUpdates(boolean freezeMocapUpdates)
   {
      this.enableMocapUpdates  = freezeMocapUpdates;
      
   }
}
