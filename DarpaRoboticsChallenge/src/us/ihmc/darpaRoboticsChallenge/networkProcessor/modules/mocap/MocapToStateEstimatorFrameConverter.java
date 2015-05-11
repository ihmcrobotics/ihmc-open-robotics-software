package us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.mocap;

import optitrack.MocapRigidBody;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.partNames.NeckJointName;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;


/**
 * This module converts MocapRigidBodies to robot world and sends them as packets the UI understands
 * This has an Atlas specific assumption!!!!!!!! 
 * This module uses the neck_ry joint as the bridge between mocap and robot
 */
public class MocapToStateEstimatorFrameConverter
{
   /** Get robot configuration from controller **/
   private final RobotDataReceiver robotDataReceiver;

   /** Atlas specific assumption!!! head frame for atlas coincides with neck_ry joint */
   private final ReferenceFrame robotHeadFrame;
   
   /** in world frame, uses mocapHeadPoseInZUp as it's transform to parent */
   private final ReferenceFrame mocapHeadFrame;
   
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

   private boolean freezeMocapUpdates = false;
   
   public MocapToStateEstimatorFrameConverter(DRCRobotModel robotModel, PacketCommunicator mocapModulePacketCommunicator)
   {
      SDFFullRobotModel fullRobotModel = robotModel.createFullRobotModel();
      robotDataReceiver = new RobotDataReceiver(fullRobotModel, null);
      ReferenceFrames referenceFrames = robotDataReceiver.getReferenceFrames();
      robotHeadFrame = referenceFrames.getNeckFrame(NeckJointName.LOWER_NECK_PITCH);
      

      mocapHeadFrame = new ReferenceFrame("headInMocapFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(mocapHeadPoseInZUp);
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
   
   public RigidBodyTransform getTransformFromMocapHeadToRobotHead()
   {
      robotDataReceiver.updateRobotModel();
      mocapHeadFrame.getTransformToDesiredFrame(transformFromMocapHeadToRobotHead , robotHeadFrame);
      return transformFromMocapHeadToRobotHead;
   }
   
   public ReferenceFrame getMocapHeadFrame()
   {
      return mocapHeadFrame;
   }

   public void update(MocapRigidBody mocapObject)
   {
      if(freezeMocapUpdates)
      {
         mocapObject.getPose(workingRigidBodyTransform);
         workingRigidBodyTransform.multiply(mocapJigCalibrationTransform);
         workingRigidBodyTransform.multiply(transformFromMocapHeadCentroidToHeadRoot);
         mocapHeadPoseInZUp.set(workingRigidBodyTransform);
         mocapHeadFrame.update();
      }
   }

   public void convertMocapPoseToRobotFrame(RigidBodyTransform pose)
   {
      pose.multiply(getTransformFromMocapHeadToRobotHead(), pose);
      
   }

   public void freezeMocapUpdates(boolean freezeMocapUpdates)
   {
      this.freezeMocapUpdates  = freezeMocapUpdates;
      
   }
}
