package us.ihmc.avatar.networkProcessor.modules;

import java.util.concurrent.atomic.AtomicReference;

import sensor_msgs.PointCloud2;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.sensing.MultisenseMocapExperimentPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.MultisenseTest;
import us.ihmc.humanoidRobotics.communication.packets.sensing.MultisenseTest.MultisenseFrameName;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;

public class MultisensePointCloudReceiver extends RosPointCloudSubscriber 
{
   private final PacketCommunicator packetCommunicator;
   private final MultisenseTest testInfo;
   private final MultisenseFrameName frame;
   private final AtomicReference<RigidBodyTransform> headInMocapFrame = new AtomicReference<RigidBodyTransform>(new RigidBodyTransform());
   private ReferenceFrame pointCloudReferenceFrame;
   private final ReferenceFrame headRootReferenceFrame;
   private final HumanoidRobotDataReceiver robotDataReceiver;
   
   public MultisensePointCloudReceiver(PacketCommunicator packetCommunicator, MultisenseTest testInfo, DRCRobotModel robotModel)
   {
      this.packetCommunicator = packetCommunicator;
      this.testInfo = testInfo;
      this.frame = testInfo.getFrame();
      
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      robotDataReceiver = new HumanoidRobotDataReceiver(fullRobotModel, null);
      HumanoidReferenceFrames referenceFrames = robotDataReceiver.getReferenceFrames();
      
      this.headRootReferenceFrame = referenceFrames.getNeckFrame(NeckJointName.PROXIMAL_NECK_PITCH);
      setupReferenceFrames(referenceFrames);
   }

   public void setHeadRootInWorldFromMocap(RigidBodyTransform headRootInWorld)
   {
      headInMocapFrame.set(new RigidBodyTransform(headRootInWorld));
   }
   
   /**
    * received a point cloud from the multisense
    * transform the points using the mocap data
    */
   @Override
   public void onNewMessage(PointCloud2 pointCloud)
   {
      robotDataReceiver.updateRobotModel();
      pointCloudReferenceFrame.update();
      UnpackedPointCloud pointCloudData = unpackPointsAndIntensities(pointCloud);
      Point3D[] points = pointCloudData.getPoints();
      
      RigidBodyTransform pointTransform = new RigidBodyTransform(headInMocapFrame.get());
      RigidBodyTransform transformFromPointCloudOriginToHeadRoot = pointCloudReferenceFrame.getTransformToDesiredFrame(headRootReferenceFrame);
      pointTransform.multiply(transformFromPointCloudOriginToHeadRoot);
      
      for(int i = 0; i < points.length; i++)
      {
    	  pointTransform.transform(points[i]);
      }
      
      MultisenseMocapExperimentPacket pointCloudPacket = new MultisenseMocapExperimentPacket();
      pointCloudPacket.setDestination(PacketDestination.UI);
      pointCloudPacket.setPointCloud(points, testInfo);
      packetCommunicator.send(pointCloudPacket);
   }
   
   public MultisenseFrameName getFrame()
   {
      return frame;
   }
   
   private void setupReferenceFrames(HumanoidReferenceFrames referenceFrames)
   {
      switch (frame)
      {
         case LEFT_CAMERA_OPTICAL_FRAME:
         {
            this.pointCloudReferenceFrame = new ReferenceFrame("leftCameraOpticalFrame", headRootReferenceFrame)
            {
   
               /**
                * 
                */
               private static final long serialVersionUID = 2902570649318946274L;

               @Override
               protected void updateTransformToParent(RigidBodyTransform transformToParent)
               {
                  transformToParent.setRotationEulerAndZeroTranslation(-Math.PI / 2, 0.0, -Math.PI / 2);
                  transformToParent.setTranslation(0, 0.035, -0.002);
               }
            };
            break;
         }
         case HEAD_ROOT:
         {
            this.pointCloudReferenceFrame = referenceFrames.getNeckFrame(NeckJointName.PROXIMAL_NECK_PITCH);
            break;
         }
         case U_TORSO:
         {
            this.pointCloudReferenceFrame = referenceFrames.getSpineFrame(SpineJointName.SPINE_ROLL);
            break;
         }
   
         case M_TORSO:
         {
            this.pointCloudReferenceFrame = referenceFrames.getSpineFrame(SpineJointName.SPINE_PITCH);
            break;
         }
         case L_TORSO:
         {
            this.pointCloudReferenceFrame = referenceFrames.getSpineFrame(SpineJointName.SPINE_YAW);
            break;
         }
   
         case PELVIS:
         {
            this.pointCloudReferenceFrame = referenceFrames.getPelvisFrame();
            break;
         }
         case WORLD:
         default:
         {
            this.pointCloudReferenceFrame = ReferenceFrame.getWorldFrame();
            break;
         }
      }
   }

}
