package us.ihmc.avatar.mocapRetarget;

import us.ihmc.avatar.mocapRetarget.bvh.MotionFrame;
import us.ihmc.avatar.mocapRetarget.bvh.SkeletonHierarchy.JointInfo;
import us.ihmc.avatar.mocapRetarget.bvh.SkeletonHierarchy.SkeletonHierarchy;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseKinematicsSolver;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;

import java.rmi.server.Skeleton;
import java.util.LinkedHashMap;
import java.util.Map;

public class CoordinateTransformer
{
   private SkeletonHierarchy skeleton;
   private Map<String, RigidBodyTransform> frameTransforms = new LinkedHashMap<>();

   public CoordinateTransformer(SkeletonHierarchy skeleton) {
      this.skeleton = skeleton;
   }

   public Map<String, RigidBodyTransform> buildGlobalTransforms(MotionFrame frame) {
      return frameTransforms;
   }

   public RigidBodyTransform buildLocalTransform(MotionFrame frame, JointInfo joint) {
     RigidBodyTransform localTransform = new RigidBodyTransform();
     localTransform.setIdentity();

      for (int i = joint.channelStartIndex(); i < (joint.channelStartIndex() + joint.channelCount()); i++) {
         String channelName = joint.channels().get(i - joint.channelStartIndex());
         if (channelName.endsWith("rotation")) {
            double radAngle = Math.toRadians(frame.channelData()[i]);
            if (channelName.startsWith("X")) {
               localTransform.getRotation().appendRollRotation(radAngle);
            } else if (channelName.startsWith("Y")) {
               localTransform.getRotation().appendPitchRotation(radAngle);
            } else if (channelName.startsWith("Z")) {
               localTransform.getRotation().appendYawRotation(radAngle);
            }
         } else if (channelName.endsWith("position")) {
            double translation = frame.channelData()[i];
            if (channelName.startsWith("X")) {
               localTransform.getTranslation().setX(translation);
            } else if (channelName.startsWith("Y")) {
               localTransform.getTranslation().setY(translation);
            } else if (channelName.startsWith("Z")) {
               localTransform.getTranslation().setZ(translation);
            }
         }
      }
     return localTransform;
   }

   public void setSkeleton(SkeletonHierarchy hierarchy) {
      skeleton = hierarchy;
   }

}
