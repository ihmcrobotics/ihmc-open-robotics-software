package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import java.util.Map;

import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import controller_msgs.msg.dds.SelectionMatrix3DMessage;
import controller_msgs.msg.dds.WeightMatrix3DMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class KinematicsToolboxRigidBodyCommand implements Command<KinematicsToolboxRigidBodyCommand, KinematicsToolboxRigidBodyMessage>
{
   /** This is the unique hash code of the end-effector to be solved for. */
   private long endEffectorNameBasedHashCode;
   /** This is the end-effector to be solved for. */
   private RigidBody endEffector;

   private final FramePose3D desiredPose = new FramePose3D();
   private final FramePose3D controlFramePose = new FramePose3D();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
   private final WeightMatrix6D weightMatrix = new WeightMatrix6D();

   @Override
   public void clear()
   {
      endEffectorNameBasedHashCode = NameBasedHashCodeTools.NULL_HASHCODE;
      endEffector = null;
      desiredPose.setToNaN(ReferenceFrame.getWorldFrame());
      controlFramePose.setToNaN(ReferenceFrame.getWorldFrame());
      selectionMatrix.resetSelection();
      weightMatrix.clear();
   }

   @Override
   public void set(KinematicsToolboxRigidBodyCommand other)
   {
      endEffectorNameBasedHashCode = other.endEffectorNameBasedHashCode;
      endEffector = other.endEffector;
      desiredPose.setIncludingFrame(other.desiredPose);
      controlFramePose.setIncludingFrame(other.controlFramePose);
      selectionMatrix.set(other.selectionMatrix);
      weightMatrix.set(other.weightMatrix);
   }

   @Override
   public void set(KinematicsToolboxRigidBodyMessage message)
   {
      set(message, null, null);
   }

   public void set(KinematicsToolboxRigidBodyMessage message, Map<Long, RigidBody> rigidBodyNamedBasedHashMap,
                   ReferenceFrameHashCodeResolver referenceFrameResolver)
   {
      endEffectorNameBasedHashCode = message.getEndEffectorNameBasedHashCode();
      if (rigidBodyNamedBasedHashMap == null)
         endEffector = null;
      else
         endEffector = rigidBodyNamedBasedHashMap.get(endEffectorNameBasedHashCode);
      desiredPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), message.getDesiredPositionInWorld(), message.getDesiredOrientationInWorld());
      ReferenceFrame referenceFrame = endEffector == null ? null : endEffector.getBodyFixedFrame();
      controlFramePose.setIncludingFrame(referenceFrame, message.getControlFramePositionInEndEffector(), message.getControlFrameOrientationInEndEffector());
      weightMatrix.clear();
      WeightMatrix3DMessage angularWeight = message.getAngularWeightMatrix();
      WeightMatrix3DMessage linearWeight = message.getLinearWeightMatrix();
      weightMatrix.setAngularWeights(angularWeight.getXWeight(), angularWeight.getYWeight(), angularWeight.getZWeight());
      weightMatrix.setLinearWeights(linearWeight.getXWeight(), linearWeight.getYWeight(), linearWeight.getZWeight());

      selectionMatrix.clearSelectionFrame();
      SelectionMatrix3DMessage angularSelection = message.getAngularSelectionMatrix();
      SelectionMatrix3DMessage linearSelection = message.getLinearSelectionMatrix();
      selectionMatrix.setAngularAxisSelection(angularSelection.getXSelected(), angularSelection.getYSelected(), angularSelection.getZSelected());
      selectionMatrix.setLinearAxisSelection(linearSelection.getXSelected(), linearSelection.getYSelected(), linearSelection.getZSelected());

      if (referenceFrameResolver != null)
      {
         ReferenceFrame angularSelectionFrame = referenceFrameResolver.getReferenceFrameFromNameBaseHashCode(angularSelection.getSelectionFrameId());
         ReferenceFrame linearSelectionFrame = referenceFrameResolver.getReferenceFrameFromNameBaseHashCode(linearSelection.getSelectionFrameId());
         selectionMatrix.setSelectionFrames(angularSelectionFrame, linearSelectionFrame);
         ReferenceFrame angularWeightFrame = referenceFrameResolver.getReferenceFrameFromNameBaseHashCode(angularWeight.getWeightFrameId());
         ReferenceFrame linearWeightFrame = referenceFrameResolver.getReferenceFrameFromNameBaseHashCode(linearWeight.getWeightFrameId());
         weightMatrix.setWeightFrames(angularWeightFrame, linearWeightFrame);
      }
   }

   public RigidBody getEndEffector()
   {
      return endEffector;
   }

   public WeightMatrix6D getWeightMatrix()
   {
      return weightMatrix;
   }

   public SelectionMatrix6D getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public FramePose3D getDesiredPose()
   {
      return desiredPose;
   }

   public FramePose3D getControlFramePose()
   {
      return controlFramePose;
   }

   @Override
   public Class<KinematicsToolboxRigidBodyMessage> getMessageClass()
   {
      return KinematicsToolboxRigidBodyMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }
}
