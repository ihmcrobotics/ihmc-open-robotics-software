package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import controller_msgs.msg.dds.SelectionMatrix3DMessage;
import controller_msgs.msg.dds.WeightMatrix3DMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class KinematicsToolboxRigidBodyCommand implements Command<KinematicsToolboxRigidBodyCommand, KinematicsToolboxRigidBodyMessage>
{
   private long sequenceId;
   /** This is the unique hash code of the end-effector to be solved for. */
   private int endEffectorHashCode;
   /** This is the end-effector to be solved for. */
   private RigidBodyBasics endEffector;

   private final FramePose3D desiredPose = new FramePose3D();
   private final FramePose3D controlFramePose = new FramePose3D();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
   private final WeightMatrix6D weightMatrix = new WeightMatrix6D();

   @Override
   public void clear()
   {
      sequenceId = 0;
      endEffectorHashCode = 0;
      endEffector = null;
      desiredPose.setToNaN(ReferenceFrame.getWorldFrame());
      controlFramePose.setToNaN(ReferenceFrame.getWorldFrame());
      selectionMatrix.resetSelection();
      weightMatrix.clear();
   }

   @Override
   public void set(KinematicsToolboxRigidBodyCommand other)
   {
      sequenceId = other.sequenceId;
      endEffectorHashCode = other.endEffectorHashCode;
      endEffector = other.endEffector;
      desiredPose.setIncludingFrame(other.desiredPose);
      controlFramePose.setIncludingFrame(other.controlFramePose);
      selectionMatrix.set(other.selectionMatrix);
      weightMatrix.set(other.weightMatrix);
   }

   @Override
   public void setFromMessage(KinematicsToolboxRigidBodyMessage message)
   {
      set(message, null, null);
   }

   public void set(KinematicsToolboxRigidBodyMessage message, RigidBodyHashCodeResolver rigidBodyHashCodeResolver,
                   ReferenceFrameHashCodeResolver referenceFrameResolver)
   {
      sequenceId = message.getSequenceId();
      endEffectorHashCode = message.getEndEffectorHashCode();
      if (rigidBodyHashCodeResolver == null)
         endEffector = null;
      else
         endEffector = (RigidBodyBasics) rigidBodyHashCodeResolver.getRigidBody(endEffectorHashCode);
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
         ReferenceFrame angularSelectionFrame = referenceFrameResolver.getReferenceFrame(angularSelection.getSelectionFrameId());
         ReferenceFrame linearSelectionFrame = referenceFrameResolver.getReferenceFrame(linearSelection.getSelectionFrameId());
         selectionMatrix.setSelectionFrames(angularSelectionFrame, linearSelectionFrame);
         ReferenceFrame angularWeightFrame = referenceFrameResolver.getReferenceFrame(angularWeight.getWeightFrameId());
         ReferenceFrame linearWeightFrame = referenceFrameResolver.getReferenceFrame(linearWeight.getWeightFrameId());
         weightMatrix.setWeightFrames(angularWeightFrame, linearWeightFrame);
      }
   }

   public RigidBodyBasics getEndEffector()
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

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
