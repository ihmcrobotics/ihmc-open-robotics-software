package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage;
import ihmc_common_msgs.msg.dds.WeightMatrix3DMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

import java.util.Objects;

public class KinematicsToolboxRigidBodyCommand implements Command<KinematicsToolboxRigidBodyCommand, KinematicsToolboxRigidBodyMessage>
{
   private long sequenceId;
   /**
    * This is the unique hash code of the end-effector to be solved for.
    */
   private int endEffectorHashCode;
   /**
    * This is the end-effector to be solved for.
    */
   private RigidBodyBasics endEffector;

   private final FramePose3D desiredPose = new FramePose3D();
   private boolean hasDesiredVelocity;
   private final SpatialVector desiredVelocity = new SpatialVector();
   private final FramePose3D controlFramePose = new FramePose3D();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
   private final WeightMatrix6D weightMatrix = new WeightMatrix6D();
   private double linearRateLimitation, angularRateLimitation;

   @Override
   public void clear()
   {
      sequenceId = 0;
      endEffectorHashCode = 0;
      endEffector = null;
      desiredPose.setToNaN(ReferenceFrame.getWorldFrame());
      desiredVelocity.setToNaN(ReferenceFrame.getWorldFrame());
      controlFramePose.setToNaN(ReferenceFrame.getWorldFrame());
      selectionMatrix.resetSelection();
      weightMatrix.clear();
      hasDesiredVelocity = false;
      linearRateLimitation = -1.0;
      angularRateLimitation = -1.0;
   }

   @Override
   public void set(KinematicsToolboxRigidBodyCommand other)
   {
      sequenceId = other.sequenceId;
      endEffectorHashCode = other.endEffectorHashCode;
      endEffector = other.endEffector;
      desiredPose.setIncludingFrame(other.desiredPose);
      desiredVelocity.setIncludingFrame(other.desiredVelocity);
      controlFramePose.setIncludingFrame(other.controlFramePose);
      selectionMatrix.set(other.selectionMatrix);
      weightMatrix.set(other.weightMatrix);

      hasDesiredVelocity = other.hasDesiredVelocity;
      linearRateLimitation = other.linearRateLimitation;
      angularRateLimitation = other.angularRateLimitation;
   }

   @Override
   public void setFromMessage(KinematicsToolboxRigidBodyMessage message)
   {
      set(message, null, null);
   }

   public void set(KinematicsToolboxRigidBodyMessage message,
                   RigidBodyHashCodeResolver rigidBodyHashCodeResolver,
                   ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver)
   {
      Objects.requireNonNull(rigidBodyHashCodeResolver);
      Objects.requireNonNull(referenceFrameHashCodeResolver);

      sequenceId = message.getSequenceId();
      endEffectorHashCode = message.getEndEffectorHashCode();
      endEffector = rigidBodyHashCodeResolver.castAndGetRigidBody(endEffectorHashCode);
      desiredPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), message.getDesiredPositionInWorld(), message.getDesiredOrientationInWorld());
      desiredVelocity.setToZero(ReferenceFrame.getWorldFrame()); // TODO Consider adding desired velocity to the message
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

      ReferenceFrame angularSelectionFrame = referenceFrameHashCodeResolver.getReferenceFrame(angularSelection.getSelectionFrameId());
      ReferenceFrame linearSelectionFrame = referenceFrameHashCodeResolver.getReferenceFrame(linearSelection.getSelectionFrameId());
      selectionMatrix.setSelectionFrames(angularSelectionFrame, linearSelectionFrame);
      ReferenceFrame angularWeightFrame = referenceFrameHashCodeResolver.getReferenceFrame(angularWeight.getWeightFrameId());
      ReferenceFrame linearWeightFrame = referenceFrameHashCodeResolver.getReferenceFrame(linearWeight.getWeightFrameId());
      weightMatrix.setWeightFrames(angularWeightFrame, linearWeightFrame);

      hasDesiredVelocity = message.getHasDesiredAngularVelocity() && message.getHasDesiredLinearVelocity();
      desiredVelocity.setIncludingFrame(ReferenceFrame.getWorldFrame(), message.getDesiredAngularVelocityInWorld(), message.getDesiredLinearVelocityInWorld());

      linearRateLimitation = message.getLinearRateLimitation();
      angularRateLimitation = message.getAngularRateLimitation();
   }

   public void setEndEffector(RigidBodyBasics endEffector)
   {
      this.endEffector = endEffector;
   }

   public void setHasDesiredVelocity(boolean hasDesiredVelocity)
   {
      this.hasDesiredVelocity = hasDesiredVelocity;
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

   public SpatialVector getDesiredVelocity()
   {
      return desiredVelocity;
   }

   public FramePose3D getControlFramePose()
   {
      return controlFramePose;
   }

   public boolean getHasDesiredVelocity()
   {
      return hasDesiredVelocity;
   }

   public double getLinearRateLimitation()
   {
      return linearRateLimitation;
   }

   public double getAngularRateLimitation()
   {
      return angularRateLimitation;
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
