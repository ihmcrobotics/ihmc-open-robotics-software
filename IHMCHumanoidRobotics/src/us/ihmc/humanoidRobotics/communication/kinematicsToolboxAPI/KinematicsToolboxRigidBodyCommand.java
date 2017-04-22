package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.KinematicsToolboxRigidBodyMessage;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.nameBasedHashCode.NameBasedHashCodeTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class KinematicsToolboxRigidBodyCommand implements Command<KinematicsToolboxRigidBodyCommand, KinematicsToolboxRigidBodyMessage>
{
   /** This is the unique hash code of the end-effector to be solved for. */
   private long endEffectorNameBasedHashCode;
   /** This is the end-effector to be solved for. */
   private RigidBody endEffector;

   private final FramePose desiredPose = new FramePose();
   private final FramePose controlFramePose = new FramePose();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
   private final DenseMatrix64F weightVector = new DenseMatrix64F(6, 1);

   @Override
   public void clear()
   {
      endEffectorNameBasedHashCode = NameBasedHashCodeTools.NULL_HASHCODE;
      endEffector = null;
      desiredPose.setToNaN(ReferenceFrame.getWorldFrame());
      controlFramePose.setToNaN(ReferenceFrame.getWorldFrame());
      selectionMatrix.resetSelection();
      weightVector.zero();
   }

   @Override
   public void set(KinematicsToolboxRigidBodyCommand other)
   {
      endEffectorNameBasedHashCode = other.endEffectorNameBasedHashCode;
      endEffector = other.endEffector;
      desiredPose.setIncludingFrame(other.desiredPose);
      controlFramePose.setIncludingFrame(other.controlFramePose);
      selectionMatrix.set(other.selectionMatrix);
      weightVector.set(other.weightVector);
   }

   @Override
   public void set(KinematicsToolboxRigidBodyMessage message)
   {
      set(message, null, null);
   }

   public void set(KinematicsToolboxRigidBodyMessage message, Map<Long, RigidBody> rigidBodyNamedBasedHashMap, ReferenceFrameHashCodeResolver referenceFrameResolver)
   {
      endEffectorNameBasedHashCode = message.getEndEffectorNameBasedHashCode();
      if (rigidBodyNamedBasedHashMap == null)
         endEffector = null;
      else
         endEffector = rigidBodyNamedBasedHashMap.get(endEffectorNameBasedHashCode);
      message.getDesiredPose(desiredPose);
      message.getControlFramePose(endEffector, controlFramePose);
      message.getWeightVector(weightVector);

      message.getSelectionMatrix(selectionMatrix);

      if (referenceFrameResolver != null)
      {
         ReferenceFrame angularSelectionFrame = referenceFrameResolver.getReferenceFrameFromNameBaseHashCode(message.getAngularSelectionFrameId());
         ReferenceFrame linearSelectionFrame = referenceFrameResolver.getReferenceFrameFromNameBaseHashCode(message.getLinearSelectionFrameId());
         selectionMatrix.setSelectionFrames(angularSelectionFrame, linearSelectionFrame);
      }
   }

   public RigidBody getEndEffector()
   {
      return endEffector;
   }

   public DenseMatrix64F getWeightVector()
   {
      return weightVector;
   }

   public SelectionMatrix6D getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public FramePose getDesiredPose()
   {
      return desiredPose;
   }

   public FramePose getControlFramePose()
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
