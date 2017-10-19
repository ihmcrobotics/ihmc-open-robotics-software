package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import java.util.Map;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.KinematicsToolboxRigidBodyMessage;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.robotics.geometry.FramePose;
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

   private final FramePose desiredPose = new FramePose();
   private final FramePose controlFramePose = new FramePose();
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

   public void set(KinematicsToolboxRigidBodyMessage message, Map<Long, RigidBody> rigidBodyNamedBasedHashMap, ReferenceFrameHashCodeResolver referenceFrameResolver)
   {
      endEffectorNameBasedHashCode = message.getEndEffectorNameBasedHashCode();
      if (rigidBodyNamedBasedHashMap == null)
         endEffector = null;
      else
         endEffector = rigidBodyNamedBasedHashMap.get(endEffectorNameBasedHashCode);
      message.getDesiredPose(desiredPose);
      message.getControlFramePose(endEffector, controlFramePose);
      message.getWeightMatrix(weightMatrix);

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

   public WeightMatrix6D getWeightMatrix()
   {
      return weightMatrix;
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
