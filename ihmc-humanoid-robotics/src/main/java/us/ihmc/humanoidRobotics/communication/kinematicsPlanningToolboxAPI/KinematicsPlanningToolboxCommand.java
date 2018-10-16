package us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI;

import java.util.Map;

import controller_msgs.msg.dds.KinematicsPlanningToolboxMessage;
import controller_msgs.msg.dds.SelectionMatrix3DMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class KinematicsPlanningToolboxCommand implements Command<KinematicsPlanningToolboxCommand, KinematicsPlanningToolboxMessage>
{
   /** This is the unique hash code of the end-effector to be solved for. */
   private long endEffectorNameBasedHashCode;
   /** This is the end-effector to be solved for. */
   private RigidBody endEffector;

   private final RecyclingArrayList<Pose3D> waypoints = new RecyclingArrayList<>(Pose3D.class);
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   private final FramePose3D controlFramePose = new FramePose3D();
   private double allowableDisplacement = 0.0;

   @Override
   public void set(KinematicsPlanningToolboxCommand other)
   {
      endEffectorNameBasedHashCode = other.endEffectorNameBasedHashCode;
      endEffector = other.endEffector;
      for (int i = 0; i < other.waypoints.size(); i++)
         waypoints.add().set(other.waypoints.get(i));
      controlFramePose.setIncludingFrame(other.controlFramePose);
      selectionMatrix.set(other.selectionMatrix);
      allowableDisplacement = other.allowableDisplacement;
   }

   public void set(KinematicsPlanningToolboxMessage message, Map<Long, RigidBody> rigidBodyNamedBasedHashMap,
                   ReferenceFrameHashCodeResolver referenceFrameResolver)
   {
      endEffectorNameBasedHashCode = message.getEndEffectorNameBasedHashCode();
      if (rigidBodyNamedBasedHashMap == null)
         endEffector = null;
      else
         endEffector = rigidBodyNamedBasedHashMap.get(endEffectorNameBasedHashCode);

      for (int i = 0; i < message.getKeyFramePoses().size(); i++)
         waypoints.add().set(message.getKeyFramePoses().get(i));

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
      }

      ReferenceFrame referenceFrame = endEffector == null ? null : endEffector.getBodyFixedFrame();
      controlFramePose.setIncludingFrame(referenceFrame, message.getControlFramePositionInEndEffector(), message.getControlFrameOrientationInEndEffector());
   }

   @Override
   public void clear()
   {
      endEffectorNameBasedHashCode = NameBasedHashCodeTools.NULL_HASHCODE;
      endEffector = null;
      waypoints.clear();
      controlFramePose.setToNaN(ReferenceFrame.getWorldFrame());
      selectionMatrix.resetSelection();
      allowableDisplacement = 0.0;
   }

   @Override
   public void setFromMessage(KinematicsPlanningToolboxMessage message)
   {
      set(message, null, null);
   }

   @Override
   public Class<KinematicsPlanningToolboxMessage> getMessageClass()
   {
      return KinematicsPlanningToolboxMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return endEffector != null && waypoints.size() > 0;
   }

}
