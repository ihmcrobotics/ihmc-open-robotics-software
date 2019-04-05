package us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI;

import java.util.Map;

import controller_msgs.msg.dds.SelectionMatrix3DMessage;
import controller_msgs.msg.dds.WaypointBasedTrajectoryMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class WaypointBasedTrajectoryCommand
      implements Command<WaypointBasedTrajectoryCommand, WaypointBasedTrajectoryMessage>, WholeBodyTrajectoryToolboxAPI<WaypointBasedTrajectoryMessage>
{
   private long sequenceId;
   /** This is the unique hash code of the end-effector to be solved for. */
   private int endEffectorHashCode;
   /** This is the end-effector to be solved for. */
   private RigidBodyBasics endEffector;

   private final TDoubleArrayList waypointTimes = new TDoubleArrayList();
   private final RecyclingArrayList<Pose3D> waypoints = new RecyclingArrayList<>(Pose3D.class);
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   private final FramePose3D controlFramePose = new FramePose3D();
   
   private double weight;

   @Override
   public void clear()
   {
      sequenceId = 0;
      endEffectorHashCode = 0;
      endEffector = null;
      waypointTimes.clear();
      waypoints.clear();
      controlFramePose.setToNaN(ReferenceFrame.getWorldFrame());
      selectionMatrix.resetSelection();
      weight = Double.NaN;
   }

   @Override
   public void set(WaypointBasedTrajectoryCommand other)
   {
      clear();

      sequenceId = other.sequenceId;

      endEffectorHashCode = other.endEffectorHashCode;
      endEffector = other.endEffector;

      for (int i = 0; i < other.getNumberOfWaypoints(); i++)
      {
         waypointTimes.add(other.waypointTimes.get(i));
         waypoints.add().set(other.waypoints.get(i));
      }

      selectionMatrix.set(other.selectionMatrix);

      controlFramePose.setIncludingFrame(other.controlFramePose);
      
      weight = other.weight;
   }

   @Override
   public void setFromMessage(WaypointBasedTrajectoryMessage message)
   {
      set(message, null, null);
   }

   @Override
   public void set(WaypointBasedTrajectoryMessage message, Map<Integer, RigidBodyBasics> rigidBodyNamedBasedHashMap,
                   ReferenceFrameHashCodeResolver referenceFrameResolver)
   {
      clear();

      sequenceId = message.getSequenceId();
      endEffectorHashCode = message.getEndEffectorHashCode();
      if (rigidBodyNamedBasedHashMap == null)
         endEffector = null;
      else
         endEffector = rigidBodyNamedBasedHashMap.get(endEffectorHashCode);

      for (int i = 0; i < message.getWaypoints().size(); i++)
      {
         waypointTimes.add(message.getWaypointTimes().get(i));
         waypoints.add().set(message.getWaypoints().get(i));
      }

      ReferenceFrame referenceFrame = endEffector == null ? null : endEffector.getBodyFixedFrame();
      controlFramePose.setIncludingFrame(referenceFrame, message.getControlFramePositionInEndEffector(), message.getControlFrameOrientationInEndEffector());
      selectionMatrix.resetSelection();
      SelectionMatrix3DMessage angularSelection = message.getAngularSelectionMatrix();
      SelectionMatrix3DMessage linearSelection = message.getLinearSelectionMatrix();
      selectionMatrix.setAngularAxisSelection(angularSelection.getXSelected(), angularSelection.getYSelected(), angularSelection.getZSelected());
      selectionMatrix.setLinearAxisSelection(linearSelection.getXSelected(), linearSelection.getYSelected(), linearSelection.getZSelected());
      
      weight = message.getWeight();
   }

   public RigidBodyBasics getEndEffector()
   {
      return endEffector;
   }

   public double getWaypointTime(int i)
   {
      return waypointTimes.get(i);
   }

   public double getLastWaypointTime()
   {
      return waypointTimes.get(getNumberOfWaypoints() - 1);
   }

   public Pose3D getWaypoint(int i)
   {
      return waypoints.get(i);
   }

   public SelectionMatrix6D getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public int getNumberOfWaypoints()
   {
      return waypoints.size();
   }

   public FramePose3D getControlFramePose()
   {
      return controlFramePose;
   }
   
   public double getWeight()
   {
      return weight;
   }

   @Override
   public Class<WaypointBasedTrajectoryMessage> getMessageClass()
   {
      return WaypointBasedTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return getNumberOfWaypoints() > 0;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
