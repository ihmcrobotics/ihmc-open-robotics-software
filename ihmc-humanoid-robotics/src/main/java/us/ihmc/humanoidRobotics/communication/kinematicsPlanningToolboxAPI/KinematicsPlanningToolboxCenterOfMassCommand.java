package us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI;

import java.util.Map;

import controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage;
import controller_msgs.msg.dds.SelectionMatrix3DMessage;
import controller_msgs.msg.dds.WeightMatrix3DMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class KinematicsPlanningToolboxCenterOfMassCommand
      implements Command<KinematicsPlanningToolboxCenterOfMassCommand, KinematicsPlanningToolboxCenterOfMassMessage>
{
   private long sequenceId;
   private final TDoubleArrayList waypointTimes = new TDoubleArrayList();
   private final RecyclingArrayList<Point3D> waypoints = new RecyclingArrayList<>(Point3D.class);

   private final SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();
   private final WeightMatrix3D weightMatrix = new WeightMatrix3D();

   @Override
   public void set(KinematicsPlanningToolboxCenterOfMassCommand other)
   {
      clear();

      sequenceId = other.sequenceId;

      for (int i = 0; i < other.waypointTimes.size(); i++)
      {
         waypointTimes.add(other.waypointTimes.get(i));
         waypoints.add().set(other.waypoints.get(i));
      }
      selectionMatrix.set(other.selectionMatrix);
      weightMatrix.set(other.weightMatrix);
   }

   public void set(KinematicsPlanningToolboxCenterOfMassMessage message, Map<Long, RigidBodyBasics> rigidBodyNamedBasedHashMap,
                   ReferenceFrameHashCodeResolver referenceFrameResolver)
   {
      clear();
      sequenceId = message.getSequenceId();
      for (int i = 0; i < message.getWayPointTimes().size(); i++)
      {
         waypointTimes.add(message.getWayPointTimes().get(i));
         waypoints.add().set(message.getDesiredWayPointPositionsInWorld().get(i));
      }

      selectionMatrix.clearSelectionFrame();
      SelectionMatrix3DMessage linearSelection = message.getSelectionMatrix();
      selectionMatrix.setAxisSelection(linearSelection.getXSelected(), linearSelection.getYSelected(), linearSelection.getZSelected());

      weightMatrix.clear();
      WeightMatrix3DMessage linearWeight = message.getWeights();
      weightMatrix.setWeights(linearWeight.getXWeight(), linearWeight.getYWeight(), linearWeight.getZWeight());

      if (referenceFrameResolver != null)
      {
         ReferenceFrame linearSelectionFrame = referenceFrameResolver.getReferenceFrame(linearSelection.getSelectionFrameId());
         selectionMatrix.setSelectionFrame(linearSelectionFrame);
         ReferenceFrame linearWeightFrame = referenceFrameResolver.getReferenceFrame(linearWeight.getWeightFrameId());
         weightMatrix.setWeightFrame(linearWeightFrame);
      }
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      waypoints.clear();
      waypointTimes.clear();
      selectionMatrix.resetSelection();
      weightMatrix.clear();
   }

   @Override
   public void setFromMessage(KinematicsPlanningToolboxCenterOfMassMessage message)
   {
      set(message, null, null);
   }

   @Override
   public Class<KinematicsPlanningToolboxCenterOfMassMessage> getMessageClass()
   {
      return KinematicsPlanningToolboxCenterOfMassMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return waypointTimes.size() == waypoints.size();
   }

   public int getNumberOfWayPoints()
   {
      return waypoints.size();
   }

   public Point3D getWayPoint(int i)
   {
      return waypoints.get(i);
   }

   public TDoubleArrayList getWayPointTimes()
   {
      return waypointTimes;
   }

   public double getWayPointTime(int i)
   {
      return waypointTimes.get(i);
   }

   public SelectionMatrix3D getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public WeightMatrix3D getWeightMatrix()
   {
      return weightMatrix;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
