package us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WaypointBasedTrajectoryMessage;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class WaypointBasedTrajectoryCommand implements Command<WaypointBasedTrajectoryCommand, WaypointBasedTrajectoryMessage>, WholeBodyTrajectoryToolboxAPI<WaypointBasedTrajectoryMessage>
{
   /** This is the unique hash code of the end-effector to be solved for. */
   private long endEffectorNameBasedHashCode;
   /** This is the end-effector to be solved for. */
   private RigidBody endEffector;

   private final TDoubleArrayList waypointTimes = new TDoubleArrayList();
   private final RecyclingArrayList<Pose3D> waypoints = new RecyclingArrayList<>(Pose3D.class);
   private final List<ConfigurationSpaceName> unconstrainedDegreesOfFreedom = new ArrayList<>();

   // TODO : control Frame TransformationMatrix;
   
   @Override
   public void clear()
   {
      endEffectorNameBasedHashCode = NameBasedHashCodeTools.NULL_HASHCODE;
      endEffector = null;
      waypointTimes.clear();
      waypoints.clear();
      unconstrainedDegreesOfFreedom.clear();
   }

   @Override
   public void set(WaypointBasedTrajectoryCommand other)
   {
      clear();

      endEffectorNameBasedHashCode = other.endEffectorNameBasedHashCode;
      endEffector = other.endEffector;

      for (int i = 0; i < other.getNumberOfWaypoints(); i++)
      {
         waypointTimes.add(other.waypointTimes.get(i));
         waypoints.add().set(other.waypoints.get(i));
      }

      for (int i = 0; i < other.unconstrainedDegreesOfFreedom.size(); i++)
      {
         unconstrainedDegreesOfFreedom.add(other.unconstrainedDegreesOfFreedom.get(i));
      }
   }

   @Override
   public void set(WaypointBasedTrajectoryMessage message)
   {
      set(message, null, null);
   }

   @Override
   public void set(WaypointBasedTrajectoryMessage message, Map<Long, RigidBody> rigidBodyNamedBasedHashMap,
                   ReferenceFrameHashCodeResolver referenceFrameResolver)
   {
      clear();

      endEffectorNameBasedHashCode = message.getEndEffectorNameBasedHashCode();
      if (rigidBodyNamedBasedHashMap == null)
         endEffector = null;
      else
         endEffector = rigidBodyNamedBasedHashMap.get(endEffectorNameBasedHashCode);

      for (int i = 0; i < message.getNumberOfWaypoints(); i++)
      {
         waypointTimes.add(message.getWaypointTime(i));
         waypoints.add().set(message.getWaypoint(i));
      }

      for (int i = 0; i < message.getNumberOfUnconstrainedDegreesOfFreedom(); i++)
      {
         unconstrainedDegreesOfFreedom.add(message.getUnconstrainedDegreeOfFreedom(i));
      }
   }

   public RigidBody getEndEffector()
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

   public ConfigurationSpaceName getUnconstrainedDegreeOfFreedom(int i)
   {
      return unconstrainedDegreesOfFreedom.get(i);
   }

   public int getNumberOfWaypoints()
   {
      return waypoints.size();
   }

   public int getNumberOfUnconstrainedDegreesOfFreedom()
   {
      return unconstrainedDegreesOfFreedom.size();
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
}
