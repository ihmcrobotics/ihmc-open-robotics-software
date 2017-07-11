package us.ihmc.humanoidRobotics.communication.packets.momentum;

import java.util.ArrayList;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * This message can be used to send a predefined angular momentum trajectory to the controller. This trajectory
 * will be used for ICP planning.
 */
public class MomentumTrajectoryPackage extends QueueableMessage<MomentumTrajectoryPackage>
{
   /**
    * List of angular momentum trajectory waypoints. Each waypoint contains the angular momentum and the
    * angular momentum rate at a given time.
    */
   public final ArrayList<TrajectoryPoint3D> angularMomentumTrajectory;

   public MomentumTrajectoryPackage()
   {
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
      angularMomentumTrajectory = new ArrayList<>();
   }

   public void addAngularMomentumTrajectoryPoint(Tuple3DReadOnly position, Tuple3DReadOnly veclocity, double time)
   {
      angularMomentumTrajectory.add(new TrajectoryPoint3D(position, veclocity, time));
   }

   public int getNumberOfAngularMomentumTrajectoryPoints()
   {
      return angularMomentumTrajectory.size();
   }

   public TrajectoryPoint3D getAngularMomentumTrajectoryPoint(int index)
   {
      return angularMomentumTrajectory.get(index);
   }
}
