package us.ihmc.humanoidRobotics.communication.packets.momentum;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * This message can be used to send a predefined angular momentum trajectory to the controller. This trajectory
 * will be used for ICP planning.
 */
public class MomentumTrajectoryMessage extends QueueableMessage<MomentumTrajectoryMessage>
{
   /**
    * List of angular momentum trajectory waypoints. Each waypoint contains the angular momentum and the
    * angular momentum rate at a given time.
    */
   public final ArrayList<TrajectoryPoint3D> angularMomentumTrajectory;

   public MomentumTrajectoryMessage()
   {
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
      angularMomentumTrajectory = new ArrayList<>();
   }

   public MomentumTrajectoryMessage(Random random)
   {
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
      angularMomentumTrajectory = new ArrayList<>();
      double time = random.nextDouble();
      for (int i = 0; i < random.nextInt(10) + 1; i++)
      {
         TrajectoryPoint3D point = new TrajectoryPoint3D();
         point.setPosition(EuclidCoreRandomTools.nextPoint3D(random, 1.0));
         point.setVelocity(EuclidCoreRandomTools.nextPoint3D(random, 1.0));
         point.setTime(time);
         angularMomentumTrajectory.add(point);
         time += random.nextDouble();
      }
   }

   @Override
   public boolean epsilonEquals(MomentumTrajectoryMessage other, double epsilon)
   {
      if (getNumberOfAngularMomentumTrajectoryPoints() != other.getNumberOfAngularMomentumTrajectoryPoints())
      {
         return false;
      }

      for (int i = 0; i < getNumberOfAngularMomentumTrajectoryPoints(); i++)
      {
         if (!getAngularMomentumTrajectoryPoint(i).epsilonEquals(other.getAngularMomentumTrajectoryPoint(i), epsilon))
         {
            return false;
         }
      }

      return super.epsilonEquals(other, epsilon);
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
