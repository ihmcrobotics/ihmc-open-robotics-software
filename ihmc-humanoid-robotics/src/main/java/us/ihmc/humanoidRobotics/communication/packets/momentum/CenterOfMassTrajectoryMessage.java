package us.ihmc.humanoidRobotics.communication.packets.momentum;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class CenterOfMassTrajectoryMessage extends QueueableMessage<CenterOfMassTrajectoryMessage>
{
   /**
    * List of center of mass trajectory waypoints. Each waypoint contains the center of mass position and
    * velocity at a given time.
    */
   public final ArrayList<TrajectoryPoint3D> comTrajectory;

   public CenterOfMassTrajectoryMessage()
   {
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
      comTrajectory = new ArrayList<>();
   }

   public CenterOfMassTrajectoryMessage(Random random)
   {
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
      comTrajectory = new ArrayList<>();
      double time = random.nextDouble();
      for (int i = 0; i < random.nextInt(10) + 1; i++)
      {
         TrajectoryPoint3D point = new TrajectoryPoint3D();
         point.setPosition(EuclidCoreRandomTools.nextPoint3D(random, 1.0));
         point.setVelocity(EuclidCoreRandomTools.nextPoint3D(random, 1.0));
         point.setTime(time);
         comTrajectory.add(point);
         time += random.nextDouble();
      }
   }

   @Override
   public boolean epsilonEquals(CenterOfMassTrajectoryMessage other, double epsilon)
   {
      if (getNumberOfComTrajectoryPoints() != other.getNumberOfComTrajectoryPoints())
      {
         return false;
      }

      for (int i = 0; i < getNumberOfComTrajectoryPoints(); i++)
      {
         if (!getComTrajectoryPoint(i).epsilonEquals(other.getComTrajectoryPoint(i), epsilon))
         {
            return false;
         }
      }

      return super.epsilonEquals(other, epsilon);
   }

   public void addComTrajectoryPoint(Tuple3DReadOnly position, Tuple3DReadOnly veclocity, double time)
   {
      comTrajectory.add(new TrajectoryPoint3D(position, veclocity, time));
   }

   public int getNumberOfComTrajectoryPoints()
   {
      return comTrajectory.size();
   }

   public TrajectoryPoint3D getComTrajectoryPoint(int index)
   {
      return comTrajectory.get(index);
   }
}
