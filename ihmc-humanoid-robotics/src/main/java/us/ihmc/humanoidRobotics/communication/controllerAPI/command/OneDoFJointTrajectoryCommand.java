package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.List;
import java.util.Random;

import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.OneDoFTrajectoryPointList;

public class OneDoFJointTrajectoryCommand extends OneDoFTrajectoryPointList implements Command<OneDoFJointTrajectoryCommand, OneDoFJointTrajectoryMessage>
{
   private long sequenceId;
   private double weight;

   public OneDoFJointTrajectoryCommand()
   {
   }

   public OneDoFJointTrajectoryCommand(Random random)
   {
      for (int i = 0; i < 10; i++)
      {
         addTrajectoryPoint(i + random.nextDouble(), random.nextDouble() * 2.0 * Math.PI, random.nextDouble() * random.nextInt(20));
      }
      weight = random.nextDouble() * random.nextInt(1000);
   }

   @Override
   public void clear()
   {
      super.clear();
      sequenceId = 0;
      setWeight(Double.NaN);

   }

   @Override
   public void set(OneDoFJointTrajectoryCommand other)
   {
      super.set(other);
      sequenceId = other.sequenceId;
      setWeight(other.getWeight());
   }

   @Override
   public void setFromMessage(OneDoFJointTrajectoryMessage message)
   {
      this.clear();
      sequenceId = message.getSequenceId();

      List<TrajectoryPoint1DMessage> trajectoryPointMessages = message.getTrajectoryPoints();
      int numberOfPoints = trajectoryPointMessages.size();

      for (int i = 0; i < numberOfPoints; i++)
      {
         TrajectoryPoint1DMessage trajectoryPoint1DMessage = trajectoryPointMessages.get(i);
         this.addTrajectoryPoint(trajectoryPoint1DMessage.getTime(), trajectoryPoint1DMessage.getPosition(), trajectoryPoint1DMessage.getVelocity());
      }
      setWeight(message.getWeight());
   }

   @Override
   public Class<OneDoFJointTrajectoryMessage> getMessageClass()
   {
      return OneDoFJointTrajectoryMessage.class;
   }

   public double getWeight()
   {
      return weight;
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   @Override
   public boolean isCommandValid()
   {
      boolean numberOfTrajectoryPointsIsPositive = getNumberOfTrajectoryPoints() > 0;
      boolean weightIsValid = !Double.isInfinite(getWeight());
      if (Double.isFinite(weight))
      {
         weightIsValid &= weight >= 0;
      }
      return numberOfTrajectoryPointsIsPositive && weightIsValid;
   }

   public boolean epsilonEquals(OneDoFJointTrajectoryCommand other, double epsilon)
   {
      if (!EuclidCoreTools.epsilonEquals(weight, other.weight, epsilon))
      {
         return false;
      }
      return super.epsilonEquals(other, epsilon);
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
