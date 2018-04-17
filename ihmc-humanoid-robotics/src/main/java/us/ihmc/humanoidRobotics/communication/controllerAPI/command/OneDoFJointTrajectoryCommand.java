package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.List;
import java.util.Random;

import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;

public class OneDoFJointTrajectoryCommand extends SimpleTrajectoryPoint1DList implements Command<OneDoFJointTrajectoryCommand, OneDoFJointTrajectoryMessage>
{
   private double weight;

   public OneDoFJointTrajectoryCommand()
   {
   }

   public OneDoFJointTrajectoryCommand(Random random)
   {
      super(random);
      weight = random.nextDouble() * random.nextInt(1000);
   }

   @Override
   public void clear()
   {
      super.clear();
      setWeight(Double.NaN);
      
   }

   @Override
   public void set(OneDoFJointTrajectoryCommand other)
   {
      super.set(other);
      setWeight(other.getWeight());
   }

   @Override
   public void set(OneDoFJointTrajectoryMessage message)
   {
      this.clear();
      
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
}
