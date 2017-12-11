package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.Packet;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;

public class WholeBodyTrajectoryToolboxConfigurationMessage extends Packet<WholeBodyTrajectoryToolboxConfigurationMessage>
{
   public int numberOfInitialGuesses = -1;
   public int maximumExpansionSize = -1;
   public KinematicsToolboxOutputStatus initialConfiguration = null;
   
   // 1: way point based trajectory
   // 2: reaching manifold
   public int trajectoryType = 0;

   public WholeBodyTrajectoryToolboxConfigurationMessage()
   {
      // empty constructor for deserialization
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   public WholeBodyTrajectoryToolboxConfigurationMessage(int numberOfInitialGuesses)
   {
      this(numberOfInitialGuesses, -1, 0);
   }

   public WholeBodyTrajectoryToolboxConfigurationMessage(int numberOfInitialGuesses, int maximumExpansionSize, int trajectoryType)
   {
      this.numberOfInitialGuesses = numberOfInitialGuesses;
      this.maximumExpansionSize = maximumExpansionSize;
      this.trajectoryType = trajectoryType;
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   public void setNumberOfInitialGuesses(int numberOfInitialGuesses)
   {
      this.numberOfInitialGuesses = numberOfInitialGuesses;
   }

   public void setMaximumExpansionSize(int maximumExpansionSize)
   {
      this.maximumExpansionSize = maximumExpansionSize;
   }

   public void setInitialConfiguration(KinematicsToolboxOutputStatus initialConfiguration)
   {
      this.initialConfiguration = initialConfiguration;
   }

   public void setInitialConfigration(FullHumanoidRobotModel fullRobotModel)
   {
      initialConfiguration = new KinematicsToolboxOutputStatus(fullRobotModel.getRootJoint(), FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel),
                                                               false);
   }
   
   public void setTrajectoryType(int trajectoryType)
   {
      this.trajectoryType = trajectoryType;
   }

   public int getNumberOfInitialGuesses()
   {
      return numberOfInitialGuesses;
   }

   public int getMaximumExpansionSize()
   {
      return maximumExpansionSize;
   }

   public KinematicsToolboxOutputStatus getInitialConfiguration()
   {
      return initialConfiguration;
   }
   
   public int getTrajectoryType()
   {
      return trajectoryType;
   }

   @Override
   public boolean epsilonEquals(WholeBodyTrajectoryToolboxConfigurationMessage other, double epsilon)
   {
      if (numberOfInitialGuesses != other.numberOfInitialGuesses)
         return false;
      if (maximumExpansionSize != other.maximumExpansionSize)
         return false;
      if (trajectoryType != other.trajectoryType)
         return false;
      if (initialConfiguration == null ^ other.initialConfiguration == null)
         return false;
      if (initialConfiguration != null)
      {
         if (!initialConfiguration.epsilonEquals(other.initialConfiguration, epsilon))
            return false;
      }
      return true;
   }
}
