package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;

public class WholeBodyTrajectoryToolboxConfigurationMessage extends Packet<WholeBodyTrajectoryToolboxConfigurationMessage>
{
   public int numberOfInitialGuesses = -1;
   public int maximumExpansionSize = -1;
   public KinematicsToolboxOutputStatus initialConfiguration = null;

   public WholeBodyTrajectoryToolboxConfigurationMessage()
   {
      // empty constructor for deserialization
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(WholeBodyTrajectoryToolboxConfigurationMessage other)
   {
      numberOfInitialGuesses = other.numberOfInitialGuesses;
      maximumExpansionSize = other.maximumExpansionSize;
      initialConfiguration = new KinematicsToolboxOutputStatus();
      initialConfiguration.set(other.initialConfiguration);
      setPacketInformation(other);
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
      initialConfiguration = MessageTools.createKinematicsToolboxOutputStatus(fullRobotModel.getRootJoint(), FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel), false);
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

   @Override
   public boolean epsilonEquals(WholeBodyTrajectoryToolboxConfigurationMessage other, double epsilon)
   {
      if (numberOfInitialGuesses != other.numberOfInitialGuesses)
         return false;
      if (maximumExpansionSize != other.maximumExpansionSize)
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
