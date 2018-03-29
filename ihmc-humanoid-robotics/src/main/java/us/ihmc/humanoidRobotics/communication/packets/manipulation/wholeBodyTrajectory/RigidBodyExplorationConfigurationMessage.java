package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import gnu.trove.list.array.TByteArrayList;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;

public class RigidBodyExplorationConfigurationMessage extends Packet<RigidBodyExplorationConfigurationMessage>
{
   public static final byte CONFIGURATION_SPACE_NAME_X = 0;
   public static final byte CONFIGURATION_SPACE_NAME_Y = 1;
   public static final byte CONFIGURATION_SPACE_NAME_Z = 2;
   public static final byte CONFIGURATION_SPACE_NAME_ROLL = 3;
   public static final byte CONFIGURATION_SPACE_NAME_PITCH = 4;
   public static final byte CONFIGURATION_SPACE_NAME_YAW = 5;

   public long rigidBodyNameBasedHashCode;
   public TByteArrayList configurationSpaceNamesToExplore = new TByteArrayList();

   public TDoubleArrayList explorationRangeUpperLimits = new TDoubleArrayList();
   public TDoubleArrayList explorationRangeLowerLimits = new TDoubleArrayList();

   /**
    * To set enable exploration for all degree of freedom, do not send this message.
    */
   public RigidBodyExplorationConfigurationMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(RigidBodyExplorationConfigurationMessage other)
   {
      rigidBodyNameBasedHashCode = other.rigidBodyNameBasedHashCode;
      MessageTools.copyData(other.configurationSpaceNamesToExplore, configurationSpaceNamesToExplore);
      MessageTools.copyData(other.explorationRangeUpperLimits, explorationRangeUpperLimits);
      MessageTools.copyData(other.explorationRangeLowerLimits, explorationRangeLowerLimits);
   }

   public long getRigidBodyNameBasedHashCode()
   {
      return rigidBodyNameBasedHashCode;
   }

   @Override
   public boolean epsilonEquals(RigidBodyExplorationConfigurationMessage other, double epsilon)
   {
      if (rigidBodyNameBasedHashCode != other.rigidBodyNameBasedHashCode)
         return false;
      if (!configurationSpaceNamesToExplore.equals(other.configurationSpaceNamesToExplore))
         return false;
      if (!MessageTools.epsilonEquals(explorationRangeUpperLimits, other.explorationRangeUpperLimits, epsilon))
         return false;
      if (!MessageTools.epsilonEquals(explorationRangeLowerLimits, other.explorationRangeLowerLimits, epsilon))
         return false;
      return true;
   }
}
