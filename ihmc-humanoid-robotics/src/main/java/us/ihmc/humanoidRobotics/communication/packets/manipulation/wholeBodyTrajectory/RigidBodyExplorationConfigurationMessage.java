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

   public void setExplorationConfigurationSpaces(byte[] degreesOfFreedomToExplore, double[] explorationRangeAmplitudes)
   {
      if (degreesOfFreedomToExplore.length != explorationRangeAmplitudes.length)
         throw new RuntimeException("Inconsistent array lengths: unconstrainedDegreesOfFreedom.length = " + degreesOfFreedomToExplore.length
               + ", explorationRangeLowerLimits.length = ");

      this.configurationSpaceNamesToExplore.reset();
      this.explorationRangeUpperLimits.reset();
      this.explorationRangeLowerLimits.reset();

      this.configurationSpaceNamesToExplore.add(degreesOfFreedomToExplore);

      for (int i = 0; i < degreesOfFreedomToExplore.length; i++)
      {
         explorationRangeUpperLimits.add(explorationRangeAmplitudes[i]);
         explorationRangeLowerLimits.add(-explorationRangeAmplitudes[i]);
      }
   }

   public void setExplorationConfigurationSpaces(byte[] degreesOfFreedomToExplore, double[] explorationRangeUpperLimits, double[] explorationRangeLowerLimits)
   {
      if (degreesOfFreedomToExplore.length != explorationRangeUpperLimits.length || degreesOfFreedomToExplore.length != explorationRangeLowerLimits.length)
         throw new RuntimeException("Inconsistent array lengths: unconstrainedDegreesOfFreedom.length = " + degreesOfFreedomToExplore.length
               + ", explorationRangeLowerLimits.length = ");

      this.configurationSpaceNamesToExplore.reset();
      this.explorationRangeUpperLimits.reset();
      this.explorationRangeLowerLimits.reset();

      this.configurationSpaceNamesToExplore.add(degreesOfFreedomToExplore);
      this.explorationRangeUpperLimits.add(explorationRangeUpperLimits);
      this.explorationRangeLowerLimits.add(explorationRangeLowerLimits);
   }

   public long getRigidBodyNameBasedHashCode()
   {
      return rigidBodyNameBasedHashCode;
   }

   public int getNumberOfDegreesOfFreedomToExplore()
   {
      if (configurationSpaceNamesToExplore == null)
         return 0;
      return configurationSpaceNamesToExplore.size();
   }

   public byte getDegreeOfFreedomToExplore(int i)
   {
      return configurationSpaceNamesToExplore.get(i);
   }

   public double getExplorationRangeUpperLimits(int i)
   {
      return explorationRangeUpperLimits.get(i);
   }

   public double getExplorationRangeLowerLimits(int i)
   {
      return explorationRangeLowerLimits.get(i);
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
