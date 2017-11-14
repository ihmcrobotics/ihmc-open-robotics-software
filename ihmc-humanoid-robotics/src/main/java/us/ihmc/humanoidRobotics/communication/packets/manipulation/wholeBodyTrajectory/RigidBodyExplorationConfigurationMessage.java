package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import java.util.Arrays;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.tools.ArrayTools;

public class RigidBodyExplorationConfigurationMessage extends Packet<RigidBodyExplorationConfigurationMessage>
{
   public long rigidBodyNameBasedHashCode;
   public ConfigurationSpaceName[] degreesOfFreedomToExplore;
   public double[] explorationRangeLowerLimits;
   public double[] explorationRangeUpperLimits;

   public RigidBodyExplorationConfigurationMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public RigidBodyExplorationConfigurationMessage(RigidBody rigidBody)
   {
      this.rigidBodyNameBasedHashCode = rigidBody.getNameBasedHashCode();
      this.degreesOfFreedomToExplore = null;
      this.explorationRangeLowerLimits = null;
      this.explorationRangeUpperLimits = null;
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }
   
   public RigidBodyExplorationConfigurationMessage(RigidBody rigidBody, ConfigurationSpaceName[] degreesOfFreedomToExplore)
   {
      this.rigidBodyNameBasedHashCode = rigidBody.getNameBasedHashCode();
      this.degreesOfFreedomToExplore = degreesOfFreedomToExplore;
      this.explorationRangeLowerLimits = WholeBodyTrajectoryToolboxMessageTools.createDefaultExplorationLowerLimitArray(degreesOfFreedomToExplore);
      this.explorationRangeUpperLimits = WholeBodyTrajectoryToolboxMessageTools.createDefaultExplorationUpperLimitArray(degreesOfFreedomToExplore);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public RigidBodyExplorationConfigurationMessage(RigidBody rigidBody, ConfigurationSpaceName[] degreesOfFreedomToExplore,
                                                   double[] explorationRangeLowerLimits, double[] explorationRangeUpperLimits)
   {
      if (degreesOfFreedomToExplore.length != explorationRangeLowerLimits.length
            || degreesOfFreedomToExplore.length != explorationRangeUpperLimits.length)
         throw new RuntimeException("Inconsistent array lengths: unconstrainedDegreesOfFreedom.length = " + degreesOfFreedomToExplore.length
               + ", explorationRangeLowerLimits.length = " + explorationRangeLowerLimits.length + ", explorationRangeUpperLimits.length = "
               + explorationRangeUpperLimits.length);

      this.rigidBodyNameBasedHashCode = rigidBody.getNameBasedHashCode();
      this.degreesOfFreedomToExplore = degreesOfFreedomToExplore;
      this.explorationRangeLowerLimits = explorationRangeLowerLimits;
      this.explorationRangeUpperLimits = explorationRangeUpperLimits;
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public long getRigidBodyNameBasedHashCode()
   {
      return rigidBodyNameBasedHashCode;
   }

   public int getNumberOfDegreesOfFreedomToExplore()
   {
      return degreesOfFreedomToExplore.length;
   }

   public ConfigurationSpaceName getDegreeOfFreedomToExplore(int i)
   {
      return degreesOfFreedomToExplore[i];
   }

   public double getExplorationLowerLimit(int i)
   {
      return explorationRangeLowerLimits[i];
   }

   public double getExplorationUpperLimit(int i)
   {
      return explorationRangeUpperLimits[i];
   }

   @Override
   public boolean epsilonEquals(RigidBodyExplorationConfigurationMessage other, double epsilon)
   {
      if (rigidBodyNameBasedHashCode != other.rigidBodyNameBasedHashCode)
         return false;
      if (!Arrays.equals(degreesOfFreedomToExplore, other.degreesOfFreedomToExplore))
         return false;
      if (!ArrayTools.deltaEquals(explorationRangeUpperLimits, other.explorationRangeUpperLimits, epsilon))
         return false;
      if (!ArrayTools.deltaEquals(explorationRangeLowerLimits, other.explorationRangeLowerLimits, epsilon))
         return false;
      return true;
   }
}
