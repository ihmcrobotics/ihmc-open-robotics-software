package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import java.util.Arrays;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.tools.ArrayTools;

public class RigidBodyExplorationConfigurationMessage extends Packet<RigidBodyExplorationConfigurationMessage>
{
   public long rigidBodyNameBasedHashCode;
   public ConfigurationSpaceName[] degreesOfFreedomToExplore;
   public double[] explorationRangeLowerLimits;
   public double[] explorationRangeUpperLimits;

   /**
    * To set enable exploration for all degree of freedom, do not send this message.
    */
   public RigidBodyExplorationConfigurationMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * To set disable exploration on this rigid body.
    */
   public RigidBodyExplorationConfigurationMessage(RigidBody rigidBody)
   {
      //this(rigidBody, new ConfigurationSpaceName[] {ConfigurationSpaceName.X}, new double[] {0.0}, new double[] {0.0});
      this.rigidBodyNameBasedHashCode = rigidBody.getNameBasedHashCode();
      
      //this(rigidBody, null, null, null);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * To set enable exploration on this rigid body with following order of ConfigurationSpaceName.
    */
   public RigidBodyExplorationConfigurationMessage(RigidBody rigidBody, ConfigurationSpaceName[] degreesOfFreedomToExplore)
   {
      this(rigidBody, degreesOfFreedomToExplore,
           WholeBodyTrajectoryToolboxMessageTools.createDefaultExplorationLowerLimitArray(degreesOfFreedomToExplore),
           WholeBodyTrajectoryToolboxMessageTools.createDefaultExplorationUpperLimitArray(degreesOfFreedomToExplore));
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public RigidBodyExplorationConfigurationMessage(RigidBody rigidBody, ConfigurationSpaceName[] degreesOfFreedomToExplore,
                                                   double[] explorationRangeLowerLimits, double[] explorationRangeUpperLimits)
   {
      if (degreesOfFreedomToExplore.length != explorationRangeLowerLimits.length || degreesOfFreedomToExplore.length != explorationRangeUpperLimits.length)
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
      if(degreesOfFreedomToExplore == null)
         return 0;
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
