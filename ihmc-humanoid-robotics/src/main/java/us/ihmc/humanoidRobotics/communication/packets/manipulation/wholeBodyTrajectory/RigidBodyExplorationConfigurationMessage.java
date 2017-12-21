package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import java.util.Arrays;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.tools.ArrayTools;

public class RigidBodyExplorationConfigurationMessage extends Packet<RigidBodyExplorationConfigurationMessage>
{
   public long rigidBodyNameBasedHashCode;
   public ConfigurationSpaceName[] degreesOfFreedomToExplore;

   public double[] explorationRangeUpperLimits;
   public double[] explorationRangeLowerLimits;

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
      this.rigidBodyNameBasedHashCode = rigidBody.getNameBasedHashCode();

      ConfigurationSpaceName[] configurations = {ConfigurationSpaceName.X, ConfigurationSpaceName.Y, ConfigurationSpaceName.Z, ConfigurationSpaceName.YAW,
            ConfigurationSpaceName.PITCH, ConfigurationSpaceName.ROLL};
      double[] regionAmplitude = new double[] {0, 0, 0, 0, 0, 0};
      setExplorationConfigurationSpaces(configurations, regionAmplitude);

      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * To set enable exploration on this rigid body with following order of ConfigurationSpaceName.
    */
   public RigidBodyExplorationConfigurationMessage(RigidBody rigidBody, ConfigurationSpaceName[] degreesOfFreedomToExplore)
   {
      this(rigidBody, degreesOfFreedomToExplore, WholeBodyTrajectoryToolboxMessageTools.createDefaultExplorationAmplitudeArray(degreesOfFreedomToExplore));
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public RigidBodyExplorationConfigurationMessage(RigidBody rigidBody, ConfigurationSpaceName[] degreesOfFreedomToExplore, double[] explorationRangeAmplitudes)
   {
      if (degreesOfFreedomToExplore.length != explorationRangeAmplitudes.length)
         throw new RuntimeException("Inconsistent array lengths: unconstrainedDegreesOfFreedom.length = " + degreesOfFreedomToExplore.length);

      this.rigidBodyNameBasedHashCode = rigidBody.getNameBasedHashCode();
      setExplorationConfigurationSpaces(degreesOfFreedomToExplore, explorationRangeAmplitudes);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public RigidBodyExplorationConfigurationMessage(RigidBody rigidBody, ConfigurationSpaceName[] degreesOfFreedomToExplore,
                                                   double[] explorationRangeUpperLimits, double[] explorationRangeLowerLimits)
   {
      if (degreesOfFreedomToExplore.length != explorationRangeUpperLimits.length || degreesOfFreedomToExplore.length != explorationRangeLowerLimits.length)
         throw new RuntimeException("Inconsistent array lengths: unconstrainedDegreesOfFreedom.length = " + degreesOfFreedomToExplore.length);

      this.rigidBodyNameBasedHashCode = rigidBody.getNameBasedHashCode();
      setExplorationConfigurationSpaces(degreesOfFreedomToExplore, explorationRangeUpperLimits, explorationRangeLowerLimits);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public void setExplorationConfigurationSpaces(ConfigurationSpaceName[] degreesOfFreedomToExplore, double[] explorationRangeAmplitudes)
   {
      if (degreesOfFreedomToExplore.length != explorationRangeAmplitudes.length)
         throw new RuntimeException("Inconsistent array lengths: unconstrainedDegreesOfFreedom.length = " + degreesOfFreedomToExplore.length
               + ", explorationRangeLowerLimits.length = ");

      this.degreesOfFreedomToExplore = degreesOfFreedomToExplore;
      this.explorationRangeUpperLimits = new double[degreesOfFreedomToExplore.length];
      this.explorationRangeLowerLimits = new double[degreesOfFreedomToExplore.length];
      for (int i = 0; i < degreesOfFreedomToExplore.length; i++)
      {
         explorationRangeUpperLimits[i] = explorationRangeAmplitudes[i];
         explorationRangeLowerLimits[i] = -explorationRangeAmplitudes[i];
      }
   }

   public void setExplorationConfigurationSpaces(ConfigurationSpaceName[] degreesOfFreedomToExplore, double[] explorationRangeUpperLimits,
                                                 double[] explorationRangeLowerLimits)
   {
      if (degreesOfFreedomToExplore.length != explorationRangeUpperLimits.length || degreesOfFreedomToExplore.length != explorationRangeLowerLimits.length)
         throw new RuntimeException("Inconsistent array lengths: unconstrainedDegreesOfFreedom.length = " + degreesOfFreedomToExplore.length
               + ", explorationRangeLowerLimits.length = ");

      this.degreesOfFreedomToExplore = degreesOfFreedomToExplore;
      this.explorationRangeUpperLimits = explorationRangeUpperLimits;
      this.explorationRangeLowerLimits = explorationRangeLowerLimits;
   }

   public long getRigidBodyNameBasedHashCode()
   {
      return rigidBodyNameBasedHashCode;
   }

   public int getNumberOfDegreesOfFreedomToExplore()
   {
      if (degreesOfFreedomToExplore == null)
         return 0;
      return degreesOfFreedomToExplore.length;
   }

   public ConfigurationSpaceName getDegreeOfFreedomToExplore(int i)
   {
      return degreesOfFreedomToExplore[i];
   }

   public double getExplorationRangeUpperLimits(int i)
   {
      return explorationRangeUpperLimits[i];
   }

   public double getExplorationRangeLowerLimits(int i)
   {
      return explorationRangeLowerLimits[i];
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
