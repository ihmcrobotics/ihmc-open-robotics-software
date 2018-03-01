package us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.RigidBodyExplorationConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class RigidBodyExplorationConfigurationCommand implements Command<RigidBodyExplorationConfigurationCommand, RigidBodyExplorationConfigurationMessage>,
      WholeBodyTrajectoryToolboxAPI<RigidBodyExplorationConfigurationMessage>
{
   private long rigidBodyNameBasedashCode;
   private RigidBody rigidBody;
   private final List<ConfigurationSpaceName> degreesOfFreedomToExplore = new ArrayList<>();

   private final TDoubleArrayList explorationRangeUpperLimits = new TDoubleArrayList();
   private final TDoubleArrayList explorationRangeLowerLimits = new TDoubleArrayList();

   public RigidBodyExplorationConfigurationCommand()
   {
   }

   public RigidBodyExplorationConfigurationCommand(RigidBody rigidBody, ConfigurationSpaceName... configurationSpaces)
   {
      clear();
      this.rigidBody = rigidBody;
      this.rigidBodyNameBasedashCode = rigidBody.getNameBasedHashCode();
      for (int i = 0; i < configurationSpaces.length; i++)
         this.degreesOfFreedomToExplore.add(configurationSpaces[i]);
      this.explorationRangeUpperLimits.addAll(WholeBodyTrajectoryToolboxMessageTools.createDefaultExplorationUpperLimitArray(configurationSpaces));
      this.explorationRangeLowerLimits.addAll(WholeBodyTrajectoryToolboxMessageTools.createDefaultExplorationLowerLimitArray(configurationSpaces));
   }

   @Override
   public void clear()
   {
      rigidBodyNameBasedashCode = NameBasedHashCodeTools.NULL_HASHCODE;
      rigidBody = null;
      degreesOfFreedomToExplore.clear();
      explorationRangeUpperLimits.reset();
      explorationRangeLowerLimits.reset();
   }

   @Override
   public void set(RigidBodyExplorationConfigurationCommand other)
   {
      clear();

      rigidBodyNameBasedashCode = other.rigidBodyNameBasedashCode;
      rigidBody = other.rigidBody;

      for (int i = 0; i < other.getNumberOfDegreesOfFreedomToExplore(); i++)
      {
         degreesOfFreedomToExplore.add(other.degreesOfFreedomToExplore.get(i));
         explorationRangeUpperLimits.add(other.explorationRangeUpperLimits.get(i));
         explorationRangeLowerLimits.add(other.explorationRangeLowerLimits.get(i));
      }
   }

   @Override
   public void set(RigidBodyExplorationConfigurationMessage message)
   {
      set(message, null, null);
   }

   @Override
   public void set(RigidBodyExplorationConfigurationMessage message, Map<Long, RigidBody> rigidBodyNamedBasedHashMap,
                   ReferenceFrameHashCodeResolver referenceFrameResolver)
   {
      clear();

      rigidBodyNameBasedashCode = message.getRigidBodyNameBasedHashCode();
      if (rigidBodyNamedBasedHashMap == null)
         rigidBody = null;
      else
         rigidBody = rigidBodyNamedBasedHashMap.get(rigidBodyNameBasedashCode);

      for (int i = 0; i < message.getNumberOfDegreesOfFreedomToExplore(); i++)
      {
         degreesOfFreedomToExplore.add(ConfigurationSpaceName.fromByte(message.getDegreeOfFreedomToExplore(i)));
         explorationRangeUpperLimits.add(message.getExplorationRangeUpperLimits(i));
         explorationRangeLowerLimits.add(message.getExplorationRangeLowerLimits(i));
      }
   }

   public RigidBody getRigidBody()
   {
      return rigidBody;
   }

   public int getNumberOfDegreesOfFreedomToExplore()
   {
      return degreesOfFreedomToExplore.size();
   }

   public ConfigurationSpaceName getDegreeOfFreedomToExplore(int i)
   {
      return degreesOfFreedomToExplore.get(i);
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
   public Class<RigidBodyExplorationConfigurationMessage> getMessageClass()
   {
      return RigidBodyExplorationConfigurationMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return false;
   }
}
