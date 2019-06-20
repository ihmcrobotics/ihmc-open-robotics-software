package us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class RigidBodyExplorationConfigurationCommand implements Command<RigidBodyExplorationConfigurationCommand, RigidBodyExplorationConfigurationMessage>,
      WholeBodyTrajectoryToolboxAPI<RigidBodyExplorationConfigurationMessage>
{
   private long sequenceId;
   private int rigidBodyHashCode;
   private RigidBodyBasics rigidBody;
   private final List<ConfigurationSpaceName> degreesOfFreedomToExplore = new ArrayList<>();

   private final TDoubleArrayList explorationRangeUpperLimits = new TDoubleArrayList();
   private final TDoubleArrayList explorationRangeLowerLimits = new TDoubleArrayList();

   public RigidBodyExplorationConfigurationCommand()
   {
   }

   public RigidBodyExplorationConfigurationCommand(RigidBodyBasics rigidBody, ConfigurationSpaceName... configurationSpaces)
   {
      clear();
      this.rigidBody = rigidBody;
      this.rigidBodyHashCode = rigidBody.hashCode();
      for (int i = 0; i < configurationSpaces.length; i++)
         this.degreesOfFreedomToExplore.add(configurationSpaces[i]);
      this.explorationRangeUpperLimits.addAll(WholeBodyTrajectoryToolboxMessageTools.createDefaultExplorationUpperLimitArray(configurationSpaces));
      this.explorationRangeLowerLimits.addAll(WholeBodyTrajectoryToolboxMessageTools.createDefaultExplorationLowerLimitArray(configurationSpaces));
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      rigidBodyHashCode = 0;
      rigidBody = null;
      degreesOfFreedomToExplore.clear();
      explorationRangeUpperLimits.reset();
      explorationRangeLowerLimits.reset();
   }

   @Override
   public void set(RigidBodyExplorationConfigurationCommand other)
   {
      clear();

      sequenceId = other.sequenceId;

      rigidBodyHashCode = other.rigidBodyHashCode;
      rigidBody = other.rigidBody;

      for (int i = 0; i < other.getNumberOfDegreesOfFreedomToExplore(); i++)
      {
         degreesOfFreedomToExplore.add(other.degreesOfFreedomToExplore.get(i));
         explorationRangeUpperLimits.add(other.explorationRangeUpperLimits.get(i));
         explorationRangeLowerLimits.add(other.explorationRangeLowerLimits.get(i));
      }
   }

   @Override
   public void setFromMessage(RigidBodyExplorationConfigurationMessage message)
   {
      set(message, null, null);
   }

   @Override
   public void set(RigidBodyExplorationConfigurationMessage message, Map<Integer, RigidBodyBasics> rigidBodyHashMap,
                   ReferenceFrameHashCodeResolver referenceFrameResolver)
   {
      clear();

      sequenceId = message.getSequenceId();
      rigidBodyHashCode = message.getRigidBodyHashCode();
      if (rigidBodyHashMap == null)
         rigidBody = null;
      else
         rigidBody = rigidBodyHashMap.get(rigidBodyHashCode);

      for (int i = 0; i < message.getConfigurationSpaceNamesToExplore().size(); i++)
      {
         degreesOfFreedomToExplore.add(ConfigurationSpaceName.fromByte(message.getConfigurationSpaceNamesToExplore().get(i)));
         explorationRangeUpperLimits.add(message.getExplorationRangeUpperLimits().get(i));
         explorationRangeLowerLimits.add(message.getExplorationRangeLowerLimits().get(i));
      }
   }

   public RigidBodyBasics getRigidBody()
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
      return rigidBody != null;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
