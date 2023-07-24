package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class HumanoidKinematicsToolboxConfigurationCommand
      implements Command<HumanoidKinematicsToolboxConfigurationCommand, HumanoidKinematicsToolboxConfigurationMessage>
{
   private long sequenceId;
   private boolean holdCurrentCenterOfMassXYPosition = true;
   private boolean enableAutoSupportPolygon = true;
   private boolean holdSupportRigidBodies = true;
   private boolean enableMultiContactSupportRegionSolver = false;
   private boolean enableJointLimitReduction = true;
   private final TDoubleArrayList jointLimitReductionValues = new TDoubleArrayList();
   private final TIntArrayList jointLimitReductionHashCodes = new TIntArrayList();

   @Override
   public void clear()
   {
      sequenceId = 0;
      holdCurrentCenterOfMassXYPosition = true;
      enableAutoSupportPolygon = true;
      holdSupportRigidBodies = true;
      enableMultiContactSupportRegionSolver = false;
      enableJointLimitReduction = true;
      jointLimitReductionValues.reset();
      jointLimitReductionHashCodes.reset();
   }

   @Override
   public void set(HumanoidKinematicsToolboxConfigurationCommand other)
   {
      clear();

      sequenceId = other.sequenceId;
      holdCurrentCenterOfMassXYPosition = other.holdCurrentCenterOfMassXYPosition;
      enableAutoSupportPolygon = other.enableAutoSupportPolygon;
      holdSupportRigidBodies = other.holdSupportRigidBodies;
      enableMultiContactSupportRegionSolver = other.enableMultiContactSupportRegionSolver;
      enableJointLimitReduction = other.enableJointLimitReduction;

      for (int i = 0; i < other.jointLimitReductionValues.size(); i++)
      {
         jointLimitReductionValues.add(other.jointLimitReductionValues.get(i));
      }
      for (int i = 0; i < other.jointLimitReductionHashCodes.size(); i++)
      {
         jointLimitReductionHashCodes.add(other.jointLimitReductionHashCodes.get(i));
      }
   }

   @Override
   public void setFromMessage(HumanoidKinematicsToolboxConfigurationMessage message)
   {
      clear();

      sequenceId = message.getSequenceId();
      holdCurrentCenterOfMassXYPosition = message.getHoldCurrentCenterOfMassXyPosition();
      enableAutoSupportPolygon = message.getEnableAutoSupportPolygon();
      holdSupportRigidBodies = message.getHoldSupportRigidBodies();
      enableMultiContactSupportRegionSolver = message.getEnableMultiContactSupportRegionSolver();
      enableJointLimitReduction = message.getEnableJointLimitReduction();

      for (int i = 0; i < message.getJointLimitReductionFactors().size(); i++)
      {
         jointLimitReductionValues.add(message.getJointLimitReductionFactors().get(i));
      }
      for (int i = 0; i < message.getJointLimitReductionHashCodes().size(); i++)
      {
         jointLimitReductionHashCodes.add(message.getJointLimitReductionHashCodes().get(i));
      }
   }

   public boolean holdCurrentCenterOfMassXYPosition()
   {
      return holdCurrentCenterOfMassXYPosition;
   }

   public boolean enableAutoSupportPolygon()
   {
      return enableAutoSupportPolygon;
   }

   public boolean holdSupportRigidBodies()
   {
      return holdSupportRigidBodies;
   }

   public boolean enableMultiContactSupportRegionSolver()
   {
      return enableMultiContactSupportRegionSolver;
   }

   public boolean enableJointLimitReduction()
   {
      return enableJointLimitReduction;
   }

   public boolean hasCustomJointRestrictionLimits()
   {
      return !jointLimitReductionValues.isEmpty();
   }

   public int getNumberOfCustomJointRestrictionLimits()
   {
      return jointLimitReductionValues.size();
   }

   public double getJointRestrictionLimitFactor(int index)
   {
      return jointLimitReductionValues.get(index);
   }

   public int getJointLimitReductionHashCode(int index)
   {
      return jointLimitReductionHashCodes.get(index);
   }

   @Override
   public Class<HumanoidKinematicsToolboxConfigurationMessage> getMessageClass()
   {
      return HumanoidKinematicsToolboxConfigurationMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      if (jointLimitReductionValues.size() != jointLimitReductionHashCodes.size())
      {
         return false;
      }

      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
