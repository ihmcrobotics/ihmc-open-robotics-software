package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class HumanoidKinematicsToolboxConfigurationCommand
      implements Command<HumanoidKinematicsToolboxConfigurationCommand, HumanoidKinematicsToolboxConfigurationMessage>
{
   private long sequenceId;
   private boolean holdCurrentCenterOfMassXYPosition = true;
   private boolean enableAutoSupportPolygon = true;
   private boolean enableJointLimitReduction = true;
   private final TDoubleArrayList jointLimitReductionValues = new TDoubleArrayList();
   private final TIntArrayList jointLimitReductionHashCodes = new TIntArrayList();

   // Configure stability assessment
   private boolean enableStabilityObjective = false; // if true, posture adjustment will run
   private boolean enableContactAdjustment = false; // if true, adjusts the hand contact to the region

   // Bracing region
   private final Vector3D leftBracingRegionNormal = new Vector3D();
   private final Vector3D rightBracingRegionNormal = new Vector3D();

   @Override
   public void clear()
   {
      sequenceId = 0;
      holdCurrentCenterOfMassXYPosition = true;
      enableAutoSupportPolygon = true;
      enableJointLimitReduction = true;
      jointLimitReductionValues.reset();
      jointLimitReductionHashCodes.reset();

      enableStabilityObjective = false;
      enableContactAdjustment = false;

      leftBracingRegionNormal.setToNaN();
      rightBracingRegionNormal.setToNaN();
   }

   @Override
   public void set(HumanoidKinematicsToolboxConfigurationCommand other)
   {
      clear();

      sequenceId = other.sequenceId;
      holdCurrentCenterOfMassXYPosition = other.holdCurrentCenterOfMassXYPosition;
      enableAutoSupportPolygon = other.enableAutoSupportPolygon;
      enableJointLimitReduction = other.enableJointLimitReduction;
      enableStabilityObjective = other.enableStabilityObjective;

      for (int i = 0; i < other.jointLimitReductionValues.size(); i++)
      {
         jointLimitReductionValues.add(other.jointLimitReductionValues.get(i));
      }
      for (int i = 0; i < other.jointLimitReductionHashCodes.size(); i++)
      {
         jointLimitReductionHashCodes.add(other.jointLimitReductionHashCodes.get(i));
      }

      enableContactAdjustment = other.enableContactAdjustment;

      leftBracingRegionNormal.set(other.leftBracingRegionNormal);
      rightBracingRegionNormal.set(other.rightBracingRegionNormal);
   }

   @Override
   public void setFromMessage(HumanoidKinematicsToolboxConfigurationMessage message)
   {
      clear();

      sequenceId = message.getSequenceId();
      holdCurrentCenterOfMassXYPosition = message.getHoldCurrentCenterOfMassXyPosition();
      enableAutoSupportPolygon = message.getEnableAutoSupportPolygon();
      enableJointLimitReduction = message.getEnableJointLimitReduction();
      enableStabilityObjective = message.getEnableStabilityObjective();

      for (int i = 0; i < message.getJointLimitReductionFactors().size(); i++)
      {
         jointLimitReductionValues.add(message.getJointLimitReductionFactors().get(i));
      }
      for (int i = 0; i < message.getJointLimitReductionHashCodes().size(); i++)
      {
         jointLimitReductionHashCodes.add(message.getJointLimitReductionHashCodes().get(i));
      }

      enableContactAdjustment = message.getEnableContactAdjustment();
      leftBracingRegionNormal.set(message.getLeftHandRegionNormal());
      rightBracingRegionNormal.set(message.getRightHandRegionNormal());
   }

   public boolean holdCurrentCenterOfMassXYPosition()
   {
      return holdCurrentCenterOfMassXYPosition;
   }

   public boolean enableAutoSupportPolygon()
   {
      return enableAutoSupportPolygon;
   }

   public boolean enableJointLimitReduction()
   {
      return enableJointLimitReduction;
   }

   public boolean enableStabilityObjective()
   {
      return enableStabilityObjective;
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

   public boolean enableContactAdjustment()
   {
      return enableContactAdjustment;
   }

   public Vector3D getLeftBracingRegionNormal()
   {
      return leftBracingRegionNormal;
   }

   public Vector3D getRightBracingRegionNormal()
   {
      return rightBracingRegionNormal;
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
