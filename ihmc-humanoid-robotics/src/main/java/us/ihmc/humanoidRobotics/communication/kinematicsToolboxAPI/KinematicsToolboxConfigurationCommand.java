package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import toolbox_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.JointHashCodeResolver;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class KinematicsToolboxConfigurationCommand implements Command<KinematicsToolboxConfigurationCommand, KinematicsToolboxConfigurationMessage>
{
   private long sequenceId;

   private double jointVelocityWeight = -1.0;
   private double jointAccelerationWeight = -1.0;

   private boolean disableCollisionAvoidance = false;
   private boolean enableCollisionAvoidance = false;

   private boolean enableJointVelocityLimits = false;
   private boolean disableJointVelocityLimits = false;

   private boolean enableInputPersistence = false;
   private boolean disableInputPersistence = false;

   private boolean enableSupportPolygonConstraint = false;
   private boolean disableSupportPolygonConstraint = false;

   private final List<OneDoFJointBasics> jointsToDeactivate = new ArrayList<>();
   private final List<OneDoFJointBasics> jointsToActivate = new ArrayList<>();

   @Override
   public void clear()
   {
      sequenceId = 0;
      jointVelocityWeight = -1.0;
      jointAccelerationWeight = -1.0;
      disableCollisionAvoidance = false;
      enableCollisionAvoidance = false;
      enableJointVelocityLimits = false;
      disableJointVelocityLimits = false;
      disableInputPersistence = false;
      enableInputPersistence = false;
      enableSupportPolygonConstraint = false;
      disableSupportPolygonConstraint = false;
      jointsToDeactivate.clear();
      jointsToActivate.clear();
   }

   @Override
   public void set(KinematicsToolboxConfigurationCommand other)
   {
      sequenceId = other.sequenceId;

      jointVelocityWeight = other.jointVelocityWeight;
      jointAccelerationWeight = other.jointAccelerationWeight;

      disableCollisionAvoidance = other.disableCollisionAvoidance;
      enableCollisionAvoidance = other.enableCollisionAvoidance;
      disableJointVelocityLimits = other.disableJointVelocityLimits;
      enableJointVelocityLimits = other.enableJointVelocityLimits;
      disableInputPersistence = other.disableInputPersistence;
      enableInputPersistence = other.enableInputPersistence;
      enableSupportPolygonConstraint = other.enableSupportPolygonConstraint;
      disableSupportPolygonConstraint = other.disableSupportPolygonConstraint;

      jointsToDeactivate.clear();
      for (int i = 0; i < other.jointsToDeactivate.size(); i++)
         jointsToDeactivate.add(other.jointsToDeactivate.get(i));
      jointsToActivate.clear();
      for (int i = 0; i < other.jointsToActivate.size(); i++)
         jointsToActivate.add(other.jointsToActivate.get(i));
   }

   @Override
   public void setFromMessage(KinematicsToolboxConfigurationMessage message)
   {
      set(message, null);
   }

   public void set(KinematicsToolboxConfigurationMessage message, JointHashCodeResolver jointHashCodeResolver)
   {
      Objects.requireNonNull(jointHashCodeResolver);

      sequenceId = message.getSequenceId();

      jointVelocityWeight = message.getJointVelocityWeight();
      jointAccelerationWeight = message.getJointAccelerationWeight();

      disableCollisionAvoidance = message.getDisableCollisionAvoidance();
      enableCollisionAvoidance = message.getEnableCollisionAvoidance();
      enableJointVelocityLimits = message.getEnableJointVelocityLimits();
      disableJointVelocityLimits = message.getDisableJointVelocityLimits();
      disableInputPersistence = message.getDisableInputPersistence();
      enableInputPersistence = message.getEnableInputPersistence();
      enableSupportPolygonConstraint = message.getEnableSupportPolygonConstraint();
      disableSupportPolygonConstraint = message.getDisableSupportPolygonConstraint();

      jointsToDeactivate.clear();
      for (int i = 0; i < message.getJointsToDeactivate().size(); i++)
         jointsToDeactivate.add((OneDoFJointBasics) jointHashCodeResolver.castAndGetJoint(message.getJointsToDeactivate().get(i)));
      jointsToActivate.clear();
      for (int i = 0; i < message.getJointsToActivate().size(); i++)
         jointsToActivate.add((OneDoFJointBasics) jointHashCodeResolver.castAndGetJoint(message.getJointsToActivate().get(i)));
   }

   public double getJointVelocityWeight()
   {
      return jointVelocityWeight;
   }

   public double getJointAccelerationWeight()
   {
      return jointAccelerationWeight;
   }

   public boolean getDisableCollisionAvoidance()
   {
      return disableCollisionAvoidance;
   }

   public boolean getEnableCollisionAvoidance()
   {
      return enableCollisionAvoidance;
   }

   public boolean getEnableJointVelocityLimits()
   {
      return enableJointVelocityLimits;
   }

   public boolean getDisableJointVelocityLimits()
   {
      return disableJointVelocityLimits;
   }

   public boolean getDisableInputPersistence()
   {
      return disableInputPersistence;
   }

   public boolean getEnableInputPersistence()
   {
      return enableInputPersistence;
   }

   public boolean getEnableSupportPolygonConstraint()
   {
      return enableSupportPolygonConstraint;
   }

   public boolean getDisableSupportPolygonConstraint()
   {
      return disableSupportPolygonConstraint;
   }

   public List<OneDoFJointBasics> getJointsToDeactivate()
   {
      return jointsToDeactivate;
   }

   public List<OneDoFJointBasics> getJointsToActivate()
   {
      return jointsToActivate;
   }

   @Override
   public Class<KinematicsToolboxConfigurationMessage> getMessageClass()
   {
      return KinematicsToolboxConfigurationMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
