package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class KinematicsToolboxConfigurationCommand implements Command<KinematicsToolboxConfigurationCommand, KinematicsToolboxConfigurationMessage>
{
   private long sequenceId;
   private boolean hasPrivilegedRootJointPosition = false;
   private final Point3D privilegedRootJointPosition = new Point3D();
   private boolean hasPrivilegedRootJointOrientation = false;
   private final Quaternion privilegedRootJointOrientation = new Quaternion();

   private boolean hasPrivilegedJointAngles = false;
   private final TIntArrayList jointHashCodes = new TIntArrayList();
   private final TFloatArrayList privilegedJointAngles = new TFloatArrayList();

   private double privilegedWeight = -1.0;
   private double privilegedGain = -1.0;

   private double jointVelocityWeight = -1.0;
   private double jointAccelerationWeight = -1.0;

   private boolean disableCollisionAvoidance = false;
   private boolean enableCollisionAvoidance = false;

   @Override
   public void clear()
   {
      sequenceId = 0;
      hasPrivilegedRootJointPosition = false;
      privilegedRootJointPosition.setToNaN();
      hasPrivilegedRootJointOrientation = false;
      privilegedRootJointOrientation.setToNaN();
      privilegedWeight = -1.0;
      privilegedGain = -1.0;
      jointVelocityWeight = -1.0;
      jointAccelerationWeight = -1.0;
      disableCollisionAvoidance = false;
      enableCollisionAvoidance = false;
   }

   @Override
   public void set(KinematicsToolboxConfigurationCommand other)
   {
      sequenceId = other.sequenceId;

      hasPrivilegedRootJointPosition = other.hasPrivilegedRootJointPosition;
      if (hasPrivilegedRootJointPosition)
         privilegedRootJointPosition.set(other.getPrivilegedRootJointPosition());
      else
         privilegedRootJointPosition.setToNaN();

      hasPrivilegedRootJointOrientation = other.hasPrivilegedRootJointOrientation;
      if (hasPrivilegedRootJointOrientation)
         privilegedRootJointOrientation.set(other.getPrivilegedRootJointOrientation());
      else
         privilegedRootJointOrientation.setToNaN();

      hasPrivilegedJointAngles = other.hasPrivilegedJointAngles;
      jointHashCodes.reset();
      privilegedJointAngles.reset();

      if (hasPrivilegedJointAngles)
      {
         jointHashCodes.addAll(other.getJointHashCodes());
         privilegedJointAngles.addAll(other.getPrivilegedJointAngles());
      }

      privilegedWeight = other.privilegedWeight;
      privilegedGain = other.privilegedGain;

      jointVelocityWeight = other.jointVelocityWeight;
      jointAccelerationWeight = other.jointAccelerationWeight;

      disableCollisionAvoidance = other.disableCollisionAvoidance;
      enableCollisionAvoidance = other.enableCollisionAvoidance;
   }

   @Override
   public void setFromMessage(KinematicsToolboxConfigurationMessage message)
   {
      sequenceId = message.getSequenceId();
      hasPrivilegedRootJointPosition = message.getUsePrivilegedRootJointPosition();
      if (hasPrivilegedRootJointPosition)
         privilegedRootJointPosition.set(message.getPrivilegedRootJointPosition());
      else
         privilegedRootJointPosition.setToNaN();

      hasPrivilegedRootJointOrientation = message.getUsePrivilegedRootJointOrientation();
      if (hasPrivilegedRootJointOrientation)
         privilegedRootJointOrientation.set(message.getPrivilegedRootJointOrientation());
      else
         privilegedRootJointOrientation.setToNaN();

      TIntArrayList messageHashCodes = message.getPrivilegedJointHashCodes();
      TFloatArrayList messageJointAngles = message.getPrivilegedJointAngles();

      hasPrivilegedJointAngles = messageHashCodes != null && messageJointAngles != null;
      jointHashCodes.reset();
      privilegedJointAngles.reset();

      if (hasPrivilegedJointAngles)
      {
         jointHashCodes.addAll(messageHashCodes);
         privilegedJointAngles.addAll(messageJointAngles);
      }

      privilegedWeight = message.getPrivilegedWeight();
      privilegedGain = message.getPrivilegedGain();

      jointVelocityWeight = message.getJointVelocityWeight();
      jointAccelerationWeight = message.getJointAccelerationWeight();

      disableCollisionAvoidance = message.getDisableCollisionAvoidance();
      enableCollisionAvoidance = message.getEnableCollisionAvoidance();
   }

   public boolean hasPrivilegedRootJointPosition()
   {
      return hasPrivilegedRootJointPosition;
   }

   public boolean hasPrivilegedRootJointOrientation()
   {
      return hasPrivilegedRootJointOrientation;
   }

   public boolean hasPrivilegedJointAngles()
   {
      return hasPrivilegedJointAngles;
   }

   public Point3D getPrivilegedRootJointPosition()
   {
      return privilegedRootJointPosition;
   }

   public Quaternion getPrivilegedRootJointOrientation()
   {
      return privilegedRootJointOrientation;
   }

   public TIntArrayList getJointHashCodes()
   {
      return jointHashCodes;
   }

   public TFloatArrayList getPrivilegedJointAngles()
   {
      return privilegedJointAngles;
   }

   public double getPrivilegedWeight()
   {
      return privilegedWeight;
   }

   public double getPrivilegedGain()
   {
      return privilegedGain;
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
