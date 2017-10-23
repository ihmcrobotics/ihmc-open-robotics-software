package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TLongArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.KinematicsToolboxConfigurationMessage;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class KinematicsToolboxConfigurationCommand implements Command<KinematicsToolboxConfigurationCommand, KinematicsToolboxConfigurationMessage>
{
   private boolean hasPrivilegedRootJointPosition = false;
   private final Point3D privilegedRootJointPosition = new Point3D();
   private boolean hasPrivilegedRootJointOrientation = false;
   private final Quaternion privilegedRootJointOrientation = new Quaternion();

   private boolean hasPrivilegedJointAngles = false;
   private final TLongArrayList jointNameBasedHashCodes = new TLongArrayList();
   private final TFloatArrayList privilegedJointAngles = new TFloatArrayList();

   @Override
   public void clear()
   {
      hasPrivilegedRootJointPosition = false;
      privilegedRootJointPosition.setToNaN();
      hasPrivilegedRootJointOrientation = false;
      privilegedRootJointOrientation.setToNaN();
   }

   @Override
   public void set(KinematicsToolboxConfigurationCommand other)
   {
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
      jointNameBasedHashCodes.reset();
      privilegedJointAngles.reset();

      if (hasPrivilegedJointAngles)
      {
         jointNameBasedHashCodes.addAll(other.getJointNameBasedHashCodes());
         privilegedJointAngles.addAll(other.getPrivilegedJointAngles());
      }
   }

   @Override
   public void set(KinematicsToolboxConfigurationMessage message)
   {
      hasPrivilegedRootJointPosition = message.getPrivilegedRootJointPosition() != null;
      if (hasPrivilegedRootJointPosition)
         privilegedRootJointPosition.set(message.getPrivilegedRootJointPosition());
      else
         privilegedRootJointPosition.setToNaN();

      hasPrivilegedRootJointOrientation = message.getPrivilegedRootJointOrientation() != null;
      if (hasPrivilegedRootJointOrientation)
         privilegedRootJointOrientation.set(message.getPrivilegedRootJointOrientation());
      else
         privilegedRootJointOrientation.setToNaN();

      long[] messageHashCodes = message.getPrivilegedJointNameBasedHashCodes();
      float[] messageJointAngles = message.getPrivilegedJointAngles();

      hasPrivilegedJointAngles = messageHashCodes != null && messageJointAngles != null;
      jointNameBasedHashCodes.reset();
      privilegedJointAngles.reset();

      if (hasPrivilegedJointAngles)
      {
         jointNameBasedHashCodes.addAll(messageHashCodes);
         privilegedJointAngles.addAll(messageJointAngles);
      }
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

   public TLongArrayList getJointNameBasedHashCodes()
   {
      return jointNameBasedHashCodes;
   }

   public TFloatArrayList getPrivilegedJointAngles()
   {
      return privilegedJointAngles;
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
}
