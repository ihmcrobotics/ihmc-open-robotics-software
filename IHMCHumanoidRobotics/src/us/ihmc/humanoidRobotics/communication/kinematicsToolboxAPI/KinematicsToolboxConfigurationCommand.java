package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TLongArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.KinematicsToolboxConfigurationMessage;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class KinematicsToolboxConfigurationCommand implements Command<KinematicsToolboxConfigurationCommand, KinematicsToolboxConfigurationMessage>
{
   private boolean holdCurrentCenterOfMassXYPosition = true;
   private boolean holdSupporFootPositions = true;

   private boolean hasPrivilegedRooJointPosition = false;
   private final Point3D privilegedRootJointPosition = new Point3D();
   private boolean hasPrivilegedRooJointOrientation = false;
   private final Quaternion privilegedRootJointOrientation = new Quaternion();

   private boolean hasPrivilegedJointAngles = false;
   private final TLongArrayList jointNameBasedHashCodes = new TLongArrayList();
   private final TFloatArrayList privilegedJointAngles = new TFloatArrayList();

   @Override
   public void clear()
   {
      holdCurrentCenterOfMassXYPosition = true;
      holdSupporFootPositions = true;

      hasPrivilegedRooJointPosition = false;
      privilegedRootJointPosition.setToNaN();
      hasPrivilegedRooJointOrientation = false;
      privilegedRootJointOrientation.setToNaN();
   }

   @Override
   public void set(KinematicsToolboxConfigurationCommand other)
   {
      holdCurrentCenterOfMassXYPosition = other.holdCurrentCenterOfMassXYPosition;
      holdSupporFootPositions = other.holdSupporFootPositions;
   }

   @Override
   public void set(KinematicsToolboxConfigurationMessage message)
   {
      holdCurrentCenterOfMassXYPosition = message.holdCurrentCenterOfMassXYPosition();
      holdSupporFootPositions = message.holdSupporFootPositions();

      hasPrivilegedRooJointPosition = message.getPrivilegedRootJointPosition() != null;
      if (hasPrivilegedRooJointPosition)
         privilegedRootJointPosition.set(message.getPrivilegedRootJointPosition());
      else
         privilegedRootJointPosition.setToNaN();

      hasPrivilegedRooJointOrientation = message.getPrivilegedRootJointOrientation() != null;
      if (hasPrivilegedRooJointOrientation)
         privilegedRootJointOrientation.set(message.getPrivilegedRootJointOrientation());
      else
         privilegedRootJointOrientation.setToNaN();

      long[] messageHashCodes = message.getPrivilegedJointNameBasedHashCodes();
      float[] messageJointAngles = message.getPrivilegedJointAngles();

      hasPrivilegedJointAngles = messageHashCodes != null && messageJointAngles != null;
      jointNameBasedHashCodes.reset();
      jointNameBasedHashCodes.addAll(messageHashCodes);
      privilegedJointAngles.reset();
      privilegedJointAngles.addAll(messageJointAngles);
   }

   public boolean holdCurrentCenterOfMassXYPosition()
   {
      return holdCurrentCenterOfMassXYPosition;
   }

   public boolean holdSupporFootPositions()
   {
      return holdSupporFootPositions;
   }

   public boolean hasPrivilegedRooJointPosition()
   {
      return hasPrivilegedRooJointPosition;
   }

   public boolean hasPrivilegedRooJointOrientation()
   {
      return hasPrivilegedRooJointOrientation;
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
