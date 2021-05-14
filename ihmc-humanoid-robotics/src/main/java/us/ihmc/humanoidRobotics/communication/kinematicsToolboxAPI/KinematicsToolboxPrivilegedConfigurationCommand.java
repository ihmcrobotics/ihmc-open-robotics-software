package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import controller_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage;
import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.JointHashCodeResolver;

public class KinematicsToolboxPrivilegedConfigurationCommand
      implements Command<KinematicsToolboxPrivilegedConfigurationCommand, KinematicsToolboxPrivilegedConfigurationMessage>
{
   private long sequenceId;
   private boolean hasPrivilegedRootJointPosition = false;
   private final Point3D privilegedRootJointPosition = new Point3D();
   private boolean hasPrivilegedRootJointOrientation = false;
   private final Quaternion privilegedRootJointOrientation = new Quaternion();

   private boolean hasPrivilegedJointAngles = false;
   private final List<OneDoFJointBasics> joints = new ArrayList<>();
   private final TFloatArrayList privilegedJointAngles = new TFloatArrayList();

   private double privilegedWeight = -1.0;
   private double privilegedGain = -1.0;

   @Override
   public void clear()
   {
      sequenceId = 0;
      hasPrivilegedRootJointPosition = false;
      privilegedRootJointPosition.setToNaN();
      hasPrivilegedRootJointOrientation = false;
      privilegedRootJointOrientation.setToNaN();
      joints.clear();
      privilegedJointAngles.reset();
      privilegedWeight = -1.0;
      privilegedGain = -1.0;
   }

   @Override
   public void set(KinematicsToolboxPrivilegedConfigurationCommand other)
   {
      clear();

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

      if (hasPrivilegedJointAngles)
      {
         for (int i = 0; i < other.joints.size(); i++)
         {
            joints.add(other.joints.get(i));
            privilegedJointAngles.add(other.privilegedJointAngles.get(i));
         }
      }

      privilegedWeight = other.privilegedWeight;
      privilegedGain = other.privilegedGain;
   }

   @Override
   public void setFromMessage(KinematicsToolboxPrivilegedConfigurationMessage message)
   {
      set(message, null);
   }

   public void set(KinematicsToolboxPrivilegedConfigurationMessage message, JointHashCodeResolver jointHashCodeResolver)
   {
      Objects.requireNonNull(jointHashCodeResolver);

      clear();

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

      hasPrivilegedJointAngles = messageHashCodes != null && !messageHashCodes.isEmpty() && messageJointAngles != null && !messageJointAngles.isEmpty();

      if (hasPrivilegedJointAngles)
      {
         for (int i = 0; i < messageHashCodes.size(); i++)
         {
            try
            {
               OneDoFJointBasics joint = jointHashCodeResolver.castAndGetJoint(messageHashCodes.get(i));
               joints.add(joint);
               privilegedJointAngles.add(messageJointAngles.get(i));
            }
            catch (RuntimeException e)
            {
               // unknown joint, skip.
            }
         }
      }

      privilegedWeight = message.getPrivilegedWeight();
      privilegedGain = message.getPrivilegedGain();
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

   public List<OneDoFJointBasics> getJoints()
   {
      return joints;
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

   @Override
   public Class<KinematicsToolboxPrivilegedConfigurationMessage> getMessageClass()
   {
      return KinematicsToolboxPrivilegedConfigurationMessage.class;
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
