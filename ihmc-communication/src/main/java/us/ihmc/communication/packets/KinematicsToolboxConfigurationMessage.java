package us.ihmc.communication.packets;

import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TLongArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

/**
 * {@link KinematicsToolboxConfigurationMessage} is part of the API of the
 * {@code KinematicsToolboxController}.
 * <p>
 * It contains auxiliary information that allows to further customized the behavior of the solver.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public class KinematicsToolboxConfigurationMessage extends Packet<KinematicsToolboxConfigurationMessage>
{
   /**
    * When provided, the solver will attempt to find the solution that is the closest to the
    * privileged configuration.
    */
   public Point3D32 privilegedRootJointPosition;
   /**
    * When provided, the solver will attempt to find the solution that is the closest to the
    * privileged configuration.
    */
   public Quaternion32 privilegedRootJointOrientation;
   /**
    * This array is used identify to which joint each angle in {@link #privilegedJointAngles}
    * belongs to. The name-based hash code can be obtained from
    * {@link OneDoFJoint#getNameBasedHashCode()}.
    */
   public TLongArrayList privilegedJointNameBasedHashCodes = new TLongArrayList();
   /**
    * When provided, the solver will attempt to find the solution that is the closest to the
    * privileged configuration.
    */
   public TFloatArrayList privilegedJointAngles = new TFloatArrayList();

   public KinematicsToolboxConfigurationMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(KinematicsToolboxConfigurationMessage other)
   {
      if (other.privilegedRootJointPosition != null)
         privilegedRootJointPosition = new Point3D32(other.privilegedRootJointPosition);
      if (other.privilegedRootJointOrientation != null)
         privilegedRootJointOrientation = new Quaternion32(other.privilegedRootJointOrientation);
      MessageTools.copyData(other.privilegedJointNameBasedHashCodes, privilegedJointNameBasedHashCodes);
      MessageTools.copyData(other.privilegedJointAngles, privilegedJointAngles);
      setPacketInformation(other);
   }

   /**
    * When provided, the {@code KinematicsToolboxController} will attempt to find the closest
    * solution to the privileged configuration.
    * <p>
    * Note that by sending a privileged configuration the solver will get reinitialized to start off
    * that configuration and thus may delay the convergence to the solution. It is therefore
    * preferable to send the privileged configuration as soon as possible.
    * </p>
    * 
    * @param rootJointPosition the privileged root joint position. Not modified.
    */
   public void setPrivilegedRootJointPosition(Tuple3DReadOnly rootJointPosition)
   {
      if (privilegedRootJointPosition == null)
         privilegedRootJointPosition = new Point3D32(rootJointPosition);
      else
         privilegedRootJointPosition.set(rootJointPosition);
   }

   /**
    * When provided, the {@code KinematicsToolboxController} will attempt to find the closest
    * solution to the privileged configuration.
    * <p>
    * Note that by sending a privileged configuration the solver will get reinitialized to start off
    * that configuration and thus may delay the convergence to the solution. It is therefore
    * preferable to send the privileged configuration as soon as possible.
    * </p>
    * 
    * @param rootJointOrientation the privileged root joint orientation. Not modified.
    */
   public void setPrivilegedRootJointOrientation(QuaternionReadOnly rootJointOrientation)
   {
      if (privilegedRootJointOrientation == null)
         privilegedRootJointOrientation = new Quaternion32(rootJointOrientation);
      else
         privilegedRootJointOrientation.set(rootJointOrientation);
   }

   public Point3D32 getPrivilegedRootJointPosition()
   {
      return privilegedRootJointPosition;
   }

   public Quaternion32 getPrivilegedRootJointOrientation()
   {
      return privilegedRootJointOrientation;
   }

   public TLongArrayList getPrivilegedJointNameBasedHashCodes()
   {
      return privilegedJointNameBasedHashCodes;
   }

   public TFloatArrayList getPrivilegedJointAngles()
   {
      return privilegedJointAngles;
   }

   /**
    * Compares each field of this message against the other message and returns {@code true} if they
    * are equal to an {@code epsilon}.
    * <p>
    * Note that this method considers two fields to be equal if they are both {@code null}, and
    * considers two fields to be different if only one is equal to {@code null}.
    * </p>
    * 
    * @return {@code true} if the two messages are equal to an {@code epsilon}, {@code false}
    *         otherwise.
    */
   @Override
   public boolean epsilonEquals(KinematicsToolboxConfigurationMessage other, double epsilon)
   {
      if (!privilegedJointNameBasedHashCodes.equals(other.privilegedJointNameBasedHashCodes))
         return false;

      if (privilegedRootJointPosition == null ^ other.privilegedRootJointPosition == null)
         return false;
      else if (privilegedRootJointPosition != null && !privilegedRootJointPosition.epsilonEquals(other.privilegedRootJointPosition, epsilon))
         return false;

      if (privilegedRootJointOrientation == null ^ other.privilegedRootJointOrientation == null)
         return false;
      else if (privilegedRootJointOrientation != null && !privilegedRootJointOrientation.epsilonEquals(other.privilegedRootJointOrientation, epsilon))
         return false;

      if (privilegedJointAngles == other.privilegedJointAngles)
         return true;
      else if (!MessageTools.epsilonEquals(privilegedJointAngles, other.privilegedJointAngles, (float) epsilon))
         return false;

      return true;
   }
}
