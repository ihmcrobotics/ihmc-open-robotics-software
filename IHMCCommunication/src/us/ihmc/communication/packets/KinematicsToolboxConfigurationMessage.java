package us.ihmc.communication.packets;

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
public class KinematicsToolboxConfigurationMessage extends TrackablePacket<KinematicsToolboxConfigurationMessage>
{
   /**
    * When set to {@code true}, the solver will hold the current x and y coordinates of the center
    * of mass. By 'current', it means that the solver will use the robot configuration data
    * broadcasted by the controller to obtain the center of mass position.
    */
   public boolean holdCurrentCenterOfMassXYPosition = true;
   /**
    * When set to {@code true}, the solver will hold the pose of the active support foot/feet.
    */
   public boolean holdSupportFootPositions = true;

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
    * This array is used identify to which joint each angle in {@link #privilegedJointAngles} belongs
    * to. The name-based hash code can be obtained from {@link OneDoFJoint#getNameBasedHashCode()}.
    */
   public long[] privilegedJointNameBasedHashCodes;
   /**
    * When provided, the solver will attempt to find the solution that is the closest to the
    * privileged configuration.
    */
   public float[] privilegedJointAngles;

   public KinematicsToolboxConfigurationMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Specifies whether the {@code KinematicsToolboxController} should hold the initial x and y
    * coordinates of the robot's center of mass.
    * 
    * @param holdCurrentCenterOfMassXYPosition whether x and y coordinates of the robot's center of
    *           mass should be held or not.
    */
   public void setHoldCurrentCenterOfMassXYPosition(boolean holdCurrentCenterOfMassXYPosition)
   {
      this.holdCurrentCenterOfMassXYPosition = holdCurrentCenterOfMassXYPosition;
   }

   /**
    * Specifies whether the {@code KinematicsToolboxController} should the initial pose of the
    * current support foot/feet.
    * 
    * @param holdSupportFootPositions whether the support foot/feet should be held in place.
    */
   public void setHoldSupportFootPositions(boolean holdSupportFootPositions)
   {
      this.holdSupportFootPositions = holdSupportFootPositions;
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

   /**
    * When provided, the {@code KinematicsToolboxController} will attempt to find the closest
    * solution to the privileged configuration.
    * <p>
    * Avoid calling this method directly, use instead the {@code KinematicsToolboxInputHelper}.
    * </p>
    * <p>
    * Note that by sending a privileged configuration the solver will get reinitialized to start off
    * that configuration and thus may delay the convergence to the solution. It is therefore
    * preferable to send the privileged configuration as soon as possible.
    * </p>
    * 
    * @param jointNameBasedHashCodes allows to safely identify to which joint each angle in
    *           {@link #privilegedJointAngles} belongs to. The name-based hash code can be obtained
    *           from {@link OneDoFJoint#getNameBasedHashCode()}. Not modified.
    * @param jointAngles the privileged joint angles. Not modified.
    * @throws IllegalArgumentException if the lengths of {@code jointAngles} and
    *            {@code jointNameBasedHashCodes} are different.
    */
   public void setPrivilegedJointAngles(long[] jointNameBasedHashCodes, float[] jointAngles)
   {
      if (jointNameBasedHashCodes.length != jointAngles.length)
         throw new IllegalArgumentException("The two arrays jointAngles and jointNameBasedHashCodes have to be of same length.");

      if (privilegedJointNameBasedHashCodes == null)
         privilegedJointNameBasedHashCodes = new long[jointNameBasedHashCodes.length];
      System.arraycopy(jointNameBasedHashCodes, 0, privilegedJointNameBasedHashCodes, 0, jointNameBasedHashCodes.length);

      if (privilegedJointAngles == null)
         privilegedJointAngles = new float[jointAngles.length];
      System.arraycopy(jointAngles, 0, privilegedJointAngles, 0, jointAngles.length);
   }

   /**
    * Provides a privileged configuration that the {@code KinematicsToolboxController} will use as a
    * reference and attempt to find the solution that is the closest.
    * <p>
    * Avoid calling this method directly, use instead the {@code KinematicsToolboxInputHelper}.
    * </p>
    * <p>
    * Note that by sending a privileged configuration the solver will get reinitialized to start off
    * that configuration and thus may delay the convergence to the solution. It is therefore
    * preferable to send the privileged configuration as soon as possible.
    * </p>
    * 
    * @param rootJointPosition the privileged root joint position. Not modified.
    * @param rootJointOrientation the privileged root joint orientation. Not modified.
    * @param jointNameBasedHashCodes allows to safely identify to which joint each angle in
    *           {@link #privilegedJointAngles} belongs to. The name-based hash code can be obtained
    *           from {@link OneDoFJoint#getNameBasedHashCode()}. Not modified.
    * @param jointAngles the privileged joint angles. Not modified.
    * @throws IllegalArgumentException if the lengths of {@code jointAngles} and
    *            {@code jointNameBasedHashCodes} are different.
    */
   public void setPrivilegedRobotConfiguration(Tuple3DReadOnly rootJointPosition, QuaternionReadOnly rootJointOrientation, long[] jointNameBasedHashCodes,
                                              float[] jointAngles)
   {
      setPrivilegedRootJointPosition(rootJointPosition);
      setPrivilegedRootJointOrientation(rootJointOrientation);
      setPrivilegedJointAngles(jointNameBasedHashCodes, jointAngles);
   }

   public boolean holdCurrentCenterOfMassXYPosition()
   {
      return holdCurrentCenterOfMassXYPosition;
   }

   public boolean holdSupportFootPositions()
   {
      return holdSupportFootPositions;
   }

   public Point3D32 getPrivilegedRootJointPosition()
   {
      return privilegedRootJointPosition;
   }

   public Quaternion32 getPrivilegedRootJointOrientation()
   {
      return privilegedRootJointOrientation;
   }

   public long[] getPrivilegedJointNameBasedHashCodes()
   {
      return privilegedJointNameBasedHashCodes;
   }

   public float[] getPrivilegedJointAngles()
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
      if (holdCurrentCenterOfMassXYPosition != other.holdCurrentCenterOfMassXYPosition)
         return false;
      if (holdSupportFootPositions != other.holdSupportFootPositions)
         return false;
      return true;
   }
}
