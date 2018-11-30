package us.ihmc.quadrupedRobotics.model;

import us.ihmc.euclid.tuple3D.Point3D;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.partNames.QuadrupedJointName;

public interface QuadrupedInitialPositionParameters
{
   /**
    * Maps joint names to initial positions.
    *
    * @param joint
    *           the joint name to lookup.
    * @return the initial angle in radians.
    */
   double getInitialJointPosition(QuadrupedJointName joint);

   Point3D getInitialBodyPosition();

   Quaternion getInitialBodyOrientation();

   default void offsetInitialConfiguration(QuadrupedInitialOffsetAndYaw offset)
   {
      getInitialBodyPosition().add(offset.getAdditionalOffset());
      getInitialBodyOrientation().appendYawRotation(offset.getYaw());
   }
}
