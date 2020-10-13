package us.ihmc.commonWalkingControlModules.configurations;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * Provides an interface for a foot orientation ankle IK solver.
 * <p>
 * This interface can be used if desired foot orientation and angular velocity should be tracked
 * using the ankle joints of a robot. The implementation of this can be robot specific and specific
 * to the desired foot behavior. It provides two main methods:</br>
 * {@link AnkleIKSolver#computeAngles(QuaternionReadOnly, DMatrixRMaj)}</br>
 * {@link AnkleIKSolver#computeVelocities(Vector3DReadOnly, DMatrixRMaj, DMatrixRMaj)}
 * </p>
 * <p>
 * The implementation {@link PitchRollAnkleWithSolePlaneConstraintSolver} provides an example and
 * default functionality for robots with two ankle joints.
 * </p>
 */
public interface AnkleIKSolver
{
   /**
    * Computes joint angles that will cause the foot to achieve the desired {@code footOrientation}.
    * The provided foot orientation should be the desired orientation of the foot with respect to the
    * shin and expressed in shin frame.
    *
    * @param footOrientation the desired orientation of the foot w.r.t. the shin in shin frame.
    * @param result will be modified and contain the resulting joint angles starting from the shin.
    */
   public abstract void computeAngles(QuaternionReadOnly footOrientation, DMatrixRMaj result);

   /**
    * Computes the joint velocities that will cause the foot to achieve the desired {@link footVelocity}.
    * The provided foot velocity should be the desired foot velocity with respect to the shin and be
    * expressed in shin frame.
    *
    * @param footVelocity the desired angular velocity of the foot w.r.t the shin in shin frame.
    * @param jointAngles the joint angles of the ankle joint starting from the shin.
    * @param result will be modified and contain the resulting joint velocities starting from the shin.
    */
   public abstract void computeVelocities(Vector3DReadOnly footVelocity, DMatrixRMaj jointAngles, DMatrixRMaj result);

   /**
    * Returns the number of ankle joints. Should be equal to the number of joints between the shin and
    * the foot.
    *
    * @return number of ankle joints.
    */
   public abstract int getNumberOfJoints();

   /**
    * An implementation of {@link AnkleIKSolver}. It assumes a two joint ankle that contains one
    * pitch joint and one roll joint starting from the shin. The solver will match the x-y plane
    * of the foot with the desired x-y plane. This should result in the sole being in the intended
    * plane.
    */
   public class PitchRollAnkleWithSolePlaneConstraintSolver implements AnkleIKSolver
   {
      private final RotationMatrix rotationMatrix = new RotationMatrix();

      @Override
      public void computeAngles(QuaternionReadOnly footOrientation, DMatrixRMaj result)
      {
         rotationMatrix.set(footOrientation);
         double q0 = Math.atan2(rotationMatrix.getM02(), rotationMatrix.getM22());
         double q1 = Math.asin(-rotationMatrix.getM12());

         result.reshape(getNumberOfJoints(), 1);
         result.set(0, q0);
         result.set(1, q1);
      }

      @Override
      public void computeVelocities(Vector3DReadOnly footVelocity, DMatrixRMaj jointAngles, DMatrixRMaj result)
      {
         double qd0 = footVelocity.getY();
         double q0 = jointAngles.get(0);
         double qd1 = footVelocity.getX() * Math.cos(q0) - footVelocity.getZ() * Math.sin(q0);

         result.reshape(getNumberOfJoints(), 1);
         result.set(0, qd0);
         result.set(1, qd1);
      }

      @Override
      public int getNumberOfJoints()
      {
         return 2;
      }

   }
}
