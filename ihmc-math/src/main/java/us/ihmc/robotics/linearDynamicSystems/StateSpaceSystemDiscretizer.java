package us.ihmc.robotics.linearDynamicSystems;

import org.ejml.data.DMatrixRMaj;

public interface StateSpaceSystemDiscretizer
{

   /**
    * discretizes a continuous time system. All input matrices are modified.
    * @param A state matrix
    * @param B input matrix
    * @param Q process noise covariance matrix
    * @param dt time step
    */
   public abstract void discretize(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj Q, double dt);

}