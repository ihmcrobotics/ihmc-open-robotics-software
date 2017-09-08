package us.ihmc.robotics.linearDynamicSystems;

import org.ejml.data.DenseMatrix64F;

public interface StateSpaceSystemDiscretizer
{

   /**
    * discretizes a continuous time system. All input matrices are modified.
    * @param A state matrix
    * @param B input matrix
    * @param Q process noise covariance matrix
    * @param dt time step
    */
   public abstract void discretize(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F Q, double dt);

}