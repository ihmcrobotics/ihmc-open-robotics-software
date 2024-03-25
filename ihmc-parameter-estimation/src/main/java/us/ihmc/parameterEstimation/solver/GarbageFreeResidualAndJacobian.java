package us.ihmc.parameterEstimation.solver;

import org.ejml.data.DMatrixRMaj;

/**
 * Abstract class for specifying the residual error function and Jacobian function of a nonlinear least squares problem to be solved by
 * {@link GarbageFreeDoglegSolver}.
 * <p>
 * We consider nonlinear least squares problems of the form: 0.5 * || r(x) ||<sup>2</sup>
 * where r = r(x) is the residual of the problem. The Jacobian of the residual evaluated at x is denoted J(x).
 * </p>
 * <p>
 * NOTE: Despite the name, this class DOES NOT enforce any garbage-free behaviour -- it is a statement of intent of usage. It is your responsibility to ensure
 * that {@link #calculateResidual} and {@link #calculateJacobian} are garbage-free!
 * </p>
 */
public interface GarbageFreeResidualAndJacobian
{
   /**
    * Calculate the residual of the nonlinear least squares problem when the problem variable takes value {@code x}.
    *
    * @param x the problem variable to calculate the residual of.
    * @param residualToPack the vector in which to pack the residual.
    */
   void calculateResidual(DMatrixRMaj x, DMatrixRMaj residualToPack);

   /**
    * Calculate the Jacobian of the residual of the nonlinear least squares problem when the problem variable takes value {@code x}.
    *
    * @param x the problem variable to calculate the Jacobian with respect to.
    * @param jacobianToPack the matrix in which to pack the Jacobian.
    */
   void calculateJacobian(DMatrixRMaj x, DMatrixRMaj jacobianToPack);

   int getParameterSize();

   int getResidualSize();
}
