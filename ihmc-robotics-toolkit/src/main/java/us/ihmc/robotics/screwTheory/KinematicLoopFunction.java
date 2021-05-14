package us.ihmc.robotics.screwTheory;

import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;

/**
 * A kinematic loop function represents the explicit formulation of the interdependency between the
 * joints composing a single kinematic loop.
 * <p>
 * The general formulation of such dependency can be written as follows:
 * 
 * <pre>
 * q     = &Gamma;(y)
 * qDot  = G * yDot
 * qDDot = G * yDDot + g
 * </pre>
 * 
 * where:
 * <ul>
 * <li><tt>q</tt>, <tt>qDot</tt>, and <tt>qDDot</tt> are the configuration, velocity, and
 * acceleration vectors, respectively, for the kinematic loop joints.
 * <li><tt>y</tt> is a vector of independent variables providing sufficient information to fully
 * define the kinematic loop configuration. This can be a chosen subset of <tt>q</tt>.
 * <li><tt>yDot</tt> is a vector of independent variables providing sufficient information to fully
 * define the kinematic loop joint velocities. It is the first time derivative of <tt>y</tt>.
 * <li><tt>yDDot</tt> is a vector of independent variables providing sufficient information to fully
 * define the kinematic loop joint accelerations. It is the second time derivative of <tt>y</tt>.
 * <li><tt>&Gamma;(.)</tt> is the function that maps from <tt>y</tt> to the loop configuration space
 * <tt>q</tt>.
 * <li><tt>G</tt> is the linear transform from <tt>yDot</tt> the loop joint velocity space, this is
 * also referred to as the <b>loop Jacobian</b>.
 * <li><tt>g</tt> is a bias term when transforming from <tt>yDot</tt> the loop joint acceleration
 * space, this is also referred to as the <b>loop convective term</b>.
 * </ul>
 * </p>
 * <p>
 * The framework is inspired from R. Featherstone's book on
 * <a href="https://bcourses.berkeley.edu/files/70847609/download?download_frd=1">Rigid-Body
 * Dynamics Algorithms - Chapter 8: Closed Loop Systems</a>.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface KinematicLoopFunction
{
   /**
    * Returns the indices of the joints that are actuated.
    * <p>
    * The indices should be consistent with the joint position in {@link #getLoopJoints()} and the
    * number of indices should be equal to the number of columns in the loop Jacobian.
    * </p>
    * 
    * @return the indices of the actuated joints.
    */
   int[] getActuatedJointIndices();

   /**
    * Returns the updated loop Jacobian used to perform the following transformations:
    * 
    * <pre>
    * qDot  = G * yDot
    * qDDot = G * yDDot + g
    * </pre>
    * <p>
    * The {@code G} matrix is a N-by-M matrix which row indexing follows the ordering of the joints
    * according to {@link #getLoopJoints()}, where N is the number of DoFs of the kinematic loop, and
    * where M is the number of independent DoFs of the loop.
    * </p>
    * 
    * @return the loop Jacobian matrix.
    * @see KinematicLoopFunction
    */
   DMatrixRMaj getLoopJacobian();

   /**
    * Returns the updated loop convective term used to perform the following transformation:
    * 
    * <pre>
    * qDDot = G * yDDot + g
    * </pre>
    * <p>
    * The {@code g} matrix is a N-by-1 matrix which indexing follows the ordering of the joints
    * according to {@link #getLoopJoints()}, and where N is the number of DoFs of the kinematic loop.
    * </p>
    * 
    * @return the loop convective term.
    * @see KinematicLoopFunction
    */
   DMatrixRMaj getLoopConvectiveTerm();

   /**
    * Given pre-computed joint configuration in {@code jointConfigurations}, this method allows to
    * enforce the loop kinematic constraint by adjusting the given vector.
    * <p>
    * The {@code jointConfigurations} matrix is a N-by-1 matrix which indexing follows the ordering of
    * the joints according to {@link #getLoopJoints()}, and where N is the configuration size of the
    * kinematic loop.
    * </p>
    * <p>
    * This method is typically invoked after integration of the joint velocities/accelerations to
    * update the joint configurations.
    * </p>
    * 
    * @param jointConfigurations the matrix containing the joint configurations to be updated.
    *                            Modified.
    */
   void adjustConfiguration(DMatrixRMaj jointConfigurations);

   /**
    * Given pre-computed joint effort in {@code jointTaus}, this method allows the loop to redistribute
    * the effort to the active joints.
    * <p>
    * The {@code jointTaus} matrix is a N-by-1 matrix which indexing follows the ordering of the joints
    * according to {@link #getLoopJoints()}, and where N is the number of DoFs of the kinematic loop.
    * </p>
    * <p>
    * This method is typically invoked after {@link InverseDynamicsCalculator}.
    * </p>
    * 
    * @param jointTaus the matrix containing the joint efforts to be updated. Modified.
    */
   void adjustTau(DMatrixRMaj jointTaus);

   /**
    * Returns the list of all the joints composing the kinematic loop.
    * <p>
    * The ordering of the joints in the returned list is used for indexing the matrices.
    * </p>
    * 
    * @return the list of the kinematic loop joints.
    */
   List<? extends OneDoFJointReadOnly> getLoopJoints();
}
