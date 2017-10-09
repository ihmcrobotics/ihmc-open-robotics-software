package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;

/**
 * This class provides similar tools to {@link GeometricJacobian} but also support additional
 * features.
 * <p>
 * The kinematic chain this Jacobian represents is mutable: The base, end-effector, and the Jacobian
 * reference frame can all be set via: {@link #setBase(RigidBody)},
 * {@link #setEndEffector(RigidBody)}, and {@link #setJacobianFrame(ReferenceFrame)}, respectively.
 * </p>
 * <p>
 * Finally, this calculator can calculate to what is referred here as convective term. When the
 * resulting joint accelerations 'qDDot' are to be computed from a given end-effector spatial
 * acceleration 'xDDot' the following is needed:<br>
 * Differentiating: J * qDot = xDot<br>
 * Gives: JDot * qDot + J * qDDot = xDDot<br>
 * Then: qDDot = J^-1 * (xDDot - JDot * qDot)<br>
 * The term 'JDot * qDot' is the convective term being referred to, needed to compute qDDot. It can
 * be calculated using the method {@link #computeConvectiveTerm(TwistCalculator)}.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public class GeometricJacobianCalculator
{
   /** The base is the predecessor of the first joint that the Jacobian considers. */
   private RigidBody base;
   /** The end-effector is the successor of the last joint that the Jacobian considers. */
   private RigidBody endEffector;
   private final List<InverseDynamicsJoint> jointsFromBaseToEndEffector = new ArrayList<>();
   private ReferenceFrame jacobianFrame;

   private int numberOfDegreesOfFreedom;
   private final DenseMatrix64F jacobianMatrix = new DenseMatrix64F(6, 12);
   private final DenseMatrix64F convectiveTerm = new DenseMatrix64F(6, 1);

   private final Twist tempTwist = new Twist();
   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(6, 1);

   private final SpatialAccelerationVector zeroAcceleration = new SpatialAccelerationVector();
   private final SpatialAccelerationVector biasAcceleration = new SpatialAccelerationVector();
   private final Twist twistOfCurrentBodyRelativeToEndEffector = new Twist();
   private final Twist twistOfCurrentBodyRelativeToPredecessor = new Twist();

   /**
    * Creates an empty calculator.
    * <p>
    * At least the base, end-effector of the desired Jacobian are to be provided before trying to
    * compute it.
    * </p>
    */
   public GeometricJacobianCalculator()
   {
      clear();
   }

   /**
    * Clears the internal memory of this calculator, ensuring new data has to be provided, including
    * base and end-effector.
    */
   public void clear()
   {
      base = null;
      endEffector = null;
      numberOfDegreesOfFreedom = -1;
      jacobianFrame = null;
      jointsFromBaseToEndEffector.clear();

      clearJacobianMatrix();
   }

   /**
    * Clears the internal memory about the Jacobian matrix and the convective term.
    * <p>
    * After calling this method, the methods {@link #computeJacobianMatrix()} and
    * {@link #computeConvectiveTerm(TwistCalculator)} will have to be called.
    * </p>
    */
   public void clearJacobianMatrix()
   {
      jacobianMatrix.reshape(0, 0);
      convectiveTerm.reshape(0, 0);
   }

   /**
    * Sets the new kinematic chain the Jacobian is computed for.
    * <p>
    * Note that if the Jacobian frame was not set beforehand, it is automatically set to
    * {@code endEffector.getBodyFixedFrame()}. It can be changed via
    * {@link #setJacobianFrame(ReferenceFrame)}.
    * </p>
    * 
    * @param base the new base to use.
    * @param endEffector the new end-effector.
    */
   public void setKinematicChain(RigidBody base, RigidBody endEffector)
   {
      this.base = base;
      this.endEffector = endEffector;
      if (jacobianFrame == null)
         jacobianFrame = endEffector.getBodyFixedFrame();
      clearJacobianMatrix();

      jointsFromBaseToEndEffector.clear();
      int distanceFromBase = ScrewTools.computeDistanceToAncestor(endEffector, base);
      if (distanceFromBase < 0)
         throw new RuntimeException("The base: " + base.getName() + "is not an ancestor of the given end-effector: " + endEffector.getName());

      while (jointsFromBaseToEndEffector.size() < distanceFromBase)
         jointsFromBaseToEndEffector.add(null);

      numberOfDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(base, endEffector);

      RigidBody currentBody = endEffector;
      int index = jointsFromBaseToEndEffector.size() - 1;

      while (currentBody != base)
      {
         InverseDynamicsJoint joint = currentBody.getParentJoint();
         jointsFromBaseToEndEffector.set(index--, joint);
         currentBody = joint.getPredecessor();
      }
   }

   /**
    * Sets the new kinematic chain the Jacobian is computed for.
    * <p>
    * Note that if the Jacobian frame was not set beforehand, it is automatically set to
    * {@code endEffector.getBodyFixedFrame()}. It can be changed via
    * {@link #setJacobianFrame(ReferenceFrame)}.
    * </p>
    * <p>
    * This method orders if necessary the joints such that the resulting kinematic chain starts from
    * the base and ends at the end-effector.
    * </p>
    * 
    * @param joints the array of joints to use for computing the Jacobian. Not modified.
    * @throws RuntimeException if {@code joints[0].getPredecessor()} is not the ancestor of
    *            {@code joints[joints.length - 1].getSuccessor()} and that
    *            {@code joints[joints.length - 1].getPredecessor()} is not the ancestor of
    *            {@code joints[0].getSuccessor()}.
    */
   public void setKinematicChain(InverseDynamicsJoint[] joints)
   {
      base = joints[0].getPredecessor();
      endEffector = joints[joints.length - 1].getSuccessor();

      numberOfDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(joints);

      if (ScrewTools.isAncestor(endEffector, base))
      {
         if (jacobianFrame == null)
            jacobianFrame = endEffector.getBodyFixedFrame();
         clearJacobianMatrix();

         jointsFromBaseToEndEffector.clear();
         for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
            jointsFromBaseToEndEffector.add(joints[jointIndex]);
      }
      else
      {
         base = joints[joints.length - 1].getPredecessor();
         endEffector = joints[0].getSuccessor();
         if (!ScrewTools.isAncestor(endEffector, base))
            throw new RuntimeException("Unable to process the array of joints: " + Arrays.toString(joints));

         if (jacobianFrame == null)
            jacobianFrame = endEffector.getBodyFixedFrame();
         clearJacobianMatrix();

         jointsFromBaseToEndEffector.clear();
         for (int jointIndex = joints.length - 1; jointIndex >= 0; jointIndex--)
            jointsFromBaseToEndEffector.add(joints[jointIndex]);
      }
   }

   /**
    * Sets the reference frame in which the Jacobian should be expressed.
    * <p>
    * This method also changes the reference of the end-effector fixed point to match the Jacobian
    * frame.
    * </p>
    * 
    * @param jacobianFrame the new frame for the Jacobian matrix.
    */
   public void setJacobianFrame(ReferenceFrame jacobianFrame)
   {
      this.jacobianFrame = jacobianFrame;
   }

   /**
    * Updates the values of the Jacobian matrix.
    * <p>
    * The base, end-effector, jacobian frame, and end-effector fixed point, have all to be properly
    * set before calling this method.
    * </p>
    * 
    * @throws RuntimeException if either the base or the end-effector has not been provided
    *            beforehand.
    */
   public void computeJacobianMatrix()
   {
      if (base == null || endEffector == null)
         throw new RuntimeException("The base and end-effector have to be set first.");

      jacobianMatrix.reshape(SpatialMotionVector.SIZE, numberOfDegreesOfFreedom);

      int column = 0;

      for (int jointIndex = 0; jointIndex < jointsFromBaseToEndEffector.size(); jointIndex++)
      {
         InverseDynamicsJoint joint = jointsFromBaseToEndEffector.get(jointIndex);

         for (int dofIndex = 0; dofIndex < joint.getDegreesOfFreedom(); dofIndex++)
         {
            joint.getUnitTwist(dofIndex, tempTwist);
            tempTwist.changeFrame(jacobianFrame);
            tempTwist.getMatrix(jacobianMatrix, 0, column++);
         }
      }
   }

   /**
    * Computes the convective term C<sub>6x1</sub> = JDot<sub>6xN</sub> * qDot<sub>Nx1</sub>.<br>
    * where N is the number of degrees of freedom between the {@code base} and {@code endEffector},
    * JDot<sub>6xN</sub> is the time-derivative of the Jacobian matrix, and qDot<sub>Nx1</sub> the
    * vector of joint velocities.
    * <p>
    * <b>WARNING: The {@code jacobianFrame} is assumed to be rigidly attached to the
    * end-effector.</b>
    * </p>
    * <p>
    * The convective term represents the Coriolis acceleration of the {@code endEffector} relative
    * to the {@code base} expressed at the {@code jacobianFrame}. The Coriolis acceleration is an
    * acceleration that results from the combination of the angular and linear velocities happening
    * on the end-effector.
    * </p>
    * <p>
    * As shown in <a href="https://en.wikipedia.org/wiki/Coriolis_force">Wikipedia</a>, the Coriolis
    * acceleration also depends on the velocity of the frame it is expressed in. This method assumes
    * that the {@code jacobianFrame} is attached to the end-effector. If this is not the case, the
    * computed acceleration will be biased.
    * </p>
    * <p>
    * Using the convective term, the spatial acceleration xDDot<sub>6x1</sub> of the
    * {@code endEffector} relative to the {@code base} expressed in the {@code jacbianFrame} can be
    * computed as follows:<br>
    * xDDot<sub>6x1</sub> = J<sub>6xN</sub> * qDDot<sub>Nx1</sub> + C<sub>6x1</sub><br>
    * where qDDot<sub>Nx1</sub> is the vector of joint accelerations.
    * </p>
    * <p>
    * The base, end-effector, jacobian frame, and end-effector fixed point, have all to be properly
    * set before calling this method.
    * </p>
    * 
    * @throws RuntimeException if either the base or the end-effector has not been provided
    *            beforehand.
    */
   public void computeConvectiveTerm()
   {
      if (base == null || endEffector == null)
         throw new RuntimeException("The base and end-effector have to be set first.");

      ReferenceFrame endEffectorFrame = getEndEffectorFrame();
      twistOfCurrentBodyRelativeToEndEffector.setToZero(endEffectorFrame, endEffectorFrame, endEffectorFrame);
      biasAcceleration.setToZero(endEffectorFrame, endEffectorFrame, jacobianFrame);

      RigidBody currentBody = endEffector;

      while (currentBody != base)
      {
         InverseDynamicsJoint joint = currentBody.getParentJoint();
         ReferenceFrame currentBodyFrame = currentBody.getBodyFixedFrame();
         RigidBody predecessor = joint.getPredecessor();
         ReferenceFrame predecessorFrame = predecessor.getBodyFixedFrame();

         // This is now the velocity of the current body relative to the end-effector expressed in the current body frame.
         twistOfCurrentBodyRelativeToEndEffector.changeFrame(currentBodyFrame);
         // Velocity of the current body relative to the predecessor expressed in the current body frame.
         joint.getSuccessorTwist(twistOfCurrentBodyRelativeToPredecessor);

         zeroAcceleration.setToZero(currentBodyFrame, predecessorFrame, currentBodyFrame);
         /*
          * This change frame on the 'zero-acceleration' vector is used to calculate the Coriolis
          * acceleration induced by the combination of the velocities from the joints that are
          * between the end-effector and the current body. As the reference frames are well enforced
          * in the changeFrame method, these specific twists have to be used.
          */
         zeroAcceleration.changeFrame(endEffectorFrame, twistOfCurrentBodyRelativeToEndEffector, twistOfCurrentBodyRelativeToPredecessor);
         // The following line is where the jacobianFrame is assumed to be rigidly attached to the end-effector.
         zeroAcceleration.changeFrameNoRelativeMotion(jacobianFrame);
         zeroAcceleration.add(biasAcceleration);
         biasAcceleration.set(zeroAcceleration);

         // This is now the twist of the predecessor relative to the current body expressed in the current body frame.
         twistOfCurrentBodyRelativeToPredecessor.invert();
         // The velocity of the predecessor relative to the end-effector expressed in the current body frame.
         twistOfCurrentBodyRelativeToEndEffector.add(twistOfCurrentBodyRelativeToPredecessor);

         currentBody = predecessor;
      }

      convectiveTerm.reshape(6, 1);
      biasAcceleration.getMatrix(convectiveTerm, 0);
      return;
   }

   /**
    * Computes and packs the twist of the end-effector relative to the base that is induced by the
    * given joint velocity vector.
    * 
    * @param jointVelocities the joint velocity column vector, starting from base child joint
    *           velocity.
    * @return the twist of the end effector with respect to the base, expressed in the
    *         jacobianFrame.
    * @throws RuntimeException if the Jacobian matrix has not been computed yet.
    */
   public void getEndEffectorTwist(DenseMatrix64F jointVelocities, Twist twistToPack)
   {
      if (jacobianMatrix.getNumRows() == 0)
         throw new RuntimeException("The Jacobian matrix has to be computed first.");

      tempMatrix.reshape(6, 1);
      CommonOps.mult(jacobianMatrix, jointVelocities, tempMatrix);
      twistToPack.set(getEndEffectorFrame(), getBaseFrame(), jacobianFrame, tempMatrix, 0);
   }

   /**
    * Computes and packs the spatial acceleration of the end-effector relative to the base that is
    * induced by the given joint acceleration vector.
    * 
    * @param jointAccelerations the joint acceleration column vector, starting from base child joint
    *           acceleration.
    * @return the twist of the end effector with respect to the base, expressed in the
    *         jacobianFrame.
    * @throws RuntimeException if the Jacobian matrix or the convective term has not been computed
    *            yet.
    */
   public void getEndEffectorAcceleration(DenseMatrix64F jointAccelerations, SpatialAccelerationVector spatialAccelerationToPack)
   {
      if (jacobianMatrix.getNumRows() == 0 || convectiveTerm.getNumRows() == 0)
         throw new RuntimeException("The Jacobian matrix and the convective term have to be computed first.");

      tempMatrix.reshape(6, 1);
      CommonOps.mult(jacobianMatrix, jointAccelerations, tempMatrix);
      CommonOps.addEquals(tempMatrix, convectiveTerm);
      spatialAccelerationToPack.set(getEndEffectorFrame(), getBaseFrame(), jacobianFrame, tempMatrix, 0);
   }

   /**
    * Computes and packs the joint torque vector that corresponds to the given wrench.
    * 
    * @param endEffectorWrench the resulting wrench at the end effector. The wrench should be
    *           expressed in {@code jacobianFrame} and the wrench's {@code bodyFrame} should be the
    *           body fixed frame of the end-effector.
    * @throws ReferenceFrameMismatchException if the given wrench
    *            {@code wrench.getExpressedInFrame() != this.getJacobianFrame()} or
    *            {@code wrench.getBodyFrame() != this.getEndEffectorFrame()}.
    * @throws RuntimeException if the Jacobian matrix has not been computed yet.
    */
   public void getJointTorques(Wrench endEffectorWrench, DenseMatrix64F jointTorquesToPack)
   {
      if (jacobianMatrix.getNumRows() == 0)
         throw new RuntimeException("The Jacobian matrix has to be computed first.");

      endEffectorWrench.getExpressedInFrame().checkReferenceFrameMatch(this.jacobianFrame);
      endEffectorWrench.getBodyFrame().checkReferenceFrameMatch(getEndEffectorFrame());

      endEffectorWrench.getMatrix(tempMatrix);
      jointTorquesToPack.reshape(1, jacobianMatrix.getNumCols());
      CommonOps.multTransA(tempMatrix, jacobianMatrix, jointTorquesToPack);
      CommonOps.transpose(jointTorquesToPack);
   }

   /**
    * Returns the base {@code RigidBody} of the current Jacobian. The base is the predecessor of the
    * first joint that the Jacobian considers.
    * 
    * @return the base of the Jacobian.
    */
   public RigidBody getBase()
   {
      return base;
   }

   /**
    * Returns the body fixed frame of the base {@code RigidBody} of the current Jacobian. The base
    * is the predecessor of the first joint that the Jacobian considers.
    * 
    * @return the body fixed frame of the base.
    */
   public ReferenceFrame getBaseFrame()
   {
      return base.getBodyFixedFrame();
   }

   /**
    * Returns the end-effector {@code RigidBody} of the current Jacobian. The end-effector is the
    * successor of the last joint the Jacobian considers.
    * 
    * @return the end-effector of the jacobian.
    */
   public RigidBody getEndEffector()
   {
      return endEffector;
   }

   /**
    * Returns the body fixed frame of the end-effector {@code RigidBody} of the current Jacobian.
    * The end-effector is the successor of the last joint the Jacobian considers.
    * 
    * @return the body fixed frame of the end-effector.
    */
   public ReferenceFrame getEndEffectorFrame()
   {
      return endEffector.getBodyFixedFrame();
   }

   /**
    * @return the frame in which the current Jacobian is expressed.
    */
   public ReferenceFrame getJacobianFrame()
   {
      return jacobianFrame;
   }

   /**
    * @return the number of degrees of freedom that the current kinematic chain considered contains.
    */
   public int getNumberOfDegreesOfFreedom()
   {
      return numberOfDegreesOfFreedom;
   }

   /**
    * Gets the list of joints considered by the current Jacobian ordered from the base to the
    * end-effector.
    * 
    * @return the list of joints.
    */
   public List<InverseDynamicsJoint> getJointsFromBaseToEndEffector()
   {
      return jointsFromBaseToEndEffector;
   }

   /**
    * Gets the current value of the Jacobian matrix.
    * <p>
    * This method does not recompute the Jacobian matrix, this has to be done beforehand via
    * {@link #computeJacobianMatrix()}.
    * </p>
    * 
    * @param jacobianMatrixToPack the dense matrix in which the Jacobian value is stored. Modified.
    * @throws RuntimeException if the Jacobian matrix has not been computed yet.
    */
   public void getJacobianMatrix(DenseMatrix64F jacobianMatrixToPack)
   {
      if (jacobianMatrix.getNumRows() == 0)
         throw new RuntimeException("The Jacobian matrix has to be computed first.");

      jacobianMatrixToPack.set(jacobianMatrix);
   }

   /**
    * Gets a subspace of the current value of the Jacobian matrix: J'<sub>MxN</sub> =
    * S<sub>Mx6</sub> * J<sub>6xN</sub>.
    * <p>
    * This method does not recompute the Jacobian matrix, this has to be done beforehand via
    * {@link #computeJacobianMatrix()}.
    * </p>
    * 
    * @param selectionMatrix the selection to apply on the Jacobian matrix. Not modified.
    * @param jacobianMatrixToPack the dense matrix in which the resulting the subspace of the
    *           Jacobian is stored. Modified.
    * @throws RuntimeException if the Jacobian matrix has not been computed yet.
    */
   public void getJacobianMatrix(DenseMatrix64F selectionMatrix, DenseMatrix64F jacobianMatrixToPack)
   {
      if (jacobianMatrix.getNumRows() == 0)
         throw new RuntimeException("The Jacobian matrix has to be computed first.");

      jacobianMatrixToPack.reshape(selectionMatrix.getNumRows(), numberOfDegreesOfFreedom);
      CommonOps.mult(selectionMatrix, jacobianMatrix, jacobianMatrixToPack);
   }

   /**
    * Gets the current value of the convective term: JDot * qDot.
    * <p>
    * This method does not recompute the convective term, this has to be done beforehand via
    * {@link #computeConvectiveTerm(TwistCalculator)}.
    * </p>
    * <p>
    * As for the Jacobian matrix, the convective term is computed in {@link #jacobianFrame}.
    * </p>
    * 
    * @param convectiveTermToPack the dense matrix in which the convective term value is stored.
    *           Modified.
    * @throws RuntimeException if the convective term has not been computed yet.
    */
   public void getConvectiveTerm(DenseMatrix64F convectiveTermToPack)
   {
      if (convectiveTerm.getNumRows() == 0)
         throw new RuntimeException("The convective term has to be computed first.");

      convectiveTermToPack.set(convectiveTerm);
   }

   /**
    * Gets a subspace of the current value of the convective term: C'<sub>Mx1</sub> =
    * S<sub>Mx6</sub> * C<sub>6x1</sub>.
    * <p>
    * This method does not recompute the convective term, this has to be done beforehand via
    * {@link #computeJacobianMatrix()}.
    * </p>
    * 
    * @param selectionMatrix the selection to apply on the convective term. Not modified.
    * @param convectiveTermToPack the dense matrix in which the convective term value is stored.
    *           Modified.
    * @throws RuntimeException if the convective term has not been computed yet.
    */
   public void getConvectiveTerm(DenseMatrix64F selectionMatrix, DenseMatrix64F convectiveTermToPack)
   {
      if (convectiveTerm.getNumRows() == 0)
         throw new RuntimeException("The convective term has to be computed first.");

      convectiveTermToPack.reshape(selectionMatrix.getNumRows(), 1);
      CommonOps.mult(selectionMatrix, convectiveTerm, convectiveTermToPack);
   }

   public void getConvectiveTerm(SpatialAccelerationVector convectiveTermToPack)
   {
      if (convectiveTerm.getNumRows() == 0)
         throw new RuntimeException("The convective term has to be computed first.");

      convectiveTermToPack.set(getEndEffectorFrame(), getBaseFrame(), jacobianFrame, convectiveTerm, 0);
   }

   /**
    * Creates a descriptive {@code String} for this Jacobian containing information such as the
    * {@code jacobianFrame}, the list of the joint names, and the current value of the Jacobian
    * matrix.
    * 
    * @return a descriptive {@code String} for this Jacobian.
    */
   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      builder.append("Jacobian. jacobianFrame = " + jacobianFrame + ". Joints:\n");

      RigidBody currentBody = endEffector;

      while (currentBody != base)
      {
         InverseDynamicsJoint joint = currentBody.getParentJoint();
         builder.append(joint.getClass().getSimpleName() + " " + joint.getName() + "\n");
         currentBody = joint.getPredecessor();
      }

      builder.append("\n");
      builder.append(jacobianMatrix.toString());

      return builder.toString();
   }

   public String getShortInfo()
   {
      return "Jacobian, end effector = " + getEndEffector() + ", base = " + getBase() + ", expressed in " + getJacobianFrame();
   }

}
