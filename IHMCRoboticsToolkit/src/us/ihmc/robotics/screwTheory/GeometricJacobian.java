package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.nameBasedHashCode.NameBasedHashCodeHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * This class provides a simple tool to compute the Jacobian matrix of a kinematic chain composed of
 * {@link InverseDynamicsJoint}s.
 * <p>
 * To use this tool, one can create an object by using the constructor
 * {@link #GeometricJacobian(RigidBody, RigidBody, ReferenceFrame)}. The Jacobian computed will
 * include all the joints from {@code ancestor} to {@code descendant} and will be expressed in
 * {@code jacobianFrame}. Then the method {@link #compute()} is to be called every time the joint
 * configuration has changed and Jacobian matrix is to be updated. Finally several options are
 * offered on how to use the Jacobian matrix:
 * <ul>
 * <li>{@link #getJacobianMatrix()} can be used to get the reference to the raw matrix can be used
 * to then perform external operations with it using the third-party library EJML.
 * <li>{@link #computeJointTorques(Wrench, DenseMatrix64F)} computes the joint torques necessary to
 * achieve a given {@code Wrench} at the end-effector or descendant of this Jacobian. It uses the
 * relation: <br>
 * &tau; = J<sup>T</sup> * W </br>
 * with &tau; being the joint torques, J the Jacobian matrix, and W the wrench to exert at the
 * end-effector.
 * </ul>
 * </p>
 */
public class GeometricJacobian implements NameBasedHashCodeHolder
{
   /** Array of the joints to be considered by this Jacobian. */
   private final InverseDynamicsJoint[] joints;
   /**
    * Array of ALL the joints from the base to the end effector of this Jacobian. This is useful in
    * {@link ConvectiveTermCalculator} when the list of joints of the Jacobian is not continuous.
    */
   private final InverseDynamicsJoint[] jointPathFromBaseToEndEffector;
   private final DenseMatrix64F jacobian;
   private ReferenceFrame jacobianFrame;

   // Temporary variables
   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(Twist.SIZE, 1);

   private final boolean allowChangeFrame;
   private final long nameBasedHashCode;

   /**
    * Creates the Jacobian for the kinematic chain described by the given joints. These joints have
    * to be ordered and going downstream (the first joint has to be the closest joint to the root of
    * the system and the last the farthest).
    * 
    * @param joints the joints that this Jacobian considers.
    * @param jacobianFrame the frame in which the resulting twist of the end effector with respect
    *           to the base frame will be expressed.
    * @throws RuntimeException if the joint ordering is incorrect.
    */
   public GeometricJacobian(InverseDynamicsJoint[] joints, ReferenceFrame jacobianFrame)
   {
      this(joints, jacobianFrame, true);
   }

   /**
    * Creates the Jacobian for the kinematic chain described by the given joints. These joints have
    * to be ordered and going downstream (the first joint has to be the closest joint to the root of
    * the system and the last the farthest).
    * 
    * @param joints the joints that this Jacobian considers.
    * @param jacobianFrame the frame in which the resulting twist of the end effector with respect
    *           to the base frame will be expressed.
    * @param allowChangeFrame whether the feature {@link #changeFrame(ReferenceFrame)} will be
    *           available or not.
    * @throws RuntimeException if the joint ordering is incorrect.
    */
   public GeometricJacobian(InverseDynamicsJoint[] joints, ReferenceFrame jacobianFrame, boolean allowChangeFrame)
   {
      checkJointOrder(joints);
      this.joints = new InverseDynamicsJoint[joints.length];
      System.arraycopy(joints, 0, this.joints, 0, joints.length);
      this.jacobianFrame = jacobianFrame;
      this.jacobian = new DenseMatrix64F(SpatialMotionVector.SIZE, ScrewTools.computeDegreesOfFreedom(joints));
      this.jointPathFromBaseToEndEffector = ScrewTools.createJointPath(getBase(), getEndEffector());
      this.allowChangeFrame = allowChangeFrame;

      nameBasedHashCode = ScrewTools.computeGeometricJacobianNameBasedHashCode(joints, jacobianFrame, allowChangeFrame);
   }

   /**
    * Constructor for internal usage of the screw theory library.
    * <p>
    * Creates a new Jacobian representing the motion subspace of the given joint.
    * </p>
    * 
    * @param joint the joint for which the motion subspace is to be created.
    * @param unitTwists the list of unitary twists representing the degrees of freedom of
    *           {@code joint}. For instance, the unit twist of a {@link RevoluteJoint} around the
    *           y-axis is: T<sub>u</sub> = [0 1 0 0 0 0]<sup>T</sup>.
    * @param jacobianFrame the frame in which the resulting twist of the end effector with respect
    *           to the base frame will be expressed.
    * @throws RuntimeException if the joint ordering is incorrect.
    */
   public GeometricJacobian(InverseDynamicsJoint joint, ReferenceFrame jacobianFrame)
   {
      this(new InverseDynamicsJoint[] {joint}, jacobianFrame, true);
   }

   /**
    * Creates a new Jacobian for the kinematic starting from {@code ancestor} and ending at
    * {@code descendant}.
    * 
    * @param ancestor the predecessor of the first joint considered by this Jacobian.
    * @param descendant the successor of the last joint considered by this Jacobian.
    * @param jacobianFrame the frame in which the resulting twist of the end effector with respect
    *           to the base frame will be expressed.
    * @throws RuntimeException if the joint ordering is incorrect.
    */
   public GeometricJacobian(RigidBody ancestor, RigidBody descendant, ReferenceFrame jacobianFrame)
   {
      this(ScrewTools.createJointPath(ancestor, descendant), jacobianFrame, true);
   }

   /**
    * Temporary twist used for intermediate garbage free operations. To use only in the method
    * {@link #compute()}.
    */
   private final Twist tempTwist = new Twist();

   /**
    * Computes the Jacobian.
    */
   public void compute()
   {
      int column = 0;
      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         InverseDynamicsJoint joint = joints[jointIndex];

         for (int dofIndex = 0; dofIndex < joint.getDegreesOfFreedom(); dofIndex++)
         {
            joint.getUnitTwist(dofIndex, tempTwist);
            tempTwist.changeFrame(jacobianFrame);
            tempTwist.getMatrix(jacobian, 0, column++);
         }
      }
   }

   /**
    * Changes the frame in which the resulting twist of the end effector with respect to the base
    * frame will be expressed. The boolean {@code allowChangeFrame} has to be initialize to
    * {@code true} at construction time.
    * <p>
    * WARNING: This method only changes the current frame without updating the Jacobian matrix. The
    * method {@link #compute()} after having changed the frame.
    * </p>
    * 
    * @param jacobianFrame the new frame of this Jacobian.
    * @throws RuntimeException if this feature is not enabled.
    */
   public void changeFrame(ReferenceFrame jacobianFrame)
   {
      if (!allowChangeFrame)
         throw new RuntimeException("Cannot change the frame of this Jacobian.");

      this.jacobianFrame = jacobianFrame;
   }

   /**
    * Computes and packs the twist that corresponds to the given joint velocity vector.
    * 
    * @param jointVelocities the joint velocity column vector. The vector should have the same
    *           ordering as the unit twists of the ordered joint list of this Jacobian that were
    *           passed in during construction.
    * @return the twist of the end effector with respect to the base, expressed in the
    *         jacobianFrame.
    */
   public void getTwist(DenseMatrix64F jointVelocities, Twist twistToPack)
   {
      CommonOps.mult(jacobian, jointVelocities, tempMatrix);
      twistToPack.set(getEndEffectorFrame(), getBaseFrame(), jacobianFrame, tempMatrix, 0);
   }

   /**
    * Computes and returns the joint torque vector that corresponds to the given wrench.
    * 
    * @param wrench the resulting wrench at the end effector. The wrench should be expressed in
    *           {@code jacobianFrame} and the wrench's {@code bodyFrame} should be the body fixed
    *           frame of the end-effector.
    * @return joint torques that result in the given wrench as a column vector.
    * @throws ReferenceFrameMismatchException if the given wrench
    *            {@code wrench.getExpressedInFrame() != this.getJacobianFrame()} or
    *            {@code wrench.getBodyFrame() != this.getEndEffectorFrame()}.
    */
   public DenseMatrix64F computeJointTorques(Wrench wrench)
   {
      DenseMatrix64F jointTorques = new DenseMatrix64F(1, jacobian.getNumCols());
      computeJointTorques(wrench, jointTorques);
      return jointTorques;
   }

   /**
    * Computes and packs the joint torque vector that corresponds to the given wrench.
    * 
    * @param wrench the resulting wrench at the end effector. The wrench should be expressed in
    *           {@code jacobianFrame} and the wrench's {@code bodyFrame} should be the body fixed
    *           frame of the end-effector.
    * @throws ReferenceFrameMismatchException if the given wrench
    *            {@code wrench.getExpressedInFrame() != this.getJacobianFrame()} or
    *            {@code wrench.getBodyFrame() != this.getEndEffectorFrame()}.
    */
   public void computeJointTorques(Wrench wrench, DenseMatrix64F jointTorquesToPack)
   {
      // reference frame check
      wrench.getExpressedInFrame().checkReferenceFrameMatch(this.jacobianFrame);
      // FIXME add the following reference frame check
      //      wrench.getBodyFrame().checkReferenceFrameMatch(getEndEffectorFrame());
      wrench.getMatrix(tempMatrix);
      jointTorquesToPack.reshape(1, jacobian.getNumCols());
      CommonOps.multTransA(tempMatrix, jacobian, jointTorquesToPack);
      CommonOps.transpose(jointTorquesToPack);
   }

   /**
    * Returns a reference to the underlying {@code DenseMatrix64F} object. Does not recompute the
    * Jacobian.
    * 
    * @return a reference to the underlying {@code DenseMatrix64F} object
    */
   public DenseMatrix64F getJacobianMatrix()
   {
      return jacobian;
   }

   /**
    * @return the determinant of the Jacobian matrix
    */
   public double det()
   {
      return CommonOps.det(jacobian);
   }

   /**
    * Returns one entry from the Jacobian matrix. Does not recompute the Jacobian.
    * 
    * @param row the desired row of the Jacobian
    * @param column the desired column of the Jacobian
    * @return the entry at (row, column)
    */
   public double getJacobianEntry(int row, int column)
   {
      return jacobian.get(row, column);
   }

   /**
    * @return the number of columns in the Jacobian matrix
    */
   public int getNumberOfColumns()
   {
      return jacobian.getNumCols();
   }

   /**
    * @return the frame in which this Jacobian is expressed
    */
   public ReferenceFrame getJacobianFrame()
   {
      return jacobianFrame;
   }

   /** @return an {@code Array} containing the joints considered by this Jacobian. */
   public InverseDynamicsJoint[] getJointsInOrder()
   {
      return joints;
   }

   /**
    * @return a {@code Array} of joints describing a continuous path from the base to the end
    *         effector of this Jacobian.
    */
   public InverseDynamicsJoint[] getJointPathFromBaseToEndEffector()
   {
      return jointPathFromBaseToEndEffector;
   }

   /**
    * Returns the base {@code RigidBody} of this Jacobian. The base is the predecessor of the first
    * joint that this Jacobian considers.
    * 
    * @return the base of this Jacobian.
    */
   public RigidBody getBase()
   {
      return joints[0].getPredecessor();
   }

   /**
    * Returns the end-effector {@code RigidBody} of this Jacobian. The end-effector is the successor
    * of the last joint this Jacobian considers.
    * 
    * @return the end-effector of this jacobian.
    */
   public RigidBody getEndEffector()
   {
      return joints[joints.length - 1].getSuccessor();
   }

   /**
    * Returns the body fixed frame of the base {@code RigidBody} of this Jacobian. The base is the
    * predecessor of the first joint that this Jacobian considers.
    * 
    * @return the body fixed frame of the base.
    */
   public ReferenceFrame getBaseFrame()
   {
      return getBase().getBodyFixedFrame();
   }

   /**
    * Returns the body fixed frame of the end-effector {@code RigidBody} of this Jacobian. The
    * end-effector is the successor of the last joint this Jacobian considers.
    * 
    * @return the body fixed frame of the end-effector.
    */
   public ReferenceFrame getEndEffectorFrame()
   {
      return getEndEffector().getBodyFixedFrame();
   }

   private static void checkJointOrder(InverseDynamicsJoint[] joints)
   {
      for (int i = 1; i < joints.length; i++)
      {
         InverseDynamicsJoint joint = joints[i];
         InverseDynamicsJoint previousJoint = joints[i - 1];
         if (ScrewTools.isAncestor(previousJoint.getPredecessor(), joint.getPredecessor()))
            throw new RuntimeException("joints must be in order from ancestor to descendant");
      }
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

      for (InverseDynamicsJoint joint : joints)
      {
         builder.append(joint.getClass().getSimpleName() + " " + joint.getName() + "\n");
      }

      builder.append("\n");
      builder.append(jacobian.toString());

      return builder.toString();
   }

   public String getShortInfo()
   {
      return "Jacobian, end effector = " + getEndEffector() + ", base = " + getBase() + ", expressed in " + getJacobianFrame();
   }

   @Override
   public long getNameBasedHashCode()
   {
      return nameBasedHashCode;
   }
}
