package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.nameBasedHashCode.NameBasedHashCodeHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class GeometricJacobian implements NameBasedHashCodeHolder
{
   /** Array of the joints to be considered by this Jacobian. */
   private final InverseDynamicsJoint[] joints;
   /** Array of ALL the joints from the base to the end effector of this Jacobian. Not necessary the same as joints. */
   private final InverseDynamicsJoint[] jointPathFromBaseToEndEffector;
   private final LinkedHashMap<InverseDynamicsJoint, List<Twist>> unitTwistMap;
   private final List<Twist> unitTwistList;
   private final DenseMatrix64F jacobian;
   private ReferenceFrame jacobianFrame;

   // Temporary variables
   private final Twist tempTwist = new Twist();
   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(Twist.SIZE, 1);

   private final boolean allowChangeFrame;
   private final long nameBasedHashCode;

   /**
    * Creates a new Jacobian for the open loop chain given as an ArrayList of frames
    * @param unitTwists an ordered list of relative joint twists.
    *       The order in which the twists are ordered will correspond to the order of the columns in the Jacobian
    * @param jacobianFrame the frame in which the resulting twist of the end effector with respect to the base frame will be expressed.
    */

   private GeometricJacobian(LinkedHashMap<InverseDynamicsJoint, List<Twist>> unitTwists, ReferenceFrame jacobianFrame, boolean allowChangeFrame)
   {
      this.joints = new InverseDynamicsJoint[unitTwists.size()];
      unitTwists.keySet().toArray(joints);
      this.unitTwistMap = unitTwists;
      this.unitTwistList = extractUnitTwistList(unitTwists);
      this.jacobianFrame = jacobianFrame;
      this.jacobian = createJacobianMatrix(this.unitTwistMap);
      this.jointPathFromBaseToEndEffector = ScrewTools.createJointPath(getBase(), getEndEffector());
      this.allowChangeFrame = allowChangeFrame;

      nameBasedHashCode = ScrewTools.computeGeometricJacobianNameBasedHashCode(joints, jacobianFrame, allowChangeFrame);
   }

   public GeometricJacobian(InverseDynamicsJoint joint, List<Twist> unitTwists, ReferenceFrame jacobianFrame)
   {
      this(createUnitTwistMap(joint, unitTwists), jacobianFrame, true);
   }

   public GeometricJacobian(RigidBody ancestor, RigidBody descendant, ReferenceFrame jacobianFrame)
   {
      this(ScrewTools.createJointPath(ancestor, descendant), jacobianFrame, true);
   }

   public GeometricJacobian(InverseDynamicsJoint[] joints, ReferenceFrame jacobianFrame)
   {
      this(extractTwistsFromJoints(joints), jacobianFrame, true);
   }

   public GeometricJacobian(InverseDynamicsJoint[] joints, ReferenceFrame jacobianFrame, boolean allowChangeFrame)
   {
      this(extractTwistsFromJoints(joints), jacobianFrame, allowChangeFrame);
   }

   public GeometricJacobian(InverseDynamicsJoint joint, ReferenceFrame jacobianFrame)
   {
      this(joint, joint.getMotionSubspace().getAllUnitTwists(), jacobianFrame);
   }

   /**
    * Computes the Jacobian.
    */
   public void compute()
   {
      int column = 0;
      for (int i = 0; i < unitTwistList.size(); i++)
      {
         Twist twist = unitTwistList.get(i);
         tempTwist.set(twist);
         tempTwist.changeFrame(jacobianFrame);
         tempTwist.getMatrix(tempMatrix, 0);
         CommonOps.extract(tempMatrix, 0, tempMatrix.getNumRows(), 0, tempMatrix.getNumCols(), jacobian, 0, column++);
      }
   }

   public void changeFrame(ReferenceFrame jacobianFrame)
   {
      if (!allowChangeFrame)
         throw new RuntimeException("Cannot change the frame of this Jacobian.");

      this.jacobianFrame = jacobianFrame;
   }

   /**
    * packs the twist that corresponds to the given joint velocity vector
    * @param jointVelocities the joint velocity vector;
    *       should have the same ordering as the unitRelativeTwistsInBodyFrame that were passed in during construction,
    *       should be a column vector
    * @return the twist of the end effector with respect to the base, expressed in the jacobianFrame
    *
    * angular velocity part of the twist is the angular velocity of end effector frame, with respect to base frame, rotated to end effector frame
    * linear velocity part of the twist is the linear velocity of end effector frame, with respect to base frame, rotated to end effector frame
    */
   public void getTwist(DenseMatrix64F jointVelocities, Twist twistToPack)
   {
      CommonOps.mult(jacobian, jointVelocities, tempMatrix);
      twistToPack.set(getEndEffectorFrame(), getBaseFrame(), jacobianFrame, tempMatrix, 0);
   }

   /**
    * returns the joint torque vector that corresponds to the given wrench
    * @param wrench the resulting wrench at the end effector.
    *       The wrench should be expressed in this Jacobian's jacobianFrame and its 'onWhat' frame should be this Jacobian's endEffectorFrame
    * @return joint torques that result in the given wrench as a column vector
    */
   public DenseMatrix64F computeJointTorques(Wrench wrench)
   {
      DenseMatrix64F jointTorques = new DenseMatrix64F(1, jacobian.getNumCols());
      computeJointTorques(wrench, jointTorques);
      return jointTorques;
   }

   /**
    * Computes and packs the joint torque vector that corresponds to the given wrench
    * @param wrench the resulting wrench at the end effector.
    *       The wrench should be expressed in this Jacobian's jacobianFrame and its 'onWhat' frame should be this Jacobian's endEffectorFrame
    */
   public void computeJointTorques(Wrench wrench, DenseMatrix64F jointTorquesToPack)
   {
      // reference frame check
      wrench.getExpressedInFrame().checkReferenceFrameMatch(this.jacobianFrame);
      wrench.getMatrix(tempMatrix);
      jointTorquesToPack.reshape(1, jacobian.getNumCols());
      CommonOps.multTransA(tempMatrix, jacobian, jointTorquesToPack);
      CommonOps.transpose(jointTorquesToPack);
   }

   /**
    * Returns a reference to the underlying Matrix object. Does not recompute the Jacobian.
    * @return a reference to the underlying Matrix object
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

   /** @return a {@code Array} of joints describing a continuous path from the base to the end effector of this Jacobian. */
   public InverseDynamicsJoint[] getJointPathFromBaseToEndEffector()
   {
      return jointPathFromBaseToEndEffector;
   }

   public RigidBody getBase()
   {
      return joints[0].getPredecessor();
   }

   public RigidBody getEndEffector()
   {
      return joints[joints.length - 1].getSuccessor();
   }

   public ReferenceFrame getBaseFrame()
   {
      return getBase().getBodyFixedFrame();
   }

   public ReferenceFrame getEndEffectorFrame()
   {
      return getEndEffector().getBodyFixedFrame();
   }

   // default access for efficiency
   List<Twist> getAllUnitTwists()
   {
      return unitTwistList;
   }

   private static LinkedHashMap<InverseDynamicsJoint, List<Twist>> extractTwistsFromJoints(InverseDynamicsJoint[] joints)
   {
      checkJointOrder(joints);
      
      LinkedHashMap<InverseDynamicsJoint, List<Twist>> ret = new LinkedHashMap<InverseDynamicsJoint, List<Twist>>();
      for(int i=0;i<joints.length;i++)
      {
         InverseDynamicsJoint joint = joints[i];
         ret.put(joint, joint.getMotionSubspace().unitTwistMap.get(joint));
      }

      return ret;
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

   private List<Twist> extractUnitTwistList(LinkedHashMap<InverseDynamicsJoint, List<Twist>> unitTwistMap)
   {
      int size = 0;
      for (List<Twist> twistList : unitTwistMap.values())
      {
         size += twistList.size();
      }

      List<Twist> ret = new ArrayList<Twist>(size);
      for (List<Twist> twistList : unitTwistMap.values())
      {
         ret.addAll(twistList);
      }

      return ret;
   }

   private static LinkedHashMap<InverseDynamicsJoint, List<Twist>> createUnitTwistMap(InverseDynamicsJoint joint, List<Twist> unitTwists)
   {
      LinkedHashMap<InverseDynamicsJoint, List<Twist>> ret = new LinkedHashMap<InverseDynamicsJoint, List<Twist>>(1);
      ret.put(joint, unitTwists);

      return ret;
   }

   private static DenseMatrix64F createJacobianMatrix(LinkedHashMap<InverseDynamicsJoint, List<Twist>> unitTwists)
   {
      return new DenseMatrix64F(SpatialMotionVector.SIZE, ScrewTools.computeDegreesOfFreedom(unitTwists.keySet()));
   }

   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      builder.append("Jacobian. jacobianFrame = " + jacobianFrame + ". Joints:\n");

      for (InverseDynamicsJoint joint : unitTwistMap.keySet())
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
   public long nameBasedHashCode()
   {
      return nameBasedHashCode;
   }
}
