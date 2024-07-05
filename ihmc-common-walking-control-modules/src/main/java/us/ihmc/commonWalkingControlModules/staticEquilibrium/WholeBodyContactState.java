package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TObjectIntHashMap;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.lists.SupplierBuilder;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.GravityCoriolisExternalWrenchMatrixCalculator;

import java.util.HashMap;
import java.util.Map;

import static us.ihmc.commonWalkingControlModules.staticEquilibrium.CenterOfMassStabilityMarginOptimizationModule.GRAVITY;
import static us.ihmc.commonWalkingControlModules.staticEquilibrium.CenterOfMassStabilityMarginOptimizationModule.LINEAR_DIMENSIONS;

public class WholeBodyContactState implements WholeBodyContactStateInterface
{
   private final RecyclingArrayList<ContactPoint> contactPoints = new RecyclingArrayList<>(20, SupplierBuilder.indexedSupplier(ContactPoint::new));
   private final Map<RigidBodyBasics, GeometricJacobian> contactJacobians = new HashMap<>();

   private final OneDoFJointBasics[] oneDoFJoints;
   private final GravityCoriolisExternalWrenchMatrixCalculator gravityCoriolisExternalWrenchMatrixCalculator;

   /* Jacobian-transpose actuation constraint corresponding to joint torque lower effort limits: - J^T f >= tau_lower - g */
   private final DMatrixRMaj constraintLowerBound;
   /* Jacobian-transpose actuation constraint corresponding to joint torque upper effort limits: - J^T f <= tau_upper - g */
   private final DMatrixRMaj constraintUpperBound;

   /* Stacked Jacobian-transpose actuation constraints matrices: J_stacked = (J^T, -J^T), so that J_stacked f <= b_stacked */
   private final DMatrixRMaj stackedConstraintMatrix;
   /* Stacked Jacobian-transpose actuation constraints vectors: b_stacked = (-tau_lower + g, tau_upper - g), so that J_stacked f <= b_stacked */
   private final DMatrixRMaj stackedConstraintVector;

   /* 3 x n_j point-jacobian */
   private final DMatrixRMaj contactJacobian;
   /* n_j x 3 point-jacobian transpose */
   private final DMatrixRMaj contactJacobianTranspose;
   /* n_j x 3n_c stacked point-jacobian transpose */
   private final DMatrixRMaj graspMatrixJacobianTranspose;
   /* 2n_j, the number of actuation constraints for lower and upper joint torque bounds */
   private final int numberOfActuationConstraints;

   /* Pose at which the Geometric Jacobian is evaluated */
   private final FramePose3D worldAlignedContactPose = new FramePose3D();
   /* Reference frame corresponding to worldAlignedContactPose */
   private final PoseReferenceFrame worldAlignedContactFrame = new PoseReferenceFrame("worldAlignedContactFrame", ReferenceFrame.getWorldFrame());
   /* Map from index to joint, indexed based on oneDoFJoints */
   private final TObjectIntMap<OneDoFJointBasics> jointIndexMap = new TObjectIntHashMap<>();
   /* Placeholder to efficiently set contact points from a PlaneContactState object */
   private final PlaneContactStateCommand tempPlaneContactStateCommand = new PlaneContactStateCommand();

   public WholeBodyContactState(OneDoFJointBasics[] oneDoFJoints, JointBasics rootJoint)
   {
      this.oneDoFJoints = oneDoFJoints;

      RigidBodyBasics elevator = MultiBodySystemTools.getRootBody(oneDoFJoints[0].getPredecessor());

      this.gravityCoriolisExternalWrenchMatrixCalculator = new GravityCoriolisExternalWrenchMatrixCalculator(elevator, false);
      gravityCoriolisExternalWrenchMatrixCalculator.setGravitionalAcceleration(-GRAVITY);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         RigidBodyBasics rigidBody = oneDoFJoints[i].getSuccessor();
         contactJacobians.put(rigidBody, new GeometricJacobian(elevator, rigidBody, rigidBody.getBodyFixedFrame()));
      }

      RigidBodyBasics pelvis = rootJoint.getSuccessor();
      contactJacobians.put(pelvis, new GeometricJacobian(elevator, pelvis, pelvis.getBodyFixedFrame()));

      constraintLowerBound = new DMatrixRMaj(oneDoFJoints.length, 1);
      constraintUpperBound = new DMatrixRMaj(oneDoFJoints.length, 1);

      numberOfActuationConstraints = 2 * oneDoFJoints.length;
      stackedConstraintMatrix = new DMatrixRMaj(0);
      stackedConstraintVector = new DMatrixRMaj(numberOfActuationConstraints, 1);

      contactJacobian = new DMatrixRMaj(LINEAR_DIMENSIONS, oneDoFJoints.length);
      contactJacobianTranspose = new DMatrixRMaj(oneDoFJoints.length, LINEAR_DIMENSIONS);
      graspMatrixJacobianTranspose = new DMatrixRMaj(0);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         jointIndexMap.put(oneDoFJoints[i], i);
      }
   }

   public void clear()
   {
      contactPoints.clear();

      stackedConstraintMatrix.zero();
      stackedConstraintVector.zero();
      contactJacobian.zero();
      contactJacobianTranspose.zero();
      graspMatrixJacobianTranspose.zero();
   }

   public void copyAndIgnoreIndex(int indexToIgnore, WholeBodyContactState other)
   {
      clear();

      for (int contactPointIndex = 0; contactPointIndex < other.getNumberOfContactPoints(); contactPointIndex++)
      {
         if (contactPointIndex == indexToIgnore)
            continue;
         contactPoints.add().set(other.contactPoints.get(contactPointIndex));
      }
   }

   public void addContactPoints(PlaneContactState planeContactState)
   {
      if (!planeContactState.inContact())
         return;

      planeContactState.getPlaneContactStateCommand(tempPlaneContactStateCommand);
      addContactPoints(tempPlaneContactStateCommand);
   }

   public void addContactPoints(PlaneContactStateCommand contactStateCommand)
   {
      RigidBodyBasics contactingRigidBody = contactStateCommand.getContactingRigidBody();
      FrameVector3DBasics contactNormal = contactStateCommand.getContactNormal();

      for (int contactIdx = 0; contactIdx < contactStateCommand.getNumberOfContactPoints(); contactIdx++)
      {
         FramePoint3DBasics contactPoint = contactStateCommand.getContactPoint(contactIdx);
         double coefficientOfFriction = contactStateCommand.getCoefficientOfFriction();
         addContactPoint(contactingRigidBody, contactPoint, contactNormal, coefficientOfFriction);
      }
   }

   public void addContactPoint(RigidBodyBasics contactingBody, FrameTuple3DReadOnly contactPoint, FrameVector3DReadOnly surfaceNormal, double coefficientOfFriction)
   {
      contactPoints.add().set(contactingBody, contactPoint, surfaceNormal, coefficientOfFriction);
   }

   public void update()
   {
      gravityCoriolisExternalWrenchMatrixCalculator.compute();

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         double gravityTorque = gravityCoriolisExternalWrenchMatrixCalculator.getComputedJointTau(oneDoFJoints[i]).get(0, 0);
         constraintLowerBound.set(i, oneDoFJoints[i].getEffortLimitLower() - gravityTorque);
         constraintUpperBound.set(i, oneDoFJoints[i].getEffortLimitUpper() - gravityTorque);
      }

      int nContactForceVariables = LINEAR_DIMENSIONS * contactPoints.size();
      graspMatrixJacobianTranspose.reshape(oneDoFJoints.length, nContactForceVariables);

      for (int contactPointIndex = 0; contactPointIndex < contactPoints.size(); contactPointIndex++)
      {
         ContactPoint contactPoint = contactPoints.get(contactPointIndex);
         contactPoint.update();

         worldAlignedContactPose.setToZero(contactPoint.contactFrame);
         worldAlignedContactPose.changeFrame(ReferenceFrame.getWorldFrame());
         worldAlignedContactPose.getOrientation().setToZero();
         worldAlignedContactFrame.setPoseAndUpdate(worldAlignedContactPose);

         contactJacobians.get(contactPoint.contactingBody).changeFrame(worldAlignedContactFrame);
         contactJacobians.get(contactPoint.contactingBody).compute();

         DMatrixRMaj jacobianMatrix = contactJacobians.get(contactPoint.contactingBody).getJacobianMatrix();
         JointBasics[] contactJacobianJoints = contactJacobians.get(contactPoint.contactingBody).getJointsInOrder();
         contactJacobian.zero();

         for (int jacobianJointIndex = 1; jacobianJointIndex < contactJacobianJoints.length; jacobianJointIndex++)
         {
            OneDoFJointBasics joint = (OneDoFJointBasics) contactJacobianJoints[jacobianJointIndex];
            int jointIndex = jointIndexMap.get(joint);

            for (int linearCoordIndex = 0; linearCoordIndex < LINEAR_DIMENSIONS; linearCoordIndex++)
            {
               // We're computing just the linear component, ignore angular block
               int rowOffset = LINEAR_DIMENSIONS;

               // Offset to ignore root joint indices
               int colOffset = 5;

               double jacobianEntry = jacobianMatrix.get(rowOffset + linearCoordIndex, colOffset + jacobianJointIndex);

               contactJacobian.set(linearCoordIndex, jointIndex, jacobianEntry);
            }
         }

         CommonOps_DDRM.transpose(contactJacobian, contactJacobianTranspose);
         MatrixTools.setMatrixBlock(graspMatrixJacobianTranspose, 0, LINEAR_DIMENSIONS * contactPointIndex, contactJacobianTranspose, 0, 0, contactJacobianTranspose.getNumRows(), contactJacobianTranspose.getNumCols(), 1.0);
      }

      stackedConstraintMatrix.reshape(numberOfActuationConstraints, nContactForceVariables);

      int rowOffsetLowerBound = 0;
      int rowOffsetUpperBound = oneDoFJoints.length;

      /* Lower torque bound constraint: J^T f <= - tau_lower + g */
      MatrixTools.setMatrixBlock(stackedConstraintMatrix, rowOffsetLowerBound, 0, graspMatrixJacobianTranspose, 0, 0, graspMatrixJacobianTranspose.getNumRows(), graspMatrixJacobianTranspose.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(stackedConstraintVector, rowOffsetLowerBound, 0, constraintLowerBound, 0, 0, constraintLowerBound.getNumRows(), constraintLowerBound.getNumCols(), -1.0);

      /* Upper torque bound constraint: -J^T f <= tau_upper - g */
      MatrixTools.setMatrixBlock(stackedConstraintMatrix, rowOffsetUpperBound, 0, graspMatrixJacobianTranspose, 0, 0, graspMatrixJacobianTranspose.getNumRows(), graspMatrixJacobianTranspose.getNumCols(), -1.0);
      MatrixTools.setMatrixBlock(stackedConstraintVector, rowOffsetUpperBound, 0, constraintUpperBound, 0, 0, constraintUpperBound.getNumRows(), constraintUpperBound.getNumCols(), 1.0);
   }

   private static class ContactPoint
   {
      private RigidBodyBasics contactingBody;
      private final FramePose3D contactPose = new FramePose3D();
      private final PoseReferenceFrame contactFrame;
      private double coefficientOfFriction;

      ContactPoint(int index)
      {
         contactFrame = new PoseReferenceFrame("contactFrame" + index, ReferenceFrame.getWorldFrame());
         clear();
      }

      void clear()
      {
         contactingBody = null;
         contactPose.setToNaN();
         coefficientOfFriction = Double.NaN;
      }

      void set(RigidBodyBasics contactingBody, FrameTuple3DReadOnly contactPoint, FrameVector3DReadOnly surfaceNormal, double coefficientOfFriction)
      {
         this.contactingBody = contactingBody;
         this.coefficientOfFriction = coefficientOfFriction;

         // Set orientation in given frame
         contactPose.setReferenceFrame(surfaceNormal.getReferenceFrame());
         EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.Z, surfaceNormal, contactPose.getOrientation());

         // Set position
         contactPose.getPosition().setMatchingFrame(contactPoint);
      }

      void set(ContactPoint other)
      {
         this.contactingBody = other.contactingBody;
         this.contactPose.set(other.contactPose);
      }

      void update()
      {
         contactPose.changeFrame(ReferenceFrame.getWorldFrame());
         contactFrame.setPoseAndUpdate(contactPose);
      }
   }

   @Override
   public int getNumberOfContactPoints()
   {
      return contactPoints.size();
   }

   @Override
   public double getCoefficientOfFriction(int contactPointIndex)
   {
      return contactPoints.get(contactPointIndex).coefficientOfFriction;
   }

   public DMatrixRMaj getConstraintLowerBound()
   {
      return constraintLowerBound;
   }

   public DMatrixRMaj getConstraintUpperBound()
   {
      return constraintUpperBound;
   }

   public DMatrixRMaj getGraspMatrixJacobianTranspose()
   {
      return graspMatrixJacobianTranspose;
   }

   public int getNumberOfJoints()
   {
      return oneDoFJoints.length;
   }

   public ReferenceFrame getContactFrame(int contactPointIndex)
   {
      return contactPoints.get(contactPointIndex).contactFrame;
   }

   @Override
   public DMatrixRMaj getActuationConstraintMatrix()
   {
      return stackedConstraintMatrix;
   }

   @Override
   public DMatrixRMaj getActuationConstraintVector()
   {
      return stackedConstraintVector;
   }

   public GravityCoriolisExternalWrenchMatrixCalculator getGravityCalculator()
   {
      return gravityCoriolisExternalWrenchMatrixCalculator;
   }

   public OneDoFJointBasics[] getOneDoFJoints()
   {
      return oneDoFJoints;
   }

   public int getJointIndex(OneDoFJointBasics joint)
   {
      return jointIndexMap.get(joint);
   }

   public JointBasics[] getKinematicChain(int contactIndex)
   {
      return contactJacobians.get(contactPoints.get(contactIndex).contactingBody).getJointsInOrder();
   }

   public GeometricJacobian updateContactJacobian(int contactIndex)
   {
      ContactPoint contactPoint = contactPoints.get(contactIndex);
      RigidBodyBasics contactingBody = contactPoint.contactingBody;

      worldAlignedContactPose.setToZero(contactPoint.contactFrame);
      worldAlignedContactPose.changeFrame(ReferenceFrame.getWorldFrame());
      worldAlignedContactPose.getOrientation().setToZero();
      worldAlignedContactFrame.setPoseAndUpdate(worldAlignedContactPose);

      GeometricJacobian jacobian = contactJacobians.get(contactingBody);
      jacobian.changeFrame(worldAlignedContactFrame);
      jacobian.compute();

      return jacobian;
   }

   /**
    * Given constraintActiveSetIndex, which represents the constraint row oNf A in the problem constraint Ax <= b,
    * this returns true if that row corresponds to a joint actuation constraint or false otherwise when the row corresponds to a
    * static equilibrium constraint.
    */
   public boolean isJointTorqueActuationConstraint(int constraintActiveSetIndex)
   {
      int staticEquilibriumInequalityConstraints = 2 * CenterOfMassStabilityMarginOptimizationModule.STATIC_EQUILIBRIUM_CONSTRAINTS;
      return constraintActiveSetIndex >= staticEquilibriumInequalityConstraints;
   }

   /**
    * Returns the joint corresponding to the constraint row actuationConstraintIndex of A in the problem constraint Ax <= b.
    */
   public OneDoFJointBasics getJointFromActuationConstraintIndex(int actuationConstraintIndex)
   {
      int staticEquilibriumInequalityConstraints = 2 * CenterOfMassStabilityMarginOptimizationModule.STATIC_EQUILIBRIUM_CONSTRAINTS;
      if (actuationConstraintIndex < staticEquilibriumInequalityConstraints)
      {
         throw new RuntimeException("Invalid constraint index");
      }

      return oneDoFJoints[(actuationConstraintIndex - staticEquilibriumInequalityConstraints) % oneDoFJoints.length];
   }

   /**
    * Assuming the given index actuationConstraintIndex corresponds to a joint torque bound, returns true
    * if the constraint corresponds to the joint's upper torque bound and false otherwise
    */
   public boolean isActuationConstraintUpperBound(int actuationConstraintIndex)
   {
      int staticEquilibriumInequalityConstraints = 2 * CenterOfMassStabilityMarginOptimizationModule.STATIC_EQUILIBRIUM_CONSTRAINTS;
      if (actuationConstraintIndex < staticEquilibriumInequalityConstraints)
      {
         throw new RuntimeException("Invalid constraint index");
      }

      return actuationConstraintIndex - staticEquilibriumInequalityConstraints >= oneDoFJoints.length;
   }
}

