package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TObjectIntHashMap;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.lists.SupplierBuilder;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialVector;
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

   private final DMatrixRMaj torqueLowerBound;
   private final DMatrixRMaj torqueUpperBound;

   private final DMatrixRMaj constraintLowerBound;
   private final DMatrixRMaj constraintUpperBound;

   private final DMatrixRMaj stackedConstraintMatrix;
   private final DMatrixRMaj stackedConstraintVector;

   private final DMatrixRMaj contactJacobian;
   private final DMatrixRMaj contactJacobianTranspose;
   private final DMatrixRMaj graspMatrixJacobianTranspose;

   private final FramePose3D worldAlignedContactPose = new FramePose3D();
   private final PoseReferenceFrame worldAlignedContactFrame = new PoseReferenceFrame("worldAlignedContactFrame", ReferenceFrame.getWorldFrame());
   private final TObjectIntMap<OneDoFJointBasics> jointIndexMap = new TObjectIntHashMap<>();

   public WholeBodyContactState(OneDoFJointBasics[] oneDoFJoints)
   {
      this.oneDoFJoints = oneDoFJoints;

      RigidBodyBasics elevator = MultiBodySystemTools.getRootBody(oneDoFJoints[0].getPredecessor());
      this.gravityCoriolisExternalWrenchMatrixCalculator = new GravityCoriolisExternalWrenchMatrixCalculator(elevator, false);
      gravityCoriolisExternalWrenchMatrixCalculator.setGravitionalAcceleration(-GRAVITY);

      RigidBodyBasics[] rigidBodies = MultiBodySystemTools.collectSuccessors(elevator.getParentJoint());
      for (int i = 0; i < rigidBodies.length; i++)
      {
         contactJacobians.put(rigidBodies[i], new GeometricJacobian(elevator, rigidBodies[i], rigidBodies[i].getBodyFixedFrame()));
      }

      torqueLowerBound = new DMatrixRMaj(oneDoFJoints.length, 1);
      torqueUpperBound = new DMatrixRMaj(oneDoFJoints.length, 1);

      constraintLowerBound = new DMatrixRMaj(oneDoFJoints.length, 1);
      constraintUpperBound = new DMatrixRMaj(oneDoFJoints.length, 1);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         torqueLowerBound.set(i, oneDoFJoints[i].getEffortLimitLower());
         torqueUpperBound.set(i, oneDoFJoints[i].getEffortLimitUpper());
      }

      stackedConstraintMatrix = new DMatrixRMaj(0);
      stackedConstraintVector = new DMatrixRMaj(oneDoFJoints.length, 1);

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

      torqueLowerBound.zero();
      torqueUpperBound.zero();
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

   public void addContactPoint(RigidBodyBasics contactingBody, FramePoint3D contactPoint, FrameVector3D surfaceNormal)
   {
      contactPoints.add().set(contactingBody, contactPoint, surfaceNormal);
   }

   public void update()
   {
      gravityCoriolisExternalWrenchMatrixCalculator.compute();

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         double gravityTorque = gravityCoriolisExternalWrenchMatrixCalculator.getComputedJointTau(oneDoFJoints[i]).get(0, 0);
         constraintLowerBound.set(i, gravityTorque - torqueUpperBound.get(i, 0));
         constraintUpperBound.set(i, gravityTorque - torqueLowerBound.get(i, 0));
      }

      int nContactForceVariables = 3 * contactPoints.size();
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

               // Offset to ignore base joint indices
               int colOffset = 5;

               double jacobianEntry = jacobianMatrix.get(rowOffset + linearCoordIndex, colOffset + jacobianJointIndex);

               contactJacobian.set(linearCoordIndex, jointIndex, jacobianEntry);
            }
         }

         CommonOps_DDRM.transpose(contactJacobian, contactJacobianTranspose);
         MatrixTools.setMatrixBlock(graspMatrixJacobianTranspose, 0, LINEAR_DIMENSIONS * contactPointIndex, contactJacobianTranspose, 0, 0, contactJacobianTranspose.getNumRows(), contactJacobianTranspose.getNumCols(), 1.0);
      }

      stackedConstraintMatrix.reshape(2 * oneDoFJoints.length, nContactForceVariables);

      MatrixTools.setMatrixBlock(stackedConstraintMatrix, 0, 0, graspMatrixJacobianTranspose, 0, 0, graspMatrixJacobianTranspose.getNumRows(), graspMatrixJacobianTranspose.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(stackedConstraintMatrix, graspMatrixJacobianTranspose.getNumRows(), 0, graspMatrixJacobianTranspose, 0, 0, graspMatrixJacobianTranspose.getNumRows(), graspMatrixJacobianTranspose.getNumCols(), -1.0);
      MatrixTools.setMatrixBlock(stackedConstraintVector, 0, 0, constraintUpperBound, 0, 0, constraintUpperBound.getNumRows(), constraintUpperBound.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(stackedConstraintVector, constraintUpperBound.getNumRows(), 0, constraintLowerBound, 0, 0, constraintLowerBound.getNumRows(), constraintLowerBound.getNumCols(), -1.0);
   }

   private static class ContactPoint
   {
      private RigidBodyBasics contactingBody;
      private final FramePose3D contactPose = new FramePose3D();
      private final PoseReferenceFrame contactFrame;

      ContactPoint(int index)
      {
         contactFrame = new PoseReferenceFrame("contactFrame" + index, ReferenceFrame.getWorldFrame());
      }

      void clear()
      {
         contactingBody = null;
         contactPose.setToNaN();
      }

      void set(RigidBodyBasics contactingBody, FramePoint3D contactPoint, FrameVector3D surfaceNormal)
      {
         this.contactingBody = contactingBody;
         contactPoint.changeFrame(contactingBody.getBodyFixedFrame());
         surfaceNormal.changeFrame(contactingBody.getBodyFixedFrame());
         contactPose.setReferenceFrame(contactingBody.getBodyFixedFrame());
         contactPose.getPosition().set(contactPoint);
         EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.Z, surfaceNormal, contactPose.getOrientation());
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
         contactPose.changeFrame(contactingBody.getBodyFixedFrame());
      }
   }

   @Override
   public int getNumberOfContactPoints()
   {
      return contactPoints.size();
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
}

