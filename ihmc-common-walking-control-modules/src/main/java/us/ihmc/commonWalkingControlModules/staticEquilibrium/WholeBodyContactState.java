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
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.GravityCoriolisExternalWrenchMatrixCalculator;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class WholeBodyContactState implements WholeBodyContactStateInterface
{
   private static final double GRAVITY = 9.81;
   private final RecyclingArrayList<ContactPoint> contactPoints = new RecyclingArrayList<>(20, SupplierBuilder.indexedSupplier(ContactPoint::new));
   private final Map<RigidBodyBasics, GeometricJacobian> contactJacobians = new HashMap<>();

   private final OneDoFJointBasics[] oneDoFJoints;
   private final GravityCoriolisExternalWrenchMatrixCalculator gravityCoriolisExternalWrenchMatrixCalculator;

   private final DMatrixRMaj torqueLowerBound;
   private final DMatrixRMaj torqueUpperBound;
   private final DMatrixRMaj constraintLowerBound;
   private final DMatrixRMaj constraintUpperBound;

   private final DMatrixRMaj contactJacobian;
   private final DMatrixRMaj contactJacobianTranspose;
   private final DMatrixRMaj basisVectorMatrix;
   private final DMatrixRMaj contactJacobianTransposeTimesBasisVector;
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

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         torqueLowerBound.set(i, oneDoFJoints[i].getEffortLimitLower());
         torqueUpperBound.set(i, oneDoFJoints[i].getEffortLimitUpper());
      }

      constraintLowerBound = new DMatrixRMaj(oneDoFJoints.length, 1);
      constraintUpperBound = new DMatrixRMaj(oneDoFJoints.length, 1);

      contactJacobian = new DMatrixRMaj(3, oneDoFJoints.length);
      contactJacobianTranspose = new DMatrixRMaj(oneDoFJoints.length, 3);
      graspMatrixJacobianTranspose = new DMatrixRMaj(0);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         jointIndexMap.put(oneDoFJoints[i], i);
      }
   }

   public void clear()
   {
      contactFrames.clear();
      contactingBodies.clear();
      contactJacobians.clear();
      graspMatrixJacobianTranspose.zero();
   }

   public void copyAndIgnoreIndex(int indexToIgnore, WholeBodyContactState other)
   {
      clear();

      for (int contactPointIndex = 0; contactPointIndex < other.getNumberOfContactPoints(); contactPointIndex++)
      {
         if (contactPointIndex == indexToIgnore)
            continue;
         addContactPoint(other.contactFrames.get(contactPointIndex), other.contactingBodies.get(contactPointIndex), other.contactJacobians.get(contactPointIndex));
      }
   }

   public void addContactPoint(RigidBodyBasics contactingBody, FramePoint3D contactPoint, FrameVector3D surfaceNormal)
   {
      contactPoint.changeFrame(contactingBody.getBodyFixedFrame());

      this.contactFrames.add(contactFrame);
      this.contactingBodies.add(contactingBody);
      this.contactJacobians.add(contactJacobian);
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

      graspMatrixJacobianTranspose.reshape(oneDoFJoints.length, numberOfBasisVectors * contactingBodies.size());

      for (int contactPointIndex = 0; contactPointIndex < contactingBodies.size(); contactPointIndex++)
      {
         ReferenceFrame contactFrame = contactFrames.get(contactPointIndex);

         worldAlignedContactPose.setToZero(contactFrame);
         worldAlignedContactPose.changeFrame(ReferenceFrame.getWorldFrame());
         worldAlignedContactPose.getOrientation().setToZero();
         worldAlignedContactFrame.setPoseAndUpdate(worldAlignedContactPose);

         contactJacobians.get(contactPointIndex).changeFrame(worldAlignedContactFrame);
         contactJacobians.get(contactPointIndex).compute();

         DMatrixRMaj jacobianMatrix = contactJacobians.get(contactPointIndex).getJacobianMatrix();
         JointBasics[] contactJacobianJoints = contactJacobians.get(contactPointIndex).getJointsInOrder();
         contactJacobian.zero();

         for (int jacobianJointIndex = 1; jacobianJointIndex < contactJacobianJoints.length; jacobianJointIndex++)
         {
            OneDoFJointBasics joint = (OneDoFJointBasics) contactJacobianJoints[jacobianJointIndex];
            int jointIndex = jointIndexMap.get(joint);

            for (int linearCoordIndex = 0; linearCoordIndex < 3; linearCoordIndex++)
            {
               // We're computing just the linear component, ignore angular block. We're also ignoring root joint acceleration
               double jacobianEntry = jacobianMatrix.get(3 + linearCoordIndex, 5 + jacobianJointIndex);

               contactJacobian.set(linearCoordIndex, jointIndex, jacobianEntry);
            }
         }

         CommonOps_DDRM.transpose(contactJacobian, contactJacobianTranspose);

         FrameVector3D contactNormal = new FrameVector3D(contactFrame, Axis3D.Z);
         for (int basisVectorIndex = 0; basisVectorIndex < numberOfBasisVectors; basisVectorIndex++)
         {
            double angleToRotateAroundInXY = basisVectorIndex * 2.0 * Math.PI / numberOfBasisVectors;
            AxisAngle basisVectorRotation = new AxisAngle(Math.cos(angleToRotateAroundInXY), Math.sin(angleToRotateAroundInXY), 0.0, basisVectorAngleFromNormal);

            FrameVector3D basisVector = new FrameVector3D(contactNormal);
            basisVectorRotation.transform(basisVector);
            basisVector.changeFrame(ReferenceFrame.getWorldFrame());
            basisVector.get(basisVectorMatrix);
            basisVectors.add(basisVector);

            CommonOps_DDRM.mult(contactJacobianTranspose, basisVectorMatrix, contactJacobianTransposeTimesBasisVector);
            MatrixTools.setMatrixBlock(graspMatrixJacobianTranspose, 0, numberOfBasisVectors * contactPointIndex + basisVectorIndex, contactJacobianTransposeTimesBasisVector, 0, 0, contactJacobianTransposeTimesBasisVector.getNumRows(), 1, 1.0);
         }
      }
   }

   private class ContactPoint
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
      return contactFrames.size();
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
      return contactFrames.get(contactPointIndex);
   }

   public GravityCoriolisExternalWrenchMatrixCalculator getGravityCalculator()
   {
      return gravityCoriolisExternalWrenchMatrixCalculator;
   }
}

