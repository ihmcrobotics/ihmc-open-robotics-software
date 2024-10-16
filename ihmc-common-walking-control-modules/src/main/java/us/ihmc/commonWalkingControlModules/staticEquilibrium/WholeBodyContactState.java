package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TObjectIntHashMap;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.lists.SupplierBuilder;
import us.ihmc.convexOptimization.linearProgram.LinearProgramSolver;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.commons.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.mecano.GravityCoriolisExternalWrenchMatrixCalculator;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Predicate;

import static us.ihmc.commonWalkingControlModules.staticEquilibrium.CenterOfMassStabilityMarginOptimizationModule.GRAVITY;
import static us.ihmc.commonWalkingControlModules.staticEquilibrium.CenterOfMassStabilityMarginOptimizationModule.LINEAR_DIMENSIONS;

public class WholeBodyContactState implements WholeBodyContactStateInterface
{
   private static final boolean INCLUDE_GRAVITY_CORIOLIS_TORQUES = true;

   private final RecyclingArrayList<ContactPoint> contactPoints = new RecyclingArrayList<>(20, SupplierBuilder.indexedSupplier(ContactPoint::new));
   private final Map<RigidBodyBasics, GeometricJacobian> contactJacobians = new HashMap<>();

   private final OneDoFJointBasics[] oneDoFJoints;
   private final GravityCoriolisExternalWrenchMatrixCalculator gravityCoriolisExternalWrenchMatrixCalculator;
   private final GeometricJacobianCalculator geometricJacobianCalculator = new GeometricJacobianCalculator();

   /* Jacobian-transpose actuation constraint corresponding to joint torque lower effort limits: - J^T f >= tau_lower - g */
   private final DMatrixRMaj constraintLowerBound;
   /* Jacobian-transpose actuation constraint corresponding to joint torque upper effort limits: - J^T f <= tau_upper - g */
   private final DMatrixRMaj constraintUpperBound;

   /* Stacked Jacobian-transpose actuation constraints matrices: J_stacked = (J^T, -J^T), so that J_stacked f <= b_stacked */
   private final DMatrixRMaj stackedConstraintMatrix = new DMatrixRMaj(0);
   /* Stacked Jacobian-transpose actuation constraints vectors: b_stacked = (-tau_lower + g, tau_upper - g), so that J_stacked f <= b_stacked */
   private final DMatrixRMaj stackedConstraintVector = new DMatrixRMaj(0);
   /* Gradient of stacked Jacobian-transpose actuation constraints matrices: J_stacked = (J^T, -J^T), so that J_stacked f <= b_stacked */
   private final DMatrixRMaj stackedConstraintMatrixGradient = new DMatrixRMaj(0);

   /* 3 x n_j point-jacobian */
   private final DMatrixRMaj contactJacobian;
   /* n_j x 3 point-jacobian transpose */
   private final DMatrixRMaj contactJacobianTranspose;
   /* n_j x 3n_c stacked point-jacobian transpose */
   private final DMatrixRMaj graspMatrixJacobianTranspose;

   /* Pose at which the Geometric Jacobian is evaluated */
   private final FramePose3D worldAlignedContactPose = new FramePose3D();
   /* Reference frame corresponding to worldAlignedContactPose */
   private final PoseReferenceFrame worldAlignedContactFrame = new PoseReferenceFrame("worldAlignedContactFrame", ReferenceFrame.getWorldFrame());
   /* Map from index to joint, indexed based on oneDoFJoints */
   private final TObjectIntMap<OneDoFJointBasics> jointIndexMap = new TObjectIntHashMap<>();
   /* Placeholder to efficiently set contact points from a PlaneContactState object */
   private final PlaneContactStateCommand tempPlaneContactStateCommand = new PlaneContactStateCommand();
   /* Map from joint to index in the reduced set of joints that are being validated for torque feasibility */
   private final TObjectIntMap<OneDoFJointBasics> jointTorqueIndexMap = new TObjectIntHashMap<>();
   /* Mask for which joints to check torque feasibility, which is static and defined in WalkingControllerParameters.getJointsToCheckTorqueFeasibilityInMultiContact */
   private final boolean[] jointTorqueStaticMask;
   /* Mask for which joints are loaded at a given tick */
   private final boolean[] jointLoadStatusMask;
   /* Number of joints being validated for torque feasibility */
   private int numberOfTorqueValidatedJoints;
   /* Array of joints being validated for torque feasibility */
   private final List<OneDoFJointBasics> torqueValidatedJoints = new ArrayList<>();

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

      contactJacobian = new DMatrixRMaj(0);
      contactJacobianTranspose = new DMatrixRMaj(0);
      graspMatrixJacobianTranspose = new DMatrixRMaj(0);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         jointIndexMap.put(oneDoFJoints[i], i);
      }

      jointTorqueStaticMask = new boolean[oneDoFJoints.length];
      jointLoadStatusMask = new boolean[oneDoFJoints.length];

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         jointIndexMap.put(oneDoFJoints[i], i);
      }

      setupForSelectedJoints(name -> true);
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
      updateContactPoints();
      updateJointIndices();
      updateActuationConstraintVector();
      updateActuationConstraintMatrix(true);
   }

   public void updateContactPoints()
   {
      for (int i = 0; i < contactPoints.size(); i++)
      {
         contactPoints.get(i).update();
      }
   }

   public void updateJointIndices()
   {
      for (int i = 0; i < contactPoints.size(); i++)
      {
         ContactPoint contactPoint = contactPoints.get(i);
         JointBasics[] contactJacobianJoints = contactJacobians.get(contactPoint.contactingBody).getJointsInOrder();
         for (int j = 1; j < contactJacobianJoints.length; j++)
         {
            jointLoadStatusMask[jointIndexMap.get(contactJacobianJoints[j])] = true;
         }
      }

      numberOfTorqueValidatedJoints = 0;
      torqueValidatedJoints.clear();
      jointTorqueIndexMap.clear();

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         if (considerJointTorque(i))
         {
            torqueValidatedJoints.add(oneDoFJoints[i]);
            jointTorqueIndexMap.put(oneDoFJoints[i], numberOfTorqueValidatedJoints++);
         }
      }
   }

   public void updateActuationConstraintVector()
   {
      if (INCLUDE_GRAVITY_CORIOLIS_TORQUES)
      {
         gravityCoriolisExternalWrenchMatrixCalculator.compute();
      }

      constraintLowerBound.reshape(numberOfTorqueValidatedJoints, 1);
      constraintUpperBound.reshape(numberOfTorqueValidatedJoints, 1);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         if (!considerJointTorque(i))
            continue;

         double gravityTorque = INCLUDE_GRAVITY_CORIOLIS_TORQUES ? gravityCoriolisExternalWrenchMatrixCalculator.getComputedJointTau(oneDoFJoints[i]).get(0, 0) : 0.0;
         double torqueConstraintLowerBound = getEffortLimitLower(oneDoFJoints[i]) - gravityTorque;
         double torqueConstraintUpperBound = getEffortLimitUpper(oneDoFJoints[i]) - gravityTorque;

         if (torqueConstraintLowerBound > torqueConstraintUpperBound)
         {
            LogTools.info("Infeasible torque constraint for " + oneDoFJoints[i].getName() + ". Likely due to high coriolis forces.");
            clear();
            return;
         }

         constraintLowerBound.set(jointTorqueIndexMap.get(oneDoFJoints[i]), torqueConstraintLowerBound);
         constraintUpperBound.set(jointTorqueIndexMap.get(oneDoFJoints[i]), torqueConstraintUpperBound);
      }
   }

   public void updateActuationConstraintMatrix(boolean updateContactPointFrame)
   {
      int nContactForceVariables = LINEAR_DIMENSIONS * contactPoints.size();
      graspMatrixJacobianTranspose.reshape(numberOfTorqueValidatedJoints, nContactForceVariables);
      graspMatrixJacobianTranspose.zero();

      contactJacobian.reshape(LINEAR_DIMENSIONS, numberOfTorqueValidatedJoints);
      contactJacobianTranspose.reshape(numberOfTorqueValidatedJoints, LINEAR_DIMENSIONS);

      for (int contactPointIndex = 0; contactPointIndex < contactPoints.size(); contactPointIndex++)
      {
         ContactPoint contactPoint = contactPoints.get(contactPointIndex);
         JointBasics[] contactJacobianJoints = contactJacobians.get(contactPoint.contactingBody).getJointsInOrder();

         if (!containsTorqueValidatedJoint(contactJacobianJoints))
         {
            continue;
         }

         if (updateContactPointFrame)
         {
            worldAlignedContactPose.setToZero(contactPoint.contactFrame);
            worldAlignedContactPose.changeFrame(ReferenceFrame.getWorldFrame());
            worldAlignedContactPose.getOrientation().setToZero();
            worldAlignedContactFrame.setPoseAndUpdate(worldAlignedContactPose);
         }

         contactJacobians.get(contactPoint.contactingBody).changeFrame(worldAlignedContactFrame);
         contactJacobians.get(contactPoint.contactingBody).compute();

         DMatrixRMaj jacobianMatrix = contactJacobians.get(contactPoint.contactingBody).getJacobianMatrix();
         contactJacobian.zero();

         for (int jacobianJointIndex = 1; jacobianJointIndex < contactJacobianJoints.length; jacobianJointIndex++)
         {
            OneDoFJointBasics joint = (OneDoFJointBasics) contactJacobianJoints[jacobianJointIndex];
            if (!considerJointTorque(jointIndexMap.get(joint)))
            {
               continue;
            }

            int jointIndex = jointTorqueIndexMap.get(joint);

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

      stackedConstraintMatrix.reshape(getNumberOfActuationConstraints(), nContactForceVariables);
      stackedConstraintVector.reshape(getNumberOfActuationConstraints(), 1);

      int rowOffsetLowerBound = 0;
      int rowOffsetUpperBound = numberOfTorqueValidatedJoints;

      /* Lower torque bound constraint: J^T f <= - tau_lower + g */
      MatrixTools.setMatrixBlock(stackedConstraintMatrix, rowOffsetLowerBound, 0, graspMatrixJacobianTranspose, 0, 0, graspMatrixJacobianTranspose.getNumRows(), graspMatrixJacobianTranspose.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(stackedConstraintVector, rowOffsetLowerBound, 0, constraintLowerBound, 0, 0, constraintLowerBound.getNumRows(), constraintLowerBound.getNumCols(), -1.0);

      /* Upper torque bound constraint: -J^T f <= tau_upper - g */
      MatrixTools.setMatrixBlock(stackedConstraintMatrix, rowOffsetUpperBound, 0, graspMatrixJacobianTranspose, 0, 0, graspMatrixJacobianTranspose.getNumRows(), graspMatrixJacobianTranspose.getNumCols(), -1.0);
      MatrixTools.setMatrixBlock(stackedConstraintVector, rowOffsetUpperBound, 0, constraintUpperBound, 0, 0, constraintUpperBound.getNumRows(), constraintUpperBound.getNumCols(), 1.0);
   }

   public void updateActuationMatrixGradient()
   {
      int nContactForceVariables = LINEAR_DIMENSIONS * contactPoints.size();
      graspMatrixJacobianTranspose.reshape(numberOfTorqueValidatedJoints, nContactForceVariables);
      graspMatrixJacobianTranspose.zero();

      contactJacobian.reshape(LINEAR_DIMENSIONS, numberOfTorqueValidatedJoints);
      contactJacobianTranspose.reshape(numberOfTorqueValidatedJoints, LINEAR_DIMENSIONS);

      for (int contactPointIndex = 0; contactPointIndex < contactPoints.size(); contactPointIndex++)
      {
         ContactPoint contactPoint = contactPoints.get(contactPointIndex);
         JointBasics[] contactJacobianJoints = contactJacobians.get(contactPoint.contactingBody).getJointsInOrder();

         if (!containsTorqueValidatedJoint(contactJacobianJoints))
         {
            continue;
         }

         worldAlignedContactPose.setToZero(contactPoint.contactFrame);
         worldAlignedContactPose.changeFrame(ReferenceFrame.getWorldFrame());
         worldAlignedContactPose.getOrientation().setToZero();
         worldAlignedContactFrame.setPoseAndUpdate(worldAlignedContactPose);

         /* Compute 6 x n geometric jacobian rate */
         geometricJacobianCalculator.setKinematicChain(contactJacobianJoints);
         geometricJacobianCalculator.setJacobianFrame(worldAlignedContactFrame);

         /* compute 3 x n point jacobian rate */
//         pointJacobianRate.set(geometricJacobianCalculator, contactPoint.contactPose.getPosition());
//         pointJacobianRate.compute();
//         DMatrixRMaj pointJacobianRateMatrix = pointJacobianRate.getJacobianRateMatrix();

         System.out.println("Jacobian rate matrix at control frame");
         System.out.println(geometricJacobianCalculator.getJacobianRateMatrix());
         System.out.println();
         System.out.println();

         geometricJacobianCalculator.setKinematicChain(contactJacobianJoints);

         System.out.println("Jacobian rate matrix at body fixed frame");
         System.out.println(geometricJacobianCalculator.getJacobianRateMatrix());
         System.out.println();
         System.out.println();

         DMatrixRMaj jacobianRateMatrix = geometricJacobianCalculator.getJacobianRateMatrix();
         contactJacobian.zero();

         for (int jacobianJointIndex = 1; jacobianJointIndex < contactJacobianJoints.length; jacobianJointIndex++)
         {
            OneDoFJointBasics joint = (OneDoFJointBasics) contactJacobianJoints[jacobianJointIndex];
            if (!considerJointTorque(jointIndexMap.get(joint)))
            {
               continue;
            }

            int jointIndex = jointTorqueIndexMap.get(joint);

            for (int linearCoordIndex = 0; linearCoordIndex < LINEAR_DIMENSIONS; linearCoordIndex++)
            {
               // We're computing just the linear component, ignore angular block
               int rowOffset = LINEAR_DIMENSIONS;

               // Offset to ignore root joint indices
               int colOffset = 5;

               double jacobianEntry = jacobianRateMatrix.get(rowOffset + linearCoordIndex, colOffset + jacobianJointIndex);

               contactJacobian.set(linearCoordIndex, jointIndex, jacobianEntry);
            }
         }

         CommonOps_DDRM.transpose(contactJacobian, contactJacobianTranspose);
         MatrixTools.setMatrixBlock(graspMatrixJacobianTranspose, 0, LINEAR_DIMENSIONS * contactPointIndex, contactJacobianTranspose, 0, 0, contactJacobianTranspose.getNumRows(), contactJacobianTranspose.getNumCols(), 1.0);
      }

      stackedConstraintMatrixGradient.reshape(getNumberOfActuationConstraints(), nContactForceVariables);

      int rowOffsetLowerBound = 0;
      int rowOffsetUpperBound = numberOfTorqueValidatedJoints;

      /* Lower torque bound constraint: J^T f <= - tau_lower + g */
      MatrixTools.setMatrixBlock(stackedConstraintMatrixGradient, rowOffsetLowerBound, 0, graspMatrixJacobianTranspose, 0, 0, graspMatrixJacobianTranspose.getNumRows(), graspMatrixJacobianTranspose.getNumCols(), 1.0);

      /* Upper torque bound constraint: -J^T f <= tau_upper - g */
      MatrixTools.setMatrixBlock(stackedConstraintMatrixGradient, rowOffsetUpperBound, 0, graspMatrixJacobianTranspose, 0, 0, graspMatrixJacobianTranspose.getNumRows(), graspMatrixJacobianTranspose.getNumCols(), -1.0);
   }

   private boolean containsTorqueValidatedJoint(JointBasics[] jacobianJoints)
   {
      for (int i = 1; i < jacobianJoints.length; i++)
      {
         if (considerJointTorque(jointIndexMap.get(jacobianJoints[i])))
         {
            return true;
         }
      }

      return false;
   }

   private boolean considerJointTorque(int jointIndex)
   {
      return jointTorqueStaticMask[jointIndex] && jointLoadStatusMask[jointIndex];
   }

   private int getNumberOfActuationConstraints()
   {
      return 2 * numberOfTorqueValidatedJoints;
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

   public DMatrixRMaj getActuationConstraintMatrixGradient()
   {
      return stackedConstraintMatrixGradient;
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

   public void setupForSelectedJoints(Predicate<String> jointsToCheckTorqueFeasibilityFilter)
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         jointTorqueStaticMask[i] = jointsToCheckTorqueFeasibilityFilter.test(oneDoFJoints[i].getName());
      }
   }

   public GeometricJacobian getJacobian(RigidBodyBasics rigidBody)
   {
      return contactJacobians.get(rigidBody);
   }

   /**
    * Given constraintActiveSetIndex, which represents the constraint row of A in the problem constraint Ax <= b,
    * this returns true if that row corresponds to a joint actuation constraint or false otherwise when the row corresponds to a
    * static equilibrium constraint. constraintActiveSetIndex can be obtained from {@link LinearProgramSolver#toConstraintIndex}
    */
   public boolean isJointTorqueActuationConstraint(int constraintActiveSetIndex)
   {
      int staticEquilibriumInequalityConstraints = 2 * CenterOfMassStabilityMarginOptimizationModule.STATIC_EQUILIBRIUM_CONSTRAINTS;
      return constraintActiveSetIndex >= staticEquilibriumInequalityConstraints;
   }

   /**
    * Returns the joint corresponding to the constraint row actuationConstraintIndex of A in the problem constraint Ax <= b.
    * constraintActiveSetIndex can be obtained from {@link LinearProgramSolver#toConstraintIndex}
    */
   public OneDoFJointBasics getJointFromActuationConstraintIndex(int constraintActiveSetIndex)
   {
      int staticEquilibriumInequalityConstraints = 2 * CenterOfMassStabilityMarginOptimizationModule.STATIC_EQUILIBRIUM_CONSTRAINTS;
      if (constraintActiveSetIndex < staticEquilibriumInequalityConstraints)
      {
         throw new RuntimeException("Invalid constraint index");
      }

      return torqueValidatedJoints.get((constraintActiveSetIndex - staticEquilibriumInequalityConstraints) % numberOfTorqueValidatedJoints);
   }

   /**
    * Assuming the given index actuationConstraintIndex corresponds to a joint torque bound, returns true
    * if the constraint corresponds to the joint's upper torque bound and false otherwise
    */
   public boolean isActuationConstraintUpperBound(int constraintActiveSetIndex)
   {
      int staticEquilibriumInequalityConstraints = 2 * CenterOfMassStabilityMarginOptimizationModule.STATIC_EQUILIBRIUM_CONSTRAINTS;
      if (constraintActiveSetIndex < staticEquilibriumInequalityConstraints)
      {
         throw new RuntimeException("Invalid constraint index");
      }

      return constraintActiveSetIndex - staticEquilibriumInequalityConstraints >= numberOfTorqueValidatedJoints;
   }

   /**
    * Returns the joint corresponding to the constraint row actuationConstraintIndex of A_c in the actuation constraint A_c x <= b_c.
    */
   public OneDoFJointBasics getJointFromActuationConstraintIndexZeroIndexed(int constraintActiveSetIndex)
   {
      return torqueValidatedJoints.get(constraintActiveSetIndex % numberOfTorqueValidatedJoints);
   }

   /**
    * Assuming the given index actuationConstraintIndex corresponds to a joint torque bound, returns true
    * if the constraint corresponds to the joint's upper torque bound and false otherwise
    */
   public boolean isActuationConstraintUpperBoundZeroIndexed(int constraintActiveSetIndex)
   {
      return constraintActiveSetIndex >= numberOfTorqueValidatedJoints;
   }

   private static double getEffortLimitLower(OneDoFJointBasics joint)
   {
      if (Double.isFinite(joint.getEffortLimitLower()))
      {
         return joint.getEffortLimitLower();
      }
      else
      {
         return -1000.0;
      }
   }

   private static double getEffortLimitUpper(OneDoFJointBasics joint)
   {
      if (Double.isFinite(joint.getEffortLimitUpper()))
      {
         return joint.getEffortLimitUpper();
      }
      else
      {
         return 1000.0;
      }
   }
}

