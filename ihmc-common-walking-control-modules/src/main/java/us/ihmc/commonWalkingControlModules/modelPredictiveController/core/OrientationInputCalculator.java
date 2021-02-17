package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.misc.UnrolledInverseFromMinor_DDRM;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.DiscreteOrientationCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;

public class OrientationInputCalculator
{
   private final FrameVector3D desiredBodyAngularMomentum = new FrameVector3D();
   private final DMatrixRMaj desiredBodyAngularMomentumVector = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj desireInternalAngularMomentumRate = new DMatrixRMaj(3, 1);

   private final DMatrixRMaj gravityVector = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj skewGravity = new DMatrixRMaj(3, 1);

   private final DMatrixRMaj rotatedBodyAngularMomentum = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj skewRotatedBodyAngularMomentum = new DMatrixRMaj(3, 1);

   private final CommonMatrix3DBasics tempRotationMatrix = new RotationMatrix();
   private final DMatrixRMaj desiredRotationMatrix = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj inertiaMatrixInBody = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj inverseInertia = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj comCoriolisForce = new DMatrixRMaj(3, 1);

   private final DMatrixRMaj desiredBodyAngularVelocity = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj desiredCoMPosition = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj desiredCoMVelocity = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj skewDesiredCoMPosition = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj skewDesiredCoMVelocity = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj comPositionJacobian = new DMatrixRMaj(3, 0);
   private final DMatrixRMaj comVelocityJacobian = new DMatrixRMaj(3, 0);
   private final DMatrixRMaj contactForceJacobian = new DMatrixRMaj(3, 0);
   private final DMatrixRMaj contactForceToOriginTorqueJacobian = new DMatrixRMaj(3, 0);
   private final DMatrixRMaj originTorqueJacobian = new DMatrixRMaj(3, 0);

   private final DMatrixRMaj a0 = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj a1 = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj a2 = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj a3 = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj a4 = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj A = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj B = new DMatrixRMaj(6, 0);
   private final DMatrixRMaj C = new DMatrixRMaj(6, 1);

   private final MatrixExponentialCalculator matrixExponentialCalculator = new MatrixExponentialCalculator(6);

   private final DMatrixRMaj Ad = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj Bd = new DMatrixRMaj(6, 0);
   private final DMatrixRMaj Cd = new DMatrixRMaj(6, 1);

   private final SE3MPCIndexHandler indexHandler;
   private final double mass;

   private static final DMatrixRMaj identity = CommonOps_DDRM.identity(6);

   public OrientationInputCalculator(SE3MPCIndexHandler indexHandler, double mass, double gravity)
   {
      this.indexHandler = indexHandler;
      this.mass = mass;

      gravityVector.set(2, 0, gravity);
      MatrixMissingTools.toSkewSymmetricMatrix(gravityVector, skewGravity);
   }

   public boolean compute(QPInputTypeA inputToPack, DiscreteOrientationCommand command)
   {
      inputToPack.setNumberOfVariables(indexHandler.getTotalProblemSize());
      inputToPack.reshape(6);

      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();
      inputToPack.setConstraintType(ConstraintType.EQUALITY);

      reset(command);
      getAllTheTermsFromTheCommandInput(command);

      calculateAffineAxisAngleErrorTerms();

      calculateStateJacobians(command);

      computeAffineTimeInvariantTerms(command.getTimeOfConstraint());
      computeDiscreteAffineTimeInvariantTerms(command.getDurationOfHold());

      if (command.getEndDiscreteTickId() == 0)
         setUpConstraintForFirstTick(inputToPack, command);
      else
         setUpConstraintForRegularTick(inputToPack, command);

      return true;
   }

   private void reset(DiscreteOrientationCommand command)
   {
      int totalContactPoints = 0;
      for (int i = 0; i < command.getNumberOfContacts(); i++)
         totalContactPoints += command.getContactPlaneHelper(i).getNumberOfContactPoints();

      comPositionJacobian.reshape(3, indexHandler.getTotalProblemSize());
      comVelocityJacobian.reshape(3, indexHandler.getTotalProblemSize());
      originTorqueJacobian.reshape(3, indexHandler.getTotalProblemSize());

      contactForceJacobian.reshape(3 * totalContactPoints, indexHandler.getTotalProblemSize());
      contactForceToOriginTorqueJacobian.reshape(3, 3 * totalContactPoints);

      comPositionJacobian.zero();
      comVelocityJacobian.zero();
      contactForceJacobian.zero();
      contactForceToOriginTorqueJacobian.zero();

   }

   private void getAllTheTermsFromTheCommandInput(DiscreteOrientationCommand command)
   {
      desiredBodyAngularMomentum.sub(command.getDesiredNetAngularMomentum(), command.getDesiredInternalAngularMomentum());
      desiredBodyAngularMomentum.get(desiredBodyAngularMomentumVector);

      command.getDesiredBodyOrientation().get(tempRotationMatrix);
      tempRotationMatrix.get(desiredRotationMatrix);

      command.getMomentOfInertiaInBodyFrame().get(inertiaMatrixInBody);
      command.getDesiredCoMPosition().get(desiredCoMPosition);
      command.getDesiredCoMVelocity().get(desiredCoMVelocity);
      command.getDesiredBodyAngularVelocity().get(desiredBodyAngularVelocity);

      MatrixMissingTools.toSkewSymmetricMatrix(desiredCoMPosition, skewDesiredCoMPosition);
      MatrixMissingTools.toSkewSymmetricMatrix(desiredCoMVelocity, skewDesiredCoMVelocity);

      UnrolledInverseFromMinor_DDRM.inv3(inertiaMatrixInBody, inverseInertia, 1.0);

      command.getDesiredInternalAngularMomentumRate().get(desireInternalAngularMomentumRate);
   }

   private void calculateStateJacobians(DiscreteOrientationCommand command)
   {
      double timeOfConstraint = command.getTimeOfConstraint();
      double omega = command.getOmega();
      int comStartIndex = indexHandler.getComCoefficientStartIndex(command.getSegmentNumber());
      int rhoStartIndex = indexHandler.getRhoCoefficientStartIndex(command.getSegmentNumber());

      CoMCoefficientJacobianCalculator.calculateCoMJacobian(comStartIndex, timeOfConstraint, comPositionJacobian, 0, 1.0);
      CoMCoefficientJacobianCalculator.calculateCoMJacobian(comStartIndex, timeOfConstraint, comVelocityJacobian, 1, 1.0);

      int contactRow = 0;
      for (int i = 0; i < command.getNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlane = command.getContactPlaneHelper(i);
         ContactPlaneJacobianCalculator.computeLinearJacobian(0, timeOfConstraint, omega, rhoStartIndex, contactPlane, comPositionJacobian);
         ContactPlaneJacobianCalculator.computeLinearJacobian(1, timeOfConstraint, omega, rhoStartIndex, contactPlane, comVelocityJacobian);
         ContactPlaneJacobianCalculator.computeContactPointAccelerationJacobian(mass, timeOfConstraint, omega, contactRow, rhoStartIndex, contactPlane, contactForceJacobian);
         computeTorqueAboutOriginJacobian(contactRow, contactPlane, contactForceToOriginTorqueJacobian);

         contactRow += contactPlane.getNumberOfContactPoints();
         rhoStartIndex += contactPlane.getCoefficientSize();
      }
   }

   private void calculateAffineAxisAngleErrorTerms()
   {
      CommonOps_DDRM.multTransB(inverseInertia, desiredRotationMatrix, a3);
      CommonOps_DDRM.mult(mass, a3, skewDesiredCoMVelocity, a1);
      CommonOps_DDRM.mult(-mass, a3, skewDesiredCoMPosition, a2);

      CommonOps_DDRM.mult(mass, skewDesiredCoMVelocity, desiredCoMPosition, comCoriolisForce);
      CommonOps_DDRM.mult(-1.0, a3, desiredBodyAngularMomentumVector, a0);
      CommonOps_DDRM.multAdd(a3, comCoriolisForce, a0);

      CommonOps_DDRM.mult(desiredRotationMatrix, desiredBodyAngularMomentumVector, rotatedBodyAngularMomentum);
      MatrixMissingTools.toSkewSymmetricMatrix(rotatedBodyAngularMomentum, skewRotatedBodyAngularMomentum);

      CommonOps_DDRM.mult(inverseInertia, skewRotatedBodyAngularMomentum, a4);
      CommonOps_DDRM.subtractEquals(a4, desiredBodyAngularVelocity);
   }


   private final DMatrixRMaj contactPoint = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj skewContactPoint = new DMatrixRMaj(3, 3);

   private void computeTorqueAboutOriginJacobian(int colStart, MPCContactPlane contactPlane, DMatrixRMaj jacobianToPack)
   {
      for (int i = 0; i < contactPlane.getNumberOfContactPoints(); i++)
      {
         contactPlane.getContactPointHelper(i).getBasisVectorOrigin().get(contactPoint);
         MatrixMissingTools.toSkewSymmetricMatrix(contactPoint, skewContactPoint);

         MatrixMissingTools.setMatrixBlock(jacobianToPack, 0, colStart, skewContactPoint, 0, 0, 3, 3, 1.0);
         colStart += 3;
      }
   }

   private void computeAffineTimeInvariantTerms(double timeOfConstraint)
   {
      MatrixTools.setMatrixBlock(A, 0, 0, a3, 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(A, 0, 3, a4, 0, 0, 3, 3, 1.0);

      B.reshape(6, indexHandler.getTotalProblemSize());
      B.zero();

      MatrixTools.multAddBlock(a1, comPositionJacobian, B, 0, 0);
      MatrixTools.multAddBlock(a2, comVelocityJacobian, B, 0, 0);

      MatrixTools.multAddBlock(-mass, skewGravity, comPositionJacobian, B, 3, 0);
      MatrixTools.multAddBlock(contactForceToOriginTorqueJacobian, contactForceJacobian, B, 3, 0);

      MatrixTools.setMatrixBlock(C, 3, 0, desireInternalAngularMomentumRate, 0, 0, 3, 1, -1.0);
      MatrixTools.multAddBlock(0.5 * timeOfConstraint * timeOfConstraint, a1, gravityVector, C, 0, 0);
      MatrixTools.multAddBlock(timeOfConstraint, a2, gravityVector, C, 0, 0);

      MatrixTools.setMatrixBlock(C, 0, 0, a0, 0, 0, 3, 1, 1.0);
   }

   private void computeDiscreteAffineTimeInvariantTerms(double duration)
   {
      matrixExponentialCalculator.reshape(6);
      CommonOps_DDRM.scale(duration, A);
      matrixExponentialCalculator.compute(Ad, A);

      CommonOps_DDRM.scale(duration, B, Bd);
      CommonOps_DDRM.scale(duration, C, Cd);
   }

   private final DMatrixRMaj initialStateVector = new DMatrixRMaj(6, 1);

   private void setUpConstraintForFirstTick(QPInputTypeA inputToPack,
                                            DiscreteOrientationCommand command)
   {
      command.getCurrentBodyAngularMomentumAboutFixedPoint().get(initialStateVector);
      command.getCurrentAxisAngleError().get(3, initialStateVector);

      CommonOps_DDRM.mult(Ad, initialStateVector, inputToPack.getTaskObjective());
      CommonOps_DDRM.addEquals(inputToPack.getTaskObjective(), Cd);

      CommonOps_DDRM.scale(-1.0, Bd, inputToPack.getTaskJacobian());

      MatrixTools.addMatrixBlock(inputToPack.getTaskJacobian(), 0, indexHandler.getOrientationTickStartIndex(command.getEndDiscreteTickId()), identity, 0, 0, 6, 6, 1.0);
   }

   private void setUpConstraintForRegularTick(QPInputTypeA inputToPack,
                                              DiscreteOrientationCommand command)
   {
      inputToPack.getTaskObjective().set(Cd);

      CommonOps_DDRM.scale(-1.0, Bd, inputToPack.getTaskJacobian());
      MatrixTools.addMatrixBlock(inputToPack.getTaskJacobian(), 0, indexHandler.getOrientationTickStartIndex(command.getEndDiscreteTickId()), identity, 0, 0, 6, 6, 1.0);
      MatrixTools.addMatrixBlock(inputToPack.getTaskJacobian(), 0, indexHandler.getOrientationTickStartIndex(command.getEndDiscreteTickId() - 1), Ad, 0, 0, 6, 6, -1.0);

   }
}
