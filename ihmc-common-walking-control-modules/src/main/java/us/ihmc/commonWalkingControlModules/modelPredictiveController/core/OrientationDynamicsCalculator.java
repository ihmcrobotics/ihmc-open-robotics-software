package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.misc.UnrolledInverseFromMinor_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPoint;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.MatrixMissingTools;

import java.util.List;

public class OrientationDynamicsCalculator
{
   private static final boolean debug = true;

   private final DMatrixRMaj gravityVector = new DMatrixRMaj(3, 1);

   private final FrameVector3D desiredNetAngularMomentumRate = new FrameVector3D();
   private final FrameVector3D desiredBodyAngularMomentumRate = new FrameVector3D();

   private final FrameVector3D rotatedBodyAngularMomentumRate = new FrameVector3D();
   private final DMatrixRMaj skewRotatedBodyAngularMomentumRate = new DMatrixRMaj(3, 3);

   private final CommonMatrix3DBasics desiredRotationMatrix = new RotationMatrix();
   private final DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj inertiaMatrixInBody = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj inverseInertia = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj desiredBodyAngularVelocity = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj skewDesiredBodyAngularVelocity = new DMatrixRMaj(3, 3);
   private final FrameVector3D desiredContactForce = new FrameVector3D();
   private final DMatrixRMaj skewDesiredContactForce = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj comPositionJacobian = new DMatrixRMaj(3, 0);

   private final DMatrixRMaj contactPointForceJacobian = new DMatrixRMaj(3, 0);
   final DMatrixRMaj contactOriginTorqueJacobian = new DMatrixRMaj(3, 0);


   private final DMatrixRMaj b0 = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj b1 = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj b2 = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj b3 = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj b4 = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj A = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj B = new DMatrixRMaj(6, 0);
   private final DMatrixRMaj C = new DMatrixRMaj(6, 1);

   private DiscretizationCalculator discretizationCalculator = new EfficientFirstOrderHoldDiscretizationCalculator();

   private final DMatrixRMaj Ad = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj Bd = new DMatrixRMaj(6, 0);
   private final DMatrixRMaj Cd = new DMatrixRMaj(6, 1);

   private final double mass;

   private static final DMatrixRMaj identity3 = CommonOps_DDRM.identity(3);

   public OrientationDynamicsCalculator(double mass, double gravity)
   {
      this.mass = mass;

      gravityVector.set(2, 0, -Math.abs(gravity));
   }

   public void setDiscretizationCalculator(DiscretizationCalculator discretizationCalculator)
   {
      this.discretizationCalculator = discretizationCalculator;
   }

   public void setMomentumOfInertiaInBodyFrame(Matrix3DReadOnly inertiaMatrixInBody)
   {
      inertiaMatrixInBody.get(this.inertiaMatrixInBody);
   }

   public boolean compute(FramePoint3DReadOnly desiredComPosition,
                          FrameVector3DReadOnly desiredCoMAcceleration,
                          FrameOrientation3DReadOnly desiredBodyOrientation,
                          Vector3DReadOnly desiredBodyAngularVelocityInBodyFrame,
                          Vector3DReadOnly desiredNetAngularMomentumRate,
                          Vector3DReadOnly desiredInternalAngularMomentumRate,
                          List<MPCContactPlane> contactPlanes,
                          double timeOfConstraint,
                          double durationOfHold,
                          double omega)
   {
      reset(contactPlanes);
      getAllTheTermsFromTheCommandInput(contactPlanes,
                                        desiredCoMAcceleration,
                                        desiredBodyOrientation,
                                        desiredBodyAngularVelocityInBodyFrame,
                                        desiredNetAngularMomentumRate,
                                        desiredInternalAngularMomentumRate);

      calculateStateJacobians(desiredComPosition, contactPlanes, timeOfConstraint, omega);

      calculateAffineAxisAngleErrorTerms(contactPlanes,
                                         desiredComPosition,
                                         desiredBodyAngularVelocityInBodyFrame,
                                         desiredNetAngularMomentumRate);

      computeAffineTimeInvariantTerms(timeOfConstraint, desiredBodyAngularVelocityInBodyFrame);
      if (!Double.isFinite(durationOfHold))
         throw new IllegalArgumentException("The duration of the hold is not finite.");

      discretizationCalculator.compute(A, B, C, Ad, Bd, Cd, durationOfHold);

      return true;
   }

   DiscretizationCalculator getDiscretizationCalculator()
   {
      return discretizationCalculator;
   }

   DMatrixRMaj getContinuousAMatrix()
   {
      return A;
   }

   DMatrixRMaj getContinuousBMatrix()
   {
      return B;
   }

   DMatrixRMaj getContinuousCMatrix()
   {
      return C;
   }

   DMatrixRMaj getDiscreteAMatrix()
   {
      return Ad;
   }

   DMatrixRMaj getDiscreteBMatrix()
   {
      return Bd;
   }

   DMatrixRMaj getDiscreteCMatrix()
   {
      return Cd;
   }

   DMatrixRMaj getB0()
   {
      return b0;
   }

   DMatrixRMaj getB1()
   {
      return b1;
   }

   DMatrixRMaj getB2()
   {
      return b2;
   }

   DMatrixRMaj getB3()
   {
      return b3;
   }

   DMatrixRMaj getB4()
   {
      return b4;
   }

   private void reset(List<MPCContactPlane> contactPlanes)
   {
      int rhoCoefficientsInSegment = 0;
      for (int i = 0; i < contactPlanes.size(); i++)
      {
         rhoCoefficientsInSegment += contactPlanes.get(i).getCoefficientSize();
      }

      int coefficientsInSegment = LinearMPCIndexHandler.comCoefficientsPerSegment + rhoCoefficientsInSegment;

      comPositionJacobian.reshape(3, coefficientsInSegment);
      contactOriginTorqueJacobian.reshape(3, rhoCoefficientsInSegment);

      B.reshape(6, coefficientsInSegment);
      Bd.reshape(6, coefficientsInSegment);

      comPositionJacobian.zero();
      contactOriginTorqueJacobian.zero();

      b0.zero();
      b1.zero();
      b2.zero();
      b3.zero();
      b4.zero();

      A.zero();
      B.zero();
      C.zero();

      Ad.zero();
      Bd.zero();
      Cd.zero();
   }

   private void getAllTheTermsFromTheCommandInput(List<MPCContactPlane> contactPlanes,
                                                  FrameVector3DReadOnly desiredCoMAcceleration,
                                                  FrameOrientation3DReadOnly desiredBodyOrientation,
                                                  Vector3DReadOnly desiredBodyAngularVelocityInBodyFrame,
                                                  Vector3DReadOnly desiredNetAngularMomentumRate,
                                                  Vector3DReadOnly desiredInternalAngularMomentumRate)
   {
      this.desiredNetAngularMomentumRate.set(desiredNetAngularMomentumRate);
      desiredBodyAngularMomentumRate.sub(desiredNetAngularMomentumRate, desiredInternalAngularMomentumRate);

      desiredBodyOrientation.get(desiredRotationMatrix);
      desiredRotationMatrix.get(rotationMatrix);

      desiredBodyAngularVelocityInBodyFrame.get(desiredBodyAngularVelocity);

      desiredContactForce.set(desiredCoMAcceleration);
      desiredContactForce.addZ(-gravityVector.get(2, 0));
      desiredContactForce.scale(mass);
      if (debug && contactPlanes.size() < 1)
      {
         if (desiredContactForce.length() > 1e-4)
            throw new RuntimeException("Should have zero desired force.");
      }


      MatrixMissingTools.toSkewSymmetricMatrix(desiredContactForce, skewDesiredContactForce);
      MatrixMissingTools.toSkewSymmetricMatrix(desiredBodyAngularVelocityInBodyFrame, skewDesiredBodyAngularVelocity);

      UnrolledInverseFromMinor_DDRM.inv3(inertiaMatrixInBody, inverseInertia, 1.0);
   }

   private void calculateStateJacobians(FramePoint3DReadOnly desiredCoMPosition, List<MPCContactPlane> contactPlanes, double timeOfConstraint, double omega)
   {
      int rhoStartIndex = 0;

      CoMCoefficientJacobianCalculator.calculateCoMJacobian(0, timeOfConstraint, comPositionJacobian, 0, 1.0);

      for (int i = 0; i < contactPlanes.size(); i++)
      {
         MPCContactPlane contactPlane = contactPlanes.get(i);
         ContactPlaneJacobianCalculator.computeLinearJacobian(0, timeOfConstraint, omega, LinearMPCIndexHandler.comCoefficientsPerSegment + rhoStartIndex, contactPlane, comPositionJacobian);

         computeTorqueJacobian(rhoStartIndex, timeOfConstraint, omega, desiredCoMPosition, contactPlane, contactOriginTorqueJacobian);

         rhoStartIndex += contactPlane.getCoefficientSize();
      }
   }

   private final DMatrixRMaj IR = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj angularMomentum = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj skewAngularMomentum = new DMatrixRMaj(3, 3);

   private final Vector3D torqueAboutPoint = new Vector3D();
   private final DMatrixRMaj torqueAboutPointVector = new DMatrixRMaj(3, 1);

   private void calculateAffineAxisAngleErrorTerms(List<MPCContactPlane> contactPlanes,
                                                   FramePoint3DReadOnly desiredCoMPosition,
                                                   Vector3DReadOnly desiredBodyAngularVelocityInBodyFrame,
                                                   Vector3DReadOnly desiredNetAngularMomentumRate)
   {
      CommonOps_DDRM.multTransB(inverseInertia, rotationMatrix, IR);

      CommonOps_DDRM.mult(IR, skewDesiredContactForce, b1);
      CommonOps_DDRM.mult(IR, contactOriginTorqueJacobian, b2);

      // multiplying by skew - can make more efficient
      desiredRotationMatrix.inverseTransform(desiredBodyAngularMomentumRate, rotatedBodyAngularMomentumRate);
      MatrixMissingTools.toSkewSymmetricMatrix(rotatedBodyAngularMomentumRate, skewRotatedBodyAngularMomentumRate);
      CommonOps_DDRM.mult(inverseInertia, skewRotatedBodyAngularMomentumRate, b3);

      CommonOps_DDRM.mult(inertiaMatrixInBody, desiredBodyAngularVelocity, angularMomentum);
      MatrixMissingTools.toSkewSymmetricMatrix(angularMomentum, skewAngularMomentum);
      crossSub(skewAngularMomentum, desiredBodyAngularVelocityInBodyFrame, inertiaMatrixInBody);
      CommonOps_DDRM.mult(inverseInertia, skewAngularMomentum, b4);

      if (debug && contactPlanes.size() < 1)
      {
         if (desiredBodyAngularMomentumRate.length() > 1e-4)
            throw new RuntimeException("This should be zero.");
         if (desiredContactForce.length() > 1e-4)
            throw new RuntimeException("Should have zero desired force.");
         if (desiredNetAngularMomentumRate.length() > 1e-4)
            throw new RuntimeException("Should be zero in flight.");
      }
      torqueAboutPoint.cross(desiredContactForce, desiredCoMPosition);
      torqueAboutPoint.add(desiredNetAngularMomentumRate);
      torqueAboutPoint.scale(-1.0);
      torqueAboutPoint.get(torqueAboutPointVector);

      CommonOps_DDRM.mult(IR, torqueAboutPointVector, b0);
   }

   private final FrameVector3D momentArm = new FrameVector3D();
   private final DMatrixRMaj momentArmSkew = new DMatrixRMaj(3, 3);


   private void computeTorqueJacobian(int colStart,
                                      double timeOfConstraint,
                                      double omega,
                                      FramePoint3DReadOnly desiredCoMPosition,
                                      MPCContactPlane contactPlane,
                                      DMatrixRMaj jacobianToPack)
   {
      for (int i = 0; i < contactPlane.getNumberOfContactPoints(); i++)
      {
         MPCContactPoint contactPoint = contactPlane.getContactPointHelper(i);
         momentArm.sub(contactPoint.getBasisVectorOrigin(), desiredCoMPosition);
         MatrixMissingTools.toSkewSymmetricMatrix(momentArm, momentArmSkew);

         contactPointForceJacobian.reshape(3, contactPoint.getCoefficientsSize());
         ContactPlaneJacobianCalculator.computeContactPointAccelerationJacobian(mass, timeOfConstraint, omega, 0, 0, contactPoint, contactPointForceJacobian);

         MatrixMissingTools.multSetBlock(momentArmSkew, contactPointForceJacobian, jacobianToPack, 0, colStart);

         colStart += contactPoint.getCoefficientsSize();
      }
   }

   private void computeAffineTimeInvariantTerms(double timeOfConstraint, Vector3DReadOnly desiredBodyAngularVelocityInBodyFrame)
   {
      MatrixMissingTools.toSkewSymmetricMatrix(-1.0, desiredBodyAngularVelocityInBodyFrame, A, 0, 0);
      MatrixTools.setMatrixBlock(A, 0, 0, skewDesiredBodyAngularVelocity, 0, 0, 3, 3, -1.0);
      MatrixTools.setMatrixBlock(A, 0, 3, identity3, 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(A, 3, 0, b3, 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(A, 3, 3, b4, 0, 0, 3, 3, 1.0);

      MatrixMissingTools.multSetBlock(b1, comPositionJacobian, B, 3, 0);
      MatrixTools.addMatrixBlock(B, 3, LinearMPCIndexHandler.comCoefficientsPerSegment, b2, 0, 0, 3, b2.getNumCols(), 1.0);

      MatrixTools.setMatrixBlock(C, 3, 0, b0, 0, 0, 3, 1, 1.0);
      MatrixTools.multAddBlock(0.5 * timeOfConstraint * timeOfConstraint, b1, gravityVector, C, 3, 0);
   }

   private static void crossSub(DMatrixRMaj result, Vector3DReadOnly vector, DMatrixRMaj matrix)
   {
      if (result.getNumCols() != 3 || result.getNumRows() != 3)
         throw new IllegalArgumentException("Improperly sized results matrix!");

      result.add(0, 0, (vector.getZ() * matrix.get(1, 0) - vector.getY() * matrix.get(2, 0)));
      result.add(0, 1, (vector.getZ() * matrix.get(1, 1) - vector.getY() * matrix.get(2, 1)));
      result.add(0, 2, (vector.getZ() * matrix.get(1, 2) - vector.getY() * matrix.get(2, 2)));

      result.add(1, 0, (vector.getX() * matrix.get(2, 0) - vector.getZ() * matrix.get(0, 0)));
      result.add(1, 1, (vector.getX() * matrix.get(2, 1) - vector.getZ() * matrix.get(0, 1)));
      result.add(1, 2, (vector.getX() * matrix.get(2, 2) - vector.getZ() * matrix.get(0, 2)));

      result.add(2, 0, (vector.getY() * matrix.get(0, 0) - vector.getX() * matrix.get(1, 0)));
      result.add(2, 1, (vector.getY() * matrix.get(0, 1) - vector.getX() * matrix.get(1, 1)));
      result.add(2, 2, (vector.getY() * matrix.get(0, 2) - vector.getX() * matrix.get(1, 2)));
   }
}
