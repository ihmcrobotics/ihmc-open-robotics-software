package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.ContactStateMagnitudeToForceMatrixHelper;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.LinearMPCIndexHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPoint;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.MatrixMissingTools;

import java.util.function.IntFunction;

public class MPCTestHelper
{
   private MPCTestHelper()
   {
   }
   public static ConvexPolygon2DReadOnly createDefaultContact()
   {
      return createContacts(0.22, 0.12);
   }

   public static ConvexPolygon2DReadOnly createContacts(double footLength, double footWidth)
   {
      ConvexPolygon2D contactPolygon = new ConvexPolygon2D();
      contactPolygon.addVertex(footLength / 2.0, footWidth / 2.0);
      contactPolygon.addVertex(footLength / 2.0, -footWidth / 2.0);
      contactPolygon.addVertex(-footLength / 2.0, footWidth / 2.0);
      contactPolygon.addVertex(-footLength / 2.0, -footWidth / 2.0);
      contactPolygon.update();

      return contactPolygon;
   }

   public static DMatrixRMaj getDCMPositionJacobian(double time, double omega, ContactStateMagnitudeToForceMatrixHelper rhoHelper)
   {
      return getDCMPositionJacobian(time, omega, rhoHelper.getRhoSize(), rhoHelper::getBasisVector);
   }

   public static DMatrixRMaj getDCMPositionJacobian(double time, double omega, MPCContactPlane contactPlane)
   {
      return getDCMPositionJacobian(time, omega, contactPlane.getRhoSize(), contactPlane::getBasisVector);
   }

   public static DMatrixRMaj getDCMPositionJacobian(double time, double omega, int rhoSize, IntFunction<FrameVector3DReadOnly> basisVectors)
   {
      DMatrixRMaj jacobian = getCoMPositionJacobian(time, omega, rhoSize, basisVectors);
      CommonOps_DDRM.addEquals(jacobian, 1.0 / omega, getCoMVelocityJacobian(time, omega, rhoSize, basisVectors));

      return jacobian;
   }

   public static DMatrixRMaj getVRPPositionJacobian(double time, double omega, ContactStateMagnitudeToForceMatrixHelper rhoHelper)
   {
      return getVRPPositionJacobian(time, omega, rhoHelper.getRhoSize(), rhoHelper::getBasisVector);
   }

   public static DMatrixRMaj getVRPPositionJacobian(double time, double omega, MPCContactPlane contactPlane)
   {
      return getVRPPositionJacobian(time, omega, contactPlane.getRhoSize(), contactPlane::getBasisVector);
   }

   public static DMatrixRMaj getVRPPositionJacobian(double time, double omega, int rhoSize, IntFunction<FrameVector3DReadOnly> basisVectors)
   {
      DMatrixRMaj jacobian = getCoMPositionJacobian(time, omega, rhoSize, basisVectors);
      CommonOps_DDRM.addEquals(jacobian, -1.0 / (omega * omega), getCoMAccelerationJacobian(time, omega, rhoSize, basisVectors));

      return jacobian;
   }

   public static DMatrixRMaj getVRPVelocityJacobian(double time, double omega, ContactStateMagnitudeToForceMatrixHelper rhoHelper)
   {
      DMatrixRMaj jacobian = getCoMVelocityJacobian(time, omega, rhoHelper);
      CommonOps_DDRM.addEquals(jacobian, -1.0 / (omega * omega), getCoMJerkJacobian(time, omega, rhoHelper));

      return jacobian;
   }

   public static DMatrixRMaj getVectorOfCoefficients(double gravityZ, ContactStateMagnitudeToForceMatrixHelper rhoHelper, DMatrixRMaj solution)
   {
      DMatrixRMaj contactForceCoefficients = getContactForceCoefficients(rhoHelper, solution);

      DMatrixRMaj coefficients = new DMatrixRMaj(3, 6);

      coefficients.set(0, 4, solution.get(0));
      coefficients.set(0, 5, solution.get(1));
      coefficients.set(1, 4, solution.get(2));
      coefficients.set(1, 5, solution.get(3));
      coefficients.set(2, 4, solution.get(4));
      coefficients.set(2, 5, solution.get(5));

      MatrixTools.addMatrixBlock(coefficients, 0, 0, contactForceCoefficients, 0, 0, 3, 4, 1.0);

      coefficients.add(2, 3, -0.5 * Math.abs(gravityZ));

      return coefficients;
   }

   public static DMatrixRMaj getContactForceCoefficients(ContactStateMagnitudeToForceMatrixHelper rhoHelper, DMatrixRMaj solution)
   {
      DMatrixRMaj coefficients = new DMatrixRMaj(3, 4);

      for (int rhoIdx  = 0; rhoIdx < rhoHelper.getRhoSize(); rhoIdx++)
      {
         int startIdx = 6 + 4 * rhoIdx;

         FrameVector3DReadOnly basisVector = rhoHelper.getBasisVector(rhoIdx);
         coefficients.add(0, 0, basisVector.getX() * solution.get(startIdx));
         coefficients.add(1, 0, basisVector.getY() * solution.get(startIdx));
         coefficients.add(2, 0, basisVector.getZ() * solution.get(startIdx));

         coefficients.add(0, 1, basisVector.getX() * solution.get(startIdx + 1));
         coefficients.add(1, 1, basisVector.getY() * solution.get(startIdx + 1));
         coefficients.add(2, 1, basisVector.getZ() * solution.get(startIdx + 1));

         coefficients.add(0, 2, basisVector.getX() * solution.get(startIdx + 2));
         coefficients.add(1, 2, basisVector.getY() * solution.get(startIdx + 2));
         coefficients.add(2, 2, basisVector.getZ() * solution.get(startIdx + 2));

         coefficients.add(0, 3, basisVector.getX() * solution.get(startIdx + 3));
         coefficients.add(1, 3, basisVector.getY() * solution.get(startIdx + 3));
         coefficients.add(2, 3, basisVector.getZ() * solution.get(startIdx + 3));
      }

      return coefficients;
   }

   public static DMatrixRMaj getContactValueJacobian(double time, double omega, MPCContactPlane contactPlane)
   {
      return getContactValueJacobian(time, omega, contactPlane.getRhoSize());
   }

   public static DMatrixRMaj getContactValueJacobian(double time, double omega, int rhoSize)
   {
      int coefficients = LinearMPCIndexHandler.comCoefficientsPerSegment + LinearMPCIndexHandler.coefficientsPerRho * rhoSize;
      DMatrixRMaj jacobian = new DMatrixRMaj(rhoSize, coefficients);

      double c2 = Math.exp(omega * time);
      double c3 = Math.exp(-omega * time);
      double c4 = time * time * time;
      double c5 = time * time;

      for (int rhoIdx  = 0; rhoIdx < rhoSize; rhoIdx++)
      {
         int startIdx = LinearMPCIndexHandler.comCoefficientsPerSegment + LinearMPCIndexHandler.coefficientsPerRho * rhoIdx;

         jacobian.set(rhoIdx, startIdx, (c2));
         jacobian.set(rhoIdx, startIdx + 1, (c3));
         jacobian.set(rhoIdx, startIdx + 2, (c4));
         jacobian.set(rhoIdx, startIdx + 3, (c5));
      }

      return jacobian;
   }

   public static DMatrixRMaj getContactRateJacobian(double time, double omega, MPCContactPlane contactPlane)
   {
      return getContactRateJacobian(time, omega, contactPlane.getRhoSize());
   }

   public static DMatrixRMaj getContactRateJacobian(double time, double omega, int rhoSize)
   {
      int coefficients = LinearMPCIndexHandler.comCoefficientsPerSegment + LinearMPCIndexHandler.coefficientsPerRho * rhoSize;
      DMatrixRMaj jacobian = new DMatrixRMaj(rhoSize, coefficients);

      double c2 = omega * Math.exp(omega * time);
      double c3 = -omega * Math.exp(-omega * time);
      double c4 = 3 * time * time;
      double c5 = 2 * time;

      for (int rhoIdx  = 0; rhoIdx < rhoSize; rhoIdx++)
      {
         int startIdx = LinearMPCIndexHandler.comCoefficientsPerSegment + LinearMPCIndexHandler.coefficientsPerRho * rhoIdx;

         jacobian.set(rhoIdx, startIdx, (c2));
         jacobian.set(rhoIdx, startIdx + 1, (c3));
         jacobian.set(rhoIdx, startIdx + 2, (c4));
         jacobian.set(rhoIdx, startIdx + 3, (c5));
      }

      return jacobian;
   }

   public static DMatrixRMaj getContactAccelerationJacobian(double time, double omega, MPCContactPlane contactPlane)
   {
      return getContactAccelerationJacobian(time, omega, contactPlane.getRhoSize());
   }

   public static DMatrixRMaj getContactAccelerationJacobian(double time, double omega, int rhoSize)
   {
      int coefficients = LinearMPCIndexHandler.comCoefficientsPerSegment + LinearMPCIndexHandler.coefficientsPerRho * rhoSize;
      DMatrixRMaj jacobian = new DMatrixRMaj(rhoSize, coefficients);

      double c2 = omega * omega * Math.exp(omega * time);
      double c3 = omega * omega * Math.exp(-omega * time);
      double c4 = 6 * time;
      double c5 = 2;

      for (int rhoIdx  = 0; rhoIdx < rhoSize; rhoIdx++)
      {
         int startIdx = LinearMPCIndexHandler.comCoefficientsPerSegment + LinearMPCIndexHandler.coefficientsPerRho * rhoIdx;

         jacobian.set(rhoIdx, startIdx, (c2));
         jacobian.set(rhoIdx, startIdx + 1, (c3));
         jacobian.set(rhoIdx, startIdx + 2, (c4));
         jacobian.set(rhoIdx, startIdx + 3, (c5));
      }

      return jacobian;
   }

   public static DMatrixRMaj getCoMPositionJacobian(double time, double omega, ContactStateMagnitudeToForceMatrixHelper rhoHelper)
   {
      return getCoMPositionJacobian(time, omega, rhoHelper.getRhoSize(), rhoHelper::getBasisVector);
   }

   public static FramePoint3DReadOnly computeCoMPosition(double time, double omega, double gravity, DMatrixRMaj coefficientVector, MPCContactPlane contactPlane)
   {
      DMatrixRMaj solutionPosition = new DMatrixRMaj(3, 1);
      CommonOps_DDRM.mult(getCoMPositionJacobian(time, omega, contactPlane), coefficientVector, solutionPosition);
      solutionPosition.add(2, 0, 0.5 * time * time * gravity);

      FramePoint3D position = new FramePoint3D();
      position.set(solutionPosition);

      return position;
   }

   public static FrameVector3DReadOnly computeCoMVelocity(double time, double omega, double gravity, DMatrixRMaj coefficientVector, MPCContactPlane contactPlane)
   {
      DMatrixRMaj solutionVelocity = new DMatrixRMaj(3, 1);
      CommonOps_DDRM.mult(getCoMVelocityJacobian(time, omega, contactPlane), coefficientVector, solutionVelocity);
      solutionVelocity.add(2, 0, time * gravity);

      FrameVector3D velocity = new FrameVector3D();
      velocity.set(solutionVelocity);

      return velocity;
   }

   public static FrameVector3DReadOnly computeCoMAcceleration(double time, double omega, double gravity, DMatrixRMaj coefficientVector, MPCContactPlane contactPlane)
   {
      DMatrixRMaj solutionAcceleration = new DMatrixRMaj(3, 1);
      CommonOps_DDRM.mult(getCoMAccelerationJacobian(time, omega, contactPlane), coefficientVector, solutionAcceleration);
      solutionAcceleration.add(2, 0, time * gravity);

      FrameVector3D acceleration = new FrameVector3D();
      acceleration.set(solutionAcceleration);

      return acceleration;
   }


   public static FramePoint3DReadOnly computeDCMPosition(double time, double omega, double gravity, DMatrixRMaj coefficientVector, MPCContactPlane contactPlane)
   {
      FramePoint3D dcmPosition = new FramePoint3D();
      CapturePointTools.computeCapturePointPosition(computeCoMPosition(time, omega, gravity, coefficientVector, contactPlane),
                                                    computeCoMVelocity(time, omega, gravity, coefficientVector, contactPlane),
                                                    omega,
                                                    dcmPosition);

      return dcmPosition;
   }

   public static FrameVector3DReadOnly computeDCMVelocity(double time, double omega, double gravity, DMatrixRMaj coefficientVector, MPCContactPlane contactPlane)
   {
      FrameVector3D dcmVelocity = new FrameVector3D();
      dcmVelocity.scaleAdd(1.0 / omega, computeCoMAcceleration(time, omega, gravity, coefficientVector, contactPlane), computeCoMVelocity(time, omega, gravity, coefficientVector, contactPlane));

      return dcmVelocity;
   }

   public static FramePoint3DReadOnly computeVRPPosition(double time, double omega, double gravity, DMatrixRMaj coefficientVector, MPCContactPlane contactPlane)
   {
      FramePoint3D dcmPosition = new FramePoint3D();
      CapturePointTools.computeCentroidalMomentumPivot(computeDCMPosition(time, omega, gravity, coefficientVector, contactPlane),
                                                    computeDCMVelocity(time, omega, gravity, coefficientVector, contactPlane),
                                                    omega,
                                                    dcmPosition);

      return dcmPosition;
   }

   public static DMatrixRMaj computeRhoValueVector(double time, double omega, DMatrixRMaj coefficientVector, MPCContactPlane contactPlane)
   {
      DMatrixRMaj rhoValueVector = new DMatrixRMaj(contactPlane.getRhoSize(), 1);
      CommonOps_DDRM.mult(getContactValueJacobian(time, omega, contactPlane), coefficientVector, rhoValueVector);

      return rhoValueVector;
   }

   public static DMatrixRMaj computeRhoAccelerationVector(double time, double omega, DMatrixRMaj coefficientVector, MPCContactPlane contactPlane)
   {
      DMatrixRMaj rhoValueVector = new DMatrixRMaj(contactPlane.getRhoSize(), 1);
      CommonOps_DDRM.mult(getContactAccelerationJacobian(time, omega, contactPlane), coefficientVector, rhoValueVector);

      return rhoValueVector;
   }

   public static DMatrixRMaj getCoMPositionJacobian(double time, double omega, MPCContactPlane contactPlane)
   {
      return getCoMPositionJacobian(time, omega, contactPlane.getRhoSize(), contactPlane::getBasisVector);
   }

   public static DMatrixRMaj getCoMPositionJacobian(double time, double omega, int rhoSize, IntFunction<FrameVector3DReadOnly> basisVectors)
   {
      int coefficients = LinearMPCIndexHandler.comCoefficientsPerSegment + LinearMPCIndexHandler.coefficientsPerRho * rhoSize;
      DMatrixRMaj jacobian = new DMatrixRMaj(3, coefficients);

      double c0 = time;
      double c1 = 1.0;
      double c2 = Math.exp(omega * time);
      double c3 = Math.exp(-omega * time);
      double c4 = time * time * time;
      double c5 = time * time;

      jacobian.set(0, 0, c0);
      jacobian.set(0, 1, c1);
      jacobian.set(1, 2, c0);
      jacobian.set(1, 3, c1);
      jacobian.set(2, 4, c0);
      jacobian.set(2, 5, c1);

      for (int rhoIdx  = 0; rhoIdx < rhoSize; rhoIdx++)
      {
         int startIdx = LinearMPCIndexHandler.comCoefficientsPerSegment + LinearMPCIndexHandler.coefficientsPerRho * rhoIdx;

         FrameVector3DReadOnly basisVector = basisVectors.apply(rhoIdx);
         jacobian.set(0, startIdx, basisVector.getX() * (c2));
         jacobian.set(1, startIdx, basisVector.getY() * (c2));
         jacobian.set(2, startIdx, basisVector.getZ() * (c2));

         jacobian.set(0, startIdx + 1, basisVector.getX() * (c3));
         jacobian.set(1, startIdx + 1, basisVector.getY() * (c3));
         jacobian.set(2, startIdx + 1, basisVector.getZ() * (c3));

         jacobian.set(0, startIdx + 2, basisVector.getX() * (c4));
         jacobian.set(1, startIdx + 2, basisVector.getY() * (c4));
         jacobian.set(2, startIdx + 2, basisVector.getZ() * (c4));

         jacobian.set(0, startIdx + 3, basisVector.getX() * (c5));
         jacobian.set(1, startIdx + 3, basisVector.getY() * (c5));
         jacobian.set(2, startIdx + 3, basisVector.getZ() * (c5));
      }

      return jacobian;
   }

   public static DMatrixRMaj getCoMVelocityJacobian(double time, double omega, ContactStateMagnitudeToForceMatrixHelper rhoHelper)
   {
      return getCoMVelocityJacobian(time, omega, rhoHelper.getRhoSize(), rhoHelper::getBasisVector);
   }

   public static DMatrixRMaj getCoMVelocityJacobian(double time, double omega, MPCContactPlane contactPlane)
   {
      return getCoMVelocityJacobian(time, omega, contactPlane.getRhoSize(), contactPlane::getBasisVector);
   }

   public static DMatrixRMaj getCoMVelocityJacobian(double time, double omega, int rhoSize, IntFunction<FrameVector3DReadOnly> basisVectors)
   {
      int coefficients = LinearMPCIndexHandler.comCoefficientsPerSegment + LinearMPCIndexHandler.coefficientsPerRho * rhoSize;
      DMatrixRMaj jacobian = new DMatrixRMaj(3, coefficients);

      double c0Dot = 1.0;
      double c1Dot = 0.0;
      double c2Dot = omega * Math.exp(omega * time);
      double c3Dot = -omega * Math.exp(-omega * time);
      double c4Dot = 3.0 * time * time;
      double c5Dot = 2.0 * time;

      jacobian.set(0, 0, c0Dot);
      jacobian.set(0, 1, c1Dot);
      jacobian.set(1, 2, c0Dot);
      jacobian.set(1, 3, c1Dot);
      jacobian.set(2, 4, c0Dot);
      jacobian.set(2, 5, c1Dot);

      for (int rhoIdx  = 0; rhoIdx < rhoSize; rhoIdx++)
      {
         int startIdx = 6 + 4 * rhoIdx;

         FrameVector3DReadOnly basisVector = basisVectors.apply(rhoIdx);
         jacobian.set(0, startIdx, basisVector.getX() * (c2Dot));
         jacobian.set(1, startIdx, basisVector.getY() * (c2Dot));
         jacobian.set(2, startIdx, basisVector.getZ() * (c2Dot));

         jacobian.set(0, startIdx + 1, basisVector.getX() * (c3Dot));
         jacobian.set(1, startIdx + 1, basisVector.getY() * (c3Dot));
         jacobian.set(2, startIdx + 1, basisVector.getZ() * (c3Dot));

         jacobian.set(0, startIdx + 2, basisVector.getX() * (c4Dot));
         jacobian.set(1, startIdx + 2, basisVector.getY() * (c4Dot));
         jacobian.set(2, startIdx + 2, basisVector.getZ() * (c4Dot));

         jacobian.set(0, startIdx + 3, basisVector.getX() * (c5Dot));
         jacobian.set(1, startIdx + 3, basisVector.getY() * (c5Dot));
         jacobian.set(2, startIdx + 3, basisVector.getZ() * (c5Dot));
      }

      return jacobian;
   }

   public static DMatrixRMaj getCoMAccelerationJacobian(double time, double omega, ContactStateMagnitudeToForceMatrixHelper rhoHelper)
   {
      return getCoMVelocityJacobian(time, omega, rhoHelper.getRhoSize(), rhoHelper::getBasisVector);
   }

   public static DMatrixRMaj getCoMAccelerationJacobian(double time, double omega, MPCContactPlane contact)
   {
      return getCoMVelocityJacobian(time, omega, contact.getRhoSize(), contact::getBasisVector);
   }

   public static DMatrixRMaj getCoMAccelerationJacobian(double time, double omega, int rhoSize, IntFunction<FrameVector3DReadOnly> basisVectors)
   {
      int coefficients = LinearMPCIndexHandler.comCoefficientsPerSegment + LinearMPCIndexHandler.coefficientsPerRho * rhoSize;
      DMatrixRMaj jacobian = new DMatrixRMaj(3, coefficients);

      double c0Ddot = 0.0;
      double c1Ddot = 0.0;
      double c2Ddot = omega * omega * Math.exp(omega * time);
      double c3Ddot = omega * omega * Math.exp(-omega * time);
      double c4Ddot = 6.0 * time;
      double c5Ddot = 2.0;

      jacobian.set(0, 0, c0Ddot);
      jacobian.set(0, 1, c1Ddot);
      jacobian.set(1, 2, c0Ddot);
      jacobian.set(1, 3, c1Ddot);
      jacobian.set(2, 4, c0Ddot);
      jacobian.set(2, 5, c1Ddot);

      for (int rhoIdx  = 0; rhoIdx < rhoSize; rhoIdx++)
      {
         int startIdx = 6 + 4 * rhoIdx;

         FrameVector3DReadOnly basisVector = basisVectors.apply(rhoIdx);
         jacobian.set(0, startIdx, basisVector.getX() * (c2Ddot));
         jacobian.set(1, startIdx, basisVector.getY() * (c2Ddot));
         jacobian.set(2, startIdx, basisVector.getZ() * (c2Ddot));

         jacobian.set(0, startIdx + 1, basisVector.getX() * (c3Ddot));
         jacobian.set(1, startIdx + 1, basisVector.getY() * (c3Ddot));
         jacobian.set(2, startIdx + 1, basisVector.getZ() * (c3Ddot));

         jacobian.set(0, startIdx + 2, basisVector.getX() * (c4Ddot));
         jacobian.set(1, startIdx + 2, basisVector.getY() * (c4Ddot));
         jacobian.set(2, startIdx + 2, basisVector.getZ() * (c4Ddot));

         jacobian.set(0, startIdx + 3, basisVector.getX() * (c5Ddot));
         jacobian.set(1, startIdx + 3, basisVector.getY() * (c5Ddot));
         jacobian.set(2, startIdx + 3, basisVector.getZ() * (c5Ddot));
      }

      return jacobian;
   }

   public static DMatrixRMaj getCoMJerkJacobian(double time, double omega, ContactStateMagnitudeToForceMatrixHelper rhoHelper)
   {
      int coefficients = LinearMPCIndexHandler.comCoefficientsPerSegment + LinearMPCIndexHandler.coefficientsPerRho * rhoHelper.getRhoSize();
      DMatrixRMaj jacobian = new DMatrixRMaj(3, coefficients);

      double c0Dddot = 0.0;
      double c1Dddot = 0.0;
      double c2Dddot = omega * omega * omega * Math.exp(omega * time);
      double c3Dddot = -omega * omega * omega * Math.exp(-omega * time);
      double c4Dddot = 6.0;
      double c5Dddot = 0.0;

      jacobian.set(0, 0, c0Dddot);
      jacobian.set(0, 1, c1Dddot);
      jacobian.set(1, 2, c0Dddot);
      jacobian.set(1, 3, c1Dddot);
      jacobian.set(2, 4, c0Dddot);
      jacobian.set(2, 5, c1Dddot);

      for (int rhoIdx  = 0; rhoIdx < rhoHelper.getRhoSize(); rhoIdx++)
      {
         int startIdx = 6 + 4 * rhoIdx;

         FrameVector3DReadOnly basisVector = rhoHelper.getBasisVector(rhoIdx);
         jacobian.set(0, startIdx, basisVector.getX() * (c2Dddot));
         jacobian.set(1, startIdx, basisVector.getY() * (c2Dddot));
         jacobian.set(2, startIdx, basisVector.getZ() * (c2Dddot));

         jacobian.set(0, startIdx + 1, basisVector.getX() * (c3Dddot));
         jacobian.set(1, startIdx + 1, basisVector.getY() * (c3Dddot));
         jacobian.set(2, startIdx + 1, basisVector.getZ() * (c3Dddot));

         jacobian.set(0, startIdx + 2, basisVector.getX() * (c4Dddot));
         jacobian.set(1, startIdx + 2, basisVector.getY() * (c4Dddot));
         jacobian.set(2, startIdx + 2, basisVector.getZ() * (c4Dddot));

         jacobian.set(0, startIdx + 3, basisVector.getX() * (c5Dddot));
         jacobian.set(1, startIdx + 3, basisVector.getY() * (c5Dddot));
         jacobian.set(2, startIdx + 3, basisVector.getZ() * (c5Dddot));
      }

      return jacobian;
   }

   public static DMatrixRMaj getContactTorqueJacobian(double mass, double time, double omega, FramePoint3DReadOnly point, MPCContactPlane contact)
   {
      int coefficients = contact.getCoefficientSize();
      DMatrixRMaj contactTorqueJacobian = new DMatrixRMaj(3, coefficients);
      DMatrixRMaj contactPointForceToTorqueJacobian = new DMatrixRMaj(3, 3 * contact.getNumberOfContactPoints());
      DMatrixRMaj coefficientsToForceJacobian = new DMatrixRMaj(3 * contact.getNumberOfContactPoints(), coefficients);

      ContactPlaneJacobianCalculator.computeContactPointAccelerationJacobian(mass, time, omega, 0, 0, contact, coefficientsToForceJacobian);

      for (int contactIdx = 0; contactIdx < contact.getNumberOfContactPoints(); contactIdx++)
      {
         MPCContactPoint contactPoint = contact.getContactPointHelper(contactIdx);
         FrameVector3D momentArm = new FrameVector3D();
         momentArm.sub(contactPoint.getBasisVectorOrigin(), point);
         DMatrixRMaj momentArmSkew = new DMatrixRMaj(3, 3);
         MatrixMissingTools.toSkewSymmetricMatrix(momentArm, momentArmSkew);
         MatrixTools.setMatrixBlock(contactPointForceToTorqueJacobian, 0, 3 * contactIdx, momentArmSkew, 0, 0, 3, 3, 1.0);
      }

      CommonOps_DDRM.mult(contactPointForceToTorqueJacobian, coefficientsToForceJacobian, contactTorqueJacobian);

      return contactTorqueJacobian;
   }
}
