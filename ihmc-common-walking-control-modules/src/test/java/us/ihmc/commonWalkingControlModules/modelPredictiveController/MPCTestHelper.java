package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.MatrixTools;

public class MPCTestHelper
{
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
      DMatrixRMaj jacobian = getCoMPositionJacobian(time, omega, rhoHelper);
      CommonOps_DDRM.addEquals(jacobian, 1.0 / omega, getCoMVelocityJacobian(time, omega, rhoHelper));

      return jacobian;
   }

   public static DMatrixRMaj getVRPPositionJacobian(double time, double omega, ContactStateMagnitudeToForceMatrixHelper rhoHelper)
   {
      DMatrixRMaj jacobian = getCoMPositionJacobian(time, omega, rhoHelper);
      CommonOps_DDRM.addEquals(jacobian, -1.0 / (omega * omega), getCoMAccelerationJacobian(time, omega, rhoHelper));

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

   public static DMatrixRMaj getCoMPositionJacobian(double time, double omega, ContactStateMagnitudeToForceMatrixHelper rhoHelper)
   {
      int coefficients = MPCIndexHandler.comCoefficientsPerSegment + MPCIndexHandler.coefficientsPerRho * rhoHelper.getRhoSize() + 3 * MPCIndexHandler.orientationCoefficientsPerSegment;
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

      for (int rhoIdx  = 0; rhoIdx < rhoHelper.getRhoSize(); rhoIdx++)
      {
         int startIdx = 6 + 4 * rhoIdx;

         FrameVector3DReadOnly basisVector = rhoHelper.getBasisVector(rhoIdx);
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
      int coefficients = MPCIndexHandler.comCoefficientsPerSegment + MPCIndexHandler.coefficientsPerRho * rhoHelper.getRhoSize() + 3 * MPCIndexHandler.orientationCoefficientsPerSegment;
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

      for (int rhoIdx  = 0; rhoIdx < rhoHelper.getRhoSize(); rhoIdx++)
      {
         int startIdx = 6 + 4 * rhoIdx;

         FrameVector3DReadOnly basisVector = rhoHelper.getBasisVector(rhoIdx);
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
      int coefficients = MPCIndexHandler.comCoefficientsPerSegment + MPCIndexHandler.coefficientsPerRho * rhoHelper.getRhoSize() + 3 * MPCIndexHandler.orientationCoefficientsPerSegment;
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

      for (int rhoIdx  = 0; rhoIdx < rhoHelper.getRhoSize(); rhoIdx++)
      {
         int startIdx = 6 + 4 * rhoIdx;

         FrameVector3DReadOnly basisVector = rhoHelper.getBasisVector(rhoIdx);
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
      int coefficients = MPCIndexHandler.comCoefficientsPerSegment + MPCIndexHandler.coefficientsPerRho * rhoHelper.getRhoSize() + 3 * MPCIndexHandler.orientationCoefficientsPerSegment;
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
}
