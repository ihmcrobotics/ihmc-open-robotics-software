package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

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
      DMatrixRMaj jacobian = new DMatrixRMaj(3, 6 + 4 * rhoHelper.getRhoSize());

      double c0 = time;
      double c1 = 1.0;
      double c2 = Math.exp(omega * time);
      double c3 = Math.exp(-omega * time);
      double c4 = time * time * time;
      double c5 = time * time;

      double c0Dot = 1.0;
      double c1Dot = 0.0;
      double c2Dot = omega * Math.exp(omega * time);
      double c3Dot = -omega * Math.exp(-omega * time);
      double c4Dot = 3.0 * time * time;
      double c5Dot = 2.0 * time;

      jacobian.set(0, 0, c0 + c0Dot / omega);
      jacobian.set(0, 1, c1 + c1Dot / omega);
      jacobian.set(1, 2, c0 + c0Dot / omega);
      jacobian.set(1, 3, c1 + c1Dot / omega);
      jacobian.set(2, 4, c0 + c0Dot / omega);
      jacobian.set(2, 5, c1 + c1Dot / omega);

      for (int rhoIdx  = 0; rhoIdx < rhoHelper.getRhoSize(); rhoIdx++)
      {
         int startIdx = 6 + 4 * rhoIdx;

         FrameVector3DReadOnly basisVector = rhoHelper.getBasisVector(rhoIdx);
         jacobian.set(0, startIdx, basisVector.getX() * (c2 + c2Dot / omega));
         jacobian.set(1, startIdx, basisVector.getY() * (c2 + c2Dot / omega));
         jacobian.set(2, startIdx, basisVector.getZ() * (c2 + c2Dot / omega));

         jacobian.set(0, startIdx + 1, basisVector.getX() * (c3 + c3Dot / omega));
         jacobian.set(1, startIdx + 1, basisVector.getY() * (c3 + c3Dot / omega));
         jacobian.set(2, startIdx + 1, basisVector.getZ() * (c3 + c3Dot / omega));

         jacobian.set(0, startIdx + 2, basisVector.getX() * (c4 + c4Dot / omega));
         jacobian.set(1, startIdx + 2, basisVector.getY() * (c4 + c4Dot / omega));
         jacobian.set(2, startIdx + 2, basisVector.getZ() * (c4 + c4Dot / omega));

         jacobian.set(0, startIdx + 3, basisVector.getX() * (c5 + c5Dot / omega));
         jacobian.set(1, startIdx + 3, basisVector.getY() * (c5 + c5Dot / omega));
         jacobian.set(2, startIdx + 3, basisVector.getZ() * (c5 + c5Dot / omega));
      }

      return jacobian;
   }

   public static DMatrixRMaj getVRPPositionJacobian(double time, double omega, ContactStateMagnitudeToForceMatrixHelper rhoHelper)
   {
      DMatrixRMaj jacobian = new DMatrixRMaj(3, 6 + 4 * rhoHelper.getRhoSize());

      double omega2 = omega * omega;

      double c0 = time;
      double c1 = 1.0;
      double c2 = Math.exp(omega * time);
      double c3 = Math.exp(-omega * time);
      double c4 = time * time * time;
      double c5 = time * time;

      double c0Ddot = 0.0;
      double c1Ddot = 0.0;
      double c2Ddot = omega * omega * Math.exp(omega * time);
      double c3Ddot = omega * omega * Math.exp(-omega * time);
      double c4Ddot = 6.0 * time;
      double c5Ddot = 2.0;

      jacobian.set(0, 0, c0 - c0Ddot / omega2);
      jacobian.set(0, 1, c1 - c1Ddot / omega2);
      jacobian.set(1, 2, c0 - c0Ddot / omega2);
      jacobian.set(1, 3, c1 - c1Ddot / omega2);
      jacobian.set(2, 4, c0 - c0Ddot / omega2);
      jacobian.set(2, 5, c1 - c1Ddot / omega2);

      for (int rhoIdx  = 0; rhoIdx < rhoHelper.getRhoSize(); rhoIdx++)
      {
         int startIdx = 6 + 4 * rhoIdx;

         FrameVector3DReadOnly basisVector = rhoHelper.getBasisVector(rhoIdx);
         jacobian.set(0, startIdx, basisVector.getX() * (c2 - c2Ddot / omega2));
         jacobian.set(1, startIdx, basisVector.getY() * (c2 - c2Ddot / omega2));
         jacobian.set(2, startIdx, basisVector.getZ() * (c2 - c2Ddot / omega2));

         jacobian.set(0, startIdx + 1, basisVector.getX() * (c3 - c3Ddot / omega2));
         jacobian.set(1, startIdx + 1, basisVector.getY() * (c3 - c3Ddot / omega2));
         jacobian.set(2, startIdx + 1, basisVector.getZ() * (c3 - c3Ddot / omega2));

         jacobian.set(0, startIdx + 2, basisVector.getX() * (c4 - c4Ddot / omega2));
         jacobian.set(1, startIdx + 2, basisVector.getY() * (c4 - c4Ddot / omega2));
         jacobian.set(2, startIdx + 2, basisVector.getZ() * (c4 - c4Ddot / omega2));

         jacobian.set(0, startIdx + 3, basisVector.getX() * (c5 - c5Ddot / omega2));
         jacobian.set(1, startIdx + 3, basisVector.getY() * (c5 - c5Ddot / omega2));
         jacobian.set(2, startIdx + 3, basisVector.getZ() * (c5 - c5Ddot / omega2));
      }


      return jacobian;
   }

   public static DMatrixRMaj getVRPVelocityJacobian(double time, double omega, ContactStateMagnitudeToForceMatrixHelper rhoHelper)
   {
      DMatrixRMaj jacobian = new DMatrixRMaj(3, 6 + 4 * rhoHelper.getRhoSize());

      double omega2 = omega * omega;
      double c0Dot = 1.0;
      double c1Dot = 0.0;
      double c2Dot = omega * Math.exp(omega * time);
      double c3Dot = -omega * Math.exp(-omega * time);
      double c4Dot = 3.0 * time * time;
      double c5Dot = 2.0 * time;

      double c0Dddot = 0.0;
      double c1Dddot = 0.0;
      double c2Dddot = omega * omega * omega * Math.exp(omega * time);
      double c3Dddot = -omega * omega * omega * Math.exp(-omega * time);
      double c4Dddot = 6.0;
      double c5Dddot = 0.0;

      jacobian.set(0, 0, c0Dot - c0Dddot / omega2);
      jacobian.set(0, 1, c1Dot - c1Dddot / omega2);
      jacobian.set(1, 2, c0Dot - c0Dddot / omega2);
      jacobian.set(1, 3, c1Dot - c1Dddot / omega2);
      jacobian.set(2, 4, c0Dot - c0Dddot / omega2);
      jacobian.set(2, 5, c1Dot - c1Dddot / omega2);

      for (int rhoIdx  = 0; rhoIdx < rhoHelper.getRhoSize(); rhoIdx++)
      {
         int startIdx = 6 + 4 * rhoIdx;

         FrameVector3DReadOnly basisVector = rhoHelper.getBasisVector(rhoIdx);
         jacobian.set(0, startIdx, basisVector.getX() * (c2Dot - c2Dddot / omega2));
         jacobian.set(1, startIdx, basisVector.getY() * (c2Dot - c2Dddot / omega2));
         jacobian.set(2, startIdx, basisVector.getZ() * (c2Dot - c2Dddot / omega2));

         jacobian.set(0, startIdx + 1, basisVector.getX() * (c3Dot - c3Dddot / omega2));
         jacobian.set(1, startIdx + 1, basisVector.getY() * (c3Dot - c3Dddot / omega2));
         jacobian.set(2, startIdx + 1, basisVector.getZ() * (c3Dot - c3Dddot / omega2));

         jacobian.set(0, startIdx + 2, basisVector.getX() * (c4Dot - c4Dddot / omega2));
         jacobian.set(1, startIdx + 2, basisVector.getY() * (c4Dot - c4Dddot / omega2));
         jacobian.set(2, startIdx + 2, basisVector.getZ() * (c4Dot - c4Dddot / omega2));

         jacobian.set(0, startIdx + 3, basisVector.getX() * (c5Dot - c5Dddot / omega2));
         jacobian.set(1, startIdx + 3, basisVector.getY() * (c5Dot - c5Dddot / omega2));
         jacobian.set(2, startIdx + 3, basisVector.getZ() * (c5Dot - c5Dddot / omega2));
      }

      return jacobian;
   }

   public static DMatrixRMaj getCoMPositionJacobian(double time, double omega, ContactStateMagnitudeToForceMatrixHelper rhoHelper)
   {
      DMatrixRMaj jacobian = new DMatrixRMaj(3, 6 + 4 * rhoHelper.getRhoSize());

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
      DMatrixRMaj jacobian = new DMatrixRMaj(3, 6 + 4 * rhoHelper.getRhoSize());

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
}
