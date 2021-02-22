package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPoint;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

import static us.ihmc.commonWalkingControlModules.modelPredictiveController.core.MPCQPInputCalculator.sufficientlyLargeValue;

public class ContactPlaneJacobianCalculator
{
   public static void computeLinearJacobian(int derivativeOrder,
                                            double time,
                                            double omega,
                                            int startColumn,
                                            MPCContactPlane contactPlane,
                                            DMatrixRMaj jacobianToPack)
   {
      computeLinearJacobian(1.0, derivativeOrder, time, omega, startColumn, contactPlane, jacobianToPack);
   }

   public static void computeLinearJacobian(double scale,
                                            int derivativeOrder,
                                            double time,
                                            double omega,
                                            int startColumn,
                                            MPCContactPlane contactPlane,
                                            DMatrixRMaj jacobianToPack)
   {
      switch (derivativeOrder)
      {
         case 0:
            computeLinearPositionJacobian(scale, time, omega, startColumn, contactPlane, jacobianToPack);
            return;
         case 1:
            computeLinearVelocityJacobian(scale, time, omega, startColumn, contactPlane, jacobianToPack);
            return;
         case 2:
            computeLinearAccelerationJacobian(scale, time, omega, startColumn, contactPlane, jacobianToPack);
            return;
         case 3:
            computeLinearJerkJacobian(scale, time, omega, startColumn, contactPlane, jacobianToPack);
            return;
         default:
            throw new IllegalArgumentException("Derivative order must be less than 4.");
      }
   }

   public static void computeLinearPositionJacobian(double scale,
                                                    double time,
                                                    double omega,
                                                    int startColumn,
                                                    MPCContactPlane contactPlane,
                                                    DMatrixRMaj positionJacobianToPack)
   {
      double t2 = scale * time * time;
      double t3 = time * t2;
      double positiveExponential = Math.min(Math.exp(omega * time), sufficientlyLargeValue);
      double negativeExponential = scale / positiveExponential;

      addLinearJacobianCoefficients(contactPlane, scale * positiveExponential, negativeExponential, t3, t2, startColumn, positionJacobianToPack);
   }

   public static void computeLinearVelocityJacobian(double scale,
                                                    double time,
                                                    double omega,
                                                    int startColumn,
                                                    MPCContactPlane contactPlane,
                                                    DMatrixRMaj velocityJacobianToPack)
   {
      double scaleOmega = scale * omega;
      double positiveExponential = Math.min(Math.exp(omega * time), sufficientlyLargeValue);
      double firstVelocityCoefficient = scaleOmega * positiveExponential;
      double secondVelocityCoefficient = -scaleOmega / positiveExponential;
      double thirdVelocityCoefficient = scale * 3 * time * time;
      double fourthVelocityCoefficient = scale * 2 * time;

      addLinearJacobianCoefficients(contactPlane,
                                    firstVelocityCoefficient,
                                    secondVelocityCoefficient,
                                    thirdVelocityCoefficient,
                                    fourthVelocityCoefficient,
                                    startColumn,
                                    velocityJacobianToPack);
   }

   public static void computeLinearAccelerationJacobian(double scale,
                                                        double time,
                                                        double omega,
                                                        int startColumn,
                                                        MPCContactPlane contactPlane,
                                                        DMatrixRMaj accelerationJacobianToPack)
   {
      double scaleOmega2 = scale * omega * omega;
      double positiveExponential = Math.min(Math.exp(omega * time), sufficientlyLargeValue);
      double firstVelocityCoefficient = scaleOmega2 * positiveExponential;
      double secondVelocityCoefficient = scaleOmega2 / positiveExponential;
      double thirdVelocityCoefficient = scale * 6 * time;
      double fourthVelocityCoefficient = scale * 2;

      addLinearJacobianCoefficients(contactPlane,
                                    firstVelocityCoefficient,
                                    secondVelocityCoefficient,
                                    thirdVelocityCoefficient,
                                    fourthVelocityCoefficient,
                                    startColumn,
                                    accelerationJacobianToPack);
   }

   public static void computeLinearJerkJacobian(double scale,
                                                double time,
                                                double omega,
                                                int startColumn,
                                                MPCContactPlane contactPlane,
                                                DMatrixRMaj jerkJacobianToPack)
   {
      double scaleOmega3 = scale * omega * omega * omega;
      double positiveExponential = Math.min(Math.exp(omega * time), sufficientlyLargeValue);
      double firstVelocityCoefficient = scaleOmega3 * positiveExponential;
      double secondVelocityCoefficient = -scaleOmega3 / positiveExponential;
      double thirdVelocityCoefficient = scale * 6;
      double fourthVelocityCoefficient = 0.0;

      addLinearJacobianCoefficients(contactPlane,
                                    firstVelocityCoefficient,
                                    secondVelocityCoefficient,
                                    thirdVelocityCoefficient,
                                    fourthVelocityCoefficient,
                                    startColumn,
                                    jerkJacobianToPack);
   }

   private static void addLinearJacobianCoefficients(MPCContactPlane contactPlane,
                                                     double firstCoefficient,
                                                     double secondCoefficient,
                                                     double thirdCoefficient,
                                                     double fourthCoefficient,
                                                     int startColumn,
                                                     DMatrixRMaj jacobianToPack)
   {
      for (int contactPointIdx = 0; contactPointIdx < contactPlane.getNumberOfContactPoints(); contactPointIdx++)
      {
         MPCContactPoint contactPoint = contactPlane.getContactPointHelper(contactPointIdx);

         for (int rhoIdx = 0; rhoIdx < contactPoint.getRhoSize(); rhoIdx++)
         {
            FrameVector3DReadOnly basisVector = contactPoint.getBasisVector(rhoIdx);
            basisVector.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

            for (int ordinal = 0; ordinal < 3; ordinal++)
            {
               unsafe_add(jacobianToPack, ordinal, startColumn, basisVector.getElement(ordinal) * firstCoefficient);
               unsafe_add(jacobianToPack, ordinal, startColumn + 1, basisVector.getElement(ordinal) * secondCoefficient);

               unsafe_add(jacobianToPack, ordinal, startColumn + 2, basisVector.getElement(ordinal) * thirdCoefficient);
               unsafe_add(jacobianToPack, ordinal, startColumn + 3, basisVector.getElement(ordinal) * fourthCoefficient);
            }

            startColumn += LinearMPCIndexHandler.coefficientsPerRho;
         }
      }
   }

   private static void unsafe_add(DMatrixRMaj matrix, int row, int col, double value)
   {
      matrix.unsafe_set(row, col, value + matrix.unsafe_get(row, col));
   }

   public static void computeRhoJacobian(int derivativeOrder,
                                         double time,
                                         double omega,
                                         int startRow,
                                         int startColumn,
                                         MPCContactPlane contactPlane,
                                         DMatrixRMaj jacobianToPack)
   {
      computeRhoJacobian(1.0, derivativeOrder, time, omega, startRow, startColumn, contactPlane, jacobianToPack);
   }

   public static void computeRhoJacobian(double scale,
                                         int derivativeOrder,
                                         double time,
                                         double omega,
                                         int startRow,
                                         int startColumn,
                                         MPCContactPlane contactPlane,
                                         DMatrixRMaj jacobianToPack)
   {
      switch (derivativeOrder)
      {
         case 0:
            computeRhoMagnitudeJacobian(scale, time, omega, startRow, startColumn, contactPlane, jacobianToPack);
            return;
         case 1:
            computeRhoRateJacobian(scale, time, omega, startRow, startColumn, contactPlane, jacobianToPack);
            return;
         case 2:
            computeRhoAccelerationJacobian(scale, time, omega, startRow, startColumn, contactPlane, jacobianToPack);
            return;
         case 3:
            computeRhoJerkJacobian(scale, time, omega, startRow, startColumn, contactPlane, jacobianToPack);
            return;
         default:
            throw new IllegalArgumentException("Derivative order must be less than 4.");
      }
   }

   public static void computeRhoMagnitudeJacobian(double scale,
                                                  double time,
                                                  double omega,
                                                  int startRow,
                                                  int startColumn,
                                                  MPCContactPlane contactPlane,
                                                  DMatrixRMaj positionJacobianToPack)
   {
      double t2 = scale * time * time;
      double t3 = time * t2;
      double positiveExponential = Math.min(Math.exp(omega * time), sufficientlyLargeValue);
      double negativeExponential = scale / positiveExponential;

      setRhoJacobianCoefficients(contactPlane, scale * positiveExponential, negativeExponential, t3, t2, startRow, startColumn, positionJacobianToPack);
   }

   public static void computeRhoRateJacobian(double scale,
                                             double time,
                                             double omega,
                                             int startRow,
                                             int startColumn,
                                             MPCContactPlane contactPlane,
                                             DMatrixRMaj velocityJacobianToPack)
   {
      double positiveExponential = Math.min(Math.exp(omega * time), sufficientlyLargeValue);
      double scaleOmega = scale * omega;
      double firstVelocityCoefficient = scaleOmega * positiveExponential;
      double secondVelocityCoefficient = -scaleOmega / positiveExponential;
      double thirdVelocityCoefficient = scale * 3 * time * time;
      double fourthVelocityCoefficient = scale * 2 * time;

      setRhoJacobianCoefficients(contactPlane,
                                 firstVelocityCoefficient,
                                 secondVelocityCoefficient,
                                 thirdVelocityCoefficient,
                                 fourthVelocityCoefficient,
                                 startRow,
                                 startColumn,
                                 velocityJacobianToPack);
   }

   public static void computeRhoAccelerationJacobian(double scale,
                                                     double time,
                                                     double omega,
                                                     int startRow,
                                                     int startColumn,
                                                     MPCContactPlane contactPlane,
                                                     DMatrixRMaj accelerationJacobianToPack)
   {
      double scaleOmega2 = scale * omega * omega;
      double positiveExponential = Math.min(Math.exp(omega * time), sufficientlyLargeValue);
      double firstVelocityCoefficient = scaleOmega2 * positiveExponential;
      double secondVelocityCoefficient = scaleOmega2 / positiveExponential;
      double thirdVelocityCoefficient = scale * 6 * time;
      double fourthVelocityCoefficient = scale * 2;

      setRhoJacobianCoefficients(contactPlane,
                                 firstVelocityCoefficient,
                                 secondVelocityCoefficient,
                                 thirdVelocityCoefficient,
                                 fourthVelocityCoefficient,
                                 startRow,
                                 startColumn,
                                 accelerationJacobianToPack);
   }

   public static void computeRhoJerkJacobian(double scale,
                                             double time,
                                             double omega,
                                             int startRow,
                                             int startColumn,
                                             MPCContactPlane contactPlane,
                                             DMatrixRMaj jerkJacobianToPack)
   {
      double scaleOmega3 = scale * omega * omega * omega;
      double positiveExponential = Math.min(Math.exp(omega * time), sufficientlyLargeValue);
      double firstVelocityCoefficient = scaleOmega3 * positiveExponential;
      double secondVelocityCoefficient = -scaleOmega3 / positiveExponential;
      double thirdVelocityCoefficient = scale * 6;
      double fourthVelocityCoefficient = 0.0;

      setRhoJacobianCoefficients(contactPlane,
                                 firstVelocityCoefficient,
                                 secondVelocityCoefficient,
                                 thirdVelocityCoefficient,
                                 fourthVelocityCoefficient,
                                 startRow,
                                 startColumn,
                                 jerkJacobianToPack);
   }

   private static void setRhoJacobianCoefficients(MPCContactPlane contactPlane,
                                                  double firstCoefficient,
                                                  double secondCoefficient,
                                                  double thirdCoefficient,
                                                  double fourthCoefficient,
                                                  int rowStart,
                                                  int columnStart,
                                                  DMatrixRMaj rhoJacobianToPack)
   {
      for (int contactPointIdx = 0; contactPointIdx < contactPlane.getNumberOfContactPoints(); contactPointIdx++)
      {
         MPCContactPoint contactPoint = contactPlane.getContactPointHelper(contactPointIdx);

         for (int basisVectorIndex = 0; basisVectorIndex < contactPoint.getRhoSize(); basisVectorIndex++)
         {
            rhoJacobianToPack.unsafe_set(rowStart + basisVectorIndex, columnStart, firstCoefficient);
            rhoJacobianToPack.unsafe_set(rowStart + basisVectorIndex, columnStart + 1, secondCoefficient);

            rhoJacobianToPack.unsafe_set(rowStart + basisVectorIndex, columnStart + 2, thirdCoefficient);
            rhoJacobianToPack.unsafe_set(rowStart + basisVectorIndex, columnStart + 3, fourthCoefficient);

            columnStart += LinearMPCIndexHandler.coefficientsPerRho;
         }

         rowStart += contactPoint.getRhoSize();
      }
   }

   public static void computeContactPointJacobian(double scale,
                                         int derivativeOrder,
                                         double time,
                                         double omega,
                                         int startRow,
                                         int startColumn,
                                         MPCContactPlane contactPlane,
                                         DMatrixRMaj jacobianToPack)
   {
      switch (derivativeOrder)
      {
         case 0:
            computeContactPointMagnitudeJacobian(scale, time, omega, startRow, startColumn, contactPlane, jacobianToPack);
            return;
         case 1:
            computeContactPointRateJacobian(scale, time, omega, startRow, startColumn, contactPlane, jacobianToPack);
            return;
         case 2:
            computeContactPointAccelerationJacobian(scale, time, omega, startRow, startColumn, contactPlane, jacobianToPack);
            return;
         case 3:
            computeContactPointJerkJacobian(scale, time, omega, startRow, startColumn, contactPlane, jacobianToPack);
            return;
         default:
            throw new IllegalArgumentException("Derivative order must be less than 4.");
      }
   }

   public static void computeContactPointMagnitudeJacobian(double scale,
                                                  double time,
                                                  double omega,
                                                  int startRow,
                                                  int startColumn,
                                                  MPCContactPlane contactPlane,
                                                  DMatrixRMaj positionJacobianToPack)
   {
      double t2 = scale * time * time;
      double t3 = time * t2;
      double positiveExponential = Math.min(Math.exp(omega * time), sufficientlyLargeValue);
      double negativeExponential = scale / positiveExponential;

      setContactPointJacobianCoefficients(contactPlane, scale * positiveExponential, negativeExponential, t3, t2, startRow, startColumn, positionJacobianToPack);
   }

   public static void computeContactPointRateJacobian(double scale,
                                             double time,
                                             double omega,
                                             int startRow,
                                             int startColumn,
                                             MPCContactPlane contactPlane,
                                             DMatrixRMaj velocityJacobianToPack)
   {
      double positiveExponential = Math.min(Math.exp(omega * time), sufficientlyLargeValue);
      double scaleOmega = scale * omega;
      double firstVelocityCoefficient = scaleOmega * positiveExponential;
      double secondVelocityCoefficient = -scaleOmega / positiveExponential;
      double thirdVelocityCoefficient = scale * 3 * time * time;
      double fourthVelocityCoefficient = scale * 2 * time;

      setContactPointJacobianCoefficients(contactPlane,
                                          firstVelocityCoefficient,
                                          secondVelocityCoefficient,
                                          thirdVelocityCoefficient,
                                          fourthVelocityCoefficient,
                                          startRow,
                                          startColumn,
                                          velocityJacobianToPack);
   }

   public static void computeContactPointAccelerationJacobian(double scale,
                                                         double time,
                                                         double omega,
                                                         int startRow,
                                                         int startColumn,
                                                         MPCContactPlane contactPlane,
                                                         DMatrixRMaj accelerationJacobianToPack)
   {
      double scaleOmega2 = scale * omega * omega;
      double positiveExponential = Math.min(Math.exp(omega * time), sufficientlyLargeValue);
      double firstAccelerationCoefficient = scaleOmega2 * positiveExponential;
      double secondAccelerationCoefficient = scaleOmega2 / positiveExponential;
      double thirdAccelerationCoefficient = scale * 6 * time;
      double fourthAccelerationCoefficient = scale * 2;

      setContactPointJacobianCoefficients(contactPlane,
                                          firstAccelerationCoefficient,
                                          secondAccelerationCoefficient,
                                          thirdAccelerationCoefficient,
                                          fourthAccelerationCoefficient,
                                          startRow,
                                          startColumn,
                                          accelerationJacobianToPack);
   }

   public static void computeContactPointJerkJacobian(double scale,
                                                 double time,
                                                 double omega,
                                                 int startRow,
                                                 int startColumn,
                                                 MPCContactPlane contactPlane,
                                                 DMatrixRMaj jerkJacobianToPack)
   {
      double scaleOmega3 = scale * omega * omega * omega;
      double positiveExponential = Math.min(Math.exp(omega * time), sufficientlyLargeValue);
      double firstJerkCoefficient = scaleOmega3 * positiveExponential;
      double secondJerkCoefficient = -scaleOmega3 / positiveExponential;
      double thirdJerkCoefficient = scale * 6;
      double fourthJerkCoefficient = 0.0;

      setContactPointJacobianCoefficients(contactPlane,
                                          firstJerkCoefficient,
                                          secondJerkCoefficient,
                                          thirdJerkCoefficient,
                                          fourthJerkCoefficient,
                                          startRow,
                                          startColumn,
                                          jerkJacobianToPack);
   }


   private static void setContactPointJacobianCoefficients(MPCContactPlane contactPlane,
                                                           double firstCoefficient,
                                                           double secondCoefficient,
                                                           double thirdCoefficient,
                                                           double fourthCoefficient,
                                                           int rowStart,
                                                           int columnStart,
                                                           DMatrixRMaj contactForceJacobianToPack)
   {
      for (int contactPointIdx = 0; contactPointIdx < contactPlane.getNumberOfContactPoints(); contactPointIdx++)
      {
         MPCContactPoint contactPoint = contactPlane.getContactPointHelper(contactPointIdx);

         for (int basisVectorIndex = 0; basisVectorIndex < contactPoint.getRhoSize(); basisVectorIndex++)
         {
            FrameVector3DReadOnly basisVector = contactPoint.getBasisVector(basisVectorIndex);
            basisVector.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

            for (int element = 0; element < 3; element++)
            {
               contactForceJacobianToPack.unsafe_set(rowStart + element, columnStart, basisVector.getElement(element) * firstCoefficient);
               contactForceJacobianToPack.unsafe_set(rowStart + element, columnStart + 1, basisVector.getElement(element) * secondCoefficient);

               contactForceJacobianToPack.unsafe_set(rowStart + element, columnStart + 2, basisVector.getElement(element) * thirdCoefficient);
               contactForceJacobianToPack.unsafe_set(rowStart + element, columnStart + 3, basisVector.getElement(element) * fourthCoefficient);
            }

            columnStart += LinearMPCIndexHandler.coefficientsPerRho;
         }

         rowStart += 3 * contactPointIdx;
      }
   }
}
