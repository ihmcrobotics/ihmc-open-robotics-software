package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

import static us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCQPInputCalculator.sufficientlyLargeValue;

public class ContactPlaneJacobianCalculator
{
   public static void computeLinearJacobian(int derivativeOrder,
                                        double time,
                                        double omega,
                                        int startColumn,
                                        ContactPlaneHelper contactPlane,
                                        DMatrixRMaj jacobianToPack)
   {
      switch (derivativeOrder)
      {
         case 0:
            computeLinearPositionJacobian(time, omega, startColumn, contactPlane, jacobianToPack);
            return;
         case 1:
            computeLinearVelocityJacobian(time, omega, startColumn, contactPlane, jacobianToPack);
            return;
         case 2:
            computeLinearAccelerationJacobian(time, omega, startColumn, contactPlane, jacobianToPack);
            return;
         case 3:
            computeLinearJerkJacobian(time, omega, startColumn, contactPlane, jacobianToPack);
            return;
         default:
            throw new IllegalArgumentException("Derivative order must be less than 4.");
      }
   }

   public static void computeLinearPositionJacobian(double time,
                                                    double omega,
                                                    int startColumn,
                                                    ContactPlaneHelper contactPlane,
                                                    DMatrixRMaj positionJacobianToPack)
   {
      double t2 = time * time;
      double t3 = time * t2;
      double positiveExponential = Math.min(Math.exp(omega * time), sufficientlyLargeValue);
      double negativeExponential = 1.0 / positiveExponential;

      setLinearJacobianCoefficients(contactPlane, positiveExponential, negativeExponential, t3, t2, startColumn, positionJacobianToPack);
   }

   public static void computeLinearVelocityJacobian(double time,
                                                    double omega,
                                                    int startColumn,
                                                    ContactPlaneHelper contactPlane,
                                                    DMatrixRMaj velocityJacobianToPack)
   {
      double positiveExponential = Math.min(Math.exp(omega * time), sufficientlyLargeValue);
      double firstVelocityCoefficient = omega * positiveExponential;
      double secondVelocityCoefficient = -omega / positiveExponential;
      double thirdVelocityCoefficient = 3 * time * time;
      double fourthVelocityCoefficient = 2 * time;

      setLinearJacobianCoefficients(contactPlane,
                                    firstVelocityCoefficient,
                                    secondVelocityCoefficient,
                                    thirdVelocityCoefficient,
                                    fourthVelocityCoefficient,
                                    startColumn,
                                    velocityJacobianToPack);
   }

   public static void computeLinearAccelerationJacobian(double time,
                                                        double omega,
                                                        int startColumn,
                                                        ContactPlaneHelper contactPlane,
                                                        DMatrixRMaj accelerationJacobianToPack)
   {
      double omega2 = omega * omega;
      double positiveExponential = Math.min(Math.exp(omega * time), sufficientlyLargeValue);
      double firstVelocityCoefficient = omega2 * positiveExponential;
      double secondVelocityCoefficient = omega2 / positiveExponential;
      double thirdVelocityCoefficient = 6 * time;
      double fourthVelocityCoefficient = 2;

      setLinearJacobianCoefficients(contactPlane,
                                    firstVelocityCoefficient,
                                    secondVelocityCoefficient,
                                    thirdVelocityCoefficient,
                                    fourthVelocityCoefficient,
                                    startColumn,
                                    accelerationJacobianToPack);
   }

   public static void computeLinearJerkJacobian(double time, double omega, int startColumn, ContactPlaneHelper contactPlane, DMatrixRMaj jerkJacobianToPack)
   {
      double omega3 = omega * omega * omega;
      double positiveExponential = Math.min(Math.exp(omega * time), sufficientlyLargeValue);
      double firstVelocityCoefficient = omega3 * positiveExponential;
      double secondVelocityCoefficient = -omega3 / positiveExponential;
      double thirdVelocityCoefficient = 6;
      double fourthVelocityCoefficient = 0.0;

      setLinearJacobianCoefficients(contactPlane,
                                    firstVelocityCoefficient,
                                    secondVelocityCoefficient,
                                    thirdVelocityCoefficient,
                                    fourthVelocityCoefficient,
                                    startColumn,
                                    jerkJacobianToPack);
   }

   private static void setLinearJacobianCoefficients(ContactPlaneHelper contactPlane,
                                                     double firstCoefficient,
                                                     double secondCoefficient,
                                                     double thirdCoefficient,
                                                     double fourthCoefficient,
                                                     int startColumn,
                                                     DMatrixRMaj jacobianToPack)
   {
      for (int contactPointIdx = 0; contactPointIdx < contactPlane.getNumberOfContactPoints(); contactPointIdx++)
      {
         ContactPointHelper contactPoint = contactPlane.getContactPointHelper(contactPointIdx);

         for (int rhoIdx = 0; rhoIdx < contactPoint.getRhoSize(); rhoIdx++)
         {
            FrameVector3DReadOnly basisVector = contactPoint.getBasisVector(rhoIdx);
            basisVector.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

            for (int ordinal = 0; ordinal < 3; ordinal++)
            {
               jacobianToPack.unsafe_set(ordinal, startColumn, basisVector.getElement(ordinal) * firstCoefficient);
               jacobianToPack.unsafe_set(ordinal, startColumn + 1, basisVector.getElement(ordinal) * secondCoefficient);

               jacobianToPack.unsafe_set(ordinal, startColumn + 2, basisVector.getElement(ordinal) * thirdCoefficient);
               jacobianToPack.unsafe_set(ordinal, startColumn + 3, basisVector.getElement(ordinal) * fourthCoefficient);
            }

            startColumn += LinearMPCIndexHandler.coefficientsPerRho;
         }
      }
   }
}
