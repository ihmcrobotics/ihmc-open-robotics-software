package us.ihmc.exampleSimulations.simpleMPCTouchdownCalculatorTesting.MPC;

import org.ejml.data.DMatrixRMaj;

import static us.ihmc.exampleSimulations.simpleMPCTouchdownCalculatorTesting.MPC.AutomaticFootstepMPCMatrixCalculator.STATE_SIZE;

public class FootstepMPCMatrixTools
{
   public static void computeJumpMatrices(DMatrixRMaj stateJump, DMatrixRMaj controlJump)
   {
      stateJump.zero();
      stateJump.set(2, 2, 1.0);
      stateJump.set(3, 3, 1.0);

      controlJump.zero();
      controlJump.set(0, 0, -1.0);
      controlJump.set(1, 1, -1.0);
   }

   public static void computeStateTransitionMatrix(double omegaX,
                                                   double omegaY,
                                                   double duration,
                                                   double massComZUpXOmegaX,
                                                   double massComZUpYOmegaY,
                                                   DMatrixRMaj resultDiscreteMatrix)
   {
      double coshXT = Math.cosh(omegaX * duration);
      double sinhXT = Math.sinh(omegaX * duration);
      double coshYT = Math.cosh(omegaY * duration);
      double sinhYT = Math.sinh(omegaY * duration);

      resultDiscreteMatrix.zero();
      resultDiscreteMatrix.set(0, 0, coshXT);
      resultDiscreteMatrix.set(0, 3, sinhXT / massComZUpXOmegaX);

      resultDiscreteMatrix.set(1, 1, coshYT);
      resultDiscreteMatrix.set(1, 2, -sinhYT / massComZUpYOmegaY);

      resultDiscreteMatrix.set(2, 1, -massComZUpYOmegaY * sinhYT);
      resultDiscreteMatrix.set(2, 2, coshYT);

      resultDiscreteMatrix.set(3, 0, massComZUpXOmegaX * sinhXT);
      resultDiscreteMatrix.set(3, 3, coshXT);
   }

   /**
    * Calculating the rotating future footstep with the given theta.
    * Matrix size is 4 x 4
    *
    * @param theta0               : is the yaw rotation angle of the next foot compare to the current foot.
    * @param rotationMatrixOutput : output rotation matrix which is calculated with &theta <sub>0</sub>
    */
   public static void computeFootRotationMatrix(double theta0, DMatrixRMaj rotationMatrixOutput)
   {
      rotationMatrixOutput.zero();
      computeCompactFootRotationMatrix(theta0, 0, 0, rotationMatrixOutput);
      computeCompactFootRotationMatrix(theta0, 2, 2, rotationMatrixOutput);
   }

   public static void computeCompactFootRotationMatrix(double theta0, int startRow, int startCol, DMatrixRMaj rotationMatrixOutput)
   {
      rotationMatrixOutput.set(startRow, startCol, Math.cos(theta0));
      rotationMatrixOutput.set(startRow, startCol + 1, -Math.sin(theta0));
      rotationMatrixOutput.set(startRow + 1, startCol, Math.sin(theta0));
      rotationMatrixOutput.set(startRow + 1, startCol + 1, Math.cos(theta0));
   }

   /**
    * calculate the rotation matrix of two-step ahead.
    * The matrix will be multiplied to desired future footstep position f<sub>k,x</sub>, f<sub>k,y</sub>, f<sub>k+1,x</sub>, f<sub>k+1,y</sub><br>
    *
    * @param firstFootAngle  yaw angle of the first footstep
    * @param secondFootAngle yaw angle of the second footstep
    */
   public static void computeFutureFootRotationMatrix(double firstFootAngle, double secondFootAngle, DMatrixRMaj futureFootStepsRotatingMatrixToPack)
   {
      //TODO it
      //Rotation matrix will be multiplied to only the desired footstep so below methods could be neglected
      //Because, the desired footstep ahead 2 step includes only x-y position. not includes the angular momentum term
      //For now, the first rotation and second rotation are independent.
      futureFootStepsRotatingMatrixToPack.zero();
      computeCompactFootRotationMatrix(Math.toRadians(firstFootAngle), 0, 0, futureFootStepsRotatingMatrixToPack);
      computeCompactFootRotationMatrix(Math.toRadians(secondFootAngle), 2, 2, futureFootStepsRotatingMatrixToPack);
   }

   public static void computeCoMPositionSelectionMatrix(int numberOfSteps, DMatrixRMaj positionSelectionMatrix)
   {
      positionSelectionMatrix.zero();
      int rowStart = 0;
      int colStart = 0;
      for (int i = 0; i < numberOfSteps; i++)
      {
         positionSelectionMatrix.set(rowStart,     colStart,  1.0);
         positionSelectionMatrix.set(rowStart + 1, colStart + 1, 1.0);
         rowStart += 2;
         colStart += STATE_SIZE;
      }
   }

   public static void computeCoMVelocitySelectionMatrix(double massComZUpX, double massComZUpY, int numberOfSteps, DMatrixRMaj velocitySelectionMatrix)
   {
      velocitySelectionMatrix.zero();
      int rowStart = 0;
      int colStart = 0;
      for (int i = 0; i < numberOfSteps; i++)
      {
         velocitySelectionMatrix.set(rowStart,     colStart + 3,  1.0 / massComZUpX);
         velocitySelectionMatrix.set(rowStart + 1, colStart + 2, -1.0 / massComZUpY);
         rowStart += 2;
         colStart += STATE_SIZE;
      }
   }

   public static void computeRelativeAngularMomentumSelectionMatrix(double massGravity, int numberOfSteps, DMatrixRMaj relativeAngularMomentumSelectionMatrix)
   {
      relativeAngularMomentumSelectionMatrix.zero();
      int rowStart = 0;
      int colStart = 0;
      for (int i = 0; i < numberOfSteps; i++)
      {
         relativeAngularMomentumSelectionMatrix.set(rowStart, colStart + 1, -massGravity);
         relativeAngularMomentumSelectionMatrix.set(rowStart + 1, colStart, massGravity);
         rowStart += 2;
         colStart += STATE_SIZE;
      }
   }
}
