package us.ihmc.exampleSimulations.simpleMPCTouchdownCalculatorTesting.MPC;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.robotSide.RobotSide;
import static us.ihmc.exampleSimulations.simpleMPCTouchdownCalculatorTesting.MPC.AutomaticFootstepMPCMatrixCalculator.*;

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

   private final DMatrixRMaj AMatrix = new DMatrixRMaj(4, 4);
   private final DMatrixRMaj ABar = new DMatrixRMaj(4, 4);
   private final DMatrixRMaj BBar = new DMatrixRMaj(4, 2);
   private final DMatrixRMaj AHat = new DMatrixRMaj(4, 4);
   private final DMatrixRMaj BHat = new DMatrixRMaj(4, 2);

   public void computeEndOfStepStateTransitionMatrices(double omegaX,
                                                       double omegaY,
                                                       double gravity,
                                                       double mass,
                                                       double swingDuration,
                                                       DMatrixRMaj stackedStateTransitionMatrixToPack,
                                                       DMatrixRMaj stackedControlInputMatrixToPack)
   {
      double zInX = gravity / MathTools.square(omegaX);
      double zInY = gravity / MathTools.square(omegaY);

      FootstepMPCMatrixTools.computeStateTransitionMatrix(omegaX, omegaY, swingDuration, mass * zInX * omegaX, mass * zInY * omegaY, AMatrix);
      FootstepMPCMatrixTools.computeJumpMatrices(ABar, BBar);
      CommonOps_DDRM.mult(AMatrix, ABar, AHat);
      CommonOps_DDRM.mult(AMatrix, BBar, BHat);

      stackedStateTransitionMatrixToPack.zero();
      MatrixTools.setMatrixBlock(stackedStateTransitionMatrixToPack, 0, 0, AHat, 0, 0, STATE_SIZE, STATE_SIZE, 1.0);
      MatrixTools.multAddBlock(AHat, AHat, stackedStateTransitionMatrixToPack, STATE_SIZE, 0);

      stackedControlInputMatrixToPack.zero();
      MatrixTools.setMatrixBlock(stackedControlInputMatrixToPack, 0, 0, BHat, 0, 0, STATE_SIZE, CONTROL_SIZE, 1.0);
      MatrixTools.multAddBlock(AHat, BHat, stackedControlInputMatrixToPack, STATE_SIZE, 0);
      MatrixTools.setMatrixBlock(stackedControlInputMatrixToPack , STATE_SIZE, CONTROL_SIZE, BHat, 0, 0, STATE_SIZE, CONTROL_SIZE, 1.0);
   }


   private final DMatrixRMaj stackedA = new DMatrixRMaj(8, 4);
   private final DMatrixRMaj stackedB = new DMatrixRMaj(8, 4);
   private final DMatrixRMaj finalA = new DMatrixRMaj(4, 4);
   private final DMatrixRMaj finalB = new DMatrixRMaj(4, 4);
   private final DMatrixRMaj expectedControlInput = new DMatrixRMaj(4, 1);
   private final DMatrixRMaj identity = new DMatrixRMaj(4, 4);
   private final DMatrixRMaj steadyStateA = new DMatrixRMaj(4, 4);
   private final DMatrixRMaj steadyStateB = new DMatrixRMaj(4, 1);
   private final DMatrixRMaj steadyStateFinal = new DMatrixRMaj(4, 1);

   private final DMatrixRMaj positionSelection = new DMatrixRMaj(2, 4);
   private final DMatrixRMaj pastStack = new DMatrixRMaj(2, 4);
   private final LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.general(NUMBER_OF_VARIABLES, NUMBER_OF_VARIABLES);


   public void computeSteadyEndState(RobotSide currentSupportSide,
                                     double omegaX,
                                     double omegaY,
                                     double gravity,
                                     double mass,
                                     double swingDuration,
                                     double nominalStanceWidth,
                                     Vector2DReadOnly desiredWalkingSpeed,
                                     DMatrixRMaj steadyStateToPack)
   {
      computeEndOfStepStateTransitionMatrices(omegaX, omegaY, gravity, mass, swingDuration, stackedA, stackedB);

      CommonOps_DDRM.extract(stackedA, 4, 8, 0, 4, finalA, 0, 0);
      CommonOps_DDRM.extract(stackedB, 4, 8, 0, 4, finalB, 0, 0);

      expectedControlInput.set(0, 0, desiredWalkingSpeed.getX() * swingDuration);
      expectedControlInput.set(1, 0, desiredWalkingSpeed.getY() * swingDuration + currentSupportSide.negateIfLeftSide(nominalStanceWidth));
      expectedControlInput.set(2, 0, desiredWalkingSpeed.getX() * swingDuration);
      expectedControlInput.set(3, 0, desiredWalkingSpeed.getY() * swingDuration - currentSupportSide.negateIfLeftSide(nominalStanceWidth));

      CommonOps_DDRM.setIdentity(identity);
      CommonOps_DDRM.setIdentity(positionSelection);

      CommonOps_DDRM.subtract(identity, finalA, steadyStateA);
      CommonOps_DDRM.mult(finalB, expectedControlInput, steadyStateB);

      solver.setA(steadyStateA);
      solver.solve(steadyStateB, steadyStateFinal);

      CommonOps_DDRM.mult(stackedA, steadyStateFinal, steadyStateToPack);
      CommonOps_DDRM.multAdd(stackedB, expectedControlInput, steadyStateToPack);
   }
}