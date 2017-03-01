package us.ihmc.sensorProcessing.stateEstimation.measurementModelElements;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

public class MeasurementModelTestTools
{
   public static void assertDeltaResidualCorrect(MeasurementModelElement modelElement, DenseMatrix64F outputMatrixBlock, DenseMatrix64F perturbationEjmlVector,
           double tol)
   {
      DenseMatrix64F residual = modelElement.computeResidual();
      DenseMatrix64F residualFromOutputMatrix = computeDeltaResidualFromOutputMatrix(outputMatrixBlock, perturbationEjmlVector);
      EjmlUnitTests.assertEquals(residual, residualFromOutputMatrix, tol);
   }

   public static DenseMatrix64F computeDeltaResidualFromOutputMatrix(DenseMatrix64F outputMatrixBlock, DenseMatrix64F perturbationEjmlVector)
   {
      DenseMatrix64F residualFromOutputMatrix = new DenseMatrix64F(3, 1);
      CommonOps.mult(outputMatrixBlock, perturbationEjmlVector, residualFromOutputMatrix);
      CommonOps.scale(-1.0, residualFromOutputMatrix);

      return residualFromOutputMatrix;
   }


   public static void assertOutputMatrixCorrectUsingPerturbation(ControlFlowOutputPort<FramePoint> statePort, MeasurementModelElement modelElement,
                                                                 FramePoint nominalState, double perturbationMagnitude, double tolerance, Runnable runnable)
   {
      DenseMatrix64F outputMatrixBlock = modelElement.getOutputMatrixBlock(statePort);
      for (Direction direction : Direction.values())
      {
         FrameVector perturbationVector = new FrameVector(nominalState.getReferenceFrame());
         perturbationVector.set(direction, perturbationMagnitude);

         DenseMatrix64F perturbationEjmlVector = new DenseMatrix64F(3, 1);
         perturbationVector.getVector().get(perturbationEjmlVector);

         FramePoint perturbedState = new FramePoint(nominalState);
         perturbedState.add(perturbationVector);
         statePort.setData(perturbedState);

         if (runnable != null)
            runnable.run();

         MeasurementModelTestTools.assertDeltaResidualCorrect(modelElement, outputMatrixBlock, perturbationEjmlVector, tolerance);
      }

      statePort.setData(nominalState);
   }

   public static void assertOutputMatrixCorrectUsingPerturbation(ControlFlowOutputPort<FrameVector> statePort, MeasurementModelElement modelElement,
           FrameVector nominalState, double perturbationMagnitude, double tolerance, Runnable runnable)
   {
      DenseMatrix64F outputMatrixBlock = modelElement.getOutputMatrixBlock(statePort);
      for (Direction direction : Direction.values())
      {
         FrameVector perturbationVector = new FrameVector(nominalState.getReferenceFrame());
         perturbationVector.set(direction, perturbationMagnitude);

         DenseMatrix64F perturbationEjmlVector = new DenseMatrix64F(3, 1);
         perturbationVector.getVector().get(perturbationEjmlVector);

         FrameVector perturbedState = new FrameVector(nominalState);
         perturbedState.add(perturbationVector);
         statePort.setData(perturbedState);

         if (runnable != null)
            runnable.run();

         MeasurementModelTestTools.assertDeltaResidualCorrect(modelElement, outputMatrixBlock, perturbationEjmlVector, tolerance);
      }

      statePort.setData(nominalState);
   }

   public static void assertOutputMatrixCorrectUsingPerturbation(ControlFlowOutputPort<FrameOrientation> statePort, MeasurementModelElement modelElement,
           FrameOrientation nominalState, double perturbationMagnitude, double tolerance, Runnable runnable)
   {
      DenseMatrix64F outputMatrixBlock = modelElement.getOutputMatrixBlock(statePort);
      for (Direction direction : Direction.values())
      {
         Vector3D perturbationRotationVector = new Vector3D();
         Direction.set(perturbationRotationVector, direction, perturbationMagnitude);

         DenseMatrix64F perturbationEjmlVector = new DenseMatrix64F(3, 1);
         perturbationRotationVector.get(perturbationEjmlVector);

         Quaternion perturbedQuaternion = new Quaternion();
         AxisAngle perturbationAxisAngle = new AxisAngle();
         perturbationAxisAngle.set(perturbationRotationVector);
         Quaternion perturbationQuaternion = new Quaternion();
         perturbationQuaternion.set(perturbationAxisAngle);
         perturbedQuaternion.multiply(nominalState.getQuaternionCopy(), perturbationQuaternion);
         FrameOrientation perturbedState = new FrameOrientation(nominalState.getReferenceFrame(), perturbedQuaternion);
         statePort.setData(perturbedState);

         if (runnable != null)
            runnable.run();

         MeasurementModelTestTools.assertDeltaResidualCorrect(modelElement, outputMatrixBlock, perturbationEjmlVector, tolerance);
      }

      statePort.setData(nominalState);
   }
}
