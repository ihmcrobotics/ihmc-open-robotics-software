package us.ihmc.sensorProcessing.stateEstimation.measurementModelElements;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

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


   public static void assertOutputMatrixCorrectUsingPerturbation(ControlFlowOutputPort<FramePoint3D> statePort, MeasurementModelElement modelElement,
                                                                 FramePoint3D nominalState, double perturbationMagnitude, double tolerance, Runnable runnable)
   {
      DenseMatrix64F outputMatrixBlock = modelElement.getOutputMatrixBlock(statePort);
      for (Axis axis : Axis.values())
      {
         FrameVector3D perturbationVector = new FrameVector3D(nominalState.getReferenceFrame());
         perturbationVector.setElement(axis.ordinal(), perturbationMagnitude);

         DenseMatrix64F perturbationEjmlVector = new DenseMatrix64F(3, 1);
         perturbationVector.getVector().get(perturbationEjmlVector);

         FramePoint3D perturbedState = new FramePoint3D(nominalState);
         perturbedState.add(perturbationVector);
         statePort.setData(perturbedState);

         if (runnable != null)
            runnable.run();

         MeasurementModelTestTools.assertDeltaResidualCorrect(modelElement, outputMatrixBlock, perturbationEjmlVector, tolerance);
      }

      statePort.setData(nominalState);
   }

   public static void assertOutputMatrixCorrectUsingPerturbation(ControlFlowOutputPort<FrameVector3D> statePort, MeasurementModelElement modelElement,
           FrameVector3D nominalState, double perturbationMagnitude, double tolerance, Runnable runnable)
   {
      DenseMatrix64F outputMatrixBlock = modelElement.getOutputMatrixBlock(statePort);
      for (Axis axis : Axis.values())
      {
         FrameVector3D perturbationVector = new FrameVector3D(nominalState.getReferenceFrame());
         perturbationVector.setElement(axis.ordinal(), perturbationMagnitude);

         DenseMatrix64F perturbationEjmlVector = new DenseMatrix64F(3, 1);
         perturbationVector.getVector().get(perturbationEjmlVector);

         FrameVector3D perturbedState = new FrameVector3D(nominalState);
         perturbedState.add(perturbationVector);
         statePort.setData(perturbedState);

         if (runnable != null)
            runnable.run();

         MeasurementModelTestTools.assertDeltaResidualCorrect(modelElement, outputMatrixBlock, perturbationEjmlVector, tolerance);
      }

      statePort.setData(nominalState);
   }

   public static void assertOutputMatrixCorrectUsingPerturbation(ControlFlowOutputPort<FrameQuaternion> statePort, MeasurementModelElement modelElement,
           FrameQuaternion nominalState, double perturbationMagnitude, double tolerance, Runnable runnable)
   {
      DenseMatrix64F outputMatrixBlock = modelElement.getOutputMatrixBlock(statePort);
      for (Axis axis : Axis.values())
      {
         Vector3D perturbationRotationVector = new Vector3D();
         Axis.set(perturbationRotationVector, axis, perturbationMagnitude);

         DenseMatrix64F perturbationEjmlVector = new DenseMatrix64F(3, 1);
         perturbationRotationVector.get(perturbationEjmlVector);

         Quaternion perturbedQuaternion = new Quaternion();
         AxisAngle perturbationAxisAngle = new AxisAngle();
         perturbationAxisAngle.set(perturbationRotationVector);
         Quaternion perturbationQuaternion = new Quaternion();
         perturbationQuaternion.set(perturbationAxisAngle);
         perturbedQuaternion.multiply(nominalState, perturbationQuaternion);
         FrameQuaternion perturbedState = new FrameQuaternion(nominalState.getReferenceFrame(), perturbedQuaternion);
         statePort.setData(perturbedState);

         if (runnable != null)
            runnable.run();

         MeasurementModelTestTools.assertDeltaResidualCorrect(modelElement, outputMatrixBlock, perturbationEjmlVector, tolerance);
      }

      statePort.setData(nominalState);
   }
}
