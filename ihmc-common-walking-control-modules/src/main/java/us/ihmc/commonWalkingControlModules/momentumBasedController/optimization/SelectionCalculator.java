package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;

/**
 * This is a helper class to handle {@link SelectionMatrix3D}, {@link SelectionMatrix6D} in the context of QP objectives.
 * <p>
 * Since these matrices do not always have the same frame as the task they are being used for this class can take care of the
 * transformation of the task matrices such that the selection matrix is applied correctly. The user computes the task
 * Jacobian ({@code J}) and task objective ({@code b}) without considering the weight and selection matrices. The convenience
 * methods defined by this class will then calculate a new task Jacobian, objective, and weight matrix.
 * </p>
 * <p>
 * A task is defined as such:</br>
 * {@code min (Jx - b)' * W * (Jx - b)}</br>
 * This class will transform the task into the selection frame and then remove the unselected rows from the task matrices.
 * </p>
 */
public class SelectionCalculator
{
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final DenseMatrix64F tempRotationMatrix = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F tempRotationMatrixWithSelection = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F tempTaskWeight = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F denseSelectionMatrix = new DenseMatrix64F(1, 1);

   private final DenseMatrix64F taskJacobian3D = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F taskObjective3D = new DenseMatrix64F(1, 1);

   private final DenseMatrix64F taskJacobianSelected3D = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F taskObjectiveSelected3D = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F taskWeightSelected3D = new DenseMatrix64F(1, 1);

   public int applySelectionToTask(SelectionMatrix3D selectionMatrix, WeightMatrix3D weightMatrix, ReferenceFrame taskFrame, DenseMatrix64F taskJacobian,
                                   DenseMatrix64F taskObjective, DenseMatrix64F taskJacobianToPack, DenseMatrix64F taskObjectiveToPack,
                                   DenseMatrix64F taskWeightToPack)
   {
      int problemSize = taskJacobian.getNumCols();
      int taskSize = 3;
      checkMatrixSizes(taskJacobian, taskObjective, taskSize);
      int reducedTaskSize = selectionMatrix.getNumberOfSelectedAxes();

      boolean selectedNone = reducedTaskSize == 0;
      if (selectedNone)
      {
         taskJacobianToPack.reshape(0, problemSize);
         taskObjectiveToPack.reshape(0, 1);
         taskWeightToPack.reshape(0, 0);
         return 0;
      }

      // Pack the selection matrix in selection frame:
      denseSelectionMatrix.reshape(3, 3);
      CommonOps.setIdentity(denseSelectionMatrix);
      for (int axis = taskSize - 1; axis >= 0; axis--)
      {
         if (!selectionMatrix.isAxisSelected(axis))
         {
            MatrixTools.removeRow(denseSelectionMatrix, axis);
         }
      }

      // Pack the transformation matrix from task frame to selection frame. Then apply these transforms to both the original
      // task Jacobian and the objective.
      ReferenceFrame selectionFrame = selectionMatrix.getSelectionFrame();
      boolean selectedAll = reducedTaskSize == 3;
      if (selectedAll || selectionFrame == null)
      {
         // If all axes are selected we can skip the transformations and just assume all axes are selected in the selection frame.
         // If no selection frame is provided it is assumed to be in the task frame.
         selectionFrame = taskFrame;
      }
      if (selectionFrame != taskFrame)
      {
         taskFrame.getTransformToDesiredFrame(tempTransform, selectionFrame);
         tempTransform.getRotation(tempRotationMatrix);
         tempRotationMatrixWithSelection.reshape(reducedTaskSize, 3);
         CommonOps.mult(denseSelectionMatrix, tempRotationMatrix, tempRotationMatrixWithSelection);
         taskJacobianToPack.reshape(reducedTaskSize, problemSize);
         taskObjectiveToPack.reshape(reducedTaskSize, 1);
         CommonOps.mult(tempRotationMatrixWithSelection, taskJacobian, taskJacobianToPack);
         CommonOps.mult(tempRotationMatrixWithSelection, taskObjective, taskObjectiveToPack);
      }
      else
      {
         // If the selection frame equals the task frame we can skip this step.
         taskJacobianToPack.set(taskJacobian);
         taskObjectiveToPack.set(taskObjective);
      }

      // Get the weight matrix in weight frame. The transform it using the transformation between weight frame and selection frame.
      ReferenceFrame weightFrame = weightMatrix.getWeightFrame();
      if (weightFrame == null)
      {
         weightFrame = taskFrame;
      }
      taskWeightToPack.reshape(taskSize, taskSize);
      CommonOps.fill(taskWeightToPack, 0.0);
      taskWeightToPack.set(0, 0, weightMatrix.getXAxisWeight());
      taskWeightToPack.set(1, 1, weightMatrix.getYAxisWeight());
      taskWeightToPack.set(2, 2, weightMatrix.getZAxisWeight());
      // Skip the transformation if the frames match. This is helpful if the user only specifies a weight for the selected axes. In
      // that case applying the transformation would fill the matrix with nasty NaNs.
      if (weightFrame != selectionFrame)
      {
         weightFrame.getTransformToDesiredFrame(tempTransform, selectionFrame);
         tempTransform.getRotation(tempRotationMatrix);
         tempRotationMatrixWithSelection.reshape(reducedTaskSize, 3);
         CommonOps.mult(denseSelectionMatrix, tempRotationMatrix, tempRotationMatrixWithSelection);
         tempTaskWeight.reshape(reducedTaskSize, 3);
         CommonOps.mult(tempRotationMatrixWithSelection, taskWeightToPack, tempTaskWeight);
         taskWeightToPack.reshape(reducedTaskSize, reducedTaskSize);
         CommonOps.multTransB(tempTaskWeight, tempRotationMatrixWithSelection, taskWeightToPack);
      }
      else
      {
         for (int axis = taskSize - 1; axis >= 0; axis--)
         {
            if (!selectionMatrix.isAxisSelected(axis))
            {
               MatrixTools.removeRow(taskWeightToPack, axis);
               MatrixTools.removeColumn(taskWeightToPack, axis);
            }
         }
      }

      checkResult(taskJacobianToPack, taskObjectiveToPack, taskWeightToPack);

      return reducedTaskSize;
   }

   public int applySelectionToTask(SelectionMatrix6D selectionMatrix, WeightMatrix6D weightMatrix, ReferenceFrame taskFrame, DenseMatrix64F taskJacobian,
                                   DenseMatrix64F taskObjective, DenseMatrix64F taskJacobianToPack, DenseMatrix64F taskObjectiveToPack,
                                   DenseMatrix64F taskWeightToPack)
   {
      int problemSize = taskJacobian.getNumCols();
      int taskSize = 6;
      checkMatrixSizes(taskJacobian, taskObjective, taskSize);
      int reducedTaskSize = selectionMatrix.getNumberOfSelectedAxes();

      // Split the problem up into two parts (angular and linear).
      taskJacobian3D.reshape(3, problemSize);
      taskObjective3D.reshape(3, 1);

      taskJacobianToPack.reshape(reducedTaskSize, problemSize);
      taskObjectiveToPack.reshape(reducedTaskSize, 1);
      taskWeightToPack.reshape(reducedTaskSize, reducedTaskSize);

      // Do the angular part:
      CommonOps.extract(taskJacobian, 0, 3, 0, problemSize, taskJacobian3D, 0, 0);
      CommonOps.extract(taskObjective, 0, 3, 0, 1, taskObjective3D, 0, 0);
      int offset = applySelectionToTask(selectionMatrix.getAngularPart(), weightMatrix.getAngularPart(), taskFrame, taskJacobian3D, taskObjective3D,
                                        taskJacobianSelected3D, taskObjectiveSelected3D, taskWeightSelected3D);
      CommonOps.insert(taskJacobianSelected3D, taskJacobianToPack, 0, 0);
      CommonOps.insert(taskObjectiveSelected3D, taskObjectiveToPack, 0, 0);
      CommonOps.insert(taskWeightSelected3D, taskWeightToPack, 0, 0);

      // Do the linear part:
      CommonOps.extract(taskJacobian, 3, 6, 0, problemSize, taskJacobian3D, 0, 0);
      CommonOps.extract(taskObjective, 3, 6, 0, 1, taskObjective3D, 0, 0);
      applySelectionToTask(selectionMatrix.getLinearPart(), weightMatrix.getLinearPart(), taskFrame, taskJacobian3D, taskObjective3D, taskJacobianSelected3D,
                           taskObjectiveSelected3D, taskWeightSelected3D);
      CommonOps.insert(taskJacobianSelected3D, taskJacobianToPack, offset, 0);
      CommonOps.insert(taskObjectiveSelected3D, taskObjectiveToPack, offset, 0);
      CommonOps.insert(taskWeightSelected3D, taskWeightToPack, offset, offset);

      return reducedTaskSize;
   }

   private static void checkResult(DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective, DenseMatrix64F taskWeight)
   {
      if (MatrixTools.containsNaN(taskJacobian))
      {
         throw new RuntimeException("The task jacobian contained NaN.");
      }
      if (MatrixTools.containsNaN(taskObjective))
      {
         throw new RuntimeException("The task objective contained NaN.");
      }
      if (MatrixTools.containsNaN(taskWeight))
      {
         throw new RuntimeException("The task weight contained NaN.");
      }
   }

   private static void checkMatrixSizes(DenseMatrix64F taskJacobian, DenseMatrix64F taskObjective, int taskSize)
   {
      if (taskJacobian.getNumRows() != taskSize || taskObjective.getNumRows() != taskSize || taskObjective.getNumCols() != 1)
      {
         throw new RuntimeException("Unexpected size of task matrices.");
      }
   }
}
