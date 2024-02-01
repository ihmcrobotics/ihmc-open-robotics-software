package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;

import java.lang.annotation.Native;

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
   private final NativeMatrix tempRotationMatrix = new NativeMatrix(3, 3);
   private final NativeMatrix tempRotationMatrixWithSelection = new NativeMatrix(1, 1);
   private final NativeMatrix tempTaskWeight = new NativeMatrix(3, 3);
   private final NativeMatrix denseSelectionMatrix = new NativeMatrix(1, 1);

   private final NativeMatrix taskJacobian3D = new NativeMatrix(1, 1);
   private final NativeMatrix taskObjective3D = new NativeMatrix(1, 1);

   private final NativeMatrix taskJacobianSelected3D = new NativeMatrix(1, 1);
   private final NativeMatrix taskObjectiveSelected3D = new NativeMatrix(1, 1);
   private final NativeMatrix taskWeightSelected3D = new NativeMatrix(1, 1);

   public int applySelectionToTask(SelectionMatrix3D selectionMatrix,
                                   WeightMatrix3D weightMatrix,
                                   ReferenceFrame taskFrame,
                                   NativeMatrix taskJacobian,
                                   NativeMatrix taskObjective,
                                   NativeMatrix taskJacobianToPack,
                                   NativeMatrix taskObjectiveToPack,
                                   NativeMatrix taskWeightToPack)
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
      denseSelectionMatrix.zero();
      denseSelectionMatrix.fillDiagonal(1.0);
      for (int axis = taskSize - 1; axis >= 0; axis--)
      {
         if (!selectionMatrix.isAxisSelected(axis))
         {
            denseSelectionMatrix.removeRow(axis);
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
         tempTransform.getRotation().get(tempRotationMatrix);
         tempRotationMatrixWithSelection.reshape(reducedTaskSize, 3);
         tempRotationMatrixWithSelection.mult(denseSelectionMatrix, tempRotationMatrix);
         taskJacobianToPack.reshape(reducedTaskSize, problemSize);
         taskObjectiveToPack.reshape(reducedTaskSize, 1);
         taskJacobianToPack.mult(tempRotationMatrixWithSelection, taskJacobian);
         taskObjectiveToPack.mult(tempRotationMatrixWithSelection, taskObjective);
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
      taskWeightToPack.zero();
      taskWeightToPack.set(0, 0, weightMatrix.getXAxisWeight());
      taskWeightToPack.set(1, 1, weightMatrix.getYAxisWeight());
      taskWeightToPack.set(2, 2, weightMatrix.getZAxisWeight());
      // Skip the transformation if the frames match. This is helpful if the user only specifies a weight for the selected axes. In
      // that case applying the transformation would fill the matrix with nasty NaNs.
      if (weightFrame != selectionFrame)
      {
         weightFrame.getTransformToDesiredFrame(tempTransform, selectionFrame);
         tempTransform.getRotation().get(tempRotationMatrix);
         tempRotationMatrixWithSelection.reshape(reducedTaskSize, 3);
         tempRotationMatrixWithSelection.mult(denseSelectionMatrix, tempRotationMatrix);
         tempTaskWeight.reshape(reducedTaskSize, 3);
         tempTaskWeight.mult(tempRotationMatrixWithSelection, taskWeightToPack);
         taskWeightToPack.reshape(reducedTaskSize, reducedTaskSize);
         taskWeightToPack.multTransB(tempTaskWeight, tempRotationMatrixWithSelection);
      }
      else
      {
         for (int axis = taskSize - 1; axis >= 0; axis--)
         {
            if (!selectionMatrix.isAxisSelected(axis))
            {
               taskWeightToPack.removeRow(axis);
               taskWeightToPack.removeColumn(axis);
            }
         }
      }

      checkResult(taskJacobianToPack, taskObjectiveToPack, taskWeightToPack);

      return reducedTaskSize;
   }

   public int applySelectionToTask(SelectionMatrix6D selectionMatrix,
                                   WeightMatrix6D weightMatrix,
                                   ReferenceFrame taskFrame,
                                   DMatrixRMaj taskJacobian,
                                   DMatrixRMaj taskObjective,
                                   NativeMatrix taskJacobianToPack,
                                   NativeMatrix taskObjectiveToPack,
                                   NativeMatrix taskWeightToPack)
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
      taskJacobian3D.insert(taskJacobian, 0, 3, 0, problemSize, 0, 0);
      taskObjective3D.insert(taskObjective, 0, 3, 0, 1, 0, 0);
      int offset = applySelectionToTask(selectionMatrix.getAngularPart(),
                                        weightMatrix.getAngularPart(),
                                        taskFrame,
                                        taskJacobian3D,
                                        taskObjective3D,
                                        taskJacobianSelected3D,
                                        taskObjectiveSelected3D,
                                        taskWeightSelected3D);
      taskJacobianToPack.insert(taskJacobianSelected3D, 0, 0);
      taskObjectiveToPack.insert(taskObjectiveSelected3D, 0, 0);
      taskWeightToPack.insert(taskWeightSelected3D, 0, 0);

      // Do the linear part:
      taskJacobian3D.insert(taskJacobian, 3, 6, 0, problemSize, 0, 0);
      taskObjective3D.insert(taskObjective, 3, 6, 0, 1, 0, 0);
      applySelectionToTask(selectionMatrix.getLinearPart(),
                           weightMatrix.getLinearPart(),
                           taskFrame,
                           taskJacobian3D,
                           taskObjective3D,
                           taskJacobianSelected3D,
                           taskObjectiveSelected3D,
                           taskWeightSelected3D);
      taskJacobianToPack.insert(taskJacobianSelected3D, offset, 0);
      taskObjectiveToPack.insert(taskObjectiveSelected3D, offset, 0);
      taskWeightToPack.insert(taskWeightSelected3D, offset, offset);

      return reducedTaskSize;
   }

   private static void checkResult(NativeMatrix taskJacobian, NativeMatrix taskObjective, NativeMatrix taskWeight)
   {
      if (taskJacobian.containsNaN())
      {
         throw new RuntimeException("The task jacobian contained NaN.");
      }
      if (taskObjective.containsNaN())
      {
         throw new RuntimeException("The task objective contained NaN.");
      }
      if (taskWeight.containsNaN())
      {
         throw new RuntimeException("The task weight contained NaN.");
      }
   }

   private static void checkMatrixSizes(DMatrix taskJacobian, DMatrix taskObjective, int taskSize)
   {
      if (taskJacobian.getNumRows() != taskSize || taskObjective.getNumRows() != taskSize || taskObjective.getNumCols() != 1)
      {
         throw new RuntimeException("Unexpected size of task matrices.");
      }
   }
}
