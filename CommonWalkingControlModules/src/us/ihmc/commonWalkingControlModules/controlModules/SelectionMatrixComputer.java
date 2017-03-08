package us.ihmc.commonWalkingControlModules.controlModules;

import org.ejml.data.DenseMatrix64F;
import org.ejml.interfaces.linsol.LinearSolver;

import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotics.math.SolvePseudoInverseSvdGCFree;
import us.ihmc.robotics.screwTheory.GeometricJacobian;

public class SelectionMatrixComputer
{
   private final LinearSolver<DenseMatrix64F> selectionMatrixSolver = new SolvePseudoInverseSvdGCFree(12, 12);

   public void computeSelectionMatrix(int jacobianId, HighLevelHumanoidControllerToolbox controllerToolbox, DenseMatrix64F selectionMatrix)
   {
      computeSelectionMatrix(controllerToolbox.getJacobian(jacobianId), selectionMatrix);
   }

   public void computeSelectionMatrix(GeometricJacobian jacobian, DenseMatrix64F selectionMatrix)
   {
      DenseMatrix64F jacobianMatrix = jacobian.getJacobianMatrix();
      if (selectionMatrixSolver.modifiesA())
      {
         throw new RuntimeException("Selection matrix solver changes A");
      }

      if (!selectionMatrixSolver.setA(jacobianMatrix))
         throw new IllegalArgumentException("Invert failed, maybe a bug?");

      selectionMatrixSolver.invert(selectionMatrix);

   }
}
