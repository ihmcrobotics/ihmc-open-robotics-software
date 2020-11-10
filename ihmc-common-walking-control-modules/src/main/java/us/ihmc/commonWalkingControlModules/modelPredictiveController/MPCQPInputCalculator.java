package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInput;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;

public class MPCQPInputCalculator
{
   private final MPCIndexHandler indexHandler;

   public MPCQPInputCalculator(MPCIndexHandler indexHandler)
   {
      this.indexHandler = indexHandler;
   }

   private void calculateCoMValueObjective(QPInput inputToPack, CoMValueObjective objective)
   {
      inputToPack.reshape(3);

      int segmentNumber = objective.getSegmentNumber();
      double timeOfObjective = objective.getTimeOfObjective();
      double omega = objective.getOmega();
      CoefficientJacobianMatrixHelper jacobianMatrixHelper = objective.getJacobianMatrixHelper();
      jacobianMatrixHelper.computeMatrices(timeOfObjective, omega);

      CoMCoefficientJacobianCalculator.calculateJacobian(segmentNumber, timeOfObjective, inputToPack.getTaskJacobian(), objective.getDerivativeOrder());
      int startIndex = indexHandler.getRhoCoefficientStartIndex(segmentNumber);
      MatrixTools.addMatrixBlock(inputToPack.getTaskJacobian(),
                                 0,
                                 startIndex,
                                 jacobianMatrixHelper.getJacobianMatrix(objective.getDerivativeOrder()),
                                 0,
                                 0,
                                 3,
                                 jacobianMatrixHelper.getTotalSize(),
                                 1.0);

      objective.getObjective().get(inputToPack.getTaskObjective());

      // TODO add weight
   }
}
