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

   public void calculateCoMContinuityObjective(QPInput inputToPack, CoMContinuityObjective objective, double weight)
   {
      inputToPack.reshape(3);

      int firstSegmentNumber = objective.getFirstSegmentNumber();
      int secondSegmentNumber = firstSegmentNumber + 1;
      double firstSegmentDuration = objective.getFirstSegmentDuration();
      double omega = objective.getOmega();
      CoefficientJacobianMatrixHelper firstSegmentJacobianMatrixHelper = objective.getFirstSegmentJacobianMatrixHelper();
      CoefficientJacobianMatrixHelper secondSegmentJacobianMatrixHelper = objective.getSecondSegmentJacobianMatrixHelper();
      firstSegmentJacobianMatrixHelper.computeMatrices(firstSegmentDuration, omega);
      secondSegmentJacobianMatrixHelper.computeMatrices(0.0, omega);

      CoMCoefficientJacobianCalculator.calculateJacobian(firstSegmentNumber, firstSegmentDuration, inputToPack.getTaskJacobian(), objective.getDerivativeOrder(), 1.0);
      CoMCoefficientJacobianCalculator.calculateJacobian(secondSegmentNumber, secondSegmentNumber, inputToPack.getTaskJacobian(), objective.getDerivativeOrder(), -1.0);

      int firstSegmentStartIndex = indexHandler.getRhoCoefficientStartIndex(firstSegmentNumber);
      int secondSegmentStartIndex = indexHandler.getRhoCoefficientStartIndex(secondSegmentNumber);
      MatrixTools.addMatrixBlock(inputToPack.getTaskJacobian(),
                                 0,
                                 firstSegmentStartIndex,
                                 firstSegmentJacobianMatrixHelper.getJacobianMatrix(objective.getDerivativeOrder()),
                                 0,
                                 0,
                                 3,
                                 firstSegmentJacobianMatrixHelper.getTotalSize(),
                                 1.0);
      MatrixTools.addMatrixBlock(inputToPack.getTaskJacobian(),
                                 0,
                                 secondSegmentStartIndex,
                                 secondSegmentJacobianMatrixHelper.getJacobianMatrix(objective.getDerivativeOrder()),
                                 0,
                                 0,
                                 3,
                                 secondSegmentJacobianMatrixHelper.getTotalSize(),
                                 -1.0);

      inputToPack.getTaskJacobian().zero();

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);
   }

   public void calculateCoMValueObjective(QPInput inputToPack, CoMValueObjective objective, double weight)
   {
      inputToPack.reshape(3);

      int segmentNumber = objective.getSegmentNumber();
      double timeOfObjective = objective.getTimeOfObjective();
      double omega = objective.getOmega();
      CoefficientJacobianMatrixHelper jacobianMatrixHelper = objective.getJacobianMatrixHelper();
      jacobianMatrixHelper.computeMatrices(timeOfObjective, omega);

      CoMCoefficientJacobianCalculator.calculateJacobian(segmentNumber, timeOfObjective, inputToPack.getTaskJacobian(), objective.getDerivativeOrder(), 1.0);
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

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);
   }
}
