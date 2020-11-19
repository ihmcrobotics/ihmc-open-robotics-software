package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInput;
import us.ihmc.matrixlib.MatrixTools;

public class MPCQPInputCalculator
{
   private final MPCIndexHandler indexHandler;

   private final DMatrixRMaj tempCoefficientJacobian = new DMatrixRMaj(0, 0);

   public MPCQPInputCalculator(MPCIndexHandler indexHandler)
   {
      this.indexHandler = indexHandler;
   }

   public boolean calculateCoMContinuityObjective(QPInput inputToPack, CoMContinuityObjective objective, double weight)
   {
      inputToPack.reshape(3);

      int firstSegmentNumber = objective.getFirstSegmentNumber();
      int secondSegmentNumber = firstSegmentNumber + 1;
      double firstSegmentDuration = objective.getFirstSegmentDuration();
      double omega = objective.getOmega();

      CoMCoefficientJacobianCalculator.calculateCoMJacobian(firstSegmentNumber, firstSegmentDuration, inputToPack.getTaskJacobian(), objective.getDerivativeOrder(), 1.0);
      CoMCoefficientJacobianCalculator.calculateCoMJacobian(secondSegmentNumber, secondSegmentNumber, inputToPack.getTaskJacobian(), objective.getDerivativeOrder(), -1.0);

      int startCol = indexHandler.getRhoCoefficientStartIndex(firstSegmentNumber);
      for (int i = 0; i < objective.getFirstSegmentNumberOfContacts(); i++)
      {
         CoefficientJacobianMatrixHelper coefficientJacobianMatrixHelper = objective.getFirstSegmentCoefficientJacobianMatrixHelper(i);
         coefficientJacobianMatrixHelper.computeMatrices(firstSegmentDuration, omega);

         DMatrixRMaj rhoToLinearForceJacobian = objective.getFirstSegmentRhoToForceMatrixHelper(i).getLinearJacobianInWorldFrame();

         tempCoefficientJacobian.set(coefficientJacobianMatrixHelper.getJacobianMatrix(objective.getDerivativeOrder()));
         CommonOps_DDRM.addEquals(tempCoefficientJacobian, -1.0 / omega, coefficientJacobianMatrixHelper.getJacobianMatrix(objective.getDerivativeOrder() + 1));

         MatrixTools.multAddBlock(rhoToLinearForceJacobian, tempCoefficientJacobian, inputToPack.getTaskJacobian(), 0, startCol);
         startCol += coefficientJacobianMatrixHelper.getCoefficientSize();
      }

      startCol = indexHandler.getRhoCoefficientStartIndex(secondSegmentNumber);
      for (int i = 0; i < objective.getSecondSegmentNumberOfContacts(); i++)
      {
         CoefficientJacobianMatrixHelper coefficientJacobianMatrixHelper = objective.getSecondSegmentCoefficientJacobianMatrixHelper(i);
         coefficientJacobianMatrixHelper.computeMatrices(0.0, omega);

         DMatrixRMaj rhoToLinearForceJacobian = objective.getSecondSegmentRhoToForceMatrixHelper(i).getLinearJacobianInWorldFrame();

         tempCoefficientJacobian.set(coefficientJacobianMatrixHelper.getJacobianMatrix(objective.getDerivativeOrder()));
         CommonOps_DDRM.addEquals(tempCoefficientJacobian, -1.0 / omega, coefficientJacobianMatrixHelper.getJacobianMatrix(objective.getDerivativeOrder() + 1));

         MatrixTools.multAddBlock(rhoToLinearForceJacobian, tempCoefficientJacobian, inputToPack.getTaskJacobian(), 0, startCol);
         startCol += coefficientJacobianMatrixHelper.getCoefficientSize();
      }

      inputToPack.getTaskJacobian().zero();

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   public boolean calculateValueObjective(QPInput inputToPack, MPCValueObjective objective, double weight)
   {
      switch (objective.getValueType())
      {
         case COM:
            return calculateCoMValueObjective(inputToPack, objective, weight);
         case VRP:
            return calculateVRPValueObjective(inputToPack, objective, weight);
         case DCM:
            return calculateDCMValueObjective(inputToPack, objective, weight);
         default:
            return false;
      }
   }

   private boolean calculateCoMValueObjective(QPInput inputToPack, MPCValueObjective objective, double weight)
   {
      inputToPack.reshape(3);

      int segmentNumber = objective.getSegmentNumber();
      double timeOfObjective = objective.getTimeOfObjective();
      double omega = objective.getOmega();

      CoMCoefficientJacobianCalculator.calculateCoMJacobian(segmentNumber, timeOfObjective, inputToPack.getTaskJacobian(), objective.getDerivativeOrder(), 1.0);

      int startCol = indexHandler.getRhoCoefficientStartIndex(segmentNumber);
      for (int i = 0; i < objective.getNumberOfContacts(); i++)
      {
         CoefficientJacobianMatrixHelper coefficientJacobianMatrixHelper = objective.getCoefficientJacobianMatrixHelper(i);
         coefficientJacobianMatrixHelper.computeMatrices(timeOfObjective, omega);

         DMatrixRMaj rhoToLinearForceJacobian = objective.getRhoToForceMatrixHelper(i).getLinearJacobianInWorldFrame();
         DMatrixRMaj coefficientToRhoForceJacobian = coefficientJacobianMatrixHelper.getJacobianMatrix(objective.getDerivativeOrder());

         MatrixTools.multAddBlock(rhoToLinearForceJacobian, coefficientToRhoForceJacobian, inputToPack.getTaskJacobian(), 0, startCol);
         startCol += coefficientToRhoForceJacobian.getNumCols();
      }

      objective.getObjective().get(inputToPack.getTaskObjective());

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   private boolean calculateDCMValueObjective(QPInput inputToPack, MPCValueObjective objective, double weight)
   {
      inputToPack.reshape(3);

      int segmentNumber = objective.getSegmentNumber();
      double timeOfObjective = objective.getTimeOfObjective();
      double omega = objective.getOmega();

      CoMCoefficientJacobianCalculator.calculateDCMJacobian(segmentNumber, omega, timeOfObjective, inputToPack.getTaskJacobian(), objective.getDerivativeOrder(), 1.0);

      int startCol = indexHandler.getRhoCoefficientStartIndex(segmentNumber);
      for (int i = 0; i < objective.getNumberOfContacts(); i++)
      {
         CoefficientJacobianMatrixHelper coefficientJacobianMatrixHelper = objective.getCoefficientJacobianMatrixHelper(i);
         coefficientJacobianMatrixHelper.computeMatrices(timeOfObjective, omega);

         DMatrixRMaj rhoToLinearForceJacobian = objective.getRhoToForceMatrixHelper(i).getLinearJacobianInWorldFrame();

         tempCoefficientJacobian.set(coefficientJacobianMatrixHelper.getJacobianMatrix(objective.getDerivativeOrder()));
         CommonOps_DDRM.addEquals(tempCoefficientJacobian, -1.0 / omega, coefficientJacobianMatrixHelper.getJacobianMatrix(objective.getDerivativeOrder() + 1));

         MatrixTools.multAddBlock(rhoToLinearForceJacobian, tempCoefficientJacobian, inputToPack.getTaskJacobian(), 0, startCol);
         startCol += coefficientJacobianMatrixHelper.getCoefficientSize();
      }

      objective.getObjective().get(inputToPack.getTaskObjective());

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   private boolean calculateVRPValueObjective(QPInput inputToPack, MPCValueObjective objective, double weight)
   {
      inputToPack.reshape(3);

      int segmentNumber = objective.getSegmentNumber();
      double timeOfObjective = objective.getTimeOfObjective();
      double omega = objective.getOmega();

      CoMCoefficientJacobianCalculator.calculateVRPJacobian(segmentNumber, omega, timeOfObjective, inputToPack.getTaskJacobian(), objective.getDerivativeOrder(), 1.0);

      int startCol = indexHandler.getRhoCoefficientStartIndex(segmentNumber);
      for (int i = 0; i < objective.getNumberOfContacts(); i++)
      {
         CoefficientJacobianMatrixHelper coefficientJacobianMatrixHelper = objective.getCoefficientJacobianMatrixHelper(i);
         coefficientJacobianMatrixHelper.computeMatrices(timeOfObjective, omega);

         DMatrixRMaj rhoToLinearForceJacobian = objective.getRhoToForceMatrixHelper(i).getLinearJacobianInWorldFrame();

         tempCoefficientJacobian.reshape(coefficientJacobianMatrixHelper.getRhoSize(), coefficientJacobianMatrixHelper.getCoefficientSize());
         tempCoefficientJacobian.set(coefficientJacobianMatrixHelper.getJacobianMatrix(objective.getDerivativeOrder()));
         CommonOps_DDRM.addEquals(tempCoefficientJacobian, -1.0 / (omega * omega), coefficientJacobianMatrixHelper.getJacobianMatrix(objective.getDerivativeOrder() + 1));

         MatrixTools.multAddBlock(rhoToLinearForceJacobian, tempCoefficientJacobian, inputToPack.getTaskJacobian(), 0, startCol);
         startCol += coefficientJacobianMatrixHelper.getCoefficientSize();
      }

      objective.getObjective().get(inputToPack.getTaskObjective());

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }
}
