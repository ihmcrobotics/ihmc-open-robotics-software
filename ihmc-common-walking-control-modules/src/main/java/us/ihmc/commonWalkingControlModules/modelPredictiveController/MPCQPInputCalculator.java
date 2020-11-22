package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.CoMContinuityCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.MPCValueCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.RhoValueObjectiveCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInput;
import us.ihmc.matrixlib.MatrixTools;

public class MPCQPInputCalculator
{
   private final MPCIndexHandler indexHandler;

   private final DMatrixRMaj tempCoefficientJacobian = new DMatrixRMaj(0, 0);
   private final double gravityZ;

   public MPCQPInputCalculator(MPCIndexHandler indexHandler, double gravityZ)
   {
      this.indexHandler = indexHandler;
      this.gravityZ = -Math.abs(gravityZ);
   }

   public boolean calculateCoMContinuityObjective(QPInput inputToPack, CoMContinuityCommand objective)
   {
      inputToPack.reshape(3);
      inputToPack.setConstraintType(ConstraintType.OBJECTIVE);

      int firstSegmentNumber = objective.getFirstSegmentNumber();
      int secondSegmentNumber = firstSegmentNumber + 1;
      double firstSegmentDuration = objective.getFirstSegmentDuration();
      double omega = objective.getOmega();
      double weight = objective.getWeight();

      CoMCoefficientJacobianCalculator.calculateCoMJacobian(firstSegmentNumber,
                                                            firstSegmentDuration,
                                                            inputToPack.getTaskJacobian(),
                                                            objective.getDerivativeOrder(),
                                                            1.0);
      CoMCoefficientJacobianCalculator.calculateCoMJacobian(secondSegmentNumber,
                                                            secondSegmentNumber,
                                                            inputToPack.getTaskJacobian(),
                                                            objective.getDerivativeOrder(),
                                                            -1.0);

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

      inputToPack.getTaskObjective().zero();
      inputToPack.getTaskObjective().add(2, 0, getGravityZObjective(objective.getDerivativeOrder(), 0.0));
      inputToPack.getTaskObjective().add(2, 0, -getGravityZObjective(objective.getDerivativeOrder(), firstSegmentDuration));

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   public boolean calculateValueObjective(QPInput inputToPack, MPCValueCommand objective)
   {
      switch (objective.getValueType())
      {
         case COM:
            return calculateCoMValueObjective(inputToPack, objective);
         case VRP:
            return calculateVRPValueObjective(inputToPack, objective);
         case DCM:
            return calculateDCMValueObjective(inputToPack, objective);
         default:
            return false;
      }
   }

   private boolean calculateCoMValueObjective(QPInput inputToPack, MPCValueCommand objective)
   {
      inputToPack.reshape(3);
      inputToPack.getTaskJacobian().zero();
      inputToPack.setConstraintType(ConstraintType.OBJECTIVE);

      int segmentNumber = objective.getSegmentNumber();
      double timeOfObjective = objective.getTimeOfObjective();
      double omega = objective.getOmega();
      double weight = objective.getWeight();

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
      inputToPack.getTaskObjective().add(2, 0, -getGravityZObjective(objective.getDerivativeOrder(), timeOfObjective));

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   private boolean calculateDCMValueObjective(QPInput inputToPack, MPCValueCommand objective)
   {
      inputToPack.reshape(3);
      inputToPack.getTaskJacobian().zero();
      inputToPack.setConstraintType(ConstraintType.OBJECTIVE);

      int segmentNumber = objective.getSegmentNumber();
      double timeOfObjective = objective.getTimeOfObjective();
      double omega = objective.getOmega();
      double weight = objective.getWeight();

      CoMCoefficientJacobianCalculator.calculateDCMJacobian(segmentNumber,
                                                            omega,
                                                            timeOfObjective,
                                                            inputToPack.getTaskJacobian(),
                                                            objective.getDerivativeOrder(),
                                                            1.0);

      int startCol = indexHandler.getRhoCoefficientStartIndex(segmentNumber);
      for (int i = 0; i < objective.getNumberOfContacts(); i++)
      {
         CoefficientJacobianMatrixHelper coefficientJacobianMatrixHelper = objective.getCoefficientJacobianMatrixHelper(i);
         coefficientJacobianMatrixHelper.computeMatrices(timeOfObjective, omega);

         DMatrixRMaj rhoToLinearForceJacobian = objective.getRhoToForceMatrixHelper(i).getLinearJacobianInWorldFrame();

         tempCoefficientJacobian.set(coefficientJacobianMatrixHelper.getJacobianMatrix(objective.getDerivativeOrder()));
         CommonOps_DDRM.addEquals(tempCoefficientJacobian, 1.0 / omega, coefficientJacobianMatrixHelper.getJacobianMatrix(objective.getDerivativeOrder() + 1));

         MatrixTools.multAddBlock(rhoToLinearForceJacobian, tempCoefficientJacobian, inputToPack.getTaskJacobian(), 0, startCol);
         startCol += coefficientJacobianMatrixHelper.getCoefficientSize();
      }

      objective.getObjective().get(inputToPack.getTaskObjective());
      inputToPack.getTaskObjective().add(2, 0, -getGravityZObjective(objective.getDerivativeOrder(), timeOfObjective));
      inputToPack.getTaskObjective().add(2, 0, -getGravityZObjective(objective.getDerivativeOrder() + 1, timeOfObjective) / omega);

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   private boolean calculateVRPValueObjective(QPInput inputToPack, MPCValueCommand objective)
   {
      inputToPack.reshape(3);
      inputToPack.getTaskJacobian().zero();
      inputToPack.setConstraintType(ConstraintType.OBJECTIVE);

      int segmentNumber = objective.getSegmentNumber();
      double timeOfObjective = objective.getTimeOfObjective();
      double omega = objective.getOmega();
      double weight = objective.getWeight();
      double omega2 = omega * omega;

      CoMCoefficientJacobianCalculator.calculateVRPJacobian(segmentNumber,
                                                            omega,
                                                            timeOfObjective,
                                                            inputToPack.getTaskJacobian(),
                                                            objective.getDerivativeOrder(),
                                                            1.0);

      int startCol = indexHandler.getRhoCoefficientStartIndex(segmentNumber);
      for (int i = 0; i < objective.getNumberOfContacts(); i++)
      {
         CoefficientJacobianMatrixHelper coefficientJacobianMatrixHelper = objective.getCoefficientJacobianMatrixHelper(i);
         coefficientJacobianMatrixHelper.computeMatrices(timeOfObjective, omega);

         DMatrixRMaj rhoToLinearForceJacobian = objective.getRhoToForceMatrixHelper(i).getLinearJacobianInWorldFrame();

         tempCoefficientJacobian.reshape(coefficientJacobianMatrixHelper.getRhoSize(), coefficientJacobianMatrixHelper.getCoefficientSize());
         tempCoefficientJacobian.set(coefficientJacobianMatrixHelper.getJacobianMatrix(objective.getDerivativeOrder()));
         CommonOps_DDRM.addEquals(tempCoefficientJacobian,
                                  -1.0 / omega2,
                                  coefficientJacobianMatrixHelper.getJacobianMatrix(objective.getDerivativeOrder() + 2));

         MatrixTools.multAddBlock(rhoToLinearForceJacobian, tempCoefficientJacobian, inputToPack.getTaskJacobian(), 0, startCol);
         startCol += coefficientJacobianMatrixHelper.getCoefficientSize();
      }

      objective.getObjective().get(inputToPack.getTaskObjective());
      inputToPack.getTaskObjective().add(2, 0, -getGravityZObjective(objective.getDerivativeOrder(), timeOfObjective));
      inputToPack.getTaskObjective().add(2, 0, getGravityZObjective(objective.getDerivativeOrder() + 2, timeOfObjective) / omega2);

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   public boolean calculateRhoValueCommand(QPInput inputToPack, RhoValueObjectiveCommand command)
   {
      int problemSize = 0;

      for (int i = 0; i < command.getNumberOfContacts(); i++)
      {
         CoefficientJacobianMatrixHelper coefficientJacobianMatrixHelper = command.getCoefficientJacobianMatrixHelper(i);
         problemSize += coefficientJacobianMatrixHelper.getRhoSize();
      }

      if (problemSize < 1)
         return false;

      inputToPack.reshape(problemSize);

      int segmentNumber = command.getSegmentNumber();
      double timeOfObjective = command.getTimeOfObjective();
      double omega = command.getOmega();

      int startCol = indexHandler.getRhoCoefficientStartIndex(segmentNumber);
      for (int i = 0; i < command.getNumberOfContacts(); i++)
      {
         CoefficientJacobianMatrixHelper coefficientJacobianMatrixHelper = command.getCoefficientJacobianMatrixHelper(i);
         coefficientJacobianMatrixHelper.computeMatrices(timeOfObjective, omega);

         MatrixTools.addMatrixBlock(inputToPack.getTaskJacobian(),
                                    0,
                                    startCol,
                                    coefficientJacobianMatrixHelper.getJacobianMatrix(2),
                                    0,
                                    0,
                                    problemSize,
                                    coefficientJacobianMatrixHelper.getCoefficientSize(),
                                    1.0);
         startCol += coefficientJacobianMatrixHelper.getCoefficientSize();
      }

      for (int i = 0; i < problemSize; i++)
      {
         inputToPack.getTaskObjective().set(i, 0, command.getObjective());
      }
      inputToPack.setConstraintType(command.getConstraintType());

      return true;
   }

   private double getGravityZObjective(int derivativeOrder, double time)
   {
      switch (derivativeOrder)
      {
         case 0:
            return 0.5 * time * time * gravityZ;
         case 1:
            return time * gravityZ;
         case 2:
            return gravityZ;
         case 3:
            return 0.0;
         default:
            throw new IllegalArgumentException("Derivative order must be less than 4.");
      }
   }
}
