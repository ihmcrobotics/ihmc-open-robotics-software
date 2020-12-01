package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.CoMContinuityCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.MPCValueCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.RhoValueObjectiveCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.matrixlib.MatrixTools;

public class MPCQPInputCalculator
{
   public static final double sufficientlyLongTime = 100.0;
   public static final double sufficientlyLargeValue = 1e5;

   private final MPCIndexHandler indexHandler;

   private final DMatrixRMaj tempCoefficientJacobian = new DMatrixRMaj(0, 0);
   private final double gravityZ;

   public MPCQPInputCalculator(MPCIndexHandler indexHandler, double gravityZ)
   {
      this.indexHandler = indexHandler;
      this.gravityZ = -Math.abs(gravityZ);
   }

   public boolean calculateCoMContinuityObjective(QPInputTypeA inputToPack, CoMContinuityCommand objective)
   {
      inputToPack.reshape(3);
      inputToPack.setConstraintType(objective.getConstraintType());

      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();

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
                                                            0.0,
                                                            inputToPack.getTaskJacobian(),
                                                            objective.getDerivativeOrder(),
                                                            -1.0);

      int startCol = indexHandler.getRhoCoefficientStartIndex(firstSegmentNumber);
      for (int i = 0; i < objective.getFirstSegmentNumberOfContacts(); i++)
      {
         ContactPlaneHelper contactPlaneHelper = objective.getFirstSegmentContactPlaneHelper(i);
         contactPlaneHelper.computeJacobians(firstSegmentDuration, omega);

         MatrixTools.addMatrixBlock(inputToPack.getTaskJacobian(), 0, startCol, contactPlaneHelper.getLinearJacobian(objective.getDerivativeOrder()), 0, 0, 3, contactPlaneHelper.getCoefficientSize(), 1.0);
         startCol += contactPlaneHelper.getCoefficientSize();
      }

      startCol = indexHandler.getRhoCoefficientStartIndex(secondSegmentNumber);
      for (int i = 0; i < objective.getSecondSegmentNumberOfContacts(); i++)
      {
         ContactPlaneHelper contactPlaneHelper = objective.getSecondSegmentContactPlaneHelper(i);
         contactPlaneHelper.computeJacobians(0.0, omega);

         MatrixTools.addMatrixBlock(inputToPack.getTaskJacobian(), 0, startCol, contactPlaneHelper.getLinearJacobian(objective.getDerivativeOrder()), 0, 0, 3, contactPlaneHelper.getCoefficientSize(), -1.0);
         startCol += contactPlaneHelper.getCoefficientSize();
      }

      inputToPack.getTaskObjective().zero();
      inputToPack.getTaskObjective().add(2, 0, getGravityZObjective(objective.getDerivativeOrder(), 0.0));
      inputToPack.getTaskObjective().add(2, 0, -getGravityZObjective(objective.getDerivativeOrder(), firstSegmentDuration));

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   public boolean calculateValueObjective(QPInputTypeA inputToPack, MPCValueCommand objective)
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

   private boolean calculateCoMValueObjective(QPInputTypeA inputToPack, MPCValueCommand objective)
   {
      inputToPack.reshape(3);
      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();
      inputToPack.setConstraintType(objective.getConstraintType());

      int segmentNumber = objective.getSegmentNumber();
      double timeOfObjective = objective.getTimeOfObjective();
      double omega = objective.getOmega();
      double weight = objective.getWeight();

      CoMCoefficientJacobianCalculator.calculateCoMJacobian(segmentNumber, timeOfObjective, inputToPack.getTaskJacobian(), objective.getDerivativeOrder(), 1.0);

      int startCol = indexHandler.getRhoCoefficientStartIndex(segmentNumber);
      for (int i = 0; i < objective.getNumberOfContacts(); i++)
      {
         ContactPlaneHelper contactPlaneHelper = objective.getContactPlaneHelper(i);
         contactPlaneHelper.computeJacobians(timeOfObjective, omega);

         DMatrixRMaj jacobian = contactPlaneHelper.getLinearJacobian(objective.getDerivativeOrder());

         MatrixTools.addMatrixBlock(inputToPack.getTaskJacobian(), 0, startCol, jacobian, 0, 0, jacobian.getNumRows(), jacobian.getNumCols(), 1.0);
         startCol += jacobian.getNumCols();
      }

      objective.getObjective().get(inputToPack.getTaskObjective());
      inputToPack.getTaskObjective().add(2, 0, -getGravityZObjective(objective.getDerivativeOrder(), timeOfObjective));

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   private boolean calculateDCMValueObjective(QPInputTypeA inputToPack, MPCValueCommand objective)
   {
      inputToPack.reshape(3);
      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();
      inputToPack.setConstraintType(objective.getConstraintType());

      int segmentNumber = objective.getSegmentNumber();
      double timeOfObjective = objective.getTimeOfObjective();
      double omega = objective.getOmega();
      double weight = objective.getWeight();

      int objectiveOrder = objective.getDerivativeOrder();
      int objectiveHigherOrder = objectiveOrder + 1;

      CoMCoefficientJacobianCalculator.calculateDCMJacobian(segmentNumber,
                                                            omega,
                                                            timeOfObjective,
                                                            inputToPack.getTaskJacobian(),
                                                            objectiveOrder,
                                                            1.0);

      int startCol = indexHandler.getRhoCoefficientStartIndex(segmentNumber);

      for (int i = 0; i < objective.getNumberOfContacts(); i++)
      {
         ContactPlaneHelper contactPlaneHelper = objective.getContactPlaneHelper(i);
         contactPlaneHelper.computeJacobians(timeOfObjective, omega);

         tempCoefficientJacobian.reshape(3, contactPlaneHelper.getCoefficientSize());
         CommonOps_DDRM.add(contactPlaneHelper.getLinearJacobian(objectiveOrder), 1.0 / omega, contactPlaneHelper.getLinearJacobian(objectiveHigherOrder), tempCoefficientJacobian);
         MatrixTools.addMatrixBlock(inputToPack.getTaskJacobian(), 0, startCol, tempCoefficientJacobian, 0, 0, 3, contactPlaneHelper.getCoefficientSize(), 1.0);

         startCol += contactPlaneHelper.getCoefficientSize();
      }

      objective.getObjective().get(inputToPack.getTaskObjective());
      inputToPack.getTaskObjective().add(2, 0, -getGravityZObjective(objectiveOrder, timeOfObjective));
      inputToPack.getTaskObjective().add(2, 0, -getGravityZObjective(objectiveHigherOrder, timeOfObjective) / omega);

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   private boolean calculateVRPValueObjective(QPInputTypeA inputToPack, MPCValueCommand objective)
   {
      inputToPack.reshape(3);
      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();
      inputToPack.setConstraintType(objective.getConstraintType());

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
         ContactPlaneHelper contactPlaneHelper = objective.getContactPlaneHelper(i);
         contactPlaneHelper.computeJacobians(timeOfObjective, omega);

         tempCoefficientJacobian.reshape(3, contactPlaneHelper.getCoefficientSize());
         CommonOps_DDRM.add(contactPlaneHelper.getLinearJacobian(objective.getDerivativeOrder()),
                            -1.0 / omega2,
                            contactPlaneHelper.getLinearJacobian(objective.getDerivativeOrder() + 2),
                            tempCoefficientJacobian);

         MatrixTools.addMatrixBlock(inputToPack.getTaskJacobian(), 0, startCol, tempCoefficientJacobian, 0, 0, 3, contactPlaneHelper.getCoefficientSize(), 1.0);
         startCol += contactPlaneHelper.getCoefficientSize();
      }

      objective.getObjective().get(inputToPack.getTaskObjective());
      inputToPack.getTaskObjective().add(2, 0, -getGravityZObjective(objective.getDerivativeOrder(), timeOfObjective));
      inputToPack.getTaskObjective().add(2, 0, getGravityZObjective(objective.getDerivativeOrder() + 2, timeOfObjective) / omega2);

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   public boolean calculateRhoValueCommand(QPInputTypeA inputToPack, RhoValueObjectiveCommand command)
   {
      int problemSize = 0;

      for (int i = 0; i < command.getNumberOfContacts(); i++)
      {
         problemSize += command.getContactPlaneHelper(i).getRhoSize();
      }

      if (problemSize < 1)
         return false;

      inputToPack.reshape(problemSize);
      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();

      int segmentNumber = command.getSegmentNumber();
      double timeOfObjective = command.getTimeOfObjective();
      double omega = command.getOmega();

      int startCol = indexHandler.getRhoCoefficientStartIndex(segmentNumber);
      int startRow = 0;
      for (int i = 0; i < command.getNumberOfContacts(); i++)
      {
         ContactPlaneHelper contactPlaneHelper = command.getContactPlaneHelper(i);
         contactPlaneHelper.computeJacobians(timeOfObjective, omega);

         MatrixTools.addMatrixBlock(inputToPack.getTaskJacobian(),
                                    startRow,
                                    startCol,
                                    contactPlaneHelper.getRhoAccelerationJacobian(),
                                    0,
                                    0,
                                    contactPlaneHelper.getRhoSize(),
                                    contactPlaneHelper.getCoefficientSize(),
                                    1.0);
         startRow += contactPlaneHelper.getRhoSize();
         startCol += contactPlaneHelper.getCoefficientSize();
      }

      if (command.getUseScalarObjective())
      {
         for (int i = 0; i < problemSize; i++)
            inputToPack.getTaskObjective().set(i, 0, command.getScalarObjective());
      }
      else
      {
         inputToPack.getTaskObjective().set(command.getObjectiveVector());
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
