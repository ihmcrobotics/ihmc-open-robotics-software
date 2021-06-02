package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPoint;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeC;

/**
 * This is a helper class that is meant to convert {@link MPCCommand}s to QP inputs that can be consumed by quadratic program solver.
 */
public class MPCQPInputCalculator
{
   public static final double sufficientlyLongTime = 5.0;
   public static final double sufficientlyLargeValue = 1e5;

   private final LinearMPCIndexHandler indexHandler;

   private final VRPTrackingCostCalculator vrpTrackingCostCalculator;

   private final double gravityZ;

   public MPCQPInputCalculator(LinearMPCIndexHandler indexHandler, double gravityZ)
   {
      this.indexHandler = indexHandler;
      this.vrpTrackingCostCalculator = new VRPTrackingCostCalculator(indexHandler, gravityZ);
      this.gravityZ = -Math.abs(gravityZ);
   }

   /**
    * Computes a {@link QPInputTypeA} from a {@link MPCContinuityCommand}. This can consist of a continuity command that is either an objective or equality
    * constraint.
    *
    * @param inputToPack QP Input that contains the encoded continuity command
    * @param objective continuity command to process
    * @return whether or not the calculation was successful.
    */
   public int calculateContinuityObjective(QPInputTypeA inputToPack, MPCContinuityCommand objective)
   {
      switch (objective.getValueType())
      {
         case COM:
            return calculateCoMContinuityObjective(inputToPack, objective);
         case VRP:
            return calculateVRPContinuityObjective(inputToPack, objective);
         case DCM:
            throw new IllegalArgumentException();
         default:
            return -1;
      }
   }

   public int calculateCompactContinuityObjective(QPInputTypeA inputToPack, MPCContinuityCommand objective)
   {
      switch (objective.getValueType())
      {
         case COM:
            return calculateCompactCoMContinuityObjective(inputToPack, objective);
         case VRP:
            return calculateCompactVRPContinuityObjective(inputToPack, objective);
         case DCM:
            throw new IllegalArgumentException();
         default:
            return -1;
      }
   }

   /**
    * Computes a {@link QPInputTypeA} from a {@link MPCContinuityCommand} if {@link MPCContinuityCommand#getValueType()} indicates the center of mass. This can
    * consist of a continuity command that is either an objective or equality constraint.
    *
    * @param inputToPack QP Input that contains the encoded continuity command
    * @param objective continuity command to process
    * @return whether or not the calculation was successful.
    */
   public int calculateCoMContinuityObjective(QPInputTypeA inputToPack, MPCContinuityCommand objective)
   {
      int variableSize = indexHandler.getTotalProblemSize();
      int firstSegmentNumber = objective.getFirstSegmentNumber();
      int secondSegmentNumber = firstSegmentNumber + 1;
      int firstCoMStartIndex = indexHandler.getComCoefficientStartIndex(firstSegmentNumber);
      int secondCoMStartIndex = indexHandler.getComCoefficientStartIndex(secondSegmentNumber);
      int firstRhoStartIndex = indexHandler.getRhoCoefficientStartIndex(firstSegmentNumber);
      int secondRhoStartIndex = indexHandler.getRhoCoefficientStartIndex(secondSegmentNumber);

      if (calculateCoMContinuityObjectiveInternal(inputToPack,
                                                  objective,
                                                  variableSize,
                                                  firstCoMStartIndex,
                                                  secondCoMStartIndex,
                                                  firstRhoStartIndex,
                                                  secondRhoStartIndex))
         return 0;

      return -1;
   }

   /**
    * Computes a {@link QPInputTypeA} from a {@link MPCContinuityCommand} if {@link MPCContinuityCommand#getValueType()} indicates the virtual repellent point.
    * This can consist of a continuity command that is either an objective or equality constraint.
    *
    * @param inputToPack QP Input that contains the encoded continuity command
    * @param objective continuity command to process
    * @return whether or not the calculation was successful.
    */
   public int calculateVRPContinuityObjective(QPInputTypeA inputToPack, MPCContinuityCommand objective)
   {
      int variableSize = indexHandler.getTotalProblemSize();
      int firstSegmentNumber = objective.getFirstSegmentNumber();
      int secondSegmentNumber = firstSegmentNumber + 1;
      int firstCoMStartIndex = indexHandler.getComCoefficientStartIndex(firstSegmentNumber);
      int secondCoMStartIndex = indexHandler.getComCoefficientStartIndex(secondSegmentNumber);
      int firstRhoStartIndex = indexHandler.getRhoCoefficientStartIndex(firstSegmentNumber);
      int secondRhoStartIndex = indexHandler.getRhoCoefficientStartIndex(secondSegmentNumber);

      if (calculateVRPContinuityObjectiveInternal(inputToPack,
                                                  objective,
                                                  variableSize,
                                                  firstCoMStartIndex,
                                                  secondCoMStartIndex,
                                                  firstRhoStartIndex,
                                                  secondRhoStartIndex))
         return 0;

      return -1;
   }

   public int calculateCompactCoMContinuityObjective(QPInputTypeA inputToPack, MPCContinuityCommand objective)
   {
      int firstSegmentNumber = objective.getFirstSegmentNumber();
      int secondSegmentNumber = firstSegmentNumber + 1;
      int variableSize = 2 * LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(firstSegmentNumber)
                         + indexHandler.getRhoCoefficientsInSegment(secondSegmentNumber);
      int firstCoMStartIndex = indexHandler.getComCoefficientStartIndex(firstSegmentNumber);
      int secondCoMStartIndex = indexHandler.getComCoefficientStartIndex(secondSegmentNumber);
      int firstRhoStartIndex = indexHandler.getRhoCoefficientStartIndex(firstSegmentNumber);
      int secondRhoStartIndex = indexHandler.getRhoCoefficientStartIndex(secondSegmentNumber);

      secondCoMStartIndex -= firstCoMStartIndex;
      firstRhoStartIndex -= firstCoMStartIndex;
      secondRhoStartIndex -= firstRhoStartIndex;

      if (calculateCoMContinuityObjectiveInternal(inputToPack, objective, variableSize, 0, secondCoMStartIndex, firstRhoStartIndex, secondRhoStartIndex))
         return firstCoMStartIndex;

      return -1;
   }

   public int calculateCompactVRPContinuityObjective(QPInputTypeA inputToPack, MPCContinuityCommand objective)
   {
      int firstSegmentNumber = objective.getFirstSegmentNumber();
      int secondSegmentNumber = firstSegmentNumber + 1;
      int variableSize = 2 * LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(firstSegmentNumber)
                         + indexHandler.getRhoCoefficientsInSegment(secondSegmentNumber);
      int firstCoMStartIndex = indexHandler.getComCoefficientStartIndex(firstSegmentNumber);
      int secondCoMStartIndex = indexHandler.getComCoefficientStartIndex(secondSegmentNumber);
      int firstRhoStartIndex = indexHandler.getRhoCoefficientStartIndex(firstSegmentNumber);
      int secondRhoStartIndex = indexHandler.getRhoCoefficientStartIndex(secondSegmentNumber);

      secondCoMStartIndex -= firstCoMStartIndex;
      firstRhoStartIndex -= firstCoMStartIndex;
      secondRhoStartIndex -= firstRhoStartIndex;

      if (calculateVRPContinuityObjectiveInternal(inputToPack, objective, variableSize, 0, secondCoMStartIndex, firstRhoStartIndex, secondRhoStartIndex))
         return firstCoMStartIndex;

      return -1;
   }

   public boolean calculateCoMContinuityObjectiveInternal(QPInputTypeA inputToPack,
                                                          MPCContinuityCommand objective,
                                                          int variableSize,
                                                          int firstComStartCol,
                                                          int secondComStartCol,
                                                          int firstRhoStartCol,
                                                          int secondRhoStartCol)
   {
      inputToPack.setNumberOfVariables(variableSize);
      inputToPack.reshape(3);
      inputToPack.setConstraintType(objective.getConstraintType());

      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();

      double firstSegmentDuration = objective.getFirstSegmentDuration();
      double omega = objective.getOmega();
      double weight = objective.getWeight();

      CoMCoefficientJacobianCalculator.calculateCoMJacobian(firstComStartCol,
                                                            firstSegmentDuration,
                                                            inputToPack.getTaskJacobian(),
                                                            objective.getDerivativeOrder(),
                                                            1.0);
      CoMCoefficientJacobianCalculator.calculateCoMJacobian(secondComStartCol, 0.0, inputToPack.getTaskJacobian(), objective.getDerivativeOrder(), -1.0);

      for (int i = 0; i < objective.getFirstSegmentNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlaneHelper = objective.getFirstSegmentContactPlaneHelper(i);
         ContactPlaneJacobianCalculator.computeLinearJacobian(objective.getDerivativeOrder(),
                                                              firstSegmentDuration,
                                                              omega,
                                                              firstRhoStartCol,
                                                              contactPlaneHelper,
                                                              inputToPack.getTaskJacobian());

         firstRhoStartCol += contactPlaneHelper.getCoefficientSize();
      }

      for (int i = 0; i < objective.getSecondSegmentNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlaneHelper = objective.getSecondSegmentContactPlaneHelper(i);
         ContactPlaneJacobianCalculator.computeLinearJacobian(-1.0,
                                                              objective.getDerivativeOrder(),
                                                              0.0,
                                                              omega,
                                                              secondRhoStartCol,
                                                              contactPlaneHelper,
                                                              inputToPack.getTaskJacobian());

         secondRhoStartCol += contactPlaneHelper.getCoefficientSize();
      }

      inputToPack.getTaskObjective().zero();
      inputToPack.getTaskObjective().add(2, 0, getGravityZObjective(objective.getDerivativeOrder(), 0.0));
      inputToPack.getTaskObjective().add(2, 0, -getGravityZObjective(objective.getDerivativeOrder(), firstSegmentDuration));

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   public boolean calculateVRPContinuityObjectiveInternal(QPInputTypeA inputToPack,
                                                          MPCContinuityCommand objective,
                                                          int variableSize,
                                                          int firstComStartCol,
                                                          int secondComStartCol,
                                                          int firstRhoStartCol,
                                                          int secondRhoStartCol)
   {
      inputToPack.setNumberOfVariables(variableSize);
      inputToPack.reshape(3);
      inputToPack.setConstraintType(objective.getConstraintType());

      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();

      double firstSegmentDuration = objective.getFirstSegmentDuration();
      double omega = objective.getOmega();
      double omega2 = omega * omega;
      double weight = objective.getWeight();

      CoMCoefficientJacobianCalculator.calculateVRPJacobian(firstComStartCol,
                                                            omega,
                                                            firstSegmentDuration,
                                                            inputToPack.getTaskJacobian(),
                                                            objective.getDerivativeOrder(),
                                                            1.0);
      CoMCoefficientJacobianCalculator.calculateVRPJacobian(secondComStartCol, omega, 0.0, inputToPack.getTaskJacobian(), objective.getDerivativeOrder(), -1.0);

      for (int i = 0; i < objective.getFirstSegmentNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlaneHelper = objective.getFirstSegmentContactPlaneHelper(i);

         ContactPlaneJacobianCalculator.computeLinearJacobian(objective.getDerivativeOrder(),
                                                              firstSegmentDuration,
                                                              omega,
                                                              firstRhoStartCol,
                                                              contactPlaneHelper,
                                                              inputToPack.getTaskJacobian());
         ContactPlaneJacobianCalculator.computeLinearJacobian(-1.0 / omega2,
                                                              objective.getDerivativeOrder() + 2,
                                                              firstSegmentDuration,
                                                              omega,
                                                              firstRhoStartCol,
                                                              contactPlaneHelper,
                                                              inputToPack.getTaskJacobian());

         firstRhoStartCol += contactPlaneHelper.getCoefficientSize();
      }

      for (int i = 0; i < objective.getSecondSegmentNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlaneHelper = objective.getSecondSegmentContactPlaneHelper(i);

         ContactPlaneJacobianCalculator.computeLinearJacobian(-1.0,
                                                              objective.getDerivativeOrder(),
                                                              0.0,
                                                              omega,
                                                              secondRhoStartCol,
                                                              contactPlaneHelper,
                                                              inputToPack.getTaskJacobian());
         ContactPlaneJacobianCalculator.computeLinearJacobian(1.0 / omega2,
                                                              objective.getDerivativeOrder() + 2,
                                                              0.0,
                                                              omega,
                                                              secondRhoStartCol,
                                                              contactPlaneHelper,
                                                              inputToPack.getTaskJacobian());

         secondRhoStartCol += contactPlaneHelper.getCoefficientSize();
      }

      inputToPack.getTaskObjective().zero();
      inputToPack.getTaskObjective().add(2, 0, getGravityZObjective(objective.getDerivativeOrder(), 0.0));
      inputToPack.getTaskObjective().add(2, 0, -getGravityZObjective(objective.getDerivativeOrder(), firstSegmentDuration));

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   public boolean calculateForceMinimizationObjective(QPInputTypeC inputToPack, ForceObjectiveCommand objective)
   {
      throw new RuntimeException("This objective is not yet properly implemented.");
      /*
      inputToPack.setNumberOfVariables(indexHandler.getTotalProblemSize());
      inputToPack.reshape();

      inputToPack.getDirectCostHessian().zero();
      inputToPack.getDirectCostGradient().zero();

      int segmentNumber = objective.getSegmentNumber();
      double weight = objective.getWeight();

      int startCol = indexHandler.getRhoCoefficientStartIndex(segmentNumber);
      for (int i = 0; i < objective.getNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlaneHelper = objective.getContactPlaneHelper(i);

         MatrixTools.setMatrixBlock(inputToPack.getDirectCostHessian(),
                                    startCol,
                                    startCol,
                                    contactPlaneHelper.getAccelerationIntegrationHessian(),
                                    0,
                                    0,
                                    contactPlaneHelper.getCoefficientSize(),
                                    contactPlaneHelper.getCoefficientSize(),
                                    1.0);
         MatrixTools.setMatrixBlock(inputToPack.getDirectCostGradient(),
                                    startCol,
                                    0,
                                    contactPlaneHelper.getAccelerationIntegrationGradient(),
                                    0,
                                    0,
                                    contactPlaneHelper.getCoefficientSize(),
                                    1,
                                    1.0);
         startCol += contactPlaneHelper.getCoefficientSize();
      }

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;

       */
   }

   public int calculateForceTrackingObjective(QPInputTypeC inputToPack, ForceTrackingCommand objective)
   {
      int segmentNumber = objective.getSegmentNumber();

      inputToPack.setNumberOfVariables(indexHandler.getRhoCoefficientsInSegment(segmentNumber));
      inputToPack.reshape();

      inputToPack.getDirectCostHessian().zero();
      inputToPack.getDirectCostGradient().zero();

      double weight = objective.getWeight();
      double duration = objective.getSegmentDuration();

      int startCol = 0;
      for (int i = 0; i < objective.getNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlane = objective.getContactPlaneHelper(i);

         IntegrationInputCalculator.computeForceTrackingMatrix(startCol,
                                                               inputToPack.getDirectCostGradient(),
                                                               inputToPack.getDirectCostHessian(),
                                                               contactPlane,
                                                               duration,
                                                               objective.getOmega(),
                                                               objective.getObjectiveValue());

         startCol += contactPlane.getCoefficientSize();
      }

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return indexHandler.getRhoCoefficientStartIndex(segmentNumber);
   }

   public int calculateForceRateTrackingObjective(QPInputTypeC inputToPack, ForceRateTrackingCommand objective)
   {
      int segmentNumber = objective.getSegmentNumber();

      inputToPack.setNumberOfVariables(indexHandler.getRhoCoefficientsInSegment(segmentNumber));
      inputToPack.reshape();

      inputToPack.getDirectCostHessian().zero();
      inputToPack.getDirectCostGradient().zero();

      double weight = objective.getWeight();
      double duration = objective.getSegmentDuration();

      int startCol = 0;
      for (int i = 0; i < objective.getNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlane = objective.getContactPlaneHelper(i);

         IntegrationInputCalculator.computeForceRateTrackingMatrix(startCol,
                                                               inputToPack.getDirectCostGradient(),
                                                               inputToPack.getDirectCostHessian(),
                                                               contactPlane,
                                                               duration,
                                                               objective.getOmega(),
                                                               objective.getObjectiveValue());

         startCol += contactPlane.getCoefficientSize();
      }

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return indexHandler.getRhoCoefficientStartIndex(segmentNumber);
   }

   public int calculateRhoTrackingObjective(QPInputTypeC inputToPack, RhoTrackingCommand objective)
   {
      int segmentNumber = objective.getSegmentNumber();

      inputToPack.setNumberOfVariables(indexHandler.getRhoCoefficientsInSegment(segmentNumber));
      inputToPack.reshape();

      inputToPack.getDirectCostHessian().zero();
      inputToPack.getDirectCostGradient().zero();

      double weight = objective.getWeight();
      double duration = objective.getSegmentDuration();

      int startCol = 0;
      for (int i = 0; i < objective.getNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlane = objective.getContactPlaneHelper(i);

         IntegrationInputCalculator.computeRhoAccelerationTrackingMatrix(startCol,
                                                                         inputToPack.getDirectCostGradient(),
                                                                         inputToPack.getDirectCostHessian(),
                                                                         contactPlane.getRhoSize(),
                                                                         duration,
                                                                         objective.getOmega(),
                                                                         objective.getObjectiveValue());

         startCol += contactPlane.getCoefficientSize();
      }

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return indexHandler.getRhoCoefficientStartIndex(segmentNumber);
   }

   /**
    * Processes a {@link MPCValueCommand} to compute a {@link QPInputTypeA}. This can then be fed to MPC QP solver.
    *
    * @param inputToPack QP input to calculate
    * @param objective value command to process
    * @return whether or not the calculation was successful. -1 if it's no successful
    */
   public int calculateValueObjective(QPInputTypeA inputToPack, MPCValueCommand objective)
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
            return -1;
      }
   }

   public int calculateCompactValueObjective(QPInputTypeA inputToPack, MPCValueCommand objective)
   {
      switch (objective.getValueType())
      {
         case COM:
            return calculateCompactCoMValueObjective(inputToPack, objective);
         case VRP:
            return calculateCompactVRPValueObjective(inputToPack, objective);
         case DCM:
            return calculateCompactDCMValueObjective(inputToPack, objective);
         default:
            return -1;
      }
   }

   private int calculateCoMValueObjective(QPInputTypeA inputToPack, MPCValueCommand objective)
   {
      int variableSize = indexHandler.getTotalProblemSize();
      int segmentNumber = objective.getSegmentNumber();
      int comStartCol = indexHandler.getComCoefficientStartIndex(segmentNumber);
      int rhoStartCol = indexHandler.getRhoCoefficientStartIndex(segmentNumber);
      inputToPack.setNumberOfVariables(variableSize);

      if (calculateCoMValueObjectiveInternal(inputToPack, objective, variableSize, comStartCol, rhoStartCol))
         return 0;

      return -1;
   }

   private int calculateVRPValueObjective(QPInputTypeA inputToPack, MPCValueCommand objective)
   {
      int variableSize = indexHandler.getTotalProblemSize();
      int segmentNumber = objective.getSegmentNumber();
      int comStartCol = indexHandler.getComCoefficientStartIndex(segmentNumber);
      int rhoStartCol = indexHandler.getRhoCoefficientStartIndex(segmentNumber);
      inputToPack.setNumberOfVariables(variableSize);

      if (calculateVRPValueObjectiveInternal(inputToPack, objective, variableSize, comStartCol, rhoStartCol))
         return 0;

      return -1;
   }

   private int calculateDCMValueObjective(QPInputTypeA inputToPack, MPCValueCommand objective)
   {
      int variableSize = indexHandler.getTotalProblemSize();
      int segmentNumber = objective.getSegmentNumber();
      int comStartCol = indexHandler.getComCoefficientStartIndex(segmentNumber);
      int rhoStartCol = indexHandler.getRhoCoefficientStartIndex(segmentNumber);
      inputToPack.setNumberOfVariables(variableSize);

      if (calculateDCMValueObjectiveInternal(inputToPack, objective, variableSize, comStartCol, rhoStartCol))
         return 0;

      return -1;
   }

   private int calculateCompactCoMValueObjective(QPInputTypeA inputToPack, MPCValueCommand objective)
   {
      int segmentNumber = objective.getSegmentNumber();
      int variableSize = LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(segmentNumber);
      int comStartCol = 0;
      int rhoStartCol = LinearMPCIndexHandler.comCoefficientsPerSegment;

      if (calculateCoMValueObjectiveInternal(inputToPack, objective, variableSize, comStartCol, rhoStartCol))
         return indexHandler.getComCoefficientStartIndex(segmentNumber);

      return -1;
   }

   private int calculateCompactVRPValueObjective(QPInputTypeA inputToPack, MPCValueCommand objective)
   {
      int segmentNumber = objective.getSegmentNumber();
      int variableSize = LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(segmentNumber);
      int comStartCol = 0;
      int rhoStartCol = LinearMPCIndexHandler.comCoefficientsPerSegment;

      if (calculateVRPValueObjectiveInternal(inputToPack, objective, variableSize, comStartCol, rhoStartCol))
         return indexHandler.getComCoefficientStartIndex(segmentNumber);

      return -1;
   }

   private int calculateCompactDCMValueObjective(QPInputTypeA inputToPack, MPCValueCommand objective)
   {
      int segmentNumber = objective.getSegmentNumber();
      int variableSize = LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(segmentNumber);
      int comStartCol = 0;
      int rhoStartCol = LinearMPCIndexHandler.comCoefficientsPerSegment;

      if (calculateDCMValueObjectiveInternal(inputToPack, objective, variableSize, comStartCol, rhoStartCol))
         return indexHandler.getComCoefficientStartIndex(segmentNumber);

      return -1;
   }

   private boolean calculateCoMValueObjectiveInternal(QPInputTypeA inputToPack,
                                                      MPCValueCommand objective,
                                                      int numberOfVariables,
                                                      int comStartCol,
                                                      int rhoStartCol)
   {
      inputToPack.setNumberOfVariables(numberOfVariables);
      inputToPack.reshape(3);
      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();
      inputToPack.setConstraintType(objective.getConstraintType());

      double timeOfObjective = objective.getTimeOfObjective();
      double omega = objective.getOmega();
      double weight = objective.getWeight();

      CoMCoefficientJacobianCalculator.calculateCoMJacobian(comStartCol, timeOfObjective, inputToPack.getTaskJacobian(), objective.getDerivativeOrder(), 1.0);

      for (int i = 0; i < objective.getNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlaneHelper = objective.getContactPlaneHelper(i);
         ContactPlaneJacobianCalculator.computeLinearJacobian(objective.getDerivativeOrder(),
                                                              timeOfObjective,
                                                              omega,
                                                              rhoStartCol,
                                                              contactPlaneHelper,
                                                              inputToPack.getTaskJacobian());

         rhoStartCol += contactPlaneHelper.getCoefficientSize();
      }

      objective.getObjective().get(inputToPack.getTaskObjective());
      inputToPack.getTaskObjective().add(2, 0, -getGravityZObjective(objective.getDerivativeOrder(), timeOfObjective));

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   private boolean calculateDCMValueObjectiveInternal(QPInputTypeA inputToPack,
                                                      MPCValueCommand objective,
                                                      int numberOfVariables,
                                                      int comStartCol,
                                                      int rhoStartCol)
   {
      inputToPack.setNumberOfVariables(numberOfVariables);
      inputToPack.reshape(3);
      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();
      inputToPack.setConstraintType(objective.getConstraintType());

      double timeOfObjective = objective.getTimeOfObjective();
      double omega = objective.getOmega();
      double weight = objective.getWeight();

      int objectiveOrder = objective.getDerivativeOrder();
      int objectiveHigherOrder = objectiveOrder + 1;

      CoMCoefficientJacobianCalculator.calculateDCMJacobian(comStartCol, omega, timeOfObjective, inputToPack.getTaskJacobian(), objectiveOrder, 1.0);

      for (int i = 0; i < objective.getNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlaneHelper = objective.getContactPlaneHelper(i);

         ContactPlaneJacobianCalculator.computeLinearJacobian(objectiveOrder,
                                                              timeOfObjective,
                                                              omega,
                                                              rhoStartCol,
                                                              contactPlaneHelper,
                                                              inputToPack.getTaskJacobian());
         ContactPlaneJacobianCalculator.computeLinearJacobian(1.0 / omega,
                                                              objectiveHigherOrder,
                                                              timeOfObjective,
                                                              omega,
                                                              rhoStartCol,
                                                              contactPlaneHelper,
                                                              inputToPack.getTaskJacobian());

         rhoStartCol += contactPlaneHelper.getCoefficientSize();
      }

      objective.getObjective().get(inputToPack.getTaskObjective());
      inputToPack.getTaskObjective().add(2, 0, -getGravityZObjective(objectiveOrder, timeOfObjective));
      inputToPack.getTaskObjective().add(2, 0, -getGravityZObjective(objectiveHigherOrder, timeOfObjective) / omega);

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   private boolean calculateVRPValueObjectiveInternal(QPInputTypeA inputToPack,
                                                      MPCValueCommand objective,
                                                      int numberOfVariables,
                                                      int comStartIdx,
                                                      int rhoStartIdx)
   {
      inputToPack.setNumberOfVariables(numberOfVariables);
      inputToPack.reshape(3);
      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();
      inputToPack.setConstraintType(objective.getConstraintType());

      double timeOfObjective = objective.getTimeOfObjective();
      double omega = objective.getOmega();
      double weight = objective.getWeight();
      double omega2 = omega * omega;

      CoMCoefficientJacobianCalculator.calculateVRPJacobian(comStartIdx,
                                                            omega,
                                                            timeOfObjective,
                                                            inputToPack.getTaskJacobian(),
                                                            objective.getDerivativeOrder(),
                                                            1.0);

      for (int i = 0; i < objective.getNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlaneHelper = objective.getContactPlaneHelper(i);

         ContactPlaneJacobianCalculator.computeLinearJacobian(objective.getDerivativeOrder(),
                                                              timeOfObjective,
                                                              omega,
                                                              rhoStartIdx,
                                                              contactPlaneHelper,
                                                              inputToPack.getTaskJacobian());
         ContactPlaneJacobianCalculator.computeLinearJacobian(-1.0 / omega2,
                                                              objective.getDerivativeOrder() + 2,
                                                              timeOfObjective,
                                                              omega,
                                                              rhoStartIdx,
                                                              contactPlaneHelper,
                                                              inputToPack.getTaskJacobian());

         rhoStartIdx += contactPlaneHelper.getCoefficientSize();
      }

      objective.getObjective().get(inputToPack.getTaskObjective());
      inputToPack.getTaskObjective().add(2, 0, -getGravityZObjective(objective.getDerivativeOrder(), timeOfObjective));
      inputToPack.getTaskObjective().add(2, 0, getGravityZObjective(objective.getDerivativeOrder() + 2, timeOfObjective) / omega2);

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   /**
    * Calculates a {@link QPInputTypeA} based on a {@link RhoObjectiveCommand}. Is typically used to set upper and lower bounds for the generalized contact
    * values.
    *
    * @param inputToPack QP input to compute
    * @param command command to process
    * @return whether or not the calculation was successful.
    */
   public int calculateRhoValueCommand(QPInputTypeA inputToPack, RhoObjectiveCommand command)
   {
      int numberOfVariables = indexHandler.getTotalProblemSize();
      int rhoStartIdx = indexHandler.getRhoCoefficientStartIndex(command.getSegmentNumber());

      if (calculateRhoValueCommandInternal(inputToPack, command, numberOfVariables, rhoStartIdx))
         return 0;

      return -1;
   }

   /**
    * Calculates a {@link QPInputTypeA} based on a {@link RhoObjectiveCommand}. Is typically used to set upper and lower bounds for the generalized contact
    * values.
    *
    * @param inputToPack QP input to compute
    * @param command command to process
    * @return whether or not the calculation was successful.
    */
   public int calculateCompactRhoValueCommand(QPInputTypeA inputToPack, RhoObjectiveCommand command)
   {
      int numberOfVariables = 0;
      for (int i = 0; i < command.getNumberOfContacts(); i++)
         numberOfVariables += command.getContactPlaneHelper(i).getCoefficientSize();

      int segmentNumber = command.getSegmentNumber();

      if (calculateRhoValueCommandInternal(inputToPack, command, numberOfVariables, 0))
         return indexHandler.getRhoCoefficientStartIndex(segmentNumber);

      return -1;
   }

   public boolean calculateRhoValueCommandInternal(QPInputTypeA inputToPack, RhoObjectiveCommand command, int numberOfVariables, int rhoStartIdx)
   {
      int problemSize = 0;

      for (int i = 0; i < command.getNumberOfContacts(); i++)
      {
         problemSize += command.getContactPlaneHelper(i).getRhoSize();
      }

      if (problemSize < 1)
         return false;

      inputToPack.setNumberOfVariables(numberOfVariables);
      inputToPack.reshape(problemSize);
      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();

      double timeOfObjective = command.getTimeOfObjective();
      double omega = command.getOmega();

      int startCol = rhoStartIdx;
      int startRow = 0;
      for (int i = 0; i < command.getNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlaneHelper = command.getContactPlaneHelper(i);

         ContactPlaneJacobianCalculator.computeRhoJacobian(command.getDerivativeOrder(),
                                                           timeOfObjective,
                                                           omega,
                                                           startRow,
                                                           startCol,
                                                           contactPlaneHelper,
                                                           inputToPack.getTaskJacobian());

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

   /**
    * Directly calculates a quadratic cost function in the form of {@link QPInputTypeC} from a {@link VRPTrackingCommand}
    *
    * @param inputToPack QP cost function to compute
    * @param objective objective to process
    * @return whether or not that calculation was successful.
    */
   public int calculateVRPTrackingObjective(QPInputTypeC inputToPack, VRPTrackingCommand objective)
   {
      inputToPack.setNumberOfVariables(indexHandler.getTotalProblemSize());
      inputToPack.reshape();

      inputToPack.getDirectCostHessian().zero();
      inputToPack.getDirectCostGradient().zero();

      boolean success = vrpTrackingCostCalculator.calculateVRPTrackingObjective(inputToPack.getDirectCostHessian(),
                                                                                inputToPack.getDirectCostGradient(),
                                                                                objective);
      double weight = objective.getWeight();

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      if (success)
         return 0;
      else
         return -1;
   }

   public int calculateCompactVRPTrackingObjective(QPInputTypeC inputToPack, VRPTrackingCommand objective)
   {
      int segmentNumber = objective.getSegmentNumber();

      inputToPack.setNumberOfVariables(indexHandler.getRhoCoefficientsInSegment(segmentNumber) + LinearMPCIndexHandler.comCoefficientsPerSegment);
      inputToPack.reshape();

      inputToPack.getDirectCostHessian().zero();
      inputToPack.getDirectCostGradient().zero();

      boolean success = vrpTrackingCostCalculator.calculateCompactVRPTrackingObjective(inputToPack.getDirectCostHessian(),
                                                                                       inputToPack.getDirectCostGradient(),
                                                                                       objective);
      double weight = objective.getWeight();

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      if (success)
         return indexHandler.getComCoefficientStartIndex(segmentNumber);
      else
         return -1;
   }

   public int calculateRhoBoundCommandCompact(QPInputTypeA inputToPack, RhoBoundCommand command)
   {
      int numberOfVariables = 0;
      for (int i = 0; i < command.getNumberOfContacts(); i++)
      {
         numberOfVariables += command.getContactPlane(i).getCoefficientSize();
      }
      int segmentNumber = command.getSegmentNumber();

      if (calculateRhoBoundCommandInternal(inputToPack, command, numberOfVariables, 0))
         return indexHandler.getRhoCoefficientStartIndex(segmentNumber);

      return -1;
   }

   public boolean calculateRhoBoundCommandInternal(QPInputTypeA inputToPack, RhoBoundCommand command, int numberOfVariables, int rhoStartIdx)
   {
      int rhoSize = 0;
      for (int i = 0; i < command.getNumberOfContacts(); i++)
         rhoSize += command.getContactPlane(i).getRhoSize();

      if (rhoSize < 1)
         return false;

      inputToPack.setNumberOfVariables(numberOfVariables);
      inputToPack.reshape(LinearMPCIndexHandler.coefficientsPerRho * rhoSize);

      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();

      double duration = command.getSegmentDuration();
      double omega = command.getOmega();
      double omega2 = omega * omega;
      double exponential = Math.exp(omega * duration);

      double alpha = 1.0 * omega;

      int startCol = rhoStartIdx;
      int startRow = 0;
      for (int i = 0; i < command.getNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlane = command.getContactPlane(i);
         double rhoValue = command.getRhoValue(i);

         for (int rhoIdx = 0; rhoIdx < contactPlane.getRhoSize(); rhoIdx++)
         {
            int startCol1 = startCol + 1;
            int startCol2 = startCol + 2;
            int startCol3 = startCol + 3;

            inputToPack.getTaskJacobian().set(startRow, startCol, omega2);
            inputToPack.getTaskJacobian().set(startRow, startCol1, omega2);
            inputToPack.getTaskJacobian().set(startRow, startCol3, 2.0);
            inputToPack.getTaskObjective().set(startRow, 0, rhoValue);
            startRow++;

            inputToPack.getTaskJacobian().set(startRow, startCol, omega2 * (1.0 + omega / alpha));
            inputToPack.getTaskJacobian().set(startRow, startCol1, omega2 * (1.0 - omega / alpha));
            inputToPack.getTaskJacobian().set(startRow, startCol2, 6.0 / alpha);
            inputToPack.getTaskJacobian().set(startRow, startCol3, 2.0);
            inputToPack.getTaskObjective().set(startRow, 0, rhoValue);
            startRow++;

            inputToPack.getTaskJacobian().set(startRow, startCol, omega2 * exponential * (1.0 - omega / alpha));
            inputToPack.getTaskJacobian().set(startRow, startCol1, omega2 / exponential * (1.0 + omega / alpha));
            inputToPack.getTaskJacobian().set(startRow, startCol2, 6.0 * (duration - 1.0 / alpha));
            inputToPack.getTaskJacobian().set(startRow, startCol3, 2.0);
            inputToPack.getTaskObjective().set(startRow, 0, rhoValue);
            startRow++;

            inputToPack.getTaskJacobian().set(startRow, startCol, omega2 * exponential);
            inputToPack.getTaskJacobian().set(startRow, startCol1, omega2 / exponential);
            inputToPack.getTaskJacobian().set(startRow, startCol2, 6.0 * duration);
            inputToPack.getTaskJacobian().set(startRow, startCol3, 2.0);
            inputToPack.getTaskObjective().set(startRow, 0, rhoValue);
            startRow++;

            startCol += LinearMPCIndexHandler.coefficientsPerRho;
         }
      }

      inputToPack.setConstraintType(command.getConstraintType());

      return true;
   }

   public int calculateNormalForceBoundCommandCompact(QPInputTypeA inputToPack, NormalForceBoundCommand command)
   {
      int numberOfVariables = 0;
      for (int i = 0; i < command.getNumberOfContacts(); i++)
      {
         numberOfVariables += command.getContactPlane(i).getCoefficientSize();
      }
      int segmentNumber = command.getSegmentNumber();

      if (calculateNormalForceBoundCommandInternal(inputToPack, command, numberOfVariables, 0))
         return indexHandler.getRhoCoefficientStartIndex(segmentNumber);

      return -1;
   }

   public boolean calculateNormalForceBoundCommandInternal(QPInputTypeA inputToPack, NormalForceBoundCommand command, int numberOfVariables, int rhoStartIdx)
   {
      int rhoSize = 0;
      for (int i = 0; i < command.getNumberOfContacts(); i++)
         rhoSize += command.getContactPlane(i).getRhoSize();

      if (rhoSize < 1)
         return false;

      inputToPack.setNumberOfVariables(numberOfVariables);
      inputToPack.reshape(4 * rhoSize);

      inputToPack.getTaskJacobian().zero();
      inputToPack.getTaskObjective().zero();

      double duration = command.getSegmentDuration();
      double omega = command.getOmega();
      double omega2 = omega * omega;
      double exponential = Math.exp(omega * duration);

      int startCol = rhoStartIdx;
      int startRow = 0;

      for (int i = 0; i < command.getNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlane = command.getContactPlane(i);
         double normalForceLimit = command.getRhoValue(i);

         int row1 = startRow + 1;
         int row2 = startRow + 2;
         int row3 = startRow + 3;

         for (int pointIdx = 0; pointIdx < contactPlane.getNumberOfContactPoints(); pointIdx++)
         {
            MPCContactPoint point = contactPlane.getContactPointHelper(pointIdx);
            for (int rhoIdx = 0; rhoIdx < point.getRhoSize(); rhoIdx++)
            {
               double rhoZ = point.getRhoNormalZ();

               inputToPack.getTaskJacobian().set(startRow, startCol, rhoZ * omega2);
               inputToPack.getTaskJacobian().set(startRow, startCol + 1, rhoZ * omega2);
               inputToPack.getTaskJacobian().set(startRow, startCol + 3, rhoZ * 2.0);

               inputToPack.getTaskJacobian().set(row1, startCol, 2.0 * rhoZ * omega2);
               inputToPack.getTaskJacobian().set(row1, startCol + 2, 6.0 / rhoZ * omega);
               inputToPack.getTaskJacobian().set(row1, startCol + 3, rhoZ * 2.0);

               inputToPack.getTaskJacobian().set(row2, startCol + 1, rhoZ * omega2 / exponential * 2.0);
               inputToPack.getTaskJacobian().set(row2, startCol + 2, 6.0 * rhoZ * (duration - 1.0 / omega));
               inputToPack.getTaskJacobian().set(row2, startCol + 3, 2.0 * rhoZ);

               inputToPack.getTaskJacobian().set(row3, startCol, rhoZ * omega2 * exponential);
               inputToPack.getTaskJacobian().set(row3, startCol + 1, rhoZ * omega2 / exponential);
               inputToPack.getTaskJacobian().set(row3, startCol + 2, 6.0 * rhoZ * duration);
               inputToPack.getTaskJacobian().set(row3, startCol + 3, 2.0 * rhoZ);

               startCol += LinearMPCIndexHandler.coefficientsPerRho;
            }
         }

         inputToPack.getTaskObjective().set(startRow, 0, normalForceLimit);
         inputToPack.getTaskObjective().set(row1, 0, normalForceLimit);
         inputToPack.getTaskObjective().set(row2, 0, normalForceLimit);
         inputToPack.getTaskObjective().set(row3, 0, normalForceLimit);

         startRow += 4;
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
