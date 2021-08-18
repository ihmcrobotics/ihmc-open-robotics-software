package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.data.DMatrixSparseCSC;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPoint;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.NativeQPInputTypeA;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeC;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.NativeQPInputTypeC;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;

/**
 * This is a helper class that is meant to convert {@link MPCCommand}s to QP inputs that can be consumed by quadratic program solver.
 */
public class MPCQPInputCalculator
{
   public static final double sufficientlyLongTime = 5.0;
   public static final double sufficientlyLargeValue = 1e5;

   private final LinearMPCIndexHandler indexHandler;

   private final VRPTrackingCostCalculator vrpTrackingCostCalculator;

   private final DMatrixRMaj selectionMatrix = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj selectedJacobian = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj selectedObjective = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj tempJacobian = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj tempObjective = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj tempHessian = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj tempGradient = new DMatrixRMaj(0, 0);

   private final double gravityZ;

   public MPCQPInputCalculator(LinearMPCIndexHandler indexHandler, double gravityZ)
   {
      this.indexHandler = indexHandler;
      this.vrpTrackingCostCalculator = new VRPTrackingCostCalculator(indexHandler, gravityZ);
      this.gravityZ = -Math.abs(gravityZ);
   }

   /**
    * Computes a {@link NativeQPInputTypeA} from a {@link MPCContinuityCommand}. This can consist of a continuity command that is either an objective or equality
    * constraint.
    *
    * @param inputToPack QP Input that contains the encoded continuity command
    * @param objective continuity command to process
    * @return whether or not the calculation was successful.
    */
   public int calculateContinuityObjective(NativeQPInputTypeA inputToPack, MPCContinuityCommand objective)
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

   public int calculateCompactContinuityObjective(NativeQPInputTypeA inputToPack, MPCContinuityCommand objective)
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
    * Computes a {@link NativeQPInputTypeA} from a {@link MPCContinuityCommand} if {@link MPCContinuityCommand#getValueType()} indicates the center of mass. This can
    * consist of a continuity command that is either an objective or equality constraint.
    *
    * @param inputToPack QP Input that contains the encoded continuity command
    * @param objective continuity command to process
    * @return whether or not the calculation was successful.
    */
   public int calculateCoMContinuityObjective(NativeQPInputTypeA inputToPack, MPCContinuityCommand objective)
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
    * Computes a {@link NativeQPInputTypeA} from a {@link MPCContinuityCommand} if {@link MPCContinuityCommand#getValueType()} indicates the virtual repellent point.
    * This can consist of a continuity command that is either an objective or equality constraint.
    *
    * @param inputToPack QP Input that contains the encoded continuity command
    * @param objective continuity command to process
    * @return whether or not the calculation was successful.
    */
   public int calculateVRPContinuityObjective(NativeQPInputTypeA inputToPack, MPCContinuityCommand objective)
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

   public int calculateCompactCoMContinuityObjective(NativeQPInputTypeA inputToPack, MPCContinuityCommand objective)
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

   public int calculateCompactVRPContinuityObjective(NativeQPInputTypeA inputToPack, MPCContinuityCommand objective)
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

   public boolean calculateCoMContinuityObjectiveInternal(NativeQPInputTypeA inputToPack,
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

      tempJacobian.reshape(3, variableSize);
      tempObjective.reshape(3, 1);
      tempJacobian.zero();
      tempObjective.zero();

      double firstSegmentDuration = objective.getFirstSegmentDuration();
      double omega = objective.getOmega();
      double weight = objective.getWeight();

      CoMCoefficientJacobianCalculator.calculateCoMJacobian(firstComStartCol,
                                                            firstSegmentDuration,
                                                            tempJacobian,
                                                            objective.getDerivativeOrder(),
                                                            1.0);
      CoMCoefficientJacobianCalculator.calculateCoMJacobian(secondComStartCol, 0.0, tempJacobian, objective.getDerivativeOrder(), -1.0);

      for (int i = 0; i < objective.getFirstSegmentNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlaneHelper = objective.getFirstSegmentContactPlaneHelper(i);
         ContactPlaneJacobianCalculator.computeLinearJacobian(objective.getDerivativeOrder(),
                                                              firstSegmentDuration,
                                                              omega,
                                                              firstRhoStartCol,
                                                              contactPlaneHelper,
                                                              tempJacobian);

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
                                                              tempJacobian);

         secondRhoStartCol += contactPlaneHelper.getCoefficientSize();
      }

      tempObjective.set(2, 0, getGravityZObjective(objective.getDerivativeOrder(), 0.0));
      tempObjective.add(2, 0, -getGravityZObjective(objective.getDerivativeOrder(), firstSegmentDuration));

      inputToPack.getTaskJacobian().set(tempJacobian);
      inputToPack.getTaskObjective().set(tempObjective);

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   public boolean calculateVRPContinuityObjectiveInternal(NativeQPInputTypeA inputToPack,
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

      tempJacobian.reshape(3, variableSize);
      tempObjective.reshape(3, 1);
      tempJacobian.zero();
      tempObjective.zero();

      double firstSegmentDuration = objective.getFirstSegmentDuration();
      double omega = objective.getOmega();
      double omega2 = omega * omega;
      double weight = objective.getWeight();

      CoMCoefficientJacobianCalculator.calculateVRPJacobian(firstComStartCol,
                                                            omega,
                                                            firstSegmentDuration,
                                                            tempJacobian,
                                                            objective.getDerivativeOrder(),
                                                            1.0);
      CoMCoefficientJacobianCalculator.calculateVRPJacobian(secondComStartCol,
                                                            omega,
                                                            0.0,
                                                            tempJacobian,
                                                            objective.getDerivativeOrder(),
                                                            -1.0);

      for (int i = 0; i < objective.getFirstSegmentNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlaneHelper = objective.getFirstSegmentContactPlaneHelper(i);

         ContactPlaneJacobianCalculator.computeLinearJacobian(objective.getDerivativeOrder(),
                                                              firstSegmentDuration,
                                                              omega,
                                                              firstRhoStartCol,
                                                              contactPlaneHelper,
                                                              tempJacobian);
         ContactPlaneJacobianCalculator.computeLinearJacobian(-1.0 / omega2,
                                                              objective.getDerivativeOrder() + 2,
                                                              firstSegmentDuration,
                                                              omega,
                                                              firstRhoStartCol,
                                                              contactPlaneHelper,
                                                              tempJacobian);

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
                                                              tempJacobian);
         ContactPlaneJacobianCalculator.computeLinearJacobian(1.0 / omega2,
                                                              objective.getDerivativeOrder() + 2,
                                                              0.0,
                                                              omega,
                                                              secondRhoStartCol,
                                                              contactPlaneHelper,
                                                              tempJacobian);

         secondRhoStartCol += contactPlaneHelper.getCoefficientSize();
      }

      tempObjective.set(2, 0, getGravityZObjective(objective.getDerivativeOrder(), 0.0));
      tempObjective.add(2, 0, -getGravityZObjective(objective.getDerivativeOrder(), firstSegmentDuration));

      inputToPack.getTaskJacobian().set(tempJacobian);
      inputToPack.getTaskObjective().set(tempObjective);

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   public boolean calculateForceMinimizationObjective(NativeQPInputTypeC inputToPack, ForceObjectiveCommand objective)
   {
      throw new RuntimeException("This objective is not yet properly implemented.");
      /*
      inputToPack.setNumberOfVariables(indexHandler.getTotalProblemSize());
      inputToPack.reshape();

      tempHessian.reshape(indexHandler.getTotalProblemSize(), indexHandler.getTotalProblemSize());
      tempGradient.reshape(indexHandler.getTotalProblemSize(), 1);
      tempHessian.zero();
      tempGradient.zero();

      int segmentNumber = objective.getSegmentNumber();
      double weight = objective.getWeight();

      int startCol = indexHandler.getRhoCoefficientStartIndex(segmentNumber);
      for (int i = 0; i < objective.getNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlaneHelper = objective.getContactPlaneHelper(i);

         MatrixTools.addMatrixBlock(tempHessian,
                                    startCol,
                                    startCol,
                                    contactPlaneHelper.getAccelerationIntegrationHessian(),
                                    0,
                                    0,
                                    contactPlaneHelper.getCoefficientSize(),
                                    contactPlaneHelper.getCoefficientSize(),
                                    1.0);
         MatrixTools.addMatrixBlock(tempGradient,
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

      inputToPack.getDirectCostHessian().set(tempHessian);
      inputToPack.getDirectCostGradient().set(tempGradient);

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;

       */
   }

   public int calculateForceTrackingObjective(NativeQPInputTypeC inputToPack, ForceTrackingCommand objective)
   {
      int segmentNumber = objective.getSegmentNumber();

      int numberOfVariables = indexHandler.getRhoCoefficientsInSegment(segmentNumber);

      inputToPack.setNumberOfVariables(numberOfVariables);
      inputToPack.reshape();

      tempHessian.reshape(numberOfVariables, numberOfVariables);
      tempGradient.reshape(numberOfVariables, 1);
      tempHessian.zero();
      tempGradient.zero();

      double weight = objective.getWeight();
      double duration = objective.getSegmentDuration();

      int startCol = 0;
      for (int i = 0; i < objective.getNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlane = objective.getContactPlaneHelper(i);

         IntegrationInputCalculator.computeForceTrackingMatrix(startCol,
                                                               tempGradient,
                                                               tempHessian,
                                                               contactPlane,
                                                               duration,
                                                               objective.getOmega(),
                                                               objective.getObjectiveValue());

         startCol += contactPlane.getCoefficientSize();
      }

      inputToPack.getDirectCostHessian().set(tempHessian);
      inputToPack.getDirectCostGradient().set(tempGradient);

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return indexHandler.getRhoCoefficientStartIndex(segmentNumber);
   }

   public int calculateForceRateTrackingObjective(NativeQPInputTypeC inputToPack, ForceRateTrackingCommand objective)
   {
      int segmentNumber = objective.getSegmentNumber();

      int numberOfVariables = indexHandler.getRhoCoefficientsInSegment(segmentNumber);
      inputToPack.setNumberOfVariables(numberOfVariables);
      inputToPack.reshape();

      tempHessian.reshape(numberOfVariables, numberOfVariables);
      tempGradient.reshape(numberOfVariables, 1);
      tempHessian.zero();
      tempGradient.zero();

      double weight = objective.getWeight();
      double duration = objective.getSegmentDuration();

      int startCol = 0;
      for (int i = 0; i < objective.getNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlane = objective.getContactPlaneHelper(i);

         IntegrationInputCalculator.computeForceRateTrackingMatrix(startCol,
                                                               tempGradient,
                                                               tempHessian,
                                                               contactPlane,
                                                               duration,
                                                               objective.getOmega(),
                                                               objective.getObjectiveValue());

         startCol += contactPlane.getCoefficientSize();
      }

      inputToPack.getDirectCostHessian().set(tempHessian);
      inputToPack.getDirectCostGradient().set(tempGradient);

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return indexHandler.getRhoCoefficientStartIndex(segmentNumber);
   }

   public int calculateRhoTrackingObjective(NativeQPInputTypeC inputToPack, RhoTrackingCommand objective)
   {
      int segmentNumber = objective.getSegmentNumber();

      int numberOfVariables = indexHandler.getRhoCoefficientsInSegment(segmentNumber);
      inputToPack.setNumberOfVariables(numberOfVariables);
      inputToPack.reshape();

      tempHessian.reshape(numberOfVariables, numberOfVariables);
      tempGradient.reshape(numberOfVariables, 1);
      tempHessian.zero();
      tempGradient.zero();

      double weight = objective.getWeight();
      double duration = objective.getSegmentDuration();

      int startCol = 0;
      for (int i = 0; i < objective.getNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlane = objective.getContactPlaneHelper(i);

         IntegrationInputCalculator.computeRhoAccelerationTrackingMatrix(startCol,
                                                                         tempGradient,
                                                                         tempHessian,
                                                                         contactPlane.getRhoSize(),
                                                                         duration,
                                                                         objective.getOmega(),
                                                                         objective.getObjectiveValue());

         startCol += contactPlane.getCoefficientSize();
      }

      inputToPack.getDirectCostHessian().set(tempHessian);
      inputToPack.getDirectCostGradient().set(tempGradient);

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return indexHandler.getRhoCoefficientStartIndex(segmentNumber);
   }

   public int calculateRhoRateTrackingObjective(NativeQPInputTypeC inputToPack, RhoRateTrackingCommand objective)
   {
      int segmentNumber = objective.getSegmentNumber();

      int numberOfVariables = indexHandler.getRhoCoefficientsInSegment(segmentNumber);
      inputToPack.setNumberOfVariables(numberOfVariables);
      inputToPack.reshape();

      tempHessian.reshape(numberOfVariables, numberOfVariables);
      tempGradient.reshape(numberOfVariables, 1);
      tempHessian.zero();
      tempGradient.zero();

      double weight = objective.getWeight();
      double duration = objective.getSegmentDuration();

      int startCol = 0;
      for (int i = 0; i < objective.getNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlane = objective.getContactPlaneHelper(i);

         IntegrationInputCalculator.computeRhoJerkTrackingMatrix(startCol,
                                                                         tempGradient,
                                                                         tempHessian,
                                                                         contactPlane.getRhoSize(),
                                                                         duration,
                                                                         objective.getOmega(),
                                                                         objective.getObjectiveValue());

         startCol += contactPlane.getCoefficientSize();
      }

      inputToPack.getDirectCostHessian().set(tempHessian);
      inputToPack.getDirectCostGradient().set(tempGradient);

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return indexHandler.getRhoCoefficientStartIndex(segmentNumber);
   }

   /**
    * Processes a {@link MPCValueCommand} to compute a {@link NativeQPInputTypeA}. This can then be fed to MPC QP solver.
    * @param inputToPack QP input to calculate
    * @param objective value command to process
    * @return whether or not the calculation was successful. -1 if it's no successful
    */
   public int calculateValueObjective(NativeQPInputTypeA inputToPack, MPCValueCommand objective)
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

   public int calculateCompactValueObjective(NativeQPInputTypeA inputToPack, MPCValueCommand objective)
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

   private int calculateCoMValueObjective(NativeQPInputTypeA inputToPack, MPCValueCommand objective)
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

   private int calculateVRPValueObjective(NativeQPInputTypeA inputToPack, MPCValueCommand objective)
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

   private int calculateDCMValueObjective(NativeQPInputTypeA inputToPack, MPCValueCommand objective)
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

   private int calculateCompactCoMValueObjective(NativeQPInputTypeA inputToPack, MPCValueCommand objective)
   {
      int segmentNumber = objective.getSegmentNumber();
      int variableSize = LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(segmentNumber);
      int comStartCol = 0;
      int rhoStartCol = LinearMPCIndexHandler.comCoefficientsPerSegment;

      if (calculateCoMValueObjectiveInternal(inputToPack, objective, variableSize, comStartCol, rhoStartCol))
         return indexHandler.getComCoefficientStartIndex(segmentNumber);

      return -1;
   }

   private int calculateCompactVRPValueObjective(NativeQPInputTypeA inputToPack, MPCValueCommand objective)
   {
      int segmentNumber = objective.getSegmentNumber();
      int variableSize = LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(segmentNumber);
      int comStartCol = 0;
      int rhoStartCol = LinearMPCIndexHandler.comCoefficientsPerSegment;

      if (calculateVRPValueObjectiveInternal(inputToPack, objective, variableSize, comStartCol, rhoStartCol))
         return indexHandler.getComCoefficientStartIndex(segmentNumber);

      return -1;
   }

   private int calculateCompactDCMValueObjective(NativeQPInputTypeA inputToPack, MPCValueCommand objective)
   {
      int segmentNumber = objective.getSegmentNumber();
      int variableSize = LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(segmentNumber);
      int comStartCol = 0;
      int rhoStartCol = LinearMPCIndexHandler.comCoefficientsPerSegment;

      if (calculateDCMValueObjectiveInternal(inputToPack, objective, variableSize, comStartCol, rhoStartCol))
         return indexHandler.getComCoefficientStartIndex(segmentNumber);

      return -1;
   }

   private boolean calculateCoMValueObjectiveInternal(NativeQPInputTypeA inputToPack,
                                                      MPCValueCommand objective,
                                                      int numberOfVariables,
                                                      int comStartCol,
                                                      int rhoStartCol)
   {
      inputToPack.setNumberOfVariables(numberOfVariables);
      inputToPack.reshape(3);
      inputToPack.setConstraintType(objective.getConstraintType());

      tempJacobian.reshape(3, numberOfVariables);
      tempObjective.reshape(3, 1);
      tempJacobian.zero();

      double timeOfObjective = objective.getTimeOfObjective();
      double omega = objective.getOmega();
      double weight = objective.getWeight();


      CoMCoefficientJacobianCalculator.calculateCoMJacobian(comStartCol, timeOfObjective, tempJacobian, objective.getDerivativeOrder(), 1.0);

      for (int i = 0; i < objective.getNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlaneHelper = objective.getContactPlaneHelper(i);
         ContactPlaneJacobianCalculator.computeLinearJacobian(objective.getDerivativeOrder(),
                                                              timeOfObjective,
                                                              omega,
                                                              rhoStartCol,
                                                              contactPlaneHelper,
                                                              tempJacobian);

         rhoStartCol += contactPlaneHelper.getCoefficientSize();
      }

      objective.getObjective().get(tempObjective);
      tempObjective.add(2, 0, -getGravityZObjective(objective.getDerivativeOrder(), timeOfObjective));

      if (objective.getSelectionMatrix().getNumberOfSelectedAxes() != 3)
      {
         selectionMatrix.reshape(3, 3);
         objective.getSelectionMatrix().getCompactSelectionMatrixInFrame(ReferenceFrame.getWorldFrame(), 0, 0, selectionMatrix);

         selectedJacobian.reshape(selectionMatrix.getNumRows(), numberOfVariables);
         selectedObjective.reshape(selectionMatrix.getNumRows(), 1);

         CommonOps_DDRM.mult(selectionMatrix, tempJacobian, selectedJacobian);
         CommonOps_DDRM.mult(selectionMatrix, tempObjective, selectedObjective);

         inputToPack.getTaskJacobian().set(selectedJacobian);
         inputToPack.getTaskObjective().set(selectedObjective);
      }
      else
      {
         inputToPack.getTaskJacobian().set(tempJacobian);
         inputToPack.getTaskObjective().set(tempObjective);
      }

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   private boolean calculateDCMValueObjectiveInternal(NativeQPInputTypeA inputToPack,
                                                      MPCValueCommand objective,
                                                      int numberOfVariables,
                                                      int comStartCol,
                                                      int rhoStartCol)
   {
      inputToPack.setNumberOfVariables(numberOfVariables);
      inputToPack.reshape(3);
      inputToPack.setConstraintType(objective.getConstraintType());

      tempJacobian.reshape(3, numberOfVariables);
      tempObjective.reshape(3, 1);
      tempJacobian.zero();
      tempObjective.zero();

      double timeOfObjective = objective.getTimeOfObjective();
      double omega = objective.getOmega();
      double weight = objective.getWeight();

      int objectiveOrder = objective.getDerivativeOrder();
      int objectiveHigherOrder = objectiveOrder + 1;

      CoMCoefficientJacobianCalculator.calculateDCMJacobian(comStartCol, omega, timeOfObjective, tempJacobian, objectiveOrder, 1.0);

      for (int i = 0; i < objective.getNumberOfContacts(); i++)
      {
         MPCContactPlane contactPlaneHelper = objective.getContactPlaneHelper(i);

         ContactPlaneJacobianCalculator.computeLinearJacobian(objectiveOrder,
                                                              timeOfObjective,
                                                              omega,
                                                              rhoStartCol,
                                                              contactPlaneHelper,
                                                              tempJacobian);
         ContactPlaneJacobianCalculator.computeLinearJacobian(1.0 / omega,
                                                              objectiveHigherOrder,
                                                              timeOfObjective,
                                                              omega,
                                                              rhoStartCol,
                                                              contactPlaneHelper,
                                                              tempJacobian);

         rhoStartCol += contactPlaneHelper.getCoefficientSize();
      }

      objective.getObjective().get(tempObjective);
      tempObjective.add(2, 0, -getGravityZObjective(objectiveOrder, timeOfObjective));
      tempObjective.add(2, 0, -getGravityZObjective(objectiveHigherOrder, timeOfObjective) / omega);

      if (objective.getSelectionMatrix().getNumberOfSelectedAxes() != 3)
      {
         selectionMatrix.reshape(3, 3);
         objective.getSelectionMatrix().getCompactSelectionMatrixInFrame(ReferenceFrame.getWorldFrame(), 0, 0, selectionMatrix);

         selectedJacobian.reshape(selectionMatrix.getNumRows(), numberOfVariables);
         selectedObjective.reshape(selectionMatrix.getNumRows(), 1);

         CommonOps_DDRM.mult(selectionMatrix, tempJacobian, selectedJacobian);
         CommonOps_DDRM.mult(selectionMatrix, tempObjective, selectedObjective);

         inputToPack.getTaskJacobian().set(selectedJacobian);
         inputToPack.getTaskObjective().set(selectedObjective);
      }
      else
      {
         inputToPack.getTaskJacobian().set(tempJacobian);
         inputToPack.getTaskObjective().set(tempObjective);
      }

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   private boolean calculateVRPValueObjectiveInternal(NativeQPInputTypeA inputToPack,
                                                      MPCValueCommand objective,
                                                      int numberOfVariables,
                                                      int comStartIdx,
                                                      int rhoStartIdx)
   {
      inputToPack.setNumberOfVariables(numberOfVariables);
      inputToPack.reshape(3);
      inputToPack.setConstraintType(objective.getConstraintType());

      tempJacobian.reshape(3, numberOfVariables);
      tempObjective.reshape(3, 1);
      tempJacobian.zero();
      tempObjective.zero();

      double timeOfObjective = objective.getTimeOfObjective();
      double omega = objective.getOmega();
      double weight = objective.getWeight();
      double omega2 = omega * omega;

      CoMCoefficientJacobianCalculator.calculateVRPJacobian(comStartIdx,
                                                            omega,
                                                            timeOfObjective,
                                                            tempJacobian,
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
                                                              tempJacobian);
         ContactPlaneJacobianCalculator.computeLinearJacobian(-1.0 / omega2,
                                                              objective.getDerivativeOrder() + 2,
                                                              timeOfObjective,
                                                              omega,
                                                              rhoStartIdx,
                                                              contactPlaneHelper,
                                                              tempJacobian);

         rhoStartIdx += contactPlaneHelper.getCoefficientSize();
      }

      objective.getObjective().get(tempObjective);
      tempObjective.add(2, 0, -getGravityZObjective(objective.getDerivativeOrder(), timeOfObjective));
      tempObjective.add(2, 0, getGravityZObjective(objective.getDerivativeOrder() + 2, timeOfObjective) / omega2);

      if (objective.getSelectionMatrix().getNumberOfSelectedAxes() != 3)
      {
         objective.getSelectionMatrix().getCompactSelectionMatrixInFrame(ReferenceFrame.getWorldFrame(), 0, 0, selectionMatrix);

         selectedJacobian.reshape(selectionMatrix.getNumRows(), numberOfVariables);
         selectedObjective.reshape(selectionMatrix.getNumRows(), 1);

         CommonOps_DDRM.mult(selectionMatrix, tempJacobian, selectedJacobian);
         CommonOps_DDRM.mult(selectionMatrix, tempObjective, selectedObjective);

         inputToPack.getTaskJacobian().set(selectedJacobian);
         inputToPack.getTaskObjective().set(selectedObjective);
      }
      else
      {
         inputToPack.getTaskJacobian().set(tempJacobian);
         inputToPack.getTaskObjective().set(tempObjective);
      }

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);

      return true;
   }

   /**
    * Calculates a {@link NativeQPInputTypeA} based on a {@link RhoObjectiveCommand}. Is typically used to set upper and lower bounds for the generalized contact
    * values.
    *
    * @param inputToPack QP input to compute
    * @param command command to process
    * @return whether or not the calculation was successful.
    */
   public int calculateRhoValueCommand(NativeQPInputTypeA inputToPack, RhoObjectiveCommand command)
   {
      int numberOfVariables = indexHandler.getTotalProblemSize();
      int rhoStartIdx = indexHandler.getRhoCoefficientStartIndex(command.getSegmentNumber());

      if (calculateRhoValueCommandInternal(inputToPack, command, numberOfVariables, rhoStartIdx))
         return 0;

      return -1;
   }

   /**
    * Calculates a {@link NativeQPInputTypeA} based on a {@link RhoObjectiveCommand}. Is typically used to set upper and lower bounds for the generalized contact
    * values.
    *
    * @param inputToPack QP input to compute
    * @param command command to process
    * @return whether or not the calculation was successful.
    */
   public int calculateCompactRhoValueCommand(NativeQPInputTypeA inputToPack, RhoObjectiveCommand command)
   {
      int numberOfVariables = 0;
      for (int i = 0; i < command.getNumberOfContacts(); i++)
         numberOfVariables += command.getContactPlaneHelper(i).getCoefficientSize();

      int segmentNumber = command.getSegmentNumber();

      if (calculateRhoValueCommandInternal(inputToPack, command, numberOfVariables, 0))
         return indexHandler.getRhoCoefficientStartIndex(segmentNumber);

      return -1;
   }

   public boolean calculateRhoValueCommandInternal(NativeQPInputTypeA inputToPack, RhoObjectiveCommand command, int numberOfVariables, int rhoStartIdx)
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

      tempJacobian.reshape(problemSize, numberOfVariables);
      tempObjective.reshape(problemSize, 1);
      tempJacobian.zero();
      tempObjective.zero();

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
                                                           tempJacobian);

         startRow += contactPlaneHelper.getRhoSize();
         startCol += contactPlaneHelper.getCoefficientSize();
      }

      if (command.getUseScalarObjective())
      {
         for (int i = 0; i < problemSize; i++)
            tempObjective.set(i, 0, command.getScalarObjective());
      }
      else
      {
         tempObjective.set(command.getObjectiveVector());
      }

      inputToPack.getTaskJacobian().set(tempJacobian);
      inputToPack.getTaskObjective().set(tempObjective);
      inputToPack.setConstraintType(command.getConstraintType());

      return true;
   }

   /**
    * Directly calculates a quadratic cost function in the form of {@link NativeQPInputTypeC} from a {@link VRPTrackingCommand}
    * @param inputToPack QP cost function to compute
    * @param objective objective to process
    * @return whether or not that calculation was successful.
    */
   public int calculateVRPTrackingObjective(NativeQPInputTypeC inputToPack, VRPTrackingCommand objective)
   {
      inputToPack.setNumberOfVariables(indexHandler.getTotalProblemSize());
      inputToPack.reshape();

      tempHessian.reshape(indexHandler.getTotalProblemSize(), indexHandler.getTotalProblemSize());
      tempGradient.reshape(indexHandler.getTotalProblemSize(), 1);
      tempHessian.zero();
      tempGradient.zero();

      // FIXME likely unnecessary
      inputToPack.getDirectCostHessian().zero();
      inputToPack.getDirectCostGradient().zero();

      boolean success = vrpTrackingCostCalculator.calculateVRPTrackingObjective(tempHessian,
                                                                                tempGradient,
                                                                                objective);
      double weight = objective.getWeight();

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);
      inputToPack.getDirectCostHessian().set(tempHessian);
      inputToPack.getDirectCostGradient().set(tempGradient);

      if (success)
         return 0;
      else
         return -1;
   }

   public int calculateCompactVRPTrackingObjective(NativeQPInputTypeC inputToPack, VRPTrackingCommand objective)
   {
      int segmentNumber = objective.getSegmentNumber();

      int numberOfVariables = indexHandler.getRhoCoefficientsInSegment(segmentNumber) + LinearMPCIndexHandler.comCoefficientsPerSegment;
      inputToPack.setNumberOfVariables(numberOfVariables);
      inputToPack.reshape();

      tempHessian.reshape(numberOfVariables, numberOfVariables);
      tempGradient.reshape(numberOfVariables, 1);
      tempHessian.zero();
      tempGradient.zero();

      boolean success = vrpTrackingCostCalculator.calculateCompactVRPTrackingObjective(tempHessian,
                                                                                       tempGradient,
                                                                                       objective);
      double weight = objective.getWeight();

      inputToPack.setUseWeightScalar(true);
      inputToPack.setWeight(weight);
      inputToPack.getDirectCostHessian().set(tempHessian);
      inputToPack.getDirectCostGradient().set(tempGradient);

      if (success)
         return indexHandler.getComCoefficientStartIndex(segmentNumber);
      else
         return -1;
   }

   public int calculateRhoBoundCommandCompact(NativeQPInputTypeA inputToPack, RhoBoundCommand command)
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

   private boolean calculateRhoBoundCommandInternal(NativeQPInputTypeA inputToPack, RhoBoundCommand command, int numberOfVariables, int rhoStartIdx)
   {
      int rhoSize = 0;
      for (int i = 0; i < command.getNumberOfContacts(); i++)
         rhoSize += command.getContactPlane(i).getRhoSize();

      if (rhoSize < 1)
         return false;

      int problemSize = LinearMPCIndexHandler.coefficientsPerRho * rhoSize;

      inputToPack.setNumberOfVariables(numberOfVariables);
      inputToPack.reshape(problemSize);

      tempJacobian.reshape(problemSize, numberOfVariables);
      tempObjective.reshape(problemSize, 1);
      tempJacobian.zero();
      tempObjective.zero();

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

            tempJacobian.set(startRow, startCol, omega2);
            tempJacobian.set(startRow, startCol1, omega2);
            tempJacobian.set(startRow, startCol3, 2.0);
            tempObjective.set(startRow, 0, rhoValue);
            startRow++;

            tempJacobian.set(startRow, startCol, omega2 * (1.0 + omega / alpha));
            tempJacobian.set(startRow, startCol1, omega2 * (1.0 - omega / alpha));
            tempJacobian.set(startRow, startCol2, 6.0 / alpha);
            tempJacobian.set(startRow, startCol3, 2.0);
            tempObjective.set(startRow, 0, rhoValue);
            startRow++;

            tempJacobian.set(startRow, startCol, omega2 * exponential * (1.0 - omega / alpha));
            tempJacobian.set(startRow, startCol1, omega2 / exponential * (1.0 + omega / alpha));
            tempJacobian.set(startRow, startCol2, 6.0 * (duration - 1.0 / alpha));
            tempJacobian.set(startRow, startCol3, 2.0);
            tempObjective.set(startRow, 0, rhoValue);
            startRow++;

            tempJacobian.set(startRow, startCol, omega2 * exponential);
            tempJacobian.set(startRow, startCol1, omega2 / exponential);
            tempJacobian.set(startRow, startCol2, 6.0 * duration);
            tempJacobian.set(startRow, startCol3, 2.0);
            tempObjective.set(startRow, 0, rhoValue);
            startRow++;

            startCol += LinearMPCIndexHandler.coefficientsPerRho;
         }
      }

      inputToPack.getTaskJacobian().set(tempJacobian);
      inputToPack.getTaskObjective().set(tempObjective);

      inputToPack.setConstraintType(command.getConstraintType());

      return true;
   }

   public int calculateNormalForceBoundCommandCompact(NativeQPInputTypeA inputToPack, NormalForceBoundCommand command)
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

   public boolean calculateNormalForceBoundCommandInternal(NativeQPInputTypeA inputToPack, NormalForceBoundCommand command, int numberOfVariables, int rhoStartIdx)
   {
      int rhoSize = 0;
      for (int i = 0; i < command.getNumberOfContacts(); i++)
         rhoSize += command.getContactPlane(i).getRhoSize();

      if (rhoSize < 1)
         return false;

      inputToPack.setNumberOfVariables(numberOfVariables);
      inputToPack.reshape(4 * rhoSize);

      tempJacobian.reshape(4 * rhoSize, numberOfVariables);
      tempObjective.reshape(4 * rhoSize, 1);
      tempJacobian.zero();
      tempObjective.zero();

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

               tempJacobian.set(startRow, startCol, rhoZ * omega2);
               tempJacobian.set(startRow, startCol + 1, rhoZ * omega2);
               tempJacobian.set(startRow, startCol + 3, rhoZ * 2.0);

               tempJacobian.set(row1, startCol, 2.0 * rhoZ * omega2);
               tempJacobian.set(row1, startCol + 2, 6.0 / rhoZ * omega);
               tempJacobian.set(row1, startCol + 3, rhoZ * 2.0);

               tempJacobian.set(row2, startCol + 1, rhoZ * omega2 / exponential * 2.0);
               tempJacobian.set(row2, startCol + 2, 6.0 * rhoZ * (duration - 1.0 / omega));
               tempJacobian.set(row2, startCol + 3, 2.0 * rhoZ);

               tempJacobian.set(row3, startCol, rhoZ * omega2 * exponential);
               tempJacobian.set(row3, startCol + 1, rhoZ * omega2 / exponential);
               tempJacobian.set(row3, startCol + 2, 6.0 * rhoZ * duration);
               tempJacobian.set(row3, startCol + 3, 2.0 * rhoZ);

               startCol += LinearMPCIndexHandler.coefficientsPerRho;
            }
         }

         tempObjective.set(startRow, 0, normalForceLimit);
         tempObjective.set(row1, 0, normalForceLimit);
         tempObjective.set(row2, 0, normalForceLimit);
         tempObjective.set(row3, 0, normalForceLimit);

         startRow += 4;
      }

      inputToPack.getTaskJacobian().set(tempJacobian);
      inputToPack.getTaskObjective().set(tempObjective);

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
