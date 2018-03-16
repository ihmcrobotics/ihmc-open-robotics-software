package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.RecycledLinkedListBuilder.RecycledLinkedListEntry;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.Axis;

public class OptimizationControlModuleHelper
{
   static final int forceCoefficients = 4;
   static final int velocityCoefficients = forceCoefficients + 1;
   static final int positionCoefficients = velocityCoefficients + 1;
   static final int defaultNumberOfNodes = 100;
   static final int numberOfAxis = Axis.values.length;

   private final DenseMatrix64F deltaT = new DenseMatrix64F(defaultNumberOfNodes - 1, 1);
   private final DenseMatrix64F[] positionCoefficientMatrix = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] positionBias = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] velocityCoefficientMatrix = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] velocityBias = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] forceCoefficientMatrix = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] forceBias = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] forceRateCoefficientMatrix = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] forceRateBias = new DenseMatrix64F[numberOfAxis];

   private final DenseMatrix64F[] decisionVariableWeightMatrix = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] decisionVariableDesiredValueMatrix = new DenseMatrix64F[numberOfAxis];
   private final double forceRegularizationWeight;
   private final double forceRateRegularizationWeight;

   private int numberOfNodes;
   private final int[] numberOfDecisionVariables = new int[numberOfAxis];
   private final double[] gravity = new double[numberOfAxis];
   private final double robotMass;
   private RecycledLinkedListBuilder<CentroidalMotionNode> nodeList;

   public OptimizationControlModuleHelper(CentroidalMotionPlannerParameters parameters)
   {
      this.robotMass = parameters.getRobotMass();
      for (int i = 0; i < numberOfAxis; i++)
      {
         positionCoefficientMatrix[i] = new DenseMatrix64F(defaultNumberOfNodes, defaultNumberOfNodes * 2);
         positionBias[i] = new DenseMatrix64F(defaultNumberOfNodes, 1);
         velocityCoefficientMatrix[i] = new DenseMatrix64F(defaultNumberOfNodes, defaultNumberOfNodes * 2);
         velocityBias[i] = new DenseMatrix64F(defaultNumberOfNodes, 1);
         forceCoefficientMatrix[i] = new DenseMatrix64F(defaultNumberOfNodes, defaultNumberOfNodes * 2);
         forceBias[i] = new DenseMatrix64F(defaultNumberOfNodes, 1);
         forceRateCoefficientMatrix[i] = new DenseMatrix64F(defaultNumberOfNodes, defaultNumberOfNodes * 2);
         forceRateBias[i] = new DenseMatrix64F(defaultNumberOfNodes, 1);
         decisionVariableWeightMatrix[i] = new DenseMatrix64F(defaultNumberOfNodes * 2, defaultNumberOfNodes * 2);
         decisionVariableDesiredValueMatrix[i] = new DenseMatrix64F(defaultNumberOfNodes * 2, 1);
      }
      this.forceRegularizationWeight = parameters.getdForceRegularizationWeight();
      this.forceRateRegularizationWeight = parameters.getdForceRegularizationWeight();
      gravity[0] = parameters.getGravityX();
      gravity[1] = parameters.getGravityY();
      gravity[2] = parameters.getGravityZ();
      reset();
   }

   public void reset()
   {
      deltaT.reshape(0, 1);
      numberOfNodes = 0;
      resetDecisionVariables();
      shapeCoefficientMatrices();
   }

   private void resetDecisionVariables()
   {
      for (Axis axis : Axis.values)
         numberOfDecisionVariables[axis.ordinal()] = 0;
   }

   private void shapeCoefficientMatrices()
   {
      for (int i = 0; i < numberOfAxis; i++)
      {
         positionCoefficientMatrix[i].reshape(numberOfNodes, numberOfDecisionVariables[i]);
         positionBias[i].reshape(numberOfNodes, 1);
         velocityCoefficientMatrix[i].reshape(numberOfNodes, numberOfDecisionVariables[i]);
         velocityBias[i].reshape(numberOfNodes, 1);
         forceCoefficientMatrix[i].reshape(numberOfNodes, numberOfDecisionVariables[i]);
         forceBias[i].reshape(numberOfNodes, 1);
         forceRateCoefficientMatrix[i].reshape(numberOfNodes, numberOfDecisionVariables[i]);
         forceRateBias[i].reshape(numberOfNodes, 1);
         decisionVariableWeightMatrix[i].reshape(numberOfDecisionVariables[i], numberOfDecisionVariables[i]);
         decisionVariableDesiredValueMatrix[i].reshape(numberOfDecisionVariables[i], 1);
      }
   }

   private void setCoefficientsToZero()
   {
      for (int i = 0; i < numberOfAxis; i++)
      {
         positionCoefficientMatrix[i].zero();
         positionBias[i].zero();
         velocityCoefficientMatrix[i].zero();
         velocityBias[i].zero();
         forceCoefficientMatrix[i].zero();
         forceBias[i].zero();
         forceRateCoefficientMatrix[i].zero();
         forceRateBias[i].zero();
      }
   }

   private final int[] decisionVariableIndex = new int[numberOfAxis];

   public void processNodeList(RecycledLinkedListBuilder<CentroidalMotionNode> nodeList)
   {
      numberOfNodes = nodeList.getSize();
      if (numberOfNodes < 2)
         throw new RuntimeException("Cannot run motion planning without atleast two nodes");
      determineNumberOfCoefficientVariables(nodeList);
      getForceVelocityPositionCoefficientBiasMatrices(nodeList);
   }

   private void determineNumberOfCoefficientVariables(RecycledLinkedListBuilder<CentroidalMotionNode> nodeList)
   {
      deltaT.reshape(numberOfNodes - 1, 1);
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry = nodeList.getFirstEntry();
      for (Axis axis : Axis.values)
      {
         if (entry.element.getForceConstraintType(axis) == EffortVariableConstraintType.OBJECTIVE)
            numberOfDecisionVariables[axis.ordinal()] = 1;
         else
            numberOfDecisionVariables[axis.ordinal()] = 0;
         if (entry.element.getForceRateConstraintType(axis) == EffortVariableConstraintType.OBJECTIVE)
            numberOfDecisionVariables[axis.ordinal()] += 1;
         else
            numberOfDecisionVariables[axis.ordinal()] = 0;

      }

      for (int i = 0; entry.getNext() != null; i++, entry = entry.getNext())
      {
         CentroidalMotionNode nextElement = entry.getNext().element;
         CentroidalMotionNode currentElement = entry.element;
         deltaT.set(i, 0, nextElement.getTime() - currentElement.getTime());
         for (Axis axis : Axis.values)
         {
            if (nextElement.getForceConstraintType(axis) == EffortVariableConstraintType.OBJECTIVE)
               numberOfDecisionVariables[axis.ordinal()] += 1;
            if (nextElement.getForceRateConstraintType(axis) == EffortVariableConstraintType.OBJECTIVE)
               numberOfDecisionVariables[axis.ordinal()] += 1;
         }
      }
   }

   private void getForceVelocityPositionCoefficientBiasMatrices(RecycledLinkedListBuilder<CentroidalMotionNode> nodeList)
   {
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry;
      // Determine the position and velocity coefficient matrices given the decision variables
      shapeCoefficientMatrices();
      setCoefficientsToZero();
      entry = nodeList.getFirstEntry();
      for (int i = 0; i < numberOfAxis; i++)
      {
         double p0 = entry.element.getPosition(Axis.values[i]);
         double v0 = entry.element.getLinearVelocity(Axis.values[i]);
         if (!Double.isFinite(p0))
            throw new RuntimeException(getClass().getSimpleName() + ": Initial node must have specified position for axis " + Axis.values[i].toString());
         positionBias[i].set(0, 0, p0);
         if (!Double.isFinite(v0))
            throw new RuntimeException(getClass().getSimpleName() + ": Initial node must have specified velocity for axis " + Axis.values[i].toString());
         velocityBias[i].set(0, 0, v0);
      }

      entry = nodeList.getFirstEntry();
      CentroidalMotionNode node = entry.element;
      int rowIndex = 0;
      for (Axis axis : Axis.values)
      {
         int axisOrdinal = axis.ordinal();
         decisionVariableIndex[axisOrdinal] = 0;
         DenseMatrix64F axisPositionCoefficientMatrix = positionCoefficientMatrix[axisOrdinal];
         DenseMatrix64F axisPositionBiasMatrix = positionBias[axisOrdinal];
         DenseMatrix64F axisVelocityCoefficientMatrix = velocityCoefficientMatrix[axisOrdinal];
         DenseMatrix64F axisVelocityBiasMatrix = velocityBias[axisOrdinal];
         DenseMatrix64F axisForceCoefficientMatrix = forceCoefficientMatrix[axisOrdinal];
         DenseMatrix64F axisForceBias = forceBias[axisOrdinal];
         DenseMatrix64F axisForceRateCoefficientMatrix = forceRateCoefficientMatrix[axisOrdinal];
         DenseMatrix64F axisForceRateBias = forceRateBias[axisOrdinal];
         DenseMatrix64F axisDecisionVariableWeightMatrix = decisionVariableWeightMatrix[axisOrdinal];
         DenseMatrix64F axisDecisionVariableDesiredValueMatrix = decisionVariableDesiredValueMatrix[axisOrdinal];
         double deltaT0 = deltaT.get(rowIndex, 0);

         double forceValue = node.getForce(axis);
         EffortVariableConstraintType forceConstraintType = node.getForceConstraintType(axis);
         double forceWeight = node.getForceWeight(axis);
         double velocityCoefficientForInitialForce = getVelocityCoefficientForInitialForce(deltaT0);
         double positionCoefficientForInitialForce = getPositionCoefficientForInitialForce(deltaT0);

         setInitialMatrixCoefficientsFromNodeQuantity(rowIndex, axisOrdinal, axisPositionCoefficientMatrix, axisPositionBiasMatrix,
                                                      axisVelocityCoefficientMatrix, axisVelocityBiasMatrix, axisForceCoefficientMatrix, axisForceBias,
                                                      axisDecisionVariableWeightMatrix, axisDecisionVariableDesiredValueMatrix, forceValue, forceConstraintType,
                                                      forceWeight, forceRegularizationWeight, velocityCoefficientForInitialForce,
                                                      positionCoefficientForInitialForce);
         double forceRateValue = node.getForceRate(axis);
         EffortVariableConstraintType forceRateConstraintType = node.getForceRateConstraintType(axis);
         double forceRateWeight = node.getForceRateWeight(axis);
         double velocityCoefficientForInitialForceRate = getVelocityCoefficientForInitialForceRate(deltaT0);
         double positionCoefficientForInitialForceRate = getPositionCoefficientForInitialForceRate(deltaT0);

         setInitialMatrixCoefficientsFromNodeQuantity(rowIndex, axisOrdinal, axisPositionCoefficientMatrix, axisPositionBiasMatrix,
                                                      axisVelocityCoefficientMatrix, axisVelocityBiasMatrix, axisForceRateCoefficientMatrix, axisForceRateBias,
                                                      axisDecisionVariableWeightMatrix, axisDecisionVariableDesiredValueMatrix, forceRateValue,
                                                      forceRateConstraintType, forceRateWeight, forceRateRegularizationWeight,
                                                      velocityCoefficientForInitialForceRate, positionCoefficientForInitialForceRate);
      }

      rowIndex++;
      for (entry = entry.getNext(); entry.getNext() != null; entry = entry.getNext(), rowIndex++)
      {
         node = entry.element;
         for (Axis axis : Axis.values)
         {
            int axisOrdinal = axis.ordinal();
            DenseMatrix64F axisPositionCoefficientMatrix = positionCoefficientMatrix[axisOrdinal];
            DenseMatrix64F axisPositionBiasMatrix = positionBias[axisOrdinal];
            DenseMatrix64F axisVelocityCoefficientMatrix = velocityCoefficientMatrix[axisOrdinal];
            DenseMatrix64F axisVelocityBiasMatrix = velocityBias[axisOrdinal];
            DenseMatrix64F axisForceCoefficientMatrix = forceCoefficientMatrix[axisOrdinal];
            DenseMatrix64F axisForceBias = forceBias[axisOrdinal];
            DenseMatrix64F axisForceRateCoefficientMatrix = forceRateCoefficientMatrix[axisOrdinal];
            DenseMatrix64F axisForceRateBias = forceRateBias[axisOrdinal];
            DenseMatrix64F axisdecisionVariableWeightMatrix = decisionVariableWeightMatrix[axisOrdinal];
            DenseMatrix64F axisdecisionVariableDesiredValueMatrix = decisionVariableDesiredValueMatrix[axisOrdinal];
            double deltaTim1 = deltaT.get(rowIndex - 1, 0);
            double deltaTi = deltaT.get(rowIndex, 0);

            double forceValue = node.getForce(axis);
            EffortVariableConstraintType forceConstraintType = node.getForceConstraintType(axis);
            double forceWeight = node.getForceWeight(axis);
            double velocityCoefficientForFinalForce = getVelocityCoefficientForFinalForce(deltaTim1);
            double velocityCoefficientForInitialForce = getVelocityCoefficientForInitialForce(deltaTi);
            double positionCoefficientForFinalForce = getPositionCoefficientForFinalForce(deltaTim1);
            double positionCoefficientForInitialForce = getPositionCoefficientForInitialForce(deltaTi);

            setInitialFinalMatrixCoefficientsFromNodeQuanitity(rowIndex, axisOrdinal, axisPositionCoefficientMatrix, axisPositionBiasMatrix,
                                                               axisVelocityCoefficientMatrix, axisVelocityBiasMatrix, axisForceCoefficientMatrix, axisForceBias,
                                                               axisdecisionVariableWeightMatrix, axisdecisionVariableDesiredValueMatrix, forceValue,
                                                               forceConstraintType, forceWeight, forceRegularizationWeight, velocityCoefficientForFinalForce,
                                                               velocityCoefficientForInitialForce, positionCoefficientForFinalForce,
                                                               positionCoefficientForInitialForce);

            double dForceValue = node.getForceRate(axis);
            EffortVariableConstraintType dForceConstraintType = node.getForceRateConstraintType(axis);
            double dForceWeight = node.getForceRateWeight(axis);
            double velocityCoefficientForFinalForceRate = getVelocityCoefficientForFinalForceRate(deltaTim1);
            double velocityCoefficientForInitialForceRate = getVelocityCoefficientForInitialForceRate(deltaTi);
            double positionCoefficientForFinalForceRate = getPositionCoefficientForFinalForceRate(deltaTim1);
            double positionCoefficientForInitialForceRate = getPositionCoefficientForInitialForceRate(deltaTi);

            setInitialFinalMatrixCoefficientsFromNodeQuanitity(rowIndex, axisOrdinal, axisPositionCoefficientMatrix, axisPositionBiasMatrix,
                                                               axisVelocityCoefficientMatrix, axisVelocityBiasMatrix, axisForceRateCoefficientMatrix,
                                                               axisForceRateBias, axisdecisionVariableWeightMatrix, axisdecisionVariableDesiredValueMatrix,
                                                               dForceValue, dForceConstraintType, dForceWeight, forceRateRegularizationWeight,
                                                               velocityCoefficientForFinalForceRate, velocityCoefficientForInitialForceRate,
                                                               positionCoefficientForFinalForceRate, positionCoefficientForInitialForceRate);
            replaceInitialConditionPlaceholder(rowIndex, axisVelocityCoefficientMatrix);
            replaceInitialConditionPlaceholderAndAddConstantBias(rowIndex, axisVelocityBiasMatrix, deltaTim1 * gravity[axisOrdinal]);
            replaceInitialConditionPlaceholder(rowIndex, axisPositionCoefficientMatrix);
            replaceInitialConditionPlaceholderAndAddConstantBias(rowIndex, axisPositionBiasMatrix, 0.5 * deltaTim1 * deltaTim1 * gravity[axisOrdinal]);
            addVelocityContributionToPosition(rowIndex, axisPositionCoefficientMatrix, axisVelocityCoefficientMatrix, deltaTim1);
            addVelocityContributionToPosition(rowIndex, axisPositionBiasMatrix, axisVelocityBiasMatrix, deltaTim1);
         }
      }

      node = entry.element;
      for (Axis axis : Axis.values)
      {
         double deltaTF = deltaT.get(rowIndex - 1, 0);
         int axisOrdinal = axis.ordinal();
         DenseMatrix64F axisVelocityCoefficientMatrix = velocityCoefficientMatrix[axisOrdinal];
         DenseMatrix64F axisVelocityBiasMatrix = velocityBias[axisOrdinal];
         DenseMatrix64F axisPositionCoefficientMatrix = positionCoefficientMatrix[axisOrdinal];
         DenseMatrix64F axisPositionBiasMatrix = positionBias[axisOrdinal];
         DenseMatrix64F axisForceCoefficientMatrix = forceCoefficientMatrix[axisOrdinal];
         DenseMatrix64F axisForceBias = forceBias[axisOrdinal];
         DenseMatrix64F axisForceRateCoefficientMatrix = forceRateCoefficientMatrix[axisOrdinal];
         DenseMatrix64F axisForceRateBias = forceRateBias[axisOrdinal];
         DenseMatrix64F axisDecisionVariableWeightMatrix = decisionVariableWeightMatrix[axisOrdinal];
         DenseMatrix64F axisDecisionVariableDesiredValueMatrix = decisionVariableDesiredValueMatrix[axisOrdinal];

         double forceValue = node.getForce(axis);
         EffortVariableConstraintType forceConstraintType = node.getForceConstraintType(axis);
         double forceWeight = node.getForceWeight(axis);
         double velocityCoefficientForFinalForce = getVelocityCoefficientForFinalForce(deltaTF);
         double positionCoefficientForFinalForce = getPositionCoefficientForFinalForce(deltaTF);
         setFinalMatrixCoefficientsFromNodeQuantity(rowIndex, axisOrdinal, axisVelocityCoefficientMatrix, axisVelocityBiasMatrix, axisPositionCoefficientMatrix,
                                                    axisPositionBiasMatrix, axisForceCoefficientMatrix, axisForceBias, axisDecisionVariableWeightMatrix,
                                                    axisDecisionVariableDesiredValueMatrix, forceValue, forceConstraintType, forceWeight,
                                                    forceRegularizationWeight, velocityCoefficientForFinalForce, positionCoefficientForFinalForce);

         double dForceValue = node.getForceRate(axis);
         EffortVariableConstraintType forceRateConstraintType = node.getForceRateConstraintType(axis);
         double dForceWeight = node.getForceRateWeight(axis);
         double velocityCoefficientForFinalForceRate = getVelocityCoefficientForFinalForceRate(deltaTF);
         double positionCoefficientForFinalForceRate = getPositionCoefficientForFinalForceRate(deltaTF);
         setFinalMatrixCoefficientsFromNodeQuantity(rowIndex, axisOrdinal, axisVelocityCoefficientMatrix, axisVelocityBiasMatrix, axisPositionCoefficientMatrix,
                                                    axisPositionBiasMatrix, axisForceRateCoefficientMatrix, axisForceRateBias, axisDecisionVariableWeightMatrix,
                                                    axisDecisionVariableDesiredValueMatrix, dForceValue, forceRateConstraintType, dForceWeight,
                                                    forceRateRegularizationWeight, velocityCoefficientForFinalForceRate, positionCoefficientForFinalForceRate);
         replaceInitialConditionPlaceholder(rowIndex, axisVelocityCoefficientMatrix);
         replaceInitialConditionPlaceholderAndAddConstantBias(rowIndex, axisVelocityBiasMatrix, deltaTF * gravity[axisOrdinal]);
         replaceInitialConditionPlaceholder(rowIndex, axisPositionCoefficientMatrix);
         replaceInitialConditionPlaceholderAndAddConstantBias(rowIndex, axisPositionBiasMatrix, 0.5 * deltaTF * deltaTF * gravity[axisOrdinal]);
         addVelocityContributionToPosition(rowIndex, axisPositionCoefficientMatrix, axisVelocityCoefficientMatrix, deltaTF);
         addVelocityContributionToPosition(rowIndex, axisPositionBiasMatrix, axisVelocityBiasMatrix, deltaTF);
      }
   }

   private void setFinalMatrixCoefficientsFromNodeQuantity(int rowIndex, int axisOrdinal, DenseMatrix64F axisVelocityCoefficientMatrix,
                                                           DenseMatrix64F axisVelocityBiasMatrix, DenseMatrix64F axisPositionCoefficientMatrix,
                                                           DenseMatrix64F axisPositionBiasMatrix, DenseMatrix64F axisDecisionVariableCoefficientMatrix,
                                                           DenseMatrix64F axisDecisionVariableBias, DenseMatrix64F axisDecisionVariableWeightMatrix,
                                                           DenseMatrix64F axisDecisionVariableDesiredValueMatrix, double decisionVariableValue,
                                                           EffortVariableConstraintType decisionVariableConstraintType, double decisionVariableWeight,
                                                           double decisionVariableRegularizationWeight, double velocityCoefficient, double positionCoefficient)
   {
      {
         double axisVelocityBias = axisVelocityBiasMatrix.get(rowIndex, 0);
         double axisPositionBias = axisPositionBiasMatrix.get(rowIndex, 0);
         switch (decisionVariableConstraintType)
         {
         case OBJECTIVE:
            processDecisionVariableWeighingMatrix(decisionVariableIndex[axisOrdinal], axisDecisionVariableWeightMatrix, axisDecisionVariableDesiredValueMatrix,
                                                  decisionVariableValue, decisionVariableWeight, decisionVariableRegularizationWeight);
            processCoefficientMatricesForFinalObjective(rowIndex, decisionVariableIndex[axisOrdinal], axisPositionCoefficientMatrix,
                                                        axisVelocityCoefficientMatrix, velocityCoefficient, positionCoefficient);
            axisDecisionVariableCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], 1.0);
            decisionVariableIndex[axisOrdinal]++;
            break;
         case CONSTRAINT:
            axisDecisionVariableBias.set(rowIndex, 0, decisionVariableValue);
            axisVelocityBias += decisionVariableValue * velocityCoefficient;
            axisPositionBias += decisionVariableValue * positionCoefficient;
            break;
         default:
            throw new RuntimeException("Unhandled constraint type");
         }
         axisVelocityBiasMatrix.set(rowIndex, 0, axisVelocityBias);
         axisPositionBiasMatrix.set(rowIndex, 0, axisPositionBias);
      }
   }

   private void setInitialMatrixCoefficientsFromNodeQuantity(int rowIndex, int axisOrdinal, DenseMatrix64F axisPositionCoefficientMatrix,
                                                             DenseMatrix64F axisPositionBiasMatrix, DenseMatrix64F axisVelocityCoefficientMatrix,
                                                             DenseMatrix64F axisVelocityBiasMatrix, DenseMatrix64F axisDecisionVariableCoefficientMatrix,
                                                             DenseMatrix64F axisDecisionVariableBias, DenseMatrix64F axisDecisionVariableWeightMatrix,
                                                             DenseMatrix64F axisDecisionVariableDesiredValueMatrix, double decisionVariableValue,
                                                             EffortVariableConstraintType decisionVariableConstraintType, double decisionVariableWeight,
                                                             double decisionVariableRegularizationWeight, double velocityCoefficient,
                                                             double positionCoefficient)
   {
      {
         double axisVelocityBias = axisVelocityBiasMatrix.get(rowIndex + 1, 0);
         double axisPositionBias = axisPositionBiasMatrix.get(rowIndex + 1, 0);
         switch (decisionVariableConstraintType)
         {
         case OBJECTIVE:
            processDecisionVariableWeighingMatrix(decisionVariableIndex[axisOrdinal], axisDecisionVariableWeightMatrix, axisDecisionVariableDesiredValueMatrix,
                                                  decisionVariableValue, decisionVariableWeight, decisionVariableRegularizationWeight);
            processCoefficientMatricesForInitialObjective(rowIndex, decisionVariableIndex[axisOrdinal], axisPositionCoefficientMatrix,
                                                          axisVelocityCoefficientMatrix, axisDecisionVariableCoefficientMatrix, velocityCoefficient,
                                                          positionCoefficient);
            decisionVariableIndex[axisOrdinal]++;
            break;
         case CONSTRAINT:
            axisDecisionVariableBias.set(rowIndex, 0, decisionVariableValue);
            axisVelocityBias += decisionVariableValue * velocityCoefficient;
            axisPositionBias += decisionVariableValue * positionCoefficient;
            break;
         default:
            throw new RuntimeException("Unhandled constraint type");
         }
         axisVelocityBiasMatrix.set(rowIndex + 1, 0, axisVelocityBias);
         axisPositionBiasMatrix.set(rowIndex + 1, 0, axisPositionBias);
      }
   }

   private void setInitialFinalMatrixCoefficientsFromNodeQuanitity(int rowIndex, int axisOrdinal, DenseMatrix64F axisPositionCoefficientMatrix,
                                                                   DenseMatrix64F axisPositionBiasMatrix, DenseMatrix64F axisVelocityCoefficientMatrix,
                                                                   DenseMatrix64F axisVelocityBiasMatrix, DenseMatrix64F axisDecisionVariableCoefficientMatrix,
                                                                   DenseMatrix64F axisDecisionVariableBias, DenseMatrix64F axisDecisionVariableWeightMatrix,
                                                                   DenseMatrix64F axisDecisionVariableDesiredValueMatrix, double decisionVariableValue,
                                                                   EffortVariableConstraintType decisionVariableConstraintType, double decisionVariableWeight,
                                                                   double decisionVariableRegularizationWeight, double velocityCoefficientFinal,
                                                                   double velocityCoefficientInitial, double positionCoefficientFinal,
                                                                   double positionCoefficientInitial)
   {
      double axisVelocityBias1 = axisVelocityBiasMatrix.get(rowIndex, 0);
      double axisVelocityBias2 = axisVelocityBiasMatrix.get(rowIndex + 1, 0);
      double axisPositionBias1 = axisPositionBiasMatrix.get(rowIndex, 0);
      double axisPositionBias2 = axisPositionBiasMatrix.get(rowIndex + 1, 0);

      switch (decisionVariableConstraintType)
      {
      case OBJECTIVE:
         processDecisionVariableWeighingMatrix(decisionVariableIndex[axisOrdinal], axisDecisionVariableWeightMatrix, axisDecisionVariableDesiredValueMatrix,
                                               decisionVariableValue, decisionVariableWeight, decisionVariableRegularizationWeight);
         processCoefficientMatricesForFinalObjective(rowIndex, decisionVariableIndex[axisOrdinal], axisPositionCoefficientMatrix, axisVelocityCoefficientMatrix,
                                                     velocityCoefficientFinal, positionCoefficientFinal);
         processCoefficientMatricesForInitialObjective(rowIndex, decisionVariableIndex[axisOrdinal], axisPositionCoefficientMatrix,
                                                       axisVelocityCoefficientMatrix, axisDecisionVariableCoefficientMatrix, velocityCoefficientInitial,
                                                       positionCoefficientInitial);
         decisionVariableIndex[axisOrdinal]++;
         break;
      case CONSTRAINT:
         axisDecisionVariableBias.set(rowIndex, 0, decisionVariableValue);
         axisVelocityBias1 += decisionVariableValue * velocityCoefficientFinal;
         axisVelocityBias2 += decisionVariableValue * velocityCoefficientInitial;
         axisPositionBias1 += decisionVariableValue * positionCoefficientFinal;
         axisPositionBias2 += decisionVariableValue * positionCoefficientInitial;
         break;
      default:
         throw new RuntimeException("Unhandled constraint type");
      }
      axisVelocityBiasMatrix.set(rowIndex, 0, axisVelocityBias1);
      axisVelocityBiasMatrix.set(rowIndex + 1, 0, axisVelocityBias2);
      axisPositionBiasMatrix.set(rowIndex, 0, axisPositionBias1);
      axisPositionBiasMatrix.set(rowIndex + 1, 0, axisPositionBias2);
   }

   private void processCoefficientMatricesForInitialObjective(int rowIndex, int decisionVariableIndex, DenseMatrix64F axisPositionCoefficientMatrix,
                                                              DenseMatrix64F axisVelocityCoefficientMatrix,
                                                              DenseMatrix64F axisDecisionVariableCoefficientMatrix, double velocityCoefficient,
                                                              double positionCoefficient)
   {
      axisDecisionVariableCoefficientMatrix.set(rowIndex, decisionVariableIndex, 1.0);
      axisVelocityCoefficientMatrix.set(rowIndex + 1, decisionVariableIndex, velocityCoefficient);
      axisPositionCoefficientMatrix.set(rowIndex + 1, decisionVariableIndex, positionCoefficient);
   }

   private void processCoefficientMatricesForFinalObjective(int rowIndex, int decisionVariableIndex, DenseMatrix64F axisPositionCoefficientMatrix,
                                                            DenseMatrix64F axisVelocityCoefficientMatrix, double velocityCoefficient,
                                                            double positionCoefficient)
   {
      axisVelocityCoefficientMatrix.set(rowIndex, decisionVariableIndex, velocityCoefficient);
      axisPositionCoefficientMatrix.set(rowIndex, decisionVariableIndex, positionCoefficient);
   }

   private double getVelocityCoefficientForInitialForce(double deltaT)
   {
      return getVelocityMultiplierForInitialForce() * deltaT;
   }

   private double getVelocityCoefficientForFinalForce(double deltaT)
   {
      return getVelocityMultiplierForFinalForce() * deltaT;
   }

   private double getVelocityCoefficientForInitialForceRate(double deltaT)
   {
      return getVelocityMultiplierForInitialForceRate() * deltaT * deltaT;
   }

   private double getVelocityCoefficientForFinalForceRate(double deltaT)
   {
      return getVelocityMultiplierForFinalForceRate() * deltaT * deltaT;
   }

   private double getPositionCoefficientForInitialForce(double deltaT)
   {
      return getPositionMultiplierForInitialForce() * deltaT * deltaT;
   }

   private double getPositionCoefficientForFinalForce(double deltaT)
   {
      return getPositionMultiplierForFinalForce() * deltaT * deltaT;
   }

   private double getPositionCoefficientForInitialForceRate(double deltaT)
   {
      return getPositionMultiplierForInitialForceRate() * deltaT * deltaT * deltaT;
   }

   private double getPositionCoefficientForFinalForceRate(double deltaT)
   {
      return getPositionMultiplierForFinalForceRate() * deltaT * deltaT * deltaT;
   }

   private double getVelocityMultiplierForInitialForce()
   {
      return 0.5 / robotMass;
   }

   private double getVelocityMultiplierForFinalForce()
   {
      return 0.5 / robotMass;
   }

   private double getVelocityMultiplierForInitialForceRate()
   {
      return 1.0 / (12.0 * robotMass);
   }

   private double getVelocityMultiplierForFinalForceRate()
   {
      return -1.0 / (12.0 * robotMass);
   }

   private double getPositionMultiplierForInitialForce()
   {
      return 0.35 / robotMass;
   }

   private double getPositionMultiplierForFinalForce()
   {
      return 0.15 / robotMass;
   }

   private double getPositionMultiplierForInitialForceRate()
   {
      return 0.05 / robotMass;
   }

   private double getPositionMultiplierForFinalForceRate()
   {
      return -2.0 / (60.0 * robotMass);
   }

   private void processDecisionVariableWeighingMatrix(int decisionVariableIndex, DenseMatrix64F axisDecisionVariableWeightMatrix,
                                                      DenseMatrix64F axisDecisionVariableDesiredValueMatrix, double value, double weight, double defaultWeight)
   {
      if (Double.isFinite(weight))
      {
         axisDecisionVariableWeightMatrix.set(decisionVariableIndex, decisionVariableIndex, weight);
         axisDecisionVariableDesiredValueMatrix.set(decisionVariableIndex, 0, value);
      }
      else
      {
         axisDecisionVariableWeightMatrix.set(decisionVariableIndex, decisionVariableIndex, defaultWeight);
         axisDecisionVariableDesiredValueMatrix.set(decisionVariableIndex, 0, 0.0);
      }
   }

   private void addVelocityContributionToPosition(int rowIndex, DenseMatrix64F positionMatrix, DenseMatrix64F velocityMatrix, double deltaT)
   {
      for (int colIndex = 0; colIndex < positionMatrix.getNumCols(); colIndex++)
         positionMatrix.add(rowIndex, colIndex, velocityMatrix.get(rowIndex - 1, colIndex) * deltaT);
   }

   private void replaceInitialConditionPlaceholder(int rowIndex, DenseMatrix64F matrixToModify)
   {
      replaceInitialConditionPlaceholderAndAddConstantBias(rowIndex, matrixToModify, 0.0);
   }

   private void replaceInitialConditionPlaceholderAndAddConstantBias(int rowIndex, DenseMatrix64F matrixToModify, double value)
   {
      for (int colIndex = 0; colIndex < matrixToModify.getNumCols(); colIndex++)
         matrixToModify.add(rowIndex, colIndex, matrixToModify.get(rowIndex - 1, colIndex) + value);
   }

   public int getNumberOfDecisionVariables(Axis axis)
   {
      return numberOfDecisionVariables[axis.ordinal()];
   }

   public DenseMatrix64F getDeltaTMatrix()
   {
      return deltaT;
   }

   public DenseMatrix64F getForceRateCoefficientMatrix(Axis axis)
   {
      return forceRateCoefficientMatrix[axis.ordinal()];
   }

   public DenseMatrix64F getForceRateBiasMatrix(Axis axis)
   {
      return forceRateBias[axis.ordinal()];
   }

   public DenseMatrix64F getForceCoefficientMatrix(Axis axis)
   {
      return forceCoefficientMatrix[axis.ordinal()];
   }

   public DenseMatrix64F getForceBiasMatrix(Axis axis)
   {
      return forceBias[axis.ordinal()];
   }

   public DenseMatrix64F getVelocityCoefficientMatrix(Axis axis)
   {
      return velocityCoefficientMatrix[axis.ordinal()];
   }

   public DenseMatrix64F getVelocityBiasMatrix(Axis axis)
   {
      return velocityBias[axis.ordinal()];
   }

   public DenseMatrix64F getPositionCoefficientMatrix(Axis axis)
   {
      return positionCoefficientMatrix[axis.ordinal()];
   }

   public DenseMatrix64F getPositionBiasMatrix(Axis axis)
   {
      return positionBias[axis.ordinal()];
   }
}
