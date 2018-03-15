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
   private final DenseMatrix64F[] rateChangeOfForceCoefficientMatrix = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] rateChangeOfForceBias = new DenseMatrix64F[numberOfAxis];

   private final DenseMatrix64F[] decisionVariableWeightMatrix = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] decisionVariableDesiredValueMatrix = new DenseMatrix64F[numberOfAxis];
   private final double forceRegularizationWeight;
   private final double dForceRegularizationWeight;

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
         rateChangeOfForceCoefficientMatrix[i] = new DenseMatrix64F(defaultNumberOfNodes, defaultNumberOfNodes * 2);
         rateChangeOfForceBias[i] = new DenseMatrix64F(defaultNumberOfNodes, 1);
         decisionVariableWeightMatrix[i] = new DenseMatrix64F(defaultNumberOfNodes * 2, defaultNumberOfNodes * 2);
         decisionVariableDesiredValueMatrix[i] = new DenseMatrix64F(defaultNumberOfNodes * 2, 1);
      }
      this.forceRegularizationWeight = parameters.getdForceRegularizationWeight();
      this.dForceRegularizationWeight = parameters.getdForceRegularizationWeight();
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
         rateChangeOfForceCoefficientMatrix[i].reshape(numberOfNodes, numberOfDecisionVariables[i]);
         rateChangeOfForceBias[i].reshape(numberOfNodes, 1);
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
         rateChangeOfForceCoefficientMatrix[i].zero();
         rateChangeOfForceBias[i].zero();
      }
   }

   private final DenseMatrix64F tempMatrixForCoefficients = new DenseMatrix64F(0, 1);
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
         if (entry.element.getForceConstraintType(axis) == EffortConstraintType.OBJECTIVE)
            numberOfDecisionVariables[axis.ordinal()] = 1;
         else
            numberOfDecisionVariables[axis.ordinal()] = 0;
         if (entry.element.getRateChangeForceConstraintType(axis) == EffortConstraintType.OBJECTIVE)
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
            if (nextElement.getForceConstraintType(axis) == EffortConstraintType.OBJECTIVE)
               numberOfDecisionVariables[axis.ordinal()] += 1;
            if (nextElement.getRateChangeForceConstraintType(axis) == EffortConstraintType.OBJECTIVE)
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
         DenseMatrix64F axisRateChangeOfForceCoefficientMatrix = rateChangeOfForceCoefficientMatrix[axisOrdinal];
         DenseMatrix64F axisRateChangeOfForceBias = rateChangeOfForceBias[axisOrdinal];
         DenseMatrix64F axisdecisionVariableWeightMatrix = decisionVariableWeightMatrix[axisOrdinal];
         DenseMatrix64F axisdecisionVariableDesiredValueMatrix = decisionVariableDesiredValueMatrix[axisOrdinal];
         double deltaT0 = deltaT.get(rowIndex, 0);
         double axisVelocityBias = axisVelocityBiasMatrix.get(rowIndex + 1, 0);
         double axisPositionBias = axisPositionBiasMatrix.get(rowIndex + 1, 0);
         double forceValue = node.getForceValue(axis);
         switch (node.getForceConstraintType(axis))
         {
         case OBJECTIVE:
            double forceWeight = node.getForceWeight(axis);
            processDecisionVariableWeighingMatrix(decisionVariableIndex[axisOrdinal], axisdecisionVariableWeightMatrix, axisdecisionVariableDesiredValueMatrix,
                                                  forceValue, forceWeight, forceRegularizationWeight);
            processCoefficientMatricesForInitialForceObjective(rowIndex, decisionVariableIndex[axisOrdinal], axisPositionCoefficientMatrix,
                                                               axisVelocityCoefficientMatrix, axisForceCoefficientMatrix, deltaT0);
            decisionVariableIndex[axisOrdinal]++;
            break;
         case EQUALITY:
            axisForceBias.set(rowIndex, 0, forceValue);
            axisVelocityBias += getVelocityBiasForForceEquality(deltaT0, forceValue);
            axisPositionBias += getPositionBiasForInitialForceEquality(deltaT0, forceValue);
            break;
         default:
            throw new RuntimeException("Unhandled constraint type");
         }

         double dForceValue = node.getRateChangeOfForceValue(axis);
         switch (node.getRateChangeForceConstraintType(axis))
         {
         case OBJECTIVE:
            double dForceWeight = node.getRateChangeOfForceWeight(axis);
            processDecisionVariableWeighingMatrix(decisionVariableIndex[axisOrdinal], axisdecisionVariableWeightMatrix, axisdecisionVariableDesiredValueMatrix,
                                                  dForceValue, dForceWeight, dForceRegularizationWeight);
            processCoefficientMatricesForInitialForceRateObjective(rowIndex, decisionVariableIndex[axisOrdinal], axisPositionCoefficientMatrix,
                                                                   axisVelocityCoefficientMatrix, axisRateChangeOfForceCoefficientMatrix, deltaT0);
            decisionVariableIndex[axisOrdinal]++;
            break;
         case EQUALITY:
            axisRateChangeOfForceBias.set(rowIndex, 0, dForceValue);
            axisVelocityBias += getVelocityBiasForInitialForceRateEquality(deltaT0, dForceValue);
            axisPositionBias += getPositionBiasForInitialForceRateEquality(deltaT0, dForceValue);
            break;
         default:
            throw new RuntimeException("Unhandled constraint type");
         }
         axisVelocityBiasMatrix.set(rowIndex + 1, 0, axisVelocityBias);
         axisPositionBiasMatrix.set(rowIndex + 1, 0, axisPositionBias);
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
            DenseMatrix64F axisRateChangeOfForceCoefficientMatrix = rateChangeOfForceCoefficientMatrix[axisOrdinal];
            DenseMatrix64F axisRateChangeOfForceBias = rateChangeOfForceBias[axisOrdinal];
            DenseMatrix64F axisdecisionVariableWeightMatrix = decisionVariableWeightMatrix[axisOrdinal];
            DenseMatrix64F axisdecisionVariableDesiredValueMatrix = decisionVariableDesiredValueMatrix[axisOrdinal];
            double deltaTim1 = deltaT.get(rowIndex - 1, 0);
            double deltaTi = deltaT.get(rowIndex, 0);
            double forceValue = node.getForceValue(axis);
            double dForceValue = node.getRateChangeOfForceValue(axis);
            double axisVelocityBias1 = axisVelocityBiasMatrix.get(rowIndex, 0);
            double axisVelocityBias2 = axisVelocityBiasMatrix.get(rowIndex + 1, 0);
            double axisPositionBias1 = axisPositionBiasMatrix.get(rowIndex, 0);
            double axisPositionBias2 = axisPositionBiasMatrix.get(rowIndex + 1, 0);
            EffortConstraintType forceConstraintType = node.getForceConstraintType(axis);
            EffortConstraintType dForceConstraintType = node.getRateChangeForceConstraintType(axis);

            switch (forceConstraintType)
            {
            case OBJECTIVE:
               double forceWeight = node.getForceWeight(axis);
               processDecisionVariableWeighingMatrix(decisionVariableIndex[axisOrdinal], axisdecisionVariableWeightMatrix,
                                                     axisdecisionVariableDesiredValueMatrix, forceValue, forceWeight, forceRegularizationWeight);
               processCoefficientMatricesForFinalForceObjective(rowIndex, decisionVariableIndex[axisOrdinal], axisPositionCoefficientMatrix,
                                                                axisVelocityCoefficientMatrix, deltaTim1);
               processCoefficientMatricesForInitialForceObjective(rowIndex, decisionVariableIndex[axisOrdinal], axisPositionCoefficientMatrix,
                                                                  axisVelocityCoefficientMatrix, axisForceCoefficientMatrix, deltaTi);
               decisionVariableIndex[axisOrdinal]++;
               break;
            case EQUALITY:
               axisForceBias.set(rowIndex, 0, forceValue);
               axisVelocityBias1 += getVelocityBiasForForceEquality(deltaTim1, forceValue);
               axisVelocityBias2 += getVelocityBiasForForceEquality(deltaTi, forceValue);
               axisPositionBias1 += getPositionBiasForFinalForceEquality(deltaTim1, forceValue);
               axisPositionBias2 += getPositionBiasForInitialForceEquality(deltaTi, forceValue);
               break;
            default:
               throw new RuntimeException("Unhandled constraint type");
            }

            switch (dForceConstraintType)
            {
            case OBJECTIVE:
               double dForceWeight = node.getRateChangeOfForceWeight(axis);
               processDecisionVariableWeighingMatrix(decisionVariableIndex[axisOrdinal], axisdecisionVariableWeightMatrix,
                                                     axisdecisionVariableDesiredValueMatrix, dForceValue, dForceWeight, dForceRegularizationWeight);
               processCoefficientMatricesForFinalForceRateObjective(rowIndex, decisionVariableIndex[axisOrdinal], axisPositionCoefficientMatrix,
                                                                    axisVelocityCoefficientMatrix, deltaTim1);
               processCoefficientMatricesForInitialForceRateObjective(rowIndex, decisionVariableIndex[axisOrdinal], axisPositionCoefficientMatrix,
                                                                      axisVelocityCoefficientMatrix, axisRateChangeOfForceCoefficientMatrix, deltaTi);
               decisionVariableIndex[axisOrdinal]++;
               break;
            case EQUALITY:
               axisRateChangeOfForceBias.set(rowIndex, 0, dForceValue);
               axisVelocityBias1 += getVelocityBiasForFinalForceRateEquality(deltaTim1, dForceValue);
               axisVelocityBias2 += getVelocityBiasForInitialForceRateEquality(deltaTi, dForceValue);
               axisPositionBias1 += getPositionBiasForFinalForceRateEquality(deltaTim1, dForceValue);
               axisPositionBias2 += getPositionBiasForInitialForceRateEquality(deltaTi, dForceValue);
               break;
            default:
               throw new RuntimeException("Unhandled constraint type");
            }
            axisVelocityBiasMatrix.set(rowIndex, 0, axisVelocityBias1);
            axisVelocityBiasMatrix.set(rowIndex + 1, 0, axisVelocityBias2);
            axisPositionBiasMatrix.set(rowIndex, 0, axisPositionBias1);
            axisPositionBiasMatrix.set(rowIndex + 1, 0, axisPositionBias2);
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
         DenseMatrix64F axisRateChangeOfForceCoefficientMatrix = rateChangeOfForceCoefficientMatrix[axisOrdinal];
         DenseMatrix64F axisRateChangeOfForceBias = rateChangeOfForceBias[axisOrdinal];
         DenseMatrix64F axisDecisionVariableWeightMatrix = decisionVariableWeightMatrix[axisOrdinal];
         DenseMatrix64F axisDecisionVariableDesiredValueMatrix = decisionVariableDesiredValueMatrix[axisOrdinal];
         double axisVelocityBias = axisVelocityBiasMatrix.get(rowIndex, 0);
         double axisPositionBias = axisPositionBiasMatrix.get(rowIndex, 0);
         double forceValue = node.getForceValue(axis);
         double dForceValue = node.getRateChangeOfForceValue(axis);
         switch (node.getForceConstraintType(axis))
         {
         case OBJECTIVE:
            double forceWeight = node.getForceWeight(axis);
            processDecisionVariableWeighingMatrix(decisionVariableIndex[axisOrdinal], axisDecisionVariableWeightMatrix, axisDecisionVariableDesiredValueMatrix,
                                                  forceValue, forceWeight, forceRegularizationWeight);
            processCoefficientMatricesForFinalForceObjective(rowIndex, decisionVariableIndex[axisOrdinal], axisPositionCoefficientMatrix,
                                                             axisVelocityCoefficientMatrix, deltaTF);
            axisForceCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], 1.0);
            decisionVariableIndex[axisOrdinal]++;
            break;
         case EQUALITY:
            axisForceBias.set(rowIndex, 0, forceValue);
            axisVelocityBias += getVelocityBiasForForceEquality(deltaTF, forceValue);
            axisPositionBias += getPositionBiasForFinalForceEquality(deltaTF, forceValue);
            break;
         default:
            throw new RuntimeException("Unhandled constraint type");
         }

         switch (node.getRateChangeForceConstraintType(axis))
         {
         case OBJECTIVE:
            double dForceWeight = node.getRateChangeOfForceWeight(axis);
            processDecisionVariableWeighingMatrix(decisionVariableIndex[axisOrdinal], axisDecisionVariableWeightMatrix, axisDecisionVariableDesiredValueMatrix,
                                                  dForceValue, dForceWeight, dForceRegularizationWeight);
            processCoefficientMatricesForFinalForceRateObjective(rowIndex, axisOrdinal, axisPositionCoefficientMatrix, axisVelocityCoefficientMatrix, deltaTF);
            axisRateChangeOfForceCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], 1.0);
            decisionVariableIndex[axisOrdinal]++;
            break;
         case EQUALITY:
            axisRateChangeOfForceBias.set(rowIndex, 0, dForceValue);
            axisVelocityBias += getVelocityBiasForFinalForceRateEquality(deltaTF, dForceValue);
            axisPositionBias += getPositionBiasForFinalForceRateEquality(deltaTF, dForceValue);
            break;
         default:
            throw new RuntimeException("Unhandled constraint type");
         }
         axisVelocityBiasMatrix.set(rowIndex, 0, axisVelocityBias);
         axisPositionBiasMatrix.set(rowIndex, 0, axisPositionBias);
         replaceInitialConditionPlaceholder(rowIndex, axisVelocityCoefficientMatrix);
         replaceInitialConditionPlaceholderAndAddConstantBias(rowIndex, axisVelocityBiasMatrix, deltaTF * gravity[axisOrdinal]);
         replaceInitialConditionPlaceholder(rowIndex, axisPositionCoefficientMatrix);
         replaceInitialConditionPlaceholderAndAddConstantBias(rowIndex, axisPositionBiasMatrix, 0.5 * deltaTF * deltaTF * gravity[axisOrdinal]);
         addVelocityContributionToPosition(rowIndex, axisPositionCoefficientMatrix, axisVelocityCoefficientMatrix, deltaTF);
         addVelocityContributionToPosition(rowIndex, axisPositionBiasMatrix, axisVelocityBiasMatrix, deltaTF);
      }
   }

   private double getPositionBiasForInitialForceRateEquality(double deltaT, double dForceValue)
   {
      return dForceValue * (3.0 / 60.0) * deltaT * deltaT * deltaT / robotMass;
   }

   private double getPositionBiasForFinalForceRateEquality(double deltaTim1, double dForceValue)
   {
      return dForceValue * (-2.0 / 60.0) * deltaTim1 * deltaTim1 * deltaTim1 / robotMass;
   }

   private double getVelocityBiasForInitialForceRateEquality(double deltaTi, double dForceValue)
   {
      return dForceValue * deltaTi * deltaTi / (12.0 * robotMass);
   }

   private double getVelocityBiasForFinalForceRateEquality(double deltaTim1, double dForceValue)
   {
      return -dForceValue * deltaTim1 * deltaTim1 / (12.0 * robotMass);
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

   private void processCoefficientMatricesForInitialForceRateObjective(int rowIndex, int decisionVariableIndex, DenseMatrix64F axisPositionCoefficientMatrix,
                                                                       DenseMatrix64F axisVelocityCoefficientMatrix,
                                                                       DenseMatrix64F axisRateChangeOfForceCoefficientMatrix, double deltaTi)
   {
      axisRateChangeOfForceCoefficientMatrix.set(rowIndex, decisionVariableIndex, 1.0);
      axisVelocityCoefficientMatrix.set(rowIndex + 1, decisionVariableIndex, deltaTi * deltaTi / (12.0 * robotMass));
      axisPositionCoefficientMatrix.set(rowIndex + 1, decisionVariableIndex, (3.0 / 60.0) * deltaTi * deltaTi * deltaTi / robotMass);
   }

   private void processCoefficientMatricesForInitialForceObjective(int rowIndex, int decisionVariableIndex, DenseMatrix64F axisPositionCoefficientMatrix,
                                                                   DenseMatrix64F axisVelocityCoefficientMatrix, DenseMatrix64F axisForceCoefficientMatrix,
                                                                   double deltaTi)
   {
      axisForceCoefficientMatrix.set(rowIndex, decisionVariableIndex, 1.0);
      axisVelocityCoefficientMatrix.set(rowIndex + 1, decisionVariableIndex, 0.5 * deltaTi / robotMass);
      axisPositionCoefficientMatrix.set(rowIndex + 1, decisionVariableIndex, 0.35 * deltaTi * deltaTi / robotMass);
   }

   private void processCoefficientMatricesForFinalObjective(int rowIndex, int decisionVariableIndex, DenseMatrix64F axisPositionCoefficientMatrix,
                                                                     DenseMatrix64F axisVelocityCoefficientMatrix, double velocityCoefficient, double positionCoefficient)
   {
      axisVelocityCoefficientMatrix.set(rowIndex, decisionVariableIndex, velocityCoefficient);
      axisPositionCoefficientMatrix.set(rowIndex, decisionVariableIndex, positionCoefficient);
   }
   
   
   private void processCoefficientMatricesForFinalForceRateObjective(int rowIndex, int decisionVariableIndex, DenseMatrix64F axisPositionCoefficientMatrix,
                                                                     DenseMatrix64F axisVelocityCoefficientMatrix, double deltaTim1)
   {
      axisVelocityCoefficientMatrix.set(rowIndex, decisionVariableIndex, -deltaTim1 * deltaTim1 / (12.0 * robotMass));
      axisPositionCoefficientMatrix.set(rowIndex, decisionVariableIndex, (-2.0 / 60.0) * deltaTim1 * deltaTim1 * deltaTim1 / robotMass);
   }

   private void processCoefficientMatricesForFinalForceObjective(int rowIndex, int decisionVariableIndex, DenseMatrix64F axisPositionCoefficientMatrix,
                                                                 DenseMatrix64F axisVelocityCoefficientMatrix, double deltaTim1)
   {
      axisVelocityCoefficientMatrix.set(rowIndex, decisionVariableIndex, 0.5 * deltaTim1 / robotMass);
      axisPositionCoefficientMatrix.set(rowIndex, decisionVariableIndex, 0.15 * deltaTim1 * deltaTim1 / robotMass);
   }

   private double getPositionBiasForInitialForceEquality(double deltaTi, double forceValue)
   {
      return forceValue * 0.35 * deltaTi * deltaTi / robotMass;
   }

   private double getPositionBiasForFinalForceEquality(double deltaTim1, double forceValue)
   {
      return forceValue * 0.15 * deltaTim1 * deltaTim1 / robotMass;
   }

   private double getVelocityBiasForForceEquality(double deltaT, double forceValue)
   {
      return forceValue * 0.5 * deltaT / robotMass;
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

   public DenseMatrix64F getRateChangeOfForceCoefficientMatrix(Axis axis)
   {
      return rateChangeOfForceCoefficientMatrix[axis.ordinal()];
   }

   public DenseMatrix64F getRateChangeOfForceBiasMatrix(Axis axis)
   {
      return rateChangeOfForceBias[axis.ordinal()];
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
