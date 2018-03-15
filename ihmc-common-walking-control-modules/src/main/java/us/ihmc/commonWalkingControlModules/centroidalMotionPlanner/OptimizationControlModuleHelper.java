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
      int rowIndex = 1;
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
         double deltaT0 = deltaT.get(rowIndex - 1, 0);
         double axisVelocityBias = axisVelocityBiasMatrix.get(rowIndex, 0);
         double axisPositionBias = axisPositionBiasMatrix.get(rowIndex, 0);
         double forceValue = node.getForceValue(axis);
         switch (node.getForceConstraintType(axis))
         {
         case OBJECTIVE:
            double forceWeight = node.getForceWeight(axis);
            if(Double.isFinite(forceWeight))
            {
               axisdecisionVariableWeightMatrix.set(decisionVariableIndex[axisOrdinal], decisionVariableIndex[axisOrdinal], forceWeight);
               axisdecisionVariableDesiredValueMatrix.set(decisionVariableIndex[axisOrdinal], 0, forceValue);
            }
            else
            {
               axisdecisionVariableWeightMatrix.set(decisionVariableIndex[axisOrdinal], decisionVariableIndex[axisOrdinal], forceRegularizationWeight);
               axisdecisionVariableDesiredValueMatrix.set(decisionVariableIndex[axisOrdinal], 0, 0.0);
            }
            axisForceCoefficientMatrix.set(rowIndex - 1, decisionVariableIndex[axisOrdinal], 1.0);
            axisVelocityCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], deltaT0 / (2.0 * robotMass));
            axisPositionCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], (21.0 / 60.0) * deltaT0 * deltaT0 / robotMass);
            decisionVariableIndex[axisOrdinal]++;
            break;
         case EQUALITY:
            axisForceBias.set(rowIndex - 1, 0, forceValue);
            axisVelocityBias += forceValue * deltaT0 / (2.0 * robotMass);
            axisPositionBias += (21.0 / 60.0) * deltaT0 * deltaT0 * forceValue / robotMass;
            break;
         default:
            throw new RuntimeException("Unhandled constraint type");
         }

         double dForceValue = node.getRateChangeOfForceValue(axis);
         switch (node.getRateChangeForceConstraintType(axis))
         {
         case OBJECTIVE:
            double dForceWeight = node.getRateChangeOfForceWeight(axis);
            if(Double.isFinite(dForceWeight))
            {
               axisdecisionVariableWeightMatrix.set(decisionVariableIndex[axisOrdinal], decisionVariableIndex[axisOrdinal], dForceWeight);
               axisdecisionVariableDesiredValueMatrix.set(decisionVariableIndex[axisOrdinal], 0, dForceValue);
            }
            else
            {
               axisdecisionVariableWeightMatrix.set(decisionVariableIndex[axisOrdinal], decisionVariableIndex[axisOrdinal], dForceRegularizationWeight);
               axisdecisionVariableDesiredValueMatrix.set(decisionVariableIndex[axisOrdinal], 0, 0.0);
            }

            axisRateChangeOfForceCoefficientMatrix.set(rowIndex - 1, decisionVariableIndex[axisOrdinal], 1.0);
            axisVelocityCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], deltaT0 * deltaT0 / (12.0 * robotMass));
            axisPositionCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], (3.0 / 60.0) * deltaT0 * deltaT0 * deltaT0 / robotMass);
            decisionVariableIndex[axisOrdinal]++;
            break;
         case EQUALITY:
            axisRateChangeOfForceBias.set(rowIndex - 1, 0, dForceValue);
            axisVelocityBias += dForceValue * deltaT0 * deltaT0 / (12.0 * robotMass);
            axisPositionBias += (3.0 / 60.0) * deltaT0 * deltaT0 * deltaT0 * dForceValue / robotMass;
            break;
         default:
            throw new RuntimeException("Unhandled constraint type");
         }
         axisVelocityBiasMatrix.set(rowIndex, 0, axisVelocityBias);
         axisPositionBiasMatrix.set(rowIndex, 0, axisPositionBias);
      }

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
               if(Double.isFinite(forceWeight))
               {
                  axisdecisionVariableWeightMatrix.set(decisionVariableIndex[axisOrdinal], decisionVariableIndex[axisOrdinal], forceWeight);
                  axisdecisionVariableDesiredValueMatrix.set(decisionVariableIndex[axisOrdinal], 0, forceValue);
               }
               else
               {
                  axisdecisionVariableWeightMatrix.set(decisionVariableIndex[axisOrdinal], decisionVariableIndex[axisOrdinal], forceRegularizationWeight);
                  axisdecisionVariableDesiredValueMatrix.set(decisionVariableIndex[axisOrdinal], 0, 0.0);
               }
               axisForceCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], 1.0);
               axisVelocityCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], 0.5 * deltaTim1 / robotMass);
               axisVelocityCoefficientMatrix.set(rowIndex + 1, decisionVariableIndex[axisOrdinal], 0.5 * deltaTi / robotMass);
               axisPositionCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], 0.15 * deltaTim1 * deltaTim1 / robotMass);
               axisPositionCoefficientMatrix.set(rowIndex + 1, decisionVariableIndex[axisOrdinal], 0.35 * deltaTi * deltaTi / robotMass);
               decisionVariableIndex[axisOrdinal]++;
               break;
            case EQUALITY:
               axisForceBias.set(rowIndex, 0, forceValue);
               axisVelocityBias1 += forceValue * 0.5 * deltaTim1 / robotMass;
               axisVelocityBias2 += forceValue * 0.5 * deltaTi / robotMass;
               axisPositionBias1 += forceValue * 0.15 * deltaTim1 * deltaTim1 / robotMass;
               axisPositionBias2 += forceValue * 0.35 * deltaTi * deltaTi / robotMass;
               break;
            default:
               throw new RuntimeException("Unhandled constraint type");
            }

            switch (dForceConstraintType)
            {
            case OBJECTIVE:
               double dForceWeight = node.getRateChangeOfForceWeight(axis);
               if(Double.isFinite(dForceWeight))
               {
                  axisdecisionVariableWeightMatrix.set(decisionVariableIndex[axisOrdinal], decisionVariableIndex[axisOrdinal], dForceWeight);
                  axisdecisionVariableDesiredValueMatrix.set(decisionVariableIndex[axisOrdinal], 0, dForceValue);
               }
               else
               {
                  axisdecisionVariableWeightMatrix.set(decisionVariableIndex[axisOrdinal], decisionVariableIndex[axisOrdinal], dForceRegularizationWeight);
                  axisdecisionVariableDesiredValueMatrix.set(decisionVariableIndex[axisOrdinal], 0, 0.0);
               }
               axisRateChangeOfForceCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], 1.0);
               axisVelocityCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], -deltaTim1 * deltaTim1 / (12.0 * robotMass));
               axisVelocityCoefficientMatrix.set(rowIndex + 1, decisionVariableIndex[axisOrdinal], deltaTi * deltaTi / (12.0 * robotMass));
               axisPositionCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], (-2.0 / 60.0) * deltaTim1 * deltaTim1 * deltaTim1 / robotMass);
               axisPositionCoefficientMatrix.set(rowIndex + 1, decisionVariableIndex[axisOrdinal], (3.0 / 60.0) * deltaTi * deltaTi * deltaTi / robotMass);
               decisionVariableIndex[axisOrdinal]++;
               break;
            case EQUALITY:
               axisRateChangeOfForceBias.set(rowIndex, 0, dForceValue);
               axisVelocityBias1 += -dForceValue * deltaTim1 * deltaTim1 / (12.0 * robotMass);
               axisVelocityBias2 += dForceValue * deltaTi * deltaTi / (12.0 * robotMass);
               axisPositionBias1 += dForceValue * (-2.0 / 60.0) * deltaTim1 * deltaTim1 * deltaTim1 / robotMass;
               axisPositionBias2 += dForceValue * (3.0 / 60.0) * deltaTim1 * deltaTim1 * deltaTim1 / robotMass;
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
         DenseMatrix64F axisdecisionVariableWeightMatrix = decisionVariableWeightMatrix[axisOrdinal];
         DenseMatrix64F axisdecisionVariableDesiredValueMatrix = decisionVariableDesiredValueMatrix[axisOrdinal];
         double axisVelocityBias = axisVelocityBiasMatrix.get(rowIndex, 0);
         double axisPositionBias = axisPositionBiasMatrix.get(rowIndex, 0);
         double forceValue = node.getForceValue(axis);
         switch (node.getForceConstraintType(axis))
         {
         case OBJECTIVE:
            double forceWeight = node.getForceWeight(axis);
            if(Double.isFinite(forceWeight))
            {
               axisdecisionVariableWeightMatrix.set(decisionVariableIndex[axisOrdinal], decisionVariableIndex[axisOrdinal], forceWeight);
               axisdecisionVariableDesiredValueMatrix.set(decisionVariableIndex[axisOrdinal], 0, forceValue);
            }
            else
            {
               axisdecisionVariableWeightMatrix.set(decisionVariableIndex[axisOrdinal], decisionVariableIndex[axisOrdinal], forceRegularizationWeight);
               axisdecisionVariableDesiredValueMatrix.set(decisionVariableIndex[axisOrdinal], 0, 0.0);
            }
            axisForceCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], 1.0);
            axisVelocityCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], deltaTF / (2.0 * robotMass));
            axisPositionCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], (9.0 / 60.0) * deltaTF * deltaTF / robotMass);
            decisionVariableIndex[axisOrdinal]++;
            break;
         case EQUALITY:
            axisForceBias.set(rowIndex, 0, forceValue);
            axisVelocityBias += forceValue * deltaTF / (2.0 * robotMass);
            axisPositionBias += forceValue * (9.0 / 60.0) * deltaTF * deltaTF / robotMass;
            break;
         default:
            throw new RuntimeException("Unhandled constraint type");
         }

         double dForceValue = node.getRateChangeOfForceValue(axis);
         switch (node.getRateChangeForceConstraintType(axis))
         {
         case OBJECTIVE:
            double dForceWeight = node.getRateChangeOfForceWeight(axis);
            if(Double.isFinite(dForceWeight))
            {
               axisdecisionVariableWeightMatrix.set(decisionVariableIndex[axisOrdinal], decisionVariableIndex[axisOrdinal], dForceWeight);
               axisdecisionVariableDesiredValueMatrix.set(decisionVariableIndex[axisOrdinal], 0, dForceValue);
            }
            else
            {
               axisdecisionVariableWeightMatrix.set(decisionVariableIndex[axisOrdinal], decisionVariableIndex[axisOrdinal], dForceRegularizationWeight);
               axisdecisionVariableDesiredValueMatrix.set(decisionVariableIndex[axisOrdinal], 0, 0.0);
            }
            axisRateChangeOfForceCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], 1.0);
            axisVelocityCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], -deltaTF * deltaTF / (12.0 * robotMass));
            axisPositionCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], (-2.0 / 60.0) * deltaTF * deltaTF * deltaTF / robotMass);
            decisionVariableIndex[axisOrdinal]++;
            break;
         case EQUALITY:
            axisRateChangeOfForceBias.set(rowIndex, 0, dForceValue);
            axisVelocityBias += -dForceValue * deltaTF * deltaTF / (12.0 * robotMass);
            axisPositionBias += dForceValue * (-2.0 / 60.0) * deltaTF * deltaTF * deltaTF / robotMass;
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
