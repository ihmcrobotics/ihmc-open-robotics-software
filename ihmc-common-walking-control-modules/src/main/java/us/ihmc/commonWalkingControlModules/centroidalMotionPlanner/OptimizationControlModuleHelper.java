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
   private final DenseMatrix64F[] forceCoefficientMatrix = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] positionCoefficientMatrix = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] positionBias = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] velocityCoefficientMatrix = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] velocityBias = new DenseMatrix64F[numberOfAxis];

   private int numberOfNodes;
   private final int[] numberOfDecisionVariables = new int[numberOfAxis];
   private final double[] gravity = new double[numberOfAxis];
   private final double robotMass;
   private RecycledLinkedListBuilder<CentroidalMotionNode> nodeList;

   public OptimizationControlModuleHelper(double gravityX, double gravityY, double gravityZ, double robotMass)
   {
      this.robotMass = robotMass;
      for (int i = 0; i < numberOfAxis; i++)
      {
         forceCoefficientMatrix[i] = new DenseMatrix64F(defaultNumberOfNodes * 2, defaultNumberOfNodes * 2);
         positionCoefficientMatrix[i] = new DenseMatrix64F(defaultNumberOfNodes, defaultNumberOfNodes * 2);
         positionBias[i] = new DenseMatrix64F(defaultNumberOfNodes, 1);
         velocityCoefficientMatrix[i] = new DenseMatrix64F(defaultNumberOfNodes, defaultNumberOfNodes * 2);
         velocityBias[i] = new DenseMatrix64F(defaultNumberOfNodes, 1);
      }
      gravity[0] = gravityX;
      gravity[1] = gravityY;
      gravity[2] = gravityZ;
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
         forceCoefficientMatrix[i].reshape(numberOfNodes * 2, numberOfDecisionVariables[i]);
         positionCoefficientMatrix[i].reshape(numberOfNodes, numberOfDecisionVariables[i]);
         positionBias[i].reshape(numberOfNodes, 1);
         velocityCoefficientMatrix[i].reshape(numberOfNodes, numberOfDecisionVariables[i]);
         velocityBias[i].reshape(numberOfNodes, 1);
      }
   }

   private void setCoefficientsToZero()
   {
      for (int i = 0; i < numberOfAxis; i++)
      {
         forceCoefficientMatrix[i].zero();
         positionCoefficientMatrix[i].zero();
         positionBias[i].zero();
         velocityCoefficientMatrix[i].zero();
         velocityBias[i].zero();
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
      getForceVelocityPositionCoefficientMatrices(nodeList);
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

   private void getForceVelocityPositionCoefficientMatrices(RecycledLinkedListBuilder<CentroidalMotionNode> nodeList)
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
         double deltaT0 = deltaT.get(rowIndex - 1, 0);
         double axisVelocityBias = axisVelocityBiasMatrix.get(rowIndex, 0);
         double axisPositionBias = axisPositionBiasMatrix.get(rowIndex, 0);
         double forceValue = node.getForceValue(axis);
         switch (node.getForceConstraintType(axis))
         {
         case OBJECTIVE:
            axisVelocityCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], deltaT0 / (2.0 * robotMass));
            axisPositionCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], (21.0 / 60.0) * deltaT0 * deltaT0 / robotMass);
            decisionVariableIndex[axisOrdinal]++;
            break;
         case EQUALITY:
            axisVelocityBias += forceValue * deltaT0 / (2.0 * robotMass);
            axisPositionBias += (21.0 / 60.0) * deltaT0 * deltaT0 * forceValue/ robotMass; 
            break;
         default:
            throw new RuntimeException("Unhandled constraint type");
         }

         double dForceValue = node.getRateChangeOfForceValue(axis);
         switch (node.getRateChangeForceConstraintType(axis))
         {
         case OBJECTIVE:
            axisVelocityCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], deltaT0 * deltaT0 / (12.0 * robotMass));
            axisPositionCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], (3.0 / 60.0) * deltaT0 * deltaT0 * deltaT0 / robotMass);
            decisionVariableIndex[axisOrdinal]++;
            break;
         case EQUALITY:
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
               axisVelocityCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], 0.5 * deltaTim1 / robotMass);
               axisVelocityCoefficientMatrix.set(rowIndex + 1, decisionVariableIndex[axisOrdinal], 0.5 * deltaTi / robotMass);
               axisPositionCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], 0.15 * deltaTim1 * deltaTim1 / robotMass);
               axisPositionCoefficientMatrix.set(rowIndex + 1, decisionVariableIndex[axisOrdinal], 0.35 * deltaTi * deltaTi / robotMass);
               decisionVariableIndex[axisOrdinal]++;
               break;
            case EQUALITY:
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
               axisVelocityCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], -deltaTim1 * deltaTim1 / (12.0 * robotMass));
               axisVelocityCoefficientMatrix.set(rowIndex + 1, decisionVariableIndex[axisOrdinal], deltaTi * deltaTi / (12.0 * robotMass));
               axisPositionCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], (-2.0 / 60.0) * deltaTim1 * deltaTim1 * deltaTim1 / robotMass);
               axisPositionCoefficientMatrix.set(rowIndex + 1, decisionVariableIndex[axisOrdinal], (3.0 / 60.0) * deltaTi * deltaTi * deltaTi / robotMass);
               decisionVariableIndex[axisOrdinal]++;
               break;
            case EQUALITY:
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
            replaceInitialConditionPlaceholderAndAddConstantBias(rowIndex, axisPositionBiasMatrix, 0.5  * deltaTim1 * deltaTim1 * gravity[axisOrdinal]);
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
         double axisVelocityBias = axisVelocityBiasMatrix.get(rowIndex, 0);
         double axisPositionBias = axisPositionBiasMatrix.get(rowIndex, 0);
         double forceValue = node.getForceValue(axis);
         switch (node.getForceConstraintType(axis))
         {
         case OBJECTIVE:
            axisVelocityCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], deltaTF / (2.0 * robotMass));
            axisPositionCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], (9.0 / 60.0) * deltaTF * deltaTF / robotMass);
            decisionVariableIndex[axisOrdinal]++;
            break;
         case EQUALITY:
            axisVelocityBias += forceValue * deltaTF / (2.0 * robotMass);
            axisPositionBias += forceValue * (9.0 / 60.0) * deltaTF * deltaTF / robotMass;
            break;
         default:
            throw new RuntimeException("Unhandled constraint type");
         }

         switch (node.getRateChangeForceConstraintType(axis))
         {
         case OBJECTIVE:
            axisVelocityCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], -deltaTF * deltaTF / (12.0 * robotMass));
            axisPositionCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], (-2.0 / 60.0) * deltaTF * deltaTF * deltaTF / robotMass);
            decisionVariableIndex[axisOrdinal]++;
            break;
         case EQUALITY:
            axisVelocityBias += -node.getRateChangeOfForceValue(axis) * deltaTF * deltaTF / (12.0 * robotMass);
            axisPositionBias += node.getRateChangeOfForceValue(axis)  * (-2.0 / 60.0) * deltaTF * deltaTF * deltaTF / robotMass;
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
