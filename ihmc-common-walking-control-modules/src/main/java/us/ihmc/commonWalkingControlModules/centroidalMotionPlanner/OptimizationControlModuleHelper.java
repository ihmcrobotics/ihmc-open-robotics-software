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
         decisionVariableIndex[axis.ordinal()] = 0;
         double deltaT0 = deltaT.get(0, 0);
         double axisBias = 0.0;
         switch (node.getForceConstraintType(axis))
         {
         case OBJECTIVE:
            velocityCoefficientMatrix[axis.ordinal()].set(rowIndex, decisionVariableIndex[axis.ordinal()]++, deltaT0 / (2.0 * robotMass));
            break;
         case EQUALITY:
            axisBias += node.getForceValue(axis) * deltaT0 / (2.0 * robotMass);
            break;
         default:
            throw new RuntimeException("Unhandled constraint type");
         }

         switch (node.getRateChangeForceConstraintType(axis))
         {
         case OBJECTIVE:
            velocityCoefficientMatrix[axis.ordinal()].set(rowIndex, decisionVariableIndex[axis.ordinal()]++, deltaT0 * deltaT0 / (12.0 * robotMass));
            break;
         case EQUALITY:
            axisBias += node.getRateChangeOfForceValue(axis) * deltaT0 * deltaT0 / (12.0 * robotMass);
            break;
         default:
            throw new RuntimeException("Unhandled constraint type");
         }
         velocityBias[axis.ordinal()].set(rowIndex, 0, axisBias);
      }

      for (entry = entry.getNext(); entry.getNext() != null; entry = entry.getNext(), rowIndex++)
      {
         node = entry.element;
         for (Axis axis : Axis.values)
         {
            DenseMatrix64F axisVelocityCoefficientMatrix = velocityCoefficientMatrix[axis.ordinal()];
            DenseMatrix64F axisBiasMatrix = velocityBias[axis.ordinal()];
            double deltaT1m1 = deltaT.get(rowIndex - 1, 0);
            double deltaTi = deltaT.get(rowIndex, 0);
            double forceValue = node.getForceValue(axis);
            double dForceValue = node.getRateChangeOfForceValue(axis);
            double axisBias1 = axisBiasMatrix.get(rowIndex, 0);
            double axisBias2 = axisBiasMatrix.get(rowIndex + 1, 0);
            EffortConstraintType forceConstraintType = node.getForceConstraintType(axis);
            EffortConstraintType dForceConstraintType = node.getRateChangeForceConstraintType(axis);

            switch (forceConstraintType)
            {
            case OBJECTIVE:
               axisVelocityCoefficientMatrix.set(rowIndex, decisionVariableIndex[axis.ordinal()], deltaT1m1 / (2.0 * robotMass));
               axisVelocityCoefficientMatrix.set(rowIndex + 1, decisionVariableIndex[axis.ordinal()], deltaTi / (2.0 * robotMass));
               decisionVariableIndex[axis.ordinal()]++;
               break;
            case EQUALITY:
               axisBias1 += forceValue * deltaT1m1 / (2.0 * robotMass);
               axisBias2 += forceValue * deltaTi / (2.0 * robotMass);
               break;
            default:
               throw new RuntimeException("Unhandled constraint type");
            }

            switch(dForceConstraintType)
            {
            case  OBJECTIVE:
               axisVelocityCoefficientMatrix.set(rowIndex, decisionVariableIndex[axis.ordinal()], -deltaT1m1 * deltaT1m1 / (12.0 * robotMass));
               axisVelocityCoefficientMatrix.set(rowIndex + 1, decisionVariableIndex[axis.ordinal()], deltaTi * deltaTi / (12.0 * robotMass));
               decisionVariableIndex[axis.ordinal()]++;
               break;
            case EQUALITY:
               axisBias1 += -dForceValue * deltaT1m1 * deltaT1m1 / (12.0 * robotMass);
               axisBias2 += dForceValue * deltaTi * deltaTi / (12.0 * robotMass);
               break;
            default:
               throw new RuntimeException("Unhandled constraint type");
            }
            axisBiasMatrix.set(rowIndex, 0, axisBias1);
            axisBiasMatrix.set(rowIndex + 1, 0, axisBias2);
            rowAddEquals(rowIndex, axisVelocityCoefficientMatrix);
            rowAddEquals(rowIndex, axisBiasMatrix, deltaT.get(rowIndex - 1, 0) * gravity[axis.ordinal()]);
         }
      }

      node = entry.element;
      for (Axis axis : Axis.values)
      {
         double deltaTF = deltaT.get(rowIndex - 2, 0);
         switch (node.getForceConstraintType(axis))
         {
         case OBJECTIVE:
            velocityCoefficientMatrix[axis.ordinal()].set(rowIndex, decisionVariableIndex[axis.ordinal()]++, deltaTF / (2.0 * robotMass));
            break;
         case EQUALITY:
            velocityBias[axis.ordinal()].set(rowIndex, 0, node.getForceValue(axis) * deltaTF / (2.0 * robotMass));
            break;
         default:
            throw new RuntimeException("Unhandled constraint type");
         }

         switch (node.getRateChangeForceConstraintType(axis))
         {
         case OBJECTIVE:
            velocityCoefficientMatrix[axis.ordinal()].set(rowIndex, decisionVariableIndex[axis.ordinal()]++, -deltaTF * deltaTF / (12.0 * robotMass));
            break;
         case EQUALITY:
            velocityBias[axis.ordinal()].set(rowIndex, 0, -node.getRateChangeOfForceValue(axis) * deltaTF * deltaTF / (12.0 * robotMass));
            break;
         default:
            throw new RuntimeException("Unhandled constraint type");
         }
         rowAddEquals(rowIndex, velocityCoefficientMatrix[axis.ordinal()]);
         rowAddEquals(rowIndex, velocityBias[axis.ordinal()], deltaTF * gravity[axis.ordinal()]);
      }
   }

   /**
    * Adds the (rowIndex - 1) th row to the rowIndex th row
    * @param rowIndex
    * @param matrixToModify
    */
   private void rowAddEquals(int rowIndex, DenseMatrix64F matrixToModify)
   {
      rowAddEquals(rowIndex, matrixToModify, 0.0);
   }

   private void rowAddEquals(int rowIndex, DenseMatrix64F matrixToModify, double value)
   {
      for (int colIndex = 0; colIndex < matrixToModify.getNumCols(); colIndex++)
      {
         matrixToModify.add(rowIndex, colIndex, matrixToModify.get(rowIndex - 1, colIndex) + value);
      }
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
}
