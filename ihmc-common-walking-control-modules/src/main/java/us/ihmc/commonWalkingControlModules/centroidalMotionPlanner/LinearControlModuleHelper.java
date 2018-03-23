package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tuple3D.Vector3D;

public class LinearControlModuleHelper
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

   private final DenseMatrix64F[] optimizedPositionValues = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] optimizedVelocityValues = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] optimizedForceValues = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] optimizedForceRateValues = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] decisionVariableValues = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] decisionVariableWeightMatrix = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] decisionVariableDesiredValueMatrix = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] H = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] f = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] Aeq = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] beq = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] decisionVariableUpperBoundMatrix = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] decisionVariableLowerBoundMatrix = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] Ain = new DenseMatrix64F[numberOfAxis];
   private final DenseMatrix64F[] bin = new DenseMatrix64F[numberOfAxis];

   private final double forceRegularizationWeight;
   private final double forceRateRegularizationWeight;

   private int numberOfNodes;
   private final int[] numberOfDecisionVariables = new int[numberOfAxis];

   private final Vector3D gravity = new Vector3D();
   private final double robotMass;
   private final Vector3D maxForce = new Vector3D();
   private final Vector3D minForce = new Vector3D();
   private final Vector3D maxForceRate = new Vector3D();
   private final Vector3D minForceRate = new Vector3D();

   private RecycledLinkedListBuilder<CentroidalMotionNode> nodeList;

   public LinearControlModuleHelper(CentroidalMotionPlannerParameters parameters)
   {
      this.robotMass = parameters.getRobotMass();
      for (int i = 0; i < numberOfAxis; i++)
      {
         int defaultNumberOfDecisionVariables = defaultNumberOfNodes * 2;
         positionCoefficientMatrix[i] = new DenseMatrix64F(defaultNumberOfNodes, defaultNumberOfDecisionVariables);
         positionBias[i] = new DenseMatrix64F(defaultNumberOfNodes, 1);
         velocityCoefficientMatrix[i] = new DenseMatrix64F(defaultNumberOfNodes, defaultNumberOfDecisionVariables);
         velocityBias[i] = new DenseMatrix64F(defaultNumberOfNodes, 1);
         forceCoefficientMatrix[i] = new DenseMatrix64F(defaultNumberOfNodes, defaultNumberOfDecisionVariables);
         forceBias[i] = new DenseMatrix64F(defaultNumberOfNodes, 1);
         forceRateCoefficientMatrix[i] = new DenseMatrix64F(defaultNumberOfNodes, defaultNumberOfDecisionVariables);
         forceRateBias[i] = new DenseMatrix64F(defaultNumberOfNodes, 1);
         optimizedForceValues[i] = new DenseMatrix64F(defaultNumberOfNodes, 1);
         optimizedForceRateValues[i] = new DenseMatrix64F(defaultNumberOfNodes, 1);
         decisionVariableValues[i] = new DenseMatrix64F(defaultNumberOfDecisionVariables, 1);
         decisionVariableWeightMatrix[i] = new DenseMatrix64F(defaultNumberOfDecisionVariables, defaultNumberOfDecisionVariables);
         decisionVariableDesiredValueMatrix[i] = new DenseMatrix64F(defaultNumberOfDecisionVariables, 1);
         H[i] = new DenseMatrix64F(defaultNumberOfDecisionVariables, defaultNumberOfDecisionVariables);
         f[i] = new DenseMatrix64F(defaultNumberOfDecisionVariables, 1);
         Aeq[i] = new DenseMatrix64F(defaultNumberOfNodes * 5, defaultNumberOfDecisionVariables);
         beq[i] = new DenseMatrix64F(defaultNumberOfNodes * 5, 1);
         decisionVariableLowerBoundMatrix[i] = new DenseMatrix64F(defaultNumberOfDecisionVariables, 1);
         decisionVariableUpperBoundMatrix[i] = new DenseMatrix64F(defaultNumberOfDecisionVariables, 1);
         Ain[i] = new DenseMatrix64F(defaultNumberOfNodes * 6, defaultNumberOfDecisionVariables);
         bin[i] = new DenseMatrix64F(defaultNumberOfNodes * 6, defaultNumberOfDecisionVariables);
      }
      this.forceRegularizationWeight = parameters.getdForceRegularizationWeight();
      this.forceRateRegularizationWeight = parameters.getdForceRegularizationWeight();
      parameters.getGravity(this.gravity);
      parameters.getMaxForce(this.maxForce);
      parameters.getMinForce(this.minForce);
      parameters.getMaxForceRate(this.maxForceRate);
      parameters.getMinForceRate(this.minForceRate);
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
         optimizedForceValues[i].reshape(0, 1);
         optimizedForceRateValues[i].reshape(0, 1);
         decisionVariableValues[i].reshape(0, 1);
         decisionVariableWeightMatrix[i].reshape(numberOfDecisionVariables[i], numberOfDecisionVariables[i]);
         decisionVariableDesiredValueMatrix[i].reshape(numberOfDecisionVariables[i], 1);
         H[i].reshape(numberOfDecisionVariables[i], numberOfDecisionVariables[i]);
         f[i].reshape(numberOfDecisionVariables[i], 1);
         Aeq[i].reshape(0, numberOfDecisionVariables[i]);
         beq[i].reshape(0, 1);
         decisionVariableLowerBoundMatrix[i].reshape(numberOfDecisionVariables[i], 1);
         decisionVariableUpperBoundMatrix[i].reshape(numberOfDecisionVariables[i], 1);
         Ain[i].reshape(0, numberOfDecisionVariables[i]);
         bin[i].reshape(0, 1);
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
         decisionVariableWeightMatrix[i].zero();
         decisionVariableDesiredValueMatrix[i].zero();
      }
   }

   private void setOptimizationMatricesToZero()
   {
      for (int i = 0; i < numberOfAxis; i++)
      {
         H[i].zero();
         f[i].zero();
         Aeq[i].zero();
         beq[i].zero();
         decisionVariableUpperBoundMatrix[i].zero();
         decisionVariableLowerBoundMatrix[i].zero();
         Ain[i].zero();
         bin[i].zero();
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
      CentroidalMotionNode node = entry.element;
      double previousNodeTime = node.getTime();
      for (Axis axis : Axis.values)
      {
         numberOfDecisionVariables[axis.ordinal()] = getNumberOfDecisionVariables(node, axis);
      }

      entry = entry.getNext();
      for (int i = 0; entry != null; i++, entry = entry.getNext())
      {
         node = entry.element;
         double nodeTime = node.getTime();
         deltaT.set(i, 0, nodeTime - previousNodeTime);
         previousNodeTime = nodeTime;
         for (Axis axis : Axis.values)
         {
            int axisOrdinal = axis.ordinal();
            numberOfDecisionVariables[axisOrdinal] += getNumberOfDecisionVariables(node, axis);
         }
      }
   }

   private int getNumberOfEqualityConstraints(CentroidalMotionNode node, Axis axis)
   {
      int numberOfEqualityConstraints = 0;
      if (node.getPositionConstraintType(axis) == DependentVariableConstraintType.EQUALITY)
         numberOfEqualityConstraints++;
      if (node.getLinearVelocityConstraintType(axis) == DependentVariableConstraintType.EQUALITY)
         numberOfEqualityConstraints++;
      //      if (node.getOrientationConstraintType(axis) == DependentVariableConstraintType.EQUALITY)
      //         numberOfEqualityConstraints++;
      //      if (node.getAngularVelocityConstraintType(axis) == DependentVariableConstraintType.EQUALITY)
      //         numberOfEqualityConstraints++;
      //      if (node.getTorqueConstraintType(axis) == DependentVariableConstraintType.EQUALITY)
      //         numberOfEqualityConstraints++;
      return numberOfEqualityConstraints;
   }

   private int getNumberOfDecisionVariables(CentroidalMotionNode node, Axis axis)
   {
      int numberOfDecisionVariables = 0;
      if (node.getForceConstraintType(axis) == EffortVariableConstraintType.OBJECTIVE)
         numberOfDecisionVariables++;
      if (node.getForceRateConstraintType(axis) == EffortVariableConstraintType.OBJECTIVE)
         numberOfDecisionVariables++;
      return numberOfDecisionVariables;
   }

   private void getForceVelocityPositionCoefficientBiasMatrices(RecycledLinkedListBuilder<CentroidalMotionNode> nodeList)
   {
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry;
      // Determine the position and velocity coefficient matrices given the decision variables
      shapeCoefficientMatrices();
      setCoefficientsToZero();
      setOptimizationMatricesToZero();
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
      double deltaT0 = deltaT.get(rowIndex, 0);
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
         DenseMatrix64F axisDecisionVariableUpperBoundMatrix = decisionVariableUpperBoundMatrix[axisOrdinal];
         DenseMatrix64F axisDecisionVariableLowerBoundMatrix = decisionVariableLowerBoundMatrix[axisOrdinal];

         double forceValue = node.getForce(axis);
         EffortVariableConstraintType forceConstraintType = node.getForceConstraintType(axis);
         double forceWeight = node.getForceWeight(axis);
         double velocityCoefficientForInitialForce = getVelocityCoefficientForInitialForce(deltaT0);
         double positionCoefficientForInitialForce = getPositionCoefficientForInitialForce(deltaT0);

         setInitialMatrixCoefficientsFromNodeQuantity(rowIndex, axisOrdinal, axisPositionCoefficientMatrix, axisPositionBiasMatrix,
                                                      axisVelocityCoefficientMatrix, axisVelocityBiasMatrix, axisForceCoefficientMatrix, axisForceBias,
                                                      axisDecisionVariableWeightMatrix, axisDecisionVariableDesiredValueMatrix,
                                                      axisDecisionVariableUpperBoundMatrix, axisDecisionVariableLowerBoundMatrix, forceValue,
                                                      maxForce.getElement(axisOrdinal), minForce.getElement(axisOrdinal), forceConstraintType, forceWeight,
                                                      forceRegularizationWeight, velocityCoefficientForInitialForce, positionCoefficientForInitialForce);
         double forceRateValue = node.getForceRate(axis);
         EffortVariableConstraintType forceRateConstraintType = node.getForceRateConstraintType(axis);
         double forceRateWeight = node.getForceRateWeight(axis);
         double velocityCoefficientForInitialForceRate = getVelocityCoefficientForInitialForceRate(deltaT0);
         double positionCoefficientForInitialForceRate = getPositionCoefficientForInitialForceRate(deltaT0);

         setInitialMatrixCoefficientsFromNodeQuantity(rowIndex, axisOrdinal, axisPositionCoefficientMatrix, axisPositionBiasMatrix,
                                                      axisVelocityCoefficientMatrix, axisVelocityBiasMatrix, axisForceRateCoefficientMatrix, axisForceRateBias,
                                                      axisDecisionVariableWeightMatrix, axisDecisionVariableDesiredValueMatrix,
                                                      axisDecisionVariableUpperBoundMatrix, axisDecisionVariableLowerBoundMatrix, forceRateValue,
                                                      maxForceRate.getElement(axisOrdinal), minForceRate.getElement(axisOrdinal), forceRateConstraintType,
                                                      forceRateWeight, forceRateRegularizationWeight, velocityCoefficientForInitialForceRate,
                                                      positionCoefficientForInitialForceRate);
         //         processConstraint(axisOrdinal, rowIndex, node.getPositionConstraintType(axis), axisPositionCoefficientMatrix, axisPositionBiasMatrix,
         //                           node.getPosition(axis), node.getPositionWeight(axis));
         //         processConstraint(axisOrdinal, rowIndex, node.getLinearVelocityConstraintType(axis), axisVelocityCoefficientMatrix, axisVelocityBiasMatrix,
         //                           node.getLinearVelocity(axis), node.getLinearVelocityWeight(axis));

      }

      rowIndex++;
      for (entry = entry.getNext(); entry.getNext() != null; entry = entry.getNext(), rowIndex++)
      {
         node = entry.element;
         double deltaTim1 = deltaT.get(rowIndex - 1, 0);
         double deltaTi = deltaT.get(rowIndex, 0);
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
            DenseMatrix64F axisDecisionVariableWeightMatrix = decisionVariableWeightMatrix[axisOrdinal];
            DenseMatrix64F axisDecisionVariableDesiredValueMatrix = decisionVariableDesiredValueMatrix[axisOrdinal];
            DenseMatrix64F axisDecisionVariableUpperBoundMatrix = decisionVariableUpperBoundMatrix[axisOrdinal];
            DenseMatrix64F axisDecisionVariableLowerBoundMatrix = decisionVariableLowerBoundMatrix[axisOrdinal];

            double forceValue = node.getForce(axis);
            EffortVariableConstraintType forceConstraintType = node.getForceConstraintType(axis);
            double forceWeight = node.getForceWeight(axis);
            double velocityCoefficientForFinalForce = getVelocityCoefficientForFinalForce(deltaTim1);
            double velocityCoefficientForInitialForce = getVelocityCoefficientForInitialForce(deltaTi);
            double positionCoefficientForFinalForce = getPositionCoefficientForFinalForce(deltaTim1);
            double positionCoefficientForInitialForce = getPositionCoefficientForInitialForce(deltaTi);

            setInitialFinalMatrixCoefficientsFromNodeQuanitity(rowIndex, axisOrdinal, axisPositionCoefficientMatrix, axisPositionBiasMatrix,
                                                               axisVelocityCoefficientMatrix, axisVelocityBiasMatrix, axisForceCoefficientMatrix, axisForceBias,
                                                               axisDecisionVariableWeightMatrix, axisDecisionVariableDesiredValueMatrix,
                                                               axisDecisionVariableUpperBoundMatrix, axisDecisionVariableLowerBoundMatrix, forceValue,
                                                               maxForce.getElement(axisOrdinal), minForce.getElement(axisOrdinal), forceConstraintType,
                                                               forceWeight, forceRegularizationWeight, velocityCoefficientForFinalForce,
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
                                                               axisForceRateBias, axisDecisionVariableWeightMatrix, axisDecisionVariableDesiredValueMatrix,
                                                               axisDecisionVariableUpperBoundMatrix, axisDecisionVariableLowerBoundMatrix, dForceValue,
                                                               maxForceRate.getElement(axisOrdinal), minForceRate.getElement(axisOrdinal), dForceConstraintType,
                                                               dForceWeight, forceRateRegularizationWeight, velocityCoefficientForFinalForceRate,
                                                               velocityCoefficientForInitialForceRate, positionCoefficientForFinalForceRate,
                                                               positionCoefficientForInitialForceRate);
            replaceInitialConditionPlaceholder(rowIndex, axisVelocityCoefficientMatrix);
            replaceInitialConditionPlaceholderAndAddConstantBias(rowIndex, axisVelocityBiasMatrix, deltaTim1 * gravity.getElement(axisOrdinal));
            replaceInitialConditionPlaceholder(rowIndex, axisPositionCoefficientMatrix);
            replaceInitialConditionPlaceholderAndAddConstantBias(rowIndex, axisPositionBiasMatrix,
                                                                 0.5 * deltaTim1 * deltaTim1 * gravity.getElement(axisOrdinal));
            addVelocityContributionToPosition(rowIndex, axisPositionCoefficientMatrix, axisVelocityCoefficientMatrix, deltaTim1);
            addVelocityContributionToPosition(rowIndex, axisPositionBiasMatrix, axisVelocityBiasMatrix, deltaTim1);

            processConstraint(axisOrdinal, rowIndex, node.getPositionConstraintType(axis), axisPositionCoefficientMatrix, axisPositionBiasMatrix,
                              node.getPosition(axis), node.getPositionMax(axis), node.getPositionMin(axis), node.getPositionWeight(axis));
            processConstraint(axisOrdinal, rowIndex, node.getLinearVelocityConstraintType(axis), axisVelocityCoefficientMatrix, axisVelocityBiasMatrix,
                              node.getLinearVelocity(axis), node.getLinearVelocityMax(axis), node.getLinearVelocityMin(axis),
                              node.getLinearVelocityWeight(axis));
         }
      }

      node = entry.element;
      double deltaTF = deltaT.get(rowIndex - 1, 0);
      for (Axis axis : Axis.values)
      {
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
         DenseMatrix64F axisDecisionVariableUpperBoundMatrix = decisionVariableUpperBoundMatrix[axisOrdinal];
         DenseMatrix64F axisDecisionVariableLowerBoundMatrix = decisionVariableLowerBoundMatrix[axisOrdinal];

         double forceValue = node.getForce(axis);
         EffortVariableConstraintType forceConstraintType = node.getForceConstraintType(axis);
         double forceWeight = node.getForceWeight(axis);
         double velocityCoefficientForFinalForce = getVelocityCoefficientForFinalForce(deltaTF);
         double positionCoefficientForFinalForce = getPositionCoefficientForFinalForce(deltaTF);
         setFinalMatrixCoefficientsFromNodeQuantity(rowIndex, axisOrdinal, axisVelocityCoefficientMatrix, axisVelocityBiasMatrix, axisPositionCoefficientMatrix,
                                                    axisPositionBiasMatrix, axisForceCoefficientMatrix, axisForceBias, axisDecisionVariableWeightMatrix,
                                                    axisDecisionVariableDesiredValueMatrix, axisDecisionVariableUpperBoundMatrix,
                                                    axisDecisionVariableLowerBoundMatrix, forceValue, maxForce.getElement(axisOrdinal),
                                                    minForce.getElement(axisOrdinal), forceConstraintType, forceWeight, forceRegularizationWeight,
                                                    velocityCoefficientForFinalForce, positionCoefficientForFinalForce);

         double dForceValue = node.getForceRate(axis);
         EffortVariableConstraintType forceRateConstraintType = node.getForceRateConstraintType(axis);
         double dForceWeight = node.getForceRateWeight(axis);
         double velocityCoefficientForFinalForceRate = getVelocityCoefficientForFinalForceRate(deltaTF);
         double positionCoefficientForFinalForceRate = getPositionCoefficientForFinalForceRate(deltaTF);
         setFinalMatrixCoefficientsFromNodeQuantity(rowIndex, axisOrdinal, axisVelocityCoefficientMatrix, axisVelocityBiasMatrix, axisPositionCoefficientMatrix,
                                                    axisPositionBiasMatrix, axisForceRateCoefficientMatrix, axisForceRateBias, axisDecisionVariableWeightMatrix,
                                                    axisDecisionVariableDesiredValueMatrix, axisDecisionVariableUpperBoundMatrix,
                                                    axisDecisionVariableLowerBoundMatrix, dForceValue, maxForceRate.getElement(axisOrdinal),
                                                    minForceRate.getElement(axisOrdinal), forceRateConstraintType, dForceWeight, forceRateRegularizationWeight,
                                                    velocityCoefficientForFinalForceRate, positionCoefficientForFinalForceRate);
         replaceInitialConditionPlaceholder(rowIndex, axisVelocityCoefficientMatrix);
         replaceInitialConditionPlaceholderAndAddConstantBias(rowIndex, axisVelocityBiasMatrix, deltaTF * gravity.getElement(axisOrdinal));
         replaceInitialConditionPlaceholder(rowIndex, axisPositionCoefficientMatrix);
         replaceInitialConditionPlaceholderAndAddConstantBias(rowIndex, axisPositionBiasMatrix, 0.5 * deltaTF * deltaTF * gravity.getElement(axisOrdinal));
         addVelocityContributionToPosition(rowIndex, axisPositionCoefficientMatrix, axisVelocityCoefficientMatrix, deltaTF);
         addVelocityContributionToPosition(rowIndex, axisPositionBiasMatrix, axisVelocityBiasMatrix, deltaTF);

         processConstraint(axisOrdinal, rowIndex, node.getPositionConstraintType(axis), axisPositionCoefficientMatrix, axisPositionBiasMatrix,
                           node.getPosition(axis), node.getPositionMax(axis), node.getPositionMin(axis), node.getPositionWeight(axis));
         processConstraint(axisOrdinal, rowIndex, node.getLinearVelocityConstraintType(axis), axisVelocityCoefficientMatrix, axisVelocityBiasMatrix,
                           node.getLinearVelocity(axis), node.getLinearVelocityMax(axis), node.getLinearVelocityMin(axis), node.getLinearVelocityWeight(axis));

         processForceWeights(axisOrdinal);
      }
   }

   private void processForceWeights(int axisOrdinal)
   {
      DenseMatrix64F axisDecisionVariableWeightMatrix = decisionVariableWeightMatrix[axisOrdinal];
      DenseMatrix64F axisDecisionVariableValueMatrix = decisionVariableDesiredValueMatrix[axisOrdinal];
      DenseMatrix64F axisH = H[axisOrdinal];
      DenseMatrix64F axisf = f[axisOrdinal];
      CommonOps.addEquals(axisH, axisDecisionVariableWeightMatrix);
      tempJ.set(axisDecisionVariableValueMatrix);
      tempf.reshape(numberOfDecisionVariables[axisOrdinal], 1);
      CommonOps.mult(axisDecisionVariableWeightMatrix, tempJ, tempf);
      CommonOps.scale(-1.0, tempf);
      CommonOps.addEquals(axisf, tempf);
   }

   private void processConstraint(int axisOrdinal, int rowIndex, DependentVariableConstraintType constraintType, DenseMatrix64F coefficientMatrix,
                                  DenseMatrix64F biasMatrix, double desiredValue, double upperBound, double lowerBound, double weight)
   {
      double bias1 = biasMatrix.get(rowIndex, 0);
      processDependentVariableConstraintToJacobianForm(rowIndex, coefficientMatrix, biasMatrix, desiredValue, extractedJacobian);
      switch (constraintType)
      {
      case IGNORE:
         return;
      case OBJECTIVE:
         if (Double.isFinite(weight) && !MathTools.epsilonEquals(weight, 0.0, Epsilons.ONE_MILLIONTH))
            addObjectiveCost(axisOrdinal, extractedJacobian, weight, desiredValue, bias1);
         if (Double.isFinite(upperBound))
            addLessThanConstraint(axisOrdinal, extractedJacobian, upperBound, bias1);
         if (Double.isFinite(lowerBound))
            addGreaterThanConstraint(axisOrdinal, extractedJacobian, lowerBound, bias1);
         return;
      case EQUALITY:
         addEqualityConstraint(axisOrdinal, extractedJacobian, desiredValue, bias1);
         return;
      default:
         throw new RuntimeException("Unhandled dependent variable constraint");
      }

   }

   private final DenseMatrix64F extractedJacobian = new DenseMatrix64F(0, 1);

   private void processDependentVariableConstraintToJacobianForm(int coefficientMatrixRowIndex, DenseMatrix64F coefficientMatrix, DenseMatrix64F biasMatrix,
                                                                 double desiredValue, DenseMatrix64F extractedJacobian)
   {
      extractedJacobian.reshape(1, coefficientMatrix.getNumCols());
      CommonOps.extractRow(coefficientMatrix, coefficientMatrixRowIndex, extractedJacobian);
   }

   private void addLessThanConstraint(int axisOrdinal, DenseMatrix64F jacobian, double upperBound, double bias)
   {
      insertIntoMatrix(Ain[axisOrdinal], bin[axisOrdinal], jacobian, upperBound - bias);
   }

   private void addGreaterThanConstraint(int axisOrdinal, DenseMatrix64F jacobian, double lowerBound, double bias)
   {
      CommonOps.scale(-1.0, jacobian);
      insertIntoMatrix(Ain[axisOrdinal], bin[axisOrdinal], jacobian, -lowerBound + bias);
   }

   private void addEqualityConstraint(int axisOrdinal, DenseMatrix64F jacobian, double desiredValue, double bias)
   {
      insertIntoMatrix(Aeq[axisOrdinal], beq[axisOrdinal], jacobian, desiredValue - bias);
   }

   private void insertIntoMatrix(DenseMatrix64F A, DenseMatrix64F b, DenseMatrix64F jacobian, double value)
   {
      // Maybe check here to ensure that jacobian is of row size 1
      int indexToInsertAt = A.getNumRows();
      int numberOfRowsToInsert = jacobian.getNumRows();
      int numberOfRows = indexToInsertAt + numberOfRowsToInsert;
      A.reshape(numberOfRows, A.getNumCols(), true);
      b.reshape(numberOfRows, 1, true);
      CommonOps.insert(jacobian, A, indexToInsertAt, 0);
      b.set(indexToInsertAt, 0, value);
   }

   private final DenseMatrix64F tempH = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempJtW = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempJ = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempf = new DenseMatrix64F(0, 1);

   private void addObjectiveCost(int axisOrdinal, DenseMatrix64F jacobian, double weight, double desiredValue, double bias)
   {
      DenseMatrix64F axisH = H[axisOrdinal];
      DenseMatrix64F axisf = f[axisOrdinal];

      tempJ.set(jacobian);
      tempJtW.reshape(jacobian.getNumCols(), jacobian.getNumRows());
      CommonOps.transpose(jacobian, tempJtW);
      CommonOps.scale(weight, tempJtW);
      tempH.reshape(jacobian.getNumCols(), jacobian.getNumCols());
      CommonOps.mult(tempJtW, tempJ, tempH);
      CommonOps.addEquals(axisH, tempH);

      tempf.reshape(jacobian.getNumCols(), jacobian.getNumRows());
      CommonOps.transpose(jacobian, tempf);
      CommonOps.scale(-weight * (desiredValue - bias), tempf);

      CommonOps.addEquals(axisf, tempf);
   }

   private void setFinalMatrixCoefficientsFromNodeQuantity(int rowIndex, int axisOrdinal, DenseMatrix64F axisVelocityCoefficientMatrix,
                                                           DenseMatrix64F axisVelocityBiasMatrix, DenseMatrix64F axisPositionCoefficientMatrix,
                                                           DenseMatrix64F axisPositionBiasMatrix, DenseMatrix64F axisDecisionVariableCoefficientMatrix,
                                                           DenseMatrix64F axisDecisionVariableBias, DenseMatrix64F axisDecisionVariableWeightMatrix,
                                                           DenseMatrix64F axisDecisionVariableDesiredValueMatrix,
                                                           DenseMatrix64F axisDecisionVariableUpperBoundMatrix,
                                                           DenseMatrix64F axisDecisionVariableLowerBoundMatrix, double decisionVariableValue,
                                                           double decisionVariableUpperBound, double decisionVariableLowerBound,
                                                           EffortVariableConstraintType decisionVariableConstraintType, double decisionVariableWeight,
                                                           double decisionVariableRegularizationWeight, double velocityCoefficient, double positionCoefficient)
   {
      {
         double axisVelocityBias = axisVelocityBiasMatrix.get(rowIndex, 0);
         double axisPositionBias = axisPositionBiasMatrix.get(rowIndex, 0);
         switch (decisionVariableConstraintType)
         {
         case OBJECTIVE:
            processDecisionVariableMatrices(decisionVariableIndex[axisOrdinal], axisDecisionVariableWeightMatrix, axisDecisionVariableDesiredValueMatrix,
                                            axisDecisionVariableUpperBoundMatrix, axisDecisionVariableLowerBoundMatrix, decisionVariableValue,
                                            decisionVariableUpperBound, decisionVariableLowerBound, decisionVariableWeight,
                                            decisionVariableRegularizationWeight);
            processCoefficientMatricesForFinalObjective(rowIndex, decisionVariableIndex[axisOrdinal], axisPositionCoefficientMatrix,
                                                        axisVelocityCoefficientMatrix, velocityCoefficient, positionCoefficient);
            axisDecisionVariableCoefficientMatrix.set(rowIndex, decisionVariableIndex[axisOrdinal], 1.0);
            decisionVariableIndex[axisOrdinal]++;
            break;
         case EQUALITY:
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
                                                             DenseMatrix64F axisDecisionVariableDesiredValueMatrix,
                                                             DenseMatrix64F axisDecisionVariableUpperBoundMatrix,
                                                             DenseMatrix64F axisDecisionVariableLowerBoundMatrix, double decisionVariableValue,
                                                             double decisionVariableLowerBound, double decisionVariableUpperBound,
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
            processDecisionVariableMatrices(decisionVariableIndex[axisOrdinal], axisDecisionVariableWeightMatrix, axisDecisionVariableDesiredValueMatrix,
                                            axisDecisionVariableUpperBoundMatrix, axisDecisionVariableLowerBoundMatrix, decisionVariableValue,
                                            decisionVariableUpperBound, decisionVariableLowerBound, decisionVariableWeight,
                                            decisionVariableRegularizationWeight);
            processCoefficientMatricesForInitialObjective(rowIndex, decisionVariableIndex[axisOrdinal], axisPositionCoefficientMatrix,
                                                          axisVelocityCoefficientMatrix, axisDecisionVariableCoefficientMatrix, velocityCoefficient,
                                                          positionCoefficient);
            decisionVariableIndex[axisOrdinal]++;
            break;
         case EQUALITY:
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
                                                                   DenseMatrix64F axisDecisionVariableDesiredValueMatrix,
                                                                   DenseMatrix64F axisDecisionVariableUpperBoundMatrix,
                                                                   DenseMatrix64F axisDecisionVariableLowerBoundMatrix, double decisionVariableValue,
                                                                   double decisionVariableUpperBound, double decisionVariableLowerBound,
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
         processDecisionVariableMatrices(decisionVariableIndex[axisOrdinal], axisDecisionVariableWeightMatrix, axisDecisionVariableDesiredValueMatrix,
                                         axisDecisionVariableUpperBoundMatrix, axisDecisionVariableLowerBoundMatrix, decisionVariableValue,
                                         decisionVariableUpperBound, decisionVariableLowerBound, decisionVariableWeight, decisionVariableRegularizationWeight);
         processCoefficientMatricesForFinalObjective(rowIndex, decisionVariableIndex[axisOrdinal], axisPositionCoefficientMatrix, axisVelocityCoefficientMatrix,
                                                     velocityCoefficientFinal, positionCoefficientFinal);
         processCoefficientMatricesForInitialObjective(rowIndex, decisionVariableIndex[axisOrdinal], axisPositionCoefficientMatrix,
                                                       axisVelocityCoefficientMatrix, axisDecisionVariableCoefficientMatrix, velocityCoefficientInitial,
                                                       positionCoefficientInitial);
         decisionVariableIndex[axisOrdinal]++;
         break;
      case EQUALITY:
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

   private void processDecisionVariableMatrices(int decisionVariableIndex, DenseMatrix64F axisDecisionVariableWeightMatrix,
                                                DenseMatrix64F axisDecisionVariableDesiredValueMatrix, DenseMatrix64F axisDecisionVariableUpperBoundMatrix,
                                                DenseMatrix64F axisDecisionVariableLowerBoundMatrix, double value, double upperBound, double lowerBound,
                                                double weight, double defaultWeight)
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
      axisDecisionVariableUpperBoundMatrix.set(decisionVariableIndex, 0, upperBound);
      axisDecisionVariableLowerBoundMatrix.set(decisionVariableIndex, 0, lowerBound);
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

   public DenseMatrix64F getObjectiveHMatrix(Axis axis)
   {
      return H[axis.ordinal()];
   }

   public DenseMatrix64F getObjectivefMatrix(Axis axis)
   {
      return f[axis.ordinal()];
   }

   public DenseMatrix64F getConstraintAeqMatrix(Axis axis)
   {
      return Aeq[axis.ordinal()];
   }

   public DenseMatrix64F getConstraintbeqMatrix(Axis axis)
   {
      return beq[axis.ordinal()];
   }

   public void setDecisionVariableValues(Axis axis, DenseMatrix64F solutionToSave)
   {
      DenseMatrix64F solution = decisionVariableValues[axis.ordinal()];
      solution.set(solutionToSave);
   }

   public DenseMatrix64F[] getOptimizedForceValues()
   {
      return optimizedForceValues;
   }

   public DenseMatrix64F[] getOptimizedForceRateValues()
   {
      return optimizedForceRateValues;
   }

   public int getNumberOfNodes()
   {
      return numberOfNodes;
   }

   public DenseMatrix64F getDecisionVariableUpperBoundMatrix(Axis axis)
   {
      return decisionVariableUpperBoundMatrix[axis.ordinal()];
   }

   public DenseMatrix64F getDecisionVariableLowerBoundMatrix(Axis axis)
   {
      return decisionVariableLowerBoundMatrix[axis.ordinal()];
   }

   public DenseMatrix64F getConstraintAinMatrix(Axis axis)
   {
      return Ain[axis.ordinal()];
   }

   public DenseMatrix64F getConstraintbinMatrix(Axis axis)
   {
      return bin[axis.ordinal()];
   }

   public DenseMatrix64F getOptimizedForceValues(Axis axis)
   {
      int axisOrdinal = axis.ordinal();
      compute(optimizedForceValues[axisOrdinal], decisionVariableValues[axisOrdinal], forceCoefficientMatrix[axisOrdinal], forceBias[axisOrdinal]);
      return optimizedForceValues[axisOrdinal];
   }

   private void compute(DenseMatrix64F result, DenseMatrix64F soln, DenseMatrix64F coefficients, DenseMatrix64F bias)
   {
      CommonOps.mult(coefficients, soln, result);
      CommonOps.addEquals(result, bias);
   }

   public DenseMatrix64F getOptimizedForceRateValues(Axis axis)
   {
      int axisOrdinal = axis.ordinal();
      compute(optimizedForceRateValues[axisOrdinal], decisionVariableValues[axisOrdinal], forceRateCoefficientMatrix[axisOrdinal], forceRateBias[axisOrdinal]);
      return optimizedForceRateValues[axisOrdinal];
   }

   public DenseMatrix64F getOptimizedVelocityValues(Axis axis)
   {
      int axisOrdinal = axis.ordinal();
      compute(optimizedVelocityValues[axisOrdinal], decisionVariableValues[axisOrdinal], velocityCoefficientMatrix[axisOrdinal], velocityBias[axisOrdinal]);
      return optimizedVelocityValues[axisOrdinal];
   }

   public DenseMatrix64F getOptimizedPositionValues(Axis axis)
   {
      int axisOrdinal = axis.ordinal();
      compute(optimizedPositionValues[axisOrdinal], decisionVariableValues[axisOrdinal], positionCoefficientMatrix[axisOrdinal], positionBias[axisOrdinal]);
      return optimizedPositionValues[axisOrdinal];
   }

}
