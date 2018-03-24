package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;

public class AngularControlModuleHelper
{
   private static final int numberOfTorqueCoefficients = 8;
   private static final int index_dz = 0;
   private static final int index_cz = 1;
   private static final int index_bz = 2;
   private static final int index_az = 3;

   private static final int defaultNumberOfNodes = LinearControlModuleHelper.defaultNumberOfNodes;

   private final DenseMatrix64F[] zForceCoefficient;
   private final DenseMatrix64F zVelocityValues;
   private final DenseMatrix64F zPositionValues;
   private final DenseMatrix64F[] xTorquePositionContributionCoefficientCoefficientMatrices;
   private final DenseMatrix64F[] xTorquePositionContributionCoefficientBiasMatrices;
   private final DenseMatrix64F[] yTorquePositionContributionCoefficientCoefficientMatrices;
   private final DenseMatrix64F[] yTorquePositionContributionCoefficientBiasMatrices;
   private final DenseMatrix64F[] xTorqueCoPContributionCoefficientCoefficientMatrices;
   private final DenseMatrix64F[] yTorqueCoPContributionCoefficientCoefficientMatrices;
   private final DenseMatrix64F xCoPSupportPolygonAinMatrix;
   private final DenseMatrix64F yCoPSupportPolygonAinMatrix;
   private final DenseMatrix64F[] copSupportPolygonAeqMatrix = new DenseMatrix64F[LinearControlModuleHelper.numberOfAngularAxis];
   private final DenseMatrix64F[] copSupportPolygonbeqMatrix = new DenseMatrix64F[LinearControlModuleHelper.numberOfAngularAxis];
   private final DenseMatrix64F copSupportPolygonbinMatrix;

   private final double robotMass;

   public AngularControlModuleHelper(CentroidalMotionPlannerParameters parameters)
   {
      this.robotMass = parameters.getRobotMass();
      zForceCoefficient = new DenseMatrix64F[index_az + 1];
      zVelocityValues = new DenseMatrix64F(defaultNumberOfNodes - 1, 1);
      zPositionValues = new DenseMatrix64F(defaultNumberOfNodes - 1, 1);

      for (int i = 0; i < 4; i++)
         zForceCoefficient[i] = new DenseMatrix64F(defaultNumberOfNodes - 1, 1);

      xTorquePositionContributionCoefficientCoefficientMatrices = new DenseMatrix64F[numberOfTorqueCoefficients];
      yTorquePositionContributionCoefficientCoefficientMatrices = new DenseMatrix64F[numberOfTorqueCoefficients];
      xTorquePositionContributionCoefficientBiasMatrices = new DenseMatrix64F[numberOfTorqueCoefficients];
      yTorquePositionContributionCoefficientBiasMatrices = new DenseMatrix64F[numberOfTorqueCoefficients];
      xTorqueCoPContributionCoefficientCoefficientMatrices = new DenseMatrix64F[numberOfTorqueCoefficients];
      yTorqueCoPContributionCoefficientCoefficientMatrices = new DenseMatrix64F[numberOfTorqueCoefficients];
      xCoPSupportPolygonAinMatrix = new DenseMatrix64F(0, 1);
      yCoPSupportPolygonAinMatrix = new DenseMatrix64F(0, 1);
      copSupportPolygonbinMatrix = new DenseMatrix64F(0, 1);

      for (Axis axis : LinearControlModuleHelper.angularAxisValues)
      {
         copSupportPolygonAeqMatrix[axis.ordinal()] = new DenseMatrix64F(defaultNumberOfNodes, defaultNumberOfNodes);
         copSupportPolygonbeqMatrix[axis.ordinal()] = new DenseMatrix64F(defaultNumberOfNodes, 1);
      }

      for (int i = 0; i < numberOfTorqueCoefficients; i++)
      {
         xTorquePositionContributionCoefficientCoefficientMatrices[i] = new DenseMatrix64F(defaultNumberOfNodes, defaultNumberOfNodes * 2);
         yTorquePositionContributionCoefficientCoefficientMatrices[i] = new DenseMatrix64F(defaultNumberOfNodes, defaultNumberOfNodes * 2);
         xTorquePositionContributionCoefficientBiasMatrices[i] = new DenseMatrix64F(defaultNumberOfNodes, 1);
         yTorquePositionContributionCoefficientBiasMatrices[i] = new DenseMatrix64F(defaultNumberOfNodes, 1);
         xTorqueCoPContributionCoefficientCoefficientMatrices[i] = new DenseMatrix64F(defaultNumberOfNodes, defaultNumberOfNodes);
         yTorqueCoPContributionCoefficientCoefficientMatrices[i] = new DenseMatrix64F(defaultNumberOfNodes, defaultNumberOfNodes);
      }
   }

   public void reset()
   {

   }

   public void setZForceValues(DenseMatrix64F forceValues, DenseMatrix64F forceRateValues, DenseMatrix64F velocityValues, DenseMatrix64F positionValues,
                               DenseMatrix64F deltaT)
   {
      int numberOfNodes = forceValues.getNumRows();
      for (int i = 0; i <= index_az; i++)
         zForceCoefficient[i].reshape(numberOfNodes - 1, 1);
      for (int i = 0; i < numberOfNodes - 1; i++)
      {
         double f0 = forceValues.get(i, 0);
         double f1 = forceValues.get(i + 1, 0);
         double m0 = forceRateValues.get(i, 0) * deltaT.get(i, 0);
         double m1 = forceRateValues.get(i + 1, 0) * deltaT.get(i, 0);
         zForceCoefficient[index_dz].set(i, 0, f0);
         zForceCoefficient[index_cz].set(i, 0, m0);
         zForceCoefficient[index_bz].set(i, 0, 3.0 * (f1 - f0) - 2.0 * m0 - m1);
         zForceCoefficient[index_az].set(i, 0, -2.0 * (f1 - f0) + m0 + m1);
      }
      zVelocityValues.set(velocityValues);
      zPositionValues.set(positionValues);
   }

   private final DenseMatrix64F tempMatrixForCoefficients = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempForceCoefficient1 = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempForceRateCoefficient1 = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempForceCoefficient2 = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempForceRateCoefficient2 = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempVelocityCoefficient = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempPositionCoefficient = new DenseMatrix64F(0, 1);

   public void computeYTorqueCoefficientsInTermsOfXDecisionVariables(DenseMatrix64F xPositionCoefficients, DenseMatrix64F xPositionBias,
                                                                     DenseMatrix64F xVelocityCoefficients, DenseMatrix64F xVelocityBias,
                                                                     DenseMatrix64F xForceCoefficients, DenseMatrix64F xForceBias,
                                                                     DenseMatrix64F xForceRateCoefficients, DenseMatrix64F xForceRateBias,
                                                                     DenseMatrix64F deltaT)
   {
      computeTorqueContributionFromCentroidalTrajectoryAssumingSignConventionsForXAxis(xPositionCoefficients, xPositionBias, xVelocityCoefficients,
                                                                                       xVelocityBias, xForceCoefficients, xForceBias, xForceRateCoefficients,
                                                                                       xForceRateBias, deltaT,
                                                                                       yTorquePositionContributionCoefficientCoefficientMatrices,
                                                                                       yTorquePositionContributionCoefficientBiasMatrices);
      for (int i = 0; i < 8; i++)
      {
         CommonOps.scale(-1.0, yTorquePositionContributionCoefficientCoefficientMatrices[i]);
         CommonOps.scale(-1.0, yTorquePositionContributionCoefficientBiasMatrices[i]);
      }
   }

   public void computeXTorqueCoefficientsInTermsOfYDecisionVariables(DenseMatrix64F yPositionCoefficients, DenseMatrix64F yPositionBias,
                                                                     DenseMatrix64F yVelocityCoefficients, DenseMatrix64F yVelocityBias,
                                                                     DenseMatrix64F yForceCoefficients, DenseMatrix64F yForceBias,
                                                                     DenseMatrix64F yForceRateCoefficients, DenseMatrix64F yForceRateBias,
                                                                     DenseMatrix64F deltaT)
   {
      computeTorqueContributionFromCentroidalTrajectoryAssumingSignConventionsForXAxis(yPositionCoefficients, yPositionBias, yVelocityCoefficients,
                                                                                       yVelocityBias, yForceCoefficients, yForceBias, yForceRateCoefficients,
                                                                                       yForceRateBias, deltaT,
                                                                                       xTorquePositionContributionCoefficientCoefficientMatrices,
                                                                                       xTorquePositionContributionCoefficientBiasMatrices);
   }

   public void computeXTorqueContributionFromCoP(int numberOfNodes)
   {
      computeTorqueContributionFromCoPTrajectoryAssumingSignConventionForXAxis(numberOfNodes, xTorqueCoPContributionCoefficientCoefficientMatrices);
   }

   public void computeYTorqueContributionFromCoP(int numberOfNodes)
   {
      computeTorqueContributionFromCoPTrajectoryAssumingSignConventionForXAxis(numberOfNodes, yTorqueCoPContributionCoefficientCoefficientMatrices);
      for (int i = 0; i < 7; i++)
         CommonOps.scale(-1.0, yTorqueCoPContributionCoefficientCoefficientMatrices[i]);
   }

   private final FrameConvexPolygon2d tempSupportPolygon = new FrameConvexPolygon2d();

   public void computeCoPPointConstraints(RecycledLinkedListBuilder<CentroidalMotionNode> nodeList,
                                          RecycledLinkedListBuilder<CentroidalMotionSupportPolygon> supportPolygonList)
   {
      int numberOfNodes = nodeList.getSize();
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> nodeEntry = nodeList.getFirstEntry();
      RecycledLinkedListBuilder<CentroidalMotionSupportPolygon>.RecycledLinkedListEntry<CentroidalMotionSupportPolygon> supportPolygonEntry = supportPolygonList.getFirstEntry();
      double nodeTime;
      for (int nodeIndex = 0; nodeIndex < numberOfNodes; nodeIndex++, nodeEntry = nodeEntry.getNext())
      {
         nodeTime = nodeEntry.element.getTime();
         for (supportPolygonEntry = supportPolygonList.getFirstEntry(); supportPolygonEntry != null; supportPolygonEntry = supportPolygonEntry.getNext())
         {
            if (nodeTime < supportPolygonEntry.element.getStartTime())
               continue;
            else if (nodeTime > supportPolygonEntry.element.getEndTime())
               supportPolygonEntry = supportPolygonEntry.getNext();
            else
            {
               supportPolygonEntry.element.getSupportPolygon(tempSupportPolygon);
               setCoPConstraintsForSupportPolygon(numberOfNodes, nodeIndex, tempSupportPolygon);
            }
         }
      }
   }

   private void setCoPConstraintsForSupportPolygon(int numberOfNodes, int i, FrameConvexPolygon2d supportPolygon)
   {
      int numberOfVertices = supportPolygon.getNumberOfVertices();
      if (numberOfVertices == 0)
         return;
      int existingNumberOfConstraints = xCoPSupportPolygonAinMatrix.getNumRows();
      xCoPSupportPolygonAinMatrix.reshape(existingNumberOfConstraints + numberOfVertices, numberOfNodes, true);
      yCoPSupportPolygonAinMatrix.reshape(existingNumberOfConstraints + numberOfVertices, numberOfNodes, true);
      copSupportPolygonbinMatrix.reshape(existingNumberOfConstraints + numberOfVertices, 1, true);
      for (int j = 0; j < numberOfVertices - 1; j++)
         setCoPPositionConstraint(numberOfNodes, i, existingNumberOfConstraints, j, supportPolygon.getVertex(j), supportPolygon.getVertex(j + 1));
      setCoPPositionConstraint(numberOfNodes, i, existingNumberOfConstraints, numberOfVertices - 1, supportPolygon.getVertex(numberOfVertices - 1),
                               supportPolygon.getVertex(0));
   }

   private void setCoPPositionConstraint(int numberOfNodes, int columnIndex, int existingNumberOfConstraints, int rowIndex, Point2DReadOnly vertex1,
                                         Point2DReadOnly vertex2)
   {
      double deltaX = vertex2.getX() - vertex1.getX();
      double deltaY = vertex2.getY() - vertex1.getY();
      xCoPSupportPolygonAinMatrix.set(existingNumberOfConstraints + rowIndex, columnIndex, -deltaY);
      yCoPSupportPolygonAinMatrix.set(existingNumberOfConstraints + rowIndex, columnIndex, deltaX);
      copSupportPolygonbinMatrix.set(existingNumberOfConstraints + rowIndex, -deltaY * vertex1.getX() + deltaX * vertex1.getY());
   }

   private void computeTorqueContributionFromCoPTrajectoryAssumingSignConventionForXAxis(int numberOfNodes, DenseMatrix64F[] torqueCoefficientCoefficientMatrix)
   {
      int numberOfDecisionVariables = numberOfNodes;
      for (int i = 0; i < 8; i++)
      {
         torqueCoefficientCoefficientMatrix[i].reshape(numberOfNodes - 1, numberOfDecisionVariables);
         torqueCoefficientCoefficientMatrix[i].zero();
      }
      for (int i = 0; i < numberOfNodes - 1; i++)
      {
         double az = zForceCoefficient[index_az].get(i);
         double bz = zForceCoefficient[index_bz].get(i);
         double cz = zForceCoefficient[index_cz].get(i);
         double dz = zForceCoefficient[index_dz].get(i);

         setCoPCoefficientT6(torqueCoefficientCoefficientMatrix[6], i, az, bz, cz, dz);
         setCoPCoefficientT5(torqueCoefficientCoefficientMatrix[5], i, az, bz, cz, dz);
         setCoPCoefficientT4(torqueCoefficientCoefficientMatrix[4], i, az, bz, cz, dz);
         setCoPCoefficientT3(torqueCoefficientCoefficientMatrix[3], i, az, bz, cz, dz);
         setCoPCoefficientT2(torqueCoefficientCoefficientMatrix[2], i, az, bz, cz, dz);
         setCoPCoefficientT1(torqueCoefficientCoefficientMatrix[1], i, az, bz, cz, dz);
         setCoPCoefficientT0(torqueCoefficientCoefficientMatrix[0], i, az, bz, cz, dz);
      }
   }

   public void setCoPEqualityConstraints(RecycledLinkedListBuilder<CentroidalMotionNode> nodeList)
   {
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry = nodeList.getFirstEntry();
      int nodeIndex = 0;
      for (Axis axis : LinearControlModuleHelper.angularAxisValues)
      {
         copSupportPolygonAeqMatrix[axis.ordinal()].reshape(0, nodeList.getSize());
         copSupportPolygonbeqMatrix[axis.ordinal()].reshape(0, 1);
      }
      for (; entry != null; nodeIndex++, entry = entry.getNext())
      {
         CentroidalMotionNode node = entry.element;
         for (Axis axis : LinearControlModuleHelper.angularAxisValues)
         {
            int axisOrdinal = axis.ordinal();
            EffortVariableConstraintType constraintType = node.getCoPConstraintType(axis);
            DenseMatrix64F axisCoPAeqMatrix = copSupportPolygonAeqMatrix[axisOrdinal];
            DenseMatrix64F axisCoPbeqMatrix = copSupportPolygonbeqMatrix[axisOrdinal];
            if(constraintType == EffortVariableConstraintType.EQUALITY)
               constraintCoPValue(nodeIndex, node.getCoPElement(axis), axisCoPAeqMatrix, axisCoPbeqMatrix);
         }
      }
   }

   private void constraintCoPValue(int nodeIndex, double copValue, DenseMatrix64F axisCoPAeqMatrix, DenseMatrix64F axisCoPbeqMatrix)
   {
      int indexToInsertConstraintAt = axisCoPAeqMatrix.getNumRows();
      axisCoPAeqMatrix.reshape(indexToInsertConstraintAt + 1, axisCoPAeqMatrix.numCols, true);
      axisCoPbeqMatrix.reshape(indexToInsertConstraintAt + 1, 1, true);
      tempMatrixForCoefficients.reshape(1, axisCoPAeqMatrix.numCols);
      tempMatrixForCoefficients.zero();
      tempMatrixForCoefficients.set(indexToInsertConstraintAt, nodeIndex, 1.0);
      CommonOps.insert(tempMatrixForCoefficients, axisCoPAeqMatrix, indexToInsertConstraintAt, 0);
      axisCoPbeqMatrix.set(indexToInsertConstraintAt, 0, copValue);
   }

   public void getCoPLocationConstraints(DenseMatrix64F Aeq, DenseMatrix64F beq)
   {
      int xForceVariables = xTorquePositionContributionCoefficientCoefficientMatrices[0].getNumCols();
      int xCoPVariables = xTorqueCoPContributionCoefficientCoefficientMatrices[0].getNumCols();
      int yForceVariables = yTorquePositionContributionCoefficientCoefficientMatrices[0].getNumCols();
      int yCoPVariables = yTorqueCoPContributionCoefficientCoefficientMatrices[0].getNumCols();
      int xVariables = xForceVariables + xCoPVariables;
      int yVariables = yForceVariables + yCoPVariables;
      
      DenseMatrix64F xCoPAeqMatrix = copSupportPolygonAeqMatrix[Axis.X.ordinal()];
      DenseMatrix64F yCoPAeqMatrix = copSupportPolygonAeqMatrix[Axis.Y.ordinal()];
      DenseMatrix64F xCoPbeqMatrix = copSupportPolygonbeqMatrix[Axis.X.ordinal()];
      DenseMatrix64F yCoPbeqMatrix = copSupportPolygonbeqMatrix[Axis.Y.ordinal()];
      int numberOfXEqualityConstraints = xCoPAeqMatrix.getNumRows();
      int numberOfYEqualityConstraints = yCoPAeqMatrix.getNumRows();
      
      Aeq.reshape(numberOfXEqualityConstraints + numberOfYEqualityConstraints, xVariables + yVariables);
      Aeq.zero();
      beq.reshape(numberOfXEqualityConstraints + numberOfYEqualityConstraints, 1);
      beq.zero();
      CommonOps.insert(xCoPAeqMatrix, Aeq, 0, xForceVariables);
      CommonOps.insert(xCoPbeqMatrix, beq, 0, 0);
      CommonOps.insert(yCoPAeqMatrix, Aeq, numberOfXEqualityConstraints, xVariables + yForceVariables);
      CommonOps.insert(yCoPbeqMatrix, beq, numberOfXEqualityConstraints, 0);
   }

   public void setCoPCoefficientT6(DenseMatrix64F coefficientMatrixToSet, int rowIndex, double az, double bz, double cz, double dz)
   {
      coefficientMatrixToSet.set(rowIndex, rowIndex, 2 * az);
      coefficientMatrixToSet.set(rowIndex, rowIndex + 1. - 2 * az);
   }

   public void setCoPCoefficientT5(DenseMatrix64F coefficientMatrixToSet, int rowIndex, double az, double bz, double cz, double dz)
   {
      double value = -3.0 * az + 2 * bz;
      coefficientMatrixToSet.set(rowIndex, rowIndex, value);
      coefficientMatrixToSet.set(rowIndex, rowIndex + 1. - value);
   }

   public void setCoPCoefficientT4(DenseMatrix64F coefficientMatrixToSet, int rowIndex, double az, double bz, double cz, double dz)
   {
      double value = -3.0 * bz + 2 * cz;
      coefficientMatrixToSet.set(rowIndex, rowIndex, value);
      coefficientMatrixToSet.set(rowIndex, rowIndex + 1. - value);
   }

   public void setCoPCoefficientT3(DenseMatrix64F coefficientMatrixToSet, int rowIndex, double az, double bz, double cz, double dz)
   {
      coefficientMatrixToSet.set(rowIndex, rowIndex, az - 3.0 * cz);
      coefficientMatrixToSet.set(rowIndex, rowIndex + 1. + 3 * cz);
   }

   public void setCoPCoefficientT2(DenseMatrix64F coefficientMatrixToSet, int rowIndex, double az, double bz, double cz, double dz)
   {
      coefficientMatrixToSet.set(rowIndex, rowIndex, bz - 3.0 * dz);
      coefficientMatrixToSet.set(rowIndex, rowIndex + 1. + 3.0 * dz);
   }

   public void setCoPCoefficientT1(DenseMatrix64F coefficientMatrixToSet, int rowIndex, double az, double bz, double cz, double dz)
   {
      coefficientMatrixToSet.set(rowIndex, rowIndex, cz);
   }

   public void setCoPCoefficientT0(DenseMatrix64F coefficientMatrixToSet, int rowIndex, double az, double bz, double cz, double dz)
   {
      coefficientMatrixToSet.set(rowIndex, rowIndex, dz);
   }

   private void computeTorqueContributionFromCentroidalTrajectoryAssumingSignConventionsForXAxis(DenseMatrix64F positionCoefficients,
                                                                                                 DenseMatrix64F positionBias,
                                                                                                 DenseMatrix64F velocityCoefficients,
                                                                                                 DenseMatrix64F velocityBias, DenseMatrix64F forceCoefficients,
                                                                                                 DenseMatrix64F forceBias, DenseMatrix64F forceRateCoefficients,
                                                                                                 DenseMatrix64F forceRateBias, DenseMatrix64F deltaT,
                                                                                                 DenseMatrix64F[] torqueCoefficientCoefficientMatrix,
                                                                                                 DenseMatrix64F[] torqueCoefficientBiasMatrix)
   {
      int numberOfNodes = positionCoefficients.getNumRows();
      int numberOfDecisionVariables = positionCoefficients.getNumCols();
      tempMatrixForCoefficients.reshape(1, numberOfDecisionVariables);
      for (int i = 0; i < 8; i++)
      {
         torqueCoefficientCoefficientMatrix[i].reshape(numberOfNodes - 1, numberOfDecisionVariables);
         torqueCoefficientBiasMatrix[i].reshape(numberOfNodes - 1, 1);
      }
      for (int i = 0; i < numberOfNodes - 1; i++)
      {
         double az = zForceCoefficient[index_az].get(i);
         double bz = zForceCoefficient[index_bz].get(i);
         double cz = zForceCoefficient[index_cz].get(i);
         double dz = zForceCoefficient[index_dz].get(i);
         double vz = zVelocityValues.get(i);
         double pz = zPositionValues.get(i);

         tempForceCoefficient1.reshape(1, numberOfDecisionVariables);
         tempForceRateCoefficient1.reshape(1, numberOfDecisionVariables);
         tempForceCoefficient2.reshape(1, numberOfDecisionVariables);
         tempForceRateCoefficient2.reshape(1, numberOfDecisionVariables);
         tempVelocityCoefficient.reshape(1, numberOfDecisionVariables);
         tempPositionCoefficient.reshape(1, numberOfDecisionVariables);

         double deltaTi = deltaT.get(i);
         CommonOps.extract(forceCoefficients, i, i + 1, 0, numberOfDecisionVariables, tempForceCoefficient1, 0, 0);
         CommonOps.extract(forceRateCoefficients, i, i + 1, 0, numberOfDecisionVariables, tempForceRateCoefficient1, 0, 0);
         CommonOps.extract(forceCoefficients, i + 1, i + 2, 0, numberOfDecisionVariables, tempForceCoefficient2, 0, 0);
         CommonOps.extract(forceRateCoefficients, i + 1, i + 2, 0, numberOfDecisionVariables, tempForceRateCoefficient2, 0, 0);
         CommonOps.extract(velocityCoefficients, i, i + 1, 0, numberOfDecisionVariables, tempVelocityCoefficient, 0, 0);
         CommonOps.extract(positionCoefficients, i, i + 1, 0, numberOfDecisionVariables, tempPositionCoefficient, 0, 0);

         setCoefficientT7(torqueCoefficientCoefficientMatrix[7], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT6(torqueCoefficientCoefficientMatrix[6], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT5(torqueCoefficientCoefficientMatrix[5], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT4(torqueCoefficientCoefficientMatrix[4], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT3(torqueCoefficientCoefficientMatrix[3], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT2(torqueCoefficientCoefficientMatrix[2], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT1(torqueCoefficientCoefficientMatrix[1], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT0(torqueCoefficientCoefficientMatrix[0], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);

         tempForceCoefficient1.reshape(1, 1);
         tempForceRateCoefficient1.reshape(1, 1);
         tempForceCoefficient2.reshape(1, 1);
         tempForceRateCoefficient2.reshape(1, 1);
         tempVelocityCoefficient.reshape(1, 1);
         tempPositionCoefficient.reshape(1, 1);

         CommonOps.extract(forceBias, i, i + 1, 0, 1, tempForceCoefficient1, 0, 0);
         CommonOps.extract(forceRateBias, i, i + 1, 0, 1, tempForceRateCoefficient1, 0, 0);
         CommonOps.extract(forceBias, i + 1, i + 2, 0, 1, tempForceCoefficient2, 0, 0);
         CommonOps.extract(forceRateBias, i + 1, i + 2, 0, 1, tempForceRateCoefficient2, 0, 0);
         CommonOps.extract(velocityBias, i, i + 1, 0, 1, tempVelocityCoefficient, 0, 0);
         CommonOps.extract(positionBias, i, i + 1, 0, 1, tempPositionCoefficient, 0, 0);

         setCoefficientT7(torqueCoefficientBiasMatrix[7], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT6(torqueCoefficientBiasMatrix[6], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT5(torqueCoefficientBiasMatrix[5], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT4(torqueCoefficientBiasMatrix[4], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT3(torqueCoefficientBiasMatrix[3], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT2(torqueCoefficientBiasMatrix[2], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT1(torqueCoefficientBiasMatrix[1], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT0(torqueCoefficientBiasMatrix[0], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
      }
   }

   private void setCoefficientT0(DenseMatrix64F coefficientMatrixToSet, int rowIndex, double az, double bz, double cz, double dz, double vz, double pz,
                                 double detlaTi, DenseMatrix64F f0, DenseMatrix64F f1, DenseMatrix64F m0, DenseMatrix64F m1, DenseMatrix64F v0,
                                 DenseMatrix64F p0)
   {
      tempMatrixForCoefficients.reshape(1, f0.getNumCols());
      tempMatrixForCoefficients.zero();
      CommonOps.addEquals(tempMatrixForCoefficients, (pz), f0);
      CommonOps.addEquals(tempMatrixForCoefficients, -dz, p0);
      CommonOps.insert(tempMatrixForCoefficients, coefficientMatrixToSet, rowIndex, 0);
   }

   private void setCoefficientT1(DenseMatrix64F coefficientMatrixToSet, int rowIndex, double az, double bz, double cz, double dz, double vz, double pz,
                                 double deltaTi, DenseMatrix64F f0, DenseMatrix64F f1, DenseMatrix64F m0, DenseMatrix64F m1, DenseMatrix64F v0,
                                 DenseMatrix64F p0)
   {
      tempMatrixForCoefficients.reshape(1, f0.getNumCols());
      tempMatrixForCoefficients.zero();
      CommonOps.addEquals(tempMatrixForCoefficients, (vz * deltaTi), f0);
      CommonOps.addEquals(tempMatrixForCoefficients, (pz) * deltaTi, m0);
      CommonOps.addEquals(tempMatrixForCoefficients, -dz * deltaTi, v0);
      CommonOps.addEquals(tempMatrixForCoefficients, -cz, p0);
      CommonOps.insert(tempMatrixForCoefficients, coefficientMatrixToSet, rowIndex, 0);
   }

   private void setCoefficientT2(DenseMatrix64F coefficientMatrixToSet, int rowIndex, double az, double bz, double cz, double dz, double vz, double pz,
                                 double deltaTi, DenseMatrix64F f0, DenseMatrix64F f1, DenseMatrix64F m0, DenseMatrix64F m1, DenseMatrix64F v0,
                                 DenseMatrix64F p0)
   {
      tempMatrixForCoefficients.reshape(1, f0.getNumCols());
      tempMatrixForCoefficients.zero();
      CommonOps.addEquals(tempMatrixForCoefficients, (-3.0 * pz), f0);
      CommonOps.addEquals(tempMatrixForCoefficients, (3.0 * pz), f1);
      CommonOps.addEquals(tempMatrixForCoefficients, (-2.0 * pz + deltaTi * vz) * deltaTi, m0);
      CommonOps.addEquals(tempMatrixForCoefficients, (-pz) * deltaTi, m1);
      CommonOps.addEquals(tempMatrixForCoefficients, -cz * deltaTi, v0);
      CommonOps.addEquals(tempMatrixForCoefficients, -bz, p0);
      CommonOps.insert(tempMatrixForCoefficients, coefficientMatrixToSet, rowIndex, 0);
   }

   private void setCoefficientT3(DenseMatrix64F coefficientMatrixToSet, int rowIndex, double az, double bz, double cz, double dz, double vz, double pz,
                                 double deltaTi, DenseMatrix64F f0, DenseMatrix64F f1, DenseMatrix64F m0, DenseMatrix64F m1, DenseMatrix64F v0,
                                 DenseMatrix64F p0)
   {
      tempMatrixForCoefficients.reshape(1, f0.getNumCols());
      tempMatrixForCoefficients.zero();
      CommonOps.addEquals(tempMatrixForCoefficients, (2 * pz - 3 * deltaTi * vz - deltaTi * deltaTi * cz / (3.0 * robotMass)), f0);
      CommonOps.addEquals(tempMatrixForCoefficients, (-2.0 * pz + 3 * deltaTi * vz), f1);
      CommonOps.addEquals(tempMatrixForCoefficients, (pz - 2.0 * vz * deltaTi - (dz * deltaTi * deltaTi) / (3.0 * robotMass)) * deltaTi, m0);
      CommonOps.addEquals(tempMatrixForCoefficients, (pz - deltaTi * vz) * deltaTi, m1);
      CommonOps.addEquals(tempMatrixForCoefficients, -bz * deltaTi, v0);
      CommonOps.addEquals(tempMatrixForCoefficients, -az, p0);
      CommonOps.insert(tempMatrixForCoefficients, coefficientMatrixToSet, rowIndex, 0);
   }

   private void setCoefficientT4(DenseMatrix64F coefficientMatrixToSet, int rowIndex, double az, double bz, double cz, double dz, double vz, double pz,
                                 double deltaTi, DenseMatrix64F f0, DenseMatrix64F f1, DenseMatrix64F m0, DenseMatrix64F m1, DenseMatrix64F v0,
                                 DenseMatrix64F p0)
   {
      tempMatrixForCoefficients.reshape(1, f0.getNumCols());
      tempMatrixForCoefficients.zero();
      CommonOps.addEquals(tempMatrixForCoefficients,
                          (2.0 * deltaTi * vz - 5.0 * bz * deltaTi * deltaTi / (12.0 * robotMass) - 15 * dz * deltaTi * deltaTi / (12.0 * robotMass)), f0);
      CommonOps.addEquals(tempMatrixForCoefficients, (-2.0 * deltaTi * vz + 15 * dz * deltaTi * deltaTi / (12.0 * robotMass)), f1);
      CommonOps.addEquals(tempMatrixForCoefficients, (1.0 * vz - 10.0 * dz * deltaTi / (12.0 * robotMass)) * deltaTi * deltaTi, m0);
      CommonOps.addEquals(tempMatrixForCoefficients, (1.0 * vz - 5.0 * dz * deltaTi / (12.0 * robotMass)) * deltaTi * deltaTi, m1);
      CommonOps.addEquals(tempMatrixForCoefficients, -az * deltaTi, v0);
      CommonOps.insert(tempMatrixForCoefficients, coefficientMatrixToSet, rowIndex, 0);
   }

   private void setCoefficientT5(DenseMatrix64F coefficientMatrixToSet, int rowIndex, double az, double bz, double cz, double dz, double vz, double pz,
                                 double deltaTi, DenseMatrix64F f0, DenseMatrix64F f1, DenseMatrix64F m0, DenseMatrix64F m1, DenseMatrix64F v0,
                                 DenseMatrix64F p0)
   {
      double timeMultiplier = deltaTi * deltaTi / (60.0 * robotMass);
      tempMatrixForCoefficients.reshape(1, f0.getNumCols());
      tempMatrixForCoefficients.zero();
      CommonOps.addEquals(tempMatrixForCoefficients, (-27 * az + 54.0 * dz - 15 * cz) * timeMultiplier, f0);
      CommonOps.addEquals(tempMatrixForCoefficients, (-54 * dz + 15 * cz) * timeMultiplier, f1);
      CommonOps.addEquals(tempMatrixForCoefficients, (27.0 * dz - 10.0 * cz - 5.0 * bz) * timeMultiplier * deltaTi, m0);
      CommonOps.addEquals(tempMatrixForCoefficients, (27.0 * dz - 5 * cz) * timeMultiplier * deltaTi, m1);
      CommonOps.insert(tempMatrixForCoefficients, coefficientMatrixToSet, rowIndex, 0);
   }

   private void setCoefficientT6(DenseMatrix64F coefficientMatrixToSet, int rowIndex, double az, double bz, double cz, double dz, double vz, double pz,
                                 double deltaTi, DenseMatrix64F f0, DenseMatrix64F f1, DenseMatrix64F m0, DenseMatrix64F m1, DenseMatrix64F v0,
                                 DenseMatrix64F p0)
   {
      double timeMultiplier = deltaTi * deltaTi * 7.0 / (60.0 * robotMass);
      tempMatrixForCoefficients.reshape(1, f0.getNumCols());
      tempMatrixForCoefficients.zero();
      CommonOps.addEquals(tempMatrixForCoefficients, 2.0 * cz * timeMultiplier, f0);
      CommonOps.addEquals(tempMatrixForCoefficients, -2.0 * cz * timeMultiplier, f1);
      CommonOps.addEquals(tempMatrixForCoefficients, (1.0 * cz - 1.0 * az) * timeMultiplier * deltaTi, m0);
      CommonOps.addEquals(tempMatrixForCoefficients, 1.0 * cz * timeMultiplier * deltaTi, m1);
      CommonOps.insert(tempMatrixForCoefficients, coefficientMatrixToSet, rowIndex, 0);
   }

   private void setCoefficientT7(DenseMatrix64F coefficientMatrixToSet, int rowIndex, double az, double bz, double cz, double dz, double vz, double pz,
                                 double deltaTi, DenseMatrix64F f0, DenseMatrix64F f1, DenseMatrix64F m0, DenseMatrix64F m1, DenseMatrix64F v0,
                                 DenseMatrix64F p0)
   {
      double timeMultiplier = deltaTi * deltaTi / (30.0 * robotMass);
      tempMatrixForCoefficients.reshape(1, f0.getNumCols());
      tempMatrixForCoefficients.zero();
      CommonOps.addEquals(tempMatrixForCoefficients, (2.0 * bz + 3.0 * az) * timeMultiplier, f0);
      CommonOps.addEquals(tempMatrixForCoefficients, (-2.0 * bz - 3.0 * az) * timeMultiplier, f1);
      CommonOps.addEquals(tempMatrixForCoefficients, (1.0 * bz + 2.0 * az) * timeMultiplier * deltaTi, m0);
      CommonOps.addEquals(tempMatrixForCoefficients, (1.0 * bz + 1.0 * az) * timeMultiplier * deltaTi, m1);
      CommonOps.insert(tempMatrixForCoefficients, coefficientMatrixToSet, rowIndex, 0);
   }

   public DenseMatrix64F getCoPSupportPolygonConstraintbinMatrix()
   {
      return copSupportPolygonbinMatrix;
   }

   public void getConsolidatedCoPSupportPolygonConstraints(DenseMatrix64F coefficientMatrixToSet, DenseMatrix64F biasMatrixToSet)
   {
      int numberOfXDecisionVariables = xTorquePositionContributionCoefficientCoefficientMatrices[0].getNumCols()
            + xTorqueCoPContributionCoefficientCoefficientMatrices[0].getNumCols();
      int numberOfYDecisionVariables = yTorquePositionContributionCoefficientCoefficientMatrices[0].getNumCols()
            + yTorqueCoPContributionCoefficientCoefficientMatrices[0].getNumCols();
      int numberOfVariables = numberOfXDecisionVariables + numberOfYDecisionVariables;
      coefficientMatrixToSet.reshape(xCoPSupportPolygonAinMatrix.getNumRows(), numberOfVariables);
      coefficientMatrixToSet.zero();
      int xCoPIndex = xTorquePositionContributionCoefficientCoefficientMatrices[0].getNumCols();
      int yCoPIndex = numberOfXDecisionVariables + yTorquePositionContributionCoefficientCoefficientMatrices[0].getNumCols();
      CommonOps.insert(xCoPSupportPolygonAinMatrix, coefficientMatrixToSet, 0, xCoPIndex);
      CommonOps.insert(yCoPSupportPolygonAinMatrix, coefficientMatrixToSet, 0, yCoPIndex);
      biasMatrixToSet.set(copSupportPolygonbinMatrix);
   }

   public void getConsolidatedTorqueConstraints(DenseMatrix64F coefficientMatrixToSet, DenseMatrix64F biasMatrixToSet)
   {
      int numberOfConstraints = 0;
      int numberOfXDecisionVariables = xTorquePositionContributionCoefficientCoefficientMatrices[0].getNumCols()
            + xTorqueCoPContributionCoefficientCoefficientMatrices[0].getNumCols();
      int numberOfYDecisionVariables = yTorquePositionContributionCoefficientCoefficientMatrices[0].getNumCols()
            + yTorqueCoPContributionCoefficientCoefficientMatrices[0].getNumCols();
      int numberOfVariables = numberOfXDecisionVariables + numberOfYDecisionVariables;
      for (int i = 0; i < 7; i++)
      {
         DenseMatrix64F positionCoefficientMatrix = xTorquePositionContributionCoefficientCoefficientMatrices[i];
         DenseMatrix64F copCoefficientMatrix = yTorqueCoPContributionCoefficientCoefficientMatrices[i];
         consolidateTorqueCoefficientMatrix(positionCoefficientMatrix, copCoefficientMatrix, tempMatrixForCoefficients);
         coefficientMatrixToSet.reshape(numberOfConstraints + positionCoefficientMatrix.getNumRows(), numberOfVariables, true);
         CommonOps.insert(tempMatrixForCoefficients, coefficientMatrixToSet, numberOfConstraints, 0);

         DenseMatrix64F positionBiasMatrix = xTorquePositionContributionCoefficientBiasMatrices[i];
         //DenseMatrix64F copBiasMatrix = xTorqueCoPContributionCoefficientBiasMatrices[i];
         biasMatrixToSet.reshape(numberOfConstraints + positionCoefficientMatrix.getNumRows(), 1, true);
         CommonOps.insert(positionBiasMatrix, biasMatrixToSet, numberOfConstraints, 0);
         numberOfConstraints += positionCoefficientMatrix.getNumRows();
      }
      {
         DenseMatrix64F positionCoefficientMatrix = xTorquePositionContributionCoefficientCoefficientMatrices[7];
         coefficientMatrixToSet.reshape(numberOfConstraints + positionCoefficientMatrix.getNumRows(), numberOfVariables, true);
         CommonOps.insert(positionCoefficientMatrix, coefficientMatrixToSet, numberOfConstraints, 0);

         DenseMatrix64F positionBiasMatrix = xTorquePositionContributionCoefficientBiasMatrices[7];
         biasMatrixToSet.reshape(numberOfConstraints + positionCoefficientMatrix.getNumRows(), 1, true);
         CommonOps.insert(positionBiasMatrix, biasMatrixToSet, numberOfConstraints, 0);
         numberOfConstraints += positionCoefficientMatrix.getNumRows();
      }
      for (int i = 0; i < 7; i++)
      {
         DenseMatrix64F positionCoefficientMatrix = yTorquePositionContributionCoefficientCoefficientMatrices[i];
         DenseMatrix64F copCoefficientMatrix = xTorqueCoPContributionCoefficientCoefficientMatrices[i];
         consolidateTorqueCoefficientMatrix(positionCoefficientMatrix, copCoefficientMatrix, tempMatrixForCoefficients);
         coefficientMatrixToSet.reshape(numberOfConstraints + positionCoefficientMatrix.getNumRows(), numberOfVariables, true);
         CommonOps.insert(tempMatrixForCoefficients, coefficientMatrixToSet, numberOfConstraints, numberOfXDecisionVariables);

         DenseMatrix64F positionBiasMatrix = yTorquePositionContributionCoefficientBiasMatrices[i];
         //DenseMatrix64F copBiasMatrix = yTorqueCoPContributionCoefficientBiasMatrices[i];
         biasMatrixToSet.reshape(numberOfConstraints + positionCoefficientMatrix.getNumRows(), 1, true);
         CommonOps.insert(positionBiasMatrix, biasMatrixToSet, numberOfConstraints, 0);
         numberOfConstraints += positionCoefficientMatrix.getNumRows();
      }
      {
         DenseMatrix64F positionCoefficientMatrix = yTorquePositionContributionCoefficientCoefficientMatrices[7];
         coefficientMatrixToSet.reshape(numberOfConstraints + positionCoefficientMatrix.getNumRows(), numberOfVariables, true);
         CommonOps.insert(positionCoefficientMatrix, coefficientMatrixToSet, numberOfConstraints, numberOfXDecisionVariables);

         DenseMatrix64F positionBiasMatrix = yTorquePositionContributionCoefficientBiasMatrices[7];
         biasMatrixToSet.reshape(numberOfConstraints + positionCoefficientMatrix.getNumRows(), 1, true);
         CommonOps.insert(positionBiasMatrix, biasMatrixToSet, numberOfConstraints, 0);
         numberOfConstraints += positionCoefficientMatrix.getNumRows();

      }
   }

   private void consolidateTorqueCoefficientMatrix(DenseMatrix64F positionCoefficientMatrix, DenseMatrix64F copCoefficientMatrix, DenseMatrix64F matrixToSet)
   {
      int numberOfForceVariables = positionCoefficientMatrix.getNumCols();
      int numberOfCoPVariables = copCoefficientMatrix.getNumCols();

      if (positionCoefficientMatrix.getNumRows() != copCoefficientMatrix.getNumRows())
         throw new RuntimeException("Cannot combine the decision variable and CoP torque contributions due to number of rows mismatch");

      matrixToSet.reshape(positionCoefficientMatrix.getNumRows(), numberOfForceVariables + numberOfCoPVariables, true);
      CommonOps.insert(positionCoefficientMatrix, matrixToSet, 0, 0);
      CommonOps.insert(copCoefficientMatrix, matrixToSet, 0, numberOfForceVariables);
   }

   public DenseMatrix64F getXCoPSupportPolygonConstraintAinMatrix()
   {
      return xCoPSupportPolygonAinMatrix;
   }

   public DenseMatrix64F getYCoPSupportPolygonConstraintAinMatrix()
   {
      return yCoPSupportPolygonAinMatrix;
   }

   public void processLinearObjectives(DenseMatrix64F Hx, DenseMatrix64F fx, DenseMatrix64F Hy, DenseMatrix64F fy, DenseMatrix64F H, DenseMatrix64F f)
   {
      int numberOfXDecisionVariables = xTorquePositionContributionCoefficientCoefficientMatrices[0].getNumCols()
            + xTorqueCoPContributionCoefficientCoefficientMatrices[0].getNumCols();
      int numberOfYDecisionVariables = yTorquePositionContributionCoefficientCoefficientMatrices[0].getNumCols()
            + yTorqueCoPContributionCoefficientCoefficientMatrices[0].getNumCols();
      int numberOfVariables = numberOfXDecisionVariables + numberOfYDecisionVariables;

      H.reshape(numberOfVariables, numberOfVariables);
      f.reshape(numberOfVariables, 1);
      H.zero();
      f.zero();

      CommonOps.insert(Hx, H, 0, 0);
      CommonOps.insert(Hy, H, numberOfXDecisionVariables, numberOfXDecisionVariables);

      CommonOps.insert(fx, f, 0, 0);
      CommonOps.insert(fy, f, numberOfXDecisionVariables, 0);
   }

   public void processLinearConstraints(DenseMatrix64F Ax, DenseMatrix64F bx, DenseMatrix64F Ay, DenseMatrix64F by, DenseMatrix64F consolidatedA,
                                        DenseMatrix64F consolidatedB)
   {
      int numberOfXDecisionVariables = xTorquePositionContributionCoefficientCoefficientMatrices[0].getNumCols()
            + xTorqueCoPContributionCoefficientCoefficientMatrices[0].getNumCols();
      int numberOfYDecisionVariables = yTorquePositionContributionCoefficientCoefficientMatrices[0].getNumCols()
            + yTorqueCoPContributionCoefficientCoefficientMatrices[0].getNumCols();
      int numberOfVariables = numberOfXDecisionVariables + numberOfYDecisionVariables;
      int totalNumberOfConstraints = Ax.getNumRows() + Ay.getNumRows();

      consolidatedA.reshape(totalNumberOfConstraints, numberOfVariables);
      consolidatedA.zero();
      CommonOps.insert(Ax, consolidatedA, 0, 0);
      CommonOps.insert(Ay, consolidatedA, Ax.getNumRows(), numberOfXDecisionVariables);

      consolidatedB.reshape(totalNumberOfConstraints, 1);
      consolidatedB.zero();
      CommonOps.insert(bx, consolidatedB, 0, 0);
      CommonOps.insert(by, consolidatedB, bx.getNumRows(), 0);
   }

   private final DenseMatrix64F yetAnotherTempMatrix = new DenseMatrix64F(0, 1);

   public void processLinearBounds(DenseMatrix64F xLowerBounds, DenseMatrix64F xUpperBounds, DenseMatrix64F yLowerBounds, DenseMatrix64F yUpperBounds,
                                   DenseMatrix64F A, DenseMatrix64F b)
   {
      int numberOfXDecisionVariables = xTorquePositionContributionCoefficientCoefficientMatrices[0].getNumCols()
            + xTorqueCoPContributionCoefficientCoefficientMatrices[0].getNumCols();
      int numberOfYDecisionVariables = yTorquePositionContributionCoefficientCoefficientMatrices[0].getNumCols()
            + yTorqueCoPContributionCoefficientCoefficientMatrices[0].getNumCols();
      int numberOfVariables = numberOfXDecisionVariables + numberOfYDecisionVariables;
      int numberOfXConstraints = xLowerBounds.getNumRows() + xUpperBounds.getNumRows();
      int numberOfYConstraints = yLowerBounds.getNumRows() + yUpperBounds.getNumRows();
      int numberOfConstraints = numberOfXConstraints + numberOfYConstraints;
      A.reshape(numberOfConstraints, numberOfVariables);
      b.reshape(numberOfConstraints, 1);
      A.zero();
      b.zero();
      yetAnotherTempMatrix.reshape(xUpperBounds.getNumRows(), xUpperBounds.getNumRows());
      CommonOps.setIdentity(yetAnotherTempMatrix);
      CommonOps.insert(yetAnotherTempMatrix, A, 0, 0);
      CommonOps.insert(xUpperBounds, b, 0, 0);

      CommonOps.scale(-1.0, yetAnotherTempMatrix);
      CommonOps.insert(yetAnotherTempMatrix, A, xUpperBounds.getNumRows(), 0);
      CommonOps.scale(-1.0, xLowerBounds);
      CommonOps.insert(xLowerBounds, b, xUpperBounds.getNumRows(), 0);

      yetAnotherTempMatrix.reshape(yUpperBounds.getNumRows(), yUpperBounds.getNumRows());
      CommonOps.setIdentity(yetAnotherTempMatrix);
      CommonOps.insert(yetAnotherTempMatrix, A, numberOfXConstraints, numberOfXDecisionVariables);
      CommonOps.insert(yUpperBounds, b, numberOfXConstraints, 0);

      CommonOps.scale(-1.0, yetAnotherTempMatrix);
      CommonOps.insert(yetAnotherTempMatrix, A, numberOfXConstraints + yUpperBounds.getNumRows(), numberOfXDecisionVariables);
      CommonOps.scale(-1.0, yLowerBounds);
      CommonOps.insert(yLowerBounds, b, numberOfXConstraints + yUpperBounds.getNumRows(), 0);
   }

   public void processQPSolution(DenseMatrix64F qpSolution, DenseMatrix64F xQPSolution, DenseMatrix64F yQPSolution, DenseMatrix64F xCoPSolution,
                                 DenseMatrix64F yCoPSolution)
   {
      int xForceVariables = xTorquePositionContributionCoefficientCoefficientMatrices[0].getNumCols();
      int xCoPVariables = xTorqueCoPContributionCoefficientCoefficientMatrices[0].getNumCols();
      int yForceVariables = yTorquePositionContributionCoefficientCoefficientMatrices[0].getNumCols();
      int yCoPVariables = yTorqueCoPContributionCoefficientCoefficientMatrices[0].getNumCols();
      int xVariables = xForceVariables + xCoPVariables;
      int yVariables = yForceVariables + yCoPVariables;
      xQPSolution.reshape(xForceVariables, 1);
      yQPSolution.reshape(yForceVariables, 1);
      xCoPSolution.reshape(xCoPVariables, 1);
      yCoPSolution.reshape(yCoPVariables, 1);

      CommonOps.extract(qpSolution, 0, xForceVariables, 0, 1, xQPSolution, 0, 0);
      CommonOps.extract(qpSolution, xForceVariables, xVariables, 0, 1, xCoPSolution, 0, 0);
      CommonOps.extract(qpSolution, xVariables, xVariables + yForceVariables, 0, 1, yQPSolution, 0, 0);
      CommonOps.extract(qpSolution, xVariables + yForceVariables, xVariables + yVariables, 0, 1, yCoPSolution, 0, 0);
   }

   public void getCoPRegularization(DenseMatrix64F regularizationH, double regularizationWeight)
   {
      int xForceVariables = xTorquePositionContributionCoefficientCoefficientMatrices[0].getNumCols();
      int xCoPVariables = xTorqueCoPContributionCoefficientCoefficientMatrices[0].getNumCols();
      int yForceVariables = yTorquePositionContributionCoefficientCoefficientMatrices[0].getNumCols();
      int yCoPVariables = yTorqueCoPContributionCoefficientCoefficientMatrices[0].getNumCols();
      int xVariables = xForceVariables + xCoPVariables;
      int yVariables = yForceVariables + yCoPVariables;

      int numberOfVariables = xVariables + yVariables;
      regularizationH.reshape(numberOfVariables, numberOfVariables);
      regularizationH.zero();
      tempMatrixForCoefficients.reshape(xCoPVariables, xCoPVariables);
      CommonOps.setIdentity(tempMatrixForCoefficients);
      CommonOps.scale(regularizationWeight, tempMatrixForCoefficients);
      CommonOps.insert(tempMatrixForCoefficients, regularizationH, xForceVariables, xForceVariables);
      tempMatrixForCoefficients.reshape(yCoPVariables, yCoPVariables);
      CommonOps.setIdentity(tempMatrixForCoefficients);
      CommonOps.scale(regularizationWeight, tempMatrixForCoefficients);
      CommonOps.insert(tempMatrixForCoefficients, regularizationH, xVariables + yForceVariables, xVariables + yForceVariables);
   }

   //   private void consolidateTorqueBiasMatrix(DenseMatrix64F forceBiasMatrix, DenseMatrix64F copBiasMatrix, DenseMatrix64F matrixToSet)
   //   {
   //      if (forceBiasMatrix.getNumRows() != copBiasMatrix.getNumRows())
   //         throw new RuntimeException("Cannot combine the decision variable and CoP torque contributions due to number of rows mismatch");
   //
   //      matrixToSet.reshape(forceBiasMatrix.getNumRows(), 1);
   //      matrixToSet.set(forceBiasMatrix);
   //      CommonOps.addEquals(matrixToSet, copBiasMatrix);
   //   }
}