package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

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
   private final DenseMatrix64F[] xTorqueCoefficientCoefficientMatrices;
   private final DenseMatrix64F[] xTorqueCoefficientBiasMatrices;
   private final DenseMatrix64F[] yTorqueCoefficientCoefficientMatrices;
   private final DenseMatrix64F[] yTorqueCoefficientBiasMatrices;
   private final double robotMass = 18.0;

   public AngularControlModuleHelper()
   {
      zForceCoefficient = new DenseMatrix64F[index_az + 1];
      zVelocityValues = new DenseMatrix64F(defaultNumberOfNodes - 1, 1);
      zPositionValues = new DenseMatrix64F(defaultNumberOfNodes - 1, 1);

      for (int i = 0; i < 4; i++)
         zForceCoefficient[i] = new DenseMatrix64F(defaultNumberOfNodes - 1, 1);

      xTorqueCoefficientCoefficientMatrices = new DenseMatrix64F[numberOfTorqueCoefficients];
      yTorqueCoefficientCoefficientMatrices = new DenseMatrix64F[numberOfTorqueCoefficients];
      xTorqueCoefficientBiasMatrices = new DenseMatrix64F[numberOfTorqueCoefficients];
      yTorqueCoefficientBiasMatrices = new DenseMatrix64F[numberOfTorqueCoefficients];
      for (int i = 0; i < numberOfTorqueCoefficients; i++)
      {
         xTorqueCoefficientCoefficientMatrices[i] = new DenseMatrix64F(defaultNumberOfNodes, defaultNumberOfNodes * 2);
         yTorqueCoefficientCoefficientMatrices[i] = new DenseMatrix64F(defaultNumberOfNodes, defaultNumberOfNodes * 2);
         xTorqueCoefficientBiasMatrices[i] = new DenseMatrix64F(defaultNumberOfNodes, 1);
         yTorqueCoefficientBiasMatrices[i] = new DenseMatrix64F(defaultNumberOfNodes, 1);
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

   public void computeXTorqueCoefficientsInTermsOfYDecisionVariables(DenseMatrix64F yPositionCoefficients, DenseMatrix64F yPositionBias,
                                                                     DenseMatrix64F yVelocityCoefficients, DenseMatrix64F yVelocityBias,
                                                                     DenseMatrix64F yForceCoefficients, DenseMatrix64F yForceBias,
                                                                     DenseMatrix64F yForceRateCoefficients, DenseMatrix64F yForceRateBias,
                                                                     DenseMatrix64F deltaT)
   {
      int numberOfNodes = yPositionCoefficients.getNumRows();
      int numberOfDecisionVariables = yPositionCoefficients.getNumCols();
      tempMatrixForCoefficients.reshape(1, numberOfDecisionVariables);
      for (int i = 0; i < 8; i++)
      {
         xTorqueCoefficientCoefficientMatrices[i].reshape(numberOfNodes - 1, numberOfDecisionVariables);
         xTorqueCoefficientBiasMatrices[i].reshape(numberOfNodes - 1, 1);
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
         CommonOps.extract(yForceCoefficients, i, i + 1, 0, numberOfDecisionVariables, tempForceCoefficient1, 0, 0);
         CommonOps.extract(yForceRateCoefficients, i, i + 1, 0, numberOfDecisionVariables, tempForceRateCoefficient1, 0, 0);
         CommonOps.extract(yForceCoefficients, i + 1, i + 2, 0, numberOfDecisionVariables, tempForceCoefficient2, 0, 0);
         CommonOps.extract(yForceRateCoefficients, i + 1, i + 2, 0, numberOfDecisionVariables, tempForceRateCoefficient2, 0, 0);
         CommonOps.extract(yVelocityCoefficients, i, i + 1, 0, numberOfDecisionVariables, tempVelocityCoefficient, 0, 0);
         CommonOps.extract(yPositionCoefficients, i, i + 1, 0, numberOfDecisionVariables, tempPositionCoefficient, 0, 0);

         setCoefficientT7(xTorqueCoefficientCoefficientMatrices[7], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT6(xTorqueCoefficientCoefficientMatrices[6], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT5(xTorqueCoefficientCoefficientMatrices[5], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT4(xTorqueCoefficientCoefficientMatrices[4], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT3(xTorqueCoefficientCoefficientMatrices[3], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT2(xTorqueCoefficientCoefficientMatrices[2], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT1(xTorqueCoefficientCoefficientMatrices[1], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT0(xTorqueCoefficientCoefficientMatrices[0], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);

         tempForceCoefficient1.reshape(1, 1);
         tempForceRateCoefficient1.reshape(1, 1);
         tempForceCoefficient2.reshape(1, 1);
         tempForceRateCoefficient2.reshape(1, 1);
         tempVelocityCoefficient.reshape(1, 1);
         tempPositionCoefficient.reshape(1, 1);

         CommonOps.extract(yForceBias, i, i + 1, 0, 1, tempForceCoefficient1, 0, 0);
         CommonOps.extract(yForceRateBias, i, i + 1, 0, 1, tempForceRateCoefficient1, 0, 0);
         CommonOps.extract(yForceBias, i + 1, i + 2, 0, 1, tempForceCoefficient2, 0, 0);
         CommonOps.extract(yForceRateBias, i + 1, i + 2, 0, 1, tempForceRateCoefficient2, 0, 0);
         CommonOps.extract(yVelocityBias, i, i + 1, 0, 1, tempVelocityCoefficient, 0, 0);
         CommonOps.extract(yPositionBias, i, i + 1, 0, 1, tempPositionCoefficient, 0, 0);

         setCoefficientT7(xTorqueCoefficientBiasMatrices[7], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT6(xTorqueCoefficientBiasMatrices[6], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT5(xTorqueCoefficientBiasMatrices[5], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT4(xTorqueCoefficientBiasMatrices[4], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT3(xTorqueCoefficientBiasMatrices[3], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT2(xTorqueCoefficientBiasMatrices[2], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT1(xTorqueCoefficientBiasMatrices[1], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);
         setCoefficientT0(xTorqueCoefficientBiasMatrices[0], i, az, bz, cz, dz, vz, pz, deltaTi, tempForceCoefficient1, tempForceCoefficient2,
                          tempForceRateCoefficient1, tempForceRateCoefficient2, tempVelocityCoefficient, tempPositionCoefficient);

      }
   }

   private void setCoefficientT0(DenseMatrix64F coefficientMatrixToSet, int rowIndex, double az, double bz, double cz, double dz, double vz, double pz,
                                 double detlaTi, DenseMatrix64F f0, DenseMatrix64F f1, DenseMatrix64F m0, DenseMatrix64F m1, DenseMatrix64F v0,
                                 DenseMatrix64F p0)
   {
      tempMatrixForCoefficients.zero();
      CommonOps.addEquals(tempMatrixForCoefficients, (pz), f0);
      CommonOps.addEquals(tempMatrixForCoefficients, -dz, p0);
      CommonOps.insert(tempMatrixForCoefficients, coefficientMatrixToSet, rowIndex, 0);
   }

   private void setCoefficientT1(DenseMatrix64F coefficientMatrixToSet, int rowIndex, double az, double bz, double cz, double dz, double vz, double pz,
                                 double deltaTi, DenseMatrix64F f0, DenseMatrix64F f1, DenseMatrix64F m0, DenseMatrix64F m1, DenseMatrix64F v0,
                                 DenseMatrix64F p0)
   {
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
      tempMatrixForCoefficients.zero();
      CommonOps.addEquals(tempMatrixForCoefficients, (2.0 * bz + 3.0 * az) * timeMultiplier, f0);
      CommonOps.addEquals(tempMatrixForCoefficients, (-2.0 * bz - 3.0 * az) * timeMultiplier, f1);
      CommonOps.addEquals(tempMatrixForCoefficients, (1.0 * bz + 2.0 * az) * timeMultiplier * deltaTi, m0);
      CommonOps.addEquals(tempMatrixForCoefficients, (1.0 * bz + 1.0 * az) * timeMultiplier * deltaTi, m1);
      CommonOps.insert(tempMatrixForCoefficients, coefficientMatrixToSet, rowIndex, 0);
   }
}