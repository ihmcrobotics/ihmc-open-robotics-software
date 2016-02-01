package us.ihmc.robotics.math.trajectories;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public class YoPolynomial
{
   private final int numberOfCoefficients;
   private double pos, vel, acc;
   private final DoubleYoVariable[] a;
   private final DenseMatrix64F constraintMatrix;
   private final DenseMatrix64F constraintVector;
   private final DenseMatrix64F coefficientVector;
   private final double[] xPowers;

   private final LinearSolver<DenseMatrix64F> solver;

   public YoPolynomial(String name, int numberOfCoefficients, YoVariableRegistry registry)
   {
      this.numberOfCoefficients = numberOfCoefficients;

      solver = LinearSolverFactory.general(numberOfCoefficients, numberOfCoefficients);

      a = new DoubleYoVariable[numberOfCoefficients];
      constraintMatrix = new DenseMatrix64F(numberOfCoefficients, numberOfCoefficients);
      constraintVector = new DenseMatrix64F(numberOfCoefficients, 1);
      coefficientVector = new DenseMatrix64F(numberOfCoefficients, 1);
      xPowers = new double[numberOfCoefficients];
      
      for (int i = 0; i < numberOfCoefficients; i++)
      {
         a[i] = new DoubleYoVariable(name + "_a" + i, registry);
      }
   }

   public double getPosition()
   {
      return pos;
   }

   public double getVelocity()
   {
      return vel;
   }
   
   public double getAcceleration()
   {
      return acc;
   }
   
   public double getCoefficient(int i)
   {
      return a[i].getDoubleValue();
   }
   
   public double[] getCoefficients()
   {
      double[] ret = new double[numberOfCoefficients];
      for (int i = 0; i < numberOfCoefficients; i++)
      {
         ret[i] = a[i].getDoubleValue();
      }
      return ret;
   }
   

   public void setConstant(double z)
   {
      coefficientVector.set(0, 0, z);
      for (int i = 1; i < numberOfCoefficients; i++)
         coefficientVector.set(i, 0, 0.0);
      setYoVariables();
   }
   
   public void setLinear(double t0, double tFinal, double z0, double zf)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 2);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tFinal, zf);
      solveForCoefficients();
      setYoVariables();
   }

   public void setLinear(double t, double z, double zd)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 2);
      setPositionRow(0, t, z);
      setVelocityRow(1, t, zd);
      solveForCoefficients();
      setYoVariables();
   }

   public void setQuintic(double t0, double tFinal, double z0, double zd0, double zdd0, double zf, double zdf, double zddf)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 6);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tFinal, zf);
      setVelocityRow(4, tFinal, zdf);
      setAccelerationRow(5, tFinal, zddf);
      solveForCoefficients();
      setYoVariables();
   }
   
   public void setQuinticUsingWayPoint(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdd0, double zIntermediate, double zf, double zdf)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 6);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tIntermediate, zIntermediate);
      setPositionRow(4, tFinal, zf);
      setVelocityRow(5, tFinal, zdf);
      solveForCoefficients();
      setYoVariables();
   }

   public void setQuinticUsingWayPoint2(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdd0, double zIntermediate, double zdIntermediate, double zf)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 6);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tIntermediate, zIntermediate);
      setVelocityRow(4, tIntermediate, zdIntermediate);
      setPositionRow(5, tFinal, zf);
      solveForCoefficients();
      setYoVariables();
   }

   public void setQuinticTwoWaypoints(double t0, double tIntermediate0, double tIntermediate1, double tFinal, double z0, double zd0, double zIntermediate0, double zIntermediate1, double zf, double zdf)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 6);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tIntermediate0, zIntermediate0);
      setPositionRow(3, tIntermediate1, zIntermediate1);
      setPositionRow(4, tFinal, zf);
      setVelocityRow(5, tFinal, zdf);
      solveForCoefficients();
      setYoVariables();
   }
   
   public void setQuinticUsingIntermediateVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdIntermediate, double zddIntermediate, double zFinal, double zdFinal)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 6);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setVelocityRow(2, tIntermediate, zdIntermediate);
      setAccelerationRow(3, tIntermediate, zddIntermediate);
      setPositionRow(4, tFinal, zFinal);
      setVelocityRow(5, tFinal, zdFinal);
      solveForCoefficients();
      setYoVariables();
   }
   
   public void setSexticUsingWaypoint(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdd0, double zIntermediate, double zf, double zdf, double zddf)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 7);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tIntermediate, zIntermediate);
      setPositionRow(4, tFinal, zf);
      setVelocityRow(5, tFinal, zdf);
      setAccelerationRow(6, tFinal, zddf);
      solveForCoefficients();
      setYoVariables();
   }

   public void setSeptic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, 
         double z0, double zd0, double zIntermediate0, double zdIntermediate0, double zIntermediate1, double zdIntermediate1, double zf, double zdf)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 8);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tIntermediate0, zIntermediate0);
      setVelocityRow(3, tIntermediate0, zdIntermediate0);
      setPositionRow(4, tIntermediate1, zIntermediate1);
      setVelocityRow(5, tIntermediate1, zdIntermediate1);      
      setPositionRow(6, tFinal, zf);
      setVelocityRow(7, tFinal, zdf);
      solveForCoefficients();
      setYoVariables();
   }

   public void setSepticInitialAndFinalAcceleration(double t0, double tIntermediate0, double tIntermediate1, double tFinal, 
         double z0, double zd0, double zdd0, double zIntermediate0, double zIntermediate1, double zf, double zdf, double zddf)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 8);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tIntermediate0, zIntermediate0);
      setPositionRow(4, tIntermediate1, zIntermediate1);
      setPositionRow(5, tFinal, zf);
      setVelocityRow(6, tFinal, zdf);
      setAccelerationRow(7, tFinal, zddf);
      solveForCoefficients();
      setYoVariables();
   }

   public void setNonic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, 
         double z0, double zd0, double zIntermediate0, double zdIntermediate0, double zIntermediate1, double zdIntermediate1, double zf, double zdf)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 10);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tIntermediate0, zIntermediate0);
      setVelocityRow(3, tIntermediate0, zdIntermediate0);
      setPositionRow(4, tIntermediate1, zIntermediate1);
      setVelocityRow(5, tIntermediate1, zdIntermediate1);      
      setPositionRow(6, tFinal, zf);
      setVelocityRow(7, tFinal, zdf);
      setAccelerationRow(8, t0, 0.0);
      setAccelerationRow(9, tFinal, 0.0);
      solveForCoefficients();
      setYoVariables();      
   }
   
   public void setSexticUsingWaypointVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdd0, double zdIntermediate, double zddIntermediate, double zFinal, double zdFinal)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 7);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setVelocityRow(3, tIntermediate, zdIntermediate);
      setAccelerationRow(4, tIntermediate, zddIntermediate);
      setPositionRow(5, tFinal, zFinal);
      setVelocityRow(6, tFinal, zdFinal);
      solveForCoefficients();
      setYoVariables();
   }
   
   public void setQuarticUsingIntermediateVelocity(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zdIntermediate, double zFinal, double zdFinal)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 5);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setVelocityRow(2, tIntermediate, zdIntermediate);
      setPositionRow(3, tFinal, zFinal);
      setVelocityRow(4, tFinal, zdFinal);
      solveForCoefficients();
      setYoVariables();
   }
   
   public void setQuartic(double t0, double tFinal, double z0, double zd0, double zdd0, double zFinal, double zdFinal)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 5);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tFinal, zFinal);
      setVelocityRow(4, tFinal, zdFinal);
      solveForCoefficients();
      setYoVariables();
   }

   public void setQuarticUsingMidPoint(double t0, double tFinal, double z0, double zd0, double zMid, double zFinal, double zdFinal)
   {
      double tMid = t0 + (tFinal - t0) / 2.0;
      setQuarticUsingWayPoint(t0, tMid, tFinal, z0, zd0, zMid, zFinal, zdFinal);
   }

   public void setQuarticUsingWayPoint(double t0, double tIntermediate, double tFinal, double z0, double zd0, double zIntermediate, double zf, double zdf)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 5);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tIntermediate, zIntermediate);
      setPositionRow(3, tFinal, zf);
      setVelocityRow(4, tFinal, zdf);
      solveForCoefficients();
      setYoVariables();      
   }
   
   public void setQuarticUsingFinalAcceleration(double t0, double tFinal, double z0, double zd0, double zFinal, double zdFinal, double zddFinal)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 5);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tFinal, zFinal);
      setVelocityRow(3, tFinal, zdFinal);
      setAccelerationRow(4, tFinal, zddFinal);
      solveForCoefficients();
      setYoVariables();
   }
   
   public void setCubic(double t0, double tFinal, double z0, double zd0, double zFinal, double zdFinal)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 4);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tFinal, zFinal);
      setVelocityRow(3, tFinal, zdFinal);
      solveForCoefficients();
      setYoVariables();
   }
   
   public void setCubicWithFinalVelocityConstraint(double t0, double tFinal, double z0, double zFinal, double zdFinal)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 3);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tFinal, zFinal);
      setVelocityRow(2, tFinal, zdFinal);
      solveForCoefficients();
      setYoVariables();
   }

   public void setCubicWithIntermediatePositionAndFinalVelocityConstraint(double t0, double tIntermediate, double tFinal, double z0, double zIntermediate, double zFinal, double zdFinal)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 4);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tIntermediate, zIntermediate);
      setPositionRow(2, tFinal, zFinal);
      setVelocityRow(3, tFinal, zdFinal);
      solveForCoefficients();
      setYoVariables();
   }
   
   public void setInitialPositionVelocityZeroFinalHighOrderDerivatives(double t0, double tFinal, double z0, double zd0, double zFinal, double zdFinal)
   {
      if (numberOfCoefficients < 4) throw new RuntimeException("Need at least 4 coefficients in order to set initial and final positions and velocities");
      
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tFinal, zFinal);
      setVelocityRow(3, tFinal, zdFinal);
      
      int order = 2;
      
      for (int row = 4; row < numberOfCoefficients; row++)
      {
         setConstraintRow(row, tFinal, 0.0, order++);
      }
      
      solveForCoefficients();
      setYoVariables();
   }

   public void setCubicUsingFinalAccelerationButNotFinalPosition(double t0, double tFinal, double z0, double zd0, double zdFinal, double zddFinal)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 4);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setVelocityRow(2, tFinal, zdFinal);
      setAccelerationRow(3, tFinal, zddFinal);
      solveForCoefficients();
      setYoVariables();
   }

   public void setQuadratic(double t0, double tFinal, double z0, double zd0, double zFinal)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 3);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setPositionRow(2, tFinal, zFinal);
      solveForCoefficients();
      setYoVariables();
   }

   public void setQuadraticUsingInitialAcceleration(double t0, double tFinal, double z0, double zd0, double zdd0)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 3);
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      solveForCoefficients();
      setYoVariables();
   }

   public void setQuadraticUsingIntermediatePoint(double t0, double tIntermediate, double tFinal, double z0, double zIntermediate, double zFinal)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 3);
      MathTools.checkIfInRange(tIntermediate, t0, tFinal);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tIntermediate, zIntermediate);
      setPositionRow(2, tFinal, zFinal);
      solveForCoefficients();
      setYoVariables();
   }
   
   public void setCubicUsingIntermediatePoints(double t0, double tIntermediate1, double tIntermediate2, double tFinal,
         double z0, double zIntermediate1, double zIntermediate2, double zFinal)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 4);
      MathTools.checkIfInRange(tIntermediate1, t0, tIntermediate1);
      MathTools.checkIfInRange(tIntermediate2, tIntermediate1, tFinal);
      setPositionRow(0, t0, z0);
      setPositionRow(1, tIntermediate1, zIntermediate1);
      setPositionRow(2, tIntermediate2, zIntermediate2);
      setPositionRow(3, tFinal, zFinal);
      solveForCoefficients();
      setYoVariables();
   }
   
   public void setCubicThreeInitialConditionsFinalPosition(double t0, double tFinal,
         double z0, double zd0, double zdd0, double zFinal)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 4);
      
      setPositionRow(0, t0, z0);
      setVelocityRow(1, t0, zd0);
      setAccelerationRow(2, t0, zdd0);
      setPositionRow(3, tFinal, zFinal);
     
      solveForCoefficients();
      setYoVariables();
   }

   public void setCubicInitialPositionThreeFinalConditions(double t0, double tFinal,
                                                           double z0, double zFinal, double zdFinal, double zddFinal)
   {
      MathTools.checkIfEqual(numberOfCoefficients, 4);

      setPositionRow(0, t0, z0);
      setPositionRow(1, tFinal, zFinal);
      setVelocityRow(2, tFinal, zdFinal);
      setAccelerationRow(3, tFinal, zddFinal);

      solveForCoefficients();
      setYoVariables();
   }

   private void solveForCoefficients()
   {
      solver.setA(constraintMatrix);
      solver.solve(constraintVector, coefficientVector);
   }

   public void setDirectly(double[] coefficients)
   {
      if (coefficients.length != numberOfCoefficients)
         throw new RuntimeException("Coefficient array is not of size " + numberOfCoefficients + ", coefficients.length = " + coefficients.length);
      for (int i = 0; i < numberOfCoefficients; i++)
      {
         this.a[i].set(coefficients[i]);
      }
   }
   
   public void compute(double x)
   {
      setXPowers(xPowers, x);
      
      pos = vel = acc = 0.0;
      for (int i = 0; i < numberOfCoefficients; i++)
      {
         pos += a[i].getDoubleValue() * xPowers[i];
      }
      
      for (int i = 1; i < numberOfCoefficients; i++)
      {
         vel += i * a[i].getDoubleValue() * xPowers[i - 1];
      }
      
      for (int i = 2; i < numberOfCoefficients; i++)
      {
         acc += (i - 1) * i * a[i].getDoubleValue() * xPowers[i - 2];
      }
   }

   public double getIntegral(double from, double to)
   {
      double[] fromPowers = new double[numberOfCoefficients + 1];
      double[] toPowers = new double[numberOfCoefficients + 1];
      setXPowers(fromPowers, from);
      setXPowers(toPowers, to);
      double integral = 0;
      for (int i = 0; i < numberOfCoefficients; i++)
      {
         integral += (1.0 / ((double)i + 1.0)) * a[i].getDoubleValue() * (toPowers[i + 1] - fromPowers[i + 1]);
      }
      return integral;
   }
   
   public void setXPowers(double[] xPowers, double x)
   {
      xPowers[0] = 1.0;
      for (int i = 1; i < xPowers.length; i++)
      {
         xPowers[i] = xPowers[i - 1] * x;
      }
   }

   public int getNumberOfCoefficients()
   {
      return numberOfCoefficients;
   }

   
   private void setConstraintRow(int row, double x, double desiredZDerivative, int derivativeOrderWithPositionBeingZero)
   {
      double xPower = 1.0;
            
      for (int col = derivativeOrderWithPositionBeingZero; col < numberOfCoefficients; col++)
      {
         double columnPower = 1.0;
         for (int i=0; i<derivativeOrderWithPositionBeingZero; i++)
         {
            columnPower *= (col-i);
         }
         constraintMatrix.set(row, col, xPower * columnPower);
         xPower *= x;
      }

      constraintVector.set(row, 0, desiredZDerivative);
   }
   
   private void setPositionRow(int row, double x, double z)
   {   
      setConstraintRow(row, x, z, 0);
   }
   
   private void setVelocityRow(int row, double x, double zVelocity)
   {
      setConstraintRow(row, x, zVelocity, 1);
   }

   private void setAccelerationRow(int row, double x, double zAcceleration)
   {
      setConstraintRow(row, x, zAcceleration, 2);
   }
   
   private void setYoVariables()
   {
      for (int row = 0; row < numberOfCoefficients; row++)
      {
         a[row].set(coefficientVector.get(row, 0));
      }
   }
   
}
