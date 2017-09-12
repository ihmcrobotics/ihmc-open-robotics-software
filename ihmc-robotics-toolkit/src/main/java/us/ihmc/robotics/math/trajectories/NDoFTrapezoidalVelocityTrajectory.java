package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.alphaToAlpha.AlphaToAlphaFunction;
import us.ihmc.robotics.alphaToAlpha.StretchedLinearAlphaToAlphaFunction;
import us.ihmc.robotics.alphaToAlpha.StretchedSlowInMiddleAlphaToAlphaFunction;
import us.ihmc.robotics.trajectories.TrapezoidalVelocityTrajectory;

/**
 * <p>Title: NDoFTrapezoidalVelocityTrajectory</p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 * @author not attributable
 * @version 1.0
 */
public class NDoFTrapezoidalVelocityTrajectory
{
   private final TrapezoidalVelocityTrajectory[] trajectories;
   private AlphaToAlphaFunction[] alphaToAlphaFunctions;
   private final double t0;
   private double tFMax;

   private final AlphaToAlphaType alphaToAlphaType;

   public NDoFTrapezoidalVelocityTrajectory(double t0, double[] x0, double[] xF, double[] v0, double[] vF, double[] vMax, double[] aMax,
           AlphaToAlphaType alphaToAlphaType)
   {
      this.trajectories = createTrajectories(t0, x0, xF, v0, vF, vMax, aMax);
      this.t0 = t0;
      this.tFMax = TrapezoidalVelocityTrajectory.getTFMax(this.trajectories);

      this.alphaToAlphaFunctions = null;    // Default to unsynchronized.
      this.alphaToAlphaType = alphaToAlphaType;

//    this.alphaToAlphaFunctions = getAlphaToAlphaFunctions(trajectories, tFMax);
   }

   public void synchronize(double tFMax)
   {
      if (alphaToAlphaType == null)
         return;

      this.tFMax = tFMax;

      switch (alphaToAlphaType)
      {
         case LINEAR :
         {
            this.alphaToAlphaFunctions = getLinearAlphaToAlphaFunctions(trajectories, tFMax);

            break;
         }

         case STRETCH_IN_MIDDLE :
         {
            this.alphaToAlphaFunctions = getStretchedInMiddleAlphaToAlphaFunctions(trajectories, tFMax);

            break;
         }

         default :
         {
            throw new RuntimeException();
         }
      }
   }

   public void synchronize()
   {
      synchronize(this.tFMax);
   }

   public void synchronizeWith(NDoFTrapezoidalVelocityTrajectory trajectory)
   {
      synchronize(new NDoFTrapezoidalVelocityTrajectory[] {this, trajectory});
   }

   public double getT0()
   {
      return t0;
   }

   public double[] getTFArray()
   {
      int n = trajectories.length;
      double[] ret = new double[n];
      for (int i = 0; i < n; i++)
      {
         ret[i] = trajectories[i].getFinalTime();
      }

      return ret;
   }

   public double[] getX0Array()
   {
      int n = trajectories.length;
      double[] ret = new double[n];
      for (int i = 0; i < n; i++)
      {
         ret[i] = trajectories[i].getX0();
      }

      return ret;
   }

   public double[] getV0Array()
   {
      int n = trajectories.length;
      double[] ret = new double[n];
      for (int i = 0; i < n; i++)
      {
         ret[i] = trajectories[i].getV0();
      }

      return ret;
   }



   public double getTFMax()
   {
      return tFMax;
   }

   public double getDTFMax()
   {
      return tFMax - t0;
   }


   public double getPosition(int index, double t)
   {
      // Without alpha mapping:
      // x = X(t)
      // t = t0 + alphaX * (tFX - t0)
      // alphaX = (t - t0) / (tFX - t0)

      // With alpha mapping:
      // y = X(tPrime)
      // tPrime = t0 + alphaPrime(alphaY) * (tFX - t0)
      // alphaY = (t - t0) / (tFY - t0)
      // where alphaPrime ensures that the velocities at t0 and tF end are unchanged

      if (alphaToAlphaFunctions == null)
      {
         return trajectories[index].getPosition(t);
      }

      else
      {
         double deltaTime = (tFMax - t0);
         double alphaY;

         if (deltaTime < 1e-10)
         {
            alphaY = 1.0;
         }

         else
         {
            alphaY = (t - t0) / deltaTime;
         }

         double alphaPrime = alphaToAlphaFunctions[index].getAlphaPrime(alphaY);
         double tPrime = t0 + trajectories[index].getMoveDuration() * alphaPrime;
         double position = trajectories[index].getPosition(tPrime);

         return position;
      }
   }

   public double getVelocity(int index, double t)
   {
      if (alphaToAlphaFunctions == null)
      {
         return trajectories[index].getVelocity(t);
      }
      else
      {
         // Calculated using chain rule:
         // y = X(tPrime(alphaPrime(alpha(t))))
         // yDot = dX/dt = dX/dtPrime * dtPrime/dAlphaPrime * dAlphaPrime/dAlpha * dAlpha/dt

         // Parameter values:
         double deltaTime = (tFMax - t0);
         double alphaY;

         if (deltaTime < 1e-10)
         {
            alphaY = 1.0;
         }
         else
         {
            alphaY = (t - t0) / deltaTime;
         }

         double alphaPrime = alphaToAlphaFunctions[index].getAlphaPrime(alphaY);
         double tPrime = t0 + (trajectories[index].getMoveDuration()) * alphaPrime;


         // First order derivatives:
         double dAlphaDT = 1.0 / deltaTime;
         double dTPrimeDAlphaPrime = trajectories[index].getMoveDuration();

         double dAlphaDT_times_dTPrimeDAlphaPrime;
         if ((deltaTime < 1e-10))
         {
            throw new RuntimeException("deltaTime < 1e-10");

//          dAlphaDT_times_dTPrimeDAlphaPrime = 1.0;
         }
         else
         {
            dAlphaDT_times_dTPrimeDAlphaPrime = dAlphaDT * dTPrimeDAlphaPrime;
         }

         double dAlphaPrimeDAlpha = alphaToAlphaFunctions[index].getDerivativeAtAlpha(alphaY);
         double dXDTPrime = trajectories[index].getVelocity(tPrime);

         // Chain rule:
         double dXDT = dAlphaDT_times_dTPrimeDAlphaPrime * dXDTPrime * dAlphaPrimeDAlpha;

         if (Double.isNaN(dXDT))
         {
            throw new RuntimeException("Double.isNaN(dXDT)");
         }

//       double epsilon = 1e-5;
//
//       if (dTPrimeDAlphaPrime * dAlphaPrimeDAlpha * dAlphaDT > 1.0 + epsilon)
//       {
//          throw new RuntimeException("dTPrimeDAlphaPrime * dAlphaPrimeDAlpha * dAlphaDT > 1.0");
//       }
//
//       if (Math.abs(dXDT) > trajectories[index].getMaximumVelocity() + epsilon)
//       {
//          throw new RuntimeException("abs(dXDT) > maximum velocity for the trajectory.\n" +
//                                     "dXDT = " + dXDT + ", maximum velocity for the trajectory = " + trajectories[index].getMaximumVelocity());
//       }

         return dXDT;
      }
   }

   public double getAcceleration(int index, double t)
   {
      if (alphaToAlphaFunctions == null)
      {
         return trajectories[index].getAcceleration(t);
      }
      else
      {
         // Calculated using chain rule for second derivatives.
         // Terms that are zero have already been dropped.
         // y = X(tPrime(alphaPrime(alpha(t))))
         // See T Koolen notes from 11/6/2008 or JPratt notes from 11/20/2008

         // Parameter values:
         double alphaY = (t - t0) / (tFMax - t0);
         double alphaPrime = alphaToAlphaFunctions[index].getAlphaPrime(alphaY);
         double tPrime = t0 + (trajectories[index].getMoveDuration()) * alphaPrime;

         // First order function derivatives:
         double dAlphaDT = 1.0 / (tFMax - t0);
         double dAlphaPrimeDAlpha = alphaToAlphaFunctions[index].getDerivativeAtAlpha(alphaY);
         double dTPrimeDAlphaPrime = trajectories[index].getMoveDuration();
         double dXDTPrime = trajectories[index].getVelocity(tPrime);

         // First order time derivatives:
         double dTPrimeDT = dTPrimeDAlphaPrime * dAlphaPrimeDAlpha * dAlphaDT;

//       double dXDT = dXDTPrime * dTPrimeDT;

         // Second order function derivatives:
         double d2AlphaPrimeDAlpha2 = alphaToAlphaFunctions[index].getSecondDerivativeAtAlpha(alphaY);
         double d2XDTPrime2 = trajectories[index].getAcceleration(tPrime);

         // Second order time derivatives using chain rule for second derivatives (T. Koolen):
         double d2AlphaPrimeDT2 = d2AlphaPrimeDAlpha2 * dAlphaDT * dAlphaDT;
         double d2TPrimeDT2 = dTPrimeDAlphaPrime * d2AlphaPrimeDT2;
         double d2XDT2 = d2XDTPrime2 * dTPrimeDT * dTPrimeDT + dXDTPrime * d2TPrimeDT2;

         return d2XDT2;


         // Second order time derivatives (J. Pratt). dX_dT = U * V * W, so d2X_dT2 = dU_dt * V * W + U * dV_dt * W + U * V * dW_dt;

//       double U = dTPrimeDAlphaPrime * dAlphaDT;
//       double V = dXDTPrime;
//       double W = dAlphaPrimeDAlpha;
//
//       double dU_dt = 0.0;
//       double dV_dt = trajectories[index].getAcceleration(tPrime) * dTPrimeDT;
//       double dW_dt = alphaToAlphaFunctions[index].getSecondDerivativeAtAlpha(alphaY) * dAlphaDT;
//
//       double d2X_dT2 = dU_dt * V * W + U * dV_dt * W + U * V * dW_dt;
//
//       return d2X_dT2;
      }
   }

   public double getMaximumVelocity(int index)
   {
      return trajectories[index].getMaximumVelocity();
   }

   public double getMaximumAcceleration(int index)
   {
      return trajectories[index].getMaximumAcceleration();
   }

   public double[] getPositionArray(double t)
   {
      int n = trajectories.length;
      double[] positions = new double[n];
      for (int i = 0; i < n; i++)
      {
         positions[i] = getPosition(i, t);
      }

      return positions;
   }

   public double[] getVelocityArray(double t)
   {
      int n = trajectories.length;
      double[] velocities = new double[n];
      for (int i = 0; i < n; i++)
      {
         velocities[i] = getVelocity(i, t);
      }

      return velocities;
   }

   public double[] getAccelerationArray(double t)
   {
      int n = trajectories.length;
      double[] accelerations = new double[n];
      for (int i = 0; i < n; i++)
      {
         accelerations[i] = getAcceleration(i, t);
      }

      return accelerations;
   }

   public double[] getMaximumVelocityArray()
   {
      int n = trajectories.length;
      double[] maximumVelocities = new double[n];
      for (int i = 0; i < n; i++)
      {
         maximumVelocities[i] = getMaximumVelocity(i);
      }

      return maximumVelocities;
   }

   public double[] getMaximumAccelerationArray()
   {
      int n = trajectories.length;
      double[] maximumAccelerations = new double[n];
      for (int i = 0; i < n; i++)
      {
         maximumAccelerations[i] = getMaximumAcceleration(i);
      }

      return maximumAccelerations;
   }

   public int size()
   {
      return trajectories.length;
   }

   public static void synchronize(NDoFTrapezoidalVelocityTrajectory[] trajectories)
   {
      double tFMaxMax = getTFMaxMax(trajectories);
      for (NDoFTrapezoidalVelocityTrajectory trajectory : trajectories)
      {
         trajectory.synchronize(tFMaxMax);
      }
   }

   @SuppressWarnings("unused")
   private TrapezoidalVelocityTrajectory[] getTrajectoriesCopy()
   {
      TrapezoidalVelocityTrajectory[] copy = new TrapezoidalVelocityTrajectory[trajectories.length];
      for (int i = 0; i < trajectories.length; i++)
      {
         copy[i] = new TrapezoidalVelocityTrajectory(trajectories[i]);
      }

      return copy;
   }

   private static TrapezoidalVelocityTrajectory[] createTrajectories(double t0, double[] x0, double[] xF, double[] v0, double[] vF, double[] vMax,
           double[] aMax)
   {
      int n = x0.length;

      if ((xF.length != n) || (v0.length != n) || (vF.length != n) || (vMax.length != n) || (aMax.length != n))
      {
         throw new RuntimeException("Array lengths are not equal");
      }

      TrapezoidalVelocityTrajectory[] trajectories = new TrapezoidalVelocityTrajectory[n];
      for (int i = 0; i < n; i++)
      {
         trajectories[i] = new TrapezoidalVelocityTrajectory(t0, x0[i], xF[i], v0[i], vF[i], vMax[i], aMax[i]);
      }

      return trajectories;
   }

   private static double getTFMaxMax(NDoFTrapezoidalVelocityTrajectory[] trajectories)
   {
      double tFMaxMax = Double.NEGATIVE_INFINITY;
      for (NDoFTrapezoidalVelocityTrajectory trajectory : trajectories)
      {
         double tFMax = trajectory.getTFMax();
         if (tFMax > tFMaxMax)
         {
            tFMaxMax = tFMax;
         }
      }

      return tFMaxMax;
   }

   private static StretchedSlowInMiddleAlphaToAlphaFunction[] getStretchedInMiddleAlphaToAlphaFunctions(TrapezoidalVelocityTrajectory[] trajectories,
           double tFMax)
   {
      int n = trajectories.length;
      StretchedSlowInMiddleAlphaToAlphaFunction[] alphaToAlphaFunctions = new StretchedSlowInMiddleAlphaToAlphaFunction[n];
      for (int i = 0; i < n; i++)
      {
         double t0 = trajectories[i].getT0();
         double tF = trajectories[i].getFinalTime();

//       if ((tF - t0) <= 0.0)
//       {
//          throw new RuntimeException("(tF - t0) <= 0.0, (tF - t0) = " + (tF - t0));
//       }
         if (tF > t0)
         {
            alphaToAlphaFunctions[i] = new StretchedSlowInMiddleAlphaToAlphaFunction((tFMax - t0) / (tF - t0));
         }
         else
         {
            alphaToAlphaFunctions[i] = new StretchedSlowInMiddleAlphaToAlphaFunction(1.0);
         }
      }

      return alphaToAlphaFunctions;
   }

   private static StretchedLinearAlphaToAlphaFunction[] getLinearAlphaToAlphaFunctions(TrapezoidalVelocityTrajectory[] trajectories, double tFMax)
   {
      int n = trajectories.length;
      StretchedLinearAlphaToAlphaFunction[] alphaToAlphaFunctions = new StretchedLinearAlphaToAlphaFunction[n];
      for (int i = 0; i < n; i++)
      {
         @SuppressWarnings("unused")
         double t0 = trajectories[i].getT0();
         @SuppressWarnings("unused")
         double tF = trajectories[i].getFinalTime();
         alphaToAlphaFunctions[i] = new StretchedLinearAlphaToAlphaFunction();
      }

      return alphaToAlphaFunctions;
   }


   public String toString()
   {
      String ret = "Trajectories:";
      for (TrapezoidalVelocityTrajectory trap : trajectories)
      {
         ret += "\n";
         ret += trap.toString();
      }

      ret += "\n\nalphaToAlphaFunctions:";

      for (AlphaToAlphaFunction alpha : alphaToAlphaFunctions)
      {
         ret += "\n";
         ret += alpha.toString();
      }

      return ret;
   }

   public enum AlphaToAlphaType {LINEAR, STRETCH_IN_MIDDLE}

   ;
}
