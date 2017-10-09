package us.ihmc.robotics.math.trajectories;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoMinimumJerkTrajectory
{
   public static final boolean DEBUG = false;
   
   private final YoDouble X0, V0, A0, Xf, Vf, Af, T0, Tf;
   private double C0, C1, C2, C3, C4, C5;

   public double pos, vel, acc;

   public YoMinimumJerkTrajectory(String name, YoVariableRegistry registry)
   {
      X0 = new YoDouble(name + "_x0", registry);
      V0 = new YoDouble(name + "_v0", registry);
      A0 = new YoDouble(name + "_a0", registry);
      T0 = new YoDouble(name + "_t0", registry);

      Xf = new YoDouble(name + "_xf", registry);
      Vf = new YoDouble(name + "_vf", registry);
      Af = new YoDouble(name + "_af", registry);
      Tf = new YoDouble(name + "_tf", registry);
   }

   public double getStartTime()
   {
      return T0.getDoubleValue();
   }

   public double getFinalTime()
   {
      return Tf.getDoubleValue();
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

   public void setParams(double X0, double V0, double A0, double Xf, double Vf, double Af, double T0, double Tf)
   {
      this.X0.set(X0);
      this.V0.set(V0);
      this.A0.set(A0);
      this.Xf.set(Xf);
      this.Vf.set(Vf);
      this.Af.set(Af);
      this.T0.set(T0);
      this.Tf.set(Tf);
      this.computeConstants();
   }

   private void computeConstants()
   {
      double DT = Tf.getDoubleValue() - T0.getDoubleValue();
      double DT2 = DT * DT;

      C0 = 1.0000 * X0.getDoubleValue();
      C1 = 1.0000 * V0.getDoubleValue() * DT;
      C2 = 0.5000 * A0.getDoubleValue() * DT2;
      C3 = -10.0000 * X0.getDoubleValue() - 6.0000 * V0.getDoubleValue() * DT - 1.5000 * A0.getDoubleValue() * DT2 + 10.0000 * Xf.getDoubleValue()
           - 4.0000 * Vf.getDoubleValue() * DT + 0.5000 * Af.getDoubleValue() * DT2;
      C4 = 15.0000 * X0.getDoubleValue() + 8.0000 * V0.getDoubleValue() * DT + 1.5000 * A0.getDoubleValue() * DT2 - 15.0000 * Xf.getDoubleValue()
           + 7.0000 * Vf.getDoubleValue() * DT - 1.0000 * Af.getDoubleValue() * DT2;
      C5 = -6.0000 * X0.getDoubleValue() - 3.0000 * V0.getDoubleValue() * DT - 0.5000 * A0.getDoubleValue() * DT2 + 6.0000 * Xf.getDoubleValue()
           - 3.0000 * Vf.getDoubleValue() * DT + 0.5000 * Af.getDoubleValue() * DT2;
   }

   public void computeTrajectoryDoubles(double t, double[] vals)
   {
      computeTrajectory(t);
      vals[0] = this.pos;
      vals[1] = this.vel;
      vals[2] = this.acc;
   }

   public void computeTrajectory(double t, YoDouble pos)
   {
      computeTrajectory(t);
      pos.set(this.pos);
   }

   public void computeTrajectory(double t, YoDouble pos, YoDouble vel)
   {
      computeTrajectory(t);
      pos.set(this.pos);
      vel.set(this.vel);
   }

   public void computeTrajectory(double t, YoDouble pos, YoDouble vel, YoDouble acc)
   {
      computeTrajectory(t);
      pos.set(this.pos);
      vel.set(this.vel);
      acc.set(this.acc);
   }

   public void computeTrajectory(double t)
   {
      computeConstants();

      double DT = Tf.getDoubleValue() - T0.getDoubleValue();
      double DT2 = DT * DT;

      if (t < T0.getDoubleValue())
      {
         pos = X0.getDoubleValue();
         vel = V0.getDoubleValue();
         acc = A0.getDoubleValue();

         return;
      }

      if (t > Tf.getDoubleValue())
      {
         pos = Xf.getDoubleValue();
         vel = Vf.getDoubleValue();
         acc = Af.getDoubleValue();

         return;
      }

      double tau = (t - T0.getDoubleValue()) / DT;
      double tau2 = tau * tau;
      double tau3 = tau * tau2;
      double tau4 = tau * tau3;
      double tau5 = tau * tau4;

      pos = C0 + C1 * tau + C2 * tau2 + C3 * tau3 + C4 * tau4 + C5 * tau5;
      vel = (C1 + 2.0 * C2 * tau + 3.0 * C3 * tau2 + 4.0 * C4 * tau3 + 5.0 * C5 * tau4) / DT;
      acc = (2.0 * C2 + 6.0 * C3 * tau + 12.0 * C4 * tau2 + 20.0 * C5 * tau3) / DT2;
   }
   
   private final int maxFindIterations = 10;
   
   /**
    * Finds the maximum absolute value of the velocity and acceleration of the MinimumJerkTrajectory from the given current time to the Trajectories final time.
    *
    * @deprecated
    * @param t double : Time to start looking for a maximum velocity and acceleration.
    * @param maximums double[2] : {max velocity, max acceleration}
    */
   public void findMaxVelocityAndAccel(double currentTime, double[] maximums)
   {
      if (currentTime >= Tf.getDoubleValue())
      {
         maximums[0] = Vf.getDoubleValue();
         maximums[1] = Af.getDoubleValue();
      }

      computeConstants();

      double DT = Tf.getDoubleValue() - T0.getDoubleValue();
      double DT2 = DT * DT;
      double deltaTime = ((Tf.getDoubleValue() - T0.getDoubleValue()) / maxFindIterations);

      double maxvel = Double.NEGATIVE_INFINITY;
      double maxacc = Double.NEGATIVE_INFINITY;

      double time = currentTime;

      do
      {
         double tau = (time - T0.getDoubleValue()) / DT;
         double tau2 = tau * tau;
         double tau3 = tau * tau2;
         double tau4 = tau * tau3;

         double currvel = Math.abs((C1 + 2.0 * C2 * tau + 3.0 * C3 * tau2 + 4.0 * C4 * tau3 + 5.0 * C5 * tau4) / DT);
         double curracc = Math.abs((2.0 * C2 + 6.0 * C3 * tau + 12.0 * C4 * tau2 + 20.0 * C5 * tau3) / DT2);

         if (currvel > maxvel)
            maxvel = currvel;
         if (curracc > maxacc)
            maxacc = curracc;

         time = time + deltaTime;
      }
      while (time <= Tf.getDoubleValue());

      maximums[0] = maxvel;
      maximums[1] = maxacc;
   }

   private final int maxTimeIterations = 20;
   private final double minTimeWindow = 0.005;
   
   /**
    * Calculates minimum final time for trajectory to keep velocity and accelerations within limits.
    * If values are already within limits it will return 0.0, otherwise it will return the suggested Tf.
    * The suggested Tf will always be >= the given currentTime, and always be >= Tf.
    *
    * @deprecated
    * @param t double: current time
    * @param maxV double: maximum velocity to use
    * @param maxA double: maximum acceleration to use
    * @return double
    */
   public double timeExtension(double currentTime, double maxV, double maxA, boolean Fix)
   {
      // Error if parameters are already higher, ignore extension
      if (Fix)
      {
         if (maxV < Math.abs(V0.getDoubleValue()))
         {
            V0.set(V0.getDoubleValue() * Math.abs(maxV / V0.getDoubleValue()) * 0.95);
            if (DEBUG)
               System.out.println("MinimumJerkTrajectory.timeExtension: clamped V0 to maxV");
         }

         if (maxV < Math.abs(Vf.getDoubleValue()))
         {
            Vf.set(Vf.getDoubleValue() * Math.abs(maxV / Vf.getDoubleValue()) * 0.95);
            if (DEBUG)
               System.out.println("MinJerkTrajectory.timeExtension: clamped Vf to maxV");
         }

         if (maxA < Math.abs(A0.getDoubleValue()))
         {
            A0.set(A0.getDoubleValue() * Math.abs(maxA / A0.getDoubleValue()) * 0.95);
            if (DEBUG)
               System.out.println("MinimumJerkTrajectory.timeExtension: clamped A0 to maxA");
         }

         if (maxA < Math.abs(Af.getDoubleValue()))
         {
            Af.set(Af.getDoubleValue() * Math.abs(maxA / Af.getDoubleValue()) * 0.95);
            if (DEBUG)
               System.out.println("MinimumJerkTrajectory.timeExtension: clamped Af to maxA");
         }
      }
      else if ((maxV < Math.abs(V0.getDoubleValue())) || (maxA < Math.abs(A0.getDoubleValue())) || (maxV < Math.abs(Vf.getDoubleValue())) || (maxA < Math.abs(Af.getDoubleValue())))
      {
         if (DEBUG)
            System.err.println("MinimumJerkTrajectory.timeExtension: Trying to extend time with maximums less than start or end parameters, nothing changed");

         return 0.0;
      }

      double[] maxs = new double[2];
      findMaxVelocityAndAccel(currentTime, maxs);

      // If the given times are within the max vel and acceleration, then just return 0.0
      // since we'll only increase the time.
      if ((maxs[0] <= maxV) && (maxs[1] <= maxA))
      {
         return 0.0;
      }

      // We need to increase Tf. Remember the original.
      final double TForig = Tf.getDoubleValue();

      double timeadd = 0.5;
      double lowerbound = Math.max(currentTime, Tf.getDoubleValue());
      double upperbound = lowerbound + timeadd;

      // March forward until you find an upper bound on Tf:
      int timeIterations = 1;

      boolean withinBounds = false;

      while (!withinBounds && (timeIterations < maxTimeIterations))
      {
         upperbound = lowerbound + timeadd;

         Tf.set(upperbound);
         findMaxVelocityAndAccel(currentTime, maxs);

         withinBounds = (maxs[0] < maxV) && (maxs[1] <= maxA);

         timeadd = timeadd * 2.0;
         timeIterations++;
      }

      // Squeeze play until you are within minTimeWindow or did more than maxTimeIterations.
      // If no iterations are left, then just return upperBound

      while ((timeIterations < maxTimeIterations) && (upperbound - lowerbound > minTimeWindow))
      {
         double middle = (lowerbound + upperbound) / 2.0;

         Tf.set(middle);
         findMaxVelocityAndAccel(currentTime, maxs);

         if ((maxs[0] <= maxV) && (maxs[1] <= maxA))
         {
            upperbound = middle;
         }
         else
         {
            lowerbound = middle;
         }

         timeIterations++;
      }

      Tf.set(TForig);

      return upperbound;
   }

   @Deprecated
   public void updateToFinalPosition(double currentTime, double Xf)
   {
      computeTrajectory(currentTime);

      this.T0.set(currentTime);
      this.Xf.set(Xf);

      this.X0.set(getPosition());
      this.V0.set(getVelocity());
      this.A0.set(getAcceleration());

      this.computeConstants();
   }

   @Deprecated
   public void setFinalPosition(double Xf)
   {
      this.Xf.set(Xf);
      this.computeConstants();
   }
}
