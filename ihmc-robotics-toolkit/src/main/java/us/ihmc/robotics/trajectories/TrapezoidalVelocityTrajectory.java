package us.ihmc.robotics.trajectories;

import java.util.ArrayList;

public class TrapezoidalVelocityTrajectory
{
   private final double epsilon = 1e-7;
   private final double t0, dT1, dT2, dTF, x0, v0, vMax, a;

   public TrapezoidalVelocityTrajectory(TrapezoidalVelocityTrajectory trapezoidalVelocityTrajectory)
   {
      this.t0 = trapezoidalVelocityTrajectory.t0;
      this.dT1 = trapezoidalVelocityTrajectory.dT1;
      this.dT2 = trapezoidalVelocityTrajectory.dT2;
      this.dTF = trapezoidalVelocityTrajectory.dTF;
      this.x0 = trapezoidalVelocityTrajectory.x0;
      this.v0 = trapezoidalVelocityTrajectory.v0;
      this.vMax = trapezoidalVelocityTrajectory.vMax;
      this.a = trapezoidalVelocityTrajectory.a;
   }

   public TrapezoidalVelocityTrajectory(double t0, double x0, double xF, double v0, double vF, double vMax, double aMax)
   {
      this(t0, x0, xF, v0, vF, vMax, aMax, true);
   }

   public TrapezoidalVelocityTrajectory(double t0, double x0, double xF, double v0, double vF, double vMax, double aMax, boolean enforceFinalVelocity)
   {
      checkPreconditions(v0, vF, vMax, aMax);

      // +++JEP: 090506. Was crashing on dog. These two lines fix it, but should be revisited.
      if (Math.abs(v0) > vMax)
         vMax = Math.abs(v0) + 1e-7;
      if (Math.abs(vF) > vMax)
         vF = (vMax - 1e-7) * Math.signum(vF);

      this.t0 = t0;
      this.x0 = x0;
      this.v0 = v0;
      this.vMax = vMax;

      double a;
      double dT1;
      double dT2;
      double dTF;

      // Optimal t1 without velocity constraint:
      a = (xF - x0 > 0.0) ? aMax : -aMax;
      dT1 = -v0 / a + Math.sqrt((v0 / a) * (v0 / a) + v0 * (vF - v0) / (a * a) + (xF - x0) / a + (vF - v0) * (vF - v0) / (2.0 * a * a));

      // Velocity constraint:
      if (Math.abs(v0 + a * dT1) > vMax)
      {
         dT1 = (vMax * Math.signum(a) - v0) / a;
      }

      dTF = ((xF - x0) + a * dT1 * dT1 - (vF - v0) * dT1 + (vF - v0) * (vF - v0) / (2.0 * a)) / (a * dT1 + v0);

      if ((a * dT1 + v0) == 0.0)
      {
         dTF = dT1;
      }

      dT2 = dTF - dT1 + (vF - v0) / a;

      // Pretty ugly, but it works:
      if ((dT1 < -epsilon) || (dTF < (dT2 - epsilon)))
      {
         if (enforceFinalVelocity)
         {
            // Now try accelerating in the other direction
            a = (xF - x0 > 0.0) ? -aMax : aMax;

            // Optimal t1 without velocity constraint:
            dT1 = -v0 / a + Math.sqrt((v0 / a) * (v0 / a) + v0 * (vF - v0) / (a * a) + (xF - x0) / a + (vF - v0) * (vF - v0) / (2.0 * a * a));

            // Velocity constraint:
            if (Math.abs(v0 + a * dT1) > vMax)
            {
               dT1 = (vMax * Math.signum(a) - v0) / a;
            }

            dTF = ((xF - x0) + a * dT1 * dT1 - (vF - v0) * dT1 + (vF - v0) * (vF - v0) / (2.0 * a)) / (a * dT1 + v0);

            if (Double.isNaN(dTF))
            {
               // This occurs if (a * dT1 + v0) happens to be exactly 0.0
               dTF = dT1;
            }

            dT2 = dTF - dT1 + (vF - v0) / a;
         }
         else
         {
            // At this point, we cannot satisfy the final veloctiy constraint, so determine
            // what the sign of the acceleration should be
            // This might not work for
            if ((xF > x0) && (v0 > vF))
               a = -aMax;
            else
               a = aMax;

            // Keep the original direction of acceleration.
            // Determine the amount of time you have before you hit the final position:
            dT1 = (-v0 + Math.sqrt(v0 * v0 - 2.0 * a * (x0 - xF))) / a;    // argument of the square root will always be positive because sign(a) = -sign(x0 - xF)

            // Velocity constraint:
            if (Math.abs(v0 + a * dT1) > vMax)
            {
               // try reversing the acceleration
               if (xF > x0)
                  a = -aMax;
               else
                  a = aMax;

               dT1 = (-v0 + Math.sqrt(v0 * v0 - 2.0 * a * (x0 - xF))) / a;    // argument of the square root will always be positive because sign(a) = -sign(x0 - xF)

               // Velocity constraint:
               if (Math.abs(v0 + a * dT1) > vMax)
               {
                  System.err.println("Should never get here: t0=" + t0 + ", x0=" + x0 + ", xF=" + xF + ", v0=" + v0 + ", vF=" + vF + ", vMax=" + vMax
                                     + " aMax=" + aMax + ", enforceFinalVelocity=" + enforceFinalVelocity);

                  throw new RuntimeException("Should never get here");    // Should never get here, because it couldn't even reach the desired final velocity, let alone the max velocity.
               }
            }

            dT2 = dT1;
            dTF = dT1;
         }
      }

      this.dT1 = dT1;
      this.dT2 = dT2;
      this.dTF = dTF;
      this.a = a;

      if (Double.isNaN(dT1) || Double.isNaN(dT2) || Double.isNaN(dTF))
      {
         System.err.println("Time contain NaN: t0=" + t0 + ", x0=" + x0 + ", xF=" + xF + ", v0=" + v0 + ", vF=" + vF + ", vMax=" + vMax + " aMax=" + aMax
                            + ", enforceFinalVelocity=" + enforceFinalVelocity);

         throw new RuntimeException("Times contain NaN. \n" + "dT1 = " + dT1 + ", dT2 = " + dT2 + ", dTF = " + dTF);
      }
   }

   public double getPosition(double t)
   {
      double dT = t - t0;

      double x;

      if (dT < 0.0)
      {
         // +++081230 pdn: it makes more sense to return the start position
         x = x0;

//       throw new RuntimeException("Time out of range");
      }
      else if (dT < dT1)
      {
         x = 0.5 * a * dT * dT + v0 * dT + x0;
      }
      else if (dT < dT2)
      {
         // x = 0.5 * a * dT * dT - 0.5 * a * (dT - dT1) * (dT - dT1) + v0 * dT + x0;
         // more consise:
         x = a * (-0.5 * dT1 * dT1 + dT * dT1) + v0 * dT + x0;
      }
      else if (dT < dTF)
      {
         // x = 0.5 * a * dT * dT - 0.5 * a * (dT - dT1) * (dT - dT1) - 0.5 * a * (dT - dT2) * (dT - dT2) + v0 * dT + x0;
         // more consise:
         x = a * (-0.5 * (dT * dT + dT1 * dT1 + dT2 * dT2) + dT * (dT1 + dT2)) + v0 * dT + x0;
      }
      else
      {
         // x = 0.5 * a * dTF * dTF - 0.5 * a * (dTF - dT1) * (dTF - dT1) - 0.5 * a * (dTF - dT2) * (dTF - dT2) + v0 * dTF + x0;
         // more consise:
         x = a * (-0.5 * (dTF * dTF + dT1 * dT1 + dT2 * dT2) + dTF * (dT1 + dT2)) + v0 * dTF + x0;
      }

      return x;
   }

   public double getVelocity(double t)
   {
      double dT = t - t0;
      if (dT < (0.0 - 1e-6))
      {
         throw new RuntimeException("Time out of range, time=" + t);
      }

      double v;

      if (dT < dT1)
      {
         v = a * dT + v0;
      }
      else if (dT < dT2)
      {
         // v = a * dT - a * (dT - dT1) + v0;
         // more consise:
         v = a * dT1 + v0;
      }
      else if (dT < dTF)
      {
         // v = a * dT - a * (dT - dT1) - a * (dT - dT2) + v0;
         // more consise:
         v = a * (dT1 + dT2 - dT) + v0;
      }
      else
      {
         // v = a * dTF - a * (dTF - dT1) - a * (dTF - dT2) + v0;
         // more consise:
         v = a * (dT1 + dT2 - dTF) + v0;
      }

      double epsilonVelocityLimit = 1e-7;

      if (Math.abs(v) > vMax + epsilonVelocityLimit)
      {
         throw new RuntimeException("Velocity limit exceeded. v = " + v + ", vMax = " + vMax);
      }

      return v;
   }

   public double getAcceleration(double t)
   {
      double dT = t - t0;
      if (dT < 0.0)
      {
         throw new RuntimeException("Time out of range");
      }

      double a;

      if (dT < dT1)
      {
         a = this.a;
      }
      else if (dT < dT2)
      {
         a = 0.0;
      }
      else if (dT < dTF)
      {
         a = -this.a;
      }
      else
      {
         a = 0.0;
      }

      return a;
   }

   public double[] getState(double t)
   {
      return new double[] {this.getPosition(t), this.getVelocity(t)};
   }

   public double getDT1()
   {
      return dT1;
   }

   public double getDT2()
   {
      return dT2;
   }

   public double getMoveDuration()
   {
      return dTF;
   }

   public double getMaximumVelocity()
   {
      return vMax;
   }

   public double getMaximumAcceleration()
   {
      return Math.abs(a);
   }

   public double getT0()
   {
      return t0;
   }

   public double getT1()
   {
      return t0 + dT1;
   }

   public double getT2()
   {
      return t0 + dT2;
   }

   public double getFinalTime()
   {
      return t0 + dTF;
   }

   public double getX0()
   {
      return x0;
   }

   public double getV0()
   {
      return v0;
   }

   public double getVMax()
   {
      return vMax;
   }

   public double getAMax()
   {
      return Math.abs(a);
   }


   public double getEpsilon()
   {
      return epsilon;
   }



   public static double getTFMax(TrapezoidalVelocityTrajectory[] trajectories)
   {
      double tFMax = Double.NEGATIVE_INFINITY;
      for (TrapezoidalVelocityTrajectory trajectory : trajectories)
      {
         double tF = trajectory.getFinalTime();
         if (tF > tFMax)
         {
            tFMax = tF;
         }
      }

      return tFMax;
   }

   private static void checkPreconditions(double v0, double vF, double vMax, double aMax)
   {
      // NaN:
      if (Double.isNaN(vMax))
      {
         throw new RuntimeException("Double.isNaN(vMax)");
      }

      if (Double.isNaN(aMax))
      {
         throw new RuntimeException("Double.isNaN(aMax)");
      }

      if (Double.isNaN(v0))
      {
         throw new RuntimeException("Double.isNaN(v0)");
      }

      if (Double.isNaN(vF))
      {
         throw new RuntimeException("Double.isNaN(vF)");
      }

      // Fix small speed limit violations:
      double epsilonInitialVelocityCheck = 1e-7;
      if (Math.abs(v0) - vMax < epsilonInitialVelocityCheck)
         ;
      {
         v0 = vMax * Math.signum(v0);
      }

      // Out of range:
      if (vMax < 0.0)
      {
         throw new RuntimeException("vMax < 0.0");
      }

      if (aMax < 0.0)
      {
         throw new RuntimeException("aMax < 0.0");
      }

      if (Math.abs(v0) > vMax)
      {
         throw new RuntimeException("v0 > vMax. v0 = " + v0 + ", vMax = " + vMax);
      }

      if (Math.abs(vF) > vMax)
      {
         throw new RuntimeException("vF > vMax. vF = " + vF + ", vMax = " + vMax);
      }
   }

   public String toString()
   {
      return "t0 = " + t0 + "\n" + "dT1 = " + dT1 + "\n" + "dT2 = " + dT2 + "\n" + "dTF = " + dTF + "\n" + "x0 = " + x0 + "\n" + "v0 = " + v0 + "\n"
             + "vMax = " + vMax + "\n" + "a = " + a + "\n" + "epsilon = " + epsilon + "\n";
   }

   public static void findErrorConditions()
   {
      double maxInitialVelocity = 0.2;
      double velocityStep = 0.001;
      double maxEndVelocity = 0.15;
      double maxAcceleration = maxInitialVelocity * 5.0;
      double accelerationStep = maxAcceleration / 20.0;
      double maxMoveDistance = 0.2;
      double moveDistanceStep = 0.005;

      int counter = 0;
      for (double initialVelocity = 0.0; initialVelocity < maxInitialVelocity; initialVelocity += velocityStep)
      {
         for (double endVelocity = 0.0; endVelocity < maxEndVelocity; endVelocity += velocityStep)
         {
            for (double acceleration = maxAcceleration; acceleration > 0.1 * maxAcceleration; acceleration -= accelerationStep)
            {
               for (double moveDistance = 0.001; moveDistance < maxMoveDistance; moveDistance += moveDistanceStep)
               {
                  TrapezoidalVelocityTrajectory trapezoidalVelocityTrajectory;

                  try
                  {
                     trapezoidalVelocityTrajectory = new TrapezoidalVelocityTrajectory(0.0, 0.0, moveDistance, initialVelocity, endVelocity,
                             maxInitialVelocity, acceleration, false);

                     boolean checkSolution = true;
                     if (checkSolution)
                     {
                        double endTime = trapezoidalVelocityTrajectory.getFinalTime();

                        if (endTime < 0.0)
                        {
                           System.err.println("endTime < 0.0: t0=" + 0.0 + ", x0=" + 0.0 + ", xF=" + moveDistance + ", v0=" + initialVelocity + ", vF="
                                              + endVelocity + ", vMax=" + maxInitialVelocity + " aMax=" + acceleration + ", enforceFinalVelocity=" + false);
                        }

                        double endPosition = trapezoidalVelocityTrajectory.getPosition(endTime);

                        if (Math.abs(endPosition - moveDistance) > 0.01 * moveDistance)
                        {
                           double moveDistanceError = Math.abs(endPosition - moveDistance);
                           System.err.println("moveDistanceError=" + moveDistanceError + ", : t0=" + 0.0 + ", x0=" + 0.0 + ", xF=" + moveDistance + ", v0="
                                              + initialVelocity + ", vF=" + endVelocity + ", vMax=" + maxInitialVelocity + " aMax=" + acceleration
                                              + ", enforceFinalVelocity=" + false);

                        }

                        double endSpeed = trapezoidalVelocityTrajectory.getVelocity(endTime);
                        if ((endSpeed < -1e-6) || (endSpeed > maxInitialVelocity))
                        {
                           System.err.println("endSpeed=" + endSpeed + ", : t0=" + 0.0 + ", x0=" + 0.0 + ", xF=" + moveDistance + ", v0=" + initialVelocity
                                              + ", vF=" + endVelocity + ", vMax=" + maxInitialVelocity + " aMax=" + acceleration + ", enforceFinalVelocity="
                                              + false);
                        }
                     }

                  }
                  catch (RuntimeException ex)
                  {
//                   System.err.println(ex);
                  }

//                System.out.println("counter=" + counter);
                  counter++;
               }
            }
         }
      }

      System.out.println("Done ");
      System.exit(-1);

   }
   
   public static void testZeroMoveCase()
   {
      double t0 = 0.0;    // 0.22;
      double x0 = 0.0;
      double xF = 0.0;
      double v0 = 0.0;
      double vF = 0.0;
      double vMax = 1.0;
      double aMax = 1.0;

      TrapezoidalVelocityTrajectory trapezoidalVelocityTrajectory = new TrapezoidalVelocityTrajectory(t0, x0, xF, v0, vF, vMax, aMax);
      double endTime = trapezoidalVelocityTrajectory.getFinalTime();
      
      System.out.println("Move time = " + endTime);
   }
   

   public static void testBadCase()
   {
      double t0 = 0.0;    // 0.22;
      double x0 = 0.0;
      double xF = 0.053;
      double v0 = 0.107;
      double vF = 0.0;
      double vMax = 0.2;
      double aMax = 0.4;
      boolean enforceFinalVelocity = false;


//    t0=0.0; x0=0.0; xF=0.08686748147796984; v0=0.0; vF=0.14; vMax=0.2; aMax=0.09999999999999987; enforceFinalVelocity=false;

      // Min distance check
      double minDistance = v0 * v0 / (2.0 * aMax);
      if (minDistance < xF)
      {
         System.err.println("The move distance is too small. MinDistance to stop in time=" + minDistance);
      }

      TrapezoidalVelocityTrajectory trapezoidalVelocityTrajectory = new TrapezoidalVelocityTrajectory(t0, x0, xF, v0, vF, vMax, aMax, enforceFinalVelocity);
      double endTime = trapezoidalVelocityTrajectory.getFinalTime();
      double deltaT = 0.01;
      ArrayList<Double> timeArray = new ArrayList<Double>();
      ArrayList<Double> positionArray = new ArrayList<Double>();
      ArrayList<Double> velocityArray = new ArrayList<Double>();

      for (double time = t0; time <= endTime; time += deltaT)
      {
         timeArray.add(time);
         double position = trapezoidalVelocityTrajectory.getPosition(time);
         positionArray.add(position);
         velocityArray.add(trapezoidalVelocityTrajectory.getVelocity(time));

         // System.out.println(JohnsUsefulFunctions.getFormattedDecimal3D(time) + ", " +
         // JohnsUsefulFunctions.getFormattedDecimal3D(position);
      }

//
//    PlotGraph2d pg = PlotGraph2d.createPlotGraph2d(timeArray, positionArray);
//    pg.setGraphTitle("Position");
//    pg.plot();
//
//
//    PlotGraph2d pgVel = PlotGraph2d.createPlotGraph2d(timeArray, velocityArray);
//    pgVel.setGraphTitle("Velocity");
//    pgVel.plot();
//

      System.out.println("Done ");

//    System.exit(-1);
   }

   public static void main(String[] args)
   {
      //testBadCase();
      testZeroMoveCase();

//    findErrorConditions();
//    TrapezoidalVelocityTrajectory trapezoidalVelocityTrajectory;
//
//    double endTime;
//    double deltaT = 0.01;
//
//    ArrayList<Double> timeArray = new ArrayList<Double>();
//    ArrayList<Double> positionArray = new ArrayList<Double>();
//    ArrayList<Double> velocityArray = new ArrayList<Double>();
//
//
//    for(double accel = 0.045; accel > 0.04; accel = accel - 0.001)
//    {
//       trapezoidalVelocityTrajectory = new TrapezoidalVelocityTrajectory(0.0, 0.0, 0.103, 0.106, 0.05, 0.104, accel, false);
//
//       endTime = trapezoidalVelocityTrajectory.getMoveDuration();
//       System.out.println(accel + ", endTime=" + endTime);
//
//       for (double time = 0.0; time <= endTime; time += deltaT)
//       {
//          timeArray.add(time);
//          double position = trapezoidalVelocityTrajectory.getPosition(time);
//          positionArray.add(position);
////          velocityArray.add(trapezoidalVelocityTrajectory.getVelocity(time));
//    //         System.out.println(JohnsUsefulFunctions.getFormattedDecimal3D(time) + ", " +
//    //                            JohnsUsefulFunctions.getFormattedDecimal3D(position);
//       }
//
//       PlotGraph2d pg = PlotGraph2d.createPlotGraph2d(timeArray, positionArray);
//       pg.setGraphTitle("aMax = " + accel);
//       pg.plot();
//
////       PlotGraph2d pgVel = PlotGraph2d.createPlotGraph2d(timeArray, velocityArray);
////       pgVel.setGraphTitle("aMax = " + accel);
////       pgVel.plot();
//
//       System.out.println("\n");
//       timeArray.clear();
//       positionArray.clear();
//       velocityArray.clear();
//    }



   }
}
