package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.math.trajectories.NDoFTrapezoidalVelocityTrajectory.AlphaToAlphaType;


public class NDoFTrapezoidalVelocityTrajectoryTest
{
   /*
    * No actual tests yet, just visualizers.
    */


   public NDoFTrapezoidalVelocityTrajectoryTest()
   {
      // empty
   }

   @Before
   public void setUp() throws Exception
   {
   }

   @After
   public void tearDown() throws Exception
   {
   }

   public void DONTtestOneTrajectory()
   {
      double t0 = 3.0;
      double[] x0 = new double[] {0.0};
      double[] xF = new double[] {1.0};
      double[] v0 = new double[] {0.0};
      double[] vF = new double[] {10.0};
      double[] vMax = new double[] {10.01};
      double[] aMax = new double[] {1.01};

      NDoFTrapezoidalVelocityTrajectory nDtrap = new NDoFTrapezoidalVelocityTrajectory(t0, x0, xF, v0, vF, vMax, aMax, AlphaToAlphaType.STRETCH_IN_MIDDLE);
      System.out.println("tFMax = " + nDtrap.getTFMax());

      double dT = 2.5e-1;
      double tMax = 60.0;
      int numberOfPoints = (int) (tMax / dT);

      double[] time = new double[numberOfPoints];
      double[][] data = new double[2][numberOfPoints];

      for (int i = 0; i < numberOfPoints; i++)
      {
         double t = t0 + i * dT;

         // Save trajectory:
         time[i] = t;
         data[0][i] = nDtrap.getPosition(0, t);
         data[1][i] = nDtrap.getVelocity(0, t);
      }

      plot(time, data, "Position and velocity for multiple trajectories");
   }

   public void DONTtestMultipleTrajectories()
   {
      double t0 = 1.0;
      double[] x0 = new double[] {0.0, 1.0, -1.0};
      double[] xF = new double[] {1.0, -5.0, 0.0};
      double[] v0 = new double[] {0.0, 0.0, -5.0};
      double[] vF = new double[] {10.0, 0.0, -5.0};
      double[] vMax = new double[] {10.01, 3.0, 6.0};
      double[] aMax = new double[] {1.01, 2.0, 4.0};

      int numberOfTrajectories = x0.length;

      NDoFTrapezoidalVelocityTrajectory nDtrap = new NDoFTrapezoidalVelocityTrajectory(t0, x0, xF, v0, vF, vMax, aMax, AlphaToAlphaType.STRETCH_IN_MIDDLE);
      System.out.println("tFMax = " + nDtrap.getTFMax());

      double dT = 2.5e-1;
      double tMax = 60.0;
      int numberOfPoints = (int) (tMax / dT);

      double[] time = new double[numberOfPoints];
      double[][] positions = new double[numberOfTrajectories][numberOfPoints];

      for (int i = 0; i < numberOfPoints; i++)
      {
         double t = t0 + i * dT;

         // Save trajectory:
         time[i] = t;

         for (int j = 0; j < numberOfTrajectories; j++)
         {
            positions[j][i] = nDtrap.getPosition(j, t);
         }
      }

      plot(time, positions, "Position for multiple trajectories");
   }

   public void DONTtestMultipleTrajectoriesSimple()
   {
      double t0 = 1.0;
      double[] x0 = new double[] {0.0, 1.0, -1.0};
      double[] xF = new double[] {1.0, -1.0, 0.0};
      double[] v0 = new double[] {0.0, 0.0, 0.0};
      double[] vF = new double[] {0.0, 0.0, 0.0};
      double[] vMax = new double[] {1.0, 3.0, 6.0};
      double[] aMax = new double[] {1.0, 2.0, 4.0};

      int numberOfTrajectories = x0.length;

      NDoFTrapezoidalVelocityTrajectory nDtrap = new NDoFTrapezoidalVelocityTrajectory(t0, x0, xF, v0, vF, vMax, aMax, AlphaToAlphaType.STRETCH_IN_MIDDLE);
      System.out.println("tFMax = " + nDtrap.getTFMax());

      double dT = 2.5e-2;
      double tMax = 10.0;
      int numberOfPoints = (int) (tMax / dT);

      double[] time = new double[numberOfPoints];
      double[][] positions = new double[numberOfTrajectories][numberOfPoints];

      for (int i = 0; i < numberOfPoints; i++)
      {
         double t = t0 + i * dT;

         // Save trajectory:
         time[i] = t;

         for (int j = 0; j < numberOfTrajectories; j++)
         {
            positions[j][i] = nDtrap.getPosition(j, t);
         }
      }

      plot(time, positions, "Position for multiple trajectories (simple)");
   }

   @Ignore
	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout=300000)
   public void testVelocityAndAcceleration()
   {
      double t0 = 1.0;
      double[] x0 = new double[] {0.0, 1.0};
      double[] xF = new double[] {1.0, -1.0};
      double[] v0 = new double[] {0.0, 0.0};
      double[] vF = new double[] {0.0, 0.0};
      double[] vMax = new double[] {1.0, 1.0};
      double[] aMax = new double[] {1.0, 1.0};

      int numberOfTrajectories = x0.length;

      NDoFTrapezoidalVelocityTrajectory nDtrap = new NDoFTrapezoidalVelocityTrajectory(t0, x0, xF, v0, vF, vMax, aMax, AlphaToAlphaType.STRETCH_IN_MIDDLE);
      System.out.println("tFMax = " + nDtrap.getTFMax());
      System.out.println("tF0: " + nDtrap.getTFArray()[0]);
      System.out.println("tF1: " + nDtrap.getTFArray()[1]);


      nDtrap.synchronize();

      double dT = 2.5e-2;
      double tMax = 10.0;
      int numberOfPoints = (int) (tMax / dT);

      double[] time = new double[numberOfPoints];
      double[][][] positions = new double[2][numberOfTrajectories][numberOfPoints];
      double[][][] velocities = new double[2][numberOfTrajectories][numberOfPoints];
      double[][][] accelerations = new double[2][numberOfTrajectories][numberOfPoints];

      for (int i = 0; i < numberOfPoints; i++)
      {
         double t = t0 + i * dT;

         // Save trajectory:
         time[i] = t;

         for (int j = 0; j < numberOfTrajectories; j++)
         {
            positions[0][j][i] = nDtrap.getPosition(j, t);
            velocities[0][j][i] = nDtrap.getVelocity(j, t);
            accelerations[0][j][i] = nDtrap.getAcceleration(j, t);
         }
      }

      for (int i = 0; i < positions[0].length; i++)
      {
         velocities[1][i] = getNumericalDerivative(positions[0][i], dT);
         accelerations[1][i] = getNumericalDerivative(velocities[0][i], dT);
      }


      plot(time, positions[0], "Position for two ");
      plot(time, velocities[0], "Velocity for two ");
      plot(time, velocities[1], "Velocity for two ");
      plot(time, accelerations[0], "Acceleration for two ");
      plot(time, accelerations[1], "Acceleration for two ");

      sleepForever();

   }


   private double[] getNumericalDerivative(double[] array, double stepSize)
   {
      int n = array.length;
      double[] ret = new double[n];
      for (int i = 0; i < (n - 1); i++)
      {
         ret[i] = (array[i + 1] - array[i]) / stepSize;
      }

      ret[n - 1] = Double.NaN;

      return ret;
   }




   private void plot(double[] abscissa, double[][] ordinate, String name)
   {
      double numberOfTrajectories = ordinate.length;

      ArrayList<double[][]> listOfCurves1 = new ArrayList<double[][]>();
      for (int i = 0; i < numberOfTrajectories; i++)
      {
         listOfCurves1.add(new double[][]
         {
            abscissa, ordinate[i]
         });
      }
      
//      PlotGraph2d pg1 = PlotGraph2d.createPlotGraph2dMultipleCurves(listOfCurves1);
//      pg1.plot();
//      pg1.setGraphTitle(name);

   }

   private void sleepForever()
   {
      while (true)
      {
         try
         {
            Thread.sleep(1000);
         }
         catch (InterruptedException ex)
         {
         }
      }
   }
}
