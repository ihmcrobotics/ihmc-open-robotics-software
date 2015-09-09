package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class IterativeBestFitPlaneCalculatorTest
{
   private static final double eps = 1e-7;

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testEasyCase()
   {
      double [][] data = new double[][]{{1.0,1.0,1.0},{0.0,1.0,1.0},{1.0,0.0,1.0}};
      double[] expectedPlane = new double[]{0.0,0.0,1.0};
      runTest(data, expectedPlane, eps);
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGenericCase()
   {
      Random gen = new Random(1223L);
      for (int i=0;i<2000;i++)
      {
         double[] expectedPlane = new double[]{gen.nextGaussian(),gen.nextGaussian(),gen.nextGaussian()};
         int numPoints = gen.nextInt(12)+3;
         double [][] data = new double[numPoints][3];
         for (int j=0;j<numPoints;j++)
         {
            data[j][0]=gen.nextGaussian();
            data[j][1]=gen.nextGaussian();
            data[j][2]=expectedPlane[2]+expectedPlane[0]*data[j][0]+expectedPlane[1]*data[j][1];
         }
         runTest(data, expectedPlane, eps);
      }
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testEasyDegenerate0Case()
   {
      double [][] data = new double[][]{{1.0,1.0,1.0}};
      double[] expectedPlane = new double[]{0.0,0.0,1.0};
      runTest(data, expectedPlane, eps);
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testEasyDegenerate1XCase()
   {
      double [][] data = new double[][]{{1.0,1.0,1.0},{0.0,1.0,0.0}};
      double[] expectedPlane = new double[]{1.0,0.0,0.0};
      runTest(data, expectedPlane, eps);
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testEasyDegenerate1YCase()
   {
      double [][] data = new double[][]{{1.0,1.0,1.0},{1.0,0.0,0.0}};
      double[] expectedPlane = new double[]{0.0,1.0,0.0};
      runTest(data, expectedPlane, eps);
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testModerateDegenerate1XYCase()
   {
      double [][] data = new double[][]{{1.0,1.0,1.0},{0.0,0.0,0.0}};
      double[] expectedPlane = new double[]{0.5,0.5,0.0};
      runTest(data, expectedPlane, eps);
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSuperEasyDegenerate1XYCase()
   {
      double [][] data = new double[][]{{1.0,1.0,1.0},{0.0,0.0,1.0}};
      double[] expectedPlane = new double[]{0.0,0.0,1.0};
      runTest(data, expectedPlane, eps);
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testDegenerate1Case()
   {
      Random gen = new Random(1223L);
      for (int i=0;i<2000;i++)
      {
         double[] expectedPlane = new double[]{gen.nextGaussian(),gen.nextGaussian(),gen.nextGaussian()};
         int numPoints = gen.nextInt(6)+2;
         double [][] data = new double[numPoints][3];
         for (int j=0;j<numPoints;j++)
         {
            double s = gen.nextGaussian();
            data[j][0]=s*expectedPlane[0];
            data[j][1]=s*expectedPlane[1];
            data[j][2]=expectedPlane[2]+expectedPlane[0]*data[j][0]+expectedPlane[1]*data[j][1];
         }
         runTest(data, expectedPlane, eps);
      }
   }

	/**
	 * Not sure if this ever worked. The nearly degenerate cases are tough. Someone needs to look into it deeper.
	 */
	@Ignore
	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 503)
   public void testNearlyDegenerate1Case()
   {
      Random gen = new Random(1223L);
      for (int i=0;i<2000;i++)
      {
         double[] expectedPlane = new double[]{gen.nextGaussian(),gen.nextGaussian(),gen.nextGaussian()};
         int numPoints = gen.nextInt(12)+7;
         double [][] data = new double[numPoints][3];
         for (int j=0;j<numPoints;j++)
         {
            double s = gen.nextGaussian();
            data[j][0]=s*expectedPlane[0]+gen.nextGaussian()*1e-4;
            data[j][1]=s*expectedPlane[1]+gen.nextGaussian()*1e-4;
            data[j][2]=expectedPlane[2]+expectedPlane[0]*data[j][0]+expectedPlane[1]*data[j][1];
         }
         runTest(data, expectedPlane, 1e-4);
      }
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testUnderDefinedCase()
   {
      Random gen = new Random(1223L);
      for (int i=0;i<2000;i++)
      {
         double[] expectedPlane = new double[]{gen.nextGaussian(),gen.nextGaussian(),gen.nextGaussian()};
         int numPoints = 2;
         double [][] data = new double[numPoints][3];
         for (int j=0;j<numPoints;j++)
         {
            double s = gen.nextGaussian();
            data[j][0]=s*expectedPlane[0];
            data[j][1]=s*expectedPlane[1];
            data[j][2]=expectedPlane[2]+expectedPlane[0]*data[j][0]+expectedPlane[1]*data[j][1];
         }
         runTest(data, expectedPlane, eps);
      }
   }

   private void runTest(double[][] data, double[] expectedPlane, double eps)
   {
      BestFitPlaneDataAccumulator accumulator = new BestFitPlaneDataAccumulator();
      IterativeBestFitPlaneCalculator calculator = new IterativeBestFitPlaneCalculator();
      for (int i=0;i<data.length;i++)
      {
         assertEquals(3,data[i].length);
         accumulator.addPoint(data[i][0], data[i][1], data[i][2]);
      }
      double [] plane = calculator.updatePlane(accumulator);
      assertEquals(3,expectedPlane.length);
      assertEquals(3,plane.length);
      for (int i=0;i<3;i++)
      {
         assertEquals("component "+i,expectedPlane[i],plane[i],eps);
      }
   }

}
