package us.ihmc.humanoidOperatorInterface.pointClouds.combinationQuadTreeOctTree;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidOperatorInterface.dataStructures.DataGridTools;
import us.ihmc.humanoidOperatorInterface.dataStructures.DoubleHashHeightMap;
import us.ihmc.humanoidOperatorInterface.dataStructures.HeightMapWithPoints;
import us.ihmc.robotics.geometry.InclusionFunction;
import us.ihmc.robotics.geometry.InsufficientDataException;

import static org.junit.jupiter.api.Assertions.*;

public abstract class AbstractHeightMapTest
{

   private static final int BIG_NUMBER = 1000;
   private static final int MAX_X = 500;
   private static final int NUMBER_OF_POINTS_TO_WRITE = 400000;
   private static final int NUMBER_OF_TIMES_TO_SAMPLE = 400;
   private static final int NUMBER_OF_SYNCHRONOUS_PAIRS = 8;
   private static final double RESOLUTION = 0.1;
   private static final double eps = 1e-7;
   private static final double NaN = Double.NaN;

   public AbstractHeightMapTest()
   {
      super();
   }

	@Test
   public void testSinglePoint()
   {
      double x = 0.00001;
      double y = 0.00001;
      int xIndex = 0;
      int yIndex = 0;
      double z = 5.25;
      assertSinglePointGridHandlesPoint(x, y, xIndex, yIndex, z);
      x = 0.00001;
      xIndex = 0;
      z = 4;
      assertSinglePointGridHandlesPoint(x, y, xIndex, yIndex, z);
      x = -0.00001;
      xIndex = 0;
      z = 4;
      assertSinglePointGridHandlesPoint(x, y, xIndex, yIndex, z);
      x = -28.00001;
      xIndex = -280;
      z = 2;
      assertSinglePointGridHandlesPoint(x, y, xIndex, yIndex, z);
      x = -28.00001;
      xIndex = -280;
      z = -21234;
      assertSinglePointGridHandlesPoint(x, y, xIndex, yIndex, z);
      y = -28.00001;
      yIndex = -280;
      assertSinglePointGridHandlesPoint(x, y, xIndex, yIndex, z);
   }

	@Test
   public void testGettingAreas()
   {
      double gridResolution = 1;
      HeightMapWithPoints map = getHeightMap(-2, -2, 2, 2, gridResolution);
      DMatrixRMaj matrix = new DMatrixRMaj(7, 7);
      matrix.setData(new double[] { 
            0, 0, 0, 0, 3, 0, 0, 
            0, 0, 8, 8, 0, 0, 0, 
            0, 0, 0, 0, 0, 1, 0, 
            0, 8, 1, 1, 1, 0, 0, 
            0, 0, 2, 2, 2, 0, 0, 
            0, 0, 0, 0, 8, 1, 0, 
            0, 0, 0, 0, 0, 0, 0 });
      DataGridTools.fillMapWithMatrixCentered(map, matrix, gridResolution);
      List<Point3D> points = map.getAllPointsWithinArea(0.0, 0.0, 2, 2);
      System.out.println("DoubleHashHeightMapTest points" + points);
      assertEquals(9, points.size());
      assertEquals(-1, points.get(0).getX(), eps);
      assertEquals(-1, points.get(1).getX(), eps);
      assertEquals(-1, points.get(2).getX(), eps);
      assertEquals(0, points.get(3).getX(), eps);
      assertEquals(0, points.get(4).getX(), eps);
      assertEquals(0, points.get(5).getX(), eps);
      assertEquals(1, points.get(6).getX(), eps);
      assertEquals(1, points.get(7).getX(), eps);
      assertEquals(1, points.get(8).getX(), eps);
      assertEquals(-1, points.get(0).getY(), eps);
      assertEquals(0, points.get(1).getY(), eps);
      assertEquals(1, points.get(2).getY(), eps);
      assertEquals(-1, points.get(3).getY(), eps);
      assertEquals(0, points.get(4).getY(), eps);
      assertEquals(1, points.get(5).getY(), eps);
      assertEquals(-1, points.get(6).getY(), eps);
      assertEquals(0, points.get(7).getY(), eps);
      assertEquals(1, points.get(8).getY(), eps);
      assertEquals(0, points.get(0).getZ(), eps);
      assertEquals(0, points.get(1).getZ(), eps);
      assertEquals(0, points.get(2).getZ(), eps);
   }
   
   
   
   
   
   
   /* To use test unhandled points include this.
    * 

	@DeployableTestMethod(estimatedDuration = 0.1)
	@Test
   public void testUnhandledPoints()
   {
      super.testUnhandledPoints();
   }
   */

   @Test
   public void testUnhandledPoints()
   {
      HeightMapWithPoints simpleMap = getHeightMap(-63.5*RESOLUTION,-63.5*RESOLUTION,64.5*RESOLUTION,64.5*RESOLUTION,RESOLUTION);
      double x = 0.00001;
      double y = 0.00001;
      double z = 5.25;
      simpleMap.addPoint(x, y, z);
      y = 1.03001;
      z = 2.25;
      simpleMap.addPoint(x, y, z);
      y = 1.33001;
      simpleMap.addPoint(x, y, z);
      y = 2.33001;
      simpleMap.addPoint(x, y, z);
      y = 3.33001;
      simpleMap.addPoint(x, y, z);
      x = 3.00001;
      y = 1.00001;
      z = -50.25;
      simpleMap.addPoint(x, y, z);
      x = 2.00001;
      z = -24.25;
      simpleMap.addPoint(x, y, z);
      x = -22.00001;
      simpleMap.addPoint(x, y, z);
      assertFalse(simpleMap.containsPoint(-22222, 22222));
      assertEquals(NaN, simpleMap.getHeightAtPoint(0, -1), eps);
      assertEquals(NaN, simpleMap.getHeightAtPoint(1, 1), eps);
      testOverridingOrigin(simpleMap);
   
   }


   protected void testOverridingOrigin(HeightMapWithPoints simpleMap)
   {
      double x;
      double y;
      double z;
      x = 0.0499;
      y = 0.0499;
      z = 4;
      simpleMap.addPoint(x, y, z);
      assertEquals(4, simpleMap.getHeightAtPoint(0, 0), eps);
   }

   public void assertSinglePointGridHandlesPoint(double x, double y, int xIndex, int yIndex, double z)
   {
      HeightMapWithPoints simpleMap = getHeightMap(-BIG_NUMBER,-BIG_NUMBER,BIG_NUMBER,BIG_NUMBER,RESOLUTION);
      simpleMap.addPoint(x, y, z);// new QuadTreeHeightMap(-3,-3,3,3,gridResolution,HEIGHT_THRESHOLD);
      assertTrue(simpleMap.containsPoint(x, y));
      assertTrue(simpleMap.containsPoint(xIndex * RESOLUTION, yIndex * RESOLUTION));
      assertEquals(z, simpleMap.getHeightAtPoint(x, y), eps);
      assertEquals(z, simpleMap.getHeightAtPoint(xIndex * RESOLUTION, yIndex * RESOLUTION), eps);
   }
   public abstract HeightMapWithPoints getHeightMap(double minX, double minY, double maxX, double maxY, double resolution);

	@Test
   public void rowModificationSynchronizationTest()
   {
   
      final HeightMapWithPoints simpleMap = new DoubleHashHeightMap(RESOLUTION);
      ExecutorService service = Executors.newCachedThreadPool();
      ArrayList<Future<String>> dataWriters = new ArrayList<Future<String>>();
      ArrayList<Future<String>> dataReaders = new ArrayList<Future<String>>();
      for (int i = 0; i < NUMBER_OF_SYNCHRONOUS_PAIRS; i++)
      {
         dataWriters.add(createWriter(simpleMap, service));
         dataReaders.add(createReader(simpleMap, service));
      }
      try
      {
         Thread.sleep(2000);
      }
      catch (InterruptedException e)
      {
      }
      for (int i = 0; i < NUMBER_OF_SYNCHRONOUS_PAIRS; i++)
      {
         handleFuture(dataWriters.get(i));
         handleFuture(dataReaders.get(i));
      }
   }

   public Future<String> createReader(final HeightMapWithPoints simpleMap, ExecutorService service)
   {
      Future<String> dataInclusionFuture = service.submit(new Callable<String>()
      {
         public String call() throws Exception
         {
            try
            {
               for (int i = 0; i < NUMBER_OF_TIMES_TO_SAMPLE; i++)
               {
                  simpleMap.getAllPointsWithinArea(-250 + 0.5 * MAX_X, 0, 250 + 0.5 * MAX_X, 400);
               }
            }
            catch (Throwable e)
            {
               return e.getMessage();
            }
   
            return "";
         }
      });
      return dataInclusionFuture;
   }

   public Future<String> createWriter(final HeightMapWithPoints simpleMap, ExecutorService service)
   {
      Future<String> dataInclusionFuture = service.submit(new Callable<String>()
      {
         public String call() throws Exception
         {
            Random gen = new Random(-8192376L);
            try
            {
               for (int i = 0; i < NUMBER_OF_POINTS_TO_WRITE; i++)
               {
                  simpleMap.addPoint(gen.nextInt(MAX_X), gen.nextInt(10), 1.337);
               }
            }
            catch (Throwable e)
            {
               e.printStackTrace();
               return e.getMessage();
            }
   
            return "";
         }
      });
      return dataInclusionFuture;
   }

   public void handleFuture(Future<String> dataInclusionFuture)
   {
      try
      {
         String resultString = dataInclusionFuture.get();
         assertTrue(dataInclusionFuture.isDone());
         assertTrue("".equals(resultString), "result string not empty: is " + resultString);
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
         fail();
      }
      catch (ExecutionException e)
      {
         e.printStackTrace();
         fail();
      }
   }

	@Test
   public void testKernelMasking() throws InsufficientDataException
   {
      double b = 10000;    // borderOfExpectedFootPlacement
       double gridResolution = 0.01;
      DMatrixRMaj matrix = new DMatrixRMaj(11, 11);
      matrix.setData(new double[]
      {
         b, b, b, b, b, b, b, b, b, b, b, 
         b, b, b, 0, 0, 0, 0, 0, b, b, b, 
         b, b, b, 0, 0, 0, 0, 0, b, b, b, 
         b, b, b, 0, 0, 0, 0, 0, b, b, b, 
         b, b, b, 0, 0, 0, 0, 0, b, b, b, 
         b, b, b, 0, 0, 0, 0, 0, b, b, b, 
         b, b, b, 0, 0, 0, 0, 0, b, b, b, 
         b, b, b, 0, 0, 0, 0, 0, b, b, b, 
         b, b, b, 0, 0, 0, 0, 0, b, b, b, 
         b, b, b, 0, 0, 0, 0, 0, b, b, b, 
         b, b, b, b, b, b, b, b, b, b, b,
      });
      HeightMapWithPoints lidarMap = new DoubleHashHeightMap(gridResolution);
      DataGridTools.fillMapWithMatrixCentered(lidarMap, matrix, gridResolution);
      List<Point3D> points = lidarMap.getAllPointsWithinArea(0.0, 0.0, 0.1, 0.1, new InclusionFunction<Point3D>(){
   
         public boolean isIncluded(Point3D test)
         {
            return test.getZ()<20;
         }
         
      });
      assertEquals(45,points.size());
   }

}
