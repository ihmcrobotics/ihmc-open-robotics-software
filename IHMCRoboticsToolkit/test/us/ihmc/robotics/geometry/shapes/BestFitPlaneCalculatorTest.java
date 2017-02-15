package us.ihmc.robotics.geometry.shapes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.DataGridTools;
import us.ihmc.robotics.dataStructures.DoubleHashHeightMap;
import us.ihmc.robotics.dataStructures.HeightMapWithPoints;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.HeightMapBestFitPlaneCalculator;
import us.ihmc.robotics.geometry.InsufficientDataException;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class BestFitPlaneCalculatorTest
{
   private static final double eps = 1e-7;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void basicBestFitPlaneCalculatorTest() throws InsufficientDataException
   {
      double gridResolution = 1;
      HeightMapWithPoints map = new DoubleHashHeightMap(gridResolution);
      FramePoint2d footCenterPoint = new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, 0.0);
      DenseMatrix64F matrix = new DenseMatrix64F(7, 7);
      matrix.setData(new double[]{0, 0, 0, 0, 3, 0, 0,
                                  0, 0, 8, 8, 0, 0, 0,
                                  0, 0, 0, 0, 0, 1, 0,
                                  0, 8, 1, 1, 1, 0, 0,
                                  0, 0, 2, 2, 2, 0, 0,
                                  0, 0, 0, 0, 8, 1, 0,
                                  0, 0, 0, 0, 0, 0, 0}
      );
      DataGridTools.fillMapWithMatrixCentered(map, matrix, gridResolution);
      HeightMapBestFitPlaneCalculator calculator = new HeightMapBestFitPlaneCalculator();
      Plane3d plane = calculator.calculatePlane(map, footCenterPoint.getPoint(), 2.0, 2.0);
      System.out.println("BestFitPlaneCalculatorTest: calculator.getPointList() = " + calculator.getPointList());
      Point3d point = plane.getPointCopy();
      Vector3d normal = plane.getNormalCopy();
      assertEquals(1.0, point.getZ(), eps);
      assertEquals(footCenterPoint.getX(), point.getX(), eps);
      assertEquals(footCenterPoint.getY(), point.getY(), eps);
      assertEquals(-Math.sqrt(2) / 2.0, normal.getX(), eps);
      assertEquals(0.0, normal.getY(), eps);
      assertEquals(Math.sqrt(2) / 2.0, normal.getZ(), eps);

   }  

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void basicBestFitPlaneCalculatorNaNTest() throws InsufficientDataException
   {
      double gridResolution = 1;
      HeightMapWithPoints map = new DoubleHashHeightMap(gridResolution);
      double n = Double.NaN;
      DenseMatrix64F matrix = new DenseMatrix64F(7, 7);
      matrix.setData(new double[]{0, 0, 0, 0, 3, 0, 0,
                                  0, 0, 0, 0, 0, 0, 0,
                                  0, 0, 0, n, 0, 1, 0,
                                  0, 0, 1, n, 1, 0, 0,
                                  0, 0, 2, n, 2, 0, 0,
                                  0, 0, 0, 0, 0, 1, 0,
                                  0, 0, 0, 0, 0, 0, 0});

      DataGridTools.fillMapWithMatrix(map, matrix, gridResolution);
      HeightMapBestFitPlaneCalculator calculator = new HeightMapBestFitPlaneCalculator();
      double xyCenter = 3.0;
      Plane3d plane = calculator.calculatePlane(map, new Point2d(xyCenter, xyCenter), 2, 2);
      Point3d point = plane.getPointCopy();
      Vector3d normal = plane.getNormalCopy();
      if (!(MathTools.epsilonEquals(normal.getX(), Math.sqrt(2) / 2.0, 1e-7) && MathTools.epsilonEquals(normal.getZ(), -Math.sqrt(2) / 2.0, 1e-7))
            && !(MathTools.epsilonEquals(normal.getX(), -Math.sqrt(2) / 2.0, 1e-7) && MathTools.epsilonEquals(normal.getZ(), Math.sqrt(2) / 2.0, 1e-7)))
      {
         fail();
      }
      assertEquals(0.0, normal.getY(), 1e-7);
      assertEquals(xyCenter, point.getX(), 1e-7);
      assertEquals(xyCenter, point.getY(), 1e-7);
      assertEquals(1.0, point.getZ(), 1e-7);
   }
}
