package us.ihmc.humanoidOperatorInterface.pointClouds.combinationQuadTreeOctTree;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.fail;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.DataGridTools;
import us.ihmc.robotics.dataStructures.DoubleHashHeightMap;
import us.ihmc.robotics.dataStructures.HeightMapWithPoints;
import us.ihmc.robotics.geometry.HeightMapBestFitPlaneCalculator;
import us.ihmc.robotics.geometry.InsufficientDataException;

public class BestFitPlaneCalculatorTest
{
   private static final double eps = 1e-7;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

	@Test
   public void basicBestFitPlaneCalculatorTest() throws InsufficientDataException
   {
      double gridResolution = 1;
      HeightMapWithPoints map = new DoubleHashHeightMap(gridResolution);
      FramePoint2D footCenterPoint = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.0, 0.0);
      DMatrixRMaj matrix = new DMatrixRMaj(7, 7);
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
      Plane3D plane = calculator.calculatePlane(map, footCenterPoint, 2.0, 2.0);
      System.out.println("BestFitPlaneCalculatorTest: calculator.getPointList() = " + calculator.getPointList());
      Point3D point = new Point3D(plane.getPoint());
      Vector3D normal = new Vector3D(plane.getNormal());
      assertEquals(1.0, point.getZ(), eps);
      assertEquals(footCenterPoint.getX(), point.getX(), eps);
      assertEquals(footCenterPoint.getY(), point.getY(), eps);
      assertEquals(-Math.sqrt(2) / 2.0, normal.getX(), eps);
      assertEquals(0.0, normal.getY(), eps);
      assertEquals(Math.sqrt(2) / 2.0, normal.getZ(), eps);

   }  

	@Test
   public void basicBestFitPlaneCalculatorNaNTest() throws InsufficientDataException
   {
      double gridResolution = 1;
      HeightMapWithPoints map = new DoubleHashHeightMap(gridResolution);
      double n = Double.NaN;
      DMatrixRMaj matrix = new DMatrixRMaj(7, 7);
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
      Plane3D plane = calculator.calculatePlane(map, new Point2D(xyCenter, xyCenter), 2, 2);
      Point3D point = new Point3D(plane.getPoint());
      Vector3D normal = new Vector3D(plane.getNormal());
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
