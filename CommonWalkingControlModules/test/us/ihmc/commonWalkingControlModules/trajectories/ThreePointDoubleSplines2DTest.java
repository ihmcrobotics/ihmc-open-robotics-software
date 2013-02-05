package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.*;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import org.junit.Test;

public class ThreePointDoubleSplines2DTest
{
   @Test
   public void testSimpleFlatExample()
   {
      ThreePointDoubleSplines2D spline = new ThreePointDoubleSplines2D();
      
      double zHeight = 0.23;
      
      Point3d pointOne = new Point3d(0.0, 0.0, zHeight);
      Point3d pointTwo = new Point3d(1.0, 0.0, zHeight);
      Point3d pointThree = new Point3d(2.0, 0.0, zHeight);
      
      spline.setPoints(pointOne, pointTwo, pointThree);
      
      Point2d queryPoint = new Point2d();
      
      double z = spline.getZ(queryPoint);
      
      assertEquals(zHeight, z, 1e-7);
   }
   
   @Test
   public void testPerfectLineExample()
   {
      ThreePointDoubleSplines2D spline = new ThreePointDoubleSplines2D();
      
      double zHeight = 0.23;
      double slope = 1.1;
      
      Point3d pointOne = new Point3d(0.0, 0.0, zHeight);
      Point3d pointTwo = new Point3d(1.0, 0.0, zHeight + slope);
      Point3d pointThree = new Point3d(2.0, 0.0, zHeight + 2.0 * slope);
      
      spline.setPoints(pointOne, pointTwo, pointThree);
      
      Point2d queryPoint = new Point2d();
      
      double z = spline.getZ(queryPoint);
      
      assertEquals(zHeight, z, 1e-7);
   }

}
