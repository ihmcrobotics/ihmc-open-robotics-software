package us.ihmc.robotics.math.trajectories;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.yoVariables.registry.YoRegistry;

import static us.ihmc.robotics.Assert.assertEquals;

public class YoConfigurablePolynomialTest
{
   private final static double epsilon = 1e-5;

   @Test
   public void test8DoF()
   {
      YoConfigurablePolynomial polynomial = new YoConfigurablePolynomial("test", 8, new YoRegistry("test"));

      Point2D point0 = new Point2D(0.007, 0.919);
      Vector2D slope0 = new Vector2D(0.007, 0.0);
      Point2D point1 = new Point2D(0.127, 0.901);
      Point2D point2 = new Point2D(0.287, 0.877);
      Point2D point3 = new Point2D(0.407, 0.860);
      Point2D point4 = new Point2D(0.527, 0.842);
      Point2D point5 = new Point2D(0.687, 0.818);
      Point2D point6 = new Point2D(0.807, 0.800);
      polynomial.reshape(8);
      polynomial.addPositionConstraint(point0.getX(), point0.getY());
      polynomial.addVelocityConstraint(slope0.getX(), slope0.getY());
      polynomial.addPositionConstraint(point1.getX(), point1.getY());
      polynomial.addPositionConstraint(point2.getX(), point2.getY());
      polynomial.addPositionConstraint(point3.getX(), point3.getY());
      polynomial.addPositionConstraint(point4.getX(), point4.getY());
      polynomial.addPositionConstraint(point5.getX(), point5.getY());
      polynomial.addPositionConstraint(point6.getX(), point6.getY());

      polynomial.solve();

      polynomial.compute(point0.getX());
      assertEquals(point0.getY(), polynomial.getPosition(), epsilon);
      assertEquals(slope0.getY(), polynomial.getVelocity(), epsilon);

      polynomial.compute(point1.getX());
      assertEquals(point1.getY(), polynomial.getPosition(), epsilon);

      polynomial.compute(point2.getX());
      assertEquals(point2.getY(), polynomial.getPosition(), epsilon);


      polynomial.compute(point3.getX());
      assertEquals(point3.getY(), polynomial.getPosition(), epsilon);

      polynomial.compute(point4.getX());
      assertEquals(point4.getY(), polynomial.getPosition(), epsilon);

      polynomial.compute(point5.getX());
      assertEquals(point5.getY(), polynomial.getPosition(), epsilon);

      polynomial.compute(point6.getX());
      assertEquals(point6.getY(), polynomial.getPosition(), epsilon);
   }


   @Test
   public void testTrickyCase()
   {
      YoConfigurablePolynomial polynomial = new YoConfigurablePolynomial("test", 8, new YoRegistry("test"));

      Point2D point0 = new Point2D(0.5716, 0.919);
//      Vector2D slope0 = new Vector2D(0.007, 0.0);
      Point2D point1 = new Point2D(0.6997, 0.8441);
      Point2D point2 = new Point2D(0.8705, 0.8978);
      Point2D point3 = new Point2D(0.9986, 0.9221);
      Point2D point4 = new Point2D(0.9994, 0.9221);
      Point2D point5 = new Point2D(1.0004, 0.9316);
      Point2D point6 = new Point2D(1.0011, 0.9388);
      polynomial.reshape(7);
      polynomial.addPositionConstraint(point0.getX(), point0.getY());
//      polynomial.addVelocityConstraint(slope0.getX(), slope0.getY());
      polynomial.addPositionConstraint(point1.getX(), point1.getY());
      polynomial.addPositionConstraint(point2.getX(), point2.getY());
      polynomial.addPositionConstraint(point3.getX(), point3.getY());
      polynomial.addPositionConstraint(point4.getX(), point4.getY());
      polynomial.addPositionConstraint(point5.getX(), point5.getY());
      polynomial.addPositionConstraint(point6.getX(), point6.getY());

      polynomial.solve();

      polynomial.compute(point0.getX());
      assertEquals(point0.getY(), polynomial.getPosition(), epsilon);
//      assertEquals(slope0.getY(), polynomial.getVelocity(), epsilon);

      polynomial.compute(point1.getX());
      assertEquals(point1.getY(), polynomial.getPosition(), epsilon);

      polynomial.compute(point2.getX());
      assertEquals(point2.getY(), polynomial.getPosition(), epsilon);


      polynomial.compute(point3.getX());
      assertEquals(point3.getY(), polynomial.getPosition(), epsilon);

      polynomial.compute(point4.getX());
      assertEquals(point4.getY(), polynomial.getPosition(), epsilon);

      polynomial.compute(point5.getX());
      assertEquals(point5.getY(), polynomial.getPosition(), epsilon);

      polynomial.compute(point6.getX());
      assertEquals(point6.getY(), polynomial.getPosition(), epsilon);
   }
}
