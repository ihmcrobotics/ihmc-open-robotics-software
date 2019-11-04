package us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
import org.ejml.ops.MatrixFeatures;
import org.jcodec.common.Assert;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.commons.RandomNumbers;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;

import java.util.ArrayList;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertTrue;

public class ConstraintToConvexRegionTest
{
   private static final double epsilon = 1e-7;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testConstraintFormulation()
   {
      Random random = new Random(10L);
      ArrayList<FramePoint2D> points = new ArrayList<>();

      for (int iter = 0; iter < 1000; iter++)
      {
         ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
         ConstraintToConvexRegion constraintToConvexRegion = new ConstraintToConvexRegion(4);

         int numberOfCorners = RandomNumbers.nextInt(random, 1, 15);
         for (int i = 0; i < numberOfCorners; i++)
            points.add(new FramePoint2D(ReferenceFrame.getWorldFrame(), RandomNumbers.nextDouble(random, -10.0, 10.0),
                                        RandomNumbers.nextDouble(random, -10.0, 10.0)));

         for (int i = 0; i < numberOfCorners; i++)
         {
            convexPolygon.addVertex(points.get(i));
            constraintToConvexRegion.addVertex(points.get(i));
         }

         convexPolygon.update();

         ConvexPolygonScaler scaler = new ConvexPolygonScaler();
         ConvexPolygon2D scaledPolygon = new ConvexPolygon2D();

         // test no offsets
         constraintToConvexRegion.setPolygon();
         constraintToConvexRegion.formulateConstraint();

         DenseMatrix64F Aineq = new DenseMatrix64F(0, 0);
         DenseMatrix64F bineq = new DenseMatrix64F(0, 0);
         PolygonWiggler.convertToInequalityConstraints(convexPolygon, Aineq, bineq, 0.0);

         EjmlUnitTests.assertEquals(Aineq, constraintToConvexRegion.Aineq, epsilon);
         EjmlUnitTests.assertEquals(bineq, constraintToConvexRegion.bineq, epsilon);

         // test with required distance inside
         constraintToConvexRegion.setDeltaInside(0.02);
         constraintToConvexRegion.formulateConstraint();

         scaler.scaleConvexPolygon(convexPolygon, 0.02, scaledPolygon);

         PolygonWiggler.convertToInequalityConstraints(scaledPolygon, Aineq, bineq, 0.0);

         EjmlUnitTests.assertEquals(Aineq, constraintToConvexRegion.Aineq, epsilon);
         EjmlUnitTests.assertEquals(bineq, constraintToConvexRegion.bineq, epsilon);

         // test with position offset to the position
         DenseMatrix64F positionOffset = new DenseMatrix64F(2, 1);
         positionOffset.set(0, 0, 0.01);
         positionOffset.set(1, 0, 0.02);
         constraintToConvexRegion.setDeltaInside(0.0);
         constraintToConvexRegion.setPositionOffset(positionOffset);
         constraintToConvexRegion.formulateConstraint();

         PolygonWiggler.convertToInequalityConstraints(convexPolygon, Aineq, bineq, 0.00);
         CommonOps.multAdd(-1.0, Aineq, positionOffset, bineq);

         EjmlUnitTests.assertEquals(Aineq, constraintToConvexRegion.Aineq, epsilon);
         EjmlUnitTests.assertEquals(bineq, constraintToConvexRegion.bineq, epsilon);
      }
   }

   @Test
   public void testSquareButWithTooBigADistanceInside()
   {
      ConstraintToConvexRegion constraint = new ConstraintToConvexRegion(50);
      constraint.addVertex(new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.05, 0.05));
      constraint.addVertex(new FramePoint2D(ReferenceFrame.getWorldFrame(), -0.05, 0.05));
      constraint.addVertex(new FramePoint2D(ReferenceFrame.getWorldFrame(), -0.05, -0.05));
      constraint.addVertex(new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.05, -0.05));

      constraint.setPolygon();
      constraint.formulateConstraint();

      DenseMatrix64F point = new DenseMatrix64F(2, 1);
      DenseMatrix64F bineqCalc = new DenseMatrix64F(constraint.bineq.numRows, 1);

      CommonOps.mult(constraint.Aineq, point, bineqCalc);
      for (int i = 0; i < constraint.bineq.numRows; i++)
      {
         assertTrue(bineqCalc.get(i, 0) < constraint.bineq.get(i, 0));
      }

      // by setting this distance to larger than possible, we are making it so that the solution is technically unsolvable. However, the best attempt to solving
      // it is located at the origin, so we should still find a solution there.
      constraint.setDeltaInside(0.06);

      constraint.setPolygon();
      constraint.formulateConstraint();

      point = new DenseMatrix64F(2, 1);
      bineqCalc = new DenseMatrix64F(constraint.bineq.numRows, 1);

      CommonOps.mult(constraint.Aineq, point, bineqCalc);
      for (int i = 0; i < constraint.bineq.numRows; i++)
      {
         assertTrue(bineqCalc.get(i, 0) <= constraint.bineq.get(i, 0));
      }
   }
}