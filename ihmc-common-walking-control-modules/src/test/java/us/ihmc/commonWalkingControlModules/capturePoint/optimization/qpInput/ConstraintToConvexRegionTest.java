package us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import org.jcodec.common.Assert;
import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.polygonWiggling.PolygonWiggler;

import java.util.ArrayList;
import java.util.Random;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class ConstraintToConvexRegionTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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

         // test no offsets
         constraintToConvexRegion.setPolygon();
         constraintToConvexRegion.formulateConstraint();

         DenseMatrix64F Aineq = new DenseMatrix64F(0, 0);
         DenseMatrix64F bineq = new DenseMatrix64F(0, 0);
         PolygonWiggler.convertToInequalityConstraints(convexPolygon, Aineq, bineq, 0.0);

         Assert.assertTrue(MatrixFeatures.isEquals(Aineq, constraintToConvexRegion.Aineq, 1e-7));
         Assert.assertTrue(MatrixFeatures.isEquals(bineq, constraintToConvexRegion.bineq, 1e-7));

         // test with required distance inside
         constraintToConvexRegion.setDeltaInside(0.02);
         constraintToConvexRegion.formulateConstraint();
         PolygonWiggler.convertToInequalityConstraints(convexPolygon, Aineq, bineq, 0.02);

         Assert.assertTrue(MatrixFeatures.isEquals(Aineq, constraintToConvexRegion.Aineq, 1e-7));
         Assert.assertTrue(MatrixFeatures.isEquals(bineq, constraintToConvexRegion.bineq, 1e-7));


         // test with position offset to the position
         DenseMatrix64F positionOffset = new DenseMatrix64F(2, 1);
         positionOffset.set(0, 0, 0.01);
         positionOffset.set(1, 0, 0.02);
         constraintToConvexRegion.setDeltaInside(0.0);
         constraintToConvexRegion.setPositionOffset(positionOffset);
         constraintToConvexRegion.formulateConstraint();

         PolygonWiggler.convertToInequalityConstraints(convexPolygon, Aineq, bineq, 0.00);
         CommonOps.multAdd(-1.0, Aineq, positionOffset, bineq);

         Assert.assertTrue(MatrixFeatures.isEquals(Aineq, constraintToConvexRegion.Aineq, 1e-7));
         Assert.assertTrue(MatrixFeatures.isEquals(bineq, constraintToConvexRegion.bineq, 1e-7));
      }
   }
}