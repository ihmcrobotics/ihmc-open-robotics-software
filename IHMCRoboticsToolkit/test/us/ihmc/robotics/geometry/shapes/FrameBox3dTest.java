package us.ihmc.robotics.geometry.shapes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.Epsilons;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameBox3dTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistance()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      FrameBox3d box = new FrameBox3d(worldFrame);
      FramePoint pointOnZFace = new FramePoint(worldFrame);
      pointOnZFace.set(0.0, 0.0, 2.0);
      double expectedDistance = 1.5;
      assertEquals(expectedDistance, box.distance(pointOnZFace), 1e-14);

      FramePoint pointOnTheEdgeBetweenXandY = new FramePoint(worldFrame);
      pointOnTheEdgeBetweenXandY.set(2.0, 2.0, 0.0);
      expectedDistance = Math.sqrt(2.0) * 1.5;
      assertEquals(expectedDistance, box.distance(pointOnTheEdgeBetweenXandY), 1e-14);

      FramePoint pointOnTheVertexBetweenXandYandZ = new FramePoint(worldFrame);
      pointOnTheVertexBetweenXandYandZ.set(-2.0, -3.5, -5.1);
      expectedDistance = Math.sqrt(MathTools.square(-1.5) + MathTools.square(-3.0) + MathTools.square(-4.6));
      assertEquals(expectedDistance, box.distance(pointOnTheVertexBetweenXandYandZ), 1e-14);

      box = new FrameBox3d(worldFrame, 1, 1, 1);
      pointOnZFace = new FramePoint(worldFrame);
      pointOnZFace.set(0.0, 0.0, 2.0);
      expectedDistance = 1.5;
      assertEquals(expectedDistance, box.distance(pointOnZFace), 1e-14);

      pointOnTheEdgeBetweenXandY = new FramePoint(worldFrame);
      pointOnTheEdgeBetweenXandY.set(2.0, 2.0, 0.0);
      expectedDistance = Math.sqrt(2.0) * 1.5;
      assertEquals(expectedDistance, box.distance(pointOnTheEdgeBetweenXandY), 1e-14);

      pointOnTheVertexBetweenXandYandZ = new FramePoint(worldFrame);
      pointOnTheVertexBetweenXandYandZ.set(-2.0, -3.5, -5.1);
      expectedDistance = Math.sqrt(MathTools.square(-1.5) + MathTools.square(-3.0) + MathTools.square(-4.6));
      assertEquals(expectedDistance, box.distance(pointOnTheVertexBetweenXandYandZ), 1e-14);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testOrthogonalProjection()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      FrameBox3d box = new FrameBox3d(worldFrame);
      FramePoint pointOnXFace = new FramePoint(worldFrame);
      pointOnXFace.set(0.0, 5.2, 0.4);
      FramePoint expectedPoint = new FramePoint(worldFrame);
      expectedPoint.set(0.0, 0.5, 0.4);
      FramePoint projectionPoint = new FramePoint(worldFrame);
      projectionPoint.set(pointOnXFace);
      box.orthogonalProjection(projectionPoint);
      assertTrue(projectionPoint.epsilonEquals(expectedPoint, 1e-14));

      FramePoint pointOnTheEdgeBetweenMinusYandMinusZ = new FramePoint(worldFrame);
      pointOnTheEdgeBetweenMinusYandMinusZ.set(0.0, -2.0, -2.0);
      expectedPoint.set(0.0, -0.5, -0.5);
      projectionPoint.set(pointOnTheEdgeBetweenMinusYandMinusZ);
      box.orthogonalProjection(projectionPoint);
      assertTrue(projectionPoint.epsilonEquals(expectedPoint, 1e-14));

      FramePoint pointOnTheVertexBetweenMinusXandMinusYandZ = new FramePoint(worldFrame);
      pointOnTheVertexBetweenMinusXandMinusYandZ.set(-2.0, -3.5, 5.1);
      expectedPoint.set(-0.5, -0.5, 0.5);
      projectionPoint.set(pointOnTheVertexBetweenMinusXandMinusYandZ);
      box.orthogonalProjection(projectionPoint);
      assertTrue(projectionPoint.epsilonEquals(expectedPoint, 1e-14));

      FramePoint point = new FramePoint(pointOnTheVertexBetweenMinusXandMinusYandZ);
      box.orthogonalProjection(point);
      assertTrue(point.epsilonEquals(expectedPoint, 1e-14));

      @SuppressWarnings("serial")
      ReferenceFrame frame = new ReferenceFrame("testFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            setTransformToParent(transformToParent);
         }
      };
      frame.update();
      box.changeFrame(frame);
      
      assertTrue(box.getReferenceFrame().getName().contains(frame.getName()));

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testgetClosestPointAndNormalAt()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FrameBox3d box = new FrameBox3d(worldFrame);

      FramePoint pointOnXFace = new FramePoint(worldFrame);
      pointOnXFace.set(0.4, 0.3, -0.1);
      FramePoint expectedPoint = new FramePoint(worldFrame);
      expectedPoint.set(0.5, 0.3, -0.1);
      FrameVector expectedNormal = new FrameVector(worldFrame);
      expectedNormal.set(1.0, 0.0, 0.0);
      FramePoint returnedPoint = new FramePoint(worldFrame);
      FrameVector returnedNormal = new FrameVector(worldFrame);
      box.getClosestPointAndNormalAt(returnedPoint, returnedNormal, pointOnXFace);
      assertTrue(expectedPoint.epsilonEquals(returnedPoint, 1e-14));
      assertTrue(expectedNormal.epsilonEquals(returnedNormal, 1e-14));

      FramePoint pointOnYFace = new FramePoint(worldFrame);
      pointOnYFace.set(-0.2, 0.4, -0.1);
      expectedPoint.set(-0.2, 0.5, -0.1);
      expectedNormal.set(0.0, 1.0, 0.0);
      box.getClosestPointAndNormalAt(returnedPoint, returnedNormal, pointOnYFace);
      assertTrue(expectedPoint.epsilonEquals(returnedPoint, 1e-14));
      assertTrue(expectedNormal.epsilonEquals(returnedNormal, 1e-14));
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsInsideOrOnSurface()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FrameBox3d box = new FrameBox3d(worldFrame);

      FramePoint pointOnXFace = new FramePoint(worldFrame);
      pointOnXFace.set(0.4, 0.3, -0.1);
      
      assertTrue(box.isInsideOrOnSurface(pointOnXFace, 1e-7));


      FramePoint pointOnYFace = new FramePoint(worldFrame);
      pointOnYFace.set(-0.2, 0.4, -0.1);
      
      assertTrue(box.isInsideOrOnSurface(pointOnYFace, 1e-7));
      
      FramePoint pointOutsideBox = new FramePoint(worldFrame);
      pointOutsideBox.set(10, 10, 10);
      
      assertFalse(box.isInsideOrOnSurface(pointOutsideBox, 1e-7));
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testApplyTransform()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      Random random = new Random(562346L);
      int nBoxes = 100;
      int nTestsPerBox = 1000;
      for (int boxNumber = 0; boxNumber < nBoxes; boxNumber++)
      {
         FrameBox3d box = createRandomBox(worldFrame, random);
         RigidBodyTransform transform = RigidBodyTransform.generateRandomTransform(random);
         FrameBox3d boxTransformed = new FrameBox3d(box);
         boxTransformed.applyTransform(transform);
         FrameBox3d biggerBox = new FrameBox3d(box);
         biggerBox.scale(2.0);

         Point3d[] vertices = new Point3d[Box3d.NUM_VERTICES];
         for (int i = 0; i < vertices.length; i++)
         {
            vertices[i] = new Point3d();
         }

         biggerBox.computeVertices(vertices);

         for (int testNumber = 0; testNumber < nTestsPerBox; testNumber++)
         {
            Point3d point = getRandomConvexCombination(random, vertices);
            Point3d pointTransformed = new Point3d(point);
            box.getGeometryObject().getTransformToShapeFrameUnsafe().transform(pointTransformed);
            boxTransformed.getGeometryObject().getTransformFromShapeFrameUnsafe().transform(pointTransformed);

            assertEquals(box.isInsideOrOnSurface(new FramePoint(worldFrame, point)),
                         boxTransformed.isInsideOrOnSurface(new FramePoint(worldFrame, pointTransformed)));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetTransform3DAndGetters()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      Random random = new Random(351235L);
      FrameBox3d box = new FrameBox3d(worldFrame);
      RigidBodyTransform transform = RigidBodyTransform.generateRandomTransform(random);
      box.setTransform(transform);

      RigidBodyTransform transformBack = new RigidBodyTransform();
      box.getTransform(transformBack);
      assertTrue(transform.epsilonEquals(transformBack, Epsilons.ONE_TRILLIONTH));
      assertTrue(transform.epsilonEquals(box.getTransformCopy(), Epsilons.ONE_TRILLIONTH));

      Matrix3d matrix = new Matrix3d();
      Vector3d vector = new Vector3d();
      transform.get(matrix, vector);

      Matrix3d matrixBack = new Matrix3d();
      FramePoint CenterBack = new FramePoint(worldFrame);

      box.getRotation(matrixBack);
      assertTrue(matrix.epsilonEquals(matrixBack, Epsilons.ONE_TRILLIONTH));
      assertTrue(matrix.epsilonEquals(box.getRotationCopy(), Epsilons.ONE_TRILLIONTH));

      box.getCenter(CenterBack);
      assertTrue(vector.epsilonEquals(new Vector3d(CenterBack.getVectorCopy()), 0.0));
      assertTrue(vector.epsilonEquals(box.getCenterCopy().getVectorCopy(), 0.0));

      assertTrue(vector.epsilonEquals(new Vector3d(box.getCenter().getVectorCopy()), 0.0));
      assertTrue(vector.epsilonEquals(box.getCenterCopy().getVectorCopy(), 0.0));
   }


   private static FrameBox3d createRandomBox(ReferenceFrame referenceFrame, Random random)
   {
      RigidBodyTransform configuration = RigidBodyTransform.generateRandomTransform(random);
      double lengthX = random.nextDouble();
      double widthY = random.nextDouble();
      double heightZ = random.nextDouble();

      return new FrameBox3d(referenceFrame, configuration, lengthX, widthY, heightZ);
   }

   private static Point3d getRandomConvexCombination(Random random, Point3d[] vertices)
   {
      double[] weightings = new double[vertices.length];
      for (int j = 0; j < weightings.length; j++)
      {
         weightings[j] = random.nextDouble();
      }

      double sum = MathTools.sumDoubles(weightings);
      for (int j = 0; j < weightings.length; j++)
      {
         weightings[j] /= sum;
      }

      Point3d pointToTest = new Point3d();
      for (int j = 0; j < weightings.length; j++)
      {
         Point3d tempPoint = new Point3d(vertices[j]);
         tempPoint.scale(weightings[j]);
         pointToTest.add(tempPoint);
      }

      return pointToTest;
   }


}
