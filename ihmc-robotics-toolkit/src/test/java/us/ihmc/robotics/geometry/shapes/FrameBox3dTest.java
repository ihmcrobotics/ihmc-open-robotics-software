package us.ihmc.robotics.geometry.shapes;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class FrameBox3dTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testDistance()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      FrameBox3d box = new FrameBox3d(worldFrame);
      FramePoint3D pointOnZFace = new FramePoint3D(worldFrame);
      pointOnZFace.set(0.0, 0.0, 2.0);
      double expectedDistance = 1.5;
      assertEquals(expectedDistance, box.distance(pointOnZFace), 1e-14);

      FramePoint3D pointOnTheEdgeBetweenXandY = new FramePoint3D(worldFrame);
      pointOnTheEdgeBetweenXandY.set(2.0, 2.0, 0.0);
      expectedDistance = Math.sqrt(2.0) * 1.5;
      assertEquals(expectedDistance, box.distance(pointOnTheEdgeBetweenXandY), 1e-14);

      FramePoint3D pointOnTheVertexBetweenXandYandZ = new FramePoint3D(worldFrame);
      pointOnTheVertexBetweenXandYandZ.set(-2.0, -3.5, -5.1);
      expectedDistance = Math.sqrt(MathTools.square(-1.5) + MathTools.square(-3.0) + MathTools.square(-4.6));
      assertEquals(expectedDistance, box.distance(pointOnTheVertexBetweenXandYandZ), 1e-14);

      box = new FrameBox3d(worldFrame, 1, 1, 1);
      pointOnZFace = new FramePoint3D(worldFrame);
      pointOnZFace.set(0.0, 0.0, 2.0);
      expectedDistance = 1.5;
      assertEquals(expectedDistance, box.distance(pointOnZFace), 1e-14);

      pointOnTheEdgeBetweenXandY = new FramePoint3D(worldFrame);
      pointOnTheEdgeBetweenXandY.set(2.0, 2.0, 0.0);
      expectedDistance = Math.sqrt(2.0) * 1.5;
      assertEquals(expectedDistance, box.distance(pointOnTheEdgeBetweenXandY), 1e-14);

      pointOnTheVertexBetweenXandYandZ = new FramePoint3D(worldFrame);
      pointOnTheVertexBetweenXandYandZ.set(-2.0, -3.5, -5.1);
      expectedDistance = Math.sqrt(MathTools.square(-1.5) + MathTools.square(-3.0) + MathTools.square(-4.6));
      assertEquals(expectedDistance, box.distance(pointOnTheVertexBetweenXandYandZ), 1e-14);
   }

   @Test
   public void testOrthogonalProjection()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      FrameBox3d box = new FrameBox3d(worldFrame);
      FramePoint3D pointOnXFace = new FramePoint3D(worldFrame);
      pointOnXFace.set(0.0, 5.2, 0.4);
      FramePoint3D expectedPoint = new FramePoint3D(worldFrame);
      expectedPoint.set(0.0, 0.5, 0.4);
      FramePoint3D projectionPoint = new FramePoint3D(worldFrame);
      projectionPoint.set(pointOnXFace);
      box.orthogonalProjection(projectionPoint);
      assertTrue(projectionPoint.epsilonEquals(expectedPoint, 1e-14));

      FramePoint3D pointOnTheEdgeBetweenMinusYandMinusZ = new FramePoint3D(worldFrame);
      pointOnTheEdgeBetweenMinusYandMinusZ.set(0.0, -2.0, -2.0);
      expectedPoint.set(0.0, -0.5, -0.5);
      projectionPoint.set(pointOnTheEdgeBetweenMinusYandMinusZ);
      box.orthogonalProjection(projectionPoint);
      assertTrue(projectionPoint.epsilonEquals(expectedPoint, 1e-14));

      FramePoint3D pointOnTheVertexBetweenMinusXandMinusYandZ = new FramePoint3D(worldFrame);
      pointOnTheVertexBetweenMinusXandMinusYandZ.set(-2.0, -3.5, 5.1);
      expectedPoint.set(-0.5, -0.5, 0.5);
      projectionPoint.set(pointOnTheVertexBetweenMinusXandMinusYandZ);
      box.orthogonalProjection(projectionPoint);
      assertTrue(projectionPoint.epsilonEquals(expectedPoint, 1e-14));

      FramePoint3D point = new FramePoint3D(pointOnTheVertexBetweenMinusXandMinusYandZ);
      box.orthogonalProjection(point);
      assertTrue(point.epsilonEquals(expectedPoint, 1e-14));

      @SuppressWarnings("serial")
      ReferenceFrame frame = new ReferenceFrame("testFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }
      };
      frame.update();
      box.changeFrame(frame);

      assertTrue(box.getReferenceFrame().getName().contains(frame.getName()));

   }

   @Test
   public void testChangeFrame() throws Exception
   {
      // This test ensures consistency between the changeFrame of FrameBox3d and FramePose.
      double epsilon = 1.0e-12;
      Random random = new Random(342554L);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame frameA = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("A", worldFrame,
                                                                                             EuclidCoreRandomTools.nextRigidBodyTransform(random));
      ReferenceFrame frameB = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("B", worldFrame,
                                                                                             EuclidCoreRandomTools.nextRigidBodyTransform(random));
      ReferenceFrame frameC = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("C", frameB,
                                                                                             EuclidCoreRandomTools.nextRigidBodyTransform(random));
      ReferenceFrame frameD = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("D", frameA,
                                                                                             EuclidCoreRandomTools.nextRigidBodyTransform(random));
      ReferenceFrame[] frames = {worldFrame, frameA, frameB, frameC, frameD};

      RigidBodyTransform randomTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      Box3D expectedBox = new Box3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      FramePose3D expectedBoxPose = new FramePose3D(worldFrame, randomTransform);
      FramePose3D actualBoxPose = new FramePose3D();
      FrameBox3d frameBox = new FrameBox3d(worldFrame, expectedBox);
      frameBox.setPose(expectedBoxPose);
      frameBox.getFramePose(actualBoxPose);

      expectedBoxPose.checkReferenceFrameMatch(actualBoxPose);
      EuclidCoreTestTools.assertTuple3DEquals(expectedBoxPose.getPosition(), actualBoxPose.getPosition(), epsilon);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expectedBoxPose.getOrientation(), actualBoxPose.getOrientation(), epsilon);

      for (int i = 0; i < 100; i++)
      {
         ReferenceFrame newFrame = frames[random.nextInt(frames.length)];

         expectedBoxPose.changeFrame(newFrame);
         expectedBox.getPose().set(expectedBoxPose);
         frameBox.changeFrame(newFrame);
         frameBox.getFramePose(actualBoxPose);

         assertTrue(expectedBox.epsilonEquals(frameBox.getBox3d(), epsilon));
         expectedBoxPose.checkReferenceFrameMatch(actualBoxPose);
         EuclidCoreTestTools.assertTuple3DEquals(expectedBoxPose.getPosition(), actualBoxPose.getPosition(), epsilon);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expectedBoxPose.getOrientation(), actualBoxPose.getOrientation(), epsilon);
      }
   }

   @Test
   public void testgetClosestPointAndNormalAt()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FrameBox3d box = new FrameBox3d(worldFrame);

      FramePoint3D pointOnXFace = new FramePoint3D(worldFrame);
      pointOnXFace.set(0.4, 0.3, -0.1);
      FramePoint3D expectedPoint = new FramePoint3D(worldFrame);
      expectedPoint.set(0.5, 0.3, -0.1);
      FrameVector3D expectedNormal = new FrameVector3D(worldFrame);
      expectedNormal.set(1.0, 0.0, 0.0);
      FramePoint3D returnedPoint = new FramePoint3D(worldFrame);
      FrameVector3D returnedNormal = new FrameVector3D(worldFrame);
      box.getClosestPointAndNormalAt(returnedPoint, returnedNormal, pointOnXFace);
      assertTrue(expectedPoint.epsilonEquals(returnedPoint, 1e-14));
      assertTrue(expectedNormal.epsilonEquals(returnedNormal, 1e-14));

      FramePoint3D pointOnYFace = new FramePoint3D(worldFrame);
      pointOnYFace.set(-0.2, 0.4, -0.1);
      expectedPoint.set(-0.2, 0.5, -0.1);
      expectedNormal.set(0.0, 1.0, 0.0);
      box.getClosestPointAndNormalAt(returnedPoint, returnedNormal, pointOnYFace);
      assertTrue(expectedPoint.epsilonEquals(returnedPoint, 1e-14));
      assertTrue(expectedNormal.epsilonEquals(returnedNormal, 1e-14));
   }

   @Test
   public void testIsInsideOrOnSurface()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FrameBox3d box = new FrameBox3d(worldFrame);

      FramePoint3D pointOnXFace = new FramePoint3D(worldFrame);
      pointOnXFace.set(0.4, 0.3, -0.1);

      assertTrue(box.isInsideOrOnSurface(pointOnXFace, 1e-7));

      FramePoint3D pointOnYFace = new FramePoint3D(worldFrame);
      pointOnYFace.set(-0.2, 0.4, -0.1);

      assertTrue(box.isInsideOrOnSurface(pointOnYFace, 1e-7));

      FramePoint3D pointOutsideBox = new FramePoint3D(worldFrame);
      pointOutsideBox.set(10, 10, 10);

      assertFalse(box.isInsideOrOnSurface(pointOutsideBox, 1e-7));
   }

   @Test
   public void testApplyTransform()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      Random random = new Random(562346L);
      int nBoxes = 100;
      int nTestsPerBox = 1000;
      for (int boxNumber = 0; boxNumber < nBoxes; boxNumber++)
      {
         FrameBox3d box = createRandomBox(worldFrame, random);
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         FrameBox3d boxTransformed = new FrameBox3d(box);
         boxTransformed.applyTransform(transform);
         FrameBox3d biggerBox = new FrameBox3d(box);
         biggerBox.scale(2.0);

         Point3D[] vertices = new Point3D[8];
         for (int i = 0; i < vertices.length; i++)
         {
            vertices[i] = new Point3D();
         }

         biggerBox.computeVertices(vertices);

         for (int testNumber = 0; testNumber < nTestsPerBox; testNumber++)
         {
            Point3D point = getRandomConvexCombination(random, vertices);
            Point3D pointTransformed = new Point3D(point);
            box.getGeometryObject().transformToLocal(pointTransformed);
            boxTransformed.getGeometryObject().transformToWorld(pointTransformed);

            assertEquals(box.isInsideOrOnSurface(new FramePoint3D(worldFrame, point)),
                         boxTransformed.isInsideOrOnSurface(new FramePoint3D(worldFrame, pointTransformed)));
         }
      }
   }

   @Test
   public void testSetTransform3DAndGetters()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      Random random = new Random(351235L);
      FrameBox3d box = new FrameBox3d(worldFrame);
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      box.setTransform(transform);

      RigidBodyTransform transformBack = new RigidBodyTransform();
      box.getTransform(transformBack);
      assertTrue(transform.epsilonEquals(transformBack, Epsilons.ONE_TRILLIONTH));
      assertTrue(transform.epsilonEquals(box.getTransformCopy(), Epsilons.ONE_TRILLIONTH));

      RotationMatrix matrix = new RotationMatrix();
      Vector3D vector = new Vector3D();
      transform.get(matrix, vector);

      RotationMatrix matrixBack = new RotationMatrix();
      FramePoint3D CenterBack = new FramePoint3D(worldFrame);

      box.getRotation(matrixBack);
      assertTrue(matrix.epsilonEquals(matrixBack, Epsilons.ONE_TRILLIONTH));
      assertTrue(matrix.epsilonEquals(box.getRotationCopy(), Epsilons.ONE_TRILLIONTH));

      box.getCenter(CenterBack);
      assertTrue(vector.epsilonEquals(new Vector3D(CenterBack), 0.0));
      assertTrue(vector.epsilonEquals(new Vector3D(box.getCenterCopy()), 0.0));

      assertTrue(vector.epsilonEquals(new Vector3D(box.getCenter()), 0.0));
      assertTrue(vector.epsilonEquals(new Vector3D(box.getCenterCopy()), 0.0));
   }

   private static FrameBox3d createRandomBox(ReferenceFrame referenceFrame, Random random)
   {
      RigidBodyTransform configuration = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      double lengthX = random.nextDouble();
      double widthY = random.nextDouble();
      double heightZ = random.nextDouble();

      return new FrameBox3d(referenceFrame, configuration, lengthX, widthY, heightZ);
   }

   private static Point3D getRandomConvexCombination(Random random, Point3D[] vertices)
   {
      double[] weightings = new double[vertices.length];
      for (int j = 0; j < weightings.length; j++)
      {
         weightings[j] = random.nextDouble();
      }

      double sum = MathTools.sum(weightings);
      for (int j = 0; j < weightings.length; j++)
      {
         weightings[j] /= sum;
      }

      Point3D pointToTest = new Point3D();
      for (int j = 0; j < weightings.length; j++)
      {
         Point3D tempPoint = new Point3D(vertices[j]);
         tempPoint.scale(weightings[j]);
         pointToTest.add(tempPoint);
      }

      return pointToTest;
   }

}
