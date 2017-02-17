package us.ihmc.geometry.polytope;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.tools.testing.MutationTestingTools;

public class ConvexPolytopeTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConvexPolytopeWithAUnitCube()
   {
      ConvexPolytope polytope = new ConvexPolytope();
      PolytopeVertex vertexOne = polytope.addVertex(0.0, 0.0, 0.0);
      PolytopeVertex vertexTwo = polytope.addVertex(1.0, 0.0, 0.0);
      PolytopeVertex vertexThree = polytope.addVertex(1.0, 1.0, 0.0);
      PolytopeVertex vertexFour = polytope.addVertex(0.0, 1.0, 0.0);

      PolytopeVertex vertexFive = polytope.addVertex(new Point3D(0.0, 0.0, 1.0));
      PolytopeVertex vertexSix = polytope.addVertex(new Point3D(1.0, 0.0, 1.0));
      PolytopeVertex vertexSeven = polytope.addVertex(new Point3D(1.0, 1.0, 1.0));
      PolytopeVertex vertexEight = polytope.addVertex(new Point3D(0.0, 1.0, 1.0));

      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(0.0, 0.0, 0.0), vertexOne.getPosition(), 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(1.0, 1.0, 0.0), vertexThree.getPosition(), 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(1.0, 0.0, 1.0), vertexSix.getPosition(), 1e-7);

      polytope.addEdge(vertexOne, vertexTwo);
      polytope.addEdge(vertexTwo, vertexThree);
      polytope.addEdge(vertexThree, vertexFour);
      polytope.addEdge(vertexFour, vertexOne);

      polytope.addEdge(vertexFive, vertexSix);
      polytope.addEdge(vertexSix, vertexSeven);
      polytope.addEdge(vertexSeven, vertexEight);
      polytope.addEdge(vertexEight, vertexFive);

      polytope.addEdge(vertexOne, vertexFive);
      polytope.addEdge(vertexTwo, vertexSix);
      polytope.addEdge(vertexThree, vertexSeven);
      polytope.addEdge(vertexFour, vertexEight);

      // Redundant edges should be ignored.
      polytope.addEdge(vertexFour, vertexEight);

      assertEquals(8, polytope.getNumberOfVertices());
      assertEquals(12, polytope.getNumberOfEdges());

      assertEquals(3, vertexOne.getNumberOfConnectingVertices());
      assertEquals(3, vertexTwo.getNumberOfConnectingVertices());
      assertEquals(3, vertexThree.getNumberOfConnectingVertices());
      assertEquals(3, vertexFour.getNumberOfConnectingVertices());
      assertEquals(3, vertexFive.getNumberOfConnectingVertices());
      assertEquals(3, vertexSix.getNumberOfConnectingVertices());
      assertEquals(3, vertexSeven.getNumberOfConnectingVertices());
      assertEquals(3, vertexEight.getNumberOfConnectingVertices());

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(1.0, 2.0, 3.0);
      polytope.applyTransform(transform);

      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(1.0, 2.0, 3.0), vertexOne.getPosition(), 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(2.0, 3.0, 3.0), vertexThree.getPosition(), 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(2.0, 2.0, 4.0), vertexSix.getPosition(), 1e-7);

      transform.setRotationEulerAndZeroTranslation(0.0, 0.0, Math.PI / 2.0);
      polytope.applyTransform(transform);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(-2.0, 1.0, 3.0), vertexOne.getPosition(), 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(-3.0, 2.0, 3.0), vertexThree.getPosition(), 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(-2.0, 2.0, 4.0), vertexSix.getPosition(), 1e-7);

      // Apply in reverse order to get back to unit box at origin.
      transform.setRotationEulerAndZeroTranslation(0.0, 0.0, -Math.PI / 2.0);
      polytope.applyTransform(transform);
      transform.setTranslationAndIdentityRotation(-1.0, -2.0, -3.0);
      polytope.applyTransform(transform);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(0.0, 0.0, 0.0), vertexOne.getPosition(), 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(1.0, 1.0, 0.0), vertexThree.getPosition(), 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(1.0, 0.0, 1.0), vertexSix.getPosition(), 1e-7);

      Vector3D supportDirection = new Vector3D(1.0, 1.0, 1.0);
      Point3D supportingVertex = polytope.getSupportingVertex(supportDirection);
      assertTrue(supportingVertex == vertexSeven.getPosition());

      supportDirection = new Vector3D(-1.0, -1.0, -1.0);
      supportingVertex = polytope.getSupportingVertex(supportDirection);
      assertTrue(supportingVertex == vertexOne.getPosition());

      supportDirection = new Vector3D(100.0, 0.01, -0.01);
      supportingVertex = polytope.getSupportingVertex(supportDirection);
      assertTrue(supportingVertex == vertexThree.getPosition());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPolytopeConstructor()
   {
      ConvexPolytope cubeOne = ConvexPolytopeConstructor.constructBoxWithCenterAtZero(100.0, 100.0, 0.5);
      assertEquals(8, cubeOne.getNumberOfVertices());
      assertEquals(12, cubeOne.getNumberOfEdges());
      ArrayList<PolytopeVertex[]> edges = cubeOne.getEdges();

      assertEquals(12, edges.size());
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testBoundingBoxes()
   {
      ConvexPolytope polytope = new ConvexPolytope();
      BoundingBox3d boundingBox = new BoundingBox3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      polytope.getBoundingBox(boundingBox);

      assertTrue(boundingBox.getXMin() == Double.NEGATIVE_INFINITY);
      assertTrue(boundingBox.getYMin() == Double.NEGATIVE_INFINITY);
      assertTrue(boundingBox.getZMin() == Double.NEGATIVE_INFINITY);
      assertTrue(boundingBox.getXMax() == Double.POSITIVE_INFINITY);
      assertTrue(boundingBox.getYMax() == Double.POSITIVE_INFINITY);
      assertTrue(boundingBox.getZMax() == Double.POSITIVE_INFINITY);
      
      PolytopeVertex vertexOne = polytope.addVertex(0.0, 0.0, 0.0);
      PolytopeVertex vertexTwo = polytope.addVertex(1.0, 0.0, 0.0);
      PolytopeVertex vertexThree = polytope.addVertex(1.0, 1.0, 0.0);
      PolytopeVertex vertexFour = polytope.addVertex(0.0, 1.0, 0.0);

      PolytopeVertex vertexFive = polytope.addVertex(new Point3D(0.0, 0.0, 1.0));
      PolytopeVertex vertexSix = polytope.addVertex(new Point3D(1.0, 0.0, 1.0));
      PolytopeVertex vertexSeven = polytope.addVertex(new Point3D(1.0, 1.0, 1.0));
      PolytopeVertex vertexEight = polytope.addVertex(new Point3D(0.0, 1.0, 1.0));

      polytope.addEdge(vertexOne, vertexTwo);
      polytope.addEdge(vertexTwo, vertexThree);
      polytope.addEdge(vertexThree, vertexFour);
      polytope.addEdge(vertexFour, vertexOne);

      polytope.addEdge(vertexFive, vertexSix);
      polytope.addEdge(vertexSix, vertexSeven);
      polytope.addEdge(vertexSeven, vertexEight);
      polytope.addEdge(vertexEight, vertexFive);

      polytope.addEdge(vertexOne, vertexFive);
      polytope.addEdge(vertexTwo, vertexSix);
      polytope.addEdge(vertexThree, vertexSeven);
      polytope.addEdge(vertexFour, vertexEight);
      
      polytope.getBoundingBox(boundingBox);
      
      Point3D minimumPoint = new Point3D();
      boundingBox.getMinPoint(minimumPoint);
      
      Point3D maximumPoint = new Point3D();
      boundingBox.getMaxPoint(maximumPoint);
      
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(0.0, 0.0, 0.0), minimumPoint, 1e-10);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(1.0, 1.0, 1.0), maximumPoint, 1e-10);
      
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(10, 20, 30);
      polytope.applyTransform(transform);
      
      polytope.getBoundingBox(boundingBox);
      
      boundingBox.getMinPoint(minimumPoint);
      boundingBox.getMaxPoint(maximumPoint);
      
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(10.0, 20.0, 30.0), minimumPoint, 1e-10);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(11.0, 21.0, 31.0), maximumPoint, 1e-10);
      
   }

   public static void main(String[] args)
   {
      String targetTests = ConvexPolytopeTest.class.getName();
      String targetClassesInSamePackage = MutationTestingTools.createClassSelectorStringFromTargetString(targetTests);
      MutationTestingTools.doPITMutationTestAndOpenResult(targetTests, targetClassesInSamePackage);
   }
}
