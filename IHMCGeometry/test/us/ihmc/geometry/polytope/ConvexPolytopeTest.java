package us.ihmc.geometry.polytope;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;
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

      PolytopeVertex vertexFive = polytope.addVertex(new Point3d(0.0, 0.0, 1.0));
      PolytopeVertex vertexSix = polytope.addVertex(new Point3d(1.0, 0.0, 1.0));
      PolytopeVertex vertexSeven = polytope.addVertex(new Point3d(1.0, 1.0, 1.0));
      PolytopeVertex vertexEight = polytope.addVertex(new Point3d(0.0, 1.0, 1.0));

      JUnitTools.assertPoint3dEquals("", new Point3d(0.0, 0.0, 0.0), vertexOne.getPosition(), 1e-7);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 1.0, 0.0), vertexThree.getPosition(), 1e-7);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 0.0, 1.0), vertexSix.getPosition(), 1e-7);

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

      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 2.0, 3.0), vertexOne.getPosition(), 1e-7);
      JUnitTools.assertPoint3dEquals("", new Point3d(2.0, 3.0, 3.0), vertexThree.getPosition(), 1e-7);
      JUnitTools.assertPoint3dEquals("", new Point3d(2.0, 2.0, 4.0), vertexSix.getPosition(), 1e-7);

      transform.setRotationEulerAndZeroTranslation(0.0, 0.0, Math.PI / 2.0);
      polytope.applyTransform(transform);
      JUnitTools.assertPoint3dEquals("", new Point3d(-2.0, 1.0, 3.0), vertexOne.getPosition(), 1e-7);
      JUnitTools.assertPoint3dEquals("", new Point3d(-3.0, 2.0, 3.0), vertexThree.getPosition(), 1e-7);
      JUnitTools.assertPoint3dEquals("", new Point3d(-2.0, 2.0, 4.0), vertexSix.getPosition(), 1e-7);

      // Apply in reverse order to get back to unit box at origin.
      transform.setRotationEulerAndZeroTranslation(0.0, 0.0, -Math.PI / 2.0);
      polytope.applyTransform(transform);
      transform.setTranslationAndIdentityRotation(-1.0, -2.0, -3.0);
      polytope.applyTransform(transform);
      JUnitTools.assertPoint3dEquals("", new Point3d(0.0, 0.0, 0.0), vertexOne.getPosition(), 1e-7);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 1.0, 0.0), vertexThree.getPosition(), 1e-7);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 0.0, 1.0), vertexSix.getPosition(), 1e-7);

      Vector3d supportDirection = new Vector3d(1.0, 1.0, 1.0);
      Point3d supportingVertex = polytope.getSupportingVertex(supportDirection);
      assertTrue(supportingVertex == vertexSeven.getPosition());

      supportDirection = new Vector3d(-1.0, -1.0, -1.0);
      supportingVertex = polytope.getSupportingVertex(supportDirection);
      assertTrue(supportingVertex == vertexOne.getPosition());

      supportDirection = new Vector3d(100.0, 0.01, -0.01);
      supportingVertex = polytope.getSupportingVertex(supportDirection);
      assertTrue(supportingVertex == vertexThree.getPosition());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test//(timeout = 30000)
   public void testPolytopeConstructor()
   {
      ConvexPolytope cubeOne = ConvexPolytopeConstructor.constructBoxWithCenterAtZero(100.0, 100.0, 0.5);
      assertEquals(8, cubeOne.getNumberOfVertices());
      assertEquals(12, cubeOne.getNumberOfEdges());
      ArrayList<PolytopeVertex[]> edges = cubeOne.getEdges();

      assertEquals(12, edges.size());
   }

   public static void main(String[] args)
   {
      String targetTests = ConvexPolytopeTest.class.getName();
      String targetClassesInSamePackage = MutationTestingTools.createClassSelectorStringFromTargetString(targetTests);
      MutationTestingTools.doPITMutationTestAndOpenResult(targetTests, targetClassesInSamePackage);
   }
}
