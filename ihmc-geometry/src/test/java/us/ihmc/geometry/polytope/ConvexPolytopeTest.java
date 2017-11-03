package us.ihmc.geometry.polytope;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.ConvexPolytopeFace;
import us.ihmc.geometry.polytope.DCELPolytope.ExtendedConvexPolytope;
import us.ihmc.geometry.polytope.DCELPolytope.ExtendedPolytopeVertex;
import us.ihmc.geometry.polytope.DCELPolytope.PolytopeHalfEdge;

public class ConvexPolytopeTest
{
   private final static double EPSILON = Epsilons.ONE_MILLIONTH;

   @Test(timeout = 1000)
   public void testAdditionWithSquarePyramid()
   {
      ExtendedConvexPolytope polytope = new ExtendedConvexPolytope();
      ExtendedPolytopeVertex vertexOne = new ExtendedPolytopeVertex(0.0, 0.0, 0.0);
      ExtendedPolytopeVertex vertexTwo = new ExtendedPolytopeVertex(1.0, 0.0, 0.0);
      ExtendedPolytopeVertex vertexThree = new ExtendedPolytopeVertex(1.0, 1.0, 0.0);
      ExtendedPolytopeVertex vertexFour = new ExtendedPolytopeVertex(0.0, 1.0, 0.0);
      ExtendedPolytopeVertex vertexFive = new ExtendedPolytopeVertex(0.0, 0.0, 1.0);
      ExtendedPolytopeVertex vertexSix = new ExtendedPolytopeVertex(0.0, 1.0, 1.0);
      polytope.addVertex(vertexOne, EPSILON);
      polytope.addVertex(vertexTwo, EPSILON);
      polytope.addVertex(vertexThree, EPSILON);
      polytope.addVertex(vertexFour, EPSILON);
      polytope.addVertex(vertexFive, EPSILON);
      polytope.addVertex(vertexSix, EPSILON);
      assertTrue(polytope.getNumberOfFaces() == 5);
      assertTrue(polytope.getNumberOfEdges() == 9);
      assertTrue(polytope.getNumberOfVertices() == 6);
      for (int j = 0; j < polytope.getNumberOfFaces(); j++)
      {
         ConvexPolytopeFace face = polytope.getFace(j);
         for (int i = 0; i < face.getNumberOfEdges(); i++)
         {
            assertTrue("Null twin edge for edge: " + face.getEdge(i).toString() + " on face: " + face.toString(), face.getEdge(i).getTwinHalfEdge() != null);
            assertTrue("Twin edge: " + face.getEdge(i).getTwinHalfEdge().toString() + " mismatch for edge: " + face.getEdge(i).toString() + " on face: "
                  + face.toString(), face.getEdge(i).getTwinHalfEdge().getOriginVertex() == face.getEdge(i).getDestinationVertex());
            assertTrue("Twin edge: " + face.getEdge(i).getTwinHalfEdge().toString() + " mismatch for edge: " + face.getEdge(i).toString() + " on face: "
                  + face.toString(), face.getEdge(i).getTwinHalfEdge().getDestinationVertex() == face.getEdge(i).getOriginVertex());
         }
      }

   }

   @Test(timeout = 10000)
   public void testPartialCylinder()
   {
      ExtendedConvexPolytope polytope = new ExtendedConvexPolytope();
      polytope.addVertex(0.0, 0.0, 0.0, EPSILON);
      polytope.addVertex(0.0, 1.0, 0.0, EPSILON);
      polytope.addVertex(1.0, 2.0, 0.0, EPSILON);
      polytope.addVertex(2.0, 2.0, 0.0, EPSILON);

      ExtendedPolytopeVertex newVertex = new ExtendedPolytopeVertex(3.0, 1.0, 0.0);
      polytope.addVertex(newVertex, EPSILON);
      polytope.addVertex(3.0, 0.0, 0.0, EPSILON);
      polytope.addVertex(2.0, -1.0, 0.0, EPSILON);
      polytope.addVertex(1.0, -1.0, 0.0, EPSILON);

      polytope.addVertex(0.0, 0.0, 1.0, EPSILON);
      polytope.addVertex(0.0, 1.0, 1.0, EPSILON);
      polytope.addVertex(1.0, 2.0, 1.0, EPSILON);
      polytope.addVertex(2.0, 2.0, 1.0, EPSILON);

      polytope.addVertex(3.0, 1.0, 1.0, EPSILON);
      polytope.addVertex(3.0, 0.0, 1.0, EPSILON);
      polytope.addVertex(2.0, -1.0, 1.0, EPSILON);
      polytope.addVertex(1.0, -1.0, 1.0, EPSILON);

      for (int j = 0; j < polytope.getNumberOfFaces(); j++)
      {
         ConvexPolytopeFace face = polytope.getFace(j);
         for (int i = 0; i < face.getNumberOfEdges(); i++)
         {
            assertTrue("Null twin edge for edge: " + face.getEdge(i).toString() + " on face: " + face.toString(), face.getEdge(i).getTwinHalfEdge() != null);
            assertTrue("Twin edge: " + face.getEdge(i).getTwinHalfEdge().toString() + " mismatch for edge: " + face.getEdge(i).toString() + " on face: "
                  + face.toString(), face.getEdge(i).getTwinHalfEdge().getOriginVertex() == face.getEdge(i).getDestinationVertex());
            assertTrue("Twin edge: " + face.getEdge(i).getTwinHalfEdge().toString() + " mismatch for edge: " + face.getEdge(i).toString() + " on face: "
                  + face.toString(), face.getEdge(i).getTwinHalfEdge().getDestinationVertex() == face.getEdge(i).getOriginVertex());
         }
      }

      //      PrintTools.debug(newVertex.getNumberOfAssociatedEdges() + "");
      //      for(int i = 0; i < newVertex.getNumberOfAssociatedEdges(); i++)
      //      {
      //         PrintTools.debug("AVertex: " + newVertex.getAssociatedEdge(i).toString());
      //         PrintTools.debug("AVertexFace: " + newVertex.getAssociatedEdge(i).getTwinHalfEdge().toString());
      //      }
      //      polytope.addVertex(1.0, 2.0, 1.0, EPSILON);
      //      PrintTools.debug(" *** " +polytope.toString());
      //      polytope.addVertex(2.0, 2.0, 1.0, EPSILON);
      //      PrintTools.debug(" *** " +polytope.toString());
      //      
      //      polytope.addVertex(3.0, 1.0, 1.0, EPSILON);
      //      polytope.addVertex(3.0, 0.0, 1.0, EPSILON);
      //      polytope.addVertex(2.0, -1.0, 1.0, EPSILON);
      //      polytope.addVertex(1.0, -1.0, 1.0, EPSILON);

      //      polytope.addVertex(new PolytopeVertex(center.getX() + enclosingRadius, 0.0, center.getZ() + length / 2.0),
      //                            EPSILON);
      //      PolytopeVertex newVertex = new PolytopeVertex(center.getX() + enclosingRadius * Math.cos(vertexAngle),
      //                                            center.getY() + enclosingRadius * Math.sin(vertexAngle), center.getZ() + length / 2.0);
      //      List<ConvexPolytopeFace> visibleFaces = new ArrayList<>();
      //      List<ConvexPolytopeFace> visibleSilhouetteFaces = new ArrayList<>();
      //      List<ConvexPolytopeFace> faces = new ArrayList<>();
      //      List<PolytopeHalfEdge> silhouette = new ArrayList<>();
      //      polytope.getFacesWhichPointIsOn(newVertex, faces, EPSILON);
      //      polytope.getVisibleFaces(visibleFaces, newVertex, EPSILON);
      //      polytope.getSilhouetteFaces(visibleSilhouetteFaces, null, visibleFaces);
      //      PrintTools.debug("On faces: ");
      //      for(int i = 0; i < faces.size(); i++)
      //         PrintTools.debug(faces.get(i).toString());
      //      
      //      PrintTools.debug("Visible faces: ");
      //      for(int i = 0; i < visibleFaces.size(); i++)
      //         PrintTools.debug(visibleFaces.get(i).toString());
      //
      //      PrintTools.debug("Silhouette faces: ");
      //      for(int i = 0; i < visibleSilhouetteFaces.size(); i++)
      //         PrintTools.debug(visibleSilhouetteFaces.get(i).toString());
      //      
      //      PolytopeHalfEdge seedEdge = faces.get(0).getFirstVisibleEdge(newVertex).getTwinHalfEdge();
      //      PrintTools.debug("Seed edge: " + seedEdge.toString());
      //      polytope.getVisibleSilhouetteUsingSeed(silhouette, seedEdge, visibleSilhouetteFaces, newVertex);
      //      PrintTools.debug("Silhouette edges: ");
      //      for(int i = 0; i < silhouette.size(); i++)
      //         PrintTools.debug(silhouette.get(i).toString());

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 1000)
   public void testConvexPolytopeVisibleSilhouetteCalculation()
   {
      ExtendedConvexPolytope polytope = new ExtendedConvexPolytope();
      ExtendedPolytopeVertex vertexOne = new ExtendedPolytopeVertex(0.0, 0.0, 0.0);
      ExtendedPolytopeVertex vertexTwo = new ExtendedPolytopeVertex(1.0, 0.0, 0.0);
      ExtendedPolytopeVertex vertexThree = new ExtendedPolytopeVertex(0.0, 1.0, 0.0);
      ExtendedPolytopeVertex vertexFour = new ExtendedPolytopeVertex(0.0, 0.0, 1.0);
      ExtendedPolytopeVertex vertexFive = new ExtendedPolytopeVertex(0.0, 1.0, 1.0);
      ExtendedPolytopeVertex vertexSix = new ExtendedPolytopeVertex(1.0, 1.0, 1.0);
      polytope.addVertex(vertexOne, EPSILON);
      polytope.addVertex(vertexTwo, EPSILON);
      polytope.addVertex(vertexThree, EPSILON);
      polytope.addVertex(vertexFour, EPSILON);
      polytope.addVertex(vertexFive, EPSILON);
      List<PolytopeHalfEdge> visibleEdges = new ArrayList<>();
      polytope.getVisibleSilhouette(vertexSix, visibleEdges, EPSILON);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 100000)
   public void testConvexPolytopeWithUnitCube()
   {
      ExtendedConvexPolytope polytope = new ExtendedConvexPolytope();
      ExtendedPolytopeVertex vertexOne = new ExtendedPolytopeVertex(0.0, 0.0, 0.0);
      ExtendedPolytopeVertex vertexTwo = new ExtendedPolytopeVertex(1.0, 0.0, 0.0);
      ExtendedPolytopeVertex vertexThree = new ExtendedPolytopeVertex(0.0, 1.0, 0.0);
      ExtendedPolytopeVertex vertexFour = new ExtendedPolytopeVertex(0.0, 0.0, 1.0);
      ExtendedPolytopeVertex vertexFive = new ExtendedPolytopeVertex(0.0, 1.0, 1.0);
      ExtendedPolytopeVertex vertexSix = new ExtendedPolytopeVertex(1.0, 1.0, 0.0);
      ExtendedPolytopeVertex vertexSeven = new ExtendedPolytopeVertex(1.0, 0.0, 1.0);
      ExtendedPolytopeVertex vertexEight = new ExtendedPolytopeVertex(1.0, 1.0, 1.0);
      polytope.addVertex(vertexOne, EPSILON);
      polytope.addVertex(vertexTwo, EPSILON);
      polytope.addVertex(vertexThree, EPSILON);
      polytope.addVertex(vertexFour, EPSILON);
      polytope.addVertex(vertexFive, EPSILON);
      polytope.addVertex(vertexSix, EPSILON);
      polytope.addVertex(vertexSeven, EPSILON);
      polytope.addVertex(vertexEight, EPSILON);
      assertTrue(polytope.getNumberOfFaces() == 6);
      assertTrue(polytope.getNumberOfEdges() == 12);
      assertTrue(polytope.getNumberOfVertices() == 8);
      ConvexPolytopeFace firstFace = polytope.getFace(0);
      ConvexPolytopeFace secondFace = polytope.getFace(1);
      ConvexPolytopeFace thirdFace = polytope.getFace(2);
      ConvexPolytopeFace fourthFace = polytope.getFace(3);
      ConvexPolytopeFace fifthFace = polytope.getFace(4);
      ConvexPolytopeFace sixthFace = polytope.getFace(5);

      assertTrue(firstFace.getNumberOfEdges() == 4);
      assertTrue(secondFace.getNumberOfEdges() == 4);
      assertTrue(thirdFace.getNumberOfEdges() == 4);
      assertTrue(fourthFace.getNumberOfEdges() == 4);
      assertTrue(fifthFace.getNumberOfEdges() == 4);
      assertTrue(sixthFace.getNumberOfEdges() == 4);
      for (int j = 0; j < polytope.getNumberOfFaces(); j++)
      {
         ConvexPolytopeFace face = polytope.getFace(j);
         for (int i = 0; i < face.getNumberOfEdges(); i++)
         {
            assertTrue("Null twin edge for edge: " + face.getEdge(i).toString() + " on face: " + face.toString(), face.getEdge(i).getTwinHalfEdge() != null);
            assertTrue("Twin edge: " + face.getEdge(i).getTwinHalfEdge().toString() + " mismatch for edge: " + face.getEdge(i).toString() + " on face: "
                  + face.toString(), face.getEdge(i).getTwinHalfEdge().getOriginVertex() == face.getEdge(i).getDestinationVertex());
            assertTrue("Twin edge: " + face.getEdge(i).getTwinHalfEdge().toString() + " mismatch for edge: " + face.getEdge(i).toString() + " on face: "
                  + face.toString(), face.getEdge(i).getTwinHalfEdge().getDestinationVertex() == face.getEdge(i).getOriginVertex());
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 1000)
   public void testConvexPolytopeWithPyramid()
   {
      ExtendedConvexPolytope polytope = new ExtendedConvexPolytope();
      ExtendedPolytopeVertex vertexOne = new ExtendedPolytopeVertex(0.0, 0.0, 0.0);
      ExtendedPolytopeVertex vertexTwo = new ExtendedPolytopeVertex(1.0, 0.0, 0.0);
      ExtendedPolytopeVertex vertexThree = new ExtendedPolytopeVertex(1.0, 1.0, 0.0);
      ExtendedPolytopeVertex vertexFour = new ExtendedPolytopeVertex(0.5, 0.5, 1.0);
      polytope.addVertex(vertexOne, EPSILON);
      polytope.addVertex(vertexTwo, EPSILON);
      polytope.addVertex(vertexThree, EPSILON);
      polytope.addVertex(vertexFour, EPSILON);
      assertTrue(polytope.getNumberOfFaces() == 4);
      assertTrue(polytope.getNumberOfEdges() == 6);
      assertTrue(polytope.getNumberOfVertices() == 4);

      ConvexPolytopeFace firstFace = polytope.getFace(0);
      ConvexPolytopeFace secondFace = polytope.getFace(1);
      ConvexPolytopeFace thirdFace = polytope.getFace(2);
      ConvexPolytopeFace fourthFace = polytope.getFace(3);
      assertTrue(firstFace.getNumberOfEdges() == 3);
      assertTrue(secondFace.getNumberOfEdges() == 3);
      assertTrue(thirdFace.getNumberOfEdges() == 3);
      assertTrue(fourthFace.getNumberOfEdges() == 3);
      for (int i = 0; i < 3; i++)
      {
         assertTrue(firstFace.getEdge(i).getTwinHalfEdge() != null);
         assertTrue(firstFace.getEdge(i).getTwinHalfEdge().getOriginVertex() == firstFace.getEdge(i).getDestinationVertex());
         assertTrue(firstFace.getEdge(i).getTwinHalfEdge().getDestinationVertex() == firstFace.getEdge(i).getOriginVertex());
      }
      for (int i = 0; i < 3; i++)
      {
         assertTrue(secondFace.getEdge(i).getTwinHalfEdge() != null);
         assertTrue(secondFace.getEdge(i).getTwinHalfEdge().getOriginVertex() == secondFace.getEdge(i).getDestinationVertex());
         assertTrue(secondFace.getEdge(i).getTwinHalfEdge().getDestinationVertex() == secondFace.getEdge(i).getOriginVertex());
      }
      for (int i = 0; i < 3; i++)
      {
         assertTrue(thirdFace.getEdge(i).getTwinHalfEdge() != null);
         assertTrue(thirdFace.getEdge(i).getTwinHalfEdge().getOriginVertex() == thirdFace.getEdge(i).getDestinationVertex());
         assertTrue(thirdFace.getEdge(i).getTwinHalfEdge().getDestinationVertex() == thirdFace.getEdge(i).getOriginVertex());
      }
      for (int i = 0; i < 3; i++)
      {
         assertTrue(fourthFace.getEdge(i).getTwinHalfEdge() != null);
         assertTrue(fourthFace.getEdge(i).getTwinHalfEdge().getOriginVertex() == fourthFace.getEdge(i).getDestinationVertex());
         assertTrue(fourthFace.getEdge(i).getTwinHalfEdge().getDestinationVertex() == fourthFace.getEdge(i).getOriginVertex());
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 1000)
   public void testConvexPolytopeWithSquarePyramid()
   {
      ExtendedConvexPolytope polytope = new ExtendedConvexPolytope();
      ExtendedPolytopeVertex vertexOne = new ExtendedPolytopeVertex(0.0, 0.0, 0.0);
      ExtendedPolytopeVertex vertexTwo = new ExtendedPolytopeVertex(1.0, 0.0, 0.0);
      ExtendedPolytopeVertex vertexThree = new ExtendedPolytopeVertex(1.0, 1.0, 0.0);
      ExtendedPolytopeVertex vertexFour = new ExtendedPolytopeVertex(0.0, 1.0, 0.0);
      ExtendedPolytopeVertex vertexFive = new ExtendedPolytopeVertex(0.5, 0.5, 1.0);
      polytope.addVertex(vertexOne, EPSILON);
      polytope.addVertex(vertexTwo, EPSILON);
      polytope.addVertex(vertexThree, EPSILON);
      polytope.addVertex(vertexFour, EPSILON);
      polytope.addVertex(vertexFive, EPSILON);
      assertTrue(polytope.getNumberOfFaces() == 5);
      assertTrue(polytope.getNumberOfEdges() == 8);
      assertTrue(polytope.getNumberOfVertices() == 5);

      ConvexPolytopeFace firstFace = polytope.getFace(0);
      ConvexPolytopeFace secondFace = polytope.getFace(1);
      ConvexPolytopeFace thirdFace = polytope.getFace(2);
      ConvexPolytopeFace fourthFace = polytope.getFace(3);
      ConvexPolytopeFace fifthFace = polytope.getFace(4);
      assertTrue(firstFace.getNumberOfEdges() == 4);
      assertTrue(secondFace.getNumberOfEdges() == 3);
      assertTrue(thirdFace.getNumberOfEdges() == 3);
      assertTrue(fourthFace.getNumberOfEdges() == 3);
      assertTrue(fifthFace.getNumberOfEdges() == 3);

      for (int j = 0; j < polytope.getNumberOfFaces(); j++)
      {
         ConvexPolytopeFace face = polytope.getFace(j);
         for (int i = 0; i < face.getNumberOfEdges(); i++)
         {
            assertTrue("Null twin edge for edge: " + face.getEdge(i).toString() + " on face: " + face.toString(), face.getEdge(i).getTwinHalfEdge() != null);
            assertTrue("Twin edge: " + face.getEdge(i).getTwinHalfEdge().toString() + " mismatch for edge: " + face.getEdge(i).toString() + " on face: "
                  + face.toString(), face.getEdge(i).getTwinHalfEdge().getOriginVertex() == face.getEdge(i).getDestinationVertex());
            assertTrue("Twin edge: " + face.getEdge(i).getTwinHalfEdge().toString() + " mismatch for edge: " + face.getEdge(i).toString() + " on face: "
                  + face.toString(), face.getEdge(i).getTwinHalfEdge().getDestinationVertex() == face.getEdge(i).getOriginVertex());
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 1000)
   public void testConvexPolytopeWithWierdShape()
   {
      ExtendedConvexPolytope polytope = new ExtendedConvexPolytope();
      ExtendedPolytopeVertex vertexOne = new ExtendedPolytopeVertex(0.0, 0.0, 0.0);
      ExtendedPolytopeVertex vertexTwo = new ExtendedPolytopeVertex(1.0, 0.0, 0.0);
      ExtendedPolytopeVertex vertexThree = new ExtendedPolytopeVertex(1.0, 1.0, 0.0);
      ExtendedPolytopeVertex vertexFour = new ExtendedPolytopeVertex(0.5, 0.5, 1.0);
      ExtendedPolytopeVertex vertexFive = new ExtendedPolytopeVertex(0.0, 1.0, 1.0);
      polytope.addVertex(vertexOne, EPSILON);
      polytope.addVertex(vertexTwo, EPSILON);
      polytope.addVertex(vertexThree, EPSILON);
      polytope.addVertex(vertexFour, EPSILON);
      polytope.addVertex(vertexFive, EPSILON);
      assertTrue(polytope.getNumberOfFaces() == 6);
      assertTrue(polytope.getNumberOfEdges() == 9);
      assertTrue(polytope.getNumberOfVertices() == 5);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 1000)
   public void testConvexPolytopeWithAUnitCube()
   {
      ExtendedConvexPolytope polytope = new ExtendedConvexPolytope();
      ExtendedPolytopeVertex vertexOne = new ExtendedPolytopeVertex(0.0, 0.0, 0.0);
      ExtendedPolytopeVertex vertexTwo = new ExtendedPolytopeVertex(1.0, 0.0, 0.0);
      ExtendedPolytopeVertex vertexThree = new ExtendedPolytopeVertex(1.0, 1.0, 0.0);
      ExtendedPolytopeVertex vertexFour = new ExtendedPolytopeVertex(0.0, 1.0, 0.0);
      ExtendedPolytopeVertex vertexFive = new ExtendedPolytopeVertex(0.0, 0.0, 1.0);
      ExtendedPolytopeVertex vertexSix = new ExtendedPolytopeVertex(1.0, 0.0, 1.0);
      ExtendedPolytopeVertex vertexSeven = new ExtendedPolytopeVertex(1.0, 1.0, 1.0);
      ExtendedPolytopeVertex vertexEight = new ExtendedPolytopeVertex(0.0, 1.0, 1.0);

      polytope.addVertex(vertexOne, EPSILON);
      polytope.addVertex(vertexTwo, EPSILON);
      polytope.addVertex(vertexThree, EPSILON);
      polytope.addVertex(vertexFour, EPSILON);
      polytope.addVertex(vertexFive, EPSILON);
      polytope.addVertex(vertexSix, EPSILON);
      polytope.addVertex(vertexSeven, EPSILON);
      polytope.addVertex(vertexEight, EPSILON);

      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(0.0, 0.0, 0.0), vertexOne.getPosition(), Epsilons.ONE_TEN_BILLIONTH);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(1.0, 1.0, 0.0), vertexThree.getPosition(), Epsilons.ONE_TEN_BILLIONTH);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(1.0, 0.0, 1.0), vertexSix.getPosition(), Epsilons.ONE_TEN_BILLIONTH);

      assertEquals(8, polytope.getNumberOfVertices());
      assertEquals(12, polytope.getNumberOfEdges());

      assertEquals(3, vertexOne.getNumberOfAssociatedEdges());
      assertEquals(3, vertexTwo.getNumberOfAssociatedEdges());
      assertEquals(3, vertexThree.getNumberOfAssociatedEdges());
      assertEquals(3, vertexFour.getNumberOfAssociatedEdges());
      assertEquals(3, vertexFive.getNumberOfAssociatedEdges());
      assertEquals(3, vertexSix.getNumberOfAssociatedEdges());
      assertEquals(3, vertexSeven.getNumberOfAssociatedEdges());
      assertEquals(3, vertexEight.getNumberOfAssociatedEdges());

      List<ExtendedPolytopeVertex> vertices = polytope.getVertices();

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(1.0, 2.0, 3.0);
      polytope.applyTransform(transform);

      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(1.0, 2.0, 3.0), vertexOne.getPosition(), Epsilons.ONE_TEN_BILLIONTH);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(2.0, 3.0, 3.0), vertexThree.getPosition(), Epsilons.ONE_TEN_BILLIONTH);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(2.0, 2.0, 4.0), vertexSix.getPosition(), Epsilons.ONE_TEN_BILLIONTH);

      transform.setRotationEulerAndZeroTranslation(0.0, 0.0, Math.PI / 2.0);
      polytope.applyTransform(transform);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(-2.0, 1.0, 3.0), vertexOne.getPosition(), Epsilons.ONE_TEN_BILLIONTH);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(-3.0, 2.0, 3.0), vertexThree.getPosition(), Epsilons.ONE_TEN_BILLIONTH);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(-2.0, 2.0, 4.0), vertexSix.getPosition(), Epsilons.ONE_TEN_BILLIONTH);

      // Apply in reverse order to get back to unit box at origin.
      transform.setRotationEulerAndZeroTranslation(0.0, 0.0, -Math.PI / 2.0);
      polytope.applyTransform(transform);
      transform.setTranslationAndIdentityRotation(-1.0, -2.0, -3.0);
      polytope.applyTransform(transform);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(0.0, 0.0, 0.0), vertexOne.getPosition(), Epsilons.ONE_TEN_BILLIONTH);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(1.0, 1.0, 0.0), vertexThree.getPosition(), Epsilons.ONE_TEN_BILLIONTH);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(1.0, 0.0, 1.0), vertexSix.getPosition(), Epsilons.ONE_TEN_BILLIONTH);

      Vector3D supportDirection = new Vector3D(1.0, 1.0, 1.0);
      Point3DReadOnly supportingVertex = polytope.getSupportingVertex(supportDirection);
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
      ExtendedConvexPolytope cubeOne = ConvexPolytopeConstructor.constructExtendedBoxWithCenterAtZero(100.0, 100.0, 0.5);
      assertEquals(8, cubeOne.getNumberOfVertices());
      assertEquals(12, cubeOne.getNumberOfEdges());
      List<PolytopeHalfEdge> edges = cubeOne.getEdges();
      assertEquals(24, edges.size());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testBoundingBoxes()
   {
      ExtendedConvexPolytope polytope = new ExtendedConvexPolytope();
      BoundingBox3D boundingBox = new BoundingBox3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      polytope.getBoundingBox(boundingBox);

      assertTrue(boundingBox.getMinX() == Double.NEGATIVE_INFINITY);
      assertTrue(boundingBox.getMinY() == Double.NEGATIVE_INFINITY);
      assertTrue(boundingBox.getMinZ() == Double.NEGATIVE_INFINITY);
      assertTrue(boundingBox.getMaxX() == Double.POSITIVE_INFINITY);
      assertTrue(boundingBox.getMaxY() == Double.POSITIVE_INFINITY);
      assertTrue(boundingBox.getMaxZ() == Double.POSITIVE_INFINITY);

      ExtendedPolytopeVertex vertexOne = new ExtendedPolytopeVertex(0.0, 0.0, 0.0);
      ExtendedPolytopeVertex vertexTwo = new ExtendedPolytopeVertex(1.0, 0.0, 0.0);
      ExtendedPolytopeVertex vertexThree = new ExtendedPolytopeVertex(1.0, 1.0, 0.0);
      ExtendedPolytopeVertex vertexFour = new ExtendedPolytopeVertex(0.0, 1.0, 0.0);

      ExtendedPolytopeVertex vertexFive = new ExtendedPolytopeVertex(new Point3D(0.0, 0.0, 1.0));
      ExtendedPolytopeVertex vertexSix = new ExtendedPolytopeVertex(new Point3D(1.0, 0.0, 1.0));
      ExtendedPolytopeVertex vertexSeven = new ExtendedPolytopeVertex(new Point3D(1.0, 1.0, 1.0));
      ExtendedPolytopeVertex vertexEight = new ExtendedPolytopeVertex(new Point3D(0.0, 1.0, 1.0));

      polytope.addVertex(vertexOne, EPSILON);
      polytope.addVertex(vertexTwo, EPSILON);
      polytope.addVertex(vertexThree, EPSILON);
      polytope.addVertex(vertexFour, EPSILON);
      polytope.addVertex(vertexFive, EPSILON);
      polytope.addVertex(vertexSix, EPSILON);
      polytope.addVertex(vertexSeven, EPSILON);
      polytope.addVertex(vertexEight, EPSILON);

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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 1000)
   public void testSupportingVectorCalculation()
   {
      ExtendedConvexPolytope cubeOne = ConvexPolytopeConstructor.constructExtendedBoxWithCenterAtZero(100.0, 100.0, 0.5);
      Vector3D supportDirection = new Vector3D(1.0, -1.0, 0.0);
      Point3DReadOnly supportVertex = cubeOne.getSupportingVertex(supportDirection);
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForPackage(ConvexPolytopeTest.class);
   }
}
