package us.ihmc.geometry.polytope;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.geometry.polytope.DCELPolytope.ConvexPolytopeFace;
import us.ihmc.geometry.polytope.DCELPolytope.ExtendedPolytopeVertex;
import us.ihmc.geometry.polytope.DCELPolytope.PolytopeHalfEdge;

public class ConvexPolytopeFaceTest
{
   @Test
   public void testConstructorAndAddVertex()
   {
      ConvexPolytopeFace face = new ConvexPolytopeFace();
      ExtendedPolytopeVertex vertex1 = new ExtendedPolytopeVertex(0.0, 0.0, 0.0);
      ExtendedPolytopeVertex vertex2 = new ExtendedPolytopeVertex(0.0, 1.0, 0.0);
      ExtendedPolytopeVertex vertex3 = new ExtendedPolytopeVertex(1.0, 0.0, 0.0);
      face.addVertex(vertex1);
      assertTrue(face.getNumberOfEdges() == 1);
      assertTrue(face.getEdge(0).getOriginVertex() == vertex1);
      assertTrue(face.getEdge(0).getDestinationVertex() == vertex1);
      face.addVertex(vertex2);
      assertTrue(face.getNumberOfEdges() == 2);
      assertTrue(face.getEdge(0).getOriginVertex() == vertex1);
      assertTrue(face.getEdge(0).getDestinationVertex() == vertex2);
      assertTrue(face.getEdge(1).getOriginVertex() == vertex2);
      assertTrue(face.getEdge(1).getDestinationVertex() == vertex1);
      face.addVertex(vertex3);
      assertTrue(face.getNumberOfEdges() == 3);
      assertTrue(face.getEdge(0).getOriginVertex() == vertex1);
      assertTrue(face.getEdge(0).getDestinationVertex() == vertex2);
      assertTrue(face.getEdge(1).getOriginVertex() == vertex2);
      assertTrue(face.getEdge(1).getDestinationVertex() == vertex3);
      assertTrue(face.getEdge(2).getOriginVertex() == vertex3);
      assertTrue(face.getEdge(2).getDestinationVertex() == vertex1);
      
      ExtendedPolytopeVertex vertex4 = new ExtendedPolytopeVertex(1.0, 1.0, 0.0);
      face.addVertex(vertex4);
      assertTrue("Got: " + face.getNumberOfEdges() + " , should have been 4", face.getNumberOfEdges() == 4);
      PolytopeHalfEdge edge = face.getEdge(0);
      assertTrue(edge.getOriginVertex() == vertex1);
      assertTrue(edge.getDestinationVertex() == vertex2);
      edge = edge.getNextHalfEdge();
      assertTrue(edge.getOriginVertex() == vertex2);
      assertTrue(edge.getDestinationVertex() == vertex4);
      edge = edge.getNextHalfEdge();
      assertTrue(edge.getOriginVertex() == vertex4);
      assertTrue(edge.getDestinationVertex() == vertex3);
      edge = edge.getNextHalfEdge();
      assertTrue(edge.getOriginVertex() == vertex3);
      assertTrue(edge.getDestinationVertex() == vertex1);

      ExtendedPolytopeVertex vertex5 = new ExtendedPolytopeVertex(2.0, 2.0, 0.0);
      face.addVertex(vertex5);
      assertTrue("Number of edges: " + face.getNumberOfEdges() + ", needed: " + 4, face.getNumberOfEdges() == 4);
      edge = face.getEdge(0);
      assertTrue(edge.getOriginVertex() == vertex1);
      assertTrue(edge.getDestinationVertex() == vertex2);
      edge = edge.getNextHalfEdge();
      assertTrue(edge.getOriginVertex() == vertex2);
      assertTrue(edge.getDestinationVertex() == vertex5);
      edge = edge.getNextHalfEdge();
      assertTrue(edge.getOriginVertex() == vertex5);
      assertTrue(edge.getDestinationVertex() == vertex3);
      edge = edge.getNextHalfEdge();
      assertTrue(edge.getOriginVertex() == vertex3);
      assertTrue(edge.getDestinationVertex() == vertex1);

      ExtendedPolytopeVertex vertex6 = new ExtendedPolytopeVertex(3.0, 3.0, 0.0);
      face.addVertex(vertex6);
      assertTrue("Number of edges: " + face.getNumberOfEdges() + ", needed: " + 4, face.getNumberOfEdges() == 4);
      edge = face.getEdge(0);
      assertTrue(edge.getOriginVertex() == vertex1);
      assertTrue(edge.getDestinationVertex() == vertex2);
      edge = edge.getNextHalfEdge();
      assertTrue(edge.getOriginVertex() == vertex2);
      assertTrue(edge.getDestinationVertex() == vertex6);
      edge = edge.getNextHalfEdge();
      assertTrue(edge.getOriginVertex() == vertex6);
      assertTrue(edge.getDestinationVertex() == vertex3);
      edge = edge.getNextHalfEdge();
      assertTrue(edge.getOriginVertex() == vertex3);
      assertTrue(edge.getDestinationVertex() == vertex1);

      ExtendedPolytopeVertex vertex7 = new ExtendedPolytopeVertex(2.0, 3.0, 0.0);
      face.addVertex(vertex7);
      assertTrue("Number of edges: " + face.getNumberOfEdges() + ", needed: " + 5, face.getNumberOfEdges() == 5);
      edge = face.getEdge(0);
      assertTrue(edge.getOriginVertex() == vertex1);
      assertTrue(edge.getDestinationVertex() == vertex2);
      edge = edge.getNextHalfEdge();
      assertTrue(edge.getOriginVertex() == vertex2);
      assertTrue(edge.getDestinationVertex() == vertex7);
      edge = edge.getNextHalfEdge();
      assertTrue(edge.getOriginVertex() == vertex7);
      assertTrue(edge.getDestinationVertex() == vertex6);
      edge = edge.getNextHalfEdge();
      assertTrue(edge.getOriginVertex() == vertex6);
      assertTrue(edge.getDestinationVertex() == vertex3);
      edge = edge.getNextHalfEdge();
      assertTrue(edge.getOriginVertex() == vertex3);
      assertTrue(edge.getDestinationVertex() == vertex1);
      
      ExtendedPolytopeVertex vertex8 = new ExtendedPolytopeVertex(0.0, 0.0, 1.0);
      assertFalse(face.isPointInFacePlane(vertex8, Epsilons.ONE_BILLIONTH));
      face.addVertex(vertex8);
      assertTrue("Number of edges: " + face.getNumberOfEdges() + ", needed: " + 5, face.getNumberOfEdges() == 5);
      edge = face.getEdge(0);
      assertTrue(edge.getOriginVertex() == vertex1);
      assertTrue(edge.getDestinationVertex() == vertex2);
      edge = edge.getNextHalfEdge();
      assertTrue(edge.getOriginVertex() == vertex2);
      assertTrue(edge.getDestinationVertex() == vertex7);
      edge = edge.getNextHalfEdge();
      assertTrue(edge.getOriginVertex() == vertex7);
      assertTrue(edge.getDestinationVertex() == vertex6);
      edge = edge.getNextHalfEdge();
      assertTrue(edge.getOriginVertex() == vertex6);
      assertTrue(edge.getDestinationVertex() == vertex3);
      edge = edge.getNextHalfEdge();
      assertTrue(edge.getOriginVertex() == vertex3);
      assertTrue(edge.getDestinationVertex() == vertex1);
      
      ExtendedPolytopeVertex vertex9 = new ExtendedPolytopeVertex(1.0, 1.0, 0.0);
      assertTrue(face.isPointInFacePlane(vertex9, Epsilons.ONE_MILLIONTH));
      assertTrue(face.isInteriorPoint(vertex9));
      face.addVertex(vertex9);
      assertTrue(face.toString() + "\nNumber of edges: " + face.getNumberOfEdges() + ", needed: " + 5, face.getNumberOfEdges() == 5);
      edge = face.getEdge(0);
      assertTrue(edge.getOriginVertex() == vertex1);
      assertTrue(edge.getDestinationVertex() == vertex2);
      edge = edge.getNextHalfEdge();
      assertTrue(edge.getOriginVertex() == vertex2);
      assertTrue(edge.getDestinationVertex() == vertex7);
      edge = edge.getNextHalfEdge();
      assertTrue(edge.getOriginVertex() == vertex7);
      assertTrue(edge.getDestinationVertex() == vertex6);
      edge = edge.getNextHalfEdge();
      assertTrue(edge.getOriginVertex() == vertex6);
      assertTrue(edge.getDestinationVertex() == vertex3);
      edge = edge.getNextHalfEdge();
      assertTrue(edge.getOriginVertex() == vertex3);
      assertTrue(edge.getDestinationVertex() == vertex1);
         }
   
   @Test
   public void testGetFirstVisibleEdge()
   {
      ExtendedPolytopeVertex vertex1 = new ExtendedPolytopeVertex(0.0, 0.0, 0.0);
      ExtendedPolytopeVertex vertex2 = new ExtendedPolytopeVertex(1.0, 0.0, 0.0);
      ExtendedPolytopeVertex vertex3 = new ExtendedPolytopeVertex(1.0, 1.0, 0.0);
      ExtendedPolytopeVertex vertex4 = new ExtendedPolytopeVertex(0.0, 1.0, 0.0);
      ExtendedPolytopeVertex vertex5 = new ExtendedPolytopeVertex(-1.0, 0.5, 0.0);
      PolytopeHalfEdge halfEdge1 = new PolytopeHalfEdge(vertex1, vertex2);
      PolytopeHalfEdge halfEdge2 = new PolytopeHalfEdge(vertex2, vertex3);
      PolytopeHalfEdge halfEdge3 = new PolytopeHalfEdge(vertex3, vertex4);
      PolytopeHalfEdge halfEdge4 = new PolytopeHalfEdge(vertex4, vertex5);
      PolytopeHalfEdge halfEdge5 = new PolytopeHalfEdge(vertex5, vertex1);
      halfEdge1.setNextHalfEdge(halfEdge2);
      halfEdge2.setNextHalfEdge(halfEdge3);
      halfEdge3.setNextHalfEdge(halfEdge4);
      halfEdge4.setNextHalfEdge(halfEdge5);
      halfEdge5.setNextHalfEdge(halfEdge1);
      halfEdge1.setPreviousHalfEdge(halfEdge5);
      halfEdge2.setPreviousHalfEdge(halfEdge1);
      halfEdge3.setPreviousHalfEdge(halfEdge2);
      halfEdge4.setPreviousHalfEdge(halfEdge3);
      halfEdge5.setPreviousHalfEdge(halfEdge4);
      ConvexPolytopeFace face = new ConvexPolytopeFace(new PolytopeHalfEdge[]{halfEdge1, halfEdge2, halfEdge3, halfEdge4, halfEdge5});
      
      ExtendedPolytopeVertex vertex6 = new ExtendedPolytopeVertex(-1.0, -1.0, 0.0);
      PolytopeHalfEdge firstVisibleEdge = face.getFirstVisibleEdge(vertex6);
      assertTrue(firstVisibleEdge == halfEdge5);
      
      ExtendedPolytopeVertex vertex7 = new ExtendedPolytopeVertex(2.0, -1.0, 0.0);
      firstVisibleEdge = face.getFirstVisibleEdge(vertex7);
      assertTrue(firstVisibleEdge == halfEdge1);
   }
   
   @Test
   public void testGetVisibleEdgeList()
   {
      ExtendedPolytopeVertex vertex1 = new ExtendedPolytopeVertex(0.0, 0.0, 0.0);
      ExtendedPolytopeVertex vertex2 = new ExtendedPolytopeVertex(1.0, 0.0, 0.0);
      ExtendedPolytopeVertex vertex3 = new ExtendedPolytopeVertex(1.0, 1.0, 0.0);
      ExtendedPolytopeVertex vertex4 = new ExtendedPolytopeVertex(0.0, 1.0, 0.0);
      ExtendedPolytopeVertex vertex5 = new ExtendedPolytopeVertex(-1.0, 0.5, 0.0);
      PolytopeHalfEdge halfEdge1 = new PolytopeHalfEdge(vertex1, vertex2);
      PolytopeHalfEdge halfEdge2 = new PolytopeHalfEdge(vertex2, vertex3);
      PolytopeHalfEdge halfEdge3 = new PolytopeHalfEdge(vertex3, vertex4);
      PolytopeHalfEdge halfEdge4 = new PolytopeHalfEdge(vertex4, vertex5);
      PolytopeHalfEdge halfEdge5 = new PolytopeHalfEdge(vertex5, vertex1);
      halfEdge1.setNextHalfEdge(halfEdge2);
      halfEdge2.setNextHalfEdge(halfEdge3);
      halfEdge3.setNextHalfEdge(halfEdge4);
      halfEdge4.setNextHalfEdge(halfEdge5);
      halfEdge5.setNextHalfEdge(halfEdge1);
      halfEdge1.setPreviousHalfEdge(halfEdge5);
      halfEdge2.setPreviousHalfEdge(halfEdge1);
      halfEdge3.setPreviousHalfEdge(halfEdge2);
      halfEdge4.setPreviousHalfEdge(halfEdge3);
      halfEdge5.setPreviousHalfEdge(halfEdge4);
      ConvexPolytopeFace face = new ConvexPolytopeFace(new PolytopeHalfEdge[]{halfEdge1, halfEdge2, halfEdge3, halfEdge4, halfEdge5});
      
      List<PolytopeHalfEdge> visibleEdgeList = new ArrayList<>();
      ExtendedPolytopeVertex vertex6 = new ExtendedPolytopeVertex(-1.0, -1.0, 0.0);
      face.getVisibleEdgeList(vertex6, visibleEdgeList);
      assertTrue(visibleEdgeList.size() == 2);
      assertTrue(visibleEdgeList.get(0) == halfEdge5);
      assertTrue(visibleEdgeList.get(1) == halfEdge1);

      ExtendedPolytopeVertex vertex7 = new ExtendedPolytopeVertex(2.0, -1.0, 0.0);
      face.getVisibleEdgeList(vertex7, visibleEdgeList);
      assertTrue(visibleEdgeList.size() == 2);
      assertTrue(visibleEdgeList.get(0) == halfEdge1);
      assertTrue(visibleEdgeList.get(1) == halfEdge2);
   
   }
}
