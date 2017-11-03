package us.ihmc.geometry.polytope;

import static org.junit.Assert.*;
import org.junit.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.ExtendedPolytopeVertex;
import us.ihmc.geometry.polytope.DCELPolytope.PolytopeHalfEdge;

public class PolytopeHalfEdgeTest
{
   @Test
   public void testConstructor()
   {
      ExtendedPolytopeVertex vertex1 = getRandomPolytopeVertex();
      ExtendedPolytopeVertex vertex2 = getRandomPolytopeVertex();
      PolytopeHalfEdge halfEdge1 = new PolytopeHalfEdge(vertex1, vertex2);
      assertTrue(halfEdge1.getOriginVertex() == vertex1);
      assertTrue(halfEdge1.getDestinationVertex() == vertex2);
      assertTrue(halfEdge1.getFace() == null);
      assertTrue(halfEdge1.getPreviousHalfEdge() == null);
      assertTrue(halfEdge1.getNextHalfEdge() == null);
      assertTrue(halfEdge1.getTwinHalfEdge() == null);
      
      PolytopeHalfEdge halfEdge2 = new PolytopeHalfEdge(halfEdge1, null);
      assertTrue(halfEdge2.getOriginVertex() == vertex2);
      assertTrue(halfEdge2.getDestinationVertex() == vertex1);
      assertTrue(halfEdge2.getFace() == null);
      assertTrue(halfEdge2.getPreviousHalfEdge() == null);
      assertTrue(halfEdge2.getNextHalfEdge() == null);
      assertTrue(halfEdge2.getTwinHalfEdge() == halfEdge1);
      
      ExtendedPolytopeVertex vertex3 = getRandomPolytopeVertex();
      PolytopeHalfEdge halfEdge3 = new PolytopeHalfEdge(vertex2, vertex3, null, null, halfEdge1, null);
      assertTrue(halfEdge3.getOriginVertex() == vertex2);
      assertTrue(halfEdge3.getDestinationVertex() == vertex3);
      assertTrue(halfEdge3.getFace() == null);
      assertTrue(halfEdge3.getPreviousHalfEdge() == halfEdge1);
      assertTrue(halfEdge3.getNextHalfEdge() == null);
      assertTrue(halfEdge3.getTwinHalfEdge() == null);
   }
   
   @Test
   public void testEdgeReverse()
   {
      ExtendedPolytopeVertex vertex1 = getRandomPolytopeVertex();
      ExtendedPolytopeVertex vertex2 = getRandomPolytopeVertex();
      ExtendedPolytopeVertex vertex3 = getRandomPolytopeVertex();
      
      PolytopeHalfEdge halfEdge1 = new PolytopeHalfEdge(vertex1, vertex2);
      PolytopeHalfEdge halfEdge2 = new PolytopeHalfEdge(vertex2, vertex3, null, null, halfEdge1, null);
      PolytopeHalfEdge halfEdge3 = new PolytopeHalfEdge(vertex3, vertex1, null, halfEdge1, halfEdge2, null);
      halfEdge1.setPreviousHalfEdge(halfEdge3);
      halfEdge1.setNextHalfEdge(halfEdge2);
      halfEdge2.setNextHalfEdge(halfEdge3);
      
      halfEdge2.reverseEdge();
      assertTrue(halfEdge2.getOriginVertex() == vertex3);
      assertTrue(halfEdge2.getDestinationVertex() == vertex2);
      assertTrue(halfEdge2.getNextHalfEdge() == halfEdge1);
      assertTrue(halfEdge2.getPreviousHalfEdge() == halfEdge3);
   }

   private ExtendedPolytopeVertex getRandomPolytopeVertex()
   {
      return new ExtendedPolytopeVertex(Math.random(), Math.random(), Math.random());
   }
   
   @Test 
   public void testCreateTwinEdge()
   {
      ExtendedPolytopeVertex vertex1 = getRandomPolytopeVertex();
      ExtendedPolytopeVertex vertex2 = getRandomPolytopeVertex();
      PolytopeHalfEdge halfEdge1 = new PolytopeHalfEdge(vertex1, vertex2);
      PolytopeHalfEdge twinOfHalfEdge1 = halfEdge1.createTwinHalfEdge();
      assertTrue(twinOfHalfEdge1.getTwinHalfEdge() == halfEdge1);
      assertTrue(twinOfHalfEdge1.getOriginVertex() == vertex2);
      assertTrue(twinOfHalfEdge1.getDestinationVertex() == vertex1);
   }

   @Test
   public void testSetAndCreateTwinEdge()
   {
      ExtendedPolytopeVertex vertex1 = getRandomPolytopeVertex();
      ExtendedPolytopeVertex vertex2 = getRandomPolytopeVertex();
      PolytopeHalfEdge halfEdge1 = new PolytopeHalfEdge(vertex1, vertex2);
      PolytopeHalfEdge twinOfHalfEdge1 = halfEdge1.setAndCreateTwinHalfEdge();
      assertTrue(twinOfHalfEdge1.getTwinHalfEdge() == halfEdge1);
      assertTrue(halfEdge1.getTwinHalfEdge() == twinOfHalfEdge1);
      assertTrue(twinOfHalfEdge1.getOriginVertex() == vertex2);
      assertTrue(twinOfHalfEdge1.getDestinationVertex() == vertex1);
   }
   
   @Test
   public void testEpsilonEquals()
   {
      ExtendedPolytopeVertex vertex1 = getRandomPolytopeVertex();
      ExtendedPolytopeVertex vertex2 = getRandomPolytopeVertex();
      PolytopeHalfEdge halfEdge1 = new PolytopeHalfEdge(vertex1, vertex2);
      assertTrue(halfEdge1.epsilonEquals(halfEdge1, 0.0));
      ExtendedPolytopeVertex vertex3 = new ExtendedPolytopeVertex(vertex1.getX() + Epsilons.ONE_THOUSANDTH, vertex1.getY() - Epsilons.ONE_THOUSANDTH, vertex1.getZ() + Epsilons.ONE_THOUSANDTH);
      ExtendedPolytopeVertex vertex4 = new ExtendedPolytopeVertex(vertex2.getX() - Epsilons.ONE_THOUSANDTH, vertex2.getY() - Epsilons.ONE_THOUSANDTH, vertex2.getZ() + Epsilons.ONE_THOUSANDTH);
      PolytopeHalfEdge halfEdge2 = new PolytopeHalfEdge(vertex3, vertex4);
      assertTrue(halfEdge1.epsilonEquals(halfEdge2, Epsilons.ONE_THOUSANDTH*2));
   }
   
   @Test 
   public void testEdgeVector()
   {
      ExtendedPolytopeVertex vertex1 = getRandomPolytopeVertex();
      ExtendedPolytopeVertex vertex2 = getRandomPolytopeVertex();
      
      PolytopeHalfEdge edge = new PolytopeHalfEdge(vertex1, vertex2);
      Vector3DReadOnly edgeVector = edge.getEdgeVector();
      assertTrue(edgeVector.getX() == vertex2.getX() - vertex1.getX());
      assertTrue(edgeVector.getY() == vertex2.getY() - vertex1.getY());
      assertTrue(edgeVector.getZ() == vertex2.getZ() - vertex1.getZ());
      
   }
}
