package us.ihmc.geometry.polytope;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public class ConvexPolytope
{
   private final ArrayList<PolytopeVertex> vertices = new ArrayList<>();

   public ConvexPolytope()
   {
   }

   public ConvexPolytope(ConvexPolytope polytope)
   {
      for (PolytopeVertex vertex : polytope.vertices)
      {
         this.vertices.add(new PolytopeVertex(vertex));
      }
   }

   public void addVertices(Point3d[] polytopePoints)
   {
      for (int i=0; i<polytopePoints.length; i++)
      {
         addVertex(polytopePoints[i]);
      }
   }

   public PolytopeVertex addVertex(Point3d position)
   {
      PolytopeVertex vertex = new PolytopeVertex(position);
      vertices.add(vertex);
      return vertex;
   }

   public PolytopeVertex addVertex(double x, double y, double z)
   {
      PolytopeVertex vertex = new PolytopeVertex(x, y, z);
      vertices.add(vertex);
      return vertex;
   }

   public void addEdge(PolytopeVertex vertexOne, PolytopeVertex vertexTwo)
   {
      vertexOne.addConnectingVertex(vertexTwo);
      vertexTwo.addConnectingVertex(vertexOne);
   }

   public int getNumberOfVertices()
   {
      return vertices.size();
   }

   public PolytopeVertex getVertex(int index)
   {
      return vertices.get(index);
   }

   public int getNumberOfEdges()
   {
      int numberOfEdges = 0;

      for (int i = 0; i < vertices.size(); i++)
      {
         numberOfEdges = numberOfEdges + vertices.get(i).getNumberOfConnectingVertices();
      }

      numberOfEdges = numberOfEdges / 2;

      return numberOfEdges;
   }

   public void applyTransform(RigidBodyTransform transform)
   {
      for (int i = 0; i < vertices.size(); i++)
      {
         PolytopeVertex polytopeVertex = vertices.get(i);
         polytopeVertex.applyTransform(transform);
      }
   }

   public PolytopeVertex getSupportingVertex(Vector3d supportDirection)
   {
      // Naive implementation. Just search through all of them.

      double maxDotSquared = Double.NEGATIVE_INFINITY;
      PolytopeVertex bestVertex = null;

      int numberOfVertices = vertices.size();
      for (int i = 0; i < numberOfVertices; i++)
      {
         PolytopeVertex vertex = vertices.get(i);
         double dotProduct = vertex.dot(supportDirection);
         if (dotProduct > maxDotSquared)
         {
            maxDotSquared = dotProduct;
            bestVertex = vertex;
         }
      }

      return bestVertex;
   }
   
   public String toString()
   {
      String string = "";
      
      for (PolytopeVertex vertex : vertices)
      {
         string = string + "\n" + vertex;
      }
      
      return string;
   }


}
