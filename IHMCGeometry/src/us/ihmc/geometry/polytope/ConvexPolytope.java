package us.ihmc.geometry.polytope;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class ConvexPolytope implements SupportingVertexHolder
{
   private final ArrayList<PolytopeVertex> vertices = new ArrayList<>();

   private boolean boundingBoxNeedsUpdating = false;
   private final BoundingBox3d boundingBox = new BoundingBox3d(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY,
                                                               Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

   public ConvexPolytope()
   {
   }

   public ConvexPolytope(ConvexPolytope polytope)
   {
      for (PolytopeVertex vertex : polytope.vertices)
      {
         this.vertices.add(new PolytopeVertex(vertex));
      }

      boundingBoxNeedsUpdating = true;
   }

   public void getBoundingBox(BoundingBox3d boundingBoxToPack)
   {
      if (boundingBoxNeedsUpdating)
      {
         updateBoundingBox();
         boundingBoxNeedsUpdating = false;
      }

      boundingBoxToPack.set(boundingBox);
   }

   private void updateBoundingBox()
   {
      double xMin = Double.POSITIVE_INFINITY;
      double yMin = Double.POSITIVE_INFINITY;
      double zMin = Double.POSITIVE_INFINITY;

      double xMax = Double.NEGATIVE_INFINITY;
      double yMax = Double.NEGATIVE_INFINITY;
      double zMax = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < vertices.size(); i++)
      {
         PolytopeVertex polytopeVertex = vertices.get(i);
         double x = polytopeVertex.getX();
         double y = polytopeVertex.getY();
         double z = polytopeVertex.getZ();

         if (x < xMin)
            xMin = x;
         if (y < yMin)
            yMin = y;
         if (z < zMin)
            zMin = z;

         if (x > xMax)
            xMax = x;
         if (y > yMax)
            yMax = y;
         if (z > zMax)
            zMax = z;
      }

      boundingBox.set(xMin, yMin, zMin, xMax, yMax, zMax);
   }

   public void copyVerticesFrom(ConvexPolytope polytope)
   {
      int numberOfVertices = this.vertices.size();

      if (numberOfVertices != polytope.vertices.size())
      {
         throw new RuntimeException("Vertices are not same size!");
      }

      for (int i = 0; i < numberOfVertices; i++)
      {
         this.vertices.get(i).setPosition(polytope.vertices.get(i));
      }

      boundingBoxNeedsUpdating = true;
   }

   public ArrayList<PolytopeVertex> getVertices()
   {
      return vertices;
   }

   public void addVertices(Point3d[] polytopePoints)
   {
      for (int i = 0; i < polytopePoints.length; i++)
      {
         addVertex(polytopePoints[i]);
      }

      boundingBoxNeedsUpdating = true;
   }

   public PolytopeVertex addVertex(Point3d position)
   {
      PolytopeVertex vertex = new PolytopeVertex(position);
      vertices.add(vertex);
      boundingBoxNeedsUpdating = true;
      return vertex;
   }

   public PolytopeVertex addVertex(double x, double y, double z)
   {
      PolytopeVertex vertex = new PolytopeVertex(x, y, z);
      vertices.add(vertex);
      boundingBoxNeedsUpdating = true;
      return vertex;
   }

   public PolytopeVertex addVertex(double[] xyzValues)
   {
      PolytopeVertex vertex = new PolytopeVertex(xyzValues[0], xyzValues[1], xyzValues[2]);
      vertices.add(vertex);
      boundingBoxNeedsUpdating = true;
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

   public ArrayList<PolytopeVertex[]> getEdges()
   {
      //TODO: Make this more efficient, and the representation of edges in a Polytope in general.
      ArrayList<PolytopeVertex[]> edgesToReturn = new ArrayList<>();

      for (int i = 0; i < vertices.size(); i++)
      {
         PolytopeVertex vertex = vertices.get(i);
         int numberOfConnectingVertices = vertex.getNumberOfConnectingVertices();
         for (int j = 0; j < numberOfConnectingVertices; j++)
         {
            PolytopeVertex connectingVertex = vertex.getConnectingVertex(j);

            if (!alreadyHaveEdgeInList(edgesToReturn, vertex, connectingVertex))
            {
               edgesToReturn.add(new PolytopeVertex[] {vertex, connectingVertex});
            }
         }
      }

      return edgesToReturn;
   }

   private boolean alreadyHaveEdgeInList(ArrayList<PolytopeVertex[]> listOfEdges, PolytopeVertex vertexOne, PolytopeVertex vertexTwo)
   {
      for (int k = 0; k < listOfEdges.size(); k++)
      {
         PolytopeVertex[] edgeToReturn = listOfEdges.get(k);
         if (((edgeToReturn[0] == vertexOne) && (edgeToReturn[1] == vertexTwo)) || ((edgeToReturn[0] == vertexTwo) && (edgeToReturn[1] == vertexOne)))
         {
            return true;
         }
      }

      return false;
   }

   public void applyTransform(RigidBodyTransform transform)
   {
      for (int i = 0; i < vertices.size(); i++)
      {
         PolytopeVertex polytopeVertex = vertices.get(i);
         polytopeVertex.applyTransform(transform);
      }

      boundingBoxNeedsUpdating = true;
   }

   @Override
   public Point3d getSupportingVertex(Vector3d supportDirection)
   {
      // Naive implementation. Just search through all of them.
      // TODO: Smart downhill march along edges. But will require always having the edges...

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

      return bestVertex.getPosition();
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
