package us.ihmc.geometry.polytope;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Point3D;

import gnu.trove.map.hash.THashMap;

public class ConvexPolytopeFromExpandingPolytopeEntryGenerator
{

   public ConvexPolytope generateConvexPolytope(ExpandingPolytopeEntry expandingPolytope)
   {
      ConvexPolytope convexPolytope = new ConvexPolytope();

      ArrayList<ExpandingPolytopeEntry> triangles = new ArrayList<>();
      expandingPolytope.getAllConnectedTriangles(triangles);

      THashMap<Point3D, PolytopeVertex> pointsToPolytopeVertices = new THashMap<>();

      for (ExpandingPolytopeEntry triangle : triangles)
      {
         for (int i = 0; i < 3; i++)
         {
            Point3D vertex = triangle.getVertex(i);

            if (!pointsToPolytopeVertices.containsKey(vertex))
            {
               PolytopeVertex polytopeVertex = convexPolytope.addVertex(vertex);
               pointsToPolytopeVertices.put(vertex, polytopeVertex);
            }
         }
      }

      for (ExpandingPolytopeEntry triangle : triangles)
      {
         for (int i = 0; i < 3; i++)
         {
            Point3D vertex = triangle.getVertex(i);
            Point3D nextVertex = triangle.getVertex((i + 1) % 3);

            PolytopeVertex polytopeVertex = pointsToPolytopeVertices.get(vertex);
            PolytopeVertex nextPolytopeVertex = pointsToPolytopeVertices.get(nextVertex);

            convexPolytope.addEdge(polytopeVertex, nextPolytopeVertex);
         }
      }

      return convexPolytope;
   }
}
