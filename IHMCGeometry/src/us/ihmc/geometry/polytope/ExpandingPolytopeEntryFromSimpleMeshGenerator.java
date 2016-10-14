package us.ihmc.geometry.polytope;

import java.util.ArrayList;

import javax.vecmath.Point3d;

import gnu.trove.map.hash.THashMap;

public class ExpandingPolytopeEntryFromSimpleMeshGenerator
{

   public ExpandingPolytopeEntry generateExpandingPolytope(SimpleTriangleMesh mesh)
   {
      ArrayList<Point3d> positions = mesh.positions;
      ArrayList<Integer> triangleIndices = mesh.triangleIndices;

      int numberOfTriangles = triangleIndices.size() / 3;

      THashMap<Point3d, ArrayList<ExpandingPolytopeEntry>> entriesContainingThisVertexMap = new THashMap<>();

      ArrayList<ExpandingPolytopeEntry> entries = new ArrayList<>();

      for (int i = 0; i < numberOfTriangles; i++)
      {
         Point3d pointOne = positions.get(triangleIndices.get(i * 3));
         Point3d pointTwo = positions.get(triangleIndices.get(i * 3 + 1));
         Point3d pointThree = positions.get(triangleIndices.get(i * 3 + 2));

         ExpandingPolytopeEntry entry = new ExpandingPolytopeEntry(pointOne, pointTwo, pointThree);
         entries.add(entry);

         addToMap(entriesContainingThisVertexMap, entry);
      }

      ArrayList<ExpandingPolytopeEntry> entriesToLinkUp = new ArrayList<>();
      entriesToLinkUp.add(entries.get(0));

      while (!entriesToLinkUp.isEmpty())
      {
         ExpandingPolytopeEntry entryToLinkUp = entriesToLinkUp.remove(entriesToLinkUp.size() - 1);

         for (int i = 0; i < 3; i++)
         {
            Point3d vertex = entryToLinkUp.getVertex(i);
            ArrayList<ExpandingPolytopeEntry> entryiesContainingThisVertex = entriesContainingThisVertexMap.get(vertex);

            for (ExpandingPolytopeEntry entry : entryiesContainingThisVertex)
            {
               boolean addedAdjacentTriangle = entryToLinkUp.setAdjacentTriangleIfPossible(entry);
               if (addedAdjacentTriangle)
               {
                  entriesToLinkUp.add(entry);
               }
            }
         }

      }

      return entries.get(0);
   }

   private void addToMap(THashMap<Point3d, ArrayList<ExpandingPolytopeEntry>> entriesContainingThisVertexMap, ExpandingPolytopeEntry entry)
   {
      for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++)
      {
         Point3d vertex = entry.getVertex(vertexIndex);

         ArrayList<ExpandingPolytopeEntry> arrayList = entriesContainingThisVertexMap.get(vertex);
         if (arrayList == null)
         {
            arrayList = new ArrayList<>();
            entriesContainingThisVertexMap.put(vertex, arrayList);
         }
         arrayList.add(entry);
      }

   }
}
