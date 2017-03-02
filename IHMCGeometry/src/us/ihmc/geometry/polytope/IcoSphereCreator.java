package us.ihmc.geometry.polytope;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import gnu.trove.map.hash.THashMap;

/**
 * From http://blog.andreaskahler.com/2009/06/creating-icosphere-mesh-in-code.html
 *
 */
public class IcoSphereCreator
{
   private class TriangleIndices
   {
      public int v1;
      public int v2;
      public int v3;

      public TriangleIndices(int v1, int v2, int v3)
      {
         this.v1 = v1;
         this.v2 = v2;
         this.v3 = v3;
      }
   }

   private SimpleTriangleMesh geometry;

   private int index;
   private THashMap<Long, Integer> middlePointIndexCache = new THashMap<>();

   // add vertex to mesh, fix position to be on unit sphere, return index
   private int addVertex(Point3D point)
   {
      Vector3D vector = new Vector3D(point);
      vector.normalize();
      point = new Point3D(vector);

      geometry.positions.add(point);
      return index++;
   }

   // return index of point in the middle of p1 and p2
   private int getMiddlePoint(int p1, int p2)
   {
      // first check if we have it already
      boolean firstIsSmaller = p1 < p2;
      long smallerIndex = firstIsSmaller ? p1 : p2;
      long greaterIndex = firstIsSmaller ? p2 : p1;
      long key = (smallerIndex << 32) + greaterIndex;

      if (this.middlePointIndexCache.containsKey(key))
      {
         return middlePointIndexCache.get(key);
      }

      // not in cache, calculate it
      Point3D point1 = this.geometry.positions.get(p1);
      Point3D point2 = this.geometry.positions.get(p2);
      Point3D middle = new Point3D((point1.getX() + point2.getX()) / 2.0, (point1.getY() + point2.getY()) / 2.0, (point1.getZ() + point2.getZ()) / 2.0);

      // add vertex makes sure point is on unit sphere
      int i = addVertex(middle);

      // store it, return index
      this.middlePointIndexCache.put(key, i);
      return i;
   }

   public SimpleTriangleMesh createIcoSphere(int recursionLevel)
   {
      this.geometry = new SimpleTriangleMesh();
      this.middlePointIndexCache = new THashMap<Long, Integer>();
      this.index = 0;

      // create 12 vertices of a icosahedron
      double t = (1.0 + Math.sqrt(5.0)) / 2.0;

      addVertex(new Point3D(-1, t, 0));
      addVertex(new Point3D(1, t, 0));
      addVertex(new Point3D(-1, -t, 0));
      addVertex(new Point3D(1, -t, 0));

      addVertex(new Point3D(0, -1, t));
      addVertex(new Point3D(0, 1, t));
      addVertex(new Point3D(0, -1, -t));
      addVertex(new Point3D(0, 1, -t));

      addVertex(new Point3D(t, 0, -1));
      addVertex(new Point3D(t, 0, 1));
      addVertex(new Point3D(-t, 0, -1));
      addVertex(new Point3D(-t, 0, 1));

      // create 20 triangles of the icosahedron
      ArrayList<TriangleIndices> faces = new ArrayList<TriangleIndices>();

      // 5 faces around point 0
      faces.add(new TriangleIndices(0, 11, 5));
      faces.add(new TriangleIndices(0, 5, 1));
      faces.add(new TriangleIndices(0, 1, 7));
      faces.add(new TriangleIndices(0, 7, 10));
      faces.add(new TriangleIndices(0, 10, 11));

      // 5 adjacent faces
      faces.add(new TriangleIndices(1, 5, 9));
      faces.add(new TriangleIndices(5, 11, 4));
      faces.add(new TriangleIndices(11, 10, 2));
      faces.add(new TriangleIndices(10, 7, 6));
      faces.add(new TriangleIndices(7, 1, 8));

      // 5 faces around point 3
      faces.add(new TriangleIndices(3, 9, 4));
      faces.add(new TriangleIndices(3, 4, 2));
      faces.add(new TriangleIndices(3, 2, 6));
      faces.add(new TriangleIndices(3, 6, 8));
      faces.add(new TriangleIndices(3, 8, 9));

      // 5 adjacent faces
      faces.add(new TriangleIndices(4, 9, 5));
      faces.add(new TriangleIndices(2, 4, 11));
      faces.add(new TriangleIndices(6, 2, 10));
      faces.add(new TriangleIndices(8, 6, 7));
      faces.add(new TriangleIndices(9, 8, 1));

      // refine triangles
      for (int i = 0; i < recursionLevel; i++)
      {
         ArrayList<TriangleIndices> faces2 = new ArrayList<TriangleIndices>();
         for (TriangleIndices tri : faces)
         {
            // replace triangle by 4 triangles
            int a = getMiddlePoint(tri.v1, tri.v2);
            int b = getMiddlePoint(tri.v2, tri.v3);
            int c = getMiddlePoint(tri.v3, tri.v1);

            faces2.add(new TriangleIndices(tri.v1, a, c));
            faces2.add(new TriangleIndices(tri.v2, b, a));
            faces2.add(new TriangleIndices(tri.v3, c, b));
            faces2.add(new TriangleIndices(a, b, c));
         }
         faces = faces2;
      }

      // done, now add triangles to mesh
      for (TriangleIndices tri : faces)
      {
         this.geometry.triangleIndices.add(tri.v1);
         this.geometry.triangleIndices.add(tri.v2);
         this.geometry.triangleIndices.add(tri.v3);
      }

      return this.geometry;
   }

}
