package us.ihmc.javaFXToolkit.shapes;

import java.util.List;

import javax.vecmath.Point2f;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;

import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TIntArrayList;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.TriangleMesh;

public class MeshBuilder
{
   private TFloatArrayList points = new TFloatArrayList();
   private TFloatArrayList texCoords = new TFloatArrayList();
   private TIntArrayList faces = new TIntArrayList();
   private TIntArrayList faceSmoothingGroups = new TIntArrayList();

   public MeshBuilder()
   {
      clear();
   }

   public void clear()
   {
      points.reset();
      texCoords.reset();
      faces.reset();
      faceSmoothingGroups.reset();
   }

   public void addSingleFace(Point3f vertex0, Point3f vertex1, Point3f vertex2, Point2f texture)
   {
      int numPoints = this.points.size() / 3;
      int numTexCoords = this.texCoords.size() / 2;
      points.add(vertex0.getX());
      points.add(vertex0.getY());
      points.add(vertex0.getZ());
      points.add(vertex1.getX());
      points.add(vertex1.getY());
      points.add(vertex1.getZ());
      points.add(vertex2.getX());
      points.add(vertex2.getY());
      points.add(vertex2.getZ());

      texCoords.add(texture.getX());
      texCoords.add(texture.getY());

      this.faces.add(translateFaces(new int[]{0, 0, 1, 0, 2, 0}, numPoints, numTexCoords));
      this.faceSmoothingGroups.add(0);
   }

   public void addSingleFace(Point3d vertex0, Point3d vertex1, Point3d vertex2, Point2f texture)
   {
      int numPoints = this.points.size() / 3;
      int numTexCoords = this.texCoords.size() / 2;
      points.add((float) vertex0.getX());
      points.add((float) vertex0.getY());
      points.add((float) vertex0.getZ());
      points.add((float) vertex1.getX());
      points.add((float) vertex1.getY());
      points.add((float) vertex1.getZ());
      points.add((float) vertex2.getX());
      points.add((float) vertex2.getY());
      points.add((float) vertex2.getZ());

      texCoords.add(texture.getX());
      texCoords.add(texture.getY());

      this.faces.add(translateFaces(new int[]{0, 0, 1, 0, 2, 0}, numPoints, numTexCoords));
      this.faceSmoothingGroups.add(0);
   }

   public void addSingleFace(Point3f vertex0, Point3f vertex1, Point3f vertex2, Point2f texture0, Point2f texture1, Point2f texture2)
   {
      int numPoints = this.points.size() / 3;
      int numTexCoords = this.texCoords.size() / 2;
      points.add(vertex0.getX());
      points.add(vertex0.getY());
      points.add(vertex0.getZ());
      points.add(vertex1.getX());
      points.add(vertex1.getY());
      points.add(vertex1.getZ());
      points.add(vertex2.getX());
      points.add(vertex2.getY());
      points.add(vertex2.getZ());

      texCoords.add(texture0.getX());
      texCoords.add(texture0.getY());
      texCoords.add(texture1.getX());
      texCoords.add(texture1.getY());
      texCoords.add(texture2.getX());
      texCoords.add(texture2.getY());

      this.faces.add(translateFaces(new int[]{0, 0, 1, 1, 2, 2}, numPoints, numTexCoords));
      this.faceSmoothingGroups.add(0);
   }

   public void addPolygon(List<Point3d> polygon)
   {
      addPolygon(polygon, new float[]{0.0f, 0.0f});
   }

   public void addPolygon(List<Point3d> polygon, float[] texture)
   {
      int numPoints = this.points.size() / 3;
      int numTexCoords = this.texCoords.size() / 2;

      Point3d average = new Point3d();

      for (int i = 0; i < polygon.size(); i++)
      {
         Point3d vertex = polygon.get(i);
         average.add(vertex);
         points.add((float) vertex.getX());
         points.add((float) vertex.getY());
         points.add((float) vertex.getZ());

         faces.add(i + numPoints);
         faces.add(0 + numTexCoords);
         faces.add((i + 1) % polygon.size() + numPoints);
         faces.add(0 + numTexCoords);
         faces.add(polygon.size() + numPoints);
         faces.add(0 + numTexCoords);
         faceSmoothingGroups.add(0);
      }
      average.scale(1.0 / polygon.size());
      points.add((float) average.getX());
      points.add((float) average.getY());
      points.add((float) average.getZ());

      texCoords.add(texture[0]);
      texCoords.add(texture[1]);
   }

   public void addMesh(Point3d[] points, float[] texCoords, int[] faces, int[] faceSmoothingGroups)
   {
      int numPoints = this.points.size() / 3;
      int numTexCoords = this.texCoords.size() / 2;
      for (int i = 0; i < points.length; i++)
      {
         this.points.add((float) points[i].getX());
         this.points.add((float) points[i].getY());
         this.points.add((float) points[i].getZ());
      }

      this.texCoords.add(texCoords);
      this.faces.add(translateFaces(faces, numPoints, numTexCoords));
      this.faceSmoothingGroups.add(faceSmoothingGroups);
   }

   public void addMesh(Point3f[] points, float[] texCoords, int[] faces, int[] faceSmoothingGroups)
   {
      int numPoints = this.points.size() / 3;
      int numTexCoords = this.texCoords.size() / 2;
      for (int i = 0; i < points.length; i++)
      {
         this.points.add(points[i].getX());
         this.points.add(points[i].getY());
         this.points.add(points[i].getZ());
      }

      this.texCoords.add(texCoords);
      this.faces.add(translateFaces(faces, numPoints, numTexCoords));
      this.faceSmoothingGroups.add(faceSmoothingGroups);
   }

   public void addMesh(float[] points, float[] texCoords, int[] faces, int[] faceSmoothingGroups)
   {
      int numPoints = this.points.size() / 3;
      int numTexCoords = this.texCoords.size() / 2;
      this.points.add(points);
      this.texCoords.add(texCoords);
      this.faces.add(translateFaces(faces, numPoints, numTexCoords));
      this.faceSmoothingGroups.add(faceSmoothingGroups);
   }

   public void addMesh(TFloatArrayList points, TFloatArrayList texCoords, TIntArrayList faces, TIntArrayList faceSmoothingGroups)
   {
      int numPoints = this.points.size() / 3;
      int numTexCoords = this.texCoords.size() / 2;
      this.points.addAll(points);
      this.texCoords.addAll(texCoords);
      this.faces.add(translateFaces(faces, numPoints, numTexCoords));
      this.faceSmoothingGroups.addAll(faceSmoothingGroups);
   }

   public void addMesh(float[] points, float[] texCoords, int[] faces, int[] faceSmoothingGroups, Tuple3f pointsOffset)
   {
      int numPoints = this.points.size() / 3;
      int numTexCoords = this.texCoords.size() / 2;
      this.points.add(translatePoints(points, pointsOffset));
      this.texCoords.add(texCoords);
      this.faces.add(translateFaces(faces, numPoints, numTexCoords));
      this.faceSmoothingGroups.add(faceSmoothingGroups);
   }

   public void addMesh(MeshBuilder other)
   {
      addMesh(other.points, other.texCoords, other.faces, other.faceSmoothingGroups);
   }

   public void addBoxMesh(float lx, float ly, float lz)
   {
      float[] boxPoints = BoxMeshGenerator.generatePoints(lx, ly, lz);
      float[] boxTexCoords = BoxMeshGenerator.texCoords;
      int[] boxFaces = BoxMeshGenerator.faces;
      int[] boxFaceSmoothingGroups = BoxMeshGenerator.faceSmoothingGroups;
      addMesh(boxPoints, boxTexCoords, boxFaces, boxFaceSmoothingGroups);
   }

   public void addCubeMesh(float size, Tuple3f cubeOffset)
   {
      addBoxMesh(size, size, size, cubeOffset);
   }


   public void addCubeMesh(double size, Tuple3d cubeOffset)
   {
      addBoxMesh(size, size, size, cubeOffset);
   }

   public void addBoxMesh(float lx, float ly, float lz, Tuple3f boxOffset)
   {
      float[] boxPoints = BoxMeshGenerator.generatePoints(lx, ly, lz);
      float[] boxTexCoords = BoxMeshGenerator.texCoords;
      int[] boxFaces = BoxMeshGenerator.faces;
      int[] boxFaceSmoothingGroups = BoxMeshGenerator.faceSmoothingGroups;
      addMesh(boxPoints, boxTexCoords, boxFaces, boxFaceSmoothingGroups, boxOffset);
   }

   public void addBoxMesh(double lx, double ly, double lz, Tuple3d boxOffset)
   {
      addBoxMesh((float) lx, (float) ly, (float) lz, new Point3f(boxOffset));
   }

   private int[] translateFaces(int[] faces, int points, int texCoords)
   {
      int[] newFaces = new int[faces.length];
      for (int i = 0; i < faces.length; i++)
      {
         newFaces[i] = faces[i] + (i % 2 == 0 ? points : texCoords);
      }
      return newFaces;
   }

   private int[] translateFaces(TIntArrayList faces, int points, int texCoords)
   {
      int[] newFaces = new int[faces.size()];
      for (int i = 0; i < faces.size(); i++)
      {
         newFaces[i] = faces.get(i) + (i % 2 == 0 ? points : texCoords);
      }
      return newFaces;
   }

   private float[] translatePoints(float[] points, Tuple3f offset)
   {
      float[] newPoints = new float[points.length];
      for (int i = 0; i < points.length / 3; i++)
      {
         newPoints[3 * i] = points[3 * i] + offset.getX();
         newPoints[3 * i + 1] = points[3 * i + 1] + offset.getY();
         newPoints[3 * i + 2] = points[3 * i + 2] + offset.getZ();
      }
      return newPoints;
   }

   public Mesh generateMesh()
   {
      TriangleMesh mesh = new TriangleMesh();
      mesh.getPoints().addAll(points.toArray());
      mesh.getTexCoords().addAll(texCoords.toArray());
      mesh.getFaces().addAll(faces.toArray());
      mesh.getFaceSmoothingGroups().addAll(faceSmoothingGroups.toArray());
      return mesh;
   }

   static class BoxMeshGenerator
   {
      static float[] generatePoints(float lx, float ly, float lz)
      {
         float halfLx = lx / 2f;
         float halfLy = ly / 2f;
         float halfLz = lz / 2f;

         float points[] = {
             -halfLx, -halfLy, -halfLz,
              halfLx, -halfLy, -halfLz,
              halfLx,  halfLy, -halfLz,
             -halfLx,  halfLy, -halfLz,
             -halfLx, -halfLy,  halfLz,
              halfLx, -halfLy,  halfLz,
              halfLx,  halfLy,  halfLz,
             -halfLx,  halfLy,  halfLz};
         return points;
      }

      static final float texCoords[] = {0, 0, 1, 0, 1, 1, 0, 1};

      static final int faceSmoothingGroups[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

      static final int faces[] = {
          0, 0, 2, 2, 1, 1,
          2, 2, 0, 0, 3, 3,            
          1, 0, 6, 2, 5, 1,
          6, 2, 1, 0, 2, 3,            
          5, 0, 7, 2, 4, 1,
          7, 2, 5, 0, 6, 3,
          4, 0, 3, 2, 0, 1,
          3, 2, 4, 0, 7, 3,            
          3, 0, 6, 2, 2, 1,
          6, 2, 3, 0, 7, 3,
          4, 0, 1, 2, 5, 1,
          1, 2, 4, 0, 0, 3,
      };
   }
}
