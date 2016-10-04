package us.ihmc.javaFXToolkit.shapes;

import java.util.List;

import javax.vecmath.Point2f;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3f;

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

      this.faces.add(translateFaces(new int[] {0, 0, 1, 0, 2, 0}, numPoints, numTexCoords));
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

   public void addMesh(float[] points, float[] texCoords, int[] faces, int[] faceSmoothingGroups, float pointsOffsetX, float pointsOffsetY, float pointsOffsetZ)
   {
      int numPoints = this.points.size() / 3;
      int numTexCoords = this.texCoords.size() / 2;
      this.points.add(translatePoints(points, pointsOffsetX, pointsOffsetY, pointsOffsetZ));
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

   public void addLineMesh(float x0, float y0, float z0, float xf, float yf, float zf, float lineWidth)
   {
      float lx = xf - x0;
      float ly = yf - y0;
      float lz = zf - z0;
      float[] linePoints = LineMeshGenerator.generatePoints(lx, ly, lz, lineWidth);
      float[] lineTexCoords = LineMeshGenerator.texCoords;
      int[] lineFaces = LineMeshGenerator.faces;
      int[] lineFaceSmoothingGroups = LineMeshGenerator.faceSmoothingGroups;
      addMesh(linePoints, lineTexCoords, lineFaces, lineFaceSmoothingGroups, x0, y0, z0);
   }

   public void addLineMesh(double x0, double y0, double z0, double xf, double yf, double zf, double lineWidth)
   {
      addLineMesh((float) x0, (float) y0, (float) z0, (float) xf, (float) yf, (float) zf, (float) lineWidth);
   }

   public void addLineMesh(Point3d start, Point3d end, double lineWidth)
   {
      double x0 = start.getX();
      double y0 = start.getY();
      double z0 = start.getZ();
      double xf = end.getX();
      double yf = end.getY();
      double zf = end.getZ();
      addLineMesh(x0, y0, z0, xf, yf, zf, lineWidth);
   }

   public void addMultiLineMesh(Point3d[] points, double lineWidth, boolean close)
   {
      if (points.length < 2)
         return;

      for (int i = 1; i < points.length; i++)
      {
         Point3d start = points[i-1];
         Point3d end = points[i];
         addLineMesh(start, end, lineWidth);
      }

      if (close)
      {
         Point3d start = points[points.length - 1];
         Point3d end = points[0];
         addLineMesh(start, end, lineWidth);
      }
   }

   public void addMultiLineMesh(List<Point3d> points, double lineWidth, boolean close)
   {
      if (points.size() < 2)
         return;

      for (int i = 1; i < points.size(); i++)
      {
         Point3d start = points.get(i-1);
         Point3d end = points.get(i);
         addLineMesh(start, end, lineWidth);
      }

      if (close)
      {
         Point3d start = points.get(points.size() - 1);
         Point3d end = points.get(0);
         addLineMesh(start, end, lineWidth);
      }
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
      return translatePoints(points, offset.getX(), offset.getY(), offset.getZ());
   }

   private float[] translatePoints(float[] points, float offsetX, float offsetY, float offsetZ)
   {
      float[] newPoints = new float[points.length];
      for (int i = 0; i < points.length / 3; i++)
      {
         newPoints[3 * i] = points[3 * i] + offsetX;
         newPoints[3 * i + 1] = points[3 * i + 1] + offsetY;
         newPoints[3 * i + 2] = points[3 * i + 2] + offsetZ;
      }
      return newPoints;
   }

   public Mesh generateMesh()
   {
      if (points.isEmpty() || faces.isEmpty())
         return null;

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
         float halfLx = lx / 2.0f;
         float halfLy = ly / 2.0f;
         float halfLz = lz / 2.0f;

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

   static class LineMeshGenerator
   {
      static float[] generatePoints(float lx, float ly, float lz, float width)
      {
         Vector3f localDirection = new Vector3f(lx, ly, lz);
         float lineLength = localDirection.length();
         localDirection.scale(1.0f / lineLength);

         float halfL = lineLength / 2.0f;

         float points[] = BoxMeshGenerator.generatePoints(width, width, lineLength);

         for (int i = 2; i < points.length; i += 3)
            points[i] += halfL;

            float yaw;
            float pitch;
            if (Math.abs(localDirection.getZ()) < 1.0 - 1.0e-3)
            {
               yaw = (float) Math.atan2(localDirection.getY(), localDirection.getX());
               double xyLength = Math.sqrt(localDirection.getX() * localDirection.getX() + localDirection.getY() * localDirection.getY());
               pitch = (float) Math.atan2(xyLength, localDirection.getZ());
            }
            else
            {
               yaw = 0.0f;
               pitch = localDirection.getZ() >= 0.0 ? 0.0f : (float) Math.PI;
            }

            float cYaw = (float) Math.cos(yaw);
            float sYaw = (float) Math.sin(yaw);

            float cPitch = (float) Math.cos(pitch);
            float sPitch = (float) Math.sin(pitch);

            for (int i = 0; i < points.length; i += 3)
            {
               float x = points[i];
               float y = points[i + 1];
               float z = points[i + 2];
               points[i] = cYaw * cPitch * x - sYaw * y + cYaw * sPitch * z;
               points[i + 1] = sYaw * cPitch * x + cYaw * y + sYaw * sPitch * z;
               points[i + 2] = -sPitch * x + cPitch * z;
            }

         return points;
      }

      static final float texCoords[] = BoxMeshGenerator.texCoords;

      static final int faceSmoothingGroups[] = BoxMeshGenerator.faceSmoothingGroups;

      static final int faces[] = BoxMeshGenerator.faces;
   }
}
