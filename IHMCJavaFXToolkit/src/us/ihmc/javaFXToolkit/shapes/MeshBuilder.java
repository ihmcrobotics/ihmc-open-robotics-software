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

      this.faces.add(translateFaces(new int[] {0, 0, 1, 0, 2, 0}, numPoints, numTexCoords));
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

      this.faces.add(translateFaces(new int[] {0, 0, 1, 1, 2, 2}, numPoints, numTexCoords));
      this.faceSmoothingGroups.add(0);
   }

   public void addPolygon(List<Point3d> polygon)
   {
      addPolygon(polygon, new float[] {0.0f, 0.0f});
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

   public void addCube(float size, Tuple3f cubeOffset)
   {
      addBox(size, size, size, cubeOffset);
   }

   public void addCube(double size, Tuple3d cubeOffset)
   {
      addBox(size, size, size, cubeOffset);
   }

   public void addBox(float lx, float ly, float lz)
   {
      float[] boxPoints = BoxMeshGenerator.generatePoints(lx, ly, lz);
      float[] boxTexCoords = BoxMeshGenerator.texCoords;
      int[] boxFaces = BoxMeshGenerator.faces;
      int[] boxFaceSmoothingGroups = BoxMeshGenerator.faceSmoothingGroups;
      addMesh(boxPoints, boxTexCoords, boxFaces, boxFaceSmoothingGroups);
   }

   public void addBox(float lx, float ly, float lz, Tuple3f boxOffset)
   {
      float[] boxPoints = BoxMeshGenerator.generatePoints(lx, ly, lz);
      float[] boxTexCoords = BoxMeshGenerator.texCoords;
      int[] boxFaces = BoxMeshGenerator.faces;
      int[] boxFaceSmoothingGroups = BoxMeshGenerator.faceSmoothingGroups;
      addMesh(boxPoints, boxTexCoords, boxFaces, boxFaceSmoothingGroups, boxOffset);
   }

   public void addBox(double lx, double ly, double lz, Tuple3d boxOffset)
   {
      addBox((float) lx, (float) ly, (float) lz, new Point3f(boxOffset));
   }

   public void addCylinder(double height, double radius, Tuple3d cylinderOffset)
   {
      addCylinder((float) height, (float) radius, new Point3f(cylinderOffset));
   }

   public void addCylinder(float height, float radius, Tuple3f cylinderOffset)
   {
      float[] cylinderPoints = CylinderMeshGenerator.generatePoints(radius, height, CylinderMeshGenerator.DEFAULT_DIVISIONS);
      float[] cylinderTexCoords = CylinderMeshGenerator.defaultTexCoords;
      int[] cylinderFaces = CylinderMeshGenerator.defaultFaces;
      int[] cylinderFaceSmoothingGroups = CylinderMeshGenerator.defaultFaceSmoothingGroups;
      addMesh(cylinderPoints, cylinderTexCoords, cylinderFaces, cylinderFaceSmoothingGroups, cylinderOffset);
   }

   public void addCone(double height, double radius, Tuple3d cylinderOffset)
   {
      addCone((float) height, (float) radius, new Point3f(cylinderOffset));
   }

   public void addCone(float height, float radius, Tuple3f cylinderOffset)
   {
      float[] cylinderPoints = ConeMeshGenerator.generatePoints(radius, height, CylinderMeshGenerator.DEFAULT_DIVISIONS);
      float[] cylinderTexCoords = ConeMeshGenerator.defaultTexCoords;
      int[] cylinderFaces = ConeMeshGenerator.defaultFaces;
      int[] cylinderFaceSmoothingGroups = ConeMeshGenerator.defaultFaceSmoothingGroups;
      addMesh(cylinderPoints, cylinderTexCoords, cylinderFaces, cylinderFaceSmoothingGroups, cylinderOffset);
   }

   public void addLine(float x0, float y0, float z0, float xf, float yf, float zf, float lineWidth)
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

   public void addLine(double x0, double y0, double z0, double xf, double yf, double zf, double lineWidth)
   {
      addLine((float) x0, (float) y0, (float) z0, (float) xf, (float) yf, (float) zf, (float) lineWidth);
   }

   public void addLine(Point3d start, Point3d end, double lineWidth)
   {
      double x0 = start.getX();
      double y0 = start.getY();
      double z0 = start.getZ();
      double xf = end.getX();
      double yf = end.getY();
      double zf = end.getZ();
      addLine(x0, y0, z0, xf, yf, zf, lineWidth);
   }

   public void addMultiLine(Point3d[] points, double lineWidth, boolean close)
   {
      if (points.length < 2)
         return;

      for (int i = 1; i < points.length; i++)
      {
         Point3d start = points[i - 1];
         Point3d end = points[i];
         addLine(start, end, lineWidth);
      }

      if (close)
      {
         Point3d start = points[points.length - 1];
         Point3d end = points[0];
         addLine(start, end, lineWidth);
      }
   }

   public void addMultiLine(List<Point3d> points, double lineWidth, boolean close)
   {
      if (points.size() < 2)
         return;

      for (int i = 1; i < points.size(); i++)
      {
         Point3d start = points.get(i - 1);
         Point3d end = points.get(i);
         addLine(start, end, lineWidth);
      }

      if (close)
      {
         Point3d start = points.get(points.size() - 1);
         Point3d end = points.get(0);
         addLine(start, end, lineWidth);
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

         float points[] = {-halfLx, -halfLy, -halfLz, halfLx, -halfLy, -halfLz, halfLx, halfLy, -halfLz, -halfLx, halfLy, -halfLz, -halfLx, -halfLy, halfLz,
               halfLx, -halfLy, halfLz, halfLx, halfLy, halfLz, -halfLx, halfLy, halfLz};
         return points;
      }

      static final float texCoords[] = {0, 0, 1, 0, 1, 1, 0, 1};

      static final int faceSmoothingGroups[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

      static final int faces[] = {0, 0, 2, 2, 1, 1, 2, 2, 0, 0, 3, 3, 1, 0, 6, 2, 5, 1, 6, 2, 1, 0, 2, 3, 5, 0, 7, 2, 4, 1, 7, 2, 5, 0, 6, 3, 4, 0, 3, 2, 0, 1,
            3, 2, 4, 0, 7, 3, 3, 0, 6, 2, 2, 1, 6, 2, 3, 0, 7, 3, 4, 0, 1, 2, 5, 1, 1, 2, 4, 0, 0, 3,};
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

   static class CylinderMeshGenerator
   {
      static final int DEFAULT_DIVISIONS = 64;

      static final float[] defaultTexCoords = generateTexCoords(DEFAULT_DIVISIONS);
      static final int[] defaultFaces = generatreFaces(DEFAULT_DIVISIONS);
      static final int[] defaultFaceSmoothingGroups = generateFaceSmoothingGroups(DEFAULT_DIVISIONS);

      static float[] generatePoints(float radius, float height, int divisions)
      {
         float points[] = new float[(divisions * 2 + 2) * 3];
         double dAngle = -2.0 * Math.PI / divisions;
         int index = 0;


         for (int i = 0; i < divisions; i++)
         {
            double a = dAngle * i;
            points[index++] = (float) (Math.sin(a) * radius);
            points[index++] = (float) (Math.cos(a) * radius);
            points[index++] = height;
         }

         for (int i = 0; i < divisions; i++)
         {
            double a = dAngle * i;
            points[index++] = (float) (Math.sin(a) * radius);
            points[index++] = (float) (Math.cos(a) * radius);
            points[index++] = 0.0f;
         }

         // add cap central points
         points[index++] = 0.0f;
         points[index++] = 0.0f;
         points[index++] = height;
         points[index++] = 0.0f;
         points[index++] = 0.0f;
         points[index++] = 0.0f;

         return points;
      }

      static float[] generateTexCoords(int divisions)
      {
         float texCoords[] = new float[((divisions + 1) * 4 + 1) * 2];
         float textureDelta = 1.0f / 256;
         float dA = -1.0f / divisions;

         int index = 0;

         for (int i = 0; i < divisions; i++)
         {
            texCoords[index++] = 1 - dA * i;
            texCoords[index++] = 1 - textureDelta;
         }

         // top edge
         texCoords[index++] = 0;
         texCoords[index++] = 1 - textureDelta;

         for (int i = 0; i < divisions; i++)
         {
            texCoords[index++] = 1 - dA * i;
            texCoords[index++] = textureDelta;
         }

         // bottom edge
         texCoords[index++] = 0;
         texCoords[index++] = textureDelta;

         // add cap central points
         // bottom cap
         for (int i = 0; i <= divisions; i++)
         {
            double a = (i < divisions) ? (dA * i * 2) * Math.PI : 0;
            texCoords[index++] = 0.5f + (float) (Math.sin(a) * 0.5f);
            texCoords[index++] = 0.5f + (float) (Math.cos(a) * 0.5f);
         }

         // top cap
         for (int i = 0; i <= divisions; ++i)
         {
            double a = (i < divisions) ? (dA * i * 2) * Math.PI : 0;
            texCoords[index++] = 0.5f + (float) (Math.sin(a) * 0.5f);
            texCoords[index++] = 0.5f - (float) (Math.cos(a) * 0.5f);
         }

         texCoords[index++] = 0.5f;
         texCoords[index++] = 0.5f;

         return texCoords;
      }

      static int[] generatreFaces(int divisions)
      {
         int faces[] = new int[divisions * 24];

         int fIndex = 0;

         for (int p0 = 0; p0 < divisions; p0++)
         {
            int p1 = p0 + 1;
            int p2 = p0 + divisions;
            int p3 = p1 + divisions;

            faces[fIndex + 0] = p0;
            faces[fIndex + 1] = p0;
            faces[fIndex + 2] = p2;
            faces[fIndex + 3] = p2 + 1;
            faces[fIndex + 4] = p1 == divisions ? 0 : p1;
            faces[fIndex + 5] = p1;
            fIndex += 6;

            faces[fIndex + 0] = p3 % divisions == 0 ? p3 - divisions : p3;
            faces[fIndex + 1] = p3 + 1;
            faces[fIndex + 2] = p1 == divisions ? 0 : p1;
            faces[fIndex + 3] = p1;
            faces[fIndex + 4] = p2;
            faces[fIndex + 5] = p2 + 1;
            fIndex += 6;

         }
         // build cap faces
         int tStart = (divisions + 1) * 2;
         int t1 = (divisions + 1) * 4;
         int p1 = divisions * 2;

         // bottom cap
         for (int p0 = 0; p0 < divisions; p0++)
         {
            int p2 = p0 + 1;
            int t0 = tStart + p0;
            int t2 = t0 + 1;

            // add p0, p1, p2
            faces[fIndex + 0] = p0;
            faces[fIndex + 1] = t0;
            faces[fIndex + 2] = p2 == divisions ? 0 : p2;
            faces[fIndex + 3] = t2;
            faces[fIndex + 4] = p1;
            faces[fIndex + 5] = t1;
            fIndex += 6;
         }

         p1 = divisions * 2 + 1;
         tStart = (divisions + 1) * 3;

         // top cap
         for (int p0 = 0; p0 < divisions; p0++)
         {
            int p2 = p0 + 1 + divisions;
            int t0 = tStart + p0;
            int t2 = t0 + 1;

            faces[fIndex + 0] = p0 + divisions;
            faces[fIndex + 1] = t0;
            faces[fIndex + 2] = p1;
            faces[fIndex + 3] = t1;
            faces[fIndex + 4] = p2 % divisions == 0 ? p2 - divisions : p2;
            faces[fIndex + 5] = t2;
            fIndex += 6;
         }

         return faces;
      }

      static int[] generateFaceSmoothingGroups(int divivions)
      {
         int smoothing[] = new int[divivions * 4];
        for (int i = 0; i < divivions * 2; ++i) 
            smoothing[i] = 1;
        for (int i = divivions * 2; i < divivions * 4; ++i) 
            smoothing[i] = 2;
        return smoothing;
      }
   }

   static class ConeMeshGenerator
   {
      static final int DEFAULT_DIVISIONS = 64;

      static final float[] defaultTexCoords = generateTexCoords(DEFAULT_DIVISIONS);
      static final int[] defaultFaces = generatreFaces(DEFAULT_DIVISIONS);
      static final int[] defaultFaceSmoothingGroups = generateFaceSmoothingGroups(DEFAULT_DIVISIONS);

      static float[] generatePoints(float radius, float height, int divisions)
      {
         float points[] = new float[(divisions + 2) * 3];
         double dAngle = -2.0 * Math.PI / divisions;
         int index = 0;

         for (int i = 0; i < divisions; i++)
         {
            double a = dAngle * i;
            points[index++] = (float) (Math.sin(a) * radius);
            points[index++] = (float) (Math.cos(a) * radius);
            points[index++] = 0.0f;
         }

         // The top
         points[index++] = 0.0f;
         points[index++] = 0.0f;
         points[index++] = height;

         // The base central point
         points[index++] = 0.0f;
         points[index++] = 0.0f;
         points[index++] = 0.0f;

         return points;
      }

      static float[] generateTexCoords(int divisions)
      {
         float texCoords[] = new float[((divisions + 1) * 4 + 1) * 2];
         float textureDelta = 1.0f / 256;
         float dA = -1.0f / divisions;

         int index = 0;

         for (int i = 0; i < divisions; i++)
         {
            texCoords[index++] = 1 - dA * i;
            texCoords[index++] = 1 - textureDelta;
         }

         // top edge
         texCoords[index++] = 0;
         texCoords[index++] = 1 - textureDelta;

         for (int i = 0; i < divisions; i++)
         {
            texCoords[index++] = 1 - dA * i;
            texCoords[index++] = textureDelta;
         }

         // bottom edge
         texCoords[index++] = 0;
         texCoords[index++] = textureDelta;

         // add cap central points
         // bottom cap
         for (int i = 0; i <= divisions; i++)
         {
            double a = (i < divisions) ? (dA * i * 2) * Math.PI : 0;
            texCoords[index++] = 0.5f + (float) (Math.sin(a) * 0.5f);
            texCoords[index++] = 0.5f + (float) (Math.cos(a) * 0.5f);
         }

         // top cap
         for (int i = 0; i <= divisions; ++i)
         {
            double a = (i < divisions) ? (dA * i * 2) * Math.PI : 0;
            texCoords[index++] = 0.5f + (float) (Math.sin(a) * 0.5f);
            texCoords[index++] = 0.5f - (float) (Math.cos(a) * 0.5f);
         }

         texCoords[index++] = 0.5f;
         texCoords[index++] = 0.5f;

         return texCoords;
      }

      static int[] generatreFaces(int divisions)
      {
         int faces[] = new int[divisions * 24];

         int fIndex = 0;

         int topIndex = divisions;
         int baseCenterIndex = divisions + 1;

         for (int p0 = 0; p0 <= divisions; p0++)
         {
            int p1 = (p0 + 1) % divisions;

            faces[fIndex + 0] = baseCenterIndex;
            faces[fIndex + 2] = p1;
            faces[fIndex + 4] = p0;

            faces[fIndex + 1] = 0;
            faces[fIndex + 3] = 0;
            faces[fIndex + 5] = 0;
            fIndex += 6;

            faces[fIndex + 0] = p0;
            faces[fIndex + 2] = p1;
            faces[fIndex + 4] = topIndex;

            faces[fIndex + 1] = 0;
            faces[fIndex + 3] = 0;
            faces[fIndex + 5] = 0;
            fIndex += 6;

         }

         return faces;
      }

      static int[] generateFaceSmoothingGroups(int divisions)
      {
         int smoothing[] = new int[divisions * 4];
        for (int i = 0; i < divisions * 2; ++i) 
            smoothing[i] = 1;
        for (int i = divisions * 2; i < divisions * 4; ++i) 
            smoothing[i] = 2;
        return smoothing;
      }
   }
}
