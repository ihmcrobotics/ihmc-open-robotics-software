package us.ihmc.javaFXToolkit.shapes;

import java.util.List;

import javax.vecmath.Point2f;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;

import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TIntArrayList;
import javafx.collections.ObservableFloatArray;
import javafx.collections.ObservableIntegerArray;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.ObservableFaceArray;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.shape.VertexFormat;
import us.ihmc.graphics3DAdapter.graphics.MeshDataHolder;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.javaFXToolkit.shapes.meshGenerators.BoxMeshGenerator;
import us.ihmc.javaFXToolkit.shapes.meshGenerators.ConeMeshGenerator;
import us.ihmc.javaFXToolkit.shapes.meshGenerators.CylinderMeshGenerator;
import us.ihmc.javaFXToolkit.shapes.meshGenerators.FXMeshDataHolder;
import us.ihmc.javaFXToolkit.shapes.meshGenerators.LineMeshGenerator;

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

   public void addMesh(MeshBuilder other)
   {
      addMesh(other.points, other.texCoords, other.faces, other.faceSmoothingGroups);
   }

   public void addMesh(MeshDataHolder meshDataHolder)
   {
      addMesh(JavaFXMeshDataInterpreter.interpretMeshData(meshDataHolder));
   }

   public void addMesh(TriangleMesh triangleMesh)
   {
      int[] faces;
      float[] points = triangleMesh.getPoints().toArray(null);
      float[] texCoords = triangleMesh.getTexCoords().toArray(null);
      int[] faceSmoothingGroups = triangleMesh.getFaceSmoothingGroups().toArray(null);

      if (triangleMesh.getVertexFormat() == VertexFormat.POINT_NORMAL_TEXCOORD)
         faces = getFacesWithoutNormalIndices(triangleMesh.getFaces().toArray(null));
      else
         faces = triangleMesh.getFaces().toArray(null);
      addMesh(points, texCoords, faces, faceSmoothingGroups);
   }

   public void addMesh(ObservableFloatArray points, ObservableFloatArray texCoords, ObservableFaceArray faces, ObservableIntegerArray faceSmoothingGroups)
   {
      addMesh(points.toArray(null), texCoords.toArray(null), faces.toArray(null), faceSmoothingGroups.toArray(null));
   }

   public void addMesh(FXMeshDataHolder meshData)
   {
      addMesh(meshData.getVertexCoordinates(), meshData.getTextureCoordinates(), meshData.getFaceIndices(), meshData.getFaceSmoothingGroups());
   }

   public void addMesh(FXMeshDataHolder meshData, Tuple3d offset)
   {
      addMesh(meshData, new Point3f(offset));
   }

   public void addMesh(FXMeshDataHolder meshData, Tuple3f offset)
   {
      addMesh(meshData.getVertexCoordinates(), meshData.getTextureCoordinates(), meshData.getFaceIndices(), meshData.getFaceSmoothingGroups(), offset);
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
      float[] cylinderPoints = CylinderMeshGenerator.generatePoints(height, radius, CylinderMeshGenerator.DEFAULT_DIVISIONS);
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
      float[] cylinderPoints = ConeMeshGenerator.generatePoints(radius, height, ConeMeshGenerator.DEFAULT_DIVISIONS);
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

   private static int[] translateFaces(int[] faces, int points, int texCoords)
   {
      int[] newFaces = new int[faces.length];
      for (int i = 0; i < faces.length; i++)
      {
         newFaces[i] = faces[i] + (i % 2 == 0 ? points : texCoords);
      }
      return newFaces;
   }

   private static int[] translateFaces(TIntArrayList faces, int points, int texCoords)
   {
      int[] newFaces = new int[faces.size()];
      for (int i = 0; i < faces.size(); i++)
      {
         newFaces[i] = faces.get(i) + (i % 2 == 0 ? points : texCoords);
      }
      return newFaces;
   }

   private static float[] translatePoints(float[] points, Tuple3f offset)
   {
      return translatePoints(points, offset.getX(), offset.getY(), offset.getZ());
   }

   private static float[] translatePoints(float[] points, float offsetX, float offsetY, float offsetZ)
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

   private static int[] getFacesWithoutNormalIndices(int[] facesWithNormals)
   {
      int[] facesWithoutNormals = new int[2 * facesWithNormals.length / 3];

      int index = 0;
      for (int i = 0; i < facesWithNormals.length; i += 3)
      {
         facesWithoutNormals[index++] = facesWithNormals[i];
         facesWithoutNormals[index++] = facesWithNormals[i + 2];
      }

      return facesWithoutNormals;
   }

   public Mesh generateMesh()
   {
      if (points.isEmpty() || faces.isEmpty())
         return null;

      TriangleMesh mesh = new TriangleMesh();
      mesh.getPoints().addAll(points.toArray());
      mesh.getTexCoords().addAll(texCoords.toArray());
      mesh.getFaces().addAll(faces.toArray());
//      mesh.getFaceSmoothingGroups().addAll(faceSmoothingGroups.toArray());
      return mesh;
   }
}
