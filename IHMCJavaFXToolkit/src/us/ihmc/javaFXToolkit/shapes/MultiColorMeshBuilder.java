package us.ihmc.javaFXToolkit.shapes;

import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;

import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TIntArrayList;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import us.ihmc.javaFXToolkit.shapes.MeshBuilder.BoxMeshGenerator;
import us.ihmc.javaFXToolkit.shapes.MeshBuilder.LineMeshGenerator;

public class MultiColorMeshBuilder
{
   private final MeshBuilder meshBuilder = new MeshBuilder();
   private TextureColorPalette colorPalette;

   public MultiColorMeshBuilder()
   {
      colorPalette = new TextureColorPalette2D();
   }

   public MultiColorMeshBuilder(TextureColorPalette colorPalette)
   {
      this.colorPalette = colorPalette;
   }

   public void changeColorPalette(TextureColorPalette newColorPalette)
   {
      colorPalette = newColorPalette;
   }

   public void clear()
   {
      meshBuilder.clear();
   }

   public void addMesh(float[] points, int[] faces, int[] faceSmoothingGroups, Color color)
   {
      int[] localFaces = new int[faces.length];
      System.arraycopy(faces, 0, localFaces, 0, faces.length);
      for (int i = 1; i < localFaces.length; i += 2)
         localFaces[i] = 0;
      float[] texCoords = colorPalette.getTextureLocation(color);
      meshBuilder.addMesh(points, texCoords, localFaces, faceSmoothingGroups);
   }

   public void addMesh(float[] points, int[] faces, int[] faceSmoothingGroups, Tuple3f pointsOffset, Color color)
   {
      int[] localFaces = new int[faces.length];
      System.arraycopy(faces, 0, localFaces, 0, faces.length);
      for (int i = 1; i < localFaces.length; i += 2)
         localFaces[i] = 0;
      float[] texCoords = colorPalette.getTextureLocation(color);
      meshBuilder.addMesh(points, texCoords, localFaces, faceSmoothingGroups, pointsOffset);
   }

   public void addMesh(float[] points, int[] faces, int[] faceSmoothingGroups, float pointsOffsetX, float pointsOffsetY, float pointsOffsetZ, Color color)
   {
      int[] localFaces = new int[faces.length];
      System.arraycopy(faces, 0, localFaces, 0, faces.length);
      for (int i = 1; i < localFaces.length; i += 2)
         localFaces[i] = 0;
      float[] texCoords = colorPalette.getTextureLocation(color);
      meshBuilder.addMesh(points, texCoords, localFaces, faceSmoothingGroups, pointsOffsetX, pointsOffsetY, pointsOffsetZ);
   }

   public void addMesh(TFloatArrayList points, TIntArrayList faces, TIntArrayList faceSmoothingGroups, Color color)
   {
      for (int i = 1; i < faces.size(); i += 2)
         faces.set(i, 0);
      float[] texCoords = colorPalette.getTextureLocation(color);
      meshBuilder.addMesh(points.toArray(), texCoords, faces.toArray(), faceSmoothingGroups.toArray());
   }

   public void addMesh(TFloatArrayList points, TIntArrayList faces, TIntArrayList faceSmoothingGroups, Tuple3f pointsOffset, Color color)
   {
      for (int i = 1; i < faces.size(); i += 2)
         faces.set(i, 0);
      float[] texCoords = colorPalette.getTextureLocation(color);
      meshBuilder.addMesh(points.toArray(), texCoords, faces.toArray(), faceSmoothingGroups.toArray(), pointsOffset);
   }

   public void addPolyon(List<Point3d> polygon, Color color)
   {
      meshBuilder.addPolygon(polygon, colorPalette.getTextureLocation(color));
   }

   public void addBoxMesh(float lx, float ly, float lz, Color color)
   {
      float[] points = BoxMeshGenerator.generatePoints(lx, ly, lz);
      int[] faces = BoxMeshGenerator.faces;
      int[] faceSmoothingGroups = BoxMeshGenerator.faceSmoothingGroups;
      addMesh(points, faces, faceSmoothingGroups, color);
   }

   public void addBoxMesh(float lx, float ly, float lz, Tuple3f pointsOffset, Color color)
   {
      float[] points = BoxMeshGenerator.generatePoints(lx, ly, lz);
      int[] faces = BoxMeshGenerator.faces;
      int[] faceSmoothingGroups = BoxMeshGenerator.faceSmoothingGroups;
      addMesh(points, faces, faceSmoothingGroups, pointsOffset, color);
   }

   public void addBoxMesh(double lx, double ly, double lz, Tuple3d pointsOffset, Color color)
   {
      addBoxMesh((float) lx, (float) ly, (float) lz, new Point3f(pointsOffset), color);
   }

   public void addCubeMesh(float size, Color color)
   {
      addBoxMesh(size, size, size, color);
   }

   public void addCubeMesh(float size, Tuple3f pointsOffset, Color color)
   {
      addBoxMesh(size, size, size, pointsOffset, color);
   }

   public void addCubeMesh(double size, Tuple3d pointsOffset, Color color)
   {
      addBoxMesh(size, size, size, pointsOffset, color);
   }

   public void addCubeMesh(double size, double xOffset, double yOffset, double zOffset, Color color)
   {
      addBoxMesh(size, size, size, new Point3d(xOffset, yOffset, zOffset), color);
   }

   public void addLineMesh(float x0, float y0, float z0, float xf, float yf, float zf, float lineWidth, Color color)
   {
      float lx = xf - x0;
      float ly = yf - y0;
      float lz = zf - z0;
      float[] linePoints = LineMeshGenerator.generatePoints(lx, ly, lz, lineWidth);
      int[] lineFaces = LineMeshGenerator.faces;
      int[] lineFaceSmoothingGroups = LineMeshGenerator.faceSmoothingGroups;
      addMesh(linePoints, lineFaces, lineFaceSmoothingGroups, x0, y0, z0, color);
   }

   public void addLineMesh(double x0, double y0, double z0, double xf, double yf, double zf, double lineWidth, Color color)
   {
      addLineMesh((float) x0, (float) y0, (float) z0, (float) xf, (float) yf, (float) zf, (float) lineWidth, color);
   }

   public void addLineMesh(Point3d start, Point3d end, double lineWidth, Color color)
   {
      double x0 = start.getX();
      double y0 = start.getY();
      double z0 = start.getZ();
      double xf = end.getX();
      double yf = end.getY();
      double zf = end.getZ();
      addLineMesh(x0, y0, z0, xf, yf, zf, lineWidth, color);
   }

   public void addMultiLineMesh(Point3d[] points, double lineWidth, Color color, boolean close)
   {
      if (points.length < 2)
         return;

      for (int i = 1; i < points.length; i++)
      {
         Point3d start = points[i-1];
         Point3d end = points[i];
         addLineMesh(start, end, lineWidth, color);
      }

      if (close)
      {
         Point3d start = points[points.length - 1];
         Point3d end = points[0];
         addLineMesh(start, end, lineWidth, color);
      }
   }

   public void addMultiLineMesh(List<Point3d> points, double lineWidth, Color color, boolean close)
   {
      if (points.size() < 2)
         return;

      for (int i = 1; i < points.size(); i++)
      {
         Point3d start = points.get(i-1);
         Point3d end = points.get(i);
         addLineMesh(start, end, lineWidth, color);
      }

      if (close)
      {
         Point3d start = points.get(points.size() - 1);
         Point3d end = points.get(0);
         addLineMesh(start, end, lineWidth, color);
      }
   }

   public Mesh generateMesh()
   {
      return meshBuilder.generateMesh();
   }

   public Material generateMaterial()
   {
      PhongMaterial material = new PhongMaterial();
      material.setDiffuseMap(colorPalette.getColorPalette());
      return material;
   }
}
