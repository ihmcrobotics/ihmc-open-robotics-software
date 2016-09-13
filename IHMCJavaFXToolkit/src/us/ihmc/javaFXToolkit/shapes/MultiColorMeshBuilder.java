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
