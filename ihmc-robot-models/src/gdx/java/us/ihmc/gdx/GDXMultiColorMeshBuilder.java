package us.ihmc.gdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;

/**
 * TODO: Make interface and use Color from ihmc-graphics-description
 */
public class GDXMultiColorMeshBuilder
{
   private static final int DEFAULT_RES = 32;

   private int hueResolution = 256;
   private int saturationResolution = -1;
   private int brightnessResolution = -1;

   GDXMeshBuilder meshBuilder = new GDXMeshBuilder();

   public GDXMultiColorMeshBuilder()
   {
   }

   public void addCylinder(double height, double radius, Tuple3DReadOnly offset, AxisAngle orientation, Color color)
   {
      addMesh(MeshDataGenerator.Cylinder(radius, height, DEFAULT_RES), offset, orientation, color);
   }

   public void addCone(double height, double radius, Tuple3DReadOnly offset, AxisAngle orientation, Color color)
   {
      addMesh(MeshDataGenerator.Cone(height, radius, DEFAULT_RES), offset, orientation, color);
   }

   public void addSphere(float radius, Tuple3DReadOnly offset, Color color)
   {
      addMesh(MeshDataGenerator.Sphere(radius, DEFAULT_RES, DEFAULT_RES), offset, new AxisAngle(), color);
   }

   public void addBox(double lx, double ly, double lz, Tuple3DReadOnly offset, Color color)
   {
      addMesh(MeshDataGenerator.Cube(lx, ly, lz, true, null), offset, new AxisAngle(), color);
   }

   public void addMesh(MeshDataHolder meshDataHolder, Tuple3DReadOnly offset, AxisAngle orientation, Color color)
   {
      meshBuilder.addMesh(createMeshDataWithColor(meshDataHolder, color), offset, orientation);
   }

   private MeshDataHolder createMeshDataWithColor(MeshDataHolder input, Color color)
   {
      if (input == null)
         return null;
      Point3D32[] vertices = input.getVertices();
      int[] triangleIndices = input.getTriangleIndices();
      Vector3D32[] vertexNormals = input.getVertexNormals();
      TexCoord2f[] inputTexturePoints = input.getTexturePoints();
      TexCoord2f[] outputTexturePoints = new TexCoord2f[inputTexturePoints.length];
      float[] textureLocation = getTextureLocation(color);
      for (int i = 0; i < inputTexturePoints.length; i++)
         outputTexturePoints[i] = new TexCoord2f(textureLocation);
      return new MeshDataHolder(vertices, outputTexturePoints, triangleIndices, vertexNormals);
   }

   public float[] getTextureLocation(Color color)
   {
      float x;
      float[] hsv = new float[3];
      color.toHsv(hsv);
      x = (float) (hsv[0] / 360.0);
      float y = 0.5f;
      return new float[] {x, y};
   }

   public Mesh generateMesh()
   {
      return meshBuilder.generateMesh();
   }

//   public Material generateMaterial(AssetManager assetManager)
//   {
//      Material material = new Material(assetManager, "Common/MatDefs/Light/Lighting.j3md");
//      Texture texture = assetManager.loadTexture("palette.png");
//      material.setTexture("DiffuseMap", texture);
//      material.createMeshDataWithColor("Diffuse", ColorRGBA.White);
//      return material;
//   }
}