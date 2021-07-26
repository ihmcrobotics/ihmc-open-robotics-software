package us.ihmc.jme;

import com.jme3.asset.AssetManager;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.scene.Mesh;
import com.jme3.texture.Texture;
import javafx.scene.paint.Color;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;

import java.util.List;

public class JMEMultiColorMeshBuilder
{
   private static final int DEFAULT_RES = 32;

   private int hueResolution = 256;
   private int saturationResolution = -1;
   private int brightnessResolution = -1;

   JMEMeshBuilder meshBuilder = new JMEMeshBuilder();

   public JMEMultiColorMeshBuilder()
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

   /**
    * Add a 2D polygon to this builder.
    *
    * @param transformToWorld the transform from the polygon's local coordinates to world. Not
    *                         modified.
    * @param polygon          the polygon to render.
    * @param color            color of the polygon. Color accuracy depends on the color palette in use.
    */
   public void addPolygon(RigidBodyTransformReadOnly transformToWorld, ConvexPolygon2DReadOnly polygon, Color color)
   {
      addMesh(MeshDataGenerator.Polygon(transformToWorld, polygon), color);
   }

   /**
    * Add a 3D line to this builder.
    *
    * @param x0        x-coordinate of the line start.
    * @param y0        y-coordinate of the line start.
    * @param z0        z-coordinate of the line start.
    * @param xf        x-coordinate of the line end.
    * @param yf        y-coordinate of the line end.
    * @param zf        z-coordinate of the line end.
    * @param lineWidth width of the line.
    * @param color     color of the line. Color accuracy depends on the color palette in use.
    */
   public void addLine(double x0, double y0, double z0, double xf, double yf, double zf, double lineWidth, Color color)
   {
      addMesh(MeshDataGenerator.Line(x0, y0, z0, xf, yf, zf, lineWidth), color);
   }

   /**
    * Add a 3D line to this builder.
    *
    * @param start     start coordinate of the line. Not modified.
    * @param end       end coordinate of the line. Not modified.
    * @param lineWidth width of the line.
    * @param color     color of the line. Color accuracy depends on the color palette in use.
    */
   public void addLine(Tuple3DReadOnly start, Tuple3DReadOnly end, double lineWidth, Color color)
   {
      addLine(start.getX(), start.getY(), start.getZ(), end.getX(), end.getY(), end.getZ(), lineWidth, color);
   }

   /**
    * Add a series of connected 2D lines to this builder.
    *
    * @param transformToWorld the transform from the mult-line local coordinates to world. Not
    *                         modified.
    * @param points           coordinates of the line end points. Not modified.
    * @param lineWidth        width of the lines.
    * @param color            color of the multi-line. Color accuracy depends on the color palette in
    *                         use.
    * @param close            whether the end of the given array of points should be connected to the
    *                         beginning or not.
    */
   public void addMultiLine(RigidBodyTransformReadOnly transformToWorld, List<? extends Point2DReadOnly> points, double lineWidth, Color color, boolean close)
   {
      if (points.size() < 2)
         return;

      Point3D start = new Point3D();
      Point3D end = new Point3D();

      for (int i = 1; i < points.size(); i++)
      {
         Point2DReadOnly start2d = points.get(i - 1);
         Point2DReadOnly end2d = points.get(i);

         start.set(start2d.getX(), start2d.getY(), 0.0);
         end.set(end2d.getX(), end2d.getY(), 0.0);
         transformToWorld.transform(start);
         transformToWorld.transform(end);

         addLine(start, end, lineWidth, color);
      }

      if (close)
      {
         Point2DReadOnly start2d = points.get(points.size() - 1);
         Point2DReadOnly end2d = points.get(0);

         start.set(start2d.getX(), start2d.getY(), 0.0);
         end.set(end2d.getX(), end2d.getY(), 0.0);
         transformToWorld.transform(start);
         transformToWorld.transform(end);

         addLine(start, end, lineWidth, color);
      }
   }

   /**
    * Combines the given mesh with the mesh contained in this builder while specifying the color of the
    * given mesh.
    *
    * @param meshDataHolder the mesh to combine. Not modified.
    * @param color          color of the given mesh. Color accuracy depends on the color palette in
    *                       use.
    */
   public void addMesh(MeshDataHolder meshDataHolder, Color color)
   {
      meshBuilder.addMesh(setColor(meshDataHolder, color));
   }

   public void addMesh(MeshDataHolder meshDataHolder, Tuple3DReadOnly offset, AxisAngle orientation, Color color)
   {
      meshBuilder.addMesh(setColor(meshDataHolder, color), offset, orientation);
   }

   private MeshDataHolder setColor(MeshDataHolder input, Color color)
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
//      if (hueResolution != -1)
         x = (float) (color.getHue() / 360.0);
//      else if (saturationResolution != -1)
//         x = (float) color.getSaturation();
//      else
//         x = (float) color.getBrightness();

      float y = 0.5f;

      return new float[] {x, y};
   }

   public Mesh generateMesh()
   {
      return meshBuilder.generateMesh();
   }

   public Material generateMaterial(AssetManager assetManager)
   {
      Material material = new Material(assetManager, "Common/MatDefs/Light/Lighting.j3md");
      Texture texture = assetManager.loadTexture("palette.png");
      material.setTexture("DiffuseMap", texture);
      material.setColor("Diffuse", ColorRGBA.White);
      return material;
   }
}