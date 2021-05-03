package us.ihmc.gdx.tools;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import org.lwjgl.opengl.GL32;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.mesh.GDXMultiColorMeshBuilder;

import java.util.function.Consumer;

// TODO: Rename to GDXModelBuilder?
public class GDXModelPrimitives
{
   public static ModelInstance createCoordinateFrameInstance(double length)
   {
      return new ModelInstance(createCoordinateFrame(length));
   }

   public static ModelInstance buildModelInstance(Consumer<GDXMultiColorMeshBuilder> buildModel, String nodeName)
   {
      return new ModelInstance(buildModel(buildModel, nodeName));
   }

   public static Model buildModel(Consumer<GDXMultiColorMeshBuilder> buildModel, String nodeName)
   {
      ModelBuilder modelBuilder = new ModelBuilder();
      modelBuilder.begin();
      modelBuilder.node().id = nodeName; // optional

      GDXMultiColorMeshBuilder meshBuilder = new GDXMultiColorMeshBuilder();
      buildModel.accept(meshBuilder);
      Mesh mesh = meshBuilder.generateMesh();

      MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL32.GL_TRIANGLES);
      Material material = new Material();
      Texture paletteTexture = new Texture(Gdx.files.classpath("palette.png"));
      material.set(TextureAttribute.createDiffuse(paletteTexture));
      material.set(ColorAttribute.createDiffuse(Color.WHITE));
      modelBuilder.part(meshPart, material);

      return modelBuilder.end();
   }

   public static Model createCoordinateFrame(double length)
   {
      return buildModel(meshBuilder ->
      {
         double radius = 0.02 * length;
         double coneHeight = 0.10 * length;
         double coneRadius = 0.05 * length;
         meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), Color.RED);
         meshBuilder.addCone(coneHeight, coneRadius, new Point3D(length, 0.0, 0.0), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), Color.RED);
         meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(1.0, 0.0, 0.0, -Math.PI / 2.0), Color.GREEN);
         meshBuilder.addCone(coneHeight, coneRadius, new Point3D(0.0, length, 0.0), new AxisAngle(1.0, 0.0, 0.0, -Math.PI / 2.0), Color.GREEN);
         meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(), Color.BLUE);
         meshBuilder.addCone(coneHeight, coneRadius, new Point3D(0.0, 0.0, length), new AxisAngle(), Color.BLUE);
      }, "coordinateFrame");
   }

   public static ModelInstance createSphere(float radius, Color color)
   {
      return createSphere(radius, color, "sphere");
   }

   public static ModelInstance createSphere(float radius, Color color, String nodeName)
   {
      return buildModelInstance(meshBuilder -> meshBuilder.addSphere(radius, color), nodeName);
   }

   public static ModelInstance createBox(float lx, float ly, float lz, Color color)
   {
      return buildModelInstance(meshBuilder -> meshBuilder.addBox(lx, ly, lz, color), "box");
   }

   public static ModelInstance createArrow(double length, Color color)
   {
      return buildModelInstance(meshBuilder ->
      {
         double coneHeight = 0.10 * length;
         double cylinderLength = length - coneHeight;
         double cylinderRadius = cylinderLength / 20.0;
         double coneRadius = 1.5 * cylinderRadius;
         meshBuilder.addCylinder(cylinderLength, cylinderRadius, new Point3D(), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), color);
         meshBuilder.addCone(coneHeight, coneRadius, new Point3D(cylinderLength, 0.0, 0.0), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), color);
      }, "arrow");
   }

   public static ModelInstance createPose(double radius, Color color)
   {
      return buildModelInstance(meshBuilder ->
      {
         double cylinderLength = radius * 6.0;
         double cylinderRadius = cylinderLength / 20.0;
         double coneHeight = 0.10 * cylinderLength;
         double coneRadius = 1.5 * cylinderRadius;
         meshBuilder.addCylinder(cylinderLength, cylinderRadius, new Point3D(), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), color);
         meshBuilder.addCone(coneHeight, coneRadius, new Point3D(cylinderLength, 0.0, 0.0), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), color);
         meshBuilder.addSphere((float) radius, color);
      }, "arrow");
   }
}
