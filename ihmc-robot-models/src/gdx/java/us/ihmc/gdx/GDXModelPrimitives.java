package us.ihmc.gdx;

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

public class GDXModelPrimitives
{
   public static ModelInstance createCoordinateFrameInstance(double length)
   {
      return new ModelInstance(createCoordinateFrame(length));
   }

   public static Model createCoordinateFrame(double length)
   {
      ModelBuilder modelBuilder = new ModelBuilder();
      modelBuilder.begin();
      modelBuilder.node().id = "coordinateFrame"; // optional

      double radius = 0.02 * length;
      double coneHeight = 0.10 * length;
      double coneRadius = 0.05 * length;
      GDXMultiColorMeshBuilder meshBuilder = new GDXMultiColorMeshBuilder();
      meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), Color.RED);
      meshBuilder.addCone(coneHeight, coneRadius, new Point3D(length, 0.0, 0.0), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), Color.RED);
      meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(1.0, 0.0, 0.0, -Math.PI / 2.0), Color.GREEN);
      meshBuilder.addCone(coneHeight, coneRadius, new Point3D(0.0, length, 0.0), new AxisAngle(1.0, 0.0, 0.0, -Math.PI / 2.0), Color.GREEN);
      meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(), Color.BLUE);
      meshBuilder.addCone(coneHeight, coneRadius, new Point3D(0.0, 0.0, length), new AxisAngle(), Color.BLUE);
      Mesh mesh = meshBuilder.generateMesh();

      MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL32.GL_TRIANGLES);
      Material material = new Material();
      Texture paletteTexture = new Texture(Gdx.files.classpath("palette.png"));
      material.set(TextureAttribute.createDiffuse(paletteTexture));
      material.set(ColorAttribute.createDiffuse(Color.WHITE));
      modelBuilder.part(meshPart, material);

      return modelBuilder.end();
   }
}
