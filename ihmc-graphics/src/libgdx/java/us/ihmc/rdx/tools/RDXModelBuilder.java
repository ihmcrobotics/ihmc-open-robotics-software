package us.ihmc.rdx.tools;

import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.model.Node;
import com.badlogic.gdx.graphics.g3d.model.NodePart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;

import java.util.List;
import java.util.function.Consumer;

public class RDXModelBuilder
{
   public static ModelInstance createCoordinateFrameInstance(double length)
   {
      return new ModelInstance(createCoordinateFrame(length));
   }

   public static ModelInstance createCoordinateFrameInstance(double length, Color color)
   {
      return new ModelInstance(createCoordinateFrame(length, color));
   }

   public static ModelInstance buildModelInstance(Consumer<RDXMultiColorMeshBuilder> buildModel, String nodeName)
   {
      return new ModelInstance(buildModel(buildModel, nodeName));
   }

   public static ModelInstance buildModelInstance(Consumer<RDXMultiColorMeshBuilder> buildModel)
   {
      return new ModelInstance(buildModel(buildModel));
   }

   public static Model buildModel(Consumer<RDXMultiColorMeshBuilder> buildModel)
   {
      return buildModel(buildModel, null);
   }

   public static Model buildModel(Consumer<RDXMultiColorMeshBuilder> buildModel, String nodeName)
   {
      RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();
      buildModel.accept(meshBuilder);

      return buildModelFromMesh(meshBuilder, nodeName);
   }

   public static Model buildModelFromMesh(RDXMultiColorMeshBuilder meshBuilder)
   {
      return buildModelFromMesh(meshBuilder, null);
   }

   public static Model buildModelFromMesh(RDXMultiColorMeshBuilder meshBuilder, String nodeName)
   {
      ModelBuilder modelBuilder = new ModelBuilder();
//      modelBuilder.node().id = nodeName;

      return buildModelFromMesh(modelBuilder, meshBuilder);
   }

   public static Model buildModelFromMesh(ModelBuilder modelBuilder, RDXMultiColorMeshBuilder meshBuilder)
   {
      modelBuilder.begin();
      Mesh mesh = meshBuilder.generateMesh();

      MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
      Material material = new Material();
      Texture paletteTexture = RDXMultiColorMeshBuilder.loadPaletteTexture();
      material.set(TextureAttribute.createDiffuse(paletteTexture));
      material.set(ColorAttribute.createDiffuse(Color.WHITE));
      modelBuilder.part(meshPart, material);

      return modelBuilder.end();
   }

   public static void rebuildMesh(Node node, Consumer<RDXMultiColorMeshBuilder> buildModel)
   {
      NodePart oldNodePart = node.parts.removeIndex(0);
      oldNodePart.meshPart.mesh.dispose();

      RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();
      buildModel.accept(meshBuilder);
      Mesh mesh = meshBuilder.generateMesh();

      MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
      Material material = new Material();
      Texture paletteTexture = RDXMultiColorMeshBuilder.loadPaletteTexture();
      material.set(TextureAttribute.createDiffuse(paletteTexture));
      material.set(ColorAttribute.createDiffuse(Color.WHITE));

      node.parts.add(new NodePart(meshPart, material));
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
         meshBuilder.addCone(coneHeight,
                             coneRadius,
                             new Point3D(0.0, length, 0.0),
                             new AxisAngle(1.0, 0.0, 0.0, -Math.PI / 2.0),
                             Color.GREEN);
         meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(), Color.BLUE);
         meshBuilder.addCone(coneHeight, coneRadius, new Point3D(0.0, 0.0, length), new AxisAngle(), Color.BLUE);
      }, "coordinateFrame");
   }

   public static Model createCoordinateFrame(double length, Color color)
   {
      return buildModel(meshBuilder ->
      {
         double radius = 0.02 * length;
         double coneHeight = 0.10 * length;
         double coneRadius = 0.05 * length;
         meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), color);
         meshBuilder.addCone(coneHeight, coneRadius, new Point3D(length, 0.0, 0.0), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), Color.RED);
         meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(1.0, 0.0, 0.0, -Math.PI / 2.0), color);
         meshBuilder.addCone(coneHeight,
                             coneRadius,
                             new Point3D(0.0, length, 0.0),
                             new AxisAngle(1.0, 0.0, 0.0, -Math.PI / 2.0),
                             Color.GREEN);
         meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(), color);
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

   public static ModelInstance createCylinder(float height, float radius, Point3D offset, Color color)
   {
      return buildModelInstance(meshBuilder -> meshBuilder.addCylinder(height, radius, offset, color), "cylinder");
   }

   public static ModelInstance createEllipsoid(float xRadius, float yRadius, float zRadius, Point3D offset, Color color)
   {
      return buildModelInstance(meshBuilder -> meshBuilder.addEllipsoid(xRadius, yRadius, zRadius, offset, color), "ellipsoid");
   }

   public static ModelInstance createPrism(float triangleWidth, float prismThickness, float triangleHeight, Point3D offset, Color color)
   {
      return buildModelInstance(meshBuilder -> meshBuilder.addIsoscelesTriangularPrism(triangleWidth, triangleHeight, prismThickness, offset, color), "prism");
   }

   public static ModelInstance createCone(float height, float radius, Point3D offset, Color color)
   {
      return buildModelInstance(meshBuilder -> meshBuilder.addCone(height, radius, offset, color), "cone");
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
         meshBuilder.addCone(coneHeight,
                             coneRadius,
                             new Point3D(cylinderLength, 0.0, 0.0),
                             new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0),
                             color);
      }, "arrow");
   }

   public static ModelInstance createLine(Point3DReadOnly start, Point3DReadOnly end, double lineWidth, Color color)
   {
      return buildModelInstance(meshBuilder ->
      {
         meshBuilder.addLine(start, end, lineWidth, color);
      }, "line");
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
         meshBuilder.addCone(coneHeight,
                             coneRadius,
                             new Point3D(cylinderLength, 0.0, 0.0),
                             new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0),
                             color);
         meshBuilder.addSphere((float) radius, color);
      }, "arrow");
   }

   public static ModelInstance createStairs(double width, double stepHeight, double stepWidth, int numberOfSteps, Color color)
   {
      return buildModelInstance(meshBuilder ->
      {
         for(int i = 1; i<numberOfSteps + 1; i++)
         {
            meshBuilder.addBox(stepWidth, width, stepHeight * (float) i, new Point3D(stepWidth * (float) i, 0.0f, (stepHeight * (float) i) / 2.0f), color);
         }

         for(int i = 1; i<numberOfSteps + 1; i++)
         {
            meshBuilder.addCylinder(1.0f, 0.02f, new Point3D(stepWidth * (float) i, -width * 0.45f, (stepHeight * (float) i)), Color.BROWN);
            meshBuilder.addCylinder(1.0f, 0.02f, new Point3D(stepWidth * (float) i, width * 0.45f, (stepHeight * (float) i)), Color.BROWN);
         }

         meshBuilder.addCylinder(EuclidCoreTools.norm(numberOfSteps * stepWidth, numberOfSteps * stepHeight), 0.03f, new Point3D(0.0f, -width * 0.45f, 1.0f), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 4.0), Color.DARK_GRAY);
         meshBuilder.addCylinder(EuclidCoreTools.norm(numberOfSteps * stepWidth, numberOfSteps * stepHeight), 0.03f, new Point3D(0.0f, width * 0.45f, 1.0f), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 4.0), Color.DARK_GRAY);

      }, "stairs");
   }

   public static ModelInstance createMultiLine(RigidBodyTransformReadOnly transformToWorld, Point2DReadOnly[] points, double lineWidth, Color color, boolean close)
   {
      return buildModelInstance(meshBuilder -> meshBuilder.addMultiLine(transformToWorld, points, lineWidth, color, close));
   }

   public static ModelInstance createPolygon(RigidBodyTransformReadOnly transformToWorld, ConvexPolygon2DReadOnly polygon, Color color)
   {
      return buildModelInstance(meshBuilder -> meshBuilder.addPolygon(transformToWorld, polygon, color));
   }

   public static ModelInstance createLinedPolygon(List<? extends Point3DReadOnly> points,
                                                  double lineWidth,
                                                  Color color,
                                                  boolean close)
   {
      return buildModelInstance(meshBuilder ->
      {
         meshBuilder.addMultiLine(points, lineWidth, color, close);
         meshBuilder.addPolygon(points, color);
      });
   }

   public static ModelInstance createLinedPolygon(RigidBodyTransformReadOnly transformToWorld,
                                                  List<? extends Point2DReadOnly> points,
                                                  double lineWidth,
                                                  Color color,
                                                  boolean close)
   {
      return buildModelInstance(meshBuilder ->
      {
         meshBuilder.addMultiLine(transformToWorld, points, lineWidth, color, close);
         meshBuilder.addPolygon(transformToWorld, points, color);
      });
   }
}
