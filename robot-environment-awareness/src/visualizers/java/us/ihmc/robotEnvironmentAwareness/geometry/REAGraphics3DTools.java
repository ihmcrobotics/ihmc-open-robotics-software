package us.ihmc.robotEnvironmentAwareness.geometry;

import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Random;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Supplier;

import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;

public class REAGraphics3DTools
{
   public static Supplier<Color> rainbowColorSupplier(Random random)
   {
      return () -> Color.hsb(360.0 * random.nextDouble(), 0.8 * random.nextDouble(), 0.9);
   }

   public static void translateNode(Node nodeToTranslate, Tuple3DReadOnly translation)
   {
      nodeToTranslate.setTranslateX(nodeToTranslate.getTranslateX() + translation.getX());
      nodeToTranslate.setTranslateY(nodeToTranslate.getTranslateY() + translation.getY());
      nodeToTranslate.setTranslateZ(nodeToTranslate.getTranslateZ() + translation.getZ());
   }

   public static void transformNode(Node nodeToTransform, RigidBodyTransform transform)
   {
      nodeToTransform.getTransforms().add(JavaFXTools.createRigidBodyTransformToAffine(transform));
   }

   public static MeshView pointcloud(List<? extends Tuple3DReadOnly> pointcloud, Color color, double size)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();
      pointcloud.forEach(point -> meshBuilder.addTetrahedron(size, point));
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static MeshView multiLine(RigidBodyTransform transformToWorld, ConcaveHullCollection concaveHullCollection, Color color, double lineWidth)
   {
      return multiLine(transformToWorld, concaveHullCollection, (a, b) -> color, lineWidth);
   }

   public static MeshView multiLine(RigidBodyTransform transformToWorld, ConcaveHullCollection concaveHullCollection, BiFunction<Double, Double, Color> color,
                                    double lineWidth)
   {
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(1024));

      int hullIndex = -1;
      int numberOfConcaveHulls = concaveHullCollection.getNumberOfConcaveHulls();

      for (ConcaveHull concaveHull : concaveHullCollection)
      {
         hullIndex++;

         if (concaveHull.getNumberOfVertices() <= 1)
            continue;

         List<Point3D> vertices = concaveHull.toVerticesInWorld(transformToWorld);
         int numberOfVertices = vertices.size();
         Point3D previousVertex = vertices.get(numberOfVertices - 1);

         for (int vertexIndex = 0; vertexIndex < numberOfVertices; vertexIndex++)
         {
            Point3D vertex = vertices.get(vertexIndex);
            meshBuilder.addLine(previousVertex,
                                vertex,
                                lineWidth,
                                color.apply(hullIndex / (numberOfConcaveHulls - 1.0), vertexIndex / (numberOfVertices - 1.0)));
            previousVertex = vertex;
         }
      }

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      return meshView;
   }

   public static MeshView multiLine(List<? extends LineSegment3DReadOnly> segments, Color color, double lineWidth)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();
      segments.forEach(segment -> meshBuilder.addLine(segment.getFirstEndpoint(), segment.getSecondEndpoint(), lineWidth));
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static MeshView triangles(Collection<Triangle3D> triangles, Supplier<Color> colorSupplier)
   {
      return triangles(triangles, triangle -> colorSupplier.get());
   }

   public static MeshView triangles(Collection<Triangle3D> triangles, Function<Triangle3D, Color> colorFunction)
   {
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(512));
      for (Triangle3D triangle : triangles)
         meshBuilder.addPolygon(Arrays.asList(triangle.getA(), triangle.getB(), triangle.getC()), colorFunction.apply(triangle));
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      return meshView;
   }
}
