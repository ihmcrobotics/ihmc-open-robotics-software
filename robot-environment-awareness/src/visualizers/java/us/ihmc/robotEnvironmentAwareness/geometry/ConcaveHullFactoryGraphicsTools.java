package us.ihmc.robotEnvironmentAwareness.geometry;

import java.util.Collections;
import java.util.List;
import java.util.Random;

import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory.ConcaveHullFactoryResult;
import us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory.ConcaveHullVariables;

public class ConcaveHullFactoryGraphicsTools
{
   public static MeshView delaunayTrianglesToRainbowTriangles(RigidBodyTransform transformToWorld, ConcaveHullFactoryResult concaveHullFactoryResult,
                                                              Random random)
   {
      List<Triangle3D> allTriangles = JTSTools.extractAllTrianglesInWorld(concaveHullFactoryResult, transformToWorld);
      return REAGraphics3DTools.triangles(allTriangles, REAGraphics3DTools.rainbowColorSupplier(random));
   }

   public static MeshView borderVerticesToMultiSpheres(RigidBodyTransform transformToWorld, ConcaveHullFactoryResult concaveHullFactoryResult, Color color,
                                                       double sphereRadius)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();
      List<Point3D> borderVertices = JTSTools.extractBorderVerticesInWorld(concaveHullFactoryResult, transformToWorld);
      borderVertices.forEach(vertex -> meshBuilder.addSphere(sphereRadius, vertex));
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static MeshView borderEdgesToMultiLine(RigidBodyTransform transformToWorld, ConcaveHullFactoryResult concaveHullFactoryResult, Color color,
                                                 double lineWidth)
   {
      List<LineSegment3D> borderEdges = JTSTools.extractBorderEdgesInWorld(concaveHullFactoryResult, transformToWorld);
      return REAGraphics3DTools.multiLine(borderEdges, color, lineWidth);
   }

   public static MeshView borderTrianglesToRainbowMultiTriangles(RigidBodyTransform transformToWorld, ConcaveHullFactoryResult concaveHullFactoryResult,
                                                                 Random random)
   {
      List<Triangle3D> borderTriangles = JTSTools.extractBorderTrianglesInWorld(concaveHullFactoryResult, transformToWorld);
      return REAGraphics3DTools.triangles(borderTriangles, REAGraphics3DTools.rainbowColorSupplier(random));
   }

   public static MeshView orderedBorderEdgesToRainbowMultiLine(RigidBodyTransform transformToWorld, ConcaveHullFactoryResult concaveHullFactoryResult, double lineThickness)
   {
      return orderedBorderEdgesToRainbowMultiLine(transformToWorld, concaveHullFactoryResult, lineThickness, 0.0, 240.0, 0.2, 1.0, 0.9, 0.2);
   }

   public static MeshView orderedBorderEdgesToRainbowMultiLine(RigidBodyTransform transformToWorld, ConcaveHullFactoryResult concaveHullFactoryResult, double lineThickness,
                                                               double hullStartHue, double hullEndHue, double edgeStartBrigthness, double edgeEndBrightness,
                                                               double firstHullSaturation, double lastHullSaturation)
   {
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(1024));

      double lineSat, lineHue;

      List<ConcaveHullVariables> intermediateVariablesList = concaveHullFactoryResult != null ? concaveHullFactoryResult.getIntermediateVariables()
            : Collections.emptyList();

      for (int variablesIndex = 0; variablesIndex < intermediateVariablesList.size(); variablesIndex++)
      {
         if (intermediateVariablesList.size() == 1)
         {
            lineSat = firstHullSaturation;
         }
         else
         {
            double alphaSat = variablesIndex / (double) (intermediateVariablesList.size() - 1.0);
            lineSat = (1.0 - alphaSat) * lastHullSaturation + alphaSat * firstHullSaturation;
         }

         List<LineSegment3D> orderedBorderEdges = JTSTools.extractOrderedBorderEdgesInWorld(intermediateVariablesList.get(variablesIndex), transformToWorld);

         for (int edgeIndex = 0; edgeIndex < orderedBorderEdges.size(); edgeIndex++)
         {
            LineSegment3D edge = orderedBorderEdges.get(edgeIndex);

            if (orderedBorderEdges.size() == 1)
            {
               lineHue = hullStartHue;
            }
            else
            {
               double alphaHue = edgeIndex / (double) (orderedBorderEdges.size() - 1.0);
               lineHue = (1.0 - alphaHue) * hullStartHue + alphaHue * hullEndHue;
            }

            Color startColor = Color.hsb(lineHue, lineSat, edgeStartBrigthness);
            Color endColor = Color.hsb(lineHue, lineSat, edgeEndBrightness);
            meshBuilder.addLine(edge.getFirstEndpoint(), edge.getSecondEndpoint(), lineThickness, startColor, endColor);
         }
      }

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      return meshView;
   }

   public static MeshView constraintEdgesToMultiLine(RigidBodyTransform transformToWorld, ConcaveHullFactoryResult concaveHullFactoryResult, Color color,
                                                     double lineWidth)
   {
      List<LineSegment3D> constraintEdges = JTSTools.extractConstraintEdges(concaveHullFactoryResult, transformToWorld);
      return REAGraphics3DTools.multiLine(constraintEdges, color, lineWidth);
   }
}
