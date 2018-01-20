package us.ihmc.footstepPlanning.ui;

import javafx.animation.AnimationTimer;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.jMonkeyEngineToolkit.tralala.Pair;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette2D;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.concurrent.atomic.AtomicReference;

public class FootstepPathMeshViewer extends AnimationTimer
{
   private static final boolean VERBOSE = false;
   private static final ConvexPolygon2D defaultFootPolygon = PlanningTestTools.createDefaultFootPolygon();

   private final MeshView footstepPathMeshView = new MeshView();
   private final AtomicReference<Pair<Mesh, Material>> meshReference = new AtomicReference<>(null);
   private final JavaFXMultiColorMeshBuilder meshBuilder;

   public FootstepPathMeshViewer()
   {
      TextureColorPalette2D colorPalette = new TextureColorPalette2D();
      colorPalette.setHueBrightnessBased(0.9);
      meshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);
   }

   public void processFootstepPath(FootstepPlan plan)
   {
      meshBuilder.clear();

      FramePose3D footPose = new FramePose3D();
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      ConvexPolygon2D foothold = new ConvexPolygon2D();

      for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         SimpleFootstep footstep = plan.getFootstep(i);
         Color regionColor = footstep.getRobotSide().equals(RobotSide.RIGHT) ? Color.RED : Color.GREEN;
         regionColor = Color.hsb(regionColor.getHue(), 0.9, 1.0);

         footstep.getSoleFramePose(footPose);
         footPose.get(transformToWorld);
         transformToWorld.appendTranslation(0.0, 0.0, 0.01);

         if(footstep.hasFoothold())
            footstep.getFoothold(foothold);
         else
            foothold.setAndUpdate(defaultFootPolygon);

         Point2D[] vertices = new Point2D[foothold.getNumberOfVertices()];
         for (int j = 0; j < vertices.length; j++)
         {
            vertices[j] = new Point2D(foothold.getVertex(j));
         }

         meshBuilder.addMultiLine(transformToWorld, vertices, 0.01, regionColor, true);
         meshBuilder.addPolygon(transformToWorld, foothold, regionColor);
      }

      meshReference.set(new Pair<>(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
   }

   @Override
   public void handle(long now)
   {
      Pair<Mesh, Material> newMeshAndMaterial = meshReference.getAndSet(null);

      if (newMeshAndMaterial != null)
      {
         if (VERBOSE)
            PrintTools.info(this, "Rendering body path line.");
         footstepPathMeshView.setMesh(newMeshAndMaterial.getKey());
         footstepPathMeshView.setMaterial(newMeshAndMaterial.getValue());
      }
   }

   public Node getRoot()
   {
      return footstepPathMeshView;
   }
}
