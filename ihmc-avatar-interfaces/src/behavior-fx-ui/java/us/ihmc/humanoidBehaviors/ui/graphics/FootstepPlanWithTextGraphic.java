package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXVisualizers.PrivateAnimationTimer;
import us.ihmc.javafx.graphics.LabelGraphic;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ExecutorService;

public class FootstepPlanWithTextGraphic extends Group
{
   private final MeshView meshView = new MeshView();
   private final Map<Pair<String, Integer>, LabelGraphic> labelRecycler = new HashMap<>();
   private final ArrayList<LabelGraphic> existingLabels = new ArrayList<>();
   private final ArrayList<LabelGraphic> labelsToAdd = new ArrayList<>();
   private final PrivateAnimationTimer animationTimer = new PrivateAnimationTimer(this::handle);
   private final ExecutorService executorService = ThreadTools.newSingleThreadExecutor(getClass().getSimpleName());
   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);
   private Mesh mesh;
   private Material material;

   public FootstepPlanWithTextGraphic()
   {
      getChildren().addAll(meshView);

      animationTimer.start();
   }

   /**
    * To process in parallel.
    */
   public void generateMeshesAsynchronously(ArrayList<MinimalFootstep> plan)
   {
      executorService.submit(() -> {
         generateMeshes(plan);
      });
   }

   public void generateMeshes(ArrayList<MinimalFootstep> message)
   {
      meshBuilder.clear();

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      ConvexPolygon2D foothold = new ConvexPolygon2D();

      ArrayList<LabelGraphic> tempLabelsToAdd = new ArrayList<>();
      for (int i = 0; i < message.size(); i++)
      {
         MinimalFootstep minimalFootstep = message.get(i);
         Color regionColor = minimalFootstep.getSide() == RobotSide.LEFT ? Color.RED : Color.GREEN;

         minimalFootstep.getSolePoseInWorld().get(transformToWorld);
         transformToWorld.appendTranslation(0.0, 0.0, 0.01);

         foothold.set(minimalFootstep.getFoothold());

         Point2D[] vertices = new Point2D[foothold.getNumberOfVertices()];
         for (int j = 0; j < vertices.length; j++)
         {
            vertices[j] = new Point2D(foothold.getVertex(j));
         }

         meshBuilder.addMultiLine(transformToWorld, vertices, 0.01, regionColor, true);
         meshBuilder.addPolygon(transformToWorld, foothold, regionColor);

         LabelGraphic labelGraphic = labelRecycler.computeIfAbsent(Pair.of(minimalFootstep.getDescription(), i), key -> new LabelGraphic(key.getLeft()));
         labelGraphic.getPose().set(minimalFootstep.getSolePoseInWorld());
         labelGraphic.update();
         tempLabelsToAdd.add(labelGraphic);
      }

      Mesh mesh = meshBuilder.generateMesh();
      Material material = meshBuilder.generateMaterial();

      synchronized (this)
      {
         this.mesh = mesh;
         this.material = material;

         labelsToAdd.clear();
         labelsToAdd.addAll(tempLabelsToAdd);
      }
   }

   private void handle(long now)
   {
      synchronized (this)
      {
         meshView.setMesh(mesh);
         meshView.setMaterial(material);

         for (LabelGraphic label : existingLabels)
         {
            getChildren().remove(label.getNode());
         }
         existingLabels.clear();
         for (LabelGraphic label : labelsToAdd)
         {
            existingLabels.add(label);
            getChildren().add(label.getNode());
         }
      }
   }

   public void clear()
   {
      generateMeshes(new ArrayList<>());
   }
}
