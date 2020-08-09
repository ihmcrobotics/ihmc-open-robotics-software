package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.FootstepForUI;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXVisualizers.PrivateAnimationTimer;
import us.ihmc.javafx.graphics.LabelGraphic;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class FootstepPlanWithTextGraphic extends Group
{
   private final MeshView meshView = new MeshView();
   private final Map<Pair<String, Integer>, LabelGraphic> labelRecycler = new HashMap<>();
   private final ArrayList<LabelGraphic> existingLabels = new ArrayList<>();
   private final ArrayList<LabelGraphic> labelsToAdd = new ArrayList<>();
   private final PrivateAnimationTimer animationTimer = new PrivateAnimationTimer(this::handle);
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
   private SideDependentList<ConvexPolygon2D> defaultContactPoints = new SideDependentList<>();
   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);
   private Mesh mesh;
   private Material material;

   public FootstepPlanWithTextGraphic(DRCRobotModel robotModel)
   {
      for(RobotSide robotSide : RobotSide.values)
      {
         ConvexPolygon2D defaultFoothold = new ConvexPolygon2D();
         robotModel.getContactPointParameters().getControllerFootGroundContactPoints().get(robotSide).forEach(point2D -> defaultFoothold.addVertex(point2D));
         defaultFoothold.update();
         defaultContactPoints.put(robotSide, defaultFoothold);
      }

      getChildren().addAll(meshView);

      animationTimer.start();
   }

   /**
    * To process in parallel.
    */
   public void generateMeshesAsynchronously(ArrayList<FootstepForUI> plan)
   {
      executorService.submit(() -> {
         generateMeshes(plan);
      });
   }

   public void generateMeshes(ArrayList<FootstepForUI> message)
   {
      meshBuilder.clear();

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      ConvexPolygon2D foothold = new ConvexPolygon2D();

      ArrayList<LabelGraphic> tempLabelsToAdd = new ArrayList<>();
      for (int i = 0; i < message.size(); i++)
      {
         FootstepForUI footstepForUI = message.get(i);
         Color regionColor = footstepForUI.getSide() == RobotSide.LEFT ? Color.RED : Color.GREEN;

         footstepForUI.getSolePoseInWorld().get(transformToWorld);
         transformToWorld.appendTranslation(0.0, 0.0, 0.01);

//         if (footstep.hasFoothold()) // TODO: Add foothold support
//            footstep.getFoothold(foothold);
//         else
            foothold.set(defaultContactPoints.get(footstepForUI.getSide()));

         Point2D[] vertices = new Point2D[foothold.getNumberOfVertices()];
         for (int j = 0; j < vertices.length; j++)
         {
            vertices[j] = new Point2D(foothold.getVertex(j));
         }

         meshBuilder.addMultiLine(transformToWorld, vertices, 0.01, regionColor, true);
         meshBuilder.addPolygon(transformToWorld, foothold, regionColor);

         LabelGraphic labelGraphic = labelRecycler.computeIfAbsent(Pair.of(footstepForUI.getDescription(), i), key -> new LabelGraphic(key.getLeft()));
         labelGraphic.getPose().set(footstepForUI.getSolePoseInWorld());
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
