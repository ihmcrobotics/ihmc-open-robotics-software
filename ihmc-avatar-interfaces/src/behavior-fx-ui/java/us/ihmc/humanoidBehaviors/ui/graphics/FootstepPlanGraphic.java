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
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ExecutorService;

public class FootstepPlanGraphic extends Group
{
   private final MeshView meshView = new MeshView();
   private final Map<Pair<String, Integer>, LabelGraphic> labelRecycler = new HashMap<>();
   private final ArrayList<LabelGraphic> existingLabels = new ArrayList<>();
   private final ArrayList<LabelGraphic> labelsToAdd = new ArrayList<>();
   private final PrivateAnimationTimer animationTimer = new PrivateAnimationTimer(this::handle);
   private final ExecutorService executorService = ThreadTools.newSingleDaemonThreadExecutor(getClass().getSimpleName());
   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);
   private Mesh mesh;
   private Material material;
   private SideDependentList<Color> footstepColors = new SideDependentList<>();
   private SideDependentList<ConvexPolygon2D> defaultContactPoints = new SideDependentList<>();

   public FootstepPlanGraphic(SegmentDependentList<RobotSide, ArrayList<Point2D>> controllerFootGroundContactPoints)
   {
      this();

      for (RobotSide robotSide : RobotSide.values)
      {
         ConvexPolygon2D defaultFoothold = new ConvexPolygon2D();
         controllerFootGroundContactPoints.get(robotSide).forEach(defaultFoothold::addVertex);
         defaultFoothold.update();
         defaultContactPoints.put(robotSide, defaultFoothold);
      }
   }

   public FootstepPlanGraphic()
   {
      footstepColors.set(RobotSide.LEFT, Color.RED);
      footstepColors.set(RobotSide.RIGHT, Color.GREEN);

      getChildren().addAll(meshView);

      animationTimer.start();
   }

   public void setTransparency(double opacity)
   {
      footstepColors.set(RobotSide.LEFT, new Color(Color.RED.getRed(), Color.RED.getGreen(), Color.RED.getBlue(), opacity));
      footstepColors.set(RobotSide.RIGHT, new Color(Color.GREEN.getRed(), Color.GREEN.getGreen(), Color.GREEN.getBlue(), opacity));
   }

   public void setColor(RobotSide side, Color color)
   {
      double priorOpacity = footstepColors.get(side).getOpacity();
      footstepColors.set(side, new Color(color.getRed(), color.getGreen(), color.getBlue(), priorOpacity));
   }

   /**
    * To process in parallel.
    */
   public void generateMeshesAsynchronously(ArrayList<MinimalFootstep> footsteps)
   {
      executorService.submit(() -> {
         generateMeshes(footsteps);
      });
   }

   public void generateMeshes(ArrayList<MinimalFootstep> footsteps)
   {
      meshBuilder.clear();

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      ConvexPolygon2D foothold = new ConvexPolygon2D();

      ArrayList<LabelGraphic> tempLabelsToAdd = new ArrayList<>();
      for (int i = 0; i < footsteps.size(); i++)
      {
         MinimalFootstep minimalFootstep = footsteps.get(i);
         Color regionColor = footstepColors.get(minimalFootstep.getSide());

         minimalFootstep.getSolePoseInWorld().get(transformToWorld);
         transformToWorld.appendTranslation(0.0, 0.0, 0.01);

         if (minimalFootstep.getFoothold() != null && !minimalFootstep.getFoothold().isEmpty())
            foothold.set(minimalFootstep.getFoothold());
         else if (defaultContactPoints.containsKey(minimalFootstep.getSide()))
            foothold.set(defaultContactPoints.get(minimalFootstep.getSide()));
         else
         {
            LogTools.error("Must specify default or per footstep foothold");
            throw new RuntimeException("Must specify default or per footstep foothold");
         }

         Point2D[] vertices = new Point2D[foothold.getNumberOfVertices()];
         for (int j = 0; j < vertices.length; j++)
         {
            vertices[j] = new Point2D(foothold.getVertex(j));
         }

         meshBuilder.addMultiLine(transformToWorld, vertices, 0.01, regionColor, true);
         meshBuilder.addPolygon(transformToWorld, foothold, regionColor);

         if (minimalFootstep.getDescription() != null && !minimalFootstep.getDescription().trim().isEmpty())
         {
            LabelGraphic labelGraphic = labelRecycler.computeIfAbsent(Pair.of(minimalFootstep.getDescription(), i), key -> new LabelGraphic(key.getLeft()));
            labelGraphic.getPose().set(minimalFootstep.getSolePoseInWorld());
            labelGraphic.update();
            tempLabelsToAdd.add(labelGraphic);
         }
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

   public void destroy()
   {
      executorService.shutdownNow();
   }
}
