package us.ihmc.footstepPlanning.ui.viewers;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;

/**
 * This class uses the shorthand:
 * compute a mesh = using a FootstepDataMessage generate a Mesh and Material pair
 * update a mesh = from a Mesh and Material pair update a MeshView
 */
public class FootstepPathMeshViewer extends AnimationTimer
{
   private final Group root = new Group();
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));
   private SideDependentList<ConvexPolygon2D> defaultContactPoints = new SideDependentList<>();

   private final AtomicBoolean reset = new AtomicBoolean(false);
   private final AtomicReference<Boolean> ignorePartialFootholds;

   private final AtomicReference<Integer> previousSelectedStep = new AtomicReference<>(-1);
   private final AtomicReference<Pair<Integer, FootstepDataMessage>> selectedStep;

   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);

   private final List<FootstepMeshManager> footstepMeshes = new ArrayList<>();
   private final AtomicReference<Pair<Integer, FootstepDataMessage>> updateStep = new AtomicReference<>();
   private final AtomicBoolean updateAllMeshes = new AtomicBoolean();

   public FootstepPathMeshViewer(Messager messager)
   {
      // call these on separate thread since they update all meshes
      messager.registerTopicListener(FootstepPlanResponse, footstepPlan -> executorService.submit(() -> computeAllMeshes(footstepPlan)));
      messager.registerTopicListener(IgnorePartialFootholds, b -> executorService.submit(() -> computeAllMeshes(null)));

      // call this on animation timer update thread since it updates a single step and might come in at higher frequency
      messager.registerTopicListener(FootstepToUpdateViz, updateStep::set);
      selectedStep = messager.createInput(SelectedFootstep, Pair.of(-1, null));

      // handle reset on animation timer thread
      messager.registerTopicListener(ComputePath, data -> reset.set(true));
      messager.registerTopicListener(GlobalReset, data -> reset.set(true));

      ignorePartialFootholds = messager.createInput(IgnorePartialFootholds, false);

      for (int i = 0; i < 200; i++)
      {
         footstepMeshes.add(new FootstepMeshManager(i));
      }
   }

   private synchronized void computeAllMeshes(FootstepDataListMessage footstepPlanResponse)
   {
      if (footstepPlanResponse != null)
      {
         clearAll();

         while (footstepPlanResponse.getFootstepDataList().size() > footstepMeshes.size())
         {
            footstepMeshes.add(new FootstepMeshManager(footstepMeshes.size()));
         }

         for (int i = 0; i < footstepPlanResponse.getFootstepDataList().size(); i++)
         {
            FootstepDataMessage footstepDataMessage = footstepPlanResponse.getFootstepDataList().get(i);
            footstepMeshes.get(i).footstepDataMessage.set(footstepDataMessage);
         }
      }

      for (int i = 0; i < footstepMeshes.size(); i++)
      {
         footstepMeshes.get(i).computeMesh();
      }

      updateAllMeshes.set(true);
   }

   private Color getFootstepColor(FootstepDataMessage footstepDataMessage, int index)
   {
      if (index == selectedStep.get().getKey())
      {
         return Color.YELLOW.darker();
      }
      else
      {
         return footstepDataMessage.getRobotSide() == 0 ? Color.RED : Color.GREEN;
      }
   }

   private class FootstepMeshManager
   {
      private static final double ADDITIONAL_HEIGHT = 0.01;

      private final int index;
      private final MeshHolder meshHolder = new MeshHolder(root);
      private final AtomicReference<FootstepDataMessage> footstepDataMessage = new AtomicReference<>();

      private final RigidBodyTransform transformToWorld = new RigidBodyTransform();
      private final ConvexPolygon2D foothold = new ConvexPolygon2D();

      public FootstepMeshManager(int index)
      {
         this.index = index;
      }

      /**
       * Call from any thread
       */
      void computeMesh()
      {
         if (footstepDataMessage.get() == null)
         {
            return;
         }

         meshBuilder.clear();
         foothold.clear();

         FootstepDataMessage footstepDataMessage = this.footstepDataMessage.get();

         Color footColor = getFootstepColor(footstepDataMessage, index);
         RobotSide robotSide = RobotSide.fromByte(footstepDataMessage.getRobotSide());

         transformToWorld.set(footstepDataMessage.getOrientation(), footstepDataMessage.getLocation());
         transformToWorld.appendTranslation(0.0, 0.0, ADDITIONAL_HEIGHT);

         if (ignorePartialFootholds.get() || footstepDataMessage.getPredictedContactPoints2d().isEmpty())
         {
            foothold.set(defaultContactPoints.get(robotSide));
         }
         else
         {
            footstepDataMessage.getPredictedContactPoints2d().forEach(p -> foothold.addVertex(p.getX(), p.getY()));
            foothold.update();
         }

         Point2D[] vertices = new Point2D[foothold.getNumberOfVertices()];
         for (int j = 0; j < vertices.length; j++)
         {
            vertices[j] = new Point2D(foothold.getVertex(j));
         }

         meshBuilder.addMultiLine(transformToWorld, vertices, 0.01, footColor, true);
         meshBuilder.addPolygon(transformToWorld, foothold, footColor);

         Pair<Mesh, Material> meshMaterialPair = Pair.of(meshBuilder.generateMesh(), meshBuilder.generateMaterial());
         meshHolder.meshReference.set(meshMaterialPair);
      }

      /**
       * Call from AnimationTimer.handle
       */
      void updateMesh()
      {
         meshHolder.update();
      }

      /**
       * Call from AnimationTimer.handle
       */
      void clear()
      {
         footstepDataMessage.set(null);
         meshHolder.remove();
      }
   }

   @Override
   public void handle(long now)
   {
      Pair<Integer, FootstepDataMessage> updateStep = this.updateStep.getAndSet(null);
      if (updateStep != null)
      {
         footstepMeshes.get(updateStep.getKey()).footstepDataMessage.set(updateStep.getValue());
         footstepMeshes.get(updateStep.getKey()).computeMesh();
         footstepMeshes.get(updateStep.getKey()).updateMesh();
      }
      else if (updateAllMeshes.getAndSet(false))
      {
         for (int i = 0; i < footstepMeshes.size(); i++)
         {
            footstepMeshes.get(i).updateMesh();
         }
      }
      else
      {
         int selectedStepIndex = selectedStep.get().getKey();
         int previousSelectedStepIndex = previousSelectedStep.get();

         if (previousSelectedStepIndex != selectedStepIndex)
         {
            if (previousSelectedStepIndex >= 0)
            {
               footstepMeshes.get(previousSelectedStepIndex).computeMesh();
               footstepMeshes.get(previousSelectedStepIndex).updateMesh();
            }

            footstepMeshes.get(selectedStepIndex).computeMesh();
            footstepMeshes.get(selectedStepIndex).updateMesh();

            previousSelectedStep.set(selectedStepIndex);
         }
         else if (reset.getAndSet(false))
         {
            clearAll();
         }
      }
   }

   private void clearAll()
   {
      for (int i = 0; i < footstepMeshes.size(); i++)
      {
         footstepMeshes.get(i).clear();
      }

      selectedStep.set(Pair.of(-1, null));
      previousSelectedStep.set(-1);
   }

   public void setDefaultContactPoints(SideDependentList<List<Point2D>> defaultContactPoints)
   {
      for(RobotSide robotSide : RobotSide.values)
      {
         ConvexPolygon2D defaultFoothold = new ConvexPolygon2D();
         for (int i = 0; i < defaultContactPoints.get(robotSide).size(); i++)
         {
            defaultFoothold.addVertex(defaultContactPoints.get(robotSide).get(i));
         }

         defaultFoothold.update();
         this.defaultContactPoints.put(robotSide, defaultFoothold);
      }
   }

   @Override
   public void stop()
   {
      super.stop();
      executorService.shutdownNow();
   }

   public Node getRoot()
   {
      return root;
   }
}