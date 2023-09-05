package us.ihmc.footstepPlanning.ui.viewers;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javafx.FootstepMeshManager;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;

/**
 * This class uses the shorthand: compute a mesh = using a FootstepDataMessage generate a Mesh and
 * Material pair update a mesh = from a Mesh and Material pair update a MeshView
 */
public class FootstepPathMeshViewer extends AnimationTimer
{
   private final Group root = new Group();
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));
   private final SideDependentList<ConvexPolygon2D> defaultContactPoints = new SideDependentList<>();

   private final AtomicBoolean reset = new AtomicBoolean(false);
   private final AtomicBoolean ignorePartialFootholds = new AtomicBoolean();
   private final AtomicReference<FootstepDataListMessage> footstepPlanResponse = new AtomicReference<>();
   private final AtomicBoolean updateAllMeshes = new AtomicBoolean();

   private final AtomicReference<Integer> previousSelectedStep = new AtomicReference<>(-1);
   private final AtomicReference<Pair<Integer, FootstepDataMessage>> selectedStep;

   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);

   private final List<FootstepMeshManager> footstepMeshes = new ArrayList<>();
   private final AtomicReference<Pair<Integer, FootstepDataMessage>> updateStep = new AtomicReference<>();

   public FootstepPathMeshViewer(Messager messager)
   {
      // call these on separate thread since they update all meshes
      messager.addTopicListener(FootstepPlanResponse, footstepPlan ->
      {
         footstepPlanResponse.set(footstepPlan);
         updateAllMeshes.set(true);
      });

      messager.addTopicListener(IgnorePartialFootholds, ignore ->
      {
         ignorePartialFootholds.set(ignore);
         updateAllMeshes.set(true);
      });

      // call this on animation timer update thread since it updates a single step and might come in at higher frequency
      messager.addTopicListener(FootstepToUpdateViz, updateStep::set);
      selectedStep = messager.createInput(SelectedFootstep, Pair.of(-1, null));

      // handle reset on animation timer thread
      messager.addTopicListener(ComputePath, data -> reset.set(true));
      messager.addTopicListener(GlobalReset, data -> reset.set(true));

      for (int i = 0; i < 200; i++)
      {
         int index = i;
         FootstepMeshManager meshManager = new FootstepMeshManager(root,
                                                                   meshBuilder,
                                                                   defaultContactPoints::get,
                                                                   () -> selectedStep.get().getKey() == index,
                                                                   ignorePartialFootholds::get);
         footstepMeshes.add(meshManager);
      }

      messager.addTopicListener(ShowFootstepPlan, show -> footstepMeshes.forEach(mesh -> mesh.getMeshHolder().getMeshView().setVisible(show)));
   }

   private void computeAllMeshes(FootstepDataListMessage footstepPlanResponse)
   {
      if (footstepPlanResponse != null)
      {
         clearAll();

         while (footstepPlanResponse.getFootstepDataList().size() > footstepMeshes.size())
         {
            int index = footstepMeshes.size();
            FootstepMeshManager meshManager = new FootstepMeshManager(root,
                                                                      meshBuilder,
                                                                      defaultContactPoints::get,
                                                                      () -> selectedStep.get().getKey() == index,
                                                                      ignorePartialFootholds::get);
            footstepMeshes.add(meshManager);
         }

         for (int i = 0; i < footstepPlanResponse.getFootstepDataList().size(); i++)
         {
            FootstepDataMessage footstepDataMessage = footstepPlanResponse.getFootstepDataList().get(i);
            footstepMeshes.get(i).setFootstepDataMessage(footstepDataMessage);
         }
      }

      for (int i = 0; i < footstepMeshes.size(); i++)
      {
         footstepMeshes.get(i).computeMesh();
      }

      updateAllMeshes.set(true);
   }

   @Override
   public void handle(long now)
   {
      if (updateAllMeshes.getAndSet(false))
      {
         computeAllMeshes(footstepPlanResponse.getAndSet(null));
      }

      Pair<Integer, FootstepDataMessage> updateStep = this.updateStep.getAndSet(null);
      if (updateStep != null)
      {
         footstepMeshes.get(updateStep.getKey()).setFootstepDataMessage(updateStep.getValue());
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
      for (RobotSide robotSide : RobotSide.values)
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