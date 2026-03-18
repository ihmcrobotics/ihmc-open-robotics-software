package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.pathPlanning.rrt.RRTStarPathPlanner;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

public class RDXRRTStarPathPlannerDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("RRT* Path Planner Demo");
   private final RDXPose3DGizmo startGizmo = new RDXPose3DGizmo();
   private final RDXPose3DGizmo goalGizmo = new RDXPose3DGizmo();
   private final RRTStarPathPlanner planner = new RRTStarPathPlanner();
   private List<Point3D> plannedPath = new ArrayList<>();
   private ModelInstance pathModel;
   private double planDurationMs = 0.0;
   private final ImBoolean autoPlan = new ImBoolean(true);
   private final int[] maxIterations = new int[] {20};
   private boolean planRequested = true;
   private RDX3DSituatedText startText;
   private RDX3DSituatedText goalText;
   private final Point3D textPosition = new Point3D();

   public RDXRRTStarPathPlannerDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            startGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
            startGizmo.getTransformToParent().getTranslation().set(0.0, 0.0, 0.0);
            startGizmo.setResizeAutomatically(false);
            startGizmo.setCenterSphereToTorusRatio(0.8f);
            startText = new RDX3DSituatedText("Start", 0.08f);

            goalGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
            goalGizmo.getTransformToParent().getTranslation().set(1.0, 0.0, 0.0);
            goalGizmo.setResizeAutomatically(false);
            goalGizmo.setCenterSphereToTorusRatio(0.8f);
            goalText = new RDX3DSituatedText("Goal", 0.08f);

            baseUI.getPrimaryScene().addRenderableProvider(startText);
            baseUI.getPrimaryScene().addRenderableProvider(goalText);

            baseUI.getPrimaryScene().addRenderableProvider((renderables, pool) ->
            {
               if (pathModel != null)
                  pathModel.getRenderables(renderables, pool);
            });

            baseUI.getImGuiPanelManager().addPanel("RRT* Path Planner", this::renderImGuiWidgets);
         }

         private void renderImGuiWidgets()
         {
            if (ImGui.button("Plan"))
               planRequested = true;
            ImGui.sameLine();
            ImGui.checkbox("Plan each frame", autoPlan);
            if (ImGui.sliderInt("Max iterations", maxIterations, 0, 100))
               planner.setMaxIterations(maxIterations[0]);

            ImGui.text("Start: " + startGizmo.getTransformToParent().getTranslation());
            ImGui.text("Goal: " + goalGizmo.getTransformToParent().getTranslation());
            ImGui.text("Path points: " + plannedPath.size());
            ImGui.text("Plan duration: %.3f ms".formatted(planDurationMs));
         }

         @Override
         public void render()
         {
            textPosition.set(startGizmo.getTransformToParent().getTranslation());
            textPosition.addZ(0.08);
            startText.setPositionFacingCamera(baseUI.getPrimary3DPanel().getCamera3D(), textPosition);
            textPosition.set(goalGizmo.getTransformToParent().getTranslation());
            textPosition.addZ(0.08);
            goalText.setPositionFacingCamera(baseUI.getPrimary3DPanel().getCamera3D(), textPosition);

            if (autoPlan.get() || planRequested)
            {
               long planStartNs = System.nanoTime();
               List<Point3D> result = planner.plan(startGizmo.getTransformToParent().getTranslation(),
                                                   goalGizmo.getTransformToParent().getTranslation(),
                                                   point -> false);
               planDurationMs = (System.nanoTime() - planStartNs) / 1_000_000.0;

               plannedPath = result != null ? result : new ArrayList<>();
               updatePathModel();
               planRequested = false;
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void updatePathModel()
         {
            if (pathModel != null)
               pathModel.model.dispose();

           pathModel = RDXModelBuilder.buildModelInstance(builder ->
           {
               RRTStarPathPlanner.Node rootNode = planner.getRootNode();
               if (rootNode != null)
               {
                  ArrayDeque<RRTStarPathPlanner.Node> stack = new ArrayDeque<>();
                  stack.push(rootNode);
                  while (!stack.isEmpty())
                  {
                     RRTStarPathPlanner.Node node = stack.pop();
                     for (RRTStarPathPlanner.Node child : node.children())
                     {
                        builder.addLine(node.position(), child.position(), 0.005, Color.ORANGE);
                        builder.addMesh(MeshDataGenerator.Sphere(0.02, 5, 5), child.position(), Color.WHITE);
                        stack.push(child);
                     }
                  }
               }

               Point3D previous = null;
               for (Point3D point : plannedPath)
               {
                  builder.addSphere(0.02, point, Color.WHITE);
                  if (previous != null)
                     builder.addLine(previous, point, 0.01, Color.CYAN);
                  previous = point;
               }
            });
         }

         @Override
         public void dispose()
         {
            if (pathModel != null)
               pathModel.model.dispose();
            startText.dispose();
            goalText.dispose();
            startGizmo.destroyDefault(baseUI.getPrimary3DPanel());
            goalGizmo.destroyDefault(baseUI.getPrimary3DPanel());
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXRRTStarPathPlannerDemo();
   }
}
