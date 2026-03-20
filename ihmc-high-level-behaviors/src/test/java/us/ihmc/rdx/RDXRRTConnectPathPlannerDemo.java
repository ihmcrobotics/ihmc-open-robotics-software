package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.rrt.RRTConnectPathPlanner;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

public class RDXRRTConnectPathPlannerDemo
{
   private static final double OBSTACLE_RADIUS = 0.5;
   private final RDXBaseUI baseUI = new RDXBaseUI("RRT-Connect Path Planner Demo");
   private final RDXPose3DGizmo startGizmo = new RDXPose3DGizmo();
   private final RDXPose3DGizmo goalGizmo = new RDXPose3DGizmo();
   private final RRTConnectPathPlanner planner = new RRTConnectPathPlanner();
   private List<Point3D> plannedPath = new ArrayList<>();
   private ModelInstance pathModel;
   private double planDurationMs = 0.0;
   private final ImBoolean autoPlan = new ImBoolean(true);
   private final int[] maxIterations = new int[] {1000};
   private final int[] numberOfObstacles = new int[] {0};
   private final List<RDXPose3DGizmo> obstacleGizmos = new ArrayList<>();
   private boolean planRequested = true;
   private RDX3DSituatedText startText;
   private RDX3DSituatedText goalText;
   private final Point3D textPosition = new Point3D();
   private final Point3D collisionQuery = new Point3D();

   public RDXRRTConnectPathPlannerDemo()
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

            for (int i = 0; i < 5; i++)
            {
               RDXPose3DGizmo obstacleGizmo = new RDXPose3DGizmo();
               obstacleGizmo.create(baseUI.getPrimary3DPanel());
               baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(obstacleGizmo::calculate3DViewPick);
               baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(obstacleGizmo::process3DViewInput);
               obstacleGizmo.getTransformToParent().getTranslation().set(0.3 + i * 0.15, 0.2, 0.0);
               obstacleGizmo.setResizeAutomatically(false);
               obstacleGizmo.setCenterSphereToTorusRatio(0.8f);
               obstacleGizmos.add(obstacleGizmo);
            }

            baseUI.getPrimaryScene().addRenderableProvider(startText);
            baseUI.getPrimaryScene().addRenderableProvider(goalText);
            baseUI.getPrimaryScene().addRenderableProvider((renderables, pool) ->
            {
               for (int i = 0; i < numberOfObstacles[0]; i++)
                  obstacleGizmos.get(i).getRenderables(renderables, pool);
            });

            baseUI.getPrimaryScene().addRenderableProvider((renderables, pool) ->
            {
               if (pathModel != null)
                  pathModel.getRenderables(renderables, pool);
            });

            baseUI.getImGuiPanelManager().addPanel("RRT-Connect Path Planner", this::renderImGuiWidgets);
         }

         private void renderImGuiWidgets()
         {
            if (ImGui.button("Plan"))
               planRequested = true;
            ImGui.sameLine();
            ImGui.checkbox("Plan each frame", autoPlan);
            if (ImGui.sliderInt("Max iterations", maxIterations, 0, 10000))
               planner.setMaxIterations(maxIterations[0]);
            ImGui.sliderInt("Obstacles", numberOfObstacles, 0, obstacleGizmos.size());

            ImGui.text("Start: " + startGizmo.getTransformToParent().getTranslation());
            ImGui.text("Goal: " + goalGizmo.getTransformToParent().getTranslation());
            for (int i = 0; i < numberOfObstacles[0]; i++)
               ImGui.text("Obstacle " + i + ": " + obstacleGizmos.get(i).getTransformToParent().getTranslation());
            ImGui.text("Path points: " + plannedPath.size());
            ImGui.text("Plan duration: %.3f ms".formatted(planDurationMs));
            ImGui.text("Iterations: %d".formatted(planner.getIterationCount()));
            ImGui.text("Tree A size: %d".formatted(planner.getTreeASize()));
            ImGui.text("Tree B size: %d".formatted(planner.getTreeBSize()));
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
                                                   segment ->
                                                   {
                                                      for (int i = 0; i < numberOfObstacles[0]; i++)
                                                      {
                                                         collisionQuery.set(obstacleGizmos.get(i).getTransformToParent().getTranslation());
                                                         if (segment.distance(collisionQuery) <= OBSTACLE_RADIUS)
                                                            return true;
                                                      }
                                                      return false;
                                                   });
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
               double radius = planner.getSearchRadius();
               if (radius > 0.0)
               {
                  Point3D center = planner.getCenter();
                  int segments = 64;
                  double angleStep = 2.0 * Math.PI / segments;
                  double centerX = center.getX();
                  double centerY = center.getY();

                  Point3D previous = new Point3D(centerX + radius, centerY, 0.0);
                  for (int i = 1; i <= segments; i++)
                  {
                     double angle = i * angleStep;
                     Point3D current = new Point3D(centerX + radius * Math.cos(angle),
                                                   centerY + radius * Math.sin(angle),
                                                   0.0);
                     builder.addLine(previous, current, 0.004, Color.DARK_GRAY);
                     previous = current;
                  }
               }

               if (numberOfObstacles[0] > 0)
               {
                  int segments = 64;
                  double angleStep = 2.0 * Math.PI / segments;
                  for (int obstacleIndex = 0; obstacleIndex < numberOfObstacles[0]; obstacleIndex++)
                  {
                     Point3D obstacleCenter = new Point3D(obstacleGizmos.get(obstacleIndex).getTransformToParent().getTranslation());
                     double centerX = obstacleCenter.getX();
                     double centerY = obstacleCenter.getY();

                     Point3D previous = new Point3D(centerX + OBSTACLE_RADIUS, centerY, 0.0);
                     for (int i = 1; i <= segments; i++)
                     {
                        double angle = i * angleStep;
                        Point3D current = new Point3D(centerX + OBSTACLE_RADIUS * Math.cos(angle),
                                                      centerY + OBSTACLE_RADIUS * Math.sin(angle),
                                                      0.0);
                        builder.addLine(previous, current, 0.004, Color.RED);
                        previous = current;
                     }
                  }
               }

               RRTConnectPathPlanner.Node rootNodeA = planner.getRootNodeA();
               if (rootNodeA != null)
               {
                  ArrayDeque<RRTConnectPathPlanner.Node> stack = new ArrayDeque<>();
                  stack.push(rootNodeA);
                  while (!stack.isEmpty())
                  {
                     RRTConnectPathPlanner.Node node = stack.pop();
                     for (RRTConnectPathPlanner.Node child : node.children)
                     {
                        builder.addLine(node.position, child.position, 0.005, Color.ORANGE);
                        builder.addCube(0.02, child.position, Color.WHITE);
                        stack.push(child);
                     }
                  }
               }

               RRTConnectPathPlanner.Node rootNodeB = planner.getRootNodeB();
               if (rootNodeB != null)
               {
                  ArrayDeque<RRTConnectPathPlanner.Node> stack = new ArrayDeque<>();
                  stack.push(rootNodeB);
                  while (!stack.isEmpty())
                  {
                     RRTConnectPathPlanner.Node node = stack.pop();
                     for (RRTConnectPathPlanner.Node child : node.children)
                     {
                        builder.addLine(node.position, child.position, 0.005, Color.FOREST);
                        builder.addCube(0.02, child.position, Color.WHITE);
                        stack.push(child);
                     }
                  }
               }

               Point3D previous = null;
               for (Point3D point : plannedPath)
               {
                  builder.addCube(0.02, point, Color.WHITE);
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
      new RDXRRTConnectPathPlannerDemo();
   }
}
