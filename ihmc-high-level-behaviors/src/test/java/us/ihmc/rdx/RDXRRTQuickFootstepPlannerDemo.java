package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.behaviors.tools.MinimalFootstep;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.simplePlanners.QuickFootstepPlanner;
import us.ihmc.pathPlanning.rrt.RRTConnectPathPlanner;
import us.ihmc.rdx.tools.LibGDXApplicationCreator;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.footstepPlanning.simplePlanners.QuickFootstepPlanner.Footstep;

public class RDXRRTQuickFootstepPlannerDemo
{
   private static final double OBSTACLE_RADIUS = 0.5;
   private final RDXBaseUI baseUI = new RDXBaseUI("RRT Quick Footstep Planner Demo");
   private final SideDependentList<RDXFootstepPlanGraphic> stanceFeet = new SideDependentList<>();
   private final SideDependentList<RDXFootstepPlanGraphic> goalFeet = new SideDependentList<>();
   private final SideDependentList<RDXPose3DGizmo> stanceGizmos = new SideDependentList<>();
   private final SideDependentList<RDXPose3DGizmo> goalGizmos = new SideDependentList<>();
   private final List<RDXPose3DGizmo> waypointGizmos = new ArrayList<>();
   private final List<RDXPose3DGizmo> obstacleGizmos = new ArrayList<>();
   private ConvexPolygon2D foothold;
   private final QuickFootstepPlanner planner = new QuickFootstepPlanner();
   private final RRTConnectPathPlanner rrtPlanner = new RRTConnectPathPlanner();
   private List<Footstep> footstepPlan = new ArrayList<>();
   private List<Point3D> rrtPath = new ArrayList<>();
   private List<Pose3D> rrtWaypoints = new ArrayList<>();
   private ModelInstance rrtModel;
   private final List<ModelInstance> visualModels = new ArrayList<>();
   private final List<RDX3DSituatedText> footstepIndexTexts = new ArrayList<>();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final ReferenceFrame footstepFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("footstepFrame",
                                                                                                              ReferenceFrame.getWorldFrame(),
                                                                                                              tempTransform);
   private final FramePose3D textFramePose = new FramePose3D();
   private int footstepIndexCounter = 0;
   private final int[] maxSteps = {50};
   private final int[] numberOfWaypoints = {0};
   private final int[] numberOfObstacles = {0};
   private final ImBoolean includeGoalSteps = new ImBoolean(true);
   private final ImBoolean autoPlanRRT = new ImBoolean(true);
   private final int[] rrtMaxIterations = {1000};
   private final int[] rrtMultiRestartIterations = {10};
   private final float[] rrtWaypointSpacingMeters = {0.4f};
   private final float[] hipWidthCm = {12.0f};
   private final float[] stepLengthCm = {28.0f};
   private final int[] nextPelvisYawLimitDeg = {35};
   private final int[] inwardLimitDeg = {10};
   private final int[] outwardLimitDeg = {50};
   private final int[] stepAngleLimitDeg = {115};
   private final Point3D midStance = new Point3D();
   private final Point3D midGoal = new Point3D();
   private final Point3D rrtStart = new Point3D();
   private final Point3D rrtGoal = new Point3D();
   private final Point3D collisionQuery = new Point3D();
   private boolean rrtGoalFromWaypoint = false;
   private boolean rrtGoalMissingWaypoint = false;
   private boolean rrtPlanRequested = true;
   private double plannerPlanDurationMs = 0.0;
   private double rrtPlanDurationMs = 0.0;
   private double rrtPathLength = 0.0;

   public RDXRRTQuickFootstepPlannerDemo()
   {
      LibGDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            rrtPlanner.setMaxIterations(rrtMaxIterations[0]);
            rrtPlanner.setMultiRestartIterations(rrtMultiRestartIterations[0]);

            foothold = planner.createFootPolygon(new Pose3D(), 0.0);

            // Initialize stance feet graphics and gizmos near origin
            for (RobotSide side : RobotSide.values)
            {
               RDXFootstepPlanGraphic stanceGraphic = new RDXFootstepPlanGraphic();
               ArrayList<MinimalFootstep> stanceFootsteps = new ArrayList<>();
               Pose3D stancePose = new Pose3D();
               stancePose.setY(side == RobotSide.LEFT ? 0.12 : -0.12);
               stanceFootsteps.add(new MinimalFootstep(side, stancePose, foothold, "Stance " + side.getPascalCaseName()));
               stanceGraphic.generateMeshes(stanceFootsteps);
               stanceFeet.put(side, stanceGraphic);
               baseUI.getPrimaryScene().addRenderableProvider(stanceGraphic);

               RDXPose3DGizmo stanceGizmo = new RDXPose3DGizmo();
               stanceGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
               stanceGizmo.getTransformToParent().getTranslation().set(0.0, side == RobotSide.LEFT ? 0.12 : -0.12, 0.0);
               stanceGizmo.setResizeAutomatically(false);
               stanceGizmo.setCenterSphereToTorusRatio(0.8f);
               stanceGizmos.put(side, stanceGizmo);
            }

            // Initialize goal feet graphics and gizmos at 0.5m forward
            for (RobotSide side : RobotSide.values)
            {
               RDXFootstepPlanGraphic goalGraphic = new RDXFootstepPlanGraphic();
               ArrayList<MinimalFootstep> goalFootsteps = new ArrayList<>();
               Pose3D goalPose = new Pose3D();
               goalPose.setX(0.5);
               goalPose.setY(side == RobotSide.LEFT ? 0.12 : -0.12);
               goalFootsteps.add(new MinimalFootstep(side, goalPose, foothold, "Goal " + side.getPascalCaseName()));
               goalGraphic.generateMeshes(goalFootsteps);
               goalFeet.put(side, goalGraphic);
               baseUI.getPrimaryScene().addRenderableProvider(goalGraphic);

               RDXPose3DGizmo goalGizmo = new RDXPose3DGizmo();
               goalGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
               goalGizmo.getTransformToParent().getTranslation().set(0.5, side == RobotSide.LEFT ? 0.12 : -0.12, 0.0);
               goalGizmo.setResizeAutomatically(false);
               goalGizmo.setCenterSphereToTorusRatio(0.8f);
               goalGizmos.put(side, goalGizmo);
            }

            for (int i = 0; i < 5; i++)
            {
               RDXPose3DGizmo waypointGizmo = new RDXPose3DGizmo();
               waypointGizmo.create(baseUI.getPrimary3DPanel());
               baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(waypointGizmo::calculate3DViewPick);
               baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(waypointGizmo::process3DViewInput);
               waypointGizmo.getTransformToParent().getTranslation().set(0.5 + i * 0.1, 0.0, 0.0);
               waypointGizmo.setResizeAutomatically(false);
               waypointGizmo.setCenterSphereToTorusRatio(0.8f);
               waypointGizmos.add(waypointGizmo);
            }
            baseUI.getPrimaryScene().addRenderableProvider((renderables, pool) ->
            {
               for (int i = 0; i < numberOfWaypoints[0]; i++)
                  waypointGizmos.get(i).getRenderables(renderables, pool);
            });

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
            baseUI.getPrimaryScene().addRenderableProvider((renderables, pool) ->
            {
               for (int i = 0; i < numberOfObstacles[0]; i++)
                  obstacleGizmos.get(i).getRenderables(renderables, pool);
            });

            baseUI.getImGuiPanelManager().addPanel("RRT Quick Footstep Planner", this::renderImGuiWidgets);
            baseUI.getPrimaryScene().addRenderableProvider((renderables, pool) ->
            {
               if (rrtModel != null)
                  rrtModel.getRenderables(renderables, pool);
            });
            baseUI.getPrimaryScene().addRenderableProvider((renderables, pool) ->
            {
               for (ModelInstance model : visualModels)
                  model.getRenderables(renderables, pool);
            });
            baseUI.getPrimaryScene().addRenderableProvider((renderables, pool) ->
            {
               for (RDX3DSituatedText text : footstepIndexTexts)
                  text.getRenderables(renderables, pool);
            });
         }

         private void renderImGuiWidgets()
         {
            ImGui.pushItemWidth(15.0f * ImGui.getFontSize());
            if (ImGui.sliderInt("###Max Steps", maxSteps, 1, 25, "Max steps: %d"))
               planner.setMaxSteps(maxSteps[0]);
            if (ImGui.sliderFloat("###Hip Width", hipWidthCm, 0.0f, 24.0f, "Hip width: %.1f cm"))
               planner.setHipWidth(hipWidthCm[0] / 100.0);
            ImGui.sameLine();
            if (ImGui.button("Default##HipWidth"))
            {
               hipWidthCm[0] = 12.0f;
               planner.setHipWidth(0.12);
            }
            if (ImGui.sliderFloat("###Step Length", stepLengthCm, 0.0f, 56.0f, "Step length: %.1f cm"))
               planner.setStepLength(stepLengthCm[0] / 100.0);
            ImGui.sameLine();
            if (ImGui.button("Default##StepLength"))
            {
               stepLengthCm[0] = 28.0f;
               planner.setStepLength(0.28);
            }
            if (ImGui.sliderInt("###Next Pelvis Yaw Limit", nextPelvisYawLimitDeg, 0, 70, "Next pelvis yaw limit: %d deg"))
               planner.setNextPelvisYawLimit(Math.toRadians(nextPelvisYawLimitDeg[0]));
            ImGui.sameLine();
            if (ImGui.button("Default##NextPelvisYawLimit"))
            {
               nextPelvisYawLimitDeg[0] = 35;
               planner.setNextPelvisYawLimit(Math.toRadians(35));
            }
            if (ImGui.sliderInt("###Inward Limit", inwardLimitDeg, 0, 20, "Inward limit: %d deg"))
               planner.setInwardLimit(Math.toRadians(inwardLimitDeg[0]));
            ImGui.sameLine();
            if (ImGui.button("Default##InwardLimit"))
            {
               inwardLimitDeg[0] = 10;
               planner.setInwardLimit(Math.toRadians(10));
            }
            if (ImGui.sliderInt("###Outward Limit", outwardLimitDeg, 0, 100, "Outward limit: %d deg"))
               planner.setOutwardLimit(Math.toRadians(outwardLimitDeg[0]));
            ImGui.sameLine();
            if (ImGui.button("Default##OutwardLimit"))
            {
               outwardLimitDeg[0] = 50;
               planner.setOutwardLimit(Math.toRadians(50));
            }
            if (ImGui.sliderInt("###Step Angle Limit", stepAngleLimitDeg, 0, 230, "Step angle limit: %d deg"))
               planner.setStepAngleLimit(Math.toRadians(stepAngleLimitDeg[0]));
            ImGui.sameLine();
            if (ImGui.button("Default##StepAngleLimit"))
            {
               stepAngleLimitDeg[0] = 115;
               planner.setStepAngleLimit(Math.toRadians(115));
            }
            if (ImGui.sliderInt("###RRT Max Iterations", rrtMaxIterations, 0, 10000, "RRT max iterations: %d"))
               rrtPlanner.setMaxIterations(rrtMaxIterations[0]);
            if (ImGui.sliderInt("###RRT Restarts", rrtMultiRestartIterations, 1, 50, "RRT restarts: %d"))
               rrtPlanner.setMultiRestartIterations(rrtMultiRestartIterations[0]);
            ImGui.sliderFloat("###RRT Waypoint Spacing", rrtWaypointSpacingMeters, 0.2f, 1.0f, "RRT waypoint spacing: %.2f m");
            if (ImGui.button("Plan RRT"))
               rrtPlanRequested = true;
            ImGui.sameLine();
            ImGui.checkbox("Plan RRT each frame", autoPlanRRT);
            ImGui.sliderInt("###Obstacles", numberOfObstacles, 0, obstacleGizmos.size(), "Obstacles: %d");
            ImGui.sliderInt("###Target Waypoints", numberOfWaypoints, 0, 5, "Target waypoints: %d");
            ImGui.checkbox("Plan to goal", includeGoalSteps);
            if (!includeGoalSteps.get() && numberOfWaypoints[0] == 0)
               ImGui.text("No waypoints: RRT goal falls back to mid-goal.");
            ImGui.popItemWidth();

            if (ImGui.button("Print Debug"))
               planner.getPrintNotification().set();

            for (RobotSide side : RobotSide.values)
               ImGui.text("Stance " + side + ": " + stanceGizmos.get(side).getTransformToParent().getTranslation()
                          + "  Yaw: (%.3f%s)".formatted(Math.toDegrees(stanceGizmos.get(side).getTransformToParent().getRotation().getYaw()), EuclidCoreMissingTools.DEGREE_SYMBOL));
            ImGui.text("Stance distance: %.3f".formatted(stanceGizmos.get(RobotSide.LEFT).getPose().getPosition()
                                                               .distance(stanceGizmos.get(RobotSide.RIGHT).getPose().getPosition())));

            for (int i = 0; i < numberOfWaypoints[0]; i++)
               ImGui.text("Target waypoint " + i + ": " + waypointGizmos.get(i).getTransformToParent().getTranslation() +
                          "  Yaw: (%.3f%s)".formatted(Math.toDegrees(waypointGizmos.get(i).getTransformToParent().getRotation().getYaw()), EuclidCoreMissingTools.DEGREE_SYMBOL));

            for (RobotSide side : RobotSide.values)
               ImGui.text("Goal " + side + ": " + goalGizmos.get(side).getTransformToParent().getTranslation()
                          + "  Yaw: (%.3f%s)".formatted(Math.toDegrees(goalGizmos.get(side).getTransformToParent().getRotation().getYaw()), EuclidCoreMissingTools.DEGREE_SYMBOL));
            ImGui.text("Goal distance: %.3f".formatted(goalGizmos.get(RobotSide.LEFT).getPose().getPosition()
                                                             .distance(goalGizmos.get(RobotSide.RIGHT).getPose().getPosition())));

            ImGui.text("RRT start: " + rrtStart);
            ImGui.text("RRT goal: " + rrtGoal + (rrtGoalFromWaypoint ? " (last waypoint)" : " (mid-goal)"));
            if (rrtGoalMissingWaypoint)
               ImGui.text("RRT goal: no waypoint set, using mid-goal.");
            ImGui.text("RRT path points: " + rrtPath.size());
            ImGui.text("RRT waypoints: " + rrtWaypoints.size());
            ImGui.text("RRT path length: %.3f".formatted(rrtPathLength));
            ImGui.text("RRT plan duration: %.3f ms".formatted(rrtPlanDurationMs));
            ImGui.text("Plan duration: %.3f ms".formatted(plannerPlanDurationMs));
            ImGui.text("Planned Footsteps: " + footstepPlan.size());
            for (int i = 0; i < footstepPlan.size(); i++)
               ImGui.text("Step " + i + ": " + footstepPlan.get(i).swingSide() + " to " + footstepPlan.get(i).swingEnd().getPosition()
               + "  Yaw: (%.3f%s)%n\tDistance: %.3f".formatted(Math.toDegrees(footstepPlan.get(i).swingEnd().getYaw()),
                                                               EuclidCoreMissingTools.DEGREE_SYMBOL,
                                                               footstepPlan.get(i).swingDistance()));
         }

         @Override
         public void render()
         {
            // Update stance feet to match gizmo transforms
            for (RobotSide side : RobotSide.values)
            {
               ArrayList<MinimalFootstep> stanceFootsteps = new ArrayList<>();
               Pose3D stancePose = new Pose3D(stanceGizmos.get(side).getTransformToParent());
               stanceFootsteps.add(new MinimalFootstep(side, stancePose, foothold, "Stance " + side.getPascalCaseName()));
               stanceFeet.get(side).generateMeshes(stanceFootsteps);
               stanceFeet.get(side).update();
            }

            // Update goal feet to match gizmo transforms
            for (RobotSide side : RobotSide.values)
            {
               ArrayList<MinimalFootstep> goalFootsteps = new ArrayList<>();
               Pose3D goalPose = new Pose3D(goalGizmos.get(side).getTransformToParent());
               goalFootsteps.add(new MinimalFootstep(side, goalPose, foothold, "Goal " + side.getPascalCaseName()));
               goalFeet.get(side).generateMeshes(goalFootsteps);
               goalFeet.get(side).update();
            }

            // Plan footsteps from stance to goal
            SideDependentList<Pose3D> stances = new SideDependentList<>(side -> new Pose3D(stanceGizmos.get(side).getTransformToParent()));
            SideDependentList<Pose3D> goals = new SideDependentList<>(side -> new Pose3D(goalGizmos.get(side).getTransformToParent()));

            midStance.interpolate(stanceGizmos.get(RobotSide.LEFT).getTransformToParent().getTranslation(),
                                  stanceGizmos.get(RobotSide.RIGHT).getTransformToParent().getTranslation(),
                                  0.5);
            midGoal.interpolate(goalGizmos.get(RobotSide.LEFT).getTransformToParent().getTranslation(),
                                goalGizmos.get(RobotSide.RIGHT).getTransformToParent().getTranslation(),
                                0.5);
            rrtStart.set(midStance);
            rrtGoalFromWaypoint = !includeGoalSteps.get() && numberOfWaypoints[0] > 0;
            rrtGoalMissingWaypoint = !includeGoalSteps.get() && numberOfWaypoints[0] == 0;
            if (rrtGoalFromWaypoint)
               rrtGoal.set(waypointGizmos.get(numberOfWaypoints[0] - 1).getTransformToParent().getTranslation());
            else
               rrtGoal.set(midGoal);

            if (autoPlanRRT.get() || rrtPlanRequested)
            {
               long rrtStartNs = System.nanoTime();
               List<Point3D> rrtResult = rrtPlanner.plan(rrtStart, rrtGoal, (LineSegment3D segment) ->
               {
                  for (int i = 0; i < numberOfObstacles[0]; i++)
                  {
                     collisionQuery.set(obstacleGizmos.get(i).getTransformToParent().getTranslation());
                     if (segment.distance(collisionQuery) <= OBSTACLE_RADIUS)
                        return true;
                  }
                  return false;
               });
               rrtPlanDurationMs = (System.nanoTime() - rrtStartNs) / 1_000_000.0;
               rrtPath = rrtResult != null ? rrtResult : new ArrayList<>();
               rrtPathLength = calculatePathLength(rrtPath);
               rrtWaypoints = buildWaypointsFromPath(rrtPath, rrtWaypointSpacingMeters[0]);
               updateRRTModel();
               rrtPlanRequested = false;
            }

            for (ModelInstance model : visualModels)
               model.model.dispose();
            visualModels.clear();
            for (RDX3DSituatedText text : footstepIndexTexts)
               text.dispose();
            footstepIndexTexts.clear();
            footstepIndexCounter = 0;
            planner.setStepPlannedCallback(() ->
            {
               int footstepIndex = footstepIndexCounter++;
               visualModels.add(RDXModelBuilder.buildModelInstance(builder ->
               {
                  SideDependentList<Point3D> swingHipAir = new SideDependentList<>(() -> new Point3D());
                  SideDependentList<Point3D> goalDirectionAir = new SideDependentList<>(() -> new Point3D());
                  for (RobotSide side : RobotSide.values)
                  {
                     swingHipAir.get(side).set(planner.getSwingHip().get(side).getPosition());
                     swingHipAir.get(side).addZ(0.8f);
                     builder.addSphere(0.015, swingHipAir.get(side), RDXFootstepGraphic.FOOT_COLORS.get(side));
                     goalDirectionAir.get(side).set(planner.getSwingHip().get(side).getPosition());
                     Vector3D shortened = new Vector3D(planner.getToGoalLinear().get(side));
                     shortened.scale(0.2);
                     goalDirectionAir.get(side).add(shortened);
                     goalDirectionAir.get(side).addZ(0.8f);
                     builder.addLine(swingHipAir.get(side), goalDirectionAir.get(side), 0.01, Color.SKY);
                  }
                  builder.addLine(swingHipAir.get(planner.getFootToSwing()), planner.getSwingEnd().getPosition(), 0.01, Color.WHITE);
//                  builder.addSphere(0.015, planner.getSwingHip().get(planner.getFootToSwing()).getPosition(),
//                                    RDXFootstepGraphic.FOOT_COLORS.get(planner.getFootToSwing()));
                  builder.addSphere(0.01, planner.getPelvis().get(planner.getFootToSwing()).getPosition(), Color.WHITE);
                  Vector3D pelvisForward = new Vector3D(Axis3D.X);
                  planner.getPelvis().get(planner.getFootToSwing()).transform(pelvisForward);
                  pelvisForward.scale(0.04);
                  Point3D pelvisEnd = new Point3D(planner.getPelvis().get(planner.getFootToSwing()).getPosition());
                  pelvisEnd.add(pelvisForward);
                  builder.addLine(planner.getPelvis().get(planner.getFootToSwing()).getPosition(), pelvisEnd, 0.003, Color.WHITE);
                  builder.addSphere(0.01, planner.getNextPelvis().get(planner.getFootToSwing()).getPosition(), Color.TAN);
                  Vector3D nextPelvisForward = new Vector3D(Axis3D.X);
                  planner.getNextPelvis().get(planner.getFootToSwing()).transform(nextPelvisForward);
                  nextPelvisForward.scale(0.04);
                  Point3D nextPelvisEnd = new Point3D(planner.getNextPelvis().get(planner.getFootToSwing()).getPosition());
                  nextPelvisEnd.add(nextPelvisForward);
                  builder.addLine(planner.getNextPelvis().get(planner.getFootToSwing()).getPosition(), nextPelvisEnd, 0.004, Color.TAN);

                  Color color = planner.getFootToSwing() == RobotSide.LEFT ? new Color(0.6f, 0.2f, 0.3f, 1.0f)
                                                                           : new Color(0.2f, 0.6f, 0.3f, 1.0f);
                  builder.addMultiLine(planner.getSwingEnd(), foothold.getPolygonVerticesView(), 0.01, color, true);
                  builder.addPolygon(planner.getSwingEnd(), foothold, color);
               }));
//               ModelInstance nextPelvisModel = RDXModelBuilder.createCoordinateFrameInstance(0.1);
//               RigidBodyTransform nextPelvisAir = new RigidBodyTransform(planner.getNextPelvis().get(planner.getFootToSwing()));
//               nextPelvisAir.getTranslation().addZ(0.8);
//               LibGDXTools.toLibGDX(nextPelvisAir, nextPelvisModel.transform);
//               visualModels.add(nextPelvisModel);
               ModelInstance goalModel = RDXModelBuilder.createCoordinateFrameInstance(0.1, Color.PINK);
               RigidBodyTransform goalAir = new RigidBodyTransform(planner.getGoalMid());
               goalAir.getTranslation().addZ(0.8);
               LibGDXTools.toLibGDX(goalAir, goalModel.transform);
               visualModels.add(goalModel);
               ModelInstance boundaryModel = RDXModelBuilder.buildModelInstance(builder ->
               {
                  builder.addArcTorus(0.0, 2.0 * Math.PI, 0.15, 0.002, Color.ORANGE);
               });
               LibGDXTools.toLibGDX(planner.getGoalMid().getPosition(), boundaryModel.transform);
               visualModels.add(boundaryModel);

               RobotSide hipSide = planner.getFootToSwing().getOppositeSide();
               Pose3D swingHip = planner.getSwingHip().get(hipSide);
               ModelInstance steppableAreaModel = RDXModelBuilder.buildModelInstance(builder ->
               {
                  List<Point3D> points = new ArrayList<>();
                  Point3D start = new Point3D();
                  points.add(start);
                  Vector3D swingHipLateral = new Vector3D(0.0, hipSide.negateIfRightSide(1.0), 0.0);
                  double stepLength = stepLengthCm[0] / 100.0;
                  swingHipLateral.scale(stepLength);
                  Vector3D frontBoundVector = new Vector3D(swingHipLateral);
                  Vector3D backBoundVector = new Vector3D(swingHipLateral);
                  double stepAngleLimit = Math.toRadians(stepAngleLimitDeg[0]);
                  new AxisAngle(Axis3D.Z, (hipSide == RobotSide.LEFT ? -1.0 : 1.0) * stepAngleLimit).transform(frontBoundVector);
                  new AxisAngle(Axis3D.Z, (hipSide == RobotSide.LEFT ? 1.0 : -1.0) * stepAngleLimit).transform(backBoundVector);
                  Point3D frontBound = new Point3D(start);
                  frontBound.add(frontBoundVector);
                  points.add(frontBound);
                  Point3D backBound = new Point3D(start);
                  backBound.add(backBoundVector);
                  points.add(backBound);

                  builder.addLine(start, frontBound, 0.002, Color.WHITE);
                  builder.addLine(start, backBound, 0.002, Color.WHITE);
                  if (hipSide == RobotSide.LEFT)
                  {
                     builder.addArcTorus(-stepAngleLimit + Math.PI / 2.0, stepAngleLimit + Math.PI / 2.0, stepLength, 0.002, Color.WHITE);
                     builder.addArcTorus(-(1.5 * Math.PI - stepAngleLimit), 0.5 * Math.PI - stepAngleLimit, 0.05, 0.002, Color.WHITE);
                  }
                  else
                  {
                     builder.addArcTorus(-stepAngleLimit - Math.PI / 2.0, stepAngleLimit - Math.PI / 2.0, stepLength, 0.002, Color.WHITE);
                     builder.addArcTorus(-(0.5 * Math.PI - stepAngleLimit), 1.5 * Math.PI - stepAngleLimit, 0.05, 0.002, Color.WHITE);
                  }
               });
               LibGDXTools.toLibGDX(new RigidBodyTransform(swingHip), steppableAreaModel.transform);
               visualModels.add(steppableAreaModel);

               float textHeight = 0.08f;
               RDX3DSituatedText footstepIndexText = new RDX3DSituatedText(String.valueOf(footstepIndex), textHeight);
               planner.getSwingEnd().get(tempTransform);
               footstepFrame.update();
               textFramePose.setToZero(footstepFrame);
               textFramePose.getOrientation().prependYawRotation(-Math.PI / 2.0);
               textFramePose.getPosition().addZ(0.01);
               textFramePose.getPosition().addY(textHeight / 4.0);
               textFramePose.getPosition().addX(-textHeight / 2.0);
               textFramePose.changeFrame(ReferenceFrame.getWorldFrame());
               LibGDXTools.toLibGDX(textFramePose, tempTransform, footstepIndexText.getModelTransform());
               footstepIndexTexts.add(footstepIndexText);
            });

            long planStartNs = System.nanoTime();
            footstepPlan = planner.plan(stances, rrtWaypoints, includeGoalSteps.get() ? goals : null);
            plannerPlanDurationMs = (System.nanoTime() - planStartNs) / 1_000_000.0;

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void updateRRTModel()
         {
            if (rrtModel != null)
               rrtModel.model.dispose();

            rrtModel = RDXModelBuilder.buildModelInstance(builder ->
            {
               double radius = rrtPlanner.getSearchRadius();
               if (radius > 0.0)
               {
                  Point3D center = rrtPlanner.getCenter();
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

               Point3D previous = null;
               for (Point3D point : rrtPath)
               {
                  builder.addCube(0.02, point, Color.WHITE);
                  if (previous != null)
                     builder.addLine(previous, point, 0.01, Color.CYAN);
                  previous = point;
               }

               for (Pose3D waypoint : rrtWaypoints)
                  builder.addCube(0.03, waypoint.getPosition(), Color.YELLOW);
            });
         }

         private List<Pose3D> buildWaypointsFromPath(List<Point3D> path, double spacingMeters)
         {
            List<Pose3D> waypoints = new ArrayList<>();
            if (path.isEmpty())
               return waypoints;

            double distanceSinceLast = 0.0;
            Point3D previous = path.get(0);
            double lastYaw = 0.0;

            for (int i = 1; i < path.size(); i++)
            {
               Point3D current = path.get(i);
               double segmentLength = previous.distance(current);
               if (segmentLength < 1.0e-9)
               {
                  previous = current;
                  continue;
               }

               distanceSinceLast += segmentLength;
               boolean isLast = i == path.size() - 1;
               if (distanceSinceLast >= spacingMeters || isLast)
               {
                  double yaw = Math.atan2(current.getY() - previous.getY(), current.getX() - previous.getX());
                  lastYaw = yaw;
                  Pose3D waypoint = new Pose3D();
                  waypoint.getPosition().set(current);
                  waypoint.getOrientation().setToYawOrientation(yaw);
                  waypoints.add(waypoint);
                  distanceSinceLast = 0.0;
               }

               previous = current;
            }

            if (waypoints.isEmpty())
            {
               Pose3D waypoint = new Pose3D();
               waypoint.getPosition().set(path.get(path.size() - 1));
               waypoint.getOrientation().setToYawOrientation(lastYaw);
               waypoints.add(waypoint);
            }

            return waypoints;
         }

         private double calculatePathLength(List<Point3D> path)
         {
            double length = 0.0;
            for (int i = 1; i < path.size(); i++)
               length += path.get(i - 1).distance(path.get(i));
            return length;
         }

         @Override
         public void dispose()
         {
            for (RobotSide side : RobotSide.values)
            {
               stanceFeet.get(side).destroy();
               goalFeet.get(side).destroy();
            }
            if (rrtModel != null)
               rrtModel.model.dispose();
            for (RDX3DSituatedText text : footstepIndexTexts)
               text.dispose();
            baseUI.dispose();
         }
      }, getClass());
   }

   public static void main(String[] args)
   {
      new RDXRRTQuickFootstepPlannerDemo();
   }
}
