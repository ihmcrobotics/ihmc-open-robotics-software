package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.behaviors.tools.MinimalFootstep;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.simplePlanners.QuickFootstep;
import us.ihmc.footstepPlanning.simplePlanners.QuickFootstepPlanner;
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

public class RDXQuickFootstepPlannerDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("Quick Footstep Planner Demo");
   private final SideDependentList<RDXFootstepPlanGraphic> stanceFeet = new SideDependentList<>();
   private final SideDependentList<RDXFootstepPlanGraphic> goalFeet = new SideDependentList<>();
   private final SideDependentList<RDXPose3DGizmo> stanceGizmos = new SideDependentList<>();
   private final SideDependentList<RDXPose3DGizmo> goalGizmos = new SideDependentList<>();
   private final List<RDXPose3DGizmo> waypointGizmos = new ArrayList<>();
   private ConvexPolygon2D foothold;
   private final QuickFootstepPlanner planner = new QuickFootstepPlanner();
   private List<QuickFootstep> footstepPlan;
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
   private final ImBoolean includeGoalSteps = new ImBoolean(true);

   public RDXQuickFootstepPlannerDemo()
   {
      LibGDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

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

            baseUI.getImGuiPanelManager().addPanel("Quick Footstep Planner", this::renderImGuiWidgets);
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
            ImGui.pushItemWidth(ImGui.getColumnWidth());
            if (ImGui.sliderInt("###Max Steps", maxSteps, 1, 25, "Max steps: %d"))
               planner.setMaxSteps(maxSteps[0]);
            ImGui.sliderInt("###Waypoints", numberOfWaypoints, 0, 5, "Waypoints: %d");
            ImGui.checkbox("Plan to goal", includeGoalSteps);
            ImGui.popItemWidth();

            if (ImGui.button("Print Debug"))
               planner.getPrintNotification().set();

            for (RobotSide side : RobotSide.values)
               ImGui.text("Stance " + side + ": " + stanceGizmos.get(side).getTransformToParent().getTranslation()
                          + "  Yaw: (%.3f%s)".formatted(Math.toDegrees(stanceGizmos.get(side).getTransformToParent().getRotation().getYaw()), EuclidCoreMissingTools.DEGREE_SYMBOL));
            ImGui.text("Stance distance: %.3f".formatted(stanceGizmos.get(RobotSide.LEFT).getPose().getPosition()
                                                               .distance(stanceGizmos.get(RobotSide.RIGHT).getPose().getPosition())));

            for (int i = 0; i < numberOfWaypoints[0]; i++)
               ImGui.text("Waypoint " + i + ": " + waypointGizmos.get(i).getTransformToParent().getTranslation() +
                          "  Yaw: (%.3f%s)".formatted(Math.toDegrees(waypointGizmos.get(i).getTransformToParent().getRotation().getYaw()), EuclidCoreMissingTools.DEGREE_SYMBOL));

            for (RobotSide side : RobotSide.values)
               ImGui.text("Goal " + side + ": " + goalGizmos.get(side).getTransformToParent().getTranslation()
                          + "  Yaw: (%.3f%s)".formatted(Math.toDegrees(goalGizmos.get(side).getTransformToParent().getRotation().getYaw()), EuclidCoreMissingTools.DEGREE_SYMBOL));
            ImGui.text("Goal distance: %.3f".formatted(goalGizmos.get(RobotSide.LEFT).getPose().getPosition()
                                                             .distance(goalGizmos.get(RobotSide.RIGHT).getPose().getPosition())));

            ImGui.text("Planned Footsteps: " + footstepPlan.size());
            for (int i = 0; i < footstepPlan.size(); i++)
               ImGui.text("Step " + i + ": " + footstepPlan.get(i).getSwingSide() + " to " + footstepPlan.get(i).getSwingEnd().getPosition()
               + "  Yaw: (%.3f%s)%n\tDistance: %.3f".formatted(Math.toDegrees(footstepPlan.get(i).getSwingEnd().getYaw()),
                                                               EuclidCoreMissingTools.DEGREE_SYMBOL,
                                                               footstepPlan.get(i).getSwingDistance()));
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
                  builder.addSphere(0.015, planner.getSwingHip().get(planner.getFootToSwing()).getPosition(),
                                    RDXFootstepGraphic.FOOT_COLORS.get(planner.getFootToSwing()));
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

            List<Pose3D> waypoints = new ArrayList<>();
            for (int i = 0; i < numberOfWaypoints[0]; i++)
               waypoints.add(new Pose3D(waypointGizmos.get(i).getGizmoFrame().getTransformToRoot()));
            footstepPlan = planner.plan(stances, waypoints, includeGoalSteps.get() ? goals : null);

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            for (RobotSide side : RobotSide.values)
            {
               stanceFeet.get(side).destroy();
               goalFeet.get(side).destroy();
            }
            for (RDX3DSituatedText text : footstepIndexTexts)
               text.dispose();
            baseUI.dispose();
         }
      }, getClass());
   }

   public static void main(String[] args)
   {
      new RDXQuickFootstepPlannerDemo();
   }
}
