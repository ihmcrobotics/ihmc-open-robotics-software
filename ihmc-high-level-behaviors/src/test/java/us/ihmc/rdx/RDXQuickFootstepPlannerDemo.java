package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import org.apache.commons.math3.util.Pair;
import us.ihmc.behaviors.tools.MinimalFootstep;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.simplePlanners.QuickFootstepPlanner;
import us.ihmc.rdx.tools.LibGDXApplicationCreator;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.robotSide.SideMap;

import java.util.ArrayList;
import java.util.List;

public class RDXQuickFootstepPlannerDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("Quick Footstep Planner Demo");
   private final SideDependentList<RDXFootstepPlanGraphic> stanceFeet = new SideDependentList<>();
   private final SideDependentList<RDXFootstepPlanGraphic> goalFeet = new SideDependentList<>();
   private final SideDependentList<RDXSelectablePose3DGizmo> stanceGizmos = new SideDependentList<>();
   private final SideDependentList<RDXSelectablePose3DGizmo> goalGizmos = new SideDependentList<>();
   private ConvexPolygon2D foothold;
   private final QuickFootstepPlanner planner = new QuickFootstepPlanner();
   private List<Pair<RobotSide, Pose3D>> footstepPlan;
   private final List<ModelInstance> visualModels = new ArrayList<>();
   private final List<RDX3DSituatedText> footstepIndexTexts = new ArrayList<>();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final ReferenceFrame footstepFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("footstepFrame",
                                                                                                              ReferenceFrame.getWorldFrame(),
                                                                                                              tempTransform);
   private final FramePose3D textFramePose = new FramePose3D();
   private int footstepIndexCounter = 0;
   private final int[] maxSteps = {50};

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
               stancePose.setY(side == RobotSide.LEFT ? 0.1 : -0.1);
               stanceFootsteps.add(new MinimalFootstep(side, stancePose, foothold, "Stance " + side.getPascalCaseName()));
               stanceGraphic.generateMeshes(stanceFootsteps);
               stanceFeet.put(side, stanceGraphic);
               baseUI.getPrimaryScene().addRenderableProvider(stanceGraphic);

               RDXSelectablePose3DGizmo stanceGizmo = new RDXSelectablePose3DGizmo();
               stanceGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
               stanceGizmo.getPoseGizmo().getTransformToParent().getTranslation().set(0.0, side == RobotSide.LEFT ? 0.1 : -0.1, 0.0);
               stanceGizmo.getPoseGizmo().setResizeAutomatically(false);
               stanceGizmo.getPoseGizmo().setCenterSphereToTorusRatio(0.8f);
               stanceGizmo.setSelected(true);
               stanceGizmos.put(side, stanceGizmo);
            }

            // Initialize goal feet graphics and gizmos at 0.5m forward
            for (RobotSide side : RobotSide.values)
            {
               RDXFootstepPlanGraphic goalGraphic = new RDXFootstepPlanGraphic();
               ArrayList<MinimalFootstep> goalFootsteps = new ArrayList<>();
               Pose3D goalPose = new Pose3D();
               goalPose.setX(0.5);
               goalPose.setY(side == RobotSide.LEFT ? 0.1 : -0.1);
               goalFootsteps.add(new MinimalFootstep(side, goalPose, foothold, "Goal " + side.getPascalCaseName()));
               goalGraphic.generateMeshes(goalFootsteps);
               goalFeet.put(side, goalGraphic);
               baseUI.getPrimaryScene().addRenderableProvider(goalGraphic);

               RDXSelectablePose3DGizmo goalGizmo = new RDXSelectablePose3DGizmo();
               goalGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
               goalGizmo.getPoseGizmo().getTransformToParent().getTranslation().set(0.5, side == RobotSide.LEFT ? 0.1 : -0.1, 0.0);
               goalGizmo.getPoseGizmo().setResizeAutomatically(false);
               goalGizmo.getPoseGizmo().setCenterSphereToTorusRatio(0.8f);
               goalGizmo.setSelected(true);
               goalGizmos.put(side, goalGizmo);
            }

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
            ImGui.popItemWidth();

            for (RobotSide side : RobotSide.values)
               ImGui.text("Stance " + side + ": " + stanceGizmos.get(side).getPoseGizmo().getTransformToParent().getTranslation()
                          + "  Yaw: (%.3f%s)".formatted(Math.toDegrees(stanceGizmos.get(side).getPoseGizmo().getTransformToParent().getRotation().getYaw()), EuclidCoreMissingTools.DEGREE_SYMBOL));
            ImGui.text("Stance distance: %.3f".formatted(stanceGizmos.get(RobotSide.LEFT).getPoseGizmo().getPose().getPosition()
                                                               .distance(stanceGizmos.get(RobotSide.RIGHT).getPoseGizmo().getPose().getPosition())));
            for (RobotSide side : RobotSide.values)
               ImGui.text("Goal " + side + ": " + goalGizmos.get(side).getPoseGizmo().getTransformToParent().getTranslation()
                          + "  Yaw: (%.3f%s)".formatted(Math.toDegrees(goalGizmos.get(side).getPoseGizmo().getTransformToParent().getRotation().getYaw()), EuclidCoreMissingTools.DEGREE_SYMBOL));
            ImGui.text("Goal distance: %.3f".formatted(goalGizmos.get(RobotSide.LEFT).getPoseGizmo().getPose().getPosition()
                                                             .distance(goalGizmos.get(RobotSide.RIGHT).getPoseGizmo().getPose().getPosition())));

            ImGui.text("Planned Footsteps: " + footstepPlan.size());
            for (int i = 0; i < footstepPlan.size(); i++)
               ImGui.text("Step " + i + ": " + footstepPlan.get(i).getFirst() + " to " + footstepPlan.get(i).getSecond().getPosition()
               + "  Yaw: (%.3f%s)".formatted(Math.toDegrees(footstepPlan.get(i).getSecond().getYaw()), EuclidCoreMissingTools.DEGREE_SYMBOL));
         }

         @Override
         public void render()
         {
            // Update stance feet to match gizmo transforms
            for (RobotSide side : RobotSide.values)
            {
               ArrayList<MinimalFootstep> stanceFootsteps = new ArrayList<>();
               Pose3D stancePose = new Pose3D(stanceGizmos.get(side).getPoseGizmo().getTransformToParent());
               stanceFootsteps.add(new MinimalFootstep(side, stancePose, foothold, "Stance " + side.getPascalCaseName()));
               stanceFeet.get(side).generateMeshes(stanceFootsteps);
               stanceFeet.get(side).update();
            }

            // Update goal feet to match gizmo transforms
            for (RobotSide side : RobotSide.values)
            {
               ArrayList<MinimalFootstep> goalFootsteps = new ArrayList<>();
               Pose3D goalPose = new Pose3D(goalGizmos.get(side).getPoseGizmo().getTransformToParent());
               goalFootsteps.add(new MinimalFootstep(side, goalPose, foothold, "Goal " + side.getPascalCaseName()));
               goalFeet.get(side).generateMeshes(goalFootsteps);
               goalFeet.get(side).update();
            }

            // Plan footsteps from stance to goal
            SideDependentList<Pose3D> stances = new SideDependentList<>(side -> new Pose3D(stanceGizmos.get(side).getPoseGizmo().getTransformToParent()));
            SideDependentList<Pose3D> goals = new SideDependentList<>(side -> new Pose3D(goalGizmos.get(side).getPoseGizmo().getTransformToParent()));

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
                  SideMap<Point3D> stanceHipAir = new SideMap<>(() -> new Point3D());
                  SideMap<Point3D> goalHipAir = new SideMap<>(() -> new Point3D());
                  for (RobotSide side : RobotSide.values)
                  {
                     stanceHipAir.get(side).set(planner.getStanceHip().get(side));
                     stanceHipAir.get(side).addZ(0.8f);
                     builder.addSphere(0.03, stanceHipAir.get(side), RDXFootstepGraphic.FOOT_COLORS.get(side));
                     goalHipAir.get(side).set(planner.getGoalHip().get(side));
                     goalHipAir.get(side).addZ(0.8f);
                     builder.addSphere(0.03, goalHipAir.get(side), Color.WHITE);
                     builder.addLine(stanceHipAir.get(side), goalHipAir.get(side), 0.01, Color.SKY);
                  }
                  builder.addLine(stanceHipAir.get(planner.getFootToSwing()), planner.getSwingEnd().getPosition(), 0.01, Color.WHITE);

                  float r = 0.5294118f;
                  float g = 0.80784315f;
                  float b = 0.92156863f;
                  if (planner.getFootToSwing() == RobotSide.LEFT)
                  {
                     r = 0.6f;
                     g = 0.2f;
                     b = 0.3f;
                  }
                  else
                  {
                     r = 0.2f;
                     g = 0.6f;
                     b = 0.3f;
                  }
                  Color color = new Color(r, g, b, 1.0f);
                  builder.addMultiLine(planner.getSwingEnd(), foothold.getPolygonVerticesView(), 0.01, color, true);
                  builder.addPolygon(planner.getSwingEnd(), foothold, color);
               }));

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

            footstepPlan = planner.plan(stances, goals);

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
