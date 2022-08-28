package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.yo.YoVariableClientHelper;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.collidables.GDXRobotCollisionModel;
import us.ihmc.gdx.ui.teleoperation.GDXDesiredRobot;
import us.ihmc.gdx.ui.teleoperation.GDXTeleoperationParameters;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.tools.gui.YoAppearanceTools;

public class GDXWholeBodyInteractable implements RenderableProvider
{
   private final GDXRobotCollisionModel selfCollisionModel;
   private final GDXRobotCollisionModel environmentCollisionModel;
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2Helper;
   private final GDXWholeBodyDesiredIKManager armSetpointManager;
   private final YoVariableClientHelper yoVariableClientHelper;
   private GDXTeleoperationParameters teleoperationParameters;

   private final ImGuiPanel panel = new ImGuiPanel("Whole Body Interactable", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean showSelfCollisionMeshes = new ImBoolean();
   private final ImBoolean showEnvironmentCollisionMeshes = new ImBoolean();
   private final ImBoolean interactablesEnabled = new ImBoolean(false);

   private final SideDependentList<GDXLiveRobotPartInteractable> footInteractables = new SideDependentList<>();
   private final SideDependentList<GDXHandInteractable> handInteractables = new SideDependentList<>();
   private GDXLiveRobotPartInteractable pelvisInteractable;
   private final GDXWalkPathControlRing walkPathControlRing = new GDXWalkPathControlRing();

   public GDXWholeBodyInteractable(RobotCollisionModel robotSelfCollisionModel,
                                   RobotCollisionModel robotEnvironmentCollisionModel,
                                   DRCRobotModel robotModel,
                                   ROS2SyncedRobotModel syncedRobot,
                                   GDXDesiredRobot desiredRobot,
                                   ROS2ControllerHelper ros2Helper,
                                   YoVariableClientHelper yoVariableClientHelper,
                                   GDXTeleoperationParameters teleoperationParameters)
   {
      selfCollisionModel = new GDXRobotCollisionModel(robotSelfCollisionModel);
      environmentCollisionModel = new GDXRobotCollisionModel(robotEnvironmentCollisionModel);
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.ros2Helper = ros2Helper;
      this.yoVariableClientHelper = yoVariableClientHelper;
      this.teleoperationParameters = teleoperationParameters;
      this.armSetpointManager = new GDXWholeBodyDesiredIKManager(robotModel,
                                                                 syncedRobot,
                                                                 desiredRobot.getDesiredFullRobotModel(),
                                                                 ros2Helper,
                                                                 teleoperationParameters);
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      selfCollisionModel.create(syncedRobot, YoAppearanceTools.makeTransparent(YoAppearance.DarkGreen(), 0.4));
      environmentCollisionModel.create(syncedRobot, YoAppearanceTools.makeTransparent(YoAppearance.DarkRed(), 0.4));

      // create the manager for the desired arm setpoints
      armSetpointManager.create(baseUI);

      for (GDXRobotCollisionLink collisionLink : environmentCollisionModel.getCollisionLinks())
      {
         RobotDefinition robotDefinition = robotModel.getRobotDefinition();
         FullHumanoidRobotModel fullRobotModel = syncedRobot.getFullRobotModel();
         String modelFileName = GDXInteractableTools.getModelFileName(robotDefinition.getRigidBodyDefinition(collisionLink.getRigidBodyName()));

         if (collisionLink.getRigidBodyName().equals(fullRobotModel.getPelvis().getName()))
         {
            if (pelvisInteractable == null)
            {
               pelvisInteractable = new GDXLiveRobotPartInteractable();
               pelvisInteractable.create(collisionLink,
                                         syncedRobot.getReferenceFrames().getPelvisFrame(),
                                         modelFileName,
                                         baseUI.getPrimary3DPanel());
               pelvisInteractable.setOnSpacePressed(() ->
               {
                  ros2Helper.publishToController(HumanoidMessageTools.createPelvisTrajectoryMessage(teleoperationParameters.getTrajectoryTime(),
                                                                                                    pelvisInteractable.getPose()));
               });
            }
            else
            {
               pelvisInteractable.addAdditionalCollisionLink(collisionLink);
            }
         }
         for (RobotSide side : RobotSide.values)
         {
            String robotSidePrefix = (side == RobotSide.LEFT) ? "l_" : "r_";
            String footName = fullRobotModel.getFoot(side).getName();
            if (collisionLink.getRigidBodyName().equals(footName))
            {
               if (!footInteractables.containsKey(side))
               {
                  GDXLiveRobotPartInteractable interactableFoot = new GDXLiveRobotPartInteractable();
   //               String modelFileName = robotSidePrefix + "foot.g3dj";
                  interactableFoot.create(collisionLink,
                                          fullRobotModel.getFrameAfterLegJoint(side, LegJointName.ANKLE_ROLL),
                                          modelFileName,
                                          baseUI.getPrimary3DPanel());
                  interactableFoot.setOnSpacePressed(() ->
                  {
                     ros2Helper.publishToController(HumanoidMessageTools.createFootTrajectoryMessage(side,
                                                                                                     teleoperationParameters.getTrajectoryTime(),
                                                                                                     interactableFoot.getPose()));
                  });
                  footInteractables.put(side, interactableFoot);
               }
               else
               {
                  footInteractables.get(side).addAdditionalCollisionLink(collisionLink);
               }
            }
            if (GDXHandInteractable.collisionLinkIsHand(side, collisionLink, fullRobotModel))
            {
               if (!handInteractables.containsKey(side))
               {
                  GDXHandInteractable handInteractable = new GDXHandInteractable(side, baseUI, collisionLink, robotModel, syncedRobot, yoVariableClientHelper);
                  handInteractables.put(side, handInteractable);
                  // TODO this should probably not handle the space event!
                  // This sends a command to the controller.
                  handInteractable.setOnSpacePressed(armSetpointManager.getSubmitDesiredArmSetpointsCallback(side));
               }
               else
               {
                  handInteractables.get(side).addAdditionalCollisionLink(collisionLink);
               }
            }
         }
      }

      walkPathControlRing.create(baseUI.getPrimary3DPanel(), robotModel, syncedRobot, teleoperationParameters);
   }

   public void update(GDXInteractableFootstepPlan plannedFootstepPlacement)
   {
      // update the desired arm setpoints
      armSetpointManager.update(handInteractables);

      if (interactablesEnabled.get())
      {
         selfCollisionModel.update();
         environmentCollisionModel.update();

         walkPathControlRing.update(plannedFootstepPlacement);

         for (RobotSide side : handInteractables.sides())
         {
            handInteractables.get(side).update();
         }
      }
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (interactablesEnabled.get())
      {
         walkPathControlRing.calculate3DViewPick(input);

         if (input.isWindowHovered())
            environmentCollisionModel.calculate3DViewPick(input);

         pelvisInteractable.calculate3DViewPick(input);
         for (GDXLiveRobotPartInteractable footInteractable : footInteractables)
         {
            footInteractable.calculate3DViewPick(input);
         }
         for (GDXLiveRobotPartInteractable handInteractable : handInteractables)
         {
            if (handInteractable != null)
               handInteractable.calculate3DViewPick(input);
         }
      }
   }

   // This happens after update.
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (interactablesEnabled.get())
      {
         walkPathControlRing.process3DViewInput(input);
         environmentCollisionModel.process3DViewInput(input);

         pelvisInteractable.process3DViewInput(input);
         for (RobotSide side : footInteractables.sides())
         {
            footInteractables.get(side).process3DViewInput(input);
         }
         for (RobotSide side : handInteractables.sides())
         {
            handInteractables.get(side).process3DViewInput(input);
         }
      }
   }

   public void renderImGuiWidgets()
   {
      armSetpointManager.renderImGuiWidgets();
      ImGui.checkbox("Interactables enabled", interactablesEnabled);
      ImGui.sameLine();
      if (ImGui.button(labels.get("Delete all")))
      {
         walkPathControlRing.delete();
         pelvisInteractable.delete();
         for (RobotSide side : footInteractables.sides())
            footInteractables.get(side).delete();
         for (RobotSide side : handInteractables.sides())
            handInteractables.get(side).delete();
      }

      ImGui.text("Pelvis:");
      ImGui.sameLine();
      pelvisInteractable.renderImGuiWidgets();

      for (RobotSide side : handInteractables.sides())
      {
         ImGui.text(side.getPascalCaseName() + " hand:");
         ImGui.sameLine();
         handInteractables.get(side).renderImGuiWidgets();
      }
      for (RobotSide side : footInteractables.sides())
      {
         ImGui.text(side.getPascalCaseName() + " foot:");
         ImGui.sameLine();
         footInteractables.get(side).renderImGuiWidgets();
      }

      ImGui.separator();

      ImGui.text("Show collisions:");
      ImGui.sameLine();
      ImGui.checkbox("Contact", showEnvironmentCollisionMeshes);
      ImGui.sameLine();
      ImGui.checkbox("Avoidance", showSelfCollisionMeshes);

      // TODO: Add transparency sliders
      // TODO: Add context menus
      // TODO: Add ghost robot
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (interactablesEnabled.get())
      {
         if (showSelfCollisionMeshes.get())
         {
            selfCollisionModel.getRenderables(renderables, pool);
         }
         if (showEnvironmentCollisionMeshes.get())
         {
            environmentCollisionModel.getRenderables(renderables, pool);
         }

         pelvisInteractable.getVirtualRenderables(renderables, pool);
         for (RobotSide side : footInteractables.sides())
         {
            footInteractables.get(side).getVirtualRenderables(renderables, pool);
         }
         for (RobotSide side : handInteractables.sides())
         {
            handInteractables.get(side).getVirtualRenderables(renderables, pool);
         }

         walkPathControlRing.getVirtualRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      walkPathControlRing.destroy();
   }

   public void setInteractablesEnabled(boolean enabled)
   {
      interactablesEnabled.set(enabled);
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }

   public void setDesiredToCurrent()
   {
      armSetpointManager.setDesiredToCurrent();
   }

   public GDXWalkPathControlRing getWalkPathControlRing()
   {
      return walkPathControlRing;
   }
}
