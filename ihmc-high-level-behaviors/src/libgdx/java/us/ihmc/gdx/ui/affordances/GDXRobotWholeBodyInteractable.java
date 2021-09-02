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
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;

public class GDXRobotWholeBodyInteractable implements RenderableProvider
{
   private final RobotCollisionModel robotSelfCollisionModel;
   private final RobotCollisionModel robotEnvironmentCollisionModel;
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2Helper;

   private final ArrayList<GDXRobotCollisionLink> selfCollisionLinks = new ArrayList<>();
   private final ArrayList<GDXRobotCollisionLink> environmentCollisionLinks = new ArrayList<>();

   private final ImGuiPanel panel = new ImGuiPanel("Whole Body Interactable", this::renderImGuiWidgets);
   private final ImBoolean showSelfCollisionMeshes = new ImBoolean();
   private final ImBoolean showEnvironmentCollisionMeshes = new ImBoolean();
   private final ImBoolean interactablesEnabled = new ImBoolean(false);

   private final SideDependentList<GDXInteractableFoot> footInteractables = new SideDependentList<>();
   private GDXInteractablePelvis pelvisInteractable;
   private final GDXWalkPathControlRing walkPathControlRing = new GDXWalkPathControlRing();

   public GDXRobotWholeBodyInteractable(RobotCollisionModel robotSelfCollisionModel,
                                        RobotCollisionModel robotEnvironmentCollisionModel,
                                        DRCRobotModel robotModel,
                                        ROS2SyncedRobotModel syncedRobot,
                                        ROS2ControllerHelper ros2Helper)
   {
      this.robotSelfCollisionModel = robotSelfCollisionModel;
      this.robotEnvironmentCollisionModel = robotEnvironmentCollisionModel;
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.ros2Helper = ros2Helper;
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      List<Collidable> robotCollidables;
      robotCollidables = robotSelfCollisionModel.getRobotCollidables(syncedRobot.getFullRobotModel().getElevator());
      AppearanceDefinition green = YoAppearance.DarkGreen();
      green.setTransparency(0.4);
      for (Collidable collidable : robotCollidables)
      {
         GDXRobotCollisionLink collisionLink = new GDXRobotCollisionLink(collidable, GDXTools.toGDX(green));
         selfCollisionLinks.add(collisionLink);
      }
      robotCollidables = robotEnvironmentCollisionModel.getRobotCollidables(syncedRobot.getFullRobotModel().getElevator());
      AppearanceDefinition red = YoAppearance.DarkRed();
      red.setTransparency(0.4);
      for (Collidable collidable : robotCollidables)
      {
         GDXRobotCollisionLink collisionLink = new GDXRobotCollisionLink(collidable, GDXTools.toGDX(red));
         environmentCollisionLinks.add(collisionLink);

         if (collidable.getRigidBody().getName().equals("pelvis"))
         {
            pelvisInteractable = new GDXInteractablePelvis(collisionLink, syncedRobot.getReferenceFrames().getPelvisFrame(), ros2Helper);
            pelvisInteractable.create(baseUI.get3DSceneManager().getCamera3D());
         }
         for (RobotSide side : RobotSide.values)
         {
            if (collidable.getRigidBody().getName().equals(side.getSideNameFirstLowerCaseLetter() + "_foot"))
            {
               GDXInteractableFoot interactableFoot = new GDXInteractableFoot(collisionLink,
                                                                              side,
                                                                              syncedRobot.getFullRobotModel()
                                                                                         .getFrameAfterLegJoint(side, LegJointName.ANKLE_ROLL),
                                                                              ros2Helper);
               interactableFoot.create(baseUI.get3DSceneManager().getCamera3D());
               footInteractables.put(side, interactableFoot);
            }
         }
      }

      walkPathControlRing.create(baseUI, robotModel, syncedRobot, ros2Helper);
   }

   public void update()
   {
      for (GDXRobotCollisionLink collisionLink : selfCollisionLinks)
      {
         collisionLink.update();
      }
      for (GDXRobotCollisionLink collisionLink : environmentCollisionLinks)
      {
         collisionLink.update();
      }

      if (interactablesEnabled.get())
      {
         walkPathControlRing.update();
      }
   }

   // This happens after update.
   public void process3DViewInput(ImGui3DViewInput input)
   {
      for (GDXRobotCollisionLink collisionLink : selfCollisionLinks)
      {
         collisionLink.process3DViewInput(input);
      }
      for (GDXRobotCollisionLink collisionLink : environmentCollisionLinks)
      {
         collisionLink.process3DViewInput(input);
      }

      if (interactablesEnabled.get())
      {
         pelvisInteractable.process3DViewInput(input);
         for (GDXInteractableFoot footInteractable : footInteractables)
         {
            footInteractable.process3DViewInput(input);
         }
      }
   }

   private void renderImGuiWidgets()
   {
      ImGui.checkbox("Interactables enabled", interactablesEnabled);
      ImGui.checkbox("Show self collision meshes", showSelfCollisionMeshes);
      ImGui.checkbox("Show environment collision meshes", showEnvironmentCollisionMeshes);
      ImGui.text("TODO:");
      ImGui.text("- Add transparency sliders");
      ImGui.text("- Add context menus");
      ImGui.text("- Add ghost robot");
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (showSelfCollisionMeshes.get())
      {
         for (GDXRobotCollisionLink collisionLink : selfCollisionLinks)
         {
            collisionLink.getRenderables(renderables, pool);
         }
      }
      if (showEnvironmentCollisionMeshes.get())
      {
         for (GDXRobotCollisionLink collisionLink : environmentCollisionLinks)
         {
            collisionLink.getRenderables(renderables, pool);
         }
      }

      if (interactablesEnabled.get())
      {
         pelvisInteractable.getVirtualRenderables(renderables, pool);
         for (GDXInteractableFoot footInteractable : footInteractables)
         {
            footInteractable.getVirtualRenderables(renderables, pool);
         }

         walkPathControlRing.getVirtualRenderables(renderables, pool);
      }
   }

   public void setInteractablesEnabled(boolean enabled)
   {
      interactablesEnabled.set(enabled);
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }
}
