package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotModels.FullHumanoidRobotModel;
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
   private final FullHumanoidRobotModel fullRobotModel;
   private final ROS2ControllerHelper ros2Helper;

   private final ArrayList<GDXRobotCollisionLink> selfCollisionLinks = new ArrayList<>();
   private final ArrayList<GDXRobotCollisionLink> environmentCollisionLinks = new ArrayList<>();

   private final ImGuiPanel panel = new ImGuiPanel("Whole Body Interactable", this::renderImGuiWidgets);
   private final ImBoolean showSelfCollisionMeshes = new ImBoolean();
   private final ImBoolean showEnvironmentCollisionMeshes = new ImBoolean();

   private final SideDependentList<GDXInteractableFoot> footInteractables = new SideDependentList<>();

   public GDXRobotWholeBodyInteractable(RobotCollisionModel robotSelfCollisionModel,
                                        RobotCollisionModel robotEnvironmentCollisionModel,
                                        FullHumanoidRobotModel fullRobotModel,
                                        ROS2ControllerHelper ros2Helper)
   {
      this.robotSelfCollisionModel = robotSelfCollisionModel;
      this.robotEnvironmentCollisionModel = robotEnvironmentCollisionModel;
      this.fullRobotModel = fullRobotModel;
      this.ros2Helper = ros2Helper;
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      List<Collidable> robotCollidables;
      robotCollidables = robotSelfCollisionModel.getRobotCollidables(fullRobotModel.getElevator());
      AppearanceDefinition green = YoAppearance.DarkGreen();
      green.setTransparency(0.4);
      for (Collidable collidable : robotCollidables)
      {
         GDXRobotCollisionLink collisionLink = new GDXRobotCollisionLink(collidable, GDXTools.toGDX(green));
         selfCollisionLinks.add(collisionLink);
      }
      robotCollidables = robotEnvironmentCollisionModel.getRobotCollidables(fullRobotModel.getElevator());
      AppearanceDefinition red = YoAppearance.DarkRed();
      red.setTransparency(0.4);
      for (Collidable collidable : robotCollidables)
      {
         GDXRobotCollisionLink collisionLink = new GDXRobotCollisionLink(collidable, GDXTools.toGDX(red));
         environmentCollisionLinks.add(collisionLink);

         for (RobotSide side : RobotSide.values)
         {
            if (collidable.getRigidBody().getName().equals(side.getSideNameFirstLowerCaseLetter() + "_foot"))
            {
               GDXInteractableFoot interactableFoot = new GDXInteractableFoot(collisionLink,
                                                                              side,
                                                                              fullRobotModel.getFrameAfterLegJoint(side, LegJointName.ANKLE_ROLL),
                                                                              ros2Helper);
               interactableFoot.create(baseUI.get3DSceneManager().getCamera3D());
               footInteractables.put(side, interactableFoot);
            }

         }
      }
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

      for (GDXInteractableFoot footInteractable : footInteractables)
      {
         footInteractable.process3DViewInput(input);
      }
   }

   private void renderImGuiWidgets()
   {
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

      for (GDXInteractableFoot footInteractable : footInteractables)
      {
         footInteractable.getVirtualRenderables(renderables, pool);
      }
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }
}
