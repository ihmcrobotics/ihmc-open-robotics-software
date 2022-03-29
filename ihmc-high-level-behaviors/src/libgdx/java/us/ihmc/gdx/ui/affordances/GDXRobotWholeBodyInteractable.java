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
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

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
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean showSelfCollisionMeshes = new ImBoolean();
   private final ImBoolean showEnvironmentCollisionMeshes = new ImBoolean();
   private final ImBoolean interactablesEnabled = new ImBoolean(false);

   private final SideDependentList<GDXLiveRobotPartInteractable> footInteractables = new SideDependentList<>();
   private final SideDependentList<GDXLiveRobotPartInteractable> handInteractables = new SideDependentList<>();
   private GDXLiveRobotPartInteractable pelvisInteractable;
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

         RobotDefinition robotDefinition = robotModel.getRobotDefinition();
         ModelFileGeometryDefinition modelFileGeometryDefinition = null;
         for (VisualDefinition visualDefinition : robotDefinition.getRigidBodyDefinition(collidable.getRigidBody().getName()).getVisualDefinitions())
         {
            if (visualDefinition.getGeometryDefinition() instanceof ModelFileGeometryDefinition)
            {
               modelFileGeometryDefinition = (ModelFileGeometryDefinition) visualDefinition.getGeometryDefinition();
            }
         }
         if (modelFileGeometryDefinition == null)
         {
            LogTools.error("Interactables need a model file or implementation of shape visuals");
         }

         if (collidable.getRigidBody().getName().equals("pelvis"))
         {
            pelvisInteractable = new GDXLiveRobotPartInteractable();
            pelvisInteractable.create(collisionLink,
                                      syncedRobot.getReferenceFrames().getPelvisFrame(),
                                      modelFileGeometryDefinition.getFileName(),
                                      baseUI.get3DSceneManager().getCamera3D());
            pelvisInteractable.setOnSpacePressed(() ->
            {
               ros2Helper.publishToController(HumanoidMessageTools.createPelvisTrajectoryMessage(1.2, pelvisInteractable.getPose()));
            });
         }
         for (RobotSide side : RobotSide.values)
         {
            String robotSidePrefix = (side == RobotSide.LEFT) ? "l_" : "r_";
            if (collidable.getRigidBody().getName().equals(robotSidePrefix + "foot"))
            {
               GDXLiveRobotPartInteractable interactableFoot = new GDXLiveRobotPartInteractable();
//               String modelFileName = robotSidePrefix + "foot.g3dj";
               interactableFoot.create(collisionLink,
                                       syncedRobot.getFullRobotModel().getFrameAfterLegJoint(side, LegJointName.ANKLE_ROLL),
                                       modelFileGeometryDefinition.getFileName(),
                                       baseUI.get3DSceneManager().getCamera3D());
               interactableFoot.setOnSpacePressed(() ->
               {
                  ros2Helper.publishToController(HumanoidMessageTools.createFootTrajectoryMessage(side, 1.2, interactableFoot.getPose()));
               });
               footInteractables.put(side, interactableFoot);
            }
            if (collidable.getRigidBody().getName().equals(robotSidePrefix + "hand"))
            {
               ReferenceFrame handFrame = syncedRobot.getFullRobotModel().getEndEffectorFrame(side, LimbName.ARM);
               ReferenceFrame collisionFrame = handFrame;
               GDXLiveRobotPartInteractable interactableHand = new GDXLiveRobotPartInteractable();
               ReferenceFrame handControlFrame = syncedRobot.getFullRobotModel().getHandControlFrame(side);
               RigidBodyTransform handGraphicToHandTransform = new RigidBodyTransform();
               handGraphicToHandTransform.getRotation().setYawPitchRoll(side == RobotSide.LEFT ? 0.0 : Math.PI, -Math.PI / 2.0, 0.0);
               // 0.168 from models/GFE/atlas_unplugged_v5_dual_robotiq_with_head.urdf
               // 0.126 from debugger on GDXGraphicsObject
               // Where does the 0.042 come from?
               handGraphicToHandTransform.getTranslation().set(-0.00179, side.negateIfRightSide(0.126), 0.0);
               ReferenceFrame handGraphicFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(robotSidePrefix + "graphicFrame",
                                                                                                                   handFrame,
                                                                                                                   handGraphicToHandTransform);
               interactableHand.create(collisionLink,
                                       handGraphicFrame,
                                       collisionFrame,
                                       handControlFrame,
                                       modelFileGeometryDefinition.getFileName(),
                                       baseUI.get3DSceneManager().getCamera3D());
               interactableHand.setOnSpacePressed(() ->
               {
                  ros2Helper.publishToController(HumanoidMessageTools.createHandTrajectoryMessage(side,
                                                                                                  1.2,
                                                                                                  interactableHand.getPose(),
                                                                                                  ReferenceFrame.getWorldFrame()));
               });
               handInteractables.put(side, interactableHand);
            }
         }
      }

      walkPathControlRing.create(baseUI.get3DSceneManager().getCamera3D(), robotModel, syncedRobot, ros2Helper);
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
      walkPathControlRing.process3DViewInput(input);
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
         for (GDXLiveRobotPartInteractable footInteractable : footInteractables)
         {
            footInteractable.process3DViewInput(input);
         }
         for (GDXLiveRobotPartInteractable handInteractable : handInteractables)
         {
            if (handInteractable != null)
               handInteractable.process3DViewInput(input);
         }
      }
   }

   private void renderImGuiWidgets()
   {
      ImGui.checkbox("Interactables enabled", interactablesEnabled);
      ImGui.sameLine();
      if (ImGui.button(labels.get("Clear graphics")))
         walkPathControlRing.clearGraphics();
      ImGui.text("Walk path control ring:");
      walkPathControlRing.renderImGuiWidgets();
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
         for (GDXLiveRobotPartInteractable footInteractable : footInteractables)
         {
            footInteractable.getVirtualRenderables(renderables, pool);
         }
         for (GDXLiveRobotPartInteractable handInteractable : handInteractables)
         {
            if (handInteractable != null)
               handInteractable.getVirtualRenderables(renderables, pool);
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
}
