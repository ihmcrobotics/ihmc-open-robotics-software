package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.yo.YoVariableClientHelper;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.graphics.GDXSpatialVectorArrows;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;

import java.util.ArrayList;
import java.util.List;

public class GDXRobotWholeBodyInteractable implements RenderableProvider
{
   private final RobotCollisionModel robotSelfCollisionModel;
   private final RobotCollisionModel robotEnvironmentCollisionModel;
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2Helper;
   private final YoVariableClientHelper yoVariableClientHelper;

   private final ArrayList<GDXRobotCollisionLink> selfCollisionLinks = new ArrayList<>();
   private final ArrayList<GDXRobotCollisionLink> environmentCollisionLinks = new ArrayList<>();

   private final ImGuiPanel panel = new ImGuiPanel("Whole Body Interactable", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean showSelfCollisionMeshes = new ImBoolean();
   private final ImBoolean showEnvironmentCollisionMeshes = new ImBoolean();
   private final ImBoolean interactablesEnabled = new ImBoolean(false);
   private final ImFloat trajectoryTime = new ImFloat(1.2f);

   private final SideDependentList<GDXLiveRobotPartInteractable> footInteractables = new SideDependentList<>();
   private final SideDependentList<GDXLiveRobotPartInteractable> handInteractables = new SideDependentList<>();
   private final SideDependentList<GDXSpatialVectorArrows> wristWrenchArrows = new SideDependentList<>();
   private GDXLiveRobotPartInteractable pelvisInteractable;
   private final GDXWalkPathControlRing walkPathControlRing = new GDXWalkPathControlRing();

   public GDXRobotWholeBodyInteractable(RobotCollisionModel robotSelfCollisionModel,
                                        RobotCollisionModel robotEnvironmentCollisionModel,
                                        DRCRobotModel robotModel,
                                        ROS2SyncedRobotModel syncedRobot,
                                        ROS2ControllerHelper ros2Helper,
                                        YoVariableClientHelper yoVariableClientHelper)
   {
      this.robotSelfCollisionModel = robotSelfCollisionModel;
      this.robotEnvironmentCollisionModel = robotEnvironmentCollisionModel;
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.ros2Helper = ros2Helper;
      this.yoVariableClientHelper = yoVariableClientHelper;
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
         String modelFileName = GDXInteractableTools.getModelFileName(robotDefinition.getRigidBodyDefinition(collidable.getRigidBody().getName()));

         if (collidable.getRigidBody().getName().equals(syncedRobot.getFullRobotModel().getPelvis().getName()))
         {
            pelvisInteractable = new GDXLiveRobotPartInteractable();
            pelvisInteractable.create(collisionLink,
                                      syncedRobot.getReferenceFrames().getPelvisFrame(),
                                      modelFileName,
                                      baseUI.get3DSceneManager().getCamera3D());
            pelvisInteractable.setOnSpacePressed(() ->
            {
               ros2Helper.publishToController(HumanoidMessageTools.createPelvisTrajectoryMessage(trajectoryTime.get(), pelvisInteractable.getPose()));
            });
         }
         for (RobotSide side : RobotSide.values)
         {
            String robotSidePrefix = (side == RobotSide.LEFT) ? "l_" : "r_";
            String footName = syncedRobot.getFullRobotModel().getFoot(side).getName();
            if (collidable.getRigidBody().getName().equals(footName))
            {
               GDXLiveRobotPartInteractable interactableFoot = new GDXLiveRobotPartInteractable();
//               String modelFileName = robotSidePrefix + "foot.g3dj";
               interactableFoot.create(collisionLink,
                                       syncedRobot.getFullRobotModel().getFrameAfterLegJoint(side, LegJointName.ANKLE_ROLL),
                                       modelFileName,
                                       baseUI.get3DSceneManager().getCamera3D());
               interactableFoot.setOnSpacePressed(() ->
               {
                  ros2Helper.publishToController(HumanoidMessageTools.createFootTrajectoryMessage(side, trajectoryTime.get(), interactableFoot.getPose()));
               });
               footInteractables.put(side, interactableFoot);
            }
            if (collidable.getRigidBody().getName().equals(syncedRobot.getFullRobotModel().getHand(side).getName()))
            {
               ReferenceFrame handFrame = syncedRobot.getFullRobotModel().getEndEffectorFrame(side, LimbName.ARM);
               ReferenceFrame collisionFrame = handFrame;
               GDXLiveRobotPartInteractable interactableHand = new GDXLiveRobotPartInteractable();
               ReferenceFrame handControlFrame = syncedRobot.getFullRobotModel().getHandControlFrame(side);
               ReferenceFrame handGraphicFrame
                     = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(robotSidePrefix + "graphicFrame",
                                                                                         handFrame,
                                                                                         robotModel.getUIParameters().getHandGraphicToHandFrameTransform(side));
               interactableHand.create(collisionLink,
                                       handGraphicFrame,
                                       collisionFrame,
                                       handControlFrame,
                                       modelFileName,
                                       baseUI.get3DSceneManager().getCamera3D());
               interactableHand.setOnSpacePressed(() ->
               {
                  ros2Helper.publishToController(HumanoidMessageTools.createHandTrajectoryMessage(side,
                                                                                                  trajectoryTime.get(),
                                                                                                  interactableHand.getPose(),
                                                                                                  ReferenceFrame.getWorldFrame()));
               });
               handInteractables.put(side, interactableHand);
               HumanoidRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
               SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();
               ForceSensorDefinition[] forceSensorDefinitions = syncedRobot.getFullRobotModel().getForceSensorDefinitions();
               for (int i = 0; i < forceSensorDefinitions.length; i++)
               {
                  if (wristForceSensorNames.containsKey(side) && wristForceSensorNames.get(side).equals(forceSensorDefinitions[i].getSensorName()))
                  {
//                     wristWrenchArrows.put(side, new GDXSpatialVectorArrows(forceSensorDefinitions[i].getSensorFrame(), i));
                     wristWrenchArrows.put(side, new GDXSpatialVectorArrows(forceSensorDefinitions[i].getSensorFrame(),
                                                                            yoVariableClientHelper,
                                                                            side.getLowerCaseName() + "WristSensor"));
                  }
               }
            }
         }
      }

      walkPathControlRing.create(baseUI.get3DSceneManager().getCamera3D(), robotModel, syncedRobot, ros2Helper);
   }

   public void update()
   {
      if (interactablesEnabled.get())
      {
         for (GDXRobotCollisionLink collisionLink : selfCollisionLinks)
         {
            collisionLink.update();
         }
         for (GDXRobotCollisionLink collisionLink : environmentCollisionLinks)
         {
            collisionLink.update();
         }

         walkPathControlRing.update();

         for (RobotSide side : wristWrenchArrows.sides())
         {
//            GDXSpatialVectorArrows wristArrows = wristWrenchArrows.get(side);
//            if (syncedRobot.getForceSensorData().size() > wristArrows.getIndexOfSensor())
//            {
//               SpatialVectorMessage forceSensorData = syncedRobot.getForceSensorData().get(wristArrows.getIndexOfSensor());
//               wristArrows.update(forceSensorData.getLinearPart(), forceSensorData.getAngularPart());
//            }
            wristWrenchArrows.get(side).updateFromYoVariables();
         }
      }
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (interactablesEnabled.get())
      {
         walkPathControlRing.calculate3DViewPick(input);
//         if (showSelfCollisionMeshes.get())
//         {
//            for (GDXRobotCollisionLink collisionLink : selfCollisionLinks)
//            {
//               collisionLink.calculatePick(input);
//            }
//         }
//         if (showEnvironmentCollisionMeshes.get())
//         {
            for (GDXRobotCollisionLink collisionLink : environmentCollisionLinks)
            {
               collisionLink.calculatePick(input);
            }
//         }

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
//         if (showSelfCollisionMeshes.get())
//         {
//            for (GDXRobotCollisionLink collisionLink : selfCollisionLinks)
//            {
//               collisionLink.process3DViewInput(input);
//            }
//         }
//         if (showEnvironmentCollisionMeshes.get())
//         {
            for (GDXRobotCollisionLink collisionLink : environmentCollisionLinks)
            {
               collisionLink.process3DViewInput(input);
            }
//         }

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
      ImGui.text("Trajectory time:");
      ImGui.sameLine();
      ImGui.pushItemWidth(100.0f);
      if (ImGui.inputFloat(labels.get("s", "Trajectory time"), trajectoryTime, 0.1f))
      {
         trajectoryTime.set((float) MathTools.clamp(trajectoryTime.get(), 0.0, 30.0));
      }
      ImGui.popItemWidth();
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
      if (interactablesEnabled.get())
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

         for (RobotSide side : wristWrenchArrows.sides())
         {
            GDXSpatialVectorArrows wristArrows = wristWrenchArrows.get(side);
            if (syncedRobot.getForceSensorData().size() > wristArrows.getIndexOfSensor())
            {
               wristArrows.getRenderables(renderables, pool);
            }
         }
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
