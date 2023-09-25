package us.ihmc.rdx.ui.behavior.editor.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sequence.actions.ChestOrientationActionData;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.affordances.RDXInteractableHighlightModel;
import us.ihmc.rdx.ui.affordances.RDXInteractableTools;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.interaction.MouseCollidable;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

import java.util.ArrayList;
import java.util.List;

public class RDXChestOrientationAction extends RDXBehaviorAction
{
   private final ChestOrientationActionData actionData = new ChestOrientationActionData();
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper yawWidget = new ImDoubleWrapper(() -> actionData.getRotation().getYaw(), actionData::setYaw,
                                                                 imDouble -> ImGuiTools.volatileInputDouble(labels.get("Yaw"), imDouble));
   private final ImDoubleWrapper pitchWidget = new ImDoubleWrapper(() -> actionData.getRotation().getPitch(), actionData::setPitch,
                                                                   imDouble -> ImGuiTools.volatileInputDouble(labels.get("Pitch"), imDouble));
   private final ImDoubleWrapper rollWidget = new ImDoubleWrapper(() -> actionData.getRotation().getRoll(), actionData::setRoll,
                                                                  imDouble -> ImGuiTools.volatileInputDouble(labels.get("Roll"), imDouble));
   private final ImDoubleWrapper trajectoryDurationWidget = new ImDoubleWrapper(actionData::getTrajectoryDuration,
                                                                                actionData::setTrajectoryDuration,
                                                                                imDouble -> ImGuiTools.volatileInputDouble(labels.get("Trajectory duration"),
                                                                                                                           imDouble));
   /** Gizmo is control frame */
   private final RDXSelectablePose3DGizmo poseGizmo = new RDXSelectablePose3DGizmo(actionData.getConditionalReferenceFrame().get(), actionData.getTransformToParent());
   private final ImBooleanWrapper selectedWrapper = new ImBooleanWrapper(() -> poseGizmo.getSelected().get(),
                                                                         value -> poseGizmo.getSelected().set(value),
                                                                         imBoolean -> ImGui.checkbox(labels.get("Selected"), imBoolean));
   private final ModifiableReferenceFrame graphicFrame = new ModifiableReferenceFrame(actionData.getConditionalReferenceFrame().get());
   private final ModifiableReferenceFrame collisionShapeFrame = new ModifiableReferenceFrame(actionData.getConditionalReferenceFrame().get());
   private boolean isMouseHovering = false;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private final ArrayList<MouseCollidable> mouseCollidables = new ArrayList<>();
   private final RDXInteractableHighlightModel highlightModel;
   private final ImGuiReferenceFrameLibraryCombo referenceFrameLibraryCombo;
   private final RDX3DPanelTooltip tooltip;

   public RDXChestOrientationAction(RDX3DPanel panel3D,
                                    DRCRobotModel robotModel,
                                    FullHumanoidRobotModel syncedFullRobotModel,
                                    RobotCollisionModel selectionCollisionModel,
                                    ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.referenceFrameLibrary = referenceFrameLibrary;

      String chestBodyName = syncedFullRobotModel.getChest().getName();
      String modelFileName = RDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(chestBodyName));
      highlightModel = new RDXInteractableHighlightModel(modelFileName);

      MultiBodySystemBasics chestOnlySystem = MultiBodySystemMissingTools.createSingleBodySystem(syncedFullRobotModel.getChest());
      List<Collidable> chestCollidables = selectionCollisionModel.getRobotCollidables(chestOnlySystem);

      for (Collidable chestCollidable : chestCollidables)
      {
         mouseCollidables.add(new MouseCollidable(chestCollidable));
      }

      referenceFrameLibraryCombo = new ImGuiReferenceFrameLibraryCombo(referenceFrameLibrary);
      poseGizmo.create(panel3D);

      tooltip = new RDX3DPanelTooltip(panel3D);
      panel3D.addImGuiOverlayAddition(this::render3DPanelImGuiOverlays);
   }

   @Override
   public void updateAfterLoading()
   {
      referenceFrameLibraryCombo.setSelectedReferenceFrame(actionData.getConditionalReferenceFrame().getConditionallyValidParentFrameName());
   }

   public void setIncludingFrame(ReferenceFrame parentFrame, RigidBodyTransform transformToParent)
   {
      actionData.getConditionalReferenceFrame().setParentFrameName(parentFrame.getName());
      actionData.setTransformToParent(transformToParent);
      update();
   }

   public void setToReferenceFrame(ReferenceFrame referenceFrame)
   {
      actionData.getConditionalReferenceFrame().setParentFrameName(ReferenceFrame.getWorldFrame().getName());
      actionData.setTransformToParent(referenceFrame.getTransformToWorldFrame());
      update();
   }

   @Override
   public void update()
   {
      actionData.update(referenceFrameLibrary);

      if (poseGizmo.getPoseGizmo().getGizmoFrame() != actionData.getConditionalReferenceFrame().get())
      {
         poseGizmo.getPoseGizmo().setGizmoFrame(actionData.getConditionalReferenceFrame().get());
         graphicFrame.changeParentFrame(actionData.getConditionalReferenceFrame().get());
         collisionShapeFrame.changeParentFrame(actionData.getConditionalReferenceFrame().get());
      }

      poseGizmo.getPoseGizmo().update();
      highlightModel.setPose(graphicFrame.getReferenceFrame());

      if (poseGizmo.isSelected() || isMouseHovering)
      {
         highlightModel.setTransparency(0.7);
      }
      else
      {
         highlightModel.setTransparency(0.5);
      }
   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      if (referenceFrameLibraryCombo.render())
      {
         actionData.getConditionalReferenceFrame().setParentFrameName(referenceFrameLibraryCombo.getSelectedReferenceFrame().get().getName());
         update();
      }
      ImGui.pushItemWidth(80.0f);
      yawWidget.renderImGuiWidget();
      ImGui.sameLine();
      pitchWidget.renderImGuiWidget();
      ImGui.sameLine();
      rollWidget.renderImGuiWidget();
      trajectoryDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   public void render3DPanelImGuiOverlays()
   {
      if (isMouseHovering)
      {
         tooltip.render("%s Action\nIndex: %d\nDescription: %s".formatted(getActionTypeTitle(),
                                                                          getActionIndex(),
                                                                          actionData.getDescription()));
      }
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      poseGizmo.calculate3DViewPick(input);

      pickResult.reset();
      for (MouseCollidable mouseCollidable : mouseCollidables)
      {
         double collision = mouseCollidable.collide(input.getPickRayInWorld(), collisionShapeFrame.getReferenceFrame());
         if (!Double.isNaN(collision))
            pickResult.addPickCollision(collision);
      }
      if (pickResult.getPickCollisionWasAddedSinceReset())
         input.addPickResult(pickResult);
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      isMouseHovering = input.getClosestPick() == pickResult;

      boolean isClickedOn = isMouseHovering && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);
      if (isClickedOn)
      {
         selectedWrapper.set(true);
      }

      poseGizmo.process3DViewInput(input, isMouseHovering);

      tooltip.setInput(input);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      highlightModel.getRenderables(renderables, pool);
      poseGizmo.getVirtualRenderables(renderables, pool);
   }

   @Override
   public ImBooleanWrapper getSelected()
   {
      return selectedWrapper;
   }

   public ReferenceFrame getReferenceFrame()
   {
      return poseGizmo.getPoseGizmo().getGizmoFrame();
   }

   @Override
   public ChestOrientationActionData getActionData()
   {
      return actionData;
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Chest Orientation";
   }
}
