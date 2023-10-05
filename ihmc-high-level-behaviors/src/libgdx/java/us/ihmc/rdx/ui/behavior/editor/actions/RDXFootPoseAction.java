package us.ihmc.rdx.ui.behavior.editor.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sequence.actions.FootPoseActionDefinition;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
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
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.HandTransformTools;

import java.util.ArrayList;
import java.util.List;

public class RDXFootPoseAction extends RDXBehaviorAction
{
   private final FootPoseActionDefinition actionDefinition = new FootPoseActionDefinition();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper trajectoryDurationWidget = new ImDoubleWrapper(actionDefinition::getTrajectoryDuration,
                                                                                actionDefinition::setTrajectoryDuration,
                                                                                imDouble -> ImGuiTools.volatileInputDouble(labels.get("Trajectory duration"),
                                                                                                                           imDouble));
   private final SideDependentList<String> footNames = new SideDependentList<>();
   /** Gizmo is control frame */
   private final RDXSelectablePose3DGizmo poseGizmo = new RDXSelectablePose3DGizmo(actionDefinition.getFootFrame(), actionDefinition.getTransformToParent());
   private final ImBooleanWrapper selectedWrapper = new ImBooleanWrapper(() -> poseGizmo.getSelected().get(),
                                                                         value -> poseGizmo.getSelected().set(value),
                                                                         imBoolean -> ImGui.checkbox(labels.get("Selected"), imBoolean));
   private final ImBooleanWrapper executeWithNextActionWrapper = new ImBooleanWrapper(actionDefinition::getExecuteWithNextAction,
                                                                                      actionDefinition::setExecuteWithNextAction,
                                                                                      imBoolean -> ImGui.checkbox(labels.get("Execute with next action"), imBoolean));
   private final ImBooleanWrapper holdPoseInWorldLaterWrapper = new ImBooleanWrapper(actionDefinition::getHoldPoseInWorldLater,
                                                                                     actionDefinition::setHoldPoseInWorldLater,
                                                                                     imBoolean -> ImGui.checkbox(labels.get("Hold pose in world later"), imBoolean));
   private final ModifiableReferenceFrame graphicFrame = new ModifiableReferenceFrame(actionDefinition.getFootFrame());
   private final ModifiableReferenceFrame collisionShapeFrame = new ModifiableReferenceFrame(actionDefinition.getFootFrame());
   private boolean isMouseHovering = false;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private final ArrayList<MouseCollidable> mouseCollidables = new ArrayList<>();
   private final SideDependentList<RDXInteractableHighlightModel> highlightModels = new SideDependentList<>();
   private final ImGuiReferenceFrameLibraryCombo referenceFrameLibraryCombo;
   private final RDX3DPanelTooltip tooltip;
   private final ROS2PublishSubscribeAPI ros2;

   public RDXFootPoseAction(RDX3DPanel panel3D,
                                    DRCRobotModel robotModel,
                                    FullHumanoidRobotModel syncedFullRobotModel,
                                    RobotCollisionModel selectionCollisionModel,
                                    ReferenceFrameLibrary referenceFrameLibrary,
                                    ROS2PublishSubscribeAPI ros2)
   {
      this.ros2 = ros2;
      actionDefinition.setReferenceFrameLibrary(referenceFrameLibrary);

      for (RobotSide side : RobotSide.values)
      {
         footNames.put(side, syncedFullRobotModel.getFoot(side).getName());

         RigidBodyTransformReadOnly graphicToControlFrameTransform = HandTransformTools.getHandGraphicToControlFrameTransform(syncedFullRobotModel,
                                                                                                                              robotModel.getUIParameters(),
                                                                                                                              side);
         graphicFrame.update(transformToParent -> transformToParent.set(graphicToControlFrameTransform));

         String footBodyName = footNames.get(side);
         String modelFileName = RDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(footBodyName));
         highlightModels.put(side, new RDXInteractableHighlightModel(modelFileName));

         MultiBodySystemBasics footOnlySystem = MultiBodySystemMissingTools.createSingleBodySystem(syncedFullRobotModel.getFoot(side));
         List<Collidable> footCollidables = selectionCollisionModel.getRobotCollidables(footOnlySystem);

         RigidBodyTransformReadOnly linkToControlFrameTransform = HandTransformTools.getHandLinkToControlFrameTransform(syncedFullRobotModel, side);
         collisionShapeFrame.update(transformToParent -> transformToParent.set(linkToControlFrameTransform));

         for (Collidable footCollidable : footCollidables)
         {
            mouseCollidables.add(new MouseCollidable(footCollidable));
         }
      }

      referenceFrameLibraryCombo = new ImGuiReferenceFrameLibraryCombo(referenceFrameLibrary);
      poseGizmo.create(panel3D);

      tooltip = new RDX3DPanelTooltip(panel3D);
      panel3D.addImGuiOverlayAddition(this::render3DPanelImGuiOverlays);
   }

   @Override
   public void updateAfterLoading()
   {
      referenceFrameLibraryCombo.setSelectedReferenceFrame(actionDefinition.getParentFrame().getName());
   }

   public void setSide(RobotSide side)
   {
      actionDefinition.setSide(side);
   }

   public void setIncludingFrame(ReferenceFrame parentFrame, RigidBodyTransform transformToParent)
   {
      actionDefinition.changeParentFrame(parentFrame);
      actionDefinition.setTransformToParent(transformToParentToPack -> transformToParentToPack.set(transformToParent));
      update();
   }

   public void setToReferenceFrame(ReferenceFrame referenceFrame)
   {
      actionDefinition.changeParentFrame(ReferenceFrame.getWorldFrame());
      actionDefinition.setTransformToParent(transformToParentToPack -> transformToParentToPack.set(referenceFrame.getTransformToWorldFrame()));
      update();
   }

   @Override
   public void update(boolean concurrentActionIsNextForExecution)
   {
      actionDefinition.update();

      if (poseGizmo.getPoseGizmo().getGizmoFrame() != actionDefinition.getFootFrame())
      {
         poseGizmo.getPoseGizmo().setGizmoFrame(actionDefinition.getFootFrame());
         graphicFrame.changeParentFrame(actionDefinition.getFootFrame());
         collisionShapeFrame.changeParentFrame(actionDefinition.getFootFrame());
      }

      poseGizmo.getPoseGizmo().update();
      highlightModels.get(actionDefinition.getSide()).setPose(graphicFrame.getReferenceFrame());

      if (poseGizmo.isSelected() || isMouseHovering)
      {
         highlightModels.get(actionDefinition.getSide()).setTransparency(0.7);
      }
      else
      {
         highlightModels.get(actionDefinition.getSide()).setTransparency(0.5);
      }
   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      ImGui.sameLine();
      executeWithNextActionWrapper.renderImGuiWidget();
      holdPoseInWorldLaterWrapper.renderImGuiWidget();
      if (referenceFrameLibraryCombo.render())
      {
         actionDefinition.changeParentFrameWithoutMoving(referenceFrameLibraryCombo.getSelectedReferenceFrame());
         update();
      }
      ImGui.pushItemWidth(80.0f);
      trajectoryDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   public void render3DPanelImGuiOverlays()
   {
      if (isMouseHovering)
      {
         tooltip.render("%s Action\nIndex: %d\nDescription: %s".formatted(getActionTypeTitle(),
                                                                          getActionIndex(),
                                                                          actionDefinition.getDescription()));
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
      highlightModels.get(actionDefinition.getSide()).getRenderables(renderables, pool);
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
   public FootPoseActionDefinition getActionDefinition()
   {
      return actionDefinition;
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Chest Orientation";
   }

   @Override
   public boolean getExecuteWithNextAction()
   {
      return executeWithNextActionWrapper.get();
   }
}
