package us.ihmc.rdx.ui.behavior.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sequence.actions.ChestOrientationActionDefinition;
import us.ihmc.behaviors.sequence.actions.ChestOrientationActionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.affordances.RDXInteractableHighlightModel;
import us.ihmc.rdx.ui.affordances.RDXInteractableTools;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.interaction.MouseCollidable;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.ArrayList;
import java.util.List;

public class RDXChestOrientationAction extends RDXActionNode<ChestOrientationActionState, ChestOrientationActionDefinition>
{
   private final ChestOrientationActionState state;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean adjustGoalPose = new ImBoolean();
   private final ImBooleanWrapper executeWithNextActionWrapper;
   private final ImBooleanWrapper holdPoseInWorldLaterWrapper;
   private final ImGuiReferenceFrameLibraryCombo parentFrameComboBox;
   private final ImDoubleWrapper yawWidget;
   private final ImDoubleWrapper pitchWidget;
   private final ImDoubleWrapper rollWidget;
   private final ImDoubleWrapper trajectoryDurationWidget;
   /** Gizmo is control frame */
   private final RDXSelectablePose3DGizmo poseGizmo;
   private final MutableReferenceFrame graphicFrame = new MutableReferenceFrame();
   private final MutableReferenceFrame collisionShapeFrame = new MutableReferenceFrame();
   private boolean isMouseHovering = false;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private final ArrayList<MouseCollidable> mouseCollidables = new ArrayList<>();
   private final RDXInteractableHighlightModel highlightModel;
   private final RDX3DPanelTooltip tooltip;
   private boolean wasConcurrent = false;

   public RDXChestOrientationAction(long id,
                                    CRDTInfo crdtInfo,
                                    WorkspaceResourceDirectory saveFileDirectory,
                                    RDX3DPanel panel3D,
                                    DRCRobotModel robotModel,
                                    FullHumanoidRobotModel syncedFullRobotModel,
                                    RobotCollisionModel selectionCollisionModel,
                                    ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(new ChestOrientationActionState(id, crdtInfo,saveFileDirectory, referenceFrameLibrary));

      state = getState();

      getDefinition().setName("Chest orientation");

      poseGizmo = new RDXSelectablePose3DGizmo(ReferenceFrame.getWorldFrame(), getDefinition().getChestToParentTransform().getValue(), adjustGoalPose);
      poseGizmo.create(panel3D);

      // TODO: Can all this be condensed?
      executeWithNextActionWrapper = new ImBooleanWrapper(getDefinition()::getExecuteWithNextAction,
                                                          getDefinition()::setExecuteWithNextAction,
                                                          imBoolean -> ImGui.checkbox(labels.get("Execute with next action"), imBoolean));
      holdPoseInWorldLaterWrapper = new ImBooleanWrapper(getDefinition()::getHoldPoseInWorldLater,
                                                         getDefinition()::setHoldPoseInWorldLater,
                                                         imBoolean -> ImGui.checkbox(labels.get("Hold pose in world later"), imBoolean));
      parentFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Parent frame",
                                                                referenceFrameLibrary,
                                                                getDefinition()::getParentFrameName,
                                                                getState().getChestFrame()::changeFrame);
      yawWidget = new ImDoubleWrapper(getDefinition().getRotation()::getYaw, getDefinition()::setYaw,
                                      imDouble -> ImGuiTools.volatileInputDouble(labels.get("Yaw"), imDouble));
      pitchWidget = new ImDoubleWrapper(getDefinition().getRotation()::getPitch, getDefinition()::setPitch,
                                        imDouble -> ImGuiTools.volatileInputDouble(labels.get("Pitch"), imDouble));
      rollWidget = new ImDoubleWrapper(getDefinition().getRotation()::getRoll, getDefinition()::setRoll,
                                       imDouble -> ImGuiTools.volatileInputDouble(labels.get("Roll"), imDouble));
      trajectoryDurationWidget = new ImDoubleWrapper(getDefinition()::getTrajectoryDuration,
                                                     getDefinition()::setTrajectoryDuration,
                                                     imDouble -> ImGuiTools.volatileInputDouble(labels.get("Trajectory duration"), imDouble));

      String chestBodyName = syncedFullRobotModel.getChest().getName();
      String modelFileName = RDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(chestBodyName));
      highlightModel = new RDXInteractableHighlightModel(modelFileName);

      MultiBodySystemBasics chestOnlySystem = MultiBodySystemMissingTools.createSingleBodySystem(syncedFullRobotModel.getChest());
      List<Collidable> chestCollidables = selectionCollisionModel.getRobotCollidables(chestOnlySystem);

      for (Collidable chestCollidable : chestCollidables)
      {
         mouseCollidables.add(new MouseCollidable(chestCollidable));
      }

      tooltip = new RDX3DPanelTooltip(panel3D);
      panel3D.addImGuiOverlayAddition(this::render3DPanelImGuiOverlays);
   }

   @Override
   public void update()
   {
      super.update();

      if (state.getChestFrame().isChildOfWorld())
      {
         if (poseGizmo.getPoseGizmo().getGizmoFrame() != state.getChestFrame().getReferenceFrame())
         {
            poseGizmo.getPoseGizmo().setGizmoFrame(state.getChestFrame().getReferenceFrame());
            graphicFrame.setParentFrame(state.getChestFrame().getReferenceFrame());
            collisionShapeFrame.setParentFrame(state.getChestFrame().getReferenceFrame());
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

         // if the action is part of a group of concurrent actions that is currently executing or about to be executed
         // send an update of the pose of the chest. Arms IK will be computed wrt this chest pose
         if (state.getIsNextForExecution() && state.getIsToBeExecutedConcurrently())
         {
            wasConcurrent = true;
         }
         else if (wasConcurrent)
         {
            wasConcurrent = false;
         }
      }
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.checkbox(labels.get("Adjust Goal Pose"), adjustGoalPose);
      ImGui.sameLine();
      executeWithNextActionWrapper.renderImGuiWidget();
      holdPoseInWorldLaterWrapper.renderImGuiWidget();
      parentFrameComboBox.render();
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
         tooltip.render("%s Action\nIndex: %d\nName: %s".formatted(getActionTypeTitle(), state.getActionIndex(), getDefinition().getName()));
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
         adjustGoalPose.set(true);
      }

      poseGizmo.process3DViewInput(input, isMouseHovering);

      tooltip.setInput(input);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (state.getChestFrame().isChildOfWorld())
      {
         highlightModel.getRenderables(renderables, pool);
         poseGizmo.getVirtualRenderables(renderables, pool);
      }
   }

   public ReferenceFrame getReferenceFrame()
   {
      return poseGizmo.getPoseGizmo().getGizmoFrame();
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Chest Orientation";
   }
}
