package us.ihmc.rdx.behaviorTree.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.behaviors.behaviorTree.action.actions.SpineActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.SpineActionState;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.behaviorTree.RDXCRDTTools;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiLabelledWidgetAligner;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiSliderDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.rdx.ui.affordances.RDXInteractableHighlightModel;
import us.ihmc.rdx.ui.affordances.RDXInteractableTools;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.interaction.MouseCollidable;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.scs2.simulation.collision.Collidable;

import java.util.ArrayList;
import java.util.List;

public class RDXSpineAction extends RDXActionNode<SpineActionState, SpineActionDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBooleanWrapper holdPoseInWorldLaterWrapper;
   private final ImBooleanWrapper jointspaceOnlyWrapper;
   private final ImGuiReferenceFrameLibraryCombo parentFrameComboBox;
   private final ImDoubleWrapper yawWidget;
   private final ImDoubleWrapper pitchWidget;
   private final ImDoubleWrapper rollWidget;
   private final ImDoubleWrapper trajectoryDurationWidget;
   private final ImGuiSliderDoubleWrapper[] jointAngleWidgets;
   private final SpineJointName[] spineJointNames;
   private final double[] jointUpperLimits;
   private final double[] jointLowerLimits;
   /** Gizmo is control frame */
   private final RDXSelectablePose3DGizmo poseGizmo;
   private final MutableReferenceFrame graphicFrame = new MutableReferenceFrame();
   private final MutableReferenceFrame collisionShapeFrame = new MutableReferenceFrame();
   private boolean isMouseHovering = false;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private final ArrayList<MouseCollidable> mouseCollidables = new ArrayList<>();
   private final RDXInteractableHighlightModel highlightModel;
   private final RDX3DPanelTooltip tooltip;

   public RDXSpineAction(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new SpineActionState(id, rootNode.getState()), rootNode);

      poseGizmo = new RDXSelectablePose3DGizmo();
      poseGizmo.create(panel3D);

      holdPoseInWorldLaterWrapper = new ImBooleanWrapper(definition::getHoldPoseInWorldLater,
                                                         definition::setHoldPoseInWorldLater,
                                                         imBoolean -> ImGui.checkbox(labels.get("Hold pose in world later"), imBoolean));
      jointspaceOnlyWrapper = new ImBooleanWrapper(definition::getJointspaceOnly,
                                                   definition::setJointspaceOnly,
                                                   imBoolean ->
                                                   {
                                                      if (ImGui.radioButton(labels.get("Taskspace"), !imBoolean.get()))
                                                         imBoolean.set(false);
                                                      ImGui.sameLine();
                                                      if (ImGui.radioButton(labels.get("Jointspace Only"), imBoolean.get()))
                                                         imBoolean.set(true);
                                                   });
      parentFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Parent frame",
                                                                scene::getAllFrameNames,
                                                                definition::getParentFrameName,
                                                                state.getChestFrame()::changeFrame);
      yawWidget = new ImDoubleWrapper(definition.getRotationReadOnly()::getYaw, definition::setYaw,
                                      imDouble -> ImGuiTools.volatileInputDouble(labels.get("Yaw"), imDouble));
      pitchWidget = new ImDoubleWrapper(definition.getRotationReadOnly()::getPitch, definition::setPitch,
                                        imDouble -> ImGuiTools.volatileInputDouble(labels.get("Pitch"), imDouble));
      rollWidget = new ImDoubleWrapper(definition.getRotationReadOnly()::getRoll, definition::setRoll,
                                       imDouble -> ImGuiTools.volatileInputDouble(labels.get("Roll"), imDouble));
      trajectoryDurationWidget = new ImDoubleWrapper(definition::getTrajectoryDuration,
                                                     definition::setTrajectoryDuration,
                                                     imDouble -> ImGuiTools.volatileInputDouble(labels.get("Trajectory duration"), imDouble));

      spineJointNames = syncedRobot.getRobotModel().getJointMap().getSpineJointNames();
      int jointCount = Math.min(spineJointNames.length, definition.getJointAngles().getLength());
      jointUpperLimits = new double[jointCount];
      jointLowerLimits = new double[jointCount];
      jointAngleWidgets = new ImGuiSliderDoubleWrapper[jointCount];
      ImGuiLabelledWidgetAligner jointWidgetAligner = new ImGuiLabelledWidgetAligner();
      for (int i = 0; i < jointCount; i++)
      {
         OneDoFJointBasics spineJoint = syncedRobot.getFullRobotModel().getSpineJoint(spineJointNames[i]);
         jointUpperLimits[i] = spineJoint.getJointLimitUpper();
         jointLowerLimits[i] = spineJoint.getJointLimitLower();
         int jointIndex = i;
         jointAngleWidgets[i] = new ImGuiSliderDoubleWrapper(spineJointNames[i].toString(),
                                                             "%.0f" + EuclidCoreMissingTools.DEGREE_SYMBOL,
                                                             Math.toDegrees(jointLowerLimits[i]),
                                                             Math.toDegrees(jointUpperLimits[i]),
                                                             () -> Math.toDegrees(definition.getJointAngles().getValueReadOnly(jointIndex)),
                                                             jointAngle -> definition.getJointAngles().setValue(jointIndex, Math.toRadians(jointAngle)));
         jointAngleWidgets[i].addWidgetAligner(jointWidgetAligner);
      }

      String chestBodyName = syncedRobot.getFullRobotModel().getChest().getName();
      String modelFileName = RDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(chestBodyName));
      highlightModel = new RDXInteractableHighlightModel(modelFileName);

      MultiBodySystemBasics chestOnlySystem = MultiBodySystemMissingTools.createSingleBodySystem(syncedRobot.getFullRobotModel().getChest());
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

         if (!getSelected())
            poseGizmo.setSelected(false);

         RDXCRDTTools.syncGizmoWithBidirectionalField(poseGizmo.getPoseGizmo(), definition.getChestToParentTransform(), definition);

         if (state.getIsNextForExecution() || getSelected())
         {
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
      }
   }

   @Override
   public void renderTreeViewRow()
   {
      super.renderRowBeginning();
      super.renderEditableName();

      ImGui.sameLine();
      ImGui.textDisabled("Spine");

      renderRowEnd();
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      trajectoryDurationWidget.renderImGuiWidget();
      jointspaceOnlyWrapper.renderImGuiWidget();
      if (definition.getJointspaceOnly())
         for (ImGuiSliderDoubleWrapper jointAngleWidget : jointAngleWidgets)
            jointAngleWidget.renderImGuiWidget();
      else
      {
         parentFrameComboBox.render();
         ImGui.checkbox(labels.get("Adjust Goal Pose"), poseGizmo.getSelected());
         holdPoseInWorldLaterWrapper.renderImGuiWidget();
         ImGui.pushItemWidth(ImGui.getFontSize() * 5.0f);
         yawWidget.renderImGuiWidget();
         ImGui.sameLine();
         pitchWidget.renderImGuiWidget();
         ImGui.sameLine();
         rollWidget.renderImGuiWidget();
         ImGui.popItemWidth();
      }
   }

   public void render3DPanelImGuiOverlays()
   {
      if (isMouseHovering)
      {
         tooltip.render("%s Action\nIndex: %d\nName: %s".formatted(getLeafTypeTitle(), state.getLeafIndex(), definition.getName()));
      }
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (getSelected() && state.getChestFrame().isChildOfWorld())
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
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (getSelected() && state.getChestFrame().isChildOfWorld())
      {
         isMouseHovering = input.getClosestPick() == pickResult;

         boolean isClickedOn = isMouseHovering && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);
         if (isClickedOn)
         {
            poseGizmo.setSelected(true);
         }

         poseGizmo.process3DViewInput(input, isMouseHovering);

         tooltip.setInput(input);
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if ((state.getIsNextForExecution() || getSelected() || state.getIsExecuting()) && state.getChestFrame().isChildOfWorld())
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
   public String getLeafTypeTitle()
   {
      return "Spine";
   }
}
