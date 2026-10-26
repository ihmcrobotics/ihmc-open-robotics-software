package us.ihmc.rdx.behaviorTree.actions;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.action.actions.RLWalkActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.RLWalkActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.RLWalkHeadingMode;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.behaviorTree.RDXCRDTTools;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.visualizers.RDXSphereAndArrowGraphic;

public class RDXRLWalkAction extends RDXActionNode<RLWalkActionState, RLWalkActionDefinition>
{
   private static final double GOAL_SPHERE_RADIUS = 0.05;
   private static final double GOAL_ARROW_LENGTH = 0.2;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiReferenceFrameLibraryCombo parentFrameComboBox;
   private final ImDoubleWrapper goalXWidget;
   private final ImDoubleWrapper goalYWidget;
   private final ImDoubleWrapper goalYawWidget;
   private final ImDoubleWrapper maxForwardSpeedWidget;
   private final ImDoubleWrapper maxTurningSpeedWidget;
   private final ImDoubleWrapper positionProximityWidget;
   private final RDXSelectablePose3DGizmo poseGizmo;
   private final RDXSphereAndArrowGraphic goalGraphic = new RDXSphereAndArrowGraphic();
   private Color goalGraphicColor;
   private final FramePose3D goalPose = new FramePose3D();

   public RDXRLWalkAction(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new RLWalkActionState(id, rootNode.getState()), rootNode);

      poseGizmo = new RDXSelectablePose3DGizmo();
      poseGizmo.create(panel3D);

      parentFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Parent frame",
                                                                scene::getAllFrameNames,
                                                                definition::getParentFrameName,
                                                                state.getGoalFrame()::changeFrame);
      goalXWidget = new ImDoubleWrapper(definition::getGoalX,
                                        definition::setGoalX,
                                        imDouble -> ImGuiTools.volatileInputDouble(labels.get("X"), imDouble));
      goalYWidget = new ImDoubleWrapper(definition::getGoalY,
                                        definition::setGoalY,
                                        imDouble -> ImGuiTools.volatileInputDouble(labels.get("Y"), imDouble));
      goalYawWidget = new ImDoubleWrapper(definition::getGoalYaw,
                                          definition::setGoalYaw,
                                          imDouble -> ImGuiTools.volatileInputDouble(labels.get("Yaw"), imDouble));
      maxForwardSpeedWidget = new ImDoubleWrapper(definition::getMaxForwardSpeed,
                                                  definition::setMaxForwardSpeed,
                                                  imDouble -> ImGuiTools.volatileInputDouble(labels.get("Max walking speed (m/s)"), imDouble));
      maxTurningSpeedWidget = new ImDoubleWrapper(definition::getMaxTurningSpeed,
                                                  definition::setMaxTurningSpeed,
                                                  imDouble -> ImGuiTools.volatileInputDouble(labels.get("Max turning speed (rad/s)"), imDouble));
      positionProximityWidget = new ImDoubleWrapper(definition::getPositionProximity,
                                                    definition::setPositionProximity,
                                                    imDouble -> ImGuiTools.volatileInputDouble(labels.get("Position proximity (m)"), imDouble));
   }

   @Override
   public void update()
   {
      super.update();

      if (state.getGoalFrame().isChildOfWorld())
      {
         if (poseGizmo.getPoseGizmo().getGizmoFrame() != state.getGoalFrame().getReferenceFrame())
            poseGizmo.getPoseGizmo().setGizmoFrame(state.getGoalFrame().getReferenceFrame());

         if (!getSelected())
            poseGizmo.setSelected(false);

         RDXCRDTTools.syncGizmoWithBidirectionalField(poseGizmo.getPoseGizmo(), definition.getGoalToParentTransform(), definition);

         // Red while walking to it, blue otherwise
         Color color = state.getIsExecuting() ? Color.RED : Color.BLUE;
         if (color != goalGraphicColor)
         {
            goalGraphic.create(GOAL_SPHERE_RADIUS, GOAL_ARROW_LENGTH, color);
            goalGraphicColor = color;
         }
         goalPose.setToZero(state.getGoalFrame().getReferenceFrame());
         goalPose.changeFrame(ReferenceFrame.getWorldFrame());
         goalGraphic.setToPose(goalPose);
      }
   }

   @Override
   public void renderTreeViewRow()
   {
      super.renderRowBeginning();
      super.renderEditableName();

      ImGui.sameLine();
      ImGui.textDisabled("RL Walk");

      renderRowEnd();
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.checkbox(labels.get("Adjust Goal Pose"), poseGizmo.getSelected());
      parentFrameComboBox.render();

      ImGui.pushItemWidth(80.0f);
      goalXWidget.renderImGuiWidget();
      ImGui.sameLine();
      goalYWidget.renderImGuiWidget();
      ImGui.sameLine();
      goalYawWidget.renderImGuiWidget();
      maxForwardSpeedWidget.renderImGuiWidget();
      ImGuiTools.previousWidgetTooltip("Backward and sideways limits scale with it. 0 uses the controller limit.");
      maxTurningSpeedWidget.renderImGuiWidget();
      ImGuiTools.previousWidgetTooltip("0 uses the controller limit.");
      positionProximityWidget.renderImGuiWidget();
      ImGui.popItemWidth();

      ImGui.text("Heading:");
      for (RLWalkHeadingMode headingMode : RLWalkHeadingMode.values)
      {
         ImGui.sameLine();
         if (ImGui.radioButton(labels.get(headingModeLabel(headingMode)), definition.getHeadingMode() == headingMode))
            definition.setHeadingMode(headingMode);
      }
   }

   private static String headingModeLabel(RLWalkHeadingMode headingMode)
   {
      return switch (headingMode)
      {
         case FACE_GOAL -> "Face goal";
         case KEEP_HEADING -> "Keep heading";
         case TURN_WALK_TURN -> "Turn-walk-turn";
      };
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (getSelected() && state.getGoalFrame().isChildOfWorld())
         poseGizmo.calculate3DViewPick(input);
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (getSelected() && state.getGoalFrame().isChildOfWorld())
         poseGizmo.process3DViewInput(input);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if ((state.getIsNextForExecution() || getSelected() || state.getIsExecuting()) && state.getGoalFrame().isChildOfWorld())
      {
         goalGraphic.getRenderables(renderables, pool);
         poseGizmo.getVirtualRenderables(renderables, pool);
      }
   }

   @Override
   public String getLeafTypeTitle()
   {
      return "RL Walk";
   }
}
