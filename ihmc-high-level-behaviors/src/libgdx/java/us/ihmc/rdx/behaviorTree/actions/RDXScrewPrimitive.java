package us.ihmc.rdx.behaviorTree.actions;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.behaviors.behaviorTree.action.actions.ArmActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.ArmActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.ScrewPrimitiveState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.rdx.behaviorTree.RDXCRDTTools;
import us.ihmc.rdx.imgui.ImGuiLabelledWidgetAligner;
import us.ihmc.rdx.imgui.ImGuiSliderDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.mesh.RDXDashedLineMesh;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXArmMultiBodyGraphic;
import us.ihmc.rdx.ui.graphics.RDXTrajectoryGraphic;
import us.ihmc.rdx.ui.teleoperation.RDXIKSolverColors;
import us.ihmc.robotics.EuclidCoreMissingTools;

public class RDXScrewPrimitive
{
   private final RDXArmAction parent;
   private final RDXSelectablePose3DGizmo screwAxisGizmo;
   private final RDXDashedLineMesh screwAxisGraphic = new RDXDashedLineMesh(Color.WHITE, Axis3D.X, 0.04);
   private final RDXTrajectoryGraphic trajectoryGraphic = new RDXTrajectoryGraphic();
   private final RecyclingArrayList<FramePose3D> trajectoryPoses = new RecyclingArrayList<>(FramePose3D::new);
   private final ImGuiSliderDoubleWrapper translationWidget;
   private final ImGuiSliderDoubleWrapper rotationWidget;
   private final ImGuiSliderDoubleWrapper maxLinearVelocityWidget;
   private final ImGuiSliderDoubleWrapper maxAngularVelocityWidget;
   private final ImGuiSliderDoubleWrapper previewTimeWidget;
   private boolean playbackPreview = false;
   private final Stopwatch playbackStopwatch = new Stopwatch();

   public RDXScrewPrimitive(RDXArmAction parent)
   {
      this.parent = parent;
      ArmActionDefinition definition = parent.getDefinition();
      ArmActionState state = parent.getState();

      RDX3DPanel panel3D = parent.getPanel3D();
      screwAxisGizmo = new RDXSelectablePose3DGizmo();
      screwAxisGizmo.create(panel3D);

      ImGuiLabelledWidgetAligner widgetAligner = parent.getWidgetAligner();
      translationWidget = new ImGuiSliderDoubleWrapper("Translation", "%.2f", -0.4, 0.4, definition::getTranslation, definition::setTranslation);
      translationWidget.addWidgetAligner(widgetAligner);
      rotationWidget = new ImGuiSliderDoubleWrapper("Rotation", "%.2f", -2.0 * Math.PI, 2.0 * Math.PI,
                                                    definition::getRotation,
                                                    definition::setRotation);
      rotationWidget.addWidgetAligner(widgetAligner);
      maxLinearVelocityWidget = new ImGuiSliderDoubleWrapper("Max Linear Velocity", "%.2f", 0.05, 1.0,
                                                             definition::getMaxLinearVelocity,
                                                             definition::setMaxLinearVelocity);
      maxLinearVelocityWidget.addWidgetAligner(widgetAligner);
      maxAngularVelocityWidget = new ImGuiSliderDoubleWrapper("Max Angular Velocity", "%.2f", 0.1, Math.PI,
                                                              definition::getMaxAngularVelocity,
                                                              definition::setMaxAngularVelocity);
      maxAngularVelocityWidget.addWidgetAligner(widgetAligner);
      previewTimeWidget = new ImGuiSliderDoubleWrapper("Preview Time", "%.2f", 0.0, 1.0,
                                                       state.getPreviewRequestedTime()::getValue,
                                                       state.getPreviewRequestedTime()::setValue);
      previewTimeWidget.addButton("Play", this::togglePlayPausePreview);
      previewTimeWidget.addWidgetAligner(widgetAligner);
   }

   public void update()
   {
      ArmActionState state = parent.getState();
      ArmActionDefinition definition = parent.getDefinition();

      if (state.getScrewFrame().isChildOfWorld())
      {
         if (screwAxisGizmo.getPoseGizmo().getGizmoFrame() != state.getScrewFrame().getReferenceFrame())
            screwAxisGizmo.getPoseGizmo().setGizmoFrame(state.getScrewFrame().getReferenceFrame());

         if (!parent.getSelected())
            screwAxisGizmo.setSelected(false);

         RDXCRDTTools.syncGizmoWithBidirectionalField(screwAxisGizmo.getPoseGizmo(), definition.getScrewAxisPoseInObjectFrame(), definition);

         double screwAxisLineWidth = 0.005;
         screwAxisGraphic.update(screwAxisGizmo.getPoseGizmo().getPose(), screwAxisLineWidth, 1.0);

         double trajectoryLineWidth = 0.01;
         trajectoryPoses.clear();
         for (int i = 0; i < state.getPreviewTrajectory().getSize(); i++)
            trajectoryPoses.add().set(state.getPreviewTrajectory().getValueReadOnly(i));
         trajectoryGraphic.update(trajectoryLineWidth, trajectoryPoses);

         if (playbackPreview)
         {
            double requestedTime = state.getPreviewRequestedTime().getValue();
            requestedTime += playbackStopwatch.lap() / state.getPreviewTrajectoryDuration().getValue();
            requestedTime %= 1.0;
            state.getPreviewRequestedTime().setValue(requestedTime);
         }
      }
   }

   public void visualizeIK(RDXArmMultiBodyGraphic armMultiBodyGraphic)
   {
      ArmActionState state = parent.getState();
      armMultiBodyGraphic.getFloatingJoint().getJointPose().set(parent.getSyncedRobot().getFramePoseReadOnly(HumanoidReferenceFrames::getChestFrame));
      for (int i = 0; i < armMultiBodyGraphic.getJoints().length; i++)
         armMultiBodyGraphic.getJoints()[i].setQ(state.getScrewPreviewJointAngles().getValueReadOnly(i));
      armMultiBodyGraphic.updateAfterModifyingConfiguration();
      armMultiBodyGraphic.setColor(RDXIKSolverColors.getColor(state.getPreviewSolutionQuality().getValue()));
   }

   public void renderImGuiWidgets()
   {
      ArmActionState state = parent.getState();

      ImGui.checkbox(parent.getLabels().get("Adjust Screw Axis Pose"), screwAxisGizmo.getSelected());
      parent.getParentFrameComboBox().render();
      int size = state.getPreviewTrajectory().getSize();
      int limit = ScrewPrimitiveState.TRAJECTORY_SIZE_LIMIT;
      if (size == limit)
         ImGui.pushStyleColor(ImGuiCol.Text, ImGuiTools.RED);
      ImGui.text("Trajectory points: %d/%d  Duration: %.1f s  Velocity %.2f m/s  %.2f %s/s"
                       .formatted(size,
                                  limit,
                                  state.getPreviewTrajectoryDuration().getValue(),
                                  state.getPreviewTrajectoryLinearVelocity().getValue(),
                                  state.getPreviewTrajectoryAngularVelocity().getValue(),
                                  EuclidCoreMissingTools.DEGREE_SYMBOL));
      if (size == limit)
         ImGui.popStyleColor();
      translationWidget.renderImGuiWidget();
      rotationWidget.renderImGuiWidget();
      maxLinearVelocityWidget.renderImGuiWidget();
      maxAngularVelocityWidget.renderImGuiWidget();
      parent.getLinearPositionWeightWidget().renderImGuiWidget();
      parent.getAngularPositionWeightWidget().renderImGuiWidget();
      parent.getJointspaceWeightWidget().renderImGuiWidget();
      ImGui.pushItemWidth(ImGui.getFontSize() * 10.0f);
      parent.getPositionErrorToleranceInput().renderImGuiWidget();
      parent.getOrientationErrorToleranceDegreesInput().renderImGuiWidget();
      ImGui.popItemWidth();
      previewTimeWidget.renderImGuiWidget();
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (parent.getState().getScrewFrame().isChildOfWorld())
         screwAxisGizmo.calculate3DViewPick(input);
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (parent.getState().getScrewFrame().isChildOfWorld())
         screwAxisGizmo.process3DViewInput(input);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (parent.getState().getScrewFrame().isChildOfWorld())
      {
         screwAxisGizmo.getVirtualRenderables(renderables, pool);
         screwAxisGraphic.getRenderables(renderables, pool);
         trajectoryGraphic.getRenderables(renderables, pool);
      }
   }

   public void deselectGizmos()
   {
      screwAxisGizmo.setSelected(false);
   }

   public String getLeafTypeTitle()
   {
      return parent.getDefinition().getSide().getPascalCaseName() + " Screw Primitive";
   }

   private void togglePlayPausePreview()
   {
      playbackPreview = !playbackPreview;
      previewTimeWidget.addButton(playbackPreview ? "Pause" : "Play", this::togglePlayPausePreview);

      if (playbackPreview)
         playbackStopwatch.reset();
   }
}
