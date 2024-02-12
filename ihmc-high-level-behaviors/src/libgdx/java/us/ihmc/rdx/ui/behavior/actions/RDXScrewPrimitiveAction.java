package us.ihmc.rdx.ui.behavior.actions;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.actions.ScrewPrimitiveActionDefinition;
import us.ihmc.behaviors.sequence.actions.ScrewPrimitiveActionState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.mesh.RDXDashedLineMesh;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXArmMultiBodyGraphic;
import us.ihmc.rdx.ui.graphics.RDXTrajectoryGraphic;
import us.ihmc.rdx.ui.teleoperation.RDXIKSolverColors;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXScrewPrimitiveAction extends RDXActionNode<ScrewPrimitiveActionState, ScrewPrimitiveActionDefinition>
{
   private final ScrewPrimitiveActionState state;
   private final ScrewPrimitiveActionDefinition definition;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiReferenceFrameLibraryCombo objectFrameComboBox;
   private final ImGuiSliderDoubleWrapper translationWidget;
   private final ImGuiSliderDoubleWrapper rotationWidget;
   private final ImGuiSliderDoubleWrapper maxLinearVelocityWidget;
   private final ImGuiSliderDoubleWrapper maxAngularVelocityWidget;
   private final ImBooleanWrapper jointspaceOnlyWidget;
   private final ImGuiSliderDoubleWrapper linearPositionWeightWidget;
   private final ImGuiSliderDoubleWrapper angularPositionWeightWidget;
   private final ImGuiSliderDoubleWrapper jointspaceWeightWidget;
   private final ImGuiSliderDoubleWrapper previewTimeWidget;
   private final RDXSelectablePose3DGizmo screwAxisGizmo;
   private final RDXDashedLineMesh screwAxisGraphic = new RDXDashedLineMesh(Color.WHITE, Axis3D.X, 0.04);
   private final RDXTrajectoryGraphic trajectoryGraphic = new RDXTrajectoryGraphic();
   private final RecyclingArrayList<FramePose3D> trajectoryPoses = new RecyclingArrayList<>(FramePose3D::new);
   private final SideDependentList<RDXArmMultiBodyGraphic> armMultiBodyGraphics = new SideDependentList<>();

   public RDXScrewPrimitiveAction(long id,
                                  CRDTInfo crdtInfo,
                                  WorkspaceResourceDirectory saveFileDirectory,
                                  RDX3DPanel panel3D,
                                  ReferenceFrameLibrary referenceFrameLibrary,
                                  ROS2SyncedRobotModel syncedRobot)
   {
      super(new ScrewPrimitiveActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      state = getState();
      definition = getDefinition();

      definition.setDescription("Screw primitive");

      this.syncedRobot = syncedRobot;

      screwAxisGizmo = new RDXSelectablePose3DGizmo(definition.getScrewAxisPoseInObjectFrame().getValue(), ReferenceFrame.getWorldFrame());
      screwAxisGizmo.create(panel3D);

      objectFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Object frame",
                                                                referenceFrameLibrary,
                                                                definition::getObjectFrameName,
                                                                definition::setObjectFrameName);
      ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
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
      jointspaceOnlyWidget = new ImBooleanWrapper(definition::getJointspaceOnly,
                                                  definition::setJointspaceOnly,
                                                      imBoolean -> {
                                                         if (ImGui.radioButton(labels.get("Hybrid"), !imBoolean.get()))
                                                            imBoolean.set(false);
                                                         ImGui.sameLine();
                                                         if (ImGui.radioButton(labels.get("Jointspace Only"), imBoolean.get()))
                                                            imBoolean.set(true);
                                                      });
      linearPositionWeightWidget = new ImGuiSliderDoubleWrapper("Linear Position Weight", "%.2f", 0.0, 70.0,
                                                                definition::getLinearPositionWeight,
                                                                definition::setLinearPositionWeight);
      linearPositionWeightWidget.addButton("Use Default Weights", () -> definition.setLinearPositionWeight(-1.0));
      linearPositionWeightWidget.addWidgetAligner(widgetAligner);
      angularPositionWeightWidget = new ImGuiSliderDoubleWrapper("Angular Position Weight", "%.2f", 0.0, 70.0,
                                                                definition::getAngularPositionWeight,
                                                                definition::setAngularPositionWeight);
      angularPositionWeightWidget.addButton("Use Default Weights", () -> definition.setAngularPositionWeight(-1.0));
      angularPositionWeightWidget.addWidgetAligner(widgetAligner);
      jointspaceWeightWidget = new ImGuiSliderDoubleWrapper("Jointspace Weight", "%.2f", 0.0, 70.0,
                                                            definition::getJointspaceWeight,
                                                            definition::setJointspaceWeight);
      jointspaceWeightWidget.addButton("Use Default Weights", () -> definition.setJointspaceWeight(-1.0));
      jointspaceWeightWidget.addWidgetAligner(widgetAligner);
      previewTimeWidget = new ImGuiSliderDoubleWrapper("Preview Time", "%.2f", 0.0, 1.0,
                                                       state.getPreviewRequestedTime()::getValue,
                                                       state.getPreviewRequestedTime()::setValue);
      previewTimeWidget.addWidgetAligner(widgetAligner);

      for (RobotSide side : RobotSide.values)
      {
         armMultiBodyGraphics.put(side, new RDXArmMultiBodyGraphic(syncedRobot.getRobotModel(), syncedRobot.getFullRobotModel(), side));
      }
   }

   @Override
   public void update()
   {
      super.update();

      if (state.getScrewFrame().isChildOfWorld())
      {
         screwAxisGizmo.getPoseGizmo().setGizmoFrame(state.getScrewFrame().getReferenceFrame());
         screwAxisGizmo.getPoseGizmo().update();

         double screwAxisLineWidth = 0.005;
         screwAxisGraphic.update(screwAxisGizmo.getPoseGizmo().getPose(), screwAxisLineWidth, 1.0);

         double trajectoryLineWidth = 0.01;
         trajectoryPoses.clear();
         for (int i = 0; i < state.getPreviewTrajectory().getSize(); i++)
            trajectoryPoses.add().set(state.getPreviewTrajectory().getValueReadOnly(i));
         trajectoryGraphic.update(trajectoryLineWidth, trajectoryPoses);

         if (state.getIsNextForExecution())
         {
            RDXArmMultiBodyGraphic armMultiBodyGraphic = armMultiBodyGraphics.get(getDefinition().getSide());
            armMultiBodyGraphic.getFloatingJoint().getJointPose().set(syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getChestFrame));
            for (int i = 0; i < armMultiBodyGraphic.getJoints().length; i++)
            {
               armMultiBodyGraphic.getJoints()[i].setQ(state.getPreviewJointAngles().getValueReadOnly(i));
            }
            armMultiBodyGraphic.updateAfterModifyingConfiguration();
            armMultiBodyGraphic.setColor(RDXIKSolverColors.getColor(state.getPreviewSolutionQuality().getValue()));
         }
      }
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.checkbox(labels.get("Adjust Screw Axis Pose"), screwAxisGizmo.getSelected());
      objectFrameComboBox.render();
      int size = state.getPreviewTrajectory().getSize();
      int limit = ScrewPrimitiveActionState.TRAJECTORY_SIZE_LIMIT;
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
      jointspaceOnlyWidget.renderImGuiWidget();
      if (definition.getJointspaceOnly())
         ImGui.beginDisabled();
      linearPositionWeightWidget.renderImGuiWidget();
      angularPositionWeightWidget.renderImGuiWidget();
      if (definition.getJointspaceOnly())
         ImGui.endDisabled();
      jointspaceWeightWidget.renderImGuiWidget();
      previewTimeWidget.renderImGuiWidget();
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (state.getScrewFrame().isChildOfWorld())
      {
         screwAxisGizmo.calculate3DViewPick(input);
      }
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (state.getScrewFrame().isChildOfWorld())
      {
         screwAxisGizmo.process3DViewInput(input);
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (state.getScrewFrame().isChildOfWorld())
      {
         screwAxisGizmo.getVirtualRenderables(renderables, pool);
         screwAxisGraphic.getRenderables(renderables, pool);
         trajectoryGraphic.getRenderables(renderables, pool);

         if (state.getIsNextForExecution())
            armMultiBodyGraphics.get(getDefinition().getSide()).getRootBody().getVisualRenderables(renderables, pool);
      }
   }

   @Override
   public String getActionTypeTitle()
   {
      return definition.getSide().getPascalCaseName() + " Screw Primitive";
   }
}
