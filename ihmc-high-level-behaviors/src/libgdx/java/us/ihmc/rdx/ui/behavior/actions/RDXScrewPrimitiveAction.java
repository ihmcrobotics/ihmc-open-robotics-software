package us.ihmc.rdx.ui.behavior.actions;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sequence.actions.ScrewPrimitiveActionDefinition;
import us.ihmc.behaviors.sequence.actions.ScrewPrimitiveActionState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.mesh.RDXDashedLineMesh;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXTrajectoryGraphic;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXScrewPrimitiveAction extends RDXActionNode<ScrewPrimitiveActionState, ScrewPrimitiveActionDefinition>
{
   private final FullHumanoidRobotModel syncedRobot;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiReferenceFrameLibraryCombo objectFrameComboBox;
   private final ImGuiSliderDoubleWrapper translationWidget;
   private final ImGuiSliderDoubleWrapper rotationWidget;
   private final ImGuiSliderDoubleWrapper maxLinearVelocityWidget;
   private final ImGuiSliderDoubleWrapper maxAngularVelocityWidget;
   private final ImGuiSliderDoubleWrapper maxForceWidget;
   private final ImGuiSliderDoubleWrapper maxTorqueWidget;
   private final ImGuiSliderDoubleWrapper linearPositionWeightWidget;
   private final ImGuiSliderDoubleWrapper angularPositionWeightWidget;
   private final RDXSelectablePose3DGizmo screwAxisGizmo;
   private final ImBoolean adjustWrenchContactPose = new ImBoolean();
   private final RDXSelectablePose3DGizmo wrenchContactPoseGizmo;
   private final RDXDashedLineMesh screwAxisGraphic = new RDXDashedLineMesh(Color.WHITE, Axis3D.X, 0.04);
   private final RDXTrajectoryGraphic trajectoryGraphic = new RDXTrajectoryGraphic();
   private final RecyclingArrayList<FramePose3D> trajectoryPoses = new RecyclingArrayList<>(FramePose3D::new);

   public RDXScrewPrimitiveAction(long id,
                                  CRDTInfo crdtInfo,
                                  WorkspaceResourceDirectory saveFileDirectory,
                                  RDX3DPanel panel3D,
                                  DRCRobotModel robotModel,
                                  FullHumanoidRobotModel syncedRobot,
                                  RobotCollisionModel selectionCollisionModel,
                                  ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(new ScrewPrimitiveActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      this.syncedRobot = syncedRobot;

      getDefinition().setDescription("Screw primitive");

      screwAxisGizmo = new RDXSelectablePose3DGizmo(getDefinition().getScrewAxisPoseInObjectFrame().getValue(),
                                                    ReferenceFrame.getWorldFrame(),
                                                    getSelected());
      screwAxisGizmo.create(panel3D);
      wrenchContactPoseGizmo = new RDXSelectablePose3DGizmo(getDefinition().getWrenchContactPoseInHandControlFrame().getValue(),
                                                            ReferenceFrame.getWorldFrame(),
                                                            adjustWrenchContactPose);
      wrenchContactPoseGizmo.create(panel3D);

      objectFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Object frame",
                                                                referenceFrameLibrary,
                                                                getDefinition()::getObjectFrameName,
                                                                getDefinition()::setObjectFrameName);
      ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
      translationWidget = new ImGuiSliderDoubleWrapper("Translation", "%.2f", -0.4, 0.4, getDefinition()::getTranslation, getDefinition()::setTranslation);
      translationWidget.addWidgetAligner(widgetAligner);
      rotationWidget = new ImGuiSliderDoubleWrapper("Rotation", "%.2f", -2.0 * Math.PI, 2.0 * Math.PI,
                                                    getDefinition()::getRotation,
                                                    getDefinition()::setRotation);
      rotationWidget.addWidgetAligner(widgetAligner);
      maxLinearVelocityWidget = new ImGuiSliderDoubleWrapper("Max Linear Velocity", "%.2f", 0.05, 1.0,
                                                             getDefinition()::getMaxLinearVelocity,
                                                             getDefinition()::setMaxLinearVelocity);
      maxLinearVelocityWidget.addWidgetAligner(widgetAligner);
      maxAngularVelocityWidget = new ImGuiSliderDoubleWrapper("Max Angular Velocity", "%.2f", 0.1, Math.PI,
                                                              getDefinition()::getMaxAngularVelocity,
                                                              getDefinition()::setMaxAngularVelocity);
      maxAngularVelocityWidget.addWidgetAligner(widgetAligner);
      maxForceWidget = new ImGuiSliderDoubleWrapper("Max Force", "%.2f", 0.0, 70.0, getDefinition()::getMaxForce, getDefinition()::setMaxForce);
      maxForceWidget.addButton("Disable Wrench", () -> getDefinition().setMaxForce(0.0));
      maxForceWidget.addWidgetAligner(widgetAligner);
      maxTorqueWidget = new ImGuiSliderDoubleWrapper("Max Torque", "%.2f", 0.5, 20.0, getDefinition()::getMaxTorque, getDefinition()::setMaxTorque);
      maxTorqueWidget.addButton("Disable Wrench", () -> getDefinition().setMaxTorque(0.0));
      maxTorqueWidget.addWidgetAligner(widgetAligner);
      linearPositionWeightWidget = new ImGuiSliderDoubleWrapper("Linear Position Weight", "%.2f", 0.0, 70.0,
                                                                getDefinition()::getLinearPositionWeight,
                                                                getDefinition()::setLinearPositionWeight);
      linearPositionWeightWidget.addButton("Use Default Weights", () -> getDefinition().setLinearPositionWeight(-1.0));
      linearPositionWeightWidget.addWidgetAligner(widgetAligner);
      angularPositionWeightWidget = new ImGuiSliderDoubleWrapper("Angular Position Weight", "%.2f", 0.0, 70.0,
                                                                getDefinition()::getAngularPositionWeight,
                                                                getDefinition()::setAngularPositionWeight);
      angularPositionWeightWidget.addButton("Use Default Weights", () -> getDefinition().setAngularPositionWeight(-1.0));
      angularPositionWeightWidget.addWidgetAligner(widgetAligner);
   }

   @Override
   public void update()
   {
      super.update();

      if (getState().getScrewFrame().isChildOfWorld())
      {
         screwAxisGizmo.getPoseGizmo().setGizmoFrame(getState().getScrewFrame().getReferenceFrame());
         screwAxisGizmo.getPoseGizmo().update();
         ReferenceFrame handControlFrame = syncedRobot.getHandControlFrame(getDefinition().getSide());
         if (wrenchContactPoseGizmo.getPoseGizmo().getGizmoFrame().getParent() != handControlFrame)
         {
            wrenchContactPoseGizmo.getPoseGizmo().setParentFrame(handControlFrame);
         }
         wrenchContactPoseGizmo.getPoseGizmo().update();

         double screwAxisLineWidth = 0.005;
         screwAxisGraphic.update(screwAxisGizmo.getPoseGizmo().getPose(), screwAxisLineWidth, 1.0);

         double trajectoryLineWidth = 0.01;
         trajectoryPoses.clear();
         for (int i = 0; i < getState().getTrajectory().getSize(); i++)
            trajectoryPoses.add().set(getState().getTrajectory().getValueReadOnly(i));
         trajectoryGraphic.update(trajectoryLineWidth, trajectoryPoses);
      }
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      objectFrameComboBox.render();
      translationWidget.renderImGuiWidget();
      rotationWidget.renderImGuiWidget();
      maxLinearVelocityWidget.renderImGuiWidget();
      maxAngularVelocityWidget.renderImGuiWidget();
      maxForceWidget.renderImGuiWidget();
      maxTorqueWidget.renderImGuiWidget();
      linearPositionWeightWidget.renderImGuiWidget();
      angularPositionWeightWidget.renderImGuiWidget();
      ImGui.checkbox(labels.get("Adjust Screw Axis Pose"), getSelected());
      ImGui.checkbox(labels.get("Adjust Wrench Contact Pose"), adjustWrenchContactPose);
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (getState().getScrewFrame().isChildOfWorld())
      {
         screwAxisGizmo.calculate3DViewPick(input);
         wrenchContactPoseGizmo.calculate3DViewPick(input);
      }
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (getState().getScrewFrame().isChildOfWorld())
      {
         screwAxisGizmo.process3DViewInput(input);
         wrenchContactPoseGizmo.process3DViewInput(input);
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (getState().getScrewFrame().isChildOfWorld())
      {
         screwAxisGizmo.getVirtualRenderables(renderables, pool);
         wrenchContactPoseGizmo.getVirtualRenderables(renderables, pool);
         screwAxisGraphic.getRenderables(renderables, pool);
         trajectoryGraphic.getRenderables(renderables, pool);
      }
   }

   @Override
   public String getActionTypeTitle()
   {
      return getDefinition().getSide().getPascalCaseName() + " Screw Primitive";
   }
}
