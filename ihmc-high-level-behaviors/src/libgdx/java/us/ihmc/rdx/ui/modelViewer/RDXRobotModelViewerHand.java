package us.ihmc.rdx.ui.modelViewer;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiInputTextFlags;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.scs2.RDXRigidBody;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableTools;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Set;

/**
 * Manages the hand model viewer as part of {@link RDXRobotModelViewer}.}
 */
public class RDXRobotModelViewerHand
{
   private final RDXReferenceFrameGraphic wristFrameGraphic;
   private final RigidBodyBasics hand;
   private final ReferenceFrame handControlFrame;
   private final RDXReferenceFrameGraphic handControlFrameGraphic;
   private final RDXModelInstance handModelGraphic;
   private final MovingReferenceFrame handFrame;
   private final ReferenceFrame handGraphicFrame;
   private final RigidBodyTransform handGraphicToHandFrameTransform;
   private final RigidBodyTransform handControlFrameToWristTransform;
   private final RDXReferenceFrameGraphic graphicFrameGraphic;
   private final RDXSelectablePose3DGizmo handControlFrameGizmo;
   private final RDXSelectablePose3DGizmo handGraphicFrameGizmo;
   private final RDXRigidBody handMultiBody;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString controlTransformString = new ImString(500);
   private final ImString graphicTransformString = new ImString(500);

   public RDXRobotModelViewerHand(RobotSide side, DRCRobotModel robotModel, FullHumanoidRobotModel fullRobotModel, RDXBaseUI baseUI)
   {
      hand = fullRobotModel.getHand(side);
      handFrame = hand.getParentJoint().getFrameAfterJoint();
      handGraphicToHandFrameTransform = robotModel.getHandGraphicToHandFrameTransform(side);
      handControlFrameToWristTransform = robotModel.getJointMap().getHandControlFrameToWristTransform(side);
      handControlFrameGraphic = new RDXReferenceFrameGraphic(0.07, Color.PURPLE);
      wristFrameGraphic = new RDXReferenceFrameGraphic(0.07, Color.TEAL);
      graphicFrameGraphic = new RDXReferenceFrameGraphic(0.07, Color.ORANGE);
      String modelFileName = RDXInteractableTools.getModelFileName(robotModel.getRobotDefinition().getRigidBodyDefinition(hand.getName()));
      handModelGraphic = new RDXModelInstance(RDXModelLoader.load(modelFileName));
      handModelGraphic.setOpacity(0.5f);
      //            handControlFrame = fullRobotModel.getHandControlFrame(side);
      handControlFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(side.getLowerCaseName() + "HandControlFrame",
                                                                                         handFrame,
                                                                                         handControlFrameToWristTransform);
      handGraphicFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(side.getLowerCaseName() + "HandGraphicFrame",
                                                                                         handFrame,
                                                                                         handGraphicToHandFrameTransform);
      handControlFrameGizmo = new RDXSelectablePose3DGizmo(handControlFrame, handControlFrameToWristTransform);
      handControlFrameGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
      handGraphicFrameGizmo = new RDXSelectablePose3DGizmo(handGraphicFrame, handGraphicToHandFrameTransform);
      handGraphicFrameGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());

      handMultiBody = RDXInteractableTools.loadAbilityHand(robotModel.getRobotDefinition(), side);
      if (handMultiBody != null)
         handMultiBody.setOpacityRecursive(0.5f);
   }

   public void update()
   {
      handGraphicFrame.update();
      handControlFrame.update();
      wristFrameGraphic.setToReferenceFrame(handFrame);
      handControlFrameGraphic.setToReferenceFrame(handControlFrame);
      handModelGraphic.setTransformToReferenceFrame(handGraphicFrame);
      graphicFrameGraphic.setTransformToReferenceFrame(handGraphicFrame);
      handMultiBody.getFloatingJoint().getJointPose().set(handGraphicFrame.getTransformToWorldFrame());
      handMultiBody.update();
   }

   public void getRenderables(Array<Renderable> renderables,
                              Pool<Renderable> pool,
                              Set<RDXSceneLevel> sceneLevels,
                              ImBoolean showHandFrames,
                              ImBoolean showHandControlFrames,
                              ImBoolean showHandGraphicFrames,
                              ImBoolean showHandGraphics,
                              ImBoolean showHandMultiBody)
   {
      if (showHandFrames.get())
      {
         wristFrameGraphic.getRenderables(renderables, pool);
      }
      if (showHandControlFrames.get())
      {
         handControlFrameGraphic.getRenderables(renderables, pool);
      }
      if (showHandGraphicFrames.get())
      {
         graphicFrameGraphic.getRenderables(renderables, pool);
      }
      if (showHandGraphics.get())
      {
         handModelGraphic.getRenderables(renderables, pool);
      }
      if (handMultiBody != null && showHandMultiBody.get())
      {
         handMultiBody.getVisualRenderables(renderables, pool);
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Wrist joint: %s".formatted(hand.getParentJoint().getFrameAfterJoint().getName()));
      ImGui.checkbox(labels.get("Control frame tuner"), handControlFrameGizmo.getSelected());
      ImGui.checkbox(labels.get("Graphic frame tuner"), handGraphicFrameGizmo.getSelected());
   }

   public void renderImGuiWidgetsLater()
   {
      controlTransformString.set(handControlFrameToWristTransform);
      ImGui.text("Control frame to wrist:");
      ImGui.inputTextMultiline(labels.getHidden("Control frame to wrist"), controlTransformString, 0, 70, ImGuiInputTextFlags.ReadOnly);
      graphicTransformString.set(handGraphicToHandFrameTransform);
      ImGui.text("Graphic frame to wrist:");
      ImGui.inputTextMultiline(labels.getHidden("Graphic frame to wrist"), graphicTransformString, 0, 70, ImGuiInputTextFlags.ReadOnly);
   }
}
