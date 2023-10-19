package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.RigidBodyTransformMessage;
import imgui.flag.ImGuiInputTextFlags;
import imgui.flag.ImGuiMouseButton;
import imgui.ImGui;
import imgui.type.ImString;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.communication.ros2.ROS2TunedRigidBodyTransform;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

import java.util.Set;

public class RDXInteractableFrameModel
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString transformText = new ImString(1000);
   private ReferenceFrame representativeReferenceFrame;
   private final FramePose3D tempFramePose = new FramePose3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private RigidBodyTransform transformToParent;
   private RDXModelInstance modelInstance;
   private RDXInteractableHighlightModel highlightModelInstance;
   private RDXMousePickRayCollisionCalculator collisionCalculator;
   private boolean isMouseHovering;
   private RDXSelectablePose3DGizmo selectablePose3DGizmo;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private final Notification contextMenuNotification = new Notification();
   private Runnable extendedContextMenu;
   private ROS2TunedRigidBodyTransform syncedTransformForTuning;
   private boolean showing = true;

   public void create(ReferenceFrame parentFrame, RDX3DPanel panel3D, ModelData modelData, RDXMousePickRayCollisionCalculator collisionCalculator)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      ReferenceFrame referenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(parentFrame, transform);
      create(referenceFrame, transform, panel3D, modelData, collisionCalculator);
   }

   public void create(ReferenceFrame referenceFrameToRepresent,
                      RigidBodyTransform transformToParentToModify,
                      RDX3DPanel panel3D,
                      ModelData modelData,
                      RDXMousePickRayCollisionCalculator collisionCalculator)
   {
      representativeReferenceFrame = referenceFrameToRepresent;
      transformToParent = transformToParentToModify;
      this.modelInstance = new RDXModelInstance(new Model(modelData));
      this.collisionCalculator = collisionCalculator;

      highlightModelInstance = new RDXInteractableHighlightModel(modelData);
      selectablePose3DGizmo = new RDXSelectablePose3DGizmo(representativeReferenceFrame, transformToParent);
      selectablePose3DGizmo.create(panel3D);
      panel3D.addImGui3DViewPickCalculator(this, this::calculate3DViewPick);
      panel3D.addImGui3DViewInputProcessor(this, this::process3DViewInput);
      panel3D.getScene().addRenderableProvider(this, this::getRenderables);
      panel3D.addImGuiOverlayAddition(this, this::renderTooltipsAndContextMenu);
   }

   public void addRemoteTuning(ROS2PublishSubscribeAPI ros2, ROS2IOTopicPair<RigidBodyTransformMessage> topicPair, RigidBodyTransform rigidBodyTransformToSync)
   {
      syncedTransformForTuning = ROS2TunedRigidBodyTransform.remoteTuner(ros2, topicPair, rigidBodyTransformToSync);
   }

   public void update()
   {
      boolean interactableSelected = isSelected();
      syncedTransformForTuning.setAcceptingUpdates(!interactableSelected);
      syncedTransformForTuning.setPublishingStatus(interactableSelected);
      syncedTransformForTuning.update();
   }

   private void calculate3DViewPick(ImGui3DViewInput input)
   {
      selectablePose3DGizmo.calculate3DViewPick(input);

      Line3DReadOnly pickRay = input.getPickRayInWorld();
      double closestCollisionDistance = collisionCalculator.calculateClosestCollision(pickRay);
      if (!Double.isNaN(closestCollisionDistance))
      {
         pickResult.setDistanceToCamera(closestCollisionDistance);
         input.addPickResult(pickResult);
      }
   }

   private void process3DViewInput(ImGui3DViewInput input)
   {
      isMouseHovering = pickResult == input.getClosestPick();

      if (isMouseHovering && ImGui.getMouseClickedCount(ImGuiMouseButton.Right) == 1)
      {
         contextMenuNotification.set();
      }

      selectablePose3DGizmo.process3DViewInput(input, isMouseHovering);
      tempFramePose.setToZero(representativeReferenceFrame);
      tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      tempFramePose.get(tempTransform);

      LibGDXTools.toLibGDX(tempTransform, modelInstance.transform);
      highlightModelInstance.setPose(tempTransform);
   }

   private void renderTooltipsAndContextMenu()
   {
      if (contextMenuNotification.poll())
      {
         ImGui.openPopup(labels.get("Popup"));
      }

      if (ImGui.beginPopup(labels.get("Popup")))
      {
         if (extendedContextMenu != null)
            extendedContextMenu.run();
         ImGui.text("Transform to parent: (" + representativeReferenceFrame.getParent().getName() + ")");
         transformText.set(String.format("Translation:\n%s\nYaw, pitch, roll:\n%s",
                                         transformToParent.getTranslation(),
                                         EuclidCoreMissingTools.getYawPitchRollValuesStringDegrees(transformToParent.getRotation())));
         ImGui.inputTextMultiline(labels.getHidden("transformToParent"), transformText, 0, 60, ImGuiInputTextFlags.ReadOnly);
         if (ImGui.menuItem("Close"))
            ImGui.closeCurrentPopup();
         ImGui.endPopup();
      }
   }

   private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (showing && (sceneLevels.contains(RDXSceneLevel.MODEL) || sceneLevels.contains(RDXSceneLevel.VIRTUAL)))
      {
         modelInstance.getRenderables(renderables, pool);
      }
      if (showing && sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
         if (isMouseHovering || selectablePose3DGizmo.isSelected())
         {
            highlightModelInstance.getRenderables(renderables, pool);
         }
         selectablePose3DGizmo.getVirtualRenderables(renderables, pool);
      }
   }

   public void destroy(RDX3DPanel panel3D)
   {
      panel3D.getScene().removeRenderable(this);
      panel3D.removeImGui3DViewInputProcessor(this);
      panel3D.removeImGui3DViewPickCalculator(this);
      panel3D.removeImGuiOverlayAddition(this);
   }

   public ReferenceFrame getReferenceFrame()
   {
      return representativeReferenceFrame;
   }

   public RigidBodyTransform getTransformToParentToModify()
   {
      return transformToParent;
   }

   public void setPose(RigidBodyTransform transformToParent)
   {
      this.transformToParent.set(transformToParent);
      representativeReferenceFrame.update();
   }

   public void setExtendedContextMenu(Runnable runnable)
   {
      this.extendedContextMenu = runnable;
   }

   public RDXPose3DGizmo getPoseGizmo()
   {
      return selectablePose3DGizmo.getPoseGizmo();
   }

   public RDXModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public boolean isSelected()
   {
      return selectablePose3DGizmo.isSelected();
   }

   public void setSelected(boolean selected)
   {
      selectablePose3DGizmo.setSelected(selected);
   }

   public void setShowing(boolean showing)
   {
      this.showing = showing;
   }

   public boolean isShowing()
   {
      return showing;
   }
}