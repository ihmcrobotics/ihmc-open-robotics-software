package us.ihmc.rdx.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage;
import behavior_msgs.msg.dds.PersistentDetectionStatusMessage;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectState;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectType;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseInstantDetection;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;
import us.ihmc.rdx.RDX3DSituatedText;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;

import java.util.List;
import java.util.Set;
import java.util.function.LongSupplier;

public class RDXBehaviorTreeScene extends BehaviorTreeSceneState
{
   private final RDXBaseUI baseUI;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RDXPanel panel = new RDXPanel("Scene", this::renderImGuiWidgets);

   private final List<RDXBehaviorTreeSceneObject> objects;
   private final ImBoolean showDetections = new ImBoolean(true);
   private final RecyclingArrayList<RDXBehaviorTreeSceneDetection> persistentDetections;

   private final ImBoolean showCameraFrame = new ImBoolean(false);
   private final RigidBodyTransform cameraGraphicTransform = new RigidBodyTransform();
   private final ModelInstance cameraFrameGraphic = RDXModelBuilder.createCoordinateFrameInstance(0.2);
   private final RDX3DSituatedText cameraText = new RDX3DSituatedText();

   private boolean needToInitializePlacementHeight = false;
   private RDXBehaviorTreeSceneObject beingPlaced;

   public RDXBehaviorTreeScene(CRDTInfo crdtInfo, LongSupplier idSupplier, ROS2SyncedRobotModel syncedRobot, RDXBaseUI baseUI, RDXPanel parentPanel)
   {
      super(crdtInfo, idSupplier, syncedRobot);

      this.baseUI = baseUI;

      this.objects = (List) super.objects;

      persistentDetections = new RecyclingArrayList<>(() -> new RDXBehaviorTreeSceneDetection(baseUI));

      parentPanel.addChild(panel);

      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::processImGui3DViewInput);
      baseUI.getPrimary3DPanel().getScene().addRenderableProvider(this::getRenderables);
   }

   public void update()
   {
      if (showCameraFrame.get())
      {
         syncedRobot.getReferenceFrames().getExperimentalCameraFrame().getTransformToDesiredFrame(cameraGraphicTransform, ReferenceFrame.getWorldFrame());
         LibGDXTools.toLibGDX(cameraGraphicTransform, cameraFrameGraphic.transform);
         cameraText.setTextWithoutCache("Experimental Camera");
         cameraText.setPositionFacingCamera(baseUI.getPrimary3DPanel().getCamera3D(), cameraGraphicTransform.getTranslation());
      }

      for (RDXBehaviorTreeSceneObject object : objects)
         object.update();
   }

   private void processImGui3DViewInput(ImGui3DViewInput input)
   {
      if (beingPlaced != null)
      {
         if (needToInitializePlacementHeight) // Initialize placed height until scene collisions take back over
         {
            needToInitializePlacementHeight = false;
            input.setLastZCollision(syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame).getZ());
         }
      }
   }

   private void renderImGuiWidgets()
   {
      ImGuiTools.smallCheckbox(labels.get("Show camera frame"), showCameraFrame);
      ImGui.sameLine();
      ImGuiTools.smallCheckbox(labels.get("Show detections"), showDetections);
      ImGui.separator();
      ImGui.text("Add virtual:");
      ImGui.indent();
      for (IsaacROSFoundationPoseObject objectType : IsaacROSFoundationPoseObject.values())
      {
         if (ImGuiTools.textWithUnderlineOnHover(objectType.titleCaseName) && ImGui.isMouseClicked(ImGuiMouseButton.Left))
         {
            BehaviorTreeSceneObjectDefinitionMessage message = new BehaviorTreeSceneObjectDefinitionMessage();
            message.setObjectType((byte) BehaviorTreeSceneObjectType.FOUNDATION_POSE.ordinal());
            message.setFoundationPoseObjectType((byte) objectType.ordinal());
            beingPlaced = (RDXBehaviorTreeSceneObject) createObject(message);
            objects.add(beingPlaced);
            objectsModifiable.modify();
            needToInitializePlacementHeight = true;
         }
      }
      ImGui.unindent();
      ImGui.separator();

      ImGui.text("Objects:");
      ImGui.indent();
      RDXBehaviorTreeSceneObject remove = null;
      for (int i = 0; i < objects.size(); i++)
      {
         RDXBehaviorTreeSceneObject object = objects.get(i);
         ImGui.text("%s %d".formatted(object.getName(), object.getID()));
         ImGui.sameLine();
         if (ImGuiTools.smallCheckbox(labels.getHidden("Select%s%d".formatted(object.getName(), object.getID())), object.getGizmo().isSelected()))
            object.getGizmo().setSelected(!object.getGizmo().isSelected());
         ImGui.sameLine();
         ImGui.pushStyleColor(ImGuiCol.Text, ImGuiTools.DARK_RED);
         if (ImGuiTools.textWithUnderlineOnHover("X") && ImGui.isMouseClicked(ImGuiMouseButton.Left))
            remove = object;
         ImGui.popStyleColor();
         ImGui.indent();
         renderPersistentDetection(object.getPersistentDetection());
         ImGui.unindent();
      }
      ImGui.unindent();

      if (remove != null)
      {
         objects.remove(remove);
         objectsModifiable.modify();
         remove.destroy();
      }

      ImGui.text("Stable Detections:");
      ImGui.indent();
      for (RDXBehaviorTreeSceneDetection persistentDetection : persistentDetections)
         renderPersistentDetection(persistentDetection.getMessage());
      ImGui.unindent();
   }

   private static void renderPersistentDetection(PersistentDetectionStatusMessage message)
   {
      String type = "(?)";
      if (message.getDetectionTypeAsString().equals(IsaacROSFoundationPoseInstantDetection.class.getSimpleName()))
         type = "(FoundationPose)";
      if (message.getDetectionTypeAsString().equals(YOLOv8InstantDetection.class.getSimpleName()))
         type = "(YOLOv8)";
      String text = "%s %s %.2f Hz Size: %d ID.%s".formatted(type,
                                                             message.getObjectClassAsString(),
                                                             message.getDecayingFrequency(),
                                                             message.getHistorySize(),
                                                             message.getIdAsString());
      if (message.getIsStable())
         ImGui.text(text);
      else
         ImGui.textDisabled(text);

      ImGui.indent();
      RigidBodyTransform transform = new RigidBodyTransform();
      MessageTools.toEuclid(message.getTransformToCamera(), transform);
      ImGui.text("To camera: (%.2f, %.2f, %.2f) YPR: (%.2f, %.2f, %.2f)".formatted(
            transform.getTranslationX(), transform.getTranslation().getY(), transform.getTranslation().getZ(),
            transform.getRotation().getYaw(), transform.getRotation().getPitch(), transform.getRotation().getRoll()
      ));
      ImGui.unindent();
   }

   private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (!sceneLevels.contains(RDXSceneLevel.VIRTUAL))
         return;

      if (showCameraFrame.get())
      {
         cameraFrameGraphic.getRenderables(renderables, pool);
         cameraText.getRenderables(renderables, pool);
      }

      if (showDetections.get())
      {
         for (RDXBehaviorTreeSceneDetection detection : persistentDetections)
            detection.getRenderables(renderables, pool);
      }

      for (RDXBehaviorTreeSceneObject object : objects)
         object.getRenderables(renderables, pool);
   }

   @Override
   protected BehaviorTreeSceneObjectState buildObject(long id, CRDTInfo crdtInfo, BehaviorTreeSceneObjectDefinitionMessage definition)
   {
      if (definition.getObjectType() == BehaviorTreeSceneObjectType.DOOR_PANEL.ordinal())
         return new RDXBehaviorTreeSceneDoorPanel(id, crdtInfo, definition, baseUI);
      else
         return new RDXBehaviorTreeSceneObject(id, crdtInfo, definition, baseUI);
   }

   @Override
   public void fromMessage(BehaviorTreeSceneStateMessage message)
   {
      super.fromMessage(message);

      persistentDetections.clear();
      for (int i = 0; i < message.getPersistentDetections().size(); i++)
         persistentDetections.add().update(message.getPersistentDetections().get(i));
   }
}
