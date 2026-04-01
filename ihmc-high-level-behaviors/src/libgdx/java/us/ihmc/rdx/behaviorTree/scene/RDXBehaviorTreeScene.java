package us.ihmc.rdx.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage;
import behavior_msgs.msg.dds.PersistentDetectionStatusMessage;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
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
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.sceneManager.RDXRenderableAdapter;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;

import java.util.Iterator;
import java.util.List;
import java.util.Set;
import java.util.function.LongSupplier;

public class RDXBehaviorTreeScene extends BehaviorTreeSceneState
{
   private final RDXBaseUI baseUI;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final List<RDXBehaviorTreeSceneObject> objects;
   private final ImBoolean showDetections = new ImBoolean(true);
   private final ImBoolean createFoundationPose = new ImBoolean(false);
   private final RecyclingArrayList<RDXBehaviorTreeSceneDetection> persistentDetections;
   private final RDXRenderableAdapter renderableAdapter;

   private final ImBoolean showCameraFrame = new ImBoolean(false);
   private final RigidBodyTransform cameraGraphicTransform = new RigidBodyTransform();
   private final ModelInstance cameraFrameGraphic = RDXModelBuilder.createCoordinateFrameInstance(0.2);
   private final RDX3DSituatedText cameraText = new RDX3DSituatedText();

   private boolean needToInitializePlacementHeight = false;
   private RDXBehaviorTreeSceneObject beingPlaced;

   public RDXBehaviorTreeScene(CRDTInfo crdtInfo, LongSupplier idSupplier, ROS2SyncedRobotModel syncedRobot, RDXBaseUI baseUI)
   {
      super(crdtInfo, idSupplier, syncedRobot);

      this.baseUI = baseUI;

      this.objects = (List) super.objects;

      persistentDetections = new RecyclingArrayList<>(() -> new RDXBehaviorTreeSceneDetection(baseUI));

      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this, this::processImGui3DViewInput);
      renderableAdapter = baseUI.getPrimary3DPanel().getScene().addRenderableProvider(this::getRenderables);
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

   public void renderImGuiWidgets()
   {
      ImGuiTools.smallCheckbox(labels.get("Show camera frame"), showCameraFrame);
      ImGui.sameLine();
      ImGuiTools.smallCheckbox(labels.get("Show detections"), showDetections);
      ImGui.separator();
      ImGui.text("Add virtual:");
      ImGui.sameLine();
      ImGuiTools.smallCheckbox(labels.get("FoundationPose"), createFoundationPose);
      ImGui.indent();
      BehaviorTreeSceneObjectDefinitionMessage objectDefinition = null;
      for (IsaacROSFoundationPoseObject objectType : IsaacROSFoundationPoseObject.values())
      {
         if (ImGuiTools.textWithUnderlineOnHover(objectType.titleCaseName) && ImGui.isMouseClicked(ImGuiMouseButton.Left))
         {
            objectDefinition = new BehaviorTreeSceneObjectDefinitionMessage();
            if (createFoundationPose.get())
            {
               objectDefinition.setObjectType((byte) BehaviorTreeSceneObjectType.FOUNDATION_POSE.ordinal());
               objectDefinition.setFoundationPoseObjectType((byte) objectType.ordinal());
            }
            else
               objectDefinition.setObjectType((byte) BehaviorTreeSceneObjectType.YOLO_ONLY.ordinal());
            objectDefinition.setYoloClassName(objectType.yoloClass);
         }
      }
      if (ImGuiTools.textWithUnderlineOnHover("Person") && ImGui.isMouseClicked(ImGuiMouseButton.Left))
      {
         objectDefinition = new BehaviorTreeSceneObjectDefinitionMessage();
         objectDefinition.setObjectType((byte) BehaviorTreeSceneObjectType.YOLO_ONLY.ordinal());
         objectDefinition.setYoloClassName("person");
      }
      if (objectDefinition != null)
      {
         beingPlaced = (RDXBehaviorTreeSceneObject) createObject(objectDefinition);
         objects.add(beingPlaced);
         objectsModifiable.modify();
         needToInitializePlacementHeight = true;
      }
      ImGui.unindent();
      ImGui.separator();

      ImGui.text("Objects:");
      ImGui.indent();
      RDXBehaviorTreeSceneObject remove = null;

      Iterator<RDXBehaviorTreeSceneObject> iterator = objects.iterator();
      while (iterator.hasNext())
      {
         RDXBehaviorTreeSceneObject object = iterator.next();
         if (object.renderImGuiWidgets())
         {
            iterator.remove();
            objectsModifiable.modify();
            object.destroy();
         }
      }

      ImGui.unindent();

      ImGui.text("Stable Detections:");
      ImGui.indent();
      for (RDXBehaviorTreeSceneDetection persistentDetection : persistentDetections)
         renderPersistentDetection(persistentDetection.getMessage());
      ImGui.unindent();
   }

   static void renderPersistentDetection(PersistentDetectionStatusMessage message)
   {
      if (message.getDetectionTypeAsString().isEmpty())
         return;

      String type = "(?)";
      if (message.getDetectionTypeAsString().equals(IsaacROSFoundationPoseInstantDetection.class.getSimpleName()))
         type = "(FoundationPose)";
      else if (message.getDetectionTypeAsString().equals(YOLOv8InstantDetection.class.getSimpleName()))
         type = "(YOLOv8)";
      ImGui.beginDisabled(!message.getIsStable());
      ImGui.text("%s %s %.2f Hz Size: %d ID.%d".formatted(type,
                                                          message.getObjectClassAsString(),
                                                          message.getDecayingFrequency(),
                                                          message.getHistorySize(),
                                                          message.getId()));
      ImGui.indent();
      RigidBodyTransform transform = new RigidBodyTransform();
      MessageTools.toEuclid(message.getTransformToCamera(), transform);
      ImGui.text("To camera: (%.2f, %.2f, %.2f) YPR: (%.2f, %.2f, %.2f)".formatted(
            transform.getTranslationX(), transform.getTranslation().getY(), transform.getTranslation().getZ(),
            transform.getRotation().getYaw(), transform.getRotation().getPitch(), transform.getRotation().getRoll()
      ));
      ImGui.endDisabled();
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
      return switch (BehaviorTreeSceneObjectType.values()[definition.getObjectType()])
      {
         case DOOR_PANEL -> new RDXBehaviorTreeSceneDoorPanel(id, crdtInfo, definition, baseUI);
         case DOOR_FRAME ->
         {
            RDXBehaviorTreeSceneDoorFrame doorFrame = new RDXBehaviorTreeSceneDoorFrame(id, crdtInfo, definition, baseUI);
            RDXBehaviorTreeSceneDoorPanel doorPanel = (RDXBehaviorTreeSceneDoorPanel) getObject(BehaviorTreeSceneObjectType.DOOR_PANEL);
            doorFrame.setDoorPanel(doorPanel);
            yield doorFrame;
         }
         case CUSTOM_FRAME -> new RDXBehaviorTreeSceneCustomFrame(id, crdtInfo, definition, baseUI);
         default -> new RDXBehaviorTreeSceneObject(id, crdtInfo, definition, baseUI);
      };
   }

   @Override
   public void fromMessage(BehaviorTreeSceneStateMessage message)
   {
      super.fromMessage(message);

      persistentDetections.clear();
      for (int i = 0; i < message.getPersistentDetections().size(); i++)
         persistentDetections.add().update(message.getPersistentDetections().get(i));
   }

   public void destroy()
   {
      baseUI.getPrimary3DPanel().removeImGui3DViewInputProcessor(this);
      baseUI.getPrimary3DPanel().getScene().removeRenderableAdapter(renderableAdapter);

      for (RDXBehaviorTreeSceneObject object : objects)
         object.destroy();
   }
}
