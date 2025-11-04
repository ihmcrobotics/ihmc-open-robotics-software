package us.ihmc.rdx.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage;
import behavior_msgs.msg.dds.PersistentDetectionStatusMessage;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectState;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;
import us.ihmc.rdx.behaviorTree.RDXCRDTTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;

import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.function.LongSupplier;

public class RDXBehaviorTreeScene extends BehaviorTreeSceneState
{
   private final RDXBaseUI baseUI;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RDXPanel panel = new RDXPanel("Scene", this::renderImGuiWidgets);

   private final List<RDXBehaviorTreeSceneObject> objects;
   private final RecyclingArrayList<PersistentDetectionStatusMessage> persistentDetections = new RecyclingArrayList<>(PersistentDetectionStatusMessage::new);
   private final Random random = new Random();
   private final RecyclingArrayList<ModelInstance> detectionFrameGraphics = new RecyclingArrayList<>(() ->
                    RDXModelBuilder.createCoordinateFrameInstance(0.2, LibGDXTools.toLibGDX(YoAppearance.randomColor(random))));

   private boolean needToInitializePlacementHeight = false;
   private RDXBehaviorTreeSceneObject beingPlaced;

   public RDXBehaviorTreeScene(CRDTInfo crdtInfo, LongSupplier idSupplier, ROS2SyncedRobotModel syncedRobot, RDXBaseUI baseUI, RDXPanel parentPanel)
   {
      super(crdtInfo, idSupplier, syncedRobot);

      this.baseUI = baseUI;

      this.objects = (List) super.objects;

      parentPanel.addChild(panel);

      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::processImGui3DViewInput);
      baseUI.getPrimary3DPanel().getScene().addRenderableProvider(this::getRenderables);
   }

   public void update()
   {
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
      ImGui.text("Add virtual:");
      ImGui.indent();
      if (ImGuiTools.textWithUnderlineOnHover("Mustard") && ImGui.isMouseClicked(ImGuiMouseButton.Left))
      {
         beingPlaced = new RDXBehaviorTreeSceneObject(idSupplier.getAsLong(), crdtInfo, IsaacROSFoundationPoseObject.MUSTARD.name(), baseUI);
         objects.add(beingPlaced);
         objectsModifiable.modify();
         needToInitializePlacementHeight = true;
      }
      ImGui.unindent();

      ImGui.text("Objects:");
      ImGui.indent();
      RDXBehaviorTreeSceneObject remove = null;
      for (int i = 0; i < objects.size(); i++)
      {
         RDXBehaviorTreeSceneObject object = objects.get(i);
         if (ImGui.checkbox(labels.getHidden("Select%s%d".formatted(object.getName(), object.getID())), object.getGizmo().isSelected()))
            object.getGizmo().setSelected(!object.getGizmo().isSelected());
         ImGui.sameLine();
         ImGui.text("%s ID: %d".formatted(object.getName(), object.getID()));
         ImGui.sameLine();
         ImGui.pushStyleColor(ImGuiCol.Button, ImGuiTools.DARK_RED);
         if (ImGui.button(labels.get("X", i)))
            remove = object;
         ImGui.popStyleColor();
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
      for (PersistentDetectionStatusMessage persistentDetection : persistentDetections)
      {
         String text = "%s %.2f Hz Size: %d".formatted(persistentDetection.getObjectClassAsString(),
                                                       persistentDetection.getDecayingFrequency(),
                                                       persistentDetection.getHistorySize());
         if (persistentDetection.getIsStable())
            ImGui.text(text);
         else
            ImGui.textDisabled(text);
      }
      ImGui.unindent();
   }

   private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      for (ModelInstance graphic : detectionFrameGraphics)
         graphic.getRenderables(renderables, pool);
   }

   @Override
   protected BehaviorTreeSceneObjectState buildObject(BehaviorTreeSceneObjectStateMessage message)
   {
      return new RDXBehaviorTreeSceneObject(message.getId(), crdtInfo, message.getTypeAsString(), baseUI);
   }

   @Override
   public void fromMessage(BehaviorTreeSceneStateMessage message)
   {
      super.fromMessage(message);

      persistentDetections.clear();
      detectionFrameGraphics.clear();
      for (int i = 0; i < message.getPersistentDetections().size(); i++)
      {
         PersistentDetectionStatusMessage status = message.getPersistentDetections().get(i);
         persistentDetections.add().set(status);
         ModelInstance graphic = detectionFrameGraphics.add();
         RDXCRDTTools.toLibGDX(status.getTransformToWorld(), graphic.transform);
      }
   }
}
