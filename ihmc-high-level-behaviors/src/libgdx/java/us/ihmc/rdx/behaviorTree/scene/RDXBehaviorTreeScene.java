package us.ihmc.rdx.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage;
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
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
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
   private final RecyclingArrayList<RDXBehaviorTreeSceneDetection> persistentDetections;

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
         beingPlaced = (RDXBehaviorTreeSceneObject) createObject(IsaacROSFoundationPoseObject.MUSTARD);
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
         ImGui.indent();
         ImGui.text("Persistent detection: %s".formatted(object.getPersistentDetectionID()));
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
      {
         String text = "%s %.2f Hz Size: %d ID: %s".formatted(persistentDetection.getMessage().getObjectClassAsString(),
                                                              persistentDetection.getMessage().getDecayingFrequency(),
                                                              persistentDetection.getMessage().getHistorySize(),
                                                              persistentDetection.getMessage().getIdAsString());
         if (persistentDetection.getMessage().getIsStable())
            ImGui.text(text);
         else
            ImGui.textDisabled(text);
      }
      ImGui.unindent();
   }

   private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      for (RDXBehaviorTreeSceneDetection detection : persistentDetections)
         detection.getRenderables(renderables, pool);
   }

   @Override
   protected BehaviorTreeSceneObjectState buildObject(long id, CRDTInfo crdtInfo, IsaacROSFoundationPoseObject objectType)
   {
      return new RDXBehaviorTreeSceneObject(id, crdtInfo, objectType, baseUI);
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
