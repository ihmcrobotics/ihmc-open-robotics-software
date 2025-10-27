package us.ihmc.rdx.behaviorTree.scene;

import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneState;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDXBaseUI;

import java.util.ArrayList;
import java.util.List;

public class RDXBehaviorTreeScene extends BehaviorTreeSceneState
{
   private final RDXBaseUI baseUI;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RDXPanel panel = new RDXPanel("Scene", this::renderImGuiWidgets);
   private final ImBoolean mustard = new ImBoolean(false);

   private final List<RDXBehaviorTreeSceneObject> objects = new ArrayList<>();

   private boolean needToInitializePlacementHeight = false;
   private RDXBehaviorTreeSceneObject beingPlaced;

   public RDXBehaviorTreeScene(ROS2SyncedRobotModel syncedRobot, RDXBaseUI baseUI, RDXPanel parentPanel)
   {
      super(syncedRobot);

      this.baseUI = baseUI;

      parentPanel.addChild(panel);

      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::processImGui3DViewInput);
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
      // place mustard in scene?

      if (ImGui.checkbox(labels.get("Mustard"), mustard))
      {
         if (mustard.get())
         {
            beingPlaced = new RDXBehaviorTreeSceneObject("environmentObjects/mustard/mustard.glb", baseUI);
            objects.add(beingPlaced);
            needToInitializePlacementHeight = true;
         }
         else
         {
            if (!objects.isEmpty())
            {
               RDXBehaviorTreeSceneObject remove = objects.remove(0);
               remove.destroy();
            }
            // TODO Remove mustard
         }
      }
   }
}
