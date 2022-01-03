package us.ihmc.gdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import org.apache.commons.lang3.tuple.MutablePair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;

import java.util.LinkedList;

public class GDXBehaviorActionSequenceEditor
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final LinkedList<GDXBehaviorAction> actionSequence = new LinkedList<>();
   private FocusBasedGDXCamera camera3D;
   private DRCRobotModel robotModel;
   private int playbackNextIndex = 0;
   private FootstepPlanningModule footstepPlanner;
   private ROS2SyncedRobotModel syncedRobot;
   private ROS2ControllerHelper ros2ControllerHelper;
   private final MutablePair<Integer, Integer> reorderRequest = MutablePair.of(-1, 0);

   public void create(FocusBasedGDXCamera camera3D, DRCRobotModel robotModel, ROS2Node ros2Node, ROS2SyncedRobotModel syncedRobot)
   {
      this.camera3D = camera3D;
      this.robotModel = robotModel;
      footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel);
      this.syncedRobot = syncedRobot;
      ros2ControllerHelper = new ROS2ControllerHelper(ros2Node, robotModel);
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      for (GDXBehaviorAction action : actionSequence)
      {
         action.process3DViewInput(input);
      }
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.button(labels.get("<")))
      {
         if (playbackNextIndex > 0)
            playbackNextIndex--;
      }
      ImGui.sameLine();
      if (playbackNextIndex < actionSequence.size())
      {
         ImGui.text("Index: " + playbackNextIndex);
         ImGui.sameLine();
         if (ImGui.button(labels.get("Execute")))
         {
            GDXBehaviorAction action = actionSequence.get(playbackNextIndex);
            if (action instanceof GDXWalkAction)
            {
               ((GDXWalkAction) action).walk(ReferenceFrame.getWorldFrame(), ros2ControllerHelper, syncedRobot);
            }
            else if (action instanceof GDXHandPoseAction)
            {
               ((GDXHandPoseAction) action).moveHand(4.0);
            }
            playbackNextIndex++;
         }
      }
      else
      {
         ImGui.text("No actions left.");
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get(">")))
      {
         if (playbackNextIndex < actionSequence.size())
            playbackNextIndex++;
      }

      ImGui.separator();

      reorderRequest.setLeft(-1);
      for (int i = 0; i < actionSequence.size(); i++)
      {
         GDXBehaviorAction action = actionSequence.get(i);
         ImGui.checkbox(labels.get("", "Selected", i), action.getSelected());
         ImGui.sameLine();
         ImGui.text(i + ": " + action.getNameForDisplay());
         ImGui.sameLine();
         if (i > 0)
         {
            if (ImGui.button(labels.get("^", i)))
            {
               reorderRequest.setLeft(i);
               reorderRequest.setRight(0);
            }
            ImGui.sameLine();
         }
         if (i < actionSequence.size() - 1)
         {
            if (ImGui.button(labels.get("v", i)))
            {
               reorderRequest.setLeft(i);
               reorderRequest.setRight(1);
            }
            ImGui.sameLine();
         }
         if (ImGui.button(labels.get("X", i)))
         {
            GDXBehaviorAction removedAction = actionSequence.remove(i);
//            removedAction.destroy();
         }
      }

      int indexToMove = reorderRequest.getLeft();
      if (indexToMove > -1)
      {
         int destinationIndex = reorderRequest.getRight() == 0 ? indexToMove - 1 : indexToMove + 1;
         actionSequence.add(destinationIndex, actionSequence.remove(indexToMove));
      }

      if (ImGui.button(labels.get("Add Walk")))
      {
         GDXWalkAction walkAction = new GDXWalkAction();
         walkAction.create(camera3D, robotModel, footstepPlanner);
         actionSequence.addLast(walkAction);
      }
      ImGui.text("Add Hand Pose");
      ImGui.sameLine();
      for (RobotSide side : RobotSide.values)
      {
         if (ImGui.button(labels.get(side.getPascalCaseName())))
         {
            GDXHandPoseAction handPoseAction = new GDXHandPoseAction();
            handPoseAction.create(camera3D, robotModel, syncedRobot.getFullRobotModel(), side, ros2ControllerHelper);
            actionSequence.addLast(handPoseAction);
         }
         if (side.ordinal() < 1)
            ImGui.sameLine();
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (GDXBehaviorAction action : actionSequence)
      {
         action.getRenderables(renderables, pool);
      }
   }
}
