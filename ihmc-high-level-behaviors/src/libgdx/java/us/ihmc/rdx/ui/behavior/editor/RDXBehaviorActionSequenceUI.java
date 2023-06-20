package us.ihmc.rdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImString;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.util.ArrayList;
import java.util.Comparator;

/**
 * This panel renders a list of available action sequences found as JSON on disk,
 * allows you to create new ones, and select one as active, which will show its
 * panel and allow you to interact with it.
 *
 * For example, this will show a list of:
 * - Push door (simulation)
 * - Pull door (simulation)
 * - Push door (real robot)
 * - Pick up box marker ID 3
 * - etc...
 */
public class RDXBehaviorActionSequenceUI
{
   private final ImGuiPanel managerPanel = new ImGuiPanel("Behavior Sequence Manager", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private WorkspaceResourceDirectory behaviorSequenceStorageDirectory;
   private RDX3DPanel panel3D;
   private DRCRobotModel robotModel;
   private ROS2Node ros2Node;
   private ROS2SyncedRobotModel syncedRobot;
   private RobotCollisionModel selectionCollisionModel;
   private ReferenceFrameLibrary referenceFrameLibrary;
   private final ImString newSequenceName = new ImString(256);
   private final ArrayList<RDXAvailableActionSequence> availableSequences = new ArrayList<>();
   private RDXBehaviorActionSequenceEditor selectedEditor = null;

   public void create(WorkspaceResourceDirectory behaviorSequenceStorageDirectory,
                      RDX3DPanel panel3D,
                      DRCRobotModel robotModel,
                      ROS2Node ros2Node,
                      ROS2SyncedRobotModel syncedRobot,
                      RobotCollisionModel selectionCollisionModel,
                      ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.behaviorSequenceStorageDirectory = behaviorSequenceStorageDirectory;
      this.panel3D = panel3D;
      this.robotModel = robotModel;
      this.ros2Node = ros2Node;
      this.syncedRobot = syncedRobot;
      this.selectionCollisionModel = selectionCollisionModel;
      this.referenceFrameLibrary = referenceFrameLibrary;

      BehaviorActionSequence.addCommonFrames(referenceFrameLibrary, syncedRobot);
      referenceFrameLibrary.build();

      reindexSequences();
   }

   public void update()
   {
      if (selectedEditor != null)
         selectedEditor.update();
   }

   private void renderImGuiWidgets()
   {
      boolean reindexClicked = ImGui.button(labels.get("Reindex sequence files"));
      if (reindexClicked)
         reindexSequences();

      if (ImGui.radioButton(labels.get("None"), selectedEditor == null))
      {
         if (selectedEditor != null)
         {
            managerPanel.queueRemoveChild(selectedEditor.getPanel());
            selectedEditor = null;
         }
      }
      for (RDXAvailableActionSequence availableSequenceFile : availableSequences)
      {
         if (ImGui.radioButton(labels.get(availableSequenceFile.getName()),
                               selectedEditor != null
                               && selectedEditor.getWorkspaceFile().getFileName().equals(availableSequenceFile.getSequenceFile().getFileName())))
         {
            if (selectedEditor != null)
               managerPanel.queueRemoveChild(selectedEditor.getPanel());

            selectedEditor = new RDXBehaviorActionSequenceEditor(availableSequenceFile.getSequenceFile());
            selectedEditor.create(panel3D, robotModel, ros2Node, syncedRobot, selectionCollisionModel, referenceFrameLibrary);
            selectedEditor.loadActionsFromFile();
            managerPanel.queueAddChild(selectedEditor.getPanel());
         }
      }

      ImGuiTools.inputText(labels.getHidden("newSequenceName"), newSequenceName);
      ImGui.sameLine();
      if (ImGui.button("Create new sequence"))
      {
         selectedEditor = new RDXBehaviorActionSequenceEditor(newSequenceName.get(), behaviorSequenceStorageDirectory);
         selectedEditor.create(panel3D, robotModel, ros2Node, syncedRobot, selectionCollisionModel, referenceFrameLibrary);
         selectedEditor.saveToFile();
         managerPanel.queueAddChild(selectedEditor.getPanel());
         availableSequences.add(new RDXAvailableActionSequence(selectedEditor.getWorkspaceFile()));
      }
   }

   private void reindexSequences()
   {
      availableSequences.clear();
      for (WorkspaceResourceFile queryContainedFile : behaviorSequenceStorageDirectory.queryContainedFiles())
      {
         availableSequences.add(new RDXAvailableActionSequence(queryContainedFile));
      }

      // Keep them in alphabetical order
      availableSequences.sort(Comparator.comparing(RDXAvailableActionSequence::getName));
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (selectedEditor != null)
         selectedEditor.calculate3DViewPick(input);
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (selectedEditor != null)
         selectedEditor.process3DViewInput(input);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (selectedEditor != null)
         selectedEditor.getRenderables(renderables, pool);
   }

   public ImGuiPanel getManagerPanel()
   {
      return managerPanel;
   }
}
