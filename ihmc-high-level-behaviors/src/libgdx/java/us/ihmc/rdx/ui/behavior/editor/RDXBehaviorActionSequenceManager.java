package us.ihmc.rdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImString;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.io.WorkspaceFile;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.util.Comparator;
import java.util.List;
import java.util.TreeSet;

public class RDXBehaviorActionSequenceManager
{
   private ImGuiPanel managerPanel = new ImGuiPanel("Behavior Sequence Manager", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private WorkspaceResourceDirectory behaviorSequenceStorageDirectory;
   private RDX3DPanel panel3D;
   private DRCRobotModel robotModel;
   private ROS2Node ros2Node;
   private ROS2SyncedRobotModel syncedRobot;
   private List<ReferenceFrame> referenceFrameLibrary;
   private final ImString newSequenceName = new ImString(256);
   private final TreeSet<RDXBehaviorActionSequenceEditor> editors = new TreeSet<>(Comparator.comparing(RDXBehaviorActionSequenceEditor::getName));

   public void create(WorkspaceResourceDirectory behaviorSequenceStorageDirectory,
                      RDX3DPanel panel3D,
                      DRCRobotModel robotModel,
                      ROS2Node ros2Node,
                      ROS2SyncedRobotModel syncedRobot,
                      List<ReferenceFrame> referenceFrameLibrary)
   {
      this.behaviorSequenceStorageDirectory = behaviorSequenceStorageDirectory;
      this.panel3D = panel3D;
      this.robotModel = robotModel;
      this.ros2Node = ros2Node;
      this.syncedRobot = syncedRobot;
      this.referenceFrameLibrary = referenceFrameLibrary;
      reindexSequences();
   }

   public void update()
   {
      for (RDXBehaviorActionSequenceEditor editor : editors)
      {
         editor.update();
      }
   }

   private void renderImGuiWidgets()
   {
      boolean reindexClicked = ImGui.button(labels.get("Reindex sequence files"));
      if (reindexClicked)
      {
         reindexSequences();
      }

      for (RDXBehaviorActionSequenceEditor editor : editors)
      {
         ImGui.checkbox(labels.get(editor.getName()), editor.getPanel().getIsShowing());
      }

      ImGuiTools.inputText(labels.getHidden("newSequenceName"), newSequenceName);
      ImGui.sameLine();
      if (ImGui.button("Create new sequence"))
      {
         RDXBehaviorActionSequenceEditor editor = new RDXBehaviorActionSequenceEditor(newSequenceName.get(), behaviorSequenceStorageDirectory);
         editor.saveToFile();
         addEditor(editor);
      }
   }

   private void reindexSequences()
   {
      for (WorkspaceResourceFile queryContainedFile : behaviorSequenceStorageDirectory.queryContainedFiles())
      {
         boolean alreadyLoaded = false;
         for (RDXBehaviorActionSequenceEditor editor : editors)
            alreadyLoaded |= editor.getWorkspaceFile().getFileName().equals(queryContainedFile.getFileName());

         if (!alreadyLoaded)
         {
            RDXBehaviorActionSequenceEditor editor = new RDXBehaviorActionSequenceEditor(queryContainedFile);
            addEditor(editor);
            editor.loadActionsFromFile();
         }
      }
   }

   private void addEditor(RDXBehaviorActionSequenceEditor editor)
   {
      editor.create(panel3D, robotModel, ros2Node, syncedRobot, referenceFrameLibrary);
      editors.add(editor);
      managerPanel.queueAddChild(editor.getPanel());
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      for (RDXBehaviorActionSequenceEditor editor : editors)
      {
         editor.calculate3DViewPick(input);
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      for (RDXBehaviorActionSequenceEditor editor : editors)
      {
         editor.process3DViewInput(input);
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (RDXBehaviorActionSequenceEditor editor : editors)
      {
         editor.getRenderables(renderables, pool);
      }
   }

   public ImGuiPanel getManagerPanel()
   {
      return managerPanel;
   }
}
