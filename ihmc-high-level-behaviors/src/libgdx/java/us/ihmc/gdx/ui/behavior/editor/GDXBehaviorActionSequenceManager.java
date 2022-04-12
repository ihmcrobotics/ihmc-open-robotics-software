package us.ihmc.gdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImString;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.GDXFocusBasedCamera;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;

import java.util.Comparator;
import java.util.List;
import java.util.TreeSet;

public class GDXBehaviorActionSequenceManager
{
   private ImGuiPanel managerPanel = new ImGuiPanel("Behavior Sequence Manager", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private WorkspaceDirectory behaviorSequenceStorageDirectory;
   private GDXFocusBasedCamera camera3D;
   private DRCRobotModel robotModel;
   private ROS2Node ros2Node;
   private ROS2SyncedRobotModel syncedRobot;
   private List<ReferenceFrame> referenceFrameLibrary;
   private final ImString newSequenceName = new ImString("", 100);
   private final TreeSet<GDXBehaviorActionSequenceEditor> editors = new TreeSet<>(Comparator.comparing(GDXBehaviorActionSequenceEditor::getName));

   public void create(WorkspaceDirectory behaviorSequenceStorageDirectory,
                      GDXFocusBasedCamera camera3D,
                      DRCRobotModel robotModel,
                      ROS2Node ros2Node,
                      ROS2SyncedRobotModel syncedRobot,
                      List<ReferenceFrame> referenceFrameLibrary)
   {
      this.behaviorSequenceStorageDirectory = behaviorSequenceStorageDirectory;
      this.camera3D = camera3D;
      this.robotModel = robotModel;
      this.ros2Node = ros2Node;
      this.syncedRobot = syncedRobot;
      this.referenceFrameLibrary = referenceFrameLibrary;
      reindexSequences();
   }

   public void update()
   {
      for (GDXBehaviorActionSequenceEditor editor : editors)
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

      for (GDXBehaviorActionSequenceEditor editor : editors)
      {
         ImGui.checkbox(labels.get(editor.getName()), editor.getPanel().getIsShowing());
      }

      ImGuiTools.inputText(labels.getHidden("newSequenceName"), newSequenceName);
      ImGui.sameLine();
      if (ImGui.button("Create new sequence"))
      {
         GDXBehaviorActionSequenceEditor editor = new GDXBehaviorActionSequenceEditor(newSequenceName.get(), behaviorSequenceStorageDirectory);
         editor.saveToFile();
         addEditor(editor);
      }
   }

   private void reindexSequences()
   {
      for (WorkspaceFile queryContainedFile : behaviorSequenceStorageDirectory.queryContainedFiles())
      {
         boolean alreadyLoaded = false;
         for (GDXBehaviorActionSequenceEditor editor : editors)
            alreadyLoaded |= editor.getWorkspaceFile().getResourceName().equals(queryContainedFile.getResourceName());

         if (!alreadyLoaded)
         {
            GDXBehaviorActionSequenceEditor editor = new GDXBehaviorActionSequenceEditor(queryContainedFile);
            addEditor(editor);
            editor.loadActionsFromFile();
         }
      }
   }

   private void addEditor(GDXBehaviorActionSequenceEditor editor)
   {
      editor.create(camera3D, robotModel, ros2Node, syncedRobot, referenceFrameLibrary);
      editors.add(editor);
      managerPanel.queueAddChild(editor.getPanel());
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      for (GDXBehaviorActionSequenceEditor editor : editors)
      {
         editor.process3DViewInput(input);
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (GDXBehaviorActionSequenceEditor editor : editors)
      {
         editor.getRenderables(renderables, pool);
      }
   }

   public ImGuiPanel getManagerPanel()
   {
      return managerPanel;
   }
}
