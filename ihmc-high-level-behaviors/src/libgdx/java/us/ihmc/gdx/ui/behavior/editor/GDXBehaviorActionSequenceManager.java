package us.ihmc.gdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImString;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;

import java.util.Comparator;
import java.util.TreeSet;

public class GDXBehaviorActionSequenceManager
{
   private ImGuiPanel managerPanel = new ImGuiPanel("Behavior Sequence Manager", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private WorkspaceDirectory behaviorSequenceStorageDirectory;
   private boolean indexesFilesOnce = false;
   private final ImString newSequenceName = new ImString("", 100);
   private final TreeSet<GDXBehaviorActionSequenceEditor> editors = new TreeSet<>(Comparator.comparing(GDXBehaviorActionSequenceEditor::getName));

   public void create(WorkspaceDirectory behaviorSequenceStorageDirectory)
   {
      this.behaviorSequenceStorageDirectory = behaviorSequenceStorageDirectory;
   }

   private void renderImGuiWidgets()
   {
      boolean reindexClicked = ImGui.button(labels.get("Reindex sequence files"));
      if (!indexesFilesOnce || reindexClicked)
      {
         indexesFilesOnce = true;
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
         }
      }
   }

   private void addEditor(GDXBehaviorActionSequenceEditor editor)
   {
      editors.add(editor);
      managerPanel.queueAddChild(editor.getPanel());
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
