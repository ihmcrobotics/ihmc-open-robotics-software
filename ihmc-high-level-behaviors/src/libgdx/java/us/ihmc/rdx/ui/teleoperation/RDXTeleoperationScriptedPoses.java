package us.ihmc.rdx.ui.teleoperation;

import imgui.ImGui;
import imgui.type.ImString;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiDirectory;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXTeleoperationScriptedPoses extends RDXPanel
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString saveFileName = new ImString();
   private final WorkspaceResourceDirectory posesDirectory = new WorkspaceResourceDirectory(getClass(), "/poses");
   private final ImGuiDirectory imPosesDirectory;

   public RDXTeleoperationScriptedPoses()
   {
      super("Scripted Poses");

      setRenderMethod(this::renderImGuiWidgets);

      imPosesDirectory = new ImGuiDirectory(posesDirectory.getFilesystemDirectory().toString(),
                                            name -> true,
                                            pathEntry -> true,
                                            this::pathSelected);
   }

   private void renderImGuiWidgets()
   {
      imPosesDirectory.renderImGuiWidgets();

      ImGui.text("Save current interactable poses as:");
      ImGuiTools.inputText(labels.getHidden("saveFileName"), saveFileName);
      ImGui.sameLine();
      ImGui.text(".json");
      if (!saveFileName.get().isEmpty() && !saveFileName.get().endsWith(".json"))
      {
         ImGui.sameLine();
         if (ImGui.button(labels.get("Save Pose")))
         {

            LogTools.info("meep");
         }
      }
   }

   private void pathSelected(String pathName)
   {
      LogTools.info("Selected {}", pathName);
   }
}
