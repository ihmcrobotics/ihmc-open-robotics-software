package us.ihmc.rdx.ui.algorithmUI;

import imgui.ImGui;
import imgui.ImGuiIO;
import imgui.ImGuiPlatformIO;
import imgui.flag.ImGuiKey;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.RDXBaseUI;

import java.util.Comparator;
import java.util.TreeSet;

public class RDXAlgorithmsPanel extends RDXPanel
{
   private static final String WINDOW_NAME = "Perception Algorithms";

   private final RDXBaseUI baseUI;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final TreeSet<RDXTunableAlgorithm> algorithms = new TreeSet<>(Comparator.comparing(RDXTunableAlgorithm::getTitle));

   public RDXAlgorithmsPanel(RDXBaseUI baseUI)
   {
      super(WINDOW_NAME);

      this.baseUI = baseUI;
      setRenderMethod(this::renderImGuiWidgets);
   }

   public void addAlgorithmPanel(RDXTunableAlgorithm algorithm)
   {
      algorithms.add(algorithm);
      if (!algorithm.isCreated())
         algorithm.create();
   }

   private void renderImGuiWidgets()
   {
      for (RDXTunableAlgorithm algorithm : algorithms)
      {
         if (algorithm.hasHeartbeat())
         {
            if (ImGui.checkbox(labels.get("##active"), algorithm.getActive()))
            {
               algorithm.updateHeartbeat();
            }
            ImGui.sameLine();
         }

         float preButtonCursorY = ImGui.getCursorPosY();
         if (ImGui.button(labels.get(algorithm.getTitle()), -1.0f, 0.0f))
         {
            if (!algorithm.getIsShowing().get())
            {
               showAlgorithmPanel(algorithm);
            }
            else
               hideAlgorithmPanel(algorithm);
         }
         float postButtonCursorY = ImGui.getCursorPosY();

         if (algorithm.getIsShowing().get())
         {
            ImGui.setCursorPosY(preButtonCursorY + (ImGui.getTextLineHeight() / 2) - 2);
            ImGuiTools.rightAlignText(">> ");
            ImGui.setCursorPosY(postButtonCursorY);
         }
      }
   }

   private void showAlgorithmPanel(RDXTunableAlgorithm algorithmToShow)
   {
      for (RDXTunableAlgorithm algorithm : algorithms)
      {
         if (algorithm != algorithmToShow && algorithm.getIsShowing().get())
         {
            hideAlgorithmPanel(algorithm);
         }
      }

      baseUI.getImGuiPanelManager().queueAddPanel(algorithmToShow);
      algorithmToShow.getIsShowing().set(true);
   }

   private void hideAlgorithmPanel(RDXTunableAlgorithm algorithm)
   {
      baseUI.getImGuiPanelManager().queueRemovePanel(algorithm);
      algorithm.getIsShowing().set(false);
   }
}
