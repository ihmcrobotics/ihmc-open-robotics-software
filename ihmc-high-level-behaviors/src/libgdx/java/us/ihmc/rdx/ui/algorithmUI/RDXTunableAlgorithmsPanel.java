package us.ihmc.rdx.ui.algorithmUI;

import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;

import java.util.Comparator;
import java.util.TreeSet;

public class RDXTunableAlgorithmsPanel extends RDXPanel
{
   private static final String WINDOW_NAME = "Perception Algorithms";

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean allowMultiplePanels = new ImBoolean(false);

   private final TreeSet<RDXTunableAlgorithm> algorithms = new TreeSet<>(Comparator.comparing(RDXTunableAlgorithm::getTitle));

   public RDXTunableAlgorithmsPanel()
   {
      super(WINDOW_NAME);

      setRenderMethod(this::renderImGuiWidgets);
   }

   public void addAlgorithmPanel(RDXTunableAlgorithm algorithm)
   {
      algorithms.add(algorithm);
      addChild(algorithm);
      if (!algorithm.isCreated())
         algorithm.create();
   }

   private void renderImGuiWidgets()
   {
      ImGui.checkbox(labels.get("Allow Multiple Settings Panels"), allowMultiplePanels);

      for (RDXTunableAlgorithm algorithm : algorithms)
      {
         if (algorithm.renderMenuEntry())
         {
            if (algorithm.isShowing() && !allowMultiplePanels.get())
            {
               hideAllOtherPanels(algorithm);
            }
         }
      }
   }

   private void hideAllOtherPanels(RDXTunableAlgorithm algorithmToShow)
   {
      for (RDXTunableAlgorithm algorithm : algorithms)
      {
         if (algorithm != algorithmToShow && algorithm.getIsShowing().get())
         {
            algorithm.getIsShowing().set(false);
         }
      }
   }
}
