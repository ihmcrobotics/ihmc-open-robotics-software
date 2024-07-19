package us.ihmc.rdx.ui.algorithmUI;

import imgui.ImGui;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.RDXBaseUI;

import java.util.Comparator;
import java.util.TreeSet;

public class RDXTunableAlgorithmsPanel extends RDXPanel
{
   private static final String WINDOW_NAME = "Perception Algorithms";

   private final RDXBaseUI baseUI;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final TreeSet<RDXTunableAlgorithm> algorithms = new TreeSet<>(Comparator.comparing(RDXTunableAlgorithm::getTitle));

   public RDXTunableAlgorithmsPanel(RDXBaseUI baseUI)
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
         if (algorithm.renderMenuEntry())
         {
            if (algorithm.isShowing())
            {
               showAlgorithmPanel(algorithm);
            }
            else
            {
               hideAlgorithmPanel(algorithm);
            }
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
   }

   private void hideAlgorithmPanel(RDXTunableAlgorithm algorithm)
   {
      baseUI.getImGuiPanelManager().queueRemovePanel(algorithm);
      algorithm.getIsShowing().set(false);
   }
}
