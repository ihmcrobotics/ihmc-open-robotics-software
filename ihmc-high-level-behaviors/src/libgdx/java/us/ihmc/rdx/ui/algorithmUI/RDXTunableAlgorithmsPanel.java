package us.ihmc.rdx.ui.algorithmUI;

import imgui.ImGui;
import imgui.ImGuiTextFilter;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.Deque;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.TreeSet;

public class RDXTunableAlgorithmsPanel extends RDXPanel
{
   private static final String WINDOW_NAME = "Tunable Algorithms";

   private final ROS2PublishSubscribeAPI ros2;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiTextFilter searchFilter = new ImGuiTextFilter();
   private final ImBoolean allowMultiplePanels = new ImBoolean(false);
   private final ImInt maxPanelsToShow = new ImInt(4);

   private final TreeSet<RDXTunableAlgorithm> algorithms = new TreeSet<>((algorithmA, algorithmB) ->
   {
      int comparePinned = Boolean.compare(algorithmB.isPinned(), algorithmA.isPinned()); // Pinned algorithms get priority
      if (comparePinned != 0)
         return comparePinned;

      return algorithmA.getTitle().compareTo(algorithmB.getTitle()); // Otherwise sort alphabetically by title
   });
   private final Deque<RDXTunableAlgorithm> currentlyShownAlgorithms = new LinkedList<>();

   public RDXTunableAlgorithmsPanel(ROS2PublishSubscribeAPI ros2)
   {
      super(WINDOW_NAME);

      this.ros2 = ros2;
      setRenderMethod(this::renderImGuiWidgets);
   }
   public void addTunableAlgorithm(StoredPropertySetBasics algorithmPropertySet, StoredPropertySetROS2TopicPair propertySetTopicPair)
   {
      addTunableAlgorithm(algorithmPropertySet, propertySetTopicPair, false);
   }
   public void addTunableAlgorithm(StoredPropertySetBasics algorithmPropertySet, StoredPropertySetROS2TopicPair propertySetTopicPair, boolean pinToTop)
   {
      RDXTunableAlgorithm tunableAlgorithm = new RDXTunableAlgorithm(algorithmPropertySet, propertySetTopicPair, ros2);
      if (pinToTop)
         tunableAlgorithm.pinToTop();
      addAlgorithm(tunableAlgorithm);
   }

   public void addAlgorithm(RDXTunableAlgorithm algorithm)
   {
      algorithms.add(algorithm);
      addChild(algorithm);
      if (!algorithm.isCreated())
         algorithm.create();
   }

   private void renderImGuiWidgets()
   {
      searchFilter.draw(labels.get("Search"));

      if (ImGuiTools.smallCheckbox(labels.get("Allow Multiple Panels"), allowMultiplePanels))
      {
         if (!allowMultiplePanels.get() && !currentlyShownAlgorithms.isEmpty())
         {
            hideAllOtherPanels(currentlyShownAlgorithms.peekLast());
         }
      }

      ImGui.sameLine();
      if (ImGuiTools.smallButton(labels.get("Close All")))
      {
         currentlyShownAlgorithms.forEach(algorithm -> algorithm.getIsShowing().set(false));
         currentlyShownAlgorithms.clear();
      }

      ImGui.setNextItemWidth(0.5f * ImGui.getColumnWidth());
      if (ImGuiTools.smallWidget(() -> ImGuiTools.volatileInputInt(labels.get("Max Panels to Show"), maxPanelsToShow)))
         showNPanelsMax(maxPanelsToShow.get());

      ImGui.separator();

      for (RDXTunableAlgorithm algorithm : algorithms)
      {
         // Only show algorithms which pass the filter
         if (!searchFilter.passFilter(algorithm.getTitle()))
            continue;

         if (algorithm.renderMenuEntry()) // If the algorithm button is pressed
         {
            if (algorithm.isShowing())
            {
               if (allowMultiplePanels.get())
                  showNPanelsMax(maxPanelsToShow.get() - 1);
               else
                  hideAllOtherPanels(algorithm);

               currentlyShownAlgorithms.addLast(algorithm);
            }
            else
            {
               currentlyShownAlgorithms.remove(algorithm);
            }
         }
      }
   }

   private void hideAllOtherPanels(RDXTunableAlgorithm algorithmToShow)
   {
      Iterator<RDXTunableAlgorithm> algorithmIterator = currentlyShownAlgorithms.iterator();
      while (algorithmIterator.hasNext())
      {
         RDXTunableAlgorithm algorithm = algorithmIterator.next();
         if (algorithm == algorithmToShow)
            continue;

         algorithm.getIsShowing().set(false);
         algorithmIterator.remove();
      }
   }

   private void showNPanelsMax(int maxPanelsToShow)
   {
      if (maxPanelsToShow <= 0)
         return;

      while (currentlyShownAlgorithms.size() > maxPanelsToShow)
         currentlyShownAlgorithms.removeFirst().getIsShowing().set(false);
   }
}
