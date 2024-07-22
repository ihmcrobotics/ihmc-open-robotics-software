package us.ihmc.rdx.ui.algorithmUI;

import imgui.ImGui;
import imgui.ImGuiTextFilter;
import imgui.type.ImBoolean;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.Comparator;
import java.util.Iterator;
import java.util.Stack;
import java.util.TreeSet;

public class RDXTunableAlgorithmsPanel extends RDXPanel
{
   private static final String WINDOW_NAME = "Perception Algorithms";

   private final ROS2PublishSubscribeAPI ros2;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiTextFilter searchFilter = new ImGuiTextFilter();
   private final ImBoolean allowMultiplePanels = new ImBoolean(false);

   private final TreeSet<RDXTunableAlgorithm> algorithms = new TreeSet<>(Comparator.comparing(RDXTunableAlgorithm::getTitle));
   private final Stack<RDXTunableAlgorithm> currentlyShownAlgorithms = new Stack<>();

   public RDXTunableAlgorithmsPanel(ROS2PublishSubscribeAPI ros2)
   {
      super(WINDOW_NAME);

      this.ros2 = ros2;
      setRenderMethod(this::renderImGuiWidgets);
   }

   public void addTunableAlgorithm(StoredPropertySetBasics algorithmPropertySet, StoredPropertySetROS2TopicPair propertySetTopicPair)
   {
      RDXTunableAlgorithm tunableAlgorithm = new RDXTunableAlgorithm(algorithmPropertySet, propertySetTopicPair, ros2);
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
         if (!allowMultiplePanels.get() && !currentlyShownAlgorithms.empty())
         {
            hideAllOtherPanels(currentlyShownAlgorithms.peek());
         }
      }

      ImGui.sameLine();
      if (ImGuiTools.smallButton(labels.get("Close All")))
      {
         currentlyShownAlgorithms.forEach(algorithm -> algorithm.getIsShowing().set(false));
         currentlyShownAlgorithms.clear();
      }

      ImGui.separator();

      for (RDXTunableAlgorithm algorithm : algorithms)
      {
         if (searchFilter.passFilter(algorithm.getTitle()))
         {
            if (algorithm.renderMenuEntry())
            {
               if (algorithm.isShowing())
               {
                  if (!allowMultiplePanels.get())
                     hideAllOtherPanels(algorithm);
                  currentlyShownAlgorithms.add(algorithm);
               }
               else
               {
                  currentlyShownAlgorithms.remove(algorithm);
               }
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
}
