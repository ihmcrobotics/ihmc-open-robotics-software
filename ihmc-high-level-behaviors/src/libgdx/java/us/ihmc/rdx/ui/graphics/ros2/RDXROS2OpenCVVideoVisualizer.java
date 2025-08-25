package us.ihmc.rdx.ui.graphics.ros2;

import imgui.type.ImBoolean;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.graphics.RDXOpenCVVideoVisualizer;

public abstract class RDXROS2OpenCVVideoVisualizer<T> extends RDXROS2SingleTopicVisualizer<T>
{
   private final RDXOpenCVVideoVisualizer openCVVideoVisualizer;
   private final ImBoolean subscriptionOnly = new ImBoolean();

   public RDXROS2OpenCVVideoVisualizer(String title, String panelName, boolean flipY)
   {
      super(title);

      openCVVideoVisualizer = new RDXOpenCVVideoVisualizer(title, panelName, flipY);
   }

   public void renderImGuiWidgetsPost()
   {
      if (ImGuiTools.smallCheckbox(labels.get("Subscription only"), subscriptionOnly))
      {
         RDXPanel panel = getPanel();
         if (panel != null)
            panel.getIsShowing().set(!subscriptionOnly.get());
      }
   }

   public RDXOpenCVVideoVisualizer getOpenCVVideoVisualizer()
   {
      return openCVVideoVisualizer;
   }

   public ImBoolean getSubscriptionOnly()
   {
      return subscriptionOnly;
   }
}
