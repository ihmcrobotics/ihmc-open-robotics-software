package us.ihmc.rdx.logging;

import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.simulation.scs2.RDXSCS2LogSession;

public class RDXLeRobotDatasetCreator
{
   private final RDXSCS2LogSession logSession;
   private final RDXPanel panel;

   public RDXLeRobotDatasetCreator(RDXSCS2LogSession logSession)
   {
      this.logSession = logSession;

      panel = new RDXPanel("LeRobot Dataset Creator", this::renderImGuiWidgets);
   }

   private void renderImGuiWidgets()
   {

   }

   public RDXPanel getPanel()
   {
      return panel;
   }
}
