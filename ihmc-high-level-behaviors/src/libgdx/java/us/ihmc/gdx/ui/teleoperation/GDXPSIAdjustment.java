package us.ihmc.gdx.ui.teleoperation;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;

public class GDXPSIAdjustment
{
   private final RobotLowLevelMessenger robotLowLevelMessenger;
   private final ImInt pumpPSI = new ImInt(1);
   private final String[] psiValues = new String[] {"1500", "2300", "2500", "2800"};

   public GDXPSIAdjustment(RobotLowLevelMessenger robotLowLevelMessenger)
   {

      this.robotLowLevelMessenger = robotLowLevelMessenger;
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.combo("PSI", pumpPSI, psiValues, psiValues.length))
      {
         sendPSIRequest();
      }
      ImGui.sameLine();
      if (ImGui.button("Resend PSI"))
      {
         sendPSIRequest();
      }
   }

   private void sendPSIRequest()
   {
      robotLowLevelMessenger.setHydraulicPumpPSI(Integer.parseInt(psiValues[pumpPSI.get()]));
   }
}
