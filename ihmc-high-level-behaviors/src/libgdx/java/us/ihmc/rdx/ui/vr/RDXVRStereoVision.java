package us.ihmc.rdx.ui.vr;

import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import us.ihmc.rdx.imgui.ImGuiFlashingText;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.perception.RDXDualBlackflySphericalProjection;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXVRStereoVision
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final ImBoolean enabled = new ImBoolean(false);
   private final RDXDualBlackflySphericalProjection dualBlackflySphericalProjection = new RDXDualBlackflySphericalProjection();
   private final ImGuiFlashingText reconnectingText = new ImGuiFlashingText(ImGuiTools.DARK_ORANGE);
   private final ImGuiFlashingText leftConnectedStatusCircle = new ImGuiFlashingText(ImGuiTools.DARK_GREEN);
   private final ImGuiFlashingText rightConnectedStatusCircle = new ImGuiFlashingText(ImGuiTools.DARK_GREEN);
   private final ImString leftConnectionAddress = new ImString();
   private final ImString rightConnectionAddress = new ImString();

   public boolean isEnabled()
   {
      return enabled.get();
   }

   public void renderProjection()
   {
      if (enabled.get())
         dualBlackflySphericalProjection.render();
   }

   public void renderControls()
   {
      if (enabled.get() && !dualBlackflySphericalProjection.isConnectingOrConnected())
         dualBlackflySphericalProjection.enable();

      if (ImGui.checkbox(labels.get("Stereo vision enabled"), enabled))
      {
         if (!enabled.get())
         {
            dualBlackflySphericalProjection.disable();
         }
      }

      if (dualBlackflySphericalProjection.isConnected())
      {
         ImGui.textColored(ImGuiTools.DARK_GREEN, "Connected");

         double leftFrequency = dualBlackflySphericalProjection.getDualBlackflyUDPReceiver().getFrequencyCalculators().get(RobotSide.LEFT).getFrequency();
         double rightFrequency = dualBlackflySphericalProjection.getDualBlackflyUDPReceiver().getFrequencyCalculators().get(RobotSide.RIGHT).getFrequency();

         leftConnectedStatusCircle.setPeriod(1000 / leftFrequency);
         rightConnectedStatusCircle.setPeriod(1000 / rightFrequency);
         ImGui.sameLine();
         leftConnectedStatusCircle.renderText("(L)", leftFrequency != 0);
         ImGui.sameLine();
         rightConnectedStatusCircle.renderText("(R)", rightFrequency != 0);
      }
      else if (dualBlackflySphericalProjection.isReconnecting())
      {
         reconnectingText.renderText("Reconnecting...", true);
      }
      else
      {
         ImGui.textColored(ImGuiTools.DARK_RED, "Not connected");
      }

      if (ImGui.collapsingHeader(labels.get("Connection controls")))
      {
         boolean updatedAddress = false;

         if (leftConnectionAddress.isEmpty())
         {
            leftConnectionAddress.set(dualBlackflySphericalProjection.getDualBlackflyUDPReceiver().getAddresses().get(RobotSide.LEFT));
         }

         if (rightConnectionAddress.isEmpty())
         {
            rightConnectionAddress.set(dualBlackflySphericalProjection.getDualBlackflyUDPReceiver().getAddresses().get(RobotSide.RIGHT));
         }

         if (ImGui.inputText(labels.get("(L) address"), leftConnectionAddress))
         {
            dualBlackflySphericalProjection.getDualBlackflyUDPReceiver().setAddress(RobotSide.LEFT, leftConnectionAddress.get());
            updatedAddress = true;
         }
         if (ImGui.inputText(labels.get("(R) address"), rightConnectionAddress))
         {
            dualBlackflySphericalProjection.getDualBlackflyUDPReceiver().setAddress(RobotSide.RIGHT, rightConnectionAddress.get());
            updatedAddress = true;
         }

         if (updatedAddress)
         {
            enabled.set(false);
            dualBlackflySphericalProjection.disable();
         }
      }

      if (ImGui.collapsingHeader(labels.get("Projection sphere controls")))
      {
         dualBlackflySphericalProjection.renderControls();
      }
   }

   public RDXDualBlackflySphericalProjection getDualBlackflySphericalProjection()
   {
      return dualBlackflySphericalProjection;
   }
}
