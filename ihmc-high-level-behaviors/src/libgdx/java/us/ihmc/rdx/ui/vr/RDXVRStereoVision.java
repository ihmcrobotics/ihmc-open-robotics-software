package us.ihmc.rdx.ui.vr;

import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.perception.RDXDualBlackflySphericalProjection;

public class RDXVRStereoVision
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final ImBoolean enabled = new ImBoolean(false);
   private final RDXDualBlackflySphericalProjection dualBlackflySphericalProjection = new RDXDualBlackflySphericalProjection();

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
         if (!enabled.get())
            dualBlackflySphericalProjection.disable();
   }

   public RDXDualBlackflySphericalProjection getDualBlackflySphericalProjection()
   {
      return dualBlackflySphericalProjection;
   }
}
