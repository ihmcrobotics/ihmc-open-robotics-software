package us.ihmc.gdx.imgui;

import us.ihmc.gdx.input.GDXInputAdapter;
import us.ihmc.gdx.input.GDXInputMode;

public class ImGuiGDXInputAdapter extends GDXInputAdapter
{
   public ImGuiGDXInputAdapter()
   {
      super(GDXInputMode.ImGui);
   }
}
