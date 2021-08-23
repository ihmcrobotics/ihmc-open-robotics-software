package us.ihmc.gdx.ui.behaviors;

import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.yo.YoBooleanClientHelper;
import us.ihmc.gdx.imgui.ImGuiLabelMap;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;

public class ImGuiBehaviorModuleDisabledNodeUI extends GDXBehaviorUIInterface
{
   private final BehaviorHelper helper;
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private final ImBoolean imEnabled = new ImBoolean(false);
   private final YoBooleanClientHelper yoEnabled;

   public ImGuiBehaviorModuleDisabledNodeUI(BehaviorHelper helper)
   {
      this.helper = helper;
      yoEnabled = helper.subscribeToYoBoolean("enabled");
   }

   @Override
   public void create(GDXImGuiBasedUI baseUI)
   {

   }

   @Override
   public void update()
   {

   }

   @Override
   public void renderTreeNodeImGuiWidgets()
   {
      if (ImGui.checkbox(labels.get("Enabled"), imEnabled))
      {
         yoEnabled.set(imEnabled.get());
      }
      ImGui.sameLine();
      ImGui.text("Server: " + yoEnabled.get());
   }

   @Override
   public void renderRegularPanelImGuiWidgets()
   {

   }

   @Override
   public void destroy()
   {

   }

   @Override
   public String getName()
   {
      return "Disabled";
   }
}
