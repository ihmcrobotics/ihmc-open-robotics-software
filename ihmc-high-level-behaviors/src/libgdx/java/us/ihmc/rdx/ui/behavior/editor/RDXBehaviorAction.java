package us.ihmc.rdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.type.ImBoolean;
import us.ihmc.rdx.input.ImGui3DViewInput;

public class RDXBehaviorAction
{
   private String nameForDisplay = "";
   private final ImBoolean selected = new ImBoolean();
   private final ImBoolean expanded = new ImBoolean(true);

   public RDXBehaviorAction()
   {

   }

   public RDXBehaviorAction(String nameForDisplay)
   {
      this.nameForDisplay = nameForDisplay;
   }

   public void update()
   {

   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {

   }

   public void process3DViewInput(ImGui3DViewInput input)
   {

   }

   public final void renderImGuiWidgets()
   {
      if (expanded.get())
      {
         renderImGuiSettingWidgets();
      }
   }

   public void renderImGuiSettingWidgets()
   {

   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

   }

   public void saveToFile(ObjectNode jsonNode)
   {

   }

   public void loadFromFile(JsonNode jsonNode)
   {

   }

   public void performAction()
   {

   }

   public boolean isExecuting()
   {
      return false;
   }

   public void destroy()
   {

   }

   public ImBoolean getSelected()
   {
      return selected;
   }

   public ImBoolean getExpanded()
   {
      return expanded;
   }

   public String getNameForDisplay()
   {
      return nameForDisplay;
   }
}
