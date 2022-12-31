package us.ihmc.rdx.ui.behavior.editor;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import imgui.type.ImDouble;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.tools.Timer;

public class RDXWaitDurationAction extends RDXBehaviorAction
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDouble waitDuration = new ImDouble(4.0);
   private final Timer timer = new Timer();

   @Override
   public void renderImGuiSettingWidgets()
   {
      ImGui.pushItemWidth(80.0f);
      ImGui.inputDouble(labels.get("Wait duration"), waitDuration);
      ImGui.popItemWidth();
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("waitDuration", waitDuration.get());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      waitDuration.set(jsonNode.get("waitDuration").asDouble());
   }

   @Override
   public void performAction()
   {
      timer.reset();
   }

   @Override
   public boolean isExecuting()
   {
      return timer.isRunning(waitDuration.get());
   }

   @Override
   public String getNameForDisplay()
   {
      return String.format("Wait %.1f s", waitDuration.get());
   }
}
