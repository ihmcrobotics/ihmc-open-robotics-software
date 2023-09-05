package us.ihmc.rdx.ui.yo;

import us.ihmc.behaviors.tools.yo.YoDoubleClientHelper;
import us.ihmc.behaviors.tools.yo.YoVariableClientPublishSubscribeAPI;
import us.ihmc.rdx.imgui.ImGuiPlot;

public class ImGuiYoDoublePlot
{
   private final ImGuiPlot imGuiPlot;
   private final YoDoubleClientHelper yoDouble;

   public ImGuiYoDoublePlot(String yoVariableName, YoVariableClientPublishSubscribeAPI yoAPI, int bufferSize, int width, int height)
   {
      imGuiPlot = new ImGuiPlot(yoVariableName, bufferSize, width, height);
      yoDouble = yoAPI.subscribeToYoDouble(yoVariableName);
   }

   public void render()
   {
      imGuiPlot.render((float) yoDouble.get());
   }
}
