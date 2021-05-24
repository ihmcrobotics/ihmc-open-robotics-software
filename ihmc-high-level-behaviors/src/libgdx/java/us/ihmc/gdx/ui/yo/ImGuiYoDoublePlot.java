package us.ihmc.gdx.ui.yo;

import us.ihmc.behaviors.tools.interfaces.YoVariableClientPublishSubscribeAPI;
import us.ihmc.gdx.imgui.ImGuiPlot;

import java.util.function.DoubleSupplier;

public class ImGuiYoDoublePlot
{
   private final ImGuiPlot imGuiPlot;
   private final DoubleSupplier doubleSupplier;

   public ImGuiYoDoublePlot(String yoVariableName, YoVariableClientPublishSubscribeAPI yoAPI)
   {
      imGuiPlot = new ImGuiPlot(yoVariableName);
      doubleSupplier = yoAPI.subscribeViaYoDouble(yoVariableName);
   }

   public void render()
   {
      imGuiPlot.render((float) doubleSupplier.getAsDouble());
   }
}
