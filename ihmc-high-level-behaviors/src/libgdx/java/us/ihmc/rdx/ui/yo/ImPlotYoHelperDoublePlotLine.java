package us.ihmc.rdx.ui.yo;

import us.ihmc.behaviors.tools.yo.YoDoubleClientHelper;
import us.ihmc.behaviors.tools.yo.YoVariableClientPublishSubscribeAPI;
import us.ihmc.rdx.imgui.ImPlotDoublePlotLine;
import us.ihmc.rdx.imgui.ImPlotPlot;

import java.text.DecimalFormat;

public class ImPlotYoHelperDoublePlotLine
{
   private final YoDoubleClientHelper yoDoubleClientHelper;
   private final ImPlotPlot plot;
   private final ImPlotDoublePlotLine plotLine;

   public ImPlotYoHelperDoublePlotLine(String variableName, double history, YoVariableClientPublishSubscribeAPI yoClientHelperAPI)
   {
      plot = new ImPlotPlot(70);
      plotLine = new ImPlotDoublePlotLine(variableName.substring(variableName.lastIndexOf(".")), 250, history, new DecimalFormat("0.000"));
      yoDoubleClientHelper = yoClientHelperAPI.subscribeToYoDouble(variableName);
      plot.getPlotLines().add(plotLine);
   }

   public void renderImGuiWidgets()
   {
      plotLine.addValue(yoDoubleClientHelper.get());
      plot.render(300, 40);
   }
}
