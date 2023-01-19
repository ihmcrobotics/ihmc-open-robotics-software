package us.ihmc.rdx.ui.yo;

public interface ImPlotPlotLine
{
   public boolean render();

   public String getVariableName();

   public String getValueString(int bufferIndex);
}
