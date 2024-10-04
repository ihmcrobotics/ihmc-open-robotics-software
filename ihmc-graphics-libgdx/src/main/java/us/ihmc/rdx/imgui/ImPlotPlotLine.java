package us.ihmc.rdx.imgui;

public interface ImPlotPlotLine
{
   public boolean render();

   public String getVariableName();

   public String getValueString(int bufferIndex);
}
