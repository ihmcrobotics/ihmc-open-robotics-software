package us.ihmc.rdx.imgui;

public interface ImPlotPlotLineSwapBuffer
{
   public void initialize(int bufferSize);

   public void setAValue(int index);

   public void setPreviousValue(int index);

   public void setUpdatedValue(int index);

   public void copyAToB();

   public void copyPreviousToUpdated(int srcPos, int destPos, int length);

   public void plot(String labelID, double[] xValues, int offset);
}
