package us.ihmc.rdx.imgui;

public class ImGuiDockingSetupInstruction
{
   private String windowName;
   private String windowToSplit;
   private String windowNameToAdd;
   private int imGuiDir;
   private double percent;

   public ImGuiDockingSetupInstruction(String windowName)
   {
      this.windowName = windowName;
   }

   public ImGuiDockingSetupInstruction(String windowToSplit, String windowNameToAdd, int imGuiDir, double percent)
   {
      this.windowToSplit = windowToSplit;
      this.windowNameToAdd = windowNameToAdd;
      this.imGuiDir = imGuiDir;
      this.percent = percent;
   }

   public String getWindowName()
   {
      return windowName;
   }

   public String getWindowToSplit()
   {
      return windowToSplit;
   }

   public String getWindowNameToAdd()
   {
      return windowNameToAdd;
   }

   public int getImGuiDir()
   {
      return imGuiDir;
   }

   public double getPercent()
   {
      return percent;
   }
}
