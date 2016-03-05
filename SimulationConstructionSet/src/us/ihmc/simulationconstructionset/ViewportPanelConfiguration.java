package us.ihmc.simulationconstructionset;

public class ViewportPanelConfiguration
{
   public String cameraName;
   public int gridx, gridy, gridwidth, gridheight;

   public ViewportPanelConfiguration(String cameraName, int gridx, int gridy, int gridwidth, int gridheight)
   {
      this.cameraName = cameraName;
      this.gridx = gridx;
      this.gridy = gridy;
      this.gridwidth = gridwidth;
      this.gridheight = gridheight;
   }
}
