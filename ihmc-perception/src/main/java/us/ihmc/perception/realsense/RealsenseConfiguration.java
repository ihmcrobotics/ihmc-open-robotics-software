package us.ihmc.perception.realsense;

public class RealsenseConfiguration
{
   private String serialNumber;

   private int depthHeight;
   private int depthWidth;
   private int depthFPS;
   private boolean useColor;
   private int colorHeight;
   private int colorWidth;
   private int colorFPS;

   public static RealsenseConfiguration L515_CONFIG_640_480_30FPS = new RealsenseConfiguration("L515", 480, 640, 30, true, 480, 640, 30);
   public static RealsenseConfiguration D455_CONFIG_640_480_30FPS = new RealsenseConfiguration("D455", 480, 640, 30, true, 480, 640, 30);

   public RealsenseConfiguration(String serialNumber, int depthHeight, int depthWidth, int depthFPS, boolean useColor, int colorHeight, int colorWidth, int colorFPS)
   {
      this.serialNumber = serialNumber;
      this.depthHeight = depthHeight;
      this.depthWidth = depthWidth;
      this.depthFPS = depthFPS;
      this.useColor = useColor;
      this.colorHeight = colorHeight;
      this.colorWidth = colorWidth;
      this.colorFPS = colorFPS;
   }

   public String getSerialNumber()
   {
      return serialNumber;
   }

   public void setSerialNumber(String serialNumber)
   {
      this.serialNumber = serialNumber;
   }

   public int getDepthHeight()
   {
      return depthHeight;
   }

   public void setDepthHeight(int depthHeight)
   {
      this.depthHeight = depthHeight;
   }

   public int getDepthWidth()
   {
      return depthWidth;
   }

   public void setDepthWidth(int depthWidth)
   {
      this.depthWidth = depthWidth;
   }

   public boolean isUseColor()
   {
      return useColor;
   }

   public void setUseColor(boolean useColor)
   {
      this.useColor = useColor;
   }

   public int getColorHeight()
   {
      return colorHeight;
   }

   public void setColorHeight(int colorHeight)
   {
      this.colorHeight = colorHeight;
   }

   public int getColorWidth()
   {
      return colorWidth;
   }

   public void setColorWidth(int colorWidth)
   {
      this.colorWidth = colorWidth;
   }

   public int getColorFPS()
   {
      return colorFPS;
   }

   public void setColorFPS(int colorFPS)
   {
      this.colorFPS = colorFPS;
   }

   public void setDepthFPS(int depthFPS)
   {
      this.depthFPS = depthFPS;
   }

   public int getDepthFPS()
   {
      return depthFPS;
   }
}
