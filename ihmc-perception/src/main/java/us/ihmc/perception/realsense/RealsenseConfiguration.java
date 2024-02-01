package us.ihmc.perception.realsense;

/**
 * Only certain combinations of settings are valid and it's cumbersome to pass them around
 * separately, so let's just make some enums.
 *
 * L515 serial numbers:
 * - F0245563
 *
 * D435 serial numbers:
 * - 752112070330
 */
public enum RealsenseConfiguration
{
   L515_COLOR_720P_DEPTH_768P_30HZ(1024, 768, 30, 1280, 720, 30),
   L515_COLOR_480P_DEPTH_480P_30HZ(640, 480, 30, 640, 640, 30),
   L515_COLOR_720P_DEPTH_768P_15HZ(1024, 768, 15, 1280, 720, 15),
   D435_COLOR_480P_DEPTH_480P_30HZ(640, 480, 30, 640, 480, 30),
   D435_COLOR_720P_DEPTH_720P_30HZ(1280, 720, 30, 1280, 720, 30),
   D455_COLOR_720P_DEPTH_720P_30HZ(1280, 720, 30, 1280, 720, 30),
   ;

   private final int depthWidth;
   private final int depthHeight;
   private final int depthFPS;
   private final int colorWidth;
   private final int colorHeight;
   private final int colorFPS;

   RealsenseConfiguration(int depthWidth, int depthHeight, int depthFPS, int colorWidth, int colorHeight, int colorFPS)
   {
      this.depthWidth = depthWidth;
      this.depthHeight = depthHeight;
      this.depthFPS = depthFPS;
      this.colorWidth = colorWidth;
      this.colorHeight = colorHeight;
      this.colorFPS = colorFPS;
   }

   public int getDepthWidth()
   {
      return depthWidth;
   }

   public int getDepthHeight()
   {
      return depthHeight;
   }

   public int getDepthFPS()
   {
      return depthFPS;
   }

   public int getColorWidth()
   {
      return colorWidth;
   }

   public int getColorHeight()
   {
      return colorHeight;
   }

   public int getColorFPS()
   {
      return colorFPS;
   }
}
