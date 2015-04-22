package us.ihmc.ihmcPerception.linemod;

public class OrganizedPointCloud
{
   public int width;
   public int height;
   public float[] xyzrgb;
   
   public OrganizedPointCloud(int width, int height, float[] xyzrgb)
   {
      this.width=width;
      this.height=height;
      this.xyzrgb=xyzrgb;
   }
}
