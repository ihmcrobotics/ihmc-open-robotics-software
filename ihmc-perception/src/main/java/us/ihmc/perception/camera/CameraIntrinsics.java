package us.ihmc.perception.camera;

public class CameraIntrinsics
{
   private double fx;
   private double fy;
   private double cx;
   private double cy;

   private int height;
   private int width;

   public CameraIntrinsics()
   {
   }

   public CameraIntrinsics(int height, int width, double fx, double fy, double cx, double cy)
   {
      this.fx = fx;
      this.fy = fy;
      this.cx = cx;
      this.cy = cy;
      this.height = height;
      this.width = width;
   }

   public double getFx()
   {
      return fx;
   }

   public double getFy()
   {
      return fy;
   }

   public double getCx()
   {
      return cx;
   }

   public double getCy()
   {
      return cy;
   }

   public int getHeight()
   {
      return height;
   }

   public int getWidth()
   {
      return width;
   }

   public void setFx(double fx)
   {
      this.fx = fx;
   }

   public void setFy(double fy)
   {
      this.fy = fy;
   }

   public void setCx(double cx)
   {
      this.cx = cx;
   }

   public void setCy(double cy)
   {
      this.cy = cy;
   }

   public void setHeight(int height)
   {
      this.height = height;
   }

   public void setWidth(int width)
   {
      this.width = width;
   }

   public String toString()
   {
      return "CameraIntrinsics [fx=" + fx + ", fy=" + fy + ", cx=" + cx + ", cy=" + cy + ", height=" + height + ", width=" + width + "]";
   }
}
