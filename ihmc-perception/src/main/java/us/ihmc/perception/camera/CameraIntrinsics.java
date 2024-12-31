package us.ihmc.perception.camera;

public class CameraIntrinsics
{
   /**
    * Focal length along the x axis of the camera.
    */
   private double fx;
   /**
    * Focal length along the y axis of the camera.
    */
   private double fy;
   /**
    * Camera central point along the x axis of the camera, also known as u
    */
   private double cx;
   /**
    * Camera central point along the y axis of the camera, also known as v
    */
   private double cy;

   /**
    * Number of pixels along the height of the image, which is in the x axis, also known as u width
    */
   private int height;
   /**
    * Number of pixels along the width of the image, which is in the y axis.
    */
   private int width;

   public CameraIntrinsics()
   {
   }

   public CameraIntrinsics(CameraIntrinsics other)
   {
      this(other.getHeight(), other.getWidth(), other.getFx(), other.getFy(), other.getCx(), other.getCy());
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
