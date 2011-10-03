package us.ihmc.vicon;

import java.io.Serializable;

public class QuaternionPose implements Serializable
{
   private static final long serialVersionUID = -6213249019411848321L;
   public boolean dataValid;

   public float xPosition;
   public float yPosition;
   public float zPosition;

   public float qx;
   public float qy;
   public float qz;
   public float qw;

   public QuaternionPose()
   {
      this.dataValid = false;
      this.xPosition = 0.0f;
      this.yPosition = 0.0f;
      this.zPosition = 0.0f;

      this.qx = 0.0f;
      this.qy = 0.0f;
      this.qz = 0.0f;
      this.qw = 0.0f;
   }

   public QuaternionPose(float xPosition, float yPosiiton, float zPosition, float qx, float qy, float qz, float qw)
   {
      this.dataValid = true;
      this.xPosition = xPosition;
      this.yPosition = yPosiiton;
      this.zPosition = zPosition;

      this.qx = qx;
      this.qy = qy;
      this.qz = qz;
      this.qw = qw;
   }

   public QuaternionPose(QuaternionPose pose)
   {
      dataValid = pose.dataValid;
      xPosition = pose.xPosition;
      yPosition = pose.yPosition;
      zPosition = pose.zPosition;
      qx = pose.qx;
      qy = pose.qy;
      qz = pose.qz;
      qw = pose.qw;
   }

   public void scaleTranslation(float factor)
   {
      xPosition *= factor;
      yPosition *= factor;
      zPosition *= factor;
   }

   public boolean equals(QuaternionPose pose)
   {
      if (xPosition != pose.xPosition)
         return false;
      if (yPosition != pose.yPosition)
         return false;
      if (zPosition != pose.zPosition)
         return false;
      if (qx != pose.qx)
         return false;
      if (qy != pose.qy)
         return false;
      if (qz != pose.qz)
         return false;

      return true;

   }

   public void invalidate()
   {
      dataValid = false;
   }

   public String toString()
   {
      return "valid=" + dataValid + "(" + xPosition + ", " + yPosition + ", " + zPosition + ")" + "(" + qx + ", " + qy + ", " + qz + ", " + qw + ")";
   }
}
