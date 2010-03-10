package us.ihmc.vicon;

public class QuaternionPose extends Pose
{
   public float w;

   public QuaternionPose(float xPosition, float yPosiiton, float zPosition, float xAxisRotation, float yAxisRotation, float zAxisRotation, float w)
   {
      super(xPosition, yPosiiton, zPosition, xAxisRotation, yAxisRotation, zAxisRotation);
      this.w = w;
   }

   public String toString()
   {
      String string = super.toString();
      string = string.replace(")", ", " + w + ")");

      return string;
   }
}
