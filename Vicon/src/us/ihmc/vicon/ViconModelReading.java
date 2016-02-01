package us.ihmc.vicon;

import java.io.Serializable;

/**
 * Last updated by: JSmith
 * On: 13/08/11
 */
public class ViconModelReading implements Serializable
{
   private static final long serialVersionUID = -5390153148211819422L;
   private String modelName;
   private long timestamp;
   private QuaternionPose QuaternionPose;

   public ViconModelReading(String modelName, long timestamp, QuaternionPose QuaternionPose)
   {
      this.modelName = modelName;
      this.timestamp = timestamp;
      this.QuaternionPose = QuaternionPose;
   }

   public long getTimestamp()
   {
      return timestamp;
   }

   public QuaternionPose getQuaternionPose()
   {
      return QuaternionPose;
   }

   public String getModelName()
   {
      return modelName;
   }

   public boolean equals(ViconModelReading QuaternionPoseReading)
   {
      if (!modelName.equals(QuaternionPoseReading.getModelName()))
         return false;
      if (timestamp != QuaternionPoseReading.getTimestamp())
         return false;
      if (!QuaternionPose.equals(QuaternionPoseReading.getQuaternionPose()))
         return false;

      return true;
   }

   public String toString()
   {
      return modelName + ": " + timestamp + ": " + QuaternionPose;
   }
}
