package us.ihmc.avatar.slamTools;

public enum DriftCase
{
   MultiplePlanes, BigZDrift, YDrift, YawDrift, YDriftSmallOverlap, RollDrift, HugeDrift;

   public String getFilePath()
   {
      String filePath = System.getProperty("user.dir") + "\\";
      switch (this)
      {
         case MultiplePlanes:
            return filePath + "PointCloudData\\DriftCase\\MultiplePlanes\\";
         case BigZDrift:
            return filePath + "PointCloudData\\DriftCase\\BigZDrift\\";
         case YDrift:
            return filePath + "PointCloudData\\DriftCase\\YDrift\\";
         case YawDrift:
            return filePath + "PointCloudData\\DriftCase\\YawDrift\\";
         case YDriftSmallOverlap:
            return filePath + "PointCloudData\\DriftCase\\YDriftSmallOverlap\\";
         case RollDrift:
            return filePath + "PointCloudData\\DriftCase\\RollDrift\\";
         case HugeDrift:
            return filePath + "PointCloudData\\DriftCase\\HugeDrift\\";
      }
      return "";
   }

   public String getInformation()
   {
      switch (this)
      {
         case MultiplePlanes:
            return "Set: 20200305_Simple, Frame Index: 30-33.";
         case BigZDrift:
            return "Set: 20200601_LidarWalking_DownStairs, Frame Index: 12-13.";
         case YDrift:
            return "Set: 20200601_LidarWalking_UpStairs2, Frame Index: 3-4.";
         case YawDrift:
            return "Set: 20200601_LidarWalking_UpStairs2, Frame Index: 4-5.";
         case YDriftSmallOverlap:
            return "Set: 20200603_LidarWalking_StairUp3, Frame Index: 4-5.";
         case RollDrift:
            return "Set: 20200603_LidarWalking_StairUp3, Frame Index: 7-8.";
         case HugeDrift:
            return "Set: 20200603_LidarWalking_StairUp3, Frame Index: 8-9.";
      }
      return "";
   }

   public String getNote()
   {
      switch (this)
      {
         case MultiplePlanes:
            return "complicated frames.";
         case BigZDrift:
            return "single plane, huge drift in Z.";
         case YDrift:
            return "small drift in Y";
         case YawDrift:
            return "small drift in Yaw";
         case YDriftSmallOverlap:
            return "most difficult case. similar with 'UpStairs2YDrift' but small overlapped area.";
         case RollDrift:
            return "drift in X and Roll. Verified low correspondence threshold (compare to 'Upstairs3YDriftSmallOverlap').";
         case HugeDrift:
            return "huge drift in X.";
      }
      return "";
   }
}
