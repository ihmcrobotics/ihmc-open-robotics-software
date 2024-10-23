package us.ihmc.avatar.slamTools;

import java.io.File;
import java.net.URL;

public enum DriftCase
{
   MultiplePlanes, BigZDrift, YDrift, YawDrift, YDriftSmallOverlap, RollDrift, HugeDrift;

   public String getFilePath()
   {
      URL pointCloudDataDirectory = DriftCase.class.getResource("PointCloudData");

      if (pointCloudDataDirectory != null)
      {
         String directoryPath = pointCloudDataDirectory.getPath();
         String separator = File.separator;
         switch (this)
         {
            case MultiplePlanes:
               return directoryPath + separator + "DriftCase" + separator + "MultiplePlanes" + separator;
            case BigZDrift:
               return directoryPath + separator + "DriftCase" + separator + "BigZDrift" + separator;
            case YDrift:
               return directoryPath + separator + "DriftCase" + separator + "YDrift" + separator;
            case YawDrift:
               return directoryPath + separator + "DriftCase" + separator + "YawDrift" + separator;
            case YDriftSmallOverlap:
               return directoryPath + separator + "DriftCase" + separator + "YDriftSmallOverlap" + separator;
            case RollDrift:
               return directoryPath + separator + "DriftCase" + separator + "RollDrift" + separator;
            case HugeDrift:
               return directoryPath + separator + "DriftCase" + separator + "HugeDrift" + separator;
         }
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
