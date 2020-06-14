package us.ihmc.avatar.slamTools;

/**
 * MultiplePlanes : complicated frames.
 * <p>
 * DownStairsBigZDrift : single plane, huge drift in Z.
 * <p>
 * UpStairs2YDrift : small drift in Y.
 * <p>
 * Upstairs3YDriftSmallOverlap : similar with UpStairs2YDrift but small overlapped area.
 */
public enum DriftCase
{
   MultiplePlanes, DownStairsBigZDrift, UpStairs2YDrift, Upstairs3YDriftSmallOverlap, Upstairs3RollDrift;

   public String getFilePath()
   {
      switch (this)
      {
         case MultiplePlanes:
            return "PointCloudData\\Data\\20200305_Simple\\PointCloud\\";
         case DownStairsBigZDrift:
            return "PointCloudData\\Data\\20200601_LidarWalking_DownStairs\\PointCloud\\";
         case UpStairs2YDrift:
            return "PointCloudData\\Data\\20200601_LidarWalking_UpStairs2\\PointCloud\\";
         case Upstairs3YDriftSmallOverlap:
         case Upstairs3RollDrift:
            return "PointCloudData\\Data\\20200603_LidarWalking_StairUp3\\PointCloud\\";
      }
      return "";
   }

   public int getPreviousFrameIndex()
   {
      switch (this)
      {
         case MultiplePlanes:
            return 30;
         case DownStairsBigZDrift:
            return 12;
         case UpStairs2YDrift:
            return 3;
         case Upstairs3YDriftSmallOverlap:
            return 4;
         case Upstairs3RollDrift:
            return 7;
      }
      return -1;
   }

   public int getNewFrameIndex()
   {
      switch (this)
      {
         case MultiplePlanes:
            return 33;
         case DownStairsBigZDrift:
            return 13;
         case UpStairs2YDrift:
            return 4;
         case Upstairs3YDriftSmallOverlap:
            return 5;
         case Upstairs3RollDrift:
            return 8;
      }
      return -1;
   }
}
