package us.ihmc.avatar.slamTools;

/**
 * MultiplePlanes : complicated frames.
 * <p>
 * DownStairsBigZDrift : single plane, huge drift in Z.
 * <p>
 * UpStairs2YDrift : small drift in Y.
 * <p>
 * Upstairs3YDriftSmallOverlap : similar with 'UpStairs2YDrift' but small overlapped area.
 * <p>
 * Upstairs3RollDrift : drift in X and Roll. Verified low correspondence threshold (compare to 'Upstairs3YDriftSmallOverlap').
 * <p>
 * UpStairs3Fast : huge drift in X.
 */
public enum DriftCase
{
   MultiplePlanes, DownStairsBigZDrift, UpStairs2YDrift, UpStairs2YawDrift, UpStairs3YDriftSmallOverlap, UpStairs3RollDrift, UpStairs3HugeDrift;

   public String getFilePath()
   {
      switch (this)
      {
         case MultiplePlanes:
            return "PointCloudData\\Data\\20200305_Simple\\PointCloud\\";
         case DownStairsBigZDrift:
            return "PointCloudData\\Data\\20200601_LidarWalking_DownStairs\\PointCloud\\";
         case UpStairs2YDrift:
         case UpStairs2YawDrift:
            return "PointCloudData\\Data\\20200601_LidarWalking_UpStairs2\\PointCloud\\";
         case UpStairs3YDriftSmallOverlap:
         case UpStairs3RollDrift:
         case UpStairs3HugeDrift:
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
         case UpStairs2YawDrift:
            return 4;
         case UpStairs3YDriftSmallOverlap:
            return 4;
         case UpStairs3RollDrift:
            return 7;
         case UpStairs3HugeDrift:
            return 8;
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
         case UpStairs2YawDrift:
            return 5;
         case UpStairs3YDriftSmallOverlap:
            return 5;
         case UpStairs3RollDrift:
            return 8;
         case UpStairs3HugeDrift:
            return 9;
      }
      return -1;
   }
   
   public String getNote()
   {
      switch (this)
      {
         case MultiplePlanes:
            return "";
         case DownStairsBigZDrift:
            return "";
         case UpStairs2YDrift:
            return "";
         case UpStairs3YDriftSmallOverlap:
         case UpStairs3RollDrift:
            return "";
      }
      return "";
   }
}
