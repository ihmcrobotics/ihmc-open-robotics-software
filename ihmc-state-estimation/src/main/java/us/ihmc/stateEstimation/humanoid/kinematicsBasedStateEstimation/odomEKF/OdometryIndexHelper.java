package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;

public class OdometryIndexHelper
{
   static final int stateSizePerLink = 16;
   static final int stateTranslationIndex = 0;
   static final int stateLinearVelocityIndex = 3;
   static final int stateOrientationIndex = 6;
   static final int stateAccelBiasIndex = 10;
   static final int stateGyroBiasIndex = 13;

   static final int errorSizePerLink = 15;
   static final int errorTranslationIndex = 0;
   static final int errorLinearVelocityIndex = 3;
   static final int errorOrientationIndex = 6;
   static final int errorAccelBiasIndex = 9;
   static final int errorGyroBiasIndex = 12;

   static final int measurementSizePerLink = 15;
   static final int measurementRelativeTranslationIndex = 0;
   static final int measurementRelativeOrientationErrorIndex = 3;
   static final int measurementRelativeVelocityIndex = 6;
   static final int measurementContactVelocityIndex = 9;
   static final int measurementAccelIndex = 12;

   public static int getFootPositionIndex(int footNumber)
   {
      return stateSizePerLink * (footNumber + 1) + stateTranslationIndex;
   }
}
