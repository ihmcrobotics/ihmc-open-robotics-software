package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator;

public class MomentumEstimatorIndexProvider
{
   // State
   private final int comPosition;
   private final int linearMomentum;
   private final int angularMomentum;
   private final int comPositionOffset;
   private final int linearMomentumOffset;
   private final int size;

   public static MomentumEstimatorIndexProvider newMomentumEstimator()
   {
      return new MomentumEstimatorIndexProvider(3, 3, 3, 0, 0);
   }

   public static MomentumEstimatorIndexProvider newOffsetEstimator()
   {
      return new MomentumEstimatorIndexProvider(3, 3, 3, 2, 3);
   }

   public MomentumEstimatorIndexProvider(int comPositionSize,
                                         int linearMomentumSize,
                                         int angularMomentumSize,
                                         int comPositionOffsetSize,
                                         int linearMomentumOffsetSize)
   {
      int currentIndex = 0;

      this.comPosition = comPositionSize > 0 ? currentIndex : -1;
      currentIndex += comPositionSize;
      this.linearMomentum = linearMomentumSize > 0 ? currentIndex : -1;
      currentIndex += linearMomentumSize;
      this.angularMomentum = angularMomentumSize > 0 ? currentIndex : -1;
      currentIndex += angularMomentumSize;
      this.comPositionOffset = comPositionOffsetSize > 0 ? currentIndex : -1;
      currentIndex += comPositionOffsetSize;
      this.linearMomentumOffset = linearMomentumOffsetSize > 0 ? currentIndex : -1;

      size = currentIndex + linearMomentumOffsetSize;
   }

   public boolean hasCoMPosition()
   {
      return comPosition > 0;
   }

   public boolean hasLinearMomentum()
   {
      return linearMomentum > 0;
   }

   public boolean hasAngularMomentum()
   {
      return angularMomentum > 0;
   }

   public boolean hasCoMPositionOffset()
   {
      return comPositionOffset > 0;
   }

   public boolean hasLinearMomentumOffset()
   {
      return linearMomentumOffset > 0;
   }

   public int getCoMPosition()
   {
      return comPosition;
   }

   public int getLinearMomentum()
   {
      return linearMomentum;
   }

   public int getAngularMomentum()
   {
      return angularMomentum;
   }

   public int getCoMPositionOffset()
   {
      return comPositionOffset;
   }

   public int getLinearMomentumOffset()
   {
      return linearMomentumOffset;
   }

   public int getSize()
   {
      return size;
   }
}
