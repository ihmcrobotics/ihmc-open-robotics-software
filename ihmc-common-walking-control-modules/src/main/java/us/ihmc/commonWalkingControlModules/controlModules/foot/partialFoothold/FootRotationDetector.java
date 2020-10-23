package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

public interface FootRotationDetector
{
   boolean compute();

   boolean isRotating();

   void reset();
}
