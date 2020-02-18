package us.ihmc.commonWalkingControlModules.controlModules.foot;

public interface FootRotationDetector
{
   boolean compute();

   boolean isRotating();

   void reset();
}
