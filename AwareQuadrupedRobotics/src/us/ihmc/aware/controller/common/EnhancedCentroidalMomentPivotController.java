package us.ihmc.aware.controller.common;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class EnhancedCentroidalMomentPivotController
{
   public final static double MINIMUM_COM_HEIGHT = 0.01;

   private double mass;
   private double gravity;
   private double comHeight;
   private final ReferenceFrame comFrame;

   public EnhancedCentroidalMomentPivotController(ReferenceFrame comFrame, double mass, double gravity, double comHeight)
   {
      this.mass = mass;
      this.gravity = gravity;
      this.comHeight = comHeight;
      this.comFrame = comFrame;
   }

   public void setComHeight(double comHeight)
   {
      this.comHeight = Math.max(comHeight, MINIMUM_COM_HEIGHT);
   }

   public double getComHeight()
   {
      return comHeight;
   }
   public double getNaturalFrequency()
   {
      return Math.sqrt(gravity / comHeight);
   }

   public double getTimeConstant()
   {
      return Math.sqrt(comHeight / gravity);
   }

   public void reset()
   {
   }

   public void compute(FrameVector comForceOutput, FramePoint cmpPositionInput)
   {
      ReferenceFrame comForceOutputFrame = comForceOutput.getReferenceFrame();
      ReferenceFrame cmpPositionInputFrame = cmpPositionInput.getReferenceFrame();
      comForceOutput.changeFrame(comFrame);
      cmpPositionInput.changeFrame(comFrame);

      // compute the horizontal components of the CoM force
      double omega = getNaturalFrequency();
      double fX = mass * Math.pow(omega, 2) * -cmpPositionInput.getX();
      double fY = mass * Math.pow(omega, 2) * -cmpPositionInput.getY();
      double fZ = mass * Math.pow(omega, 2) * -cmpPositionInput.getZ();
      comForceOutput.set(fX, fY, fZ);

      comForceOutput.changeFrame(comForceOutputFrame);
      cmpPositionInput.changeFrame(cmpPositionInputFrame);
   }
}
