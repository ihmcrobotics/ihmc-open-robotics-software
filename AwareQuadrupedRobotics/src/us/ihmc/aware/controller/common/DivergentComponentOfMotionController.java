package us.ihmc.aware.controller.common;

import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class DivergentComponentOfMotionController
{
   public final static double MINIMUM_COM_HEIGHT = 0.01;

   private double dt;
   private double mass;
   private double gravity;
   private double comHeight;
   private final ReferenceFrame comFrame;
   private final PIDController[] pidController;

   public DivergentComponentOfMotionController(String suffix, ReferenceFrame comFrame, double dt, double mass, double gravity, double comHeight, YoVariableRegistry parentRegistry)
   {
      this.dt = dt;
      this.mass = mass;
      this.gravity = gravity;
      this.comHeight = comHeight;
      this.comFrame = comFrame;

      pidController = new PIDController[3];
      pidController[0] = new PIDController(suffix + "X", parentRegistry);
      pidController[1] = new PIDController(suffix + "Y", parentRegistry);
      pidController[2] = new PIDController(suffix + "Z", parentRegistry);
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
      for (int i = 0; i < 3; i++)
      {
         pidController[i].resetIntegrator();
      }
   }

   public void setProportionalGains(double[] proportionalGains)
   {
      for (int i = 0; i < 3; i++)
      {
         pidController[i].setProportionalGain(proportionalGains[i]);
      }
   }

   public void setIntegralGains(double[] integralGains, double maxIntegralError)
   {
      for (int i = 0; i < 3; i++)
      {
         pidController[i].setIntegralGain(integralGains[i]);
         pidController[i].setMaxIntegralError(maxIntegralError);
      }
   }

   public void setIntegralGains(double[] integralGains, double[] maxIntegralErrors)
   {
      for (int i = 0; i < 3; i++)
      {
         pidController[i].setIntegralGain(integralGains[i]);
         pidController[i].setMaxIntegralError(maxIntegralErrors[i]);
      }
   }

   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      pidController[0].setProportionalGain(proportionalGainX);
      pidController[1].setProportionalGain(proportionalGainY);
      pidController[2].setProportionalGain(proportionalGainZ);
   }

   public void setIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
      pidController[0].setIntegralGain(integralGainX);
      pidController[1].setIntegralGain(integralGainY);
      pidController[2].setIntegralGain(integralGainZ);
      pidController[0].setMaxIntegralError(maxIntegralError);
      pidController[1].setMaxIntegralError(maxIntegralError);
      pidController[2].setMaxIntegralError(maxIntegralError);
   }

   public void setIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralErrorX, double maxIntegralErrorY, double maxIntegralErrorZ)
   {
      pidController[0].setIntegralGain(integralGainX);
      pidController[1].setIntegralGain(integralGainY);
      pidController[2].setIntegralGain(integralGainZ);
      pidController[0].setMaxIntegralError(maxIntegralErrorX);
      pidController[1].setMaxIntegralError(maxIntegralErrorY);
      pidController[2].setMaxIntegralError(maxIntegralErrorZ);
   }

   public void compute(FrameVector comForceOutput, FramePoint vrpPositionOutput, FramePoint cmpPositionOutput, FramePoint dcmPositionSetpoint, FrameVector dcmVelocitySetpoint, FramePoint dcmPositionEstimate)
   {
      ReferenceFrame comForceOutputFrame = comForceOutput.getReferenceFrame();
      ReferenceFrame vrpPositionOutputFrame = vrpPositionOutput.getReferenceFrame();
      ReferenceFrame cmpPositionOutputFrame = cmpPositionOutput.getReferenceFrame();
      ReferenceFrame dcmPositionSetpointFrame = dcmPositionSetpoint.getReferenceFrame();
      ReferenceFrame dcmPositionVelocityFrame = dcmVelocitySetpoint.getReferenceFrame();
      ReferenceFrame dcmPositionEstimateFrame = dcmPositionEstimate.getReferenceFrame();

      comForceOutput.changeFrame(comFrame);
      vrpPositionOutput.changeFrame(comFrame);
      cmpPositionOutput.changeFrame(comFrame);
      dcmPositionSetpoint.changeFrame(comFrame);
      dcmPositionEstimate.changeFrame(comFrame);

      double omega = getNaturalFrequency();
      double vrpX = dcmPositionEstimate.getX() - 1 / omega * (dcmVelocitySetpoint.getX() + pidController[0].compute(dcmPositionEstimate.getX(), dcmPositionSetpoint.getX(), 0, 0, dt));
      double vrpY = dcmPositionEstimate.getY() - 1 / omega * (dcmVelocitySetpoint.getY() + pidController[1].compute(dcmPositionEstimate.getY(), dcmPositionSetpoint.getY(), 0, 0, dt));
      double vrpZ = dcmPositionEstimate.getZ() - 1 / omega * (dcmVelocitySetpoint.getZ() + pidController[2].compute(dcmPositionEstimate.getZ(), dcmPositionSetpoint.getZ(), 0, 0, dt));
      double cmpX = vrpX;
      double cmpY = vrpY;
      double cmpZ = vrpZ - comHeight;
      double fX = mass * Math.pow(omega, 2) * -cmpX;
      double fY = mass * Math.pow(omega, 2) * -cmpY;
      double fZ = mass * Math.pow(omega, 2) * -cmpZ;
      comForceOutput.set(fX, fY, fZ);
      cmpPositionOutput.set(cmpX, cmpY, cmpZ);
      vrpPositionOutput.set(vrpX, vrpY, vrpZ);

      comForceOutput.changeFrame(comForceOutputFrame);
      vrpPositionOutput.changeFrame(vrpPositionOutputFrame);
      cmpPositionOutput.changeFrame(cmpPositionOutputFrame);
      dcmPositionSetpoint.changeFrame(dcmPositionSetpointFrame);
      dcmPositionEstimate.changeFrame(dcmPositionEstimateFrame);
   }
}
