package us.ihmc.aware.controller.common;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.ArtifactList;

public class DivergentComponentOfMotionController
{
   private double dt;
   private double mass;
   private double gravity;
   private double comHeight;
   private final ReferenceFrame comFrame;
   private final PIDController[] pidController;
   private final FramePoint vrpPositionSetpoint;
   private final FramePoint cmpPositionSetpoint;

   YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
   ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

   YoFramePoint yoDcmPositionEstimate = new YoFramePoint("dcmPositionEstimate", ReferenceFrame.getWorldFrame(), registry);
   YoFramePoint yoDcmPositionSetpoint = new YoFramePoint("dcmPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);
   YoFrameVector yoDcmVelocitySetpoint = new YoFrameVector("dcmVelocitySetpoint", ReferenceFrame.getWorldFrame(), registry);
   YoFramePoint yoIcpPositionEstimate = new YoFramePoint("icpPositionEstimate", ReferenceFrame.getWorldFrame(), registry);
   YoFramePoint yoIcpPositionSetpoint = new YoFramePoint("icpPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);
   YoFrameVector yoIcpVelocitySetpoint = new YoFrameVector("icpVelocitySetpoint", ReferenceFrame.getWorldFrame(), registry);
   YoFramePoint yoVrpPositionSetpoint = new YoFramePoint("vrpPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);
   YoFramePoint yoCmpPositionSetpoint = new YoFramePoint("cmpPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);
   DoubleYoVariable yoLipNaturalFrequency = new DoubleYoVariable("lipNaturalFrequency", registry);

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
      vrpPositionSetpoint = new FramePoint();
      cmpPositionSetpoint = new FramePoint();

      YoGraphicPosition yoIcpPositionEstimateViz = new YoGraphicPosition("icpPositionEstimate", yoIcpPositionEstimate, 0.025, YoAppearance.Magenta());
      YoGraphicPosition yoIcpPositionSetpointViz = new YoGraphicPosition("icpPositionSetpoint", yoIcpPositionSetpoint, 0.025, YoAppearance.Blue());
      YoGraphicPosition yoCmpPositionSetpointViz = new YoGraphicPosition("cmpPositionSetpoint", yoCmpPositionSetpoint, 0.025, YoAppearance.Chartreuse());
      yoGraphicsList.add(yoIcpPositionEstimateViz);
      yoGraphicsList.add(yoIcpPositionSetpointViz);
      yoGraphicsList.add(yoCmpPositionSetpointViz);
      artifactList.add(yoIcpPositionEstimateViz.createArtifact());
      artifactList.add(yoIcpPositionSetpointViz.createArtifact());
      artifactList.add(yoCmpPositionSetpointViz.createArtifact());
      parentRegistry.addChild(registry);
   }

   public void registerGraphics(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   public void setComHeight(double comHeight)
   {
      this.comHeight = Math.max(comHeight, 0.001);
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
      return 1.0 / getNaturalFrequency();
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

   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      pidController[0].setProportionalGain(proportionalGainX);
      pidController[1].setProportionalGain(proportionalGainY);
      pidController[2].setProportionalGain(proportionalGainZ);
   }

   public void setDerivativeGains(double[] derivativeGains)
   {
      for (int i = 0; i < 3; i++)
      {
         pidController[i].setDerivativeGain(derivativeGains[i]);
      }
   }

   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      pidController[0].setDerivativeGain(derivativeGainX);
      pidController[1].setDerivativeGain(derivativeGainY);
      pidController[2].setDerivativeGain(derivativeGainZ);
   }

   public void setIntegralGains(double[] integralGains, double maxIntegralError)
   {
      for (int i = 0; i < 3; i++)
      {
         pidController[i].setIntegralGain(integralGains[i]);
         pidController[i].setMaxIntegralError(maxIntegralError);
      }
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

   public void setGains(double[] proportionalGains, double[] derivativeGains, double[] integralGains, double maxIntegralError)
   {
      setProportionalGains(proportionalGains);
      setDerivativeGains(derivativeGains);
      setIntegralGains(integralGains, maxIntegralError);
   }

   public void compute(FrameVector comForceCommand, FramePoint dcmPositionSetpoint, FrameVector dcmVelocitySetpoint, FramePoint dcmPositionEstimate)
   {
      ReferenceFrame comForceCommandFrame = comForceCommand.getReferenceFrame();
      ReferenceFrame dcmPositionSetpointFrame = dcmPositionSetpoint.getReferenceFrame();
      ReferenceFrame dcmPositionVelocityFrame = dcmVelocitySetpoint.getReferenceFrame();
      ReferenceFrame dcmPositionEstimateFrame = dcmPositionEstimate.getReferenceFrame();

      comForceCommand.changeFrame(comFrame);
      dcmPositionSetpoint.changeFrame(comFrame);
      dcmVelocitySetpoint.changeFrame(comFrame);
      dcmPositionEstimate.changeFrame(comFrame);
      vrpPositionSetpoint.changeFrame(comFrame);
      cmpPositionSetpoint.changeFrame(comFrame);

      double omega = getNaturalFrequency();
      double vrpX = dcmPositionEstimate.getX() - 1 / omega * (dcmVelocitySetpoint.getX() + pidController[0].compute(dcmPositionEstimate.getX(), dcmPositionSetpoint.getX(), 0, 0, dt));
      double vrpY = dcmPositionEstimate.getY() - 1 / omega * (dcmVelocitySetpoint.getY() + pidController[1].compute(dcmPositionEstimate.getY(), dcmPositionSetpoint.getY(), 0, 0, dt));
      double vrpZ = dcmPositionEstimate.getZ() - 1 / omega * (dcmVelocitySetpoint.getZ() + pidController[2].compute(dcmPositionEstimate.getZ(), dcmPositionSetpoint.getZ(), 0, 0, dt));
      double cmpX = vrpX;
      double cmpY = vrpY;
      double cmpZ = vrpZ - getComHeight();
      double fX = mass * Math.pow(omega, 2) * -cmpX;
      double fY = mass * Math.pow(omega, 2) * -cmpY;
      double fZ = mass * Math.pow(omega, 2) * -cmpZ;
      comForceCommand.set(fX, fY, fZ);
      vrpPositionSetpoint.set(vrpX, vrpY, vrpZ);
      cmpPositionSetpoint.set(cmpX, cmpY, cmpZ);

      yoDcmPositionEstimate.setAndMatchFrame(dcmPositionEstimate);
      yoDcmPositionSetpoint.setAndMatchFrame(dcmPositionSetpoint);
      yoDcmVelocitySetpoint.setAndMatchFrame(dcmVelocitySetpoint);
      yoIcpPositionEstimate.set(yoDcmPositionEstimate);
      yoIcpPositionEstimate.add(0, 0, -getComHeight());
      yoIcpPositionSetpoint.set(yoDcmPositionSetpoint);
      yoIcpPositionSetpoint.add(0, 0, -getComHeight());
      yoIcpVelocitySetpoint.set(yoDcmVelocitySetpoint);
      yoVrpPositionSetpoint.setAndMatchFrame(vrpPositionSetpoint);
      yoCmpPositionSetpoint.setAndMatchFrame(cmpPositionSetpoint);
      yoLipNaturalFrequency.set(getNaturalFrequency());

      comForceCommand.changeFrame(comForceCommandFrame);
      dcmPositionSetpoint.changeFrame(dcmPositionSetpointFrame);
      dcmVelocitySetpoint.changeFrame(dcmPositionVelocityFrame);
      dcmPositionEstimate.changeFrame(dcmPositionEstimateFrame);
   }
}
