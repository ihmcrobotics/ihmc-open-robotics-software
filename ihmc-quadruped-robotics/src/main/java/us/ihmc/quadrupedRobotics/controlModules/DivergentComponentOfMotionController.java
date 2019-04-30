package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.quadrupedRobotics.controller.toolbox.LinearInvertedPendulumModel;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPID3DGains;
import us.ihmc.robotics.math.filters.RateLimitedYoFramePoint;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

public class DivergentComponentOfMotionController
{
   private final ReferenceFrame comZUpFrame;
   private final LinearInvertedPendulumModel lipModel;
   private final double controlDT;
   private final FramePoint3D vrpPositionSetpoint;
   private final FramePoint3D cmpPositionSetpoint;
   private final PIDController[] pidController;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ParameterizedPID3DGains dcmPositionGains;
   private final DoubleParameter vrpPositionRateLimitParameter = new DoubleParameter("vrpPositionRateLimit", registry, Double.MAX_VALUE);

   private final YoFrameVector3D vrpControllerEffort;

   private final RateLimitedYoFramePoint yoLimitedVrpPositionSetpoint;

   private final FramePoint3D dcmPositionEstimate = new FramePoint3D();
   private final FramePoint3D dcmPositionSetpoint = new FramePoint3D();
   private final FrameVector3D dcmVelocitySetpoint = new FrameVector3D();

   public DivergentComponentOfMotionController(ReferenceFrame comZUpFrame, double controlDT, LinearInvertedPendulumModel lipModel, YoVariableRegistry parentRegistry)
   {
      this.comZUpFrame = comZUpFrame;
      this.controlDT = controlDT;
      this.lipModel = lipModel;

      vrpControllerEffort = new YoFrameVector3D("vrpControllerEffort", comZUpFrame, registry);

      vrpPositionSetpoint = new FramePoint3D();
      cmpPositionSetpoint = new FramePoint3D();
      pidController = new PIDController[3];
      pidController[0] = new PIDController("dcmPositionX", registry);
      pidController[1] = new PIDController("dcmPositionY", registry);
      pidController[2] = new PIDController("dcmPositionZ", registry);

      DefaultPID3DGains defaultDcmPositionGains = new DefaultPID3DGains();
      defaultDcmPositionGains.setProportionalGains(1.0, 1.0, 0.0);
      dcmPositionGains = new ParameterizedPID3DGains("_DcmPosition", GainCoupling.NONE, true, defaultDcmPositionGains, registry);

      yoLimitedVrpPositionSetpoint = new RateLimitedYoFramePoint("vrpPositionSetpointInCoMZUpFrame", "", registry, vrpPositionRateLimitParameter, controlDT, comZUpFrame);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      for (int i = 0; i < 3; i++)
      {
         pidController[i].resetIntegrator();
      }
      yoLimitedVrpPositionSetpoint.reset();
   }

   public void compute(FixedFramePoint3DBasics vrpPositionSetpointToPack, FramePoint3DReadOnly dcmPositionEstimate, FramePoint3DReadOnly dcmPositionSetpoint,
                       FrameVector3DReadOnly dcmVelocitySetpoint)
   {
      this.dcmPositionEstimate.setIncludingFrame(dcmPositionEstimate);
      this.dcmPositionSetpoint.setIncludingFrame(dcmPositionSetpoint);
      this.dcmVelocitySetpoint.setIncludingFrame(dcmVelocitySetpoint);

      this.dcmPositionSetpoint.changeFrame(comZUpFrame);
      this.dcmVelocitySetpoint.changeFrame(comZUpFrame);
      this.dcmPositionEstimate.changeFrame(comZUpFrame);

      vrpPositionSetpoint.changeFrame(comZUpFrame);
      cmpPositionSetpoint.changeFrame(comZUpFrame);

      for (int i = 0; i < 3; i++)
      {
         pidController[i].setProportionalGain(dcmPositionGains.getProportionalGains()[i]);
         pidController[i].setIntegralGain(dcmPositionGains.getIntegralGains()[i]);
         pidController[i].setMaxIntegralError(dcmPositionGains.getIntegralGains()[i]);
         pidController[i].setMaximumOutputLimit(dcmPositionGains.getMaximumFeedback());
      }

      double omega = lipModel.getNaturalFrequency();

      vrpControllerEffort.setX(pidController[0].compute(this.dcmPositionEstimate.getX(), this.dcmPositionSetpoint.getX(), 0, 0, controlDT));
      vrpControllerEffort.setY(pidController[1].compute(this.dcmPositionEstimate.getY(), this.dcmPositionSetpoint.getY(), 0, 0, controlDT));
      vrpControllerEffort.setZ(pidController[2].compute(this.dcmPositionEstimate.getZ(), this.dcmPositionSetpoint.getZ(), 0, 0, controlDT));

      vrpPositionSetpoint.add(this.dcmVelocitySetpoint, vrpControllerEffort);
      vrpPositionSetpoint.scaleAdd(-1.0 / omega, this.dcmPositionEstimate);

      yoLimitedVrpPositionSetpoint.update(vrpPositionSetpoint);

      vrpPositionSetpoint.setIncludingFrame(yoLimitedVrpPositionSetpoint);
      vrpPositionSetpoint.changeFrame(vrpPositionSetpointToPack.getReferenceFrame());
      vrpPositionSetpointToPack.set(vrpPositionSetpoint);
   }
}
