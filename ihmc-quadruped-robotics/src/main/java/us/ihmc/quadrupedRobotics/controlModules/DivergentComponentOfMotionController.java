package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.LinearInvertedPendulumModel;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.parameter.DoubleArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.robotics.math.filters.RateLimitedYoFramePoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class DivergentComponentOfMotionController
{
   private final ReferenceFrame comZUpFrame;
   private final LinearInvertedPendulumModel lipModel;
   private final double controlDT;
   private final FramePoint3D vrpPositionSetpoint;
   private final FramePoint3D cmpPositionSetpoint;
   private final PIDController[] pidController;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleArrayParameter proportionalGainsParameter = parameterFactory.createDoubleArray("dcmPositionProportionalGains", 1, 1, 0);
   private final DoubleArrayParameter integralGainsParameter = parameterFactory.createDoubleArray("dcmPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter maxIntegralErrorParameter = parameterFactory.createDouble("dcmPositionMaxIntegralError", 0);

   private final RateLimitedYoFramePoint yoLimitedVrpPositionSetpoint;

   private final FramePoint3D dcmPositionEstimate = new FramePoint3D();
   private final FramePoint3D dcmPositionSetpoint = new FramePoint3D();
   private final FrameVector3D dcmVelocitySetpoint = new FrameVector3D();

   public DivergentComponentOfMotionController(ReferenceFrame comZUpFrame, double controlDT, LinearInvertedPendulumModel lipModel, YoVariableRegistry parentRegistry)
   {
      this.comZUpFrame = comZUpFrame;
      this.controlDT = controlDT;
      this.lipModel = lipModel;

      vrpPositionSetpoint = new FramePoint3D();
      cmpPositionSetpoint = new FramePoint3D();
      pidController = new PIDController[3];
      pidController[0] = new PIDController("dcmPositionX", registry);
      pidController[1] = new PIDController("dcmPositionY", registry);
      pidController[2] = new PIDController("dcmPositionZ", registry);

      DoubleParameter vrpPositionRateLimitParameter = parameterFactory.createDouble("vrpPositionRateLimit", Double.MAX_VALUE);
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
         pidController[i].setProportionalGain(proportionalGainsParameter.get(i));
         pidController[i].setIntegralGain(integralGainsParameter.get(i));
         pidController[i].setMaxIntegralError(maxIntegralErrorParameter.get());
      }

      double omega = lipModel.getNaturalFrequency();

      double xEffort = pidController[0].compute(this.dcmPositionEstimate.getX(), this.dcmPositionSetpoint.getX(), 0, 0, controlDT);
      double yEffort = pidController[1].compute(this.dcmPositionEstimate.getY(), this.dcmPositionSetpoint.getY(), 0, 0, controlDT);
      double zEffort = pidController[2].compute(this.dcmPositionEstimate.getZ(), this.dcmPositionSetpoint.getZ(), 0, 0, controlDT);

      vrpPositionSetpoint.setX(this.dcmPositionEstimate.getX() - 1 / omega * (this.dcmVelocitySetpoint.getX() + xEffort));
      vrpPositionSetpoint.setY(this.dcmPositionEstimate.getY() - 1 / omega * (this.dcmVelocitySetpoint.getY() + yEffort));
      vrpPositionSetpoint.setZ(this.dcmPositionEstimate.getZ() - 1 / omega * (this.dcmVelocitySetpoint.getZ() + zEffort));

      yoLimitedVrpPositionSetpoint.update(vrpPositionSetpoint);

      vrpPositionSetpoint.setIncludingFrame(yoLimitedVrpPositionSetpoint);
      vrpPositionSetpoint.changeFrame(vrpPositionSetpointToPack.getReferenceFrame());
      vrpPositionSetpointToPack.set(vrpPositionSetpoint);
   }
}
