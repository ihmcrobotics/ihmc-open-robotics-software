package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPID3DGains;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DivergentComponentOfMotionController
{
   public static class Setpoints
   {
      private final FramePoint3D dcmPosition = new FramePoint3D();
      private final FrameVector3D dcmVelocity = new FrameVector3D();

      public void initialize(FramePoint3D dcmPositionEstimate)
      {
         dcmPosition.setIncludingFrame(dcmPositionEstimate);
         dcmVelocity.setToZero();
      }

      public FramePoint3D getDcmPosition()
      {
         return dcmPosition;
      }

      public FrameVector3D getDcmVelocity()
      {
         return dcmVelocity;
      }
   }

   private final ReferenceFrame comZUpFrame;
   private final LinearInvertedPendulumModel lipModel;
   private final double controlDT;
   private final FramePoint3D vrpPositionSetpoint;
   private final FramePoint3D cmpPositionSetpoint;
   private final PIDController[] pidController;
   private final YoPID3DGains pidControllerGains;

   YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
   ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

   YoFramePoint yoIcpPositionSetpoint = new YoFramePoint("icpPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);
   YoFrameVector yoIcpVelocitySetpoint = new YoFrameVector("icpVelocitySetpoint", ReferenceFrame.getWorldFrame(), registry);
   YoFramePoint yoVrpPositionSetpoint = new YoFramePoint("vrpPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);
   YoFramePoint yoCmpPositionSetpoint = new YoFramePoint("cmpPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);
   YoFramePoint yoDcmPositionSetpoint = new YoFramePoint("dcmPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);
   YoFrameVector yoDcmVelocitySetpoint = new YoFrameVector("dcmVelocitySetpoint", ReferenceFrame.getWorldFrame(), registry);

   YoDouble yoVrpPositionRateLimit;
   RateLimitedYoVariable yoVrpPositionSetpointX;
   RateLimitedYoVariable yoVrpPositionSetpointY;
   RateLimitedYoVariable yoVrpPositionSetpointZ;

   public DivergentComponentOfMotionController(ReferenceFrame comZUpFrame, double controlDT, LinearInvertedPendulumModel lipModel,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
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
      pidControllerGains = new DefaultYoPID3DGains("dcmPosition", GainCoupling.NONE, true, registry);

      yoVrpPositionRateLimit = new YoDouble("vrpPositionRateLimit", registry);
      yoVrpPositionRateLimit.set(Double.MAX_VALUE);
      yoVrpPositionSetpointX = new RateLimitedYoVariable("vrpPositionSetpointXInComZUpFrame", registry, yoVrpPositionRateLimit, controlDT);
      yoVrpPositionSetpointY = new RateLimitedYoVariable("vrpPositionSetpointYInComZUpFrame", registry, yoVrpPositionRateLimit, controlDT);
      yoVrpPositionSetpointZ = new RateLimitedYoVariable("vrpPositionSetpointZInComZUpFrame", registry, yoVrpPositionRateLimit, controlDT);

      YoGraphicPosition yoIcpPositionSetpointViz = new YoGraphicPosition("icpPositionSetpoint", yoIcpPositionSetpoint, 0.025, YoAppearance.Blue());
      YoGraphicPosition yoCmpPositionSetpointViz = new YoGraphicPosition("cmpPositionSetpoint", yoCmpPositionSetpoint, 0.025, YoAppearance.Chartreuse());
      yoGraphicsList.add(yoIcpPositionSetpointViz);
      yoGraphicsList.add(yoCmpPositionSetpointViz);
      artifactList.add(yoIcpPositionSetpointViz.createArtifact());
      artifactList.add(yoCmpPositionSetpointViz.createArtifact());
      parentRegistry.addChild(registry);
      graphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      graphicsListRegistry.registerArtifactList(artifactList);
   }

   public ReferenceFrame getReferenceFrame()
   {
      return comZUpFrame;
   }

   public LinearInvertedPendulumModel getLinearInvertedPendulumModel()
   {
      return lipModel;
   }

   public void reset()
   {
      for (int i = 0; i < 3; i++)
      {
         pidController[i].resetIntegrator();
      }
      yoVrpPositionSetpointX.reset();
      yoVrpPositionSetpointY.reset();
      yoVrpPositionSetpointZ.reset();
   }

   public YoPID3DGains getGains()
   {
      return pidControllerGains;
   }

   public void setVrpPositionRateLimit(double vrpPositionRateLimit)
   {
      yoVrpPositionRateLimit.set(vrpPositionRateLimit);
   }

   public void compute(FrameVector3D comForceCommand, Setpoints setpoints, FramePoint3D dcmPositionEstimate)
   {
      FramePoint3D dcmPositionSetpoint = setpoints.getDcmPosition();
      FrameVector3D dcmVelocitySetpoint = setpoints.getDcmVelocity();
      ReferenceFrame comForceCommandFrame = comForceCommand.getReferenceFrame();
      ReferenceFrame dcmPositionSetpointFrame = dcmPositionSetpoint.getReferenceFrame();
      ReferenceFrame dcmPositionVelocityFrame = dcmVelocitySetpoint.getReferenceFrame();
      ReferenceFrame dcmPositionEstimateFrame = dcmPositionEstimate.getReferenceFrame();

      comForceCommand.changeFrame(comZUpFrame);
      dcmPositionSetpoint.changeFrame(comZUpFrame);
      dcmVelocitySetpoint.changeFrame(comZUpFrame);
      dcmPositionEstimate.changeFrame(comZUpFrame);
      vrpPositionSetpoint.changeFrame(comZUpFrame);
      cmpPositionSetpoint.changeFrame(comZUpFrame);

      for (int i = 0; i < 3; i++)
      {
         pidController[i].setProportionalGain(pidControllerGains.getProportionalGains()[i]);
         pidController[i].setIntegralGain(pidControllerGains.getIntegralGains()[i]);
         pidController[i].setMaxIntegralError(pidControllerGains.getMaximumIntegralError());
      }

      double omega = lipModel.getNaturalFrequency();
      vrpPositionSetpoint.setX(dcmPositionEstimate.getX() - 1 / omega * (dcmVelocitySetpoint.getX() + pidController[0]
            .compute(dcmPositionEstimate.getX(), dcmPositionSetpoint.getX(), 0, 0, controlDT)));
      vrpPositionSetpoint.setY(dcmPositionEstimate.getY() - 1 / omega * (dcmVelocitySetpoint.getY() + pidController[1]
            .compute(dcmPositionEstimate.getY(), dcmPositionSetpoint.getY(), 0, 0, controlDT)));
      vrpPositionSetpoint.setZ(dcmPositionEstimate.getZ() - 1 / omega * (dcmVelocitySetpoint.getZ() + pidController[2]
            .compute(dcmPositionEstimate.getZ(), dcmPositionSetpoint.getZ(), 0, 0, controlDT)));
      yoVrpPositionSetpointX.update(vrpPositionSetpoint.getX());
      yoVrpPositionSetpointY.update(vrpPositionSetpoint.getY());
      yoVrpPositionSetpointZ.update(vrpPositionSetpoint.getZ());
      vrpPositionSetpoint.setX(yoVrpPositionSetpointX.getDoubleValue());
      vrpPositionSetpoint.setY(yoVrpPositionSetpointY.getDoubleValue());
      vrpPositionSetpoint.setZ(yoVrpPositionSetpointZ.getDoubleValue());
      cmpPositionSetpoint.set(vrpPositionSetpoint);
      cmpPositionSetpoint.add(0, 0, -lipModel.getComHeight());
      lipModel.computeComForce(comForceCommand, cmpPositionSetpoint);

      yoDcmPositionSetpoint.setAndMatchFrame(dcmPositionSetpoint);
      yoDcmVelocitySetpoint.setAndMatchFrame(dcmVelocitySetpoint);
      yoIcpPositionSetpoint.set(yoDcmPositionSetpoint);
      yoIcpPositionSetpoint.add(0, 0, -lipModel.getComHeight());
      yoIcpVelocitySetpoint.set(yoDcmVelocitySetpoint);
      yoVrpPositionSetpoint.setAndMatchFrame(vrpPositionSetpoint);
      yoCmpPositionSetpoint.setAndMatchFrame(cmpPositionSetpoint);

      comForceCommand.changeFrame(comForceCommandFrame);
      dcmPositionSetpoint.changeFrame(dcmPositionSetpointFrame);
      dcmVelocitySetpoint.changeFrame(dcmPositionVelocityFrame);
      dcmPositionEstimate.changeFrame(dcmPositionEstimateFrame);
   }
}
