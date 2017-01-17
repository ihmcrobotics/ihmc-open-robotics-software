package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.controllers.YoEuclideanPositionGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class DivergentComponentOfMotionController
{
   public static class Setpoints
   {
      private final FramePoint dcmPosition = new FramePoint();
      private final FrameVector dcmVelocity = new FrameVector();

      public void initialize(FramePoint dcmPositionEstimate)
      {
         dcmPosition.setIncludingFrame(dcmPositionEstimate);
         dcmVelocity.setToZero();
      }

      public FramePoint getDcmPosition()
      {
         return dcmPosition;
      }

      public FrameVector getDcmVelocity()
      {
         return dcmVelocity;
      }
   }

   private final ReferenceFrame comZUpFrame;
   private final LinearInvertedPendulumModel lipModel;
   private final double controlDT;
   private final FramePoint vrpPositionSetpoint;
   private final FramePoint cmpPositionSetpoint;
   private final PIDController[] pidController;
   private final YoEuclideanPositionGains pidControllerGains;

   YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
   ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

   YoFramePoint yoIcpPositionSetpoint = new YoFramePoint("icpPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);
   YoFrameVector yoIcpVelocitySetpoint = new YoFrameVector("icpVelocitySetpoint", ReferenceFrame.getWorldFrame(), registry);
   YoFramePoint yoVrpPositionSetpoint = new YoFramePoint("vrpPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);
   YoFramePoint yoCmpPositionSetpoint = new YoFramePoint("cmpPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);
   YoFramePoint yoDcmPositionSetpoint = new YoFramePoint("dcmPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);
   YoFrameVector yoDcmVelocitySetpoint = new YoFrameVector("dcmVelocitySetpoint", ReferenceFrame.getWorldFrame(), registry);

   DoubleYoVariable yoVrpPositionRateLimit;
   RateLimitedYoVariable yoVrpPositionSetpointX;
   RateLimitedYoVariable yoVrpPositionSetpointY;
   RateLimitedYoVariable yoVrpPositionSetpointZ;

   public DivergentComponentOfMotionController(ReferenceFrame comZUpFrame, double controlDT, LinearInvertedPendulumModel lipModel,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.comZUpFrame = comZUpFrame;
      this.controlDT = controlDT;
      this.lipModel = lipModel;

      vrpPositionSetpoint = new FramePoint();
      cmpPositionSetpoint = new FramePoint();
      pidController = new PIDController[3];
      pidController[0] = new PIDController("dcmPositionX", registry);
      pidController[1] = new PIDController("dcmPositionY", registry);
      pidController[2] = new PIDController("dcmPositionZ", registry);
      pidControllerGains = new YoEuclideanPositionGains("dcmPosition", registry);

      yoVrpPositionRateLimit = new DoubleYoVariable("vrpPositionRateLimit", registry);
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

   public YoEuclideanPositionGains getGains()
   {
      return pidControllerGains;
   }

   public void setVrpPositionRateLimit(double vrpPositionRateLimit)
   {
      yoVrpPositionRateLimit.set(vrpPositionRateLimit);
   }

   public void compute(FrameVector comForceCommand, Setpoints setpoints, FramePoint dcmPositionEstimate)
   {
      FramePoint dcmPositionSetpoint = setpoints.getDcmPosition();
      FrameVector dcmVelocitySetpoint = setpoints.getDcmVelocity();
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
