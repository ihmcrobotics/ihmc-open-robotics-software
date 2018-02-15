package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPID3DGains;
import us.ihmc.robotics.math.filters.RateLimitedYoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import static us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType.BALL_WITH_CROSS;
import static us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS;

public class DivergentComponentOfMotionController
{
   private final ReferenceFrame comZUpFrame;
   private final LinearInvertedPendulumModel lipModel;
   private final double controlDT;
   private final FramePoint3D vrpPositionSetpoint;
   private final FramePoint3D cmpPositionSetpoint;
   private final PIDController[] pidController;
   private final YoPID3DGains pidControllerGains;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFramePoint yoIcpPositionSetpoint = new YoFramePoint("icpPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector yoIcpVelocitySetpoint = new YoFrameVector("icpVelocitySetpoint", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint yoVrpPositionSetpoint = new YoFramePoint("vrpPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint yoCmpPositionSetpoint = new YoFramePoint("cmpPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint yoDcmPositionSetpoint = new YoFramePoint("dcmPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector yoDcmVelocitySetpoint = new YoFrameVector("dcmVelocitySetpoint", ReferenceFrame.getWorldFrame(), registry);

   private final YoDouble yoVrpPositionRateLimit;
   private final RateLimitedYoFramePoint yoLimitedVrpPositionSetpoint;

   private final FramePoint3D dcmPositionSetpoint = new FramePoint3D();
   private final FrameVector3D dcmVelocitySetpoint = new FrameVector3D();

   public DivergentComponentOfMotionController(ReferenceFrame comZUpFrame, double controlDT, LinearInvertedPendulumModel lipModel,
                                               YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this(comZUpFrame, controlDT, lipModel, "", parentRegistry, graphicsListRegistry);
   }

   public DivergentComponentOfMotionController(ReferenceFrame comZUpFrame, double controlDT, LinearInvertedPendulumModel lipModel, String nameSuffix,
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

      yoLimitedVrpPositionSetpoint = new RateLimitedYoFramePoint("vrpPositionSetpointInCoMZUpFrame", "", registry, yoVrpPositionRateLimit, controlDT, comZUpFrame);

      parentRegistry.addChild(registry);

      if (graphicsListRegistry != null)
         createGraphics(nameSuffix, graphicsListRegistry);
   }

   private void createGraphics(String suffix, YoGraphicsListRegistry graphicsListRegistry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoGraphicPosition yoIcpPositionSetpointViz = new YoGraphicPosition("icpPositionSetpoint" + suffix, yoIcpPositionSetpoint, 0.01, YoAppearance.Yellow(), BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition yoCmpPositionSetpointViz = new YoGraphicPosition("cmpPositionSetpoint" + suffix, yoCmpPositionSetpoint, 0.01, YoAppearance.Purple(), BALL_WITH_CROSS);

      yoGraphicsList.add(yoIcpPositionSetpointViz);
      yoGraphicsList.add(yoCmpPositionSetpointViz);
      artifactList.add(yoIcpPositionSetpointViz.createArtifact());
      artifactList.add(yoCmpPositionSetpointViz.createArtifact());
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
      yoLimitedVrpPositionSetpoint.reset();
   }

   public YoPID3DGains getGains()
   {
      return pidControllerGains;
   }

   public void setVrpPositionRateLimit(double vrpPositionRateLimit)
   {
      yoVrpPositionRateLimit.set(vrpPositionRateLimit);
   }

   public void compute(FrameVector3D comForceCommand,  FramePoint3D dcmPositionEstimate, FramePoint3D dcmPositionSetpoint, FrameVector3D dcmVelocitySetpoint)
   {
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

      double xEffort = pidController[0].compute(dcmPositionEstimate.getX(), dcmPositionSetpoint.getX(), 0, 0, controlDT);
      double yEffort = pidController[1].compute(dcmPositionEstimate.getY(), dcmPositionSetpoint.getY(), 0, 0, controlDT);
      double zEffort = pidController[2].compute(dcmPositionEstimate.getZ(), dcmPositionSetpoint.getZ(), 0, 0, controlDT);

      vrpPositionSetpoint.setX(dcmPositionEstimate.getX() - 1 / omega * (dcmVelocitySetpoint.getX() + xEffort));
      vrpPositionSetpoint.setY(dcmPositionEstimate.getY() - 1 / omega * (dcmVelocitySetpoint.getY() + yEffort));
      vrpPositionSetpoint.setZ(dcmPositionEstimate.getZ() - 1 / omega * (dcmVelocitySetpoint.getZ() + zEffort));

      yoLimitedVrpPositionSetpoint.update(vrpPositionSetpoint);

      cmpPositionSetpoint.set(yoLimitedVrpPositionSetpoint);
      cmpPositionSetpoint.subZ(lipModel.getComHeight());
      lipModel.computeComForce(comForceCommand, cmpPositionSetpoint);

      yoDcmPositionSetpoint.setAndMatchFrame(dcmPositionSetpoint);
      yoDcmVelocitySetpoint.setAndMatchFrame(dcmVelocitySetpoint);

      yoIcpPositionSetpoint.set(yoDcmPositionSetpoint);
      yoIcpPositionSetpoint.subZ(lipModel.getComHeight());
      yoIcpVelocitySetpoint.set(yoDcmVelocitySetpoint);

      yoVrpPositionSetpoint.setAndMatchFrame(yoLimitedVrpPositionSetpoint);
      yoCmpPositionSetpoint.setAndMatchFrame(cmpPositionSetpoint);

      comForceCommand.changeFrame(comForceCommandFrame);
      dcmPositionSetpoint.changeFrame(dcmPositionSetpointFrame);
      dcmVelocitySetpoint.changeFrame(dcmPositionVelocityFrame);
      dcmPositionEstimate.changeFrame(dcmPositionEstimateFrame);
   }

   public void initializeSetpoint(FramePoint3D dcmPositionEstimate)
   {
      dcmPositionSetpoint.setIncludingFrame(dcmPositionEstimate);
      dcmVelocitySetpoint.setToZero();
   }

   public FramePoint3D getDCMPositionSetpoint()
   {
      return dcmPositionSetpoint;
   }

   public FrameVector3D getDCMVelocitySetpoint()
   {
      return dcmVelocitySetpoint;
   }
}
