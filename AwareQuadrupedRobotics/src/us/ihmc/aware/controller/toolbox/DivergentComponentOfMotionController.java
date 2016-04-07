package us.ihmc.aware.controller.toolbox;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.controllers.EuclideanPositionController;
import us.ihmc.robotics.controllers.YoEuclideanPositionGains;
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

   private double mass;
   private double gravity;
   private double comHeight;
   private final ReferenceFrame comFrame;
   private final FramePoint vrpPositionSetpoint;
   private final FramePoint cmpPositionSetpoint;
   private final FrameVector dcmVelocityEstimate;
   private final FrameVector dcmPositionControllerFeedback;
   private final FrameVector dcmPositionControllerFeedforward;
   private final EuclideanPositionController dcmPositionController;
   private final YoEuclideanPositionGains dcmPositionControllerGains;

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

   public DivergentComponentOfMotionController(ReferenceFrame comFrame, double controlDT, double mass, double gravity, double comHeight, YoVariableRegistry parentRegistry)
   {
      this.mass = mass;
      this.gravity = gravity;
      this.comHeight = comHeight;
      this.comFrame = comFrame;

      vrpPositionSetpoint = new FramePoint();
      cmpPositionSetpoint = new FramePoint();
      dcmVelocityEstimate = new FrameVector();
      dcmPositionControllerFeedback = new FrameVector();
      dcmPositionControllerFeedforward = new FrameVector();
      dcmPositionController = new EuclideanPositionController("dcmPosition", comFrame, controlDT, registry);
      dcmPositionControllerGains = new YoEuclideanPositionGains("dcmPosition", registry);

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
      dcmPositionController.reset();
   }

   public YoEuclideanPositionGains getGains()
   {
      return dcmPositionControllerGains;
   }

   public void compute(FrameVector comForceCommand, Setpoints setpoints, FramePoint dcmPositionEstimate)
   {
      FramePoint dcmPositionSetpoint = setpoints.getDcmPosition();
      FrameVector dcmVelocitySetpoint = setpoints.getDcmVelocity();
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
      dcmPositionControllerFeedback.changeFrame(comFrame);

      dcmVelocityEstimate.setToZero();
      dcmPositionControllerFeedforward.setToZero();
      dcmPositionController.setGains(dcmPositionControllerGains);
      dcmPositionController.compute(dcmPositionControllerFeedback, dcmPositionSetpoint, dcmVelocitySetpoint, dcmVelocityEstimate, dcmPositionControllerFeedforward);

      double omega = getNaturalFrequency();
      vrpPositionSetpoint.set(dcmPositionControllerFeedback);
      vrpPositionSetpoint.scale(-1.0 / omega);
      vrpPositionSetpoint.add(dcmPositionEstimate);
      cmpPositionSetpoint.add(0, 0, -getComHeight());
      comForceCommand.set(cmpPositionSetpoint);
      comForceCommand.scale(-mass * Math.pow(omega, 2));

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
