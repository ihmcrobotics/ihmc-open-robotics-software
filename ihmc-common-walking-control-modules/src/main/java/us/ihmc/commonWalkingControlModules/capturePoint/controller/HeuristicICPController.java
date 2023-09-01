package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Purple;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicPoint2D;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.ParameterizedICPControlGains;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * HeuristicICPController controls the ICP using a few simple heuristics, including: a) Use a simple
 * proportional controller on the ICP error for a feedback term. b) Use the perfect CMP for a
 * feedforward term. c) If there is a large perpendicular error, ignore the feedforward term and
 * only do the feedback term. d) Project the unconstrained CoP answer into the foot along the Vector
 * from the unconstrained CMP to the ICP, but do not project too far into the foot, and also do not
 * project closer to the ICP than a certain threshold.
 **/
public class HeuristicICPController implements ICPControllerInterface
{
   //TODO: Similar to the optimization ICP controller, if control cannot be achieved well due to the large difference between perfectCoP and perfectCMP, then change that distance and set a YoBoolean, like useCMPFeedback and useAngularMomentum.
   //TODO: Perhaps add either an integrator, or a constant feedback term to better achieve the control?
   //TODO: Perhaps add a LPF or rate limit, or add them outside this class?

   private static final boolean VISUALIZE = true;

   private final String yoNamePrefix = "heuristic";
   private final YoRegistry registry = new YoRegistry("HeuristicICPController");
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   // Control Parameters:
   private final YoDouble pureFeedbackErrorThreshold = new YoDouble(yoNamePrefix + "PureFeedbackErrorThresh",
                                                                    "Amount of ICP error before feedforward terms are ignored.",
                                                                    registry);

   private final ICPControlGainsReadOnly feedbackGains;

   // Algorithm Inputs:
   private final FramePoint2D desiredICP = new FramePoint2D();
   private final FrameVector2D desiredICPVelocity = new FrameVector2D();
   private final FrameVector2D parallelDirection = new FrameVector2D();
   private final FrameVector2D perpDirection = new FrameVector2D();
   private final FrameVector2D perfectCMPOffset = new FrameVector2D();
   private final FramePoint2D currentICP = new FramePoint2D();
   private final FramePoint2D currentCoMPosition = new FramePoint2D();
   private final FrameVector2D currentCoMVelocity = new FrameVector2D();

   private final YoFramePoint2D perfectCoP = new YoFramePoint2D(yoNamePrefix + "PerfectCoP", worldFrame, registry);
   private final YoFramePoint2D perfectCMP = new YoFramePoint2D(yoNamePrefix + "PerfectCMP", worldFrame, registry);

   // Feedback control computations before projection (unconstrained)
   final YoFrameVector2D icpError = new YoFrameVector2D(yoNamePrefix + "ICPError", "", worldFrame, registry);
   private final YoDouble icpErrorMagnitude = new YoDouble(yoNamePrefix + "ICPErrorMagnitude", registry);
   private final YoDouble icpParallelError = new YoDouble(yoNamePrefix + "ICPParallelError", registry);
   private final YoDouble icpPerpError = new YoDouble(yoNamePrefix + "ICPPerpError", registry);

   private final YoFrameVector2D pureFeedforwardControl = new YoFrameVector2D(yoNamePrefix + "PureFeedforwardControl", "", worldFrame, registry);
   private final YoDouble pureFeedforwardMagnitude = new YoDouble(yoNamePrefix + "PureFeedforwardMagnitude", registry);

   private final YoFrameVector2D pureFeedbackControl = new YoFrameVector2D(yoNamePrefix + "PureFeedbackControl", "", worldFrame, registry);
   private final YoDouble pureFeedbackMagnitude = new YoDouble(yoNamePrefix + "PureFeedbackMagnitude", registry);

   private final YoDouble feedbackFeedforwardAlpha = new YoDouble(yoNamePrefix + "FeedbackFeedforwardAlpha", registry);

   private final YoDouble icpParallelFeedback = new YoDouble(yoNamePrefix + "ICPParallelFeedback", registry);
   private final YoDouble icpPerpFeedback = new YoDouble(yoNamePrefix + "ICPPerpFeedback", registry);

   private final YoFrameVector2D unconstrainedFeedback = new YoFrameVector2D(yoNamePrefix + "UnconstrainedFeedback", worldFrame, registry);
   private final YoFramePoint2D unconstrainedFeedbackCMP = new YoFramePoint2D(yoNamePrefix + "UnconstrainedFeedbackCMP", worldFrame, registry);
   private final YoFramePoint2D unconstrainedFeedbackCoP = new YoFramePoint2D(yoNamePrefix + "UnconstrainedFeedbackCoP", worldFrame, registry);

   // Outputs:
   private final YoFramePoint2D feedbackCoP = new YoFramePoint2D(yoNamePrefix + "FeedbackCoPSolution", worldFrame, registry);
   private final YoFramePoint2D feedbackCMP = new YoFramePoint2D(yoNamePrefix + "FeedbackCMPSolution", worldFrame, registry);
   private final YoFrameVector2D expectedControlICPVelocity = new YoFrameVector2D(yoNamePrefix + "ExpectedControlICPVelocity", worldFrame, registry);

   private final YoFrameVector2D residualError = new YoFrameVector2D(yoNamePrefix + "ResidualDynamicsError", worldFrame, registry);

   private final ExecutionTimer controllerTimer = new ExecutionTimer("icpControllerTimer", 0.5, registry);

   private final ICPControllerParameters.FeedbackProjectionOperator feedbackProjectionOperator;
   private final ICPControllerParameters.FeedForwardAlphaCalculator feedForwardAlphaCalculator;

   public HeuristicICPController(ICPControllerParameters icpControllerParameters,
                                 double controlDT,
                                 YoRegistry parentRegistry,
                                 YoGraphicsListRegistry yoGraphicsListRegistry)
   {

      pureFeedbackErrorThreshold.set(icpControllerParameters.getPureFeedbackErrorThreshold());

      pureFeedbackErrorThreshold.set(0.06);

      feedbackGains = new ParameterizedICPControlGains("", icpControllerParameters.getICPFeedbackGains(), registry);

      icpControllerParameters.createFeedbackProjectionOperator(registry, yoGraphicsListRegistry);
      icpControllerParameters.createFeedForwardAlphaCalculator(registry, yoGraphicsListRegistry);
      feedbackProjectionOperator = icpControllerParameters.getFeedbackProjectionOperator();
      feedForwardAlphaCalculator = icpControllerParameters.getFeedForwardAlphaCalculator();

      if (yoGraphicsListRegistry != null)
         setupVisualizers(yoGraphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   public ICPControlGainsReadOnly getFeedbackGains()
   {
      return feedbackGains;
   }

   @Override
   public void compute(FrameConvexPolygon2DReadOnly supportPolygonInWorld,
                       FramePoint2DReadOnly desiredICP,
                       FrameVector2DReadOnly desiredICPVelocity,
                       FramePoint2DReadOnly finalICP,
                       FramePoint2DReadOnly perfectCoP,
                       FrameVector2DReadOnly perfectCMPOffset,
                       FramePoint2DReadOnly currentICP,
                       FramePoint2DReadOnly currentCoMPosition,
                       double omega0)
   {
      //TODO: Try working in velocity and angle space instead of xy space.
      // Have a gain on the velocity based on whether leading or lagging. 
      // Then have a gain on the angle or something to push towards the ICP direction line.
      // This should reduce the dependence on the perfect CoP/CMP, which can throw things off
      // when there is a big error. And also should not cause so much outside to project fixes.
      // Especially if you limit the amount of velocity increase you can have.
      controllerTimer.startMeasurement();

      this.desiredICP.setMatchingFrame(desiredICP);
      this.desiredICPVelocity.setMatchingFrame(desiredICPVelocity);
      if (perfectCMPOffset == null)
         this.perfectCMPOffset.setToZero();
      else
         this.perfectCMPOffset.setMatchingFrame(perfectCMPOffset);
      this.currentICP.setMatchingFrame(currentICP);
      this.currentCoMPosition.setMatchingFrame(currentCoMPosition);

      CapturePointTools.computeCenterOfMassVelocity(currentCoMPosition, currentICP, omega0, currentCoMVelocity);

      this.perfectCoP.setMatchingFrame(perfectCoP);
      this.perfectCMP.add(this.perfectCoP, this.perfectCMPOffset);

      this.icpError.sub(currentICP, desiredICP);
      this.icpErrorMagnitude.set(icpError.length());

      this.parallelDirection.set(this.desiredICPVelocity);

      if (parallelDirection.lengthSquared() > 1e-7)
      {
         parallelDirection.normalize();
         perpDirection.set(-parallelDirection.getY(), parallelDirection.getX());

         icpParallelError.set(icpError.dot(parallelDirection));
         icpPerpError.set(icpError.dot(perpDirection));
      }
      else
      {
         parallelDirection.setToNaN();

         perpDirection.set(icpError);
         perpDirection.normalize();
         if (perpDirection.containsNaN())
         {
            perpDirection.set(1.0, 0.0);
         }

         icpPerpError.set(icpError.length());
         icpParallelError.set(0.0);
      }

      icpParallelFeedback.set(icpParallelError.getValue());
      icpParallelFeedback.mul(feedbackGains.getKpParallelToMotion());

      icpPerpFeedback.set(icpPerpError.getValue());
      icpPerpFeedback.mul(feedbackGains.getKpOrthogonalToMotion());

      pureFeedbackControl.set(icpError);
      pureFeedbackControl.scale(feedbackGains.getKpOrthogonalToMotion());
      pureFeedbackMagnitude.set(pureFeedbackControl.length());

      pureFeedforwardControl.sub(perfectCMP, desiredICP);
      pureFeedforwardMagnitude.set(pureFeedforwardControl.length());

      // Add the scaled amount of the feedforward to the feedback control based on the perpendicular error.
      unconstrainedFeedback.add(pureFeedforwardControl, pureFeedbackControl);
      unconstrainedFeedbackCMP.add(currentICP, unconstrainedFeedback);

      feedbackFeedforwardAlpha.set(0.0);
      if (feedForwardAlphaCalculator != null)
      {
         feedbackFeedforwardAlpha.set(feedForwardAlphaCalculator.computeAlpha(currentICP,
                                                                              desiredICP,
                                                                              finalICP,
                                                                              perfectCMP,
                                                                              unconstrainedFeedbackCMP,
                                                                              supportPolygonInWorld));
      }

      icpParallelFeedback.set(MathTools.clamp(icpParallelFeedback.getValue(), feedbackGains.getFeedbackPartMaxValueParallelToMotion()));
      icpPerpFeedback.set(MathTools.clamp(icpPerpFeedback.getValue(), feedbackGains.getFeedbackPartMaxValueOrthogonalToMotion()));

      // Add the scaled amount of the feedforward to the feedback control based on the perpendicular error.
      unconstrainedFeedback.scaleAdd(1.0 - feedbackFeedforwardAlpha.getValue(), pureFeedforwardControl, pureFeedbackControl);

      unconstrainedFeedbackCMP.add(currentICP, unconstrainedFeedback);
      unconstrainedFeedbackCoP.sub(unconstrainedFeedbackCMP, this.perfectCMPOffset);

      if (feedbackProjectionOperator != null)
      {
         feedbackProjectionOperator.projectFeedback(currentICP,
                                                    unconstrainedFeedbackCMP,
                                                    this.perfectCMPOffset,
                                                    supportPolygonInWorld,
                                                    feedbackCoP,
                                                    feedbackCMP);
      }

      expectedControlICPVelocity.sub(currentICP, feedbackCMP);
      expectedControlICPVelocity.scale(omega0);

      controllerTimer.stopMeasurement();
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoGraphicPosition feedbackCoPViz = new YoGraphicPosition(yoNamePrefix + "FeedbackCoP",
                                                               this.feedbackCoP,
                                                               0.005,
                                                               YoAppearance.Darkorange(),
                                                               YoGraphicPosition.GraphicType.BALL_WITH_CROSS);

      YoGraphicPosition unconstrainedFeedbackCMPViz = new YoGraphicPosition(yoNamePrefix + "UnconstrainedFeedbackCMP",
                                                                            this.unconstrainedFeedbackCMP,
                                                                            0.008,
                                                                            Purple(),
                                                                            GraphicType.BALL_WITH_CROSS);

      YoGraphicPosition unconstrainedFeedbackCoPViz = new YoGraphicPosition(yoNamePrefix + "UnconstrainedFeedbackCoP",
                                                                            this.unconstrainedFeedbackCoP,
                                                                            0.004,
                                                                            YoAppearance.Green(),
                                                                            GraphicType.BALL_WITH_ROTATED_CROSS);

      artifactList.add(feedbackCoPViz.createArtifact());
      artifactList.add(unconstrainedFeedbackCMPViz.createArtifact());
      artifactList.add(unconstrainedFeedbackCoPViz.createArtifact());
      //      artifactList.add(projectionLineViz);

      artifactList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void setKeepCoPInsideSupportPolygon(boolean keepCoPInsideSupportPolygon)
   {
   }

   @Override
   public FramePoint2DReadOnly getDesiredCMP()
   {
      return feedbackCMP;
   }

   @Override
   public FramePoint2DReadOnly getDesiredCoP()
   {
      return feedbackCoP;
   }

   @Override
   public FrameVector2DReadOnly getExpectedControlICPVelocity()
   {
      return expectedControlICPVelocity;
   }

   @Override
   public boolean useAngularMomentum()
   {
      return false;
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(newYoGraphicPoint2D("FeedbackCoP", feedbackCoP, 0.01, ColorDefinitions.DarkOrange(), DefaultPoint2DGraphic.CIRCLE_PLUS));
      group.addChild(newYoGraphicPoint2D("UnconstrainedFeedbackCMP",
                                         unconstrainedFeedbackCMP,
                                         0.016,
                                         ColorDefinitions.Purple(),
                                         DefaultPoint2DGraphic.CIRCLE_PLUS));
      group.addChild(newYoGraphicPoint2D("UnconstrainedFeedbackCoP",
                                         unconstrainedFeedbackCoP,
                                         0.008,
                                         ColorDefinitions.Green(),
                                         DefaultPoint2DGraphic.CIRCLE_CROSS));
      group.addChild(feedbackProjectionOperator.getSCS2YoGraphics());
      group.addChild(feedForwardAlphaCalculator.getSCS2YoGraphics());
      group.setVisible(VISUALIZE);
      return group;
   }
}
