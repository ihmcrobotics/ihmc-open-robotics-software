package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Purple;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.ParameterizedICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationControllerHelper;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLine2d;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLine2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class HeuristicICPController implements ICPControllerInterface
{
   private static final boolean VISUALIZE = true;

   private final String yoNamePrefix = "controller";
   private final YoRegistry registry = new YoRegistry("ICPController");

   private final BooleanProvider useCMPFeedback;
   private final BooleanProvider useAngularMomentum;

   private final BooleanProvider scaleFeedbackWeightWithGain;

   private final DoubleProvider copFeedbackForwardWeight;
   private final DoubleProvider copFeedbackLateralWeight;
   //   private final DoubleProvider cmpFeedbackWeight;
   private final DMatrixRMaj scaledCoPFeedbackWeight = new DMatrixRMaj(2, 2);

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final DoubleProvider feedbackRateWeight;
   private final DoubleProvider copCMPFeedbackRateWeight;
   //   private final DoubleProvider dynamicsObjectiveWeight;

   private final ICPControlGainsReadOnly feedbackGains;
   private final DMatrixRMaj transformedGains = new DMatrixRMaj(2, 2);

   final YoFrameVector2D icpError = new YoFrameVector2D(yoNamePrefix + "ICPError", "", worldFrame, registry);
//   private final YoDouble icpParallelError = new YoDouble(yoNamePrefix + "ICPParallelError", registry);
//   private final YoDouble icpPerpError = new YoDouble(yoNamePrefix + "ICPPerpError", registry);

   private final FramePoint2D desiredICP = new FramePoint2D();
   private final FrameVector2D desiredICPVelocity = new FrameVector2D();
   private final FrameVector2D perfectCMPOffset = new FrameVector2D();
   private final FramePoint2D currentICP = new FramePoint2D();
   private final FramePoint2D currentCoMPosition = new FramePoint2D();
   private final FrameVector2D currentCoMVelocity = new FrameVector2D();

   private final YoFrameVector2D unconstrainedFeedback = new YoFrameVector2D(yoNamePrefix + "UnconstrainedFeedback", worldFrame, registry);
   private final YoFramePoint2D unconstrainedFeedbackCMP = new YoFramePoint2D(yoNamePrefix + "UnconstrainedFeedbackCMP", worldFrame, registry);
   private final YoFramePoint2D unconstrainedFeedbackCoP = new YoFramePoint2D(yoNamePrefix + "UnconstrainedFeedbackCoP", worldFrame, registry);

   private final YoFramePoint2D feedbackCoP = new YoFramePoint2D(yoNamePrefix + "FeedbackCoPSolution", worldFrame, registry);
   private final YoFramePoint2D coPProjection = new YoFramePoint2D(yoNamePrefix + "CoPProjection", worldFrame, registry);
   private final YoFramePoint2D feedbackCMP = new YoFramePoint2D(yoNamePrefix + "FeedbackCMPSolution", worldFrame, registry);

   final YoFramePoint2D perfectCoP = new YoFramePoint2D(yoNamePrefix + "PerfectCoP", worldFrame, registry);
   final YoFramePoint2D perfectCMP = new YoFramePoint2D(yoNamePrefix + "PerfectCMP", worldFrame, registry);

   private final YoFrameVector2D projectionVector = new YoFrameVector2D(yoNamePrefix + "ProjectionVector", worldFrame, registry);
   private final YoFrameLine2D projectionLine = new YoFrameLine2D(yoNamePrefix + "ProjectionLine", worldFrame, registry);

   private final YoFramePoint2D firstProjectionIntersection = new YoFramePoint2D(yoNamePrefix + "FirstIntersection", worldFrame, registry);
   private final YoFramePoint2D secondProjectionIntersection = new YoFramePoint2D(yoNamePrefix + "SecondIntersection", worldFrame, registry);
   private final YoFramePoint2D closestPointWithProjectionLine = new YoFramePoint2D(yoNamePrefix + "ClosestPointWithProjectionLine", worldFrame, registry);

   private final YoFrameVector2D residualError = new YoFrameVector2D(yoNamePrefix + "ResidualDynamicsError", worldFrame, registry);

   private final ExecutionTimer controllerTimer = new ExecutionTimer("icpControllerTimer", 0.5, registry);

   private final double controlDT;
   private final double controlDTSquare;

   private final ICPOptimizationControllerHelper helper = new ICPOptimizationControllerHelper();

   private final BipedSupportPolygons bipedSupportPolygons;

   private final FrameVector2D tempVector = new FrameVector2D();

   public HeuristicICPController(WalkingControllerParameters walkingControllerParameters,
                                 BipedSupportPolygons bipedSupportPolygons,
                                 ICPControlPolygons icpControlPolygons,
                                 SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                 double controlDT,
                                 YoRegistry parentRegistry,
                                 YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(walkingControllerParameters, walkingControllerParameters.getICPOptimizationParameters(), bipedSupportPolygons, icpControlPolygons, contactableFeet,
           controlDT, parentRegistry, yoGraphicsListRegistry);
   }

   public HeuristicICPController(WalkingControllerParameters walkingControllerParameters,
                                 ICPOptimizationParameters icpOptimizationParameters,
                                 BipedSupportPolygons bipedSupportPolygons,
                                 ICPControlPolygons icpControlPolygons,
                                 SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                 double controlDT,
                                 YoRegistry parentRegistry,
                                 YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.controlDT = controlDT;
      this.controlDTSquare = controlDT * controlDT;

      this.bipedSupportPolygons = bipedSupportPolygons;

      useCMPFeedback = new BooleanParameter(yoNamePrefix + "UseCMPFeedback", registry, icpOptimizationParameters.useCMPFeedback());
      useAngularMomentum = new BooleanParameter(yoNamePrefix + "UseAngularMomentum", registry, icpOptimizationParameters.useAngularMomentum());

      scaleFeedbackWeightWithGain = new BooleanParameter(yoNamePrefix + "ScaleFeedbackWeightWithGain",
                                                         registry,
                                                         icpOptimizationParameters.scaleFeedbackWeightWithGain());

      copFeedbackForwardWeight = new DoubleParameter(yoNamePrefix + "CoPFeedbackForwardWeight", registry, icpOptimizationParameters.getFeedbackForwardWeight());
      copFeedbackLateralWeight = new DoubleParameter(yoNamePrefix + "CoPFeedbackLateralWeight", registry, icpOptimizationParameters.getFeedbackLateralWeight());

      copCMPFeedbackRateWeight = new DoubleParameter(yoNamePrefix + "CoPCMPFeedbackRateWeight",
                                                     registry,
                                                     icpOptimizationParameters.getCoPCMPFeedbackRateWeight());
      feedbackRateWeight = new DoubleParameter(yoNamePrefix + "FeedbackRateWeight", registry, icpOptimizationParameters.getFeedbackRateWeight());

      feedbackGains = new ParameterizedICPControlGains("", icpOptimizationParameters.getICPFeedbackGains(), registry);

      if (yoGraphicsListRegistry != null)
         setupVisualizers(yoGraphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   public ICPControlGainsReadOnly getFeedbackGains()
   {
      return feedbackGains;
   }

   private final FrameVector2D desiredCMPOffsetToThrowAway = new FrameVector2D();

   @Override
   public void compute(FramePoint2DReadOnly desiredICP,
                       FrameVector2DReadOnly desiredICPVelocity,
                       FramePoint2DReadOnly perfectCoP,
                       FramePoint2DReadOnly currentICP,
                       FramePoint2DReadOnly currentCoMPosition,
                       double omega0)
   {
      desiredCMPOffsetToThrowAway.setToZero(worldFrame);
      compute(desiredICP, desiredICPVelocity, perfectCoP, desiredCMPOffsetToThrowAway, currentICP, currentCoMPosition, omega0);
   }

   @Override
   public void compute(FramePoint2DReadOnly desiredICP,
                       FrameVector2DReadOnly desiredICPVelocity,
                       FramePoint2DReadOnly perfectCoP,
                       FrameVector2DReadOnly perfectCMPOffset,
                       FramePoint2DReadOnly currentICP,
                       FramePoint2DReadOnly currentCoMPosition,
                       double omega0)
   {
      controllerTimer.startMeasurement();

      this.desiredICP.setMatchingFrame(desiredICP);
      this.desiredICPVelocity.setMatchingFrame(desiredICPVelocity);
      this.perfectCMPOffset.setMatchingFrame(perfectCMPOffset);
      this.currentICP.setMatchingFrame(currentICP);
      this.currentCoMPosition.setMatchingFrame(currentCoMPosition);

      CapturePointTools.computeCenterOfMassVelocity(currentCoMPosition, currentICP, omega0, currentCoMVelocity);

      this.perfectCoP.setMatchingFrame(perfectCoP);
      this.perfectCMP.add(this.perfectCoP, this.perfectCMPOffset);

      this.icpError.sub(currentICP, desiredICP);

      //      System.out.println("currentICP = " + currentICP);
      //      System.out.println("desiredICP = " + desiredICP);
      //      System.out.println("perfectCMP = " + perfectCMP);

      //      double kpOrthogonalToMotion = feedbackGains.getKpOrthogonalToMotion();
      //      System.out.println("kpOrthogonalToMotion = " + kpOrthogonalToMotion);

      //      System.out.println("icpError = " + icpError);
      feedbackCMP.set(icpError);

      helper.transformGainsFromDynamicsFrame(transformedGains,
                                             desiredICPVelocity,
                                             feedbackGains.getKpParallelToMotion(),
                                             feedbackGains.getKpOrthogonalToMotion());

      computeUnconstrainedFeedback();

      //      // There needs to be a +1.0 here to match the other Optimization based ICP Controller. Not sure why? 
      //      feedbackCMP.scale(1.0 + kpOrthogonalToMotion);
      //      feedbackCMP.add(perfectCMP);
      //      
      //      System.out.println("feedbackCMP = " + feedbackCMP);

      unconstrainedFeedbackCoP.set(unconstrainedFeedbackCMP);
      unconstrainedFeedbackCoP.sub(perfectCMPOffset);

      // Project the CoP onto the support polygon, using a projection vector.
      coPProjection.setToNaN();
      firstProjectionIntersection.setToNaN();
      secondProjectionIntersection.setToNaN();
      closestPointWithProjectionLine.setToNaN();
      projectionVector.setToNaN();
      projectionLine.setToNaN();

      // Determine the closest point, intersection point, etc.
      FrameConvexPolygon2DReadOnly supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();

      if (supportPolygonInWorld.isPointInside(unconstrainedFeedbackCoP))
      {
         feedbackCoP.set(unconstrainedFeedbackCoP);
      }

      else
      {
         //Compute the projection vector and projection line:
         projectionVector.set(currentICP);
         projectionVector.sub(unconstrainedFeedbackCMP);

         //TODO: What if projection vector length is small?
         projectionVector.normalize();

         projectionLine.set(unconstrainedFeedbackCoP, projectionVector);

         supportPolygonInWorld.intersectionWith(projectionLine, firstProjectionIntersection, secondProjectionIntersection);
         if (!firstProjectionIntersection.containsNaN())
         {
            if (secondProjectionIntersection.containsNaN())
            {
               feedbackCoP.set(firstProjectionIntersection);
            }
            else
            {
               double firstDistanceSquared = firstProjectionIntersection.distanceSquared(unconstrainedFeedbackCoP);
               double secondDistanceSquared = secondProjectionIntersection.distanceSquared(unconstrainedFeedbackCoP);

               if (firstDistanceSquared <= secondDistanceSquared)
               {
                  feedbackCoP.set(firstProjectionIntersection);
               }
               else
               {
                  feedbackCoP.set(secondProjectionIntersection);
               }
            }
         }
         else
         {
            coPProjection.set(unconstrainedFeedbackCoP);
            supportPolygonInWorld.orthogonalProjection(coPProjection);

            supportPolygonInWorld.getClosestPointWithRay(projectionLine, closestPointWithProjectionLine);

                        
            tempVector.set(currentICP);
            tempVector.sub(coPProjection);
            tempVector.normalize();
            double copProjectionDot = tempVector.dot(projectionVector);
            
            
            tempVector.set(currentICP);
            tempVector.sub(closestPointWithProjectionLine);
            tempVector.normalize();
            double closestPointWithProjectionLineDot = tempVector.dot(projectionVector);
            
            if (copProjectionDot >= closestPointWithProjectionLineDot)
               feedbackCoP.set(coPProjection);
            else
               feedbackCoP.set(closestPointWithProjectionLine);
            
//            double coPProjectionDistanceSquared = coPProjection.distanceSquared(unconstrainedFeedbackCoP);
//            double closestPointWithProjectionLineDistanceSquared = closestPointWithProjectionLine.distanceSquared(unconstrainedFeedbackCoP);

//            if (coPProjectionDistanceSquared <= closestPointWithProjectionLineDistanceSquared)
//               feedbackCoP.set(coPProjection);
//            else
//               feedbackCoP.set(closestPointWithProjectionLine);
         }
      }

      feedbackCMP.set(feedbackCoP);
      feedbackCMP.add(perfectCMPOffset);

      controllerTimer.stopMeasurement();
   }

   private void computeUnconstrainedFeedback()
   {
      unconstrainedFeedback.setX(transformedGains.get(0, 0) * icpError.getX() + transformedGains.get(0, 1) * icpError.getY());
      unconstrainedFeedback.setY(transformedGains.get(1, 0) * icpError.getX() + transformedGains.get(1, 1) * icpError.getY());
      unconstrainedFeedbackCMP.add(perfectCoP, perfectCMPOffset);
      unconstrainedFeedbackCMP.add(unconstrainedFeedback);
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoGraphicPosition feedbackCoPViz = new YoGraphicPosition(yoNamePrefix
            + "FeedbackCoP", this.feedbackCoP, 0.005, YoAppearance.Darkorange(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);

      YoGraphicPosition unconstrainedFeedbackCMPViz = new YoGraphicPosition(yoNamePrefix
            + "UnconstrainedFeedbackCMP", this.unconstrainedFeedbackCMP, 0.008, Purple(), GraphicType.BALL_WITH_CROSS);

      YoGraphicPosition unconstrainedFeedbackCoPViz = new YoGraphicPosition(yoNamePrefix
            + "UnconstrainedFeedbackCoP", this.unconstrainedFeedbackCoP, 0.004, YoAppearance.Green(), GraphicType.BALL_WITH_ROTATED_CROSS);

      YoArtifactLine2d projectionLineViz = new YoArtifactLine2d(yoNamePrefix + "ProjectionLine", this.projectionLine, YoAppearance.Aqua().getAwtColor());

      YoGraphicPosition firstIntersectionViz = new YoGraphicPosition(yoNamePrefix
            + "FirstIntersection", this.firstProjectionIntersection, 0.004, YoAppearance.Green(), GraphicType.SOLID_BALL);

      YoGraphicPosition secondIntersectionViz = new YoGraphicPosition(yoNamePrefix
            + "SecondIntersection", this.secondProjectionIntersection, 0.004, YoAppearance.Green(), GraphicType.SOLID_BALL);

      YoGraphicPosition closestPointWithProjectionLineViz = new YoGraphicPosition(yoNamePrefix
            + "ClosestToProjectionLine", this.closestPointWithProjectionLine, 0.003, YoAppearance.Green(), GraphicType.SOLID_BALL);

      YoGraphicPosition copProjectionViz = new YoGraphicPosition(yoNamePrefix
            + "CoPProjection", this.coPProjection, 0.002, YoAppearance.Green(), GraphicType.SOLID_BALL);

      artifactList.add(feedbackCoPViz.createArtifact());
      artifactList.add(unconstrainedFeedbackCMPViz.createArtifact());
      artifactList.add(unconstrainedFeedbackCoPViz.createArtifact());
      artifactList.add(projectionLineViz);

      artifactList.add(firstIntersectionViz.createArtifact());
      artifactList.add(secondIntersectionViz.createArtifact());
      artifactList.add(closestPointWithProjectionLineViz.createArtifact());
      artifactList.add(copProjectionViz.createArtifact());

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
   public void getDesiredCMP(FixedFramePoint2DBasics desiredCMPToPack)
   {
      desiredCMPToPack.set(feedbackCMP);
   }

   @Override
   public void getDesiredCoP(FixedFramePoint2DBasics desiredCoPToPack)
   {
      desiredCoPToPack.set(feedbackCoP);
   }

   @Override
   public boolean useAngularMomentum()
   {
      return false;
   }

   @Override
   public FrameVector2DReadOnly getResidualError()
   {
      return residualError;
   }

}
