package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.ParameterizedICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class HeuristicICPController implements ICPControllerInterface
{
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

   final YoFrameVector2D icpError = new YoFrameVector2D(yoNamePrefix + "ICPError", "", worldFrame, registry);

   
   private final FramePoint2D desiredICP = new FramePoint2D();
   private final FrameVector2D desiredICPVelocity = new FrameVector2D();
   private final FrameVector2D perfectCMPOffset = new FrameVector2D();
   private final FramePoint2D currentICP = new FramePoint2D();
   private final FramePoint2D currentCoMPosition = new FramePoint2D();
   private final FrameVector2D currentCoMVelocity = new FrameVector2D();
   
   
   private final YoFramePoint2D feedbackCoP = new YoFramePoint2D(yoNamePrefix + "FeedbackCoPSolution", worldFrame, registry);
   private final YoFramePoint2D feedbackCMP = new YoFramePoint2D(yoNamePrefix + "FeedbackCMPSolution", worldFrame, registry);

   final YoFramePoint2D perfectCoP = new YoFramePoint2D(yoNamePrefix + "PerfectCoP", worldFrame, registry);
   final YoFramePoint2D perfectCMP = new YoFramePoint2D(yoNamePrefix + "PerfectCMP", worldFrame, registry);

   private final YoFrameVector2D residualError = new YoFrameVector2D(yoNamePrefix + "ResidualDynamicsError", worldFrame, registry);

   private final ExecutionTimer controllerTimer = new ExecutionTimer("icpControllerTimer", 0.5, registry);

   private final double controlDT;
   private final double controlDTSquare;
   
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

      
      double kpOrthogonalToMotion = feedbackGains.getKpOrthogonalToMotion();
      
      feedbackCMP.set(icpError);
      feedbackCMP.scale(kpOrthogonalToMotion);
      feedbackCMP.add(perfectCMP);
      
      feedbackCoP.set(feedbackCMP);
      
      controllerTimer.stopMeasurement();
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
