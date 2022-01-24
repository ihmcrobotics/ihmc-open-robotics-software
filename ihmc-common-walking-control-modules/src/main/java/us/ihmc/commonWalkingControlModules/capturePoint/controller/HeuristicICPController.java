package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class HeuristicICPController implements ICPControllerInterface
{
   private final String yoNamePrefix = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(yoNamePrefix);
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private final ICPControlGainsReadOnly feedbackGains;

   private final YoFramePoint2D feedbackCoP = new YoFramePoint2D(yoNamePrefix + "FeedbackCoPSolution", worldFrame, registry);
   private final YoFramePoint2D feedbackCMP = new YoFramePoint2D(yoNamePrefix + "FeedbackCMPSolution", worldFrame, registry);
   
   private final YoFrameVector2D residualError = new YoFrameVector2D(yoNamePrefix + "ResidualDynamicsError", worldFrame, registry);

   public HeuristicICPController(ICPControlGainsReadOnly feedbackGains)
   {
      this.feedbackGains = feedbackGains;
   }
   
   
   @Override
   public void setKeepCoPInsideSupportPolygon(boolean keepCoPInsideSupportPolygon)
   {      
   }

   @Override
   public void compute(FramePoint2DReadOnly desiredICP,
                       FrameVector2DReadOnly desiredICPVelocity,
                       FramePoint2DReadOnly perfectCoP,
                       FramePoint2DReadOnly currentICP,
                       FramePoint2DReadOnly currentCoMPosition,
                       double omega0)
   {      
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
   }

   @Override
   public void initialize()
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
