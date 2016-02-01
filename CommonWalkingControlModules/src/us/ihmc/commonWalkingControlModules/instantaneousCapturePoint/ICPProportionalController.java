package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.filters.AccelerationLimitedYoFrameVector2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.FilteredVelocityYoFrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector2d;


public class ICPProportionalController
{   
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FrameVector2d tempControl = new FrameVector2d(worldFrame);
   private final YoFrameVector2d icpError = new YoFrameVector2d("icpError", "", worldFrame, registry);
   private final YoFrameVector2d icpErrorIntegrated = new YoFrameVector2d("icpErrorIntegrated", "", worldFrame, registry);

   private final YoFrameVector2d feedbackPart = new YoFrameVector2d("feedbackPart", "", worldFrame, registry);
   
   private final DoubleYoVariable alphaICPVelocityFeedForward = new DoubleYoVariable("alphaICPVelocityFeedForward", registry);

   private final YoFrameVector2d desiredCMPToICP = new YoFrameVector2d("desiredCMPToICP", "", worldFrame, registry);

   private final YoFrameVector2d rawCMPOutput = new YoFrameVector2d("rawCMPOutput", "", worldFrame, registry);
   private final AlphaFilteredYoFrameVector2d filteredCMPOutput;
   private final AccelerationLimitedYoFrameVector2d rateLimitedCMPOutput;

   private final YoFramePoint icpPosition;
   private final DoubleYoVariable alphaICPVelocity;
   private final FilteredVelocityYoFrameVector icpVelocity;
   private final FrameVector2d icpDamping = new FrameVector2d(worldFrame);
   private final FrameVector2d icpIntegral = new FrameVector2d(worldFrame);
   
   private final double controlDT;
   private final DoubleYoVariable captureKpParallelToMotion = new DoubleYoVariable("captureKpParallel", registry);
   private final DoubleYoVariable captureKpOrthogonalToMotion = new DoubleYoVariable("captureKpOrthogonal", registry);

   private final DoubleYoVariable captureKd = new DoubleYoVariable("captureKd", registry);
   private final DoubleYoVariable captureKi = new DoubleYoVariable("captureKi", registry);
   private final DoubleYoVariable captureKiBleedoff = new DoubleYoVariable("captureKiBleedoff", registry);
   
   private final Vector2dZUpFrame icpVelocityDirectionFrame;
   
   private final FrameVector2d tempICPErrorIntegrated = new FrameVector2d(worldFrame);

   private final SmartCMPProjectorTwo smartCMPProjector;

   private final boolean useRawCMP;

   private final BooleanYoVariable useHackToReduceFeedForward = new BooleanYoVariable("icpControlUseHackToReduceFeedForward", registry);

   public ICPProportionalController(ICPControlGains gains, double controlDT, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      smartCMPProjector = new SmartCMPProjectorTwo(parentRegistry, yoGraphicsListRegistry);

      this.controlDT = controlDT;

      icpVelocityDirectionFrame = new Vector2dZUpFrame("icpVelocityDirectionFrame", worldFrame);
      
      icpPosition = new YoFramePoint("icpPosition", ReferenceFrame.getWorldFrame(), registry);
      alphaICPVelocity = new DoubleYoVariable("alphaICPVelocity", registry);
      icpVelocity = FilteredVelocityYoFrameVector.createFilteredVelocityYoFrameVector("icpVelocity", "", alphaICPVelocity, controlDT, parentRegistry, icpPosition);
      parentRegistry.addChild(registry);
      
      captureKpParallelToMotion.set(gains.getKpParallelToMotion());
      captureKpOrthogonalToMotion.set(gains.getKpOrthogonalToMotion());
      captureKi.set(gains.getKi());
      captureKiBleedoff.set(gains.getKiBleedOff());

      useRawCMP = gains.useRawCMP();
      useHackToReduceFeedForward.set(gains.useHackToReduceFeedForward());

      if (useRawCMP)
      {
         filteredCMPOutput = null;
         rateLimitedCMPOutput = null;
      }
      else
      {
         DoubleYoVariable alphaCMP = new DoubleYoVariable("alphaCMP", registry);
         DoubleYoVariable rateLimitCMP = new DoubleYoVariable("rateLimitCMP", registry);
         DoubleYoVariable accelerationLimitCMP = new DoubleYoVariable("accelerationLimitCMP", registry);

         filteredCMPOutput = AlphaFilteredYoFrameVector2d.createAlphaFilteredYoFrameVector2d("filteredCMPOutput", "", registry, alphaCMP, rawCMPOutput);
         rateLimitedCMPOutput = AccelerationLimitedYoFrameVector2d.createAccelerationLimitedYoFrameVector2d("rateLimitedCMPOutput", "", registry, rateLimitCMP, accelerationLimitCMP, controlDT, filteredCMPOutput);      

         double filterBreakFrequencyHertz = gains.getCMPFilterBreakFrequencyInHertz();
         alphaCMP.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filterBreakFrequencyHertz, controlDT));
         rateLimitCMP.set(gains.getCMPRateLimit());
         accelerationLimitCMP.set(gains.getCMPAccelerationLimit());

         rateLimitedCMPOutput.setGainsByPolePlacement(2.0 * Math.PI * filterBreakFrequencyHertz, 1.0);

         double frequencyRatios = filterBreakFrequencyHertz / (accelerationLimitCMP.getDoubleValue() / rateLimitCMP.getDoubleValue());

         if (frequencyRatios > 0.5)
         {
            System.err.println("Warning. Shouldn't have frequency ratios greater than 0.5 in ICPProportionalController!!");
            System.err.println("frequencyRatios = " + frequencyRatios);
         }

      }
   
   }

   public void reset()
   {
      if (!useRawCMP)
      {
         filteredCMPOutput.reset();
         rateLimitedCMPOutput.reset();
      }
      icpErrorIntegrated.set(0.0, 0.0);
   }

   private final FrameVector2d desICPToFinalDesICPVector = new FrameVector2d();
   private final FrameLineSegment2d desiredICPToFinalDesiredICPSegment = new FrameLineSegment2d();
   private final FramePoint2d icpProjected = new FramePoint2d();
   private final FramePoint2d desiredCMP = new FramePoint2d();
      
   public FramePoint2d doProportionalControl(FramePoint2d capturePoint, FramePoint2d desiredCapturePoint, FramePoint2d finalDesiredCapturePoint, FrameVector2d desiredCapturePointVelocity,
           double omega0, boolean projectIntoSupportPolygon, FrameConvexPolygon2d supportPolygon)
   {      
      capturePoint.changeFrame(worldFrame);
      desiredCapturePoint.changeFrame(worldFrame);
      finalDesiredCapturePoint.changeFrame(worldFrame);
      desiredCapturePointVelocity.changeFrame(worldFrame);
      
      desiredCMP.setIncludingFrame(capturePoint);

      icpPosition.set(capturePoint.getX(), capturePoint.getY(), 0.0);
      icpVelocity.update();
      alphaICPVelocityFeedForward.set(1.0);
      if (useHackToReduceFeedForward.getBooleanValue())
         correctICPFeedForward(capturePoint, desiredCapturePoint, finalDesiredCapturePoint, desiredCapturePointVelocity);
      // feed forward part
      tempControl.setIncludingFrame(desiredCapturePointVelocity);
      tempControl.scale(alphaICPVelocityFeedForward.getDoubleValue() / omega0);
      
      
      desiredCMP.sub(tempControl);

      // feedback part
      icpError.set(capturePoint);
      icpError.sub(desiredCapturePoint);
      
      icpError.getFrameTuple2d(tempControl);
      double epsilonZeroICPVelocity = 1e-5;
      if (desiredCapturePointVelocity.lengthSquared() > MathTools.square(epsilonZeroICPVelocity))
      {
         icpVelocityDirectionFrame.setXAxis(desiredCapturePointVelocity);
         tempControl.changeFrame(icpVelocityDirectionFrame);
         tempControl.setX(tempControl.getX() * captureKpParallelToMotion.getDoubleValue());
         tempControl.setY(tempControl.getY() * captureKpOrthogonalToMotion.getDoubleValue());
         tempControl.changeFrame(desiredCMP.getReferenceFrame());
      }
      else
      {
         tempControl.scale(captureKpOrthogonalToMotion.getDoubleValue());
      }

      icpDamping.set(icpVelocity.getX(), icpVelocity.getY());
      icpDamping.scale(captureKd.getDoubleValue());
      double length = icpDamping.length();
      double maxLength = 0.02;
      if (length > maxLength)
      {
         icpDamping.scale(maxLength/length);
      }
      
      tempControl.add(icpDamping);
      
      icpError.getFrameTuple2d(tempICPErrorIntegrated);
      tempICPErrorIntegrated.scale(controlDT);
      tempICPErrorIntegrated.scale(captureKi.getDoubleValue());
      
      icpErrorIntegrated.scale(captureKiBleedoff.getDoubleValue());
      icpErrorIntegrated.add(tempICPErrorIntegrated);

      length = icpErrorIntegrated.length();
      if (length > maxLength)
      {
         icpErrorIntegrated.scale(maxLength/length);
      }
      
      if (Math.abs(captureKi.getDoubleValue()) < 1e-10)
      {
         icpErrorIntegrated.set(0.0, 0.0);
      }
      
      icpErrorIntegrated.getFrameTuple2d(icpIntegral);
      tempControl.add(icpIntegral);
      
      feedbackPart.set(tempControl);
      desiredCMP.add(tempControl);

      desiredCMPToICP.sub(capturePoint, desiredCMP);

      if (projectIntoSupportPolygon)
      {
         smartCMPProjector.projectCMPIntoSupportPolygonIfOutside(capturePoint, supportPolygon, finalDesiredCapturePoint, desiredCMP);
         capturePoint.changeFrame(worldFrame);
         desiredCMP.changeFrame(worldFrame);
         
         if (desiredCMP.containsNaN())
         {
            //TODO: Track down why we get NaNs and fix them, rather than just setting CMP to ICP...
            desiredCMP.set(capturePoint);
            System.err.println("ICPProportionalController: desiredCMP contained NaN. Set it to capturePoint...");
//            throw new RuntimeException("desiredCMP.containsNaN()!");
         }
         if (smartCMPProjector.getWasCMPProjected())
         {
            icpErrorIntegrated.scale(0.9); //Bleed off quickly when projecting. 0.9 is a pretty arbitrary magic number.
         }
      }
      
      desiredCMP.changeFrame(rawCMPOutput.getReferenceFrame());
      rawCMPOutput.set(desiredCMP);

      if (!useRawCMP)
      {
         filteredCMPOutput.update();
         rateLimitedCMPOutput.update();
         rateLimitedCMPOutput.getFrameTuple2d(desiredCMP);
      }

      return desiredCMP;
   }

   private void correctICPFeedForward(FramePoint2d capturePoint, FramePoint2d desiredCapturePoint, FramePoint2d finalDesiredCapturePoint,
         FrameVector2d desiredCapturePointVelocity)
   {
      if (!finalDesiredCapturePoint.containsNaN())
      {
         desICPToFinalDesICPVector.setIncludingFrame(desiredCapturePoint);
         desICPToFinalDesICPVector.sub(finalDesiredCapturePoint);
         boolean isDesICPMovingTowardsFinalDesICP = desiredCapturePointVelocity.dot(desICPToFinalDesICPVector) > 0.0;
         if (!isDesICPMovingTowardsFinalDesICP && desiredCapturePointVelocity.lengthSquared() > 1e-3 && finalDesiredCapturePoint.distance(desiredCapturePoint) > 0.01)
         {
            desiredICPToFinalDesiredICPSegment.setAndChangeFrame(finalDesiredCapturePoint, desiredCapturePoint);
            icpProjected.setIncludingFrame(capturePoint);
            desiredICPToFinalDesiredICPSegment.orthogonalProjection(icpProjected);

            double percentageAlongLineSegment = MathTools.clipToMinMax(desiredICPToFinalDesiredICPSegment.percentageAlongLineSegment(icpProjected), 0.0, 1.0);
            alphaICPVelocityFeedForward.set(percentageAlongLineSegment);
         }
      }
   }
   
   private class Vector2dZUpFrame extends ReferenceFrame
   {
      private static final long serialVersionUID = -1810366869361449743L;
      private final FrameVector2d xAxis;
      private final Vector3d x = new Vector3d();
      private final Vector3d y = new Vector3d();
      private final Vector3d z = new Vector3d();
      private final Matrix3d rotation = new Matrix3d();

      public Vector2dZUpFrame(String string, ReferenceFrame parentFrame)
      {
         super(string, parentFrame);
         xAxis = new FrameVector2d(parentFrame);
      }

      public void setXAxis(FrameVector2d xAxis)
      {
         this.xAxis.setIncludingFrame(xAxis);
         this.xAxis.changeFrame(parentFrame);
         this.xAxis.normalize();
         update();
      }

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         x.set(xAxis.getX(), xAxis.getY(), 0.0);
         z.set(0.0, 0.0, 1.0);
         y.cross(z, x);

         rotation.setColumn(0, x);
         rotation.setColumn(1, y);
         rotation.setColumn(2, z);

         transformToParent.setRotationAndZeroTranslation(rotation);
      }
   }
}
