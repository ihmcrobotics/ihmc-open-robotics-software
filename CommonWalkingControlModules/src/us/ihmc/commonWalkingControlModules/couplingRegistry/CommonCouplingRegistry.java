package us.ihmc.commonWalkingControlModules.couplingRegistry;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;

public class CommonCouplingRegistry implements CouplingRegistry
{
   private RobotSide supportLeg;

   private final YoVariableRegistry registry = new YoVariableRegistry("CouplingRegistry");
   
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);
   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration", registry);
   private final DoubleYoVariable estimatedSwingTimeRemaining = new DoubleYoVariable("estimatedSwingTimeRemaining", registry);
   private final BooleanYoVariable forceHindOnToes = new BooleanYoVariable("forceHindOnToes", registry);
   private BooleanYoVariable isLunging = new BooleanYoVariable("isLunging", registry);

   //TODO: May need to YoVariablize the following to make things rewindable?
   private FrameConvexPolygon2d captureRegion;
   private FramePoint capturePoint = new FramePoint(ReferenceFrame.getWorldFrame());
   
   private YoFramePoint2d desiredCoP = new YoFramePoint2d("desiredCenterOfPressure", "", ReferenceFrame.getWorldFrame(), registry);

   private FrameVector2d desiredVelocity;

   private Footstep desiredStepLocation;

   private BipedSupportPolygons bipedSupportPolygons;

   private CommonWalkingReferenceFrames referenceFrames;

   private Wrench upperBodyWrench;
   



   public CommonCouplingRegistry(CommonWalkingReferenceFrames referenceFrames, BipedSupportPolygons bipedSupportPolygons, YoVariableRegistry parentRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.bipedSupportPolygons = bipedSupportPolygons;
      
      parentRegistry.addChild(registry);
   }

   public void setReferenceFrames(CommonWalkingReferenceFrames referenceFrames)
   {
      this.referenceFrames = referenceFrames;
   }

   public CommonWalkingReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }


   public void setSingleSupportDuration(double singleSupportDuration)
   {
      this.singleSupportDuration.set(singleSupportDuration);
   }

   public double getSingleSupportDuration()
   {
      return singleSupportDuration.getDoubleValue();
   }

   public void setDoubleSupportDuration(double doubleSupportDuration)
   {
      this.doubleSupportDuration.set(doubleSupportDuration);
   }

   public double getDoubleSupportDuration()
   {
      return doubleSupportDuration.getDoubleValue();
   }

   public void setDesiredVelocity(FrameVector2d desiredVelocity)
   {
      this.desiredVelocity = desiredVelocity;
   }

   public FrameVector2d getDesiredVelocity()
   {
      return this.desiredVelocity;
   }


   public void setCaptureRegion(FrameConvexPolygon2d captureRegion)
   {
      this.captureRegion = captureRegion;
   }

   public FrameConvexPolygon2d getCaptureRegion()
   {
      return captureRegion;
   }

   public void setCapturePoint(FramePoint capturePoint)
   {
      this.capturePoint.setAndChangeFrame(capturePoint);
   }

   public FramePoint getCapturePointInFrame(ReferenceFrame desiredFrame)
   {
      return capturePoint.changeFrameCopy(desiredFrame);
   }


   public void setDesiredFootstep(Footstep desiredStepLocation)
   {
      this.desiredStepLocation = desiredStepLocation;
   }

   public Footstep getDesiredFootstep()
   {
      return desiredStepLocation;
   }

   public void setSupportLeg(RobotSide supportLeg)
   {
      this.supportLeg = supportLeg;
   }

   public RobotSide getSupportLeg()
   {
      return supportLeg;
   }

   public void setEstimatedSwingTimeRemaining(double estimatedSwingTimeRemaining)
   {
      this.estimatedSwingTimeRemaining.set(estimatedSwingTimeRemaining);
   }

   public double getEstimatedSwingTimeRemaining()
   {
      return estimatedSwingTimeRemaining.getDoubleValue();
   }

   public void setBipedSupportPolygons(BipedSupportPolygons bipedSupportPolygons)
   {
      this.bipedSupportPolygons = bipedSupportPolygons;
   }

   public BipedSupportPolygons getBipedSupportPolygons()
   {
      return bipedSupportPolygons;
   }

   public void setForceHindOnToes(boolean forceHindOnToes)
   {
      this.forceHindOnToes.set(forceHindOnToes);
   }
   
   public boolean getForceHindOnToes()
   {
      return forceHindOnToes.getBooleanValue();
   }

   public void setUpperBodyWrench(Wrench upperBodyWrench)
   {
      this.upperBodyWrench = upperBodyWrench;
   }
   
   public Wrench getUpperBodyWrench()
   {
      return upperBodyWrench;
   }

   public void setDesiredCoP(FramePoint2d desiredCoP)
   {
      this.desiredCoP.set(desiredCoP);
   }

   public FramePoint2d getDesiredCoP()
   {
      return desiredCoP.getFramePoint2dCopy();
   }
   
   public void setIsLunging(boolean isLunging)
   {
      this.isLunging.set(isLunging);
   }
   
   public boolean getIsLunging()
   {
      return this.isLunging.getBooleanValue();
   }

}
