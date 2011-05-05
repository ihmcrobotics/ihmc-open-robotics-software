package us.ihmc.commonWalkingControlModules.couplingRegistry;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.SideDependentList;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class CommonCouplingRegistry implements CouplingRegistry
{
   private RobotSide supportLeg;

   private final YoVariableRegistry registry = new YoVariableRegistry("CouplingRegistry");
   
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);
   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration", registry);
   private final DoubleYoVariable estimatedSwingTimeRemaining = new DoubleYoVariable("estimatedSwingTimeRemaining", registry);

   //TODO: May need to YoVariablize the following to make things rewindable?
   private FrameConvexPolygon2d captureRegion;
   private FramePoint capturePoint = new FramePoint(ReferenceFrame.getWorldFrame());

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


   public void setSingleSupportDuration(DoubleYoVariable singleSupportDuration)
   {
      this.singleSupportDuration.set(singleSupportDuration.getDoubleValue());
   }

   public double getSingleSupportDuration()
   {
      return singleSupportDuration.getDoubleValue();
   }

   public void setDoubleSupportDuration(DoubleYoVariable doubleSupportDuration)
   {
      this.doubleSupportDuration.set(doubleSupportDuration.getDoubleValue());
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
      this.capturePoint.set(capturePoint);
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

 
   public boolean getForceHindOnToes()
   {
      // TODO Auto-generated method stub
      return false;
   }

 
   public Wrench getUpperBodyWrench()
   {
      return upperBodyWrench;
   }



 
   public void setForceHindOnToes(BooleanYoVariable forceHindOnToes)
   {
      // TODO Auto-generated method stub

   }

 
   public void setUpperBodyWrench(Wrench upperBodyWrench)
   {
      this.upperBodyWrench = upperBodyWrench;
   }

}
