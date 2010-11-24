package us.ihmc.commonWalkingControlModules.couplingRegistry;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.SideDependentList;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.desiredStepLocation.Footstep;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;

public class CommonCouplingRegistry implements CouplingRegistry
{
   private RobotSide supportLeg;

   private DoubleYoVariable singleSupportDuration;
   private DoubleYoVariable doubleSupportDuration;
   private double estimatedSwingTimeRemaining;

   private FrameConvexPolygon2d captureRegion;
   private FramePoint capturePoint;

   private FrameVector2d desiredVelocity;

   private Footstep desiredStepLocation;

   private BipedSupportPolygons bipedSupportPolygons;

   private CommonWalkingReferenceFrames referenceFrames;


   public CommonCouplingRegistry(CommonWalkingReferenceFrames referenceFrames)
   {
      SideDependentList<ReferenceFrame> ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
      ReferenceFrame midFeetZUp = referenceFrames.getMidFeetZUpFrame();

//    ReferenceFrame chestZUp = referenceFrames.getChestZUpFrame();
      bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUp);
      this.referenceFrames = referenceFrames;
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
      this.singleSupportDuration = singleSupportDuration;
   }

   public double getSingleSupportDuration()
   {
      return singleSupportDuration.getDoubleValue();
   }

   public void setDoubleSupportDuration(DoubleYoVariable doubleSupportDuration)
   {
      this.doubleSupportDuration = doubleSupportDuration;
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
      this.capturePoint = capturePoint;
   }

   public FramePoint getCapturePoint()
   {
      return capturePoint;
   }


   public void setDesiredStepLocation(Footstep desiredStepLocation)
   {
      this.desiredStepLocation = desiredStepLocation;
   }

   public Footstep getDesiredStepLocation()
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
      this.estimatedSwingTimeRemaining = estimatedSwingTimeRemaining;
   }

   public double getEstimatedSwingTimeRemaining()
   {
      return estimatedSwingTimeRemaining;    // .getDoubleValue();
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
      // TODO Auto-generated method stub
      return null;
   }



 
   public void setForceHindOnToes(BooleanYoVariable forceHindOnToes)
   {
      // TODO Auto-generated method stub

   }

 
   public void setUpperBodyWrench(Wrench upperBodyWrench)
   {
      // TODO Auto-generated method stub

   }

}
