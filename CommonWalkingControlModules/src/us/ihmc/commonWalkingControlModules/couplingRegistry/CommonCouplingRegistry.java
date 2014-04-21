package us.ihmc.commonWalkingControlModules.couplingRegistry;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector2d;

public class CommonCouplingRegistry implements CouplingRegistry
{
   private RobotSide supportLeg;

   private final YoVariableRegistry registry = new YoVariableRegistry("CouplingRegistry");
   
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);
   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration", registry);
   private final DoubleYoVariable estimatedSwingTimeRemaining = new DoubleYoVariable("estimatedSwingTimeRemaining", registry);
   private final DoubleYoVariable estimatedDoubleSupportTimeRemaining = new DoubleYoVariable("estimatedDoubleSupportTimeRemaining", registry);

   private final BooleanYoVariable forceHindOnToes = new BooleanYoVariable("forceHindOnToes", registry);

   //TODO: May need to YoVariablize the following to make things rewindable?
   private FrameConvexPolygon2d captureRegion;
   private FramePoint capturePoint = new FramePoint(ReferenceFrame.getWorldFrame());
   
   private YoFramePoint2d desiredCoP = new YoFramePoint2d("desiredCenterOfPressure", "", ReferenceFrame.getWorldFrame(), registry);

   private FrameVector2d desiredVelocity;

   private Footstep desiredFootstep;

   private BipedSupportPolygons bipedSupportPolygons;

   private CommonWalkingReferenceFrames referenceFrames;

   private Wrench desiredUpperBodyWrench;
 
   private Wrench actualUpperBodyWrench;
   
   private YoFrameVector2d lungeAxis = new YoFrameVector2d("lungeAxis", "", ReferenceFrame.getWorldFrame(), registry);
   private YoFramePoint2d desiredCMP = new YoFramePoint2d("desiredCMP", "", ReferenceFrame.getWorldFrame(), registry);

   private FramePoint2d desiredICP = new FramePoint2d(ReferenceFrame.getWorldFrame());

   private final PoseReferenceFrame footstepFrame = new PoseReferenceFrame("footstepFrame", ReferenceFrame.getWorldFrame());
   private final DynamicGraphicReferenceFrame footstepFrameGraphic;
   
  
   private EnumYoVariable<RobotSide> upcomingSupportLeg = new EnumYoVariable<RobotSide>("upcomingSupportLeg", registry, RobotSide.class);

   public CommonCouplingRegistry(CommonWalkingReferenceFrames referenceFrames, BipedSupportPolygons bipedSupportPolygons, YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.bipedSupportPolygons = bipedSupportPolygons;
      
      parentRegistry.addChild(registry);
      if (dynamicGraphicObjectsListRegistry != null)
      {
         footstepFrameGraphic = new DynamicGraphicReferenceFrame(footstepFrame, registry, 0.1);
         DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList(getClass().getSimpleName());
         dynamicGraphicObjectsList.add(footstepFrameGraphic);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectsList);

      }
      else
      {
         footstepFrameGraphic = null;
      }
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
      this.capturePoint.setIncludingFrame(capturePoint);
   }

   public FramePoint getCapturePointInFrame(ReferenceFrame desiredFrame)
   {
      return capturePoint.changeFrameCopy(desiredFrame);
   }


   public void setDesiredFootstep(Footstep desiredFootstep)
   {
      this.desiredFootstep = desiredFootstep;
      if (desiredFootstep != null)
      {
         footstepFrame.updatePose(desiredFootstep.getPoseCopy().changeFrameCopy(footstepFrame.getParent()));
         footstepFrame.update();
         if(footstepFrameGraphic != null)
         {
            footstepFrameGraphic.showGraphicObject();
            footstepFrameGraphic.update();
         }
      }
      else
      {
         if(footstepFrameGraphic != null)
         {
            footstepFrameGraphic.hideGraphicObject();
         }
      }
   }

   public Footstep getDesiredFootstep()
   {
      return desiredFootstep;
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
   
   public void setEstimatedDoubleSupportTimeRemaining(double estimatedDoubleSupportTimeRemaining)
   {
      this.estimatedDoubleSupportTimeRemaining.set(estimatedDoubleSupportTimeRemaining);
   }
   
   public double getEstimatedDoubleSupportTimeRemaining()
   {
      return estimatedDoubleSupportTimeRemaining.getDoubleValue();
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

   public void setDesiredUpperBodyWrench(Wrench upperBodyWrench)
   {
      this.desiredUpperBodyWrench = upperBodyWrench;
   }
   
   public Wrench getDesiredUpperBodyWrench()
   {
      return desiredUpperBodyWrench;
   }
   
   public void setActualUpperBodyLungingWrench(Wrench wrenchOnPelvis)
   {
      this.actualUpperBodyWrench = wrenchOnPelvis;
   }
   
   public Wrench getActualUpperBodyLungingWrench()
   {
      return actualUpperBodyWrench;
   }

   public void setDesiredCoP(FramePoint2d desiredCoP)
   {
      desiredCoP.changeFrame(this.desiredCoP.getReferenceFrame());
      this.desiredCoP.set(desiredCoP);
   }

   public FramePoint2d getDesiredCoP()
   {
      return desiredCoP.getFramePoint2dCopy();
   }
   
   public void setLungeAxis(FrameVector2d lungeAxis)
   {
      this.lungeAxis.set(lungeAxis);
      if (lungeAxis.length() != 0.0)
      {
         this.lungeAxis.normalize();
      }
   }
   
   // returns null if not lunging
   public FrameVector2d getLungeAxisInFrame(ReferenceFrame expressedInFrame)
   {
      if (lungeAxis.length() == 0.0)
      {
         return null;
      }
      else
      {
         FrameVector2d ret = lungeAxis.getFrameVector2dCopy();
         ret.changeFrame(expressedInFrame);
         return ret;
      }
   }

   public void setDesiredCMP(FramePoint2d desiredCMP)
   {  
      this.desiredCMP.set(desiredCMP.changeFrameCopy(ReferenceFrame.getWorldFrame()));
   }

   public FramePoint2d getDesiredCMP()
   {
      return this.desiredCMP.getFramePoint2dCopy();
   }
   
   
   public void setDesiredCapturePoint(FramePoint2d desiredCapturePoint)
   {
      this.desiredICP.set(desiredCapturePoint.getReferenceFrame(), desiredCapturePoint.getX(), desiredCapturePoint.getY());
   }

   public FramePoint2d getDesiredCapturePointInFrame(ReferenceFrame desiredFrame)
   {
      return desiredICP.changeFrameCopy(desiredFrame);
   }


   public void setUpcomingSupportLeg(RobotSide upcomingSupportLeg)
   {
      this.upcomingSupportLeg.set(upcomingSupportLeg);
   }
   
   public RobotSide getUpcomingSupportLeg()
   {
      return upcomingSupportLeg.getEnumValue();
   }

}
