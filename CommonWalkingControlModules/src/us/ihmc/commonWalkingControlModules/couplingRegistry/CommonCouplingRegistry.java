package us.ihmc.commonWalkingControlModules.couplingRegistry;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.OldBipedSupportPolygons;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;


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

   private OldBipedSupportPolygons bipedSupportPolygons;

   private CommonHumanoidReferenceFrames referenceFrames;

   private Wrench desiredUpperBodyWrench;
 
   private Wrench actualUpperBodyWrench;
   
   private YoFrameVector2d lungeAxis = new YoFrameVector2d("lungeAxis", "", ReferenceFrame.getWorldFrame(), registry);
   private YoFramePoint2d desiredCMP = new YoFramePoint2d("desiredCMP", "", ReferenceFrame.getWorldFrame(), registry);

   private FramePoint2d desiredICP = new FramePoint2d(ReferenceFrame.getWorldFrame());

   private final PoseReferenceFrame footstepFrame = new PoseReferenceFrame("footstepFrame", ReferenceFrame.getWorldFrame());
   private final YoGraphicReferenceFrame footstepFrameGraphic;
   
  
   private EnumYoVariable<RobotSide> upcomingSupportLeg = new EnumYoVariable<RobotSide>("upcomingSupportLeg", registry, RobotSide.class);

   public CommonCouplingRegistry(CommonHumanoidReferenceFrames referenceFrames, OldBipedSupportPolygons bipedSupportPolygons, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.bipedSupportPolygons = bipedSupportPolygons;
      
      parentRegistry.addChild(registry);
      if (yoGraphicsListRegistry != null)
      {
         footstepFrameGraphic = new YoGraphicReferenceFrame(footstepFrame, registry, 0.1);
         YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
         yoGraphicsList.add(footstepFrameGraphic);
         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);

      }
      else
      {
         footstepFrameGraphic = null;
      }
   }

   public void setReferenceFrames(CommonHumanoidReferenceFrames referenceFrames)
   {
      this.referenceFrames = referenceFrames;
   }

   public CommonHumanoidReferenceFrames getReferenceFrames()
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
      FramePoint ret = new FramePoint(capturePoint);
      ret.changeFrame(desiredFrame);
      return ret;
   }


   public void setDesiredFootstep(Footstep desiredFootstep)
   {
      this.desiredFootstep = desiredFootstep;
      if (desiredFootstep != null)
      {
         FramePose footstepPose = new FramePose();
         desiredFootstep.getPose(footstepPose);
         footstepPose.changeFrame(footstepFrame.getParent());
         footstepFrame.setPoseAndUpdate(footstepPose);
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

   public OldBipedSupportPolygons getOldBipedSupportPolygons()
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
      FramePoint2d temp = new FramePoint2d(desiredCMP);
      temp.changeFrame(ReferenceFrame.getWorldFrame());
      this.desiredCMP.set(temp);
   }

   public FramePoint2d getDesiredCMP()
   {
      return this.desiredCMP.getFramePoint2dCopy();
   }
   
   
   public void setDesiredCapturePoint(FramePoint2d desiredCapturePoint)
   {
      this.desiredICP.setIncludingFrame(desiredCapturePoint.getReferenceFrame(), desiredCapturePoint.getX(), desiredCapturePoint.getY());
   }

   public FramePoint2d getDesiredCapturePointInFrame(ReferenceFrame desiredFrame)
   {
      FramePoint2d ret = new FramePoint2d(desiredICP);
      ret.changeFrame(desiredFrame);
      return ret;
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
