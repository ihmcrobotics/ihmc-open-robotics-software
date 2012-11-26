package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredVelocityControlModule;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class ComponentBasedDesiredFootstepCalculator implements DesiredFootstepCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("ComponentBasedDesiredFootstepCalculator");

   private final DoubleYoVariable inPlaceWidth = new DoubleYoVariable("inPlaceWidth", registry);
   private final DoubleYoVariable walkingForwardWidth = new DoubleYoVariable("walkingForwardWidth", registry);
   private final DoubleYoVariable maxStepLength = new DoubleYoVariable("maxStepLength", registry);

   private final DoubleYoVariable minStepWidth = new DoubleYoVariable("minStepWidth", registry);
   private final DoubleYoVariable maxStepWidth = new DoubleYoVariable("maxStepWidth", registry);


   private final DoubleYoVariable sidestepMaxWidth = new DoubleYoVariable("sidestepMaxWidth", registry);
   private final DoubleYoVariable sidestepMinWidth = new DoubleYoVariable("sidestepMinWidth", registry);

   private final DoubleYoVariable velocityMagnitudeInHeading = new DoubleYoVariable("velocityMagnitudeInHeading", registry);
   private final DoubleYoVariable velocityMagnitudeToLeftOfHeading = new DoubleYoVariable("velocityMagnitudeToLeftOfHeading", registry);

   private final SideDependentList<? extends ReferenceFrame> ankleZUpFrames;

   private final DesiredHeadingControlModule desiredHeadingControlModule;
   private final DesiredVelocityControlModule desiredVelocityControlModule;

   private DesiredFootstepAdjustor desiredFootstepAdjustor;

   public ComponentBasedDesiredFootstepCalculator(SideDependentList<? extends ReferenceFrame> ankleZUpFrames,
           DesiredHeadingControlModule desiredHeadingControlModule, DesiredVelocityControlModule desiredVelocityControlModule,
           YoVariableRegistry parentRegistry)
   {
      this.ankleZUpFrames = ankleZUpFrames;

      this.desiredHeadingControlModule = desiredHeadingControlModule;
      this.desiredVelocityControlModule = desiredVelocityControlModule;

      parentRegistry.addChild(registry);
   }


   public Footstep updateAndGetDesiredFootstep(RobotSide supportLegSide)
   {
      RobotSide swingLegSide = supportLegSide.getOppositeSide();

      ReferenceFrame supportAnkleZUpFrame = ankleZUpFrames.get(supportLegSide);
      ReferenceFrame desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();

      FrameVector2d desiredHeading = desiredHeadingControlModule.getDesiredHeading();
      FrameVector2d desiredVelocity = desiredVelocityControlModule.getDesiredVelocity();
      FrameVector2d toLeftOfDesiredHeading = new FrameVector2d(desiredHeading.getReferenceFrame(), -desiredHeading.getY(), desiredHeading.getX());

      velocityMagnitudeInHeading.set(desiredVelocity.dot(desiredHeading));
      velocityMagnitudeToLeftOfHeading.set(desiredVelocity.dot(toLeftOfDesiredHeading));

      double stepForward = maxStepLength.getDoubleValue() * velocityMagnitudeInHeading.getDoubleValue();
      double stepSideways = swingLegSide.negateIfRightSide(inPlaceWidth.getDoubleValue());    // maxStepLength.getDoubleValue() * velocityMagnitudeToLeftOfHeading;

      FrameVector2d desiredVelocityInSupportAnkleZUpFrame = desiredVelocity.changeFrameCopy(supportAnkleZUpFrame);
      FrameVector2d desiredVelocityInHeadingFrame = desiredVelocity.changeFrameCopy(desiredHeadingFrame);

      FrameVector2d desiredPositionInHeading = new FrameVector2d(desiredHeadingFrame, 0.0, swingLegSide.negateIfRightSide(inPlaceWidth.getDoubleValue()));    // desiredVelocityInHeadingFrame);
      desiredPositionInHeading.add(desiredVelocityInHeadingFrame);

      if (desiredPositionInHeading.getX() > maxStepLength.getDoubleValue())
         desiredPositionInHeading.setX(maxStepLength.getDoubleValue());

      if (swingLegSide == RobotSide.LEFT)
      {
         if (desiredPositionInHeading.getY() > maxStepWidth.getDoubleValue())
            desiredPositionInHeading.setY(maxStepWidth.getDoubleValue());
         if (desiredPositionInHeading.getY() < minStepWidth.getDoubleValue())
            desiredPositionInHeading.setY(minStepWidth.getDoubleValue());
      }
      else
      {
         if (desiredPositionInHeading.getY() < -maxStepWidth.getDoubleValue())
            desiredPositionInHeading.setY(-maxStepWidth.getDoubleValue());
         if (desiredPositionInHeading.getY() > -minStepWidth.getDoubleValue())
            desiredPositionInHeading.setY(-minStepWidth.getDoubleValue());
      }

      FrameVector2d desiredPositionInSupport = desiredPositionInHeading.changeFrameCopy(supportAnkleZUpFrame);

      FramePoint footstepPosition = new FramePoint(supportAnkleZUpFrame, desiredPositionInSupport.getX(), desiredPositionInSupport.getY(), 0.0);


//    FramePoint footstepPosition = new FramePoint(supportAnkleZUpFrame, stepForward, stepSideways, 0.0);
//    Orientation footstepOrientation = new Orientation(supportAnkleZUpFrame);
      FrameOrientation footstepOrientation = new FrameOrientation(desiredHeadingFrame);
      footstepOrientation.changeFrame(supportAnkleZUpFrame);

      FramePose footstepPose = new FramePose(footstepPosition, footstepOrientation);
      Footstep footstep = new Footstep(swingLegSide, footstepPose);

      if (desiredFootstepAdjustor != null)
      {
         return desiredFootstepAdjustor.adjustDesiredFootstep(footstep);
      }

      return footstep;
   }

   public void initializeDesiredFootstep(RobotSide supportLegSide)
   {
   }

   public void setupParametersForM2V2()
   {
      setInPlaceWidth(0.25);
      setWalkingForwardWidth(0.17);

      setMaxStepLength(0.4);

      setSidestepMaxWidth(0.3);
      setSidestepMinWidth(0.15);

      setMinStepWidth(0.2);
      setMaxStepWidth(0.4);
   }

   public void setupParametersForR2()
   {
      setInPlaceWidth(0.4);
      setWalkingForwardWidth(0.25);

      setMaxStepLength(0.6);

      setSidestepMaxWidth(0.4);
      setSidestepMinWidth(0.15);

      setMinStepWidth(0.25);
      setMaxStepWidth(0.5);
   }

   public void setInPlaceWidth(double inPlaceWidth)
   {
      this.inPlaceWidth.set(inPlaceWidth);
   }

   public void setWalkingForwardWidth(double walkingForwardWidth)
   {
      this.walkingForwardWidth.set(walkingForwardWidth);
   }

   public void setMaxStepLength(double maxStepLength)
   {
      this.maxStepLength.set(maxStepLength);
   }

   public void setSidestepMaxWidth(double sidestepMaxWidth)
   {
      this.sidestepMaxWidth.set(sidestepMaxWidth);
   }

   public void setSidestepMinWidth(double sidestepMinWidth)
   {
      this.sidestepMinWidth.set(sidestepMinWidth);
   }

   public void setMinStepWidth(double minStepWidth)
   {
      this.minStepWidth.set(minStepWidth);
   }

   public void setMaxStepWidth(double maxStepWidth)
   {
      this.maxStepWidth.set(maxStepWidth);
   }

   public void addDesiredFootstepAdjustor(DesiredFootstepAdjustor desiredFootstepAdjustor)
   {
      this.desiredFootstepAdjustor = desiredFootstepAdjustor;

   }

}
