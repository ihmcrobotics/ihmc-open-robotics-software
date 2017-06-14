package us.ihmc.commonWalkingControlModules.controlModules.leapOfFaith;

import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class PelvisLeapOfFaithModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFrameOrientation orientationOffset = new YoFrameOrientation("leapOfFaithPelvisOrientationOffset", worldFrame, registry);

   private final SideDependentList<? extends ReferenceFrame> soleZUpFrames;

   private boolean isInSwing = true;
   private RobotSide supportSide;
   private Footstep upcomingFootstep;

   private double swingDuration;

   private final BooleanYoVariable usePelvisRelaxation = new BooleanYoVariable("leapOfFaithUsePelvisRelaxation", registry);
   private final DoubleYoVariable yawGain = new DoubleYoVariable("leapOfFaithPelvisYawGain", registry);
   private final DoubleYoVariable rollGain = new DoubleYoVariable("leapOfFaithPelvisRollGain", registry);

   public PelvisLeapOfFaithModule(SideDependentList<? extends ReferenceFrame> soleZUpFrames, LeapOfFaithParameters parameters,
                                  YoVariableRegistry parentRegistry)
   {
      this.soleZUpFrames = soleZUpFrames;

      usePelvisRelaxation.set(parameters.usePelvisRelaxation());
      yawGain.set(parameters.getPelvisYawGain());
      rollGain.set(parameters.getPelvisRollGain());

      parentRegistry.addChild(registry);
   }

   public void setUpcomingFootstep(Footstep upcomingFootstep)
   {
      this.upcomingFootstep = upcomingFootstep;
      supportSide = upcomingFootstep.getRobotSide().getOppositeSide();
   }

   public void initializeStanding()
   {
      isInSwing = false;
   }

   public void initializeTransfer()
   {
      isInSwing = false;
   }

   public void initializeSwing(double swingDuration)
   {
      this.swingDuration = swingDuration;

      isInSwing = true;
   }

   private final FramePoint tempPoint = new FramePoint();
   public void update(double currentTimeInState)
   {
      orientationOffset.setToZero();

      if (isInSwing && usePelvisRelaxation.getBooleanValue())
      {
         double exceededTime = Math.max(currentTimeInState - swingDuration, 0.0);

         tempPoint.setToZero(upcomingFootstep.getSoleReferenceFrame());
         tempPoint.changeFrame(soleZUpFrames.get(supportSide));
         double stepLength = tempPoint.getX();

         double yawAngleOffset = yawGain.getDoubleValue() * exceededTime * stepLength;
         double rollAngleOffset = rollGain.getDoubleValue() * exceededTime;

         yawAngleOffset = supportSide.negateIfRightSide(yawAngleOffset);
         rollAngleOffset = supportSide.negateIfRightSide(rollAngleOffset);

         orientationOffset.setRoll(rollAngleOffset);
         orientationOffset.setYaw(yawAngleOffset);
      }
   }

   public void addAngularOffset(FrameOrientation orientationToPack)
   {
      orientationToPack.preMultiply(orientationOffset.getFrameOrientation());
   }
}
