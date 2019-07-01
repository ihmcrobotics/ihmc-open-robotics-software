package us.ihmc.commonWalkingControlModules.controlModules.leapOfFaith;

import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;

public class PelvisLeapOfFaithModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final String yoNamePrefix = "leapOfFaith";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFrameYawPitchRoll orientationOffset = new YoFrameYawPitchRoll(yoNamePrefix + "PelvisOrientationOffset", worldFrame, registry);

   private final YoBoolean isInSwing = new YoBoolean(yoNamePrefix + "IsInSwing", registry);
   private final YoBoolean usePelvisRotation = new YoBoolean(yoNamePrefix + "UsePelvisRotation", registry);
   private final YoBoolean relaxPelvis = new YoBoolean(yoNamePrefix + "RelaxPelvis", registry);

   private final YoDouble reachingYawGain = new YoDouble(yoNamePrefix + "PelvisReachingYawGain", registry);
   private final YoDouble reachingRollGain = new YoDouble(yoNamePrefix + "PelvisReachingRollGain", registry);
   private final YoDouble reachingMaxYaw = new YoDouble(yoNamePrefix + "PelvisReachingMaxYaw", registry);
   private final YoDouble reachingMaxRoll = new YoDouble(yoNamePrefix + "PelvisReachingMaxRoll", registry);
   private final YoDouble reachingFractionOfSwing = new YoDouble(yoNamePrefix + "PelvisReachingFractionOfSwing", registry);

   private final YoDouble relaxationRate = new YoDouble(yoNamePrefix + "PelvisRelaxationRate", registry);
   private final YoDouble relaxationFraction = new YoDouble(yoNamePrefix + "PelvisRelaxationFraction", registry);
   private final YoDouble minimumWeight = new YoDouble(yoNamePrefix + "PelvisMinimumWeight", registry);

   private final SideDependentList<? extends ReferenceFrame> soleZUpFrames;

   private RobotSide supportSide;
   private Footstep upcomingFootstep;

   private double stateDuration;

   private double timeInLeapOfFaith;
   private double phaseOutTime;

   public PelvisLeapOfFaithModule(SideDependentList<? extends ReferenceFrame> soleZUpFrames, LeapOfFaithParameters parameters,
                                  YoVariableRegistry parentRegistry)
   {
      this.soleZUpFrames = soleZUpFrames;

      usePelvisRotation.set(parameters.usePelvisRotation());
      relaxPelvis.set(parameters.relaxPelvisControl());

      reachingYawGain.set(parameters.getPelvisReachingYawGain());
      reachingRollGain.set(parameters.getPelvisReachingRollGain());
      reachingMaxYaw.set(parameters.getPelvisReachingMaxYaw());
      reachingMaxRoll.set(parameters.getPelvisReachingMaxRoll());
      reachingFractionOfSwing.set(parameters.getPelvisReachingFractionOfSwing());

      relaxationRate.set(parameters.getRelaxationRate());
      minimumWeight.set(parameters.getMinimumPelvisWeight());

      parentRegistry.addChild(registry);
   }

   public void setUpcomingFootstep(Footstep upcomingFootstep)
   {
      this.upcomingFootstep = upcomingFootstep;
      supportSide = upcomingFootstep.getRobotSide().getOppositeSide();
   }

   public void initializeStanding()
   {
      isInSwing.set(false);
   }

   public void initializeTransfer(double transferDuration)
   {
      stateDuration = transferDuration;
      isInSwing.set(false);
   }

   public void initializeSwing(double swingDuration)
   {
      stateDuration = swingDuration;

      isInSwing.set(true);
   }

   public void update(double currentTimeInState)
   {
      if (isInSwing.getBooleanValue())
         timeInLeapOfFaith = Math.max(currentTimeInState - reachingFractionOfSwing.getDoubleValue() * stateDuration, 0.0);
      else
         phaseOutTime = Math.max(timeInLeapOfFaith - currentTimeInState, 0.0);
   }

   private final FramePoint3D tempPoint = new FramePoint3D();
   public void updateAngularOffsets()
   {
      orientationOffset.setToZero();

      if (isInSwing.getBooleanValue() && usePelvisRotation.getBooleanValue())
      {
         if (timeInLeapOfFaith == 0.0)
            return;

         tempPoint.setToZero(upcomingFootstep.getSoleReferenceFrame());
         tempPoint.changeFrame(soleZUpFrames.get(supportSide));
         double stepLength = tempPoint.getX();

         double yawAngleOffset = reachingYawGain.getDoubleValue() * timeInLeapOfFaith * stepLength;
         double rollAngleOffset = reachingRollGain.getDoubleValue() * timeInLeapOfFaith;

         yawAngleOffset = MathTools.clamp(yawAngleOffset, reachingMaxYaw.getDoubleValue());
         rollAngleOffset = MathTools.clamp(rollAngleOffset, reachingMaxRoll.getDoubleValue());

         yawAngleOffset = supportSide.negateIfRightSide(yawAngleOffset);
         rollAngleOffset = supportSide.negateIfRightSide(rollAngleOffset);

         orientationOffset.setRoll(rollAngleOffset);
         orientationOffset.setYaw(yawAngleOffset);
      }
      else if (usePelvisRotation.getBooleanValue())
      {
         if (phaseOutTime == 0.0)
            return;

         tempPoint.setToZero(soleZUpFrames.get(supportSide.getOppositeSide()));
         tempPoint.changeFrame(soleZUpFrames.get(supportSide));
         double stepLength = tempPoint.getX();

         double yawAngleOffset = reachingYawGain.getDoubleValue() * phaseOutTime * stepLength;
         double rollAngleOffset = reachingRollGain.getDoubleValue() * phaseOutTime;

         yawAngleOffset = MathTools.clamp(yawAngleOffset, reachingMaxYaw.getDoubleValue());
         rollAngleOffset = MathTools.clamp(rollAngleOffset, reachingMaxRoll.getDoubleValue());

         yawAngleOffset = supportSide.negateIfRightSide(yawAngleOffset);
         rollAngleOffset = supportSide.negateIfLeftSide(rollAngleOffset);

         orientationOffset.setRoll(rollAngleOffset);
         orientationOffset.setYaw(yawAngleOffset);
      }
   }

   public void relaxAngularWeight(Vector3D angularWeightToPack)
   {
      relaxationFraction.set(0.0);

      if (isInSwing.getBooleanValue() && relaxPelvis.getBooleanValue())
      {
         if (timeInLeapOfFaith == 0.0)
            return;

         double relaxationFraction = relaxationRate.getDoubleValue() * timeInLeapOfFaith;
         relaxationFraction = MathTools.clamp(relaxationFraction, 0.0, 1.0);

         this.relaxationFraction.set(relaxationFraction);

         angularWeightToPack.scale(1.0 - relaxationFraction);
         angularWeightToPack.setX(Math.max(minimumWeight.getDoubleValue(), angularWeightToPack.getX()));
         angularWeightToPack.setY(Math.max(minimumWeight.getDoubleValue(), angularWeightToPack.getY()));
         angularWeightToPack.setZ(Math.max(minimumWeight.getDoubleValue(), angularWeightToPack.getZ()));
      }
      else if (relaxPelvis.getBooleanValue())
      {
         if (phaseOutTime == 0.0)
            return;

         double relaxationFraction = relaxationRate.getDoubleValue() * phaseOutTime;
         relaxationFraction = MathTools.clamp(relaxationFraction, 0.0, 1.0);

         this.relaxationFraction.set(relaxationFraction);

         angularWeightToPack.scale(1.0 - relaxationFraction);
         angularWeightToPack.setX(Math.max(minimumWeight.getDoubleValue(), angularWeightToPack.getX()));
         angularWeightToPack.setY(Math.max(minimumWeight.getDoubleValue(), angularWeightToPack.getY()));
         angularWeightToPack.setZ(Math.max(minimumWeight.getDoubleValue(), angularWeightToPack.getZ()));
      }
   }

   public void addAngularOffset(FrameQuaternion orientationToPack)
   {
      orientationToPack.prepend(orientationOffset);
   }
}
