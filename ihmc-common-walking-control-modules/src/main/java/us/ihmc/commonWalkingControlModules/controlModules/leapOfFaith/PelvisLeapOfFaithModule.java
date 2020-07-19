package us.ihmc.commonWalkingControlModules.controlModules.leapOfFaith;

import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class PelvisLeapOfFaithModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final String yoNamePrefix = "leapOfFaith";

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoFrameYawPitchRoll orientationOffset = new YoFrameYawPitchRoll(yoNamePrefix + "PelvisOrientationOffset", worldFrame, registry);

   private final YoBoolean isInSwing = new YoBoolean(yoNamePrefix + "IsInSwing", registry);

   private final BooleanProvider usePelvisRotation;
   private final BooleanProvider relaxPelvisWeight;

   private final DoubleProvider reachingYawGain;
   private final DoubleProvider reachingRollGain;
   private final DoubleProvider reachingMaxYaw;
   private final DoubleProvider reachingMaxRoll;
   private final DoubleProvider reachingFractionOfSwing;

   private final YoDouble relaxationFraction = new YoDouble(yoNamePrefix + "PelvisRelaxationFraction", registry);
   private final DoubleProvider weightRelaxationRate;
   private final DoubleProvider minimumWeight;

   private final SideDependentList<? extends ReferenceFrame> soleZUpFrames;

   private RobotSide supportSide;
   private Footstep upcomingFootstep;

   private double stateDuration;

   private final YoDouble timeInLeapOfFaith = new YoDouble("timeInPelvisLeapOfFaith", registry);
   private double phaseOutTime;

   private final FramePoint3D tempPoint = new FramePoint3D();

   public PelvisLeapOfFaithModule(SideDependentList<? extends ReferenceFrame> soleZUpFrames, LeapOfFaithParameters parameters,
                                  YoRegistry parentRegistry)
   {
      this.soleZUpFrames = soleZUpFrames;

      usePelvisRotation = new BooleanParameter(yoNamePrefix + "UsePelvisRotationForReaching", registry, parameters.usePelvisRotation());
      relaxPelvisWeight = new BooleanParameter(yoNamePrefix + "RelaxPelvisWeight", registry, parameters.relaxPelvisControl());

      reachingYawGain = new DoubleParameter(yoNamePrefix + "PelvisReachingYawGain", registry, parameters.getPelvisReachingYawGain());
      reachingRollGain = new DoubleParameter(yoNamePrefix + "PelvisReachingRollGain", registry, parameters.getPelvisReachingRollGain());
      reachingMaxYaw = new DoubleParameter(yoNamePrefix + "PelvisReachingMaxYaw", registry, parameters.getPelvisReachingMaxYaw());
      reachingMaxRoll = new DoubleParameter(yoNamePrefix + "PelvisReachingMaxRoll", registry, parameters.getPelvisReachingMaxRoll());
      reachingFractionOfSwing = new DoubleParameter(yoNamePrefix + "PelvisReachingFractionOfSwing", registry, parameters.getPelvisReachingFractionOfSwing());

      weightRelaxationRate = new DoubleParameter(yoNamePrefix + "PelvisWeightRelaxationRate", registry, parameters.getRelaxationRate());
      minimumWeight = new DoubleParameter(yoNamePrefix + "PelvisMinimumWeight", registry, parameters.getMinimumPelvisWeight());

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
         timeInLeapOfFaith.set(Math.max(currentTimeInState - reachingFractionOfSwing.getValue() * stateDuration, 0.0));
      else
         phaseOutTime = Math.max(timeInLeapOfFaith.getDoubleValue() - currentTimeInState, 0.0);
   }

   public void updateAngularOffsets()
   {
      orientationOffset.setToZero();

      if (!usePelvisRotation.getValue())
         return;

      double timeForReaching;
      if (isInSwing.getBooleanValue())
      {
         if (timeInLeapOfFaith.getDoubleValue() <= 0.0)
            return;

         // doing the leap of faith, so modify the pelvis setpoints to have it reach
         timeForReaching = timeInLeapOfFaith.getDoubleValue();
      }
      else
      {
         if (phaseOutTime <= 0.0)
            return;

         // need to return the offset back to zero, so use the monotonically decreasing phase out time
         timeForReaching = phaseOutTime;
      }

      tempPoint.setToZero(upcomingFootstep.getSoleReferenceFrame());
      tempPoint.changeFrame(soleZUpFrames.get(supportSide));
      double stepLength = tempPoint.getX();

      double yawAngleOffset = reachingYawGain.getValue() * timeForReaching * stepLength;
      double rollAngleOffset = reachingRollGain.getValue() * timeForReaching;

      yawAngleOffset = MathTools.clamp(yawAngleOffset, reachingMaxYaw.getValue());
      rollAngleOffset = MathTools.clamp(rollAngleOffset, reachingMaxRoll.getValue());

      yawAngleOffset = supportSide.negateIfRightSide(yawAngleOffset);
      rollAngleOffset = supportSide.negateIfRightSide(rollAngleOffset);

      orientationOffset.setRoll(rollAngleOffset);
      orientationOffset.setYaw(yawAngleOffset);
   }

   public void relaxAngularWeight(Vector3DBasics angularWeightToPack)
   {
      relaxationFraction.set(0.0);

      if (!relaxPelvisWeight.getValue())
         return;

      double relaxationTime;
      if (isInSwing.getBooleanValue())
      {
         if (timeInLeapOfFaith.getDoubleValue() < 0.0)
            return;

         // doing the leap of faith, so start relaxing the weight
         relaxationTime = timeInLeapOfFaith.getDoubleValue();
      }
      else
      {
         if (phaseOutTime < 0.0)
            return;

         // need to return the weight back to nominal
         relaxationTime = phaseOutTime;
      }

      double relaxationFraction = weightRelaxationRate.getValue() * relaxationTime;
      relaxationFraction = MathTools.clamp(relaxationFraction, 0.0, 1.0);

      this.relaxationFraction.set(relaxationFraction);

      angularWeightToPack.scale(1.0 - relaxationFraction);
      angularWeightToPack.setX(Math.max(minimumWeight.getValue(), angularWeightToPack.getX()));
      angularWeightToPack.setY(Math.max(minimumWeight.getValue(), angularWeightToPack.getY()));
      angularWeightToPack.setZ(Math.max(minimumWeight.getValue(), angularWeightToPack.getZ()));
   }

   public void addAngularOffset(FixedFrameQuaternionBasics orientationToPack)
   {
      orientationToPack.prepend(orientationOffset);
   }
}
