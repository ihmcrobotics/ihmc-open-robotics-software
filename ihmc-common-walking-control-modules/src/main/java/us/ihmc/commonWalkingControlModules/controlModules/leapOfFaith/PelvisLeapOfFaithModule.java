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
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
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

   private final BooleanProvider usePelvisRotation;
   private final BooleanProvider relaxPelvis;

   private final DoubleProvider reachingYawGain;
   private final DoubleProvider reachingRollGain;
   private final DoubleProvider reachingMaxYaw;
   private final DoubleProvider reachingMaxRoll;
   private final DoubleProvider reachingFractionOfSwing;

   private final YoDouble relaxationFraction = new YoDouble(yoNamePrefix + "PelvisRelaxationFraction", registry);
   private final DoubleProvider relaxationRate;
   private final DoubleProvider minimumWeight;

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

      usePelvisRotation = new BooleanParameter(yoNamePrefix + "UsePelvisRotation", registry, parameters.usePelvisRotation());
      relaxPelvis = new BooleanParameter(yoNamePrefix + "RelaxPelvis", registry, parameters.relaxPelvisControl());

      reachingYawGain = new DoubleParameter(yoNamePrefix + "PelvisReachingYawGain", registry, parameters.getPelvisReachingYawGain());
      reachingRollGain = new DoubleParameter(yoNamePrefix + "PelvisReachingRollGain", registry, parameters.getPelvisReachingRollGain());
      reachingMaxYaw = new DoubleParameter(yoNamePrefix + "PelvisReachingMaxYaw", registry, parameters.getPelvisReachingMaxYaw());
      reachingMaxRoll = new DoubleParameter(yoNamePrefix + "PelvisReachingMaxRoll", registry, parameters.getPelvisReachingMaxRoll());
      reachingFractionOfSwing = new DoubleParameter(yoNamePrefix + "PelvisReachingFractionOfSwing", registry, parameters.getPelvisReachingFractionOfSwing());

      relaxationRate = new DoubleParameter(yoNamePrefix + "PelvisRelaxationRate", registry, parameters.getRelaxationRate());
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
         timeInLeapOfFaith = Math.max(currentTimeInState - reachingFractionOfSwing.getValue() * stateDuration, 0.0);
      else
         phaseOutTime = Math.max(timeInLeapOfFaith - currentTimeInState, 0.0);
   }

   private final FramePoint3D tempPoint = new FramePoint3D();
   public void updateAngularOffsets()
   {
      orientationOffset.setToZero();

      if (isInSwing.getBooleanValue() && usePelvisRotation.getValue())
      {
         if (timeInLeapOfFaith == 0.0)
            return;

         tempPoint.setToZero(upcomingFootstep.getSoleReferenceFrame());
         tempPoint.changeFrame(soleZUpFrames.get(supportSide));
         double stepLength = tempPoint.getX();

         double yawAngleOffset = reachingYawGain.getValue() * timeInLeapOfFaith * stepLength;
         double rollAngleOffset = reachingRollGain.getValue() * timeInLeapOfFaith;

         yawAngleOffset = MathTools.clamp(yawAngleOffset, reachingMaxYaw.getValue());
         rollAngleOffset = MathTools.clamp(rollAngleOffset, reachingMaxRoll.getValue());

         yawAngleOffset = supportSide.negateIfRightSide(yawAngleOffset);
         rollAngleOffset = supportSide.negateIfRightSide(rollAngleOffset);

         orientationOffset.setRoll(rollAngleOffset);
         orientationOffset.setYaw(yawAngleOffset);
      }
      else if (usePelvisRotation.getValue())
      {
         if (phaseOutTime == 0.0)
            return;

         tempPoint.setToZero(soleZUpFrames.get(supportSide.getOppositeSide()));
         tempPoint.changeFrame(soleZUpFrames.get(supportSide));
         double stepLength = tempPoint.getX();

         double yawAngleOffset = reachingYawGain.getValue() * phaseOutTime * stepLength;
         double rollAngleOffset = reachingRollGain.getValue() * phaseOutTime;

         yawAngleOffset = MathTools.clamp(yawAngleOffset, reachingMaxYaw.getValue());
         rollAngleOffset = MathTools.clamp(rollAngleOffset, reachingMaxRoll.getValue());

         yawAngleOffset = supportSide.negateIfRightSide(yawAngleOffset);
         rollAngleOffset = supportSide.negateIfLeftSide(rollAngleOffset);

         orientationOffset.setRoll(rollAngleOffset);
         orientationOffset.setYaw(yawAngleOffset);
      }
   }

   public void relaxAngularWeight(Vector3D angularWeightToPack)
   {
      relaxationFraction.set(0.0);

      if (isInSwing.getBooleanValue() && relaxPelvis.getValue())
      {
         if (timeInLeapOfFaith == 0.0)
            return;

         double relaxationFraction = relaxationRate.getValue() * timeInLeapOfFaith;
         relaxationFraction = MathTools.clamp(relaxationFraction, 0.0, 1.0);

         this.relaxationFraction.set(relaxationFraction);

         angularWeightToPack.scale(1.0 - relaxationFraction);
         angularWeightToPack.setX(Math.max(minimumWeight.getValue(), angularWeightToPack.getX()));
         angularWeightToPack.setY(Math.max(minimumWeight.getValue(), angularWeightToPack.getY()));
         angularWeightToPack.setZ(Math.max(minimumWeight.getValue(), angularWeightToPack.getZ()));
      }
      else if (relaxPelvis.getValue())
      {
         if (phaseOutTime == 0.0)
            return;

         double relaxationFraction = relaxationRate.getValue() * phaseOutTime;
         relaxationFraction = MathTools.clamp(relaxationFraction, 0.0, 1.0);

         this.relaxationFraction.set(relaxationFraction);

         angularWeightToPack.scale(1.0 - relaxationFraction);
         angularWeightToPack.setX(Math.max(minimumWeight.getValue(), angularWeightToPack.getX()));
         angularWeightToPack.setY(Math.max(minimumWeight.getValue(), angularWeightToPack.getY()));
         angularWeightToPack.setZ(Math.max(minimumWeight.getValue(), angularWeightToPack.getZ()));
      }
   }

   public void addAngularOffset(FrameQuaternion orientationToPack)
   {
      orientationToPack.prepend(orientationOffset);
   }
}
