package us.ihmc.avatar.obstacleCourseTests;

import java.util.EnumMap;
import java.util.List;
import java.util.Random;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;
import us.ihmc.robotics.robotController.ModularRobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.perturbance.GroundContactPointsSlipper;

public class SlipRandomOnNextStepPerturber extends ModularRobotController
{
   private enum SlipState
   {
      NO_CONTACT, CONTACT_WILL_SLIP, CONTACT_SLIP, CONTACT_DONE_SLIP, CONTACT
   }

   private final SideDependentList<GroundContactPointsSlipper> groundContactPointsSlippers;
   private final FloatingRootJointRobot robot;
   private final YoBoolean slipNextStep;
   private final YoDouble minSlipAfterTimeDelta, maxSlipAfterTimeDelta, nextSlipAfterTimeDelta;
   private final YoDouble minSlipPercentSlipPerTick, maxSlipPercentSlipPerTick, nextSlipPercentSlipPerTick;
   private final EnumMap<RobotSide, YoDouble> touchdownTimeForSlipMap = new EnumMap<RobotSide, YoDouble>(RobotSide.class);
   private final EnumMap<RobotSide, YoEnum<SlipState>> slipStateMap = new EnumMap<RobotSide, YoEnum<SlipState>>(RobotSide.class);
   private final EnumMap<RobotSide, List<GroundContactPoint>> groundContactPointsMap = new EnumMap<RobotSide, List<GroundContactPoint>>(RobotSide.class);

   private final YoFrameVector3D maxTranslationToSlipNextStep;
   private final YoFrameVector3D minTranslationToSlipNextStep;
   private final YoFrameVector3D nextTranslationToSlip;

   private final YoFrameYawPitchRoll maxRotationToSlipNextStep;
   private final YoFrameYawPitchRoll minRotationToSlipNextStep;
   private final YoFrameYawPitchRoll nextRotationToSlip;

   private double probabilitySlip = 0.0;
   private final Random random = new Random();

   public SlipRandomOnNextStepPerturber(HumanoidFloatingRootJointRobot robot, long randomSeed)
   {
      this(robot);
      this.random.setSeed(randomSeed);
   }

   public SlipRandomOnNextStepPerturber(HumanoidFloatingRootJointRobot robot)
   {
      super("SlipRandomOnNextStepPerturber");
      String name = "SlipRandom";

      this.robot = robot;

      groundContactPointsSlippers = new SideDependentList<GroundContactPointsSlipper>();

      for (RobotSide robotSide : RobotSide.values())
      {
         YoDouble touchdownTimeForSlip = new YoDouble(robotSide.getCamelCaseNameForStartOfExpression() + "TouchdownTimeForSlip"
               + robotSide.getCamelCaseNameForMiddleOfExpression(), registry);
         touchdownTimeForSlipMap.put(robotSide, touchdownTimeForSlip);

         YoEnum<SlipState> slipState = new YoEnum<SlipState>(name + "SlipState" + robotSide.getCamelCaseNameForMiddleOfExpression(), registry,
               SlipState.class);
         slipState.set(SlipState.NO_CONTACT);
         slipStateMap.put(robotSide, slipState);

         groundContactPointsMap.put(robotSide, robot.getFootGroundContactPoints(robotSide));
         
         GroundContactPointsSlipper groundContactPointsSlipper = new GroundContactPointsSlipper(robotSide.getLowerCaseName());
         groundContactPointsSlippers.put(robotSide, groundContactPointsSlipper);
         this.addRobotController(groundContactPointsSlipper);
      }

      this.minSlipAfterTimeDelta = new YoDouble(name + "MinSlipAfterTimeDelta", registry);
      this.maxSlipAfterTimeDelta = new YoDouble(name + "MaxSlipAfterTimeDelta", registry);
      this.nextSlipAfterTimeDelta = new YoDouble(name + "NextSlipAfterTimeDelta", registry);
      this.minSlipPercentSlipPerTick = new YoDouble(name + "MinSlipPercentSlipPerTick", registry);
      this.maxSlipPercentSlipPerTick = new YoDouble(name + "MaxSlipPercentSlipPerTick", registry);
      this.nextSlipPercentSlipPerTick = new YoDouble(name + "NextSlipPercentSlipPerTick", registry);

      this.slipNextStep = new YoBoolean(name + "SlipNextStep", registry);

      maxTranslationToSlipNextStep = new YoFrameVector3D(name + "MaxTranslationToSlipNextStep", ReferenceFrame.getWorldFrame(), registry);
      minTranslationToSlipNextStep = new YoFrameVector3D(name + "MinTranslationToSlipNextStep", ReferenceFrame.getWorldFrame(), registry);
      nextTranslationToSlip = new YoFrameVector3D(name + "NextTranslationToSlip", ReferenceFrame.getWorldFrame(), registry);

      maxRotationToSlipNextStep = new YoFrameYawPitchRoll(name + "MaxRotationToSlipNextStep", ReferenceFrame.getWorldFrame(), registry);
      minRotationToSlipNextStep = new YoFrameYawPitchRoll(name + "MinRotationToSlipNextStep", ReferenceFrame.getWorldFrame(), registry);
      nextRotationToSlip = new YoFrameYawPitchRoll(name + "NextRotationToSlip", ReferenceFrame.getWorldFrame(), registry);

      setTranslationRangeToSlipNextStep(new double[] { 0.0, 0.0, 0.0 }, new double[] { 0.05, 0.05, 0.0 });
      setRotationRangeToSlipNextStep(new double[] { 0.0, 0.0, 0.0 }, new double[] { 0.3, 0.15, 0.1 });
      setSlipAfterStepTimeDeltaRange(0.01, 0.10);
      setSlipPercentSlipPerTickRange(0.01, 0.05);
      setProbabilityOfSlip(1.0);
   }

   public void setSlipParameters(double[] slipXYZMin, double[] slipXYZMax, double[] slipYPRMin, double[] slipYPRMax, double minSlipAfterStepTimeDelta,
         double maxSlipAfterStepTimeDelta, double minSlipPercentSlipPerTick, double maxSlipPercentSlipPerTick, double slipProbability)
   {
      setTranslationRangeToSlipNextStep(slipXYZMin, slipXYZMax);
      setRotationRangeToSlipNextStep(slipYPRMin, slipYPRMax);

      setSlipAfterStepTimeDeltaRange(minSlipAfterStepTimeDelta, maxSlipAfterStepTimeDelta);
      setSlipPercentSlipPerTickRange(minSlipPercentSlipPerTick, maxSlipPercentSlipPerTick);
      setProbabilityOfSlip(slipProbability);
   }

   public void setSlipAfterStepTimeDeltaRange(double minSlipAfterStepTimeDelta, double maxSlipAfterStepTimeDelta)
   {
      this.minSlipAfterTimeDelta.set(minSlipAfterStepTimeDelta);
      this.maxSlipAfterTimeDelta.set(maxSlipAfterStepTimeDelta);
   }

   public void setSlipPercentSlipPerTickRange(double minSlipPercentSlipPerTick, double maxSlipPercentSlipPerTick)
   {
      this.minSlipPercentSlipPerTick.set(minSlipPercentSlipPerTick);
      this.maxSlipPercentSlipPerTick.set(maxSlipPercentSlipPerTick);
   }

   public void setTranslationRangeToSlipNextStep(double[] slipXYZMin, double[] slipXYZMax)
   {
      assertValidLimits(slipXYZMin, slipXYZMax);
      this.minTranslationToSlipNextStep.set(slipXYZMin[0], slipXYZMin[1], slipXYZMin[2]);
      this.maxTranslationToSlipNextStep.set(slipXYZMax[0], slipXYZMax[1], slipXYZMax[2]);
   }

   public void setRotationRangeToSlipNextStep(double[] slipYPRMin, double[] slipYPRMax)
   {
      assertValidLimits(slipYPRMin, slipYPRMax);
      this.minRotationToSlipNextStep.setYawPitchRoll(slipYPRMin[0], slipYPRMin[1], slipYPRMin[2]);
      this.maxRotationToSlipNextStep.setYawPitchRoll(slipYPRMax[0], slipYPRMax[1], slipYPRMax[2]);
   }

   public void assertValidLimits(double[] min, double[] max)
   {
      if (min.length != 3 || max.length != 3)
      {
         throw new RuntimeException("Slip min/max distance and rotation should contain 3 elements " + getClass().getName());
      }

      for (int i = 0; i < 3; i++)
      {
         if (min[i] < 0.0 || min[i] > max[i])
         {
            throw new RuntimeException("Slip min/max magnitude should be given in given in abs values. And min < max. In " + getClass().getName());
         }
      }
   }

   public void setProbabilityOfSlip(double slipProbability)
   {
      if ((slipProbability < 0.0) || (slipProbability > 1.0))
      {
         throw new RuntimeException("Probability prob = " + slipProbability + ", should be set between 0 and 100. In " + getClass().getName());
      }

      this.probabilitySlip = slipProbability;
   }

   @Override
   public void doControl()
   {
      super.doControl();

      for (RobotSide robotSide : RobotSide.values())
      {
         GroundContactPointsSlipper groundContactPointsSlipper = groundContactPointsSlippers.get(robotSide);

         switch (slipStateMap.get(robotSide).getEnumValue())
         {
         case NO_CONTACT:
         {
            if (footTouchedDown(robotSide))
            {
               if (doSlipThisStance())
               {
                  slipStateMap.get(robotSide).set(SlipState.CONTACT_WILL_SLIP);
                  touchdownTimeForSlipMap.get(robotSide).set(robot.getTime());
               }
               else
               // Wait till foot lift back up before allowing a slip.
               {
                  slipStateMap.get(robotSide).set(SlipState.CONTACT);
               }
            }

            break;
         }

         case CONTACT_WILL_SLIP:
         {
            if (robot.getTime() > (touchdownTimeForSlipMap.get(robotSide).getDoubleValue() + nextSlipAfterTimeDelta.getDoubleValue()))
            {
               if (slipStateMap.get(robotSide.getOppositeSide()).getEnumValue() == SlipState.CONTACT_SLIP)
               {
                  // Stop other foot from slipping, two slipping feet no implemented yet
                  groundContactPointsSlipper.setDoSlip(false);
                  slipStateMap.get(robotSide.getOppositeSide()).set(SlipState.CONTACT_DONE_SLIP);
               }

               slipStateMap.get(robotSide).set(SlipState.CONTACT_SLIP);
               startSlipping(robotSide);
            }

            break;
         }

         case CONTACT_SLIP:
         {
            if (groundContactPointsSlipper.isDoneSlipping())
            {
               slipStateMap.get(robotSide).set(SlipState.CONTACT_DONE_SLIP);
            }

            break;
         }

         case CONTACT_DONE_SLIP:
         case CONTACT:
         {
            if (footLiftedUp(robotSide))
            {
               slipStateMap.get(robotSide).set(SlipState.NO_CONTACT);
            }

            break;
         }

         }
      }
   }

   private void startSlipping(RobotSide robotSide)
   {
      GroundContactPointsSlipper groundContactPointsSlipper = groundContactPointsSlippers.get(robotSide);

      generateRandomSlipParamters();
      groundContactPointsSlipper.setGroundContactPoints(groundContactPointsMap.get(robotSide));
      groundContactPointsSlipper.setPercentToSlipPerTick(nextSlipPercentSlipPerTick.getDoubleValue());
      groundContactPointsSlipper.setDoSlip(true);
      groundContactPointsSlipper.setSlipTranslation(nextTranslationToSlip);
      groundContactPointsSlipper.setSlipRotationYawPitchRoll(nextRotationToSlip.getYawPitchRoll());

      //      System.out.println("Slip of " + robotSide.getLowerCaseName() + " foot with amount" + nextTranslationToSlip.getVector3dCopy().toString()
      //                         + " with rotation amount " + nextRotationToSlip.getFrameOrientationCopy().toStringAsYawPitchRoll()
      //                         + " with slippercentage per tick " + nextSlipPercentSlipPerTick.getDoubleValue() + ", " + nextSlipAfterTimeDelta.getDoubleValue()
      //                         + " [s] after touchdown.");
   }

   private void generateRandomSlipParamters()
   {
      double randomSlipTranslateX = pseudoRandomRealNumberWithinRange(minTranslationToSlipNextStep.getX(), maxTranslationToSlipNextStep.getX());
      double randomSlipTranslateY = pseudoRandomRealNumberWithinRange(minTranslationToSlipNextStep.getY(), maxTranslationToSlipNextStep.getY());
      double randomSlipTranslateZ = pseudoRandomRealNumberWithinRange(minTranslationToSlipNextStep.getZ(), maxTranslationToSlipNextStep.getZ());
      nextTranslationToSlip.set(randomSlipTranslateX, randomSlipTranslateY, randomSlipTranslateZ);

      nextRotationToSlip.setYawPitchRoll(pseudoRandomRealNumberWithinRange(minRotationToSlipNextStep.getYawPitchRoll(),
            maxRotationToSlipNextStep.getYawPitchRoll()));

      double randomSlipAfterTimeDelta = pseudoRandomPositiveNumberWithinRange(minSlipAfterTimeDelta.getDoubleValue(), maxSlipAfterTimeDelta.getDoubleValue());
      double randomPercentToSlipPerTick = pseudoRandomPositiveNumberWithinRange(minSlipPercentSlipPerTick.getDoubleValue(),
            maxSlipPercentSlipPerTick.getDoubleValue());

      nextSlipAfterTimeDelta.set(randomSlipAfterTimeDelta);
      nextSlipPercentSlipPerTick.set(randomPercentToSlipPerTick);

   }

   private double[] pseudoRandomRealNumberWithinRange(double[] min, double[] max)
   {
      double[] randoms = new double[3];
      for (int i = 0; i < randoms.length; i++)
      {
         randoms[i] = pseudoRandomRealNumberWithinRange(min[i], max[i]);
      }
      return randoms;
   }

   private double pseudoRandomRealNumberWithinRange(double minRange, double maxRange)
   {
      double realUnitPsuedoRandom = (random.nextDouble() * 2.0 - 1.0);
      double value = realUnitPsuedoRandom * (maxRange - minRange) + MathTools.sign(realUnitPsuedoRandom) * minRange;

      return value;
   }

   private double pseudoRandomPositiveNumberWithinRange(double minRange, double maxRange)
   {
      double posUnitPsuedoRandom = random.nextDouble();
      double value = posUnitPsuedoRandom * (maxRange - minRange) + minRange;

      return value;
   }

   private boolean doSlipThisStance()
   {
      slipNextStep.set(random.nextDouble() < probabilitySlip);
      return slipNextStep.getBooleanValue();
   }

   private boolean footTouchedDown(RobotSide robotSide)
   {
      for (GroundContactPoint groundContactPoint : groundContactPointsMap.get(robotSide))
      {
         if (groundContactPoint.isInContact())
            return true;
      }

      return false;
   }

   private boolean footLiftedUp(RobotSide robotSide)
   {
      for (GroundContactPoint groundContactPoint : groundContactPointsMap.get(robotSide))
      {
         if (groundContactPoint.isInContact())
            return false;
      }

      return true;
   }
}
