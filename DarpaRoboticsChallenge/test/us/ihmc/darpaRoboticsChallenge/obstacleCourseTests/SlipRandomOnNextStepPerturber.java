package us.ihmc.darpaRoboticsChallenge.obstacleCourseTests;

import java.util.EnumMap;
import java.util.List;
import java.util.Random;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.GroundContactPoint;
import com.yobotics.simulationconstructionset.robotController.ModularRobotController;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.perturbance.GroundContactPointsSlipper;

public class SlipRandomOnNextStepPerturber extends ModularRobotController
{
   private enum SlipState {NOT_SLIPPING, TOUCHED_DOWN, SLIPPING, DONE_SLIPPING}

   private final GroundContactPointsSlipper groundContactPointsSlipper;
   private final SDFRobot robot;
   private final BooleanYoVariable slipNextStep;
   private final DoubleYoVariable minSlipAfterTimeDelta, maxSlipAfterTimeDelta, nextSlipAfterTimeDelta;
   private final DoubleYoVariable minSlipPercentSlipPerTick, maxSlipPercentSlipPerTick, nextSlipPercentSlipPerTick;
   private final EnumMap<RobotSide, DoubleYoVariable> touchdownTimeForSlipMap = new EnumMap<RobotSide, DoubleYoVariable>(RobotSide.class);
   private final EnumMap<RobotSide, EnumYoVariable<SlipState>> slipStateMap = new EnumMap<RobotSide, EnumYoVariable<SlipState>>(RobotSide.class);
   private final EnumMap<RobotSide, List<GroundContactPoint>> groundContactPointsMap = new EnumMap<RobotSide, List<GroundContactPoint>>(RobotSide.class);

   private final YoFrameVector maxMagnitudeToSlipNextStep;
   private final YoFrameVector minMagnitudeToSlipNextStep;
   private final YoFrameVector nextAmountToSlip;

// private final YoFrameVector amountToSlipNextStep;
   private int probabilitySlip = 0;
   private final Random random = new Random();

   public SlipRandomOnNextStepPerturber(SDFRobot robot, long randomSeed)
   {
      this(robot);
      this.random.setSeed(randomSeed);
   }

   public SlipRandomOnNextStepPerturber(SDFRobot robot)
   {
      super("SlipRandomOnNextStepPerturber");
      String name = "SlipRandom";

      this.robot = robot;

      for (RobotSide robotSide : RobotSide.values())
      {
         DoubleYoVariable touchdownTimeForSlip = new DoubleYoVariable(robotSide.toString() + "TouchdownTimeForSlip" + robotSide.toString(), registry);
         touchdownTimeForSlipMap.put(robotSide, touchdownTimeForSlip);

         EnumYoVariable<SlipState> slipState = new EnumYoVariable<SlipState>(name + "SlipState" + robotSide.toString(), registry, SlipState.class);
         slipState.set(SlipState.NOT_SLIPPING);
         slipStateMap.put(robotSide, slipState);

         groundContactPointsMap.put(robotSide, robot.getFootGroundContactPoints(robotSide));
      }

      this.minSlipAfterTimeDelta = new DoubleYoVariable(name + "minSlipAfterTimeDelta", registry);
      this.maxSlipAfterTimeDelta = new DoubleYoVariable(name + "maxSlipAfterTimeDelta", registry);
      this.nextSlipAfterTimeDelta = new DoubleYoVariable(name + "nextSlipAfterTimeDelta", registry);
      this.minSlipPercentSlipPerTick = new DoubleYoVariable(name + "minSlipPercentSlipPerTick", registry);
      this.maxSlipPercentSlipPerTick = new DoubleYoVariable(name + "maxSlipPercentSlipPerTick", registry);
      this.nextSlipPercentSlipPerTick = new DoubleYoVariable(name + "nextSlipPercentSlipPerTick", registry);

      this.slipNextStep = new BooleanYoVariable(name + "SlipNextStep", registry);

      maxMagnitudeToSlipNextStep = new YoFrameVector(name + "maxMagnitudeToSlipNextStep", ReferenceFrame.getWorldFrame(), registry);
      minMagnitudeToSlipNextStep = new YoFrameVector(name + "minMagnitudeToSlipNextStep", ReferenceFrame.getWorldFrame(), registry);
      nextAmountToSlip = new YoFrameVector(name + "nextAmountToSlip", ReferenceFrame.getWorldFrame(), registry);

//    amountToSlipNextStep = new YoFrameVector(name + "AmountToSlipNextStep", ReferenceFrame.getWorldFrame(), registry);
      groundContactPointsSlipper = new GroundContactPointsSlipper();
      this.addRobotController(groundContactPointsSlipper);
   }


   public void initialize(double minSlipMagnitudeInX, double minSlipMagnitudeInY, double minSlipMagnitudeInZ, double maxSlipMagnitudeInX,
                          double maxSlipMagnitudeInY, double maxSlipMagnitudeInZ, double minSlipAfterStepTimeDelta, double maxSlipAfterStepTimeDelta,
                          double minSlipPercentSlipPerTick, double maxSlipPercentSlipPerTick, int slipProbability)
   {
      setSlipAfterStepTimeDeltaRange(minSlipAfterStepTimeDelta, maxSlipAfterStepTimeDelta);
      setSlipPercentSlipPerTickRange(minSlipPercentSlipPerTick, maxSlipPercentSlipPerTick);
      setMagnitudeRangeToSlipNextStep(minSlipMagnitudeInX, minSlipMagnitudeInY, minSlipMagnitudeInZ, maxSlipMagnitudeInX, maxSlipMagnitudeInY,
                                      maxSlipMagnitudeInZ);
      setProbabilityOfSlipInPercentage(slipProbability);
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

   public void setMagnitudeRangeToSlipNextStep(double minSlipMagnitudeInX, double minSlipMagnitudeInY, double minSlipMagnitudeInZ, double maxSlipMagnitudeInX,
           double maxSlipMagnitudeInY, double maxSlipMagnitudeInZ)
   {
      boolean positive = ((minSlipMagnitudeInX <= 0) || (minSlipMagnitudeInY <= 0) || (minSlipMagnitudeInZ <= 0) || (maxSlipMagnitudeInX <= 0)
                          || (maxSlipMagnitudeInY <= 0) || (maxSlipMagnitudeInZ <= 0));
      boolean maxmin = ((minSlipMagnitudeInX < maxSlipMagnitudeInX) || (minSlipMagnitudeInY < maxSlipMagnitudeInY)
                        || (minSlipMagnitudeInZ < maxSlipMagnitudeInZ));

      if (!positive ||!maxmin)
      {
         throw new RuntimeException("Slip min/max magnitude should be given in given in abs values. And min < max. In " + getClass().getName());
      }


      this.minMagnitudeToSlipNextStep.set(minSlipMagnitudeInX, minSlipMagnitudeInY, minSlipMagnitudeInZ);
      this.maxMagnitudeToSlipNextStep.set(maxSlipMagnitudeInX, maxSlipMagnitudeInY, maxSlipMagnitudeInZ);
   }

   public void setProbabilityOfSlipInPercentage(int slipProbability)
   {
      if ((slipProbability < 0) || (slipProbability > 100))
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
         switch (slipStateMap.get(robotSide).getEnumValue())
         {
            case NOT_SLIPPING :
            {
               if (footTouchedDown(robotSide))
               {
                  if (slipOnNextStep())
                  {
                     slipStateMap.get(robotSide).set(SlipState.TOUCHED_DOWN);
                     touchdownTimeForSlipMap.get(robotSide).set(robot.getTime());
                  }
                  else    // Wait till foot lift back up before allowing a slip.
                  {
                     slipStateMap.get(robotSide).set(SlipState.DONE_SLIPPING);
                  }
               }

               break;
            }

            case TOUCHED_DOWN :
            {
               if (robot.getTime() > (touchdownTimeForSlipMap.get(robotSide).getDoubleValue() + nextSlipAfterTimeDelta.getDoubleValue()))
               {
                  if (slipStateMap.get(robotSide.getOppositeSide()).getEnumValue() == SlipState.SLIPPING)
                  {
                     // Stop other foot from slipping, two slipping feet no implemented yet
                     slipStateMap.get(robotSide.getOppositeSide()).set(SlipState.DONE_SLIPPING);
                  }

                  slipStateMap.get(robotSide).set(SlipState.SLIPPING);
                  startSlipping(robotSide);
               }

               break;
            }

            case SLIPPING :
            {
               if (groundContactPointsSlipper.isDoneSlipping())
               {
                  slipStateMap.get(robotSide).set(SlipState.DONE_SLIPPING);
               }

               break;
            }

            case DONE_SLIPPING :
            {
               if (footLiftedUp(robotSide))
               {
                  slipStateMap.get(robotSide).set(SlipState.NOT_SLIPPING);
               }

               break;
            }
         }
      }
   }

   private void startSlipping(RobotSide robotSide)
   {
      generateRandomSlipParamters();
      groundContactPointsSlipper.setGroundContactPoints(groundContactPointsMap.get(robotSide));
      groundContactPointsSlipper.setPercentToSlipPerTick(nextSlipPercentSlipPerTick.getDoubleValue());
      groundContactPointsSlipper.setDoSlip(true);
      groundContactPointsSlipper.setSlipAmount(nextAmountToSlip.getVector3dCopy());

      System.out.println("Slip of " + robotSide.getLowerCaseName() + " foot with amount" + nextAmountToSlip.getVector3dCopy().toString()
                         + " with slippercentage per tick " + nextSlipPercentSlipPerTick.getDoubleValue() + ", " + nextSlipAfterTimeDelta.getDoubleValue()
                         + " [s] after touchdown.");
   }

   private void generateRandomSlipParamters()
   {
      double randomSlipDeltaX = pseudoRandomRealNumberWithinRange(minMagnitudeToSlipNextStep.getX(), maxMagnitudeToSlipNextStep.getX());
      double randomSlipDeltaY = pseudoRandomRealNumberWithinRange(minMagnitudeToSlipNextStep.getY(), maxMagnitudeToSlipNextStep.getY());
      double randomSlipDeltaZ = pseudoRandomRealNumberWithinRange(minMagnitudeToSlipNextStep.getZ(), maxMagnitudeToSlipNextStep.getZ());
      double randomSlipAfterTimeDelta = pseudoRandomRealNumberWithinRange(minSlipAfterTimeDelta.getDoubleValue(), maxSlipAfterTimeDelta.getDoubleValue());
      double randomPercentToSlipPerTick = pseudoRandomRealNumberWithinRange(minSlipPercentSlipPerTick.getDoubleValue(),
                                             maxSlipPercentSlipPerTick.getDoubleValue());

      nextSlipAfterTimeDelta.set(randomSlipAfterTimeDelta);
      nextSlipPercentSlipPerTick.set(randomPercentToSlipPerTick);
      nextAmountToSlip.set(randomSlipDeltaX, randomSlipDeltaY, randomSlipDeltaZ);
   }

   private double pseudoRandomRealNumberWithinRange(double minRange, double maxRange)
   {
      double realUnitPsuedoRandom = (random.nextDouble() * 2.0 - 1.0);
      System.out.println("realUnitPsuedoRandom " + realUnitPsuedoRandom);
      double value = realUnitPsuedoRandom * (maxRange - minRange) + MathTools.sign(realUnitPsuedoRandom) * minRange;

      return value;
   }

   private boolean slipOnNextStep()
   {
      if (probabilitySlip == 0)
      {
         slipNextStep.set(false);
      }
      else
      {
         int value = (int) Math.round((100.0 / probabilitySlip));
         boolean bool = (random.nextInt(value) == 0);
         slipNextStep.set(bool);
      }

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
