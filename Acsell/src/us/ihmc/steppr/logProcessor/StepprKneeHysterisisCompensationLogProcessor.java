package us.ihmc.steppr.logProcessor;

import us.ihmc.avatar.logProcessor.LogDataProcessorFunction;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.HysteresisFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class StepprKneeHysterisisCompensationLogProcessor implements LogDataProcessorFunction
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SideDependentList<DoubleYoVariable> ankleOutsideMotorPositions = new SideDependentList<>();
   private final SideDependentList<DoubleYoVariable> ankleOutsideMotorVelocities = new SideDependentList<>();
   private final SideDependentList<DoubleYoVariable> qKnees = new SideDependentList<>();

   private final SideDependentList<DoubleYoVariable> kneeHysteresisAmounts = new SideDependentList<>();
   private final SideDependentList<HysteresisFilteredYoVariable> qKneesWithHysteresis = new SideDependentList<>();

   public StepprKneeHysterisisCompensationLogProcessor(YoVariableHolder yoVariableHolder)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String sideLowerCase = robotSide.getCamelCaseNameForStartOfExpression();
         String sideUpperCase = robotSide.getCamelCaseNameForMiddleOfExpression();
         String actuatorName = sideLowerCase + "Ankle" + sideUpperCase + "Actuator";
         DoubleYoVariable ankleOutsideMotorPosition = (DoubleYoVariable) yoVariableHolder.getVariable(actuatorName, actuatorName + "MotorEncoderPosition");
         ankleOutsideMotorPositions.put(robotSide, ankleOutsideMotorPosition);

         DoubleYoVariable ankleOutsideMotorVelocity = (DoubleYoVariable) yoVariableHolder.getVariable(actuatorName, actuatorName + "MotorVelocityEstimate");
         ankleOutsideMotorVelocities.put(robotSide, ankleOutsideMotorVelocity);
         
         qKnees.put(robotSide, (DoubleYoVariable) yoVariableHolder.getVariable("bono", "q_" + robotSide.getSideNameFirstLetter().toLowerCase() + "_leg_kny"));
      }
      
      for (RobotSide robotSide : RobotSide.values)
      {
         DoubleYoVariable hysteresisAmount = new DoubleYoVariable(robotSide.getCamelCaseNameForStartOfExpression() + "KneeHysteresisAmount", registry);
         hysteresisAmount.set(0.015);
         HysteresisFilteredYoVariable qKneeWithHysteresis = new HysteresisFilteredYoVariable("hyst_q_" + robotSide.getShortLowerCaseName() + "_leg_kny", registry, hysteresisAmount);
         kneeHysteresisAmounts.put(robotSide, hysteresisAmount);
         qKneesWithHysteresis.put(robotSide, qKneeWithHysteresis);
      }
   }

   @Override
   public void processDataAtControllerRate()
   {
   }

   @Override
   public void processDataAtStateEstimatorRate()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         qKneesWithHysteresis.get(robotSide).update(qKnees.get(robotSide).getDoubleValue());
      }
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return null;
   }

}
