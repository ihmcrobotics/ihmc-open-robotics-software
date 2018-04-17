package us.ihmc.quadrupedRobotics.input;

import us.ihmc.quadrupedRobotics.input.managers.QuadrupedBodyPoseTeleopManager;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedStepTeleopManager;
import us.ihmc.simulationconstructionset.scripts.Script;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.Random;

public class QuadrupedTestTeleopScript implements Script
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final QuadrupedStepTeleopManager stepTeleopManager;
   private final QuadrupedBodyPoseTeleopManager bodyPoseTeleopManager;
   private final YoInteger counter = new YoInteger("counter", registry);
   private final int updateFrequency;

   private final Random random = new Random(125937L);
   private final int updateJitter;

   public QuadrupedTestTeleopScript(QuadrupedStepTeleopManager teleopManager, QuadrupedBodyPoseTeleopManager bodyPoseTeleopManager, int updateFrequency, YoVariableRegistry parentRegistry)
   {
      this(teleopManager, bodyPoseTeleopManager, updateFrequency, 0, parentRegistry);
   }

   public QuadrupedTestTeleopScript(QuadrupedStepTeleopManager stepTeleopManager, QuadrupedBodyPoseTeleopManager bodyPoseTeleopManager, int updateFrequency, int updateJitter, YoVariableRegistry parentRegistry)
   {
      this.stepTeleopManager = stepTeleopManager;
      this.bodyPoseTeleopManager = bodyPoseTeleopManager;
      this.updateFrequency = updateFrequency;
      this.updateJitter = updateJitter;
      parentRegistry.addChild(registry);
   }

   @Override
   public void doScript(double t)
   {
      if(counter.getIntegerValue() == 0)
      {
         stepTeleopManager.update();
         bodyPoseTeleopManager.update();
         updateCounter();
      }
      else
      {
         counter.decrement();
      }
   }

   private void updateCounter()
   {
      if(updateJitter > 0)
      {
         int countVariation = random.nextInt(updateJitter) * (random.nextBoolean() ? -1 : 1);
         int newCount = Math.max(0, countVariation + updateFrequency);
         counter.set(newCount - 1);
      }
      else
      {
         counter.set(updateFrequency - 1);
      }
   }
}
