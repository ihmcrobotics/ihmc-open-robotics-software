package us.ihmc.quadrupedRobotics.input;

import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.simulationconstructionset.scripts.Script;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.Random;

public class QuadrupedTestTeleopScript implements Script
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final QuadrupedTeleopManager teleopManager;
   private final YoInteger counter = new YoInteger("counter", registry);
   private final int updateFrequency;

   private final Random random = new Random(125937L);
   private final int updateJitter;

   public QuadrupedTestTeleopScript(QuadrupedTeleopManager teleopManager, int updateFrequency, YoVariableRegistry parentRegistry)
   {
      this(teleopManager, updateFrequency, 0, parentRegistry);
   }

   public QuadrupedTestTeleopScript(QuadrupedTeleopManager teleopManager, int updateFrequency, int updateJitter, YoVariableRegistry parentRegistry)
   {
      this.teleopManager = teleopManager;
      this.updateFrequency = updateFrequency;
      this.updateJitter = updateJitter;
      parentRegistry.addChild(registry);
   }

   @Override
   public void doScript(double t)
   {
      if(counter.getIntegerValue() == 0)
      {
         teleopManager.update();
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
