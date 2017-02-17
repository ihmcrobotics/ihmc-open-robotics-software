package us.ihmc.sensorProcessing.diagnostic;

import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;

public class YoFrameQuaternionValidityChecker implements DiagnosticUpdatable
{
   private final YoVariableRegistry registry;
   private final YoFrameQuaternion input;

   private final DoubleYoVariableValidityChecker[] validityCheckers = new DoubleYoVariableValidityChecker[4];

   public YoFrameQuaternionValidityChecker(String inputName, YoVariableRegistry parentRegistry)
   {
      this(inputName, null, parentRegistry);
   }
   
   public YoFrameQuaternionValidityChecker(YoFrameQuaternion input, YoVariableRegistry parentRegistry)
   {
      this(input.getNamePrefix() + input.getNameSuffix(), input, parentRegistry);
   }

   private YoFrameQuaternionValidityChecker(String inputName, YoFrameQuaternion input, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(inputName + "ValidityChecker");
      parentRegistry.addChild(registry);
      this.input = input;

      if (input != null)
      {
         validityCheckers[0] = new DoubleYoVariableValidityChecker(input.getYoQx(), registry);
         validityCheckers[1] = new DoubleYoVariableValidityChecker(input.getYoQy(), registry);
         validityCheckers[2] = new DoubleYoVariableValidityChecker(input.getYoQz(), registry);
         validityCheckers[3] = new DoubleYoVariableValidityChecker(input.getYoQs(), registry);
      }
      else
      {
         validityCheckers[0] = new DoubleYoVariableValidityChecker(inputName + "Qx", registry);
         validityCheckers[1] = new DoubleYoVariableValidityChecker(inputName + "Qy", registry);
         validityCheckers[2] = new DoubleYoVariableValidityChecker(inputName + "Qz", registry);
         validityCheckers[3] = new DoubleYoVariableValidityChecker(inputName + "Qs", registry);
      }
   }

   @Override
   public void update()
   {
      if (input == null)
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with a non null "
               + "input variable to call update(), otherwise use update(Vector3d)");

      for (DoubleYoVariableValidityChecker validityChecker : validityCheckers)
         validityChecker.update();
   }

   public void update(Quaternion newInputValue)
   {
      validityCheckers[0].update(newInputValue.getX());
      validityCheckers[1].update(newInputValue.getY());
      validityCheckers[2].update(newInputValue.getZ());
      validityCheckers[3].update(newInputValue.getS());
   }

   public void setupForLogging(String loggerName)
   {
      for (DoubleYoVariableValidityChecker validityChecker : validityCheckers)
         validityChecker.setupForLogging(loggerName);
   }
   
   @Override
   public void enable()
   {
      for (DoubleYoVariableValidityChecker validityChecker : validityCheckers)
         validityChecker.enable();
   }
   
   @Override
   public void disable()
   {
      for (DoubleYoVariableValidityChecker validityChecker : validityCheckers)
         validityChecker.disable();
   }

   public boolean isInputSane()
   {
      for (DoubleYoVariableValidityChecker validityChecker : validityCheckers)
      {
         if (!validityChecker.isInputSane())
            return false;
      }
      return true;
   }

   public boolean isInputAlive()
   {
      for (DoubleYoVariableValidityChecker validityChecker : validityCheckers)
      {
         if (!validityChecker.isInputAlive())
            return false;
      }
      return true;
   }

   public boolean variableCannotBeTrusted()
   {
      for (DoubleYoVariableValidityChecker validityChecker : validityCheckers)
      {
         if (validityChecker.variableCannotBeTrusted())
            return true;
      }
      return false;
   }
}
