package us.ihmc.sensorProcessing.diagnostic;

import java.util.EnumMap;

import javax.vecmath.Tuple3d;

import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameTuple;

public class YoFrameTupleValidityChecker implements DiagnosticUpdatable
{
   private final YoVariableRegistry registry;
   private final YoFrameTuple<?> input;

   private final EnumMap<Axis, DoubleYoVariableValidityChecker> validityCheckers = new EnumMap<>(Axis.class);

   public YoFrameTupleValidityChecker(String inputName, YoVariableRegistry parentRegistry)
   {
      this(inputName, null, parentRegistry);
   }
   
   public YoFrameTupleValidityChecker(YoFrameTuple<?> input, YoVariableRegistry parentRegistry)
   {
      this(input.getNamePrefix() + input.getNameSuffix(), input, parentRegistry);
   }

   private YoFrameTupleValidityChecker(String inputName, YoFrameTuple<?> input, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(inputName + "ValidityChecker");
      parentRegistry.addChild(registry);
      this.input = input;

      if (input != null)
      {
         validityCheckers.put(Axis.X, new DoubleYoVariableValidityChecker(input.getYoX(), registry));
         validityCheckers.put(Axis.Y, new DoubleYoVariableValidityChecker(input.getYoY(), registry));
         validityCheckers.put(Axis.Z, new DoubleYoVariableValidityChecker(input.getYoZ(), registry));
      }
      else
      {
         validityCheckers.put(Axis.X, new DoubleYoVariableValidityChecker(inputName + "X", registry));
         validityCheckers.put(Axis.Y, new DoubleYoVariableValidityChecker(inputName + "Y", registry));
         validityCheckers.put(Axis.Z, new DoubleYoVariableValidityChecker(inputName + "Z", registry));
      }
   }

   @Override
   public void update()
   {
      if (input == null)
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with a non null "
               + "input variable to call update(), otherwise use update(Vector3d)");

      for (Axis axis : Axis.values)
         validityCheckers.get(axis).update();
   }

   public void update(Tuple3d newInputValue)
   {
      validityCheckers.get(Axis.X).update(newInputValue.getX());
      validityCheckers.get(Axis.Y).update(newInputValue.getY());
      validityCheckers.get(Axis.Z).update(newInputValue.getZ());
   }

   public void setupForLogging(String loggerName)
   {
      for (Axis axis : Axis.values)
         validityCheckers.get(axis).setupForLogging(loggerName);
   }

   public boolean isInputSane()
   {
      for (Axis axis : Axis.values)
      {
         if (!validityCheckers.get(axis).isInputSane())
            return false;
      }
      return true;
   }

   public boolean isInputAlive()
   {
      for (Axis axis : Axis.values)
      {
         if (!validityCheckers.get(axis).isInputAlive())
            return false;
      }
      return true;
   }

   public boolean variableCannotBeTrusted()
   {
      for (Axis axis : Axis.values)
      {
         if (validityCheckers.get(axis).variableCannotBeTrusted())
            return true;
      }
      return false;
   }
}
