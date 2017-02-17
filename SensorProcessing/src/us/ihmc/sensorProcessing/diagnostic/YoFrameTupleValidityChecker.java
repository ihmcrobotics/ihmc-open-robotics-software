package us.ihmc.sensorProcessing.diagnostic;

import java.util.EnumMap;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.math.frames.YoFrameTuple;

public class YoFrameTupleValidityChecker implements DiagnosticUpdatable
{
   private final YoVariableRegistry registry;
   private final YoFrameTuple<?, ?> input;

   private final EnumMap<Direction, DoubleYoVariableValidityChecker> validityCheckers = new EnumMap<>(Direction.class);

   public YoFrameTupleValidityChecker(String inputName, YoVariableRegistry parentRegistry)
   {
      this(inputName, null, parentRegistry);
   }
   
   public YoFrameTupleValidityChecker(YoFrameTuple<?, ?> input, YoVariableRegistry parentRegistry)
   {
      this(input.getNamePrefix() + input.getNameSuffix(), input, parentRegistry);
   }

   private YoFrameTupleValidityChecker(String inputName, YoFrameTuple<?, ?> input, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(inputName + "ValidityChecker");
      parentRegistry.addChild(registry);
      this.input = input;

      if (input != null)
      {
         validityCheckers.put(Direction.X, new DoubleYoVariableValidityChecker(input.getYoX(), registry));
         validityCheckers.put(Direction.Y, new DoubleYoVariableValidityChecker(input.getYoY(), registry));
         validityCheckers.put(Direction.Z, new DoubleYoVariableValidityChecker(input.getYoZ(), registry));
      }
      else
      {
         validityCheckers.put(Direction.X, new DoubleYoVariableValidityChecker(inputName + "X", registry));
         validityCheckers.put(Direction.Y, new DoubleYoVariableValidityChecker(inputName + "Y", registry));
         validityCheckers.put(Direction.Z, new DoubleYoVariableValidityChecker(inputName + "Z", registry));
      }
   }

   @Override
   public void update()
   {
      if (input == null)
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with a non null "
               + "input variable to call update(), otherwise use update(Vector3d)");

      for (Direction direction : Direction.values)
         validityCheckers.get(direction).update();
   }

   public void update(Tuple3DBasics newInputValue)
   {
      validityCheckers.get(Direction.X).update(newInputValue.getX());
      validityCheckers.get(Direction.Y).update(newInputValue.getY());
      validityCheckers.get(Direction.Z).update(newInputValue.getZ());
   }

   public void setupForLogging(String loggerName)
   {
      for (Direction direction : Direction.values)
         validityCheckers.get(direction).setupForLogging(loggerName);
   }

   @Override
   public void enable()
   {
      for (Direction direction : Direction.values)
         validityCheckers.get(direction).enable();
   }

   @Override
   public void disable()
   {
      for (Direction direction : Direction.values)
         validityCheckers.get(direction).disable();
   }

   public boolean isInputSane()
   {
      for (Direction direction : Direction.values)
      {
         if (!validityCheckers.get(direction).isInputSane())
            return false;
      }
      return true;
   }

   public boolean isInputAlive()
   {
      for (Direction direction : Direction.values)
      {
         if (!validityCheckers.get(direction).isInputAlive())
            return false;
      }
      return true;
   }

   public boolean variableCannotBeTrusted()
   {
      for (Direction direction : Direction.values)
      {
         if (validityCheckers.get(direction).variableCannotBeTrusted())
            return true;
      }
      return false;
   }
}
