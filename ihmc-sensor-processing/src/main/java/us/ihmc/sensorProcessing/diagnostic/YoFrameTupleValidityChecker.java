package us.ihmc.sensorProcessing.diagnostic;

import java.util.EnumMap;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameTuple3D;

public class YoFrameTupleValidityChecker implements DiagnosticUpdatable
{
   private final YoVariableRegistry registry;
   private final YoFrameTuple3D input;

   private final EnumMap<Axis, DoubleYoVariableValidityChecker> validityCheckers = new EnumMap<>(Axis.class);

   public YoFrameTupleValidityChecker(String inputName, YoVariableRegistry parentRegistry)
   {
      this(inputName, null, parentRegistry);
   }
   
   public YoFrameTupleValidityChecker(YoFrameTuple3D input, YoVariableRegistry parentRegistry)
   {
      this(input.getNamePrefix() + input.getNameSuffix(), input, parentRegistry);
   }

   private YoFrameTupleValidityChecker(String inputName, YoFrameTuple3D input, YoVariableRegistry parentRegistry)
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

   public void update(Tuple3DBasics newInputValue)
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

   @Override
   public void enable()
   {
      for (Axis axis : Axis.values)
         validityCheckers.get(axis).enable();
   }

   @Override
   public void disable()
   {
      for (Axis axis : Axis.values)
         validityCheckers.get(axis).disable();
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
