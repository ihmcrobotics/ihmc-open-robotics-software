package us.ihmc.sensorProcessing.diagnostic;

import java.util.EnumMap;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameTuple3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoFrameTupleValidityChecker implements DiagnosticUpdatable
{
   private final YoRegistry registry;
   private final YoFrameTuple3D input;

   private final EnumMap<Axis3D, DoubleYoVariableValidityChecker> validityCheckers = new EnumMap<>(Axis3D.class);

   public YoFrameTupleValidityChecker(String inputName, YoRegistry parentRegistry)
   {
      this(inputName, null, parentRegistry);
   }
   
   public YoFrameTupleValidityChecker(YoFrameTuple3D input, YoRegistry parentRegistry)
   {
      this(input.getNamePrefix() + input.getNameSuffix(), input, parentRegistry);
   }

   private YoFrameTupleValidityChecker(String inputName, YoFrameTuple3D input, YoRegistry parentRegistry)
   {
      registry = new YoRegistry(inputName + "ValidityChecker");
      parentRegistry.addChild(registry);
      this.input = input;

      if (input != null)
      {
         validityCheckers.put(Axis3D.X, new DoubleYoVariableValidityChecker(input.getYoX(), registry));
         validityCheckers.put(Axis3D.Y, new DoubleYoVariableValidityChecker(input.getYoY(), registry));
         validityCheckers.put(Axis3D.Z, new DoubleYoVariableValidityChecker(input.getYoZ(), registry));
      }
      else
      {
         validityCheckers.put(Axis3D.X, new DoubleYoVariableValidityChecker(inputName + "X", registry));
         validityCheckers.put(Axis3D.Y, new DoubleYoVariableValidityChecker(inputName + "Y", registry));
         validityCheckers.put(Axis3D.Z, new DoubleYoVariableValidityChecker(inputName + "Z", registry));
      }
   }

   @Override
   public void update()
   {
      if (input == null)
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with a non null "
               + "input variable to call update(), otherwise use update(Vector3d)");

      for (Axis3D axis : Axis3D.values)
         validityCheckers.get(axis).update();
   }

   public void update(Tuple3DBasics newInputValue)
   {
      validityCheckers.get(Axis3D.X).update(newInputValue.getX());
      validityCheckers.get(Axis3D.Y).update(newInputValue.getY());
      validityCheckers.get(Axis3D.Z).update(newInputValue.getZ());
   }

   public void setupForLogging(String loggerName)
   {
      for (Axis3D axis : Axis3D.values)
         validityCheckers.get(axis).setupForLogging(loggerName);
   }

   @Override
   public void enable()
   {
      for (Axis3D axis : Axis3D.values)
         validityCheckers.get(axis).enable();
   }

   @Override
   public void disable()
   {
      for (Axis3D axis : Axis3D.values)
         validityCheckers.get(axis).disable();
   }

   public boolean isInputSane()
   {
      for (Axis3D axis : Axis3D.values)
      {
         if (!validityCheckers.get(axis).isInputSane())
            return false;
      }
      return true;
   }

   public boolean isInputAlive()
   {
      for (Axis3D axis : Axis3D.values)
      {
         if (!validityCheckers.get(axis).isInputAlive())
            return false;
      }
      return true;
   }

   public boolean variableCannotBeTrusted()
   {
      for (Axis3D axis : Axis3D.values)
      {
         if (validityCheckers.get(axis).variableCannotBeTrusted())
            return true;
      }
      return false;
   }
}
