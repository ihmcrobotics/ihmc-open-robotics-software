package us.ihmc.alexander.parameters;

import java.util.Objects;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public abstract class AbstractRobotParameters implements ParameterInitializationHelper
{
   protected final YoRegistry root;

   protected AbstractRobotParameters(YoRegistry root)
   {
      this.root = Objects.requireNonNull(root, "root");
   }

   // ---------------- Registry finders ----------------

   @Override
   public YoRegistry findReg(String name)
   {
      YoRegistry reg = root.findRegistry(name);
      if (reg == null)
         throw new IllegalArgumentException("Registry not found: " + name);
      return reg;
   }

   @Override
   public YoRegistry findReg(YoRegistry parent, String name)
   {
      YoRegistry reg = parent.findRegistry(name);
      if (reg == null)
         throw new IllegalArgumentException("Registry not found under " + parent.getName() + ": " + name);
      return reg;
   }

   @Override
   public YoRegistry findReg(YoRegistry parent, String... names)
   {
      YoRegistry r = findRegIfPresent(parent, names);
      if (r == null)
         throw new IllegalArgumentException("Registry not found under " + parent.getName() + " for any of: " + String.join(", ", names));
      return r;
   }

   @Override
   public YoRegistry findRegIfPresent(YoRegistry parent, String... names)
   {
      if (parent == null) return null;
      for (String n : names)
      {
         YoRegistry r = parent.findRegistry(n);
         if (r != null) return r;
      }
      return null;
   }

   // ---------------- Setters: single-name exact ----------------

   @Override
   public YoBoolean setBoolean(YoRegistry reg, String varName, boolean value)
   {
      Object var = Objects.requireNonNull(reg.findVariable(varName),
                                          "Variable not found: " + reg.getName() + "." + varName);
      if (!(var instanceof YoBoolean))
         throw new IllegalArgumentException("Expected YoBoolean for " + reg.getName() + "." + varName
                                            + " but got " + var.getClass().getSimpleName());
      YoBoolean yo = (YoBoolean) var;
      yo.set(value);
      return yo;
   }

   @Override
   public YoDouble setDouble(YoRegistry reg, String varName, double value)
   {
      Object var = Objects.requireNonNull(reg.findVariable(varName),
                                          "Variable not found: " + reg.getName() + "." + varName);
      if (!(var instanceof YoDouble))
         throw new IllegalArgumentException("Expected YoDouble for " + reg.getName() + "." + varName
                                            + " but got " + var.getClass().getSimpleName());
      YoDouble yo = (YoDouble) var;
      yo.set(value);
      return yo;
   }

   @Override
   public YoInteger setInt(YoRegistry reg, String varName, int value)
   {
      Object var = Objects.requireNonNull(reg.findVariable(varName),
                                          "Variable not found: " + reg.getName() + "." + varName);
      if (!(var instanceof YoInteger))
         throw new IllegalArgumentException("Expected YoInteger for " + reg.getName() + "." + varName
                                            + " but got " + var.getClass().getSimpleName());
      YoInteger yo = (YoInteger) var;
      yo.set(value);
      return yo;
   }

   // ---------------- Setters: alias enable----------------

   @Override
   public YoBoolean setAnyBoolean(YoRegistry reg, String[] varNames, boolean value)
   {
      for (String n : varNames)
      {
         Object v = reg.findVariable(n);
         if (v instanceof YoBoolean)
         {
            YoBoolean b = (YoBoolean) v;
            b.set(value);
            return b;
         }
      }
      throw new IllegalArgumentException("YoBoolean not found in " + reg.getName()
                                         + " for any of: " + String.join(", ", varNames));
   }

   @Override
   public YoDouble setAnyDouble(YoRegistry reg, String[] varNames, double value)
   {
      for (String n : varNames)
      {
         Object v = reg.findVariable(n);
         if (v instanceof YoDouble)
         {
            YoDouble d = (YoDouble) v;
            d.set(value);
            return d;
         }
      }
      throw new IllegalArgumentException("YoDouble not found in " + reg.getName()
                                         + " for any of: " + String.join(", ", varNames));
   }

   @Override
   public YoInteger setAnyInt(YoRegistry reg, String[] varNames, int value)
   {
      for (String n : varNames)
      {
         Object v = reg.findVariable(n);
         if (v instanceof YoInteger)
         {
            YoInteger i = (YoInteger) v;
            i.set(value);
            return i;
         }
      }
      throw new IllegalArgumentException("YoInteger not found in " + reg.getName()
                                         + " for any of: " + String.join(", ", varNames));
   }
}