package us.ihmc.robotics.dataStructures.validation;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;

import javassist.CannotCompileException;
import javassist.ClassPool;
import javassist.CtClass;
import javassist.CtField;
import javassist.CtMethod;
import javassist.NotFoundException;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

/**
 * Helper class to test that variables in a registry are only changed by a single thread
 * @author jesper
 *
 */
public class YoVariableThreadAccessValidator
{
   private static final boolean DEBUG = false;
   
   private final YoVariableRegistry root;
   private Thread accessorThread = null;
   
   private static boolean REGISTERED_ACCESS_VALIDATOR = false;
   
   public static void registerAccessValidator()
   {
      ClassPool pool = ClassPool.getDefault();
      try
      {
         
         CtClass yoVariable = pool.get("us.ihmc.simulationconstructionset.YoVariable");
         CtClass yoVariableThreadAccessValidator = pool.get(YoVariableThreadAccessValidator.class.getCanonicalName());
         CtField validatorField = new CtField(yoVariableThreadAccessValidator, "validator", yoVariable);
         validatorField.setModifiers(Modifier.PUBLIC);
         yoVariable.addField(validatorField, "null");
         yoVariable.toClass();

         
         CtClass doubleYoVariable = pool.get("us.ihmc.simulationconstructionset.DoubleYoVariable");
         CtMethod method = doubleYoVariable.getDeclaredMethod("getDoubleValue");
         method.insertBefore("if(validator != null) { validator.validateReadAccess(this); }");
         doubleYoVariable.toClass();
         
         REGISTERED_ACCESS_VALIDATOR = true;
         
      }
      catch (NotFoundException | CannotCompileException e)
      {
         e.printStackTrace();
      }
   }

   public YoVariableThreadAccessValidator(YoVariableRegistry root)
   {
      this.root = root;
   }

   public void validateReadAccess(YoVariable<?> v)
   {
      Thread currentThread = Thread.currentThread();
      
      switch(currentThread.getName())
      {
      case "AWT-EventQueue-0":
      case "SCS simulation thread":
      case "CombinedVarPanelTimer":
         return;
      }


      if (accessorThread == null)
      {
         accessorThread = currentThread;
      }
      if (!currentThread.equals(accessorThread))
      {
         try
         {
            throw new Exception("Variable " + v.getFullNameWithNameSpace() + " read by thread " + currentThread + ", expected: " + accessorThread);
         }
         catch(Exception e)
         {
            if (DEBUG)
            {
               e.printStackTrace();
            }
         }
      }
   }

   private void testAccess(YoVariable<?> v)
   {
      Thread currentThread = Thread.currentThread();

      switch (currentThread.getName())
      {
      case "AWT-EventQueue-0":
      case "SCS simulation thread":
         System.out.println("[" + getClass().getSimpleName() + "] Variable " + v.getName() + " was changed from the UI.");
         return;
      }

      if (accessorThread == null)
      {
         accessorThread = currentThread;
      }

      if (!currentThread.equals(accessorThread))
      {
         try
         {
            throw new Exception("Variable " + v.getFullNameWithNameSpace() + " changed by thread " + currentThread + ", expected: " + accessorThread);
         }
         catch(Exception e)
         {
            if (DEBUG)
            {
               e.printStackTrace();
            }
         }
      }
   }

   public void start()
   {
      ArrayList<YoVariable<?>> variables = root.getAllVariablesIncludingDescendants();
      
      Field validator = null;
      if(REGISTERED_ACCESS_VALIDATOR)
      {
         try
         {
            validator = YoVariable.class.getField("validator");
         }
         catch (NoSuchFieldException | SecurityException e)
         {
            throw new RuntimeException(e);
         }
      }
      
      for (int i = 0; i < variables.size(); i++)
      {
         YoVariable<?> yoVariable = variables.get(i);
         yoVariable.addVariableChangedListener(new VariableChangedListener()
         {

            @Override
            public void variableChanged(YoVariable<?> v)
            {
               testAccess(v);
            }
         });
         
         if(validator != null)
         {
            try
            {
               validator.set(yoVariable, this);
            }
            catch (IllegalArgumentException | IllegalAccessException e)
            {
               throw new RuntimeException(e);
            }
         }

         
         
      }

   }
}
