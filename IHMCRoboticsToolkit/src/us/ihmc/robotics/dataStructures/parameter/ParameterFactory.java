package us.ihmc.robotics.dataStructures.parameter;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

/**
 * A factory for creating and registering parameters and their default values.
 * <p/>
 * This class is the front-end for declaring parameters in user classes.
 * <p/>
 * Example usage:
 * <pre>
 * class MyClass {
 *    ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
 *    DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 2);
 * }
 * </pre>
 */
public class ParameterFactory
{
   private final String namespace;
   private final YoVariableRegistry registry;

   /**
    * Create a new parameter factory. Each namespace (class) should create its own
    *
    * @param namespace
    */
   private ParameterFactory(Class<?> namespace)
   {
      this.namespace = namespace.getName();
      this.registry = null;
   }

   private ParameterFactory(Class<?> namespace, YoVariableRegistry registry)
   {
      this.namespace = namespace.getName();
      this.registry = registry;
   }

   /**
    * Creates a new ParameterFactory without a {@link YoVariableRegistry}. Without a registry, {@link YoVariable}s will not be registered at all. If you would
    * like to access parameter values as {@link YoVariable}s, then use {@link #createWithRegistry(Class, YoVariableRegistry)} instead.
    *
    * @param namespace the namespace in which to register parameters
    * @return a new parameter factory
    */
   public static ParameterFactory createWithoutRegistry(Class<?> namespace)
   {
      return new ParameterFactory(namespace);
   }

   /**
    * Creates a new ParameterFactory with a {@link YoVariableRegistry}. For {@link Parameter} types that are supported, {@link YoVariable}s will be created and
    * values will be mirrored.
    *
    * @param namespace the namespace in which to register parameters
    * @return a new parameter factory
    */
   public static ParameterFactory createWithRegistry(Class<?> namespace, YoVariableRegistry registry)
   {
      return new ParameterFactory(namespace, registry);
   }

   public BooleanParameter createBoolean(String name, boolean defaultValue)
   {
      final BooleanParameter parameter = new BooleanParameter(namespace + "." + name, defaultValue);
      register(parameter);

      if (registry != null)
      {
         final BooleanYoVariable variable = new BooleanYoVariable("param__" + parameter.getShortPath(), registry);
         variable.set(parameter.get());
         variable.addVariableChangedListener(new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable<?> v)
            {
               parameter.set(((BooleanYoVariable) v).getBooleanValue());
            }
         });

         parameter.addChangeListener(new ParameterChangeListener()
         {
            @Override
            public void onChange(Parameter parameter)
            {
               variable.set(((BooleanParameter) parameter).get(),false);
            }
         });
      }

      return parameter;
   }

   public DoubleParameter createDouble(String name, double defaultValue)
   {
      final DoubleParameter parameter = new DoubleParameter(namespace + "." + name, defaultValue);
      register(parameter);

      if (registry != null)
      {
         final DoubleYoVariable variable = new DoubleYoVariable("param__" + parameter.getShortPath(), registry);
         variable.set(parameter.get());
         variable.addVariableChangedListener(new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable<?> v)
            {
               parameter.set(((DoubleYoVariable) v).getDoubleValue());
            }
         });

         parameter.addChangeListener(new ParameterChangeListener()
         {
            @Override
            public void onChange(Parameter parameter)
            {
               variable.set(((DoubleParameter) parameter).get(), false);
            }
         });
      }
      return parameter;
   }

   public DoubleArrayParameter createDoubleArray(String name, double... defaultValue)
   {
      final DoubleArrayParameter parameter = new DoubleArrayParameter(namespace + "." + name, defaultValue);
      register(parameter);

      if (registry != null)
      {
         for (int i = 0; i < parameter.get().length; i++)
         {
            final int count = i;

            final DoubleYoVariable variable = new DoubleYoVariable("param__" + parameter.getShortPath() + count, registry);
            variable.set(parameter.get(i));
            variable.addVariableChangedListener(new VariableChangedListener()
            {
               @Override
               public void variableChanged(YoVariable<?> v)
               {
                  parameter.set(count, ((DoubleYoVariable) v).getDoubleValue());
               }
            });

            parameter.addChangeListener(new ParameterChangeListener()
            {
               @Override
               public void onChange(Parameter parameter)
               {
                  variable.set(((DoubleArrayParameter) parameter).get()[count], false);
               }
            });
         }
      }

      return parameter;
   }

   public IntegerParameter createInteger(String name, int defaultValue)
   {
      final IntegerParameter parameter = new IntegerParameter(namespace + "." + name, defaultValue);
      register(parameter);

      if (registry != null)
      {
         final IntegerYoVariable variable = new IntegerYoVariable("param__" + parameter.getShortPath(), registry);
         variable.set(parameter.get());
         variable.addVariableChangedListener(new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable<?> v)
            {
               parameter.set(((IntegerYoVariable) v).getIntegerValue());
            }
         });

         parameter.addChangeListener(new ParameterChangeListener()
         {
            @Override
            public void onChange(Parameter parameter)
            {
               variable.set(((IntegerParameter) parameter).get(), false);
            }
         });
      }
      return parameter;
   }

   public IntegerArrayParameter createIntegerArray(String name, int... defaultValue)
   {
      final IntegerArrayParameter parameter = new IntegerArrayParameter(namespace + "." + name, defaultValue);
      register(parameter);

      if (registry != null)
      {
         for (int i = 0; i < parameter.get().length; i++)
         {
            final int count = i;

            final IntegerYoVariable variable = new IntegerYoVariable("param__" + parameter.getShortPath() + count, registry);
            variable.set(parameter.get(i));
            variable.addVariableChangedListener(new VariableChangedListener()
            {
               @Override
               public void variableChanged(YoVariable<?> v)
               {
                  parameter.set(count, ((IntegerYoVariable) v).getIntegerValue());
               }
            });

            parameter.addChangeListener(new ParameterChangeListener()
            {
               @Override
               public void onChange(Parameter parameter)
               {
                  variable.set(((IntegerArrayParameter) parameter).get()[count], false);
               }
            });
         }
      }

      return parameter;
   }

   public StringParameter createString(String name, String defaultValue)
   {
      StringParameter parameter = new StringParameter(namespace + "." + name, defaultValue);
      register(parameter);

      return parameter;
   }

   private static void register(Parameter parameter)
   {
      ParameterRegistry.getInstance().register(parameter);
   }
}
