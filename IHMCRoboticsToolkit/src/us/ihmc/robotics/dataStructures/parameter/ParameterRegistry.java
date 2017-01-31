package us.ihmc.robotics.dataStructures.parameter;

import java.io.IOException;
import java.io.InputStreamReader;
import java.net.URL;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

import com.google.common.base.Preconditions;
import com.google.common.io.LineReader;

/**
 * A centralized repository of all registered parameters in the system. Provides methods to load and store parameters to and from the filesystem.
 * <p/>
 * <pre>
 * ParameterRegistry parameterRegistry = ParameterRegistry.getInstance();
 * // First, load the default parameters file. This should likely always be done.
 * parameterRegistry.loadFromDefaultParametersResource();
 * // Then, load any overlay-specific parameter files. This can also be done at runtime from a user-facing utility.
 * parameterRegistry.loadFromResources("parameters_new_pid_gains_testing.conf");
 * </pre>
 * <p/>
 * Users of the Parameters API should not directly interface with this class except when initially loading parameter files. Instead, parameters can be created
 * with the {@link ParameterFactory} class.
 */
public class ParameterRegistry
{
   private static final class AtomicInstanceHolder
   {
      // Creation of INSTANCE is guaranteed to be thread-safe by the class loader.
      static ParameterRegistry INSTANCE = new ParameterRegistry();
   }

   /**
    * @return the singleton instance.
    */
   public static ParameterRegistry getInstance()
   {
      return AtomicInstanceHolder.INSTANCE;
   }
   
   public static void destroyAndRecreateInstance()
   {
      AtomicInstanceHolder.INSTANCE = null;
      AtomicInstanceHolder.INSTANCE = new ParameterRegistry();
   }

   // Disallow construction to enforce singleton.
   private ParameterRegistry()
   {

   }

   /**
    * The list of registered parameters.
    */
   private final List<Parameter> parameters = new ArrayList<>();

   /**
    * The list of loaded parameter lines to which no registered parameter exists. This is maintained in case a parameter is added after loading a file, in which
    * case the value will be pulled from this list (if it exists).
    */
   private final List<String> unregistered = new ArrayList<>();

   /**
    * Loads the resource with the given filename from the class path. If more than one class path resource exists with the given name then every matching
    * resource will be loaded in an undefined order.
    *
    * @param parametersPath the full parameters file name, i.e. "parameters_testing_123.conf"
    * @throws IOException if the resource is not found or an I/O error occurs
    */
   public void loadFromResources(String parametersPath) throws IOException
   {
      ClassLoader loader = Thread.currentThread().getContextClassLoader();
      Enumeration<URL> resources = loader.getResources(parametersPath);
      if (!resources.hasMoreElements())
      {
         throw new IOException("Cannot locate " + parametersPath + " as a classpath resource");
      }

      // Load from all parameters files in the class path that match the given name.
      while (resources.hasMoreElements())
      {
         URL url = resources.nextElement();
         Readable readable = new InputStreamReader(url.openStream());
         loadFromReadable(readable);
      }
   }

   /**
    * Load parameters from the given {@link Readable} source.
    *
    * @param readable
    * @throws IOException
    */
   public void loadFromReadable(Readable readable) throws IOException
   {
      LineReader reader = new LineReader(readable);

      String line;
      while ((line = reader.readLine()) != null)
      {
         // Check if any registered parameter matches, and pack the value if one does.
         boolean found = false;
         for (Parameter parameter : parameters)
         {
            if (parameter.tryLoad(line))
            {
               found = true;
               break;
            }
         }

         if (!found)
         {
//            System.out.println("Tried to load parameter not present in registry: " + line);
            unregistered.add(line);
         }
      }
   }

   void register(Parameter parameter)
   {
      Preconditions.checkNotNull(parameter, "Registered parameter cannot be null");

      // Check if any unregistered values match this new parameter, and pack the value if one does.
      for (String line : unregistered)
      {
         if (parameter.tryLoad(line))
         {
            unregistered.remove(line);
            break;
         }
      }

      parameters.add(parameter);
   }

   public List<Parameter> getParameters()
   {
      return parameters;
   }

   public Parameter getParameter(String path)
   {
      for (int i = 0; i < parameters.size(); i++)
      {
         if (parameters.get(i).getPath().equals(path))
         {
            return parameters.get(i);
         }
      }

      return null;
   }
   
}
