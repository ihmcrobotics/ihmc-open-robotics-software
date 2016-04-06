package us.ihmc.aware.params;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.io.Reader;
import java.net.URL;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

public class ParameterRegistry
{
   private static final class AtomicInstanceHolder
   {
      // Creation of INSTANCE is guaranteed to be thread-safe by the class loader.
      static final ParameterRegistry INSTANCE = new ParameterRegistry();
   }

   public static ParameterRegistry getInstance()
   {
      return AtomicInstanceHolder.INSTANCE;
   }

   private ParameterRegistry()
   {

   }

   private final List<Parameter> parameters = new ArrayList<>();

   public void loadFromDefaultParametersResource() throws IOException
   {
      final String defaultResourcePath = "parameters.conf";
      loadFromResources(defaultResourcePath);
   }

   public void loadFromResources(String name) throws IOException
   {
      ClassLoader loader = Thread.currentThread().getContextClassLoader();
      Enumeration<URL> resources = loader.getResources(name);
      if (!resources.hasMoreElements())
      {
         throw new IOException("Cannot locate " + name + " as a classpath resource");
      }

      while (resources.hasMoreElements())
      {
         URL url = resources.nextElement();
         Reader reader = new InputStreamReader(url.openStream());
         loadFromReader(reader);
      }
   }

   public void loadFromReader(Reader reader) throws IOException
   {
      BufferedReader br = new BufferedReader(reader);

      String line;
      while ((line = br.readLine()) != null)
      {
         for (Parameter parameter : parameters)
         {
            if (parameter.tryLoad(line))
            {
               break;
            }
         }
      }
   }

   public void save(PrintStream ps)
   {
      for (Parameter parameter : parameters)
      {
         ps.println(parameter.dump());
      }
   }

   void register(Parameter parameter)
   {
      parameters.add(parameter);
   }

   Parameter getParameter(String path)
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
