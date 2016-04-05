package us.ihmc.aware.config;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
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

   public void load(BufferedReader br) throws IOException
   {
      String line = null;
      while ((line = br.readLine()) != null)
      {
         for (Parameter parameter : parameters)
         {
            if(parameter.tryLoad(line))
            {
               System.out.println("Setting " + parameter.getPath());
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

   protected void register(Parameter parameter)
   {
      parameters.add(parameter);
   }

   protected Parameter getParameter(String path)
   {
      for (int i = 0; i < parameters.size(); i++)
      {
         if(parameters.get(i).getPath().equals(path))
         {
            return parameters.get(i);
         }
      }

      return null;
   }
}
