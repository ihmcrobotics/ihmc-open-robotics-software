package us.ihmc.valkyrie.roboNet.schedule;

import java.io.IOException;
import java.io.InputStream;

import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.constructor.AbstractConstruct;
import org.yaml.snakeyaml.constructor.SafeConstructor;
import org.yaml.snakeyaml.nodes.Node;
import org.yaml.snakeyaml.nodes.ScalarNode;
import org.yaml.snakeyaml.nodes.Tag;

import us.ihmc.valkyrie.configuration.ValkyrieConfigurationRoot;

public class YamlWithIncludesLoader
{
   public static Object load(Class<?> loader, String subDirectory, String filename)
   {
      
      InputStream inputStream = loader.getResourceAsStream(subDirectory + "/" + filename);
      
      Yaml yaml = new Yaml(new IncludeConstructor(loader, subDirectory));
      Object object = yaml.load(inputStream);
      try
      {
         inputStream.close();
      }
      catch (IOException e)
      {
      }
      return object;
   }
   
   
   static class IncludeConstructor extends SafeConstructor
   {
      private final Class<?> loader;
      private final String subDirectory;
      
      public IncludeConstructor(Class<?> loader, String subDirectory)
      {
         this.loader = loader;
         this.subDirectory = subDirectory;
         
         this.yamlConstructors.put(new Tag("!include"), new IncludeYaml());
      }

      private class IncludeYaml extends AbstractConstruct
      {
         public Object construct(Node node)
         {
            String val = (String) constructScalar((ScalarNode) node);
            Yaml yaml = new Yaml(new IncludeConstructor(loader, subDirectory));
            InputStream inputStream = loader.getResourceAsStream(subDirectory + "/" + val);
            Object object = yaml.load(inputStream);
            try
            {
               inputStream.close();
            }
            catch (IOException e)
            {
            }
            return object;
         }
      }
   }
   
   public static void main(String[] args) throws IOException
   {
      System.out.println(YamlWithIncludesLoader.load(ValkyrieConfigurationRoot.class, "schedules", "main_sim.yaml"));
   }

}
