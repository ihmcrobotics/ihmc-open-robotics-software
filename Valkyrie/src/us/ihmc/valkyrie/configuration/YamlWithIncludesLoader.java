package us.ihmc.valkyrie.configuration;

import java.io.IOException;
import java.io.InputStream;

import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.constructor.AbstractConstruct;
import org.yaml.snakeyaml.constructor.SafeConstructor;
import org.yaml.snakeyaml.nodes.Node;
import org.yaml.snakeyaml.nodes.ScalarNode;
import org.yaml.snakeyaml.nodes.Tag;

public class YamlWithIncludesLoader
{
   public static Object load(String subDirectory, String filename)
   {

      InputStream inputStream = YamlWithIncludesLoader.class.getClassLoader().getResourceAsStream(subDirectory + "/" + filename);

      Yaml yaml = new Yaml(new IncludeConstructor(subDirectory));
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
      private final String subDirectory;

      public IncludeConstructor(String subDirectory)
      {
         this.subDirectory = subDirectory;

         this.yamlConstructors.put(new Tag("!include"), new IncludeYaml());
      }

      private class IncludeYaml extends AbstractConstruct
      {
         public Object construct(Node node)
         {
            String val = (String) constructScalar((ScalarNode) node);
            Yaml yaml = new Yaml(new IncludeConstructor(subDirectory));
            InputStream inputStream = YamlWithIncludesLoader.class.getClassLoader().getResourceAsStream(subDirectory + "/" + val);
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


}
