package us.ihmc.tools.property;

import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceDirectory;

public class StoredPropertySetJavaGenerator
{
   private final String jsonFileName;
   private Class<?> clazz;
   private WorkspaceDirectory javaDirectory;

   public StoredPropertySetJavaGenerator(Class<?> clazz, WorkspaceDirectory javaDirectory)
   {
      this.clazz = clazz;
      this.javaDirectory = javaDirectory;
      jsonFileName = clazz.getSimpleName() + ".json";
   }

   public void generate()
   {
      JSONFileTools.loadFromClasspath(clazz, jsonFileName, node ->
      {
         if (node instanceof ObjectNode objectNode)
         {
            objectNode.fieldNames().forEachRemaining(fieldName ->
            {
               LogTools.info("Name: {} Value: {}", fieldName, objectNode.get(fieldName));
            });
         }
      });
   }
}
