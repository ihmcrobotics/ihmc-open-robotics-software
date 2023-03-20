package us.ihmc.behaviors.sharedControl;

// Java program to read JSON from a file

import com.fasterxml.jackson.databind.JsonNode;
import org.junit.Test;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import static org.junit.Assert.*;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

public class JSONReaderTest
{
   @Test
   public void testJSONReader()
   {
      System.out.println("parsing file \"ProMPAssistantTest.json\"");
      String configurationFile = "ProMPAssistantTest.json";
      LogTools.info("Loading parameters from resource: {}", configurationFile);
      WorkspaceResourceDirectory directory = new WorkspaceResourceDirectory(getClass(), "/us/ihmc/behaviors/sharedControl");
      WorkspaceResourceFile file = new WorkspaceResourceFile(directory, configurationFile);
      JSONFileTools.load(file, jsonNode ->
      {
         int numberBasisFunctions = jsonNode.get("numberBasisFunctions").asInt();
         boolean logEnabled = jsonNode.get("logging").asBoolean();

         System.out.println(numberBasisFunctions);
         assertTrue(numberBasisFunctions == 20);
         System.out.println(logEnabled);
         assertTrue(logEnabled);

         // getting tasks
         JsonNode tasksArrayNode = jsonNode.get("tasks");
         //iterating tasks
         for (JsonNode taskNode : tasksArrayNode)
         {
            Iterator<Map.Entry<String, JsonNode>> taskFields = taskNode.fields();
            while (taskFields.hasNext())
            {
               Map.Entry<String, JsonNode> taskPropertyMap = taskFields.next();
               switch (taskPropertyMap.getKey())
               {
                  case "name" ->
                  {
                     String taskName = taskPropertyMap.getValue().asText();
                     System.out.println(taskName);
                     assertEquals(taskName, "PushDoor");
                  }
                  case "bodyParts" ->
                  {
                     JsonNode bodyPartsArrayNode = taskPropertyMap.getValue();
                     HashMap<String, String> bodyPartsGeometry = new HashMap<>();
                     for (JsonNode bodyPartObject : bodyPartsArrayNode) {
                        Iterator<Map.Entry<String, JsonNode>> bodyPartFields = bodyPartObject.fields();
                        while (bodyPartFields.hasNext()) {
                           Map.Entry<String, JsonNode> bodyPartProperty = bodyPartFields.next();
                           switch (bodyPartProperty.getKey()) {
                              case "name" ->
                              {
                                 String name = bodyPartProperty.getValue().asText();
                                 System.out.println(name);
                                 assertTrue(name.equals("leftHand") || name.equals("rightHand"));
                              }
                              case "geometry" ->
                              {
                                 String geometry = bodyPartProperty.getValue().asText();
                                 System.out.println(geometry);
                                 assertEquals(geometry, "Pose");
                              }
                           }
                        }
                     }
                  }
               }
            }
         }
      });
   }
}
