package us.ihmc.behaviors.sharedControl;

// Java program to read JSON from a file

import com.fasterxml.jackson.databind.JsonNode;
import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import static org.junit.jupiter.api.Assertions.*;

import java.util.HashMap;

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
            String taskName = taskNode.get("name").asText();
            System.out.println(taskName);
            assertEquals(taskName, "PushDoor");

            JsonNode bodyPartsArrayNode = taskNode.get("bodyParts");
            HashMap<String, String> bodyPartsGeometry = new HashMap<>();
            for (JsonNode bodyPartObject : bodyPartsArrayNode)
            {
               String name = bodyPartObject.get("name").asText();
               System.out.println(name);
               assertTrue(name.equals("leftHand") || name.equals("rightHand"));
               String geometry = bodyPartObject.get("geometry").asText();
               System.out.println(geometry);
               assertEquals(geometry, "Pose");
            }
         }
      });
   }
}
