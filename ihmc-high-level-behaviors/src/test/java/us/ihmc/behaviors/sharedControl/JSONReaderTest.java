package us.ihmc.behaviors.sharedControl;

// Java program to read JSON from a file

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.Iterator;
import java.util.Map;

public class JSONReaderTest
{
   public static void main(String[] args)
   {
      // parsing file "ProMPAssistant.json"
      try
      {
         JSONObject jsonObject = (JSONObject) new JSONParser().parse(new FileReader(Paths.get(System.getProperty("user.home"),
                                                                                              "repository-group/ihmc-open-robotics-software/ihmc-high-level-behaviors/src/main/resources/us/ihmc/behaviors/sharedControl/ProMPAssistant.json")
                                                                                         .toString()));
         int numberObservations = (int) ((long) jsonObject.get("numberObservations"));
         boolean logEnabled = (boolean) jsonObject.get("logging");

         System.out.println(numberObservations);
         System.out.println(logEnabled);
         // getting tasks
         JSONArray tasksArray = (JSONArray) jsonObject.get("tasks");
         //iterating tasks
         Iterator taskIterator = tasksArray.iterator();
         while (taskIterator.hasNext())
         {
            Iterator<Map.Entry> taskPropertiesIterator = ((Map) taskIterator.next()).entrySet().iterator();
            while (taskPropertiesIterator.hasNext())
            {
               Map.Entry taskPropertyMap = taskPropertiesIterator.next();
               switch (taskPropertyMap.getKey().toString())
               {
                  case "name":
                     String taskName = (String) taskPropertyMap.getValue();
                     System.out.println(taskName);
                     break;
                  case "relevantBodyPart":
                     String relevantBodyPart = (String) taskPropertyMap.getValue();
                     System.out.println(relevantBodyPart);
                     break;
                  case "bodyParts":
                     JSONArray bodyPartsArray = (JSONArray) taskPropertyMap.getValue();
                     for (Object bodyPartObject : bodyPartsArray)
                     {
                        JSONObject jsonBodyPartObject = (JSONObject) bodyPartObject;
                        jsonBodyPartObject.keySet().forEach(bodyPartProperty ->
                                                            {
                                                               if ("name".equals(bodyPartProperty.toString()))
                                                               {
                                                                  // extract your value here
                                                                  String name = String.valueOf(jsonBodyPartObject.get(bodyPartProperty));
                                                                  System.out.println(name);
                                                               }
                                                               else if ("geometry".equals(bodyPartProperty.toString()))
                                                               {
                                                                  // extract your value here
                                                                  String geometry = String.valueOf(jsonBodyPartObject.get(bodyPartProperty));
                                                                  System.out.println(geometry);
                                                               }
                                                            });
                     }
                     break;
                  default:
                     break;
               }
            }
         }
      }
      catch (FileNotFoundException ex)
      {
         ex.printStackTrace();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      catch (ParseException e)
      {
         throw new RuntimeException(e);
      }
   }
}
