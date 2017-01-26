package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;

import com.thoughtworks.xstream.io.StreamException;

import us.ihmc.humanoidRobotics.communication.packets.walking.EndOfScriptCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseWalkingMessage;
import us.ihmc.tools.io.printing.PrintTools;

/**
 * Created with IntelliJ IDEA.
 * User: pneuhaus
 * Date: 5/31/13
 * Time: 3:07 PM
 * To change this template use File | Settings | File Templates.
 */
public class ScriptTools
{

   public static void removeAllUnnecessaryScripObjects(ArrayList<ScriptObject> listToEdit)
   {
      removeAllEndOfScripts(listToEdit);
      removeAllPauses(listToEdit);
      removeAllZeroStepLists(listToEdit);
      //removeAllFootPosePackets(listToEdit);
   }

   private static void removePausesAndZeroStepLists(ArrayList<ScriptObject> listToEdit)
   {
      int beforeSize, afterSize;
      do
      {
         beforeSize = listToEdit.size();
         removePausesAndZeroStepListsInnnerOpeartion(listToEdit);
         afterSize = listToEdit.size();
      }
      while (afterSize < beforeSize);
   }

   private static void removeAllEndOfScripts(ArrayList<ScriptObject> listToEdit)
   {
      for (int i = listToEdit.size() - 1; i >= 0; i--)
      {
         if (listToEdit.get(i).getScriptObject() instanceof EndOfScriptCommand)
            listToEdit.remove(i);
      }
   }

   private static void removeAllFootPosePackets(ArrayList<ScriptObject> listToEdit)
   {
      for (int i = listToEdit.size() - 1; i >= 0; i--)
      {
         if (listToEdit.get(i).getScriptObject() instanceof FootTrajectoryMessage)
            listToEdit.remove(i);
      }
   }


   private static void removeAllPauses(ArrayList<ScriptObject> listToEdit)
   {
      for (int i = listToEdit.size() - 1; i >= 0; i--)
      {
         if (listToEdit.get(i).getScriptObject() instanceof PauseWalkingMessage)
            listToEdit.remove(i);
      }
   }

   private static void removeAllZeroStepLists(ArrayList<ScriptObject> listToEdit)
   {
      for (int i = listToEdit.size() - 1; i >= 0; i--)
      {
         if (listToEdit.get(i).getScriptObject() instanceof FootstepDataListMessage)
         {
            if (((FootstepDataListMessage) listToEdit.get(i).getScriptObject()).size() == 0)
               listToEdit.remove(i);
         }
      }
   }

   private static void removePausesAndZeroStepListsInnnerOpeartion(ArrayList<ScriptObject> listToEdit)
   {
      boolean previousWasPause = false;
      ArrayList<Integer> elementsToRemove = new ArrayList<Integer>();

      for (int i = 0; i < listToEdit.size(); i++)
      {
         ScriptObject scriptObject = listToEdit.get(i);

         if (scriptObject.getScriptObject() instanceof PauseWalkingMessage)
         {
            if (((PauseWalkingMessage) scriptObject.getScriptObject()).isPaused())
            {
               if (previousWasPause)
               {
                  elementsToRemove.add(i);
               }
               else
               {
                  previousWasPause = true;
               }
            }
            else
            {
               previousWasPause = false;
            }

         }
         else
            previousWasPause = false;

         if (scriptObject.getScriptObject() instanceof FootstepDataListMessage)
         {
            if (((FootstepDataListMessage) scriptObject.getScriptObject()).size() == 0)
               elementsToRemove.add(i);
         }
      }

      Collections.reverse(elementsToRemove);

      for (Integer integer : elementsToRemove)
      {
         listToEdit.remove(integer.intValue());
      }
   }

   public static void main(String[] args) //throws IOException
   {
      File file = ScriptFileSelector.getScriptFileFromUserSelection(ScriptEngineSettings.extension);

      System.out.println("Loading file: " + file.getAbsolutePath());

      ScriptFileLoader loader = null;
      try
      {
         loader = new ScriptFileLoader(file);
      }
      catch (StreamException | IOException e)
      {
         PrintTools.error(e.getMessage());
         System.exit(-1);
      }
      
      ArrayList<ScriptObject> scriptObjects = loader.readIntoList();
      loader.close();

      System.out.println("Before:");
      System.out.println(ScriptObject.getListInfo(scriptObjects));

      ScriptTools.removeAllUnnecessaryScripObjects(scriptObjects);

      //ScriptTools.removePausesAndZeroStepLists(scriptObjects);
      //ScriptTools.removeAllEndOfScripts(scriptObjects);

      System.out.println("\nAfter:");
      System.out.println(ScriptObject.getListInfo(scriptObjects));

      System.exit(-1);
   }
}
