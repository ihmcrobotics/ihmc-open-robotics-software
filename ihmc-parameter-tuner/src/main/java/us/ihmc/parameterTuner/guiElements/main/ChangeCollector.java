package us.ihmc.parameterTuner.guiElements.main;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.ParameterChangeListener;

public class ChangeCollector implements ParameterChangeListener
{
   private final Queue<GuiParameter> changedParameters = new ConcurrentLinkedQueue<>();
   private final Map<String, String> pendingChanges = new ConcurrentHashMap<>();

   private boolean recordingChange = true;

   @Override
   public void changed(GuiParameter parameter)
   {
      // this is called on parameter changes (they happen in the Plattform.runLater(Runnable) thread)
      if (recordingChange)
      {
         changedParameters.add(new GuiParameter(parameter));
      }
   }

   public List<GuiParameter> getChangedParametersAndClear()
   {
      // this is called by the animation timer
      GuiParameter userChangedParameter;
      Map<String, GuiParameter> parametersToUpdate = new HashMap<>();
      while ((userChangedParameter = changedParameters.poll()) != null)
      {
         parametersToUpdate.put(userChangedParameter.getUniqueName(), userChangedParameter);
      }

      for (GuiParameter changedParameter : parametersToUpdate.values())
      {
         String uniqueName = changedParameter.getUniqueName();
         pendingChanges.put(uniqueName, changedParameter.getCurrentValue());
      }

      return new ArrayList<>(parametersToUpdate.values());
   }

   public void stopRecording()
   {
      recordingChange = false;
   }

   public void startRecording()
   {
      recordingChange = true;
   }

   public boolean isPending(String uniqueName)
   {
      return pendingChanges.containsKey(uniqueName);
   }

   public void parameterWasUpdated(String uniqueName, String newValue)
   {
      String expectedValue = pendingChanges.get(uniqueName);
      if (newValue.equals(expectedValue))
      {
         pendingChanges.remove(uniqueName);
      }
   }
}
