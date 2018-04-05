package us.ihmc.parameterTuner.guiElements.main;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.ParameterChangeListener;

public class ChangeCollector implements ParameterChangeListener
{
   private final Map<String, GuiParameter> changedParameters = new ConcurrentHashMap<>();
   private final Map<String, String> pendingChanges = new ConcurrentHashMap<>();

   private boolean recordingChange = true;

   @Override
   public void changed(GuiParameter parameter)
   {
      // this is called on parameter changes (they happen in the Plattform.runLater(Runnable) thread)
      if (recordingChange)
      {
         changedParameters.put(parameter.getUniqueName(), new GuiParameter(parameter));
      }
   }

   public List<GuiParameter> getChangedParametersAndClear()
   {
      // this is called by the animation timer
      List<GuiParameter> outgoingChanges = new ArrayList<>();
      for (GuiParameter changedParameter : changedParameters.values())
      {
         String uniqueName = changedParameter.getUniqueName();
         if (!isPending(uniqueName))
         {
            outgoingChanges.add(changedParameter);
            pendingChanges.put(uniqueName, changedParameter.getCurrentValue());
            changedParameters.remove(uniqueName);
         }
      }
      return outgoingChanges;
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
