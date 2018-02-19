package us.ihmc.parameterTuner.guiElements.main;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;

import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.ParameterChangeListener;

public class ChangeCollector implements ParameterChangeListener
{
   private final ConcurrentHashMap<String, GuiParameter> changedParameters = new ConcurrentHashMap<>();
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
      List<GuiParameter> list = new ArrayList<>(changedParameters.values());
      changedParameters.clear();
      return list;
   }

   public void stopRecording()
   {
      recordingChange = false;
   }

   public void startRecording()
   {
      recordingChange = true;
   }
}
