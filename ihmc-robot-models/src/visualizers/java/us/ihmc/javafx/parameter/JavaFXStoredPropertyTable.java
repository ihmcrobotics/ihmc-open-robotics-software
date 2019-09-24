package us.ihmc.javafx.parameter;

import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.control.TableView;
import us.ihmc.log.LogTools;
import us.ihmc.tools.property.*;

public class JavaFXStoredPropertyTable extends JavaFXParameterTable
{
   public JavaFXStoredPropertyTable(TableView tableView)
   {
      super(tableView);
   }

   public void setup(StoredPropertySet storedPropertySet, StoredPropertyKeyList keys, Runnable parametersUpdated)
   {
      for (StoredPropertyKey parameterKey : keys.keys())
      {
         SpinnerValueFactory spinnerValueFactory = null;

         if (parameterKey.getType().equals(Double.class)) // TODO: Guess have to store these too? Optionally override?
         {
            spinnerValueFactory = new SpinnerValueFactory.DoubleSpinnerValueFactory(-100.0, 100.0, storedPropertySet.get((DoubleStoredPropertyKey) parameterKey), 0.1);
         }
         else if (parameterKey.getType().equals(Integer.class))
         {
            spinnerValueFactory = new SpinnerValueFactory.IntegerSpinnerValueFactory(-100, 100, storedPropertySet.get((IntegerStoredPropertyKey) parameterKey), 1);
         }

         JavaFXParameterTableEntry javaFXParameterTableEntry = new JavaFXParameterTableEntry<>(
               parameterKey.getTitleCasedName(),
               () ->
               {
                  Object accessedValue = storedPropertySet.get(parameterKey);
                  LogTools.debug("Accessed {}: {}", parameterKey.getCamelCasedName(), accessedValue);
                  return accessedValue;
               },
               newValue ->
               {
                  LogTools.debug("Setting stored value {}: {}", parameterKey.getCamelCasedName(), newValue);
                  storedPropertySet.set(parameterKey, newValue);
               },
               observable ->
               {
                  LogTools.debug("Observed: {}", parameterKey.getCamelCasedName());
                  parametersUpdated.run();
               },
               spinnerValueFactory);
         addEntry(javaFXParameterTableEntry);
      }

      updateEntries();
   }
}
