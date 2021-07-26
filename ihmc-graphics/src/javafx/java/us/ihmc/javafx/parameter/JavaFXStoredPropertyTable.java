package us.ihmc.javafx.parameter;

import javafx.beans.Observable;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.control.TableView;
import us.ihmc.log.LogTools;
import us.ihmc.tools.property.*;

public class JavaFXStoredPropertyTable extends JavaFXParameterTable
{
   private StoredPropertySetBasics storedPropertySet;
   private Runnable parametersUpdated;

   public JavaFXStoredPropertyTable(TableView tableView)
   {
      super(tableView);
   }

   public void setup(StoredPropertySetBasics storedPropertySet, StoredPropertyKeyList keys, Runnable parametersUpdated)
   {
      this.storedPropertySet = storedPropertySet;
      this.parametersUpdated = parametersUpdated;

      for (StoredPropertyKey parameterKey : keys.keys())
      {
         SpinnerValueFactory spinnerValueFactory = null;

         if (parameterKey.getType().equals(Double.class)) // TODO: Guess have to store these too? Optionally override?
         {
            DoubleStoredPropertyKey doubleKey = (DoubleStoredPropertyKey) parameterKey;
            spinnerValueFactory = new SpinnerValueFactory.DoubleSpinnerValueFactory(-100.0, 100.0, storedPropertySet.get(doubleKey), 0.1);
         }
         else if (parameterKey.getType().equals(Integer.class))
         {
            IntegerStoredPropertyKey integerKey = (IntegerStoredPropertyKey) parameterKey;
            spinnerValueFactory = new SpinnerValueFactory.IntegerSpinnerValueFactory(-100, 200, storedPropertySet.get(integerKey), 1);
         }
         else if (parameterKey.getType().equals(Boolean.class))
         {
            BooleanStoredPropertyKey booleanKey = (BooleanStoredPropertyKey) parameterKey;
            spinnerValueFactory = new SpinnerValueFactory.IntegerSpinnerValueFactory(0, 1, storedPropertySet.get(booleanKey) ? 1 : 0, 1);
         }
         else
         {
            throw new RuntimeException("Please implement spinner for type: " + parameterKey.getType());
         }

         JavaFXParameterTableEntry javaFXParameterTableEntry = new JavaFXParameterTableEntry<>(parameterKey.getTitleCasedName(),
                                                                                               () -> getFromSet(parameterKey),
                                                                                               newValue -> setToSet(parameterKey, newValue),
                                                                                               observable -> onUserSpunSpinner(parameterKey, observable),
                                                                                               spinnerValueFactory);
         addEntry(javaFXParameterTableEntry);
      }

      updateEntries();
   }

   private Object getFromSet(StoredPropertyKey parameterKey)
   {
      Object accessedValue = storedPropertySet.get(parameterKey);
      LogTools.debug("Accessed {}: {}", parameterKey.getCamelCasedName(), accessedValue);
      return accessedValue;
   }

   private void setToSet(StoredPropertyKey parameterKey, Object newValue)
   {
      LogTools.debug("Setting stored value {}: {}", parameterKey.getCamelCasedName(), newValue);
      storedPropertySet.set(parameterKey, newValue);
   }

   private void onUserSpunSpinner(StoredPropertyKey parameterKey, Observable observable)
   {
      LogTools.debug("Observed: {}", parameterKey.getCamelCasedName());
      parametersUpdated.run();
   }
}
