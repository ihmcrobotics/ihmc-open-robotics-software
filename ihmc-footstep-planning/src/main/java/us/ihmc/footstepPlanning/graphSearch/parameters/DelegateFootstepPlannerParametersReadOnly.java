package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyReadOnly;

import java.util.List;

/**
 * Allows for easily swapping different parameter sets while not changing the API of existing code.
 */
public class DelegateFootstepPlannerParametersReadOnly implements FootstepPlannerParametersReadOnly
{
   private FootstepPlannerParametersReadOnly parameters;

   public void setParameters(FootstepPlannerParametersReadOnly parameters)
   {
      this.parameters = parameters;
   }

   public FootstepPlannerParametersReadOnly getParameters()
   {
      return parameters;
   }

   @Override
   public double get(DoubleStoredPropertyKey key)
   {
      return parameters.get(key);
   }

   @Override
   public int get(IntegerStoredPropertyKey key)
   {
      return parameters.get(key);
   }

   @Override
   public boolean get(BooleanStoredPropertyKey key)
   {
      return parameters.get(key);
   }

   @Override
   public <T> T get(StoredPropertyKey<T> key)
   {
      return parameters.get(key);
   }

   @Override
   public <T> StoredPropertyReadOnly<T> getProperty(StoredPropertyKey<T> key)
   {
      return parameters.getProperty(key);
   }

   @Override
   public List<Object> getAll()
   {
      return parameters.getAll();
   }

   @Override
   public List<String> getAllAsStrings()
   {
      return parameters.getAllAsStrings();
   }

   @Override
   public String getTitle()
   {
      return parameters.getTitle();
   }

   @Override
   public String getCurrentVersionSuffix()
   {
      return parameters.getCurrentVersionSuffix();
   }

   @Override
   public String getCapitalizedClassName()
   {
      return parameters.getCapitalizedClassName();
   }
}
