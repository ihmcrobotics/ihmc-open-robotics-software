package us.ihmc.footstepPlanning.graphSearch.parameters;

import java.util.ArrayList;
import java.util.List;

public class FootstepPlannerParameterKeys
{
   private int indexCount = 0;

   private final List<FootstepPlannerParameterKey<?>> keys = new ArrayList<>();

   private String saveFileName;

   public FootstepPlannerParameterKeys(String saveFileName)
   {
      this.saveFileName = saveFileName;
   }

   public DoubleFootstepPlannerParameterKey addDoubleKey(String titleCasedName)
   {
      DoubleFootstepPlannerParameterKey key = new DoubleFootstepPlannerParameterKey(indexCount++, titleCasedName);
      keys.add(key);
      return key;
   }

   public IntegerFootstepPlannerParameterKey addIntegerKey(String titleCasedName)
   {
      IntegerFootstepPlannerParameterKey key = new IntegerFootstepPlannerParameterKey(indexCount++, titleCasedName);
      keys.add(key);
      return key;
   }

   public BooleanFootstepPlannerParameterKey addBooleanKey(String titleCasedName)
   {
      BooleanFootstepPlannerParameterKey key = new BooleanFootstepPlannerParameterKey(indexCount++, titleCasedName);
      keys.add(key);
      return key;
   }

   public List<FootstepPlannerParameterKey<?>> keys()
   {
      return keys;
   }

   public String getSaveFileName()
   {
      return saveFileName;
   }
}
