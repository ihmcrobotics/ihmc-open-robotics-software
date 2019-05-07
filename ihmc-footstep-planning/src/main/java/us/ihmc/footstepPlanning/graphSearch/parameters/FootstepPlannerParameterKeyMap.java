package us.ihmc.footstepPlanning.graphSearch.parameters;

import java.util.ArrayList;
import java.util.List;

public class FootstepPlannerParameterKeyMap
{
   private int idCount = 0;

//   private final TreeMap<Class, List<FootstepPlannerParameterKey<?>>> keyMap = new TreeMap<>();
   private final List<FootstepPlannerParameterKey<?>> keys = new ArrayList<>();

   private String saveFileName;

   public FootstepPlannerParameterKeyMap(String saveFileName)
   {
      this.saveFileName = saveFileName;
   }

   public DoubleFootstepPlannerParameterKey addDoubleKey(String titleCasedName)
   {
      DoubleFootstepPlannerParameterKey key = new DoubleFootstepPlannerParameterKey(idCount++, titleCasedName);
      keys.add(key);
      return key;
   }

   public IntegerFootstepPlannerParameterKey addIntegerKey(String titleCasedName)
   {
      IntegerFootstepPlannerParameterKey key = new IntegerFootstepPlannerParameterKey(idCount++, titleCasedName);
      keys.add(key);
      return key;
   }

   public BooleanFootstepPlannerParameterKey addBooleanKey(String titleCasedName)
   {
      BooleanFootstepPlannerParameterKey key = new BooleanFootstepPlannerParameterKey(idCount++, titleCasedName);
      keys.add(key);
      return key;
   }

//   private <T> FootstepPlannerParameterKey<T> addKey(Class<T> type, String titleCasedName)
//   {
//
//      FootstepPlannerParameterKey<T> key = new FootstepPlannerParameterKey<>(type, idCount++, titleCasedName);
//
////      if (!keyMap.containsKey(type))
////         keyMap.put(type, new ArrayList<>());
////
////      keyMap.get(type).add(key);
//
//      keyMap.add(key);
//
//      return key;
//   }

//   public TreeMap<Class, List<FootstepPlannerParameterKey<?>>> keys()
   public List<FootstepPlannerParameterKey<?>> keys()
   {
      return keys;
   }

   public String getSaveFileName()
   {
      return saveFileName;
   }
}
