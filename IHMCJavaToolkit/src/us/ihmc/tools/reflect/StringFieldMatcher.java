package us.ihmc.tools.reflect;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Set;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.tools.string.StringAndRegularExpressionMatcher;

public class StringFieldMatcher
{
   private final LinkedHashMap<Class<?>, ArrayList<ImmutablePair<Field, StringAndRegularExpressionMatcher>>> fieldsToMatch = new LinkedHashMap<Class<?>, ArrayList<ImmutablePair<Field,StringAndRegularExpressionMatcher>>>();
   
      
   public StringFieldMatcher()
   {
      
   }
   
   public void addStringFieldToMatchRegularExpression(Class<?> type, Field field, String regularExpressionToMatch)
   {
      StringAndRegularExpressionMatcher matcher = new StringAndRegularExpressionMatcher();
      matcher.addRegularExpression(regularExpressionToMatch);
      
      addStringFieldToMatch(type, field, matcher);
   }
   
   public void addStringFieldToMatchExactly(Class<?> type, Field field, String stringToMatch)
   {
      addStringFieldToMatchExactly(type, field, stringToMatch, true);
   }
   
   public void addStringFieldToMatchExactly(Class<?> type, Field field, String stringToMatch, boolean caseSensitive)
   {
      StringAndRegularExpressionMatcher matcher = new StringAndRegularExpressionMatcher();
      matcher.addExactStringToMatch(stringToMatch);
      
      addStringFieldToMatch(type, field, matcher);
   }
  
   
   public void addStringFieldToMatch(Class<?> type, Field field, StringAndRegularExpressionMatcher matcher)
   {
      field.setAccessible(true);

      ArrayList<ImmutablePair<Field, StringAndRegularExpressionMatcher>> list = fieldsToMatch.get(type);
      if (list == null)
      {
         list = new ArrayList<ImmutablePair<Field,StringAndRegularExpressionMatcher>>();
         fieldsToMatch.put(type, list);
      }

      ImmutablePair<Field, StringAndRegularExpressionMatcher> pair = new ImmutablePair<Field, StringAndRegularExpressionMatcher>(field, matcher);
      list.add(pair);
   }

   public boolean matches(Object objectToTest) throws IllegalArgumentException, IllegalAccessException
   {
      ArrayList<ImmutablePair<Field, StringAndRegularExpressionMatcher>> pairs = fieldsToMatch.get(objectToTest.getClass());
      if (pairs == null) return false;
      
      for (ImmutablePair<Field, StringAndRegularExpressionMatcher> pair : pairs)
      {
         Field field = pair.getLeft();
         StringAndRegularExpressionMatcher matcher = pair.getRight();
                  
         String objectsFieldStringValue = (String) (field.get(objectToTest));
         if (matcher.matches(objectsFieldStringValue)) return true;
      }
      
     return false;
   }

   public void combine(StringFieldMatcher stringFieldMatcherToCombine)
   {
      Set<Class<?>> classesToCombine = stringFieldMatcherToCombine.fieldsToMatch.keySet();
      
      for (Class<?> type : classesToCombine)
      {
         ArrayList<ImmutablePair<Field, StringAndRegularExpressionMatcher>> pairs = stringFieldMatcherToCombine.fieldsToMatch.get(type);
         
         this.addStringFieldsToMatch(type, pairs);
      }
            
   }

   private void addStringFieldsToMatch(Class<?> type, ArrayList<ImmutablePair<Field, StringAndRegularExpressionMatcher>> pairs)
   {
      for (ImmutablePair<Field, StringAndRegularExpressionMatcher> pair : pairs)
      {
         Field field = pair.getLeft();
         StringAndRegularExpressionMatcher matcher = pair.getRight();
         
         this.addStringFieldToMatch(type, field,  matcher);
      }
   }
}
