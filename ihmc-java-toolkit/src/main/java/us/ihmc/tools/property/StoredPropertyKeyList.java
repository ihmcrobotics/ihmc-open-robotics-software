package us.ihmc.tools.property;

import com.sun.xml.internal.ws.util.StringUtils;

import java.util.ArrayList;
import java.util.List;

public class StoredPropertyKeyList
{
   private int indexCount = 0;

   private final List<StoredPropertyKey<?>> keys = new ArrayList<>();

   private String saveFileName;

   public StoredPropertyKeyList(Class<?> classToNameAfter)
   {
      this(StringUtils.decapitalize(classToNameAfter.getSimpleName()));
   }

   public StoredPropertyKeyList(String saveFileName)
   {
      this.saveFileName = saveFileName;
   }

   public DoubleStoredPropertyKey addDoubleKey(String titleCasedName)
   {
      DoubleStoredPropertyKey key = new DoubleStoredPropertyKey(indexCount++, titleCasedName);
      keys.add(key);
      return key;
   }

   public IntegerStoredPropertyKey addIntegerKey(String titleCasedName)
   {
      IntegerStoredPropertyKey key = new IntegerStoredPropertyKey(indexCount++, titleCasedName);
      keys.add(key);
      return key;
   }

   public BooleanStoredPropertyKey addBooleanKey(String titleCasedName)
   {
      BooleanStoredPropertyKey key = new BooleanStoredPropertyKey(indexCount++, titleCasedName);
      keys.add(key);
      return key;
   }

   public List<StoredPropertyKey<?>> keys()
   {
      return keys;
   }

   public String getSaveFileName()
   {
      return saveFileName;
   }
}
