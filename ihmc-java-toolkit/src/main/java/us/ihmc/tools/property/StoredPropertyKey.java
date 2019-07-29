package us.ihmc.tools.property;

import javafx.beans.InvalidationListener;
import javafx.beans.Observable;
import org.apache.commons.lang3.ClassUtils;
import org.apache.commons.lang3.StringUtils;

import java.util.ArrayList;
import java.util.List;

public class StoredPropertyKey<T> implements Observable
{
   private final String titleCasedName;
   private final String saveName;
   private final Class<T> type;
   private final int index;
   private final Object defaultValue;

   private final List<InvalidationListener> listeners = new ArrayList<>();

   public StoredPropertyKey(Class<T> type, int index, String titleCasedName)
   {
      this(type, index, titleCasedName, null);
   }

   public StoredPropertyKey(Class<T> type, int index, String titleCasedName, Object defaultValue)
   {
      if (!ClassUtils.isPrimitiveOrWrapper(type))
      {
         throw new RuntimeException("Type must be primitive!");
      }

      this.type = type;
      this.index = index;
      this.titleCasedName = titleCasedName;
      this.defaultValue = defaultValue;

      saveName = buildCamelCasedName();
   }

   public String getTitleCasedName()
   {
      return titleCasedName;
   }

   public int getIndex()
   {
      return index;
   }

   public Class<T> getType()
   {
      return type;
   }

   public void addListener(InvalidationListener var1)
   {
      this.listeners.add(var1);
   }

   public void removeListener(InvalidationListener var1)
   {
      this.listeners.remove(var1);
   }

   public void notifyOfVariableChanged()
   {
      for (int i = 0; i < listeners.size(); i++)
      {
         listeners.get(i).invalidated(this);
      }
   }

   public Object getDefaultValue()
   {
      return defaultValue;
   }

   public boolean hasDefaultValue()
   {
      return defaultValue != null;
   }

   public String getCamelCasedName()
   {
      return saveName;
   }

   private String buildCamelCasedName() // to lower camel case
   {
      String[] splitBySpaces = titleCasedName.split(" ");

      String saveName = "";
      for (String splitBySpace : splitBySpaces)
      {
         saveName += StringUtils.capitalize(splitBySpace);
      }
      saveName = StringUtils.uncapitalize(saveName);

      return saveName;
   }
}