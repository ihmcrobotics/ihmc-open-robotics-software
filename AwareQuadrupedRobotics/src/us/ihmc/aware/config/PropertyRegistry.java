package us.ihmc.aware.config;

import java.util.ArrayList;
import java.util.List;

public class PropertyRegistry
{
   private final List<Property> properties = new ArrayList<>();

   public PropertyRegistry()
   {

   }

   public void register(Property property)
   {
      properties.add(property);
   }

   public void save()
   {
      for (Property property : properties)
      {
         System.out.println(property.dump());
      }
   }
}
