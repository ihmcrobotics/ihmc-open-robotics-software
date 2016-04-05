package us.ihmc.aware.config;

import java.util.HashSet;
import java.util.Set;

public class DynamicPropertyRegistry
{
   private final Set<DynamicProperty> properties = new HashSet<>();

   public DynamicPropertyRegistry()
   {

   }

   public void register(DynamicProperty property)
   {
      properties.add(property);
   }

   public void save()
   {
      for (DynamicProperty property : properties)
      {
         System.out.println(property.dump());
      }
   }
}
