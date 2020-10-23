package us.ihmc.footstepPlanning.log;

import us.ihmc.yoVariables.variable.YoVariableType;

import java.util.Objects;

public class VariableDescriptor
{
   private final String name;
   private final YoVariableType type;
   private final String registryName;
   private final String[] enumValues;

   public VariableDescriptor(String name, YoVariableType type, String registryName)
   {
      this(name, type, registryName, null);
   }

   public VariableDescriptor(String name, YoVariableType type, String registryName, String[] enumValues)
   {
      this.name = name;
      this.type = type;
      this.registryName = registryName;
      this.enumValues = enumValues;
   }

   public String getName()
   {
      return name;
   }

   public YoVariableType getType()
   {
      return type;
   }

   public String getRegistryName()
   {
      return registryName;
   }

   public String[] getEnumValues()
   {
      return enumValues;
   }

   @Override
   public String toString()
   {
      return name;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
      {
         return false;
      }
      else if (!(other instanceof VariableDescriptor))
      {
         return false;
      }

      VariableDescriptor otherDescriptor = (VariableDescriptor) other;

      // for now don't worry about namespace TODO
      if (!otherDescriptor.name.equals(this.name))
      {
         return false;
      }
      else if (otherDescriptor.type != this.type)
      {
         return false;
      }

      return true;
   }

   @Override
   public int hashCode()
   {
      return Objects.hash(name, type);
   }
}
