package us.ihmc.parameterTuner.guiElements;

public abstract class GuiElement
{
   private final String name;
   private final GuiRegistry parent;
   private final String uniqueName;
   private final int hash;

   public GuiElement(String name, GuiRegistry parent)
   {
      this.name = name;
      this.parent = parent;
      this.uniqueName = createUniqueName();
      this.hash = createHashCode();
   }

   public String getName()
   {
      return name;
   }

   public String getUniqueName()
   {
      return uniqueName;
   }

   public GuiRegistry getParent()
   {
      return parent;
   }

   private String createUniqueName()
   {
      if (parent == null)
      {
         return name;
      }
      else
      {
         return parent.getUniqueName() + ":" + name;
      }
   }

   private int createHashCode()
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + ((uniqueName == null) ? 0 : uniqueName.hashCode());
      return result;
   }

   @Override
   public int hashCode()
   {
      return hash;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      GuiElement other = (GuiElement) obj;
      if (uniqueName == null)
      {
         if (other.uniqueName != null)
            return false;
      }
      else if (!uniqueName.equals(other.uniqueName))
         return false;
      return true;
   }
}
