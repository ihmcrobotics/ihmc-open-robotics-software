package us.ihmc.parameterTuner.guiElements;

public abstract class GuiElement
{
   private final String name;
   private final GuiRegistry parent;
   private final String uniqueName;

   public GuiElement(String name, GuiRegistry parent)
   {
      this(name, parent, createUniqueName(name, parent));
   }

   public GuiElement(String name, GuiRegistry parent, String uniqueName)
   {
      this.name = name;
      this.parent = parent;
      this.uniqueName = uniqueName;
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

   public static String createUniqueName(String name, GuiRegistry parent)
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
}
