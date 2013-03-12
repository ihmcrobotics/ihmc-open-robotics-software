package us.ihmc.graphics3DAdapter.graphics.instructions;

import java.util.ArrayList;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;

public class Graphics3DAddModelFileInstruction extends Graphics3DInstruction
{
   private String fileName;
   private ArrayList<String> resourceDirectories = new ArrayList<String>();

   public Graphics3DAddModelFileInstruction(String fileName)
   {
      this(fileName, null);
   }
   
   public Graphics3DAddModelFileInstruction(String fileName, AppearanceDefinition appearance, ArrayList<String> resourceDirectories)
   {
      this.fileName = fileName;
      setAppearance(appearance);
      this.resourceDirectories.addAll(resourceDirectories);
   }

   public Graphics3DAddModelFileInstruction(String fileName, AppearanceDefinition appearance)
   {
      this.fileName = fileName;
      setAppearance(appearance);
   }

   public String getFileName()
   {
      return fileName;
   }

   public ArrayList<String> getResourceDirectories()
   {
      return resourceDirectories;
   }

   public String toString()
   {
	   String ret = "\t\t\t<Add3DSFile>\n\t\t\t\t<Name>"+fileName+"</Name>\n";
	   if (getAppearance() != null)
         ret += getAppearance().toString();
	   ret += "\t\t\t</Add3DSFile>\n";
	   return ret;
   }
}
