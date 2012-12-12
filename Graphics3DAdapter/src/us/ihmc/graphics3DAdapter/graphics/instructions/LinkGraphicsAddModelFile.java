package us.ihmc.graphics3DAdapter.graphics.instructions;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceDefinition;

public class LinkGraphicsAddModelFile extends LinkGraphicsInstruction
{
   private String fileName;

   public LinkGraphicsAddModelFile(String fileName)
   {
      this(fileName, null);
   }

   public LinkGraphicsAddModelFile(String fileName, YoAppearanceDefinition appearance)
   {
      this.fileName = fileName;
      setAppearance(appearance);
   }

   public String getFileName()
   {
      return fileName;
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
