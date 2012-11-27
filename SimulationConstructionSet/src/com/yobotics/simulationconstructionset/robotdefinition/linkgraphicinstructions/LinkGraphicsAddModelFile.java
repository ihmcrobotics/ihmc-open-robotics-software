package com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions;

import com.yobotics.simulationconstructionset.graphics.YoAppearanceDefinition;

public class LinkGraphicsAddModelFile implements LinkGraphicsInstruction
{
   private String fileName;
   private YoAppearanceDefinition appearance = null;

   public LinkGraphicsAddModelFile(String fileName)
   {
      this(fileName, null);
   }

   public LinkGraphicsAddModelFile(String fileName, YoAppearanceDefinition appearence)
   {
      this.fileName = fileName;
      this.appearance = appearence;
   }

   public String getFileName()
   {
      return fileName;
   }

   public YoAppearanceDefinition getAppearance()
   {
      return appearance;
   }

   public String toString()
   {
	   String ret = "\t\t\t<Add3DSFile>\n\t\t\t\t<Name>"+fileName+"</Name>\n";
	   if (appearance != null) {
		   ret += appearance;
	   }
	   ret += "\t\t\t</Add3DSFile>\n";
	   return ret;
   }
}
