package com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions;

import com.yobotics.simulationconstructionset.robotdefinition.AppearanceDefinition;

public class LinkGraphicsAddModelFile implements LinkGraphicsInstruction
{
   private String fileName;
   private AppearanceDefinition appearance = null;

   public LinkGraphicsAddModelFile(String fileName)
   {
      this(fileName, null);
   }

   public LinkGraphicsAddModelFile(String fileName, AppearanceDefinition appearence)
   {
      this.fileName = fileName;
      this.appearance = appearence;
   }

   public String getFileName()
   {
      return fileName;
   }

   public AppearanceDefinition getAppearance()
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
