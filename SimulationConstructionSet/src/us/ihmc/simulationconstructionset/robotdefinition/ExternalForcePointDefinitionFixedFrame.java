package us.ihmc.simulationconstructionset.robotdefinition;

import javax.vecmath.Vector3d;

public class ExternalForcePointDefinitionFixedFrame
{
   String name = null;

   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public Vector3d getOffset()
   {
      return offset;
   }

   public void setOffset(Vector3d offset)
   {
      this.offset = offset;
   }

   Vector3d offset = null;


   public String getXMLRepresentation()
   {
      String returnString = "";
      returnString += "<ExternalForcePoint>\n";
      returnString += "\t<Name>" + name + "</Name>\n";
      returnString += "\t<Offset>" + offset + "</Offset>\n";
      returnString += "</ExternalForcePoint>\n";

      return returnString;
   }

}
