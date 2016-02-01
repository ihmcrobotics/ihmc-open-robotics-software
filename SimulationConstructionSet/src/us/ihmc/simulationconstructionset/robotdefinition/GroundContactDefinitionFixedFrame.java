package us.ihmc.simulationconstructionset.robotdefinition;

import javax.vecmath.Vector3d;

public class GroundContactDefinitionFixedFrame
{
   String name = null;

   Vector3d offset = null;


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

   public String getXMLRepresentation()
   {
      String returnString = "";
      returnString += "<GroundContactPoint>\n";
      returnString += "\t<Name>" + name + "</Name>\n";
      returnString += "\t<Offset>" + offset + "</Offset>\n";

      returnString += "</GroundContactPoint>\n";

      return returnString;
   }

}
