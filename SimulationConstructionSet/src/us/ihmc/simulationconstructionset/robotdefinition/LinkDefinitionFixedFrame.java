package us.ihmc.simulationconstructionset.robotdefinition;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.tools.io.xml.XMLReaderUtility;

public class LinkDefinitionFixedFrame
{
   private String name;
   private double mass;
   private Vector3d comOffset = new Vector3d();
   private Matrix3d momentOfInertia = new Matrix3d();
   private Graphics3DObject graphicsDefinition = new Graphics3DObject();


   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public double getMass()
   {
      return mass;
   }

   public void setMass(double mass)
   {
      this.mass = mass;
   }

   public Vector3d getComOffset()
   {
      return comOffset;
   }

   public void setComOffset(Vector3d comOffset)
   {
      this.comOffset = comOffset;
   }

   public Matrix3d getInertia()
   {
      return momentOfInertia;
   }

   public void setInertia(Matrix3d inertia)
   {
      momentOfInertia = inertia;
   }

   public Graphics3DObject getGraphicsDefinition()
   {
      return graphicsDefinition;
   }

   public void setLinkGraphics(Graphics3DObject graphicsDefinition)
   {
      this.graphicsDefinition = graphicsDefinition;
   }

   public String toString()
   {
      String returnString = "\t<Link>\n";

      returnString += "\t\t<Mass>" + mass + "</Mass>\n";
      //      returnString += "Mass = " + mass + "\n";

      returnString += "\t\t<ComOffset>" + comOffset + "</ComOffset>\n";
      //      returnString += "comOffset = " + comOffset + "\n";

      returnString += "\t\t<MomentOfInertia>" + XMLReaderUtility.matrix3DToString(momentOfInertia) + "</MomentOfInertia>\n";
      //      returnString += "momentOfInertia = \n" + momentOfInertia + "\n";

      returnString += "\t\t<Graphics>\n";
      //      returnString +="***************************** GRAPHICS START************************************\n";
      if (graphicsDefinition != null)
      {
         for(Graphics3DPrimitiveInstruction instruction: graphicsDefinition.getGraphics3DInstructions())
         {
            returnString += instruction;
         }
      }
      returnString += "\t\t</Graphics>\n";
      //      returnString +="***************************** GRAPHICS END************************************\n";

      returnString += "\t</Link>\n";

      return returnString;
   }


}
