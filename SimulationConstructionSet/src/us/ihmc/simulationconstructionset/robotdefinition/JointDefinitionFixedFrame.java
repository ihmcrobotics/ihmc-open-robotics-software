package us.ihmc.simulationconstructionset.robotdefinition;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.Plane;

public class JointDefinitionFixedFrame implements Comparable<JointDefinitionFixedFrame>
{
   public enum JointType {PIN_JOINT, SLIDER_JOINT, FLOATING_JOINT, FLOATING_PLANAR_JOINT}

   private String jointName;

   private JointDefinitionFixedFrame parentJoint = null;

   // temporary use for creating jointdefinitions from files
   private String parentName = null;

   public String getParentName()
   {
      return parentName;
   }

   public void setParentName(String parentName)
   {
      this.parentName = parentName;
   }

   private JointType type;
   private boolean rootJoint = false;
   private LinkDefinitionFixedFrame link;

   private Vector3d offset;
   private Vector3d jointAxis;

   private ArrayList<JointDefinitionFixedFrame> childrenJoints = new ArrayList<JointDefinitionFixedFrame>();

   // only used for planar joints.
   private Plane planarType = Plane.YZ;

   private ArrayList<GroundContactDefinitionFixedFrame> groundContactDefinitionsFixedFrame = new ArrayList<GroundContactDefinitionFixedFrame>();
   private ArrayList<ExternalForcePointDefinitionFixedFrame> externalForcePointDefinitionsFixedFrame = new ArrayList<ExternalForcePointDefinitionFixedFrame>();

   public Plane getPlanarType()
   {
      return planarType;
   }

   public void setRootJoint(boolean rootJoint)
   {
      this.rootJoint = rootJoint;
   }

   public boolean isRootJoint()
   {
      return rootJoint;
   }


   public ArrayList<GroundContactDefinitionFixedFrame> getGroundContactDefinitionsFixedFrame()
   {
      return groundContactDefinitionsFixedFrame;
   }

   public void addGroundContactDefinitionFixedFrame(GroundContactDefinitionFixedFrame groundContactDefinitionFixedFrame)
   {
      groundContactDefinitionsFixedFrame.add(groundContactDefinitionFixedFrame);
   }


   public ArrayList<ExternalForcePointDefinitionFixedFrame> getExternalForcePointDefinitionsFixedFrame()
   {
      return externalForcePointDefinitionsFixedFrame;
   }

   public void addExternalForcePointDefinition(ExternalForcePointDefinitionFixedFrame externalForcePointDefinitionFixedFrame)
   {
      externalForcePointDefinitionsFixedFrame.add(externalForcePointDefinitionFixedFrame);
   }


   public void setPlanarType(Plane type)
   {
      this.planarType = type;
   }

   public int getNumberOfChildJoints()
   {
      int numberOfJoints = childrenJoints.size();
      for (JointDefinitionFixedFrame joint : childrenJoints)
      {
         numberOfJoints += joint.getNumberOfChildJoints();
      }

      return numberOfJoints;
   }


   public String getJointName()
   {
      return jointName;
   }

   public void setJointName(String jointName)
   {
      this.jointName = jointName;
   }

   public JointDefinitionFixedFrame getParentJoint()
   {
      return parentJoint;
   }

   public void setParentJoint(JointDefinitionFixedFrame parentJoint)
   {
      this.parentJoint = parentJoint;
   }

   public JointType getType()
   {
      return type;
   }

   public void setType(JointType type)
   {
      this.type = type;
   }

   public LinkDefinitionFixedFrame getLinkDefinition()
   {
      return link;
   }

   public void setLinkDefinition(LinkDefinitionFixedFrame link)
   {
      this.link = link;
   }

   public Vector3d getOffset()
   {
      return offset;
   }

   public void setOffset(Vector3d offset)
   {
      this.offset = offset;
   }

   public Vector3d getJointAxis()
   {
      return jointAxis;
   }

   public void setJointAxis(Vector3d jointAxis)
   {
      this.jointAxis = jointAxis;
   }

   public ArrayList<JointDefinitionFixedFrame> getChildrenJoints()
   {
      return childrenJoints;
   }

   public void addChildJoint(JointDefinitionFixedFrame jointDef)
   {
      childrenJoints.add(jointDef);
   }


   @Override
   public String toString()
   {
      String returnString = "";

//    if (parentJoint == null)
//       returnString += "Found Root Joint.\n";


      returnString += "<Joint>\n";
      returnString += "\t<Name>" + jointName + "</Name>\n";

      returnString += "\t<Parent>";
      if (parentJoint != null)
         returnString += parentJoint.getJointName();
      else
         returnString += "null";
      returnString += "</Parent>\n";



      returnString += "\t<RootJoint>" + rootJoint + "</RootJoint>\n";


//    returnString += "Joint name = " + jointName + "\n";



      returnString += "\t<Offset>" + offset + "</Offset>\n";

//    returnString += "Joint offset = " + offset + "\n";


      returnString += "\t<Axis>" + jointAxis + "</Axis>\n";

//    returnString += "Joint axis = " + jointAxis + "\n";

      returnString += "\t<Type>" + type + "</Type>\n";

      if (type == JointType.PIN_JOINT)
      {
//       returnString += "Joint is a Pin Joint.\n";
//       returnString += "Its q variable is named q_" + jointName + "\n";
      }

      else if (type == JointType.SLIDER_JOINT)
      {
//       returnString += "Joint is a Slider Joint.\n";

//       returnString += "Its q variable is named q_" + jointName + "\n";
      }

      else if (type == JointType.FLOATING_JOINT)
      {
//       returnString += "Joint is a Floating Joint.\n";
      }

      else if (type == JointType.FLOATING_PLANAR_JOINT)
      {
//       returnString += "Joint is a Floating Planar Joint.\n";
      }

      else
      {
         throw new RuntimeException("Only Pin, Slider, Floating, and Floating Planar joints implemented right now");
      }


//    for (GroundContactDefinitionFixedFrame gcDefinition : groundContactDefinitionsFixedFrame)
//    {
//       returnString += gcDefinition.toString();
//    }

      returnString += getGroundContactPointsString();
      returnString += getExternalForcePointsString();
      returnString += getLinkDefinition();
      returnString += "</Joint>\n";

      return returnString;
   }

   private String getGroundContactPointsString()
   {
      String returnString = "<GroundContactPoints>\n";
      for (GroundContactDefinitionFixedFrame gcDef : groundContactDefinitionsFixedFrame)
      {
         returnString += gcDef.getXMLRepresentation();
      }

      returnString += "</GroundContactPoints>\n";

      return returnString;
   }

   private String getExternalForcePointsString()
   {
      String returnString = "<ExternalForcePoints>\n";
      for (ExternalForcePointDefinitionFixedFrame efpDef : externalForcePointDefinitionsFixedFrame)
      {
         returnString += efpDef.getXMLRepresentation();
      }

      returnString += "</ExternalForcePoints>\n";

      return returnString;
   }

   @Override
   public int compareTo(JointDefinitionFixedFrame o)
   {
      return o.getJointName().compareTo(getJointName());
   }

}
