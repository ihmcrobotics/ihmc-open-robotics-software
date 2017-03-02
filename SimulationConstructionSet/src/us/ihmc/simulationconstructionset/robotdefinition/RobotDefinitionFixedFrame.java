package us.ihmc.simulationconstructionset.robotdefinition;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.zip.GZIPInputStream;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddModelFileInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DIdentityInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DTranslateInstruction;
import us.ihmc.simulationconstructionset.DummyOneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.simulationconstructionset.robotdefinition.JointDefinitionFixedFrame.JointType;
import us.ihmc.tools.io.xml.XMLReaderUtility;

public class RobotDefinitionFixedFrame
{
   private final ArrayList<JointDefinitionFixedFrame> rootJointDefinitions = new ArrayList<JointDefinitionFixedFrame>();
   private String robotName = "defaultName";

   // used when recreating a definition from a file.


   public void createRobotDefinitionFromRobot(Robot r)
   {
      ArrayList<Joint> rootJoints = r.getRootJoints();
      for (Joint rootJoint : rootJoints)
      {
         rootJointDefinitions.add(createJointDefinition(rootJoint, null));
      }
   }

   public String getRobotName()
   {
      return robotName;
   }

   public void createRobotDefinitionFromRobotConfigurationFile(File file)
   {
      BufferedReader reader;
      try
      {
         if (file.getName().endsWith(".gz"))
         {
            reader = new BufferedReader(new InputStreamReader(new GZIPInputStream(new BufferedInputStream(new FileInputStream(file)))));
         }
         else
         {
            reader = new BufferedReader(new FileReader(file));
         }

         String xmlRepresentation = "";
         String tempLine;

         while ((tempLine = reader.readLine()) != null)
         {
            System.out.println(tempLine);
            xmlRepresentation += tempLine;


            if (tempLine.contains("$MODEL"))
            {
               robotName = tempLine.substring(6).trim();
               System.out.println(robotName);
            }
            else if (tempLine.contains("$END_HEADER"))
            {
               break;
            }

            else if (tempLine.contains("</RobotDefinition>"))
               break;
         }

         if (xmlRepresentation.startsWith("$<"))
         {
            xmlRepresentation = replaceAll(xmlRepresentation, "\n$", "\n");

         }

         createRobotDefinitionFromRobotConfigurationString(xmlRepresentation);
      }
      catch (IOException ioException)
      {
         ioException.printStackTrace();
      }
   }

   private String replaceAll(String orig, String regex, String rep)
   {
      int index = 0;
      while (index + regex.length() < orig.length())
      {
         if (orig.substring(index, index + regex.length()).equals(regex))
         {
            orig = replace(orig, index, index + regex.length(), rep);
            index += rep.length() - 1;
         }

         index++;
      }

      return orig;
   }

   private String replace(String fullString, int beginIndex, int endIndex, String replacement)
   {
      if ((beginIndex >= 0) && (beginIndex < fullString.length()) && (endIndex >= 0) && (endIndex < fullString.length()))
      {
         String prefix = fullString.substring(0, beginIndex);
         String suffix = fullString.substring(endIndex);

         return prefix + replacement + suffix;
      }
      else
      {
         return null;
      }
   }

   public void createRobotDefinitionFromRobotConfigurationString(String xmlString)
   {
      ArrayList<JointDefinitionFixedFrame> allJoints = new ArrayList<JointDefinitionFixedFrame>();
      String jointString = "";
      int currentJointIndex = 0;
      while ((jointString = XMLReaderUtility.getMiddleString(currentJointIndex, xmlString, "<Joint>", "</Joint>")) != null)
      {
         currentJointIndex = XMLReaderUtility.getEndIndexOfSubString(currentJointIndex, xmlString, "</Joint>");



         String type = XMLReaderUtility.getMiddleString(0, jointString, "<Type>", "</Type>");
         String name = XMLReaderUtility.getMiddleString(0, jointString, "<Name>", "</Name>");
         String parent = XMLReaderUtility.getMiddleString(0, jointString, "<Parent>", "</Parent>");
         String rootJoint = XMLReaderUtility.getMiddleString(0, jointString, "<RootJoint>", "</RootJoint>");
         Vector3D offset = XMLReaderUtility.parseVector3d(XMLReaderUtility.getMiddleString(0, jointString, "<Offset>", "</Offset>"));
         Vector3D axis = XMLReaderUtility.parseVector3d(XMLReaderUtility.getMiddleString(0, jointString, "<Axis>", "</Axis>"));


         JointDefinitionFixedFrame joint = new JointDefinitionFixedFrame();
         if (type.equals("FLOATING_JOINT"))
         {
            joint.setType(JointType.FLOATING_JOINT);

         }
         else if (type.equals("PIN_JOINT"))
         {
            joint.setType(JointType.PIN_JOINT);

         }
         else if (type.equals("SLIDER_JOINT"))
         {
            joint.setType(JointType.SLIDER_JOINT);

         }
         else if (type.equals("FLOATING_PLANAR_JOINT"))
         {
            joint.setType(JointType.FLOATING_PLANAR_JOINT);

         }
         else
         {
            XMLReaderUtility.displayErrorMessage("Unknown type: " + type);
         }

         joint.setJointName(name);
         joint.setRootJoint(rootJoint.equals("true"));

         if (!joint.isRootJoint())
         {
            joint.setParentName(parent);

         }

         joint.setOffset(offset);

         joint.setJointAxis(axis);


         String GcString = XMLReaderUtility.getMiddleString(0, jointString, "<GroundContactPoints>", "</GroundContactPoints>");


         createGroundContactPointDefinitionsFromString(GcString, joint);

         String efpString = XMLReaderUtility.getMiddleString(0, jointString, "<ExternalForcePoints>", "</ExternalForcePoints>");

         createExternalForcePointDefinitionsFromString(efpString, joint);

         String link = XMLReaderUtility.getMiddleString(0, jointString, "<Link>", "</Link>");
         LinkDefinitionFixedFrame linkDefinition = createLinkDefinitionFromFile(link);
         joint.setLinkDefinition(linkDefinition);

         if (joint.isRootJoint())
            rootJointDefinitions.add(joint);
         allJoints.add(joint);

      }

      setParentsAndChildren(allJoints);
   }

   private void createGroundContactPointDefinitionsFromString(String xmlString, JointDefinitionFixedFrame jointToAddTo)
   {
      String jointString = "";
      int currentGroundContactPointIndex = 0;
      if (xmlString != null)
      {
         while ((jointString = XMLReaderUtility.getMiddleString(currentGroundContactPointIndex, xmlString, "<GroundContactPoint>", "</GroundContactPoint>"))
                != null)
         {
            currentGroundContactPointIndex = XMLReaderUtility.getEndIndexOfSubString(currentGroundContactPointIndex, xmlString, "</GroundContactPoint>");
            String name = XMLReaderUtility.getMiddleString(0, jointString, "<Name>", "</Name>");
            Vector3D offset = XMLReaderUtility.parseVector3d(XMLReaderUtility.getMiddleString(0, jointString, "<Offset>", "</Offset>"));

            GroundContactDefinitionFixedFrame gc = new GroundContactDefinitionFixedFrame();
            gc.setName(name);
            gc.setOffset(offset);
            jointToAddTo.addGroundContactDefinitionFixedFrame(gc);
         }
      }
   }

   private void createExternalForcePointDefinitionsFromString(String xmlString, JointDefinitionFixedFrame jointToAddTo)
   {
      String jointString = "";
      int currentExternalForcePointIndex = 0;
      if (xmlString != null)
      {
         while ((jointString = XMLReaderUtility.getMiddleString(currentExternalForcePointIndex, xmlString, "<ExternalForcePoint>", "</ExternalForcePoint>"))
                != null)
         {
            currentExternalForcePointIndex = XMLReaderUtility.getEndIndexOfSubString(currentExternalForcePointIndex, xmlString, "</ExternalForcePoint>");
            String name = XMLReaderUtility.getMiddleString(0, jointString, "<Name>", "</Name>");
            Vector3D offset = XMLReaderUtility.parseVector3d(XMLReaderUtility.getMiddleString(0, jointString, "<Offset>", "</Offset>"));

            ExternalForcePointDefinitionFixedFrame efp = new ExternalForcePointDefinitionFixedFrame();
            efp.setName(name);
            efp.setOffset(offset);
            jointToAddTo.addExternalForcePointDefinition(efp);
         }
      }
   }


   private void setParentsAndChildren(ArrayList<JointDefinitionFixedFrame> allJoints)
   {
      for (JointDefinitionFixedFrame currentJoint : allJoints)
      {
         if (!currentJoint.isRootJoint())
         {
            JointDefinitionFixedFrame parent = getJointFromListByName(currentJoint.getParentName(), allJoints);
            parent.addChildJoint(currentJoint);
            currentJoint.setParentJoint(parent);
         }

      }
   }

   private JointDefinitionFixedFrame getJointFromListByName(String name, ArrayList<JointDefinitionFixedFrame> allJoints)
   {
      for (JointDefinitionFixedFrame joint : allJoints)
      {
         if (joint.getJointName().equals(name))
            return joint;
      }

      return null;
   }

   public LinkDefinitionFixedFrame createLinkDefinitionFromFile(String linkString)
   {
      LinkDefinitionFixedFrame l = new LinkDefinitionFixedFrame();
      double mass = XMLReaderUtility.parseDoubleBetweenTwoStrings(0, linkString, "<Mass>", "</Mass>");
      Vector3D comOffset = XMLReaderUtility.parseVector3d(XMLReaderUtility.getMiddleString(0, linkString, "<ComOffset>", "</ComOffset>"));
      Matrix3D inertia = XMLReaderUtility.parseMatrix3d(XMLReaderUtility.getMiddleString(0, linkString, "<MomentOfInertia>", "</MomentOfInertia>"));
      l.setMass(mass);
      l.setComOffset(comOffset);
      l.setInertia(inertia);
      l.setLinkGraphics(createLinkGraphicsDefinitionFromFile(XMLReaderUtility.getMiddleString(0, linkString, "<Graphics>", "</Graphics>")));

      return l;
   }

   public Graphics3DObject createLinkGraphicsDefinitionFromFile(String graphicsString)
   {
      Graphics3DObject def = new Graphics3DObject();
      String type = "";
      int currentTypeIndex = 0;
      while ((type = XMLReaderUtility.getMiddleString(currentTypeIndex, graphicsString, "<", ">")) != null)
      {
         String data = null;
         if (type.equals("Add3DSFile"))
         {
            data = XMLReaderUtility.getMiddleString(currentTypeIndex, graphicsString, "<Add3DSFile>", "</Add3DSFile>");
            String name = XMLReaderUtility.getMiddleString(0, data, "<Name>", "</Name>");
            Graphics3DAddModelFileInstruction t = new Graphics3DAddModelFileInstruction(name);
            def.addInstruction(t);

//          System.out.println("Add3DSFile");
            currentTypeIndex = XMLReaderUtility.getEndIndexOfSubString(currentTypeIndex, graphicsString, "</Add3DSFile>");
         }
         else if (type.equals("Identity"))
         {
            def.addInstruction(new Graphics3DIdentityInstruction());

//          System.out.println("Identity");
            currentTypeIndex = XMLReaderUtility.getEndIndexOfSubString(currentTypeIndex, graphicsString, "<Identity>");
         }
         else if (type.equals("Translate"))
         {
            data = XMLReaderUtility.getMiddleString(currentTypeIndex, graphicsString, "<Translate>", "</Translate>");
            Vector3D translation = XMLReaderUtility.parseVector3d(data);
            Graphics3DTranslateInstruction t = new Graphics3DTranslateInstruction(translation);
            def.addInstruction(t);

//          System.out.println("Translate");
            currentTypeIndex = XMLReaderUtility.getEndIndexOfSubString(currentTypeIndex, graphicsString, "</Translate>");
         }
         else
         {
            System.err.println("Type: " + type + " not supported");
            currentTypeIndex = XMLReaderUtility.getEndIndexOfSubString(currentTypeIndex, graphicsString, ">");
         }
      }

      return def;
   }



   public void addRootJoint(JointDefinitionFixedFrame jointDef)
   {
      rootJointDefinitions.add(jointDef);
   }

   public int getNumberOfJoints()
   {
      int numberOfJoints = rootJointDefinitions.size();
      for (JointDefinitionFixedFrame joint : rootJointDefinitions)
      {
         numberOfJoints += joint.getNumberOfChildJoints();
      }

      return numberOfJoints;
   }

   private JointDefinitionFixedFrame createJointDefinition(Joint joint, JointDefinitionFixedFrame parentJoint)
   {
      JointDefinitionFixedFrame jointDef = new JointDefinitionFixedFrame();

      if (parentJoint == null)
         jointDef.setRootJoint(true);

      jointDef.setJointName(joint.getName());

      jointDef.setParentJoint(parentJoint);

      if (joint.getGroundContactPointGroup() != null)
      {
         for (GroundContactPoint groundContactPoint : (joint.getGroundContactPointGroup().getGroundContactPoints()))
         {
            GroundContactDefinitionFixedFrame groundContactDefinitionFixedFrame = new GroundContactDefinitionFixedFrame();
            groundContactDefinitionFixedFrame.setName(groundContactPoint.getName());
            groundContactDefinitionFixedFrame.setOffset(groundContactPoint.getOffsetCopy());
            jointDef.addGroundContactDefinitionFixedFrame(groundContactDefinitionFixedFrame);
         }

      }

      if (joint.getExternalForcePoints() != null)
      {
         for (ExternalForcePoint externalForcePoint : joint.getExternalForcePoints())
         {
            ExternalForcePointDefinitionFixedFrame externalForcePointDefinitionFixedFrame = new ExternalForcePointDefinitionFixedFrame();
            externalForcePointDefinitionFixedFrame.setName(externalForcePoint.getName());
            externalForcePointDefinitionFixedFrame.setOffset(externalForcePoint.getOffsetCopy());
            jointDef.addExternalForcePointDefinition(externalForcePointDefinitionFixedFrame);
         }

      }

      Vector3D offset = new Vector3D();
      joint.getOffset(offset);
      jointDef.setOffset(offset);

      Vector3D jointAxis = new Vector3D();
      joint.getJointAxis(jointAxis);
      jointDef.setJointAxis(jointAxis);

      if (joint instanceof PinJoint)
      {
         jointDef.setType(JointType.PIN_JOINT);
      }

      else if (joint instanceof SliderJoint)
      {
         jointDef.setType(JointType.SLIDER_JOINT);
      }

      else if (joint instanceof FloatingJoint)
      {
         jointDef.setType(JointType.FLOATING_JOINT);
      }

      else if (joint instanceof FloatingPlanarJoint)
      {
         jointDef.setType(JointType.FLOATING_PLANAR_JOINT);
         jointDef.setPlanarType(((FloatingPlanarJoint) joint).getType());
      }
      else if (joint instanceof DummyOneDegreeOfFreedomJoint)
      {
         jointDef.setType(JointType.PIN_JOINT);
      }
      else
      {
         throw new RuntimeException("Only Pin, Slider, Floating, and Floating Planar joints implemented right now. joint = " + joint + ", joint class = " + joint.getClass());
      }


      Link link = joint.getLink();
      LinkDefinitionFixedFrame linkDef = createLinkDefinition(link);

      jointDef.setLinkDefinition(linkDef);

      ArrayList<Joint> childrenJoints = joint.getChildrenJoints();

      for (Joint childJoint : childrenJoints)
      {
         jointDef.addChildJoint(createJointDefinition(childJoint, jointDef));
      }

      return jointDef;
   }

   private LinkDefinitionFixedFrame createLinkDefinition(Link link)
   {
      LinkDefinitionFixedFrame linkDef = new LinkDefinitionFixedFrame();

      linkDef.setName(link.getName());

      linkDef.setMass(link.getMass());

      Vector3D comOffset = new Vector3D();
      link.getComOffset(comOffset);

      linkDef.setComOffset(comOffset);

      Matrix3D momentOfInertia = new Matrix3D();
      link.getMomentOfInertia(momentOfInertia);

      linkDef.setInertia(momentOfInertia);

      linkDef.setLinkGraphics(link.getLinkGraphics());

      return linkDef;
   }



   public ArrayList<JointDefinitionFixedFrame> getRootJointDefinitions()
   {
      return rootJointDefinitions;
   }

   public String returnAllJointStrings(JointDefinitionFixedFrame currentJoint, JointDefinitionFixedFrame parent)
   {
      String returnString = "";
      ArrayList<JointDefinitionFixedFrame> childrenJoints = currentJoint.getChildrenJoints();

//    if (parent == null)
//       returnString += "Found Root Joint";
//    else
//       returnString += "Found Child Joint of " + parent.getJointName() + ".\n";

      returnString += currentJoint.toString();


      for (JointDefinitionFixedFrame childJoint : childrenJoints)
      {
         returnString += returnAllJointStrings(childJoint, currentJoint);
      }

      return returnString;

   }

   @Override
   public String toString()
   {
      String returnString = "<RobotDefinition>\n";
      for (JointDefinitionFixedFrame rootJoint : rootJointDefinitions)
      {
         // returnString += rootJoint;
         returnString += returnAllJointStrings(rootJoint, null);
      }

      returnString += "</RobotDefinition>\n";

      return returnString;
   }
}
