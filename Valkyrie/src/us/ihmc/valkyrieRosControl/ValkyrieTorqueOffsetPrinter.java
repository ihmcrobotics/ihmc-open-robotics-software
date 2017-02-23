package us.ihmc.valkyrieRosControl;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Marshaller;
import javax.xml.bind.Unmarshaller;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.valkyrieRosControl.XMLJoints.XMLJointWithTorqueOffset;
import us.ihmc.wholeBodyController.diagnostics.JointTorqueOffsetEstimator;
import us.ihmc.wholeBodyController.diagnostics.TorqueOffsetPrinter;

public class ValkyrieTorqueOffsetPrinter implements TorqueOffsetPrinter
{
   private static final boolean PRINT_TORQUE_OFFSETS = false;
   private static final String TORQUE_OFFSET_FILE = System.getProperty("user.home") + File.separator + "valkyrie/ValkyrieJointTorqueOffsets.xml";

   private final java.text.NumberFormat doubleFormat = new java.text.DecimalFormat(" 0.00;-0.00");
   private String robotName = "Valkyrie";

   public void setRobotName(String robotName)
   {
      this.robotName = robotName;
   }

   @Override
   public void printTorqueOffsets(JointTorqueOffsetEstimator jointTorqueOffsetEstimator)
   {
      if (PRINT_TORQUE_OFFSETS)
      {
         System.out.println();
         
         List<OneDoFJoint> oneDoFJoints = jointTorqueOffsetEstimator.getOneDoFJoints();
         
         int maxNameLength = 0;
         for (OneDoFJoint oneDoFJoint : oneDoFJoints)
            if (jointTorqueOffsetEstimator.hasTorqueOffsetForJoint(oneDoFJoint))
               maxNameLength = Math.max(maxNameLength, oneDoFJoint.getName().length());
         
         for (OneDoFJoint oneDoFJoint : oneDoFJoints)
         {
            if (jointTorqueOffsetEstimator.hasTorqueOffsetForJoint(oneDoFJoint))
            {
               double torqueOffset = jointTorqueOffsetEstimator.getEstimatedJointTorqueOffset(oneDoFJoint);
               String offsetString = doubleFormat.format(torqueOffset);
               int nblankSpaces = maxNameLength - oneDoFJoint.getName().length() + 1;
               String blanks = String.format("%1$" + nblankSpaces + "s", "");
               System.out.println(oneDoFJoint.getName() + blanks + "torque offset = " + offsetString);
            }
         }
      }

      try
      {
         Path torqueOffsetFilePath = Paths.get(TORQUE_OFFSET_FILE);
         Files.createDirectories(torqueOffsetFilePath.getParent());
         File file = torqueOffsetFilePath.toFile();
         Map<String, Double> oldTorqueOffsets = loadTorqueOffsetsFromFile();
         exportTorqueOffsetsToFile(file, buildXMLJoints(jointTorqueOffsetEstimator, oldTorqueOffsets));
      }
      catch (JAXBException | IOException e)
      {
         e.printStackTrace();
      }
   }

   private XMLJoints buildXMLJoints(JointTorqueOffsetEstimator jointTorqueOffsetEstimator, Map<String, Double> oldTorqueOffsets)
   {
      XMLJoints xmlJoints = new XMLJoints();
      xmlJoints.setRobotName(robotName);
      ArrayList<XMLJointWithTorqueOffset> jointsWithTorqueOffset = new ArrayList<>();

      List<OneDoFJoint> oneDoFJoints = jointTorqueOffsetEstimator.getOneDoFJoints();

      for (OneDoFJoint joint : oneDoFJoints)
      {
         if (!jointTorqueOffsetEstimator.hasTorqueOffsetForJoint(joint))
            continue;

         String jointName = joint.getName();
         String position = Double.toString(joint.getQ());
         
         double jointTorqueOffsetToExport = jointTorqueOffsetEstimator.getEstimatedJointTorqueOffset(joint);
         if (oldTorqueOffsets != null && oldTorqueOffsets.containsKey(jointName))
         {
            jointTorqueOffsetToExport += oldTorqueOffsets.get(jointName);
         }
         String torqueOffset = Double.toString(jointTorqueOffsetToExport);
            
         String type = null;

         if (jointName.contains("leftAnkle"))
            type = "leftAnkle";
         if (jointName.contains("rightAnkle"))
            type = "rightAnkle";
         else if (jointName.contains("torsoRoll") || jointName.contains("torsoPitch"))
            type = "waist";

         XMLJointWithTorqueOffset xmlJointWithTorqueOffset = new XMLJointWithTorqueOffset();
         xmlJointWithTorqueOffset.setName(jointName);
         xmlJointWithTorqueOffset.setPosition(position);
         xmlJointWithTorqueOffset.setTorqueOffset(torqueOffset);
         xmlJointWithTorqueOffset.setType(type);

         jointsWithTorqueOffset.add(xmlJointWithTorqueOffset);
      }

      xmlJoints.setJoints(jointsWithTorqueOffset);

      return xmlJoints;
   }

   private void exportTorqueOffsetsToFile(File file, XMLJoints joints) throws JAXBException
   {
      JAXBContext context = JAXBContext.newInstance(XMLJoints.class);
      Marshaller marshaller = context.createMarshaller();

      marshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
      marshaller.marshal(joints, file);
   }

   public static Map<String, Double> loadTorqueOffsetsFromFile()
   {
      JAXBContext context;
      try
      {
         File file = new File(TORQUE_OFFSET_FILE);
         context = JAXBContext.newInstance(XMLJoints.class);
         Unmarshaller um = context.createUnmarshaller();
         XMLJoints xmlJoints = (XMLJoints) um.unmarshal(file);
         List<XMLJointWithTorqueOffset> joints = xmlJoints.getJoints();
         if (xmlJoints == null || joints == null)
            return null;

         Map<String, Double> torqueOffsetMap = new HashMap<>();

         for (XMLJointWithTorqueOffset jointWithTorqueOffset : joints)
         {
            String jointName = jointWithTorqueOffset.getName();
            double torqueOffset = Double.parseDouble(jointWithTorqueOffset.getTorqueOffset());

            torqueOffsetMap.put(jointName, torqueOffset);
         }
         return torqueOffsetMap;
      }
      catch (JAXBException e)
      {
         return null;
      }
   }

   public static void main(String[] args)
   {
      ValkyrieTorqueOffsetPrinter printer = new ValkyrieTorqueOffsetPrinter();
      
      List<RevoluteJoint> revoluteJoints = new ArrayList<>();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("elevatorFrame", ReferenceFrame.getWorldFrame(), new RigidBodyTransform());
      RigidBody rootBody = new RigidBody("elevator", elevatorFrame);
      final Random random = new Random();
      Vector3D[] jointAxes = new Vector3D[random.nextInt(10)];
      for (int i = 0; i < jointAxes.length; i++)
         jointAxes[i] = RandomTools.generateRandomVector(random, 1.0);
      ScrewTestTools.createRandomChainRobot("blop", revoluteJoints, rootBody, jointAxes, random);
      final List<OneDoFJoint> oneDoFJoints = new ArrayList<>();
      for (RevoluteJoint revoluteJoint : revoluteJoints)
         oneDoFJoints.add(revoluteJoint);
      
      JointTorqueOffsetEstimator jointTorqueOffsetEstimator = new JointTorqueOffsetEstimator()
      {
         @Override
         public void resetEstimatedJointTorqueOffset(OneDoFJoint joint)
         {
         }
         
         @Override
         public boolean hasTorqueOffsetForJoint(OneDoFJoint joint)
         {
            return oneDoFJoints.contains(joint);
         }
         
         @Override
         public List<OneDoFJoint> getOneDoFJoints()
         {
            return oneDoFJoints;
         }
         
         @Override
         public double getEstimatedJointTorqueOffset(OneDoFJoint joint)
         {
            return random.nextDouble();
         }
         
         @Override
         public void enableJointTorqueOffsetEstimationAtomic(boolean enable)
         {
         }
      };
      printer.printTorqueOffsets(jointTorqueOffsetEstimator);
   }
   
}