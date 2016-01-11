package us.ihmc.valkyrieRosControl;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Map;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Marshaller;

import org.yaml.snakeyaml.Yaml;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.DiagnosticsWhenHangingHelper;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.valkyrieRosControl.XMLJoints.XMLJointWithTorqueOffset;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticsWhenHangingController;
import us.ihmc.wholeBodyController.diagnostics.TorqueOffsetPrinter;

public class ValkyrieTorqueOffsetPrinter implements TorqueOffsetPrinter
{
   private static final String NASA_TORQUE_OFFSET_FILE = System.getProperty("user.home") + File.separator + "valkyrie/ValkyrieJointTorqueOffsets.xml";
   public static final String IHMC_TORQUE_OFFSET_FILE = System.getProperty("user.home") + File.separator + ".ihmc/ControllerConfig/jointTorqueOffsets.yaml";
   
   private static final boolean WRITE_OFFSETS_TO_FILE_FOR_NASA = false;
   private static final boolean WRITE_OFFSETS_TO_FILE_FOR_IHMC = true;

   private final java.text.NumberFormat doubleFormat = new java.text.DecimalFormat(" 0.00;-0.00");
   private String robotName = "Valkyrie";

   public void setRobotName(String robotName)
   {
      this.robotName = robotName;
   }

   @SuppressWarnings("unchecked")
   @Override
   public void printTorqueOffsets(DiagnosticsWhenHangingController diagnosticsWhenHangingController)
   {

      System.out.println();

      ArrayList<OneDoFJoint> oneDoFJoints = diagnosticsWhenHangingController.getOneDoFJoints();

      int maxNameLength = 0;
      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
         if (diagnosticsWhenHangingController.getDiagnosticsWhenHangingHelper(oneDoFJoint) != null)
            maxNameLength = Math.max(maxNameLength, oneDoFJoint.getName().length());

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         DiagnosticsWhenHangingHelper diagnosticsWhenHangingHelper = diagnosticsWhenHangingController.getDiagnosticsWhenHangingHelper(oneDoFJoint);

         if (diagnosticsWhenHangingHelper != null)
         {
            double torqueOffset = diagnosticsWhenHangingHelper.getTorqueOffset();
            double torqueOffsetSign = diagnosticsWhenHangingController.getTorqueOffsetSign(oneDoFJoint);

            double signedTorqueOffset = torqueOffset * torqueOffsetSign;

            String offsetString = doubleFormat.format(signedTorqueOffset);
            int nblankSpaces = maxNameLength - oneDoFJoint.getName().length() + 1;
            String blanks = String.format("%1$" + nblankSpaces + "s", "");
            System.out.println(oneDoFJoint.getName() + blanks + "torque offset = " + offsetString);
         }
      }

      System.out.println();

      if (WRITE_OFFSETS_TO_FILE_FOR_NASA)
      {
         File file = new File(NASA_TORQUE_OFFSET_FILE);
         try
         {
            writeTorqueOffsetsToFile(file, buildXMLJoints(diagnosticsWhenHangingController));
         }
         catch (JAXBException e)
         {
            e.printStackTrace();
         }
      }

      if (WRITE_OFFSETS_TO_FILE_FOR_IHMC)
      {

         Yaml yaml = new Yaml();
         InputStream offsetsStream;
         Map<String, Double> oldOffsetMap = null;
         try
         {
            offsetsStream = new FileInputStream(new File(IHMC_TORQUE_OFFSET_FILE));
            oldOffsetMap = (Map<String, Double>) yaml.load(offsetsStream);
            if (offsetsStream != null) offsetsStream.close();
         }
         catch (Exception e)
         {
            offsetsStream = null;
            oldOffsetMap = null;
         }

         Path torqueOffsetFilePath = Paths.get(IHMC_TORQUE_OFFSET_FILE);
         try
         {
            Files.createDirectories(torqueOffsetFilePath.getParent());
            writeTorqueOffsetForIHMC(diagnosticsWhenHangingController, torqueOffsetFilePath.toFile(), oldOffsetMap);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
   }

   private XMLJoints buildXMLJoints(DiagnosticsWhenHangingController diagnosticsWhenHangingController)
   {
      XMLJoints xmlJoints = new XMLJoints();
      xmlJoints.setRobotName(robotName);
      ArrayList<XMLJointWithTorqueOffset> jointsWithTorqueOffset = new ArrayList<>();

      ArrayList<OneDoFJoint> oneDoFJoints = diagnosticsWhenHangingController.getOneDoFJoints();

      for (OneDoFJoint joint : oneDoFJoints)
      {
         DiagnosticsWhenHangingHelper diagnosticsWhenHangingHelper = diagnosticsWhenHangingController.getDiagnosticsWhenHangingHelper(joint);

         if (diagnosticsWhenHangingHelper == null)
            continue;

         String name = joint.getName();
         String position = Double.toString(joint.getQ());
         String torqueOffset = Double.toString(diagnosticsWhenHangingHelper.getTorqueOffset());
         String type = null;

         if (name.contains("Ankle"))
            type = "ankle";
         else if (name.contains("torso"))
            type = "waist";

         XMLJointWithTorqueOffset xmlJointWithTorqueOffset = new XMLJointWithTorqueOffset();
         xmlJointWithTorqueOffset.setName(name);
         xmlJointWithTorqueOffset.setPosition(position);
         xmlJointWithTorqueOffset.setTorqueOffset(torqueOffset);
         xmlJointWithTorqueOffset.setType(type);

         jointsWithTorqueOffset.add(xmlJointWithTorqueOffset);
      }

      xmlJoints.setJoints(jointsWithTorqueOffset);

      return xmlJoints;
   }

   private void writeTorqueOffsetsToFile(File file, XMLJoints joints) throws JAXBException
   {
      JAXBContext context = JAXBContext.newInstance(XMLJoints.class);
      Marshaller marshaller = context.createMarshaller();

      marshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
      marshaller.marshal(joints, file);
   }

   private void writeTorqueOffsetForIHMC(DiagnosticsWhenHangingController diagnosticsWhenHangingController, File file, Map<String, Double> oldOffsetMap) throws IOException
   {
      
      FileWriter fileWriter = new FileWriter(file);
      BufferedWriter bufferedWriter = new BufferedWriter(fileWriter);

      ArrayList<OneDoFJoint> oneDoFJoints = diagnosticsWhenHangingController.getOneDoFJoints();

      int maxNameLength = 0;
      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
         if (diagnosticsWhenHangingController.getDiagnosticsWhenHangingHelper(oneDoFJoint) != null)
            maxNameLength = Math.max(maxNameLength, oneDoFJoint.getName().length());

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         DiagnosticsWhenHangingHelper diagnosticsWhenHangingHelper = diagnosticsWhenHangingController.getDiagnosticsWhenHangingHelper(oneDoFJoint);

         if (diagnosticsWhenHangingHelper != null)
         {
            double oldTorqueOffset = 0.0;
            if (oldOffsetMap != null && oldOffsetMap.containsKey(oneDoFJoint.getName()))
               oldTorqueOffset = oldOffsetMap.get(oneDoFJoint.getName());
            double newTorqueOffset = oldTorqueOffset - diagnosticsWhenHangingHelper.getTorqueOffset();

            String offsetString = doubleFormat.format(newTorqueOffset);
            int nblankSpaces = maxNameLength - oneDoFJoint.getName().length() + 1;
            String blanks = String.format("%1$" + nblankSpaces + "s", "");
            bufferedWriter.write(oneDoFJoint.getName() + blanks + ": " + offsetString);
            bufferedWriter.newLine();
         }
      }
      
      bufferedWriter.flush();
      bufferedWriter.close();
   }
}