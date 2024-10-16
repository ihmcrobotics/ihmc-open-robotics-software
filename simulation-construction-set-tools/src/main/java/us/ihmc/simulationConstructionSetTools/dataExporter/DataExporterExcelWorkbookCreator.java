package us.ihmc.simulationConstructionSetTools.dataExporter;

import java.io.File;
import java.io.IOException;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.DoubleStream;

import jxl.Cell;
import jxl.Workbook;
import jxl.write.DateTime;
import jxl.write.Label;
import jxl.write.Number;
import jxl.write.NumberFormats;
import jxl.write.WritableCell;
import jxl.write.WritableCellFormat;
import jxl.write.WritableFont;
import jxl.write.WritableSheet;
import jxl.write.WritableWorkbook;
import jxl.write.WriteException;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.commons.MultiBodySystemMissingTools;
import us.ihmc.scs2.sharedMemory.YoSharedBuffer;
import us.ihmc.scs2.sharedMemory.YoVariableBuffer;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFloatingRootJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.yoVariables.buffer.YoBuffer;
import us.ihmc.yoVariables.buffer.interfaces.YoBufferVariableEntryReader;
import us.ihmc.yoVariables.euclid.YoPoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoVariable;

public class DataExporterExcelWorkbookCreator
{
   private final List<OneDoFJointData> pinJoints = new ArrayList<>();
   private final List<JointInfo> allJoints = new ArrayList<>();
   private final Function<YoVariable, VariableData> dataBuffer;
   private final YoVariable timeVariable;
   private final String robotName;
   private final Class<?> robotType;
   private final double robotMass;
   private final Vector3D gravity = new Vector3D();
   private final YoPoint3D robotPosition;

   private static class JointInfo
   {
      final String jointName;
      final String linkName;
      final Vector3D jointOffset = new Vector3D();
      final double linkMass;
      final Vector3D linkCoM = new Vector3D();
      final Matrix3D linkMomentOfInertia = new Matrix3D();

      public JointInfo(Joint joint)
      {
         jointName = joint.getName();
         Link link = joint.getLink();
         linkName = link.getName();
         joint.getOffset(jointOffset);
         linkMass = link.getMass();
         link.getComOffset(linkCoM);
         link.getMomentOfInertia(linkMomentOfInertia);
      }

      public JointInfo(SimJointBasics joint)
      {
         jointName = joint.getName();
         SimRigidBodyBasics link = joint.getSuccessor();
         linkName = link.getName();
         jointOffset.set(joint.getFrameBeforeJoint().getTransformToParent().getTranslation());
         linkMass = link.getInertia().getMass();
         linkCoM.set(link.getBodyFixedFrame().getTransformToParent().getTranslation());
         linkMomentOfInertia.set(link.getInertia().getMomentOfInertia());
      }
   }

   private static class OneDoFJointData
   {
      private final String name;
      private final YoVariable position, speed, acceleration, torque;

      public OneDoFJointData(OneDegreeOfFreedomJoint joint)
      {
         name = joint.getName();
         position = joint.getQYoVariable();
         speed = joint.getQDYoVariable();
         acceleration = joint.getQDDYoVariable();
         torque = joint.getTauYoVariable();
      }

      public OneDoFJointData(SimRevoluteJoint joint)
      {
         name = joint.getName();
         position = joint.getYoQ();
         speed = joint.getYoQd();
         acceleration = joint.getYoQdd();
         torque = joint.getYoTau();
      }

      public String getName()
      {
         return name;
      }
   }

   private static class VariableData
   {
      final String variableName;
      final double[] buffer;
      final double lowerBound, upperBound;

      public VariableData(YoBufferVariableEntryReader entry)
      {
         variableName = entry.getVariableName();
         buffer = entry.getBuffer();
         lowerBound = entry.getLowerBound();
         upperBound = entry.getUpperBound();
      }

      public VariableData(YoVariableBuffer<?> entry)
      {
         variableName = entry.getYoVariable().getName();
         buffer = entry.getAsDoubleBuffer();
         lowerBound = DoubleStream.of(buffer).min().getAsDouble();
         upperBound = DoubleStream.of(buffer).max().getAsDouble();
      }

      public double[] getBuffer()
      {
         return buffer;
      }

      public double getLowerBound()
      {
         return lowerBound;
      }

      public double getUpperBound()
      {
         return upperBound;
      }
   }

   private final WritableCellFormat defaultFormat = new WritableCellFormat();
   private final WritableCellFormat headerCellFormat = new WritableCellFormat(new WritableFont(WritableFont.ARIAL, 10, WritableFont.BOLD));
   private final WritableCellFormat defaultNumberFormat = new WritableCellFormat(NumberFormats.FLOAT);
   private final WritableCellFormat smallNumberFormat = new WritableCellFormat(NumberFormats.EXPONENTIAL);

   // TODO: currently only does PinJoints
   public DataExporterExcelWorkbookCreator(Robot robot, YoBuffer dataBuffer)
   {
      robotName = robot.getName();
      robotType = robot.getClass();
      robotMass = robot.computeCenterOfMass(new Point3D());
      robot.getGravity(gravity);
      timeVariable = robot.getYoTime();

      robotPosition = extractRobotPosition(robot);

      List<Joint> jointList = new ArrayList<>();

      for (Joint rootJoint : robot.getRootJoints())
      {
         recursivelyAddJoints(rootJoint, jointList);
      }

      for (Joint joint : jointList)
      {
         allJoints.add(new JointInfo(joint));

         if (joint instanceof PinJoint)
         {
            pinJoints.add(new OneDoFJointData((PinJoint) joint));
         }
      }

      this.dataBuffer = new Function<YoVariable, VariableData>()
      {
         private Map<YoVariable, VariableData> cache = new HashMap<>();

         @Override
         public VariableData apply(YoVariable var)
         {
            VariableData varData = cache.get(var);
            if (varData == null)
               varData = new VariableData(dataBuffer.getEntry(var));
            return varData;
         }
      };
   }

   public DataExporterExcelWorkbookCreator(YoVariable time, us.ihmc.scs2.simulation.robot.Robot robot, YoSharedBuffer sharedBuffer)
   {
      robotName = robot.getName();
      robotType = robot.getClass();
      robotMass = MultiBodySystemMissingTools.computeSubTreeMass(robot.getRootBody());
      gravity.set(0.0, 0.0, -9.81); // FIXME
      timeVariable = time;

      YoFramePoint3D position = ((SimFloatingRootJoint) robot.getFloatingRootJoint()).getJointPose().getPosition();
      robotPosition = new YoPoint3D(position.getYoX(), position.getYoY(), position.getYoZ());

      List<? extends SimJointBasics> jointList = robot.getAllJoints();

      for (SimJointBasics joint : jointList)
      {
         allJoints.add(new JointInfo(joint));

         if (joint instanceof SimRevoluteJoint)
         {
            pinJoints.add(new OneDoFJointData((SimRevoluteJoint) joint));
         }
      }

      this.dataBuffer = new Function<YoVariable, VariableData>()
      {
         private Map<YoVariable, VariableData> cache = new HashMap<>();

         @Override
         public VariableData apply(YoVariable var)
         {
            VariableData varData = cache.get(var);
            if (varData == null)
               varData = new VariableData(sharedBuffer.getRegistryBuffer().findYoVariableBuffer(var));
            return varData;
         }
      };
   }

   public YoPoint3D extractRobotPosition(Robot robot)
   {
      Joint rootJoint = robot.getRootJoints().get(0);
      if (rootJoint instanceof FloatingJoint)
      {
         return new YoPoint3D(((FloatingJoint) rootJoint).getQx(), ((FloatingJoint) rootJoint).getQy(), ((FloatingJoint) rootJoint).getQz());
      }
      else
      {
         GroundContactPoint groundContactPoint = robot.getAllGroundContactPoints().get(0);
         YoFramePoint3D yoPosition = groundContactPoint.getYoPosition();
         return new YoPoint3D(yoPosition.getYoX(), yoPosition.getYoY(), yoPosition.getYoZ());
      }
   }

   public void createAndSaveTorqueAndSpeedSpreadSheet(File dataAndVideosTagDirectory, String fileHeader)
   {
      // write data to Excel workbook
      File workbookFile = new File(dataAndVideosTagDirectory, fileHeader + "_TorqueSpeedPowerEstimates.xls");
      WritableWorkbook writableWorkBook = createWorkbook(workbookFile);

      if (writableWorkBook != null)
      {
         writeInfoToWorkBook(writableWorkBook);
         writeVelocityAndTorqueNumbersToWorkBook(writableWorkBook);
         writeRobotConfigurationToWorkbook(writableWorkBook);
         writeJointDataToWorkbook(writableWorkBook);

         // save and close Excel workbook
         try
         {
            writableWorkBook.write();
            writableWorkBook.close();
            System.out.println("Done creating Excel workbook");
         }
         catch (Exception ex)
         {
            LogTools.error("Trouble saving Excel workbook " + workbookFile.getAbsolutePath());
         }
      }
   }

   private WritableWorkbook createWorkbook(File workbookFile)
   {
      WritableWorkbook writableWorkBook = null;
      try
      {
         writableWorkBook = Workbook.createWorkbook(workbookFile);
      }
      catch (IOException ex)
      {
         LogTools.error("Failed to open Excel workbook. " + workbookFile);
      }

      return writableWorkBook;
   }

   private void writeInfoToWorkBook(WritableWorkbook workbook)
   {
      WritableSheet infoSheet = workbook.createSheet("Run info", workbook.getNumberOfSheets());
      int labelColumn = 0;
      int dataColumn = 1;
      int row = 0;

      addStringToSheet(infoSheet, labelColumn, row, "Date: ", headerCellFormat);
      WritableCell dateCell = new DateTime(dataColumn, row, Date.from(ZonedDateTime.now().toInstant()));
      addCell(infoSheet, dateCell);
      row++;

      addStringToSheet(infoSheet, labelColumn, row, "Robot type: ", headerCellFormat);
      addStringToSheet(infoSheet, dataColumn, row, robotType.getSimpleName());
      row++;

      addStringToSheet(infoSheet, labelColumn, row, "Robot name: ", headerCellFormat);
      addStringToSheet(infoSheet, dataColumn, row, robotName);
      row++;

      addStringToSheet(infoSheet, labelColumn, row, "Total mass [kg]: ", headerCellFormat);
      addNumberToSheet(infoSheet, dataColumn, row, robotMass);
      row++;

      addStringToSheet(infoSheet, labelColumn, row, "Run time [s]: ", headerCellFormat);
      addNumberToSheet(infoSheet, dataColumn, row, dataBuffer.apply(timeVariable).getUpperBound());
      row++;

      addStringToSheet(infoSheet, labelColumn, row, "Mechanical cost of transport: ", headerCellFormat);
      addNumberToSheet(infoSheet, dataColumn, row, computeMechanicalCostOfTransport());
      row++;
   }

   private void writeVelocityAndTorqueNumbersToWorkBook(WritableWorkbook workbook)
   {
      WritableSheet dataSheet = workbook.createSheet("Velocity and torque", workbook.getNumberOfSheets());
      int row = 1;

      for (OneDoFJointData jointData : pinJoints)
      {
         int column = 0;

         VariableData position = dataBuffer.apply(jointData.position);
         VariableData speed = dataBuffer.apply(jointData.speed);
         VariableData torque = dataBuffer.apply(jointData.torque);

         addHeaderEntry(dataSheet, column, "Joint");
         String jointName = jointData.getName();
         addStringToSheet(dataSheet, column++, row, jointName);

         addHeaderEntry(dataSheet, column, "Min joint position [rad]");
         addNumberToSheet(dataSheet, column++, row, position.getLowerBound());

         addHeaderEntry(dataSheet, column, "Max joint position [rad]");
         addNumberToSheet(dataSheet, column++, row, position.getUpperBound());

         addHeaderEntry(dataSheet, column, "Min joint speed [rad / s]");
         addNumberToSheet(dataSheet, column++, row, speed.getLowerBound());

         addHeaderEntry(dataSheet, column, "Max joint speed [rad / s]");
         addNumberToSheet(dataSheet, column++, row, speed.getUpperBound());

         addHeaderEntry(dataSheet, column, "Min joint torque [Nm]");
         addNumberToSheet(dataSheet, column++, row, torque.getLowerBound());

         addHeaderEntry(dataSheet, column, "Max joint torque [Nm]");
         addNumberToSheet(dataSheet, column++, row, torque.getUpperBound());

         addHeaderEntry(dataSheet, column, "Range of Motion [rad]");
         double rangeOfMotion = position.getUpperBound() - position.getLowerBound();
         addNumberToSheet(dataSheet, column++, row, rangeOfMotion);

         addHeaderEntry(dataSheet, column, "Max unsigned speed [rad / s]");
         double maxUnsignedSpeed = Math.max(Math.abs(speed.getUpperBound()), Math.abs(speed.getLowerBound()));
         addNumberToSheet(dataSheet, column++, row, maxUnsignedSpeed);

         addHeaderEntry(dataSheet, column, "Average of unsigned speed [rad / s]");
         double averageOfUnsignedSpeed = computeAverage(speed.getBuffer(), true);
         addNumberToSheet(dataSheet, column++, row, averageOfUnsignedSpeed);

         addHeaderEntry(dataSheet, column, "Max unsigned torque [Nm]");
         double maxUnsignedTorque = Math.max(Math.abs(torque.getUpperBound()), Math.abs(torque.getLowerBound()));
         addNumberToSheet(dataSheet, column++, row, maxUnsignedTorque);

         addHeaderEntry(dataSheet, column, "Average of unsigned torque [Nm]");
         double averageOfUnsignedTorque = computeAverage(torque.getBuffer(), true);
         addNumberToSheet(dataSheet, column++, row, averageOfUnsignedTorque);

         double[] mechanicalPower = computeMechanicalPower(speed.getBuffer(), torque.getBuffer());

         addHeaderEntry(dataSheet, column, "Max unsigned mechanical power [W]");
         double maxUnsignedMechanicalPower = computeMax(mechanicalPower, true);
         addNumberToSheet(dataSheet, column++, row, maxUnsignedMechanicalPower);

         addHeaderEntry(dataSheet, column, "Average of unsigned mechanical power [W]");
         double averageOfUnsignedMechanicalPower = computeAverage(mechanicalPower, true);
         addNumberToSheet(dataSheet, column++, row, averageOfUnsignedMechanicalPower);

         row++;
      }
   }

   private double computeMechanicalCostOfTransport()
   {
      double cot = computeTotalMechanicalEnergy();
      // Get the distance traveled through a random GroundContactPoint assuming it is attached to the robot
      // TODO: it would be way nicer to get access to the actual robot position
      double[] xPosition = dataBuffer.apply(robotPosition.getYoX()).getBuffer();
      double[] yPosition = dataBuffer.apply(robotPosition.getYoY()).getBuffer();
      double[] zPosition = dataBuffer.apply(robotPosition.getYoZ()).getBuffer();
      int dataLength = xPosition.length;
      Vector3D totalDistance = new Vector3D();
      totalDistance.setX(xPosition[dataLength - 1] - xPosition[0]);
      totalDistance.setY(yPosition[dataLength - 1] - yPosition[0]);
      totalDistance.setZ(zPosition[dataLength - 1] - zPosition[0]);

      cot = cot / (robotMass * gravity.length() * totalDistance.length());

      return cot;
   }

   private double computeTotalMechanicalEnergy()
   {
      double ret = 0.0;
      double[] mechanicalPower = computeTotalUnsignedMechanicalPower();
      VariableData timeData = dataBuffer.apply(timeVariable);
      double simulationTime = timeData.getUpperBound() - timeData.getLowerBound();
      double simulationDT = simulationTime / timeData.getBuffer().length;

      for (int i = 0; i < mechanicalPower.length; i++)
      {
         ret += simulationDT * Math.abs(mechanicalPower[i]);
      }

      return ret;
   }

   private double[] computeTotalUnsignedMechanicalPower()
   {
      int dataLength = dataBuffer.apply(timeVariable).getBuffer().length;
      double[] ret = new double[dataLength];
      for (int i = 0; i < dataLength; i++)
         ret[i] = 0.0;

      for (OneDoFJointData jointData : pinJoints)
      {
         double[] speed = dataBuffer.apply(jointData.speed).getBuffer();
         double[] torque = dataBuffer.apply(jointData.torque).getBuffer();

         double[] jointMechincalPower = computeMechanicalPower(speed, torque);

         for (int i = 0; i < dataLength; i++)
         {
            ret[i] += Math.abs(jointMechincalPower[i]);
         }
      }

      return ret;
   }

   private double[] computeMechanicalPower(double[] speed, double[] torque)
   {
      if (speed.length != torque.length)
         throw new RuntimeException("speed.length != torque.length");
      double[] ret = new double[speed.length];
      for (int i = 0; i < ret.length; i++)
      {
         ret[i] = speed[i] * torque[i];
      }

      return ret;
   }

   private double computeAverage(double[] data, boolean unsigned)
   {
      double ret = 0.0;
      for (int i = 0; i < data.length; i++)
      {
         double value = data[i];
         if (unsigned)
            value = Math.abs(value);
         ret += value / data.length;
      }

      return ret;
   }

   private double computeMax(double[] data, boolean unsigned)
   {
      double max = Double.NEGATIVE_INFINITY;
      for (int i = 0; i < data.length; i++)
      {
         double value = data[i];
         if (unsigned)
            value = Math.abs(value);
         if (value > max)
            max = value;
      }

      return max;
   }

   private void writeRobotConfigurationToWorkbook(WritableWorkbook workbook)
   {
      WritableSheet configDataSheet = workbook.createSheet("Robot Configuration", workbook.getNumberOfSheets());

      int row = 1;
      for (JointInfo jointInfo : allJoints)
      {
         int column = 0;

         // joint name
         addHeaderEntry(configDataSheet, column, "Joint name");
         String jointName = jointInfo.jointName;
         addStringToSheet(configDataSheet, column++, row, jointName);

         // link name
         addHeaderEntry(configDataSheet, column, "Link name");
         String linkName = jointInfo.linkName;
         addStringToSheet(configDataSheet, column++, row, linkName);

         // Link offset
         Vector3D offset = jointInfo.jointOffset;

         for (Axis3D axis : Axis3D.values())
         {
            addHeaderEntry(configDataSheet, column, "Joint offset " + axis.toString().toLowerCase());
            addNumberToSheet(configDataSheet, column++, row, axis.extract(offset));
         }

         // Mass
         Double mass = jointInfo.linkMass;
         addHeaderEntry(configDataSheet, column, "Mass");
         addNumberToSheet(configDataSheet, column++, row, mass);

         // CoM offset
         Vector3D comOffset = jointInfo.linkCoM;

         for (Axis3D axis : Axis3D.values())
         {
            addHeaderEntry(configDataSheet, column, "CoM offset " + axis.toString().toLowerCase());
            addNumberToSheet(configDataSheet, column++, row, axis.extract(comOffset));
         }

         // Mass moment of inertia
         Matrix3D momentOfInertia = jointInfo.linkMomentOfInertia;

         for (int i = 0; i < 3; i++)
         {
            for (int j = 0; j < 3; j++)
            {
               addHeaderEntry(configDataSheet, column, "MOI " + (i + 1) + "," + (j + 1));
               addNumberToSheet(configDataSheet, column++, row, momentOfInertia.getElement(i, j), smallNumberFormat);
            }
         }

         row++;
      }
   }

   private void writeJointDataToWorkbook(WritableWorkbook workbook)
   {
      WritableSheet jointDataSheet = workbook.createSheet("Joint Data", workbook.getNumberOfSheets());
      int column = 0;
      writeJointDataColumn(jointDataSheet, column++, dataBuffer.apply(timeVariable), true);

      for (OneDoFJointData joint : pinJoints)
      {
         VariableData position = dataBuffer.apply(joint.position);
         VariableData speed = dataBuffer.apply(joint.speed);
         VariableData torque = dataBuffer.apply(joint.torque);
         VariableData acceleration = dataBuffer.apply(joint.acceleration);

         writeJointDataColumn(jointDataSheet, column++, position, false);
         writeJointDataColumn(jointDataSheet, column++, speed, false);
         writeJointDataColumn(jointDataSheet, column++, speed, true);
         writeJointDataColumn(jointDataSheet, column++, torque, false);
         writeJointDataColumn(jointDataSheet, column++, torque, true);
         writeJointDataColumn(jointDataSheet, column++, acceleration, false);
         writeJointDataColumn(jointDataSheet, column++, acceleration, true);
         writeMechanicalPowerJointDataColumn(jointDataSheet, column++, speed, torque, joint.getName());
      }
   }

   private void writeJointDataColumn(WritableSheet dataSheet, int column, VariableData dataBufferEntry, boolean unsigned)
   {
      int row = 0;
      String name = dataBufferEntry.variableName;
      if (unsigned)
         name = name + " unsigned";
      addStringToSheet(dataSheet, column, row++, name);
      double[] data = dataBufferEntry.getBuffer();
      for (int i = 0; i < data.length; i++)
      {
         double value = data[i];
         if (unsigned)
            value = Math.abs(value);
         addNumberToSheet(dataSheet, column, row++, value);
      }
   }

   private void writeMechanicalPowerJointDataColumn(WritableSheet dataSheet, int column, VariableData speed, VariableData torque, String jointName)
   {
      int row = 0;
      String name = jointName + " unsigned mechanical power";
      addStringToSheet(dataSheet, column, row++, name);
      double[] speedData = speed.getBuffer();
      double[] torqueData = torque.getBuffer();

      if (speedData.length != torqueData.length)
         throw new RuntimeException("speedData.length != torqueData.length");

      for (int i = 0; i < speedData.length; i++)
      {
         double value = Math.abs(speedData[i] * torqueData[i]);
         addNumberToSheet(dataSheet, column, row++, value);
      }
   }

   private void addHeaderEntry(WritableSheet dataSheet, int column, String string)
   {
      int row = 0;
      Cell cell = dataSheet.getCell(column, row);
      if (cell.getContents() != string)
         addStringToSheet(dataSheet, column, row, string, headerCellFormat);
   }

   private void addNumberToSheet(WritableSheet dataSheet, int column, int row, double value)
   {
      addNumberToSheet(dataSheet, column, row, value, defaultNumberFormat);
   }

   private void addNumberToSheet(WritableSheet dataSheet, int column, int row, double value, WritableCellFormat format)
   {
      Number cell = new Number(column, row, value, format);
      addCell(dataSheet, cell);
   }

   private void addStringToSheet(WritableSheet dataSheet, int column, int row, String string)
   {
      addStringToSheet(dataSheet, column, row, string, defaultFormat);
   }

   private static void addStringToSheet(WritableSheet dataSheet, int column, int row, String string, WritableCellFormat format)
   {
      Label cell = new Label(column, row, string, format);
      addCell(dataSheet, cell);
   }

   private static void addCell(WritableSheet dataSheet, WritableCell cell)
   {
      try
      {
         dataSheet.addCell(cell);
      }
      catch (WriteException e)
      {
         e.printStackTrace();
      }
   }

   private static void recursivelyAddJoints(Joint joint, List<Joint> allJoints)
   {
      allJoints.add(joint);

      for (Joint child : joint.getChildrenJoints())
      {
         recursivelyAddJoints(child, allJoints);
      }
   }
}
