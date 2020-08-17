package us.ihmc.simulationConstructionSetTools.dataExporter;

import java.io.File;
import java.io.IOException;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

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
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.yoVariables.buffer.YoBuffer;
import us.ihmc.yoVariables.buffer.YoBufferVariableEntry;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;

public class DataExporterExcelWorkbookCreator
{
   private static boolean DEBUG = false;

   private final Robot robot;
   private final List<PinJoint> pinJoints = new ArrayList<PinJoint>();
   private final List<Joint> allJoints = new ArrayList<Joint>();
   private final YoBuffer dataBuffer;

   private final WritableCellFormat defaultFormat;
   private final WritableCellFormat headerCellFormat;
   private final WritableCellFormat defaultNumberFormat;
   private final WritableCellFormat smallNumberFormat;

// TODO: currently only does PinJoints
   public DataExporterExcelWorkbookCreator(Robot robot, YoBuffer dataBuffer)
   {
      this.robot = robot;

      for (Joint rootJoint : robot.getRootJoints())
      {
         recursivelyAddPinJoints(rootJoint, pinJoints);
         recursivelyAddJoints(rootJoint, allJoints);
      }

      this.dataBuffer = dataBuffer;

      defaultFormat = new WritableCellFormat();

      WritableFont headerFont = new WritableFont(WritableFont.ARIAL, 10, WritableFont.BOLD);
      headerCellFormat = new WritableCellFormat(headerFont);

      defaultNumberFormat = new WritableCellFormat(NumberFormats.FLOAT);
      smallNumberFormat = new WritableCellFormat(NumberFormats.EXPONENTIAL);
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
            PrintTools.error(this, "Trouble saving Excel workbook " + workbookFile.getAbsolutePath());
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
         PrintTools.error(this, "Failed to open Excel workbook. " + workbookFile);
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
      addStringToSheet(infoSheet, dataColumn, row, robot.getClass().getSimpleName());
      row++;

      addStringToSheet(infoSheet, labelColumn, row, "Robot name: ", headerCellFormat);
      addStringToSheet(infoSheet, dataColumn, row, robot.getName());
      row++;

      addStringToSheet(infoSheet, labelColumn, row, "Total mass [kg]: ", headerCellFormat);
      addNumberToSheet(infoSheet, dataColumn, row, robot.computeCenterOfMass(new Point3D()));
      row++;

      addStringToSheet(infoSheet, labelColumn, row, "Run time [s]: ", headerCellFormat);
      addNumberToSheet(infoSheet, dataColumn, row, dataBuffer.getEntry(robot.getYoTime()).getUpperBound());
      row++;

      addStringToSheet(infoSheet, labelColumn, row, "Mechanical cost of transport: ", headerCellFormat);
      addNumberToSheet(infoSheet, dataColumn, row, computeMechanicalCostOfTransport());
      row++;
   }

   private void writeVelocityAndTorqueNumbersToWorkBook(WritableWorkbook workbook)
   {
      WritableSheet dataSheet = workbook.createSheet("Velocity and torque", workbook.getNumberOfSheets());
      int row = 1;

      for (PinJoint joint : pinJoints)
      {
         int column = 0;

         YoBufferVariableEntry position = dataBuffer.getEntry(joint.getQYoVariable());
         YoBufferVariableEntry speed = dataBuffer.getEntry(joint.getQDYoVariable());
         YoBufferVariableEntry torque = dataBuffer.getEntry(joint.getTauYoVariable());

         addHeaderEntry(dataSheet, column, "Joint");
         String jointName = joint.getName();
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
      double robotMass = robot.computeCenterOfMass(new Point3D());
      Vector3D gravity = new Vector3D();
      robot.getGravity(gravity);
      // Get the distance traveled through a random GroundContactPoint assuming it is attached to the robot
      // TODO: it would be way nicer to get access to the actual robot position
      double[] xPosition;
      double[] yPosition;
      double[] zPosition;
      try
      {
         xPosition = dataBuffer.findVariableEntry("q_x").getBuffer();
         yPosition = dataBuffer.findVariableEntry("q_y").getBuffer();
         zPosition = dataBuffer.findVariableEntry("q_z").getBuffer();
      }
      catch (NullPointerException e)
      {
         GroundContactPoint groundContactPoint = robot.getAllGroundContactPoints().get(0);
         YoFramePoint3D yoPosition = groundContactPoint.getYoPosition();
         xPosition = dataBuffer.getEntry(yoPosition.getYoX()).getBuffer();
         yPosition = dataBuffer.getEntry(yoPosition.getYoY()).getBuffer();
         zPosition = dataBuffer.getEntry(yoPosition.getYoZ()).getBuffer();
      }
      int dataLength = xPosition.length;
      Vector3D totalDistance = new Vector3D();
      totalDistance.setX(xPosition[dataLength-1]-xPosition[0]);
      totalDistance.setY(yPosition[dataLength-1]-yPosition[0]);
      totalDistance.setZ(zPosition[dataLength-1]-zPosition[0]);

      cot = cot / (robotMass * gravity.length() * totalDistance.length());

      return cot;
   }

   private double computeTotalMechanicalEnergy()
   {
      double ret = 0.0;
      double[] mechanicalPower = computeTotalUnsignedMechanicalPower();
      double simulationTime = dataBuffer.getEntry(robot.getYoTime()).getUpperBound();
      double simulationDT = simulationTime / dataBuffer.getBufferSize();

      for (int i = 0; i < mechanicalPower.length; i++)
      {
         ret += simulationDT * Math.abs(mechanicalPower[i]);
      }

      return ret;
   }

   private double[] computeTotalUnsignedMechanicalPower()
   {
      int dataLength = dataBuffer.getEntry(robot.getYoTime()).getBuffer().length;
      double[] ret = new double[dataLength];
      for (int i = 0; i < dataLength; i++)
         ret[i] = 0.0;

      for (PinJoint joint : pinJoints)
      {
         double[] speed = dataBuffer.getEntry(joint.getQDYoVariable()).getBuffer();
         double[] torque = dataBuffer.getEntry(joint.getTauYoVariable()).getBuffer();

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
      for (Joint joint : allJoints)
      {
         Link link = joint.getLink();
         int column = 0;

         // joint name
         addHeaderEntry(configDataSheet, column, "Joint name");
         String jointName = joint.getName();
         addStringToSheet(configDataSheet, column++, row, jointName);

         // link name
         addHeaderEntry(configDataSheet, column, "Link name");
         String linkName = link.getName();
         addStringToSheet(configDataSheet, column++, row, linkName);

         // Link offset
         Vector3D offset = new Vector3D();
         joint.getOffset(offset);

         for (Axis3D axis : Axis3D.values())
         {
            addHeaderEntry(configDataSheet, column, "Joint offset " + axis.toString().toLowerCase());
            addNumberToSheet(configDataSheet, column++, row, Axis3D.get(offset, axis));
         }

         // Mass
         Double mass = link.getMass();
         addHeaderEntry(configDataSheet, column, "Mass");
         addNumberToSheet(configDataSheet, column++, row, mass);

         // CoM offset
         Vector3D comOffset = new Vector3D();
         link.getComOffset(comOffset);

         for (Axis3D axis : Axis3D.values())
         {
            addHeaderEntry(configDataSheet, column, "CoM offset " + axis.toString().toLowerCase());
            addNumberToSheet(configDataSheet, column++, row, Axis3D.get(offset, axis));
         }

         // Mass moment of inertia
         Matrix3D momentOfInertia = new Matrix3D();
         link.getMomentOfInertia(momentOfInertia);

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
      writeJointDataColumn(jointDataSheet, column++, dataBuffer.findVariableEntry("t"), true);
      
      for (PinJoint joint : pinJoints)
      {
         YoBufferVariableEntry position = dataBuffer.getEntry(joint.getQYoVariable());
         YoBufferVariableEntry speed = dataBuffer.getEntry(joint.getQDYoVariable());
         YoBufferVariableEntry torque = dataBuffer.getEntry(joint.getTauYoVariable());

         writeJointDataColumn(jointDataSheet, column++, position, false);
         writeJointDataColumn(jointDataSheet, column++, speed, false);
         writeJointDataColumn(jointDataSheet, column++, speed, true);
         writeJointDataColumn(jointDataSheet, column++, torque, false);
         writeJointDataColumn(jointDataSheet, column++, torque, true);
         writeMechanicalPowerJointDataColumn(jointDataSheet, column++, speed, torque, joint.getName());
      }
   }

   private void writeJointDataColumn(WritableSheet dataSheet, int column, YoBufferVariableEntry dataBufferEntry, boolean unsigned)
   {
      int row = 0;
      String name = dataBufferEntry.getVariableName();
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

   private void writeMechanicalPowerJointDataColumn(WritableSheet dataSheet, int column, YoBufferVariableEntry speed, YoBufferVariableEntry torque, String jointName)
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

   private static void recursivelyAddPinJoints(Joint joint, List<PinJoint> pinJoints)
   {
      if (joint instanceof PinJoint)
         pinJoints.add((PinJoint) joint);
      else if (DEBUG && !(joint instanceof FloatingJoint))
         PrintTools.error("Joint " + joint.getName() + " not currently handled by " + DataExporterExcelWorkbookCreator.class.getSimpleName());

      for (Joint child : joint.getChildrenJoints())
      {
         recursivelyAddPinJoints(child, pinJoints);
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
