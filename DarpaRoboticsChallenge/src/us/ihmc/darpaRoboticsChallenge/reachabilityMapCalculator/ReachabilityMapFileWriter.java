package us.ihmc.darpaRoboticsChallenge.reachabilityMapCalculator;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.file.Path;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

import org.apache.poi.hssf.usermodel.HSSFRow;
import org.apache.poi.hssf.usermodel.HSSFSheet;
import org.apache.poi.hssf.usermodel.HSSFWorkbook;

import us.ihmc.darpaRoboticsChallenge.reachabilityMapCalculator.example.RobotArm;
import us.ihmc.darpaRoboticsChallenge.reachabilityMapCalculator.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.darpaRoboticsChallenge.reachabilityMapCalculator.voxelPrimitiveShapes.SphereVoxelShape.SphereVoxelType;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.tools.io.files.FileTools;

public class ReachabilityMapFileWriter
{
   private static final int MAX_NUMBER_OF_ROWS = 65535;

   private FileOutputStream fileOutputStream;
   private HSSFWorkbook workbook = new HSSFWorkbook();
   private HSSFSheet currentDataSheet;
   private int currentDataSheetNameIndex = 1;
   
   private int currentDataRow = 0;

   public ReachabilityMapFileWriter(String robotName, OneDoFJoint[] robotArmJoints, Voxel3DGrid gridToWrite, Class<?> classForFilePath)
   {
      String fileName = prependDateToFileName(robotName) + ".xls";
      Path filePath = FileTools.deriveResourcesPath(classForFilePath);
      FileTools.ensureDirectoryExists(filePath);
      filePath = filePath.resolve(fileName);
      try
      {
         fileOutputStream = new FileOutputStream(filePath.toFile());
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();
      }
      createDescriptionSheet(robotName, robotArmJoints, gridToWrite);

      addDataSheet();
   }

   private void createDescriptionSheet(String robotName, OneDoFJoint[] robotArmJoints, Voxel3DGrid gridToWrite)
   {
      HSSFSheet descriptionSheet = workbook.createSheet("Description");
      int currentRowIndex = 0;
      int currentCellIndex = 0;
      HSSFRow currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue("Reachability Map for the robot:");
      currentRow.createCell(currentCellIndex++).setCellValue(robotName);

      currentCellIndex = 0;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue("Grid size = ");
      currentRow.createCell(currentCellIndex++).setCellValue(gridToWrite.getGridSize());

      currentCellIndex = 0;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue("Number of voxels per dimension = ");
      currentRow.createCell(currentCellIndex++).setCellValue((double) gridToWrite.getNumberOfVoxelsPerDimension());

      currentCellIndex = 0;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue("Voxel properties:");
      currentRow.createCell(currentCellIndex++).setCellValue("Size:");
      currentRow.createCell(currentCellIndex++).setCellValue(gridToWrite.getVoxelSize());
      currentCellIndex = 1;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue("Number of rays:");
      currentRow.createCell(currentCellIndex++).setCellValue((double) gridToWrite.getSphereVoxelShape().getNumberOfRays());
      currentCellIndex = 1;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue("Number of rotations per ray:");
      currentRow.createCell(currentCellIndex++).setCellValue((double) gridToWrite.getSphereVoxelShape().getNumberOfRotationsAroundRay());

      currentCellIndex = 0;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue("Grid reference frame information:");
      currentRow.createCell(currentCellIndex++).setCellValue("Name:");
      ReferenceFrame gridReferenceFrame = gridToWrite.getReferenceFrame();
      currentRow.createCell(currentCellIndex++).setCellValue(gridReferenceFrame.getName());

      currentCellIndex = 1;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue("Parent frame:");
      String parentFrameName = gridReferenceFrame.isWorldFrame() ? "null" : gridReferenceFrame.getParent().getName();
      currentRow.createCell(currentCellIndex++).setCellValue(parentFrameName);

      FramePose poseToParent = new FramePose(gridToWrite.getReferenceFrame());
      if (!gridReferenceFrame.isWorldFrame())
         poseToParent.changeFrame(gridToWrite.getReferenceFrame().getParent());
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      poseToParent.getPose(transformToParent);

      currentCellIndex = 1;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue("Transform to parent frame:");
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.mat00);
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.mat01);
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.mat02);
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.mat03);

      currentCellIndex = 2;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.mat10);
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.mat11);
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.mat12);
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.mat13);

      currentCellIndex = 2;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.mat20);
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.mat21);
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.mat22);
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.mat23);

      currentCellIndex = 1;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue("Kinematic chain joints:");
      for (int i = 0; i < robotArmJoints.length; i++)
         currentRow.createCell(currentCellIndex++).setCellValue(robotArmJoints[i].getName());
   }

   private static String prependDateToFileName(String fileName)
   {
      DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss_");
      Date date = new Date();
      String dateAsString = dateFormat.format(date);

      return dateAsString + fileName;
   }

   public void registerReachablePose(int xIndex, int yIndex, int zIndex, int rayIndex, int rotationAroundRayIndex)
   {
      if (currentDataRow > MAX_NUMBER_OF_ROWS)
      {
         addDataSheet();
      }

      HSSFRow row = currentDataSheet.createRow(currentDataRow++);
      int cellIndex = 0;
      row.createCell(cellIndex++).setCellValue((double) xIndex);
      row.createCell(cellIndex++).setCellValue((double) yIndex);
      row.createCell(cellIndex++).setCellValue((double) zIndex);
      row.createCell(cellIndex++).setCellValue((double) rayIndex);
      row.createCell(cellIndex++).setCellValue((double) rotationAroundRayIndex);
   }

   private void addDataSheet()
   {
      currentDataSheet = workbook.createSheet("Data" + currentDataSheetNameIndex++);
      currentDataRow = 0;
      HSSFRow headerRow = currentDataSheet.createRow(currentDataRow++);
      int currentCellIndex = 0;
      headerRow.createCell(currentCellIndex++).setCellValue("xIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("yIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("zIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("rayIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("rotationAroundRayIndex");
   }

   public void exportAndClose()
   {
      try
      {
         workbook.write(fileOutputStream);
         fileOutputStream.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public static void main(String[] args) throws IOException
   {
      FramePose framePose = new FramePose();
      framePose.setOrientation(1.0, 0.8, -1.1);
      framePose.setPosition(3.1, 0.1, 1.0);
      System.out.println(framePose.getFrameOrientationCopy().toStringAsQuaternion());

      RigidBodyTransform transformToParent = new RigidBodyTransform();
      framePose.getPose(transformToParent);
      ReferenceFrame gridFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("blop", ReferenceFrame.getWorldFrame(), transformToParent);
      SphereVoxelShape sphereVoxelShape = new SphereVoxelShape(gridFrame, 0.1, 10, 12, SphereVoxelType.graspOrigin);
      Voxel3DGrid voxel3dGrid = new Voxel3DGrid(gridFrame, sphereVoxelShape, 10, 0.1);
      RobotArm robot = new RobotArm();
      OneDoFJoint[] armJoints = ScrewTools.filterJoints(robot.getJacobian().getJointsInOrder(), OneDoFJoint.class);
      ReachabilityMapFileWriter reachabilityMapFileWriter = new ReachabilityMapFileWriter(robot.getName(), armJoints, voxel3dGrid,
            ReachabilityMapFileWriter.class);
      
      for (int i = 0; i < 70000; i++)
         reachabilityMapFileWriter.registerReachablePose(0, 1, 2, 3, 4);
      reachabilityMapFileWriter.exportAndClose();
   }
}
