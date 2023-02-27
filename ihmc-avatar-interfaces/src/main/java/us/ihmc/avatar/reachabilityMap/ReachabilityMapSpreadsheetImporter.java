package us.ihmc.avatar.reachabilityMap;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.apache.poi.hssf.usermodel.HSSFCell;
import org.apache.poi.hssf.usermodel.HSSFRow;
import org.apache.poi.hssf.usermodel.HSSFSheet;
import org.apache.poi.hssf.usermodel.HSSFWorkbook;
import org.apache.poi.poifs.filesystem.NPOIFSFileSystem;

import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DData;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;

public class ReachabilityMapSpreadsheetImporter implements ReachabilityMapFileReader
{
   public ReachabilityMapSpreadsheetImporter()
   {
   }

   @Override
   public Voxel3DGrid read(File fileToLoad, ReachabilityMapRobotInformation robotInformation)
   {
      NPOIFSFileSystem fileSystem;
      HSSFWorkbook workbook;
      try
      {
         fileSystem = new NPOIFSFileSystem(fileToLoad);
         workbook = new HSSFWorkbook(fileSystem.getRoot(), true);
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }

      try
      {
         HSSFSheet descriptionSheet = workbook.getSheet("Description");

         checkRobotMatchesData(robotInformation, descriptionSheet);
         loadControlFramePose(robotInformation, descriptionSheet);

         Voxel3DGrid reachabilityMap = createGrid(descriptionSheet);
         loadReachabilityMapData(robotInformation.getEvaluatedJoints().size(), reachabilityMap, workbook);
         return reachabilityMap;
      }
      catch (Exception e)
      {
         e.printStackTrace();
         return null;
      }
      finally
      {
         try
         {
            fileSystem.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
   }

   private static void loadControlFramePose(ReachabilityMapRobotInformation robotInformation, HSSFSheet descriptionSheet)
   {
      HSSFRow row = descriptionSheet.getRow(14);
      int cellIndex = 1;
      double x = row.getCell(cellIndex++).getNumericCellValue();
      double y = row.getCell(cellIndex++).getNumericCellValue();
      double z = row.getCell(cellIndex++).getNumericCellValue();
      double yaw = row.getCell(cellIndex++).getNumericCellValue();
      double pitch = row.getCell(cellIndex++).getNumericCellValue();
      double roll = row.getCell(cellIndex++).getNumericCellValue();
      robotInformation.setControlFramePoseInParentJoint(new Pose3D(x, y, z, yaw, pitch, roll));
   }

   private static void checkRobotMatchesData(ReachabilityMapRobotInformation robotInformation, HSSFSheet descriptionSheet)
   {
      String robotNameInWorkbook = descriptionSheet.getRow(0).getCell(1).getStringCellValue();

      if (!robotInformation.getRobotDefinition().getName().equals(robotNameInWorkbook))
      {
         throw new RuntimeException("Trying to load the data for another robot: Loading data for " + robotInformation.getRobotDefinition().getName()
               + ", workbook contains data for " + robotNameInWorkbook);
      }

      ArrayList<String> jointNames = new ArrayList<>();

      int currentIndexValue = 1;
      HSSFRow currentRow = descriptionSheet.getRow(8);
      HSSFCell currentCell = currentRow.getCell(currentIndexValue);

      while (currentCell != null)
      {
         jointNames.add(currentCell.getStringCellValue());
         currentIndexValue++;
         currentCell = currentRow.getCell(currentIndexValue);
      }

      boolean jointsMatch = true;
      List<OneDoFJointDefinition> evaluatedJoints = robotInformation.getEvaluatedJoints();

      if (jointNames.size() != evaluatedJoints.size())
      {
         jointsMatch = false;
      }
      else
      {
         for (int i = 0; i < evaluatedJoints.size(); i++)
         {
            if (!jointNames.get(i).equals(evaluatedJoints.get(i).getName()))
            {
               jointsMatch = false;
               break;
            }
         }
      }

      if (!jointsMatch)
      {
         throw new RuntimeException("Could not find all the joints, expected:\n " + jointNames + "\nwas:\n["
               + EuclidCoreIOTools.getCollectionString(", ", evaluatedJoints, j -> j.getName()) + "]");
      }
   }

   private static Voxel3DGrid createGrid(HSSFSheet descriptionSheet)
   {
      HSSFRow poseRow = descriptionSheet.getRow(7);
      int poseCellIndex = 1;
      double x = poseRow.getCell(poseCellIndex++).getNumericCellValue();
      double y = poseRow.getCell(poseCellIndex++).getNumericCellValue();
      double z = poseRow.getCell(poseCellIndex++).getNumericCellValue();
      double yaw = poseRow.getCell(poseCellIndex++).getNumericCellValue();
      double pitch = poseRow.getCell(poseCellIndex++).getNumericCellValue();
      double roll = poseRow.getCell(poseCellIndex++).getNumericCellValue();

      int numberOfVoxelsPerDimension = (int) descriptionSheet.getRow(2).getCell(1).getNumericCellValue();
      double voxelSize = descriptionSheet.getRow(3).getCell(2).getNumericCellValue();
      int numberOfRaysPerVoxel = (int) descriptionSheet.getRow(4).getCell(2).getNumericCellValue();
      int numberOfRotationsPerRay = (int) descriptionSheet.getRow(5).getCell(2).getNumericCellValue();

      Voxel3DGrid grid = Voxel3DGrid.newVoxel3DGrid(numberOfVoxelsPerDimension, voxelSize, numberOfRaysPerVoxel, numberOfRotationsPerRay);
      grid.setGridPose(new Pose3D(x, y, z, yaw, pitch, roll));
      return grid;
   }

   private static void loadReachabilityMapData(int numberOfJoints, Voxel3DGrid reachabilityMap, HSSFWorkbook workbook)
   {
      loadPositionData(numberOfJoints, reachabilityMap, workbook);
      loadRayData(numberOfJoints, reachabilityMap, workbook);
      loadPoseData(numberOfJoints, reachabilityMap, workbook);
   }

   private static void loadPositionData(int numberOfJoints, Voxel3DGrid reachabilityMap, HSSFWorkbook workbook)
   {
      HSSFRow currentRow;
      HSSFSheet currentDataSheet;

      int currentDataSheetNameIndex = 1;
      currentDataSheet = workbook.getSheet(ReachabilityMapSpreadsheetExporter.getPositionDataSheetName(currentDataSheetNameIndex++));

      while (currentDataSheet != null)
      {
         int currentRowIndex = 1;
         currentRow = currentDataSheet.getRow(currentRowIndex++);

         int cellIndex = 0;

         while (currentRow != null)
         {
            cellIndex = 0;
            int xIndex = (int) currentRow.getCell(cellIndex++).getNumericCellValue();
            int yIndex = (int) currentRow.getCell(cellIndex++).getNumericCellValue();
            int zIndex = (int) currentRow.getCell(cellIndex++).getNumericCellValue();
            Point3D desiredPosition = new Point3D();
            desiredPosition.setX(currentRow.getCell(cellIndex++).getNumericCellValue());
            desiredPosition.setY(currentRow.getCell(cellIndex++).getNumericCellValue());
            desiredPosition.setZ(currentRow.getCell(cellIndex++).getNumericCellValue());
            float[] jointPositions = new float[numberOfJoints];
            for (int i = 0; i < numberOfJoints; i++)
               jointPositions[i] = (float) currentRow.getCell(cellIndex++).getNumericCellValue();
            float[] jointTorques = new float[numberOfJoints];
            for (int i = 0; i < numberOfJoints; i++)
               jointTorques[i] = (float) currentRow.getCell(cellIndex++).getNumericCellValue();
            Voxel3DData voxel = reachabilityMap.getOrCreateVoxel(xIndex, yIndex, zIndex);
            voxel.registerReachablePosition(desiredPosition, jointPositions, jointTorques);
            currentRow = currentDataSheet.getRow(currentRowIndex++);
         }

         currentDataSheet = workbook.getSheet(ReachabilityMapSpreadsheetExporter.getPositionDataSheetName(currentDataSheetNameIndex++));
      }
   }

   private static void loadRayData(int numberOfJoints, Voxel3DGrid reachabilityMap, HSSFWorkbook workbook)
   {
      HSSFRow currentRow;
      HSSFSheet currentDataSheet;

      int currentDataSheetNameIndex = 1;
      currentDataSheet = workbook.getSheet(ReachabilityMapSpreadsheetExporter.getRayDataSheetName(currentDataSheetNameIndex++));

      while (currentDataSheet != null)
      {
         int currentRowIndex = 1;
         currentRow = currentDataSheet.getRow(currentRowIndex++);

         int cellIndex = 0;

         while (currentRow != null)
         {
            cellIndex = 0;
            int xIndex = (int) currentRow.getCell(cellIndex++).getNumericCellValue();
            int yIndex = (int) currentRow.getCell(cellIndex++).getNumericCellValue();
            int zIndex = (int) currentRow.getCell(cellIndex++).getNumericCellValue();
            int rayIndex = (int) currentRow.getCell(cellIndex++).getNumericCellValue();
            Point3D desiredPosition = new Point3D();
            desiredPosition.setX(currentRow.getCell(cellIndex++).getNumericCellValue());
            desiredPosition.setY(currentRow.getCell(cellIndex++).getNumericCellValue());
            desiredPosition.setZ(currentRow.getCell(cellIndex++).getNumericCellValue());
            YawPitchRoll desiredOrientation = new YawPitchRoll();
            desiredOrientation.setYaw(currentRow.getCell(cellIndex++).getNumericCellValue());
            desiredOrientation.setPitch(currentRow.getCell(cellIndex++).getNumericCellValue());
            desiredOrientation.setRoll(currentRow.getCell(cellIndex++).getNumericCellValue());
            float[] jointPositions = new float[numberOfJoints];
            for (int i = 0; i < numberOfJoints; i++)
               jointPositions[i] = (float) currentRow.getCell(cellIndex++).getNumericCellValue();
            float[] jointTorques = new float[numberOfJoints];
            for (int i = 0; i < numberOfJoints; i++)
               jointTorques[i] = (float) currentRow.getCell(cellIndex++).getNumericCellValue();

            Voxel3DData voxel = reachabilityMap.getOrCreateVoxel(xIndex, yIndex, zIndex);
            voxel.registerReachableRay(rayIndex, new Pose3D(desiredPosition, desiredOrientation), jointPositions, jointTorques);
            currentRow = currentDataSheet.getRow(currentRowIndex++);
         }

         currentDataSheet = workbook.getSheet(ReachabilityMapSpreadsheetExporter.getRayDataSheetName(currentDataSheetNameIndex++));
      }
   }

   private static void loadPoseData(int numberOfJoints, Voxel3DGrid reachabilityMap, HSSFWorkbook workbook)
   {
      HSSFRow currentRow;
      HSSFSheet currentDataSheet;

      int currentDataSheetNameIndex = 1;
      currentDataSheet = workbook.getSheet(ReachabilityMapSpreadsheetExporter.getPoseDataSheetName(currentDataSheetNameIndex++));

      while (currentDataSheet != null)
      {
         int currentRowIndex = 1;
         currentRow = currentDataSheet.getRow(currentRowIndex++);

         int cellIndex = 0;

         while (currentRow != null)
         {
            cellIndex = 0;
            int xIndex = (int) currentRow.getCell(cellIndex++).getNumericCellValue();
            int yIndex = (int) currentRow.getCell(cellIndex++).getNumericCellValue();
            int zIndex = (int) currentRow.getCell(cellIndex++).getNumericCellValue();
            int rayIndex = (int) currentRow.getCell(cellIndex++).getNumericCellValue();
            int rotationIndex = (int) currentRow.getCell(cellIndex++).getNumericCellValue();
            Point3D desiredPosition = new Point3D();
            desiredPosition.setX(currentRow.getCell(cellIndex++).getNumericCellValue());
            desiredPosition.setY(currentRow.getCell(cellIndex++).getNumericCellValue());
            desiredPosition.setZ(currentRow.getCell(cellIndex++).getNumericCellValue());
            YawPitchRoll desiredOrientation = new YawPitchRoll();
            desiredOrientation.setYaw(currentRow.getCell(cellIndex++).getNumericCellValue());
            desiredOrientation.setPitch(currentRow.getCell(cellIndex++).getNumericCellValue());
            desiredOrientation.setRoll(currentRow.getCell(cellIndex++).getNumericCellValue());
            float[] jointPositions = new float[numberOfJoints];
            for (int i = 0; i < numberOfJoints; i++)
               jointPositions[i] = (float) currentRow.getCell(cellIndex++).getNumericCellValue();
            float[] jointTorques = new float[numberOfJoints];
            for (int i = 0; i < numberOfJoints; i++)
               jointTorques[i] = (float) currentRow.getCell(cellIndex++).getNumericCellValue();

            Voxel3DData voxel = reachabilityMap.getOrCreateVoxel(xIndex, yIndex, zIndex);
            voxel.registerReachablePose(rayIndex, rotationIndex, new Pose3D(desiredPosition, desiredOrientation), jointPositions, jointTorques);
            currentRow = currentDataSheet.getRow(currentRowIndex++);
         }

         currentDataSheet = workbook.getSheet(ReachabilityMapSpreadsheetExporter.getPoseDataSheetName(currentDataSheetNameIndex++));
      }
   }

   @Override
   public String getFileType()
   {
      return "Spreadsheet";
   }

   @Override
   public String getFileExtension()
   {
      return ".xls";
   }
}
