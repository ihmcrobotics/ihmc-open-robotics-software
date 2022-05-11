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
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DData;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
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

      HSSFSheet descriptionSheet = workbook.getSheet("Description");

      checkRobotMatchesData(robotInformation, descriptionSheet);
      loadControlFramePose(robotInformation, descriptionSheet);

      Voxel3DGrid reachabilityMap = createGrid(descriptionSheet);
      loadReachabilityMapData(reachabilityMap, workbook);

      try
      {
         fileSystem.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      return reachabilityMap;
   }

   private static void loadControlFramePose(ReachabilityMapRobotInformation robotInformation, HSSFSheet descriptionSheet)
   {
      HSSFCell positionCell = descriptionSheet.getRow(16).getCell(2);
      HSSFCell orientationCell = descriptionSheet.getRow(16).getCell(4);

      Pose3D controlFramePose = new Pose3D();
      controlFramePose.getPosition().set(parseDoubleArray(positionCell.getStringCellValue()));
      controlFramePose.getOrientation().set(new YawPitchRoll(parseDoubleArray(orientationCell.getStringCellValue())));
      robotInformation.setControlFramePoseInParentJoint(controlFramePose);
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

      int currentIndexValue = 2;
      HSSFRow currentRow = descriptionSheet.getRow(11);
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
      DMatrixRMaj transformToParentFrameAsDenseMatrix = CommonOps_DDRM.identity(4);
      int row = 0;
      for (int rowIndex = 8; rowIndex < 11; rowIndex++)
      {
         int col = 0;
         for (int cellIndex = 2; cellIndex < 6; cellIndex++)
         {
            transformToParentFrameAsDenseMatrix.set(row, col, descriptionSheet.getRow(rowIndex).getCell(cellIndex).getNumericCellValue());
            col++;
         }
         row++;
      }

      int numberOfVoxelsPerDimension = (int) descriptionSheet.getRow(2).getCell(1).getNumericCellValue();
      double voxelSize = descriptionSheet.getRow(3).getCell(2).getNumericCellValue();
      int numberOfRaysPerVoxel = (int) descriptionSheet.getRow(4).getCell(2).getNumericCellValue();
      int numberOfRotationsPerRay = (int) descriptionSheet.getRow(5).getCell(2).getNumericCellValue();

      Voxel3DGrid grid = Voxel3DGrid.newVoxel3DGrid(numberOfVoxelsPerDimension, voxelSize, numberOfRaysPerVoxel, numberOfRotationsPerRay);
      grid.setGridPose(new RigidBodyTransform(transformToParentFrameAsDenseMatrix));
      return grid;
   }

   private static void loadReachabilityMapData(Voxel3DGrid reachabilityMap, HSSFWorkbook workbook)
   {
      loadPositionData(reachabilityMap, workbook);
      loadRayData(reachabilityMap, workbook);
      loadPoseData(reachabilityMap, workbook);
   }

   private static void loadPositionData(Voxel3DGrid reachabilityMap, HSSFWorkbook workbook)
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
            Point3D desiredPosition = new Point3D(parseDoubleArray(currentRow.getCell(cellIndex++).getStringCellValue()));
            float[] jointPositions = parseFloatArray(currentRow.getCell(cellIndex++).getStringCellValue());
            float[] jointTorques = parseFloatArray(currentRow.getCell(cellIndex++).getStringCellValue());
            Voxel3DData voxel = reachabilityMap.getOrCreateVoxel(xIndex, yIndex, zIndex);
            voxel.registerReachablePosition(desiredPosition, jointPositions, jointTorques);
            currentRow = currentDataSheet.getRow(currentRowIndex++);
         }

         currentDataSheet = workbook.getSheet(ReachabilityMapSpreadsheetExporter.getPositionDataSheetName(currentDataSheetNameIndex++));
      }
   }

   private static void loadRayData(Voxel3DGrid reachabilityMap, HSSFWorkbook workbook)
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
            Point3D desiredPosition = new Point3D(parseDoubleArray(currentRow.getCell(cellIndex++).getStringCellValue()));
            YawPitchRoll desiredOrientation = new YawPitchRoll(parseDoubleArray(currentRow.getCell(cellIndex++).getStringCellValue()));
            float[] jointPositions = parseFloatArray(currentRow.getCell(cellIndex++).getStringCellValue());
            float[] jointTorques = parseFloatArray(currentRow.getCell(cellIndex++).getStringCellValue());

            Voxel3DData voxel = reachabilityMap.getOrCreateVoxel(xIndex, yIndex, zIndex);
            voxel.registerReachableRay(rayIndex, new Pose3D(desiredPosition, desiredOrientation), jointPositions, jointTorques);
            currentRow = currentDataSheet.getRow(currentRowIndex++);
         }

         currentDataSheet = workbook.getSheet(ReachabilityMapSpreadsheetExporter.getRayDataSheetName(currentDataSheetNameIndex++));
      }
   }

   private static void loadPoseData(Voxel3DGrid reachabilityMap, HSSFWorkbook workbook)
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
            Point3D desiredPosition = new Point3D(parseDoubleArray(currentRow.getCell(cellIndex++).getStringCellValue()));
            YawPitchRoll desiredOrientation = new YawPitchRoll(parseDoubleArray(currentRow.getCell(cellIndex++).getStringCellValue()));
            float[] jointPositions = parseFloatArray(currentRow.getCell(cellIndex++).getStringCellValue());
            float[] jointTorques = parseFloatArray(currentRow.getCell(cellIndex++).getStringCellValue());

            Voxel3DData voxel = reachabilityMap.getOrCreateVoxel(xIndex, yIndex, zIndex);
            voxel.registerReachablePose(rayIndex, rotationIndex, new Pose3D(desiredPosition, desiredOrientation), jointPositions, jointTorques);
            currentRow = currentDataSheet.getRow(currentRowIndex++);
         }

         currentDataSheet = workbook.getSheet(ReachabilityMapSpreadsheetExporter.getPoseDataSheetName(currentDataSheetNameIndex++));
      }
   }

   private static float[] parseFloatArray(String string)
   {
      if (string == null)
         return null;

      string = string.replace("[", "").replace("]", "").trim();

      if (string.isEmpty())
         return null;

      String[] elements = string.split(",");
      float[] array = new float[elements.length];

      for (int i = 0; i < elements.length; i++)
      {
         array[i] = Float.parseFloat(elements[i].trim());
      }
      return array;
   }

   private static double[] parseDoubleArray(String string)
   {
      if (string == null)
         return null;

      string = string.replace("[", "").replace("]", "").trim();

      if (string.isEmpty())
         return null;

      String[] elements = string.split(",");
      double[] array = new double[elements.length];

      for (int i = 0; i < elements.length; i++)
      {
         array[i] = Double.parseDouble(elements[i].trim());
      }
      return array;
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
