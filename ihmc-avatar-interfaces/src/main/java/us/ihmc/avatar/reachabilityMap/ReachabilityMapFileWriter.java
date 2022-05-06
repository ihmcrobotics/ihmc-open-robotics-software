package us.ihmc.avatar.reachabilityMap;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;

import org.apache.commons.lang3.StringUtils;
import org.apache.poi.hssf.usermodel.HSSFRow;
import org.apache.poi.hssf.usermodel.HSSFSheet;
import org.apache.poi.hssf.usermodel.HSSFWorkbook;

import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DData;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DKey;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.VoxelJointData;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public class ReachabilityMapFileWriter
{
   private static final int MAX_NUMBER_OF_ROWS = 65535;

   private FileOutputStream fileOutputStream;
   private HSSFWorkbook workbook = new HSSFWorkbook();

   private HSSFSheet positionDataSheet;
   private HSSFSheet rayDataSheet;
   private HSSFSheet poseDataSheet;

   private int positionDataSheetNameIndex = 1;
   private int rayDataSheetNameIndex = 1;
   private int poseDataSheetNameIndex = 1;

   private int positionDataRow = 0;
   private int rayDataRow = 0;
   private int poseDataRow = 0;

   private final String robotName;

   public static void exportVoxel3DGridToFile(String robotName, Class<?> classForFilePath, OneDoFJointBasics[] robotArmJoints, Voxel3DGrid gridToWrite)
         throws IOException
   {
      ReachabilityMapFileWriter writer = new ReachabilityMapFileWriter(robotName, classForFilePath);
      writer.write(robotArmJoints, gridToWrite);
      writer.exportAndClose();
   }

   public ReachabilityMapFileWriter(String robotName, Class<?> classForFilePath) throws IOException
   {
      this.robotName = robotName;
      String fileName = prependDateToFileName(robotName) + ".xls";
      Path filePath = deriveResourcesPath(classForFilePath);
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
   }

   public void write(OneDoFJointBasics[] robotArmJoints, Voxel3DGrid gridToWrite)
   {
      createDescriptionSheet(robotName, robotArmJoints, gridToWrite);

      for (int voxelIndex = 0; voxelIndex < gridToWrite.getNumberOfVoxels(); voxelIndex++)
      {
         writeVoxelData(gridToWrite.getVoxel(voxelIndex));
      }
   }

   public static Path deriveResourcesPath(Class<?> clazz)
   {
      List<String> pathNames = new ArrayList<String>();

      String[] packageNames = clazz.getPackage().getName().split("\\.");

      pathNames.addAll(Arrays.asList(packageNames));
      pathNames.add(StringUtils.uncapitalize(clazz.getSimpleName()));

      return Paths.get("resources", pathNames.toArray(new String[0]));
   }

   private void createDescriptionSheet(String robotName, OneDoFJointBasics[] robotArmJoints, Voxel3DGrid gridToWrite)
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
      currentRow.createCell(currentCellIndex++).setCellValue(gridToWrite.getGridSizeMeters());

      currentCellIndex = 0;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue("Number of voxels per dimension = ");
      currentRow.createCell(currentCellIndex++).setCellValue((double) gridToWrite.getGridSizeVoxels());

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

      FramePose3D poseToParent = new FramePose3D(gridToWrite.getReferenceFrame());
      if (!gridReferenceFrame.isWorldFrame())
         poseToParent.changeFrame(gridToWrite.getReferenceFrame().getParent());
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      poseToParent.get(transformToParent);

      currentCellIndex = 1;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue("Transform to parent frame:");
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.getM00());
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.getM01());
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.getM02());
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.getM03());

      currentCellIndex = 2;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.getM10());
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.getM11());
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.getM12());
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.getM13());

      currentCellIndex = 2;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.getM20());
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.getM21());
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.getM22());
      currentRow.createCell(currentCellIndex++).setCellValue(transformToParent.getM23());

      currentCellIndex = 1;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      HSSFRow lowerLimitRow = descriptionSheet.createRow(currentRowIndex++);
      HSSFRow upperLimitRow = descriptionSheet.createRow(currentRowIndex++);

      currentRow.createCell(currentCellIndex).setCellValue("Kinematic chain joints:");
      lowerLimitRow.createCell(currentCellIndex).setCellValue("lower limit:");
      upperLimitRow.createCell(currentCellIndex++).setCellValue("upper limit:");

      for (int i = 0; i < robotArmJoints.length; i++)
      {
         currentRow.createCell(currentCellIndex).setCellValue(robotArmJoints[i].getName());
         lowerLimitRow.createCell(currentCellIndex).setCellValue(robotArmJoints[i].getJointLimitLower());
         upperLimitRow.createCell(currentCellIndex++).setCellValue(robotArmJoints[i].getJointLimitUpper());
      }
   }

   private static String prependDateToFileName(String fileName)
   {
      DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss_");
      Date date = new Date();
      String dateAsString = dateFormat.format(date);

      return dateAsString + fileName;
   }

   public void writeVoxelData(Voxel3DData voxel3DData)
   {
      if (voxel3DData == null)
         return;
      writePositionData(voxel3DData);
      writeRayData(voxel3DData);
      writePoseData(voxel3DData);
   }

   private void writePositionData(Voxel3DData voxel3DData)
   {
      if (poseDataSheet == null || positionDataRow > MAX_NUMBER_OF_ROWS)
         addPositionDataSheet();

      Voxel3DKey key = voxel3DData.getKey();
      VoxelJointData positionJointData = voxel3DData.getPositionJointData();

      HSSFRow row = positionDataSheet.createRow(positionDataRow++);
      int cellIndex = 0;
      row.createCell(cellIndex++).setCellValue((double) key.getX());
      row.createCell(cellIndex++).setCellValue((double) key.getY());
      row.createCell(cellIndex++).setCellValue((double) key.getZ());
      row.createCell(cellIndex++).setCellValue(Arrays.toString(positionJointData.getJointPositions()));
      row.createCell(cellIndex++).setCellValue(Arrays.toString(positionJointData.getJointTorques()));
   }

   private void writeRayData(Voxel3DData voxel3DData)
   {
      if (!voxel3DData.atLeastOneReachableRay())
         return;

      if (rayDataSheet == null)
         addRayDataSheet();

      for (int rayIndex = 0; rayIndex < voxel3DData.getNumberOfRays(); rayIndex++)
      {
         if (rayDataRow > MAX_NUMBER_OF_ROWS)
            addRayDataSheet();

         Voxel3DKey key = voxel3DData.getKey();
         VoxelJointData rayJointData = voxel3DData.getRayJointData(rayIndex);

         HSSFRow row = rayDataSheet.createRow(rayDataRow++);
         int cellIndex = 0;
         row.createCell(cellIndex++).setCellValue((double) key.getX());
         row.createCell(cellIndex++).setCellValue((double) key.getY());
         row.createCell(cellIndex++).setCellValue((double) key.getZ());
         row.createCell(cellIndex++).setCellValue((double) rayIndex);
         row.createCell(cellIndex++).setCellValue(Arrays.toString(rayJointData.getJointPositions()));
         row.createCell(cellIndex++).setCellValue(Arrays.toString(rayJointData.getJointTorques()));
      }
   }

   private void writePoseData(Voxel3DData voxel3DData)
   {
      if (!voxel3DData.atLeastOneReachablePose())
         return;

      if (poseDataSheet == null)
         addPoseDataSheet();

      for (int rayIndex = 0; rayIndex < voxel3DData.getNumberOfRays(); rayIndex++)
      {
         for (int rotationIndex = 0; rotationIndex < voxel3DData.getNumberOfRotationsAroundRay(); rotationIndex++)
         {
            if (poseDataRow > MAX_NUMBER_OF_ROWS)
               addPoseDataSheet();

            Voxel3DKey key = voxel3DData.getKey();
            VoxelJointData poseJointData = voxel3DData.getPoseJointData(rayIndex, rotationIndex);

            HSSFRow row = poseDataSheet.createRow(poseDataRow++);
            int cellIndex = 0;
            row.createCell(cellIndex++).setCellValue((double) key.getX());
            row.createCell(cellIndex++).setCellValue((double) key.getY());
            row.createCell(cellIndex++).setCellValue((double) key.getZ());
            row.createCell(cellIndex++).setCellValue((double) rayIndex);
            row.createCell(cellIndex++).setCellValue((double) rotationIndex);
            row.createCell(cellIndex++).setCellValue(Arrays.toString(poseJointData.getJointPositions()));
            row.createCell(cellIndex++).setCellValue(Arrays.toString(poseJointData.getJointTorques()));
         }
      }
   }

   private void addPositionDataSheet()
   {
      positionDataSheet = workbook.createSheet(getPositionDataSheetName(positionDataSheetNameIndex++));
      positionDataRow = 0;
      HSSFRow headerRow = positionDataSheet.createRow(positionDataRow++);
      int currentCellIndex = 0;
      headerRow.createCell(currentCellIndex++).setCellValue("xIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("yIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("zIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("positions");
      headerRow.createCell(currentCellIndex++).setCellValue("torques");
   }

   private void addRayDataSheet()
   {
      rayDataSheet = workbook.createSheet(getRayDataSheetName(rayDataSheetNameIndex++));
      rayDataRow = 0;
      HSSFRow headerRow = positionDataSheet.createRow(rayDataRow++);
      int currentCellIndex = 0;
      headerRow.createCell(currentCellIndex++).setCellValue("xIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("yIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("zIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("rayIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("positions");
      headerRow.createCell(currentCellIndex++).setCellValue("torques");
   }

   private void addPoseDataSheet()
   {
      poseDataSheet = workbook.createSheet(getPoseDataSheetName(poseDataSheetNameIndex++));
      poseDataRow = 0;
      HSSFRow headerRow = poseDataSheet.createRow(poseDataRow++);
      int currentCellIndex = 0;
      headerRow.createCell(currentCellIndex++).setCellValue("xIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("yIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("zIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("rayIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("rotationAroundRayIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("positions");
      headerRow.createCell(currentCellIndex++).setCellValue("torques");
   }

   public static String getPositionDataSheetName(int sheetIndex)
   {
      return "Position Data " + sheetIndex;
   }

   public static String getRayDataSheetName(int sheetIndex)
   {
      return "Ray Data " + sheetIndex;
   }

   public static String getPoseDataSheetName(int sheetIndex)
   {
      return "Pose Data " + sheetIndex;
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
}
