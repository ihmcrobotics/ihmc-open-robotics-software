package us.ihmc.avatar.reachabilityMap;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.List;

import org.apache.poi.hssf.usermodel.HSSFRow;
import org.apache.poi.hssf.usermodel.HSSFSheet;
import org.apache.poi.hssf.usermodel.HSSFWorkbook;

import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DData;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DKey;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.VoxelExtraData;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;

public class ReachabilityMapSpreadsheetExporter implements ReachabilityMapFileWriter
{
   private static final int MAX_NUMBER_OF_ROWS = 65535;

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

   public ReachabilityMapSpreadsheetExporter()
   {
   }

   @Override
   public void write(File file, ReachabilityMapRobotInformation robotInformation, Voxel3DGrid reachabilityMap)
   {
      createDescriptionSheet(robotInformation, reachabilityMap);

      String[] jointNames = robotInformation.getEvaluatedJoints().stream().map(JointDefinition::getName).toArray(String[]::new);

      for (int voxelIndex = 0; voxelIndex < reachabilityMap.getNumberOfVoxels(); voxelIndex++)
      {
         Voxel3DData voxel = reachabilityMap.getVoxel(voxelIndex);
         if (voxel == null)
            continue;

         writePositionData(jointNames, voxel);
         writeRayData(jointNames, voxel);
         writePoseData(jointNames, voxel);
      }

      FileOutputStream os = null;
      try
      {
         os = new FileOutputStream(file);
         workbook.write(os);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      finally
      {
         try
         {
            workbook.close();
            if (os != null)
               os.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
   }

   @Override
   public String getFileExtension()
   {
      return ".xls";
   }

   private void createDescriptionSheet(ReachabilityMapRobotInformation robotInformation, Voxel3DGrid reachabilityMap)
   {
      HSSFSheet descriptionSheet = workbook.createSheet("Description");
      int currentRowIndex = 0;
      int currentCellIndex = 0;
      HSSFRow currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue("Reachability Map for the robot:");
      currentRow.createCell(currentCellIndex++).setCellValue(robotInformation.getRobotDefinition().getName());

      currentCellIndex = 0;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue("Grid size = ");
      currentRow.createCell(currentCellIndex++).setCellValue(reachabilityMap.getGridSizeMeters());

      currentCellIndex = 0;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue("Number of voxels per dimension = ");
      currentRow.createCell(currentCellIndex++).setCellValue((double) reachabilityMap.getGridSizeVoxels());

      currentCellIndex = 0;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue("Voxel properties:");
      currentRow.createCell(currentCellIndex++).setCellValue("Size:");
      currentRow.createCell(currentCellIndex++).setCellValue(reachabilityMap.getVoxelSize());
      currentCellIndex = 1;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue("Number of rays:");
      currentRow.createCell(currentCellIndex++).setCellValue((double) reachabilityMap.getSphereVoxelShape().getNumberOfRays());
      currentCellIndex = 1;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue("Number of rotations per ray:");
      currentRow.createCell(currentCellIndex++).setCellValue((double) reachabilityMap.getSphereVoxelShape().getNumberOfRotationsAroundRay());

      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentCellIndex = 1;
      currentRow.createCell(currentCellIndex++).setCellValue("x");
      currentRow.createCell(currentCellIndex++).setCellValue("y");
      currentRow.createCell(currentCellIndex++).setCellValue("z");
      currentRow.createCell(currentCellIndex++).setCellValue("yaw");
      currentRow.createCell(currentCellIndex++).setCellValue("pitch");
      currentRow.createCell(currentCellIndex++).setCellValue("roll");
      currentCellIndex = 0;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue("Grid pose:");
      Pose3D gridPose = new Pose3D(reachabilityMap.getReferenceFrame().getTransformToRoot());
      currentRow.createCell(currentCellIndex++).setCellValue(gridPose.getX());
      currentRow.createCell(currentCellIndex++).setCellValue(gridPose.getY());
      currentRow.createCell(currentCellIndex++).setCellValue(gridPose.getZ());
      currentRow.createCell(currentCellIndex++).setCellValue(gridPose.getYaw());
      currentRow.createCell(currentCellIndex++).setCellValue(gridPose.getPitch());
      currentRow.createCell(currentCellIndex++).setCellValue(gridPose.getRoll());

      currentCellIndex = 0;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      HSSFRow positionLowerLimitRow = descriptionSheet.createRow(currentRowIndex++);
      HSSFRow positionUpperLimitRow = descriptionSheet.createRow(currentRowIndex++);
      HSSFRow effortLowerLimitRow = descriptionSheet.createRow(currentRowIndex++);
      HSSFRow effortUpperLimitRow = descriptionSheet.createRow(currentRowIndex++);

      currentRow.createCell(currentCellIndex).setCellValue("Kinematic chain joints:");
      positionLowerLimitRow.createCell(currentCellIndex).setCellValue("position lower limit:");
      positionUpperLimitRow.createCell(currentCellIndex).setCellValue("position upper limit:");
      effortLowerLimitRow.createCell(currentCellIndex).setCellValue("effort lower limit:");
      effortUpperLimitRow.createCell(currentCellIndex++).setCellValue("effort upper limit:");

      List<OneDoFJointDefinition> evaluatedJoints = robotInformation.getEvaluatedJoints();

      for (int i = 0; i < evaluatedJoints.size(); i++)
      {
         currentRow.createCell(currentCellIndex).setCellValue(evaluatedJoints.get(i).getName());
         positionLowerLimitRow.createCell(currentCellIndex).setCellValue(evaluatedJoints.get(i).getPositionLowerLimit());
         positionUpperLimitRow.createCell(currentCellIndex).setCellValue(evaluatedJoints.get(i).getPositionUpperLimit());
         effortLowerLimitRow.createCell(currentCellIndex).setCellValue(evaluatedJoints.get(i).getEffortLowerLimit());
         effortUpperLimitRow.createCell(currentCellIndex++).setCellValue(evaluatedJoints.get(i).getEffortUpperLimit());
      }

      Pose3DReadOnly controlFramePose = robotInformation.getControlFramePoseInParentJoint();
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentCellIndex = 1;
      currentRow.createCell(currentCellIndex++).setCellValue("x");
      currentRow.createCell(currentCellIndex++).setCellValue("y");
      currentRow.createCell(currentCellIndex++).setCellValue("z");
      currentRow.createCell(currentCellIndex++).setCellValue("yaw");
      currentRow.createCell(currentCellIndex++).setCellValue("pitch");
      currentRow.createCell(currentCellIndex++).setCellValue("roll");
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentCellIndex = 0;
      currentRow.createCell(currentCellIndex++).setCellValue("Control frame pose in parent joint:");
      currentRow.createCell(currentCellIndex++).setCellValue(controlFramePose.getX());
      currentRow.createCell(currentCellIndex++).setCellValue(controlFramePose.getY());
      currentRow.createCell(currentCellIndex++).setCellValue(controlFramePose.getZ());
      currentRow.createCell(currentCellIndex++).setCellValue(controlFramePose.getYaw());
      currentRow.createCell(currentCellIndex++).setCellValue(controlFramePose.getPitch());
      currentRow.createCell(currentCellIndex++).setCellValue(controlFramePose.getRoll());

   }

   private void writePositionData(String[] jointNames, Voxel3DData voxel3DData)
   {
      if (positionDataSheet == null || positionDataRow > MAX_NUMBER_OF_ROWS)
         addPositionDataSheet(jointNames);

      Voxel3DKey key = voxel3DData.getKey();
      VoxelExtraData extraData = voxel3DData.getPositionExtraData();

      HSSFRow row = positionDataSheet.createRow(positionDataRow++);
      int cellIndex = 0;
      row.createCell(cellIndex++).setCellValue((double) key.getX());
      row.createCell(cellIndex++).setCellValue((double) key.getY());
      row.createCell(cellIndex++).setCellValue((double) key.getZ());
      row.createCell(cellIndex++).setCellValue(extraData.getDesiredPosition().getX());
      row.createCell(cellIndex++).setCellValue(extraData.getDesiredPosition().getY());
      row.createCell(cellIndex++).setCellValue(extraData.getDesiredPosition().getZ());
      for (int i = 0; i < jointNames.length; i++)
         row.createCell(cellIndex++).setCellValue(extraData.getJointPositions()[i]);
      for (int i = 0; i < jointNames.length; i++)
         row.createCell(cellIndex++).setCellValue(extraData.getJointTorques()[i]);
   }

   private void writeRayData(String[] jointNames, Voxel3DData voxel3DData)
   {
      if (!voxel3DData.atLeastOneReachableRay())
         return;

      if (rayDataSheet == null)
         addRayDataSheet(jointNames);

      for (int rayIndex = 0; rayIndex < voxel3DData.getNumberOfRays(); rayIndex++)
      {
         if (!voxel3DData.isRayReachable(rayIndex))
            continue;

         if (rayDataRow > MAX_NUMBER_OF_ROWS)
            addRayDataSheet(jointNames);

         Voxel3DKey key = voxel3DData.getKey();
         VoxelExtraData extraData = voxel3DData.getRayExtraData(rayIndex);

         HSSFRow row = rayDataSheet.createRow(rayDataRow++);
         int cellIndex = 0;
         row.createCell(cellIndex++).setCellValue((double) key.getX());
         row.createCell(cellIndex++).setCellValue((double) key.getY());
         row.createCell(cellIndex++).setCellValue((double) key.getZ());
         row.createCell(cellIndex++).setCellValue((double) rayIndex);
         row.createCell(cellIndex++).setCellValue(extraData.getDesiredPosition().getX());
         row.createCell(cellIndex++).setCellValue(extraData.getDesiredPosition().getY());
         row.createCell(cellIndex++).setCellValue(extraData.getDesiredPosition().getZ());
         row.createCell(cellIndex++).setCellValue(extraData.getDesiredOrientation().getYaw());
         row.createCell(cellIndex++).setCellValue(extraData.getDesiredOrientation().getPitch());
         row.createCell(cellIndex++).setCellValue(extraData.getDesiredOrientation().getRoll());
         for (int i = 0; i < jointNames.length; i++)
            row.createCell(cellIndex++).setCellValue(extraData.getJointPositions()[i]);
         for (int i = 0; i < jointNames.length; i++)
            row.createCell(cellIndex++).setCellValue(extraData.getJointTorques()[i]);
      }
   }

   private void writePoseData(String[] jointNames, Voxel3DData voxel3DData)
   {
      if (!voxel3DData.atLeastOneReachablePose())
         return;

      if (poseDataSheet == null)
         addPoseDataSheet(jointNames);

      for (int rayIndex = 0; rayIndex < voxel3DData.getNumberOfRays(); rayIndex++)
      {
         for (int rotationIndex = 0; rotationIndex < voxel3DData.getNumberOfRotationsAroundRay(); rotationIndex++)
         {
            if (!voxel3DData.isPoseReachable(rayIndex, rotationIndex))
               continue;

            if (poseDataRow > MAX_NUMBER_OF_ROWS)
               addPoseDataSheet(jointNames);

            Voxel3DKey key = voxel3DData.getKey();
            VoxelExtraData extraData = voxel3DData.getPoseExtraData(rayIndex, rotationIndex);

            HSSFRow row = poseDataSheet.createRow(poseDataRow++);
            int cellIndex = 0;
            row.createCell(cellIndex++).setCellValue((double) key.getX());
            row.createCell(cellIndex++).setCellValue((double) key.getY());
            row.createCell(cellIndex++).setCellValue((double) key.getZ());
            row.createCell(cellIndex++).setCellValue((double) rayIndex);
            row.createCell(cellIndex++).setCellValue((double) rotationIndex);
            row.createCell(cellIndex++).setCellValue(extraData.getDesiredPosition().getX());
            row.createCell(cellIndex++).setCellValue(extraData.getDesiredPosition().getY());
            row.createCell(cellIndex++).setCellValue(extraData.getDesiredPosition().getZ());
            row.createCell(cellIndex++).setCellValue(extraData.getDesiredOrientation().getYaw());
            row.createCell(cellIndex++).setCellValue(extraData.getDesiredOrientation().getPitch());
            row.createCell(cellIndex++).setCellValue(extraData.getDesiredOrientation().getRoll());
            for (int i = 0; i < jointNames.length; i++)
               row.createCell(cellIndex++).setCellValue(extraData.getJointPositions()[i]);
            for (int i = 0; i < jointNames.length; i++)
               row.createCell(cellIndex++).setCellValue(extraData.getJointTorques()[i]);
         }
      }
   }

   private void addPositionDataSheet(String[] jointNames)
   {
      positionDataSheet = workbook.createSheet(getPositionDataSheetName(positionDataSheetNameIndex++));
      positionDataRow = 0;
      HSSFRow headerRow = positionDataSheet.createRow(positionDataRow++);
      int currentCellIndex = 0;
      headerRow.createCell(currentCellIndex++).setCellValue("xIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("yIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("zIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("desired x");
      headerRow.createCell(currentCellIndex++).setCellValue("desired y");
      headerRow.createCell(currentCellIndex++).setCellValue("desired z");
      for (String jointName : jointNames)
         headerRow.createCell(currentCellIndex++).setCellValue("q_" + jointName);
      for (String jointName : jointNames)
         headerRow.createCell(currentCellIndex++).setCellValue("tau_" + jointName);
   }

   private void addRayDataSheet(String[] jointNames)
   {
      rayDataSheet = workbook.createSheet(getRayDataSheetName(rayDataSheetNameIndex++));
      rayDataRow = 0;
      HSSFRow headerRow = rayDataSheet.createRow(rayDataRow++);
      int currentCellIndex = 0;
      headerRow.createCell(currentCellIndex++).setCellValue("xIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("yIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("zIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("rayIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("desired x");
      headerRow.createCell(currentCellIndex++).setCellValue("desired y");
      headerRow.createCell(currentCellIndex++).setCellValue("desired z");
      headerRow.createCell(currentCellIndex++).setCellValue("desired yaw");
      headerRow.createCell(currentCellIndex++).setCellValue("desired pitch");
      headerRow.createCell(currentCellIndex++).setCellValue("desired roll");
      for (String jointName : jointNames)
         headerRow.createCell(currentCellIndex++).setCellValue("q_" + jointName);
      for (String jointName : jointNames)
         headerRow.createCell(currentCellIndex++).setCellValue("tau_" + jointName);
   }

   private void addPoseDataSheet(String[] jointNames)
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
      headerRow.createCell(currentCellIndex++).setCellValue("desired x");
      headerRow.createCell(currentCellIndex++).setCellValue("desired y");
      headerRow.createCell(currentCellIndex++).setCellValue("desired z");
      headerRow.createCell(currentCellIndex++).setCellValue("desired yaw");
      headerRow.createCell(currentCellIndex++).setCellValue("desired pitch");
      headerRow.createCell(currentCellIndex++).setCellValue("desired roll");
      for (String jointName : jointNames)
         headerRow.createCell(currentCellIndex++).setCellValue("q_" + jointName);
      for (String jointName : jointNames)
         headerRow.createCell(currentCellIndex++).setCellValue("tau_" + jointName);
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
}
