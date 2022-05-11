package us.ihmc.avatar.reachabilityMap;

import java.io.IOException;
import java.io.OutputStream;
import java.util.Arrays;
import java.util.List;

import org.apache.poi.hssf.usermodel.HSSFRow;
import org.apache.poi.hssf.usermodel.HSSFSheet;
import org.apache.poi.hssf.usermodel.HSSFWorkbook;

import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DData;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DKey;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.VoxelExtraData;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;

public class ReachabilityMapSpreadsheetExporterV0 implements ReachabilityMapFileWriter
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

   public ReachabilityMapSpreadsheetExporterV0()
   {
   }

   @Override
   public void write(OutputStream os, ReachabilityMapRobotInformation robotInformation, Voxel3DGrid reachabilityMap)
   {
      createDescriptionSheet(robotInformation, reachabilityMap);

      for (int voxelIndex = 0; voxelIndex < reachabilityMap.getNumberOfVoxels(); voxelIndex++)
      {
         Voxel3DData voxel = reachabilityMap.getVoxel(voxelIndex);
         if (voxel == null)
            continue;

         writePositionData(voxel);
         writeRayData(voxel);
         writePoseData(voxel);
      }

      try
      {
         workbook.write(os);
         os.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
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

      currentCellIndex = 0;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue("Grid reference frame information:");
      currentRow.createCell(currentCellIndex++).setCellValue("Name:");
      ReferenceFrame gridReferenceFrame = reachabilityMap.getReferenceFrame();
      currentRow.createCell(currentCellIndex++).setCellValue(gridReferenceFrame.getName());

      currentCellIndex = 1;
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentRow.createCell(currentCellIndex++).setCellValue("Parent frame:");
      String parentFrameName = gridReferenceFrame.isWorldFrame() ? "null" : gridReferenceFrame.getParent().getName();
      currentRow.createCell(currentCellIndex++).setCellValue(parentFrameName);

      FramePose3D poseToParent = new FramePose3D(reachabilityMap.getReferenceFrame());
      if (!gridReferenceFrame.isWorldFrame())
         poseToParent.changeFrame(reachabilityMap.getReferenceFrame().getParent());
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

      Pose3DReadOnly controlFramePoseInParentJoint = robotInformation.getControlFramePoseInParentJoint();
      currentRow = descriptionSheet.createRow(currentRowIndex++);
      currentCellIndex = 0;
      currentRow.createCell(currentCellIndex++).setCellValue("Control frame pose in parent joint:");
      currentRow.createCell(currentCellIndex++).setCellValue("position (xyz)=");
      currentRow.createCell(currentCellIndex++).setCellValue(printPosition(controlFramePoseInParentJoint.getPosition()));
      currentRow.createCell(currentCellIndex++).setCellValue("orientation (ypr)=");
      currentRow.createCell(currentCellIndex++).setCellValue(printOrientation(controlFramePoseInParentJoint.getOrientation()));

   }

   private void writePositionData(Voxel3DData voxel3DData)
   {
      if (positionDataSheet == null || positionDataRow > MAX_NUMBER_OF_ROWS)
         addPositionDataSheet();

      Voxel3DKey key = voxel3DData.getKey();
      VoxelExtraData positionExtraData = voxel3DData.getPositionExtraData();

      HSSFRow row = positionDataSheet.createRow(positionDataRow++);
      int cellIndex = 0;
      row.createCell(cellIndex++).setCellValue((double) key.getX());
      row.createCell(cellIndex++).setCellValue((double) key.getY());
      row.createCell(cellIndex++).setCellValue((double) key.getZ());
      row.createCell(cellIndex++).setCellValue(printPosition(positionExtraData.getDesiredPosition()));
      row.createCell(cellIndex++).setCellValue(Arrays.toString(positionExtraData.getJointPositions()));
      row.createCell(cellIndex++).setCellValue(Arrays.toString(positionExtraData.getJointTorques()));
   }

   private void writeRayData(Voxel3DData voxel3DData)
   {
      if (!voxel3DData.atLeastOneReachableRay())
         return;

      if (rayDataSheet == null)
         addRayDataSheet();

      for (int rayIndex = 0; rayIndex < voxel3DData.getNumberOfRays(); rayIndex++)
      {
         if (!voxel3DData.isRayReachable(rayIndex))
            continue;

         if (rayDataRow > MAX_NUMBER_OF_ROWS)
            addRayDataSheet();

         Voxel3DKey key = voxel3DData.getKey();
         VoxelExtraData rayExtraData = voxel3DData.getRayExtraData(rayIndex);

         HSSFRow row = rayDataSheet.createRow(rayDataRow++);
         int cellIndex = 0;
         row.createCell(cellIndex++).setCellValue((double) key.getX());
         row.createCell(cellIndex++).setCellValue((double) key.getY());
         row.createCell(cellIndex++).setCellValue((double) key.getZ());
         row.createCell(cellIndex++).setCellValue((double) rayIndex);
         row.createCell(cellIndex++).setCellValue(printPosition(rayExtraData.getDesiredPosition()));
         row.createCell(cellIndex++).setCellValue(printOrientation(rayExtraData.getDesiredOrientation()));
         row.createCell(cellIndex++).setCellValue(Arrays.toString(rayExtraData.getJointPositions()));
         row.createCell(cellIndex++).setCellValue(Arrays.toString(rayExtraData.getJointTorques()));
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
            if (!voxel3DData.isPoseReachable(rayIndex, rotationIndex))
               continue;

            if (poseDataRow > MAX_NUMBER_OF_ROWS)
               addPoseDataSheet();

            Voxel3DKey key = voxel3DData.getKey();
            VoxelExtraData poseExtraData = voxel3DData.getPoseExtraData(rayIndex, rotationIndex);

            HSSFRow row = poseDataSheet.createRow(poseDataRow++);
            int cellIndex = 0;
            row.createCell(cellIndex++).setCellValue((double) key.getX());
            row.createCell(cellIndex++).setCellValue((double) key.getY());
            row.createCell(cellIndex++).setCellValue((double) key.getZ());
            row.createCell(cellIndex++).setCellValue((double) rayIndex);
            row.createCell(cellIndex++).setCellValue((double) rotationIndex);
            row.createCell(cellIndex++).setCellValue(printPosition(poseExtraData.getDesiredPosition()));
            row.createCell(cellIndex++).setCellValue(printOrientation(poseExtraData.getDesiredOrientation()));
            row.createCell(cellIndex++).setCellValue(Arrays.toString(poseExtraData.getJointPositions()));
            row.createCell(cellIndex++).setCellValue(Arrays.toString(poseExtraData.getJointTorques()));
         }
      }
   }

   private static String printPosition(Point3DReadOnly point)
   {
      return EuclidCoreIOTools.getStringOf("[", "]", ", ", point.getX(), point.getY(), point.getZ());
   }

   private static String printOrientation(Orientation3DReadOnly orientation)
   {
      return EuclidCoreIOTools.getStringOf("[", "]", ", ", orientation.getYaw(), orientation.getPitch(), orientation.getRoll());
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
      headerRow.createCell(currentCellIndex++).setCellValue("desired position (xyz)");
      headerRow.createCell(currentCellIndex++).setCellValue("joint positions");
      headerRow.createCell(currentCellIndex++).setCellValue("joint torques");
   }

   private void addRayDataSheet()
   {
      rayDataSheet = workbook.createSheet(getRayDataSheetName(rayDataSheetNameIndex++));
      rayDataRow = 0;
      HSSFRow headerRow = rayDataSheet.createRow(rayDataRow++);
      int currentCellIndex = 0;
      headerRow.createCell(currentCellIndex++).setCellValue("xIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("yIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("zIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("rayIndex");
      headerRow.createCell(currentCellIndex++).setCellValue("desired position (xyz)");
      headerRow.createCell(currentCellIndex++).setCellValue("desired orientation (ypr)");
      headerRow.createCell(currentCellIndex++).setCellValue("joint positions");
      headerRow.createCell(currentCellIndex++).setCellValue("joint torques");
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
      headerRow.createCell(currentCellIndex++).setCellValue("desired position (xyz)");
      headerRow.createCell(currentCellIndex++).setCellValue("desired orientation (ypr)");
      headerRow.createCell(currentCellIndex++).setCellValue("joint positions");
      headerRow.createCell(currentCellIndex++).setCellValue("joint torques");
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
