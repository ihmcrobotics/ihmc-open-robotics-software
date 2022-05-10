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
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.VoxelExtraData;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public class ReachabilityMapSpreadsheetExporterV0
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

   public static void exportVoxel3DGridToFile(String robotName,
                                              Class<?> classForFilePath,
                                              OneDoFJointBasics[] robotArmJoints,
                                              FramePose3DReadOnly controlFramePose,
                                              Voxel3DGrid gridToWrite)
         throws IOException
   {
      ReachabilityMapSpreadsheetExporterV0 writer = new ReachabilityMapSpreadsheetExporterV0(robotName, classForFilePath);
      writer.write(robotArmJoints, controlFramePose, gridToWrite);
      writer.exportAndClose();
   }

   public ReachabilityMapSpreadsheetExporterV0(String robotName, Class<?> classForFilePath) throws IOException
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

   public void write(OneDoFJointBasics[] robotArmJoints, FramePose3DReadOnly controlFramePose, Voxel3DGrid gridToWrite)
   {
      createDescriptionSheet(robotName, robotArmJoints, controlFramePose, gridToWrite);

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

   private void createDescriptionSheet(String robotName, OneDoFJointBasics[] robotArmJoints, FramePose3DReadOnly controlFramePose, Voxel3DGrid gridToWrite)
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

      for (int i = 0; i < robotArmJoints.length; i++)
      {
         currentRow.createCell(currentCellIndex).setCellValue(robotArmJoints[i].getName());
         positionLowerLimitRow.createCell(currentCellIndex).setCellValue(robotArmJoints[i].getJointLimitLower());
         positionUpperLimitRow.createCell(currentCellIndex).setCellValue(robotArmJoints[i].getJointLimitUpper());
         effortLowerLimitRow.createCell(currentCellIndex).setCellValue(robotArmJoints[i].getEffortLimitLower());
         effortUpperLimitRow.createCell(currentCellIndex++).setCellValue(robotArmJoints[i].getEffortLimitUpper());
      }

      currentRow = descriptionSheet.createRow(currentRowIndex++);
      FramePose3D controlFramePoseInParentJoint = new FramePose3D(controlFramePose);
      controlFramePoseInParentJoint.changeFrame(robotArmJoints[robotArmJoints.length - 1].getFrameAfterJoint());
      currentCellIndex = 0;
      currentRow.createCell(currentCellIndex++).setCellValue("Control frame pose in parent joint:");
      currentRow.createCell(currentCellIndex++).setCellValue("position (xyz)=");
      currentRow.createCell(currentCellIndex++).setCellValue(printPosition(controlFramePoseInParentJoint.getPosition()));
      currentRow.createCell(currentCellIndex++).setCellValue("orientation (ypr)=");
      currentRow.createCell(currentCellIndex++).setCellValue(printOrientation(controlFramePoseInParentJoint.getOrientation()));

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
