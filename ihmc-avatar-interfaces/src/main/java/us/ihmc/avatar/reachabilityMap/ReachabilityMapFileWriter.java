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

import us.ihmc.avatar.reachabilityMap.example.RobotArm;
import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape.SphereVoxelType;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class ReachabilityMapFileWriter
{
   private static final int MAX_NUMBER_OF_ROWS = 65535;

   private FileOutputStream fileOutputStream;
   private HSSFWorkbook workbook = new HSSFWorkbook();
   private HSSFSheet currentDataSheet;
   private int currentDataSheetNameIndex = 1;

   private int currentDataRow = 0;

   private final String robotName;

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

   public void initialize(OneDoFJointBasics[] robotArmJoints, Voxel3DGrid gridToWrite)
   {
      createDescriptionSheet(robotName, robotArmJoints, gridToWrite);

      addDataSheet();
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
      FramePose3D framePose = new FramePose3D();
      framePose.getOrientation().setYawPitchRoll(1.0, 0.8, -1.1);
      framePose.getPosition().set(3.1, 0.1, 1.0);
      System.out.println(framePose.getOrientation());

      RigidBodyTransform transformToParent = new RigidBodyTransform();
      framePose.get(transformToParent);
      ReferenceFrame gridFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("blop", ReferenceFrame.getWorldFrame(), transformToParent);
      SphereVoxelShape sphereVoxelShape = new SphereVoxelShape(gridFrame, 0.1, 10, 12, SphereVoxelType.graspOrigin);
      Voxel3DGrid voxel3dGrid = new Voxel3DGrid(gridFrame, sphereVoxelShape, 10, 0.1);
      RobotArm robot = new RobotArm();
      OneDoFJointBasics[] armJoints = MultiBodySystemTools.filterJoints(robot.getJacobian().getJointsInOrder(), OneDoFJointBasics.class);
      ReachabilityMapFileWriter reachabilityMapFileWriter = new ReachabilityMapFileWriter(robot.getName(), ReachabilityMapFileWriter.class);
      reachabilityMapFileWriter.initialize(armJoints, voxel3dGrid);

      for (int i = 0; i < 70000; i++)
         reachabilityMapFileWriter.registerReachablePose(0, 1, 2, 3, 4);
      reachabilityMapFileWriter.exportAndClose();
   }
}
