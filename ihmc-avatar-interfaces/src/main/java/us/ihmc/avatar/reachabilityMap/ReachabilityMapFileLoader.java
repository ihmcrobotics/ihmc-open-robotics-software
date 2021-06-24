package us.ihmc.avatar.reachabilityMap;

import java.awt.FileDialog;
import java.awt.Frame;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.stream.Stream;

import org.apache.poi.hssf.usermodel.HSSFCell;
import org.apache.poi.hssf.usermodel.HSSFRow;
import org.apache.poi.hssf.usermodel.HSSFSheet;
import org.apache.poi.hssf.usermodel.HSSFWorkbook;
import org.apache.poi.poifs.filesystem.NPOIFSFileSystem;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import cern.colt.Arrays;
import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape.SphereVoxelType;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class ReachabilityMapFileLoader
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private NPOIFSFileSystem fileSystem;
   private HSSFWorkbook workBookToLoad;
   private final Voxel3DGrid loadedGrid;

   public ReachabilityMapFileLoader(String robotName, RigidBodyBasics rootBody)
   {
      this(robotName, rootBody, null);
   }

   public ReachabilityMapFileLoader(String robotName, RigidBodyBasics rootBody, HumanoidReferenceFrames referenceFrames)
   {
      this(selectionFileDialog(), robotName, rootBody, referenceFrames);
   }

   public ReachabilityMapFileLoader(File fileToLoad, String robotName, RigidBodyBasics rootBody)
   {
      this(selectionFileDialog(), robotName, rootBody, null);
   }

   public ReachabilityMapFileLoader(File fileToLoad, String robotName, RigidBodyBasics rootBody, HumanoidReferenceFrames referenceFrames)
   {
      try
      {
         fileSystem = new NPOIFSFileSystem(fileToLoad);
         workBookToLoad = new HSSFWorkbook(fileSystem.getRoot(), true);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      HSSFSheet descriptionSheet = workBookToLoad.getSheet("Description");

      checkRobotMatchesData(robotName, rootBody, descriptionSheet);

      ReferenceFrame gridFrame = createGridReferenceFrame(rootBody, referenceFrames, descriptionSheet);

      loadedGrid = createGrid(descriptionSheet, gridFrame);

      loadData();

      close();
   }

   public static File selectionFileDialog()
   {
      
      final FileDialog fileDialog = new FileDialog((Frame) null, "Choose reachability map to load");
      fileDialog.setMode(FileDialog.LOAD);

      fileDialog.setVisible(true);
      String directory = fileDialog.getDirectory();
      String filename = fileDialog.getFile();
      fileDialog.dispose();
      if (directory != null && filename != null)
      {
         return new File(directory, filename);
      }
      else
      {
         return null;
      }
   }

   private void checkRobotMatchesData(String robotName, RigidBodyBasics rootBody, HSSFSheet descriptionSheet)
   {
      String robotNameInWorkbook = descriptionSheet.getRow(0).getCell(1).getStringCellValue();

      if (!robotName.equals(robotNameInWorkbook))
      {
         throw new RuntimeException("Trying to load the data for another robot: Loading data for " + robotName + ", workbook contains data for " + robotNameInWorkbook);
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

      JointBasics[] joints = Stream.of(MultiBodySystemTools.collectSubtreeJoints(rootBody)).filter(joint -> jointNames.contains(joint.getName())).toArray(JointBasics[]::new);
      OneDoFJointBasics[] oneDoFJoints = MultiBodySystemTools.filterJoints(joints, OneDoFJointBasics.class);

      if (oneDoFJoints.length != jointNames.size())
      {
         throw new RuntimeException("Could not find all the joints, expected:\n " + jointNames + "\nwas:\n" + Arrays.toString(oneDoFJoints));
      }
   }

   private ReferenceFrame createGridReferenceFrame(RigidBodyBasics rootBody, HumanoidReferenceFrames referenceFrames, HSSFSheet descriptionSheet)
   {
      String gridFrameName = descriptionSheet.getRow(6).getCell(2).getStringCellValue();
      String parentFrameName = descriptionSheet.getRow(7).getCell(2).getStringCellValue();
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
      RigidBodyTransform transformToParentFrame = new RigidBodyTransform(transformToParentFrameAsDenseMatrix);
      ReferenceFrame parentFrame = searchParentFrameInCommonRobotFrames(parentFrameName, referenceFrames, rootBody);

      ReferenceFrame gridFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(gridFrameName, parentFrame, transformToParentFrame);
      return gridFrame;
   }

   private Voxel3DGrid createGrid(HSSFSheet descriptionSheet, ReferenceFrame gridFrame)
   {
      int numberOfVoxelsPerDimension = (int) descriptionSheet.getRow(2).getCell(1).getNumericCellValue();
      double voxelSize = descriptionSheet.getRow(3).getCell(2).getNumericCellValue();
      int numberOfRaysPerVoxel = (int) descriptionSheet.getRow(4).getCell(2).getNumericCellValue();
      int numberOfRotationsPerRay = (int) descriptionSheet.getRow(5).getCell(2).getNumericCellValue();
      SphereVoxelShape sphereVoxelShape = new SphereVoxelShape(gridFrame, voxelSize, numberOfRaysPerVoxel, numberOfRotationsPerRay, SphereVoxelType.graspOrigin);
      return new Voxel3DGrid(gridFrame, sphereVoxelShape, numberOfVoxelsPerDimension, voxelSize);
   }

   private void loadData()
   {
      HSSFRow currentRow;
      HSSFSheet currentDataSheet;

      int currentDataSheetNameIndex = 1;
      currentDataSheet = workBookToLoad.getSheet("Data" + currentDataSheetNameIndex++);

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
            int rotationAroundRayIndex = (int) currentRow.getCell(cellIndex++).getNumericCellValue();

            loadedGrid.registerReachablePose(xIndex, yIndex, zIndex, rayIndex, rotationAroundRayIndex);
            currentRow = currentDataSheet.getRow(currentRowIndex++);
         }

         currentDataSheet = workBookToLoad.getSheet("Data" + currentDataSheetNameIndex++);
      }
   }

   public Voxel3DGrid getLoadedGrid()
   {
      return loadedGrid;
   }

   private ReferenceFrame searchParentFrameInCommonRobotFrames(String parentFrameName, HumanoidReferenceFrames referenceFrames, RigidBodyBasics rootBody)
   {
      if (parentFrameName.equals(worldFrame.getName()))
         return worldFrame;

      if (parentFrameName.equals(rootBody.getBodyFixedFrame().getName()))
         return rootBody.getBodyFixedFrame();

      for (JointBasics joint : rootBody.childrenSubtreeIterable())
      {
         ReferenceFrame frameAfterJoint = joint.getFrameAfterJoint();
         ReferenceFrame frameBeforeJoint = joint.getFrameBeforeJoint();
         RigidBodyBasics successor = joint.getSuccessor();

         if (parentFrameName.equals(frameAfterJoint.getName()))
            return frameAfterJoint;
         
         if (parentFrameName.equals(frameBeforeJoint.getName()))
            return frameBeforeJoint;

         if (successor != null)
         {
            ReferenceFrame bodyFixedFrame = successor.getBodyFixedFrame();
            if (parentFrameName.equals(bodyFixedFrame.getName()))
               return bodyFixedFrame;
         }
      }

      if (referenceFrames == null)
         return null;

      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      if (parentFrameName.equals(pelvisZUpFrame.getName()))
         return pelvisZUpFrame;

      ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      if (parentFrameName.equals(midFeetZUpFrame.getName()))
         return midFeetZUpFrame;

      ReferenceFrame centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      if (parentFrameName.equals(centerOfMassFrame.getName()))
         return centerOfMassFrame;

      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         if (parentFrameName.equals(soleFrame.getName()))
            return soleFrame;

         ReferenceFrame ankleZUpFrame = referenceFrames.getAnkleZUpFrame(robotSide);
         if (parentFrameName.equals(ankleZUpFrame.getName()))
            return ankleZUpFrame;

         ReferenceFrame handFrame = referenceFrames.getHandFrame(robotSide);
         if (parentFrameName.equals(handFrame.getName()))
            return handFrame;
      }

      return null;
   }

   private void close()
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
