package us.ihmc.avatar.reachabilityMap;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.function.Consumer;
import java.util.stream.Stream;

import org.apache.poi.hssf.usermodel.HSSFCell;
import org.apache.poi.hssf.usermodel.HSSFRow;
import org.apache.poi.hssf.usermodel.HSSFSheet;
import org.apache.poi.hssf.usermodel.HSSFWorkbook;
import org.apache.poi.poifs.filesystem.NPOIFSFileSystem;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import cern.colt.Arrays;
import javafx.stage.FileChooser;
import javafx.stage.FileChooser.ExtensionFilter;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DData;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.javafx.JavaFXMissingTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;

public class ReachabilityMapSpreadsheetImporterV0
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private NPOIFSFileSystem fileSystem;
   private HSSFWorkbook workBookToLoad;
   private final Voxel3DGrid loadedGrid;

   private Consumer<Voxel3DData> voxel3DPositionDataLoadedListener = null;
   private Consumer<Voxel3DData> voxel3DRayDataLoadedListener = null;
   private Consumer<Voxel3DData> voxel3DPoseDataLoadedListener = null;

   private final FramePose3D controlFramePose;

   public ReachabilityMapSpreadsheetImporterV0(String robotName, RigidBodyBasics rootBody)
   {
      this(robotName, rootBody, null);
   }

   public ReachabilityMapSpreadsheetImporterV0(String robotName, RigidBodyBasics rootBody, Collection<ReferenceFrame> referenceFrames)
   {
      this(selectionFileDialog(), robotName, rootBody, referenceFrames);
   }

   public ReachabilityMapSpreadsheetImporterV0(File fileToLoad, String robotName, RigidBodyBasics rootBody)
   {
      this(selectionFileDialog(), robotName, rootBody, null);
   }

   public ReachabilityMapSpreadsheetImporterV0(File fileToLoad, String robotName, RigidBodyBasics rootBody, Collection<ReferenceFrame> referenceFrames)
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

      loadedGrid = createGrid(rootBody, referenceFrames, descriptionSheet);

      controlFramePose = loadControlFramePose(rootBody, descriptionSheet);

      loadData();

      close();
   }

   public FramePose3D getControlFramePose()
   {
      return controlFramePose;
   }

   private FramePose3D loadControlFramePose(RigidBodyBasics rootBody, HSSFSheet descriptionSheet)
   {
      RigidBodyBasics endEffector = MultiBodySystemTools.collectSubtreeEndEffectors(rootBody)[0];

      HSSFCell positionCell = descriptionSheet.getRow(16).getCell(2);
      HSSFCell orientationCell = descriptionSheet.getRow(16).getCell(4);

      FramePose3D controlFramePose = new FramePose3D(endEffector.getParentJoint().getFrameAfterJoint());
      controlFramePose.getPosition().set(parseDoubleArray(positionCell.getStringCellValue()));
      controlFramePose.getOrientation().set(new YawPitchRoll(parseDoubleArray(orientationCell.getStringCellValue())));

      return controlFramePose;
   }

   public void setVoxel3DPositionDataLoadedListener(Consumer<Voxel3DData> voxel3dPositionDataLoadedListener)
   {
      voxel3DPositionDataLoadedListener = voxel3dPositionDataLoadedListener;
   }

   public void setVoxel3DRayDataLoadedListener(Consumer<Voxel3DData> voxel3dRayDataLoadedListener)
   {
      voxel3DRayDataLoadedListener = voxel3dRayDataLoadedListener;
   }

   public void setVoxel3DPoseDataLoadedListener(Consumer<Voxel3DData> voxel3dPoseDataLoadedListener)
   {
      voxel3DPoseDataLoadedListener = voxel3dPoseDataLoadedListener;
   }

   public static File selectionFileDialog()
   {
      JavaFXMissingTools.startup();
      return JavaFXMissingTools.runAndWait(() ->
      {
         FileChooser fileChooser = new FileChooser();
         fileChooser.setTitle("Choose reachability map to load");
         File initialDirectory = SessionVisualizerIOTools.getDefaultFilePath("humanoid-reachability-map-load");
         if (initialDirectory == null)
            initialDirectory = new File(".");
         fileChooser.setInitialDirectory(initialDirectory);
         fileChooser.getExtensionFilters().add(new ExtensionFilter("Spreadsheet", "*.xls"));
         fileChooser.getExtensionFilters().add(new ExtensionFilter("All Files", "*.*"));
         File result = fileChooser.showOpenDialog(null);
         if (result != null)
            SessionVisualizerIOTools.setDefaultFilePath("humanoid-reachability-map-load", result.getParentFile());
         return result;
      });
   }

   private void checkRobotMatchesData(String robotName, RigidBodyBasics rootBody, HSSFSheet descriptionSheet)
   {
      String robotNameInWorkbook = descriptionSheet.getRow(0).getCell(1).getStringCellValue();

      if (!robotName.equals(robotNameInWorkbook))
      {
         throw new RuntimeException("Trying to load the data for another robot: Loading data for " + robotName + ", workbook contains data for "
               + robotNameInWorkbook);
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

      JointBasics[] joints = Stream.of(MultiBodySystemTools.collectSubtreeJoints(rootBody)).filter(joint -> jointNames.contains(joint.getName()))
                                   .toArray(JointBasics[]::new);
      OneDoFJointBasics[] oneDoFJoints = MultiBodySystemTools.filterJoints(joints, OneDoFJointBasics.class);

      if (oneDoFJoints.length != jointNames.size())
      {
         throw new RuntimeException("Could not find all the joints, expected:\n " + jointNames + "\nwas:\n" + Arrays.toString(oneDoFJoints));
      }
   }

   private Voxel3DGrid createGrid(RigidBodyBasics rootBody, Collection<ReferenceFrame> referenceFrames, HSSFSheet descriptionSheet)
   {
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
      ReferenceFrame parentFrame = searchParentFrameInCommonRobotFrames(parentFrameName, referenceFrames, rootBody);

      int numberOfVoxelsPerDimension = (int) descriptionSheet.getRow(2).getCell(1).getNumericCellValue();
      double voxelSize = descriptionSheet.getRow(3).getCell(2).getNumericCellValue();
      int numberOfRaysPerVoxel = (int) descriptionSheet.getRow(4).getCell(2).getNumericCellValue();
      int numberOfRotationsPerRay = (int) descriptionSheet.getRow(5).getCell(2).getNumericCellValue();

      Voxel3DGrid grid = Voxel3DGrid.newVoxel3DGrid(parentFrame, numberOfVoxelsPerDimension, voxelSize, numberOfRaysPerVoxel, numberOfRotationsPerRay);
      grid.setGridPose(new RigidBodyTransform(transformToParentFrameAsDenseMatrix));
      return grid;
   }

   private void loadData()
   {
      loadPositionData();
      loadRayData();
      loadPoseData();
   }

   private void loadPositionData()
   {
      HSSFRow currentRow;
      HSSFSheet currentDataSheet;

      int currentDataSheetNameIndex = 1;
      currentDataSheet = workBookToLoad.getSheet(ReachabilityMapSpreadsheetExporterV0.getPositionDataSheetName(currentDataSheetNameIndex++));

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
            Voxel3DData voxel = loadedGrid.getOrCreateVoxel(xIndex, yIndex, zIndex);
            voxel.registerReachablePosition(desiredPosition, jointPositions, jointTorques);

            if (voxel3DPositionDataLoadedListener != null)
               voxel3DPositionDataLoadedListener.accept(voxel);

            currentRow = currentDataSheet.getRow(currentRowIndex++);
         }

         currentDataSheet = workBookToLoad.getSheet(ReachabilityMapSpreadsheetExporterV0.getPositionDataSheetName(currentDataSheetNameIndex++));
      }
   }

   private void loadRayData()
   {
      HSSFRow currentRow;
      HSSFSheet currentDataSheet;

      int currentDataSheetNameIndex = 1;
      currentDataSheet = workBookToLoad.getSheet(ReachabilityMapSpreadsheetExporterV0.getRayDataSheetName(currentDataSheetNameIndex++));

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

            Voxel3DData voxel = loadedGrid.getOrCreateVoxel(xIndex, yIndex, zIndex);
            voxel.registerReachableRay(rayIndex, new Pose3D(desiredPosition, desiredOrientation), jointPositions, jointTorques);

            if (voxel3DRayDataLoadedListener != null)
               voxel3DRayDataLoadedListener.accept(voxel);

            currentRow = currentDataSheet.getRow(currentRowIndex++);
         }

         currentDataSheet = workBookToLoad.getSheet(ReachabilityMapSpreadsheetExporterV0.getRayDataSheetName(currentDataSheetNameIndex++));
      }
   }

   private void loadPoseData()
   {
      HSSFRow currentRow;
      HSSFSheet currentDataSheet;

      int currentDataSheetNameIndex = 1;
      currentDataSheet = workBookToLoad.getSheet(ReachabilityMapSpreadsheetExporterV0.getPoseDataSheetName(currentDataSheetNameIndex++));

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

            Voxel3DData voxel = loadedGrid.getOrCreateVoxel(xIndex, yIndex, zIndex);
            voxel.registerReachablePose(rayIndex, rotationIndex, new Pose3D(desiredPosition, desiredOrientation), jointPositions, jointTorques);

            if (voxel3DPoseDataLoadedListener != null)
               voxel3DPoseDataLoadedListener.accept(voxel);

            currentRow = currentDataSheet.getRow(currentRowIndex++);
         }

         currentDataSheet = workBookToLoad.getSheet(ReachabilityMapSpreadsheetExporterV0.getPoseDataSheetName(currentDataSheetNameIndex++));
      }
   }

   private float[] parseFloatArray(String string)
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

   private double[] parseDoubleArray(String string)
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

   public Voxel3DGrid getLoadedGrid()
   {
      return loadedGrid;
   }

   private ReferenceFrame searchParentFrameInCommonRobotFrames(String parentFrameName, Collection<ReferenceFrame> referenceFrames, RigidBodyBasics rootBody)
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

      if (referenceFrames == null || referenceFrames.isEmpty())
         return null;

      for (ReferenceFrame referenceFrame : referenceFrames)
      {
         if (parentFrameName.equals(referenceFrame.getName()))
            return referenceFrame;
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
