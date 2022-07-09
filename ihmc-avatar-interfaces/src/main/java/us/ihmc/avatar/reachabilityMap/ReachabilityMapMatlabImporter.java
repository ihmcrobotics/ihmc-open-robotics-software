package us.ihmc.avatar.reachabilityMap;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import us.hebi.matlab.mat.format.Mat5;
import us.hebi.matlab.mat.format.Mat5File;
import us.hebi.matlab.mat.types.Cell;
import us.hebi.matlab.mat.types.MatFile.Entry;
import us.hebi.matlab.mat.types.Matrix;
import us.hebi.matlab.mat.types.Struct;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DData;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;

public class ReachabilityMapMatlabImporter implements ReachabilityMapFileReader
{
   public ReachabilityMapMatlabImporter()
   {
   }

   @Override
   public Voxel3DGrid read(File fileToLoad, ReachabilityMapRobotInformation robotInformation)
   {
      Mat5File matFile;

      try
      {
         matFile = Mat5.readFromFile(fileToLoad);
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }

      List<Entry> entries = new ArrayList<>();

      for (Entry entry : matFile.getEntries())
         entries.add(entry);

      Struct descriptionStruct = matFile.getStruct("Description");
      checkRobotMatchesData(robotInformation, descriptionStruct);
      loadControlFramePose(robotInformation, descriptionStruct);

      Voxel3DGrid reachabilityMap = createGrid(descriptionStruct);

      loadPositionData(reachabilityMap, matFile.getStruct("PositionReachData"));
      loadRayData(reachabilityMap, matFile.getStruct("RayReachData"));
      loadPoseData(reachabilityMap, matFile.getStruct("PoseReachData"));

      return reachabilityMap;
   }

   private static void loadControlFramePose(ReachabilityMapRobotInformation robotInformation, Struct descriptionStruct)
   {
      Matrix controlFramePoseField = descriptionStruct.getMatrix("controlFramePoseXYZYPR");
      int index = 0;
      double x = controlFramePoseField.getDouble(index++);
      double y = controlFramePoseField.getDouble(index++);
      double z = controlFramePoseField.getDouble(index++);
      double yaw = controlFramePoseField.getDouble(index++);
      double pitch = controlFramePoseField.getDouble(index++);
      double roll = controlFramePoseField.getDouble(index++);
      robotInformation.setControlFramePoseInParentJoint(new Pose3D(x, y, z, yaw, pitch, roll));
   }

   private static void checkRobotMatchesData(ReachabilityMapRobotInformation robotInformation, Struct descriptionStruct)
   {
      String robotNameInFile = descriptionStruct.getChar("robotName").getString();

      if (!robotInformation.getRobotDefinition().getName().equals(robotNameInFile))
      {
         LogTools.warn("Robot name mismatch: expected {}, was: {}", robotInformation.getRobotDefinition().getName(), robotNameInFile);
      }

      ArrayList<String> jointNames = new ArrayList<>();

      Cell jointNamesCell = descriptionStruct.getCell("jointNames");
      for (int i = 0; i < jointNamesCell.getNumElements(); i++)
      {
         jointNames.add(jointNamesCell.getChar(i).getString());
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

   private Voxel3DGrid createGrid(Struct descriptionStruct)
   {
      Matrix gridPoseField = descriptionStruct.getMatrix("gridPoseXYZYPR");
      int index = 0;
      double x = gridPoseField.getDouble(index++);
      double y = gridPoseField.getDouble(index++);
      double z = gridPoseField.getDouble(index++);
      double yaw = gridPoseField.getDouble(index++);
      double pitch = gridPoseField.getDouble(index++);
      double roll = gridPoseField.getDouble(index++);

      int numberOfVoxelsPerDimension = descriptionStruct.getMatrix("gridSizeInVoxels").getInt(0);
      double voxelSize = descriptionStruct.getMatrix("voxelSizeInMeters").getDouble(0);
      int numberOfRaysPerVoxel = descriptionStruct.getMatrix("numberOfRays").getInt(0);
      int numberOfRotationsPerRay = descriptionStruct.getMatrix("numberOfRotationsAroundRay").getInt(0);

      Voxel3DGrid grid = Voxel3DGrid.newVoxel3DGrid(numberOfVoxelsPerDimension, voxelSize, numberOfRaysPerVoxel, numberOfRotationsPerRay);
      grid.setGridPose(new Pose3D(x, y, z, yaw, pitch, roll));
      return grid;
   }

   private static void loadPositionData(Voxel3DGrid reachabilityMap, Struct dataStruct)
   {
      Matrix voxelKeyField = dataStruct.getMatrix("voxelKey");
      Matrix desiredPoseField = dataStruct.getMatrix("desiredXYZ");
      Matrix jointPositionsField = dataStruct.getMatrix("jointPositions");
      Matrix jointTorquesField = dataStruct.getMatrix("jointTorques");

      for (int row = 0; row < voxelKeyField.getNumRows(); row++)
      {
         int xIndex = voxelKeyField.getInt(row, 0);
         int yIndex = voxelKeyField.getInt(row, 1);
         int zIndex = voxelKeyField.getInt(row, 2);

         Point3D desiredPosition = new Point3D(desiredPoseField.getFloat(row, 0), desiredPoseField.getFloat(row, 1), desiredPoseField.getFloat(row, 2));

         float[] jointPositions = new float[jointPositionsField.getNumCols()];
         for (int col = 0; col < jointPositionsField.getNumCols(); col++)
            jointPositions[col] = jointPositionsField.getFloat(row, col);
         float[] jointTorques = new float[jointTorquesField.getNumCols()];
         for (int col = 0; col < jointTorquesField.getNumCols(); col++)
            jointTorques[col] = jointTorquesField.getFloat(row, col);

         Voxel3DData voxel = reachabilityMap.getOrCreateVoxel(xIndex, yIndex, zIndex);
         voxel.registerReachablePosition(desiredPosition, jointPositions, jointTorques);
      }
   }

   private static void loadRayData(Voxel3DGrid reachabilityMap, Struct dataStruct)
   {
      Matrix voxelKeyField = dataStruct.getMatrix("voxelKey");
      Matrix desiredPoseField = dataStruct.getMatrix("desiredXYZYPR");
      Matrix jointPositionsField = dataStruct.getMatrix("jointPositions");
      Matrix jointTorquesField = dataStruct.getMatrix("jointTorques");

      for (int row = 0; row < voxelKeyField.getNumRows(); row++)
      {
         int xIndex = voxelKeyField.getInt(row, 0);
         int yIndex = voxelKeyField.getInt(row, 1);
         int zIndex = voxelKeyField.getInt(row, 2);
         int rayIndex = voxelKeyField.getInt(row, 3);

         Pose3D desiredPose = new Pose3D();
         desiredPose.getPosition().set(desiredPoseField.getFloat(row, 0), desiredPoseField.getFloat(row, 1), desiredPoseField.getFloat(row, 2));
         desiredPose.getOrientation().setYawPitchRoll(desiredPoseField.getFloat(row, 3), desiredPoseField.getFloat(row, 4), desiredPoseField.getFloat(row, 5));

         float[] jointPositions = new float[jointPositionsField.getNumCols()];
         for (int col = 0; col < jointPositionsField.getNumCols(); col++)
            jointPositions[col] = jointPositionsField.getFloat(row, col);
         float[] jointTorques = new float[jointTorquesField.getNumCols()];
         for (int col = 0; col < jointTorquesField.getNumCols(); col++)
            jointTorques[col] = jointTorquesField.getFloat(row, col);

         Voxel3DData voxel = reachabilityMap.getOrCreateVoxel(xIndex, yIndex, zIndex);
         voxel.registerReachableRay(rayIndex, desiredPose, jointPositions, jointTorques);
      }
   }

   private static void loadPoseData(Voxel3DGrid reachabilityMap, Struct dataStruct)
   {
      Matrix voxelKeyField = dataStruct.getMatrix("voxelKey");
      Matrix desiredPoseField = dataStruct.getMatrix("desiredXYZYPR");
      Matrix jointPositionsField = dataStruct.getMatrix("jointPositions");
      Matrix jointTorquesField = dataStruct.getMatrix("jointTorques");

      for (int row = 0; row < voxelKeyField.getNumRows(); row++)
      {
         int xIndex = voxelKeyField.getInt(row, 0);
         int yIndex = voxelKeyField.getInt(row, 1);
         int zIndex = voxelKeyField.getInt(row, 2);
         int rayIndex = voxelKeyField.getInt(row, 3);
         int rotationIndex = voxelKeyField.getInt(row, 4);

         Pose3D desiredPose = new Pose3D();
         desiredPose.getPosition().set(desiredPoseField.getFloat(row, 0), desiredPoseField.getFloat(row, 1), desiredPoseField.getFloat(row, 2));
         desiredPose.getOrientation().setYawPitchRoll(desiredPoseField.getFloat(row, 3), desiredPoseField.getFloat(row, 4), desiredPoseField.getFloat(row, 5));

         float[] jointPositions = new float[jointPositionsField.getNumCols()];
         for (int col = 0; col < jointPositionsField.getNumCols(); col++)
            jointPositions[col] = jointPositionsField.getFloat(row, col);
         float[] jointTorques = new float[jointTorquesField.getNumCols()];
         for (int col = 0; col < jointTorquesField.getNumCols(); col++)
            jointTorques[col] = jointTorquesField.getFloat(row, col);

         Voxel3DData voxel = reachabilityMap.getOrCreateVoxel(xIndex, yIndex, zIndex);
         voxel.registerReachablePose(rayIndex, rotationIndex, desiredPose, jointPositions, jointTorques);
      }
   }

   @Override
   public String getFileType()
   {
      return "MATLab data";
   }

   @Override
   public String getFileExtension()
   {
      return ".mat";
   }
}
