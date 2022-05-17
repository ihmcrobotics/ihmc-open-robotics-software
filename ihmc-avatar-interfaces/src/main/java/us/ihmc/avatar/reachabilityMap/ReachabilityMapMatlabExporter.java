package us.ihmc.avatar.reachabilityMap;

import java.io.File;
import java.io.IOException;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

import us.hebi.matlab.mat.format.Mat5;
import us.hebi.matlab.mat.types.Array;
import us.hebi.matlab.mat.types.Cell;
import us.hebi.matlab.mat.types.MatFile;
import us.hebi.matlab.mat.types.Matrix;
import us.hebi.matlab.mat.types.Sinks;
import us.hebi.matlab.mat.types.Struct;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DData;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.VoxelExtraData;
import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;

public class ReachabilityMapMatlabExporter implements ReachabilityMapFileWriter
{
   @Override
   public void write(File file, ReachabilityMapRobotInformation robotInformation, Voxel3DGrid reachabilityMap)
   {
      MatFile matFile = Mat5.newMatFile();

      matFile.addArray("Description", createDescriptionStruct(robotInformation, reachabilityMap));
      matFile.addArray("PositionReachData", createPositionReachDataStruct(robotInformation, reachabilityMap));
      matFile.addArray("RayReachData", createRayReachDataStruct(robotInformation, reachabilityMap));
      matFile.addArray("PoseReachData", createPoseReachDataStruct(robotInformation, reachabilityMap));

      try
      {
         matFile.writeTo(Sinks.newStreamingFile(file));
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private static Struct createDescriptionStruct(ReachabilityMapRobotInformation robotInformation, Voxel3DGrid reachabilityMap)
   {
      Struct descriptionStruct = Mat5.newStruct();

      RobotDefinition robotDefinition = robotInformation.getRobotDefinition();
      List<OneDoFJointDefinition> evaluatedJoints = robotInformation.getEvaluatedJoints();
      SphereVoxelShape sphereVoxelShape = reachabilityMap.getSphereVoxelShape();

      descriptionStruct.set("robotName", Mat5.newString(robotDefinition.getName()));
      descriptionStruct.set("gridSizeInMeters", newDouble(reachabilityMap.getGridSizeMeters()));
      descriptionStruct.set("gridSizeInVoxels", newInt(reachabilityMap.getGridSizeVoxels()));
      descriptionStruct.set("voxelSizeInMeters", newDouble(reachabilityMap.getVoxelSize()));
      descriptionStruct.set("numberOfRays", newInt(sphereVoxelShape.getNumberOfRays()));
      descriptionStruct.set("numberOfRotationsAroundRay", newInt(sphereVoxelShape.getNumberOfRotationsAroundRay()));
      descriptionStruct.set("gridPoseXYZYPR", newPose3D(new Pose3D(reachabilityMap.getReferenceFrame().getTransformToRoot()), true));
      descriptionStruct.set("jointNames", newStrings(evaluatedJoints.stream().map(j -> j.getName()).collect(Collectors.toList())));
      descriptionStruct.set("jointPositionLimitLower", newDoubleVector(evaluatedJoints.stream().mapToDouble(j -> j.getPositionLowerLimit()).toArray()));
      descriptionStruct.set("jointPositionLimitUpper", newDoubleVector(evaluatedJoints.stream().mapToDouble(j -> j.getPositionUpperLimit()).toArray()));
      descriptionStruct.set("jointEffortLimitLower", newDoubleVector(evaluatedJoints.stream().mapToDouble(j -> j.getEffortLowerLimit()).toArray()));
      descriptionStruct.set("jointEffortLimitUpper", newDoubleVector(evaluatedJoints.stream().mapToDouble(j -> j.getEffortUpperLimit()).toArray()));
      descriptionStruct.set("controlFramePoseXYZYPR", newPose3D(robotInformation.getControlFramePoseInParentJoint(), true));

      return descriptionStruct;
   }

   private static Struct createPositionReachDataStruct(ReachabilityMapRobotInformation robotInformation, Voxel3DGrid reachabilityMap)
   {
      List<OneDoFJointDefinition> evaluatedJoints = robotInformation.getEvaluatedJoints();
      Struct dataStruct = Mat5.newStruct();

      int dataLength = 0;

      for (int voxelIndex = 0; voxelIndex < reachabilityMap.getNumberOfVoxels(); voxelIndex++)
      {
         Voxel3DData voxel = reachabilityMap.getVoxel(voxelIndex);
         if (voxel != null && voxel.getPositionExtraData() != null)
            dataLength++;
      }

      Matrix voxelIndexMatrix = Mat5.newMatrix(dataLength, 3);
      Matrix desiredPositionMatrix = Mat5.newMatrix(dataLength, 3);
      Matrix jointPositions = Mat5.newMatrix(dataLength, evaluatedJoints.size());
      Matrix jointTorques = Mat5.newMatrix(dataLength, evaluatedJoints.size());

      int row = 0;

      for (int voxelIndex = 0; voxelIndex < reachabilityMap.getNumberOfVoxels(); voxelIndex++)
      {
         Voxel3DData voxel = reachabilityMap.getVoxel(voxelIndex);

         if (voxel == null)
            continue;

         VoxelExtraData extraData = voxel.getPositionExtraData();

         if (extraData == null)
            continue;

         voxelIndexMatrix.setInt(row, 0, voxel.getKey().getX());
         voxelIndexMatrix.setInt(row, 1, voxel.getKey().getY());
         voxelIndexMatrix.setInt(row, 2, voxel.getKey().getZ());
         desiredPositionMatrix.setDouble(row, 0, (float) extraData.getDesiredPosition().getX());
         desiredPositionMatrix.setDouble(row, 1, (float) extraData.getDesiredPosition().getY());
         desiredPositionMatrix.setDouble(row, 2, (float) extraData.getDesiredPosition().getZ());

         for (int col = 0; col < evaluatedJoints.size(); col++)
         {
            jointPositions.setFloat(row, col, extraData.getJointPositions()[col]);
            jointTorques.setFloat(row, col, extraData.getJointTorques()[col]);
         }

         row++;
      }

      dataStruct.set("voxelKey", voxelIndexMatrix);
      dataStruct.set("desiredXYZ", desiredPositionMatrix);
      dataStruct.set("jointPositions", jointPositions);
      dataStruct.set("jointTorques", jointTorques);

      return dataStruct;
   }

   private static Struct createRayReachDataStruct(ReachabilityMapRobotInformation robotInformation, Voxel3DGrid reachabilityMap)
   {
      List<OneDoFJointDefinition> evaluatedJoints = robotInformation.getEvaluatedJoints();
      Struct dataStruct = Mat5.newStruct();

      int dataLength = 0;

      for (int voxelIndex = 0; voxelIndex < reachabilityMap.getNumberOfVoxels(); voxelIndex++)
      {
         Voxel3DData voxel = reachabilityMap.getVoxel(voxelIndex);
         if (voxel != null)
            dataLength += voxel.getNumberOfReachableRays();
      }

      Matrix voxelIndexMatrix = Mat5.newMatrix(dataLength, 4);
      Matrix desiredPoseMatrix = Mat5.newMatrix(dataLength, 6);
      Matrix jointPositions = Mat5.newMatrix(dataLength, evaluatedJoints.size());
      Matrix jointTorques = Mat5.newMatrix(dataLength, evaluatedJoints.size());

      int row = 0;

      for (int voxelIndex = 0; voxelIndex < reachabilityMap.getNumberOfVoxels(); voxelIndex++)
      {
         Voxel3DData voxel = reachabilityMap.getVoxel(voxelIndex);

         if (voxel == null)
            continue;

         for (int rayIndex = 0; rayIndex < voxel.getNumberOfRays(); rayIndex++)
         {
            VoxelExtraData extraData = voxel.getRayExtraData(rayIndex);

            if (extraData == null)
               continue;

            voxelIndexMatrix.setInt(row, 0, voxel.getKey().getX());
            voxelIndexMatrix.setInt(row, 1, voxel.getKey().getY());
            voxelIndexMatrix.setInt(row, 2, voxel.getKey().getZ());
            voxelIndexMatrix.setInt(row, 3, rayIndex);
            desiredPoseMatrix.setFloat(row, 0, (float) extraData.getDesiredPosition().getX());
            desiredPoseMatrix.setFloat(row, 1, (float) extraData.getDesiredPosition().getY());
            desiredPoseMatrix.setFloat(row, 2, (float) extraData.getDesiredPosition().getZ());
            desiredPoseMatrix.setFloat(row, 3, (float) extraData.getDesiredOrientation().getYaw());
            desiredPoseMatrix.setFloat(row, 4, (float) extraData.getDesiredOrientation().getPitch());
            desiredPoseMatrix.setFloat(row, 5, (float) extraData.getDesiredOrientation().getRoll());

            for (int col = 0; col < evaluatedJoints.size(); col++)
            {
               jointPositions.setFloat(row, col, extraData.getJointPositions()[col]);
               jointTorques.setFloat(row, col, extraData.getJointTorques()[col]);
            }

            row++;
         }
      }

      dataStruct.set("voxelKey", voxelIndexMatrix);
      dataStruct.set("desiredXYZYPR", desiredPoseMatrix);
      dataStruct.set("jointPositions", jointPositions);
      dataStruct.set("jointTorques", jointTorques);

      return dataStruct;
   }

   private static Struct createPoseReachDataStruct(ReachabilityMapRobotInformation robotInformation, Voxel3DGrid reachabilityMap)
   {
      List<OneDoFJointDefinition> evaluatedJoints = robotInformation.getEvaluatedJoints();
      Struct dataStruct = Mat5.newStruct();

      int dataLength = 0;

      for (int voxelIndex = 0; voxelIndex < reachabilityMap.getNumberOfVoxels(); voxelIndex++)
      {
         Voxel3DData voxel = reachabilityMap.getVoxel(voxelIndex);
         if (voxel != null)
            dataLength += voxel.getNumberOfReachablePoses();
      }

      Matrix voxelIndexMatrix = Mat5.newMatrix(dataLength, 5);
      Matrix desiredPoseMatrix = Mat5.newMatrix(dataLength, 6);
      Matrix jointPositions = Mat5.newMatrix(dataLength, evaluatedJoints.size());
      Matrix jointTorques = Mat5.newMatrix(dataLength, evaluatedJoints.size());

      int row = 0;

      for (int voxelIndex = 0; voxelIndex < reachabilityMap.getNumberOfVoxels(); voxelIndex++)
      {
         Voxel3DData voxel = reachabilityMap.getVoxel(voxelIndex);

         if (voxel == null || !voxel.atLeastOneReachablePose())
            continue;

         for (int rayIndex = 0; rayIndex < voxel.getNumberOfRays(); rayIndex++)
         {
            for (int rotationIndex = 0; rotationIndex < voxel.getNumberOfRotationsAroundRay(); rotationIndex++)
            {
               VoxelExtraData extraData = voxel.getPoseExtraData(rayIndex, rotationIndex);

               if (extraData == null)
                  continue;

               voxelIndexMatrix.setInt(row, 0, voxel.getKey().getX());
               voxelIndexMatrix.setInt(row, 1, voxel.getKey().getY());
               voxelIndexMatrix.setInt(row, 2, voxel.getKey().getZ());
               voxelIndexMatrix.setInt(row, 3, rayIndex);
               voxelIndexMatrix.setInt(row, 4, rotationIndex);
               desiredPoseMatrix.setFloat(row, 0, (float) extraData.getDesiredPosition().getX());
               desiredPoseMatrix.setFloat(row, 1, (float) extraData.getDesiredPosition().getY());
               desiredPoseMatrix.setFloat(row, 2, (float) extraData.getDesiredPosition().getZ());
               desiredPoseMatrix.setFloat(row, 3, (float) extraData.getDesiredOrientation().getYaw());
               desiredPoseMatrix.setFloat(row, 4, (float) extraData.getDesiredOrientation().getPitch());
               desiredPoseMatrix.setFloat(row, 5, (float) extraData.getDesiredOrientation().getRoll());

               for (int col = 0; col < evaluatedJoints.size(); col++)
               {
                  jointPositions.setFloat(row, col, extraData.getJointPositions()[col]);
                  jointTorques.setFloat(row, col, extraData.getJointTorques()[col]);
               }

               row++;
            }
         }
      }

      dataStruct.set("voxelKey", voxelIndexMatrix);
      dataStruct.set("desiredXYZYPR", desiredPoseMatrix);
      dataStruct.set("jointPositions", jointPositions);
      dataStruct.set("jointTorques", jointTorques);

      return dataStruct;
   }

   private static Array newDouble(double value)
   {
      Matrix matrix = Mat5.newMatrix(1, 1);
      matrix.setDouble(0, value);
      return matrix;
   }

   private static Array newInt(int value)
   {
      Matrix matrix = Mat5.newMatrix(1, 1);
      matrix.setInt(0, value);
      return matrix;
   }

   private static Array newStrings(Collection<String> strings)
   {
      Cell cell = Mat5.newCell(strings.size(), 1);
      int index = 0;
      for (String string : strings)
      {
         cell.set(index++, Mat5.newString(string));
      }
      return cell;
   }

   private static Array newPose3D(Pose3DReadOnly pose, boolean useYawPitchRoll)
   {
      if (useYawPitchRoll)
      {
         Matrix matrix = Mat5.newMatrix(6, 1);
         int index = 0;
         matrix.setDouble(index++, pose.getX());
         matrix.setDouble(index++, pose.getY());
         matrix.setDouble(index++, pose.getZ());
         matrix.setDouble(index++, pose.getYaw());
         matrix.setDouble(index++, pose.getPitch());
         matrix.setDouble(index++, pose.getRoll());
         return matrix;
      }
      else
      {
         Matrix matrix = Mat5.newMatrix(7, 1);
         int index = 0;
         matrix.setDouble(index++, pose.getX());
         matrix.setDouble(index++, pose.getY());
         matrix.setDouble(index++, pose.getZ());
         matrix.setDouble(index++, pose.getOrientation().getX());
         matrix.setDouble(index++, pose.getOrientation().getY());
         matrix.setDouble(index++, pose.getOrientation().getZ());
         matrix.setDouble(index++, pose.getOrientation().getS());
         return matrix;
      }
   }

   private static Array newDoubleVector(double... values)
   {
      Matrix matrix = Mat5.newMatrix(values.length, 1);

      for (int i = 0; i < values.length; i++)
      {
         matrix.setDouble(i, values[i]);
      }
      return matrix;
   }

   @Override
   public String getFileExtension()
   {
      return ".mat";
   }

}
