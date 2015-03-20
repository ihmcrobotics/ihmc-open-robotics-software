package us.ihmc.darpaRoboticsChallenge.reachabilityMapCalculator;

import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Path;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

import us.ihmc.darpaRoboticsChallenge.reachabilityMapCalculator.example.RobotArm;
import us.ihmc.darpaRoboticsChallenge.reachabilityMapCalculator.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.darpaRoboticsChallenge.reachabilityMapCalculator.voxelPrimitiveShapes.SphereVoxelShape.SphereVoxelType;
import us.ihmc.utilities.io.files.FileTools;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.ScrewTools;

public class ReachabilityMapFileWriter
{
   private PrintWriter printWriter;

   public ReachabilityMapFileWriter(String robotName, OneDoFJoint[] robotArmJoints, Voxel3DGrid gridToWrite, Class<?> classForFilePath)
   {
      String fileName = prependDateToFileName(robotName) + ".txt";
      Path filePath = FileTools.deriveResourcesPath(classForFilePath);
      FileTools.ensureDirectoryExists(filePath);
      filePath = filePath.resolve(fileName);
      printWriter = FileTools.newPrintWriter(filePath);
      printWriter.write("Reachability Map for the robot " + robotName + "\n");
      printWriter.write("Grid size = " + gridToWrite.getGridSize() + "\n");
      printWriter.write("Number of voxels per dimension = " + gridToWrite.getNumberOfVoxelsPerDimension() + "\n");
      printWriter.write("Voxel size = " + gridToWrite.getVoxelSize() + "\n");
      printWriter.write(createGridFrameDescription(gridToWrite));
      if (robotArmJoints != null && robotArmJoints.length != 0)
         printWriter.write(createKinematicChainDescription(robotArmJoints));
      printWriter.write("Reachable poses:\n");
      printWriter.write("xIndex yIndex zIndex rayIndex rotationAroundRayIndex\n");
   }

   private static String prependDateToFileName(String fileName)
   {
      DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss_");
      Date date = new Date();
      String dateAsString = dateFormat.format(date);

      return dateAsString + fileName;
   }

   private String createGridFrameDescription(Voxel3DGrid gridToWrite)
   {
      String gridFrameDescription = "Grid reference frame information:\n";
      gridFrameDescription += " - Name: " + gridToWrite.getReferenceFrame().getName() + "\n";
      if (!gridToWrite.getReferenceFrame().isWorldFrame())
      {
         gridFrameDescription += " - Parent frame: " + gridToWrite.getReferenceFrame().getParent().getName() + "\n";
         FramePose transformToParent = new FramePose(gridToWrite.getReferenceFrame());
         transformToParent.changeFrame(gridToWrite.getReferenceFrame().getParent());
         gridFrameDescription += " - Transform to parent: " + transformToParent.getFrameOrientationCopy().toStringAsQuaternion() + ", position: "
               + transformToParent.printOutPosition() + "\n";
      }

      return gridFrameDescription;
   }

   private String createKinematicChainDescription(OneDoFJoint[] robotArmJoints)
   {
      String kinematicChainDescription = "Kinematic chain joints: ";
      for (int i = 0; i < robotArmJoints.length; i++)
      {
         kinematicChainDescription += robotArmJoints[i].getName();
         if (i < robotArmJoints.length - 1)
            kinematicChainDescription += ", ";
         else
            kinematicChainDescription += "\n";
      }

      return kinematicChainDescription;
   }

   public void registerReachablePose(int xIndex, int yIndex, int zIndex, int rayIndex, int rotationAroundRayIndex)
   {
      printWriter.write(xIndex + " " + yIndex + " " + zIndex + " " + rayIndex + " " + rotationAroundRayIndex + "\n");
   }

   public void close()
   {
      printWriter.close();
   }

   public static void main(String[] args) throws IOException
   {
      FramePose framePose = new FramePose();
      framePose.setOrientation(1.0, 0.8, -1.1);
      framePose.setPosition(3.1, 0.1, 1.0);
      System.out.println(framePose.getFrameOrientationCopy().toStringAsQuaternion());
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      framePose.getPose(transformToParent);
      ReferenceFrame gridFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("blop", ReferenceFrame.getWorldFrame(), transformToParent);
      SphereVoxelShape sphereVoxelShape = new SphereVoxelShape(gridFrame, 0.1, 10, 12, SphereVoxelType.graspOrigin);
      Voxel3DGrid voxel3dGrid = new Voxel3DGrid(gridFrame, sphereVoxelShape, 10, 0.1);
      RobotArm robot = new RobotArm();
      OneDoFJoint[] armJoints = ScrewTools.filterJoints(robot.getJacobian().getJointsInOrder(), OneDoFJoint.class);
      ReachabilityMapFileWriter reachabilityMapFileWriter = new ReachabilityMapFileWriter(robot.getName(), armJoints, voxel3dGrid, ReachabilityMapFileWriter.class);
      reachabilityMapFileWriter.close();
   }
}
