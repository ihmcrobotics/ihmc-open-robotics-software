package us.ihmc.avatar.reachabilityMap.footstep;

import org.apache.commons.io.FilenameUtils;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.multiContact.KinematicsToolboxSnapshotDescription;
import us.ihmc.avatar.multiContact.MultiContactScriptReader;
import us.ihmc.avatar.multiContact.SixDoFMotionControlAnchorDescription;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityData;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityLatticePoint;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspacePathTools;

import java.io.*;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Map;
import java.util.Scanner;

public class StepReachabilityFileTools
{
   public static void writeToFile(String fileName, Map<StepReachabilityLatticePoint, Double> feasibilityMap, double spacingXY, int yawDivisions, double yawSpacing)
   {
      FileWriter fileWriter;

      try
      {
         File reachabilityDataFile = new File(fileName);
         FileTools.ensureFileExists(reachabilityDataFile.toPath());
         fileWriter = new FileWriter(reachabilityDataFile);

         fileWriter.write(spacingXY + ",");
         fileWriter.write(yawDivisions + ",");
         fileWriter.write(yawSpacing + "\n");

         for (StepReachabilityLatticePoint latticePoint : feasibilityMap.keySet())
         {
            fileWriter.write(latticePoint.getXIndex() + ",");
            fileWriter.write(latticePoint.getYIndex() + ",");
            fileWriter.write(latticePoint.getZIndex() + ",");
            fileWriter.write(latticePoint.getYawIndex() + ",");
            fileWriter.write(String.valueOf(feasibilityMap.get(latticePoint)));
            fileWriter.write("\n");
         }

         fileWriter.flush();
         LogTools.info("Done writing to file");
      }
      catch (Exception e)
      {
         LogTools.error("Error logging reachability file");
         e.printStackTrace();
      }
   }

   public static List<KinematicsToolboxSnapshotDescription> loadKinematicsSnapshots(DRCRobotModel robotModel)
   {
      MultiContactScriptReader scriptReader = new MultiContactScriptReader();

      Path rootPath = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software");
      Path filePath = Paths.get(rootPath.toString(), robotModel.getStepReachabilityResourceName());
      scriptReader.loadScript(filePath.toFile());
      return scriptReader.getAllItems();
   }

   public static StepReachabilityData loadStepReachability(DRCRobotModel robotModel)
   {
      List<KinematicsToolboxSnapshotDescription> kinematicsSnapshots = loadKinematicsSnapshots(robotModel);
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      StepReachabilityData reachabilityData = new StepReachabilityData();

      // TODO figure out a way to export this.
      // It can be a small header file that sits next to the .json file, and we can add a method in DRCRobotModel for it's file name
      // Or it it's possible to have MultiContactScriptWriter to export it into the json. Maybe MultiContactWriter.writeScript could optionally
      // take in an extra json node, but it'll need to stay backwards compatible so that might get tricky.
      double spacingXYZ = 0.1;
      int yawDivisions = 6;
      double minimumOffsetYaw = - Math.toRadians(70.0);
      double maximumOffsetYaw = Math.toRadians(80.0);
      double yawSpacing = (maximumOffsetYaw - minimumOffsetYaw) / yawDivisions;;

      reachabilityData.setGridData(spacingXYZ, yawSpacing * yawDivisions, yawDivisions);
      for (int i = 0; i < kinematicsSnapshots.size(); i++)
      {
         KinematicsToolboxSnapshotDescription snapshot = kinematicsSnapshots.get(i);

         // Exporter should do left foot at 0 index then right foot at 1 index. Double check this is the case.
         SixDoFMotionControlAnchorDescription leftFoot = snapshot.getSixDoFAnchors().get(0);
         SixDoFMotionControlAnchorDescription rightFoot = snapshot.getSixDoFAnchors().get(1);
         assert(leftFoot.getRigidBodyName().equals(fullRobotModel.getFoot(RobotSide.LEFT).getName()));
         assert(rightFoot.getRigidBodyName().equals(fullRobotModel.getFoot(RobotSide.RIGHT).getName()));

         // Right foot is at origin. Back out lattice point of left foot
         Point3D leftFootDesiredPosition = leftFoot.getInputMessage().getDesiredPositionInWorld();
         Quaternion leftFootDesiredOrientation = leftFoot.getInputMessage().getDesiredOrientationInWorld();
         StepReachabilityLatticePoint latticePoint = new StepReachabilityLatticePoint(leftFootDesiredPosition.getX(),
                                                                                      leftFootDesiredPosition.getY(),
                                                                                      leftFootDesiredPosition.getZ(),
                                                                                      leftFootDesiredOrientation.getYaw(),
                                                                                      spacingXYZ,
                                                                                      yawDivisions,
                                                                                      yawSpacing);
         double solutionQuality = snapshot.getIkSolution().getSolutionQuality();
         reachabilityData.getLegReachabilityMap().put(latticePoint, solutionQuality);
      }

      return reachabilityData;
   }
}
