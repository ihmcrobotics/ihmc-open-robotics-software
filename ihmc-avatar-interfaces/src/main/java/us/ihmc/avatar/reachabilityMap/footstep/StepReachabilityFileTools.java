package us.ihmc.avatar.reachabilityMap.footstep;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
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

   private static MultiContactScriptReader setupScriptReader(DRCRobotModel robotModel)
   {
      MultiContactScriptReader scriptReader = new MultiContactScriptReader();
      Path rootPath = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software");
      Path filePath = Paths.get(rootPath.toString(), robotModel.getStepReachabilityResourceName());
      scriptReader.loadScript(filePath.toFile());
      return scriptReader;
   }

   public static List<KinematicsToolboxSnapshotDescription> loadKinematicsSnapshots(DRCRobotModel robotModel)
   {
      MultiContactScriptReader scriptReader = setupScriptReader(robotModel);
      return scriptReader.getAllItems();
   }

   public static StepReachabilityData loadStepReachability(DRCRobotModel robotModel)
   {
      MultiContactScriptReader scriptReader = setupScriptReader(robotModel);
      return loadStepReachability(scriptReader, robotModel);
   }

   public static StepReachabilityData loadStepReachability(MultiContactScriptReader scriptReader, DRCRobotModel robotModel)
   {
      List<KinematicsToolboxSnapshotDescription> kinematicsSnapshots = scriptReader.getAllItems();
      double[] gridData = loadGridDataFromJson(scriptReader.getAuxiliaryData());
      if (gridData == null)
         return null;

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      StepReachabilityData reachabilityData = new StepReachabilityData();

      double spacingXYZ = gridData[0];
      double gridSizeYaw = gridData[1];
      int yawDivisions = (int) gridData[2];

      reachabilityData.setGridData(spacingXYZ, gridSizeYaw, yawDivisions);

      for (int i = 0; i < kinematicsSnapshots.size(); i++)
      {
         KinematicsToolboxSnapshotDescription snapshot = kinematicsSnapshots.get(i);

         // Exports left foot at 0 index then right foot at 1 index
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
                                                                                      gridSizeYaw/yawDivisions);
         double solutionQuality = snapshot.getIkSolution().getSolutionQuality();
         reachabilityData.getLegReachabilityMap().put(latticePoint, solutionQuality);
      }
      return reachabilityData;
   }

   public static double[] loadGridDataFromJson(JsonNode jsonNode)
   {
      if (jsonNode == null)
         return null;

      JsonNode auxDataNode = jsonNode.get("Auxiliary Data");
      JsonNode gridDataNode = auxDataNode.get("Reachability Grid Data");
      double[] gridData = new double[3];
      gridData[0] = gridDataNode.get("spacingXYZ").asDouble();
      gridData[1] = gridDataNode.get("gridSizeYaw").asDouble();
      gridData[2] = gridDataNode.get("yawDivisions").asDouble();
      return gridData;
   }

   public static JsonNode gridDataToJson(double[] gridData)
   {
      JsonFactory jsonFactory = new JsonFactory();
      ObjectMapper objectMapper = new ObjectMapper(jsonFactory);
      ObjectNode root = objectMapper.createObjectNode();
      ObjectNode configurationJSON = root.putObject("Reachability Grid Data");

      configurationJSON.put("spacingXYZ", gridData[0]);
      configurationJSON.put("gridSizeYaw", gridData[1]);
      configurationJSON.put("yawDivisions", gridData[2]);
      return root;
   }

   public static boolean isReachabilityFile(File file)
   {
      if (file == null || !file.exists() || !file.isFile())
      {
         return false;
      }

      try
      {
         return loadReachabilityScript(new FileInputStream(file));
      }
      catch (FileNotFoundException e)
      {
         return false;
      }
   }

   public static boolean loadReachabilityScript(InputStream inputStream)
   {
      if (inputStream == null)
      {
         return false;
      }

      try
      {
         ObjectMapper objectMapper = new ObjectMapper();
         JsonNode jsonNode = objectMapper.readTree(inputStream);

         JsonNode auxiliaryData = null;
         try
         {
            KinematicsToolboxSnapshotDescription.fromJSON(jsonNode.get(0));
         }
         catch (RuntimeException e)
         {
            auxiliaryData = jsonNode.get(1);
         }

         if (auxiliaryData != null)
         {
            JsonNode auxDataNode = auxiliaryData.get("Auxiliary Data");
            if (!auxDataNode.isNull());
            {
               JsonNode gridDataNode = auxDataNode.get("Reachability Grid Data");
               if (!gridDataNode.isNull()) return true;
            }
         }
      }
      catch (IOException e)
      {
         return false;
      }
      return false;
   }
}
