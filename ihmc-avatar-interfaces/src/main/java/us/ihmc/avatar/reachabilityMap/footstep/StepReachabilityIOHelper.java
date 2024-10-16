package us.ihmc.avatar.reachabilityMap.footstep;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.multiContact.KinematicsToolboxSnapshotDescription;
import us.ihmc.avatar.multiContact.SixDoFMotionControlAnchorDescription;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityData;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityLatticePoint;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.commons.robotics.robotSide.RobotSide;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;

public class StepReachabilityIOHelper
{
   private static final String gridDataTag = "Reachability Grid Data";
   private static final String spacingTag = "spacingXYZ";
   private static final String gridSizeTag = "gridSizeYaw";
   private static final String yawDivisionsTag = "yawDivisions";

   private final List<KinematicsToolboxSnapshotDescription> reachabilityIKData = new ArrayList<>();

   public static boolean writeToFile(File file,
                                     List<KinematicsToolboxSnapshotDescription> reachabilityMap,
                                     double spacingXYZ,
                                     double yawDivisions,
                                     double yawSpacing)
   {
      try
      {
         if (file.exists())
         {
            file.delete();
         }
         else
         {
            FileTools.ensureDirectoryExists(file.getParentFile().toPath());
         }

         file.createNewFile();
         PrintStream printStream = new PrintStream(file);

         JsonFactory jsonFactory = new JsonFactory();
         ObjectMapper objectMapper = new ObjectMapper(jsonFactory);

         ArrayNode root = objectMapper.createArrayNode();
         ArrayNode reachabilityMapArray = objectMapper.createArrayNode();

         for (KinematicsToolboxSnapshotDescription message : reachabilityMap)
         {
            reachabilityMapArray.add(message.toJSON(objectMapper));
         }

         ObjectNode auxiliaryData = objectMapper.createObjectNode().putObject(gridDataTag);
         auxiliaryData.put(spacingTag, spacingXYZ);
         auxiliaryData.put(gridSizeTag, yawSpacing);
         auxiliaryData.put(yawDivisionsTag, yawDivisions);

         root.add(reachabilityMapArray);
         root.add(auxiliaryData);

         objectMapper.writerWithDefaultPrettyPrinter().writeValue(printStream, root);
         printStream.close();

         return true;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return false;
      }
   }

   public StepReachabilityData loadStepReachability(DRCRobotModel robotModel)
   {
      Class<StepReachabilityIOHelper> loadingClass = StepReachabilityIOHelper.class;
      InputStream inputStream = loadingClass.getClassLoader().getResourceAsStream(robotModel.getStepReachabilityResourceName());
      reachabilityIKData.clear();

      if (inputStream == null)
      {
         LogTools.info("Stream is null");
         return null;
      }

      try
      {
         StepReachabilityData stepReachabilityData = new StepReachabilityData();

         // Load JSON tree
         ObjectMapper objectMapper = new ObjectMapper();
         JsonNode jsonNode = objectMapper.readTree(inputStream);

         // Unpack auxiliary data
         JsonNode gridDataNode = jsonNode.get(1);
         stepReachabilityData.setGridData(gridDataNode.get(spacingTag).asDouble(),
                                          gridDataNode.get(gridSizeTag).asDouble(),
                                          gridDataNode.get(yawDivisionsTag).asInt());

         // Unpack reachability map
         JsonNode script = jsonNode.get(0);
         int numberOfSnapshots = script.size();
         FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

         for (int i = 0; i < numberOfSnapshots; i++)
         {
            KinematicsToolboxSnapshotDescription snapshot = KinematicsToolboxSnapshotDescription.fromJSON(script.get(i));
            reachabilityIKData.add(snapshot);

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
                                                                                         stepReachabilityData.getXyzSpacing(),
                                                                                         stepReachabilityData.getYawDivisions(),
                                                                                         stepReachabilityData.getGridSizeYaw());
            double solutionQuality = snapshot.getIkSolution().getSolutionQuality();
            stepReachabilityData.getLegReachabilityMap().put(latticePoint, solutionQuality);
         }

         return stepReachabilityData;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }
   }

   public List<KinematicsToolboxSnapshotDescription> getReachabilityIKData()
   {
      return reachabilityIKData;
   }
}
