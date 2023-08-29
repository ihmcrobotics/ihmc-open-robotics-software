package us.ihmc.rdx.ui.affordances.editor;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.ui.interactable.RDXInteractableObjectBuilder;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class AffordanceExporter
{
   private static final double DEFAULT_DURATION = 1.0;
   private static final double LINEAR_VELOCITY = 0.1;
   private static final double ANGULAR_VELOCITY = 1.0; // [rad/s] for the sake gripper this is ~= to 0.1 m/s for a point on the edge of the gripper

   private ReferenceFrame initialObjectFrame;
   private ModifiableReferenceFrame affordanceFrame = new ModifiableReferenceFrame("affordanceFrame", ReferenceFrame.getWorldFrame());
   private final RDXAffordanceFrame graspFrame;
   private final RDXAffordanceFrames preGraspFrames;
   private final RDXAffordanceFrames postGraspFrames;
   private final Set<RobotSide> activeSides;
   private final RDXInteractableObjectBuilder objectBuilder;
   private final WorkspaceResourceDirectory configurationsDirectory = new WorkspaceResourceDirectory(getClass(), "/affordances");
   private final SideDependentList<List<double[]>> csvDataMatrices = new SideDependentList<>();

   public AffordanceExporter(Set<RobotSide> activeSides, RDXAffordanceFrames preGraspFrames, RDXAffordanceFrame graspFrame, RDXAffordanceFrames postGraspFrames, RDXInteractableObjectBuilder objectBuilder)
   {
      this.preGraspFrames = preGraspFrames;
      this.graspFrame = graspFrame;
      this.postGraspFrames = postGraspFrames;
      this.objectBuilder = objectBuilder;
      this.activeSides = activeSides;

      for (RobotSide side : activeSides)
         csvDataMatrices.put(side, new ArrayList<>());
   }

   public void saveToFile(String fileName)
   {
      // change affordance reference from
      // world to initial object frame
      RigidBodyTransform initialObjectTransform = new RigidBodyTransform(objectBuilder.getSelectedObject().getInitialTransformToWorld());
      initialObjectFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(ReferenceFrame.getWorldFrame(), initialObjectTransform);
      affordanceFrame.changeParentFrame(initialObjectFrame);

      WorkspaceResourceFile file = new WorkspaceResourceFile(configurationsDirectory, fileName + ".json");
      if (file.isFileAccessAvailable())
      {
         LogTools.info("Saving to file ...");
         JSONFileTools.save(file, jsonNode ->
         {
            jsonNode.put("name", fileName);
            ArrayNode actionsArrayNode = jsonNode.putArray("actions");

            var preGraspPoses = preGraspFrames.getPoses();
            var graspPoses = graspFrame.getPoses();
            var postGraspPoses = postGraspFrames.getPoses();

            SideDependentList<List<Double>> trajectoryDurations = new SideDependentList<>();
            for (RobotSide side : activeSides)
               trajectoryDurations.put(side, computeTrajectoryDurations(preGraspPoses.get(side), graspPoses.get(side), postGraspPoses.get(side)));

            var arePreGraspPosesSet = preGraspFrames.getArePosesSet();
            var arePostGraspPosesSet = postGraspFrames.getArePosesSet();

            var preGraspHandConfigurations = preGraspFrames.getHandConfigurations();
            var postGraspHandConfigurations = postGraspFrames.getHandConfigurations();

            int numberOfPreGraspFrames = activeSides.contains(RobotSide.RIGHT) ? preGraspPoses.get(RobotSide.RIGHT).size() : preGraspPoses.get(RobotSide.LEFT).size();
            for (int i = 0; i < numberOfPreGraspFrames; i++)
            {
               for (RobotSide side : activeSides)
               {
                  if (arePreGraspPosesSet.get(side).get(i))
                  {
                     ObjectNode actionNode = actionsArrayNode.addObject();
                     actionNode.put("type", "RDXHandPoseAction");
                     actionNode.put("parentFrame", objectBuilder.getSelectedObjectName());
                     actionNode.put("side", side.getLowerCaseName());
                     actionNode.put("trajectoryDuration", trajectoryDurations.get(side).get(i));
                     preGraspPoses.get(side).get(i).changeFrame(affordanceFrame.getReferenceFrame());
                     RigidBodyTransform transformToParent = new RigidBodyTransform(preGraspPoses.get(side).get(i));
                     JSONTools.toJSON(actionNode, transformToParent);

                     double[] dataTrajectories = new double[16];
                     transformToParent.get(dataTrajectories);
                     csvDataMatrices.get(side).add(dataTrajectories);

                     if (preGraspHandConfigurations.get(side).get(i) != null)
                     {
                        dataTrajectories = new double[16];
                        for (int data = 0; data < dataTrajectories.length; data++)
                           dataTrajectories[data] = 0.0;
                        dataTrajectories[0] = HandConfiguration.valueOf(preGraspHandConfigurations.get(side).get(i).toString()).ordinal();
                        csvDataMatrices.get(side).add(dataTrajectories);

                        ObjectNode configurationActionNode = actionsArrayNode.addObject();
                        configurationActionNode.put("type", "RDXHandConfigurationAction");
                        configurationActionNode.put("side", side.getLowerCaseName());
                        configurationActionNode.put("grip", preGraspHandConfigurations.get(side).get(i).toString());
                     }
                  }
               }
            }
            for (RobotSide side : activeSides)
            {
               if (graspFrame.isSet(side))
               {
                  ObjectNode actionNode = actionsArrayNode.addObject();
                  actionNode.put("type", "RDXHandPoseAction");
                  actionNode.put("parentFrame", objectBuilder.getSelectedObjectName());
                  actionNode.put("side", side.getLowerCaseName());
                  actionNode.put("trajectoryDuration", trajectoryDurations.get(side).get(preGraspPoses.get(side).size()));
                  graspPoses.get(side).changeFrame(affordanceFrame.getReferenceFrame());
                  RigidBodyTransform transformToParent = new RigidBodyTransform(graspPoses.get(side));
                  JSONTools.toJSON(actionNode, transformToParent);

                  double[] dataTrajectories = new double[16];
                  transformToParent.get(dataTrajectories);
                  csvDataMatrices.get(side).add(dataTrajectories);

                  if (graspFrame.getHandConfiguration(side) != null)
                  {
                     ObjectNode configurationActionNode = actionsArrayNode.addObject();
                     configurationActionNode.put("type", "RDXHandConfigurationAction");
                     configurationActionNode.put("side", side.getLowerCaseName());
                     configurationActionNode.put("grip", graspFrame.getHandConfiguration(side).toString());

                     dataTrajectories = new double[16];
                     for (int data = 0; data < dataTrajectories.length; data++)
                        dataTrajectories[data] = 0.0;
                     dataTrajectories[0] = HandConfiguration.valueOf(graspFrame.getHandConfiguration(side).toString()).ordinal();
                     csvDataMatrices.get(side).add(dataTrajectories);
                  }
               }
               else
               {
                  LogTools.error("A Grasp Frame is not defined for each considered hand!");
               }
            }
            int numberOfPostGraspFrames = activeSides.contains(RobotSide.RIGHT) ? postGraspPoses.get(RobotSide.RIGHT).size() : postGraspPoses.get(RobotSide.LEFT).size();
            for (int i = 0; i < numberOfPostGraspFrames; i++)
            {
               for (RobotSide side : activeSides)
               {
                  if (arePostGraspPosesSet.get(side).get(i))
                  {
                     ObjectNode actionNode = actionsArrayNode.addObject();
                     actionNode.put("type", "RDXHandPoseAction");
                     actionNode.put("parentFrame", objectBuilder.getSelectedObjectName());
                     actionNode.put("side", side.getLowerCaseName());
                     actionNode.put("trajectoryDuration", trajectoryDurations.get(side).get(postGraspPoses.get(side).size() + 1 + i));
                     postGraspPoses.get(side).get(i).changeFrame(affordanceFrame.getReferenceFrame());
                     RigidBodyTransform transformToParent = new RigidBodyTransform(postGraspPoses.get(side).get(i));
                     JSONTools.toJSON(actionNode, transformToParent);

                     double[] dataTrajectories = new double[16];
                     transformToParent.get(dataTrajectories);
                     csvDataMatrices.get(side).add(dataTrajectories);

                     if (postGraspHandConfigurations.get(side).get(i) != null)
                     {
                        dataTrajectories = new double[16];
                        for (int data = 0; data < dataTrajectories.length; data++)
                           dataTrajectories[data] = 0.0;
                        dataTrajectories[0] = HandConfiguration.valueOf(postGraspHandConfigurations.get(side).get(i).toString()).ordinal();
                        csvDataMatrices.get(side).add(dataTrajectories);

                        ObjectNode configurationActionNode = actionsArrayNode.addObject();
                        configurationActionNode.put("type", "RDXHandConfigurationAction");
                        configurationActionNode.put("side", side.getLowerCaseName());
                        configurationActionNode.put("grip", postGraspHandConfigurations.get(side).get(i).toString());
                     }
                  }
               }
            }
         });
         LogTools.info("SAVED to file {}", file.getFileName());
      }
      else
      {
         LogTools.warn("Could not save to {}", file.getFileName());
      }

      saveExtraInfoToFile(fileName);
      for (RobotSide side : activeSides)
         generateCSVFiles(fileName + side.getPascalCaseName(), csvDataMatrices.get(side));
   }

   private void saveExtraInfoToFile(String fileName)
   {
      WorkspaceResourceFile extraFile = new WorkspaceResourceFile(configurationsDirectory, fileName + "Extra.json");
      if (extraFile.isFileAccessAvailable())
      {
         JSONFileTools.save(extraFile, jsonNode ->
         {
            jsonNode.put("name", fileName);
            jsonNode.put("object", objectBuilder.getSelectedObjectName());
            JSONTools.toJSON(jsonNode, new RigidBodyTransform(objectBuilder.getSelectedObject().getInitialPose()));
            ArrayNode framesArrayNode = jsonNode.putArray("frames");
            var preGraspObjectTransforms = preGraspFrames.getObjectTransforms();
            var preGraspHandConfigurations = preGraspFrames.getHandConfigurations();
            jsonNode.put("numberPreGraspFrames", preGraspObjectTransforms.size());
            for (int i = 0; i < preGraspObjectTransforms.size(); i++)
            {
               ObjectNode frameArray = framesArrayNode.addObject();
               JSONTools.toJSON(frameArray, preGraspObjectTransforms.get(i));

               ArrayNode gripArrayNode = frameArray.putArray("grip");
               for (RobotSide side : activeSides)
               {
                  ObjectNode gripArray = gripArrayNode.addObject();
                  gripArray.put("side", side.getLowerCaseName());
                  gripArray.put("config", preGraspHandConfigurations.get(side).get(i) == null ? "" : preGraspHandConfigurations.get(side).get(i).toString());
               }
            }

            if (graspFrame.isSet(RobotSide.LEFT) || graspFrame.isSet(RobotSide.RIGHT))
            {
               ObjectNode frameArray = framesArrayNode.addObject();
               JSONTools.toJSON(frameArray, graspFrame.getObjectTransform());
               ArrayNode gripArrayNode = frameArray.putArray("grip");
               for (RobotSide side : activeSides)
               {
                  ObjectNode gripArray = gripArrayNode.addObject();
                  gripArray.put("side", side.getLowerCaseName());
                  gripArray.put("config", graspFrame.getHandConfiguration(side) == null ? "" : graspFrame.getHandConfiguration(side).toString());
               }
            }

            var postGraspObjectTransforms = postGraspFrames.getObjectTransforms();
            var postGraspHandConfigurations = postGraspFrames.getHandConfigurations();
            jsonNode.put("numberPostGraspFrames", postGraspObjectTransforms.size());
            for (int i = 0; i < postGraspObjectTransforms.size(); i++)
            {
               ObjectNode frameArray = framesArrayNode.addObject();
               JSONTools.toJSON(frameArray, postGraspObjectTransforms.get(i));
               ArrayNode gripArrayNode = frameArray.putArray("grip");
               for (RobotSide side : activeSides)
               {
                  ObjectNode gripArray = gripArrayNode.addObject();
                  gripArray.put("side", side.getLowerCaseName());
                  gripArray.put("config", postGraspHandConfigurations.get(side).get(i) == null ? "" : postGraspHandConfigurations.get(side).get(i).toString());
               }
            }
         });
         LogTools.info("SAVED to file {}", extraFile.getFileName());
      }
      else
      {
         LogTools.warn("Could not save extra info to {}", extraFile.getFileName());
      }
   }

   private List<Double> computeTrajectoryDurations(List<FramePose3D> preGraspPoses, FramePose3D graspPose, List<FramePose3D> postGraspPoses)
   {
      List<Double> trajectoryDurations = new ArrayList<>();
      int sizePreGrasp = preGraspPoses.size();
      // compute trajectory duration as the necessary duration required to reach two consecutive frames at the desired velocity
      for (int i = 0; i < sizePreGrasp; i++)
      {
         if (i != 0)
         {
            double positionDistance = preGraspPoses.get(i).getPositionDistance(preGraspPoses.get(i - 1));
            double angularDistance = preGraspPoses.get(i).getOrientationDistance(preGraspPoses.get(i - 1));
            // take max duration required to achieve the desired linear velocity or angular velocity
            trajectoryDurations.add(Math.max(positionDistance / LINEAR_VELOCITY, angularDistance / ANGULAR_VELOCITY));
         }
         else
            trajectoryDurations.add(DEFAULT_DURATION);
      }

      if (sizePreGrasp > 0)
      {
         double positionDistance = graspPose.getPositionDistance(preGraspPoses.get(sizePreGrasp - 1));
         double angularDistance = graspPose.getOrientationDistance(preGraspPoses.get(sizePreGrasp - 1));
         // take max duration required to achieve the desired linear velocity or angular velocity
         trajectoryDurations.add(Math.max(positionDistance / LINEAR_VELOCITY, angularDistance / ANGULAR_VELOCITY));
      }
      else
         trajectoryDurations.add(DEFAULT_DURATION);

      for (int i = 0; i < postGraspPoses.size(); i++)
      {
         if (i != 0)
         {
            double positionDistance = postGraspPoses.get(i).getPositionDistance(postGraspPoses.get(i - 1));
            double angularDistance = postGraspPoses.get(i).getOrientationDistance(postGraspPoses.get(i - 1));
            // take max duration required to achieve the desired linear velocity or angular velocity
            trajectoryDurations.add(Math.max(positionDistance / LINEAR_VELOCITY, angularDistance / ANGULAR_VELOCITY));
         }
         else if (graspPose != null)
         {
            double positionDistance = postGraspPoses.get(i).getPositionDistance(graspPose);
            double angularDistance = postGraspPoses.get(i).getOrientationDistance(graspPose);
            // take max duration required to achieve the desired linear velocity or angular velocity
            trajectoryDurations.add(Math.max(positionDistance / LINEAR_VELOCITY, angularDistance / ANGULAR_VELOCITY));
         }
         else
            trajectoryDurations.add(DEFAULT_DURATION);
      }

      return trajectoryDurations;
   }

   private void generateCSVFiles(String fileName, List<double[]> dataMatrix)
   {
      Path filePath = Paths.get(configurationsDirectory.getFilesystemDirectory().toString(), fileName + ".csv");
      File csvFile = new File(filePath.toString());
      try (PrintWriter writer = new PrintWriter(csvFile))
      {
         for (int row = 0; row < dataMatrix.size(); row++)
         {
            double[] dataLine = dataMatrix.get(row);
            for (int col = 0; col < dataLine.length; col++)
            {
               writer.print(dataLine[col]);
               if (col < dataLine.length - 1)
                  writer.append(",");
            }
            if (row < dataMatrix.size() - 1)
            {
               writer.println();
            }
         }
         LogTools.info("SAVED to file {}", csvFile.getName());
         dataMatrix.clear();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

      public void loadFromFile(String fileName)
      {
         Path filePath = Paths.get(configurationsDirectory.getFilesystemDirectory().toString(), fileName + "Extra.json");
         final int[] preGraspFramesSize = new int[1];
         final int[] postGraspFramesSize = new int[1];
         if (Files.exists(filePath))
         {
            JSONFileTools.load(filePath, jsonNode ->
            {
               String objectName = jsonNode.get("object").asText();
               if (!objectName.isEmpty())
               {
                  objectBuilder.loadObject(jsonNode.get("object").asText());
                  RigidBodyTransform initialTransform = new RigidBodyTransform();
                  JSONTools.toEuclid(jsonNode, initialTransform);
                  objectBuilder.getSelectedObject().setPose(initialTransform);
               }
               preGraspFramesSize[0] = jsonNode.get("numberPreGraspFrames").asInt();
               postGraspFramesSize[0] = jsonNode.get("numberPostGraspFrames").asInt();
               JsonNode framesArrayNode = jsonNode.get("frames");
               for (int i = 0; i < preGraspFramesSize[0]; i++)
               {
                  RigidBodyTransform preGraspObjectTransform = new RigidBodyTransform();
                  JSONTools.toEuclid(framesArrayNode.get(i), preGraspObjectTransform);
                  preGraspFrames.addObjectTransform(preGraspObjectTransform);
                  String configuration = framesArrayNode.get(i).get("grip").asText();
                  preGraspFrames.addHandConfiguration(configuration.isEmpty() ? null : HandConfiguration.valueOf(configuration));
               }
               RigidBodyTransform graspObjectTransform = new RigidBodyTransform();
               JSONTools.toEuclid(framesArrayNode.get(preGraspFramesSize[0]), graspObjectTransform);
               graspFrame.setObjectTransform(graspObjectTransform);
               String configuration = framesArrayNode.get(preGraspFramesSize[0]).get("grip").asText();
               graspFrame.setHandConfiguration(configuration.isEmpty() ? null : HandConfiguration.valueOf(configuration));
               for (int i = 0; i < postGraspFramesSize[0]; i++)
               {
                  RigidBodyTransform postGraspObjectTransform = new RigidBodyTransform();
                  JSONTools.toEuclid(framesArrayNode.get(i + preGraspFramesSize[0] + 1), postGraspObjectTransform);
                  postGraspFrames.addObjectTransform(postGraspObjectTransform);
                  configuration = framesArrayNode.get(i + preGraspFramesSize[0] + 1).get("grip").asText();
                  postGraspFrames.addHandConfiguration(configuration.isEmpty() ? null : HandConfiguration.valueOf(configuration));
               }
            });
            LogTools.info("LOADED file {}", filePath);
         }
         else
         {
            LogTools.warn("Could not load file {}", filePath);
         }

         // change affordance reference from whatever it is now to loaded initial object pose
         RigidBodyTransform initialObjectTransform = new RigidBodyTransform(objectBuilder.getSelectedObject().getInitialTransformToWorld());
         initialObjectFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(ReferenceFrame.getWorldFrame(), initialObjectTransform);
         affordanceFrame.changeParentFrame(initialObjectFrame);

         filePath = Paths.get(configurationsDirectory.getFilesystemDirectory().toString(), fileName + ".json");
         if (Files.exists(filePath))
         {
            JSONFileTools.load(filePath, jsonNode ->
            {
               JSONTools.forEachArrayElement(jsonNode, "actions", actionNode ->
               {
                  String actionType = actionNode.get("type").asText();
                  if (actionType.equals("RDXHandPoseAction"))
                  {
                     //                  side = RobotSide.getSideFromString(jsonNode.get("side").asText());
                     RigidBodyTransform frameTransform = new RigidBodyTransform();
                     JSONTools.toEuclid(actionNode, frameTransform);
                     if (preGraspFrames.getNumberOfFrames() < preGraspFramesSize[0])
                        preGraspFrames.addFrame(new FramePose3D(affordanceFrame.getReferenceFrame(), frameTransform));
                     else if (!graspFrame.isSet())
                        graspFrame.setFrame(new FramePose3D(affordanceFrame.getReferenceFrame(), frameTransform));
                     else
                        postGraspFrames.addFrame(new FramePose3D(affordanceFrame.getReferenceFrame(), frameTransform));
                  }
               });
            });
            LogTools.info("LOADED file {}", filePath);
         }
         else
         {
            LogTools.warn("Could not load file {}", filePath);
         }
      }
}
