package us.ihmc.rdx.ui.affordances.editor;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.avatar.sakeGripper.SakeHandCommandOption;
import us.ihmc.behaviors.sequence.actions.HandPoseActionDefinition;
import us.ihmc.behaviors.sequence.actions.SakeHandCommandActionDefinition;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.ui.interactable.RDXInteractableObjectBuilder;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
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

import static us.ihmc.robotics.robotSide.RobotSide.getSideFromString;

public class RDXAffordanceTemplateFileManager
{
   private static final double DEFAULT_DURATION = 1.0;
   private static final double LINEAR_VELOCITY = 0.1;
   private static final double ANGULAR_VELOCITY = 1.0; // for the sake gripper this is ~= to 0.1 m/s for a point on the edge of the gripper

   private ReferenceFrame initialObjectFrame;
   private MutableReferenceFrame affordanceFrame = new MutableReferenceFrame("affordanceFrame", ReferenceFrame.getWorldFrame());
   private final RDXAffordanceTemplateFrame graspFrame;
   private final RDXAffordanceTemplateFrames preGraspFrames;
   private final RDXAffordanceTemplateFrames postGraspFrames;
   private final Set<RobotSide> activeSides;
   private final RDXInteractableObjectBuilder objectBuilder;
   private final WorkspaceResourceDirectory configurationsDirectory = new WorkspaceResourceDirectory(getClass(), "/affordances");
   private final SideDependentList<List<double[]>> csvDataMatrices = new SideDependentList<>();
   private String loadingFileName = "";

   final int[] numberActiveSides = new int[1];
   final double[] index = new double[1];

   public RDXAffordanceTemplateFileManager(Set<RobotSide> activeSides, RDXAffordanceTemplateFrames preGraspFrames, RDXAffordanceTemplateFrame graspFrame, RDXAffordanceTemplateFrames postGraspFrames, RDXInteractableObjectBuilder objectBuilder)
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
      saveAffordanceActionToFile(fileName);
      saveFrameInfoToFile(fileName);
      saveExtraInfoToFile(fileName);
      for (RobotSide side : activeSides)
         generateCSVFiles(fileName + side.getPascalCaseName(), csvDataMatrices.get(side));
   }

   private void saveAffordanceActionToFile(String fileName)
   {
      // change affordance reference from
      // world to initial object frame
      RigidBodyTransform initialObjectTransform = new RigidBodyTransform(objectBuilder.getSelectedObject().getInitialTransformToWorld());
      initialObjectFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(ReferenceFrame.getWorldFrame(), initialObjectTransform);
      affordanceFrame.setParentFrame(initialObjectFrame);

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
                     actionNode.put("type", HandPoseActionDefinition.class.getSimpleName());
                     actionNode.put("description", "Pre-grasp " + side.getPascalCaseName() + " Hand Pose");
                     actionNode.put("parentFrame", objectBuilder.getSelectedObjectName());
                     actionNode.put("side", side.getLowerCaseName());
                     actionNode.put("trajectoryDuration", trajectoryDurations.get(side).get(i));
                     preGraspPoses.get(side).get(i).changeFrame(affordanceFrame.getReferenceFrame());
                     RigidBodyTransform transformToParent = new RigidBodyTransform(preGraspPoses.get(side).get(i));
                     JSONTools.toJSON(actionNode, transformToParent);
                     boolean bothPosesAreSet = false;
                     if (activeSides.size() == 2)
                     {
                        // check if both poses are set
                        bothPosesAreSet = arePreGraspPosesSet.get(RobotSide.RIGHT).get(i) && arePreGraspPosesSet.get(RobotSide.LEFT).get(i);
                        // also check if a hand configuration has been set
                        boolean oneHandConfigurationSet =
                              (preGraspHandConfigurations.get(RobotSide.RIGHT).get(i) != null) || (preGraspHandConfigurations.get(RobotSide.LEFT).get(i) != null);
                        // if right side (left side always comes first), then do not execute this concurrently with the next action
                        // otherwise check if the pose of the both hands has been set
                        boolean executeWithNextAction;
                        if (oneHandConfigurationSet)
                           executeWithNextAction = true;
                        else if (side == RobotSide.RIGHT)
                           executeWithNextAction = false;
                        else
                           executeWithNextAction = bothPosesAreSet;
                        actionNode.put("executeWithNextAction", executeWithNextAction);
                     }
                     else
                        actionNode.put("executeWithNextAction", preGraspHandConfigurations.get(side).get(i) != null ? true : false);
                     actionNode.put("holdPoseInWorldLater", true);

                     double[] dataTrajectories = new double[16];
                     transformToParent.get(dataTrajectories);
                     csvDataMatrices.get(side).add(dataTrajectories);

                     if (preGraspHandConfigurations.get(side).get(i) != null)
                     {
                        dataTrajectories = new double[16];
                        for (int data = 0; data < dataTrajectories.length; data++)
                           dataTrajectories[data] = 0.0;
                        dataTrajectories[0] = SakeHandCommandOption.valueOf(preGraspHandConfigurations.get(side).get(i).toString()).ordinal();
                        csvDataMatrices.get(side).add(dataTrajectories);

                        ObjectNode configurationActionNode = actionsArrayNode.addObject();
                        configurationActionNode.put("type", SakeHandCommandActionDefinition.class.getSimpleName());
                        configurationActionNode.put("description", "Pre-grasp " + side.getPascalCaseName() + " Hand Configuration");
                        configurationActionNode.put("side", side.getLowerCaseName());
                        configurationActionNode.put("configuration", preGraspHandConfigurations.get(side).get(i).toString());
                        configurationActionNode.put("position", SakeHandCommandOption.valueOf(preGraspHandConfigurations.get(side).get(i)).getGoalPosition());
                        configurationActionNode.put("torque", SakeHandCommandOption.valueOf(preGraspHandConfigurations.get(side).get(i)).getGoalTorque());
                        configurationActionNode.put("executeWithNextAction", side != RobotSide.RIGHT && bothPosesAreSet);
                     }
                  }
               }
            }
            for (RobotSide side : activeSides)
            {
               if (graspFrame.isSet(side))
               {
                  ObjectNode actionNode = actionsArrayNode.addObject();
                  actionNode.put("type", HandPoseActionDefinition.class.getSimpleName());
                  actionNode.put("description", "Grasp " + side.getPascalCaseName() + " Hand Pose");
                  actionNode.put("parentFrame", objectBuilder.getSelectedObjectName());
                  actionNode.put("side", side.getLowerCaseName());
                  actionNode.put("trajectoryDuration", trajectoryDurations.get(side).get(preGraspPoses.get(side).size()));
                  graspPoses.get(side).changeFrame(affordanceFrame.getReferenceFrame());
                  RigidBodyTransform transformToParent = new RigidBodyTransform(graspPoses.get(side));
                  JSONTools.toJSON(actionNode, transformToParent);
                  boolean bothPosesAreSet = false;
                  if (activeSides.size() == 2)
                  {
                     // check if both poses are set
                     bothPosesAreSet = graspFrame.isSet(RobotSide.RIGHT) && graspFrame.isSet(RobotSide.LEFT);
                     // also check if a hand configuration has been set
                     boolean oneHandConfigurationSet = (graspFrame.getHandConfiguration(RobotSide.RIGHT) != null) || (graspFrame.getHandConfiguration(RobotSide.LEFT) != null);
                     // if right side (left side always comes first), then do not execute this concurrently with the next action
                     // otherwise check if the pose of the both hands has been set
                     boolean executeWithNextAction;
                     if (oneHandConfigurationSet)
                        executeWithNextAction = true;
                     else if (side == RobotSide.RIGHT)
                        executeWithNextAction = false;
                     else
                        executeWithNextAction = bothPosesAreSet;
                     actionNode.put("executeWithNextAction", executeWithNextAction);
                  }
                  else
                     actionNode.put("executeWithNextAction", graspFrame.getHandConfiguration(side) != null ? true : false);
                  actionNode.put("holdPoseInWorldLater", true);

                  double[] dataTrajectories = new double[16];
                  transformToParent.get(dataTrajectories);
                  csvDataMatrices.get(side).add(dataTrajectories);

                  if (graspFrame.getHandConfiguration(side) != null)
                  {
                     ObjectNode configurationActionNode = actionsArrayNode.addObject();
                     configurationActionNode.put("type", SakeHandCommandActionDefinition.class.getSimpleName());
                     configurationActionNode.put("description", "Grasp " + side.getPascalCaseName() + " Hand Configuration");
                     configurationActionNode.put("side", side.getLowerCaseName());
                     configurationActionNode.put("configuration", graspFrame.getHandConfiguration(side).toString());
                     configurationActionNode.put("position", SakeHandCommandOption.valueOf(graspFrame.getHandConfiguration(side)).getGoalPosition());
                     configurationActionNode.put("torque", SakeHandCommandOption.valueOf(graspFrame.getHandConfiguration(side)).getGoalTorque());
                     configurationActionNode.put("executeWithNextAction", side != RobotSide.RIGHT && bothPosesAreSet);

                     dataTrajectories = new double[16];
                     for (int data = 0; data < dataTrajectories.length; data++)
                        dataTrajectories[data] = 0.0;
                     dataTrajectories[0] = SakeHandCommandOption.valueOf(graspFrame.getHandConfiguration(side).toString()).ordinal();
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
                     actionNode.put("type", HandPoseActionDefinition.class.getSimpleName());
                     actionNode.put("description", "Post-grasp " + side.getPascalCaseName() + " Hand Pose");
                     actionNode.put("parentFrame", objectBuilder.getSelectedObjectName());
                     actionNode.put("side", side.getLowerCaseName());
                     actionNode.put("trajectoryDuration", trajectoryDurations.get(side).get(preGraspPoses.get(side).size() + 1 + i));
                     postGraspPoses.get(side).get(i).changeFrame(affordanceFrame.getReferenceFrame());
                     RigidBodyTransform transformToParent = new RigidBodyTransform(postGraspPoses.get(side).get(i));
                     JSONTools.toJSON(actionNode, transformToParent);
                     boolean bothPosesAreSet = false;
                     if (activeSides.size() == 2)
                     {
                        // check if both poses are set
                        bothPosesAreSet = arePostGraspPosesSet.get(RobotSide.RIGHT).get(i) && arePostGraspPosesSet.get(RobotSide.LEFT).get(i);
                        // also check if a hand configuration has been set
                        boolean oneHandConfigurationSet =
                              (postGraspHandConfigurations.get(RobotSide.RIGHT).get(i) != null) || (postGraspHandConfigurations.get(RobotSide.LEFT).get(i) != null);
                        // if right side (left side always comes first), then do not execute this concurrently with the next action
                        // otherwise check if the pose of the both hands has been set
                        boolean executeWithNextAction;
                        if (oneHandConfigurationSet)
                           executeWithNextAction = true;
                        else if (side == RobotSide.RIGHT)
                           executeWithNextAction = false;
                        else
                           executeWithNextAction = bothPosesAreSet;
                        actionNode.put("executeWithNextAction", executeWithNextAction);
                     }
                     else
                        actionNode.put("executeWithNextAction", postGraspHandConfigurations.get(side).get(i) != null ? true : false);
                     actionNode.put("holdPoseInWorldLater", (i != numberOfPostGraspFrames - 1));

                     double[] dataTrajectories = new double[16];
                     transformToParent.get(dataTrajectories);
                     csvDataMatrices.get(side).add(dataTrajectories);

                     if (postGraspHandConfigurations.get(side).get(i) != null)
                     {
                        dataTrajectories = new double[16];
                        for (int data = 0; data < dataTrajectories.length; data++)
                           dataTrajectories[data] = 0.0;
                        dataTrajectories[0] = SakeHandCommandOption.valueOf(postGraspHandConfigurations.get(side).get(i).toString()).ordinal();
                        csvDataMatrices.get(side).add(dataTrajectories);

                        ObjectNode configurationActionNode = actionsArrayNode.addObject();
                        configurationActionNode.put("type", SakeHandCommandActionDefinition.class.getSimpleName());
                        configurationActionNode.put("description", "Post-grasp " + side.getPascalCaseName() + " Hand Configuration");
                        configurationActionNode.put("side", side.getLowerCaseName());
                        configurationActionNode.put("configuration", postGraspHandConfigurations.get(side).get(i).toString());
                        configurationActionNode.put("position", SakeHandCommandOption.valueOf(postGraspHandConfigurations.get(side).get(i)).getGoalPosition());
                        configurationActionNode.put("torque", SakeHandCommandOption.valueOf(postGraspHandConfigurations.get(side).get(i)).getGoalTorque());
                        configurationActionNode.put("executeWithNextAction", side != RobotSide.RIGHT && bothPosesAreSet);
                     }
                  }
               }
            }
         });
         LogTools.info("Saved to file {}", file.getFileName());
      }
      else
      {
         LogTools.warn("Could not save to {}", file.getFileName());
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

   private void saveFrameInfoToFile(String fileName)
   {
      WorkspaceResourceFile framesFile = new WorkspaceResourceFile(configurationsDirectory, fileName + "Frames.json");
      if (framesFile.isFileAccessAvailable())
      {
         JSONFileTools.save(framesFile, jsonNode ->
         {
            jsonNode.put("name", fileName);

            ArrayNode preGraspArrayNode = jsonNode.putArray("preGraspFrames");
            var preGraspPoses = preGraspFrames.getPoses();
            var arePreGraspPosesSet = preGraspFrames.getArePosesSet();
            int numberOfPreGraspFrames = activeSides.contains(RobotSide.RIGHT) ? preGraspPoses.get(RobotSide.RIGHT).size() : preGraspPoses.get(RobotSide.LEFT).size();
            for (int i = 0; i < numberOfPreGraspFrames; i++)
            {
               for (RobotSide side : activeSides)
               {
                  ObjectNode frameNode = preGraspArrayNode.addObject();
                  frameNode.put("side", side.getLowerCaseName());
                  frameNode.put("isPoseSet", arePreGraspPosesSet.get(side).get(i));
                  preGraspPoses.get(side).get(i).changeFrame(affordanceFrame.getReferenceFrame());
                  RigidBodyTransform transformToParent = new RigidBodyTransform(preGraspPoses.get(side).get(i));
                  JSONTools.toJSON(frameNode, transformToParent);
               }
            }

            ArrayNode graspArrayNode = jsonNode.putArray("graspFrames");
            var graspPoses = graspFrame.getPoses();
            for (RobotSide side : activeSides)
            {
               ObjectNode frameNode = graspArrayNode.addObject();
               frameNode.put("side", side.getLowerCaseName());
               frameNode.put("isPoseSet", graspFrame.isSet(side));
               graspPoses.get(side).changeFrame(affordanceFrame.getReferenceFrame());
               RigidBodyTransform transformToParent = new RigidBodyTransform(graspPoses.get(side));
               JSONTools.toJSON(frameNode, transformToParent);
            }

            ArrayNode postGraspArrayNode = jsonNode.putArray("postGraspFrames");
            var postGraspPoses = postGraspFrames.getPoses();
            var arePostGraspPosesSet = postGraspFrames.getArePosesSet();
            int numberOfPostGraspFrames = activeSides.contains(RobotSide.RIGHT) ? postGraspPoses.get(RobotSide.RIGHT).size() : postGraspPoses.get(RobotSide.LEFT).size();
            for (int i = 0; i < numberOfPostGraspFrames; i++)
            {
               for (RobotSide side : activeSides)
               {
                  ObjectNode frameNode = postGraspArrayNode.addObject();
                  frameNode.put("side", side.getLowerCaseName());
                  frameNode.put("isPoseSet", arePostGraspPosesSet.get(side).get(i));
                  postGraspPoses.get(side).get(i).changeFrame(affordanceFrame.getReferenceFrame());
                  RigidBodyTransform transformToParent = new RigidBodyTransform(postGraspPoses.get(side).get(i));
                  JSONTools.toJSON(frameNode, transformToParent);
               }
            }
         });
         LogTools.info("SAVED to file {}", framesFile.getFileName());
      }
      else
      {
         LogTools.warn("Could not save to {}", framesFile.getFileName());
      }
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
            int numberActiveSides = (activeSides.contains(RobotSide.RIGHT) && activeSides.contains(RobotSide.LEFT)) ? 2 : 1;
            jsonNode.put("activeSides", numberActiveSides);
            var preGraspObjectTransforms = preGraspFrames.getObjectTransforms();
            var preGraspHandConfigurations = preGraspFrames.getHandConfigurations();
            jsonNode.put("numberPreGraspFrames", preGraspObjectTransforms.size());
            for (int i = 0; i < preGraspObjectTransforms.size(); i++)
            {
               ObjectNode frameArray = framesArrayNode.addObject();
               JSONTools.toJSON(frameArray, preGraspObjectTransforms.get(i));

               ArrayNode handConfigurationArrayNode = frameArray.putArray("configuration");
               for (RobotSide side : activeSides)
               {
                  ObjectNode gripArray = handConfigurationArrayNode.addObject();
                  gripArray.put("side", side.getLowerCaseName());
                  gripArray.put("type", preGraspHandConfigurations.get(side).get(i) == null ? "" : preGraspHandConfigurations.get(side).get(i).toString());
               }
            }

            if (graspFrame.isSet(RobotSide.LEFT) || graspFrame.isSet(RobotSide.RIGHT))
            {
               ObjectNode frameArray = framesArrayNode.addObject();
               JSONTools.toJSON(frameArray, graspFrame.getObjectTransform());
               ArrayNode handConfigurationArrayNode = frameArray.putArray("configuration");
               for (RobotSide side : activeSides)
               {
                  ObjectNode gripArray = handConfigurationArrayNode.addObject();
                  gripArray.put("side", side.getLowerCaseName());
                  gripArray.put("type", graspFrame.getHandConfiguration(side) == null ? "" : graspFrame.getHandConfiguration(side).toString());
               }
            }

            var postGraspObjectTransforms = postGraspFrames.getObjectTransforms();
            var postGraspHandConfigurations = postGraspFrames.getHandConfigurations();
            jsonNode.put("numberPostGraspFrames", postGraspObjectTransforms.size());
            for (int i = 0; i < postGraspObjectTransforms.size(); i++)
            {
               ObjectNode frameArray = framesArrayNode.addObject();
               JSONTools.toJSON(frameArray, postGraspObjectTransforms.get(i));
               ArrayNode handConfigurationArrayNode = frameArray.putArray("configuration");
               for (RobotSide side : activeSides)
               {
                  ObjectNode gripArray = handConfigurationArrayNode.addObject();
                  gripArray.put("side", side.getLowerCaseName());
                  gripArray.put("type", postGraspHandConfigurations.get(side).get(i) == null ? "" : postGraspHandConfigurations.get(side).get(i).toString());
               }
            }
            if (!objectBuilder.getSelectedObject().getShape().toString().equals("NULL"))
            {
               jsonNode.put("resizablePrimitiveShape", objectBuilder.getSelectedObject().getShape().toString());
               jsonNode.put("xLength", objectBuilder.getSelectedObject().getResizablePrimitiveSize().get(0));
               jsonNode.put("yLength", objectBuilder.getSelectedObject().getResizablePrimitiveSize().get(1));
               jsonNode.put("zLength", objectBuilder.getSelectedObject().getResizablePrimitiveSize().get(2));
               jsonNode.put("xRadius", objectBuilder.getSelectedObject().getResizablePrimitiveSize().get(3));
               jsonNode.put("yRadius", objectBuilder.getSelectedObject().getResizablePrimitiveSize().get(4));
               jsonNode.put("zRadius", objectBuilder.getSelectedObject().getResizablePrimitiveSize().get(5));
            }
            else
            {
               jsonNode.put("resizablePrimitiveShape", "NULL");
            }
         });
         LogTools.info("SAVED to file {}", extraFile.getFileName());
      }
      else
      {
         LogTools.warn("Could not save extra info to {}", extraFile.getFileName());
      }
   }

   public void setLoadingFile(String fileName)
   {
      String suffixToRemove = ".json";
      if (fileName.endsWith(suffixToRemove))
         loadingFileName = fileName.substring(0, fileName.length() - suffixToRemove.length());
      else
         loadingFileName = fileName;
   }

   public String getLoadingFile()
   {
      String suffixToAdd = ".json";
      return (loadingFileName + suffixToAdd);
   }

   public void load()
   {
      if(loadingFileName.isEmpty())
         LogTools.warn("Cannot load file - Please select a file from the radio button list");
      else
      {
         Path filePath = Paths.get(configurationsDirectory.getFilesystemDirectory().toString(), loadingFileName + "Extra.json");
         loadExtraFile(filePath);
         loadFramesFile(filePath);
      }
   }

   private void loadExtraFile(Path filePath)
   {
      if (Files.exists(filePath))
      {
         JSONFileTools.load(filePath, jsonNode ->
         {
            int preGraspFramesSize;
            int postGraspFramesSize;
            String objectName = jsonNode.get("object").asText();
            if (!objectName.isEmpty())
            {
               RigidBodyTransform initialTransform = new RigidBodyTransform();
               JSONTools.toEuclid(jsonNode, initialTransform);
               objectBuilder.getSelectedObject().setPose(initialTransform);
            }
            preGraspFramesSize = jsonNode.get("numberPreGraspFrames").asInt();
            postGraspFramesSize = jsonNode.get("numberPostGraspFrames").asInt();
            numberActiveSides[0] = jsonNode.get("activeSides").asInt();
            JsonNode framesArrayNode = jsonNode.get("frames");
            for (int i = 0; i < preGraspFramesSize; i++)
            {
               RigidBodyTransform preGraspObjectTransform = new RigidBodyTransform();
               JSONTools.toEuclid(framesArrayNode.get(i), preGraspObjectTransform);
               preGraspFrames.addObjectTransform(preGraspObjectTransform);

               JsonNode handConfigurationArrayNode = framesArrayNode.get(i).get("configuration");
               for (int sideIndex = 0; sideIndex < numberActiveSides[0]; sideIndex++)
               {
                  RobotSide side = getSideFromString((handConfigurationArrayNode.get(sideIndex).get("side").asText()));
                  String configuration = handConfigurationArrayNode.get(sideIndex).get("type").asText();
                  preGraspFrames.addHandConfiguration(configuration.isEmpty() ? null : configuration, side);
               }
            }

            RigidBodyTransform graspObjectTransform = new RigidBodyTransform();
            JSONTools.toEuclid(framesArrayNode.get(preGraspFramesSize), graspObjectTransform);
            graspFrame.setObjectTransform(graspObjectTransform);
            JsonNode handConfigurationArrayNode = framesArrayNode.get(preGraspFramesSize).get("configuration");
            for (int sideIndex = 0; sideIndex < numberActiveSides[0]; sideIndex++)
            {
               RobotSide side = getSideFromString((handConfigurationArrayNode.get(sideIndex).get("side").asText()));
               String configuration = handConfigurationArrayNode.get(sideIndex).get("type").asText();
               graspFrame.setHandConfiguration(configuration.isEmpty() ? null : configuration, side);
            }

            for (int i = 0; i < postGraspFramesSize; i++)
            {
               RigidBodyTransform postGraspObjectTransform = new RigidBodyTransform();
               JSONTools.toEuclid(framesArrayNode.get(i + preGraspFramesSize + 1), postGraspObjectTransform);
               postGraspFrames.addObjectTransform(postGraspObjectTransform);

               handConfigurationArrayNode = framesArrayNode.get(i + preGraspFramesSize + 1).get("configuration");
               for (int sideIndex = 0; sideIndex < numberActiveSides[0]; sideIndex++)
               {
                  RobotSide side = getSideFromString((handConfigurationArrayNode.get(sideIndex).get("side").asText()));
                  String configuration = handConfigurationArrayNode.get(sideIndex).get("type").asText();
                  postGraspFrames.addHandConfiguration(configuration.isEmpty() ? null : configuration, side);
               }
            }

            String resizablePrimitiveShape = jsonNode.get("resizablePrimitiveShape").asText();
            if (!resizablePrimitiveShape.equals("NULL"))
            {
               List<Float> readPrimitiveSize = new ArrayList<>();
               readPrimitiveSize.add(jsonNode.get("xLength").floatValue());
               readPrimitiveSize.add(jsonNode.get("yLength").floatValue());
               readPrimitiveSize.add(jsonNode.get("zLength").floatValue());
               readPrimitiveSize.add(jsonNode.get("xRadius").floatValue());
               readPrimitiveSize.add(jsonNode.get("yRadius").floatValue());
               readPrimitiveSize.add(jsonNode.get("zRadius").floatValue());
               objectBuilder.getSelectedObject().setReadResizablePrimitiveSize(readPrimitiveSize);
            }
         });
         LogTools.info("Loaded file {}", filePath);
      }
      else
      {
         LogTools.warn("Could not load file {}", filePath);
      }
   }

   private void loadFramesFile(Path filePath)
   {
      // change affordance reference from whatever it is now to loaded initial object pose
      RigidBodyTransform initialObjectTransform = new RigidBodyTransform(objectBuilder.getSelectedObject().getInitialTransformToWorld());
      initialObjectFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(ReferenceFrame.getWorldFrame(), initialObjectTransform);
      affordanceFrame.setParentFrame(initialObjectFrame);

      filePath = Paths.get(configurationsDirectory.getFilesystemDirectory().toString(), loadingFileName + "Frames.json");
      if (Files.exists(filePath))
      {
         JSONFileTools.load(filePath, jsonNode ->
         {
            index[0] = 0;
            JSONTools.forEachArrayElement(jsonNode, "preGraspFrames", frameNode ->
            {
               RobotSide side = RobotSide.getSideFromString(frameNode.get("side").asText());
               RigidBodyTransform frameTransform = new RigidBodyTransform();
               JSONTools.toEuclid(frameNode, frameTransform);
               var arePreGraspPosesSet = preGraspFrames.getArePosesSet();
               arePreGraspPosesSet.get(side).add(frameNode.get("isPoseSet").asBoolean());
               if (numberActiveSides[0] > 1)
               {
                  index[0] = index[0] + 0.5;
                  preGraspFrames.loadFrame(new FramePose3D(affordanceFrame.getReferenceFrame(), frameTransform), side, (int) Math.ceil(index[0]));
                  if (Math.ceil(index[0]) - index[0] > 0)
                     preGraspFrames.addIndexPose((int) Math.ceil(index[0]));
               }
               else
               {
                  index[0]++;
                  preGraspFrames.loadFrame(new FramePose3D(affordanceFrame.getReferenceFrame(), frameTransform), side, (int) index[0]);
                  preGraspFrames.addIndexPose((int) index[0]);
               }
            });

            JSONTools.forEachArrayElement(jsonNode, "graspFrames", frameNode ->
            {
               RobotSide side = RobotSide.getSideFromString(frameNode.get("side").asText());
               RigidBodyTransform frameTransform = new RigidBodyTransform();
               JSONTools.toEuclid(frameNode, frameTransform);
               graspFrame.setFrame(new FramePose3D(affordanceFrame.getReferenceFrame(), frameTransform), side);
            });

            index[0] = 0;
            JSONTools.forEachArrayElement(jsonNode, "postGraspFrames", frameNode ->
            {
               RobotSide side = RobotSide.getSideFromString(frameNode.get("side").asText());
               RigidBodyTransform frameTransform = new RigidBodyTransform();
               JSONTools.toEuclid(frameNode, frameTransform);
               var arePostGraspPosesSet = postGraspFrames.getArePosesSet();
               arePostGraspPosesSet.get(side).add(frameNode.get("isPoseSet").asBoolean());
               if (numberActiveSides[0] > 1)
               {
                  index[0] = index[0] + 0.5;
                  postGraspFrames.loadFrame(new FramePose3D(affordanceFrame.getReferenceFrame(), frameTransform), side, (int) Math.ceil(index[0]));
                  if (Math.ceil(index[0]) - index[0] > 0)
                     postGraspFrames.addIndexPose((int) Math.ceil(index[0]));
               }
               else
               {
                  index[0]++;
                  postGraspFrames.loadFrame(new FramePose3D(affordanceFrame.getReferenceFrame(), frameTransform), side, (int) index[0]);
                  postGraspFrames.addIndexPose((int) index[0]);
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

   public String getConfigurationDirectory()
   {
      return configurationsDirectory.getFilesystemDirectory().toString();
   }
}