package us.ihmc.behaviors.sharedControl;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkspaceDirectory;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Class to pack a teleoperated referenceFrame and modify it by using some assistance from the robot.
 * The assistance comes from pre-trained probabilistic models: the ProMPs,
 * which represent a probabilistic prediction of multidimensional trajectories.
 * Initially the input from the user is not modified and is simply observed to produce a fitting prediction.
 * Once the task is detected (by recognizing the protagonist object of that task and/or by observing the current motion of the user),
 * the teleoperated referenceFrame gradually shifts from the reference specified by the user to the predicted assistance of the ProMPs.
 */
public class ProMPAssistant
{
   private final HashMap<String, ProMPManager> proMPManagers = new HashMap<>(); //proMPManagers stores a proMPManager for each task
   private String currentTask = ""; // detected task
   private int numberObservations = 0; // number of observations used to update the prediction
   private String bodyPartRecognition = "";
   private String bodyPartGoal = "";
   private final HashMap<String, String> taskBodyPartRecognitionMap = new HashMap<>();
   private final HashMap<String, String> taskBodyPartGoalMap = new HashMap<>();
   private Pose3DReadOnly taskGoalPose;
   private final HashMap<String, List<Pose3DReadOnly>> bodyPartObservedTrajectoryMap = new HashMap<>();
   private final HashMap<String, List<FramePose3D>> bodyPartGeneratedTrajectoryMap = new HashMap<>();
   private final HashMap<String, Integer> bodyPartTrajectorySampleCounter = new HashMap<>(); // to track the last used sample of a generated trajectory
   private boolean doneInitialProcessingTask = false;
   private boolean doneCurrentTask = false; // used to communicate to higher level classes that the task has been completed
   private final AtomicBoolean isLastViaPoint = new AtomicBoolean(false); // check if last observed viapoint before update
   private int testNumber = 0;
   private boolean conditionOnlyLastObservation = true;

   public ProMPAssistant()
   {
      List<String> taskNames = new ArrayList<>();
      List<String> bodyPartsRecognition = new ArrayList<>();
      List<String> bodyPartsGoal = new ArrayList<>();
      List<HashMap<String, String>> bodyPartsGeometries = new ArrayList<>();
      boolean logEnabled = false;
      // read parameters regarding the properties of available learned tasks from json file
      try
      {
         WorkspaceDirectory directory = new WorkspaceDirectory("ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/main/resources");
         String directoryAbsolutePath = directory.getDirectoryPath().toAbsolutePath().toString();
         String configurationFile = "/us/ihmc/behaviors/sharedControl/ProMPAssistant.json";
         String demoDirectory = directoryAbsolutePath + configurationFile;
         Path pathFile = Paths.get(demoDirectory);
         LogTools.info("Loading parameters from resource: {}", configurationFile);
         JSONObject jsonObject = (JSONObject) new JSONParser().parse(new FileReader(pathFile.toAbsolutePath().toString()));
         testNumber = (int) ((long) jsonObject.get("testNumberUseOnlyForTesting"));
         logEnabled = (boolean) jsonObject.get("logging");
         numberObservations = (int) ((long) jsonObject.get("numberObservations"));
         conditionOnlyLastObservation = (boolean) jsonObject.get("conditionOnlyLastObservation");
         // getting tasks
         JSONArray tasksArray = (JSONArray) jsonObject.get("tasks");
         // iterating tasks
         for (Object taskObject : tasksArray)
         {
            for (Map.Entry taskPropertyMap : (Iterable<Map.Entry>) ((Map) taskObject).entrySet())
            {
               switch (taskPropertyMap.getKey().toString())
               {
                  case "name" -> taskNames.add((String) taskPropertyMap.getValue());
                  case "bodyPartForRecognition" -> bodyPartsRecognition.add((String) taskPropertyMap.getValue());
                  case "bodyPartWithObservableGoal" -> bodyPartsGoal.add((String) taskPropertyMap.getValue());
                  case "bodyParts" ->
                  {
                     JSONArray bodyPartsArray = (JSONArray) taskPropertyMap.getValue();
                     HashMap<String, String> bodyPartsGeometry = new HashMap<>();
                     //parse body parts
                     for (Object bodyPartObject : bodyPartsArray)
                     {
                        JSONObject jsonBodyPartObject = (JSONObject) bodyPartObject;
                        List<String> name = new ArrayList<>();
                        List<String> geometry = new ArrayList<>();
                        jsonBodyPartObject.keySet().forEach(bodyPartProperty ->
                                                            {
                                                               switch (bodyPartProperty.toString())
                                                               {
                                                                  case "name" -> name.add(String.valueOf((jsonBodyPartObject.get(bodyPartProperty))));
                                                                  case "geometry" -> geometry.add(String.valueOf(jsonBodyPartObject.get(bodyPartProperty)));
                                                                  default ->
                                                                  {
                                                                  }
                                                               }
                                                            });
                        for (int i = 0; i < name.size(); i++)
                           bodyPartsGeometry.put(name.get(i), geometry.get(i));
                     }
                     bodyPartsGeometries.add(bodyPartsGeometry);
                  }
                  default ->
                  {
                  }
               }
            }
         }
         int numberBasisFunctions = (int) ((long) jsonObject.get("numberBasisFunctions"));
         long speedFactor = ((long) jsonObject.get("allowedIncreaseDecreaseSpeedFactor"));
         int numberOfInferredSpeeds = (int) ((long) jsonObject.get("numberOfInferredSpeeds"));
         for (int i = 0; i < taskNames.size(); i++)
         {
            LogTools.info("Learning ProMPs for task: {}", taskNames.get(i));
            for (HashMap<String, String> bodyPartsGeometry : bodyPartsGeometries)
            {
               for (String key : bodyPartsGeometry.keySet())
               {
                  LogTools.info("     {} {}", key, bodyPartsGeometry.get(key));
               }
            }
            proMPManagers.put(taskNames.get(i),
                              new ProMPManager(taskNames.get(i),
                                               bodyPartsGeometries.get(i),
                                               logEnabled,
                                               isLastViaPoint,
                                               numberBasisFunctions,
                                               speedFactor,
                                               numberOfInferredSpeeds));
            taskBodyPartRecognitionMap.put(taskNames.get(i), bodyPartsRecognition.get(i));
            taskBodyPartGoalMap.put(taskNames.get(i), bodyPartsGoal.get(i));
         }
         for (ProMPManager proMPManager : proMPManagers.values())
            proMPManager.learnTaskFromDemos();
         LogTools.info("ProMPs are ready to be used!");
      }
      catch (FileNotFoundException ex)
      {
         ex.printStackTrace();
      }
      catch (IOException | ParseException e)
      {
         throw new RuntimeException(e);
      }
   }

   public void processFrameAndObjectInformation(Pose3DReadOnly observedPose, String bodyPart, Pose3DReadOnly objectPose, String objectName)
   {
      LogTools.info("out processing");
      if (taskDetected(objectName))
      {
         LogTools.info("PROCESSING");
         if ((proMPManagers.get(currentTask).getBodyPartsGeometry()).containsKey(bodyPart))
         { // if bodyPart is used in current task
            if (!bodyPartGoal.isEmpty() && objectPose!=null) // if there is an observable goal this body part can reach
               taskGoalPose = objectPose;
            // store observed pose
            Pose3D lastObservedPose = new Pose3D();
            lastObservedPose.getPosition().set(observedPose.getPosition().getX(), observedPose.getPosition().getY(), observedPose.getPosition().getZ());
            lastObservedPose.getOrientation()
                            .set(observedPose.getOrientation().getX(),
                                 observedPose.getOrientation().getY(),
                                 observedPose.getOrientation().getZ(),
                                 observedPose.getOrientation().getS());
            bodyPartObservedTrajectoryMap.get(bodyPart).add(lastObservedPose);

            // update the proMP prediction according to observations and generate mean trajectory
            if (bodyPartObservedTrajectoryMap.get(bodyPart).size() > numberObservations) // if observed a sufficient number of poses
            {
               updateTask();
               generateTaskTrajectories();
               doneInitialProcessingTask = true;
               LogTools.info("Generating prediction ...");
            }
         }
      }
   }

   private boolean taskDetected(String objectName)
   {
      if (currentTask.isEmpty() && !objectName.isEmpty())
      {
         // TODO A.1. if multiple tasks are available for a single object, use also promp-to-object initial values to identify correct task
         // TODO B.1. what if someone is lefthanded, or simply wants to use the left hand for that task?
         //  Learn task for both hands and called them ...L and ...R, just check initial velocity of hands to determine which one is being used
         currentTask = objectName;
         // get the body part used for recognition for this task
         bodyPartRecognition = taskBodyPartRecognitionMap.get(currentTask);
         // get the body part that has to reach a goal for this task
         bodyPartGoal = taskBodyPartGoalMap.get(currentTask);

         // initialize bodyPartObservedFrameTrajectory that will contain for each body part a list of observed FramePoses
         for (String bodyPart : (proMPManagers.get(currentTask).getBodyPartsGeometry()).keySet())
            bodyPartObservedTrajectoryMap.put(bodyPart, new ArrayList<>());
      }
      return !currentTask.isEmpty();
   }

   private void updateTask()
   {
      //      if(!bodyPartGoal.isEmpty() && taskGoalPose != null)
      //         proMPManagers.get(currentTask).updateTaskSpeed(bodyPartObservedTrajectoryMap.get(bodyPartGoal), taskGoalPose, bodyPartGoal);
      //      else
      proMPManagers.get(currentTask).updateTaskSpeed(bodyPartObservedTrajectoryMap.get(bodyPartRecognition), bodyPartRecognition);
      // update all proMP trajectories based on initial observations (stored observed poses)
      for (String robotPart : bodyPartObservedTrajectoryMap.keySet())
      {
         List<Pose3DReadOnly> observedTrajectory = bodyPartObservedTrajectoryMap.get(robotPart);
         if (conditionOnlyLastObservation)
         {
            if (observedTrajectory.size() > 0)
            {
               isLastViaPoint.set(true);
               proMPManagers.get(currentTask)
                            .updateTaskTrajectory(robotPart, observedTrajectory.get(observedTrajectory.size() - 1), observedTrajectory.size() - 1);
            }
         }
         else
         {
            for (int i = 0; i < observedTrajectory.size(); i++)
            {
               if (i == observedTrajectory.size() - 1)
                  isLastViaPoint.set(true);
               proMPManagers.get(currentTask).updateTaskTrajectory(robotPart, observedTrajectory.get(i), i);
            }
         }
      }
      if (taskGoalPose != null) // if there is an observable goal this body part can reach
      {
         // update only proMP trajectory of the body part relevant for goal of the task, based on observed goal
         proMPManagers.get(currentTask).updateTaskTrajectoryGoal(bodyPartGoal, taskGoalPose);
      }
   }

   private void generateTaskTrajectories()
   {
      // for each body part generate the mean trajectory of the learned promp
      for (String bodyPart : bodyPartObservedTrajectoryMap.keySet())
      {
         bodyPartGeneratedTrajectoryMap.put(bodyPart, proMPManagers.get(currentTask).generateTaskTrajectory(bodyPart));
         // start using it after the last sample we observed, not from the beginning. We do not want to restart the motion
         bodyPartTrajectorySampleCounter.put(bodyPart, numberObservations);
      }
   }

   public boolean readyToPack()
   {
      return doneInitialProcessingTask;
   }

   public void framePoseToPack(FramePose3D framePose, String bodyPart)
   {
      if ((proMPManagers.get(currentTask).getBodyPartsGeometry()).containsKey(bodyPart))
      { // if bodyPart is used in current task
         List<FramePose3D> generatedFramePoseTrajectory = bodyPartGeneratedTrajectoryMap.get(bodyPart);
         int sampleCounter = bodyPartTrajectorySampleCounter.get(bodyPart);
         if (sampleCounter < generatedFramePoseTrajectory.size())
         {
            // take a sample (frame) from the trajectory
            FramePose3D generatedFramePose = generatedFramePoseTrajectory.get(bodyPartTrajectorySampleCounter.get(bodyPart));
            FixedFrameQuaternionBasics generatedFrameOrientation = generatedFramePose.getOrientation();
            FixedFramePoint3DBasics generatedFramePosition = generatedFramePose.getPosition();
            framePose.getPosition().set(generatedFramePosition);
            framePose.getOrientation().set(generatedFrameOrientation);

            // take the next sample from the trajectory next time
            bodyPartTrajectorySampleCounter.replace(bodyPart, bodyPartTrajectorySampleCounter.get(bodyPart) + 1);
         }
         else
         {
            // check that inferred timesteps are not lower than the observed setpoints.
            if (generatedFramePoseTrajectory.size() < numberObservations)
            {
               // the predicted motion is already over before being available and assistance should be exited
               String configurationFile = "us/ihmc/behaviors/sharedControl/ProMPAssistant.json";
               LogTools.warn(
                     "The predicted motion results being faster than the time set to observe it. You can either decrease the number of required observations or increase the range of possible inferred speeds in {}",
                     configurationFile);
            }
            else
            {
               // take previous sample (frame) to avoid jump when exiting assistance mode
               FramePose3D generatedFramePose = generatedFramePoseTrajectory.get(bodyPartTrajectorySampleCounter.get(bodyPart) - 1);
               FixedFrameQuaternionBasics generatedFrameOrientation = generatedFramePose.getOrientation();
               FixedFramePoint3DBasics generatedFramePosition = generatedFramePose.getPosition();
               framePose.getPosition().set(generatedFramePosition);
               framePose.getOrientation().set(generatedFrameOrientation);
            }
            // exit assistance mode
            doneCurrentTask = true;
         }
      }
   }

   public void reset()
   {
      // reset manager of current task (reset reference of proMP object of current task to initial proMP before any conditioning)
      if (!currentTask.isEmpty())
      {
         proMPManagers.get(currentTask).resetTask();
         currentTask = "";
      }
      taskGoalPose = null;
      bodyPartObservedTrajectoryMap.clear();
      bodyPartGeneratedTrajectoryMap.clear();
      bodyPartTrajectorySampleCounter.clear();
      doneInitialProcessingTask = false;
      isLastViaPoint.set(false);
   }

   public int getTestNumber()
   {
      return testNumber;
   }

   public void setCurrentTaskDone(boolean doneCurrentTask)
   {
      this.doneCurrentTask = doneCurrentTask;
   }

   public boolean isCurrentTaskDone()
   {
      return this.doneCurrentTask;
   }

   public ProMPManager getProMPManager(String task)
   {
      return proMPManagers.get(task);
   }

   public Set<String> getTaskNames()
   {
      return proMPManagers.keySet();
   }
}
