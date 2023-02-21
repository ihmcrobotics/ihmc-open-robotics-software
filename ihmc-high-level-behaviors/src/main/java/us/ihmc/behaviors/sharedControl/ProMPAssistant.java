package us.ihmc.behaviors.sharedControl;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
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
   private final HashMap<String, ProMPManager> proMPManagers = new HashMap<>(); // proMPManagers stores a proMPManager for each task
   private final HashMap<String, List<String>> contextTasksMap = new HashMap<>(); // map to store all the tasks available for each context (object)
   private String currentTask = ""; // detected task
   private int numberObservations = 0; // number of observations used to update the prediction
   private String bodyPartRecognition = "";
   private String bodyPartGoal = "";
   private final HashMap<String, String> taskBodyPartRecognitionMap = new HashMap<>();
   private final HashMap<String, String> taskBodyPartGoalMap = new HashMap<>();
   private final HashMap<String, RigidBodyTransform> taskTransformGoalMap = new HashMap<>();
   private FramePose3D taskGoalPose;
   private final HashMap<String, List<FramePose3D>> bodyPartObservedTrajectoryMap = new HashMap<>();
   private final HashMap<String, List<FramePose3D>> bodyPartGeneratedTrajectoryMap = new HashMap<>();
   private final HashMap<String, Integer> bodyPartTrajectorySampleCounter = new HashMap<>(); // to track the last used sample of a generated trajectory
   private boolean doneInitialProcessingTask = false;
   private boolean doneCurrentTask = false; // used to communicate to higher level classes that the task has been completed
   private final AtomicBoolean isLastViaPoint = new AtomicBoolean(false); // check if last observed viapoint before update
   private int testNumber = 0;
   private boolean conditionOnlyLastObservation = true;
   private final ArrayList<Pose3DReadOnly> observationRecognitionPart = new ArrayList<>();
   private boolean isMoving = false;
   private ReferenceFrame objectFrame;
   private double isMovingThreshold = -1;

   public ProMPAssistant()
   {
      String lastContext = "";
      List<String> taskNames = new ArrayList<>();
      List<String> bodyPartsRecognition = new ArrayList<>();
      List<String> bodyPartsGoal = new ArrayList<>();
      List<HashMap<String, String>> bodyPartsGeometries = new ArrayList<>();
      List<Point3D> goalToEETranslations = new ArrayList<>();
      List<Quaternion> goalToEERotations = new ArrayList<>();
      boolean logEnabled;
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
                  case "context" ->
                  {
                     String context = (String) taskPropertyMap.getValue();
                     if(!contextTasksMap.containsKey(context))
                        contextTasksMap.put(context,new ArrayList<>());
                     lastContext = context;
                  }
                  case "name" -> taskNames.add((String) taskPropertyMap.getValue());
                  case "bodyPartForRecognition" -> bodyPartsRecognition.add((String) taskPropertyMap.getValue());
                  case "bodyPartWithObservableGoal" -> bodyPartsGoal.add((String) taskPropertyMap.getValue());
                  case "translationGoalToEE" ->
                  {
                     JSONArray translationArray = (JSONArray) taskPropertyMap.getValue();
                     Iterator translationIterator = translationArray.iterator();
                     List<Double> translation = new ArrayList<>(3);
                     while (translationIterator.hasNext())
                        translation.add((Double) translationIterator.next());
                     goalToEETranslations.add(new Point3D(translation.get(0), translation.get(1), translation.get(2)));
                  }
                  case "rotationGoalToEE" ->
                  {
                     JSONArray rotationArray = (JSONArray) taskPropertyMap.getValue();
                     Iterator rotationIterator = rotationArray.iterator();
                     List<Double> rotation = new ArrayList<>(4);
                     while (rotationIterator.hasNext())
                        rotation.add((Double) rotationIterator.next());
                     goalToEERotations.add(new Quaternion(rotation.get(0), rotation.get(1), rotation.get(2), rotation.get(3)));
                  }
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
            // in contextTaskMap add the last task to the last context
            contextTasksMap.get(lastContext).add(taskNames.get(taskNames.size() - 1));
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
            taskTransformGoalMap.put(taskNames.get(i), new RigidBodyTransform(goalToEERotations.get(i), goalToEETranslations.get(i)));
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

   public void processFrameAndObjectInformation(Pose3DReadOnly observedPose, String bodyPart, ReferenceFrame objectFrame, String objectName)
   {
      if (taskDetected(objectName))
      {
         if ((proMPManagers.get(currentTask).getBodyPartsGeometry()).containsKey(bodyPart)) // if bodyPart is used in current task
         {
            FramePose3D lastObservedPose = new FramePose3D();
            lastObservedPose.getPosition().set(observedPose.getPosition().getX(), observedPose.getPosition().getY(), observedPose.getPosition().getZ());
            lastObservedPose.getOrientation()
                            .set(observedPose.getOrientation().getX(),
                                 observedPose.getOrientation().getY(),
                                 observedPose.getOrientation().getZ(),
                                 observedPose.getOrientation().getS());

            if (!bodyPartGoal.isEmpty() && objectFrame != null) // if there is an observable object and goal
            {
               // change observed pose in object frame
               this.objectFrame = objectFrame;
               lastObservedPose.changeFrame(objectFrame);
            }
            if (userIsMoving(lastObservedPose, bodyPart)) // check if user has started moving after activating the assistance (after pressing the button)
            {
               // store observed pose
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
   }

   public void processFrameAndObjectInformation(Pose3DReadOnly observedPose, String bodyPart, FramePose3D objectPose, String objectName)
   {
      if (taskDetected(objectName))
      {
         if ((proMPManagers.get(currentTask).getBodyPartsGeometry()).containsKey(bodyPart)) // if bodyPart is used in current task
         {
            FramePose3D lastObservedPose = new FramePose3D();
            lastObservedPose.getPosition().set(observedPose.getPosition().getX(), observedPose.getPosition().getY(), observedPose.getPosition().getZ());
            lastObservedPose.getOrientation()
                            .set(observedPose.getOrientation().getX(),
                                 observedPose.getOrientation().getY(),
                                 observedPose.getOrientation().getZ(),
                                 observedPose.getOrientation().getS());
            if (userIsMoving(lastObservedPose, bodyPart)) // check if user has started moving after activating the assistance (pressed the button)
            {
               // store observed pose
               bodyPartObservedTrajectoryMap.get(bodyPart).add(lastObservedPose);
               // update the proMP prediction according to observations and generate mean trajectory
               if (bodyPartObservedTrajectoryMap.get(bodyPart).size() > numberObservations) // if observed a sufficient number of poses
               {
                  if (!bodyPartGoal.isEmpty() && objectPose != null) // if there is an observable goal this body part can reach
                  {
                     taskGoalPose = new FramePose3D(objectPose);
                     taskGoalPose.appendTransform(taskTransformGoalMap.get(currentTask));
                     // check resulting frame is not in the wrong 2pi range of quaternion [-2pi,2pi]. q = -q but the ProMPs do not know that
                     if (Math.signum(proMPManagers.get(currentTask).getMeanEndValueQS() * taskGoalPose.getOrientation().getS()) == -1)
                        taskGoalPose.getOrientation().negate();
                  }
                  updateTask();
                  generateTaskTrajectories();
                  doneInitialProcessingTask = true;
                  LogTools.info("Generating prediction ...");
               }
            }
         }
      }
   }

   private boolean taskDetected(String objectName)
   {
      if (currentTask.isEmpty() && !objectName.isEmpty())
      {
         if(contextTasksMap.containsKey(objectName))
         {
            List<String> candidateTasks = contextTasksMap.get(objectName);
            if(candidateTasks.size() > 0)
            {
               // TODO if multiple tasks are available for a single object, use also promp-to-object initial values to identify correct task
               currentTask = candidateTasks.get(0);
               // TODO what if someone is lefthanded, or simply wants to use the left hand for that task?
               //  Learn task for both hands and called them ...L and ...R, check initial velocity of hands to determine which one is being used
               // get the body part used for recognition for this task
               bodyPartRecognition = taskBodyPartRecognitionMap.get(currentTask);
               // get the body part that has to reach a goal for this task
               bodyPartGoal = taskBodyPartGoalMap.get(currentTask);

               // initialize bodyPartObservedFrameTrajectory that will contain for each body part a list of observed FramePoses
               for (String bodyPart : (proMPManagers.get(currentTask).getBodyPartsGeometry()).keySet())
                  bodyPartObservedTrajectoryMap.put(bodyPart, new ArrayList<>());
            }
            else
            {
               LogTools.warn("Detected object ({}) should but does NOT have any associated learned policy for assistance. Check file ProMPAssistant.json", objectName);
               return false;
            }
         }
         else
         {
            LogTools.info("Detected object ({}) does not have any associated learned policy for assistance", objectName);
            return false;
         }

      }
      return !currentTask.isEmpty();
   }

   private boolean userIsMoving(Pose3DReadOnly lastObservedPose, String bodyPart)
   {
      if (bodyPart.equals(bodyPartRecognition) && !isMoving)
      {
         observationRecognitionPart.add(lastObservedPose);
         if (observationRecognitionPart.size() > 1)
         {
            double distance = (observationRecognitionPart.get(observationRecognitionPart.size() - 1)).getTranslation()
                                                                                                     .distance(observationRecognitionPart.get(0)
                                                                                                                                         .getTranslation());
            if (isMovingThreshold < 0)
               isMoving = distance > 0.04;
            else
               isMoving = distance > isMovingThreshold;
            LogTools.info("Is user moving? {}, body part moved by {}[m]", isMoving, distance);
         }
      }
      return isMoving;
   }

   private void updateTask()
   {
      proMPManagers.get(currentTask).updateTaskSpeed(bodyPartObservedTrajectoryMap.get(bodyPartRecognition), bodyPartRecognition);
      // update all proMP trajectories based on initial observations (stored observed poses)
      for (String robotPart : bodyPartObservedTrajectoryMap.keySet())
      {
         List<FramePose3D> observedTrajectory = bodyPartObservedTrajectoryMap.get(robotPart);
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
         if (objectFrame != null)
            bodyPartGeneratedTrajectoryMap.put(bodyPart, proMPManagers.get(currentTask).generateTaskTrajectory(bodyPart, objectFrame));
         else
            bodyPartGeneratedTrajectoryMap.put(bodyPart, proMPManagers.get(currentTask).generateTaskTrajectory(bodyPart, ReferenceFrame.getWorldFrame()));
         // start using it after the last sample we observed, not from the beginning. We do not want to restart the motion
         setStartTrajectories(numberObservations);
      }
   }

   public void setStartTrajectories(int sample)
   {
      doneCurrentTask = false;
      // for each body part generate the mean trajectory of the learned promp
      for (String bodyPart : bodyPartObservedTrajectoryMap.keySet())
         bodyPartTrajectorySampleCounter.put(bodyPart, sample);
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
         List<FramePose3D> observedFramePoseTrajectory = bodyPartObservedTrajectoryMap.get(bodyPart);
         int sampleCounter = bodyPartTrajectorySampleCounter.get(bodyPart);
         if (sampleCounter < generatedFramePoseTrajectory.size())
         {
            if (sampleCounter < numberObservations) // this statement is true only during replay preview
            { // replay the observed motion, do not want the unconditioned ProMP mean for the first part
               FramePose3D observedFramePose = observedFramePoseTrajectory.get(sampleCounter);
               if (objectFrame != null) // change to object frame if one is available
                  observedFramePose.changeFrame(ReferenceFrame.getWorldFrame());
               framePose.getPosition().set(observedFramePose.getPosition());
               framePose.getOrientation().set(observedFramePose.getOrientation());
            }
            else
            { // pack the frame using the trajectory generated from prediction
               FramePose3D generatedFramePose = generatedFramePoseTrajectory.get(sampleCounter);
               if (objectFrame != null) // change back to world frame if before it was changed to object frame
                  generatedFramePose.changeFrame(ReferenceFrame.getWorldFrame());
               framePose.getPosition().set(generatedFramePose.getPosition());
               framePose.getOrientation().set(generatedFramePose.getOrientation());
            }

            // take the next sample from the trajectory next time
            bodyPartTrajectorySampleCounter.replace(bodyPart, sampleCounter + 1);
         }
         else
         { // motion is over
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
               FramePose3D generatedFramePose = generatedFramePoseTrajectory.get(sampleCounter - 1);
               if (objectFrame != null)
                  generatedFramePose.changeFrame(ReferenceFrame.getWorldFrame());
               framePose.getPosition().set(generatedFramePose.getPosition());
               framePose.getOrientation().set(generatedFramePose.getOrientation());
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
      objectFrame = null;
      bodyPartObservedTrajectoryMap.clear();
      bodyPartGeneratedTrajectoryMap.clear();
      bodyPartTrajectorySampleCounter.clear();
      observationRecognitionPart.clear();
      doneInitialProcessingTask = false;
      isLastViaPoint.set(false);
      isMoving = false;
   }

   public int getTestNumber()
   {
      return testNumber;
   }

   public void setIsMovingThreshold(double distance)
   {
      isMovingThreshold = distance;
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
