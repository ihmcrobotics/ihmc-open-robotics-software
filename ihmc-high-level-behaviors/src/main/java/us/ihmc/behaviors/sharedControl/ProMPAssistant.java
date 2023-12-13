package us.ihmc.behaviors.sharedControl;

import com.fasterxml.jackson.databind.JsonNode;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;

import java.io.IOException;
import java.io.InputStream;
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
   private static final int INTERPOLATION_SAMPLES = 10;
   private final HashMap<String, ProMPManager> proMPManagers = new HashMap<>(); // proMPManagers stores a proMPManager for each task
   private final HashMap<String, List<String>> contextTasksMap = new HashMap<>(); // map to store all the tasks available for each context (object)
   private final List<Double> distanceCandidateTasks = new ArrayList<>();
   private boolean firstObservedBodyPart = true;
   private String currentTask = ""; // detected task
   private int numberObservations = 0; // number of observations used to update the prediction
   private final HashMap<String, Point3D[]> initialStdDeviations = new HashMap<>(); // std deviations of position used for visualization
   private final HashMap<String, Point3D[]> initialMeans = new HashMap<>(); // mean trajectories of position used for visualization
   private String bodyPartInference = "";
   private String bodyPartGoal = "";
   private final HashMap<String, String> taskBodyPartInferenceMap = new HashMap<>();
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
   private int numberOfInferredSpeeds = 0;
   private boolean conditionOnlyLastObservation = true;
   private final ArrayList<Pose3DReadOnly> observationRecognition = new ArrayList<>();
   private boolean isMoving = false;
   private ReferenceFrame objectFrame;
   private double isMovingThreshold = -1;

   public ProMPAssistant()
   {
      InputStream inputStream = getClass().getClassLoader().getResourceAsStream("us/ihmc/behaviors/sharedControl/ProMPAssistant.json");

      // If the inputStream is null it's likely because the file doesn't exist or got moved. Check file path
      if (inputStream == null)
      {
         LogTools.info("File path is null");
         return;
      }

      JSONFileTools.load(inputStream, jsonNode ->
      {
         testNumber = jsonNode.get("testNumberUseOnlyForTesting").asInt();
         boolean logEnabled = jsonNode.get("logging").asBoolean();
         numberObservations = jsonNode.get("numberObservations").asInt();
         conditionOnlyLastObservation = jsonNode.get("conditionOnlyLastObservation").asBoolean();
         int numberBasisFunctions = jsonNode.get("numberBasisFunctions").asInt();
         long speedFactor = jsonNode.get("allowedIncreaseDecreaseSpeedFactor").asLong();
         numberOfInferredSpeeds = jsonNode.get("numberOfInferredSpeeds").asInt();
         // getting tasks
         JsonNode tasksArrayNode = jsonNode.get("tasks");
         int numberOfTasks = tasksArrayNode.size();
         String[] taskNames = new String[numberOfTasks];
         String[] bodyPartsInference = new String[numberOfTasks];
         String[] bodyPartsGoal = new String[numberOfTasks];
         HashMap<String, String>[] bodyPartsGeometries = new HashMap[numberOfTasks];
         Point3D[] goalToEETranslations = new Point3D[numberOfTasks];
         Quaternion[] goalToEERotations = new Quaternion[numberOfTasks];
         for (int i = 0; i < numberOfTasks; i++)
         {
            JsonNode taskNode = tasksArrayNode.get(i);
            String context = taskNode.get("context").asText();
            if (!contextTasksMap.containsKey(context))
               contextTasksMap.put(context, new ArrayList<>());
            taskNames[i] = taskNode.get("name").asText();
            // in contextTaskMap add task to last parsed context
            contextTasksMap.get(context).add(taskNames[i]);

            bodyPartsInference[i] = taskNode.get("bodyPartForInference").asText();
            bodyPartsGoal[i] = taskNode.get("bodyPartWithObservableGoal").asText();
            JsonNode translationArrayNode = taskNode.get("translationGoalToEE");
            goalToEETranslations[i] = new Point3D(translationArrayNode.get(0).asDouble(),
                                                  translationArrayNode.get(1).asDouble(),
                                                  translationArrayNode.get(2).asDouble());
            JsonNode rotationArrayNode = taskNode.get("rotationGoalToEE");
            goalToEERotations[i] = new Quaternion(rotationArrayNode.get(0).asDouble(),
                                                  rotationArrayNode.get(1).asDouble(),
                                                  rotationArrayNode.get(2).asDouble(),
                                                  rotationArrayNode.get(3).asDouble());
            JsonNode bodyPartsArrayNode = taskNode.get("bodyParts");
            HashMap<String, String> bodyPartsGeometry = new HashMap<>();
            for (JsonNode bodyPartObject : bodyPartsArrayNode)
            {
               bodyPartsGeometry.put(bodyPartObject.get("name").asText(), bodyPartObject.get("geometry").asText());
            }
            bodyPartsGeometries[i] = bodyPartsGeometry;
            // initialize the proMP managers
            proMPManagers.put(taskNames[i],
                              new ProMPManager(taskNames[i],
                                               bodyPartsGeometries[i],
                                               logEnabled,
                                               isLastViaPoint,
                                               numberBasisFunctions,
                                               speedFactor,
                                               numberOfInferredSpeeds));
            // initialize maps
            taskBodyPartInferenceMap.put(taskNames[i], bodyPartsInference[i]);
            taskBodyPartGoalMap.put(taskNames[i], bodyPartsGoal[i]);
            taskTransformGoalMap.put(taskNames[i], new RigidBodyTransform(goalToEERotations[i], goalToEETranslations[i]));
            LogTools.info("Loading ProMPs for tasks:");
            LogTools.info("{}", taskNames[i]);
            for (HashMap<String, String> partsGeometry : bodyPartsGeometries)
               for (String key : partsGeometry.keySet())
                  LogTools.info("     {} {}", key, partsGeometry.get(key));
         }

         for (ProMPManager prompManager : proMPManagers.values())
            prompManager.loadTaskFromDemos();

         LogTools.info("ProMPs are ready to be used!");
      });

      try
      {
         inputStream.close();
      }
      catch (IOException e)
      {
         LogTools.info(e);
      }
   }

   public void framePoseToPack(FramePose3D framePose, String bodyPart)
   {
      if ((proMPManagers.get(currentTask).getBodyPartsGeometry()).containsKey(bodyPart))
      { // if bodyPart is used in current task
         List<FramePose3D> generatedFramePoseTrajectory = bodyPartGeneratedTrajectoryMap.get(bodyPart);

         int sampleCounter = bodyPartTrajectorySampleCounter.get(bodyPart);
         if (sampleCounter < numberObservations)
         {
            FramePose3D observedFramePose = bodyPartObservedTrajectoryMap.get(bodyPart).get(sampleCounter);
            if (objectFrame != null)
               observedFramePose.changeFrame(ReferenceFrame.getWorldFrame());
            framePose.getPosition().set(observedFramePose.getPosition());
            framePose.getOrientation().set(observedFramePose.getOrientation());
            // take the next sample from the trajectory next time
            bodyPartTrajectorySampleCounter.replace(bodyPart, sampleCounter + 1);
         }
         else if (sampleCounter < generatedFramePoseTrajectory.size() - 1)
         {
            // pack the frame using the trajectory generated from prediction
            FramePose3D generatedFramePose = generatedFramePoseTrajectory.get(
                  sampleCounter + 1); // +1 to avoid discontinuity due to imprecision in conditioning
            List<FramePose3D> observations = bodyPartObservedTrajectoryMap.get(bodyPart);
            FramePose3D lastObservedFramePose = observations.get(observations.size() - 1);
            if (objectFrame != null) // change back to world frame if before it was changed to object frame
            {
               generatedFramePose.changeFrame(ReferenceFrame.getWorldFrame());
               lastObservedFramePose.changeFrame(ReferenceFrame.getWorldFrame());
            }

            double alpha = (double) (sampleCounter - numberObservations) / (INTERPOLATION_SAMPLES);
            if (alpha > 1.0)
            {
               framePose.getPosition().set(generatedFramePose.getPosition());
               framePose.getOrientation().set(generatedFramePose.getOrientation());
            }
            else
            { // gradually interpolate last observation to prediction
               FixedFrameQuaternionBasics arbitratedFrameOrientation = framePose.getOrientation();
               arbitratedFrameOrientation.set((1 - alpha) * lastObservedFramePose.getOrientation().getX() + alpha * generatedFramePose.getOrientation().getX(),
                                              (1 - alpha) * lastObservedFramePose.getOrientation().getY() + alpha * generatedFramePose.getOrientation().getY(),
                                              (1 - alpha) * lastObservedFramePose.getOrientation().getZ() + alpha * generatedFramePose.getOrientation().getZ(),
                                              (1 - alpha) * lastObservedFramePose.getOrientation().getS() + alpha * generatedFramePose.getOrientation().getS());
               FixedFramePoint3DBasics arbitratedFramePosition = framePose.getPosition();
               arbitratedFramePosition.setX((1 - alpha) * lastObservedFramePose.getPosition().getX() + alpha * generatedFramePose.getPosition().getX());
               arbitratedFramePosition.setY((1 - alpha) * lastObservedFramePose.getPosition().getY() + alpha * generatedFramePose.getPosition().getY());
               arbitratedFramePosition.setZ((1 - alpha) * lastObservedFramePose.getPosition().getZ() + alpha * generatedFramePose.getPosition().getZ());
               framePose.getPosition().set(arbitratedFramePosition);
               framePose.getOrientation().set(arbitratedFrameOrientation);
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
               FramePose3D generatedFramePose = generatedFramePoseTrajectory.get(generatedFramePoseTrajectory.size() - 1);
               if (objectFrame != null)
                  generatedFramePose.changeFrame(ReferenceFrame.getWorldFrame());
               framePose.getPosition().set(generatedFramePose.getPosition());
               framePose.getOrientation().set(generatedFramePose.getOrientation());
            }
            LogTools.info("Assistance completed");
            // exit assistance mode
            doneCurrentTask = true;
         }
      }
   }

   public void processFrameAndObjectInformation(Pose3DReadOnly observedPose, String bodyPart, String objectName, ReferenceFrame objectFrame)
   {
      if (taskDetected(observedPose, bodyPart, objectName, objectFrame))
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

            if (!bodyPartGoal.isEmpty()) // if there is an observable object and goal
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
                  LogTools.info("Generated prediction");
               }
            }
         }
      }
   }

   public void processFrameAndObjectInformation(Pose3DReadOnly observedPose, String bodyPart, String objectName, FramePose3D objectPose)
   {
      if (taskDetected(observedPose, bodyPart, objectName, null))
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
               if (bodyPartObservedTrajectoryMap.get(bodyPart).size() > numberObservations + 1) // if observed a sufficient number of poses
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
                  LogTools.info("Generated prediction");
               }
            }
         }
      }
   }

   // TODO what if someone is lefthanded, or simply wants to use the left hand for that task?
   //  Learn task for both hands and call them ...L and ...R, check initial pose of hands to determine which one is being used
   private boolean taskDetected(Pose3DReadOnly observedPose, String bodyPart, String objectName, ReferenceFrame objectFrame)
   {
      if (currentTask.isEmpty())
      {
         if (contextTasksMap.containsKey(objectName))
         {
            List<String> candidateTasks = contextTasksMap.get(objectName);
            FramePose3D lastObservedPose = new FramePose3D(observedPose);
            if (objectFrame != null)
               lastObservedPose.changeFrame(objectFrame);
            if (candidateTasks.size() > 1) // more than 1 task in this context
            {
               for (int i = 0; i < candidateTasks.size(); i++)
               {
                  // here compute the distance wrt to initial value for each cadidate task
                  if (firstObservedBodyPart)
                  { // compute distance wrt to first body part
                     distanceCandidateTasks.add(proMPManagers.get(candidateTasks.get(i)).computeInitialDistance(lastObservedPose, bodyPart));
                     firstObservedBodyPart = false;
                  }
                  else
                  { // compute distance wrt to second body part
                     distanceCandidateTasks.set(i,
                                                distanceCandidateTasks.get(i) + proMPManagers.get(candidateTasks.get(i))
                                                                                             .computeInitialDistance(lastObservedPose, bodyPart));
                  }
               }
               if (!firstObservedBodyPart)
               { // select task with minimum distance observation - learned mean
                  currentTask = candidateTasks.get(getMinIndex(distanceCandidateTasks));
                  // get the body part used for recognition for this task
                  bodyPartInference = taskBodyPartInferenceMap.get(currentTask);
                  // get the body part that has to reach a goal for this task
                  bodyPartGoal = taskBodyPartGoalMap.get(currentTask);

                  // initialize bodyPartObservedFrameTrajectory that will contain for each body part a list of observed FramePoses
                  (proMPManagers.get(currentTask).getBodyPartsGeometry()).keySet().forEach(part -> bodyPartObservedTrajectoryMap.put(part, new ArrayList<>()));
               }
            }
            else // 1 task in this context
            {
               currentTask = candidateTasks.get(0);
               // get the body part used for recognition for this task
               bodyPartInference = taskBodyPartInferenceMap.get(currentTask);
               // get the body part that has to reach a goal for this task
               bodyPartGoal = taskBodyPartGoalMap.get(currentTask);

               // initialize bodyPartObservedFrameTrajectory that will contain for each body part a list of observed FramePoses
               (proMPManagers.get(currentTask).getBodyPartsGeometry()).keySet().forEach(part -> bodyPartObservedTrajectoryMap.put(part, new ArrayList<>()));
            }
            if (!currentTask.isEmpty())
            {
               LogTools.info("Found task! {}", currentTask);
               for (Map.Entry<String, String> entryPart : proMPManagers.get(currentTask).getBodyPartsGeometry().entrySet())
               {
                  // store stdDeviation trajectories for informing user through graphics
                  initialStdDeviations.put(entryPart.getKey(),
                                           proMPManagers.get(currentTask).generateStdDeviationTrajectory(entryPart.getKey()));
                  initialMeans.put(entryPart.getKey(),
                                   proMPManagers.get(currentTask).generateMeanTrajectory(entryPart.getKey(), objectFrame));
               }
            }
         }
         else // no tasks in this context
         {
            firstObservedBodyPart = true;
            LogTools.info("Detected object ({}) does not have any associated learned policy for assistance", objectName);
            return false;
         }
      }
      return !currentTask.isEmpty();
   }

   public static int getMinIndex(List<Double> list)
   {
      if (list == null || list.isEmpty())
      {
         throw new IllegalArgumentException("List cannot be null or empty.");
      }

      double min = list.get(0);
      int minIndex = 0;

      for (int i = 1; i < list.size(); i++)
      {
         double current = list.get(i);
         if (current < min)
         {
            min = current;
            minIndex = i;
         }
      }

      return minIndex;
   }

   private boolean userIsMoving(Pose3DReadOnly lastObservedPose, String bodyPart)
   {
      if (bodyPart.equals(bodyPartInference) && !isMoving)
      {
         observationRecognition.add(lastObservedPose);
         if (observationRecognition.size() > 1)
         {
            double distance = (observationRecognition.get(observationRecognition.size() - 1)).getTranslation()
                                                                                             .distance(observationRecognition.get(0).getTranslation());
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
      if (numberOfInferredSpeeds > 0)
         proMPManagers.get(currentTask).updateTaskSpeed(bodyPartObservedTrajectoryMap.get(bodyPartInference), bodyPartInference);
      // update all proMP trajectories based on initial observations (stored observed poses)
      for (Map.Entry<String, List<FramePose3D>> entryPartObservation : bodyPartObservedTrajectoryMap.entrySet())
      {
         List<FramePose3D> observedTrajectory = entryPartObservation.getValue();
         if (conditionOnlyLastObservation)
         {
            if (observedTrajectory.size() > 0)
            {
               isLastViaPoint.set(true);
               proMPManagers.get(currentTask)
                            .updateTaskTrajectory(entryPartObservation.getKey(),
                                                  observedTrajectory.get(observedTrajectory.size() - 1),
                                                  observedTrajectory.size() - 1);
            }
         }
         else
         {
            for (int i = 0; i < observedTrajectory.size(); i++)
            {
               if (i == observedTrajectory.size() - 1)
                  isLastViaPoint.set(true);
               proMPManagers.get(currentTask).updateTaskTrajectory(entryPartObservation.getKey(), observedTrajectory.get(i), i);
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
      ReferenceFrame frame = objectFrame != null ? objectFrame : ReferenceFrame.getWorldFrame();
      // for each body part generate the mean trajectory of the learned promp
      for (String bodyPart : bodyPartObservedTrajectoryMap.keySet())
      {
         bodyPartGeneratedTrajectoryMap.put(bodyPart, proMPManagers.get(currentTask).generateTaskTrajectory(bodyPart, frame));
         // start using it after the last sample we observed, not from the beginning. We do not want to restart the motion
         setStartTrajectories(numberObservations + 1);
      }
   }

   public void setStartTrajectories(int sample)
   {
      doneCurrentTask = false;
      for (String bodyPart : bodyPartObservedTrajectoryMap.keySet())
         bodyPartTrajectorySampleCounter.put(bodyPart, sample);
   }

   public boolean readyToPack()
   {
      return doneInitialProcessingTask;
   }

   public void reset()
   {
      // reset manager of current task (reset reference of proMP object of current task to initial proMP before any conditioning)
      if (!currentTask.isEmpty())
      {
         proMPManagers.get(currentTask).resetTask();
         currentTask = "";
      }
      doneCurrentTask = false;
      taskGoalPose = null;
      objectFrame = null;
      bodyPartObservedTrajectoryMap.clear();
      bodyPartGeneratedTrajectoryMap.clear();
      bodyPartTrajectorySampleCounter.clear();
      initialStdDeviations.clear();
      initialMeans.clear();
      observationRecognition.clear();
      doneInitialProcessingTask = false;
      isLastViaPoint.set(false);
      isMoving = false;
      firstObservedBodyPart = true;
      distanceCandidateTasks.clear();
   }

   public int getTestNumber()
   {
      return testNumber;
   }

   public void setIsMovingThreshold(double distance)
   {
      isMovingThreshold = distance;
   }

   public boolean isCurrentTaskDone()
   {
      return this.doneCurrentTask;
   }

   public ProMPManager getProMPManager(String task)
   {
      return proMPManagers.get(task);
   }

   public Point3D[] getInitialStdDeviation(String bodyPart)
   {
      return initialStdDeviations.get(bodyPart);
   }

   public Point3D[] getInitialMean(String bodyPart)
   {
      return initialMeans.get(bodyPart);
   }

   public Set<String> getTaskNames()
   {
      return proMPManagers.keySet();
   }

   public boolean startedProcessing()
   {
      return !currentTask.isEmpty();
   }
}
