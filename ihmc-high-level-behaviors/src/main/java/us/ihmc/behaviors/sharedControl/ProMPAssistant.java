package us.ihmc.behaviors.sharedControl;

import com.fasterxml.jackson.databind.JsonNode;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
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
   private static final int INTERPOLATION_SAMPLES = 10; // TODO make this a fraction of the estimated timesteps
   public static final int AFFORDANCE_BLENDING_SAMPLES = 25; // TODO make this a fraction of the estimated timesteps (1/3 or 1/4 could be good values)
   private final HashMap<String, ProMPManager> proMPManagers = new HashMap<>(); // proMPManagers stores a proMPManager for each task
   private final HashMap<String, List<String>> contextTasksMap = new HashMap<>(); // map to store all the tasks available for each context (object)
   private final List<Double> distanceCandidateTasks = new ArrayList<>();
   private boolean hasObservedBothHand = false;
   private String currentTask = ""; // detected task
   private int numberObservations = 0; // number of observations used to update the prediction
   private final HashMap<String, Point3D[]> priorStdDeviations = new HashMap<>(); // std deviations of position used for visualization
   private final HashMap<String, Point3D[]> priorMeans = new HashMap<>(); // mean trajectories of position used for visualization
   private String bodyPartInference = "";
   private String bodyPartGoal = "";
   private final HashMap<String, String> taskBodyPartInferenceMap = new HashMap<>();
   private final HashMap<String, String> taskBodyPartGoalMap = new HashMap<>();
   private FramePose3D taskGoalPose;
   private final HashMap<String, List<FramePose3D>> bodyPartObservedTrajectoryMap = new HashMap<>();
   private final HashMap<String, List<FramePose3D>> bodyPartGeneratedTrajectoryMap = new HashMap<>();
   private final HashMap<String, Integer> bodyPartTrajectorySampleCounter = new HashMap<>(); // to track the last used sample of a generated trajectory
   private boolean doneInitialProcessingTask = false;
   private boolean doneCurrentTask = false; // used to communicate to higher level classes that the task has been completed
   private boolean inEndZone = false; // used for blending with affordance
   private final AtomicBoolean isLastViaPoint = new AtomicBoolean(false); // check if last observed viapoint before update
   private int conditioningStep = 1;
   private int testNumber = 0;
   private int numberOfInferredSpeeds = 0;
   private boolean conditionOnlyLastObservation = true;
   private final ArrayList<Pose3DReadOnly> observationRecognition = new ArrayList<>();
   private boolean isMoving = false;
   private ReferenceFrame objectFrame;
   private double isMovingThreshold = -1;
   private final HashMap<String, FramePose3D> previousObservedPose = new HashMap<>();
   private boolean useCustomSpeed;
   private int executionAdjuster;
   private int waitForEnd = 0;

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
         conditioningStep = jsonNode.get("conditioningStep").asInt();
         int numberBasisFunctions = jsonNode.get("numberBasisFunctions").asInt();
         useCustomSpeed = jsonNode.get("useCustomSpeed").asBoolean();
         long speedFactor = jsonNode.get("allowedIncreaseDecreaseSpeedFactor").asLong();
         executionAdjuster = jsonNode.get("postProcessingVelocityAdjuster").asInt();
         numberOfInferredSpeeds = jsonNode.get("numberOfInferredSpeeds").asInt();
         // getting tasks
         JsonNode tasksArrayNode = jsonNode.get("tasks");
         int numberOfTasks = tasksArrayNode.size();
         String[] taskNames = new String[numberOfTasks];
         String[] bodyPartsInference = new String[numberOfTasks];
         String[] bodyPartsGoal = new String[numberOfTasks];
         HashMap<String, String>[] bodyPartsGeometries = new HashMap[numberOfTasks];
         LogTools.info("Loading ProMPs for tasks:");
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
            int customSpeed = taskNode.get("customSpeed").asInt();
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
                                               conditioningStep,
                                               isLastViaPoint,
                                               numberBasisFunctions,
                                               speedFactor,
                                               numberOfInferredSpeeds,
                                               useCustomSpeed,
                                               customSpeed));
            // initialize maps
            taskBodyPartInferenceMap.put(taskNames[i], bodyPartsInference[i]);
            taskBodyPartGoalMap.put(taskNames[i], bodyPartsGoal[i]);
            LogTools.info("{}", taskNames[i]);
            for (String key : bodyPartsGeometries[i].keySet())
               LogTools.info("     {} {}", key, bodyPartsGeometries[i].get(key));
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
      framePoseToPack(framePose, bodyPart, true);
   }

   public void framePoseToPack(FramePose3D framePose, String bodyPart, boolean play)
   {
      if (containsBodyPart(bodyPart))
      { // if bodyPart is used in current task
         List<FramePose3D> generatedFramePoseTrajectory = bodyPartGeneratedTrajectoryMap.get(bodyPart);

         int sampleCounter = bodyPartTrajectorySampleCounter.get(bodyPart);
         if (sampleCounter >= generatedFramePoseTrajectory.size() - AFFORDANCE_BLENDING_SAMPLES)
            inEndZone = true;
         else
            inEndZone = false;
         // -- Observing
         if (sampleCounter < numberObservations)
         {
            FramePose3D observedFramePose = new FramePose3D(bodyPartObservedTrajectoryMap.get(bodyPart).get(sampleCounter));
            if (objectFrame != null)
               observedFramePose.changeFrame(ReferenceFrame.getWorldFrame());
            framePose.getPosition().set(observedFramePose.getPosition());
            framePose.getOrientation().set(observedFramePose.getOrientation());

            // update sample from the trajectory to take next time
            if (play)
               bodyPartTrajectorySampleCounter.replace(bodyPart, sampleCounter + 1);
         }
         // -- Get trajectory from ProMP
         else if (sampleCounter < generatedFramePoseTrajectory.size() - executionAdjuster)
         {
            // pack the frame using the trajectory generated from prediction
            FramePose3D generatedFramePose = generatedFramePoseTrajectory.get(sampleCounter + 1);
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
               FixedFrameQuaternionBasics arbitratedFrameOrientation = new FrameQuaternion(lastObservedFramePose.getOrientation());
               // Perform slerp for quaternion blending
               arbitratedFrameOrientation.interpolate(generatedFramePose.getOrientation(), alpha);
               FixedFramePoint3DBasics arbitratedFramePosition = new FramePoint3D(lastObservedFramePose.getPosition());
               arbitratedFramePosition.interpolate(generatedFramePose.getPosition(), alpha);
               framePose.getPosition().set(arbitratedFramePosition);
               framePose.getOrientation().set(arbitratedFrameOrientation);
            }

            // update sample from the trajectory to take next time
            if (play)
               bodyPartTrajectorySampleCounter.replace(bodyPart, sampleCounter + executionAdjuster);
         }
         // -- Motion is over
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
               FramePose3D generatedFramePose = generatedFramePoseTrajectory.get(generatedFramePoseTrajectory.size() - 1);
               if (objectFrame != null)
                  generatedFramePose.changeFrame(ReferenceFrame.getWorldFrame());
               framePose.getPosition().set(generatedFramePose.getPosition());
               framePose.getOrientation().set(generatedFramePose.getOrientation());
            }
            waitForEnd++;
            // exit assistance mode
            if (waitForEnd > 10) // wait just a bit so that you can visualize preview of end for enough time
            {
               doneCurrentTask = true;
               waitForEnd = 0;
            }
         }
      }
   }

   public void processFrameAndObjectInformation(Pose3DReadOnly observedPose, String bodyPart, String objectName, ReferenceFrame objectFrame)
   {
      if (taskDetected(observedPose, bodyPart, objectName, objectFrame))
      {
         if (containsBodyPart(bodyPart)) // if bodyPart is used in current task
         {
            FramePose3D lastObservedPose = new FramePose3D(observedPose);
            if (!bodyPartGoal.isEmpty()) // if there is an observable object and goal
            {
               // change observed pose in object frame
               this.objectFrame = objectFrame;
               lastObservedPose.changeFrame(objectFrame);
            }
            if (userIsMoving(lastObservedPose, bodyPart)) // check if user has started moving after activating the assistance (after pressing the button)
            {
               ensureOrientationContinuity(bodyPart, lastObservedPose);
               // store observed pose
               bodyPartObservedTrajectoryMap.get(bodyPart).add(new FramePose3D(lastObservedPose));

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

   private void ensureOrientationContinuity(String bodyPart, FramePose3D lastObservedPose)
   {
      QuaternionReadOnly quaternionToCheck = lastObservedPose.getOrientation();
      if (previousObservedPose.containsKey(bodyPart))
      {
         if (!bodyPartGoal.isEmpty())
            previousObservedPose.get(bodyPart).changeFrame(objectFrame);

         QuaternionReadOnly previousQuaternion = previousObservedPose.get(bodyPart).getOrientation();
         // Check that quaternion is not changing 2pi range. Even if q = -q, the observed motion has to be continuous
         lastObservedPose.getOrientation().interpolate(previousQuaternion, quaternionToCheck, 1.0);

         previousObservedPose.get(bodyPart).set(lastObservedPose);

         if (Math.abs(lastObservedPose.getOrientation().getX() - previousQuaternion.getX()) > 0.05
             || Math.abs(lastObservedPose.getOrientation().getY() - previousQuaternion.getY()) > 0.05
             || Math.abs(lastObservedPose.getOrientation().getZ() - previousQuaternion.getZ()) > 0.05
             || Math.abs(lastObservedPose.getOrientation().getS() - previousQuaternion.getS()) > 0.05)
         {
            LogTools.error("Quaternion discontinuity asymmetric wrt zero. Check recorded part was not disconnected nor occluded during recording.");
            lastObservedPose.getOrientation().set(previousQuaternion);
         }
      }
      else
      {
         double x = proMPManagers.get(currentTask).getMeanStartValueQX();
         double y = proMPManagers.get(currentTask).getMeanStartValueQY();
         double z = proMPManagers.get(currentTask).getMeanStartValueQZ();
         double s = proMPManagers.get(currentTask).getMeanStartValueQS();

         // Calculate the maximum absolute value
         double max = Math.max(Math.abs(x), Math.max(Math.abs(y), Math.max(Math.abs(z), Math.abs(s))));

         // Check if the maximum absolute value is flipped in sign wrt to learned promp
         if ((Math.abs(x) == max && Math.signum(x * quaternionToCheck.getX()) == -1) || (Math.abs(y) == max && Math.signum(y * quaternionToCheck.getY()) == -1)
             || (Math.abs(z) == max && Math.signum(z * quaternionToCheck.getZ()) == -1) || (Math.abs(s) == max && Math.signum(s * quaternionToCheck.getS()) == -1))
         {
            lastObservedPose.getOrientation().negate();
         }

         previousObservedPose.put(bodyPart, new FramePose3D(lastObservedPose));
      }
   }

   public void processFrameAndObjectInformation(Pose3DReadOnly observedPose, String bodyPart, String objectName, FramePose3D objectPose)
   {
      if (taskDetected(observedPose, bodyPart, objectName, null))
      {
         if (containsBodyPart(bodyPart)) // if bodyPart is used in current task
         {
            FramePose3D lastObservedPose = new FramePose3D(observedPose);
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
//               for (int i = 0; i < candidateTasks.size(); i++)
//               {
//                  // here compute the distance wrt to initial value of the hands for each candidate task
//                  if (!hasObservedBothHand && bodyPart.contains("Hand"))
//                  { // compute distance wrt to first hand
//                     distanceCandidateTasks.add(proMPManagers.get(candidateTasks.get(i)).computeInitialDistance(lastObservedPose, bodyPart));
//                     hasObservedBothHand = true;
//                  }
//                  else if (bodyPart.contains("Hand"))
//                  { // compute distance wrt to second hand
//                     distanceCandidateTasks.set(i,
//                                                distanceCandidateTasks.get(i) + proMPManagers.get(candidateTasks.get(i))
//                                                                                             .computeInitialDistance(lastObservedPose, bodyPart));
//                  }
//               }
//               if (hasObservedBothHand) // if observed both hands
//               { // select task with minimum distance observation - learned mean
//                  currentTask = candidateTasks.get(getMinIndex(distanceCandidateTasks));
//                  // get the body part used for recognition for this task
//                  bodyPartInference = taskBodyPartInferenceMap.get(currentTask);
//                  // get the body part that has to reach a goal for this task
//                  bodyPartGoal = taskBodyPartGoalMap.get(currentTask);
//
//                  // initialize bodyPartObservedFrameTrajectory that will contain for each body part a list of observed FramePoses
//                  (proMPManagers.get(currentTask).getBodyPartsGeometry()).keySet().forEach(part -> bodyPartObservedTrajectoryMap.put(part, new ArrayList<>()));
//               }
               if (objectName.equals("Target"))
               {
                  Vector3DBasics objectTranslationToWorld = objectFrame.getTransformToWorldFrame().getTranslation();
                  if (objectTranslationToWorld.getY() > 0 && objectTranslationToWorld.getY() < 0  &&
                      objectTranslationToWorld.getZ() > 0 && objectTranslationToWorld.getZ() < 0)
                     currentTask = "LeftPunch0";
                  else if (objectTranslationToWorld.getY() > 0 && objectTranslationToWorld.getY() < 0  &&
                           objectTranslationToWorld.getZ() > 0 && objectTranslationToWorld.getZ() < 0)
                     currentTask = "LeftPunch1";
                  else if (objectTranslationToWorld.getY() > 0 && objectTranslationToWorld.getY() < 0  &&
                           objectTranslationToWorld.getZ() > 0 && objectTranslationToWorld.getZ() < 0)
                     currentTask = "LeftPunch2";
                  else if (objectTranslationToWorld.getY() > 0 && objectTranslationToWorld.getY() < 0  &&
                           objectTranslationToWorld.getZ() > 0 && objectTranslationToWorld.getZ() < 0)
                     currentTask = "LeftPunch3";
                  else if (objectTranslationToWorld.getY() > 0 && objectTranslationToWorld.getY() < 0  &&
                           objectTranslationToWorld.getZ() > 0 && objectTranslationToWorld.getZ() < 0)
                     currentTask = "LeftPunch4";
                  else if (objectTranslationToWorld.getY() > 0 && objectTranslationToWorld.getY() < 0  &&
                           objectTranslationToWorld.getZ() > 0 && objectTranslationToWorld.getZ() < 0)
                     currentTask = "LeftPunch5";
                  else if (objectTranslationToWorld.getY() > 0 && objectTranslationToWorld.getY() < 0  &&
                           objectTranslationToWorld.getZ() > 0 && objectTranslationToWorld.getZ() < 0)
                     currentTask = "LeftPunch6";
                  else if (objectTranslationToWorld.getY() > 0 && objectTranslationToWorld.getY() < 0  &&
                           objectTranslationToWorld.getZ() > 0 && objectTranslationToWorld.getZ() < 0)
                     currentTask = "LeftPunch7";
                  else
                     currentTask = "LeftPunch8";
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
                  priorStdDeviations.put(entryPart.getKey(), proMPManagers.get(currentTask).generateStdDeviationTrajectory(entryPart.getKey()));
                  priorMeans.put(entryPart.getKey(), proMPManagers.get(currentTask).generateMeanTrajectory(entryPart.getKey(), objectFrame));
               }
               if (useCustomSpeed)
               {
                  proMPManagers.get(currentTask).setTaskCustomSpeed();
               }
            }
         }
         else // no tasks in this context
         {
            hasObservedBothHand = false;
            LogTools.info("Detected object ({}) does not have any associated learned model for assistance", objectName);
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
            for (int i = 0; i < observedTrajectory.size(); i += conditioningStep)
            {
               if (i >= observedTrajectory.size() - conditioningStep)
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
      waitForEnd = 0;
      inEndZone = false;
      taskGoalPose = null;
      objectFrame = null;
      bodyPartObservedTrajectoryMap.clear();
      bodyPartGeneratedTrajectoryMap.clear();
      bodyPartTrajectorySampleCounter.clear();
      priorStdDeviations.clear();
      priorMeans.clear();
      observationRecognition.clear();
      doneInitialProcessingTask = false;
      isLastViaPoint.set(false);
      isMoving = false;
      hasObservedBothHand = false;
      distanceCandidateTasks.clear();
   }

   public void setCustomSpeed(double speed)
   {
      proMPManagers.get(currentTask).setTaskCustomSpeed(speed);
   }

   public int getTestNumber()
   {
      return testNumber;
   }

   public void setIsMovingThreshold(double distance)
   {
      isMovingThreshold = distance;
   }

   public boolean containsBodyPart(String bodyPart)
   {
      return (proMPManagers.get(currentTask).getBodyPartsGeometry()).containsKey(bodyPart);
   }

   public boolean hasPosition(String bodyPart)
   {
      return !((proMPManagers.get(currentTask).getBodyPartsGeometry()).get(bodyPart).equals("Orientation"));
   }

   public boolean isCurrentTaskDone()
   {
      return this.doneCurrentTask;
   }

   public String getCurrentTask()
   {
      return currentTask;
   }

   public boolean inEndZone()
   {
      return inEndZone;
   }

   public int getNumberOfSamples()
   {
      if(bodyPartGeneratedTrajectoryMap.containsKey(bodyPartGoal))
         return bodyPartGeneratedTrajectoryMap.get(bodyPartGoal).size();
      else
         return -1;
   }

   public int getCurrentSample()
   {
      if(bodyPartTrajectorySampleCounter.containsKey(bodyPartGoal))
         return bodyPartTrajectorySampleCounter.get(bodyPartGoal);
      else
         return -1;
   }

   public ProMPManager getProMPManager(String task)
   {
      return proMPManagers.get(task);
   }

   public Point3D[] getPriorStdDeviation(String bodyPart)
   {
      return priorStdDeviations.get(bodyPart);
   }

   public Point3D[] getPriorMean(String bodyPart)
   {
      return priorMeans.get(bodyPart);
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