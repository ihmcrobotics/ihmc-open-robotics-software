package us.ihmc.avatar.sharedControl;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.*;

import static java.lang.Math.exp;

/**
 * Class to pack a teleoperated referenceFrame and modify it by using some assistance from the robot.
 * The assistance comes from pre-trained probabilistic models: the ProMPs,
 * which represent a probabilistic prediction of multi-dimensional trajectories.
 * Initially the input from the user is not modified and is simply observed to produce a fitting prediction.
 * Once the task is detected (by recognizing the protagonist object of that task and observing the current motion of the user),
 * the teleoperated referenceFrame gradually shifts from the reference specified by the user to the predicted assistance of the ProMPs.
 */
public class ProMPAssistant implements TeleoperationAssistant
{
   private final HashMap<String, ProMPManager> proMPManagers = new HashMap<>(); //proMPManagers stores a proMPManager for each task
   private String currentTask = ""; //detected task
   private int numberObservations = 0; //number of observations used to update the prediction
   private String relevantBodyPart = ""; // e.g., right hand is the robot part being used to reach the handle and open the door in the task "open door"
   private HashMap<String,String> taskRelevantBodyPart = new HashMap<>();
   private final FramePose3D taskGoalPose = new FramePose3D(); //detected goal
   private final HashMap<String, List<Pose3DReadOnly>> bodyPartObservedFrameTrajectory = new HashMap<>();
   private final HashMap<String, List<FramePose3D>> bodyPartGeneratedFrameTrajectory = new HashMap<>();
   private final HashMap<String, Integer> bodyPartTrajectorySampleCounter = new HashMap<>();
   private boolean doneInitialProcessingTask = false;

   public ProMPAssistant()
   {
      List<String> taskNames = new ArrayList<>();
      List<String> relevantBodyParts = new ArrayList<>();
      List<HashMap<String, String>> bodyPartsGeometries = new ArrayList<>();
      boolean logEnabled = false;
      // read parameters regarding the properties of available learned tasks from json file
      try
      {
         JSONObject jsonObject = (JSONObject) new JSONParser().parse(new FileReader(Paths.get(System.getProperty("user.home"),
                                                                                              "repository-group/ihmc-open-robotics-software/ihmc-avatar-interfaces/src/main/resources/us/ihmc/avatar/sharedControl/ProMPAssistant.json")
                                                                                         .toString()));
         numberObservations = (int) ((long) jsonObject.get("numberObservations"));
         logEnabled = (boolean) jsonObject.get("logging");
         // getting tasks
         JSONArray tasksArray = (JSONArray) jsonObject.get("tasks");
         //iterating tasks
         Iterator taskIterator = tasksArray.iterator();
         while (taskIterator.hasNext())
         {
            Iterator<Map.Entry> taskPropertiesIterator = ((Map) taskIterator.next()).entrySet().iterator();
            while (taskPropertiesIterator.hasNext())
            {
               Map.Entry taskPropertyMap = taskPropertiesIterator.next();
               switch (taskPropertyMap.getKey().toString())
               {
                  case "name":
                     taskNames.add((String) taskPropertyMap.getValue());
                     break;
                  case "relevantBodyPart":
                     relevantBodyParts.add((String) taskPropertyMap.getValue());
                     break;
                  case "bodyParts":
                     JSONArray bodyPartsArray = (JSONArray) taskPropertyMap.getValue();
                     for (Object bodyPartObject : bodyPartsArray)
                     {
                        HashMap<String, String> bodyPartsGeometry = new HashMap<>();
                        JSONObject jsonBodyPartObject = (JSONObject) bodyPartObject;
                        jsonBodyPartObject.keySet().forEach(bodyPartProperty ->
                        {
                           String name = "";
                           String geometry = "";
                           if ("name".equals(bodyPartProperty.toString()))
                              name = String.valueOf(jsonBodyPartObject.get(bodyPartProperty));
                           else if ("geometry".equals(bodyPartProperty.toString()))
                              geometry = String.valueOf(jsonBodyPartObject.get(bodyPartProperty));
                           if (!(name.isEmpty()))
                              bodyPartsGeometry.put(name, geometry);
                        });
                        bodyPartsGeometries.add(bodyPartsGeometry);
                     }
                     break;
                  default:
                     break;
               }
            }
         }
      }
      catch (FileNotFoundException ex)
      {
         ex.printStackTrace();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      catch (ParseException e)
      {
         throw new RuntimeException(e);
      }

      for (int i=0; i<taskNames.size(); i++){
         proMPManagers.put(taskNames.get(i), new ProMPManager(taskNames.get(i), bodyPartsGeometries.get(i), logEnabled));
         taskRelevantBodyPart.put(taskNames.get(i),relevantBodyParts.get(i));
      }
      for (ProMPManager proMPManager : proMPManagers.values())
         proMPManager.learnTaskFromDemos();
   }

   @Override
   public void processFrameInformation(Pose3DReadOnly observedPose, String bodyPart)
   {
      if (objectDetected())
      {
         if (objectPoseEstimated())
         {
            //store observed pose
            bodyPartObservedFrameTrajectory.get(bodyPart).add(observedPose);
            if (bodyPartObservedFrameTrajectory.get(bodyPart).size() > numberObservations) //if observed a sufficient number of poses
            {
               updateTask();
               generateTaskTrajectories();
               doneInitialProcessingTask = true;
            }
         }
      }
   }

   private boolean objectDetected()
   {
      if (currentTask.isEmpty())
      {
         //TODO 1. recognize task with object detection algorithm (or Aruco Markers to begin with)
         currentTask = "PushDoor";
         relevantBodyPart = taskRelevantBodyPart.get(currentTask);
         //initialize bodyPartObservedFrameTrajectory that will contain for each body part a list of observed FramePoses
         for (String bodyPart : (proMPManagers.get(currentTask).getBodyPartsGeometry()).keySet())
            bodyPartObservedFrameTrajectory.put(bodyPart, new ArrayList<>());
         return !(currentTask.isEmpty());
      }
      else
         return true;
   }

   private boolean objectPoseEstimated()
   {
      //TODO 2. identify object pose (with Aruco Markers to begin with)
      // it can also be multiple goal poses (e.g., two grasping points for bi-manipulation)
      //taskGoalPose = ;
      return !(taskGoalPose.equals(new FramePose3D()));
   }

   private void updateTask()
   {
      //update speed proMP based on relevant body part observed trajectory and goal
      proMPManagers.get(currentTask).updateTaskSpeed(bodyPartObservedFrameTrajectory.get(relevantBodyPart), taskGoalPose, relevantBodyPart);
      //update all proMP trajectories based on initial observations (stored observed poses)
      for (String robotPart : bodyPartObservedFrameTrajectory.keySet())
      {
         List<Pose3DReadOnly> robotPartObservedTrajectory = bodyPartObservedFrameTrajectory.get(robotPart);
         for (int i = 0; i < robotPartObservedTrajectory.size(); i++)
         {
            proMPManagers.get(currentTask).updateTaskTrajectory(robotPart, robotPartObservedTrajectory.get(i), i);
         }
      }
      //update only proMP trajectory of the body part relevant for goal of the task, based on observed goal
      proMPManagers.get(currentTask).updateTaskTrajectoryGoal(relevantBodyPart, taskGoalPose);
   }

   private void generateTaskTrajectories()
   {
      for (String bodyPart : bodyPartObservedFrameTrajectory.keySet())
      {
         bodyPartGeneratedFrameTrajectory.put(bodyPart, proMPManagers.get(currentTask).generateTaskTrajectory(bodyPart));
         bodyPartTrajectorySampleCounter.put(bodyPart, numberObservations);
      }
   }

   @Override
   public boolean readyToPack()
   {
      return doneInitialProcessingTask;
   }

   @Override
   public void framePoseToPack(FramePose3D framePose, String bodyPart)
   {
      List<FramePose3D> generatedFramePoseTrajectory = bodyPartGeneratedFrameTrajectory.get(bodyPart);
      //take a sample from the trajectory
      FramePose3D generatedFramePose = generatedFramePoseTrajectory.get(bodyPartTrajectorySampleCounter.get(bodyPart));
      //TODO compute distance from region close to the goal and use this to select the next sample.
      // If distance is increasing, go back to previous sample

      if (bodyPart.equals(relevantBodyPart))
      {
         //TODO compute distance from region close to the goal and use this to modulate alpha
         // compute initial distance when goal is detected
         // set alpha according to distance
      }
      // shared-control arbitration law
      int sampleCounter = bodyPartTrajectorySampleCounter.get(bodyPart);
      if (sampleCounter <= generatedFramePoseTrajectory.size())
      {
         double x = (sampleCounter - numberObservations) / (generatedFramePoseTrajectory.size() - numberObservations);
         //sigmoid with [X:0,Y:~0],[X:0.6,Y:~1],[X>1,Y:1]
         double alpha = 1.0 / (1 + 4 * exp(-18 * (x - 0.2)));
         //set orientation
         FixedFrameQuaternionBasics frameOrientation = framePose.getOrientation();
         FixedFrameQuaternionBasics generatedFrameOrientation = generatedFramePose.getOrientation();
         FixedFrameQuaternionBasics arbitratedFrameOrientation = framePose.getOrientation();
         arbitratedFrameOrientation.set((1 - alpha) * frameOrientation.getX() + alpha * generatedFrameOrientation.getX(),
                                        (1 - alpha) * frameOrientation.getY() + alpha * generatedFrameOrientation.getY(),
                                        (1 - alpha) * frameOrientation.getZ() + alpha * generatedFrameOrientation.getZ(),
                                        (1 - alpha) * frameOrientation.getS() + alpha * generatedFrameOrientation.getS());
         //set position
         FixedFramePoint3DBasics framePosition = framePose.getPosition();
         FixedFramePoint3DBasics generatedFramePosition = generatedFramePose.getPosition();
         FixedFramePoint3DBasics arbitratedFramePosition = framePose.getPosition();
         arbitratedFramePosition.setX((1 - alpha) * framePosition.getX() + alpha * generatedFramePosition.getX());
         arbitratedFramePosition.setY((1 - alpha) * framePosition.getY() + alpha * generatedFramePosition.getY());
         arbitratedFramePosition.setZ((1 - alpha) * framePosition.getZ() + alpha * generatedFramePosition.getZ());

         framePose.getPosition().set(arbitratedFramePosition);
         framePose.getOrientation().set(arbitratedFrameOrientation);

         //take the next sample from the trajectory next time
         bodyPartTrajectorySampleCounter.replace(bodyPart, bodyPartTrajectorySampleCounter.get(bodyPart) + 1);
      }
      else
      {
         reset();
      }
   }

   private void reset()
   {
      //reset manager of current task (reset reference of proMP object of current task to initial proMP before any conditioning)
      proMPManagers.get(currentTask).resetTask();
      currentTask = "";
      taskGoalPose.setToZero(taskGoalPose.getReferenceFrame().getWorldFrame());
      bodyPartObservedFrameTrajectory.clear();
      bodyPartGeneratedFrameTrajectory.clear();
      bodyPartTrajectorySampleCounter.clear();
      doneInitialProcessingTask = false;
   }
}
