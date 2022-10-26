package us.ihmc.avatar.sharedControl;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

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
   private String relevantBodyPart = "";
   private final FramePose3D taskGoalPose = new FramePose3D(); //detected goal
   private final HashMap<String, List<Pose3DReadOnly>> bodyPartObservedFrameTrajectory = new HashMap<>();
   private final HashMap<String, List<FramePose3D>> bodyPartGeneratedFrameTrajectory = new HashMap<>();
   private final HashMap<String, Integer> bodyPartTrajectorySampleCounter = new HashMap<>();
   private boolean doneInitialProcessingTask = false;

   public ProMPAssistant()
   {
      HashMap<String, String> bodyPartsGeometry = new HashMap<>();
      String taskName = "";
      // TODO change 100 to number from config related to observation time and kinematics streaming period
      numberObservations = 100;
      //TODO read list of available tasks and body parts involved from config file
      taskName = "PushDoor";
      bodyPartsGeometry.put("leftHand", "Pose");
      bodyPartsGeometry.put("rightHand", "Pose");
      boolean logEnabled = false;
      proMPManagers.put(taskName, new ProMPManager(taskName, bodyPartsGeometry,logEnabled));
      for (ProMPManager proMPManager : proMPManagers.values())
         proMPManager.learnTaskFromDemos();
   }

   @Override
   public void processFrameInformation(Pose3DReadOnly observedPose, String bodyPart)
   {
      if (objectDetected())
      {
         //TODO change to retrieve from config file what is the relevant robot part for the goal of that task
         //TODO change to guess what side is the one being used
         relevantBodyPart = "rightHand"; // e.g., right hand is the robot part being used to reach the handle and open the door in the task "open door"
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
