package us.ihmc.avatar.sharedControl;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;

import java.util.HashMap;
import java.util.List;

public class ProMPAssistant implements TeleoperationAssistant
{
   private final HashMap<String, ProMPManager> proMPManagers = new HashMap<>(); //each Manager stores a multi-D proMP for different bodyParts
   private String currentTask = "";
   private boolean doneInitialProcessingTask = false;

   public ProMPAssistant()
   {
      HashMap<String, String> bodyPartsGeometry = new HashMap<>();
      String taskName = "";
      //TODO read list of available tasks and body parts involved from config file
      taskName = "PushDoor";
      bodyPartsGeometry.put("leftHand", "Pose");
      bodyPartsGeometry.put("rightHand", "Pose");
      proMPManagers.put(taskName, new ProMPManager(taskName, bodyPartsGeometry));

      for (ProMPManager proMPManager : proMPManagers.values())
         proMPManager.learnTaskFromDemos();
   }

   @Override
   public void processFrameInformation(Pose3DReadOnly bodyPartObservedPose, String bodyPart)
   {
      if (objectDetected())
      {
         //TODO change to retrieve from config file what is the relevant robot part for the goal of that task
         String relevantBodyPart = "rightHand";
         if(objectPoseEstimated())
         {

         }
      }
      if (readMore than > 0){

         proMPManagers.get(currentTask).updateTaskSpeed(List < Pose3DReadOnly > observedFrameTrajectory, objectPose, relevantBodyPart);
      }
      //update proMP based on initial observations
      proMPManagers.get(currentTask).updateTaskTrajectories(HashMap < String, Pose3DReadOnly > bodyPartObservedPose, int conditioningTimestep);
      //update goal of proMP trajectory relevant for goal of the task, based on observed goal
      proMPManagers.get(currentTask).updateTaskTrajectoryGoal("rightHand", objectPose);

      doneInitialProcessingTask = true;
   }

   private boolean objectDetected()
   {
      //TODO 1. recognize task with the help of object detection (or Aruco Markers to begin with)
      currentTask = "PushDoor";
      return !(currentTask.isEmpty());
   }

   private boolean objectPoseEstimated()
   {
      //TODO 2. identify object pose (with Aruco Markers to begin with)
      // it can also be multiple goal poses (e.g., two grasping points for bi-manipulation)
      FramePose3D objectPose = new FramePose3D();
      return ;
   }

   @Override
   public boolean readyToPack()
   {
      return doneInitialProcessingTask;
   }

   @Override
   public void framePoseToPack(FramePose3D framePose, String robotPart)
   {

      List<FramePose3D> framePoseTrajectory = proMPManagers.get(currentTask).generateTaskTrajectory(robotPart);
   }
}
