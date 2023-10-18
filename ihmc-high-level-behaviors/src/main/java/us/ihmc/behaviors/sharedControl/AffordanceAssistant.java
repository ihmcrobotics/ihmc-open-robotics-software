package us.ihmc.behaviors.sharedControl;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.behaviors.tools.TrajectoryRecordReplay;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;

public class AffordanceAssistant
{
   private final SideDependentList<RigidBodyTransform> affordanceToHandControlFrameTransform;
   private final WorkspaceResourceDirectory configurationsDirectory = new WorkspaceResourceDirectory(getClass(),
                                                                                                     "/us/ihmc/behaviors/sharedControl/affordances");
   private TrajectoryRecordReplay affordancePlayer;
   private boolean isActive = false;
   private final HashMap<String, Pose3DReadOnly> bodyPartInitialPoseMap = new HashMap<>();
   private final HashMap<String, Pose3DReadOnly> bodyPartPreviousFrameMap = new HashMap<>();
   private boolean isHandConfigurationCommand = false;
   private Pair<RobotSide, HandConfiguration> handConfigurationToSend;
   private ReferenceFrame objectFrame;
   private boolean affordanceStarted = false;

   public AffordanceAssistant(SideDependentList<RigidBodyTransform> affordanceToHandControlFrameTransform)
   {
      this.affordanceToHandControlFrameTransform = affordanceToHandControlFrameTransform;
   }

   public void loadAffordance(String fileName, ReferenceFrame objectFrame)
   {
      //TODO generalize for multiple body parts
      this.objectFrame = objectFrame;
      Path filePath = Paths.get(configurationsDirectory.getFilesystemDirectory().toString(), fileName + ".csv");
      affordancePlayer = new TrajectoryRecordReplay(filePath.toString(), 1); // data of each part has been already concatenated into single rows

      double[] initialData = affordancePlayer.play();
      RigidBodyTransform initialTransform = new RigidBodyTransform(initialData);
      FramePose3D initialBodyPartPose = new FramePose3D(objectFrame, initialTransform);
      initialBodyPartPose.appendTranslation(affordanceToHandControlFrameTransform.get(RobotSide.RIGHT).getTranslation());
      initialBodyPartPose.changeFrame(ReferenceFrame.getWorldFrame());
      bodyPartPreviousFrameMap.put("rightHand", new FramePose3D(initialBodyPartPose));
      // pre-apply rotation of VR controllers to cancel out the rotation in the VRKinematicsStreaming
      initialBodyPartPose.appendPitchRotation(Math.PI / 2.0);
      initialBodyPartPose.appendRollRotation(Math.PI / 2.0);
      bodyPartInitialPoseMap.put("rightHand", initialBodyPartPose);

      isActive = true;
   }

   public void framePoseToPack(FramePose3D framePose, String bodyPart, boolean playForward)
   {
      //TODO generalize for multiple body parts
      if (bodyPart.equals("rightHand"))
      {
         if (playForward)
         {
            // Read file with stored trajectories: read set point per timestep until file is over
            double[] dataPoint = affordancePlayer.play(false); //play split data (a body part per time)
            isHandConfigurationCommand = true;
            for (int i = 1; i < dataPoint.length; i++)
               isHandConfigurationCommand &= dataPoint[i] == 0.0;

            if (!isHandConfigurationCommand)
            {
               affordanceStarted = true;
               RigidBodyTransform transform = new RigidBodyTransform(dataPoint);
               FramePose3D affordancePose = new FramePose3D(objectFrame, transform);
               affordancePose.appendTranslation(affordanceToHandControlFrameTransform.get(RobotSide.RIGHT).getTranslation());
               affordancePose.changeFrame(ReferenceFrame.getWorldFrame());
               framePose.set(affordancePose);

               bodyPartPreviousFrameMap.replace(bodyPart, new FramePose3D(framePose));
            }
            else
            {
               framePose.set(bodyPartPreviousFrameMap.get(bodyPart));
               handConfigurationToSend = Pair.of(RobotSide.getSideFromName(bodyPart), HandConfiguration.values[(int) dataPoint[0]]);
            }
         }
         else
         {
            framePose.set(bodyPartPreviousFrameMap.get(bodyPart));
            isHandConfigurationCommand = false;
         }
      }
   }

   public void checkForHandConfigurationUpdates(HandConfigurationListener handConfigurationListener)
   {
      if (isHandConfigurationCommand)
      {
         // send notification to listener
         if (handConfigurationListener != null)
            handConfigurationListener.onNotification();
         isHandConfigurationCommand = false;
      }
   }

   public boolean hasAffordance(String affordanceName)
   {
      File affordanceFolder = new File(configurationsDirectory.getFilesystemDirectory().toString());
      File[] files = affordanceFolder.listFiles((dir, name) -> name.toLowerCase().endsWith(".csv") && !new File(dir, name).isDirectory());
      for (File file : files)
      {
         if (file.getName().contains(affordanceName))
         {
            LogTools.info("Found affordance! {}", affordanceName);
            return true;
         }
      }
      return false;
   }

   public boolean isActive()
   {
      return isActive;
   }

   public void reset()
   {
      isActive = false;
      affordancePlayer = null;
      isHandConfigurationCommand = false;
      handConfigurationToSend = null;
      affordanceStarted = false;
   }

   public HashMap<String, Pose3DReadOnly> getInitialHandPoseMap()
   {
      return bodyPartInitialPoseMap;
   }

   public boolean hasAffordanceStarted()
   {
      return affordanceStarted;
   }

   public boolean isAffordanceOver()
   {
      return affordancePlayer.hasDoneReplay();
   }

   //TODO generalize for multiple body parts
   public boolean containsBodyPart(String bodyPart)
   {
      return bodyPart.equals("rightHand");
   }

   public Pair<RobotSide, HandConfiguration> getHandConfigurationToSend()
   {
      return handConfigurationToSend;
   }

   public int getNumberOfSamples()
   {
      if(isActive)
         return affordancePlayer.getData().size();
      else
         return -1;
   }

   public int getCurrentSample()
   {
      if(isActive)
         return affordancePlayer.getTimeStepReplay();
      else
         return -1;
   }
}