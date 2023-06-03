package us.ihmc.behaviors.sharedControl;

import us.ihmc.behaviors.tools.TrajectoryRecordReplay;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;

public class AffordanceAssistant
{
   private final WorkspaceResourceDirectory configurationsDirectory = new WorkspaceResourceDirectory(getClass(), "/us/ihmc/behaviors/sharedControl/affordances");
   private TrajectoryRecordReplay affordancePlayer;
   private boolean isActive = false;
   private final HashMap<String, Pose3DReadOnly> bodyPartInitialPoseMap = new HashMap<>();
   private static final Tuple3DReadOnly AFFORDANCE_TO_HAND_COM_TRANSFORM = new Point3D(0.0, 0.0, 0.06441);

   public void loadAffordance(String fileName, ReferenceFrame objectFrame)
   {
      Path filePath = Paths.get(configurationsDirectory.getFilesystemDirectory().toString(), fileName + ".csv");
      LogTools.info(filePath.toString());
      affordancePlayer = new TrajectoryRecordReplay(filePath.toString(), 1);

      double[] initialPose = affordancePlayer.play();
      LogTools.info(" {}, {}, {}", initialPose[4], initialPose[5], initialPose[6]);
      FramePose3D initialBodyPartPose = new FramePose3D(objectFrame);
      initialBodyPartPose.getOrientation().set(initialPose);
      initialBodyPartPose.getPosition().set(4, initialPose);
      initialBodyPartPose.appendTranslation(AFFORDANCE_TO_HAND_COM_TRANSFORM);
      initialBodyPartPose.changeFrame(ReferenceFrame.getWorldFrame());
      bodyPartInitialPoseMap.put("rightHand", initialBodyPartPose);

//      for (RobotSide side : RobotSide.values)
//      {
//         RigidBodyTransform initialBodyPartTransform = new RigidBodyTransform();
//
//         initialBodyPartTransform.appendRollRotation(side.negateIfLeftSide(Math.PI / 2.0));
//         initialBodyPartTransform.appendYawRotation(-side.negateIfLeftSide(Math.PI / 2.0));
//         initialBodyPartTransform.appendTranslation(AFFORDANCE_TO_HAND_COM_TRANSFORM);
//         FramePose3D initialBodyPartPose = new FramePose3D(objectFrame, initialBodyPartTransform);
//         initialBodyPartPose.changeFrame(ReferenceFrame.getWorldFrame());
//         bodyPartInitialPoseMap.put(side.getCamelCaseName() + "Hand", initialBodyPartPose);
//      }

      isActive = true;
   }

   public void framePoseToPack(FramePose3D framePose, String bodyPart, boolean playForward)
   {
      if (bodyPart.equals("rightHand"))
      {
         if (playForward)
         {
            // Read file with stored trajectories: read set point per timestep until file is over
            double[] dataPoint = affordancePlayer.play(false); //play split data (a body part per time)
            // [0,1,2,3] quaternion of body segment; [4,5,6] position of body segment
            framePose.getOrientation().set(dataPoint);
            framePose.getPosition().set(4, dataPoint);
            framePose.changeFrame(ReferenceFrame.getWorldFrame());
         }
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
   }

   public HashMap<String,Pose3DReadOnly> getInitialHandPoseMap()
   {
      return bodyPartInitialPoseMap;
   }

   public boolean isAffordanceOver()
   {
      return affordancePlayer.hasDoneReplay();
   }

   public boolean containsBodyPart(String bodyPart)
   {
      return bodyPart.equals("rightHand");
   }
}
