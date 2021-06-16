package us.ihmc.gdx.ui.graphics.live;

import javafx.scene.transform.Translate;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.ui.graphics.GDXRobotModelGraphic;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

public class GDXROS2RobotVisualizer extends GDXRobotModelGraphic
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final ExceptionHandlingThreadScheduler scheduler;

   private boolean trackRobot = false;
   private FocusBasedGDXCamera cameraForOptionalTracking;
   private Translate robotTranslate;
   private Translate savedCameraTranslate;
   private boolean hasPrepended = false;

   public GDXROS2RobotVisualizer(DRCRobotModel robotModel, ROS2NodeInterface ros2Node)
   {
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      loadRobotModelAndGraphics(robotModel.getRobotDescription(), fullRobotModel.getElevator());
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node, fullRobotModel);
      scheduler = new ExceptionHandlingThreadScheduler(getClass().getSimpleName(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, 1, true);
   }

   public void setTrackRobot(FocusBasedGDXCamera camera, boolean trackRobot)
   {
      this.trackRobot = trackRobot;
      cameraForOptionalTracking = camera;
      if (!hasPrepended)
      {
         hasPrepended = true;
         robotTranslate = new Translate();
//         cameraForOptionalTracking.prependTransform(robotTranslate);
      }
      else if (trackRobot)
      {
//         cameraForOptionalTracking.getTranslate().setX(savedCameraTranslate.getX());
//         cameraForOptionalTracking.getTranslate().setY(savedCameraTranslate.getY());
//         cameraForOptionalTracking.getTranslate().setZ(savedCameraTranslate.getZ());
      }
      else // !trackRobot
      {
//         savedCameraTranslate = cameraForOptionalTracking.getTranslate().clone();
      }
   }

   public void update()
   {
      if (robotLoadedActivator.poll())
      {
         syncedRobot.update();

         super.update();

         if (trackRobot)
         {
            FramePose3DReadOnly walkingFrame = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame);

            robotTranslate.setX(walkingFrame.getPosition().getX());
            robotTranslate.setY(walkingFrame.getPosition().getY());
            robotTranslate.setZ(walkingFrame.getPosition().getZ());
         }
      }
   }

   public void destroy()
   {
      scheduler.shutdownNow();
      super.destroy();
   }
}
