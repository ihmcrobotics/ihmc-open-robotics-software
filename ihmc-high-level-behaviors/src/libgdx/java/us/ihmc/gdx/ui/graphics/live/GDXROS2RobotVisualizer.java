package us.ihmc.gdx.ui.graphics.live;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.ui.graphics.GDXRobotModelGraphic;

public class GDXROS2RobotVisualizer extends GDXRobotModelGraphic
{
//   private boolean trackRobot = false;
//   private FocusBasedGDXCamera cameraForOptionalTracking;
//   private Translate robotTranslate;
//   private Translate savedCameraTranslate;
//   private boolean hasPrepended = false;
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ImGuiPlot receivedPlot = new ImGuiPlot("RobotConfigurationData", 1000, 230, 20);
   private volatile long receivedPackets = 0;

   public GDXROS2RobotVisualizer(DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobot)
   {
      super(robotModel.getSimpleRobotName() + " Robot Visualizer (ROS 2)");
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      syncedRobot.addRobotConfigurationDataReceivedCallback(() -> ++receivedPackets);
   }

   @Override
   public void create()
   {
      super.create();
      loadRobotModelAndGraphics(robotModel.getRobotDescription(), syncedRobot.getFullRobotModel().getElevator());
   }

   //   public void setTrackRobot(FocusBasedGDXCamera camera, boolean trackRobot)
//   {
//      this.trackRobot = trackRobot;
//      cameraForOptionalTracking = camera;
//      if (!hasPrepended)
//      {
//         hasPrepended = true;
////         robotTranslate = new Translate();
////         cameraForOptionalTracking.prependTransform(robotTranslate);
//      }
//      else if (trackRobot)
//      {
////         cameraForOptionalTracking.getTranslate().setX(savedCameraTranslate.getX());
////         cameraForOptionalTracking.getTranslate().setY(savedCameraTranslate.getY());
////         cameraForOptionalTracking.getTranslate().setZ(savedCameraTranslate.getZ());
//      }
//      else // !trackRobot
//      {
////         savedCameraTranslate = cameraForOptionalTracking.getTranslate().clone();
//      }
//   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      receivedPlot.render(receivedPackets);
   }

   @Override
//   public void update()
//   {
//      if (isRobotLoaded())
//      {
//         super.update();
//
////         if (trackRobot)
////         {
////            FramePose3DReadOnly walkingFrame = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame);
////
////            robotTranslate.setX(walkingFrame.getPosition().getX());
////            robotTranslate.setY(walkingFrame.getPosition().getY());
////            robotTranslate.setZ(walkingFrame.getPosition().getZ());
////         }
//      }
//   }

   public void destroy()
   {
      super.destroy();
   }
}
