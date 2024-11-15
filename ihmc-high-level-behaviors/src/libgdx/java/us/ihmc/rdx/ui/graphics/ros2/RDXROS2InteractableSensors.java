package us.ihmc.rdx.ui.graphics.ros2;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableFrameModel;
import us.ihmc.rdx.ui.interactable.RDXInteractableBlackflyFujinon;
import us.ihmc.rdx.ui.interactable.RDXInteractableOuster;
import us.ihmc.rdx.ui.interactable.RDXInteractableRealsenseD455;
import us.ihmc.rdx.ui.interactable.RDXInteractableZED2i;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

public class RDXROS2InteractableSensors
{
   private final RDXBaseUI baseUI;
   private final ROS2PublishSubscribeAPI ros2;
   private final ROS2SyncedRobotModel syncedRobot;
   private final HumanoidReferenceFrames referenceFramesToUseForSensors;
   private final RDXROS2RobotVisualizer robotVisualizer;

   private final ArrayList<RDXInteractableFrameModel> interactableFrameModels = new ArrayList<>();
   private RDXInteractableOuster interactableOuster;
   private RDXInteractableRealsenseD455 interactableRealsenseD455;
   private final SideDependentList<RDXInteractableBlackflyFujinon> interactableBlackflyFujinons = new SideDependentList<>();
   private RDXInteractableZED2i interactableZED2i;

   /**
    * @param referenceFramesToUseForSensors In simulation we want to use the ground truth frames rather than state estimator frames.
    */
   public RDXROS2InteractableSensors(RDXBaseUI baseUI,
                                     ROS2PublishSubscribeAPI ros2,
                                     ROS2SyncedRobotModel syncedRobot,
                                     HumanoidReferenceFrames referenceFramesToUseForSensors,
                                     RDXROS2RobotVisualizer robotVisualizer)
   {
      this.baseUI = baseUI;
      this.ros2 = ros2;
      this.syncedRobot = syncedRobot;
      this.referenceFramesToUseForSensors = referenceFramesToUseForSensors;
      this.robotVisualizer = robotVisualizer;

      robotVisualizer.addUpdateListener(this::update);
      robotVisualizer.addOpacityListener(this::setOpacity);
      robotVisualizer.addActiveListener(this::setShowing);
   }

   public void setupOuster()
   {
      interactableOuster = new RDXInteractableOuster(baseUI.getPrimary3DPanel(),
                                                     referenceFramesToUseForSensors.getOusterLidarFrame(),
                                                     syncedRobot.getRobotModel().getSensorInformation().getOusterLidarTransform());
      interactableOuster.getInteractableFrameModel()
                        .addRemoteTuning(ros2,
                                         PerceptionAPI.OUSTER_TO_CHEST_TUNING,
                                         syncedRobot.getRobotModel().getSensorInformation().getOusterLidarTransform());
      interactableFrameModels.add(interactableOuster.getInteractableFrameModel());
   }

   public void setupRealsenseD455()
   {
      interactableRealsenseD455 = new RDXInteractableRealsenseD455(baseUI.getPrimary3DPanel(),
                                                                   referenceFramesToUseForSensors.getSteppingCameraFrame(),
                                                                   syncedRobot.getRobotModel().getSensorInformation().getSteppingCameraTransform());
      interactableRealsenseD455.getInteractableFrameModel()
                               .addRemoteTuning(ros2,
                                                PerceptionAPI.STEPPING_CAMERA_TO_PARENT_TUNING,
                                                syncedRobot.getRobotModel().getSensorInformation().getSteppingCameraTransform());
      interactableFrameModels.add(interactableRealsenseD455.getInteractableFrameModel());
   }

   public void setupBlackflyFujinons()
   {
      RDXInteractableBlackflyFujinon interactableBlackflyLeftFujinon = new RDXInteractableBlackflyFujinon(baseUI.getPrimary3DPanel(),
                                                                                                          referenceFramesToUseForSensors.getSituationalAwarenessCameraFrame(RobotSide.LEFT),
                                                                                                          syncedRobot.getRobotModel().getSensorInformation().getSituationalAwarenessCameraTransform(RobotSide.LEFT));
      interactableBlackflyLeftFujinon.getInteractableFrameModel()
                                     .addRemoteTuning(ros2,
                                                      PerceptionAPI.SITUATIONAL_AWARENESS_CAMERA_TO_PARENT_TUNING.get(RobotSide.LEFT),
                                                      syncedRobot.getRobotModel().getSensorInformation().getSituationalAwarenessCameraTransform(RobotSide.LEFT));
      interactableBlackflyFujinons.set(RobotSide.LEFT, interactableBlackflyLeftFujinon);
      interactableFrameModels.add(interactableBlackflyLeftFujinon.getInteractableFrameModel());

      RDXInteractableBlackflyFujinon interactableBlackflyRightFujinon = new RDXInteractableBlackflyFujinon(baseUI.getPrimary3DPanel(),
                                                                                                           referenceFramesToUseForSensors.getSituationalAwarenessCameraFrame(RobotSide.RIGHT),
                                                                                                           syncedRobot.getRobotModel().getSensorInformation().getSituationalAwarenessCameraTransform(RobotSide.RIGHT));
      interactableBlackflyRightFujinon.getInteractableFrameModel()
                                      .addRemoteTuning(ros2,
                                                       PerceptionAPI.SITUATIONAL_AWARENESS_CAMERA_TO_PARENT_TUNING.get(RobotSide.RIGHT),
                                                       syncedRobot.getRobotModel().getSensorInformation().getSituationalAwarenessCameraTransform(RobotSide.RIGHT));
      interactableBlackflyFujinons.set(RobotSide.RIGHT, interactableBlackflyRightFujinon);
      interactableFrameModels.add(interactableBlackflyRightFujinon.getInteractableFrameModel());
   }

   public void setupZED2i()
   {
      interactableZED2i = new RDXInteractableZED2i(baseUI.getPrimary3DPanel(),
                                                   referenceFramesToUseForSensors.getExperimentalCameraFrame(),
                                                   syncedRobot.getRobotModel().getSensorInformation().getExperimentalCameraTransform());
      interactableZED2i.getInteractableFrameModel().addRemoteTuning(ros2,
                                                                    PerceptionAPI.EXPERIMENTAL_CAMERA_TO_PARENT_TUNING,
                                                                    syncedRobot.getRobotModel().getSensorInformation().getExperimentalCameraTransform());
      interactableFrameModels.add(interactableZED2i.getInteractableFrameModel());
   }

   private void update()
   {
      setShowing(!robotVisualizer.getHideChest().get());

      for (RDXInteractableFrameModel interactableFrameModel : interactableFrameModels)
         interactableFrameModel.update();
   }

   public void setShowing(boolean showing)
   {
      for (RDXInteractableFrameModel interactableFrameModel : interactableFrameModels)
         interactableFrameModel.setShowing(showing);
   }

   public void setOpacity(float opacity)
   {
      for (RDXInteractableFrameModel interactableFrameModel : interactableFrameModels)
         interactableFrameModel.getModelInstance().setOpacity(opacity);
   }
}
