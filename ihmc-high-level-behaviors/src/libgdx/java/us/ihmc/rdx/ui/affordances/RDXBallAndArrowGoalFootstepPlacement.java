package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;

import java.util.ArrayList;
import java.util.function.Consumer;

public class RDXBallAndArrowGoalFootstepPlacement extends RDXBallAndArrowPosePlacement implements RenderableProvider
{
   private ROS2SyncedRobotModel syncedRobot;
   private RDXFootstepGraphic leftGoalFootstepGraphic;
   private RDXFootstepGraphic rightGoalFootstepGraphic;
   private final FramePose3D leftFootstepGoalPose = new FramePose3D();
   private final FramePose3D rightFootstepGoalPose = new FramePose3D();
   private final RigidBodyTransform goalToWorldTransform = new RigidBodyTransform();
   private final ReferenceFrame goalPoseFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                                       goalToWorldTransform);
   private double halfIdealFootstepWidth;

   public void create(Color color, ROS2SyncedRobotModel syncedRobot)
   {
      create(null, color, syncedRobot);
   }

   public void create(Consumer<Pose3D> placedPoseConsumer, Color color, ROS2SyncedRobotModel syncedRobot)
   {
      super.create(placedPoseConsumer, color);

      this.syncedRobot = syncedRobot;

      SegmentDependentList<RobotSide, ArrayList<Point2D>> contactPoints = syncedRobot.getRobotModel().getContactPointParameters().getControllerFootGroundContactPoints();
      leftGoalFootstepGraphic = new RDXFootstepGraphic(contactPoints, RobotSide.LEFT);
      rightGoalFootstepGraphic = new RDXFootstepGraphic(contactPoints, RobotSide.RIGHT);
      leftGoalFootstepGraphic.create();
      rightGoalFootstepGraphic.create();

      halfIdealFootstepWidth = syncedRobot.getRobotModel().getFootstepPlannerParameters().getIdealFootstepWidth() / 2;
   }

   @Override
   public void processImGui3DViewInput(ImGui3DViewInput input)
   {
      super.processImGui3DViewInput(input);

      if (isPlacingGoal())
      {
         updateGoalFootstepGraphics(super.getGoalPose());
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      super.getRenderables(renderables, pool);

      if (isPlaced())
      {
         leftGoalFootstepGraphic.getRenderables(renderables, pool);
         rightGoalFootstepGraphic.getRenderables(renderables, pool);
      }
   }

   private void updateGoalFootstepGraphics(Pose3DReadOnly goalPose)
   {
      // If placing position, set graphic orientation to match robot's orientation
      if (isPlacingPosition())
      {
         leftFootstepGoalPose.set(goalPose);
         leftFootstepGoalPose.changeFrame(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
         leftFootstepGoalPose.setRotationToZero();
         leftFootstepGoalPose.getPosition().addY(halfIdealFootstepWidth);
         leftFootstepGoalPose.changeFrame(ReferenceFrame.getWorldFrame());

         rightFootstepGoalPose.set(goalPose);
         rightFootstepGoalPose.changeFrame(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
         rightFootstepGoalPose.setRotationToZero();
         rightFootstepGoalPose.getPosition().subY(halfIdealFootstepWidth);
         rightFootstepGoalPose.changeFrame(ReferenceFrame.getWorldFrame());
      }
      // If placing orientation, set graphic orientation to match arrow's orientation
      else
      {
         goalPose.get(goalToWorldTransform);
         goalPoseFrame.update();

         leftFootstepGoalPose.setToZero(goalPoseFrame);
         leftFootstepGoalPose.getPosition().addY(halfIdealFootstepWidth);
         leftFootstepGoalPose.changeFrame(ReferenceFrame.getWorldFrame());

         rightFootstepGoalPose.setToZero(goalPoseFrame);
         rightFootstepGoalPose.getPosition().subY(halfIdealFootstepWidth);
         rightFootstepGoalPose.changeFrame(ReferenceFrame.getWorldFrame());
      }

      leftGoalFootstepGraphic.setPose(leftFootstepGoalPose);
      rightGoalFootstepGraphic.setPose(rightFootstepGoalPose);
   }
}
