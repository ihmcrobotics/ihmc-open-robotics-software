package us.ihmc.behaviors.lookAndStep;

import behavior_msgs.msg.dds.MinimalFootstepListMessage;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage;
import toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonsMessage;
import us.ihmc.behaviors.tools.MinimalFootstep;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.Random;

public class LookAndStepAPITest
{
   @Test
   public void testMinimalFoostepMessagePacking()
   {
      FootstepPlan footstepPlan = new FootstepPlan();

      Random random = new Random();
      FramePose3D foot0Pose = new FramePose3D(ReferenceFrame.getWorldFrame(), EuclidCoreRandomTools.nextPoint3D(random),
                                              EuclidCoreRandomTools.nextQuaternion(random));
      FramePose3D foot1Pose = new FramePose3D(ReferenceFrame.getWorldFrame(), EuclidCoreRandomTools.nextPoint3D(random),
                                              EuclidCoreRandomTools.nextQuaternion(random));
      footstepPlan.addFootstep(RobotSide.LEFT, foot0Pose);
      footstepPlan.addFootstep(RobotSide.RIGHT, foot1Pose);

      String description = "My plan";
      MinimalFootstepListMessage minimalFootstepListMessage = MinimalFootstep.reduceFootstepPlanForUIROS2(footstepPlan, description);

      ArrayList<MinimalFootstep> minimalFootsteps = MinimalFootstep.convertMinimalFootstepListMessage(minimalFootstepListMessage);

      MinimalFootstep minimalFootstep0 = minimalFootsteps.get(0);
      MinimalFootstep minimalFootstep1 = minimalFootsteps.get(1);

      Assertions.assertEquals(minimalFootstep0.getSide(), RobotSide.LEFT);
      Assertions.assertEquals(minimalFootstep1.getSide(), RobotSide.RIGHT);
      Assertions.assertEquals(minimalFootstep0.getDescription(), "");
      Assertions.assertEquals(minimalFootstep1.getDescription(), description);

      EuclidCoreTestTools.assertEquals(minimalFootstep0.getSolePoseInWorld(), foot0Pose, 1e-5);
      EuclidCoreTestTools.assertEquals(minimalFootstep1.getSolePoseInWorld(), foot1Pose, 1e-5);
   }

   @Test
   public void testRejectionReasonsMessagePacking()
   {
      FootstepPlannerRejectionReasonsMessage footstepPlannerRejectionReasonsMessage = new FootstepPlannerRejectionReasonsMessage();
      FootstepPlannerRejectionReasonMessage reason0 = footstepPlannerRejectionReasonsMessage.getRejectionReasons().add();
      reason0.setReason(BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH.ordinal());
      reason0.setRejectionPercentage(0.5f);
      FootstepPlannerRejectionReasonMessage reason1 = footstepPlannerRejectionReasonsMessage.getRejectionReasons().add();
      reason1.setReason(BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP.ordinal());
      reason1.setRejectionPercentage(0.6f);
      FootstepPlannerRejectionReasonMessage reason2 = footstepPlannerRejectionReasonsMessage.getRejectionReasons().add();
      reason2.setReason(BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FAR.ordinal());
      reason2.setRejectionPercentage(0.1f);
   }
}
