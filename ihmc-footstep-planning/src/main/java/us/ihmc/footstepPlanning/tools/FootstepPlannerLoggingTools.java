package us.ihmc.footstepPlanning.tools;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.log.LogTools;
import us.ihmc.perception.logging.PerceptionDataLogger;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootstepPlannerLoggingTools
{
   public static void printFootstepPlan(FootstepPlannerOutput plannerOutput)
   {
      FootstepPlan plan = plannerOutput.getFootstepPlan();
      Point3D footstepPosition = new Point3D();
      Quaternion footstepOrientation = new Quaternion();

      LogTools.info("\n\n  -----  Number of Steps - {}", plan.getNumberOfSteps());

      for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         PlannedFootstep footstep = plan.getFootstep(i);
         footstepPosition.set(footstep.getFootstepPose().getTranslation());
         footstepOrientation.set(footstep.getFootstepPose().getOrientation());

         LogTools.info("\t ----------- Footstep: {} {} {}", footstepPosition, footstepOrientation, footstep.getRobotSide());
      }
   }

   public static void logFootsteps(FootstepPlannerOutput plannerOutput,
                                   PerceptionDataLogger perceptionDataLogger,
                                   RigidBodyTransform transformToWorld,
                                   SideDependentList<FramePose3D> startPose,
                                   SideDependentList<FramePose3D> goalPose,
                                   boolean sidednessBit)
   {
      FootstepPlan plan = plannerOutput.getFootstepPlan();
      Point3D footstepPosition = new Point3D();
      Quaternion footstepOrientation = new Quaternion();
      float sideValue = 0.0f;
      for (int i = 0; i < PerceptionLoggerConstants.LEGACY_BLOCK_SIZE; i++)
      {
         if (i < plan.getNumberOfSteps())
         {
            PlannedFootstep footstep = plan.getFootstep(i);
            footstepPosition.set(footstep.getFootstepPose().getTranslation());
            footstepOrientation.set(footstep.getFootstepPose().getOrientation());
            sideValue = footstep.getRobotSide() == RobotSide.LEFT ? 0.0f : 1.0f;
         }
         else
         {
            footstepPosition.set(0.0, 0.0, 0.0);
            sideValue = 0.0f;
         }
         perceptionDataLogger.storeFloats(PerceptionLoggerConstants.FOOTSTEP_SIDE, sideValue);
         perceptionDataLogger.storeFloats(PerceptionLoggerConstants.FOOTSTEP_POSITION, footstepPosition);
         perceptionDataLogger.storeFloats(PerceptionLoggerConstants.FOOTSTEP_ORIENTATION, footstepOrientation);
      }
      perceptionDataLogger.storeFloats(PerceptionLoggerConstants.INITIAL_FOOTSTEP_SIDE, sidednessBit ? 0.0f : 1.0f);
      perceptionDataLogger.storeFloats(PerceptionLoggerConstants.L515_SENSOR_POSITION, new Point3D(transformToWorld.getTranslation()));
      perceptionDataLogger.storeFloats(PerceptionLoggerConstants.L515_SENSOR_ORIENTATION, new Quaternion(transformToWorld.getRotation()));

      perceptionDataLogger.storeFloats(PerceptionLoggerConstants.START_FOOTSTEP_POSITION, new Point3D(startPose.get(sidednessBit ? RobotSide.LEFT : RobotSide.RIGHT).getTranslation()));
      perceptionDataLogger.storeFloats(PerceptionLoggerConstants.START_FOOTSTEP_ORIENTATION, new Quaternion(startPose.get(sidednessBit ? RobotSide.LEFT : RobotSide.RIGHT).getOrientation()));
      perceptionDataLogger.storeFloats(PerceptionLoggerConstants.GOAL_FOOTSTEP_POSITION, new Point3D(goalPose.get(RobotSide.LEFT).getTranslation()));
      perceptionDataLogger.storeFloats(PerceptionLoggerConstants.GOAL_FOOTSTEP_ORIENTATION, new Quaternion(goalPose.get(RobotSide.LEFT).getOrientation()));
   }
}
