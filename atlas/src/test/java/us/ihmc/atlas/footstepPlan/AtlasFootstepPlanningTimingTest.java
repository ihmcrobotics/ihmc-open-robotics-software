package us.ihmc.atlas.footstepPlan;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.robotics.robotSide.RobotSide;

import java.text.SimpleDateFormat;
import java.util.Date;

public class AtlasFootstepPlanningTimingTest
{
   public AtlasFootstepPlanningTimingTest()
   {
      FootstepPlanningModule planningModule = FootstepPlanningModuleLauncher.createModule(new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ));
      DataSet dataSet = DataSetIOTools.loadDataSet(DataSetName._20190220_172417_EOD_Cinders);
      PlannerInput plannerInput = dataSet.getPlannerInput();

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setTimeout(Double.MAX_VALUE);
      Pose3D initialMidFootPose = new Pose3D(plannerInput.getStartPosition(), new Quaternion(plannerInput.getStartYaw(), 0.0, 0.0));
      Pose3D goalMidFootPose = new Pose3D(plannerInput.getGoalPosition(), new Quaternion(plannerInput.getGoalYaw(), 0.0, 0.0));
      request.setStartFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), initialMidFootPose);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);
      request.setRequestedInitialStanceSide(RobotSide.LEFT);
      request.setPlanarRegionsList(dataSet.getPlanarRegionsList());
      request.setPlanBodyPath(false);
      request.setAbortIfBodyPathPlannerFails(false);

      int warmups = 3;
      for (int i = 0; i < warmups; i++)
      {
         planningModule.handleRequest(request);
      }

      FootstepPlannerOutput output = planningModule.handleRequest(request);
      long iterations = output.getPlannerTimings().getStepPlanningIterations();
      double timePlanningStepsSeconds = output.getPlannerTimings().getTimePlanningStepsSeconds();

      double timePerIteration = timePlanningStepsSeconds / iterations;

      String date = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
      System.out.println("Date" + "\t\t\t\t" + "Time per iteration" + "\t\t" + "# iter" + "\t\t" + "total time" + "\t\t" + "Note");
      System.out.println(date + "\t\t" + timePerIteration + "\t" + output.getPlannerTimings().getStepPlanningIterations() + "\t\t\t" + String.format("%.3f", output.getPlannerTimings().getTimePlanningStepsSeconds()));
   }

   //
   // Date				      Time per iteration      # iter   total time		Note
   // 20200427_112554		0.024912095983042597	   59			1.470          before reducing branch factor
   // 20200427_112958		0.0223873512068794	   58			1.298          reduced branch factor, no expansion mask
   // 20200427_113057		0.005916966183650977	   49			0.290          reduced branch factor and expansion mask
   //

   public static void main(String[] args)
   {
      new AtlasFootstepPlanningTimingTest();
   }
}
