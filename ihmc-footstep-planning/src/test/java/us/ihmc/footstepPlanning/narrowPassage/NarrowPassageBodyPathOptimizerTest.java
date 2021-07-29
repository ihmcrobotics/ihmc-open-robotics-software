package us.ihmc.footstepPlanning.narrowPassage;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.ui.components.FootstepPathCalculatorModule;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;

public class NarrowPassageBodyPathOptimizerTest
{
   private static final double timeout = 240.0;

   private Messager messager = null;
   private FootstepPathCalculatorModule module = null;

   @BeforeEach
   public void setup()
   {
      messager = new SharedMemoryMessager(FootstepPlannerMessagerAPI.API);
      module = new FootstepPathCalculatorModule(messager);
      module.start();

      try
      {
         messager.startMessager();
      }
      catch (Exception e)
      {
         throw new RuntimeException("Failed to start messager.");
      }


   }

   @AfterEach
   public void tearDown() throws Exception
   {
      module.stop();
      messager.closeMessager();
      module = null;
      messager = null;
   }

   @Test
   public void testJerseyBarriers60cm()
   {
      DataSet dataset = DataSetIOTools.loadDataSet(DataSetName._20190220_172417_Jersey_Barriers_JSC_60cm);
      packPlanningRequest(dataset, messager);
      FootstepPlanningModule planningModule = module.getPlanningModule();
      int numberOfWaypoints = planningModule.getBodyPathPlan().getNumberOfWaypoints();
      LogTools.info("numberOfWaypoints: " + numberOfWaypoints);
      assert(numberOfWaypoints > 2);
   }

   private void packPlanningRequest(DataSet dataset, Messager messager)
   {
      PlannerInput plannerInput = dataset.getPlannerInput();
      double startYaw = plannerInput.getStartYaw();
      double goalYaw = plannerInput.getGoalYaw();
      SideDependentList<Pose3D> startSteps = PlannerTools.createSquaredUpFootsteps(plannerInput.getStartPosition(),
                                                                                   startYaw,
                                                                                   module.getPlanningModule()
                                                                                         .getFootstepPlannerParameters()
                                                                                         .getIdealFootstepWidth());
      SideDependentList<Pose3D> goalSteps = PlannerTools.createSquaredUpFootsteps(plannerInput.getGoalPosition(),
                                                                                  goalYaw,
                                                                                  module.getPlanningModule()
                                                                                        .getFootstepPlannerParameters()
                                                                                        .getIdealFootstepWidth());

      messager.submitMessage(FootstepPlannerMessagerAPI.LeftFootPose, startSteps.get(RobotSide.LEFT));
      messager.submitMessage(FootstepPlannerMessagerAPI.RightFootPose, startSteps.get(RobotSide.RIGHT));
      messager.submitMessage(FootstepPlannerMessagerAPI.LeftFootGoalPose, goalSteps.get(RobotSide.LEFT));
      messager.submitMessage(FootstepPlannerMessagerAPI.RightFootGoalPose, goalSteps.get(RobotSide.RIGHT));

      messager.submitMessage(PlanBodyPath, true);
      messager.submitMessage(PlanNarrowPassage, true);

      messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionData, dataset.getPlanarRegionsList());

      messager.submitMessage(FootstepPlannerMessagerAPI.MaxIterations, 300);
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerTimeout, timeout);
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerHorizonLength, Double.MAX_VALUE);
      messager.submitMessage(FootstepPlannerMessagerAPI.ComputePath, true);
   }
}
