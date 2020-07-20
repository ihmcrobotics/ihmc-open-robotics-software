package us.ihmc.footstepPlanning.swing;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsVisualizer;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.icp.DefaultSplitFractionCalculatorParameters;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDSE3Configuration;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import java.awt.*;
import java.io.File;

public class SwingOverPlanarRegionsLogViewer
{
   public SwingOverPlanarRegionsLogViewer(String fileName)
   {
      FootstepPlannerLogLoader logLoader = new FootstepPlannerLogLoader();
      File file = new File(getClass().getClassLoader().getResource(fileName).getFile());
      logLoader.load(file);
      FootstepPlannerLog log = logLoader.getLog();

      FootstepDataListMessage footsteps = log.getStatusPacket().getFootstepDataList();
      FootstepPlan footstepPlan = new FootstepPlan();
      for (int i = 0; i < footsteps.getFootstepDataList().size(); i++)
      {
         FootstepDataMessage message = footsteps.getFootstepDataList().get(i);
         footstepPlan.addFootstep(RobotSide.fromByte(message.getRobotSide()),
                                  new FramePose3D(ReferenceFrame.getWorldFrame(), message.getLocation(), message.getOrientation()));
      }
      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setFromPacket(log.getRequestPacket());

      SwingPlannerParametersBasics parameters = new DefaultSwingPlannerParameters();
      parameters.set(log.getSwingPlannerParametersPacket());

      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(log.getStatusPacket().getPlanarRegionsList());

      WalkingControllerParameters walkingControllerParameters = getWalkingControllerParameters();
      ConvexPolygon2D foot = getFootPolygon();

      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>(side -> getFootPolygon());
      FootstepPlanningModule planningModule = new FootstepPlanningModule(getClass().getSimpleName(),
                                                                         new DefaultVisibilityGraphParameters(),
                                                                         new DefaultFootstepPlannerParameters(),
                                                                         parameters,
                                                                         new DefaultSplitFractionCalculatorParameters(),
                                                                         walkingControllerParameters,
                                                                         footPolygons);

      Graphics3DObject startGraphics = new Graphics3DObject();
      Graphics3DObject endGraphics = new Graphics3DObject();
      startGraphics.addExtrudedPolygon(foot, 0.02, YoAppearance.Color(Color.blue));
      endGraphics.addExtrudedPolygon(foot, 0.02, YoAppearance.Color(Color.RED));

      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      YoGraphicsListRegistry yoGraphicsListRegistry = planningModule.getSwingOverPlanarRegionsTrajectoryExpander().getGraphicsListRegistry();

      YoFramePoint3D firstWaypoint = new YoFramePoint3D("firstWaypoint", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint3D secondWaypoint = new YoFramePoint3D("secondWaypoint", ReferenceFrame.getWorldFrame(), registry);

      yoGraphicsListRegistry.registerYoGraphic("outputWaypoints", new YoGraphicPosition("firstWaypoint", firstWaypoint, 0.02, YoAppearance.White()));
      yoGraphicsListRegistry.registerYoGraphic("outputWaypoints", new YoGraphicPosition("secondWaypoint", secondWaypoint, 0.02, YoAppearance.White()));

      PlanarRegionsListDefinedEnvironment environment = new PlanarRegionsListDefinedEnvironment("environment", planarRegionsList, 1e-2, false);

      SwingOverPlanarRegionsTrajectoryExpander expander = planningModule.getSwingOverPlanarRegionsTrajectoryExpander();
      registry.addChild(planningModule.getYoVariableRegistry());

      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));

      SwingOverPlanarRegionsVisualizer visualizer = new SwingOverPlanarRegionsVisualizer(scs, registry, yoGraphicsListRegistry, foot, expander);
      expander.attachVisualizer(visualizer::update);

      scs.setDT(1.0, 1);
      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setGroundVisible(false);
      scs.addStaticLinkGraphics(environment.getTerrainObject3D().getLinkGraphics());

      planningModule.getPostProcessHandler().computeSwingWaypoints(request, footstepPlan);

      scs.startOnAThread();
      scs.cropBuffer();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      String fileName = "20200717_143721207_FootstepPlannerLog";
      SwingOverPlanarRegionsLogViewer viewer = new SwingOverPlanarRegionsLogViewer(fileName);
   }

   private WalkingControllerParameters getWalkingControllerParameters()
   {
      return new WalkingControllerParameters()
      {
         @Override
         public double getOmega0()
         {
            return 0;
         }

         @Override
         public boolean allowDisturbanceRecoveryBySpeedingUpSwing()
         {
            return false;
         }

         @Override
         public double getMinimumSwingTimeForDisturbanceRecovery()
         {
            return 0;
         }

         @Override
         public double getICPErrorThresholdToSpeedUpSwing()
         {
            return 0;
         }

         @Override
         public boolean allowAutomaticManipulationAbort()
         {
            return false;
         }

         @Override
         public ICPControlGains createICPControlGains()
         {
            return null;
         }

         @Override
         public PDGains getCoMHeightControlGains()
         {
            return null;
         }

         @Override
         public PIDSE3Configuration getSwingFootControlGains()
         {
            return null;
         }

         @Override
         public PIDSE3Configuration getHoldPositionFootControlGains()
         {
            return null;
         }

         @Override
         public PIDSE3Configuration getToeOffFootControlGains()
         {
            return null;
         }

         @Override
         public double getDefaultTransferTime()
         {
            return 0;
         }

         @Override
         public double getDefaultSwingTime()
         {
            return 0;
         }

         @Override
         public FootSwitchFactory getFootSwitchFactory()
         {
            return null;
         }

         @Override
         public String[] getJointsToIgnoreInController()
         {
            return new String[0];
         }

         @Override
         public MomentumOptimizationSettings getMomentumOptimizationSettings()
         {
            return null;
         }

         @Override
         public ICPAngularMomentumModifierParameters getICPAngularMomentumModifierParameters()
         {
            return null;
         }

         @Override
         public double getMaxICPErrorBeforeSingleSupportForwardX()
         {
            return 0;
         }

         @Override
         public double getMaxICPErrorBeforeSingleSupportInnerY()
         {
            return 0;
         }

         @Override
         public ToeOffParameters getToeOffParameters()
         {
            return null;
         }

         @Override
         public SwingTrajectoryParameters getSwingTrajectoryParameters()
         {
            return getTestSwingTrajectoryParameters();
         }

         @Override
         public ICPOptimizationParameters getICPOptimizationParameters()
         {
            return null;
         }

         @Override
         public double getMaximumLegLengthForSingularityAvoidance()
         {
            return 0;
         }

         @Override
         public double minimumHeightAboveAnkle()
         {
            return 0;
         }

         @Override
         public double nominalHeightAboveAnkle()
         {
            return 0;
         }

         @Override
         public double maximumHeightAboveAnkle()
         {
            return 0;
         }

         @Override
         public double defaultOffsetHeightAboveAnkle()
         {
            return 0;
         }

         @Override
         public SteppingParameters getSteppingParameters()
         {
            return getTestSteppingParameters();
         }
      };
   }

   private SteppingParameters getTestSteppingParameters()
   {
      return new SteppingParameters()
      {
         @Override
         public double getMinSwingHeightFromStanceFoot()
         {
            return 0.10;
         }

         @Override
         public double getDefaultSwingHeightFromStanceFoot()
         {
            return getMinSwingHeightFromStanceFoot();
         }

         @Override
         public double getMaxSwingHeightFromStanceFoot()
         {
            return 0.30;
         }

         @Override
         public double getFootForwardOffset()
         {
            return getFootLength() - getFootBackwardOffset();
         }

         @Override
         public double getFootBackwardOffset()
         {
            return 0.085;
         }

         @Override
         public double getInPlaceWidth()
         {
            return 0.25;
         }

         @Override
         public double getDesiredStepForward()
         {
            return 0.5; // 0.35;
         }

         @Override
         public double getMaxStepLength()
         {
            return 0.6; // 0.5; //0.35;
         }

         @Override
         public double getMinStepWidth()
         {
            return 0.15;
         }

         @Override
         public double getMaxStepWidth()
         {
            return 0.6; // 0.4;
         }

         @Override
         public double getStepPitch()
         {
            return 0.0;
         }

         @Override
         public double getDefaultStepLength()
         {
            return 0.6;
         }

         @Override
         public double getMaxStepUp()
         {
            return 0.25;
         }

         @Override
         public double getMaxStepDown()
         {
            return 0.2;
         }

         @Override
         public double getMaxAngleTurnOutwards()
         {
            //increased atlas turn speed defaults
            // return Math.PI / 4.0;
            return 0.6;
         }

         @Override
         public double getMaxAngleTurnInwards()
         {
            //increased atlas turn speed defaults
            //  return 0;
            return -0.1;
         }

         @Override
         public double getTurningStepWidth()
         {
            return 0.25;
         }

         @Override
         public double getMinAreaPercentForValidFootstep()
         {
            return 0.5;
         }

         @Override
         public double getDangerAreaPercentForValidFootstep()
         {
            return 0.75;
         }

         @Override
         public double getFootWidth()
         {
            return 0.11;
         }

         @Override
         public double getToeWidth()
         {
            return 0.085;
         }

         @Override
         public double getFootLength()
         {
            return 0.22;
         }

         @Override
         public double getActualFootWidth()
         {
            return 0.138;
         }

         @Override
         public double getActualFootLength()
         {
            return 0.26;
         }
      };
   }

   public SwingTrajectoryParameters getTestSwingTrajectoryParameters()
   {
      return new SwingTrajectoryParameters()
      {
         @Override
         public boolean doToeTouchdownIfPossible()
         {
            return false;
         }

         @Override
         public double getToeTouchdownAngle()
         {
            return Math.toRadians(20.0);
         }

         @Override
         public boolean doHeelTouchdownIfPossible()
         {
            return false;
         }

         @Override
         public double getHeelTouchdownAngle()
         {
            return Math.toRadians(-5.0);
         }

         @Override
         public double getMinMechanicalLegLength()
         {
            return 0.420;
         }

         @Override
         public double getDesiredTouchdownHeightOffset()
         {
            return 0;
         }

         @Override
         public double getDesiredTouchdownVelocity()
         {
            return -0.3;
         }

         @Override
         public double getDesiredTouchdownAcceleration()
         {
            return -1.0;
         }

         /** {@inheritDoc} */
         @Override
         public double getSwingFootVelocityAdjustmentDamping()
         {
            return 0.8;
         }

         /** {@inheritDoc} */
         @Override
         public boolean addOrientationMidpointForObstacleClearance()
         {
            return false;
         }

         /** {@inheritDoc} */
         @Override
         public boolean useSingularityAvoidanceInSupport()
         {
            return true;
         }
      };
   }

   private ConvexPolygon2D getFootPolygon()
   {
      SteppingParameters steppingParameters = getWalkingControllerParameters().getSteppingParameters();

      ConvexPolygon2D foot = new ConvexPolygon2D();
      foot.addVertex(steppingParameters.getFootForwardOffset(), -0.5 * steppingParameters.getToeWidth());
      foot.addVertex(steppingParameters.getFootForwardOffset(), 0.5 * steppingParameters.getToeWidth());
      foot.addVertex(-steppingParameters.getFootBackwardOffset(), -0.5 * steppingParameters.getFootWidth());
      foot.addVertex(-steppingParameters.getFootBackwardOffset(), 0.5 * steppingParameters.getFootWidth());
      foot.update();

      return foot;
   }
}
