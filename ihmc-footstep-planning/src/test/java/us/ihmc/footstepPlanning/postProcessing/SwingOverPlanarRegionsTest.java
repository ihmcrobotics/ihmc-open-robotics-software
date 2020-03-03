package us.ihmc.footstepPlanning.postProcessing;

import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepPostProcessingPacket;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander.SwingOverPlanarRegionsTrajectoryCollisionType;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsStandaloneVisualizer;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsVisualizer;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.postProcessing.parameters.DefaultFootstepPostProcessingParameters;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicEllipsoid;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDSE3Configuration;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;

import java.awt.Color;
import java.util.HashMap;
import java.util.List;

import static us.ihmc.robotics.Assert.assertTrue;

public class SwingOverPlanarRegionsTest
{
   @Test
   public void testBigStepDown()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      generator.translate(0.0, 0.0, 0.5);
      generator.addRectangle(0.4, 0.4);

      generator.translate(0.5, 0.0, -0.15);
      generator.rotateEuler(new Vector3D(0.0, Math.toRadians(20), 0.0));
      generator.addRectangle(0.4, 0.4);



      double width = 0.25;
      FramePose3D startFoot = new FramePose3D();
      startFoot.setPosition(0.0, -width / 2.0, 0.5);

      FramePose3D endFoot = new FramePose3D();
      endFoot.setPosition(0.6, -width / 2.0, 0.36);
      endFoot.setOrientationYawPitchRoll(0.0, Math.toRadians(20.0), 0.0);

      runTest(startFoot, endFoot, generator.getPlanarRegionsList());
   }

   @Test
   public void testBigStepDown2()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      generator.translate(0.0, 0.0, 0.5);
      generator.addRectangle(0.4, 0.4);

      generator.translate(0.5, 0.0, -0.15);
      generator.rotateEuler(new Vector3D(0.0, Math.toRadians(20), 0.0));
      generator.addRectangle(0.4, 0.4);



      double width = 0.25;
      FramePose3D startFoot = new FramePose3D();
      startFoot.setPosition(0.0, -width / 2.0, 0.5);

      FramePose3D endFoot = new FramePose3D();
      endFoot.setPosition(0.6, -width / 2.0, 0.31);
      endFoot.setOrientationYawPitchRoll(0.0, Math.toRadians(20.0), 0.0);

      runTest(startFoot, endFoot, generator.getPlanarRegionsList());
   }

   private void runTest(FramePose3DReadOnly startFoot, FramePose3DReadOnly endFoot, PlanarRegionsList planarRegionsList)
   {
      ConvexPolygon2D feet = PlannerTools.createDefaultFootPolygon();

      Graphics3DObject startGraphics = new Graphics3DObject();
      Graphics3DObject endGraphics = new Graphics3DObject();
      startGraphics.addExtrudedPolygon(feet, 0.02, YoAppearance.Color(Color.blue));
      endGraphics.addExtrudedPolygon(feet, 0.02, YoAppearance.Color(Color.RED));

      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      RobotSide swingSide = RobotSide.RIGHT;

      YoFramePoint3D firstWaypoint = new YoFramePoint3D("firstWaypoint", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint3D secondWaypoint = new YoFramePoint3D("secondWaypoint", ReferenceFrame.getWorldFrame(), registry);

      YoFramePoseUsingYawPitchRoll yoStartFoot = new YoFramePoseUsingYawPitchRoll("start", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoseUsingYawPitchRoll yoEndFoot = new YoFramePoseUsingYawPitchRoll("end", ReferenceFrame.getWorldFrame(), registry);
      yoStartFoot.set(startFoot);
      yoEndFoot.set(endFoot);

      yoGraphicsListRegistry.registerYoGraphic("footsteps", new YoGraphicShape("startFootstep", startGraphics, yoStartFoot, 1.0));
      yoGraphicsListRegistry.registerYoGraphic("footsteps", new YoGraphicShape("endFootstep", endGraphics, yoEndFoot, 1.0));
      yoGraphicsListRegistry.registerYoGraphic("outputWaypoints", new YoGraphicPosition("firstWaypoint", firstWaypoint, 0.02, YoAppearance.White()));
      yoGraphicsListRegistry.registerYoGraphic("outputWaypoints", new YoGraphicPosition("secondWaypoint", secondWaypoint, 0.02, YoAppearance.White()));

      DefaultFootstepPostProcessingParameters parameters = new DefaultFootstepPostProcessingParameters();
      parameters.setMaximumWaypointAdjustmentDistance(1.0);
      SwingOverRegionsPostProcessingElement swingOverElement = new SwingOverRegionsPostProcessingElement(parameters, getWalkingControllerParameters(), registry,
                                                                                                         yoGraphicsListRegistry);

      FootstepPostProcessingPacket postProcessingPacket = new FootstepPostProcessingPacket();
      postProcessingPacket.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));
      FootstepDataMessage firstStep = postProcessingPacket.getFootstepDataList().getFootstepDataList().add();

      firstStep.getLocation().set(endFoot.getPosition());
      firstStep.getOrientation().set(endFoot.getOrientation());
      firstStep.setRobotSide(swingSide.toByte());

      postProcessingPacket.getRightFootPositionInWorld().set(startFoot.getPosition());
      postProcessingPacket.getRightFootOrientationInWorld().set(startFoot.getOrientation());



      PlanarRegionsListDefinedEnvironment environment = new PlanarRegionsListDefinedEnvironment("environment", planarRegionsList, 1e-2, false);
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));


      SwingOverPlanarRegionsTrajectoryExpander expander = swingOverElement.swingOverPlanarRegionsTrajectoryExpander;
      SwingOverPlanarRegionsVisualizer visualizer = new SwingOverPlanarRegionsVisualizer(scs, registry, yoGraphicsListRegistry, feet, expander);
      expander.attachVisualizer(visualizer::update);

      scs.setDT(1.0, 1);
      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setGroundVisible(false);
      scs.addStaticLinkGraphics(environment.getTerrainObject3D().getLinkGraphics());


      FootstepPostProcessingPacket processedPacket = swingOverElement.postProcessFootstepPlan(postProcessingPacket);

      boolean wasAdjusted = expander.wereWaypointsAdjusted();
      if (wasAdjusted)
         assertTrue(processedPacket.getFootstepDataList().getFootstepDataList().get(0).getCustomPositionWaypoints().size() > 0);

      if (wasAdjusted)
      {
         firstWaypoint.set(processedPacket.getFootstepDataList().getFootstepDataList().get(0).getCustomPositionWaypoints().get(0));
         secondWaypoint.set(processedPacket.getFootstepDataList().getFootstepDataList().get(0).getCustomPositionWaypoints().get(1));

         List<FramePoint3D> expandedWaypoints = expander.getExpandedWaypoints();
         for (int i = 0; i < expandedWaypoints.size(); i++)
         {
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expandedWaypoints.get(i), processedPacket.getFootstepDataList().getFootstepDataList().get(0).getCustomPositionWaypoints().get(i), 1e-8);
         }
      }
      else
      {
         firstWaypoint.setToNaN();
         secondWaypoint.setToNaN();
      }




      scs.startOnAThread();
      scs.cropBuffer();
      ThreadTools.sleepForever();
   }

   private void update(SwingOverPlanarRegionsTrajectoryExpander expander, SimulationConstructionSet scs, YoFramePoseUsingYawPitchRoll solePose,
                       HashMap<SwingOverPlanarRegionsTrajectoryCollisionType, YoGraphicPosition> intersectionMap, YoGraphicEllipsoid collisionSphere)
   {
      solePose.setFromReferenceFrame(expander.getSolePoseReferenceFrame());

      for (SwingOverPlanarRegionsTrajectoryCollisionType swingOverPlanarRegionsTrajectoryCollisionType : SwingOverPlanarRegionsTrajectoryCollisionType.values())
      {
         intersectionMap.get(swingOverPlanarRegionsTrajectoryCollisionType)
                        .setPosition(expander.getClosestPolygonPoint(swingOverPlanarRegionsTrajectoryCollisionType));
      }

      double sphereRadius = expander.getSphereRadius();
      collisionSphere.setRadii(new Vector3D(sphereRadius, sphereRadius, sphereRadius));
      collisionSphere.update();

      scs.tickAndUpdate(scs.getTime() + 0.1);
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
}
