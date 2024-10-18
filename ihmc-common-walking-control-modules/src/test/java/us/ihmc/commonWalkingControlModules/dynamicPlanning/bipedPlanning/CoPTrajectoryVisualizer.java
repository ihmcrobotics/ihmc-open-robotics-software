package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.euclid.referenceFrame.PoseReferenceFrame;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.commons.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

import static us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer.defaultFeetColors;

public class CoPTrajectoryVisualizer
{
   public static void visualize(WalkingCoPTrajectoryGenerator copTrajectoryGenerator)
   {
      YoRegistry registry = new YoRegistry("visualizer");
      registry.addChild(copTrajectoryGenerator.getYoRegistry());
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      CoPPointViewer viewer = new CoPPointViewer(registry, graphicsListRegistry);

      YoFramePoint3D desiredCoP = new YoFramePoint3D("desiredCoP", ReferenceFrame.getWorldFrame(), registry);
      BagOfBalls desiredCoPViz = new BagOfBalls(100, 0.005, YoAppearance.Black(), YoGraphicPosition.GraphicType.SOLID_BALL, registry, graphicsListRegistry);

      int maxNumberOfContactPointsPerFoot = 6;
      SideDependentList<YoFrameConvexPolygon2D> footPolygonsViz = new SideDependentList<>();


      for (RobotSide robotSide : RobotSide.values)
      {
         String robotSidePrefix = robotSide.getCamelCaseNameForStartOfExpression();

         YoFrameConvexPolygon2D footPolygonViz = new YoFrameConvexPolygon2D(robotSidePrefix + "FootPolygon", "", ReferenceFrame.getWorldFrame(), maxNumberOfContactPointsPerFoot,
                                                                            registry);
         footPolygonsViz.put(robotSide, footPolygonViz);
         YoArtifactPolygon footPolygonArtifact = new YoArtifactPolygon(robotSide.getCamelCaseNameForMiddleOfExpression() + " Foot Polygon", footPolygonViz,
                                                                       defaultFeetColors.get(robotSide), false);
         graphicsListRegistry.registerArtifact("Viz", footPolygonArtifact);
      }

      List<YoFrameConvexPolygon2D> stepPolygonViz = new ArrayList<>();
      for (int i = 0; i < 1; i++)
      {
         YoFrameConvexPolygon2D footPolygonViz = new YoFrameConvexPolygon2D("upcomingStepPolygon", "" + i, ReferenceFrame.getWorldFrame(), maxNumberOfContactPointsPerFoot,
                                                                            registry);
         stepPolygonViz.add(footPolygonViz);
         YoArtifactPolygon footPolygonArtifact = new YoArtifactPolygon("Step Polygon" + i, footPolygonViz, Color.GREEN, false);
         graphicsListRegistry.registerArtifact("Viz", footPolygonArtifact);

      }


      Robot robot = new Robot("dummy");
      robot.getRobotsYoRegistry().addChild(registry);
      SimulationConstructionSet simulationConstructionSet = new SimulationConstructionSet(robot);
      simulationConstructionSet.addYoGraphicsListRegistry(graphicsListRegistry);

      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = simulationConstructionSet.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();

      simulationConstructionSet.startOnAThread();


      List<? extends ContactStateProvider> contactStateProviderList = copTrajectoryGenerator.getContactStateProviders();
      double totalDuration = Math.min(10.0, contactStateProviderList.get(contactStateProviderList.size() - 1).getTimeInterval().getEndTime());
      viewer.updateWaypoints(contactStateProviderList);
      for (RobotSide robotSide : RobotSide.values)
         footPolygonsViz.get(robotSide).setMatchingFrame(copTrajectoryGenerator.state.getFootPolygonInSole(robotSide), false);
      for (int i = 0; i < Math.min(1, copTrajectoryGenerator.state.getNumberOfFootstep()); i++)
      {
         PoseReferenceFrame pose = new PoseReferenceFrame("footPose", ReferenceFrame.getWorldFrame());
         pose.setPoseAndUpdate(copTrajectoryGenerator.state.getFootstep(i).getFootstepPose());
         FrameConvexPolygon2D polygon = new FrameConvexPolygon2D(pose);
         polygon.addVertices(Vertex2DSupplier.asVertex2DSupplier(copTrajectoryGenerator.state.getFootstep(i).getPredictedContactPoints()));
         polygon.update();
         polygon.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

         stepPolygonViz.get(i).setMatchingFrame(polygon, false);
      }

      for (double time = 0; time <= totalDuration; time += 0.01)
      {
         robot.getYoTime().set(time);
         copTrajectoryGenerator.update(time, desiredCoP);
         desiredCoPViz.setBallLoop(desiredCoP);
         simulationConstructionSet.tickAndUpdate();
      }

      ThreadTools.sleepForever();
   }

}
