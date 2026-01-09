package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

import static us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer.defaultFeetColorDefinitions;
import static us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer.defaultFeetColors;

public class CoPTrajectoryVisualizer
{
   public static void visualize(WalkingCoPTrajectoryGenerator copTrajectoryGenerator)
   {
      YoRegistry registry = new YoRegistry("visualizer");
      registry.addChild(copTrajectoryGenerator.getYoRegistry());
      CoPPointViewer viewer = new CoPPointViewer(registry);

      YoFramePoint3D desiredCoP = new YoFramePoint3D("desiredCoP", ReferenceFrame.getWorldFrame(), registry);
      BagOfBalls desiredCoPViz = new BagOfBalls(100, 0.005, YoAppearance.Black(), YoGraphicPosition.GraphicType.SOLID_BALL, registry, null);

      int maxNumberOfContactPointsPerFoot = 6;
      SideDependentList<YoFrameConvexPolygon2D> footPolygonsViz = new SideDependentList<>();


      SimulationConstructionSet2 scs2 = new SimulationConstructionSet2();
      for (int i = 0; i < 100; i++)
         scs2.addYoGraphic(YoGraphicDefinitionFactory.newYoGraphicPoint3D("desired CoP " + i, desiredCoPViz.getPositions().get(i), 0.0025, ColorDefinitions.Black()));

      for (RobotSide robotSide : RobotSide.values)
      {
         String robotSidePrefix = robotSide.getCamelCaseNameForStartOfExpression();

         YoFrameConvexPolygon2D footPolygonViz = new YoFrameConvexPolygon2D(robotSidePrefix + "FootPolygon", "", ReferenceFrame.getWorldFrame(), maxNumberOfContactPointsPerFoot,
                                                                            registry);
         footPolygonsViz.put(robotSide, footPolygonViz);
         scs2.addYoGraphic(YoGraphicDefinitionFactory.newYoGraphicPolygon2D(robotSide.getCamelCaseNameForMiddleOfExpression() + " Foot Polygon", footPolygonViz,
                                                                       defaultFeetColorDefinitions.get(robotSide), false));
      }

      List<YoFrameConvexPolygon2D> stepPolygonViz = new ArrayList<>();
      for (int i = 0; i < 1; i++)
      {
         YoFrameConvexPolygon2D footPolygonViz = new YoFrameConvexPolygon2D("upcomingStepPolygon", "" + i, ReferenceFrame.getWorldFrame(), maxNumberOfContactPointsPerFoot,
                                                                            registry);
         stepPolygonViz.add(footPolygonViz);
         scs2.addYoGraphic(YoGraphicDefinitionFactory.newYoGraphicPolygon2D("Step Polygon" + i, footPolygonViz, ColorDefinitions.Green(), false));
      }

      scs2.setDT(0.01);
      scs2.addRegistry(registry);
      scs2.addYoGraphic(viewer.getSCS2YoGraphics());

      scs2.startSimulationThread();

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

      for (double time = 0; time <= totalDuration; time += scs2.getDT())
      {
         copTrajectoryGenerator.update(time, desiredCoP);
         desiredCoPViz.setBallLoop(desiredCoP);
         scs2.tick();
      }

      ThreadTools.sleepForever();
   }

}
