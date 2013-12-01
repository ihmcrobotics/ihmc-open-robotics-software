package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import static org.junit.Assert.*;

import javax.swing.JPanel;
import javax.swing.JScrollPane;

import org.junit.Test;

import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.plotting.SimulationOverheadPlotter;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class SmartCMPProjectorTest
{
   private static final boolean SHOW_SCS = false;

   @Test
   public void testSimpleProjections()
   {
      YoVariableRegistry registry;
      SimulationConstructionSet scs;

      if (SHOW_SCS)
      {
         scs = new SimulationConstructionSet(new Robot("null"));
         registry = scs.getRootRegistry();
      }
      else
      {
         scs = null;
         registry = new YoVariableRegistry("Test");
      }


      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
      SmartCMPProjector smartCMPProjector = new SmartCMPProjector(registry, dynamicGraphicObjectsListRegistry);
      double cmpEdgeProjectionInside = 0.05;
      smartCMPProjector.setCMPEdgeProjectionInside(cmpEdgeProjectionInside);
      double minICPToCMPProjection = 0.02;
      smartCMPProjector.setMinICPToCMPProjection(minICPToCMPProjection);

      if (SHOW_SCS)
      {
         dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);
         createOverheadPlotter(dynamicGraphicObjectsListRegistry, scs, true);
         scs.startOnAThread();
      }

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      // Test all in same place
      double[][] pointList = new double[][]
      {
         {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}
      };
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);

      FramePoint2d capturePoint = new FramePoint2d(worldFrame, 0.5, 0.5);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, 0.5, 0.5);
      FramePoint2d expectedCMPProjection = new FramePoint2d(worldFrame, 0.5, 0.5);

      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection away from edge.

      desiredCMP = new FramePoint2d(worldFrame, 1.0, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.95, 0.5);
      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);


      // Test simple projection away from edge.
      desiredCMP = new FramePoint2d(worldFrame, 0.97, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.95, 0.5);
      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection away from edge.
      desiredCMP = new FramePoint2d(worldFrame, 1.02, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.95, 0.5);
      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection away from edge.
      desiredCMP = new FramePoint2d(worldFrame, 1.9, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.95, 0.5);
      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection away from edge, but not too far.
      capturePoint = new FramePoint2d(worldFrame, 0.94, 0.5);
      desiredCMP = new FramePoint2d(worldFrame, 0.97, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.96, 0.5);
      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection away from edge, but not too far.
      capturePoint = new FramePoint2d(worldFrame, 0.94, 0.5);
      desiredCMP = new FramePoint2d(worldFrame, 1.5, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.96, 0.5);
      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection away from edge, but not too far.
      capturePoint = new FramePoint2d(worldFrame, 0.94, 0.5);
      desiredCMP = new FramePoint2d(worldFrame, 0.95, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.95, 0.5);
      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection away from edge, but not too far.
      capturePoint = new FramePoint2d(worldFrame, -1.0, 0.5);
      desiredCMP = new FramePoint2d(worldFrame, 0.97, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.95, 0.5);
      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test simple projection away from edge, but not too far.
      capturePoint = new FramePoint2d(worldFrame, -1.0, 0.5);
      desiredCMP = new FramePoint2d(worldFrame, 0.01, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.01, 0.5);
      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test just projecting to the ICP.
      capturePoint = new FramePoint2d(worldFrame, 1.0, 0.5);
      desiredCMP = new FramePoint2d(worldFrame, 1.5, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 1.0, 0.5);
      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test with narrow foot
      pointList = new double[][]
      {
         {0.0, 0.0}, {0.1, 0.0}, {0.1, 1.0}, {0.0, 1.0}
      };
      supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);

      capturePoint = new FramePoint2d(worldFrame, 0.05, 0.5);
      desiredCMP = new FramePoint2d(worldFrame, 0.05, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.05, 0.5);

      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);


      // Test with narrow foot
      capturePoint = new FramePoint2d(worldFrame, 0.01, 0.5);
      desiredCMP = new FramePoint2d(worldFrame, 0.02, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.02, 0.5);

      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);

      // Test with narrow foot
      capturePoint = new FramePoint2d(worldFrame, 0.02, 0.5);
      desiredCMP = new FramePoint2d(worldFrame, 0.01, 0.5);
      expectedCMPProjection = new FramePoint2d(worldFrame, 0.01, 0.5);

      checkOne(scs, smartCMPProjector, supportPolygon, capturePoint, desiredCMP, expectedCMPProjection);




      if (SHOW_SCS)
      {
         ThreadTools.sleepForever();
      }
   }

   private void checkOne(SimulationConstructionSet scs, SmartCMPProjector smartCMPProjector, FrameConvexPolygon2d supportPolygon, FramePoint2d capturePoint,
                         FramePoint2d desiredCMP, FramePoint2d expectedCMPProjection)
   {
      FramePoint2d desiredCMPProjection = new FramePoint2d(desiredCMP);

      smartCMPProjector.projectCMPIntoSupportPolygonIfOutside(capturePoint, supportPolygon, desiredCMPProjection);
      tickAndUpdate(scs);
      assertTrue(expectedCMPProjection.epsilonEquals(desiredCMPProjection, 1e-7));
   }

   private void foo()
   {
   }

   private void tickAndUpdate(SimulationConstructionSet scs)
   {
      if (SHOW_SCS)
      {
         scs.tickAndUpdate();
      }
   }

   private static void createOverheadPlotter(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, SimulationConstructionSet scs,
           boolean showOverheadView)
   {
      SimulationOverheadPlotter plotter = new SimulationOverheadPlotter();
      plotter.setDrawHistory(false);
      plotter.setXVariableToTrack(null);
      plotter.setYVariableToTrack(null);

      scs.attachPlaybackListener(plotter);
      JPanel plotterPanel = plotter.getJPanel();
      String plotterName = "Plotter";
      scs.addExtraJpanel(plotterPanel, plotterName);
      JPanel plotterKeyJPanel = plotter.getJPanelKey();

      JScrollPane scrollPane = new JScrollPane(plotterKeyJPanel);

      scs.addExtraJpanel(scrollPane, "Plotter Legend");

      dynamicGraphicObjectsListRegistry.addArtifactListsToPlotter(plotter.getPlotter());

      if (showOverheadView)
         scs.getStandardSimulationGUI().selectPanel(plotterName);
   }

}
