package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.tools.lists.ArraySorter;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.awt.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class CoMTrajectoryPlannerWalkingVisualizer
{
   private static final double gravity = 9.81;
   //   private static final double nominalHeight = 0.75;
   private static final double nominalHeight = 9.81 / 9;

   private static final double initialTransferDuration = 1.0;
   private static final double finalTransferDuration = 1.0;
   private static final double settlingTime = 1.0;
   private static final double swingDuration = 0.4;
   private static final double transferDuration = 0.2;
   private static final double stepLength = 0.5;
   private static final double stepWidth = 0.25;
   private static final int numberOfSteps = 5;

   private static final double simDt = 7.5e-3;


   AppearanceDefinition defaultColor = new YoAppearanceRGBColor(new Color(0.85f, 0.35f, 0.65f, 1.0f).darker(), 0.0);
   private double simDuration;

   private static final int BUFFER_SIZE = 160000;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SimulationConstructionSet scs;
   private final YoDouble yoTime;
   private final YoDouble timeInPhase;

   private final CoMTrajectoryPlannerInterface planner;

   private List<ContactStateProvider> contactStates;
   private List<ContactStateProvider> contactStatesToUse = new ArrayList<>();

   private final YoFramePoint3D desiredCoMPosition;
   private final YoFrameVector3D desiredCoMVelocity;
   private final YoFrameVector3D desiredCoMAcceleration;
   private final YoFramePoint3D desiredICPPosition;
   private final YoFramePoint3D desiredDCMPosition;
   private final YoFrameVector3D desiredDCMVelocity;
   private final YoFramePoint3D desiredVRPPosition;
   private final YoFrameVector3D desiredGroundReactionForce;
   private final YoFramePoint3D desiredECMPPosition;

   private final BagOfBalls dcmTrajectory;
   private final BagOfBalls comTrajectory;
   private final BagOfBalls vrpTrajectory;

   private final YoDouble omega;

   private final ConvexPolygon2D polygon = new ConvexPolygon2D();
   private final List<YoGraphicPolygon> graphicPolygons = new ArrayList<>();

   public CoMTrajectoryPlannerWalkingVisualizer()
   {
      YoRegistry registry = new YoRegistry("testJacobian");
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      polygon.addVertex(0.1, 0.05);
      polygon.addVertex(0.1, -0.05);
      polygon.addVertex(-0.1, -0.05);
      polygon.addVertex(-0.1, 0.05);
      polygon.update();

      desiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", worldFrame, registry);
      desiredCoMVelocity = new YoFrameVector3D("desiredCoMVelocity", worldFrame, registry);
      desiredCoMAcceleration = new YoFrameVector3D("desiredCoMAcceleration", worldFrame, registry);
      desiredDCMPosition = new YoFramePoint3D("desiredDCMPosition", worldFrame, registry);
      desiredICPPosition = new YoFramePoint3D("desiredICPPosition", worldFrame, registry);
      desiredDCMVelocity = new YoFrameVector3D("desiredDCMVelocity", worldFrame, registry);
      desiredVRPPosition = new YoFramePoint3D("desiredVRPPosition", worldFrame, registry);
      desiredGroundReactionForce = new YoFrameVector3D("desiredGroundReactionForce", worldFrame, registry);
      desiredECMPPosition = new YoFramePoint3D("desiredECMPPosition", worldFrame, registry);

      omega = new YoDouble("omega", registry);
      omega.set(Math.sqrt(gravity / nominalHeight));

      double size = 0.01;
      int balls = 400;
      dcmTrajectory = new BagOfBalls(balls, size, "dcmTrajectory", YoAppearance.Yellow(), registry, graphicsListRegistry);
      comTrajectory = new BagOfBalls(balls, size, "comTrajectory", YoAppearance.Black(), registry, graphicsListRegistry);
      vrpTrajectory = new BagOfBalls(balls, size, "vrpTrajectory", YoAppearance.Green(), registry, graphicsListRegistry);

      yoTime = new YoDouble("timeToCheck", registry);
      timeInPhase = new YoDouble("timeInPhase", registry);

      contactStates = createContacts(registry, graphicsListRegistry);

      planner = new CoMTrajectoryPlanner(gravity, nominalHeight, registry);
//      ((CoMTrajectoryPlanner) planner).setCornerPointViewer(new CornerPointViewer(registry, graphicsListRegistry));


      YoGraphicPosition dcmViz = new YoGraphicPosition("desiredDCM", desiredDCMPosition, 0.02, YoAppearance.Yellow(),
                                                       YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition icpViz = new YoGraphicPosition("desiredDCM", desiredICPPosition, 0.03, YoAppearance.Yellow(),
                                                       YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition comViz = new YoGraphicPosition("desiredCoM", desiredCoMPosition, 0.02, YoAppearance.Black(), YoGraphicPosition.GraphicType.SOLID_BALL);
      YoGraphicPosition comBigViz = new YoGraphicPosition("desiredCoM", desiredCoMPosition, 0.05, YoAppearance.Black(), YoGraphicPosition.GraphicType.SOLID_BALL);
      YoGraphicPosition vrpViz = new YoGraphicPosition("desiredVRP", desiredVRPPosition, 0.02, YoAppearance.Purple(),
                                                       YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);

      YoGraphicVector forceViz = new YoGraphicVector("desiredGRF", desiredECMPPosition, desiredGroundReactionForce, 0.1, YoAppearance.Red());

      graphicsListRegistry.registerYoGraphic("dcmPlanner", forceViz);
      graphicsListRegistry.registerArtifact("dcmPlanner", dcmViz.createArtifact());
      graphicsListRegistry.registerArtifact("dcmPlanner", comViz.createArtifact());
      graphicsListRegistry.registerYoGraphic("dcmPlanner", comBigViz);
      graphicsListRegistry.registerYoGraphic("dcmPlanner", icpViz);
      graphicsListRegistry.registerArtifact("dcmPlanner", vrpViz.createArtifact());

      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters(true, BUFFER_SIZE);
      Robot robot = new Robot("Dummy");

      scs = new SimulationConstructionSet(robot, scsParameters);
      scs.setDT(simDt, 1);
      scs.addYoRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.setPlaybackRealTimeRate(0.75);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      scs.setCameraFix(0.0, 0.0, 0.5);
      scs.setCameraPosition(-0.5, 0.0, 1.0);

      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();

      scs.startOnAThread();
      simulateReverse();
      ThreadTools.sleepForever();
   }

   private List<ContactStateProvider> createContacts(YoRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      List<ContactStateProvider> contacts = new ArrayList<>();

      double contactPosition = 0.0;
      int counter = 0;

      double yValue = 0.5 * stepWidth;
      SettableContactStateProvider initialContactStateProvider = new SettableContactStateProvider();
      initialContactStateProvider.getTimeInterval().setInterval(0.0, initialTransferDuration);
      initialContactStateProvider.setStartECMPPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, 0.0));
      initialContactStateProvider.setEndECMPPosition(new FramePoint3D(worldFrame, contactPosition, yValue, 0.0));
      initialContactStateProvider.setLinearECMPVelocity();

      {
         YoFramePose3D yoFootstepPoseStart = new YoFramePose3D("FootPose" + counter, worldFrame, registry);
         YoFrameConvexPolygon2D yoFootholdStart = new YoFrameConvexPolygon2D("Foothold" + counter, "", worldFrame, 4, registry);
         YoGraphicPolygon footholdVizStart = new YoGraphicPolygon("Foothold" + counter, yoFootholdStart, yoFootstepPoseStart, 1.0, defaultColor);
         graphicsListRegistry.registerYoGraphic("footholds", footholdVizStart);

         counter++;
         YoFramePose3D yoFootstepPoseEnd = new YoFramePose3D("FootPose" + counter, worldFrame, registry);
         YoFrameConvexPolygon2D yoFootholdEnd = new YoFrameConvexPolygon2D("Foothold" + counter, "", worldFrame, 4, registry);
         YoGraphicPolygon footholdVizEnd = new YoGraphicPolygon("Foothold" + counter, yoFootholdEnd, yoFootstepPoseEnd, 1.0, defaultColor);
         graphicsListRegistry.registerYoGraphic("footholds", footholdVizEnd);

         yoFootstepPoseStart.getPosition().set(contactPosition, -yValue, 0.0);
         yoFootstepPoseEnd.getPosition().set(initialContactStateProvider.getECMPEndPosition());
         yoFootholdStart.set(polygon);
         yoFootholdEnd.set(polygon);
//         yoFootholdStart.translate(yoFootstepPoseStart.getPosition().getX(),yoFootstepPoseStart.getPosition().getY());
//         yoFootholdEnd.translate(yoFootstepPoseEnd.getPosition().getX(),yoFootstepPoseEnd.getPosition().getY());

         yoFootholdStart.update();
         yoFootholdEnd.update();

         graphicPolygons.add(footholdVizEnd);
         graphicPolygons.add(footholdVizStart);

      }

      contacts.add(initialContactStateProvider);

      double currentTime = initialTransferDuration;

      for (int i = 0; i < numberOfSteps; i++)
      {
         SettableContactStateProvider swingState = new SettableContactStateProvider();

         // swing
         swingState.setStartECMPPosition(new FramePoint3D(worldFrame, contactPosition, yValue, 0.0));
         swingState.setEndECMPPosition(new FramePoint3D(worldFrame, contactPosition, yValue, 0.0));
         swingState.getTimeInterval().setInterval(currentTime, currentTime + swingDuration);
         swingState.setLinearECMPVelocity();

         contacts.add(swingState);

         currentTime += swingDuration;

         SettableContactStateProvider transferState = new SettableContactStateProvider();

         if (i < numberOfSteps - 1)
         {
            // transfer
            transferState.setStartECMPPosition(new FramePoint3D(worldFrame, contactPosition, yValue, 0.0));
            transferState.setEndECMPPosition(new FramePoint3D(worldFrame, contactPosition + stepLength, -yValue, 0.0));
            transferState.getTimeInterval().setInterval(currentTime, currentTime + transferDuration);
            transferState.setLinearECMPVelocity();

            contacts.add(transferState);

            {
               counter++;
               YoFramePose3D yoFootstepPoseStart = new YoFramePose3D("FootPose" + counter, worldFrame, registry);
               YoFrameConvexPolygon2D yoFootholdStart = new YoFrameConvexPolygon2D("Foothold" + counter, "", worldFrame, 4, registry);
               YoGraphicPolygon footholdVizStart = new YoGraphicPolygon("Foothold" + counter, yoFootholdStart, yoFootstepPoseStart, 1.0, defaultColor);
               graphicsListRegistry.registerYoGraphic("footholds", footholdVizStart);

               counter++;
               YoFramePose3D yoFootstepPoseEnd = new YoFramePose3D("FootPose" + counter, worldFrame, registry);
               YoFrameConvexPolygon2D yoFootholdEnd = new YoFrameConvexPolygon2D("Foothold" + counter, "", worldFrame, 4, registry);
               YoGraphicPolygon footholdVizEnd = new YoGraphicPolygon("Foothold" + counter, yoFootholdEnd, yoFootstepPoseEnd, 1.0, defaultColor);
               graphicsListRegistry.registerYoGraphic("footholds", footholdVizEnd);

               yoFootstepPoseStart.getPosition().set(transferState.getECMPStartPosition());
               yoFootstepPoseEnd.getPosition().set(transferState.getECMPEndPosition());
               yoFootholdStart.set(polygon);
               yoFootholdEnd.set(polygon);
//               yoFootholdStart.translate(yoFootstepPoseStart.getPosition().getX(), yoFootstepPoseStart.getPosition().getY());
//               yoFootholdEnd.translate(yoFootstepPoseEnd.getPosition().getX(), yoFootstepPoseEnd.getPosition().getY());
               yoFootholdStart.update();
               yoFootholdEnd.update();

               graphicPolygons.add(footholdVizEnd);
               graphicPolygons.add(footholdVizStart);
            }

            currentTime += transferDuration;

            contactPosition += stepLength;
            yValue = -yValue;
         }
      }

      SettableContactStateProvider finalTransferState = new SettableContactStateProvider();
      finalTransferState.setStartECMPPosition(new FramePoint3D(worldFrame, contactPosition, yValue, 0.0));
      finalTransferState.setEndECMPPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, 0.0));
      finalTransferState.getTimeInterval().setInterval(currentTime, currentTime + finalTransferDuration);
      finalTransferState.setLinearECMPVelocity();




      {
         counter++;
         YoFramePose3D yoFootstepPoseEnd = new YoFramePose3D("FootPose" + counter, worldFrame, registry);
         YoFrameConvexPolygon2D yoFootholdEnd = new YoFrameConvexPolygon2D("Foothold" + counter, "", worldFrame, 4, registry);
         YoGraphicPolygon footholdVizEnd = new YoGraphicPolygon("Foothold" + counter, yoFootholdEnd, yoFootstepPoseEnd, 1.0, defaultColor);
         graphicsListRegistry.registerYoGraphic("footholds", footholdVizEnd);

         yoFootstepPoseEnd.getPosition().set(contactPosition, -yValue, 0.0);
         yoFootholdEnd.set(polygon);
         yoFootholdEnd.update();

         graphicPolygons.add(footholdVizEnd);
      }

      contacts.add(finalTransferState);

      simDuration = currentTime + settlingTime;

      return contacts;
   }

   private void simulate()
   {
      desiredCoMPosition.setToZero();
      desiredCoMPosition.setZ(nominalHeight);
      desiredCoMVelocity.setToZero();
      planner.setInitialCenterOfMassState(desiredCoMPosition, desiredCoMVelocity);
      planner.solveForTrajectory(contactStates);

      while (simDuration > yoTime.getDoubleValue())
      {
         if (!MathTools.epsilonEquals(contactStates.get(0).getTimeInterval().getStartTime(), 0.0, 1e-5))
            throw new RuntimeException("This is a problem");

         //         planner.solveForTrajectory();
         planner.compute(timeInPhase.getDoubleValue());

         desiredCoMPosition.set(planner.getDesiredCoMPosition());
         desiredCoMVelocity.set(planner.getDesiredCoMVelocity());
         desiredCoMAcceleration.set(planner.getDesiredCoMAcceleration());
         desiredDCMPosition.set(planner.getDesiredDCMPosition());
         desiredDCMVelocity.set(planner.getDesiredDCMVelocity());
         desiredVRPPosition.set(planner.getDesiredVRPPosition());

         desiredGroundReactionForce.set(desiredCoMAcceleration);
         desiredGroundReactionForce.addZ(gravity);

         desiredECMPPosition.set(desiredVRPPosition);
         desiredECMPPosition.subZ(gravity / (omega.getDoubleValue() * omega.getDoubleValue()));

         dcmTrajectory.setBallLoop(desiredDCMPosition);
         comTrajectory.setBallLoop(desiredCoMPosition);
         vrpTrajectory.setBallLoop(desiredECMPPosition);

         yoTime.add(simDt);
         timeInPhase.add(simDt);

         updateContactState();

         scs.tickAndUpdate();
      }
   }

   private void simulateReverse()
   {
      desiredCoMPosition.setToZero();
      desiredCoMPosition.setZ(nominalHeight);
      desiredCoMVelocity.setToZero();
      planner.setInitialCenterOfMassState(desiredCoMPosition, desiredCoMVelocity);
      planner.solveForTrajectory(contactStates);

      yoTime.set(simDuration);

      while (yoTime.getValue() >= 0.0)
      {
         if (!MathTools.epsilonEquals(contactStates.get(0).getTimeInterval().getStartTime(), 0.0, 1e-5))
            throw new RuntimeException("This is a problem");

         //         planner.solveForTrajectory();
         planner.compute(yoTime.getDoubleValue());

         desiredCoMPosition.set(planner.getDesiredCoMPosition());
         desiredCoMVelocity.set(planner.getDesiredCoMVelocity());
         desiredCoMAcceleration.set(planner.getDesiredCoMAcceleration());
         desiredDCMPosition.set(planner.getDesiredDCMPosition());
         desiredDCMVelocity.set(planner.getDesiredDCMVelocity());
         desiredVRPPosition.set(planner.getDesiredVRPPosition());

         desiredGroundReactionForce.set(desiredCoMAcceleration);
         desiredGroundReactionForce.addZ(gravity);

         desiredECMPPosition.set(desiredVRPPosition);
         desiredECMPPosition.subZ(gravity / (omega.getDoubleValue() * omega.getDoubleValue()));

         FramePoint3D comGround = new FramePoint3D(desiredCoMPosition);
         comGround.setZ(0.0);
         desiredICPPosition.set(desiredDCMPosition);
         desiredICPPosition.setZ(0.0);
         dcmTrajectory.setBallLoop(desiredICPPosition);
         comTrajectory.setBallLoop(comGround);
         vrpTrajectory.setBallLoop(desiredECMPPosition);

         yoTime.sub(simDt);

         for (YoGraphicPolygon graphicPolygon : graphicPolygons)
            graphicPolygon.update();
         //         updateContactState();

         scs.tickAndUpdate();
      }
   }

   private void updateContactState()
   {
      if (timeInPhase.getDoubleValue() > contactStates.get(0).getTimeInterval().getEndTime() && contactStates.size() > 1)
      {
         contactStates.remove(0);

         // has to be done to reinitialize from zero
         double timeShift = -contactStates.get(0).getTimeInterval().getStartTime();

         for (ContactStateProvider contactState : contactStates)
            contactState.getTimeInterval().shiftInterval(timeShift);

         planner.setInitialCenterOfMassState(desiredCoMPosition, desiredCoMVelocity);
         planner.solveForTrajectory(contactStates);

         timeInPhase.set(0.0);
      }
   }

   public static void main(String[] args)
   {
      CoMTrajectoryPlannerWalkingVisualizer visualizer = new CoMTrajectoryPlannerWalkingVisualizer();
   }
}
