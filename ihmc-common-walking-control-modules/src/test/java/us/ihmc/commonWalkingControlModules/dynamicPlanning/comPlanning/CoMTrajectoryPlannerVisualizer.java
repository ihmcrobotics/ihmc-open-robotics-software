package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.ArrayList;
import java.util.List;

public class CoMTrajectoryPlannerVisualizer
{
   private static final double gravity = 9.81;
   //   private static final double nominalHeight = 0.75;
   private static final double nominalHeight = 9.81 / 9;

   private static final double initialTransferDuration = 1.0;
   private static final double finalTransferDuration = 1.0;
   private static final double settlingTime = 1.0;
   private static final double stepDuration = 0.4;
   private static final double flightDuration = 0.1;
   private static final double stepLength = 0.5;
   private static final int numberOfSteps = 5;

   //   private static final double initialVerticalOffsetBound = 0.05;
   //   private static final double finalVerticalOffsetBound = 0.15;
   //private static final double verticalOffset = 0.25;
   private static final double initialVerticalOffsetBound = 0.0;
   private static final double finalVerticalOffsetBound = 0.0;
   private static final double verticalOffset = 0.0;

   private static final boolean includeFlight = true;

   private static final double simDt = 1e-3;

   private double simDuration;

   private static final int BUFFER_SIZE = 160000;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SimulationConstructionSet scs;
   private final YoDouble yoTime;
   private final YoDouble timeInPhase;

   private final CoMTrajectoryPlanner planner;

   private List<ContactStateProvider> contactStates;

   private final YoFramePoint3D desiredCoMPosition;
   private final YoFrameVector3D desiredCoMVelocity;
   private final YoFrameVector3D desiredCoMAcceleration;
   private final YoFramePoint3D desiredDCMPosition;
   private final YoFrameVector3D desiredDCMVelocity;
   private final YoFramePoint3D desiredVRPPosition;
   private final YoFrameVector3D desiredGroundReactionForce;
   private final YoFramePoint3D desiredECMPPosition;

   private final BagOfBalls dcmTrajectory;
   private final BagOfBalls comTrajectory;
   private final BagOfBalls vrpTrajectory;

   private final YoDouble omega;

   public CoMTrajectoryPlannerVisualizer()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      desiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", worldFrame, registry);
      desiredCoMVelocity = new YoFrameVector3D("desiredCoMVelocity", worldFrame, registry);
      desiredCoMAcceleration = new YoFrameVector3D("desiredCoMAcceleration", worldFrame, registry);
      desiredDCMPosition = new YoFramePoint3D("desiredDCMPosition", worldFrame, registry);
      desiredDCMVelocity = new YoFrameVector3D("desiredDCMVelocity", worldFrame, registry);
      desiredVRPPosition = new YoFramePoint3D("desiredVRPPosition", worldFrame, registry);
      desiredGroundReactionForce = new YoFrameVector3D("desiredGroundReactionForce", worldFrame, registry);
      desiredECMPPosition = new YoFramePoint3D("desiredECMPPosition", worldFrame, registry);

      omega = new YoDouble("omega", registry);
      omega.set(Math.sqrt(gravity / nominalHeight));

      dcmTrajectory = new BagOfBalls(50, 0.02, "dcmTrajectory", YoAppearance.Yellow(), registry, graphicsListRegistry);
      comTrajectory = new BagOfBalls(50, 0.02, "comTrajectory", YoAppearance.Black(), registry, graphicsListRegistry);
      vrpTrajectory = new BagOfBalls(50, 0.02, "vrpTrajectory", YoAppearance.Green(), registry, graphicsListRegistry);

      yoTime = new YoDouble("timeToCheck", registry);
      timeInPhase = new YoDouble("timeInPhase", registry);

      contactStates = createContacts();
      planner = new CoMTrajectoryPlanner(omega, gravity, nominalHeight, registry, graphicsListRegistry);

      YoGraphicPosition dcmViz = new YoGraphicPosition("desiredDCM", desiredDCMPosition, 0.02, YoAppearance.Yellow(),
                                                       YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition comViz = new YoGraphicPosition("desiredCoM", desiredCoMPosition, 0.02, YoAppearance.Black(), YoGraphicPosition.GraphicType.SOLID_BALL);
      YoGraphicPosition vrpViz = new YoGraphicPosition("desiredVRP", desiredVRPPosition, 0.02, YoAppearance.Purple(),
                                                       YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);

      YoGraphicVector forceViz = new YoGraphicVector("desiredGRF", desiredECMPPosition, desiredGroundReactionForce, 0.05, YoAppearance.Red());

      graphicsListRegistry.registerYoGraphic("dcmPlanner", forceViz);
      graphicsListRegistry.registerArtifact("dcmPlanner", dcmViz.createArtifact());
      graphicsListRegistry.registerArtifact("dcmPlanner", comViz.createArtifact());
      graphicsListRegistry.registerArtifact("dcmPlanner", vrpViz.createArtifact());

      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters(true, BUFFER_SIZE);
      Robot robot = new Robot("Dummy");

      scs = new SimulationConstructionSet(robot, scsParameters);
      scs.setDT(simDt, 1);
      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.setPlaybackRealTimeRate(0.25);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      scs.setCameraFix(0.0, 0.0, 0.5);
      scs.setCameraPosition(-0.5, 0.0, 1.0);

      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();

      scs.startOnAThread();
      simulate();
      ThreadTools.sleepForever();
   }

   private List<ContactStateProvider> createContacts()
   {
      List<ContactStateProvider> contacts = new ArrayList<>();

      double contactPosition = 0.0;

      SettableContactStateProvider initialContactStateProvider = new SettableContactStateProvider();
      initialContactStateProvider.getTimeInterval().setInterval(0.0, 0.5 * initialTransferDuration);
      initialContactStateProvider.setStartCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, 0.0));
      initialContactStateProvider.setEndCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, 0.0));
      initialContactStateProvider.setContactState(ContactState.IN_CONTACT);

      double currentTime = 0.5 * initialTransferDuration;

      SettableContactStateProvider initialContactStateProvider2 = new SettableContactStateProvider();
      initialContactStateProvider2.getTimeInterval().setInterval(currentTime, currentTime + 0.5 * initialTransferDuration);
      initialContactStateProvider2.setStartCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, -initialVerticalOffsetBound));
      initialContactStateProvider2.setEndCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, -initialVerticalOffsetBound));
      initialContactStateProvider2.setContactState(ContactState.IN_CONTACT);

      contacts.add(initialContactStateProvider);
      contacts.add(initialContactStateProvider2);

      currentTime += 0.5 * initialTransferDuration;

      for (int i = 0; i < numberOfSteps; i++)
      {
         SettableContactStateProvider contactStateProvider = new SettableContactStateProvider();

         contactStateProvider.setStartCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, -verticalOffset));
         if (includeFlight)
            contactStateProvider.setEndCopPosition(contactStateProvider.getCopStartPosition());
         else
            contactStateProvider.setEndCopPosition(new FramePoint3D(worldFrame, contactPosition + stepLength, 0.0, -verticalOffset));
         contactStateProvider.getTimeInterval().setInterval(currentTime, currentTime + stepDuration);
         contactStateProvider.setContactState(ContactState.IN_CONTACT);

         contacts.add(contactStateProvider);

         currentTime += stepDuration;

         if (includeFlight)
         {
            SettableContactStateProvider flightStateProvider = new SettableContactStateProvider();

            flightStateProvider.getTimeInterval().setInterval(currentTime, currentTime + flightDuration);
            flightStateProvider.setContactState(ContactState.FLIGHT);

            contacts.add(flightStateProvider);

            currentTime += flightDuration;
         }
         contactPosition += stepLength;
      }

      SettableContactStateProvider finalStateProvider = new SettableContactStateProvider();
      finalStateProvider.setStartCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, -finalVerticalOffsetBound));
      finalStateProvider.setEndCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, -finalVerticalOffsetBound));
      finalStateProvider.getTimeInterval().setInterval(currentTime, currentTime + 0.5 * finalTransferDuration);
      finalStateProvider.setContactState(ContactState.IN_CONTACT);

      currentTime += 0.5 * finalTransferDuration;

      SettableContactStateProvider finalStateProvider2 = new SettableContactStateProvider();
      finalStateProvider2.setStartCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, 0.0));
      finalStateProvider2.setEndCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, 0.0));
      finalStateProvider2.getTimeInterval().setInterval(currentTime, currentTime + 5.0);
      finalStateProvider2.setContactState(ContactState.IN_CONTACT);

      currentTime += 0.5 * finalTransferDuration;

      SettableContactStateProvider finalStateProvider3 = new SettableContactStateProvider();
      finalStateProvider3.setStartCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, 0.0));
      finalStateProvider3.setEndCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, 0.0));
      finalStateProvider3.getTimeInterval().setInterval(currentTime, currentTime + settlingTime);
      finalStateProvider3.setContactState(ContactState.IN_CONTACT);

      contacts.add(finalStateProvider);
      contacts.add(finalStateProvider2);

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
         vrpTrajectory.setBallLoop(desiredVRPPosition);

         yoTime.add(simDt);
         timeInPhase.add(simDt);

         updateContactState();

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
      CoMTrajectoryPlannerVisualizer visualizer = new CoMTrajectoryPlannerVisualizer();
   }
}
