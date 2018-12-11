package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.utils.TimeIntervalTools;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
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
   private static final double nominalHeight = 0.75;

   private static final double initialTransferDuration = 1.0;
   private static final double stepDuration = 0.4;
   private static final double flightDuration = 0.1;
   private static final double stepLength = 0.5;
   private static final int numberOfSteps = 5;

   private static final boolean includeFlight = false;

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
   private final YoFramePoint3D desiredICPPosition;
   private final YoFrameVector3D desiredICPVelocity;

   private final BagOfBalls dcmTrajectory;
   private final BagOfBalls comTrajectory;

   public CoMTrajectoryPlannerVisualizer()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      desiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", worldFrame, registry);
      desiredICPPosition = new YoFramePoint3D("desiredICPPosition", worldFrame, registry);
      desiredICPVelocity = new YoFrameVector3D("desiredICPVelocity", worldFrame, registry);
      YoDouble omega = new YoDouble("omega", registry);
      omega.set(Math.sqrt(gravity / nominalHeight));

      dcmTrajectory = new BagOfBalls(50, 0.02, "dcmTrajectory", YoAppearance.Yellow(), registry, graphicsListRegistry);
      comTrajectory = new BagOfBalls(50, 0.02, "comTrajectory", YoAppearance.Black(), registry, graphicsListRegistry);

      yoTime = new YoDouble("timeToCheck", registry);
      timeInPhase = new YoDouble("timeInPhase", registry);

      contactStates = createContacts();
      planner = new CoMTrajectoryPlanner(contactStates, omega, gravity, nominalHeight, registry, graphicsListRegistry);

      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters(true, BUFFER_SIZE);
      Robot robot = new Robot("Dummy");

      scs = new SimulationConstructionSet(robot, scsParameters);
      scs.setDT(simDt, 1);
      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.setPlaybackRealTimeRate(0.025);
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
      initialContactStateProvider.getTimeInterval().setInterval(0.0, initialTransferDuration);
      initialContactStateProvider.setCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, 0.0));
      initialContactStateProvider.setContactState(ContactState.IN_CONTACT);

      contacts.add(initialContactStateProvider);

      double currentTime = initialTransferDuration;

      for (int i = 0; i < numberOfSteps; i++)
      {

         SettableContactStateProvider contactStateProvider = new SettableContactStateProvider();

         contactStateProvider.setCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, 0.0));
         contactStateProvider.getTimeInterval().setInterval(currentTime, currentTime + stepDuration);
         contactStateProvider.setContactState(ContactState.IN_CONTACT);

         contacts.add(contactStateProvider);

         currentTime += stepDuration;

         if (includeFlight)
         {
            SettableContactStateProvider flightStateProvider = new SettableContactStateProvider();

            flightStateProvider.getTimeInterval().setInterval(currentTime, currentTime + flightDuration);
            flightStateProvider.setContactState(ContactState.NO_CONTACT);

            contacts.add(flightStateProvider);

            currentTime += flightDuration;
         }
         contactPosition += stepLength;
      }

      SettableContactStateProvider finalStateProvider = new SettableContactStateProvider();
      finalStateProvider.setCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, 0.0));
      finalStateProvider.getTimeInterval().setInterval(currentTime, Double.POSITIVE_INFINITY);
      finalStateProvider.setContactState(ContactState.NO_CONTACT);

      simDuration = currentTime + 5;

      contacts.add(finalStateProvider);

      return contacts;
   }

   private void simulate()
   {
      desiredCoMPosition.setToZero();
      desiredCoMPosition.setZ(nominalHeight);
      planner.setCurrentCoMPosition(desiredCoMPosition);
      planner.solveForTrajectory();

      while (true)
      {
         yoTime.add(simDt);
         timeInPhase.add(simDt);

         updateContactState();

         planner.compute(timeInPhase.getDoubleValue());

         desiredCoMPosition.set(planner.getDesiredCoMPosition());
         desiredICPPosition.set(planner.getDesiredDCMPosition());
         desiredICPVelocity.set(planner.getDesiredDCMVelocity());

         dcmTrajectory.setBallLoop(desiredICPPosition);
         comTrajectory.setBallLoop(desiredCoMPosition);

         scs.tickAndUpdate();

         if (yoTime.getDoubleValue() > simDuration)
            break;
      }
   }

   private void updateContactState()
   {
      int previousNumberOfStates = contactStates.size();
      TimeIntervalTools.removeEndTimesLessThan(yoTime.getDoubleValue(), contactStates);

      if (contactStates.size() < previousNumberOfStates)
      {
         planner.setCurrentCoMPosition(desiredCoMPosition);
         planner.solveForTrajectory();
         timeInPhase.set(0.0);
      }
   }

   public static void main(String[] args)
   {
      CoMTrajectoryPlannerVisualizer visualizer = new CoMTrajectoryPlannerVisualizer();
   }
}
