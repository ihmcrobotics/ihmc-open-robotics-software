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

public class SimpleCoMTrajectoryPlannerVisualizer
{
   private static final double gravity = 9.81;
   private static final double nominalHeight = 9.81 / 9;


   private static final double initialTransferDuration = 1.0;
   private static final double finalTransferDuration = 1.0;
   private static final double settlingTime = 1.0;
   private static final double stepDuration = 0.4;
   private static final double flightDuration = 0.1;
   private static final double stepLength = 0.5;
   private static final int numberOfSteps = 5;

   private static final double finalVerticalOffsetBound = 0.0;
   private static final double verticalOffset = 0.0;

   private static final double simDt = 1e-3;

   private double simDuration;

   private static final int BUFFER_SIZE = 160000;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SimulationConstructionSet scs;
   private final YoDouble yoTime;
   private final YoDouble timeInPhase;

   private final SimpleCoMTrajectoryPlanner planner;

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

   public SimpleCoMTrajectoryPlannerVisualizer()
   {
      YoVariableRegistry registry = new YoVariableRegistry("testJacobian");
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
//      while (contactStates.size() > 1)
//         contactStates.remove(contactStates.size() - 1);

      planner = new SimpleCoMTrajectoryPlanner(omega);
      planner.setNominalCoMHeight(nominalHeight);

      planner.setCornerPointViewer(new CornerPointViewer(registry, graphicsListRegistry));

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
      simulate();
      ThreadTools.sleepForever();
   }

   private List<ContactStateProvider> createContacts()
   {
      List<ContactStateProvider> contacts = new ArrayList<>();

      double contactPosition = 0.0;

      SettableContactStateProvider initialContactStateProvider = new SettableContactStateProvider();
      initialContactStateProvider.getTimeInterval().setInterval(0.0, 0.5);
      initialContactStateProvider.setStartCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, 0.0));
      initialContactStateProvider.setEndCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, 0.0));
      initialContactStateProvider.setContactState(ContactState.IN_CONTACT);

      contacts.add(initialContactStateProvider);

      double currentTime = initialTransferDuration;

      for (int i = 0; i < numberOfSteps; i++)
      {
         SettableContactStateProvider contactStateProvider = new SettableContactStateProvider();

         contactStateProvider.setStartCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, -verticalOffset));
         contactStateProvider.setEndCopPosition(new FramePoint3D(worldFrame, contactPosition + stepLength, 0.0, -verticalOffset));
         contactStateProvider.getTimeInterval().setInterval(currentTime, currentTime + stepDuration);
         contactStateProvider.setContactState(ContactState.IN_CONTACT);

         contacts.add(contactStateProvider);

         currentTime += stepDuration;

         contactPosition += stepLength;
      }

      SettableContactStateProvider finalStateProvider = new SettableContactStateProvider();
      finalStateProvider.setStartCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, -finalVerticalOffsetBound));
      finalStateProvider.setEndCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, -finalVerticalOffsetBound));
      finalStateProvider.getTimeInterval().setInterval(currentTime, currentTime + finalTransferDuration);
      finalStateProvider.setContactState(ContactState.IN_CONTACT);

      contacts.add(finalStateProvider);

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
         int segment = getActiveSegment();
         timeInPhase.set(getTimeInPhase(segment));
         planner.compute(segment, timeInPhase.getDoubleValue());

         desiredCoMPosition.set(planner.getDesiredCoMPosition());
         desiredCoMVelocity.set(planner.getDesiredCoMVelocity());
         desiredCoMAcceleration.set(planner.getDesiredCoMAcceleration());
         desiredDCMPosition.set(planner.getDesiredDCMPosition());
         desiredDCMVelocity.set(planner.getDesiredDCMVelocity());
         desiredVRPPosition.set(planner.getDesiredVRPPosition());
         desiredECMPPosition.set(planner.getDesiredECMPPosition());

         desiredGroundReactionForce.set(desiredCoMAcceleration);
         desiredGroundReactionForce.addZ(gravity);

         dcmTrajectory.setBallLoop(desiredDCMPosition);
         comTrajectory.setBallLoop(desiredCoMPosition);
         vrpTrajectory.setBallLoop(desiredVRPPosition);

         yoTime.add(simDt);

         scs.tickAndUpdate();
      }
   }

   private int getActiveSegment()
   {
      for (int i = 0; i < contactStates.size(); i++)
      {
         if (yoTime.getDoubleValue() >= contactStates.get(i).getTimeInterval().getStartTime())
            return i;
      }

      throw new RuntimeException("Unable to find segment.");
   }

   private double getTimeInPhase(int segment)
   {
      return yoTime.getDoubleValue() - contactStates.get(segment).getTimeInterval().getStartTime();
   }



   public static void main(String[] args)
   {
      SimpleCoMTrajectoryPlannerVisualizer visualizer = new SimpleCoMTrajectoryPlannerVisualizer();
   }
}
