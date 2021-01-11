package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import us.ihmc.commonWalkingControlModules.capturePoint.lqrControl.*;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.*;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.yoVariables.registry.YoRegistry;

import javax.swing.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class SphereJumpVisualizer
{
   private static final boolean include1 = true;
   private static final boolean include2 = false;
   private static final boolean include3 = true;

   private final static double desiredHeight = 0.75;
   private final static double controlDT = 0.001;
   private final static double gravity = 9.81;

   private final SimulationConstructionSet scs;

   private final SphereControllerInterface controller1;
   private final SphereControllerInterface controller2;
   private final SphereControllerInterface controller3;

   private final HashMap<SphereControllerInterface, Vector3DReadOnly> shift = new HashMap<>();

   public SphereJumpVisualizer()
   {
      List<Robot> robots = new ArrayList<>();
      YoGraphicsListRegistry yoGraphicsListRegistry1, yoGraphicsListRegistry2, yoGraphicsListRegistry3;
      PusherController pushController1, pushController2, pushController3;

      if (include1)
      {
         Vector3D initialPosition1 = new Vector3D(0.0, 0.0, desiredHeight);
         yoGraphicsListRegistry1 = new YoGraphicsListRegistry();
         SphereRobot sphereRobot1 = new SphereRobot("SphereRobot1", gravity, controlDT, desiredHeight, yoGraphicsListRegistry1);
         sphereRobot1.initRobot(initialPosition1, new Vector3D());

         robots.add(sphereRobot1.getScsRobot());

         YoRegistry registry = sphereRobot1.getScsRobot().getRobotsYoRegistry();
         CoMTrajectoryPlanner dcmPlan = new CoMTrajectoryPlanner(gravity, desiredHeight, registry);
         dcmPlan.setCornerPointViewer(new CornerPointViewer(registry, yoGraphicsListRegistry1));

         controller1 = new BasicSphereController(sphereRobot1, dcmPlan, yoGraphicsListRegistry1);

         pushController1 = createPusher(sphereRobot1, yoGraphicsListRegistry1);
         setupGroundContactModel(sphereRobot1.getScsRobot());

         shift.put(controller1, initialPosition1);
      }
      else
      {
         controller1 = null;
         yoGraphicsListRegistry1 = null;
         pushController1 = null;
      }

      if (include2)
      {
         Vector3D initialPosition2 = new Vector3D(0.0, 0.5, desiredHeight);
         yoGraphicsListRegistry2 = new YoGraphicsListRegistry();
         SphereRobot sphereRobot2 = new SphereRobot("SphereRobot2", gravity, controlDT, desiredHeight, yoGraphicsListRegistry2);
         sphereRobot2.initRobot(initialPosition2, new Vector3D());

         robots.add(sphereRobot2.getScsRobot());

         YoRegistry registry = sphereRobot2.getScsRobot().getRobotsYoRegistry();
         CoMTrajectoryPlanner dcmPlan = new CoMTrajectoryPlanner(gravity, desiredHeight, registry);
         dcmPlan.setCornerPointViewer(new CornerPointViewer(registry, yoGraphicsListRegistry2));

         controller2 = new LQRSphereController(sphereRobot2, dcmPlan, yoGraphicsListRegistry2);

         pushController2 = createPusher(sphereRobot2, yoGraphicsListRegistry2);
         setupGroundContactModel(sphereRobot2.getScsRobot());

         shift.put(controller2, initialPosition2);
      }
      else
      {
         controller2 = null;
         yoGraphicsListRegistry2 = null;
         pushController2 = null;
      }

      if (include3)
      {
         Vector3D initialPosition3 = new Vector3D(0.0, 1.0, desiredHeight);
         yoGraphicsListRegistry3 = new YoGraphicsListRegistry();
         SphereRobot sphereRobot2 = new SphereRobot("SphereRobot3", gravity, controlDT, desiredHeight, yoGraphicsListRegistry3);
         sphereRobot2.initRobot(initialPosition3, new Vector3D());

         robots.add(sphereRobot2.getScsRobot());

         YoRegistry registry = sphereRobot2.getScsRobot().getRobotsYoRegistry();
         CoMTrajectoryPlanner dcmPlan = new CoMTrajectoryPlanner(gravity, desiredHeight, registry);
         dcmPlan.setCornerPointViewer(new CornerPointViewer(registry, yoGraphicsListRegistry3));

         controller3 = new LQRJumpSphereController(sphereRobot2, dcmPlan, yoGraphicsListRegistry3);

         pushController3 = createPusher(sphereRobot2, yoGraphicsListRegistry3);
         setupGroundContactModel(sphereRobot2.getScsRobot());

         shift.put(controller3, initialPosition3);
      }
      else
      {
         controller3 = null;
         yoGraphicsListRegistry3 = null;
         pushController3 = null;
      }

      Robot[] robotArray = robots.toArray(new Robot[robots.size()]);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(160000);
      scs = new SimulationConstructionSet(robotArray, parameters);
      SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
      plotterFactory.setShowOnStart(true);
      plotterFactory.setVariableNameToTrack("centerOfMass");

      if (yoGraphicsListRegistry1 != null)
         plotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry1);
      if (yoGraphicsListRegistry2 != null)
         plotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry2);
      if (yoGraphicsListRegistry3 != null)
         plotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry3);

      plotterFactory.createOverheadPlotter();

      JButton button = new JButton("PushRobot");
      button.setToolTipText("Click to push the robot as defined in the variables 'pushDirection' and 'pushMagnitude'");

      if (pushController1 != null)
         pushController1.bindSCSPushButton(button);
      if (pushController2 != null)
         pushController2.bindSCSPushButton(button);
      if (pushController3 != null)
         pushController3.bindSCSPushButton(button);

      scs.addButton(button);

      scs.setDT(controlDT, 1);

      scs.setCameraPosition(-3.0, -5.0, 2.0);
      scs.setCameraFix(0.0, 0.0, 0.4);

      scs.setCameraTracking(false, true, true, false);
      scs.setCameraDolly(false, true, true, false);

      scs.startOnAThread();
   }

   private static PusherController createPusher(SphereRobot sphereRobot, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      Joint joint = sphereRobot.getScsRobot().getJoint(sphereRobot.getRootJoint().getName());
      PusherController pushController = new PusherController(sphereRobot.getScsRobot(), joint, new Vector3D(), 0.05);

      pushController.setPushForceMagnitude(10.0);
      pushController.setPushDuration(0.25);
      pushController.setPushForceDirection(new Vector3D(0.0, 1.0, 0.0));
      yoGraphicsListRegistry.registerYoGraphic(sphereRobot.getScsRobot().getName() + "Pusher", pushController.getForceVisualizer());

      return pushController;
   }

   private static void setupGroundContactModel(Robot robot)
   {
      double kXY = 1000.0; //1422.0;
      double bXY = 100.0; //150.6;
      double kZ = 20.0; //50.0;
      double bZ = 50.0; //1000.0;
      GroundContactModel groundContactModel = new LinearGroundContactModel(robot, kXY, bXY, kZ, bZ, robot.getRobotsYoRegistry());

      GroundProfile3D groundProfile = new FlatGroundProfile();
      groundContactModel.setGroundProfile3D(groundProfile);
      robot.setGroundContactModel(groundContactModel);
   }

   public void setTrajectories(List<? extends ContactStateProvider> contacts)
   {
      if (controller1 != null)
      {
         controller1.solveForTrajectory(copyContactsWithShift(contacts, shift.get(controller1)));
      }
      if (controller2 != null)
      {
         controller2.solveForTrajectory(copyContactsWithShift(contacts, shift.get(controller2)));
      }
      if (controller3 != null)
      {
         controller3.solveForTrajectory(copyContactsWithShift(contacts, shift.get(controller3)));
      }
   }

   private static List<? extends ContactStateProvider> copyContactsWithShift(List<? extends ContactStateProvider> contacts, Vector3DReadOnly shift)
   {
      List<SettableContactStateProvider> newContacts = new ArrayList<>();

      for (ContactStateProvider contact : contacts)
      {
         SettableContactStateProvider newContact = new SettableContactStateProvider();
         newContact.getTimeInterval().set(contact.getTimeInterval());
         FramePoint2D start = new FramePoint2D(contact.getCopStartPosition());
         FramePoint2D end = new FramePoint2D(contact.getCopEndPosition());

         start.add(shift.getX(), shift.getY());
         end.add(shift.getX(), shift.getY());

         newContact.setStartCopPosition(start);
         newContact.setEndCopPosition(end);
         newContact.setContactState(contact.getContactState());

         newContacts.add(newContact);
      }

      return newContacts;
   }

   private static final double initialTransferDuration = 1.0;
   private static final double finalTransferDuration = 1.0;
   private static final double settlingTime = 1.0;
   private static final double stepDuration = 0.6;
   private static final double flightDuration = 0.3;
   private static final double stepLength = 0.8;
   private static final double stepWidth = 0.15;
   private static final int numberOfSteps = 10;
   private static final int numberOfRunningSteps = 4;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static List<ContactStateProvider> createContacts()
   {
      List<ContactStateProvider> contacts = new ArrayList<>();

      double contactPosition = 0.0;

      double width = stepWidth;
      us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider initialContactStateProvider = new us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider();
      initialContactStateProvider.getTimeInterval().setInterval(0.0, initialTransferDuration);
      initialContactStateProvider.setStartCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, 0.0));
      initialContactStateProvider.setEndCopPosition(new FramePoint3D(worldFrame, contactPosition, width, 0.0));
      initialContactStateProvider.setContactState(ContactState.IN_CONTACT);

      contacts.add(initialContactStateProvider);

      double currentTime = initialTransferDuration;

      for (int i = 0; i < numberOfSteps; i++)
      {
         us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider contactStateProvider = new us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider();

         contactStateProvider.setStartCopPosition(new FramePoint3D(worldFrame, contactPosition, width, 0.0));
         contactStateProvider.setEndCopPosition(new FramePoint3D(worldFrame, contactPosition + stepLength, -width, 0.0));
         contactStateProvider.getTimeInterval().setInterval(currentTime, currentTime + stepDuration);
         contactStateProvider.setContactState(ContactState.IN_CONTACT);

         contacts.add(contactStateProvider);

         width = -width;
         currentTime += stepDuration;

         contactPosition += stepLength;
      }

      us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider finalStateProvider = new us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider();
      finalStateProvider.setStartCopPosition(new FramePoint3D(worldFrame, contactPosition, width, 0.0));
      finalStateProvider.setEndCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, 0.0));
      finalStateProvider.getTimeInterval().setInterval(currentTime, currentTime + finalTransferDuration);
      finalStateProvider.setContactState(ContactState.IN_CONTACT);

      contacts.add(finalStateProvider);

      return contacts;
   }

   private static List<ContactStateProvider> createRunningContacts()
   {
      List<ContactStateProvider> contacts = new ArrayList<>();

      double contactPosition = 0.0;

      double width = stepWidth;
      SettableContactStateProvider initialContactStateProvider = new SettableContactStateProvider();
      initialContactStateProvider.getTimeInterval().setInterval(0.0, initialTransferDuration);
      initialContactStateProvider.setStartCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, 0.0));
      initialContactStateProvider.setEndCopPosition(new FramePoint3D(worldFrame, contactPosition, width, 0.0));
      initialContactStateProvider.setContactState(ContactState.IN_CONTACT);

      contacts.add(initialContactStateProvider);

      double currentTime = initialTransferDuration;

      for (int i = 0; i < numberOfRunningSteps; i++)
      {
         SettableContactStateProvider contactStateProvider = new SettableContactStateProvider();

         contactStateProvider.setStartCopPosition(new FramePoint3D(worldFrame, contactPosition, width, 0.0));
         contactStateProvider.setEndCopPosition(new FramePoint3D(worldFrame, contactPosition + stepLength, -width, 0.0));
         contactStateProvider.getTimeInterval().setInterval(currentTime, currentTime + stepDuration);
         contactStateProvider.setContactState(ContactState.IN_CONTACT);

         contacts.add(contactStateProvider);

         currentTime += stepDuration;


         SettableContactStateProvider flightStateProvider = new SettableContactStateProvider();

         flightStateProvider.getTimeInterval().setInterval(currentTime, currentTime + flightDuration);
         flightStateProvider.setContactState(ContactState.FLIGHT);

         contacts.add(flightStateProvider);

         width = -width;
         currentTime += flightDuration;

         contactPosition += stepLength;
      }

      SettableContactStateProvider finalStateProvider = new SettableContactStateProvider();
      finalStateProvider.setStartCopPosition(new FramePoint3D(worldFrame, contactPosition, width, 0.0));
      finalStateProvider.setEndCopPosition(new FramePoint3D(worldFrame, contactPosition, 0.0, 0.0));
      finalStateProvider.getTimeInterval().setInterval(currentTime, currentTime + finalTransferDuration);
      finalStateProvider.setContactState(ContactState.IN_CONTACT);

      contacts.add(finalStateProvider);

      return contacts;
   }


   public static void main(String[] args)
   {
      SphereJumpVisualizer simulation = new SphereJumpVisualizer();

      SettableContactStateProvider fakeState = new SettableContactStateProvider();
      fakeState.getTimeInterval().setInterval(0.0, 5.0);
      fakeState.setStartCopPosition(new FramePoint2D());
      fakeState.setEndCopPosition(new FramePoint2D());
      List<ContactStateProvider> fakeProvider = new ArrayList<>();
      fakeProvider.add(fakeState);

      simulation.setTrajectories(createRunningContacts());
      //      simulation.setTrajectories(fakeProvider);
   }
}
