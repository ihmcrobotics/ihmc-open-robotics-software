package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
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
import us.ihmc.tools.ArrayTools;

import javax.swing.*;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class LQRMomentumControllerSimulation
{
   private static final boolean include1 = true;
   private static final boolean include2 = true;

   private final static double desiredHeight = 0.75;
   private final static double controlDT = 0.001;
   private final static double gravity = 9.81;

   private final SimulationConstructionSet scs;

   private final BasicSphereController controller1;
   private final BasicSphereController controller2;

   private final HashMap<BasicSphereController, Vector3DReadOnly> shift = new HashMap<>();

   public LQRMomentumControllerSimulation()
   {
      List<Robot> robots = new ArrayList<>();
      YoGraphicsListRegistry yoGraphicsListRegistry1, yoGraphicsListRegistry2;
      PusherController pushController1, pushController2;
      if (include1)
      {
         Vector3D initialPosition1 = new Vector3D(0.0, 0.0, desiredHeight);
         yoGraphicsListRegistry1 = new YoGraphicsListRegistry();
         SphereRobot sphereRobot1 = new SphereRobot("SphereRobot1", gravity, controlDT, desiredHeight, yoGraphicsListRegistry1);
         sphereRobot1.initRobot(initialPosition1, new Vector3D());
         controller1 = new BasicSphereController(sphereRobot1, yoGraphicsListRegistry1);
         robots.add(sphereRobot1.getScsRobot());

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

         controller2 = new BasicSphereController(sphereRobot2, yoGraphicsListRegistry2);

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

      plotterFactory.createOverheadPlotter();

      JButton button = new JButton("PushRobot");
      button.setToolTipText("Click to push the robot as defined in the variables 'pushDirection' and 'pushMagnitude'");

      if (pushController1 != null)
         pushController1.bindSCSPushButton(button);
      if (pushController2 != null)
         pushController2.bindSCSPushButton(button);

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
      pushController.setPushForceDirection(new Vector3D(1.0, 0.0, 0.0));
      yoGraphicsListRegistry.registerYoGraphic(sphereRobot.getScsRobot().getName() + "Pusher", pushController.getForceVisualizer());

      return pushController;
   }

   private static void setupGroundContactModel(Robot robot)
   {
      double kXY = 1000.0; //1422.0;
      double bXY = 100.0; //150.6;
      double kZ = 20.0; //50.0;
      double bZ = 50.0; //1000.0;
      GroundContactModel groundContactModel = new LinearGroundContactModel(robot, kXY, bXY, kZ, bZ, robot.getRobotsYoVariableRegistry());

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

         newContacts.add(newContact);
      }

      return newContacts;
   }

   public static void main(String[] args)
   {
      LQRMomentumControllerSimulation simulation = new LQRMomentumControllerSimulation();

      SettableContactStateProvider state1 = new SettableContactStateProvider();
      SettableContactStateProvider state2 = new SettableContactStateProvider();
      SettableContactStateProvider state3 = new SettableContactStateProvider();
      SettableContactStateProvider state4 = new SettableContactStateProvider();
      SettableContactStateProvider state5 = new SettableContactStateProvider();

      FramePoint2D contact1 = new FramePoint2D();
      FramePoint2D contact2 = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.0, 0.15);
      FramePoint2D contact3 = new FramePoint2D(ReferenceFrame.getWorldFrame(), 1.0, -0.15);
      FramePoint2D contact4 = new FramePoint2D(ReferenceFrame.getWorldFrame(), 2.0, 0.15);
      FramePoint2D contact5 = new FramePoint2D(ReferenceFrame.getWorldFrame(), 2.0, 0.0);

      state1.getTimeInterval().setInterval(0.0, 0.5);
      state2.getTimeInterval().setInterval(0.5, 1.0);
      state3.getTimeInterval().setInterval(1.0, 1.5);
      state4.getTimeInterval().setInterval(1.5, 2.0);
      state5.getTimeInterval().setInterval(2.0, 2.5);

      state1.setStartCopPosition(contact1);
      state1.setEndCopPosition(contact2);

      state2.setStartCopPosition(contact2);
      state2.setEndCopPosition(contact3);

      state3.setStartCopPosition(contact3);
      state3.setEndCopPosition(contact4);

      state4.setStartCopPosition(contact4);
      state4.setEndCopPosition(contact5);

      state5.setStartCopPosition(contact5);
      state5.setEndCopPosition(contact5);

      List<ContactStateProvider> contactStateProviders = new ArrayList<>();
      contactStateProviders.add(state1);
      contactStateProviders.add(state2);
      contactStateProviders.add(state3);
      contactStateProviders.add(state4);
      contactStateProviders.add(state5);

      SettableContactStateProvider fakeState = new SettableContactStateProvider();
      fakeState.getTimeInterval().setInterval(0.0, 5.0);
      fakeState.setStartCopPosition(new FramePoint2D());
      fakeState.setEndCopPosition(new FramePoint2D());
      List<ContactStateProvider> fakeProvider = new ArrayList<>();
      fakeProvider.add(fakeState);

      //      simulation.setTrajectories(contactStateProviders);
      simulation.setTrajectories(fakeProvider);
   }
}
