package us.ihmc.simpleWholeBodyWalking.SimpleSphere;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.BipedTimedStep;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.*;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.tools.ArrayTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.simpleWholeBodyWalking.SimpleBipedCoMTrajectoryPlanner;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;

import javax.swing.*;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/*
 * Simulation of two sphere robots, one with the basic ICP controller, the other with the LQR controller for tracking DCM
 */

public class SimpleLQRMomentumControllerSimulation
{
   private static final boolean include1 = true;
   private static final boolean include2 = true;

   private static boolean visualize = true;

   private final static double desiredHeight = 0.75;
   private final static double controlDT = 0.001;
   private final static double gravity = 9.81;
   private final static double NextRobotOffset = 1;

   private static final double initialTransferDuration = 1.0;
   private static final double finalTransferDuration = 1.0;
   private static final double stepDuration = 0.7;
   private static final double swingDuration = 0.5;
   private static final double stanceDuration = 0.2;
   private static final double stepLength = 0.5;
   private static final double stepWidth = 0.3;
   private static final int numberOfSteps = 10;

   private final static int NumRobots = 1;

   private final SimulationConstructionSet scs;

   private static List<Vector3D> initialPositions = new ArrayList<>();
   private static List<SimpleSphereControllerInterface> sphereControllers = new ArrayList<>();
   private static List<SimpleBipedCoMTrajectoryPlanner> dcmPlans = new ArrayList<>();

   public SimpleLQRMomentumControllerSimulation()
   {
      List<SimpleSphereRobot> sphereRobots = new ArrayList<>();
      List<Robot> robots = new ArrayList<>();
      List<YoGraphicsListRegistry> yoGraphicsListRegistries = new ArrayList<>();
      List<SimplePusherController> pusherControllers = new ArrayList<>();

      for (int i = 0; i < NumRobots; i++)
      {
         yoGraphicsListRegistries.add(new YoGraphicsListRegistry());
         initialPositions.add(new Vector3D(0.0, NextRobotOffset * i, desiredHeight));

         sphereRobots.add(new SimpleSphereRobot(i,
                                                "SphereRobot" + Integer.toString(i + 1),
                                                gravity,
                                                controlDT,
                                                desiredHeight,
                                                yoGraphicsListRegistries.get(i)));
         sphereRobots.get(i).initRobot(initialPositions.get(i), new Vector3D());
         robots.add(sphereRobots.get(i).getScsRobot());

         dcmPlans.add(new SimpleBipedCoMTrajectoryPlanner(createSoleFrames(),
                                                          gravity,
                                                          desiredHeight,
                                                          sphereRobots.get(i).getOmega0Provider(),
                                                          sphereRobots.get(i).getScsRobot().getRobotsYoVariableRegistry(),
                                                          yoGraphicsListRegistries.get(i)));
         //This step actually add the footstep plan to the BipedPlanner
         AddCSPStepsToDCMPlan(dcmPlans.get(i), initialPositions.get(i));
         AddFootstepAndTimingStepsToDCMPlan(dcmPlans.get(i), initialPositions.get(i));
         
         switch (i)
         {
            case 1:
               sphereControllers.add(new SimpleBasicSphereController(sphereRobots.get(i), dcmPlans.get(i), yoGraphicsListRegistries.get(i)));
               break;
            case 0:
               sphereControllers.add(new SimpleLQRSphereController(sphereRobots.get(i), dcmPlans.get(i), yoGraphicsListRegistries.get(i)));
               break;
         }
         sphereControllers.get(i).solveForTrajectory();

         pusherControllers.add(createPusher(sphereRobots.get(i), yoGraphicsListRegistries.get(i)));
         setupGroundContactModel(sphereRobots.get(i).getScsRobot());
      }

      Robot[] robotArray = robots.toArray(new Robot[robots.size()]);

      //Create Simulatiion Construction Set
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(160000);
      scs = new SimulationConstructionSet(robotArray, parameters);

      // Create Plotter Factory
      SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
      plotterFactory.setShowOnStart(true);
      plotterFactory.setVariableNameToTrack("centerOfMass");
      for (int i = 0; i < NumRobots; i++)
      {
         plotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistries.get(i));
      }
      plotterFactory.createOverheadPlotter();

      //Create Push Button
      JButton button = new JButton("PushRobot");
      button.setToolTipText("Click to push the robot as defined in the variables 'pushDirection' and 'pushMagnitude'");
      for (int i = 0; i < NumRobots; i++)
      {
         pusherControllers.get(i).bindSCSPushButton(button);
      }
      scs.addButton(button);

      // Define simulation Parameters and run
      scs.setDT(controlDT, 1);
      scs.setCameraPosition(-3.0, -5.0, 2.0);
      scs.setCameraFix(0.0, 0.0, 0.4);
      scs.setCameraTracking(false, true, true, false);
      scs.setCameraDolly(false, true, true, false);
      
      scs.setupGraph("t");
      
      scs.startOnAThread();
   }

   //Create SoleFrames to feed into the Planner
   private static SideDependentList<MovingReferenceFrame> createSoleFrames()
   {
      SideDependentList<MovingReferenceFrame> soleFrames = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         TranslationMovingReferenceFrame soleFrame = new TranslationMovingReferenceFrame(robotSide + "SoleFrame", worldFrame);
         Vector3D translation = new Vector3D();
         translation.setY(robotSide.negateIfRightSide(stepWidth / 2));
         soleFrame.updateTranslation(translation);

         soleFrames.put(robotSide, soleFrame);
      }

      return soleFrames;
   }

   // Add ability to push the robot during the simulation
   private static SimplePusherController createPusher(SimpleSphereRobot sphereRobot, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      Joint joint = sphereRobot.getScsRobot().getJoint(sphereRobot.getRootJoint().getName());
      SimplePusherController pushController = new SimplePusherController(sphereRobot.getScsRobot(), joint, new Vector3D(), 0.05);

      pushController.setPushForceMagnitude(10.0);
      pushController.setPushDuration(0.25);
      pushController.setPushForceDirection(new Vector3D(0.0, 1.0, 0.0));
      yoGraphicsListRegistry.registerYoGraphic(sphereRobot.getScsRobot().getName() + "Pusher", pushController.getForceVisualizer());

      return pushController;
   }

   // Implement a ground contact model
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

   // Define the contact sequence for the robot as a List of Contact State Providers

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static void AddCSPStepsToDCMPlan(SimpleBipedCoMTrajectoryPlanner dcmPlan, Vector3DReadOnly shift)
   {
      double contactX = shift.getX();
      double contactY = shift.getY();
      double width = stepWidth;
      dcmPlan.clearInputStepSequence();

      //Create starting contact: COP moves from origin to first footstep location
      SettableContactStateProvider initialContactStateProvider = new SettableContactStateProvider();
      initialContactStateProvider.getTimeInterval().setInterval(0.0, initialTransferDuration);
      initialContactStateProvider.setStartCopPosition(new FramePoint3D(worldFrame, contactX, contactY, 0.0));
      contactY += width / 2;
      initialContactStateProvider.setEndCopPosition(new FramePoint3D(worldFrame, contactX, contactY, 0.0));
      initialContactStateProvider.setContactState(ContactState.IN_CONTACT);
      dcmPlan.addStepToSequence(initialContactStateProvider);
      double currentTime = initialTransferDuration;

      //Create steps, COP starts at one step location and moves to the next
      for (int i = 0; i < numberOfSteps; i++)
      {
         SettableContactStateProvider contactStateProvider = new SettableContactStateProvider();

         contactStateProvider.setStartCopPosition(new FramePoint3D(worldFrame, contactX, contactY, 0.0));
         contactX += stepLength;
         width = -width;
         contactY += width;
         contactStateProvider.setEndCopPosition(new FramePoint3D(worldFrame, contactX, contactY, 0.0));
         contactStateProvider.getTimeInterval().setInterval(currentTime, currentTime + stepDuration);
         contactStateProvider.setContactState(ContactState.IN_CONTACT);

         dcmPlan.addStepToSequence(contactStateProvider);

         currentTime += stepDuration;
      }

      SettableContactStateProvider finalStateProvider = new SettableContactStateProvider();
      finalStateProvider.setStartCopPosition(new FramePoint3D(worldFrame, contactX, contactY, 0.0));
      finalStateProvider.setEndCopPosition(new FramePoint3D(worldFrame, contactX, shift.getY(), 0.0));
      finalStateProvider.getTimeInterval().setInterval(currentTime, currentTime + finalTransferDuration);
      finalStateProvider.setContactState(ContactState.IN_CONTACT);

      dcmPlan.addStepToSequence(finalStateProvider);

   }

   private final static double swingDurationShiftFraction = 0.5;
   private final static double swingSplitFraction = 0.5;
   private final static double transferSplitFraction = 0.5;
   private final static double transferWeightDistribution = 0.5;

   private static void AddFootstepAndTimingStepsToDCMPlan(SimpleBipedCoMTrajectoryPlanner dcmPlan, Vector3DReadOnly shift)
   {
      FootstepShiftFractions newShiftFractions = 
            new FootstepShiftFractions(swingDurationShiftFraction,swingSplitFraction,
                                       transferSplitFraction,transferWeightDistribution);
      double contactX = shift.getX();
      double contactY = shift.getY();
      double width = stepWidth;
      RobotSide currentSide = RobotSide.RIGHT;
      dcmPlan.clearConvertedStepSequence();

      //Initial Position
      //Footstep InitStep = GenerateFootstep(currentSide, new Point3D(contactX,contactY,0), new Quaternion(0, 0, 0, 1));
      //dcmPlan.addStepToSequence(InitStep, new FootstepTiming(initialTransferDuration, 0), newShiftFractions, 0);
      contactY += -stepWidth / 2;

      //Main Steps
      for (int i = 0; i < numberOfSteps; i++)
      {
         contactX += stepLength;
         contactY += width;
         width = -width;
         currentSide = currentSide.getOppositeSide();
         Footstep newStep = GenerateFootstep(currentSide, new Point3D(contactX, contactY, 0), new Quaternion(0, 0, 0, 1));
         dcmPlan.addStepToSequence(newStep, new FootstepTiming(swingDuration, stanceDuration), newShiftFractions, 0);
      }

      //Final Position
      contactY += stepWidth / 2;
      Footstep FinStep = GenerateFootstep(currentSide, new Point3D(contactX, contactY, 0), new Quaternion(0, 0, 0, 1));
      dcmPlan.addStepToSequence(FinStep, new FootstepTiming(finalTransferDuration, 0), newShiftFractions, 0);
   }

   private static Footstep GenerateFootstep(RobotSide robotSide, Point3D posePoint, Quaternion poseQuaternion)
   {
      FramePose3D newPose = new FramePose3D(ReferenceFrame.getWorldFrame(), posePoint, poseQuaternion);
      return new Footstep(robotSide, newPose);
   }

   private static void GenerateFootstepDataMessage()
   {
      //The swing trajectory has two default waypoints, the startPoint(set to current foot state at lift-off) and the 
      //final point defined by the location and orientation
      controller_msgs.msg.dds.FootstepDataMessage message = new controller_msgs.msg.dds.FootstepDataMessage();
      message.sequence_id_ = 1; //ID of footstep
      message.robot_side_ = 0; //0-Left 1-Right
      message.location_.set(new us.ihmc.euclid.tuple3D.Point3D(0, 0, 0)); //Point3D that gives position of the footstep (sole frame) in world frame.
      message.orientation_.set(0, 0, 0, 1); // quaterion entered (x,y,z,s) where x,y,z is the vector part, s is scalar
      //message.predicted_contact_points_2d_ //vertices of expected contact polygon btwn foot and world
      //leave empty to use default support polygon
      message.trajectory_type_ = us.ihmc.robotics.trajectories.TrajectoryType.DEFAULT.toByte(); //What swing trajectory should be, recommended is default
      message.swing_height_ = -1.0; //how high the robot should swing its foot, (Setting less than 0.0 will initiate default value

      // Can define two additional waypoints to shape the swing trajectory
      //message.custom_position_waypoints_ = ; 
      // The percentages along the trajectory that the waypoints are. If value is empty, sets default. 
      message.custom_waypoint_proportions_ = new us.ihmc.idl.IDLSequence.Double(1, "type_6");

      /*
       * if TRAJECTORY_TYPE_WAYPOINTS then there will be a list of waypoints for the swing foot to follow
       * if the expected_initial_location and expected_initial_orientation are filled then the
       * swing_trajectory_blend_duration_ defines the length of time from the beginning of the swing phase
       * that the trajectory will be altered to account for the error btwn the actual beginning state and
       * expected
       */
      //message.swing_trajectory_ = ; //list of waypoints for the trajectory
      message.swing_trajectory_blend_duration_ = 0;

      message.swing_duration_ = -1.0; //The swingDuration is the time a foot is not in ground contact during a step (non-positive means default will be used)
      message.transfer_duration_ = -1.0; //The transferDuration is the time spent with the feet in ground contact before a step (non-positive means default will be used)
      message.execution_delay_time_ = 0; //The time to delay this command on the controller side before being executed.
      message.swing_duration_shift_fraction_ = -1.0; //fraction of the swing duration spent shifting the weight from the heel to the toe (remain at toe after)
      message.swing_split_fraction_ = -1.0; //fraction of the shift portion of swing duration spent shifting the weight from the heel of the foot to the ball of the foot.
      message.transfer_split_fraction_ = -1.0; //fraction of the transfer duration spent shifting the weight from the trailing foot to the middle of the stance.
      message.transfer_weight_distribution_ = -1.0; //fraction through transfer that the CoP midpoint is located at (lower means at trailing foot, higher means at leading)
      message.touchdown_duration_ = -1.0; //Time spent after touchdown to transition from heel or toe support to full foot support.
      message.liftoff_duration_ = -1.0; //Time spent in toe or heel support before the step. This duration is part of the transfer duration
   }

   private static void GenerateFootstepDataCommand(controller_msgs.msg.dds.FootstepDataMessage message)
   {
      us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataCommand command = new us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataCommand();
      //command.sequenceId = message.getSequenceId();
      command.setRobotSide(RobotSide.fromByte(message.getRobotSide()));
      command.setTrajectoryType(TrajectoryType.fromByte(message.getTrajectoryType()));
      command.setSwingHeight(message.getSwingHeight());
      command.setSwingTrajectoryBlendDuration(message.getSwingTrajectoryBlendDuration());
      command.setPose(message.getLocation(), message.getOrientation()); //Worldframe set at initiation of Pose within footstep object
      //command.setPredictedContactPoints(message.getPredictedContactPoints2d());

   }

   public static void main(String[] args)
   {
      SimpleLQRMomentumControllerSimulation simulation = new SimpleLQRMomentumControllerSimulation();
   }
}
