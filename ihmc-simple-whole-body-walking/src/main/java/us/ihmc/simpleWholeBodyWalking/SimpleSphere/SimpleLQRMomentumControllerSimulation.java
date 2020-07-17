package us.ihmc.simpleWholeBodyWalking.SimpleSphere;

import us.ihmc.commonWalkingControlModules.capturePoint.lqrControl.BasicSphereController;
import us.ihmc.commonWalkingControlModules.capturePoint.lqrControl.LQRSphereController;
import us.ihmc.commonWalkingControlModules.capturePoint.lqrControl.PusherController;
import us.ihmc.commonWalkingControlModules.capturePoint.lqrControl.SphereControllerInterface;
import us.ihmc.commonWalkingControlModules.capturePoint.lqrControl.SphereRobot;
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
   // From previous simulation
   private static final boolean include1 = true;
   private static final boolean include2 = true;

   private final static double desiredHeight = 0.75;
   private final static double controlDT = 0.001;
   private final static double gravity = 9.81;
   
   // from previous simulation footstep planning
   private static final double initialTransferDuration = 1.0;
   private static final double finalTransferDuration = 1.0;
   private static final double stepDuration = 0.7;
   private static final double swingDuration = 0.5;
   private static final double stanceDuration = 0.2;
   private static final double stepLength = 0.75;
   private static final double stepWidth = 0.3;
   private static final int numberOfSteps = 10;

   //I added
   private final static double NextRobotOffset = 1;

   private final SimulationConstructionSet scs;
   
   private final SimpleSphereControllerInterface controller1;
   private final SimpleSphereControllerInterface controller2;
   
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public SimpleLQRMomentumControllerSimulation()
   {
      List<Robot> robots = new ArrayList<>();
      YoGraphicsListRegistry yoGraphicsListRegistry1, yoGraphicsListRegistry2;
      if (include1)
      {
         Vector3D initialPosition1 = new Vector3D(0.0, 0.0, desiredHeight);
         yoGraphicsListRegistry1 = new YoGraphicsListRegistry();
         SimpleSphereRobot sphereRobot1 = new SimpleSphereRobot(0,"SphereRobot1", gravity, controlDT, desiredHeight, yoGraphicsListRegistry1);
         sphereRobot1.initRobot(initialPosition1, stepWidth, new Vector3D());

         robots.add(sphereRobot1.getScsRobot());
         SimpleBipedCoMTrajectoryPlanner dcmPlan1 = new SimpleBipedCoMTrajectoryPlanner(sphereRobot1.getSoleFrames(),
                                                                                       gravity,
                                                                                       sphereRobot1.getDesiredHeight(),
                                                                                       sphereRobot1.getOmega0Provider(),
                                                                                       sphereRobot1.getScsRobot().getRobotsYoVariableRegistry(),
                                                                                       yoGraphicsListRegistry1);
         //This step actually add the footstep plan to the BipedPlanner
         addFootstepandTimingStepsToDCMPlan(dcmPlan1, initialPosition1);
         controller1 = new SimpleBasicSphereController(sphereRobot1, dcmPlan1, yoGraphicsListRegistry1);
         controller1.solveForTrajectory();
         setupGroundContactModel(sphereRobot1.getScsRobot());
      }
      else
      {
         controller1 = null;
         yoGraphicsListRegistry1 = null;
      }

      if (include2)
      {
         Vector3D initialPosition2 = new Vector3D(0.0, NextRobotOffset, desiredHeight);
         yoGraphicsListRegistry2 = new YoGraphicsListRegistry();
         SimpleSphereRobot sphereRobot2 = new SimpleSphereRobot(1, "SphereRobot2", gravity, controlDT, desiredHeight, yoGraphicsListRegistry2);
         sphereRobot2.initRobot(initialPosition2, stepWidth, new Vector3D());

         robots.add(sphereRobot2.getScsRobot());
         SimpleBipedCoMTrajectoryPlanner dcmPlan2 = new SimpleBipedCoMTrajectoryPlanner(sphereRobot2.getSoleFrames(),
                                                                                        gravity,
                                                                                        sphereRobot2.getDesiredHeight(),
                                                                                        sphereRobot2.getOmega0Provider(),
                                                                                        sphereRobot2.getScsRobot().getRobotsYoVariableRegistry(),
                                                                                        yoGraphicsListRegistry2);
         //This step actually add the footstep plan to the BipedPlanner
         addFootstepandTimingStepsToDCMPlan(dcmPlan2, initialPosition2);
         
         controller2 = new SimpleLQRSphereController(sphereRobot2, dcmPlan2, yoGraphicsListRegistry2);
         controller2.solveForTrajectory();
         setupGroundContactModel(sphereRobot2.getScsRobot());
      }
      else
      {
         controller2 = null;
         yoGraphicsListRegistry2 = null;
      }

      Robot[] robotArray = robots.toArray(new Robot[robots.size()]);
      
      //Create Simulatiion Construction Set
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(16000);
      scs = new SimulationConstructionSet(robotArray, parameters);

      // Create Plotter Factory
      SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
      plotterFactory.setShowOnStart(true);
      plotterFactory.setVariableNameToTrack("centerOfMass");
      if (yoGraphicsListRegistry1 != null)
         plotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry1);
      if (yoGraphicsListRegistry2 != null)
         plotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry2);
      plotterFactory.createOverheadPlotter();

      // Define simulation Parameters and run
      scs.setDT(controlDT, 1);
      scs.setCameraPosition(-3.0, -5.0, 2.0);
      scs.setCameraFix(0.0, 0.0, 0.4);
      scs.setCameraTracking(false, true, true, false);
      scs.setCameraDolly(false, true, true, false);
      
      scs.setupGraph("t");
      scs.setSimulateDuration(12);
      scs.setSimulateNoFasterThanRealTime(true);
      
      scs.startOnAThread();
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

   private final static double swingDurationShiftFraction = 0.5;
   private final static double swingSplitFraction = 0.5;
   private final static double transferSplitFraction = 0.5;
   private final static double transferWeightDistribution = 0.5;

   /*
    * Footsteps contain information by the position of a planned footstep. The RobotSide of the footstep is the 
    * side that will swing to touch down at the planned position. The footstep timing begins at the lift_off of
    * this swing foot. The swing takes swingDuration time, then the double support takes transitionDuration where
    * the COP moves from previous foot to new (formerly swinging) foot.
    */
   private static void addFootstepandTimingStepsToDCMPlan(SimpleBipedCoMTrajectoryPlanner dcmPlan, Vector3DReadOnly shift)
   {
      FootstepShiftFractions newShiftFractions = 
            new FootstepShiftFractions(swingDurationShiftFraction,swingSplitFraction,
                                       transferSplitFraction,transferWeightDistribution);
      double contactX = shift.getX();
      double contactY = shift.getY();
      double width = stepWidth;
      double stepStartTime = initialTransferDuration;
      RobotSide currentSide = RobotSide.LEFT;
      dcmPlan.clearStepSequence();
      Quaternion unitQuaternion = new Quaternion(0, 0, 0, 1);
      //Initial Position
      //Footstep InitStep = GenerateFootstep(currentSide, new Point3D(contactX,contactY,0), new Quaternion(0, 0, 0, 1));
      //dcmPlan.addStepToSequence(InitStep, new FootstepTiming(initialTransferDuration, 0), newShiftFractions, 0);
      contactY += stepWidth / 2; //begin planning ctnctY at pos of left foot

      //Main Steps
      for (int i = 0; i < numberOfSteps; i++)
      {
         contactX += stepLength;
         width = -width;
         contactY += width + 0.1;
         currentSide = currentSide.getOppositeSide();

         Footstep newStep = new Footstep();
         newStep.setRobotSide(currentSide);
         newStep.setPose(new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(contactX, contactY, 0), unitQuaternion));
         
         FootstepTiming newTiming = new FootstepTiming(swingDuration, stanceDuration);
         newTiming.setAbsoluteTime(0, stepStartTime);
         
         dcmPlan.addStepToSequence(newStep, newTiming, newShiftFractions, 0);
         stepStartTime += newTiming.getStepTime();
      }
      
      //Final Position
      width = -width;
      contactY += width + 0.1;
      
      Footstep finStep = new Footstep();
      finStep.setRobotSide(currentSide);
      finStep.setPose(new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(contactX, contactY, 0), unitQuaternion));
      
      FootstepTiming finTiming = new FootstepTiming(swingDuration, finalTransferDuration);
      finTiming.setAbsoluteTime(0, stepStartTime);
      
      dcmPlan.addStepToSequence(finStep, finTiming, newShiftFractions, 0);
   }

   private static Footstep GenerateFootstep(RobotSide robotSide, Point3D posePoint, Quaternion poseQuaternion)
   {
      FramePose3D newPose = new FramePose3D(ReferenceFrame.getWorldFrame(), posePoint, poseQuaternion);
      return new Footstep(robotSide, newPose);
   }


   public static void main(String[] args)
   {
      SimpleLQRMomentumControllerSimulation simulation = new SimpleLQRMomentumControllerSimulation();
   }
}
