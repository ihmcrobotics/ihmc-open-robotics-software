package us.ihmc.exampleSimulations.footPathRunners;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

/**
 * Simulation of a FastRunner model using just the foot path and not the mechanism required to implement the footpath.
 * The idea is to generate a footpath trajectory in body coordinates, and an impedance of the leg as it moves through the
 * path. For the planar FastRunner model, the resulting motion is open loop stable. 
 * 
 * @author JerryPratt
 *
 */
public class FootPathRunnerSimulation
{
   public FootPathRunnerSimulation()
   {
      FootPathRunnerRobot robot = new FootPathRunnerRobot();
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      robot.setupReferenceFrameGraphics(yoGraphicsListRegistry);

      double dt = 0.00002;
      FootPathRunnerController controller = new FootPathRunnerController(robot, dt);
      int simulationTicksPerControlTick = 1;
      robot.setController(controller, simulationTicksPerControlTick);

      Pose3D desiredFootPoseInBodyFrame = controller.getDesiredFootPoseInBodyFrame(RobotSide.LEFT);
      robot.setDesiredFootLocation(RobotSide.LEFT, desiredFootPoseInBodyFrame);
      robot.setActualFootLocation(RobotSide.LEFT, desiredFootPoseInBodyFrame);
      
      desiredFootPoseInBodyFrame = controller.getDesiredFootPoseInBodyFrame(RobotSide.RIGHT);
      robot.setDesiredFootLocation(RobotSide.RIGHT, desiredFootPoseInBodyFrame);
      robot.setActualFootLocation(RobotSide.RIGHT, desiredFootPoseInBodyFrame);

      LinearGroundContactModel groundContactModel = new LinearGroundContactModel(robot, robot.getRobotsYoRegistry());
      //      groundContactModel.setXYStiffness(xyStiffness);
      robot.setGroundContactModel(groundContactModel);

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setDT(dt, 100);
      scs.setPlaybackRealTimeRate(0.1);
      scs.setSimulateNoFasterThanRealTime(true);

      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      new FootPathRunnerSimulation();
   }

}
