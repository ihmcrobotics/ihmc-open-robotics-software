package us.ihmc.exampleSimulations.simpleArm;


import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;

/**
 * Simple example of using the inverse dynamics structure to control a robotic arm.
 */
public class SimpleArmController implements RobotController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry;
   private final SimpleArmRobot robot;

   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   private final TwistCalculator twistCalculator;

   private final YoFramePoint targetPosition;
   private final DoubleYoVariable kp;
   private final FramePoint endEffectorPosition = new FramePoint();
   private final FrameVector errorVector = new FrameVector();
   private final Wrench endEffectorWrench = new Wrench();

   public SimpleArmController(SimpleArmRobot simpleArmRobot, String name)
   {
      robot = simpleArmRobot;
      registry = new YoVariableRegistry(name);
      targetPosition = new YoFramePoint("targetPosition", worldFrame, registry);
      kp = new DoubleYoVariable("kpTaskspace", registry);
      kp.set(0.5);

      targetPosition.setX(0.3);
      targetPosition.setY(0.3);
      targetPosition.setZ(0.7);

      twistCalculator = new TwistCalculator(worldFrame, robot.getEndEffectorBody());
      inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, -robot.getGravityZ());
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

   @Override
   public String getDescription()
   {
      return registry.getName();
   }

   @Override
   public void doControl()
   {
      robot.updateInverseDynamicsStructureFromSimulation();

      // --- compute force to pull the end effector towards the target position
      ReferenceFrame endEffectorFrame = robot.getEndEffectorFrame();
      RigidBody endEffectorBody = robot.getEndEffectorBody();
      ReferenceFrame endEffectorBodyFrame = endEffectorBody.getBodyFixedFrame();

      endEffectorPosition.setToZero(endEffectorFrame);
      endEffectorPosition.changeFrame(worldFrame);
      errorVector.setIncludingFrame(targetPosition.getFrameTuple());
      errorVector.sub(endEffectorPosition);
      errorVector.changeFrame(endEffectorFrame);

      endEffectorWrench.setToZero(endEffectorBodyFrame, endEffectorFrame);
      endEffectorWrench.setLinearPart(errorVector);
      endEffectorWrench.changeFrame(endEffectorBodyFrame);

      endEffectorWrench.scale(-kp.getDoubleValue());
      inverseDynamicsCalculator.setExternalWrench(endEffectorBody, endEffectorWrench);
      // ---

      twistCalculator.compute();
      inverseDynamicsCalculator.compute();

      robot.updateSimulationFromInverseDynamicsTorques();
   }

}
