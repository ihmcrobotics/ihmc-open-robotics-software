package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class LIPMWalkerController implements RobotController
{
   private final LIPMWalkerRobot robot;
   private YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoDouble kpKnee = new YoDouble("kpKnee", registry);
   private final YoDouble kdKnee = new YoDouble("kdKnee", registry);

   private final YoDouble q_d_leftKnee = new YoDouble("q_d_leftKnee", registry);
   private final YoDouble q_d_rightKnee = new YoDouble("q_d_rightKnee", registry);

   private final YoDouble comHeight = new YoDouble("comHeight", registry);

   
   private final SideDependentList<YoDouble> desiredKneeLengths = new SideDependentList<YoDouble>(q_d_leftKnee, q_d_rightKnee);

   private final YoDouble desiredHeight = new YoDouble("desiredHeight", registry);

   public LIPMWalkerController(LIPMWalkerRobot robot)
   {
      this.robot = robot;
      initialize();
   }

   @Override
   public void initialize()
   {
      kpKnee.set(5000.0);
      kdKnee.set(500.0);

      q_d_leftKnee.set(1.0);
      q_d_rightKnee.set(0.8);

      desiredHeight.set(0.812);
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public void doControl()
   {
      RobotSide side = RobotSide.LEFT;

      // Support Side:
      double kneeLength = robot.getKneeLength(side);
      double kneeVelocity = robot.getKneeVelocity(side);

      //         double desiredKneeLength = desiredKneeLengths.get(side).getValue();

      Point3DReadOnly centerOfMassPosition = robot.getCenterOfMassPosition();
      Vector3DReadOnly centerOfMassVelocity = robot.getCenterOfMassVelocity();
      double mass = robot.getMass();
      
      comHeight.set(centerOfMassPosition.getZ());
      
      double feedForwardSupportKneeForce = 9.81 * mass * kneeLength / centerOfMassPosition.getZ();
      
      double feedBackKneeForce = kpKnee.getValue() * (desiredHeight.getValue() - comHeight.getValue()) + kdKnee.getValue() * (0.0 - centerOfMassVelocity.getZ());
      robot.setKneeForce(side, feedForwardSupportKneeForce + feedBackKneeForce);

      // Swing Side:
      side = RobotSide.RIGHT;

      kneeLength = robot.getKneeLength(side);
      kneeVelocity = robot.getKneeVelocity(side);

      double desiredKneeLength = desiredKneeLengths.get(side).getValue();

      feedBackKneeForce = kpKnee.getValue() * (desiredKneeLength - kneeLength) + kdKnee.getValue() * (0.0 - kneeVelocity);
      robot.setKneeForce(side, feedBackKneeForce);

   }

}
