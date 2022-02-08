package us.ihmc.exampleSimulations.lipmWalkerTobi;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.exampleSimulations.lipmWalker.LIPMWalkerRobot;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class LIPMWalkerControllerTobi implements RobotController
{
   private final LIPMWalkerRobot robot;
   private YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoDouble kpKnee = new YoDouble("kpKnee", registry);
   private final YoDouble kdKnee = new YoDouble("kdKnee", registry);
   private final YoDouble kpHip = new YoDouble("kpHip", registry);
   private final YoDouble kdHip = new YoDouble("kdHip", registry);

   private final YoDouble bodyAngle = new YoDouble("bodyAngle", registry);

   private final YoDouble comHeight = new YoDouble("comHeight", registry);
   private final YoDouble orbitalEnergy = new YoDouble("orbitalEnergy", registry);
   private final YoDouble stepTime = new YoDouble("stepTime", registry);
   private final YoDouble requiredStepLength = new YoDouble("requiredStepLength", registry);
   private final YoDouble progress = new YoDouble("progress", registry);
   private final YoDouble stepProgress = new YoDouble("stepProgress", registry);
   private RobotSide supportSide;

   
   private final YoDouble desiredHeight = new YoDouble("desiredHeight", registry);

   public LIPMWalkerControllerTobi(LIPMWalkerRobot robot)
   {
      this.robot = robot;
      initialize();
   }

   @Override
   public void initialize()
   {
      kpKnee.set(1000.0);
      kdKnee.set(100.0);
      kpHip.set(10.0);
      kdHip.set(1.0);

      desiredHeight.set(0.812);
      stepTime.set(1000.0);
      supportSide = RobotSide.RIGHT;
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public void doControl()
   {
      RobotSide swingSide = supportSide.getOppositeSide();

      // Statistics:
      Point3DReadOnly centerOfMassPosition = robot.getCenterOfMassPosition();
      Vector3DReadOnly centerOfMassVelocity = robot.getCenterOfMassVelocity();
      double mass = robot.getMass();
      comHeight.set(centerOfMassPosition.getZ());
      orbitalEnergy.set(0.5f * centerOfMassVelocity.getX() * centerOfMassVelocity.getX() - 9.81f / 2.0f / centerOfMassPosition.getZ() * centerOfMassPosition.getX() * centerOfMassPosition.getX());
      bodyAngle.set(robot.getBodyPitchAngle());
      progress.set(progress.getValue() + 1.0);
      double numSteps = progress.getValue() / stepTime.getValue();
      double stepProgress = (progress.getValue() % stepTime.getValue()) / stepTime.getValue();
      this.stepProgress.set(stepProgress);
      double sqrtFact = centerOfMassPosition.getZ() / 9.81 * (centerOfMassVelocity.getX() * centerOfMassVelocity.getX() - 0.2); // Desired ORB ENERGY
      if (sqrtFact > 0)
         requiredStepLength.set(Math.sqrt(sqrtFact));
//      stepProgress = (-robot.getFootPosition(supportSide).getX() + centerOfMassPosition.getX()) / requiredStepLength.getValue();


      // Swing Side:
      System.out.println(robot.getFootPosition(swingSide));
      double parabola = 4 * (stepProgress - .5) * (stepProgress - .5);
      double desiredKneeLength = 0.7 + 0.3 * parabola;
      double desiredFootSpeed = 0.7 + 0.3 * 8 * (stepProgress - 0.5);
      desiredKneeLength = robot.getKneeLength(supportSide) * desiredKneeLength;
      desiredFootSpeed = robot.getKneeLength(supportSide) * desiredFootSpeed + robot.getKneeVelocity(supportSide) * desiredKneeLength;
      robot.setKneeForce(swingSide, kpKnee.getValue() * (desiredKneeLength - robot.getKneeLength(swingSide)) + kdKnee.getValue() * (desiredFootSpeed - robot.getKneeVelocity(swingSide)));
      double desiredHipAngle = -robot.getHipAngle(swingSide.getOppositeSide());
      double hipTorque = kpHip.getValue() * (desiredHipAngle - robot.getHipAngle(swingSide)) + kdHip.getValue() * (0.0 - robot.getHipVelocity(swingSide));
      robot.setHipTorque(swingSide, hipTorque);

      // Support Side:
      double kneeLength = robot.getKneeLength(supportSide);
      double kneeVelocity = robot.getKneeVelocity(supportSide);
      //         double desiredKneeLength = desiredKneeLengths.get(side).getValue();
//      double feedForwardSupportKneeForce = 9.81 * mass * kneeLength / centerOfMassPosition.getZ();
      double feedBackKneeForce = kpKnee.getValue() * (desiredHeight.getValue() - comHeight.getValue()) + kdKnee.getValue() * (0.0 - centerOfMassVelocity.getZ());
      robot.setKneeForce(supportSide, feedBackKneeForce);

      hipTorque = hipTorque - kpHip.getValue() * robot.getBodyPitchAngle() - kdHip.getValue() * robot.getBodyPitchAngularVelocity();
      robot.setHipTorque(supportSide, -hipTorque);

      // Switch support
      if (robot.getFootPosition(swingSide).getZ() < 0.01)
      {
         supportSide = supportSide.getOppositeSide();
         progress.set(200);
      }
   }
}
