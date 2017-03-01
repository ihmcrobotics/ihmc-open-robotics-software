package us.ihmc.exampleSimulations.trebuchet;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.ExternalForcePoint;

public class TrebuchetController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getName());

   private final BooleanYoVariable isBallAttachedToRope = new BooleanYoVariable("isBallAttachedToRope", registry);

   private final YoFrameVector poleToBallPosition = new YoFrameVector("poleToBallPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector poleToBallVelocity = new YoFrameVector("poleToBallVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final DoubleYoVariable poleTipToBallCenterDistance = new DoubleYoVariable("poleTipToBallCenterDistance", registry);
   private final DoubleYoVariable poleTipToBallCenterVelocity = new DoubleYoVariable("poleTipToBallCenterVelocity", registry);
   private final DoubleYoVariable ropeForce = new DoubleYoVariable("ropeForce", registry);

   private final TrebuchetRobot trebuchetRobot;
   private final ExternalForcePoint ballCenterExternalForcePoint, poleTipExternalForcePoint;

   public TrebuchetController(TrebuchetRobot rob)
   {
      ballCenterExternalForcePoint = rob.getBallCenterExternalForcePoint();
      poleTipExternalForcePoint = rob.getPoleTipExternalForcePoint();

      this.trebuchetRobot = rob;
      isBallAttachedToRope.set(true);
   }

   public static final double BALL_MIN_HEIGHT = TrebuchetRobot.WHEEL_RADIUS + TrebuchetRobot.BASE_HEIGHT / 2.0 + TrebuchetRobot.BALL_RADIUS;
   private final Vector3D forceOnPole = new Vector3D();
   private final Vector3D forceOnBall = new Vector3D();

   public void doControl()
   {
      if (trebuchetRobot.getPivotAngle() >= 0.0)
         isBallAttachedToRope.set(false);

      trebuchetRobot.applyFrictionDamping();

      // Calculate rope length and force:

      YoFramePoint poleTipPosition = poleTipExternalForcePoint.getYoPosition();
      YoFramePoint ballCenterPosition = ballCenterExternalForcePoint.getYoPosition();

      YoFrameVector poleTipVelocity = poleTipExternalForcePoint.getYoVelocity();
      YoFrameVector ballCenterVelocity = ballCenterExternalForcePoint.getYoVelocity();

      poleToBallPosition.sub(ballCenterPosition, poleTipPosition);
      poleToBallVelocity.sub(ballCenterVelocity, poleTipVelocity);

      poleTipToBallCenterDistance.set(poleToBallPosition.length());
      poleTipToBallCenterVelocity.set(poleToBallVelocity.length());

      // Pull from the rope:

      if (isBallAttachedToRope.getBooleanValue() && (poleTipToBallCenterDistance.getDoubleValue() > TrebuchetRobot.ROPE_LENGTH))
      {
         ropeForce.set(
               -5000.0 * (TrebuchetRobot.ROPE_LENGTH - poleTipToBallCenterDistance.getDoubleValue()) + 50.0 * poleTipToBallCenterVelocity.getDoubleValue());
      }
      else
         ropeForce.set(0.0);

      // Apply to external force points:
      poleToBallPosition.get(forceOnPole);
      forceOnPole.normalize();

      forceOnPole.scale(ropeForce.getDoubleValue());
      poleTipExternalForcePoint.setForce(forceOnPole);

      forceOnBall.set(forceOnPole);
      forceOnBall.scale(-1.0);
      ballCenterExternalForcePoint.setForce(forceOnBall);

      // Prevent ball from falling through the base:
      if (isBallAttachedToRope.getBooleanValue() && (ballCenterPosition.getZ() < BALL_MIN_HEIGHT))
      {
         forceOnBall.setZ(forceOnBall.getZ() + 10000.0 * (BALL_MIN_HEIGHT - ballCenterPosition.getZ()) - 1000.0 * ballCenterVelocity.getZ());
         ballCenterExternalForcePoint.setForce(forceOnBall);
      }
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return "TrebuchetController";
   }

   public void initialize()
   {
   }

   public String getDescription()
   {
      return "Dynamics for the forces on the ball and the pole for a Trebuchet Simulation. Not actually a controller, but more dynamics.";
   }
}
