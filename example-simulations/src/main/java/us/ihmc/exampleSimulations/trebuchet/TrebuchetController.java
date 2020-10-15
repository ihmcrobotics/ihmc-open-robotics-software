package us.ihmc.exampleSimulations.trebuchet;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class TrebuchetController implements RobotController
{
   private final YoRegistry registry = new YoRegistry(getName());

   private final YoBoolean isBallAttachedToRope = new YoBoolean("isBallAttachedToRope", registry);

   private final YoFrameVector3D poleToBallPosition = new YoFrameVector3D("poleToBallPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D poleToBallVelocity = new YoFrameVector3D("poleToBallVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final YoDouble poleTipToBallCenterDistance = new YoDouble("poleTipToBallCenterDistance", registry);
   private final YoDouble poleTipToBallCenterVelocity = new YoDouble("poleTipToBallCenterVelocity", registry);
   private final YoDouble ropeForce = new YoDouble("ropeForce", registry);

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

      YoFramePoint3D poleTipPosition = poleTipExternalForcePoint.getYoPosition();
      YoFramePoint3D ballCenterPosition = ballCenterExternalForcePoint.getYoPosition();

      YoFrameVector3D poleTipVelocity = poleTipExternalForcePoint.getYoVelocity();
      YoFrameVector3D ballCenterVelocity = ballCenterExternalForcePoint.getYoVelocity();

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
      forceOnPole.set(poleToBallPosition);
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

   public YoRegistry getYoRegistry()
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
