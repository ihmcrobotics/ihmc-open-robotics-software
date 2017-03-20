package us.ihmc.simulationconstructionset.util.perturbance;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.input.SelectedListener;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.tools.inputDevices.keyboard.Key;
import us.ihmc.tools.inputDevices.keyboard.ModifierKeyInterface;

public class ApplyPerturbanceViaMouseListener implements RobotController, SelectedListener
{
   private final String name = "ApplyPerturbanceViaMouseListener";
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final ArrayList<LaunchedBall> poolOfBalls;
   private int nextBallIndex = 0;

   private final YoFramePoint ballTarget;
   private final YoFrameVector ballTargetVelocity;
   private final DirectedPerturbance directedPerturbance;

   private boolean mouseClicked = false;
   private Point3D intersectionPosition = new Point3D();
   private Point3D cameraPosition = new Point3D();

   private Point3D tempPoint = new Point3D();

   public ApplyPerturbanceViaMouseListener(Robot launchedBallsRobot, YoFramePoint ballTarget, YoFrameVector ballTargetVelocity,
                                           DirectedPerturbance directedPerturbance, int numberOfBallsAvailable)
   {
      this.ballTarget = ballTarget;
      this.ballTargetVelocity = ballTargetVelocity;
      this.directedPerturbance = directedPerturbance;

      poolOfBalls = new ArrayList<LaunchedBall>(numberOfBallsAvailable);

      for (int i = 0; i < numberOfBallsAvailable; i++)
      {
         String name = "ball_" + i;
         LaunchedBall launchedBall = new LaunchedBall(name, launchedBallsRobot, 0.1, 0.01);
         poolOfBalls.add(launchedBall);
      }
   }

   @Override
   public void doControl()
   {
      directedPerturbance.doEveryTick();

      if (mouseClicked)
      {
         handleClick(intersectionPosition, cameraPosition);
         mouseClicked = false;
      }

      // See if any balls are close and if so, apply disturbance:
      for (LaunchedBall ball : poolOfBalls)
      {
         if (ball.isCloseToFinalPosition())
         {
            directedPerturbance.perturb(ball.getDirection());
            ball.bounceAwayAfterCollision();
         }
      }
   }

   public void handleClick(Point3D intersectionPosition, Point3D cameraPosition)
   {
      LaunchedBall nextBall = poolOfBalls.get(nextBallIndex);
      final double ballVelocityMagnitude = directedPerturbance.getBallVelocityMagnitude();
      tempPoint.set(ballTarget.getX(), ballTarget.getY(), ballTarget.getZ());
      Point3D initialPosition = computeInitialPosition(intersectionPosition, ballTarget.getZ());
      final Point3D finalPosition = computeFinalPosition(initialPosition, ballVelocityMagnitude);
      nextBall.launch(initialPosition, finalPosition, directedPerturbance.getBallMass(), ballVelocityMagnitude);
      nextBallIndex++;

      if (nextBallIndex >= poolOfBalls.size())
      {
         nextBallIndex = 0;
      }
   }

   private Point3D computeFinalPosition(Point3D initialPosition, double ballVelocityMagnitude)
   {
      tempPoint.set(ballTarget.getX(), ballTarget.getY(), ballTarget.getZ());
      double distance = initialPosition.distance(tempPoint);
      double estimatedCollisionTime = distance / ballVelocityMagnitude;

      Point3D ret = new Point3D(ballTargetVelocity.getFrameVectorCopy().getVector());
      ret.scale(estimatedCollisionTime);
      ret.add(tempPoint);
      return ret;
   }

   private Point3D computeInitialPosition(Point3D intersectionPosition, double height)
   {
      Point3D ret = new Point3D(intersectionPosition);
      ret.setZ(height);

      return ret;
   }

   @Override
   public void selected(Graphics3DNode graphics3dNode, ModifierKeyInterface modifierKeyHolder, Point3DReadOnly location, Point3DReadOnly cameraLocation,
                        QuaternionReadOnly cameraRotation)
   {

      if (mouseClicked)
         return;

      if (!modifierKeyHolder.isKeyPressed(Key.CTRL))
         return;

      System.out.println("Applying perturbance");

      this.intersectionPosition.set(location);
      this.cameraPosition.set(cameraLocation);

      mouseClicked = true;
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

}
