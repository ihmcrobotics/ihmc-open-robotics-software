package us.ihmc.simulationconstructionset.util.perturbance;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

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
   private Point3d intersectionPosition = new Point3d();
   private Point3d cameraPosition = new Point3d();
   
   private Point3d tempPoint = new Point3d();

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

   public void handleClick(Point3d intersectionPosition, Point3d cameraPosition)
   {
      LaunchedBall nextBall = poolOfBalls.get(nextBallIndex);
      final double ballVelocityMagnitude = directedPerturbance.getBallVelocityMagnitude();
      tempPoint.set(ballTarget.getX(), ballTarget.getY(), ballTarget.getZ());
      Point3d initialPosition = computeInitialPosition(intersectionPosition, ballTarget.getZ());
      final Point3d finalPosition = computeFinalPosition(initialPosition, ballVelocityMagnitude);
      nextBall.launch(initialPosition, finalPosition, directedPerturbance.getBallMass(), ballVelocityMagnitude);
      nextBallIndex++;

      if (nextBallIndex >= poolOfBalls.size())
      {
         nextBallIndex = 0;
      }
   }

   private Point3d computeFinalPosition(Point3d initialPosition, double ballVelocityMagnitude)
   {
      tempPoint.set(ballTarget.getX(), ballTarget.getY(), ballTarget.getZ());
      double distance = initialPosition.distance(tempPoint);
      double estimatedCollisionTime = distance / ballVelocityMagnitude;
      
      Point3d ret = new Point3d(ballTargetVelocity.getFrameVectorCopy().getVector());
      ret.scale(estimatedCollisionTime);
      ret.add(tempPoint);
      return ret;
   }

   private Point3d computeInitialPosition(Point3d intersectionPosition, double height)
   {
      Point3d ret = new Point3d(intersectionPosition);
      ret.setZ(height);

      return ret;
   }

   @Override
   public void selected(Graphics3DNode graphics3dNode, ModifierKeyInterface modifierKeyHolder, Point3d location, Point3d cameraLocation, Quat4d cameraRotation)
   {
      
      if (mouseClicked)
         return;

      
      if(!modifierKeyHolder.isKeyPressed(Key.CTRL))
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
