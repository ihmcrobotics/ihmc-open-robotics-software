package us.ihmc.simulationconstructionset.util.perturbance;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DScaleInstruction;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;

public class LaunchedBall extends FloatingJoint
{
   private static final long serialVersionUID = -1070304629726153858L;

   private boolean launched = false;

   private final Point3D finalPosition = new Point3D();
   private final Point3D currentPosition = new Point3D();
   private final Vector3D directionVector = new Vector3D();
   private final Vector3D velocityVector = new Vector3D();
   private final double collisionDistance;
   private final double density;

   private final Graphics3DScaleInstruction linkGraphicsScale;

   public LaunchedBall(String name, Robot robot, double collisionDistance, double density)
   {
      super(name, name, new Vector3D(), robot);

      setDynamic(false);

      setPositionAndVelocity(1000.0, 1000.0, -1000.0, 0.0, 0.0, 0.0); // Hide them away at the start.

      Link link = new Link(name);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.setChangeable(true);
      linkGraphicsScale = linkGraphics.scale(1.0);

      linkGraphics.addSphere(0.1);

      link.setLinkGraphics(linkGraphics);
      setLink(link);

      robot.addRootJoint(this);

      this.collisionDistance = collisionDistance;
      this.density = density;
   }

   public boolean isCloseToFinalPosition()
   {
      if (!launched)
         return false;

      this.getPosition(currentPosition);

      return currentPosition.epsilonEquals(finalPosition, collisionDistance);
   }

   public void launch(Point3D initialPosition, Point3D finalPosition, double mass, double velocityMagnitude)
   {
      updateBallSize(mass);
      updatePointsAndVectors(initialPosition, finalPosition, velocityMagnitude);
      setPosition(initialPosition);
      setVelocity(velocityVector);

      this.launched = true;
   }

   public void bounceAwayAfterCollision()
   {
      this.launched = false;
      velocityVector.scale(-1.0);
      final double zVelocityAfterBounce = -10.0;
      velocityVector.setZ(zVelocityAfterBounce);
      setVelocity(velocityVector);
   }

   private void updatePointsAndVectors(Point3D initialPosition, Point3D finalPosition, double velocityMagnitude)
   {
      this.finalPosition.set(finalPosition);

      directionVector.set(finalPosition);
      directionVector.sub(initialPosition);
      directionVector.normalize();

      velocityVector.set(directionVector);
      velocityVector.scale(velocityMagnitude);
   }

   private void updateBallSize(double mass)
   {
      double volume = mass / density;
      double radius = Math.pow(volume / (4.0 / 3.0 * Math.PI), 1.0 / 3.0);

      linkGraphicsScale.setScale(radius);
   }

   public Vector3D getDirection()
   {
      return directionVector;
   }
}
