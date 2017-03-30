package us.ihmc.manipulation.planning.robotcollisionmodel;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionShapeFactory;

public class CollisionModelCapsule extends AbstractCollisionModel
{
   public OneDoFJoint currentJoint;
   private OneDoFJoint nextJoint;

   private double height;
   private double radius;
   private Graphics3DObject graphicObject;

   private Point3D currentLocation = new Point3D();
   private Point3D nextLocation = new Point3D();

   public CollisionModelCapsule(SimpleCollisionShapeFactory shapeFactory, OneDoFJoint currentJoint, OneDoFJoint nextJoint, double radius)
   {
      this.currentJoint = currentJoint;
      this.nextJoint = nextJoint;

      this.shapeFactory = shapeFactory;

      this.transform = new RigidBodyTransform();

      this.graphicObject = new Graphics3DObject();

      updateRighdBodyTransform();

      this.radius = radius;
      this.height = currentLocation.distance(nextLocation) + radius * 2;

      create();
      updateCollisionShape();
   }

   public void create()
   {
      collisionShapeDescription = shapeFactory.createCapsule(radius, height);
      collisionShape = shapeFactory.addShape(collisionShapeDescription);
   }

   public void updateRighdBodyTransform()
   {
      RigidBodyTransform currentTransform = currentJoint.getFrameAfterJoint().getTransformToWorldFrame();
      RigidBodyTransform nextTransform = nextJoint.getFrameAfterJoint().getTransformToWorldFrame();
      currentTransform.getTranslation(currentLocation);
      nextTransform.getTranslation(nextLocation);

      Point3D centerLocation = new Point3D((currentLocation.getX() + nextLocation.getX()) / 2, (currentLocation.getY() + nextLocation.getY()) / 2,
                                           (currentLocation.getZ() + nextLocation.getZ()) / 2);

      Vector3D centerXAxis = new Vector3D();
      Vector3D centerYAxis = new Vector3D();
      Vector3D centerZAxis = new Vector3D();
      Vector3D rotationAxis = currentJoint.getJointAxis().getVector();
      if (rotationAxis.getX() == 1)
      {
         currentTransform.getRotationMatrix().getColumn(0, centerXAxis);
         PrintTools.info("getX centerXAxis " + centerXAxis.getX() + " " + centerXAxis.getY() + " " + centerXAxis.getZ());
      }
      if (rotationAxis.getY() == 1)
      {
         currentTransform.getRotationMatrix().getColumn(1, centerXAxis);
         PrintTools.info("getY centerXAxis " + centerXAxis.getX() + " " + centerXAxis.getY() + " " + centerXAxis.getZ());
      }
      if (rotationAxis.getZ() == 1)
      {
         currentTransform.getRotationMatrix().getColumn(2, centerXAxis);
         PrintTools.info("getZ centerXAxis " + centerXAxis.getX() + " " + centerXAxis.getY() + " " + centerXAxis.getZ());
      }

      double norm = currentLocation.distance(nextLocation);
      centerZAxis.setX((nextLocation.getX() - currentLocation.getX()) / norm);
      centerZAxis.setY((nextLocation.getY() - currentLocation.getY()) / norm);
      centerZAxis.setZ((nextLocation.getZ() - currentLocation.getZ()) / norm);

      centerYAxis.cross(centerZAxis, centerXAxis);
      RotationMatrix centerRotationMatrix = new RotationMatrix();

      centerRotationMatrix.setAndNormalize(centerXAxis.getX(), centerYAxis.getX(), centerZAxis.getX(), centerXAxis.getY(), centerYAxis.getY(),
                                           centerZAxis.getY(), centerXAxis.getZ(), centerYAxis.getZ(), centerZAxis.getZ());

      transform.setTranslation(centerLocation);
      transform.setRotation(centerRotationMatrix);
   }

   public double getRadius()
   {
      return radius;
   }

   public double getHeight()
   {
      return height;
   }

   public CollisionShape getCollisionShape()
   {
      return collisionShape;
   }

   public Graphics3DObject getGraphicObject()
   {
      graphicObject.transform(transform);
      graphicObject.addCapsule(radius, height, YoAppearance.Yellow());

      return graphicObject;
   }

   @Override
   public void updateCollisionShape()
   {
      collisionShape.setTransformToWorld(transform);
   }
}
