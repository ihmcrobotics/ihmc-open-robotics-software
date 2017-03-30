package us.ihmc.manipulation.planning.robotcollisionmodel;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionShapeFactory;

public class CollisionModelBox extends AbstractCollisionModel
{
   private ReferenceFrame referenceFrame;

   private Point3D translationToCenter;
   private double sizeX;
   private double sizeY;
   private double sizeZ;

   private Graphics3DObject graphicObject;

   public CollisionModelBox(SimpleCollisionShapeFactory shapeFactory, ReferenceFrame referenceFrame, Point3D translationToCenter, double sizeX, double sizeY,
                            double sizeZ)
   {
      this.shapeFactory = shapeFactory;

      this.transform = new RigidBodyTransform();
      this.translationToCenter = translationToCenter;

      this.referenceFrame = referenceFrame;

      this.sizeX = sizeX;
      this.sizeY = sizeY;
      this.sizeZ = sizeZ;

      this.graphicObject = new Graphics3DObject();

      updateRighdBodyTransform();

      create();
      updateCollisionShape();
   }

   public CollisionModelBox(SimpleCollisionShapeFactory shapeFactory, RigidBodyTransform rigidbodyTransform, double sizeX, double sizeY, double sizeZ)
   {
      this.shapeFactory = shapeFactory;

      this.transform = rigidbodyTransform;

      this.sizeX = sizeX;
      this.sizeY = sizeY;
      this.sizeZ = sizeZ;

      this.graphicObject = new Graphics3DObject();

      create();
      updateCollisionShape();
   }

   @Override
   public void create()
   {
      collisionShapeDescription = shapeFactory.createBox(sizeX / 2, sizeY / 2, sizeZ / 2);
      collisionShape = shapeFactory.addShape(collisionShapeDescription);
   }

   @Override
   public void updateRighdBodyTransform()
   {
      RigidBodyTransform rigidbodyTransform = referenceFrame.getTransformToWorldFrame();
      rigidbodyTransform.appendTranslation(translationToCenter);

      transform = rigidbodyTransform;
   }

   @Override
   public void updateCollisionShape()
   {
      collisionShape.setTransformToWorld(transform);
   }

   @Override
   public CollisionShape getCollisionShape()
   {
      return collisionShape;
   }

   @Override
   public Graphics3DObject getGraphicObject()
   {
      RigidBodyTransform rigidbodyTransform = transform;
      Point3D translationToCreate = new Point3D(0, 0, -sizeZ / 2);
      rigidbodyTransform.appendTranslation(translationToCreate);

      graphicObject.transform(rigidbodyTransform);

      AppearanceDefinition app;
      app = YoAppearance.Yellow();
      graphicObject.addCube(sizeX, sizeY, sizeZ, app);

      return graphicObject;
   }

}
