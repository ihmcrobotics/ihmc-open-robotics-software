package us.ihmc.exampleSimulations.newtonsCradle;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.NullJoint;
import us.ihmc.simulationconstructionset.Robot;

public class GroundAsABoxRobot extends Robot
{
   private final Link baseLink;

   public GroundAsABoxRobot()
   {
      this(false);
   }

   public GroundAsABoxRobot(boolean addWalls)
   {
      this(addWalls, 0xffffffff, 0xffffffff);
   }

   public GroundAsABoxRobot(boolean addWalls, int collisionGroup, int collisionMask)
   {
      this(0.0, addWalls, collisionGroup, collisionMask);
   }

   public GroundAsABoxRobot(double groundAngle, boolean addWalls, int collisionGroup, int collisionMask)
   {
      super("GroundAsABoxRobot");
      NullJoint baseJoint = new NullJoint("base", new Vector3d(), this);

      //    FloatingJoint baseJoint = new FloatingJoint("base", new Vector3d(), this);
      baseLink = new Link("base");
      baseLink.setMassAndRadiiOfGyration(100000000000.0, 100.0, 100.0, 100.0);

      double floorLength = 4.0;
      double floorWidth = 4.0;
      double floorThickness = 0.05;

      Graphics3DObject baseLinkGraphics = new Graphics3DObject();
      baseLinkGraphics.translate(0.0, 0.0, -floorThickness);
      baseLinkGraphics.rotate(groundAngle, Axis.Y);
      baseLinkGraphics.addCube(floorLength, floorWidth, floorThickness, YoAppearance.Green());

      CollisionMeshDescription collisonMeshDescription = new CollisionMeshDescription();
      collisonMeshDescription.translate(0.0, 0.0, -floorThickness);
      collisonMeshDescription.rotate(groundAngle, Axis.Y);
      collisonMeshDescription.addCubeReferencedAtBottomMiddle(floorLength, floorWidth, floorThickness);
      collisonMeshDescription.setIsGround(true);
      collisonMeshDescription.setEstimatedNumberOfContactPoints(400);

      collisonMeshDescription.setCollisionGroup(collisionGroup);
      collisonMeshDescription.setCollisionMask(collisionMask);
//      CollisionShapeDescription<?> groundShapeDescription = collisionShapeFactory.createBox(floorLength / 2.0, floorWidth / 2.0, floorThickness / 2.0);
//      RigidBodyTransform shapeToLinkTransform = new RigidBodyTransform();
//      shapeToLinkTransform.setTranslation(new Vector3d(-0.0, 0.0, 0.0));
//      CollisionShape groundShape = collisionShapeFactory.addShape(baseLink, shapeToLinkTransform, groundShapeDescription, true, 0xFFFFFFFF, 0xFFFFFFFF);
//      groundShape.setIsGround(true);


      if (addWalls)
      {
         double offsetX = -0.75;
         double offsetY = 0.0;
         double xRotation = 0.0;
         double yRotation = Math.PI/8.0;
         addWall(floorLength, floorWidth, floorThickness, baseLinkGraphics, collisonMeshDescription, offsetX, offsetY, xRotation, yRotation);

         offsetX = 0.75;
         offsetY = 0.0;
         xRotation = 0.0;
         yRotation = -Math.PI/8.0;
         addWall(floorLength, floorWidth, floorThickness, baseLinkGraphics, collisonMeshDescription, offsetX, offsetY, xRotation, yRotation);

         offsetX = 0.0;
         offsetY = -0.75;
         xRotation = -Math.PI/8.0;
         yRotation = 0.0;
         addWall(floorLength, floorWidth, floorThickness, baseLinkGraphics, collisonMeshDescription, offsetX, offsetY, xRotation, yRotation);

         offsetX = 0.0;
         offsetY = 0.75;
         xRotation = Math.PI/8.0;
         yRotation = 0.0;
         addWall(floorLength, floorWidth, floorThickness, baseLinkGraphics, collisonMeshDescription, offsetX, offsetY, xRotation, yRotation);
      }

      //    baseJoint.setVelocity(0.0, 0.0, 1.0);

      baseLink.setLinkGraphics(baseLinkGraphics);
      baseLink.addCollisionMesh(collisonMeshDescription);

      baseJoint.setLink(baseLink);
      this.addRootJoint(baseJoint);
      this.addStaticLink(baseLink);
   }

   private void addWall(double floorLength, double floorWidth, double floorThickness,
         Graphics3DObject baseLinkGraphics, CollisionMeshDescription collisonMeshDescription,
         double offsetX, double offsetY, double xRotation, double yRotation)
   {
      baseLinkGraphics.identity();
      baseLinkGraphics.translate(new Vector3d(offsetX, offsetY, -floorThickness));
      Matrix3d rotationMatrixX = new Matrix3d();
      rotationMatrixX.rotX(xRotation);
      baseLinkGraphics.rotate(rotationMatrixX);
      Matrix3d rotationMatrixY = new Matrix3d();
      rotationMatrixY.rotY(yRotation);
      baseLinkGraphics.rotate(rotationMatrixY);
      baseLinkGraphics.addCube(floorLength, floorWidth, floorThickness, YoAppearance.Green());

      collisonMeshDescription.identity();
      collisonMeshDescription.translate(offsetX, offsetY, -floorThickness);
      collisonMeshDescription.rotateEuler(new Vector3d(xRotation, yRotation, 0.0));
      collisonMeshDescription.addCubeReferencedAtBottomMiddle(floorLength, floorWidth, floorThickness);
      collisonMeshDescription.setIsGround(true);

//      groundShapeDescription = collisionShapeFactory.createBox(floorLength / 2.0,  / 2.0, floorThickness / 2.0);
//      shapeToLinkTransform = new RigidBodyTransform();
//      shapeToLinkTransform.setRotationEulerAndZeroTranslation(new Vector3d(xRotation, yRotation, 0.0));
//      shapeToLinkTransform.setTranslation(new Vector3d(offsetX, offsetY, 0.0));
//
//      groundShape = collisionShapeFactory.addShape(baseLink, shapeToLinkTransform, groundShapeDescription, true, 0xFFFFFFFF, 0xFFFFFFFF);
//      groundShape.setIsGround(true);
   }
}
