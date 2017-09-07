package us.ihmc.exampleSimulations.newtonsCradle;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.RigidJoint;
import us.ihmc.simulationconstructionset.Robot;

public class GroundAsABoxRobot
{
   //   private final Link baseLink;

   private int estimatedNumberOfContactPoints = 40;
   private double groundAngle = 0.0;
   private boolean addWalls = false;
   private int collisionGroup = 0xffffffff;
   private int collisionMask = 0xffffffff;

   private double groundLength = 4.0;
   private double groundWidth = 4.0;
   private double groundThickness = 0.05;

   public GroundAsABoxRobot()
   {
   }

   public void setGroundLength(double groundLength)
   {
      this.groundLength = groundLength;
   }

   public void setGroundWidth(double groundWidth)
   {
      this.groundWidth = groundWidth;
   }

   public void setGroundThickness(double groundThickness)
   {
      this.groundThickness = groundThickness;
   }

   public void setEstimatedNumberOfContactPoints(int estimatedNumberOfContactPoints)
   {
      this.estimatedNumberOfContactPoints = estimatedNumberOfContactPoints;
   }

   public void setGroundAngle(double groundAngle)
   {
      this.groundAngle = groundAngle;
   }

   public void setAddWalls(boolean addWalls)
   {
      this.addWalls = addWalls;
   }

   public void setCollisionGroup(int collisionGroup)
   {
      this.collisionGroup = collisionGroup;
   }

   public void setCollisionMask(int collisionMask)
   {
      this.collisionMask = collisionMask;
   }

   public void setFloorLength(double floorLength)
   {
      this.groundLength = floorLength;
   }

   public void setFloorWidth(double floorWidth)
   {
      this.groundWidth = floorWidth;
   }

   public void setFloorThickness(double floorThickness)
   {
      this.groundThickness = floorThickness;
   }

   public Robot createRobot()
   {
      Robot robot = new Robot("GroundAsABoxRobot");
      RigidJoint baseJoint = new RigidJoint("base", new Vector3D(), robot);

      //    FloatingJoint baseJoint = new FloatingJoint("base", new Vector3d(), this);
      Link baseLink = new Link("base");
      baseLink.setMassAndRadiiOfGyration(100000000000.0, 100.0, 100.0, 100.0);

      Graphics3DObject baseLinkGraphics = new Graphics3DObject();
      baseLinkGraphics.translate(0.0, 0.0, -groundThickness);
      baseLinkGraphics.rotate(groundAngle, Axis.Y);
      baseLinkGraphics.addCube(groundLength, groundWidth, groundThickness, YoAppearance.Green());

      CollisionMeshDescription collisonMeshDescription = new CollisionMeshDescription();
      collisonMeshDescription.translate(0.0, 0.0, -groundThickness);
      collisonMeshDescription.rotate(groundAngle, Axis.Y);
      collisonMeshDescription.addCubeReferencedAtBottomMiddle(groundLength, groundWidth, groundThickness);
      collisonMeshDescription.setIsGround(true);
      collisonMeshDescription.setEstimatedNumberOfContactPoints(estimatedNumberOfContactPoints);

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
         double yRotation = Math.PI / 8.0;
         addWall(groundLength, groundWidth, groundThickness, baseLinkGraphics, collisonMeshDescription, offsetX, offsetY, xRotation, yRotation);

         offsetX = 0.75;
         offsetY = 0.0;
         xRotation = 0.0;
         yRotation = -Math.PI / 8.0;
         addWall(groundLength, groundWidth, groundThickness, baseLinkGraphics, collisonMeshDescription, offsetX, offsetY, xRotation, yRotation);

         offsetX = 0.0;
         offsetY = -0.75;
         xRotation = -Math.PI / 8.0;
         yRotation = 0.0;
         addWall(groundLength, groundWidth, groundThickness, baseLinkGraphics, collisonMeshDescription, offsetX, offsetY, xRotation, yRotation);

         offsetX = 0.0;
         offsetY = 0.75;
         xRotation = Math.PI / 8.0;
         yRotation = 0.0;
         addWall(groundLength, groundWidth, groundThickness, baseLinkGraphics, collisonMeshDescription, offsetX, offsetY, xRotation, yRotation);
      }

      //    baseJoint.setVelocity(0.0, 0.0, 1.0);

      baseLink.setLinkGraphics(baseLinkGraphics);
      baseLink.addCollisionMesh(collisonMeshDescription);

      baseJoint.setLink(baseLink);
      robot.addRootJoint(baseJoint);
      robot.addStaticLink(baseLink);

      return robot;
   }

   private void addWall(double floorLength, double floorWidth, double floorThickness, Graphics3DObject baseLinkGraphics,
                        CollisionMeshDescription collisonMeshDescription, double offsetX, double offsetY, double xRotation, double yRotation)
   {
      baseLinkGraphics.identity();
      baseLinkGraphics.translate(new Vector3D(offsetX, offsetY, -floorThickness));
      RotationMatrix rotationMatrixX = new RotationMatrix();
      rotationMatrixX.setToRollMatrix(xRotation);
      baseLinkGraphics.rotate(rotationMatrixX);
      RotationMatrix rotationMatrixY = new RotationMatrix();
      rotationMatrixY.setToPitchMatrix(yRotation);
      baseLinkGraphics.rotate(rotationMatrixY);
      baseLinkGraphics.addCube(floorLength, floorWidth, floorThickness, YoAppearance.Green());

      collisonMeshDescription.identity();
      collisonMeshDescription.translate(offsetX, offsetY, -floorThickness);
      collisonMeshDescription.rotateEuler(new Vector3D(xRotation, yRotation, 0.0));
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
