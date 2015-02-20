package us.ihmc.simulationconstructionset.util.environments;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graphics3DAdapter.input.Key;
import us.ihmc.graphics3DAdapter.input.ModifierKeyInterface;
import us.ihmc.graphics3DAdapter.input.SelectedListener;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.math.RotationalInertiaCalculator;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;

public class ContactableDoorRobot extends ContactablePinJointRobot implements SelectableObject, SelectedListener
{
   public static final Vector2d DEFAULT_HANDLE_OFFSET = new Vector2d(0.65, 0.85);
   public static final Vector3d DEFAULT_DOOR_DIMENSIONS = new Vector3d(0.7, 0.03, 2.0);
   public static final double DEFAULT_MASS = 20.28; // 5lbs
   public static final double DEFAULT_HANDLE_DOOR_SEPARATION = 0.03;
   public static final Vector3d DEFAULT_HANDLE_DIMENSIONS = new Vector3d(0.05, 0.01, 0.005);
   public static final double DEFAULT_HANDLE_HINGE_RADIUS = 0.0075;
   
   private static final AppearanceDefinition defaultColor = YoAppearance.Gray();
   
   private double widthX;
   private double depthY;
   private double heightZ;
   
   private double mass;
   private Matrix3d inertiaMatrix;
      
   private final Vector3d positionInWorld;
   
   private Link doorLink;
   private PinJoint doorHingePinJoint;
   private Graphics3DObject doorLinkGraphics = new Graphics3DObject();
   
   private PinJoint handlePinJoint;
   private Link handleLink;
   private Graphics3DObject doorHandleGraphics = new Graphics3DObject();
   private final Vector2d handleOffset;
   private final Vector3d handleDimensions;
   private final double handleDoorSeparation;
   private final double handleHingeRadius;
   
   public ContactableDoorRobot(String name, Vector3d positionInWorld)
   {
      this(name, DEFAULT_DOOR_DIMENSIONS, DEFAULT_MASS, positionInWorld, DEFAULT_HANDLE_OFFSET, 
            DEFAULT_HANDLE_DIMENSIONS, DEFAULT_HANDLE_DOOR_SEPARATION, DEFAULT_HANDLE_HINGE_RADIUS);
   }
   
   public ContactableDoorRobot(String name, Vector3d boxDimensions, double mass, Vector3d positionInWorld, Vector2d handleOffset, 
         Vector3d handleDimensions, double handleDoorSeparation, double handleHingeRadius)
   {
      super(name);
      
      this.mass = mass;           
      
      this.widthX = boxDimensions.x;
      this.depthY = boxDimensions.y;
      this.heightZ = boxDimensions.z;
      
      this.positionInWorld = positionInWorld;
      
      this.handleOffset = handleOffset;
      this.handleDimensions = handleDimensions;
      this.handleHingeRadius = handleHingeRadius;
      
      this.handleDoorSeparation = handleDoorSeparation;      
      
      createDoor();
      createDoorGraphics();
      createHandle();
      createHandleGraphics();
   }
   
   private void createDoor()
   {
      // creating the pinJoint, i.e. door hinge
      Vector3d jointAxisVector = new Vector3d(0.0, 0.0, 1.0);      
      doorHingePinJoint = new PinJoint("doorHingePinJoint", positionInWorld, this, jointAxisVector);
      
      // door link
      doorLink = new Link("doorLink");
      doorLink.setMass(mass);
      doorLink.setComOffset(0.5*widthX, 0.5*depthY, 0.5*heightZ);
      
      inertiaMatrix = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidBox(widthX, depthY, heightZ, mass);
      doorLink.setMomentOfInertia(inertiaMatrix);
      doorHingePinJoint.setLink(doorLink);
      this.addRootJoint(doorHingePinJoint);
   }
   
   private void createHandle()
   {
      // create handle
      handlePinJoint = new PinJoint("handlePinJoint", new Vector3d(handleOffset.x, 0.0, handleOffset.y), this, Axis.Y);
      
      // handle link
      handleLink = new Link("handleHorizontalLink");
      handleLink.setMass(0.2);
      handleLink.setMomentOfInertia(0.1, 0.1, 0.1); // TODO
      handlePinJoint.setLink(handleLink);
      doorHingePinJoint.addJoint(handlePinJoint);
   }
   
   private void createDoorGraphics()
   {      
      // graphics - door
      List<Point2d> doorPoints = new ArrayList<Point2d>();
      doorPoints.add(new Point2d());
      doorPoints.add(new Point2d(widthX, 0.0));
      doorPoints.add(new Point2d(widthX, depthY));
      doorPoints.add(new Point2d(0.0,    depthY));
      doorLinkGraphics.addExtrudedPolygon(doorPoints, heightZ, defaultColor);
      doorLink.setLinkGraphics(doorLinkGraphics);
   }
   
   private void createHandleGraphics()
   {
      // graphics - handle hinge
      Matrix3d rotX90 = new Matrix3d();
      rotX90.rotX(Math.PI / 2.0);
      doorHandleGraphics.rotate(rotX90);
      doorHandleGraphics.addCylinder(handleDoorSeparation, handleHingeRadius, YoAppearance.Grey());
      doorHandleGraphics.addCylinder(-handleDoorSeparation-depthY, handleHingeRadius, YoAppearance.Grey());
      doorHandleGraphics.translate(new Vector3d(0.0, 0.0, -0.5 * depthY)); // center graphics
      
      for(RobotSide robotSide : RobotSide.values())
      {
         // graphics - handle
         double translation = robotSide.negateIfLeftSide(0.03 + 0.5*depthY);
         doorHandleGraphics.translate(new Vector3d(0.0, 0.0, translation));
         List<Point2d> handlePoints = new ArrayList<Point2d>();
         handlePoints.add(new Point2d(0.0  , -0.5 * handleDimensions.y));
         handlePoints.add(new Point2d(0.0  ,  0.5 * handleDimensions.y));
         handlePoints.add(new Point2d(-handleDimensions.x,  0.5 * handleDimensions.y));
         handlePoints.add(new Point2d(-handleDimensions.x, -0.5 * handleDimensions.y));
         double extrusionLength = robotSide.negateIfLeftSide(0.005);
         doorHandleGraphics.addExtrudedPolygon(handlePoints, extrusionLength, YoAppearance.Grey());
         doorHandleGraphics.translate(new Vector3d(0.0, 0.0, -translation)); 
      }
      
      handleLink.setLinkGraphics(doorHandleGraphics);
   }
   
   @Override
   public boolean isClose(Point3d pointInWorldToCheck)
   {
      double epsilon = 2.0*handleDoorSeparation;
      
      Point3d pointToCheck = toDoorFrame(pointInWorldToCheck);
      
      System.out.println(pointToCheck);
      
      boolean b1 = pointToCheck.getX() > -epsilon && pointToCheck.getX() < widthX + epsilon && 
            pointToCheck.getY() > -epsilon && pointToCheck.getY() < depthY + epsilon && 
            pointToCheck.getZ() > -epsilon && pointToCheck.getZ() < heightZ + epsilon;
            
      return b1;
   }
   
   @Override
   public boolean isPointOnOrInside(Point3d pointInWorldToCheck)
   {
      Point3d pointInDoorFrame = toDoorFrame(pointInWorldToCheck);
      
      // check if on/inside door
      if(pointInDoorFrame.getX() >= 0.0 && pointInDoorFrame.getX() <= widthX && 
            pointInDoorFrame.getY() >= 0.0 && pointInDoorFrame.getY() <= depthY && 
                  pointInDoorFrame.getZ() >= 0.0 && pointInDoorFrame.getZ() <= heightZ)
         return true;
      
//      // check if on/inside handle
//      for(RobotSide robotSide : RobotSide.values())
//      {
//         pointToCheck.changeFrame(handleFrames.get(robotSide));
//         
//         if(pointToCheck.getX() >= 0.0 && pointToCheck.getX() <= handleDimensions.x &&
//               pointToCheck.getY() >= 0.0 && pointToCheck.getY() <= handleDimensions.y &&
//               pointToCheck.getZ() >= 0.0 && pointToCheck.getZ() <= handleDimensions.z)
//            return true;
//      }
//      
//      // check if on/inside hinge
//      pointToCheck.changeFrame(handleFrames.get(RobotSide.LEFT));
//      if(pointToCheck.getY() >= 0.0 && pointToCheck.getY() <= 2.0 * handleDoorSeparation &&
//            pointToCheck.getX() * pointToCheck.getX() + pointToCheck.getZ() + pointToCheck.getZ() <= handleHingeRadius * handleHingeRadius)
//         return true;
      
      return false;
   }
   
   public void setKdDoor(double kd)
   {
      doorHingePinJoint.setKd(kd);
   }
   
   public void setKpDoor(double kp)
   {
      doorHingePinJoint.setKp(kp);
   }
   
   public void setTauDoor(double tau)
   {
      doorHingePinJoint.setTau(tau);
   }
   
   public void setKdHandle(double kd)
   {
      handlePinJoint.setKd(kd);
   }
   
   public void setKpHandle(double kp)
   {
      handlePinJoint.setKp(kp);
   }
   
   public void setTauHandle(double tau)
   {
      doorHingePinJoint.setTau(tau);
   }
   
   public void setYaw(double yaw)
   {
      doorHingePinJoint.setQ(yaw);
   }
   
   public double getYaw()
   {
      return doorHingePinJoint.getQ().getDoubleValue();
   }
   
   private Point3d toDoorFrame(Point3d pointInWorld)
   {
      Point3d ret = new Point3d(pointInWorld);
      ret.sub(positionInWorld);
      double q = getYaw();
      double rotX = ret.x * Math.cos(q) + ret.y * Math.sin(q);
      ret.y       = -ret.x * Math.sin(q) + ret.y * Math.cos(q);
      ret.x = rotX;
      return ret;
   }
   
   private Point3d toHandleFrame(Point3d pointInWorld, RobotSide robotSide)
   {
      return null;
   }

   @Override
   public void closestIntersectionAndNormalAt(Point3d intersectionToPack, Vector3d normalToPack, Point3d pointInWorldToCheck)
   {
      Point3d pointInDoorFrame = toDoorFrame(pointInWorldToCheck);
      
   }

   @Override
   public void selected(Graphics3DNode graphics3dNode, ModifierKeyInterface modifierKeyInterface, Point3d location, Point3d cameraLocation, Quat4d cameraRotation)
   {
      if (!modifierKeyInterface.isKeyPressed(Key.N))
         return;
      select();  
   }

   @Override
   public void select()
   {
      // TODO
   }

   @Override
   public void unSelect(boolean reset)
   {
      // TODO
   }

   @Override
   public void addSelectedListeners(SelectableObjectListener selectedListener)
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void getBodyTransformToWorld(RigidBodyTransform transformToWorld)
   {
      // TODO
//      transformToWorld.set(doorFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame()));
   }

   @Override
   public void setMass(double mass)
   {
      this.mass = mass;
   }

   @Override
   public void setMomentOfInertia(double Ixx, double Iyy, double Izz)
   {
      inertiaMatrix.setM00(Ixx);
      inertiaMatrix.setM01(0.0);
      inertiaMatrix.setM02(0.0);
      inertiaMatrix.setM10(0.0);
      inertiaMatrix.setM11(Iyy);
      inertiaMatrix.setM12(0.0);
      inertiaMatrix.setM20(0.0);
      inertiaMatrix.setM21(0.0);
      inertiaMatrix.setM22(Izz);
   }

   @Override
   public PinJoint getPinJoint()
   {
      return doorHingePinJoint;
   }

}
