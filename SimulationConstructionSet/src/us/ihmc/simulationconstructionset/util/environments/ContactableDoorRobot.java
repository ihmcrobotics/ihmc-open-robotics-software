package us.ihmc.simulationconstructionset.util.environments;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.Contactable;
import us.ihmc.tools.inputDevices.keyboard.Key;
import us.ihmc.tools.inputDevices.keyboard.ModifierKeyInterface;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.input.SelectedListener;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.geometry.shapes.FrameBox3d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ContactableDoorRobot extends Robot implements SelectableObject, SelectedListener, Contactable
{
   public static final Vector2d DEFAULT_HANDLE_OFFSET = new Vector2d(0.83, 0.98);
   public static final Vector3d DEFAULT_DOOR_DIMENSIONS = new Vector3d(0.9144, 0.045, 2.04);
   public static final double DEFAULT_MASS = 2.28; // 5lbs
   public static final double DEFAULT_HANDLE_DOOR_SEPARATION = 0.06;
   public static final double DEFAULT_HANDLE_HINGE_RADIUS = 0.01;
   public static final double DEFAULT_HANDLE_RADIUS = 0.02;
   public static final double DEFAULT_HANDLE_LENGTH = 0.12;
      
   private static final AppearanceDefinition defaultColor = YoAppearance.Gray();
   
   private double widthX;
   private double depthY;
   private double heightZ;
   
   private final RigidBodyTransform originalDoorPose;
   private final PoseReferenceFrame doorFrame;
   private final FrameBox3d doorBox;
   
   private final SideDependentList<PoseReferenceFrame> handlePoses = new SideDependentList<PoseReferenceFrame>();
   
   private double mass;
   private Matrix3d inertiaMatrix;
   
   private Link doorLink;
   private PinJoint doorHingePinJoint;
   private Graphics3DObject doorLinkGraphics = new Graphics3DObject();
   
   private PinJoint handlePinJoint;
   private Link handleLink;
   private Graphics3DObject doorHandleGraphics = new Graphics3DObject();
   private final Vector2d handleOffset;
   private final double handleDoorSeparation;
   private final double handleHingeRadius;
   private final double handleRadius, handleLength;
      
   private final InternalMultiJointArticulatedContactable internalMultiJointArticulatedContactable;
   
   public ContactableDoorRobot(String name, Point3d positionInWorld)
   {
      this(name, DEFAULT_DOOR_DIMENSIONS, DEFAULT_MASS, positionInWorld, DEFAULT_HANDLE_OFFSET, 
            DEFAULT_HANDLE_RADIUS, DEFAULT_HANDLE_LENGTH, DEFAULT_HANDLE_DOOR_SEPARATION, DEFAULT_HANDLE_HINGE_RADIUS);
   }
   
   public ContactableDoorRobot(String name, Vector3d boxDimensions, double mass, Point3d positionInWorld, Vector2d handleOffset, 
         double handleRadius, double handleLength, double handleDoorSeparation, double handleHingeRadius)
   {
      super(name);
      
      this.mass = mass;           
      
      this.widthX = boxDimensions.getX();
      this.depthY = boxDimensions.getY();
      this.heightZ = boxDimensions.getZ();
            
      this.handleOffset = handleOffset;
      this.handleHingeRadius = handleHingeRadius;
      
      this.handleDoorSeparation = handleDoorSeparation;     
      
      this.handleRadius = handleRadius;
      this.handleLength = handleLength;
      
      createDoor(new Vector3d(positionInWorld));
      createDoorGraphics();
      createHandle();
      createHandleGraphics();
      
      // set up reference frames
      originalDoorPose = new RigidBodyTransform(new AxisAngle4d(), new Vector3d(positionInWorld)); 
      doorFrame = new PoseReferenceFrame("doorFrame", new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(positionInWorld), new AxisAngle4d()));
      doorBox = new FrameBox3d(doorFrame, widthX, depthY, heightZ);
      
      for(RobotSide robotSide : RobotSide.values())
      {
         Vector3d offset = new Vector3d(handleOffset.getX(), 0.5*depthY + robotSide.negateIfLeftSide(0.5*depthY + handleDoorSeparation), handleOffset.getY());
         handlePoses.put(robotSide, new PoseReferenceFrame(robotSide.getCamelCaseNameForStartOfExpression() + "HandleFrame",
               new FramePose(doorFrame, new Point3d(offset), new AxisAngle4d())));
      }
      
      internalMultiJointArticulatedContactable = new InternalMultiJointArticulatedContactable(getName(), this);
   }

   private void createDoor(Vector3d positionInWorld)
   {
      // creating the pinJoint, i.e. door hinge
      doorHingePinJoint = new PinJoint("doorHingePinJoint", positionInWorld, this, Axis.Z);
      
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
      handlePinJoint = new PinJoint("handlePinJoint", new Vector3d(handleOffset.getX(), 0.0, handleOffset.getY()), this, Axis.Y);
      
      // handle link
      handleLink = new Link("handleHorizontalLink");
      handleLink.setMass(0.2);
      handleLink.setMomentOfInertia(0.1, 0.1, 0.1); // TODO
      handlePinJoint.setLink(handleLink);
      doorHingePinJoint.addJoint(handlePinJoint);
      handlePinJoint.setKd(2.0);
      handlePinJoint.setKp(0.5);
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
      doorHandleGraphics.translate(new Vector3d(0.0, 0.0, -0.5 * depthY)); // center graphics
      
      for(RobotSide robotSide : RobotSide.values())
      {
         doorHandleGraphics.addCylinder(robotSide.negateIfLeftSide(handleDoorSeparation+0.5*depthY), handleHingeRadius, YoAppearance.Grey());
         // graphics - handle
         double translation = robotSide.negateIfLeftSide(handleDoorSeparation + 0.5*depthY);
         doorHandleGraphics.translate(new Vector3d(0.0, 0.0, translation));
         
         doorHandleGraphics.rotate(new AxisAngle4d(0.0, 1.0, 0.0, robotSide.negateIfRightSide(0.5 * Math.PI)));
         doorHandleGraphics.addCylinder(robotSide.negateIfLeftSide(handleLength), handleRadius, YoAppearance.Grey());
         doorHandleGraphics.rotate(new AxisAngle4d(0.0, 1.0, 0.0, -robotSide.negateIfRightSide(0.5 * Math.PI)));
         
         doorHandleGraphics.translate(new Vector3d(0.0, 0.0, -translation)); 
      }
      
      handleLink.setLinkGraphics(doorHandleGraphics);
   }
   
   @Override
   public boolean isClose(Point3d pointInWorldToCheck)
   {      
      return isPointOnOrInside(pointInWorldToCheck);
   }
   
   private final FramePoint pointToCheck = new FramePoint();
   @Override
   public boolean isPointOnOrInside(Point3d pointInWorldToCheck)
   {
      pointToCheck.setIncludingFrame(ReferenceFrame.getWorldFrame(), pointInWorldToCheck);
      pointToCheck.changeFrame(doorFrame);
      pointToCheck.add(-0.5*widthX, -0.5*depthY, -0.5*heightZ); // since FrameBox3d.isInsideOrOnSurface assumes center of box is origin

      if (doorBox.isInsideOrOnSurface(pointToCheck))
      {
         lastInsideHandles = false;
         return true;
      }
      
      pointToCheck.add(0.5*widthX, 0.5*depthY, 0.5*heightZ);
      if(isInsideHandles(pointToCheck))
      {
         lastInsideHandles = true;
         return true;
      }
      
      return false;
   }
   
   private boolean lastInsideHandles; // TODO hack since SlipStickModel doesn't seem to address multi-Link situations
   
   private boolean isInsideHandles(FramePoint pointInDoorFrame)
   {
      for(RobotSide robotSide : RobotSide.values)
      {
         pointInDoorFrame.changeFrame(handlePoses.get(robotSide));
         
         if(pointInDoorFrame.getX() <= 0.0 && pointInDoorFrame.getX() >= -handleLength && 
               pointInDoorFrame.getY()*pointInDoorFrame.getY() + pointInDoorFrame.getZ()*pointInDoorFrame.getZ() < handleRadius*handleRadius)
            return true;
      }
      return false;
   }   
   
   @Override
   public void closestIntersectionAndNormalAt(Point3d intersectionToPack, Vector3d normalToPack, Point3d pointInWorldToCheck)
   {
      FramePoint pointToCheck = new FramePoint(ReferenceFrame.getWorldFrame(), pointInWorldToCheck);
      
      boolean packedByHandles = false;
      
      FramePoint frameIntersectionToPack = new FramePoint();
      FrameVector frameNormalToPack = new FrameVector();
      
      // check if close enough to handles
      for (RobotSide robotSide : RobotSide.values())
      {
         pointToCheck.changeFrame(handlePoses.get(robotSide));
         
         if(robotSide.negateIfLeftSide(pointToCheck.getY()) < -0.5*handleDoorSeparation)
            continue;
         
         packedByHandles = true;
         
         frameIntersectionToPack.changeFrame(handlePoses.get(robotSide));
         frameNormalToPack.changeFrame(handlePoses.get(robotSide));
         
         double mag = Math.sqrt(pointToCheck.getY()*pointToCheck.getY() + pointToCheck.getZ()*pointToCheck.getZ());
         double normY = pointToCheck.getY() / mag;
         double normZ = pointToCheck.getZ() / mag;
         frameNormalToPack.set(0.0, normY, normZ);
      }
      
      if(!packedByHandles)
      {
         pointToCheck.changeFrame(doorFrame);
         doorBox.getClosestPointAndNormalAt(frameIntersectionToPack, frameNormalToPack, pointToCheck);         
      }
      
      frameNormalToPack.changeFrame(ReferenceFrame.getWorldFrame());
      normalToPack.set(frameNormalToPack.getVectorCopy());
      
      frameIntersectionToPack.changeFrame(ReferenceFrame.getWorldFrame());
      intersectionToPack.set(frameIntersectionToPack.getPointCopy());
   }
   
   @Override
   public void updateAllGroundContactPointVelocities()
   {
      Matrix3d hingeRotation = new Matrix3d();
      hingeRotation.rotZ(getHingeYaw());
      RigidBodyTransform newDoorPose = new RigidBodyTransform(originalDoorPose);
      newDoorPose.setRotation(hingeRotation);
      doorFrame.setPoseAndUpdate(newDoorPose);
      
      for(RobotSide robotSide : RobotSide.values())
      {
         RigidBodyTransform handlePose = handlePoses.get(robotSide).getTransformToDesiredFrame(doorFrame);
         handlePose.setRotation(new AxisAngle4d(0.0, 1.0, 0.0, getHandleAngle()));
         handlePoses.get(robotSide).setPoseAndUpdate(handlePose);
      }
      
      super.updateAllGroundContactPointVelocities();
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
   
   public double getHingeYaw()
   {
      return doorHingePinJoint.getQYoVariable().getDoubleValue();
   }
   
   public double getHandleAngle()
   {
      return handlePinJoint.getQYoVariable().getDoubleValue();
   }
   
   public void setHandleAngle(double theta)
   {
      handlePinJoint.setQ(theta);
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
   }

   @Override
   public void unSelect(boolean reset)
   {
   }

   @Override
   public void addSelectedListeners(SelectableObjectListener selectedListener)
   {
   }

   public void getBodyTransformToWorld(RigidBodyTransform transformToWorld)
   {
      transformToWorld.set(doorFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame()));
   }

   public void setMass(double mass)
   {
      this.mass = mass;
   }

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

   public PinJoint getPinJoint()
   {
      return doorHingePinJoint;
   }
   
   public PinJoint getHandleJoint()
   {
      return handlePinJoint;
   }
   
   @Override
   public void updateContactPoints()
   {
      internalMultiJointArticulatedContactable.updateContactPoints();
   }

   @Override
   public int getAndLockAvailableContactPoint()
   {
      return internalMultiJointArticulatedContactable.getAndLockAvailableContactPoint();
   }

   @Override
   public void unlockContactPoint(GroundContactPoint groundContactPoint)
   {      
      internalMultiJointArticulatedContactable.unlockContactPoint(groundContactPoint);
   }

   @Override
   public GroundContactPoint getLockedContactPoint(int contactPointIndex)
   {
      return internalMultiJointArticulatedContactable.getLockedContactPoint(contactPointIndex);
   }
      
   public void createAvailableContactPoints(int groupId, int numDoorContactPoints, int numHandleContactPoints, double forceVectorScale, boolean visualizeContacts)
   {
      ArrayList<Integer> numContactPoints = new ArrayList<Integer>();
      numContactPoints.add(new Integer(numDoorContactPoints));
      numContactPoints.add(new Integer(numHandleContactPoints));
      internalMultiJointArticulatedContactable.createAvailableContactPoints(groupId, numContactPoints, forceVectorScale, visualizeContacts);
      
      internalMultiJointArticulatedContactable.setNumDoorContactPoints(numDoorContactPoints);
   }

   private static class InternalMultiJointArticulatedContactable extends MultiJointArticulatedContactable
   {
      private final ContactableDoorRobot robot;
      public int numDoorContactPoints;
      
      public InternalMultiJointArticulatedContactable(String name, ContactableDoorRobot robot)
      {
         super(name, robot);
         this.robot = robot;
      }

      @Override
      public boolean isClose(Point3d pointInWorldToCheck)
      {
         return robot.isClose(pointInWorldToCheck);
      }

      @Override
      public boolean isPointOnOrInside(Point3d pointInWorldToCheck)
      {
         return robot.isPointOnOrInside(pointInWorldToCheck);
      }

      @Override
      public void closestIntersectionAndNormalAt(Point3d intersectionToPack, Vector3d normalToPack, Point3d pointInWorldToCheck)
      {
         robot.closestIntersectionAndNormalAt(intersectionToPack, normalToPack, pointInWorldToCheck);
      }

      @Override
      public GroundContactPoint getLockedContactPoint(int contactPointIndex)
      {
         if (contactPointIndex > numDoorContactPoints)
         {
            contactPointIndex -= numDoorContactPoints;
         }
         
         int jointIndex = robot.lastInsideHandles ? 1 : 0;
         
         GroundContactPoint point = getLockedContactPoint(jointIndex, contactPointIndex);
                 
         return point;
      }
      
      @Override
      public int getAndLockAvailableContactPoint()
      {
         int jointIndex = robot.lastInsideHandles ? 1 : 0;
         return getAndLockAvailableContactPoint(jointIndex);
      }

      @Override
      public ArrayList<Joint> getJoints()
      {
         ArrayList<Joint> joints = new ArrayList<Joint>();
         joints.add(robot.getPinJoint());
         joints.add(robot.getHandleJoint());
         return joints;
      }
      
      public void setNumDoorContactPoints(int numDoorContactPoints) 
      {
         this.numDoorContactPoints = numDoorContactPoints;
      }
   }
}
