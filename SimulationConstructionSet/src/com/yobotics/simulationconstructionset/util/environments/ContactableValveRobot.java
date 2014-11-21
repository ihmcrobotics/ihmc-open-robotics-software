package com.yobotics.simulationconstructionset.util.environments;

import java.util.ArrayList;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.graphics3DAdapter.input.ModifierKeyInterface;
import us.ihmc.graphics3DAdapter.input.SelectedListener;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.util.environments.ContactablePinJointRobot;
import us.ihmc.simulationconstructionset.util.environments.SelectableObject;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.utilities.math.RotationalInertiaCalculator;
import us.ihmc.utilities.math.geometry.FrameCylinder3d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameTorus3d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class ContactableValveRobot extends ContactablePinJointRobot implements SelectableObject, SelectedListener
{

   //   public static final double DEFAULT_RADIUS = 0.2;
   //   public static final double DEFAULT_THICKNESS = 0.05;
   //   private static final double DEFAULT_MASS = 1.0;
   private String name;

   private double valveRadius;
   private double valveOffsetFromWall;
   private double valveThickness;

   private int numberOfSpokes;
   private double spokesThickness;

   private FramePose valvePoseInWorld = new FramePose();

   private double valveNumberOfPossibleTurns;

   private double valveMass;
   private double valveIxx, valveIyy, valveIzz;

   private FrameTorus3d valveTorus;
   private ArrayList<FrameCylinder3d> spokesCylinders = new ArrayList<FrameCylinder3d>();

   private Link valveLink;
   private PinJoint valvePinJoint;
   private Graphics3DObject valveLinkGraphics = new Graphics3DObject();

   private PoseReferenceFrame valveFrame;

   private final RigidBodyTransform originalValvePose = new RigidBodyTransform();

   //   public ContactableValveRobot(String name, RigidBodyTransform pinJointTransformFromWorld)
   //   {
   //      this(name, pinJointTransformFromWorld, DEFAULT_RADIUS, DEFAULT_THICKNESS, DEFAULT_MASS);
   //   }

   public ContactableValveRobot(String name, double valveRadius, double valveOffsetFromWall, double valveThickness, int numberOfSpokes, double spokesThickness,
         FramePose valvePoseInWorld, double valveNumberOfPossibleTurns, double valveMass, double valveIxx, double valveIyy, double valveIzz)
   {
      super(name);
      this.name = name;
      setValveProperties(valveRadius, valveOffsetFromWall, valveThickness, numberOfSpokes, spokesThickness, valveNumberOfPossibleTurns, valveMass, valveIxx,
            valveIyy, valveIzz);
      setValvePoseInWorld(valvePoseInWorld);
      setMass(valveMass);
      setMomentOfInertia(valveIxx, valveIyy, valveIzz);
   }

   public ContactableValveRobot(String name, double valveRadius, double valveOffsetFromWall, double valveThickness, int numberOfSpokes, double spokesThickness,
         Point3d valvePosition, Quat4d valveOrientation, double valveNumberOfPossibleTurns, double valveMass, double valveIxx, double valveIyy, double valveIzz)
   {
      this(name, valveRadius, valveOffsetFromWall, valveThickness, numberOfSpokes, spokesThickness, new FramePose(ReferenceFrame.getWorldFrame(),
            valvePosition, valveOrientation), valveNumberOfPossibleTurns, valveMass, valveIxx, valveIyy, valveIzz);
   }

   public void setValveProperties(double valveRadius, double valveOffsetFromWall, double valveThickness, int numberOfSpokes, double spokesThickness,
         double valveNumberOfPossibleTurns, double valveMass, double valveIxx, double valveIyy, double valveIzz)
   {
      this.valveRadius = valveRadius;
      this.valveOffsetFromWall = valveOffsetFromWall;
      this.valveThickness = valveThickness;

      this.numberOfSpokes = numberOfSpokes;
      this.spokesThickness = spokesThickness;

      this.valveNumberOfPossibleTurns = valveNumberOfPossibleTurns;

      this.valveMass = valveMass;
      this.valveIxx = valveIxx;
      this.valveIyy = valveIyy;
      this.valveIzz = valveIzz;

   }

   public void setValvePoseInWorld(FramePose valvePoseInWorld)
   {
      this.valvePoseInWorld.setPose(valvePoseInWorld);
   }

   public void setValvePoseInWorld(Point3d position, Quat4d orientation)
   {
      this.valvePoseInWorld.setPose(position, orientation);
   }

   public void createValveRobot()
   {
      YoGraphicsListRegistry graphListRegistry = new YoGraphicsListRegistry();

      valveFrame = new PoseReferenceFrame("valveFrame", valvePoseInWorld);
      valveFrame.getPose(originalValvePose);

      //creating the pinJoint 
      Vector3d jointAxisVector = new Vector3d(1.0, 0.0, 0.0);
      RigidBodyTransform valveTransformToWorld = new RigidBodyTransform();
      valveFrame.getTransformToDesiredFrame(valveTransformToWorld, ReferenceFrame.getWorldFrame());
      valveTransformToWorld.transform(jointAxisVector);

      Vector3d valvePositionInWorld = new Vector3d();
      valvePoseInWorld.getPosition(valvePositionInWorld);
      valvePinJoint = new PinJoint("valvePinJoint", valvePositionInWorld, this, jointAxisVector);
      valvePinJoint.setLimitStops(0.0, valveNumberOfPossibleTurns * 2 * Math.PI, 1000, 100);
      valvePinJoint.setDamping(5);

      //put the graphics frame in the proper orientation
      Matrix3d rotationMatrix = new Matrix3d();
      valvePoseInWorld.getOrientation(rotationMatrix);
      valveLinkGraphics.rotate(rotationMatrix);
      RigidBodyTransform rotationTransform = new RigidBodyTransform();
      rotationTransform.setRotation(rotationMatrix);

      //Creating the physical link for the simulation
      valveLink = new Link("valveLink");
      valveLink.setMass(valveMass);
      valveLink.setComOffset(new Vector3d(0.0, 0.0, 0.0));

      Matrix3d inertia = RotationalInertiaCalculator.getRotationalInertiaMatrixOfTorus(valveMass, valveRadius, valveThickness);
      valveLink.setMomentOfInertia(inertia);
      valvePinJoint.setLink(valveLink);
      this.addRootJoint(valvePinJoint);

      //torus and offsetCylinder
      RigidBodyTransform transform = new RigidBodyTransform();
      RigidBodyTransform invertTransform = new RigidBodyTransform();
      Quat4d quat = new Quat4d();

      RotationFunctions.setQuaternionBasedOnYawPitchRoll(quat, 0.0, Math.PI / 2.0, 0.0);
      transform.setRotation(quat);
      invertTransform.set(transform);
      invertTransform.invert();

      valveTorus = new FrameTorus3d(valveFrame, transform, valveRadius, valveThickness);
      valveLinkGraphics.transform(transform);
      valveLinkGraphics.addArcTorus(0.0, 2 * Math.PI, valveRadius - valveThickness / 2.0, valveThickness / 2.0, YoAppearance.DarkRed());
      valveLinkGraphics.addCylinder(valveOffsetFromWall, spokesThickness / 2.0, YoAppearance.DarkRed());
      valveLinkGraphics.transform(invertTransform);

      //spokes
      for (int i = 0; i < numberOfSpokes; i++)
      {
         RotationFunctions.setQuaternionBasedOnYawPitchRoll(quat, 0.0, 0.0, i * 2.0 * Math.PI / numberOfSpokes);
         transform.setRotation(quat);
         invertTransform.set(transform);
         invertTransform.invert();

         RigidBodyTransform yoGraphicTransform = new RigidBodyTransform();
         yoGraphicTransform.multiply(rotationTransform, transform);

         FrameCylinder3d spokeCylinder = new FrameCylinder3d(valveFrame, transform, valveRadius - spokesThickness / 2.0, spokesThickness / 2.0);
         spokesCylinders.add(spokeCylinder);

         FramePoint cylinderOrigin = new FramePoint(valveFrame);
         cylinderOrigin.changeFrame(ReferenceFrame.getWorldFrame());

         FrameVector cylinderVector = new FrameVector(valveFrame, 0.0, 0.0, 1.0);
         cylinderVector.applyTransform(transform);
         cylinderVector.changeFrame(ReferenceFrame.getWorldFrame());

         valveLinkGraphics.transform(transform);
         valveLinkGraphics.addCylinder(valveRadius - spokesThickness / 2.0, spokesThickness / 2.0, YoAppearance.DarkRed());
      }

      //setting the graphics for the link
      valveLink.setLinkGraphics(valveLinkGraphics);

      yoGraphicsListRegistries.add(graphListRegistry);
   }

   @Override
   public void updateAllGroundContactPointVelocities()
   {
      RigidBodyTransform pinJointTransform = new RigidBodyTransform();
      RigidBodyTransform newValvePose = new RigidBodyTransform();
      pinJointTransform.rotX(valvePinJoint.getQ().getDoubleValue());
      newValvePose.multiply(originalValvePose, pinJointTransform);
      valveFrame.setPoseAndUpdate(newValvePose);

      super.updateAllGroundContactPointVelocities();
   }

   @Override
   public boolean isPointOnOrInside(Point3d pointInWorldToCheck)
   {
      FramePoint pointToCheck = new FramePoint(ReferenceFrame.getWorldFrame(), pointInWorldToCheck);
      pointToCheck.changeFrame(valveFrame);

      if (valveTorus.isInsideOrOnSurface(pointToCheck))
         return true;
      for (int i = 0; i < spokesCylinders.size(); i++)
      {
         if (spokesCylinders.get(i).isInsideOrOnSurface(pointToCheck))
            return true;
      }
      return false;
   }

   @Override
   public boolean isClose(Point3d pointInWorldToCheck)
   {
      return isPointOnOrInside(pointInWorldToCheck);
   }

   @Override
   public void closestIntersectionAndNormalAt(Point3d intersectionToPack, Vector3d normalToPack, Point3d pointInWorldToCheck)
   {
      FramePoint pointToCheck = new FramePoint(ReferenceFrame.getWorldFrame(), pointInWorldToCheck);
      pointToCheck.changeFrame(valveFrame);

      if (valveTorus.checkIfInside(pointToCheck, intersectionToPack, normalToPack))
         return;
      for (int i = 0; i < spokesCylinders.size(); i++)
      {
         if (spokesCylinders.get(i).checkIfInside(pointToCheck, intersectionToPack, normalToPack))
            return;
      }
   }

   @Override
   public void selected(Graphics3DNode graphics3dNode, ModifierKeyInterface modifierKeyInterface, Point3d location, Point3d cameraLocation,
         Quat4d cameraRotation)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void select()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void unSelect(boolean reset)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void addSelectedListeners(SelectableObjectListener selectedListener)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public PinJoint getPinJoint()
   {
      return valvePinJoint;
   }

   @Override
   public void setMass(double mass)
   {
      this.valveMass = mass;
   }

   @Override
   public void setMomentOfInertia(double Ixx, double Iyy, double Izz)
   {
      this.valveIxx = Ixx;
      this.valveIyy = Iyy;
      this.valveIzz = Izz;
   }

}
