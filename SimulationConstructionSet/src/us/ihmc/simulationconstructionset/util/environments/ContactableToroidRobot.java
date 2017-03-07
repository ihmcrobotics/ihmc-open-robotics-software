package us.ihmc.simulationconstructionset.util.environments;

import java.awt.Color;
import java.util.ArrayList;

import org.apache.commons.math3.util.FastMath;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.input.SelectedListener;
import us.ihmc.graphicsDescription.instructions.ArcTorusGraphics3DInstruction;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.geometry.shapes.FrameTorus3d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBodyInertia;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.GroundContactPointGroup;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.tools.inputDevices.keyboard.Key;
import us.ihmc.tools.inputDevices.keyboard.ModifierKeyInterface;

public class ContactableToroidRobot extends ContactablePinJointRobot implements SelectableObject, SelectedListener
{
   public static final double DEFAULT_RADIUS = 0.2;
   public static final double DEFAULT_THICKNESS = 0.05;
   private static final double DEFAULT_MASS = 1.0;

   private final RigidBodyTransform pinJointTransformToWorld;
   private final FrameTorus3d wheelTorus;
   
   private final PinJoint pinJoint;
   private final Link wheelLink;
   private final Graphics3DObject linkGraphics;
   
   private static final Color defaultColor = Color.BLACK;
   private static final Color selectedColor = Color.CYAN;
   
   private final double selectTransparency = 0.0;
   private final double unselectTransparency = 0.0;
   
   private ArcTorusGraphics3DInstruction wheelGraphic;
   
   private final ArrayList<SelectableObjectListener> selectedListeners = new ArrayList<SelectableObjectListener>();

   public ContactableToroidRobot(String name, RigidBodyTransform pinJointTransformFromWorld)
   {
      this(name, pinJointTransformFromWorld, DEFAULT_RADIUS, DEFAULT_THICKNESS, DEFAULT_MASS);
   }

   public ContactableToroidRobot(String name, RigidBodyTransform pinJointTransform, double steeringWheelRadius,
           double steeringWheelThickness, double mass)
   {
      super(name);
      pinJointTransformToWorld = pinJointTransform;
      wheelTorus = new FrameTorus3d(ReferenceFrame.getWorldFrame(), pinJointTransform, steeringWheelRadius, steeringWheelThickness);
      
      RotationMatrix rotation = new RotationMatrix();
      Vector3D offset = new Vector3D();
      pinJointTransform.getTranslation(offset);
      pinJointTransform.getRotation(rotation);
      
      Vector3D axis = new Vector3D(0.0, 0.0, 1.0);
      RigidBodyTransform rotationTransform = new RigidBodyTransform();
      rotationTransform.setRotation(rotation);
      rotationTransform.transform(axis);
      
      
      pinJoint = new PinJoint(name + "PinJoint", offset, this, axis);

      wheelLink = new Link(name + "Link");
      wheelLink.setMass(mass);
      wheelLink.setComOffset(new Vector3D());
      
      Matrix3D inertia = RotationalInertiaCalculator.getRotationalInertiaMatrixOfTorus(mass, wheelTorus.getRadius(), wheelTorus.getThickness());
      
      RigidBodyInertia rigidBodyInertia = new RigidBodyInertia(ReferenceFrame.getWorldFrame(), inertia, mass);
      ReferenceFrame jointFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("toroidFrame", ReferenceFrame.getWorldFrame(), pinJointTransform);
      rigidBodyInertia.changeFrame(jointFrame);
      wheelLink.setMomentOfInertia(rigidBodyInertia.getMassMomentOfInertiaPartCopy());

      linkGraphics = new Graphics3DObject();
      linkGraphics.rotate(rotation);
      wheelLink.setLinkGraphics(linkGraphics);

      pinJoint.setLink(wheelLink);
      this.addRootJoint(pinJoint);
      
      createWheelGraphics(wheelTorus);
      
      unSelect(true);
      
      linkGraphics.registerSelectedListener(this);
   }
   
   private void createWheelGraphics(FrameTorus3d wheelTorus)
   {
      linkGraphics.addCone(0.05, 0.05);
      linkGraphics.addArcTorus(FastMath.PI / 3, 2 * FastMath.PI / 3, wheelTorus.getRadius(), wheelTorus.getThickness(), YoAppearance.Yellow());
      wheelGraphic = linkGraphics.addArcTorus(2 * FastMath.PI / 3, 7 * FastMath.PI / 3, wheelTorus.getRadius(), wheelTorus.getThickness(), YoAppearance.Black());
   }

   public Link getWheelLink()
   {
      return wheelLink;
   }
   
   public void addDynamicGraphicForceVectorsToGroundContactPoints(double forceVectorScale, AppearanceDefinition appearance, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      addDynamicGraphicForceVectorsToGroundContactPoints(1, forceVectorScale, appearance, yoGraphicsListRegistry);
   }
   
   public void addDynamicGraphicForceVectorsToGroundContactPoints(int groupIdentifier, double forceVectorScale, AppearanceDefinition appearance, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (yoGraphicsListRegistry == null) return;
      
      GroundContactPointGroup groundContactPointGroup = pinJoint.getGroundContactPointGroup(groupIdentifier);
      System.out.println("GroundContactPointGroup" + groundContactPointGroup.getGroundContactPoints());
      ArrayList<GroundContactPoint> groundContactPoints = groundContactPointGroup.getGroundContactPoints();
      
      for (GroundContactPoint groundContactPoint : groundContactPoints)
      {
         YoGraphicVector yoGraphicVector = new YoGraphicVector(groundContactPoint.getName(), groundContactPoint.getYoPosition(), groundContactPoint.getYoForce(), forceVectorScale, appearance);
         yoGraphicsListRegistry.registerYoGraphic("ContactableToroidRobot", yoGraphicVector);
      }
   }
   
   @Override
   public PinJoint getPinJoint()
   {
      return pinJoint;
   }

   @Override
   public synchronized boolean isPointOnOrInside(Point3D pointInWorldToCheck)
   {
      return wheelTorus.getTorus3d().isInsideOrOnSurface(pointInWorldToCheck);
   }
   
   @Override
   public boolean isClose(Point3D pointInWorldToCheck)
   {
      return isPointOnOrInside(pointInWorldToCheck);
   }

   @Override
   public synchronized void closestIntersectionAndNormalAt(Point3D intersectionToPack, Vector3D normalToPack, Point3D pointInWorldToCheck)
   {
      wheelTorus.getTorus3d().checkIfInside(pointInWorldToCheck, intersectionToPack, normalToPack);
   }


   @Override
   public void selected(Graphics3DNode graphics3dNode, ModifierKeyInterface modifierKeyInterface, Point3DReadOnly location, Point3DReadOnly cameraLocation,
         QuaternionReadOnly cameraRotation)
   {
      if (!modifierKeyInterface.isKeyPressed(Key.N))
         return;
      select();
   }

   @Override
   public void select()
   {
      unSelect(false);
      
      wheelGraphic.setAppearance(new YoAppearanceRGBColor(selectedColor, selectTransparency));
      
      notifySelectedListenersThisWasSelected(this);
   }

   @Override
   public void unSelect(boolean reset)
   {
      wheelGraphic.setAppearance(new YoAppearanceRGBColor(defaultColor, unselectTransparency));
      
   }

   @Override
   public void addSelectedListeners(SelectableObjectListener selectedListener)
   {
      this.selectedListeners.add(selectedListener);
      
   }
   
   private void notifySelectedListenersThisWasSelected(Object selectedInformation)
   {
      for (SelectableObjectListener listener : selectedListeners)
      {
         listener.wasSelected(this, selectedInformation);
      }
   }

   @Override
   public void setMass(double mass)
   {
      wheelLink.setMass(mass);
   }

   @Override
   public void setMomentOfInertia(double Ixx, double Iyy, double Izz)
   {
      wheelLink.setMomentOfInertia(Ixx, Iyy, Izz);
   }
   
   public void setTau(double desiredTau)
   {
      pinJoint.setTau(desiredTau);
   }
   
   public void setDamping(double desiredDamping)
   {
      pinJoint.setDamping(desiredDamping);
   }

   @Override
   public void getBodyTransformToWorld(RigidBodyTransform transformToWorld)
   {
      transformToWorld.set(pinJointTransformToWorld);
   }
}
