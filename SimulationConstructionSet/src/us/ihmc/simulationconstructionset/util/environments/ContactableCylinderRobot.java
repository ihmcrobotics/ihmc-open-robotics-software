package us.ihmc.simulationconstructionset.util.environments;

import java.util.ArrayList;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.GroundContactPointGroup;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.math.RotationalInertiaCalculator;
import us.ihmc.utilities.math.geometry.FrameCylinder3d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.screwTheory.RigidBodyInertia;
import us.ihmc.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class ContactableCylinderRobot extends ContactableRobot
{
   public static final double DEFAULT_RADIUS = 0.2;
   public static final double DEFAULT_THICKNESS = 0.05;
   private static final double DEFAULT_MASS = 1.0;

   private final RigidBodyTransform rootJointTransformToWorld;
   private final FrameCylinder3d frameCylinder;

   private final FloatingJoint floatingJoint;

   private final Link link;
   private final Graphics3DObject linkGraphics;

   public ContactableCylinderRobot(String name, RigidBodyTransform rootJointTransformFromWorld)
   {
      this(name, rootJointTransformFromWorld, DEFAULT_RADIUS, DEFAULT_THICKNESS, DEFAULT_MASS);
   }

   public ContactableCylinderRobot(String name, RigidBodyTransform rootJointTransform, double radius, double height, double mass)
   {
      
      this(name,rootJointTransform,radius,height,mass,null);
   }
   public ContactableCylinderRobot(String name, RigidBodyTransform rootJointTransform, double radius, double height, double mass, String modelName)
   {
      super(name);
      rootJointTransformToWorld = rootJointTransform;
      frameCylinder = new FrameCylinder3d(ReferenceFrame.getWorldFrame(), rootJointTransform, height, radius);


      link = new Link(name + "Link");
      link.setMass(mass);
      link.setComOffset(new Vector3d());

      Matrix3d inertia = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(mass, radius, height, Axis.Z);
      RigidBodyInertia rigidBodyInertia = new RigidBodyInertia(ReferenceFrame.getWorldFrame(), inertia, mass);
      ReferenceFrame jointFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent(name + "Frame", ReferenceFrame.getWorldFrame(),
                                     rootJointTransform);
      rigidBodyInertia.changeFrame(jointFrame);
      link.setMomentOfInertia(rigidBodyInertia.getMassMomentOfInertiaPartCopy());

      linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.2);
      linkGraphics.addCylinder(frameCylinder.getHeight(), frameCylinder.getRadius(), YoAppearance.Gold());
      if(modelName!=null)
         linkGraphics.addModelFile(modelName);
      link.setLinkGraphics(linkGraphics);

      floatingJoint = new FloatingJoint(name + "Base", name, new Vector3d(), this);
      floatingJoint.setRotationAndTranslation(rootJointTransform);
      floatingJoint.setLink(link);
      this.addRootJoint(floatingJoint);
   }
   

   public void addDynamicGraphicForceVectorsToGroundContactPoints(double forceVectorScale, AppearanceDefinition appearance,
           YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      addDynamicGraphicForceVectorsToGroundContactPoints(1, forceVectorScale, appearance, yoGraphicsListRegistry);
   }

   public void addDynamicGraphicForceVectorsToGroundContactPoints(int groupIdentifier, double forceVectorScale, AppearanceDefinition appearance,
           YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (yoGraphicsListRegistry == null)
         return;

      GroundContactPointGroup groundContactPointGroup = floatingJoint.physics.getGroundContactPointGroup(groupIdentifier);
      System.out.println("GroundContactPointGroup" + groundContactPointGroup.getGroundContactPoints());
      ArrayList<GroundContactPoint> groundContactPoints = groundContactPointGroup.getGroundContactPoints();

      for (GroundContactPoint groundContactPoint : groundContactPoints)
      {
         YoGraphicVector dynamicGraphicVector = new YoGraphicVector(groundContactPoint.getName(), groundContactPoint.getYoPosition(),
                                                   groundContactPoint.getYoForce(), forceVectorScale, appearance);
         yoGraphicsListRegistry.registerYoGraphic("ContactableToroidRobot", dynamicGraphicVector);
      }
   }



   public synchronized boolean isPointOnOrInside(Point3d pointInWorldToCheck)
   {
      return frameCylinder.getCylinder3d().isInsideOrOnSurface(pointInWorldToCheck);
   }

   public boolean isClose(Point3d pointInWorldToCheck)
   {
      return isPointOnOrInside(pointInWorldToCheck);
   }

   public synchronized void closestIntersectionAndNormalAt(Point3d intersectionToPack, Vector3d normalToPack, Point3d pointInWorldToCheck)
   {
      frameCylinder.getCylinder3d().checkIfInside(pointInWorldToCheck, intersectionToPack, normalToPack);
   }



   @Override
   public void setMass(double mass)
   {
      link.setMass(mass);
   }

   @Override
   public void setMomentOfInertia(double Ixx, double Iyy, double Izz)
   {
      link.setMomentOfInertia(Ixx, Iyy, Izz);
   }


   @Override
   public void getBodyTransformToWorld(RigidBodyTransform transformToWorld)
   {
      transformToWorld.set(rootJointTransformToWorld);
   }

   @Override
   public FloatingJoint getFloatingJoint()
   {
      return floatingJoint;
   }
}
