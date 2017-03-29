package us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots;

import java.util.ArrayList;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.geometry.shapes.FrameCylinder3d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.GroundContactPointGroup;
import us.ihmc.simulationconstructionset.Link;

public class ContactableCylinderRobot extends ContactableRobot
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   public static final double DEFAULT_RADIUS = 0.15;
   public static final double DEFAULT_THICKNESS = 0.05;
   private static final double DEFAULT_MASS = 1.0;

   private final RigidBodyTransform rootJointTransformToWorld;
   private final FrameCylinder3d frameCylinder;

   private final FloatingJoint floatingJoint;
   private final ReferenceFrame afterRootJointFrame;

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
      
      afterRootJointFrame = new ReferenceFrame("rootJointFrame", worldFrame)
      {
         private static final long serialVersionUID = -5359633108342066963L;

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            floatingJoint.getTransformToWorld(transformToParent);
         }
      };
      frameCylinder = new FrameCylinder3d(afterRootJointFrame, height, radius);


      link = new Link(name + "Link");
      link.setMass(mass);
      link.setComOffset(new Vector3D(0.0, 0.0, height / 3.0));

      Matrix3D inertia = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(mass, radius, height, Axis.Z);
      link.setMomentOfInertia(inertia);

      linkGraphics = new Graphics3DObject();
      //linkGraphics.addCoordinateSystem(0.2);
      linkGraphics.addCylinder(frameCylinder.getHeight(), frameCylinder.getRadius(), YoAppearance.Transparent());
      if(modelName!=null)
         linkGraphics.addModelFile(modelName);
      link.setLinkGraphics(linkGraphics);

      floatingJoint = new FloatingJoint(name + "Base", name, new Vector3D(), this);
      floatingJoint.setRotationAndTranslation(rootJointTransform);
      floatingJoint.setLink(link);
      this.addRootJoint(floatingJoint);
   }
   

   public void addYoGraphicForceVectorsToGroundContactPoints(double forceVectorScale, AppearanceDefinition appearance,
           YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      addYoGraphicForceVectorsToGroundContactPoints(1, forceVectorScale, appearance, yoGraphicsListRegistry);
   }

   public void addYoGraphicForceVectorsToGroundContactPoints(int groupIdentifier, double forceVectorScale, AppearanceDefinition appearance,
           YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (yoGraphicsListRegistry == null)
         return;

      GroundContactPointGroup groundContactPointGroup = floatingJoint.getGroundContactPointGroup(groupIdentifier);
      System.out.println("GroundContactPointGroup" + groundContactPointGroup.getGroundContactPoints());
      ArrayList<GroundContactPoint> groundContactPoints = groundContactPointGroup.getGroundContactPoints();

      for (GroundContactPoint groundContactPoint : groundContactPoints)
      {
         YoGraphicVector yoGraphicVector = new YoGraphicVector(groundContactPoint.getName(), groundContactPoint.getYoPosition(),
                                                   groundContactPoint.getYoForce(), forceVectorScale, appearance);
         yoGraphicsListRegistry.registerYoGraphic("ContactableToroidRobot", yoGraphicVector);
      }
   }

   @Override
   public synchronized boolean isPointOnOrInside(Point3D pointInWorldToCheck)
   {
      afterRootJointFrame.update();
      frameCylinder.changeFrame(worldFrame);
      boolean insideOrOnSurface = frameCylinder.getCylinder3d().isInsideOrOnSurface(pointInWorldToCheck);
      frameCylinder.changeFrame(afterRootJointFrame);
      return insideOrOnSurface;
   }

   @Override
   public boolean isClose(Point3D pointInWorldToCheck)
   {
      return isPointOnOrInside(pointInWorldToCheck);
   }

   @Override
   public synchronized void closestIntersectionAndNormalAt(Point3D intersectionToPack, Vector3D normalToPack, Point3D pointInWorldToCheck)
   {
      afterRootJointFrame.update();
      frameCylinder.changeFrame(worldFrame);
      frameCylinder.getCylinder3d().checkIfInside(pointInWorldToCheck, intersectionToPack, normalToPack);
      frameCylinder.changeFrame(afterRootJointFrame);
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
