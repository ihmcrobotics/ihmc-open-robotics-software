package us.ihmc.simulationconstructionset.util.environments;

import java.awt.Color;
import java.util.ArrayList;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.GroundContactPointGroup;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.NullJoint;
import us.ihmc.tools.inputDevices.keyboard.Key;
import us.ihmc.tools.inputDevices.keyboard.ModifierKeyInterface;
import us.ihmc.robotics.geometry.shapes.FrameCylinder3d;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.input.SelectedListener;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddMeshDataInstruction;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;

public class ContactableStaticCylinderRobot extends ContactableStaticRobot implements SelectableObject, SelectedListener
{
   private static final long serialVersionUID = -9222718517284272299L;

   private static final double DEFAULT_MASS = 1000000.0;

   private final FrameCylinder3d cylinder;
   
   private final RigidBodyTransform cylinderCenterTransformToWorld = new RigidBodyTransform();
   
   private final NullJoint nullJoint;
   private final Link cylinderLink;
   private final Graphics3DObject linkGraphics;
   
   private static final Color defaultColor = Color.BLACK;
   private static final Color selectedColor = Color.CYAN;
   
   private final double selectTransparency = 0.0;
   private final double unselectTransparency = 0.0;
   
   private Graphics3DAddMeshDataInstruction cylinderGraphic;
   
   private final ArrayList<SelectableObjectListener> selectedListeners = new ArrayList<SelectableObjectListener>();

   public ContactableStaticCylinderRobot(String name, RigidBodyTransform cylinderTransform, double cylinderHeight, double cylinderRadius, AppearanceDefinition appearance)
   {
      super(name);
      cylinder = new FrameCylinder3d(ReferenceFrame.getWorldFrame(), cylinderTransform, cylinderHeight, cylinderRadius);
      
      Matrix3d rotation = new Matrix3d();
      Vector3d offset = new Vector3d();
      cylinderTransform.getTranslation(offset);
      cylinderTransform.getRotation(rotation);
      
      FramePoint cylinderCenter = new FramePoint(new TransformReferenceFrame("cylinderCenter", ReferenceFrame.getWorldFrame(), cylinderTransform), 0.0, 0.0, cylinderHeight / 2.0 );
      cylinderCenter.changeFrame(ReferenceFrame.getWorldFrame());
      cylinderCenterTransformToWorld.set(rotation, cylinderCenter.getVectorCopy());
      
      Vector3d axis = new Vector3d(0.0, 0.0, 1.0);
      RigidBodyTransform rotationTransform = new RigidBodyTransform();
      rotationTransform.setRotation(rotation);
      rotationTransform.transform(axis);
      
      nullJoint = new NullJoint(name + "NullJoint", offset, this);

      cylinderLink = new Link(name + "Link");
      cylinderLink.setMassAndRadiiOfGyration(DEFAULT_MASS, 1.0, 1.0, 1.0);
      cylinderLink.setComOffset(new Vector3d());
      
      linkGraphics = new Graphics3DObject();
      linkGraphics.rotate(rotation);
      cylinderLink.setLinkGraphics(linkGraphics);

      nullJoint.setLink(cylinderLink);
      this.addRootJoint(nullJoint);
      
      createCylinderGraphics(cylinder, appearance);
      
      unSelect(true);
      
      linkGraphics.registerSelectedListener(this);
   }
   
   private void createCylinderGraphics(FrameCylinder3d cylinder, AppearanceDefinition appearance)
   {
      cylinderGraphic = linkGraphics.addCylinder(cylinder.getHeight(), cylinder.getRadius(), appearance);
   }

   public Link getCylinderLink()
   {
      return cylinderLink;
   }
   
   public void addDynamicGraphicForceVectorsToGroundContactPoints(double forceVectorScale, AppearanceDefinition appearance, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      addDynamicGraphicForceVectorsToGroundContactPoints(1, forceVectorScale, appearance, yoGraphicsListRegistry);
   }
   
   public void addDynamicGraphicForceVectorsToGroundContactPoints(int groupIdentifier, double forceVectorScale, AppearanceDefinition appearance, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (yoGraphicsListRegistry == null) return;
      
      GroundContactPointGroup groundContactPointGroup = nullJoint.getGroundContactPointGroup(groupIdentifier);
      System.out.println("GroundContactPointGroup" + groundContactPointGroup.getGroundContactPoints());
      ArrayList<GroundContactPoint> groundContactPoints = groundContactPointGroup.getGroundContactPoints();
      
      for (GroundContactPoint groundContactPoint : groundContactPoints)
      {
         YoGraphicVector dynamicGraphicVector = new YoGraphicVector(groundContactPoint.getName(), groundContactPoint.getYoPosition(), groundContactPoint.getYoForce(), forceVectorScale, appearance);
         yoGraphicsListRegistry.registerYoGraphic("ContactableToroidRobot", dynamicGraphicVector);
      }
   }
   
   @Override
   public NullJoint getNullJoint()
   {
      return nullJoint;
   }
   
   @Override
   public void getBodyTransformToWorld(RigidBodyTransform transformToWorld)
   {
      transformToWorld.set(cylinderCenterTransformToWorld);
   }

   public synchronized boolean isPointOnOrInside(Point3d pointInWorldToCheck)
   {
      return cylinder.getCylinder3d().isInsideOrOnSurface(pointInWorldToCheck);
   }
   
   public boolean isClose(Point3d pointInWorldToCheck)
   {
      return isPointOnOrInside(pointInWorldToCheck);
   }

   public synchronized void closestIntersectionAndNormalAt(Point3d intersectionToPack, Vector3d normalToPack, Point3d pointInWorldToCheck)
   {
      cylinder.getCylinder3d().checkIfInside(pointInWorldToCheck, intersectionToPack, normalToPack);
   }


   public void selected(Graphics3DNode graphics3dNode, ModifierKeyInterface modifierKeyInterface, Point3d location, Point3d cameraLocation,
         Quat4d cameraRotation)
   {
      if (!modifierKeyInterface.isKeyPressed(Key.N))
         return;
      select();
   }

   public void select()
   {
      unSelect(false);
      
      cylinderGraphic.setAppearance(new YoAppearanceRGBColor(selectedColor, selectTransparency));
      
      notifySelectedListenersThisWasSelected(this);
   }

   public void unSelect(boolean reset)
   {
      cylinderGraphic.setAppearance(new YoAppearanceRGBColor(defaultColor, unselectTransparency));
      
   }

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

}
