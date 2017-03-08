package us.ihmc.simulationconstructionset.util.environments;

import java.awt.Color;
import java.util.ArrayList;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.input.SelectedListener;
import us.ihmc.graphicsDescription.instructions.CylinderGraphics3DInstruction;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.shapes.FrameCylinder3d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.GroundContactPointGroup;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.RigidJoint;
import us.ihmc.tools.inputDevices.keyboard.Key;
import us.ihmc.tools.inputDevices.keyboard.ModifierKeyInterface;

public class ContactableStaticCylinderRobot extends ContactableStaticRobot implements SelectableObject, SelectedListener
{
   private static final double DEFAULT_MASS = 1000000.0;

   private final FrameCylinder3d cylinder;
   
   private final RigidBodyTransform cylinderCenterTransformToWorld = new RigidBodyTransform();
   
   private final RigidJoint nullJoint;
   private final Link cylinderLink;
   private final Graphics3DObject linkGraphics;
   
   private static final Color defaultColor = Color.BLACK;
   private static final Color selectedColor = Color.CYAN;
   
   private final double selectTransparency = 0.0;
   private final double unselectTransparency = 0.0;
   
   private CylinderGraphics3DInstruction cylinderGraphic;
   
   private final ArrayList<SelectableObjectListener> selectedListeners = new ArrayList<SelectableObjectListener>();

   public ContactableStaticCylinderRobot(String name, RigidBodyTransform cylinderTransform, double cylinderHeight, double cylinderRadius, AppearanceDefinition appearance)
   {
      super(name);
      cylinder = new FrameCylinder3d(ReferenceFrame.getWorldFrame(), cylinderTransform, cylinderHeight, cylinderRadius);
      
      RotationMatrix rotation = new RotationMatrix();
      Vector3D offset = new Vector3D();
      cylinderTransform.getTranslation(offset);
      cylinderTransform.getRotation(rotation);
      
      FramePoint cylinderCenter = new FramePoint(new TransformReferenceFrame("cylinderCenter", ReferenceFrame.getWorldFrame(), cylinderTransform), 0.0, 0.0, cylinderHeight / 2.0 );
      cylinderCenter.changeFrame(ReferenceFrame.getWorldFrame());
      cylinderCenterTransformToWorld.set(rotation, cylinderCenter.getVectorCopy());
      
      Vector3D axis = new Vector3D(0.0, 0.0, 1.0);
      RigidBodyTransform rotationTransform = new RigidBodyTransform();
      rotationTransform.setRotation(rotation);
      rotationTransform.transform(axis);
      
      nullJoint = new RigidJoint(name + "NullJoint", offset, this);

      cylinderLink = new Link(name + "Link");
      cylinderLink.setMassAndRadiiOfGyration(DEFAULT_MASS, 1.0, 1.0, 1.0);
      cylinderLink.setComOffset(new Vector3D());
      
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
   
   public void addYoGraphicForceVectorsToGroundContactPoints(double forceVectorScale, AppearanceDefinition appearance, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      addYoGraphicForceVectorsToGroundContactPoints(1, forceVectorScale, appearance, yoGraphicsListRegistry);
   }
   
   public void addYoGraphicForceVectorsToGroundContactPoints(int groupIdentifier, double forceVectorScale, AppearanceDefinition appearance, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (yoGraphicsListRegistry == null) return;
      
      GroundContactPointGroup groundContactPointGroup = nullJoint.getGroundContactPointGroup(groupIdentifier);
      System.out.println("GroundContactPointGroup" + groundContactPointGroup.getGroundContactPoints());
      ArrayList<GroundContactPoint> groundContactPoints = groundContactPointGroup.getGroundContactPoints();
      
      for (GroundContactPoint groundContactPoint : groundContactPoints)
      {
         YoGraphicVector yoGraphicVector = new YoGraphicVector(groundContactPoint.getName(), groundContactPoint.getYoPosition(), groundContactPoint.getYoForce(), forceVectorScale, appearance);
         yoGraphicsListRegistry.registerYoGraphic("ContactableToroidRobot", yoGraphicVector);
      }
   }
   
   @Override
   public RigidJoint getNullJoint()
   {
      return nullJoint;
   }
   
   @Override
   public void getBodyTransformToWorld(RigidBodyTransform transformToWorld)
   {
      transformToWorld.set(cylinderCenterTransformToWorld);
   }

   @Override
   public synchronized boolean isPointOnOrInside(Point3D pointInWorldToCheck)
   {
      return cylinder.getCylinder3d().isInsideOrOnSurface(pointInWorldToCheck);
   }
   
   @Override
   public boolean isClose(Point3D pointInWorldToCheck)
   {
      return isPointOnOrInside(pointInWorldToCheck);
   }

   @Override
   public synchronized void closestIntersectionAndNormalAt(Point3D intersectionToPack, Vector3D normalToPack, Point3D pointInWorldToCheck)
   {
      cylinder.getCylinder3d().checkIfInside(pointInWorldToCheck, intersectionToPack, normalToPack);
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
      
      cylinderGraphic.setAppearance(new YoAppearanceRGBColor(selectedColor, selectTransparency));
      
      notifySelectedListenersThisWasSelected(this);
   }

   @Override
   public void unSelect(boolean reset)
   {
      cylinderGraphic.setAppearance(new YoAppearanceRGBColor(defaultColor, unselectTransparency));
      
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

}
