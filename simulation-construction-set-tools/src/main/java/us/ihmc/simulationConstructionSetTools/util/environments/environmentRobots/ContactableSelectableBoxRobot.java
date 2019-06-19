package us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots;

import java.awt.Color;
import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.input.SelectedListener;
import us.ihmc.graphicsDescription.instructions.Graphics3DInstruction;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.geometry.shapes.FrameBox3d;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObject;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.GroundContactPointGroup;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.tools.inputDevices.keyboard.Key;
import us.ihmc.tools.inputDevices.keyboard.ModifierKeyInterface;

public class ContactableSelectableBoxRobot extends ContactableRobot implements SelectableObject, SelectedListener
{
   private static final double DEFAULT_LENGTH = 1.0;
   private static final double DEFAULT_WIDTH = 0.6;
   private static final double DEFAULT_HEIGHT = 1.2;

   private static final double DEFAULT_MASS = 10.0;

   private final FrameBox3d frameBox;

   private final FloatingJoint floatingJoint;
   private final Link boxLink;
   private final Graphics3DObject linkGraphics;

   // graphics
   private final Graphics3DInstruction boxGraphics;
   private static final Color defaultColor = Color.BLUE;
   private static final Color selectedColor = Color.RED;

   private double selectTransparency = 1.0;
   private double unselectTransparency = 1.0;

   private final ArrayList<SelectableObjectListener> selectedListeners = new ArrayList<SelectableObjectListener>();

   public ContactableSelectableBoxRobot(String name, double length, double width, double height, double mass)
   {
      super(name);

      floatingJoint = new FloatingJoint(name + "Base", name, new Vector3D(0.0, 0.0, 0.0), this);
      linkGraphics = new Graphics3DObject();
      linkGraphics.setChangeable(true);
      boxLink = boxLink(linkGraphics, length, width, height, mass);
      floatingJoint.setLink(boxLink);
      this.addRootJoint(floatingJoint);

      frameBox = new FrameBox3d(ReferenceFrame.getWorldFrame(), length, width, height);

      Box3D box = frameBox.getBox3d();
      boxGraphics = linkGraphics.addCube(box.getSizeX(), box.getSizeY(), box.getSizeZ());
      setUpGroundContactPoints(frameBox);

      unSelect(true);

      linkGraphics.registerSelectedListener(this);
   }

   public static ContactableSelectableBoxRobot createContactableCardboardBoxRobot(String name, double length, double width, double height, double mass)
   {
      ContactableSelectableBoxRobot contactableBoxRobot = new ContactableSelectableBoxRobot(name, length, width, height, mass);
      contactableBoxRobot.createCardboardBoxGraphics(length, width, height);

      initializeDefaults(contactableBoxRobot);

      return contactableBoxRobot;
   }

   public static ContactableSelectableBoxRobot createContactableWoodBoxRobot(String name, double length, double width, double height, double mass)
   {
      ContactableSelectableBoxRobot contactableBoxRobot = new ContactableSelectableBoxRobot(name, length, width, height, mass);
      contactableBoxRobot.createWoodBoxGraphics(length, width, height);

      initializeDefaults(contactableBoxRobot);

      return contactableBoxRobot;
   }

   public static ContactableSelectableBoxRobot createContactable2By4Robot(String name, double length, double width, double height, double mass)
   {
      ContactableSelectableBoxRobot contactableBoxRobot = new ContactableSelectableBoxRobot(name, length, width, height, mass);
      contactableBoxRobot.create2By4Graphics(length, width, height);

      initializeDefaults(contactableBoxRobot);

      return contactableBoxRobot;
   }

   private static void initializeDefaults(ContactableSelectableBoxRobot contactableBoxRobot)
   {
      contactableBoxRobot.selectTransparency = 0.1;
      contactableBoxRobot.unselectTransparency = 0.9;

      contactableBoxRobot.unSelect(true);
   }

   public ContactableSelectableBoxRobot()
   {
      this("ContactableBoxRobot");
   }

   public ContactableSelectableBoxRobot(String name)
   {
      this(name, DEFAULT_LENGTH, DEFAULT_WIDTH, DEFAULT_HEIGHT, DEFAULT_MASS);
   }

   public void addYoGraphicForceVectorsToGroundContactPoints(double forceVectorScale, AppearanceDefinition appearance,
                                                             YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      addYoGraphicForceVectorsToGroundContactPoints(0, forceVectorScale, appearance, yoGraphicsListRegistry);
   }

   public void addYoGraphicForceVectorsToGroundContactPoints(int groupIdentifier, double forceVectorScale, AppearanceDefinition appearance,
                                                             YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (yoGraphicsListRegistry == null)
         return;

      GroundContactPointGroup groundContactPointGroup = floatingJoint.getGroundContactPointGroup(groupIdentifier);
      ArrayList<GroundContactPoint> groundContactPoints = groundContactPointGroup.getGroundContactPoints();

      for (GroundContactPoint groundContactPoint : groundContactPoints)
      {
         YoGraphicVector yoGraphicVector = new YoGraphicVector(groundContactPoint.getName(), groundContactPoint.getYoPosition(),
                                                               groundContactPoint.getYoForce(), forceVectorScale, appearance);
         yoGraphicsListRegistry.registerYoGraphic("ContactableSelectableBoxRobot", yoGraphicVector);
      }
   }

   private void setUpGroundContactPoints(FrameBox3d frameBox)
   {
      String name = this.getName();

      for (int i = 0; i < 8; i++)
      {
         Point3D vertex = new Point3D();
         frameBox.getBox3d().getVertex(i, vertex);
         GroundContactPoint groundContactPoint = new GroundContactPoint("gc_" + name + i, new Vector3D(vertex), this.getRobotsYoVariableRegistry());

         floatingJoint.addGroundContactPoint(groundContactPoint);
      }
   }

   @Override
   public FloatingJoint getFloatingJoint()
   {
      return floatingJoint;
   }

   private Link boxLink(Graphics3DObject linkGraphics, double length, double width, double height, double mass)
   {
      Link ret = new Link("box");

      ret.setMass(mass);

      ret.setMomentOfInertia(RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidBox(length, width, height, mass));
      ret.setComOffset(0.0, 0.0, 0.0);

      //      linkGraphics.translate(0.0, 0.0, -height / 2.0);
      //      linkGraphics.addCube(length, width, height, YoAppearance.EarthTexture(null));
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   @Override
   public synchronized boolean isPointOnOrInside(Point3D pointInWorldToCheck)
   {
      return frameBox.getBox3d().isPointInside(pointInWorldToCheck);
   }

   public synchronized void getCurrentBox3d(FrameBox3d frameBoxToPack)
   {
      frameBoxToPack.setIncludingFrame(frameBox);
   }

   @Override
   public boolean isClose(Point3D pointInWorldToCheck)
   {
      return isPointOnOrInside(pointInWorldToCheck);
   }

   @Override
   public synchronized void closestIntersectionAndNormalAt(Point3D intersectionToPack, Vector3D normalToPack, Point3D pointInWorldToCheck)
   {
      frameBox.getBox3d().evaluatePoint3DCollision(pointInWorldToCheck, intersectionToPack, normalToPack);
   }

   @Override
   public void setMass(double mass)
   {
      boxLink.setMass(mass);
   }

   @Override
   public void setMomentOfInertia(double Ixx, double Iyy, double Izz)
   {
      boxLink.setMomentOfInertia(Ixx, Iyy, Izz);
   }

   @Override
   public void select()
   {
      unSelect(false);
      boxGraphics.setAppearance(new YoAppearanceRGBColor(selectedColor, selectTransparency));

      notifySelectedListenersThisWasSelected(this);
   }

   @Override
   public void unSelect(boolean reset)
   {
      boxGraphics.setAppearance(new YoAppearanceRGBColor(defaultColor, unselectTransparency));
   }

   @Override
   public void addSelectedListeners(SelectableObjectListener listener)
   {
      selectedListeners.add(listener);
   }

   private void notifySelectedListenersThisWasSelected(Object selectedInformation)
   {
      for (SelectableObjectListener listener : selectedListeners)
      {
         listener.wasSelected(this, selectedInformation);
      }
   }

   private void createCardboardBoxGraphics(double sizeX, double sizeY, double sizeZ)
   {
      add1x13DObject("models/cardboardBox.obj", sizeX, sizeY, sizeZ);
   }

   private void createWoodBoxGraphics(double sizeX, double sizeY, double sizeZ)
   {
      add1x13DObject("models/woodBox2.obj", sizeX, sizeY, sizeZ);
   }

   // TODO Create some graphics for the 2-by-4 debris
   private void create2By4Graphics(double sizeX, double sizeY, double sizeZ)
   {
      add1x13DObject("models/woodBox2.obj", sizeX, sizeY, sizeZ);
   }

   protected void add1x13DObject(String fileName, double length, double width, double height)
   {
      Graphics3DObject graphics = new Graphics3DObject();
      graphics.translate(0.0, 0.0, -height / 2.0); // TODO: Center the 3ds files so we don't have to do this translate.
      graphics.scale(new Vector3D(length, width, height));
      graphics.addModelFile(fileName);

      linkGraphics.combine(graphics);
   }

   @Override
   public void update()
   {
      super.update();
      updateCurrentBox3d();
   }

   private final RigidBodyTransform temporaryTransform3D = new RigidBodyTransform();

   private synchronized void updateCurrentBox3d()
   {
      floatingJoint.getTransformToWorld(temporaryTransform3D);
      frameBox.setTransform(temporaryTransform3D);
   }

   public double getHalfHeight()
   {
      return frameBox.getBox3d().getSizeX() * 0.5;
   }

   @Override
   public void selected(Graphics3DNode graphics3dNode, ModifierKeyInterface modifierKeyHolder, Point3DReadOnly location, Point3DReadOnly cameraLocation,
                        QuaternionReadOnly cameraRotation)
   {
      if (!modifierKeyHolder.isKeyPressed(Key.P))
         return;
      //      System.out.println("Selected box " + this.getName());
      select();
   }
}
