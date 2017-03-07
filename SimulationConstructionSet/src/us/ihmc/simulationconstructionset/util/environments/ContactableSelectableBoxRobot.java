package us.ihmc.simulationconstructionset.util.environments;

import java.awt.Color;
import java.util.ArrayList;
import java.util.EnumMap;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.input.SelectedListener;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddMeshDataInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DInstruction;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.geometry.shapes.Box3d;
import us.ihmc.robotics.geometry.shapes.Box3d.FaceName;
import us.ihmc.robotics.geometry.shapes.FrameBox3d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.GroundContactPointGroup;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.tools.containers.EnumTools;
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
   private final EnumMap<FaceName, Graphics3DInstruction> faceGraphics = new EnumMap<FaceName, Graphics3DInstruction>(FaceName.class);
   private final EnumYoVariable<Direction> selectedDirection = EnumYoVariable.create("selectedDirection", "", Direction.class, yoVariableRegistry, true);
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

      createBoxGraphics(frameBox);
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

   public void addDynamicGraphicForceVectorsToGroundContactPoints(double forceVectorScale, AppearanceDefinition appearance,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      addDynamicGraphicForceVectorsToGroundContactPoints(0, forceVectorScale, appearance, yoGraphicsListRegistry);
   }

   public void addDynamicGraphicForceVectorsToGroundContactPoints(int groupIdentifier, double forceVectorScale, AppearanceDefinition appearance,
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

      Point3D[] vertices = new Point3D[Box3d.NUM_VERTICES];
      for (int i = 0; i < vertices.length; i++)
      {
         vertices[i] = new Point3D();
      }

      frameBox.getBox3d().computeVertices(vertices);

      for (int i = 0; i < vertices.length; i++)
      {
         Point3D vertex = vertices[i];
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
      return frameBox.getBox3d().isInsideOrOnSurface(pointInWorldToCheck);
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
      frameBox.getBox3d().checkIfInside(pointInWorldToCheck, intersectionToPack, normalToPack);
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
      if (selectedDirection.getEnumValue() == null)
         selectedDirection.set(Direction.Y);
      else
         selectedDirection.set(EnumTools.getNext(selectedDirection.getEnumValue()));

      for (boolean positive : new boolean[] { true, false })
      {
         FaceName faceName = FaceName.get(positive, selectedDirection.getEnumValue());
         faceGraphics.get(faceName).setAppearance(new YoAppearanceRGBColor(selectedColor, selectTransparency));
      }

      notifySelectedListenersThisWasSelected(this);
   }

   @Override
   public void unSelect(boolean reset)
   {
      if (reset)
         selectedDirection.set(null);

      for (FaceName faceName : FaceName.values())
      {
         faceGraphics.get(faceName).setAppearance(new YoAppearanceRGBColor(defaultColor, unselectTransparency));
      }

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

   private void createBoxGraphics(FrameBox3d frameBox)
   {
      for (FaceName faceName : FaceName.values())
      {
         int nVerticesPerFace = Box3d.NUM_VERTICES_PER_FACE;
         Point3D[] vertices = new Point3D[nVerticesPerFace];
         for (int i = 0; i < vertices.length; i++)
         {
            vertices[i] = new Point3D();
         }

         frameBox.getBox3d().computeVertices(vertices, faceName);
         Graphics3DAddMeshDataInstruction faceGraphic = linkGraphics.addPolygon(vertices, YoAppearance.Red());
         faceGraphics.put(faceName, faceGraphic);
      }
   }

   public Direction getSelectedDirection()
   {
      return selectedDirection.getEnumValue();
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
      return frameBox.getBox3d().getHeight() * 0.5;
   }

   @Override
   public void selected(Graphics3DNode graphics3dNode, ModifierKeyInterface modifierKeyHolder, Point3DReadOnly location, Point3DReadOnly cameraLocation, QuaternionReadOnly cameraRotation)
   {
      if (!modifierKeyHolder.isKeyPressed(Key.P))
         return;
      //      System.out.println("Selected box " + this.getName());
      select();
   }
}
