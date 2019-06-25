package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.List;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class ValkyrieEODObstacleCourseEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject3D;

   private static final double wedgeBarrierLength = 24 * 0.0254;
   private static final double wedgeBarrierWength = 42 * 0.0254;
   private static final double wedgeBarrierHeight = 42 * 0.0254;

   private static final double palletLength = 48 * 0.0254;
   private static final double palletWidth = 40 * 0.0254;
   private static final double palletHeight = 6 * 0.0254;

   private static final double tableHeight = 0.8;
   private static final double tableWidth = 0.76;
   private static final double tableThickness = 0.03;

   private static final double dummyVanLength = 5.54;
   private static final double dummyVanWidth = 2.02;
   private static final double dummyVanHeight = 1.43;
   private static final double dummyVanWheelRadius = 0.3;
   private static final double dummyVanWheelDistance = dummyVanLength * 0.7;

   private static final double vesselCartLength = 1.5;
   private static final double vesselCartWheelRadius = dummyVanWheelRadius;
   private static final double vesselCartBodyHeight = 0.3;
   private static final double vesselRadius = 0.89 / 2;

   private static final Point2D doorLocation = new Point2D(4.0, 1.5);
   private static final double dummyDoorHeight = 1.0;
   private static final double dummyDoorHeightOffset = 0.5;
   private static final double dummyDoorWidth = 0.7;
   private static final double dummyDoorThickness = 0.05;
   private static final double dummyDoorKnobLength = 0.15;
   private static final double dummyDoorKnobDiameter = 0.04;
   private RigidBodyTransform knob = new RigidBodyTransform();

   public ValkyrieEODObstacleCourseEnvironment()
   {
      combinedTerrainObject3D = new CombinedTerrainObject3D("MarchDemoObstacleCourse");

      combinedTerrainObject3D.addTerrainObject(setUpGround("ground"));

      combinedTerrainObject3D.addTerrainObject(setUpWedgeBarrier("barrier", new Point2D(1.6, 0.36)));
      combinedTerrainObject3D.addTerrainObject(setUpWedgeBarrier("barrier", new Point2D(1.6, -1.31)));

      combinedTerrainObject3D.addTerrainObject(setUpPallet("pallet", new Point2D(2.51, 0.49)));
      combinedTerrainObject3D.addTerrainObject(setUpPallet("pallet", new Point2D(3.73, -0.52)));
      combinedTerrainObject3D.addTerrainObject(setUpPallet("pallet", new Point2D(3.73, -1.56)));

      combinedTerrainObject3D.addTerrainObject(setUpTable("table", new Point2D(0.5, -2.68), 4.5));
      combinedTerrainObject3D.addTerrainObject(setUpTable("table", new Point2D(6.55, -2.68), 1.5));
      combinedTerrainObject3D.addTerrainObject(setUpTable("table", new Point2D(6.55, -2.68), 1.5));

      combinedTerrainObject3D.addTerrainObject(setUpVessel("vessel", new Point2D(7.0, -0.6)));

      combinedTerrainObject3D.addTerrainObject(setUpVan("van", doorLocation));
   }

   private CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      int lateralDimension = 15;
      int sideDimension = 10;
      double leteralOffset = 2.0;

      double unitLength = 1.0;
      double groundThickness = 0.3;

      YoAppearanceTexture texture = new YoAppearanceTexture("Textures/brick.png");

      for (int i = 0; i < lateralDimension; i++)
      {
         for (int j = 0; j < sideDimension; j++)
         {
            RigidBodyTransform location = new RigidBodyTransform();
            location.appendTranslation(new Vector3D(lateralDimension / 2 * unitLength + leteralOffset, sideDimension / 2 * unitLength, -groundThickness / 2));
            location.appendTranslation(new Vector3D(-unitLength * (i + 0.5) + leteralOffset, -unitLength * (j + 0.5), 0.0));

            RotatableBoxTerrainObject unitBox = new RotatableBoxTerrainObject(new Box3D(location, unitLength, unitLength, groundThickness), texture);
            combinedTerrainObject.addTerrainObject(unitBox);
         }
      }

      return combinedTerrainObject;
   }

   private CombinedTerrainObject3D setUpWedgeBarrier(String name, Point2D location)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);
      AppearanceDefinition appearance = YoAppearance.Orange();

      combinedTerrainObject.addRamp(location.getX() - wedgeBarrierLength, location.getY() - wedgeBarrierWength / 2, location.getX(),
                                    location.getY() + wedgeBarrierWength / 2, wedgeBarrierHeight, appearance);

      return combinedTerrainObject;
   }

   private CombinedTerrainObject3D setUpPallet(String name, Point2D location)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);
      AppearanceDefinition appearance = YoAppearance.DarkGray();

      combinedTerrainObject.addBox(location.getX() - palletLength / 2, location.getY() - palletWidth / 2, location.getX() + palletLength / 2,
                                   location.getY() + palletWidth / 2, palletHeight, appearance);

      return combinedTerrainObject;
   }

   private CombinedTerrainObject3D setUpTable(String name, Point2D location, double length)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      combinedTerrainObject.addTable(location.getX() - length / 2, location.getY() - tableWidth / 2, location.getX() + length / 2,
                                     location.getY() + tableWidth / 2, tableHeight - tableThickness, tableHeight);

      return combinedTerrainObject;
   }

   private CombinedTerrainObject3D setUpVan(String name, Point2D doorLocation)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      combinedTerrainObject.addBox(doorLocation.getX() - dummyVanLength / 2, doorLocation.getY(), doorLocation.getX() + dummyVanLength / 2,
                                   doorLocation.getY() + dummyVanWidth, dummyVanWheelRadius, dummyVanWheelRadius + dummyVanHeight, YoAppearance.Gray());

      RigidBodyTransform frontWheel = new RigidBodyTransform();
      RigidBodyTransform rearWheel = new RigidBodyTransform();

      frontWheel.setTranslation(doorLocation.getX() + dummyVanWheelDistance / 2, doorLocation.getY() + dummyVanWidth / 2, dummyVanWheelRadius);
      rearWheel.setTranslation(doorLocation.getX() - dummyVanWheelDistance / 2, doorLocation.getY() + dummyVanWidth / 2, dummyVanWheelRadius);

      frontWheel.appendRollRotation(Math.PI / 2);
      rearWheel.appendRollRotation(Math.PI / 2);

      double offset = 0.05;
      combinedTerrainObject.addCylinder(frontWheel, dummyVanWidth - offset, dummyVanWheelRadius, YoAppearance.DarkGray());
      combinedTerrainObject.addCylinder(rearWheel, dummyVanWidth - offset, dummyVanWheelRadius, YoAppearance.DarkGray());

      combinedTerrainObject.addBox(doorLocation.getX(), doorLocation.getY() - dummyDoorThickness, doorLocation.getX() + dummyDoorWidth, doorLocation.getY(),
                                   dummyDoorHeightOffset, dummyDoorHeightOffset + dummyDoorHeight, YoAppearance.DarkGray());

      knob.appendTranslation(doorLocation.getX(), doorLocation.getY(), dummyDoorHeightOffset + dummyDoorHeight / 2);
      knob.appendTranslation(dummyDoorKnobLength / 2 + dummyDoorKnobDiameter, -dummyDoorThickness - dummyDoorKnobDiameter, 0);

      knob.appendPitchRotation(Math.PI / 2);
      combinedTerrainObject.addCylinder(knob, dummyDoorKnobLength, dummyDoorKnobDiameter / 2, YoAppearance.Gray());

      return combinedTerrainObject;
   }

   private CombinedTerrainObject3D setUpVessel(String name, Point2D doorLocation)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      combinedTerrainObject.addSphere(doorLocation.getX(), doorLocation.getY(), vesselCartWheelRadius + vesselCartBodyHeight + vesselRadius, vesselRadius,
                                      YoAppearance.Blue());

      combinedTerrainObject.addBox(doorLocation.getX() - vesselCartLength / 2, doorLocation.getY() - vesselCartLength / 2,
                                   doorLocation.getX() + vesselCartLength / 2, doorLocation.getY() + vesselCartLength / 2, vesselCartWheelRadius,
                                   vesselCartWheelRadius + vesselCartBodyHeight, YoAppearance.Gray());

      RigidBodyTransform frontWheel = new RigidBodyTransform();
      RigidBodyTransform rearWheel = new RigidBodyTransform();

      frontWheel.setTranslation(doorLocation.getX() + vesselCartLength * 0.25, doorLocation.getY(), vesselCartWheelRadius);
      rearWheel.setTranslation(doorLocation.getX() - vesselCartLength * 0.25, doorLocation.getY(), vesselCartWheelRadius);

      frontWheel.appendRollRotation(Math.PI / 2);
      rearWheel.appendRollRotation(Math.PI / 2);

      double offset = 0.05;
      combinedTerrainObject.addCylinder(frontWheel, vesselCartLength - offset, vesselCartWheelRadius, YoAppearance.DarkGray());
      combinedTerrainObject.addCylinder(rearWheel, vesselCartLength - offset, vesselCartWheelRadius, YoAppearance.DarkGray());

      return combinedTerrainObject;
   }

   public Point2D getDoorLocation()
   {
      return doorLocation;
   }

   public Vector3DReadOnly getDoorKnobGraspingPoint()
   {
      return knob.getTranslationVector();
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject3D;
   }

   @Override
   public List<? extends Robot> getEnvironmentRobots()
   {
      return null;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {

   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {

   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {

   }
}
