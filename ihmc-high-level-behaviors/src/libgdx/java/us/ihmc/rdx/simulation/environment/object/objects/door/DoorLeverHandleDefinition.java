package us.ihmc.rdx.simulation.environment.object.objects.door;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class DoorLeverHandleDefinition extends RigidBodyDefinition
{
   public DoorLeverHandleDefinition()
   {
      super("doorLeverHandle");

      double sizeX = 0.065;
      double sizeY = 0.14;
      double sizeZ = 0.065;
      Point3D centerOfMassOffset = new Point3D(-sizeX / 2.0, -0.04, 0.0);

      double mass = 0.7;
      setMass(mass);
      double radiusOfGyrationPercent = 0.8;
      setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(getMass(),
                                                                           radiusOfGyrationPercent * sizeX,
                                                                           radiusOfGyrationPercent * sizeY,
                                                                           radiusOfGyrationPercent * sizeZ));

      getInertiaPose().getTranslation().set(centerOfMassOffset);
      getInertiaPose().getRotation().setToZero();

      VisualDefinition modelVisualDefinition = new VisualDefinition();
      ModelFileGeometryDefinition geometryDefinition = new ModelFileGeometryDefinition("environmentObjects/door/doorLeverHandle/DoorLeverHandle.g3dj");
      modelVisualDefinition.setGeometryDefinition(geometryDefinition);
      addVisualDefinition(modelVisualDefinition);

      VisualDefinition modelVisualDefinitionOtherSide = new VisualDefinition();
      ModelFileGeometryDefinition geometryDefinitionOtherSide = new ModelFileGeometryDefinition("environmentObjects/door/doorLeverHandle/DoorLeverHandle.g3dj");
      modelVisualDefinitionOtherSide.setGeometryDefinition(geometryDefinitionOtherSide);
      modelVisualDefinitionOtherSide.getOriginPose().prependPitchRotation(Math.PI);
      modelVisualDefinitionOtherSide.getOriginPose().getTranslation().setX(0.0508);
      addVisualDefinition(modelVisualDefinitionOtherSide);

      // This code enables the full mesh collisions, but they aren't super useful, and they're slow
////      Point3D collisionShapeOffset = new Point3D(0.0, sizeY / 2.0 + 0.025, sizeZ / 2.0);
//      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition();
//      ModelFileGeometryDefinition collisionShapeGeometryDefinition
//            = new ModelFileGeometryDefinition("environmentObjects/door/doorLeverHandle/DoorLeverHandle.stl");
//      collisionShapeDefinition.setGeometryDefinition(collisionShapeGeometryDefinition);
//      collisionShapeDefinition.setConcave(true);
//      collisionShapeDefinition.getOriginPose().set(new YawPitchRoll(0.0, 0.0, -Math.toRadians(90.0)), new Point3D());
//      addCollisionShapeDefinition(collisionShapeDefinition);

      CollisionShapeDefinition cylinderCollisionShapeDefinition = new CollisionShapeDefinition();
      double length = 0.12528;
      Cylinder3DDefinition cylinderDefinition = new Cylinder3DDefinition(length, 0.012);
      cylinderCollisionShapeDefinition.setGeometryDefinition(cylinderDefinition);
      cylinderCollisionShapeDefinition.getOriginPose().set(new YawPitchRoll(0.0, 0.0, -Math.toRadians(90.0)), new Point3D(-0.045, -length / 2.0, 0.0));
      addCollisionShapeDefinition(cylinderCollisionShapeDefinition);

      CollisionShapeDefinition cylinderCollisionShapeDefinition2 = new CollisionShapeDefinition();
      cylinderCollisionShapeDefinition2.setGeometryDefinition(cylinderDefinition);
      cylinderCollisionShapeDefinition2.getOriginPose().set(new YawPitchRoll(0.0, 0.0, -Math.toRadians(90.0)),
                                                            new Point3D(DoorDefinition.DOOR_PANEL_THICKNESS + 0.045, -length / 2.0, 0.0));
      addCollisionShapeDefinition(cylinderCollisionShapeDefinition2);
   }
}
