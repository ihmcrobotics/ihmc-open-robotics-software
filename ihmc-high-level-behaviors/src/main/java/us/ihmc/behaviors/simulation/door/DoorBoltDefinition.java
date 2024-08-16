package us.ihmc.behaviors.simulation.door;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Ramp3DDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class DoorBoltDefinition extends RigidBodyDefinition
{
   public DoorBoltDefinition()
   {
      super("doorBolt");

      double sizeX = 0.015;
      double sizeY = 0.015;
      double sizeZ = 0.015;
      Point3D centerOfMassOffset = new Point3D(sizeX / 2.0, sizeY / 2.0, sizeZ / 2.0);

      double mass = 0.2;
      setMass(mass);
      double radiusOfGyrationPercent = 0.8;
      setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(getMass(),
                                                                           radiusOfGyrationPercent * sizeX,
                                                                           radiusOfGyrationPercent * sizeY,
                                                                           radiusOfGyrationPercent * sizeZ));

      getInertiaPose().getTranslation().set(centerOfMassOffset);
      getInertiaPose().getRotation().setToZero();

      double rampUpX = sizeY;
      double rampWidth = sizeZ;
      double rampHeight = sizeX;
      Ramp3DDefinition geometryDefinition = new Ramp3DDefinition(rampUpX, rampWidth, rampHeight);

      YawPitchRoll yawPitchRoll = new YawPitchRoll(-Math.PI / 2.0, 0.0, -Math.PI / 2.0);

      VisualDefinition visualDefinition = new VisualDefinition();
      visualDefinition.setGeometryDefinition(geometryDefinition);
      visualDefinition.setMaterialDefinition(new MaterialDefinition(ColorDefinitions.Gray()));
      visualDefinition.getOriginPose().appendOrientation(yawPitchRoll);
      addVisualDefinition(visualDefinition);

      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition();
      collisionShapeDefinition.setGeometryDefinition(geometryDefinition);
      collisionShapeDefinition.getOriginPose().appendOrientation(yawPitchRoll);
      addCollisionShapeDefinition(collisionShapeDefinition);
   }
}
