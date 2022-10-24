package us.ihmc.gdx.simulation.environment.object.objects;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class TableDefinition  extends RobotDefinition
{
   private SixDoFJointState initialSixDoFState;

   public TableDefinition()
   {
      super("table");
   }

   public void build()
   {
      RigidBodyDefinition rootBodyDefinition = new RigidBodyDefinition("tableRootBody");

      SixDoFJointDefinition rootJointDefinition = new SixDoFJointDefinition("tableRootJoint");
      rootBodyDefinition.addChildJoint(rootJointDefinition);
      initialSixDoFState = new SixDoFJointState(new YawPitchRoll(), new Point3D());
      initialSixDoFState.setVelocity(new Vector3D(), new Vector3D());
      rootJointDefinition.setInitialJointState(initialSixDoFState);

      double sizeX = 2.0;
      double sizeY = 0.75;
      double sizeZ = 0.05;
      RigidBodyDefinition tableSurface = new RigidBodyDefinition("tableSurface");
      VisualDefinition modelVisualDefinition = new VisualDefinition();
      ModelFileGeometryDefinition geometryDefinition = new ModelFileGeometryDefinition("environmentObjects/table/Table.g3dj");
      MaterialDefinition materialDefinition = new MaterialDefinition(ColorDefinitions.Brown());
      modelVisualDefinition.setGeometryDefinition(geometryDefinition);
      modelVisualDefinition.setMaterialDefinition(materialDefinition);
      tableSurface.addVisualDefinition(modelVisualDefinition);

      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(new Box3DDefinition(sizeX, sizeY, sizeZ));
      collisionShapeDefinition.getOriginPose().getTranslation().set(sizeX / 2.0, sizeY / 2.0, sizeZ / 2.0);
      tableSurface.addCollisionShapeDefinition(collisionShapeDefinition);

      CollisionShapeDefinition leg1 = new CollisionShapeDefinition(new Box3DDefinition(0.05, 0.05, 0.85));
      leg1.getOriginPose().getTranslation().set(0.075, 0.075, -0.85 / 2.0);
      tableSurface.addCollisionShapeDefinition(leg1);

      CollisionShapeDefinition leg2 = new CollisionShapeDefinition(new Box3DDefinition(0.05, 0.05, 0.85));
      leg2.getOriginPose().getTranslation().set(sizeX - 0.075, 0.075, -0.85 / 2.0);
      tableSurface.addCollisionShapeDefinition(leg2);

      CollisionShapeDefinition leg3 = new CollisionShapeDefinition(new Box3DDefinition(0.05, 0.05, 0.85));
      leg3.getOriginPose().getTranslation().set(sizeX - 0.075, sizeY - 0.075, -0.85 / 2.0);
      tableSurface.addCollisionShapeDefinition(leg3);

      CollisionShapeDefinition leg4 = new CollisionShapeDefinition(new Box3DDefinition(0.05, 0.05, 0.85));
      leg4.getOriginPose().getTranslation().set(0.075, sizeY - 0.075, -0.85 / 2.0);
      tableSurface.addCollisionShapeDefinition(leg4);

      tableSurface.setMass(50.0);
      double radiusOfGyrationPercent = 0.8;
      tableSurface.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(tableSurface.getMass(),
                                                                           radiusOfGyrationPercent * sizeX,
                                                                           radiusOfGyrationPercent * sizeY,
                                                                           radiusOfGyrationPercent * sizeZ));
      Point3D centerOfMassOffset = new Point3D(sizeX / 2.0, sizeY / 2.0, sizeZ / 2.0);
      tableSurface.getInertiaPose().getTranslation().set(centerOfMassOffset);
      tableSurface.getInertiaPose().getRotation().setToZero();
      rootJointDefinition.setSuccessor(tableSurface);

      setRootBodyDefinition(rootBodyDefinition);
   }

   public SixDoFJointState getInitialSixDoFState()
   {
      return initialSixDoFState;
   }
}
