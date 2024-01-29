package us.ihmc.behaviors.simulation;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.perception.sceneGraph.rigidBody.TableModelParameters;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

/**
 * A table for putting things on for manipulation tasks.
 */
public class TableDefinition  extends RobotDefinition
{
   private SixDoFJointState initialSixDoFState;
   private boolean addArUcoMarkers = false;

   public TableDefinition()
   {
      super("table");
   }

   public void setAddArUcoMarkers(boolean addFiducials)
   {
      this.addArUcoMarkers = addFiducials;
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
      double sizeZ = TableModelParameters.TABLE_THICKNESS;
      RigidBodyDefinition tableSurface = new RigidBodyDefinition("tableSurface");
      VisualDefinition modelVisualDefinition = new VisualDefinition();
      ModelFileGeometryDefinition geometryDefinition = new ModelFileGeometryDefinition("environmentObjects/table/Table.g3dj");
      MaterialDefinition materialDefinition = new MaterialDefinition(ColorDefinitions.Brown());
      modelVisualDefinition.setGeometryDefinition(geometryDefinition);
      modelVisualDefinition.setMaterialDefinition(materialDefinition);
      tableSurface.addVisualDefinition(modelVisualDefinition);

      if (addArUcoMarkers)
      {
         VisualDefinition arUcoMarkerModelVisualDefinition = new VisualDefinition();
         ModelFileGeometryDefinition fiducialGeometryDefinition = new ModelFileGeometryDefinition("environmentObjects/table/TableArUcoMarker.g3dj");
         arUcoMarkerModelVisualDefinition.setGeometryDefinition(fiducialGeometryDefinition);
         tableSurface.addVisualDefinition(arUcoMarkerModelVisualDefinition);
      }

      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(new Box3DDefinition(sizeX, sizeY, sizeZ));
      collisionShapeDefinition.getOriginPose().getTranslation().set(sizeX / 2.0, sizeY / 2.0, sizeZ / 2.0);
      tableSurface.addCollisionShapeDefinition(collisionShapeDefinition);

      CollisionShapeDefinition leg1 = new CollisionShapeDefinition(new Box3DDefinition(TableModelParameters.TABLE_THICKNESS, TableModelParameters.TABLE_THICKNESS, TableModelParameters.TABLE_LEG_LENGTH));
      leg1.getOriginPose().getTranslation().set(0.075, 0.075, -TableModelParameters.TABLE_LEG_LENGTH / 2.0);
      tableSurface.addCollisionShapeDefinition(leg1);

      CollisionShapeDefinition leg2 = new CollisionShapeDefinition(new Box3DDefinition(TableModelParameters.TABLE_THICKNESS, TableModelParameters.TABLE_THICKNESS, TableModelParameters.TABLE_LEG_LENGTH));
      leg2.getOriginPose().getTranslation().set(sizeX - 0.075, 0.075, -TableModelParameters.TABLE_LEG_LENGTH / 2.0);
      tableSurface.addCollisionShapeDefinition(leg2);

      CollisionShapeDefinition leg3 = new CollisionShapeDefinition(new Box3DDefinition(TableModelParameters.TABLE_THICKNESS, TableModelParameters.TABLE_THICKNESS, TableModelParameters.TABLE_LEG_LENGTH));
      leg3.getOriginPose().getTranslation().set(sizeX - 0.075, sizeY - 0.075, -TableModelParameters.TABLE_LEG_LENGTH / 2.0);
      tableSurface.addCollisionShapeDefinition(leg3);

      CollisionShapeDefinition leg4 = new CollisionShapeDefinition(new Box3DDefinition(TableModelParameters.TABLE_THICKNESS, TableModelParameters.TABLE_THICKNESS, TableModelParameters.TABLE_LEG_LENGTH));
      leg4.getOriginPose().getTranslation().set(0.075, sizeY - 0.075, -TableModelParameters.TABLE_LEG_LENGTH / 2.0);
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
