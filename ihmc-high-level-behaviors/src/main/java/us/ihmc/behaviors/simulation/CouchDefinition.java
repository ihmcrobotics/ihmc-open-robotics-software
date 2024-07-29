package us.ihmc.behaviors.simulation;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.visual.VisualDefinition;

/**
 * A couch that we have in the lab.
 */
public class CouchDefinition extends RobotDefinition
{
   private SixDoFJointState initialSixDoFState;

   public CouchDefinition(String name)
   {
      super(name);
   }

   public void build()
   {
      RigidBodyDefinition rootBodyDefinition = new RigidBodyDefinition(getName() + "_couchRootBody");

      SixDoFJointDefinition rootJointDefinition = new SixDoFJointDefinition(getName() + "_couchRootJoint");
      rootBodyDefinition.addChildJoint(rootJointDefinition);
      initialSixDoFState = new SixDoFJointState(new YawPitchRoll(), new Point3D());
      initialSixDoFState.setVelocity(new Vector3D(), new Vector3D());
      rootJointDefinition.setInitialJointState(initialSixDoFState);

      double length = 2.0;
      double height = 0.6;
      double depth = 0.86;
      double heightOffGround = 0.2;

      RigidBodyDefinition body = new RigidBodyDefinition(getName() + "_couchBody");
      VisualDefinition modelVisualDefinition = new VisualDefinition();
      ModelFileGeometryDefinition geometryDefinition = new ModelFileGeometryDefinition("environmentObjects/couch/Couch.gltf");
      modelVisualDefinition.setGeometryDefinition(geometryDefinition);
      modelVisualDefinition.getOriginPose().getTranslation().setZ(heightOffGround);
      modelVisualDefinition.getOriginPose().getLinearTransform().setToYawMatrix(-Math.PI / 2.0);
      body.addVisualDefinition(modelVisualDefinition);

      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(new Box3DDefinition(depth, length, height + heightOffGround));
      collisionShapeDefinition.getOriginPose().getTranslation().setX(-depth / 2.0);
      collisionShapeDefinition.getOriginPose().getTranslation().setY(-length / 2.0);
      collisionShapeDefinition.getOriginPose().getTranslation().setZ((heightOffGround + height) / 2.0);
      body.addCollisionShapeDefinition(collisionShapeDefinition);

      body.setMass(40.0);
      double radiusOfGyrationPercent = 0.8;
      body.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(body.getMass(),
                                                                                         radiusOfGyrationPercent * length,
                                                                                         radiusOfGyrationPercent * length,
                                                                                         radiusOfGyrationPercent * length));
      Point3D centerOfMassOffset = new Point3D(-depth / 2.0, -length / 2.0, (heightOffGround + height) / 2.0);
      body.getInertiaPose().getTranslation().set(centerOfMassOffset);
      body.getInertiaPose().getRotation().setToZero();
      rootJointDefinition.setSuccessor(body);

      setRootBodyDefinition(rootBodyDefinition);
   }

   public SixDoFJointState getInitialSixDoFState()
   {
      return initialSixDoFState;
   }
}
