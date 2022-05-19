package us.ihmc.gdx.simulation.environment.object.objects;

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

public class BoxWithFiducial3RobotDefinition extends RobotDefinition
{
   private SixDoFJointState initialSixDoFState;

   public BoxWithFiducial3RobotDefinition()
   {
      super("boxWithFiducial");

      RigidBodyDefinition elevator = new RigidBodyDefinition("boxRootBody");
      SixDoFJointDefinition floatingJoint = new SixDoFJointDefinition("boxRootJoint");

      double size = 0.35;
      RigidBodyDefinition box = new RigidBodyDefinition("Box");
      VisualDefinition modelVisualDefinition = new VisualDefinition();
      ModelFileGeometryDefinition geometryDefinition = new ModelFileGeometryDefinition("environmentObjects/box/box.g3dj");
      modelVisualDefinition.setGeometryDefinition(geometryDefinition);
      box.addVisualDefinition(modelVisualDefinition);

      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(new Box3DDefinition(size, size, size));
      collisionShapeDefinition.getOriginPose().getTranslation().set(size / 2.0, size / 2.0, size / 2.0);
      box.addCollisionShapeDefinition(collisionShapeDefinition);

      box.setMass(0.5);
      double radiusOfGyrationPercent = 0.8;
      box.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(box.getMass(),
                                                                               radiusOfGyrationPercent * size,
                                                                               radiusOfGyrationPercent * size,
                                                                               radiusOfGyrationPercent * size));
      Point3D centerOfMassOffset = new Point3D(size / 2.0, size / 2.0, size / 2.0);
      box.getInertiaPose().getTranslation().set(centerOfMassOffset);
      box.getInertiaPose().getRotation().setToZero();

      setRootBodyDefinition(elevator);
      elevator.addChildJoint(floatingJoint);

      initialSixDoFState = new SixDoFJointState(new YawPitchRoll(), new Point3D());
      initialSixDoFState.setVelocity(new Vector3D(), new Vector3D());
      floatingJoint.setInitialJointState(initialSixDoFState);

      floatingJoint.setSuccessor(box);
   }

   public SixDoFJointState getInitialSixDoFState()
   {
      return initialSixDoFState;
   }
}
