package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import java.util.ArrayList;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.IMUMount;
import us.ihmc.simulationconstructionset.KinematicPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

public class StateEstimatorEvaluatorRobot extends Robot
{
   private static final boolean ADD_ARM_LINKS = true;
   private static final boolean OFFSET_IMU_FRAMES = true;
   private static final boolean ROTATE_IMU_FRAMES = true;
   
   private static final boolean SET_INITIAL_POSITIONS = true;
   private static final boolean SET_INITIAL_VELOCITIES = true;
   
   private static final long serialVersionUID = 2647791981594204134L;
   private final Link bodyLink;
   private final FloatingJoint rootJoint;
   private final ArrayList<IMUMount> imuMounts = new ArrayList<IMUMount>();
   
   private final ArrayList<KinematicPoint> positionPoints = new ArrayList<KinematicPoint>();
   private final ArrayList<KinematicPoint> velocityPoints = new ArrayList<KinematicPoint>();

   private final Vector3D gravitationalAcceleration = new Vector3D(0.0, 0.0, -9.81);
   
   public StateEstimatorEvaluatorRobot()
   {
      super(StateEstimatorEvaluatorRobot.class.getSimpleName());

      rootJoint = new FloatingJoint("root", new Vector3D(), this);

      bodyLink = new Link("body");
      bodyLink.setMassAndRadiiOfGyration(10.0, 0.1, 0.2, 0.3);

      ExternalForcePoint externalForcePoint = new ExternalForcePoint("ef_rootJoint", this.getRobotsYoVariableRegistry());
      rootJoint.addExternalForcePoint(externalForcePoint);

      Graphics3DObject bodyLinkGraphics = new Graphics3DObject();
      bodyLinkGraphics.translate(0.0, 0.0, -0.15);
      bodyLinkGraphics.addCube(0.1, 0.2, 0.3, YoAppearance.Red());
      bodyLink.setLinkGraphics(bodyLinkGraphics);
      rootJoint.setLink(bodyLink);

      RigidBodyTransform imu0Offset = new RigidBodyTransform();
      Vector3D offsetVector0 = new Vector3D();
      imu0Offset.setTranslation(offsetVector0);

      IMUMount imuMount0 = new IMUMount("imuMount0", imu0Offset, this);
      KinematicPoint kinematicPoint0 = new KinematicPoint("kp0", offsetVector0, this.getRobotsYoVariableRegistry());
      rootJoint.addKinematicPoint(kinematicPoint0);
      rootJoint.addIMUMount(imuMount0);

      Vector3D velocityPointOffsetVector0 = new Vector3D(0.0, 0.0, 0.2);
      KinematicPoint positionAndVelocityPoint0 = new KinematicPoint("vp0", velocityPointOffsetVector0, this.getRobotsYoVariableRegistry());
      rootJoint.addKinematicPoint(positionAndVelocityPoint0);
      
      this.addRootJoint(rootJoint);

      if (ADD_ARM_LINKS)
      {
         PinJoint pinJoint1 = new PinJoint("pinJoint1", new Vector3D(), this, Axis.X);
         Link armLink1 = new Link("armLink1");
         armLink1.setMassAndRadiiOfGyration(0.3, 0.1, 0.1, 0.1);
         armLink1.setComOffset(new Vector3D(0.0, 0.0, 0.5));

         Graphics3DObject armLink1Graphics = new Graphics3DObject();

         // armLink1Graphics.rotate(-Math.PI/2.0, Graphics3DObject.X);
         armLink1Graphics.addCylinder(1.0, 0.02, YoAppearance.Green());
         armLink1.setLinkGraphics(armLink1Graphics);
         pinJoint1.setLink(armLink1);

         RigidBodyTransform imu1Offset = new RigidBodyTransform();
         if (ROTATE_IMU_FRAMES)
         {
            imu1Offset.setRotationRollAndZeroTranslation(Math.PI / 7.0);
         }

         Vector3D offsetVector1 = new Vector3D();
         if (OFFSET_IMU_FRAMES)
         {
            offsetVector1.set(0.1, 0.2, 0.3);
         }

         imu1Offset.setTranslation(offsetVector1);

         KinematicPoint kinematicPoint1 = new KinematicPoint("kp1", offsetVector1, this.getRobotsYoVariableRegistry());
         pinJoint1.addKinematicPoint(kinematicPoint1);
         IMUMount imuMount1 = new IMUMount("imuMount1", imu1Offset, this);
         pinJoint1.addIMUMount(imuMount1);

         rootJoint.addJoint(pinJoint1);

         PinJoint pinJoint2 = new PinJoint("pinJoint2", new Vector3D(0.0, 0.0, 1.0), this, Axis.Z);
         Link armLink2 = new Link("armLink2");
         armLink2.setMassAndRadiiOfGyration(0.2, 0.1, 0.1, 0.1);
         armLink2.setComOffset(new Vector3D(0.2, 0.0, 0.0));

         Graphics3DObject armLink2Graphics = new Graphics3DObject();
         armLink2Graphics.rotate(Math.PI / 2.0, Axis.Y);
         armLink2Graphics.addCylinder(1.0, 0.02, YoAppearance.Blue());
         armLink2.setLinkGraphics(armLink2Graphics);
         pinJoint2.setLink(armLink2);

         RigidBodyTransform imu2Offset = new RigidBodyTransform();
         if (ROTATE_IMU_FRAMES)
         {
            imu2Offset.setRotationPitchAndZeroTranslation(Math.PI / 8.0);
         }

         Vector3D offsetVector2 = new Vector3D();
         if (OFFSET_IMU_FRAMES)
         {
            offsetVector2.set(0.1, 0.03, 0.007);
         }

         imu2Offset.setTranslation(offsetVector2);

         KinematicPoint kinematicPoint2 = new KinematicPoint("kp2", offsetVector2, this.getRobotsYoVariableRegistry());
         pinJoint2.addKinematicPoint(kinematicPoint2);
         IMUMount imuMount2 = new IMUMount("imuMount2", imu2Offset, this);
         pinJoint2.addIMUMount(imuMount2);

         Vector3D velocityPointOffsetVector2 = new Vector3D(0.1, 0.2, 0.3);
         KinematicPoint positionAndVelocityPoint2 = new KinematicPoint("vp2", velocityPointOffsetVector2, this.getRobotsYoVariableRegistry());
         pinJoint2.addKinematicPoint(positionAndVelocityPoint2);
         

         pinJoint1.addJoint(pinJoint2);

         if (SET_INITIAL_POSITIONS)
         {
            pinJoint1.setQ(1.2);
            pinJoint2.setQ(0.8);
         }
         
         if (SET_INITIAL_VELOCITIES)
         {
            pinJoint1.setQd(-0.5);
            pinJoint2.setQd(0.77);
         }
         
         // Modify here what sensors are available:
         imuMounts.add(imuMount0);
         imuMounts.add(imuMount1);
         imuMounts.add(imuMount2);

         positionPoints.add(positionAndVelocityPoint0);
         positionPoints.add(positionAndVelocityPoint2);
         
         velocityPoints.add(positionAndVelocityPoint0);
         velocityPoints.add(positionAndVelocityPoint2);
      }

      else
      {
         //TODO: Temp since get an exception if there are no pin joints right now...
         PinJoint pinJoint1 = new PinJoint("pinJoint1", new Vector3D(), this, Axis.X);
         Link armLink1 = new Link("armLink1");
         armLink1.setMassAndRadiiOfGyration(0.001, 0.1, 0.1, 0.1);
         armLink1.setComOffset(new Vector3D(0.0, 0.0, 0.0));
         pinJoint1.setLink(armLink1);
         rootJoint.addJoint(pinJoint1);
         
         imuMounts.add(imuMount0);
         positionPoints.add(positionAndVelocityPoint0);
         velocityPoints.add(positionAndVelocityPoint0);
      }

      this.setGravity(gravitationalAcceleration);

      if (ADD_ARM_LINKS)
      {
         if (SET_INITIAL_POSITIONS)
         {
         rootJoint.setPosition(new Point3D(0.0, 0.0, 0.4));
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.setToRollMatrix(0.6);
         rootJoint.setRotation(rotationMatrix);
         }
         
         if (SET_INITIAL_VELOCITIES)
         {
            rootJoint.setAngularVelocityInBody(new Vector3D(0.1, 0.2, 0.07));
            //         rootJoint.setAngularVelocityInBody(new Vector3d(0.2, 2.2, 0.3));
         }
      }

      else
      {
         RigidBodyTransform rootJointPostionAndRotation = new RigidBodyTransform();
         rootJointPostionAndRotation.setRotationPitchAndZeroTranslation(Math.PI*0.9);
         rootJointPostionAndRotation.setTranslation(new Vector3D(0.0, 0.0, 0.4));
         rootJoint.setRotationAndTranslation(rootJointPostionAndRotation);
         
         if (SET_INITIAL_VELOCITIES)
         {
            rootJoint.setVelocity(new Vector3D());
            //         rootJoint.setAngularVelocityInBody(new Vector3d(0.2, 2.5, 0.0));
         }
      }

      Vector3D externalForceVector = new Vector3D(gravitationalAcceleration);
      Point3D comPoint = new Point3D();
      double mass = this.computeCenterOfMass(comPoint);
      externalForceVector.scale(-mass);
      externalForcePoint.setForce(externalForceVector);

      update();
   }

   // public Link getBodyLink()
   // {
   // return bodyLink;
   // }

   public FloatingJoint getRootJoint()
   {
      return rootJoint;
   }

   public ArrayList<KinematicPoint> getPositionPoints()
   {
      return positionPoints;
   }
   
   public ArrayList<KinematicPoint> getVelocityPoints()
   {
      return velocityPoints;
   }

   public Vector3D getActualAngularAccelerationInBodyFrame()
   {
      return rootJoint.getAngularAccelerationInBody();
   }

   public Quaternion getActualOrientation()
   {
      return rootJoint.getQuaternion();
   }
   
   public Vector3D getActualAngularVelocity()
   {
      return rootJoint.getAngularVelocityInBody();
   }
}

