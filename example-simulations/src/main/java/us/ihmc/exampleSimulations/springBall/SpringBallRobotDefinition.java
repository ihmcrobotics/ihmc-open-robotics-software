package us.ihmc.exampleSimulations.springBall;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.QuaternionDefinition;
import us.ihmc.scs2.definition.robot.GroundContactPointDefinition;
import us.ihmc.scs2.definition.robot.PrismaticJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;

public class SpringBallRobotDefinition extends RobotDefinition
{
   public static final int NUM_SPIKES = 100;
   private static final double
      R1 = 0.3, R2 = 0.2, R3 = 0.2;
   private static final double
      BODY_M = 1.0, BODY_Ixx = BODY_M * R2 * R3 * 0.6, BODY_Iyy = BODY_M * R1 * R3 * 0.6, BODY_Izz = BODY_M * R1 * R2 * 0.6;

   private static final double
      SLIDER_R = 0.01, SLIDER_LENGTH = 0.25;
   private static final double
      SLIDER_M = 0.1, SLIDER_Ixx = SLIDER_M * SLIDER_R * SLIDER_R * 0.6, SLIDER_Iyy = SLIDER_M * SLIDER_R * SLIDER_R * 0.6,
      SLIDER_Izz = SLIDER_M * SLIDER_R * SLIDER_R * 0.6;

   // A spike's ground contact point can be as far as (ellipsoid surface distance + SLIDER_LENGTH) from the body
   // center - the ellipsoid surface distance is at most max(R1, R2, R3). Drop from comfortably above that reach
   // so the ball starts fully clear of the ground (z = 0) and visibly falls onto it, instead of spawning with
   // some spikes already interpenetrating the ground.
   private static final double MAX_SPIKE_REACH = Math.max(R1, Math.max(R2, R3)) + SLIDER_LENGTH;
   private static final double DROP_HEIGHT = MAX_SPIKE_REACH + 1.0;

   public SpringBallRobotDefinition()
   {
      super("SpringBall");

      RigidBodyDefinition rootBody = new RigidBodyDefinition("rootBody");
      setRootBodyDefinition(rootBody);

      SixDoFJointDefinition bodyJoint = new SixDoFJointDefinition("body");
      RigidBodyDefinition bodyLink = body();
      bodyJoint.setSuccessor(bodyLink);
      rootBody.addChildJoint(bodyJoint);

      // Initial state: drop height and spin, set declaratively here instead of by the controller poking joint YoVariables.
      // Use the combined setConfiguration/setVelocity overloads - hasOutputFor(CONFIGURATION)/(VELOCITY) require
      // orientation+position (respectively angular+linear velocity) to both be non-NaN, or the whole pair is dropped.
      SixDoFJointState initialState = new SixDoFJointState();
      initialState.setConfiguration(new QuaternionDefinition(), new Point3D(0.0, 0.0, DROP_HEIGHT));
      initialState.setVelocity(new Vector3D(-1.5, 1.0, 2.0), new Vector3D());
      bodyJoint.setInitialJointState(initialState);

      for (int i = 0; i < NUM_SPIKES; i++)
      {
         double xOffset = 1.0 - 2.0 * Math.random();
         double yOffset = 1.0 - 2.0 * Math.random();
         double zOffset = 1.0 - 2.0 * Math.random();
         Vector3D offsetVector = new Vector3D(xOffset, yOffset, zOffset);

         double scale = xOffset * xOffset / (R1 * R1) + yOffset * yOffset / (R2 * R2) + zOffset * zOffset / (R3 * R3);
         scale = 1.0 / Math.sqrt(scale);
         offsetVector.scale(scale);

         Vector3D axisVector = new Vector3D(offsetVector);
         axisVector.normalize();

         PrismaticJointDefinition nextSlider = new PrismaticJointDefinition("slider" + i, offsetVector, axisVector);
         RigidBodyDefinition nextLink = sliderLink(i, offsetVector);
         nextSlider.setSuccessor(nextLink);
         bodyLink.addChildJoint(nextSlider);

         Vector3D gcVector = new Vector3D(offsetVector);
         gcVector.normalize();
         gcVector.scale(SLIDER_LENGTH);

         GroundContactPointDefinition gc = new GroundContactPointDefinition("gc" + i, gcVector);
         nextSlider.addGroundContactPointDefinition(gc);
      }

      addControllerDefinition((controllerInput, controllerOutput) -> new SpringBallController(controllerInput, controllerOutput, "springBallController"));
   }

   private RigidBodyDefinition body()
   {
      RigidBodyDefinition ret = new RigidBodyDefinition("body");
      ret.setMass(BODY_M);
      ret.setCenterOfMassOffset(0.0, 0.0, 0.0);
      ret.setMomentOfInertia(new Matrix3D(BODY_Ixx, 0.0, 0.0, 0.0, BODY_Iyy, 0.0, 0.0, 0.0, BODY_Izz));

      VisualDefinitionFactory linkGraphics = new VisualDefinitionFactory();
      linkGraphics.addEllipsoid(R1, R2, R3, ColorDefinitions.Aqua());
      ret.addVisualDefinitions(linkGraphics.getVisualDefinitions());

      return ret;
   }

   private RigidBodyDefinition sliderLink(int i, Vector3D u_i_hat)
   {
      RigidBodyDefinition ret = new RigidBodyDefinition("slider" + i);
      ret.setMass(SLIDER_M);

      Vector3D comOff = new Vector3D(u_i_hat);
      comOff.normalize();
      comOff.scale(SLIDER_LENGTH / 2.0);

      ret.setCenterOfMassOffset(comOff.getX(), comOff.getY(), comOff.getZ());

      // Z axis points away from camera look ray...
      Vector3D zAxis = new Vector3D(u_i_hat);
      zAxis.normalize();

      Vector3D yAxis = new Vector3D(0.0, 0.0, 1.0);

      if (yAxis.equals(zAxis))
      {
         yAxis.set(0.0, 1.0, 0.0);
      }

      Vector3D xAxis = new Vector3D();

      xAxis.cross(yAxis, zAxis);
      xAxis.normalize();

      yAxis.cross(zAxis, xAxis);
      RotationMatrix rotation = new RotationMatrix(xAxis.getX(), yAxis.getX(), zAxis.getX(), xAxis.getY(), yAxis.getY(), zAxis.getY(), xAxis.getZ(), yAxis.getZ(), zAxis.getZ());

      VisualDefinitionFactory linkGraphics = new VisualDefinitionFactory();
      linkGraphics.appendRotation(rotation);
      linkGraphics.addCylinder(SLIDER_LENGTH, SLIDER_R, ColorDefinitions.DarkBlue());
      ret.addVisualDefinitions(linkGraphics.getVisualDefinitions());

      // Ixx == Iyy == Izz here, so the tensor is isotropic and invariant under rotation -
      // no need to rotate it into the slider's local frame like the original SCS1 code did.
      ret.setMomentOfInertia(new Matrix3D(SLIDER_Ixx, 0.0, 0.0, 0.0, SLIDER_Iyy, 0.0, 0.0, 0.0, SLIDER_Izz));

      return ret;
   }

}
