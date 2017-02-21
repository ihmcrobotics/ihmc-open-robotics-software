package us.ihmc.exampleSimulations.springBall;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;

public class SpringBallRobot extends Robot
{
   private static final long serialVersionUID = 6695159376507584365L;

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


   // private static final double L1 = 0.3,  M1 = 0.1,  R1 = 0.01,  Ixx1 = 0.01, Iyy1 = 0.01, Izz1 = 0.01;
   // private static final double L2 = 0.12, M2 = 0.05, R2 = 0.005, Ixx2 = 0.01, Iyy2 = 0.01, Izz2 = 0.01;
   // private static final double L3 = 0.08, M3 = 0.03, R3 = 0.001, Ixx3 = 0.01, Iyy3 = 0.01, Izz3 = 0.01;;
   // private static final double TOY_L = 0.02, TOY_W = 0.04, TOY_H = 0.03, TOY_R = 0.02;

// private static final double DAMP1 = 0.06, DAMP2 = 0.006, DAMP3 = 0.003;

   public SpringBallRobot()
   {
      super("SpringBall");

      FloatingJoint bodyJoint = new FloatingJoint("body", new Vector3D(), this);
      Link bodyLink = body();
      bodyJoint.setLink(bodyLink);
      this.addRootJoint(bodyJoint);

      double xOffset, yOffset, zOffset;
      SliderJoint nextSlider;
      Link nextLink;

      for (int i = 0; i < NUM_SPIKES; i++)
      {
         xOffset = 1.0 - 2.0 * Math.random();
         yOffset = 1.0 - 2.0 * Math.random();
         zOffset = 1.0 - 2.0 * Math.random();
         Vector3D offsetVector = new Vector3D(xOffset, yOffset, zOffset);

         // offsetVector.normalize();
         double scale = xOffset * xOffset / (R1 * R1) + yOffset * yOffset / (R2 * R2) + zOffset * zOffset / (R3 * R3);
         scale = 1.0 / Math.sqrt(scale);
         offsetVector.scale(scale);

         nextSlider = new SliderJoint("slider" + i, offsetVector, this, offsetVector);
         nextLink = sliderLink(offsetVector);
         nextSlider.setLink(nextLink);
         bodyJoint.addJoint(nextSlider);

         Vector3D gcVector = new Vector3D(offsetVector);
         gcVector.normalize();
         gcVector.scale(SLIDER_LENGTH);

         GroundContactPoint gc = new GroundContactPoint("gc" + i, gcVector, this);
         nextSlider.addGroundContactPoint(gc);
      }
   }

// private void initJoint(GimbalJoint joint)
// {
//   double init_q1 = (2.0*Math.random()-1.0)*0.25;
//   double init_q2 = (2.0*Math.random()-1.0)*0.25;
//   double init_q3 = (2.0*Math.random()-1.0)*Math.PI;
//   double init_qd1 = (2.0*Math.random()-1.0)*0.5;
//   double init_qd2 = (2.0*Math.random()-1.0)*0.5;
//   double init_qd3 = (2.0*Math.random()-1.0)*2.0;
//
//   joint.setInitialState(init_q1, init_qd1, init_q2, init_qd2, init_q3, init_qd3);
// }

   private Link body()
   {
      Link ret = new Link("body");
      ret.setMass(BODY_M);
      ret.setComOffset(0.0, 0.0, 0.0);
      ret.setMomentOfInertia(BODY_Ixx, BODY_Iyy, BODY_Izz);

      AppearanceDefinition app = YoAppearance.Aqua();

      // YoAppearance.makeTransparent(app, 0.9f);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addEllipsoid(R1, R2, R3, app);    // YoAppearance.DarkBlue());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }


   private Link sliderLink(Vector3D u_i_hat)
   {
      Link ret = new Link("slider");
      ret.setMass(SLIDER_M);

      Vector3D comOff = new Vector3D(u_i_hat);
      comOff.normalize();
      comOff.scale(SLIDER_LENGTH / 2.0);

      ret.setComOffset(comOff.getX(), comOff.getY(), comOff.getZ());

      // ret.setMomentOfInertia(SLIDER_Ixx, SLIDER_Iyy, SLIDER_Izz);

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

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.rotate(rotation);
      linkGraphics.addCylinder(SLIDER_LENGTH, SLIDER_R, YoAppearance.DarkBlue());    // R1, R2, R3, YoAppearance.DarkBlue());

      ret.setLinkGraphics(linkGraphics);

      Matrix3D MOI = new Matrix3D(SLIDER_Ixx, 0.0, 0.0, 0.0, SLIDER_Iyy, 0.0, 0.0, 0.0, SLIDER_Izz);

      Matrix3D MOI_rot = new Matrix3D(rotation);
      MOI_rot.transpose();
      MOI_rot.multiply(MOI);
      MOI_rot.multiply(rotation);

      ret.setMomentOfInertia(MOI_rot);

      return ret;
   }




}
