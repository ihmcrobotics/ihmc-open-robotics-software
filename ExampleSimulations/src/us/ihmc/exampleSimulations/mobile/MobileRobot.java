package us.ihmc.exampleSimulations.mobile;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.GimbalJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;

/**
 * MobileRobot is a representation of a child's mobile toy that uses a tree structure of 21 gimbal
 * joints (63 degrees of freedom total).
 */
public class MobileRobot extends Robot
{
   private static final double L1 = 0.3, M1 = 0.1, R1 = 0.01, Ixx1 = 0.01, Iyy1 = 0.01, Izz1 = 0.01;
   private static final double L2 = 0.12, M2 = 0.05, R2 = 0.005, Ixx2 = 0.01, Iyy2 = 0.01, Izz2 = 0.01;
   private static final double L3 = 0.08, M3 = 0.03, R3 = 0.001, Ixx3 = 0.01, Iyy3 = 0.01, Izz3 = 0.01;;
   private static final double TOY_L = 0.02, TOY_W = 0.04, TOY_H = 0.03, TOY_R = 0.02;

   private static final double DAMP1 = 0.06, DAMP2 = 0.006, DAMP3 = 0.003;

   public MobileRobot()
   {
      super("Mobile");

      // Create the top (fixed) link that serves as the base of the mobile
      Link topLink = new Link("top");

      Graphics3DObject topLinkGraphics = new Graphics3DObject();
      topLinkGraphics.translate(0.0, 0.0, 1.0 + R1 / 2.0);
      topLinkGraphics.addCylinder(L1 / 60.0, L1 / 3.0, YoAppearance.DarkBlue());
      topLink.setLinkGraphics(topLinkGraphics);

      this.addStaticLink(topLink);

      // create first gimbal joint at the top of the mobile
      GimbalJoint firstGimbal = new GimbalJoint("gimbal_x", "gimbal_y", "gimbal_z", new Vector3D(0.0, 0.0, 1.0), this, Axis.X, Axis.Y, Axis.Z);

      // attach a crossbar to the top gimbal joint
      Link bar1 = createCrossBarLink(M1, L1, R1, Ixx1, Iyy1, Izz1);
      firstGimbal.setLink(bar1);
      firstGimbal.setDamping(DAMP1);
      initializeGimbalJoint(firstGimbal);

      this.addRootJoint(firstGimbal);

      GimbalJoint nextGimbal;
      GimbalJoint finalGimbal;
      Link nextLink;

      // for each point of the cross bar, attach a new gimbal joint and (smaller crossbar)
      // four toys will hang from each of these smaller crossbars
      for (int i = 0; i < 4; i++)
      {
         double xOffset = 0.0;
         double yOffset = 0.0;

         if (i == 0)
            xOffset = L1;
         else if (i == 1)
            xOffset = -L1;
         else if (i == 2)
            yOffset = L1;
         else // i == 3
            yOffset = -L1;

         nextGimbal = new GimbalJoint("gimbal1_" + i + "_x", "gimbal1_" + i + "_y", "gimbal1_" + i + "_z", new Vector3D(xOffset, yOffset, -L1 / 2.0), this,
               Axis.X, Axis.Y, Axis.Z);
         nextLink = createCrossBarLink(M2, L2, R2, Ixx2, Iyy2, Izz2);
         nextGimbal.setLink(nextLink);
         nextGimbal.setDamping(DAMP2);
         initializeGimbalJoint(nextGimbal);
         firstGimbal.addJoint(nextGimbal);

         // for each point of the smaller crossbar, add a gimbal joint with a "toy" link attached
         for (int j = 0; j < 4; j++)
         {
            xOffset = 0.0;
            yOffset = 0.0;

            if (j == 0)
               xOffset = L2;
            else if (j == 1)
               xOffset = -L2;
            else if (j == 2)
               yOffset = L2;
            else // j == 3
               yOffset = -L2;

            finalGimbal = new GimbalJoint("gimbal2_" + i + "_" + j + "_x", "gimbal2_" + i + "_" + j + "_y", "gimbal2_" + i + "_" + j + "_z",
                  new Vector3D(xOffset, yOffset, -L2 / 2.0), this, Axis.X, Axis.Y, Axis.Z);

            // generate a random toy link and attach it to gimbal
            nextLink = createRandomShapeLink();
            finalGimbal.setLink(nextLink);

            // add external force point at the toy COM
            Vector3D offset = new Vector3D();
            nextLink.getComOffset(offset);
            ExternalForcePoint point = new ExternalForcePoint("ef_track" + i + j, offset, this);
            finalGimbal.addExternalForcePoint(point);

            finalGimbal.setDamping(DAMP3);
            initializeGimbalJoint(finalGimbal);
            nextGimbal.addJoint(finalGimbal);

         }
      }
   }

   /**
    * Initializes a GimbalJoint to a random initial position and velocity.
    */
   private void initializeGimbalJoint(GimbalJoint joint)
   {
      double init_q1 = (2.0 * Math.random() - 1.0) * 0.25;
      double init_q2 = (2.0 * Math.random() - 1.0) * 0.25;
      double init_q3 = (2.0 * Math.random() - 1.0) * Math.PI;
      double init_qd1 = (2.0 * Math.random() - 1.0) * 0.5;
      double init_qd2 = (2.0 * Math.random() - 1.0) * 0.5;
      double init_qd3 = (2.0 * Math.random() - 1.0) * 2.0;

      joint.setInitialState(init_q1, init_qd1, init_q2, init_qd2, init_q3, init_qd3);
   }

   /**
    * Creates a cross bar link from the given parameters.
    */
   private Link createCrossBarLink(double mass, double length, double radius, double Ixx, double Iyy, double Izz)
   {
      Link ret = new Link("CrossBar");
      ret.setMass(mass);
      ret.setComOffset(0.0, 0.0, -length / 2.0);
      ret.setMomentOfInertia(Ixx, Iyy, Izz);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addSphere(R1, YoAppearance.Red());
      linkGraphics.translate(0.0, 0.0, -length / 2.0);
      linkGraphics.addCylinder(length / 2.0, radius);

      linkGraphics.identity();
      linkGraphics.translate(length, 0.0, -length / 2.0);
      linkGraphics.rotate(-Math.PI / 2.0, Axis.Y);
      linkGraphics.addCylinder(2.0 * length, radius);
      linkGraphics.addSphere(radius, YoAppearance.Red());
      linkGraphics.translate(0.0, 0.0, 2.0 * length);
      linkGraphics.addSphere(radius, YoAppearance.Red());

      linkGraphics.identity();
      linkGraphics.translate(0.0, length, -length / 2.0);
      linkGraphics.rotate(Math.PI / 2.0, Axis.X);
      linkGraphics.addCylinder(2.0 * length, radius);
      linkGraphics.addSphere(radius, YoAppearance.Red());
      linkGraphics.translate(0.0, 0.0, 2.0 * length);
      linkGraphics.addSphere(radius, YoAppearance.Red());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   /**
    * Generates a random link shape with a thin cylinder attached to represent a string.
    * The toys are generated from one of 9 colors and 7 shapes.
    */
   private Link createRandomShapeLink()
   {
      Link ret = new Link("randomShape");
      double stringLength = L3 * (1.0 + 2.0 * Math.random());

      ret.setMass(M3);
      // assume a massless string
      ret.setComOffset(0.0, 0.0, -stringLength);
      ret.setMomentOfInertia(Ixx3, Iyy3, Izz3);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -stringLength);
      linkGraphics.addCylinder(stringLength, R3);

      AppearanceDefinition app = YoAppearance.Black();

      int appSelection = (int) (Math.random() * 9.0);
      switch (appSelection)
      {
      case 0:
         app = YoAppearance.Black();
         break;

      case 1:
         app = YoAppearance.Red();
         break;

      case 2:
         app = YoAppearance.DarkRed();
         break;

      case 3:
         app = YoAppearance.Green();
         break;

      case 4:
         app = YoAppearance.DarkGreen();
         break;

      case 5:
         app = YoAppearance.Blue();
         break;

      case 6:
         app = YoAppearance.DarkBlue();
         break;

      case 7:
         app = YoAppearance.AluminumMaterial();
         break;

      case 8:
         app = YoAppearance.BlackMetalMaterial();
         break;
      }

      int toySelection = (int) (Math.random() * 7.0);
      switch (toySelection)
      {
      case 0:
         linkGraphics.addSphere(TOY_R, app);
         break;

      case 1:
         linkGraphics.addCylinder(TOY_H, TOY_R, app);
         break;

      case 2:
         linkGraphics.addCube(TOY_L, TOY_W, TOY_H, app);
         break;

      case 3:
         linkGraphics.addCone(TOY_H, TOY_R, app);
         break;

      case 4:
         linkGraphics.addEllipsoid(TOY_L, TOY_W, TOY_H, app);
         break;

      case 5:
         linkGraphics.addHemiEllipsoid(TOY_L, TOY_W, TOY_H, app);
         break;

      case 6:
         linkGraphics.addGenTruncatedCone(TOY_H, TOY_L, TOY_W, TOY_W, TOY_L, app);
      }

      ret.setLinkGraphics(linkGraphics);

      return ret;
   }
}
