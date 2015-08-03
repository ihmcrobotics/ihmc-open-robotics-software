package us.ihmc.exampleSimulations.trebuchet;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.robotics.Axis;

public class TrebuchetRobot extends Robot
{

   private static final long serialVersionUID = 2900485516542596045L;

   public static final double ROPE_LENGTH = 6.0;

   private static final double BALL_MASS = 80.0;
   private static final double BALL_INERTIA = 1.0;
   public static final double BALL_RADIUS = 0.5;


   // Variables for the base and tower:

   private static final double TOWER_HEIGHT = 9.0;

   private static final double SUPPORT_COLUMN_ANGLE = 0.18 * Math.PI;

   private static final double CROSSBAR_Z1 = TOWER_HEIGHT / 4.0;
   private static final double CROSSBAR_Z2 = TOWER_HEIGHT * 0.45;
   private static final double CROSSBAR_Z3 = TOWER_HEIGHT * 0.70;

   private static final double LOWER_CROSSBAR_ANGLE = 0.08 * Math.PI;
   private static final double UPPER_CROSSBAR_ANGLE = 0.18 * Math.PI;

   private static final double TOWER_MASS = 1000.0;
   private static final double TOWER_INERTIA = 100.0;
   private static final double TOWER_COM_Z = 3.0;
   private static final double TOWER_COM_Y = 0.0;


   private static final double BASE_WIDTH = 3.25;
   public static final double BASE_HEIGHT = 0.5;

   public static final double WHEEL_RADIUS = 0.5;
   private static final double WHEEL_WIDTH = 0.2;

   private static final double WHEEL_PIN_RADIUS = WHEEL_RADIUS / 3.0;
   private static final double WHEEL_PIN_WIDTH = WHEEL_WIDTH * 0.8;

   private static final double COLUMN_WIDTH = 0.4;
   private static final double MIDDLE_COLUMN_WIDTH = 0.55;
   private static final double CROSSBAR_HEIGHT = COLUMN_WIDTH;
   private static final double CROSSBAR_WIDTH = CROSSBAR_HEIGHT / 3.0;


   private static final double MIDDLE_COLUMN_HEIGHT = TOWER_HEIGHT - 0.3;

   private static final double PIVOT_RADIUS = 0.3;

   private static final double PIVOT_HOLDER_LENGTH = 2.0;
   private static final double PIVOT_HOLDER_WIDTH = 1.75 * COLUMN_WIDTH;
   private static final double PIVOT_HOLDER_HEIGHT = 2.0 * COLUMN_WIDTH;

   private static final double SUPPORT_COLUMN_LENGTH = TOWER_HEIGHT / Math.cos(SUPPORT_COLUMN_ANGLE);
   private static final double SUPPORT_COLUMN_OFFSET = TOWER_HEIGHT * Math.tan(SUPPORT_COLUMN_ANGLE);

   private static final double WHEEL_X_SEPARATION = 2.0 * SUPPORT_COLUMN_OFFSET;
   public static final double BASE_LENGTH = 2.4 * SUPPORT_COLUMN_OFFSET;


   private static final double LOWER_CROSSBAR_TOP_LENGTH = (Math.sin(Math.PI / 2.0 - SUPPORT_COLUMN_ANGLE)
                                                            / Math.sin(Math.PI / 2.0 + SUPPORT_COLUMN_ANGLE - LOWER_CROSSBAR_ANGLE)) * (SUPPORT_COLUMN_OFFSET
                                                               * (TOWER_HEIGHT - CROSSBAR_Z1) / TOWER_HEIGHT);

   private static final double LOWER_CROSSBAR_BOT_LENGTH = (Math.sin(Math.PI / 2.0 + SUPPORT_COLUMN_ANGLE)
                                                            / Math.sin(Math.PI / 2.0 - SUPPORT_COLUMN_ANGLE - LOWER_CROSSBAR_ANGLE)) * (SUPPORT_COLUMN_OFFSET
                                                               * (TOWER_HEIGHT - CROSSBAR_Z1) / TOWER_HEIGHT);


   private static final double LOWER_CROSSBAR_LENGTH = LOWER_CROSSBAR_TOP_LENGTH + LOWER_CROSSBAR_BOT_LENGTH + COLUMN_WIDTH;

   private static final double MIDDLE_CROSSBAR_LENGTH = (2.0 * SUPPORT_COLUMN_OFFSET * (TOWER_HEIGHT - CROSSBAR_Z2) / TOWER_HEIGHT);

   private static final double UPPER_CROSSBAR_TOP_LENGTH = (Math.sin(Math.PI / 2.0 - SUPPORT_COLUMN_ANGLE)
                                                            / Math.sin(Math.PI / 2.0 + SUPPORT_COLUMN_ANGLE - UPPER_CROSSBAR_ANGLE)) * (SUPPORT_COLUMN_OFFSET
                                                               * (TOWER_HEIGHT - CROSSBAR_Z3) / TOWER_HEIGHT);

   private static final double UPPER_CROSSBAR_BOT_LENGTH = (Math.sin(Math.PI / 2.0 + SUPPORT_COLUMN_ANGLE)
                                                            / Math.sin(Math.PI / 2.0 - SUPPORT_COLUMN_ANGLE - UPPER_CROSSBAR_ANGLE)) * (SUPPORT_COLUMN_OFFSET
                                                               * (TOWER_HEIGHT - CROSSBAR_Z3) / TOWER_HEIGHT);

   private static final double UPPER_CROSSBAR_LENGTH = UPPER_CROSSBAR_TOP_LENGTH + UPPER_CROSSBAR_BOT_LENGTH + COLUMN_WIDTH;


   // Variables for the pole:
   private static final double POLE_MASS = 5000.0;
   private static final double POLE_INERTIA = 1000.0;

   private static final double POLE_BALLSIDE_LENGTH = TOWER_HEIGHT * 1.1;
   private static final double POLE_WEIGHTSIDE_LENGTH = TOWER_HEIGHT * 0.95;

   private static final double POLE_COM_Z = -POLE_WEIGHTSIDE_LENGTH * 0.75;

   private static final double POLE_LENGTH = POLE_BALLSIDE_LENGTH + POLE_WEIGHTSIDE_LENGTH;

   private static final double POLE_BALLSIDE_RADIUS = 0.55;
   private static final double POLE_WEIGHTSIDE_RADIUS = 0.7;

   private static final double ROPEHOLDER_RADIUS = POLE_BALLSIDE_RADIUS * 0.65;
   private static final double ROPEHOLDER_LENGTH = TOWER_HEIGHT * 0.06;



   private static final double WEIGHT_RADIUS = POLE_WEIGHTSIDE_RADIUS * 1.75;
   private static final double WEIGHT_HEIGHT = WEIGHT_RADIUS * 0.8;



   SliderJoint x_slider;
   PinJoint pivot;

   // ArrayList gcPoints = new ArrayList(4);



   public TrebuchetRobot()
   {
      super("Trebuchet");


      /*
       * System.out.println("1:" + (Math.PI + SUPPORT_COLUMN_ANGLE - LOWER_CROSSBAR_ANGLE));
       * System.out.println("2:" + Math.PI);
       * System.out.println("3:" + SUPPORT_COLUMN_ANGLE);
       * System.out.println("4:" + LOWER_CROSSBAR_ANGLE);
       *
       *
       * System.out.println("5:" + Math.sin(Math.PI - SUPPORT_COLUMN_ANGLE));
       * System.out.println("6:" + Math.sin(Math.PI + SUPPORT_COLUMN_ANGLE - LOWER_CROSSBAR_ANGLE));
       * System.out.println("7:" + (SUPPORT_COLUMN_OFFSET*(TOWER_HEIGHT - CROSSBAR_Z1)/TOWER_HEIGHT));
       *
       *
       * System.out.println("8:" + SUPPORT_COLUMN_OFFSET);
       * System.out.println("9:" + LOWER_CROSSBAR_BOT_LENGTH);
       * System.out.println("10:" + LOWER_CROSSBAR_TOP_LENGTH);
       */

      this.setGravity(0.0, 0.0, -9.81);

      /** ************************ BranchJoint ********************************** */

      // RootBranchJoint branch = new RootBranchJoint("branch", new Vector3d(), this);
      // branch.setLink(new Link("branch"));
      // this.addRootJoint(branch);

      /** ************************ Ball ********************************** */

      FloatingJoint ballJoint = new FloatingJoint("ball", "ball", new Vector3d(), this);
      Link ball = ball();
      ballJoint.setLink(ball);
      this.addRootJoint(ballJoint);

      ExternalForcePoint ballCenter = new ExternalForcePoint("ef_ball", new Vector3d(), this);
      ballJoint.physics.addExternalForcePoint(ballCenter);

      /** ************************ Tower ********************************** */

      x_slider = new SliderJoint("x", new Vector3d(0.0, 0.0, WHEEL_RADIUS + BASE_HEIGHT / 2.0), this, Axis.X);
      Link tower = tower();
      x_slider.setLink(tower);
      this.addRootJoint(x_slider);

      // this.rootJoint = x_slider;


      /** ************************ Pole ********************************** */

      pivot = new PinJoint("pivot", new Vector3d(0.0, 0.0, TOWER_HEIGHT), this, Axis.Y);
      Link pole = pole();
      pivot.setLink(pole);
      x_slider.addJoint(pivot);

      ExternalForcePoint poleTip = new ExternalForcePoint("ef_pole", new Vector3d(0.0, 0.0, POLE_BALLSIDE_LENGTH), this);
      pivot.physics.addExternalForcePoint(poleTip);


      this.setController(new TrebuchetController(this, "trebuchetController"));

   }

   private Link ball()
   {
      Link ret = new Link("ball");

      // Appearances:
      AppearanceDefinition ballAppearance = YoAppearance.Red();

      // javax.media.j3d.Appearance ballAppearance = YoAppearance.EarthTexture(new JPanel());
      // javax.media.j3d.Appearance ballAppearance = YoAppearance.StoneTexture(new JPanel());

      ret.setMass(BALL_MASS);
      ret.setMomentOfInertia(BALL_INERTIA, BALL_INERTIA, BALL_INERTIA);

      ret.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics  = new Graphics3DObject();
      linkGraphics.addSphere(BALL_RADIUS, ballAppearance);

      // linkGraphics.translate(10.0, 0.0, 0.0);
      // linkGraphics.addCube(4.0*BALL_RADIUS, 4.0*BALL_RADIUS, 4.0*BALL_RADIUS, ballAppearance);

      ret.setLinkGraphics(linkGraphics);
      
      return ret;
   }

   private Link tower()
   {
      Link ret = new Link("tower");

      // Appearances:
      AppearanceDefinition baseAppearance = YoAppearance.Red();    // EarthTexture(new JPanel()); //YoAppearance.Red();
      AppearanceDefinition wheelAppearance = YoAppearance.White();


      ret.setMass(TOWER_MASS);
      ret.setMomentOfInertia(TOWER_INERTIA, TOWER_INERTIA, TOWER_INERTIA);

      ret.setComOffset(0.0, TOWER_COM_Y, TOWER_COM_Z);

      Graphics3DObject linkGraphics  = new Graphics3DObject();
      // Base:
      linkGraphics.translate(0.0, 0.0, -BASE_HEIGHT);
      linkGraphics.addCube(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT, baseAppearance);

      // Wheels:
      linkGraphics.identity();
      linkGraphics.translate(WHEEL_X_SEPARATION / 2.0, BASE_WIDTH / 2.0, -BASE_HEIGHT / 2.0);
      linkGraphics.rotate(-Math.PI / 2.0, Axis.X);
      linkGraphics.addCylinder(WHEEL_WIDTH, WHEEL_RADIUS, wheelAppearance);
      linkGraphics.translate(0.0, 0.0, WHEEL_WIDTH);
      linkGraphics.addCylinder(WHEEL_PIN_RADIUS, WHEEL_PIN_WIDTH, wheelAppearance);

      linkGraphics.identity();
      linkGraphics.translate(-WHEEL_X_SEPARATION / 2.0, BASE_WIDTH / 2.0, -BASE_HEIGHT / 2.0);
      linkGraphics.rotate(-Math.PI / 2.0, Axis.X);
      linkGraphics.addCylinder(WHEEL_WIDTH, WHEEL_RADIUS, wheelAppearance);
      linkGraphics.translate(0.0, 0.0, WHEEL_WIDTH);
      linkGraphics.addCylinder(WHEEL_PIN_RADIUS, WHEEL_PIN_WIDTH, wheelAppearance);

      linkGraphics.identity();
      linkGraphics.translate(WHEEL_X_SEPARATION / 2.0, -BASE_WIDTH / 2.0, -BASE_HEIGHT / 2.0);
      linkGraphics.rotate(Math.PI / 2.0, Axis.X);
      linkGraphics.addCylinder(WHEEL_WIDTH, WHEEL_RADIUS, wheelAppearance);
      linkGraphics.translate(0.0, 0.0, WHEEL_WIDTH);
      linkGraphics.addCylinder(WHEEL_PIN_RADIUS, WHEEL_PIN_WIDTH, wheelAppearance);

      linkGraphics.identity();
      linkGraphics.translate(-WHEEL_X_SEPARATION / 2.0, -BASE_WIDTH / 2.0, -BASE_HEIGHT / 2.0);
      linkGraphics.rotate(Math.PI / 2.0, Axis.X);
      linkGraphics.addCylinder(WHEEL_WIDTH, WHEEL_RADIUS, wheelAppearance);
      linkGraphics.translate(0.0, 0.0, WHEEL_WIDTH);
      linkGraphics.addCylinder(WHEEL_PIN_RADIUS, WHEEL_PIN_WIDTH, wheelAppearance);

      // Middle Support Column:

      linkGraphics.identity();
      linkGraphics.translate(0.0, -(BASE_WIDTH - COLUMN_WIDTH * 0.95) / 2.0, 0.0);
      linkGraphics.addCube(MIDDLE_COLUMN_WIDTH, COLUMN_WIDTH, MIDDLE_COLUMN_HEIGHT);

      linkGraphics.identity();
      linkGraphics.translate(0.0, (BASE_WIDTH - COLUMN_WIDTH * 0.95) / 2.0, 0.0);
      linkGraphics.addCube(MIDDLE_COLUMN_WIDTH, COLUMN_WIDTH, MIDDLE_COLUMN_HEIGHT);

      // Side Support Columns:

      linkGraphics.identity();
      linkGraphics.translate(SUPPORT_COLUMN_OFFSET, -(BASE_WIDTH - COLUMN_WIDTH * 0.95) / 2.0, 0.0);
      linkGraphics.rotate(-SUPPORT_COLUMN_ANGLE, Axis.Y);
      linkGraphics.addCube(COLUMN_WIDTH, COLUMN_WIDTH, SUPPORT_COLUMN_LENGTH);

      linkGraphics.identity();
      linkGraphics.translate(-SUPPORT_COLUMN_OFFSET, -(BASE_WIDTH - COLUMN_WIDTH * 0.95) / 2.0, 0.0);
      linkGraphics.rotate(SUPPORT_COLUMN_ANGLE, Axis.Y);
      linkGraphics.addCube(COLUMN_WIDTH, COLUMN_WIDTH, SUPPORT_COLUMN_LENGTH);

      linkGraphics.identity();
      linkGraphics.translate(SUPPORT_COLUMN_OFFSET, (BASE_WIDTH - COLUMN_WIDTH * 0.95) / 2.0, 0.0);
      linkGraphics.rotate(-SUPPORT_COLUMN_ANGLE, Axis.Y);
      linkGraphics.addCube(COLUMN_WIDTH, COLUMN_WIDTH, SUPPORT_COLUMN_LENGTH);

      linkGraphics.identity();
      linkGraphics.translate(-SUPPORT_COLUMN_OFFSET, (BASE_WIDTH - COLUMN_WIDTH * 0.95) / 2.0, 0.0);
      linkGraphics.rotate(SUPPORT_COLUMN_ANGLE, Axis.Y);
      linkGraphics.addCube(COLUMN_WIDTH, COLUMN_WIDTH, SUPPORT_COLUMN_LENGTH);

      // Lower Crossbars:

      linkGraphics.identity();
      linkGraphics.translate(0.0, -(BASE_WIDTH - COLUMN_WIDTH * 0.95) / 2.0 - (COLUMN_WIDTH / 2.0 + CROSSBAR_WIDTH / 2.0), CROSSBAR_Z1);
      linkGraphics.rotate(LOWER_CROSSBAR_ANGLE, Axis.Y);
      linkGraphics.translate((LOWER_CROSSBAR_BOT_LENGTH - LOWER_CROSSBAR_TOP_LENGTH) / 2.0, 0.0, 0.0);
      linkGraphics.addCube(LOWER_CROSSBAR_LENGTH, CROSSBAR_WIDTH, CROSSBAR_HEIGHT);

      linkGraphics.identity();
      linkGraphics.translate(0.0, -(BASE_WIDTH - COLUMN_WIDTH * 0.95) / 2.0 - (COLUMN_WIDTH / 2.0 + CROSSBAR_WIDTH / 2.0), CROSSBAR_Z1);
      linkGraphics.rotate(-LOWER_CROSSBAR_ANGLE, Axis.Y);
      linkGraphics.translate(-(LOWER_CROSSBAR_BOT_LENGTH - LOWER_CROSSBAR_TOP_LENGTH) / 2.0, 0.0, 0.0);
      linkGraphics.addCube(LOWER_CROSSBAR_LENGTH, CROSSBAR_WIDTH, CROSSBAR_HEIGHT);

      linkGraphics.identity();
      linkGraphics.translate(0.0, (BASE_WIDTH - COLUMN_WIDTH * 0.95) / 2.0 + (COLUMN_WIDTH / 2.0 + CROSSBAR_WIDTH / 2.0), CROSSBAR_Z1);
      linkGraphics.rotate(LOWER_CROSSBAR_ANGLE, Axis.Y);
      linkGraphics.translate((LOWER_CROSSBAR_BOT_LENGTH - LOWER_CROSSBAR_TOP_LENGTH) / 2.0, 0.0, 0.0);
      linkGraphics.addCube(LOWER_CROSSBAR_LENGTH, CROSSBAR_WIDTH, CROSSBAR_HEIGHT);

      linkGraphics.identity();
      linkGraphics.translate(0.0, (BASE_WIDTH - COLUMN_WIDTH * 0.95) / 2.0 + (COLUMN_WIDTH / 2.0 + CROSSBAR_WIDTH / 2.0), CROSSBAR_Z1);
      linkGraphics.rotate(-LOWER_CROSSBAR_ANGLE, Axis.Y);
      linkGraphics.translate(-(LOWER_CROSSBAR_BOT_LENGTH - LOWER_CROSSBAR_TOP_LENGTH) / 2.0, 0.0, 0.0);
      linkGraphics.addCube(LOWER_CROSSBAR_LENGTH, CROSSBAR_WIDTH, CROSSBAR_HEIGHT);

      // Middle Crossbars:

      linkGraphics.identity();
      linkGraphics.translate(0.0, -(BASE_WIDTH - COLUMN_WIDTH * 0.95) / 2.0 - (COLUMN_WIDTH / 2.0 + CROSSBAR_WIDTH / 2.0), CROSSBAR_Z2);
      linkGraphics.addCube(MIDDLE_CROSSBAR_LENGTH, CROSSBAR_WIDTH, CROSSBAR_HEIGHT);

      linkGraphics.identity();
      linkGraphics.translate(0.0, (BASE_WIDTH - COLUMN_WIDTH * 0.95) / 2.0 + (COLUMN_WIDTH / 2.0 + CROSSBAR_WIDTH / 2.0), CROSSBAR_Z2);
      linkGraphics.addCube(MIDDLE_CROSSBAR_LENGTH, CROSSBAR_WIDTH, CROSSBAR_HEIGHT);

      // Upper Crossbars:

      linkGraphics.identity();
      linkGraphics.translate(0.0, -(BASE_WIDTH - COLUMN_WIDTH * 0.95) / 2.0 - (COLUMN_WIDTH / 2.0 + CROSSBAR_WIDTH / 2.0), CROSSBAR_Z3);
      linkGraphics.rotate(UPPER_CROSSBAR_ANGLE, Axis.Y);
      linkGraphics.translate((UPPER_CROSSBAR_BOT_LENGTH - UPPER_CROSSBAR_TOP_LENGTH) / 2.0, 0.0, 0.0);
      linkGraphics.addCube(UPPER_CROSSBAR_LENGTH, CROSSBAR_WIDTH, CROSSBAR_HEIGHT);

      linkGraphics.identity();
      linkGraphics.translate(0.0, -(BASE_WIDTH - COLUMN_WIDTH * 0.95) / 2.0 - (COLUMN_WIDTH / 2.0 + CROSSBAR_WIDTH / 2.0), CROSSBAR_Z3);
      linkGraphics.rotate(-UPPER_CROSSBAR_ANGLE, Axis.Y);
      linkGraphics.translate(-(UPPER_CROSSBAR_BOT_LENGTH - UPPER_CROSSBAR_TOP_LENGTH) / 2.0, 0.0, 0.0);
      linkGraphics.addCube(UPPER_CROSSBAR_LENGTH, CROSSBAR_WIDTH, CROSSBAR_HEIGHT);

      linkGraphics.identity();
      linkGraphics.translate(0.0, (BASE_WIDTH - COLUMN_WIDTH * 0.95) / 2.0 + (COLUMN_WIDTH / 2.0 + CROSSBAR_WIDTH / 2.0), CROSSBAR_Z3);
      linkGraphics.rotate(UPPER_CROSSBAR_ANGLE, Axis.Y);
      linkGraphics.translate((UPPER_CROSSBAR_BOT_LENGTH - UPPER_CROSSBAR_TOP_LENGTH) / 2.0, 0.0, 0.0);
      linkGraphics.addCube(UPPER_CROSSBAR_LENGTH, CROSSBAR_WIDTH, CROSSBAR_HEIGHT);

      linkGraphics.identity();
      linkGraphics.translate(0.0, (BASE_WIDTH - COLUMN_WIDTH * 0.95) / 2.0 + (COLUMN_WIDTH / 2.0 + CROSSBAR_WIDTH / 2.0), CROSSBAR_Z3);
      linkGraphics.rotate(-UPPER_CROSSBAR_ANGLE, Axis.Y);
      linkGraphics.translate(-(UPPER_CROSSBAR_BOT_LENGTH - UPPER_CROSSBAR_TOP_LENGTH) / 2.0, 0.0, 0.0);
      linkGraphics.addCube(UPPER_CROSSBAR_LENGTH, CROSSBAR_WIDTH, CROSSBAR_HEIGHT);


      // Top Pivot Holder:

      linkGraphics.identity();
      linkGraphics.translate(0.0, (BASE_WIDTH - COLUMN_WIDTH * 0.95) / 2.0, TOWER_HEIGHT - PIVOT_HOLDER_HEIGHT + PIVOT_RADIUS);
      linkGraphics.addCube(PIVOT_HOLDER_LENGTH, PIVOT_HOLDER_WIDTH, PIVOT_HOLDER_HEIGHT);

      linkGraphics.identity();
      linkGraphics.translate(0.0, -(BASE_WIDTH - COLUMN_WIDTH * 0.95) / 2.0, TOWER_HEIGHT - PIVOT_HOLDER_HEIGHT + PIVOT_RADIUS);
      linkGraphics.addCube(PIVOT_HOLDER_LENGTH, PIVOT_HOLDER_WIDTH, PIVOT_HOLDER_HEIGHT);

      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link pole()
   {
      Link ret = new Link("pole");

      AppearanceDefinition poleAppearance = YoAppearance.Blue();

      ret.setMass(POLE_MASS);
      ret.setMomentOfInertia(POLE_INERTIA, POLE_INERTIA, POLE_INERTIA);
      ret.setComOffset(0.0, 0.0, POLE_COM_Z);

      Graphics3DObject linkGraphics  = new Graphics3DObject();
      // Pivot:
      linkGraphics.rotate(Math.PI / 2.0, Axis.X);
      linkGraphics.translate(0.0, 0.0, -0.6 * BASE_WIDTH);
      linkGraphics.addCylinder(1.2 * BASE_WIDTH, PIVOT_RADIUS, YoAppearance.BlackMetalMaterial());

      // Pole:
      linkGraphics.identity();
      linkGraphics.translate(0.0, 0.0, -POLE_WEIGHTSIDE_LENGTH);
      linkGraphics.addGenTruncatedCone(POLE_LENGTH, POLE_WEIGHTSIDE_RADIUS, POLE_WEIGHTSIDE_RADIUS, POLE_BALLSIDE_RADIUS, POLE_BALLSIDE_RADIUS, poleAppearance);

      // Weights:
      linkGraphics.identity();;
      linkGraphics.translate(0.0, 0.0, -POLE_WEIGHTSIDE_LENGTH * 0.90);
      linkGraphics.addCylinder(WEIGHT_HEIGHT, WEIGHT_RADIUS, YoAppearance.AluminumMaterial());

      // Rope Holder:
      linkGraphics.identity();
      linkGraphics.translate(ROPEHOLDER_RADIUS / 2.0, 0.0, POLE_BALLSIDE_LENGTH);
      linkGraphics.addGenTruncatedCone(ROPEHOLDER_LENGTH, 1.1 * ROPEHOLDER_RADIUS, 1.1 * ROPEHOLDER_RADIUS, ROPEHOLDER_RADIUS, ROPEHOLDER_RADIUS, poleAppearance);

      ret.setLinkGraphics(linkGraphics);

      return ret;
   }


}
