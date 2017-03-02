package us.ihmc.exampleSimulations.stewartPlatform;

//import java.applet.Applet;

//import java.awt.*;
//import com.sun.j3d.utils.applet.MainFrame;
//import com.sun.j3d.utils.universe.*;
//import com.sun.j3d.utils.geometry.*;
//import RigidBodyTransform;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.simulationconstructionset.UniversalJoint;

public class StewartPlatformRobot extends Robot
{
 
   private static final long serialVersionUID = 5454099548298298482L;

	/*
    * public static final float PLAT_R = 2.3f/2.0f, PLAT_H = 0.2f;
    * public static final float BASE_R = 3.8f/2.0f, BASE_H = 0.2f;
    * public static final float ACT_MIN = 2.3f, ACT_MAX = 6.8f;
    * public static final float LLEG_R = 0.15f, ULEG_R = 0.3f;
    * public static final float VECTOR_H = 3.0f, VECTOR_R = 0.1f;
    */

   public static float PLAT_R = 0.23f / 2.0f;
   public static float PLAT_H = 0.02f;
   public static float BASE_R = 0.38f / 2.0f;
   public static float BASE_H = 0.02f;
   public static float ACT_MIN = .23f;
   public static float ACT_MAX = .40f;
   public static float ULEG_R = 0.010f;
   public static float LLEG_R = 0.02f;
   public static float VECTOR_H = 0.3f;
   public static float VECTOR_R = 0.01f;

   public static float Z_MIN = 0.8f * ACT_MIN;
   public static float Z_MAX = 1.2f * ACT_MAX;
   public static float X_RANGE = Z_MAX;
   public static float Y_RANGE = Z_MAX;
   public static float Z_RANGE = Z_MAX - Z_MIN;
   public static double YAW_RANGE = 1.2;
   public static double ROLL_RANGE = 0.8;
   public static double PITCH_RANGE = 0.8;

   public static float ULEG_H = ACT_MAX - ACT_MIN;    // ACT_MAX/2.0f;
   public static float LLEG_H = ACT_MIN;    // ACT_MAX/2.0f;
   public static float PLAT_ANG = (float) Math.PI / 8.0f;
   public static float BASE_ANG = (float) Math.PI / 8.0f;

   public static double PLAT_MASS = 10.0;
   public static double
      PLAT_Ixx = 0.4 * PLAT_MASS * PLAT_R * PLAT_R, PLAT_Iyy = PLAT_Ixx, PLAT_Izz = 0.6 * PLAT_MASS * PLAT_R * PLAT_R;

   public static double LOWER_LEG_MASS = 1.0;
   public static double
      LOWER_LEG_Ixx = 0.4 * LOWER_LEG_MASS * LLEG_H * LLEG_H, LOWER_LEG_Iyy = LOWER_LEG_Ixx, LOWER_LEG_Izz = 0.6 * LOWER_LEG_MASS * LLEG_R * LLEG_R;

   public static double UPPER_LEG_MASS = 1.0;
   public static double
      UPPER_LEG_Ixx = 0.4 * UPPER_LEG_MASS * ULEG_H * ULEG_H, UPPER_LEG_Iyy = 0.1, UPPER_LEG_Izz = 0.6 * UPPER_LEG_MASS * ULEG_R * ULEG_R;


//   private double[] act_forces = new double[6];
//   private double leg_forces;

   public ExternalForcePoint[] pointa = new ExternalForcePoint[6];
   public ExternalForcePoint[] pointb = new ExternalForcePoint[6];
   public ExternalForcePoint[] pointc = new ExternalForcePoint[6];

   public Vector3D[] platform_offsets = new Vector3D[6], base_offsets = new Vector3D[6];

   // private Joint[] bJoint = new Joint[6];

//   private static final Vector3d xHat = new Vector3d(1.0, 0.0, 0.0);
//   private static final Vector3d yHat = new Vector3d(0.0, 1.0, 0.0);
//   private static final Vector3d zHat = new Vector3d(0.0, 0.0, 1.0);
//   private static final Vector3d emptyVector = new Vector3d(0.0, 0.0, 0.0);

//   private Vector3d legOffsetVector;
//   private Link forceVector;
//
//   private Transform3D vecTransform = new Transform3D();
//   private Matrix3d vecMatrix = new Matrix3d();
//
//   private Vector2d pitchNRollVector2d = new Vector2d();

//   private Transform3D bodyTransToJoint = new Transform3D();
   public FloatingJoint platformJoint;

   public StewartPlatformRobot(String name)
   {
      super(name);

      this.addStaticLink(base());

      // Platform:

      platformJoint = new FloatingJoint("platform", "platform", new Vector3D(), this);
      platformJoint.setLink(platform());
      this.addRootJoint(platformJoint);

      ExternalForcePoint ef_platform = new ExternalForcePoint("ef_platform", new Vector3D(), this);
      platformJoint.addExternalForcePoint(ef_platform);

      // Platform External Force Points

      for (int i = 0; i < 6; i++)
      {
         platform_offsets[i] = new Vector3D();
         base_offsets[i] = new Vector3D();
      }

      configPolar(base_offsets, platform_offsets, 0.0, 0.8 * BASE_R, BASE_ANG, 0.8 * PLAT_R, PLAT_ANG);

      for (int i = 0; i < 6; i++)
      {
         ExternalForcePoint ef_p = new ExternalForcePoint("ef_p" + i, platform_offsets[i], this);
         platformJoint.addExternalForcePoint(ef_p);
      }

      // 6 Legs:

      for (int i = 0; i < 6; i++)
      {
         UniversalJoint uni = new UniversalJoint("base_uni_x_" + i, "base_uni_y_" + i, base_offsets[i], this, Axis.X, Axis.Y);
         Link lowerLeg = lowerLeg();
         uni.setLink(lowerLeg);
         this.addRootJoint(uni);

         SliderJoint slider = new SliderJoint("act" + i, new Vector3D(), this, Axis.Z);
         Link upperLeg = upperLeg();
         slider.setLink(upperLeg);
         uni.addJoint(slider);

         ExternalForcePoint ef = new ExternalForcePoint("ef_a" + i, new Vector3D(), this);
         slider.addExternalForcePoint(ef);
      }


      // Force Vector

      /*
       * Joint vectorJoint = new FreeJoint("vector",new Vector3d(),forceVectorVars,"vector_x","vector_y","vector_z","vector_yaw","vector_roll","vector_pitch");
       * forceVector = vector();
       * vectorJoint.setLink(forceVector);
       * rootJoint.addJoint(vectorJoint);
       *
       * //StewartPlatformVars.getVar("z").val = LLEG_H + ULEG_H + STICK_H;
       * stewartPlatformVars.getVar("platform_z").set(-LLEG_H-ULEG_H);
       */

   }


   /*
    * public void hideForceVector()
    * {
    * forceVector.hide();
    * }
    *
    * public void showForceVector()
    * {
    * forceVector.show();
    * }
    */

   private Link base()
   {
      Link ret = new Link("base");

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.identity();
      // linkGraphics.translate(baseOffsetX, baseOffsetY,baseOffsetZ);
      linkGraphics.addCylinder(BASE_H, BASE_R, YoAppearance.Black());
      ret.setLinkGraphics(linkGraphics);
      
      return ret;
   }


   private Link platform()
   {
      Link ret = new Link("platform");
      ret.setMass(PLAT_MASS);
      ret.setMomentOfInertia(PLAT_Ixx, PLAT_Iyy, PLAT_Izz);
      ret.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, 0.0);
      linkGraphics.addCylinder(PLAT_H, PLAT_R, YoAppearance.Black());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link lowerLeg()
   {
      Link ret = new Link("lowerLeg");
      ret.setMass(LOWER_LEG_MASS);
      ret.setMomentOfInertia(LOWER_LEG_Ixx, LOWER_LEG_Iyy, LOWER_LEG_Izz);
      ret.setComOffset(0.0, 0.0, LLEG_H / 2.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, 0.0);
      linkGraphics.addCylinder(LLEG_H, LLEG_R, YoAppearance.Red());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link upperLeg()
   {
      Link ret = new Link("upperLeg");
      ret.setMass(UPPER_LEG_MASS);
      ret.setMomentOfInertia(UPPER_LEG_Ixx, UPPER_LEG_Iyy, UPPER_LEG_Izz);
      ret.setComOffset(0.0, 0.0, -ULEG_H / 2.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -ULEG_H);
      linkGraphics.addCylinder(ULEG_H, ULEG_R, YoAppearance.Black());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

 /*
   private Link vector()
   {
      Link ret = new Link("vector");

      LinkGraphics linkGraphics = new LinkGraphics();
      linkGraphics.addCylinder(VECTOR_H, VECTOR_R, YoAppearance.Black());    // .PlaneMaterial());
      linkGraphics.translate(0.0, 0.0, VECTOR_H);
      linkGraphics.addCone(VECTOR_R * 4.0f, VECTOR_R * 3.0f, YoAppearance.Black());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link forceVector()
   {
      Link ret = new Link("force vector");
      LinkGraphics linkGraphics = new LinkGraphics();
      linkGraphics.translate(0.0, 0.0, -VECTOR_H * 4.0f);
      linkGraphics.addCylinder(VECTOR_H * 4.0f, VECTOR_R, YoAppearance.FenceMaterial());

      // ret.translate(0.0,0.0,VECTOR_H);
      // ret.rotate(Math.PI,Link.X);
      linkGraphics.identity();
      linkGraphics.addCone(VECTOR_R * 4.0f, VECTOR_R * 3.0f, YoAppearance.FenceMaterial());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }
*/

   public void configPolar(Vector3D[] b, Vector3D[] p, double shiftAng, double baseRad, double baseAng, double platRad, double platAng)
   {
      double mirror = 1.0;

      b[0].setX(Math.cos(shiftAng + 9.0 * Math.PI / 6.0 - baseAng) * baseRad);
      b[0].setY(mirror * Math.sin(shiftAng + 9.0 * Math.PI / 6.0 - baseAng) * baseRad);
      b[0].setZ(0.0);

      b[1].setX(Math.cos(shiftAng + 9.0 * Math.PI / 6.0 + baseAng) * baseRad);
      b[1].setY(mirror * Math.sin(shiftAng + 9.0 * Math.PI / 6.0 + baseAng) * baseRad);
      b[1].setZ(0.0);

      b[2].setX(Math.cos(shiftAng + Math.PI / 6.0 - baseAng) * baseRad);
      b[2].setY(mirror * Math.sin(shiftAng + Math.PI / 6.0 - baseAng) * baseRad);
      b[2].setZ(0.0);

      b[3].setX(Math.cos(shiftAng + Math.PI / 6.0 + baseAng) * baseRad);
      b[3].setY(mirror * Math.sin(shiftAng + Math.PI / 6.0 + baseAng) * baseRad);
      b[3].setZ(0.0);

      b[4].setX(Math.cos(shiftAng + 5.0 * Math.PI / 6.0 - baseAng) * baseRad);
      b[4].setY(mirror * Math.sin(shiftAng + 5.0 * Math.PI / 6.0 - baseAng) * baseRad);
      b[4].setZ(0.0);

      b[5].setX(Math.cos(shiftAng + 5.0 * Math.PI / 6.0 + baseAng) * baseRad);
      b[5].setY(mirror * Math.sin(shiftAng + 5.0 * Math.PI / 6.0 + baseAng) * baseRad);
      b[5].setZ(0.0);

      // Platform

      p[3].setX(Math.cos(shiftAng + 3.0 * Math.PI / 6.0 - platAng) * platRad);
      p[3].setY(mirror * Math.sin(shiftAng + 3.0 * Math.PI / 6.0 - platAng) * platRad);
      p[3].setZ(0.0);

      p[4].setX(Math.cos(shiftAng + 3.0 * Math.PI / 6.0 + platAng) * platRad);
      p[4].setY(mirror * Math.sin(shiftAng + 3.0 * Math.PI / 6.0 + platAng) * platRad);
      p[4].setZ(0.0);

      p[1].setX(Math.cos(shiftAng + 11.0 * Math.PI / 6.0 - platAng) * platRad);
      p[1].setY(mirror * Math.sin(shiftAng + 11.0 * Math.PI / 6.0 - platAng) * platRad);
      p[1].setZ(0.0);

      p[2].setX(Math.cos(shiftAng + 11.0 * Math.PI / 6.0 + platAng) * platRad);
      p[2].setY(mirror * Math.sin(shiftAng + 11.0 * Math.PI / 6.0 + platAng) * platRad);
      p[2].setZ(0.0);

      p[0].setX(Math.cos(shiftAng + 7.0 * Math.PI / 6.0 + platAng) * platRad);
      p[0].setY(mirror * Math.sin(shiftAng + 7.0 * Math.PI / 6.0 + platAng) * platRad);
      p[0].setZ(0.0);

      p[5].setX(Math.cos(shiftAng + 7.0 * Math.PI / 6.0 - platAng) * platRad);
      p[5].setY(mirror * Math.sin(shiftAng + 7.0 * Math.PI / 6.0 - platAng) * platRad);
      p[5].setZ(0.0);

   }

/*
   private Vector3d tempAxis1 = new Vector3d();
   private Vector3d tempAxis2 = new Vector3d();
   private Vector3d tempAxis3 = new Vector3d();

   
   private void computePitchNRoll(Transform3D t1, Vector3d vec, Tuple2d pitchNRoll)
   {
      // Rotate the diffVectors by the rotations up to them.
      bodyTransToJoint.set(t1);
      bodyTransToJoint.setTranslation(emptyVector);
      bodyTransToJoint.invert();
      bodyTransToJoint.transform(vec);

      // tempAxis1 points away from camera look ray...
      // tempAxis1.set(-vec.x,-vec.y,-vec.z);
      tempAxis1.set(vec.x, vec.y, vec.z);
      tempAxis1.normalize();

      // yaxis is z cross x
      tempAxis3.set(1.0, 0.0, 0.0);

      tempAxis2.cross(tempAxis1, tempAxis3);
      tempAxis2.normalize();

      tempAxis3.cross(tempAxis2, tempAxis1);

      pitchNRoll.set(Math.atan2(tempAxis1.x, tempAxis3.x), Math.atan2(tempAxis2.z, tempAxis2.y));
   }
*/

//   private Vector3d diffVector = new Vector3d();
//   private double[] tempAct = new double[6];


   public static void setParams()
   {
      if (PLAT_R == 0.0f)
         PLAT_R = 0.115f;
      if (PLAT_H == 0.0f)
         PLAT_H = 0.02f;
      if (BASE_R == 0.0f)
         BASE_R = 0.19f;
      if (BASE_H == 0.0f)
         BASE_H = 0.02f;
      if (ACT_MIN == 0.0f)
         ACT_MIN = .23f;
      if (ACT_MAX == 0.0f)
         ACT_MAX = .68f;
      if (LLEG_R == 0.0f)
         LLEG_R = 0.015f;
      if (ULEG_R == 0.0f)
         ULEG_R = 0.03f;
      if (VECTOR_H == 0.0f)
         VECTOR_H = 0.3f;
      if (VECTOR_R == 0.0f)
         VECTOR_R = 0.01f;

      Z_MIN = 0.8f * ACT_MIN;
      Z_MAX = 1.2f * ACT_MAX;
      X_RANGE = Z_MAX;
      Y_RANGE = Z_MAX;
      Z_RANGE = Z_MAX - Z_MIN;
      YAW_RANGE = 1.2;
      ROLL_RANGE = 0.8;
      PITCH_RANGE = 0.8;

      LLEG_H = ACT_MAX - ACT_MIN;
      ULEG_H = ACT_MIN;
      PLAT_ANG = (float) Math.PI / 8.0f;
      BASE_ANG = (float) Math.PI / 8.0f;

      // System.out.println(Z_MAX);
      // System.out.println(Z_MIN);
   }

}
