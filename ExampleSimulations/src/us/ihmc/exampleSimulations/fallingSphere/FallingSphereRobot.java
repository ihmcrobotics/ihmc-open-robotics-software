package us.ihmc.exampleSimulations.fallingSphere;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.CollisionGroundContactModel;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.RollingGroundProfile;

public class FallingSphereRobot extends Robot
{
   private static final double EPSILON = 0.2; // Coefficient of Restitution
   private static final double MU = 0.5; // Coefficient of Friction
   
   private static final double R = 0.5;
   private static final double M1 = 1.0;
   private static final double
      Ixx1 = 0.1, Iyy1 = 0.1, Izz1 = 0.1;
   private static final double G = 9.81;

   DoubleYoVariable q_x, q_y, q_z, qd_x, qd_y, qd_z, qdd_x, qdd_y, qdd_z;
   DoubleYoVariable q_qs, q_qx, q_qy, q_qz, qd_wx, qd_wy, qd_wz, qdd_wx, qdd_wy, qdd_wz;

   private final YoVariableRegistry registry = new YoVariableRegistry("FallingSphereEnergy");
   
   private final DoubleYoVariable qdd2_wx = new DoubleYoVariable("qdd2_wx", registry);
   private final DoubleYoVariable qdd2_wy = new DoubleYoVariable("qdd2_wy", registry);
   private final DoubleYoVariable qdd2_wz = new DoubleYoVariable("qdd2_wz", registry);

   private final DoubleYoVariable energy = new DoubleYoVariable("energy", registry);
   
   FloatingJoint floatingJoint;

   public FallingSphereRobot(String name, boolean useImpulseGroundModel)
   {
      super(name);

      this.setGravity(0.0, 0.0, -G);

      // Base:

      floatingJoint = new FloatingJoint("base", new Vector3D(0.0, 0.0, 0.0), this);

      Link link1 = ball();
      floatingJoint.setLink(link1);
      this.addRootJoint(floatingJoint);

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      
      // Ground contact points:
      int N = 20; //8;

      for (int i = 0; i < N; i++)
      {
         double latitude = -Math.PI / 2.0 + (i * Math.PI) / N;

         int nForThisLatitude = (int) ((Math.cos(latitude) * N) + 0.5);
         
         for (int j = 0; j < nForThisLatitude; j++)
         {
            double longitude = (j * 2.0 * Math.PI) / nForThisLatitude;
            
            double z = R * Math.sin(latitude);
            double x = R * Math.cos(latitude) * Math.cos(longitude);
            double y = R * Math.cos(latitude) * Math.sin(longitude);

            // System.out.println("x,y,z: " + x + ", " + y + ", " + z);
            String gcName = "gc" + i + "_" + j;
            GroundContactPoint gc = new GroundContactPoint(gcName, new Vector3D(x, y, z), this);
            floatingJoint.addGroundContactPoint(gc);
            
            YoGraphicPosition dynamicGraphicPosition = new YoGraphicPosition(gcName + "Position", gc.getYoPosition(), 0.01, YoAppearance.Red());
            yoGraphicsListRegistry.registerYoGraphic("FallingSphereGCPoints", dynamicGraphicPosition);

            if (useImpulseGroundModel)
            {
               YoGraphicVector dynamicGraphicVector = new YoGraphicVector(gcName + "Force", gc.getYoPosition(), gc.getYoImpulse(), 10.0, YoAppearance.Pink());
               yoGraphicsListRegistry.registerYoGraphic("FallingSphereForces", dynamicGraphicVector);
            }
            else
            {
               YoGraphicVector dynamicGraphicVector = new YoGraphicVector(gcName + "Force", gc.getYoPosition(), gc.getYoForce(), 1.0/50.0);
               yoGraphicsListRegistry.registerYoGraphic("FallingSphereForces", dynamicGraphicVector);
            }
         }
      }

      GroundContactModel groundContactModel;
      if (useImpulseGroundModel)
      {
         groundContactModel = new CollisionGroundContactModel(this, EPSILON, MU, registry);
      }
      else
      {  
         double kXY = 1000.0; //1422.0;
         double bXY = 100.0; //150.6;
         double kZ = 20.0; //50.0;
         double bZ = 50.0; //1000.0;

         groundContactModel = new LinearGroundContactModel(this, kXY, bXY, kZ, bZ, registry);
      }
      
      double amplitude = 0.1;
      double frequency = 0.3;
      double offset = 0.5;
      GroundProfile3D groundProfile = new RollingGroundProfile(amplitude, frequency, offset);
      groundContactModel.setGroundProfile3D(groundProfile);
      this.setGroundContactModel(groundContactModel);

      initRobot();
      
      this.getRobotsYoVariableRegistry().addChild(registry);
      this.addYoGraphicsListRegistry(yoGraphicsListRegistry);
   }

   private Link ball()
   {
      Link ret = new Link("ball");

      ret.setMass(M1);
      ret.setMomentOfInertia(Ixx1, Iyy1, Izz1);

      ret.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addSphere(R/2.0, YoAppearance.EarthTexture());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }


   public void initRobot()
   {
      t.set(0.0);

      q_x = (DoubleYoVariable) this.getVariable("q_x");
      q_y = (DoubleYoVariable) this.getVariable("q_y");
      q_z = (DoubleYoVariable) this.getVariable("q_z");
      qd_x = (DoubleYoVariable) this.getVariable("qd_x");
      qd_y = (DoubleYoVariable) this.getVariable("qd_y");
      qd_z = (DoubleYoVariable) this.getVariable("qd_z");
      qdd_x = (DoubleYoVariable) this.getVariable("qdd_x");
      qdd_y = (DoubleYoVariable) this.getVariable("qdd_y");
      qdd_z = (DoubleYoVariable) this.getVariable("qdd_z");

      q_qs = (DoubleYoVariable) this.getVariable("q_qs");
      q_qx = (DoubleYoVariable) this.getVariable("q_qx");
      q_qy = (DoubleYoVariable) this.getVariable("q_qy");
      q_qz = (DoubleYoVariable) this.getVariable("q_qz");
      qd_wx = (DoubleYoVariable) this.getVariable("qd_wx");
      qd_wy = (DoubleYoVariable) this.getVariable("qd_wy");
      qd_wz = (DoubleYoVariable) this.getVariable("qd_wz");
      qdd_wx = (DoubleYoVariable) this.getVariable("qdd_wx");
      qdd_wy = (DoubleYoVariable) this.getVariable("qdd_wy");
      qdd_wz = (DoubleYoVariable) this.getVariable("qdd_wz");

      q_x.set(0.0);
      q_y.set(0.0);
      q_z.set(1.2 * R);    // 4.0*BASE_H;

      qd_x.set(0.0);    // 10;
      qd_y.set(0.0);
      qd_z.set(-0.10);

      // q_qs.set(0.707; q_qx.set(0.3; q_qy.set(0.4; q_qz.set(0.5;
      // q_qs.set(1.0; q_qx.set(0.0; q_qy.set(0.0; q_qz.set(0.0;

      q_qs.set(0.707106);
      q_qx.set(0.0);
      q_qy.set(0.707106);
      q_qz.set(0.0);

      // q_qs.set(Math.cos(Math.PI/4.0); q_qx.set(1.0 * Math.cos(Math.PI/4.0); q_qy.set(0.0 * Math.cos(Math.PI/4.0); q_qz.set(0.0 * Math.cos(Math.PI/4.0);

      qd_wx.set(0.0);    // 1.0;
      qd_wy.set(0.0);    // 6.0;
      qd_wz.set(0.0);

   }

   
   public void computeEnergy()
   {
      energy.set(M1 * G * q_z.getDoubleValue() + 0.5 * M1 * qd_x.getDoubleValue() * qd_x.getDoubleValue()
                 + 0.5 * M1 * qd_y.getDoubleValue() * qd_y.getDoubleValue() + 0.5 * M1 * qd_z.getDoubleValue() * qd_z.getDoubleValue()
                 + 0.5 * Ixx1 * qd_wx.getDoubleValue() * qd_wx.getDoubleValue() + 0.5 * Iyy1 * qd_wy.getDoubleValue() * qd_wy.getDoubleValue()
                 + 0.5 * Izz1 * qd_wz.getDoubleValue() * qd_wz.getDoubleValue());

      qdd2_wx.set((Iyy1 - Izz1) / Ixx1 * qd_wy.getDoubleValue() * qd_wz.getDoubleValue());
      qdd2_wy.set((Izz1 - Ixx1) / Iyy1 * qd_wz.getDoubleValue() * qd_wx.getDoubleValue());
      qdd2_wz.set((Ixx1 - Iyy1) / Izz1 * qd_wx.getDoubleValue() * qd_wy.getDoubleValue());

   }


}
