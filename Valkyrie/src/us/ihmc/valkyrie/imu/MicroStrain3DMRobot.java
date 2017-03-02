package us.ihmc.valkyrie.imu;

import java.io.IOException;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.valkyrie.imu.MicroStrainData.MicrostrainFilterType;
import us.ihmc.valkyrie.parameters.ValkyrieSensorInformation;

@SuppressWarnings("serial")
public class MicroStrain3DMRobot extends Robot
{
   private static double MS3DM_MASS = 0.0018;
   private static double MS3DM_LENGTH = 0.044;
   private static double MS3DM_WIDTH = 0.025;
   private static double MS3DM_HEIGHT = 0.011;

   protected FloatingJoint ms3DM;
   protected final RotationMatrix rotation = new RotationMatrix();

   private YoVariableRegistry registry = new YoVariableRegistry("MicroStrain3DMData");

   private DoubleYoVariable yaw = new DoubleYoVariable("yaw", registry);
   private DoubleYoVariable pitch = new DoubleYoVariable("pitch", registry);
   private DoubleYoVariable roll = new DoubleYoVariable("roll", registry);

   private DoubleYoVariable wz = new DoubleYoVariable("wz", registry);
   private DoubleYoVariable wy = new DoubleYoVariable("wy", registry);
   private DoubleYoVariable wx = new DoubleYoVariable("wx", registry);

   private DoubleYoVariable xdd = new DoubleYoVariable("xdd", registry);
   private DoubleYoVariable ydd = new DoubleYoVariable("ydd", registry);
   private DoubleYoVariable zdd = new DoubleYoVariable("zdd", registry);
   
   private final RotationMatrix temporaryMatrix = new RotationMatrix();
   
   public MicroStrain3DMRobot()
   {
      super("MicroStrain3DMRobot");

      ms3DM = new FloatingJoint("ms3DM", new Vector3D(0.0, 0.0, 0.0), this);
      ms3DM.setLink(MS3DMLink());

      this.addRootJoint(ms3DM);

      this.setGravity(0.0, 0.0, 0.0);

      addYoVariableRegistry(registry);

   }

   public double getYaw()
   {
      return yaw.getDoubleValue();
   }

   public double getPitch()
   {
      return pitch.getDoubleValue();
   }

   public double getRoll()
   {
      return roll.getDoubleValue();
   }

   private Link MS3DMLink()
   {
      Link p = new Link("ms3DMLink");
      p.setMass(MS3DM_MASS);
      p.setComOffset(0.0, 0.0, 0.0);
      p.setMomentOfInertia(0.1, 0.1, 0.1);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.1);
      linkGraphics.addCube(MS3DM_LENGTH, MS3DM_WIDTH, MS3DM_HEIGHT, YoAppearance.Purple());

      p.setLinkGraphics(linkGraphics);
      

      return p;

   }
   
   public void set(Vector3D accel, Vector3D angRate, Quaternion orientation)
   {
      temporaryMatrix.set(orientation);
      rotation.set(MicroStrainData.MICROSTRAIN_TO_ZUP_WORLD);
      rotation.multiply(temporaryMatrix);
      
      ms3DM.setRotation(rotation);
      yaw.set(rotation.getYaw());
      pitch.set(rotation.getPitch());
      roll.set(rotation.getRoll());
      
      xdd.set(accel.getX() * MicroStrainData.MICROSTRAIN_GRAVITY);
      ydd.set(accel.getY() * MicroStrainData.MICROSTRAIN_GRAVITY);
      zdd.set(accel.getZ() * MicroStrainData.MICROSTRAIN_GRAVITY);
      
      wx.set(angRate.getX());
      wy.set(angRate.getY());
      wz.set(angRate.getZ());
      
   }
   
   
   public static void main(String[] args) throws IOException
   {
      ValkyrieSensorInformation sensorInformation = new ValkyrieSensorInformation(DRCRobotModel.RobotTarget.REAL_ROBOT);
      final MicrostrainUDPPacketListener listener = MicrostrainUDPPacketListener.createNonRealtimeListener(sensorInformation.getImuUSBSerialIds().get("v1Pelvis_leftIMU"));
      
      final MicroStrain3DMRobot robot = new MicroStrain3DMRobot();
      RobotController controller = new RobotController()
      {
         YoVariableRegistry registry = new YoVariableRegistry("controller");
         @Override
         public void initialize()
         {
         }
         
         @Override
         public YoVariableRegistry getYoVariableRegistry()
         {
            return registry;
         }
         
         @Override
         public String getName()
         {
            return "updater";
         }
         
         @Override
         public String getDescription()
         {
            return getName();
         }
         
         @Override
         public void doControl()
         {
            MicroStrainData data = listener.getLatestData(MicrostrainFilterType.COMPLIMENTARY_FILTER);
            robot.set(data.getLinearAcceleration(), data.getAngularRate(), data.getQuaternion());
            ThreadTools.sleep(1);
         }
      };
      
      robot.setController(controller);
      
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setGroundVisible(false);
      scs.startOnAThread();
      
   }
}
