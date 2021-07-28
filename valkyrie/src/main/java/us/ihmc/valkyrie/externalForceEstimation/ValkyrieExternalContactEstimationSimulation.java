package us.ihmc.valkyrie.externalForceEstimation;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.contactEstimation.AvatarExternalContactEstimationSimulation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

public class ValkyrieExternalContactEstimationSimulation extends AvatarExternalContactEstimationSimulation
{
   private static final double simDT = 2e-4; // normally 6.6e-4. (controlDT=4e-3)

//       static String jointName = "torsoRoll";          static Point3D offset = new Point3D(0.156, 0.093, 0.197);
//       static String jointName = "pelvis";             static Point3D offset =new Point3D(0.131, 0.000, -0.044);
//       static String jointName = "torsoRoll";          static Point3D offset = new Point3D(0.113, -0.050, 0.090);
//       static String jointName = "torsoRoll";          static Point3D offset = new Point3D(0.137, 0.050, 0.329);
       static String jointName = "torsoRoll";          static Point3D offset = new Point3D(0.124, -0.176, 0.259);
//       static String jointName = "leftForearmYaw";     static Point3D offset = new Point3D(0.081, 0.020, 0.026);
//       static String jointName = "leftForearmYaw";     static Point3D offset = new Point3D(0.076, 0.120, -0.036);
//       static String jointName = "leftForearmYaw";     static Point3D offset = new Point3D(-0.068, 0.170, -0.033);
//       static String jointName = "leftForearmYaw";     static Point3D offset = new Point3D(0.071, 0.259, 0.009);
//       static String jointName = "rightForearmYaw";    static Point3D offset = new Point3D(0.081, -0.020, 0.026);
//       static String jointName = "rightForearmYaw";    static Point3D offset = new Point3D(0.076, -0.120, -0.036);
//       static String jointName = "rightForearmYaw";    static Point3D offset = new Point3D(-0.068, -0.170, -0.033);
//       static String jointName = "rightForearmYaw";    static Point3D offset = new Point3D(0.071, -0.245, -0.034);
//       static String jointName = "pelvis";             static Point3D offset =new Point3D(0.140, 0.000, -0.100);
//       static String jointName = "leftHipPitch";       static Point3D offset = new Point3D(0.100, 0.122, 0.045);
//       static String jointName = "pelvis";             static Point3D offset =new Point3D(-0.130, 0.052, -0.200);
//       static String jointName = "pelvis";             static Point3D offset =new Point3D(0.133, 0.000, -0.261);
//       static String jointName = "leftHipPitch";       static Point3D offset = new Point3D(0.019, 0.187, -0.072);
//       static String jointName = "leftHipPitch";       static Point3D offset = new Point3D(0.057, 0.192, -0.131);
//       static String jointName = "leftHipPitch";       static Point3D offset = new Point3D(-0.030, 0.192, -0.235);
//       static String jointName = "leftHipPitch";       static Point3D offset = new Point3D(0.144, 0.132, -0.208);
//       static String jointName = "leftHipPitch";       static Point3D offset = new Point3D(0.121, 0.172, -0.242);
//       static String jointName = "leftHipPitch";       static Point3D offset = new Point3D(0.048, 0.199, -0.289);
//       static String jointName = "rightHipPitch";      static Point3D offset = new Point3D(0.019, -0.187, -0.072);
//       static String jointName = "rightHipPitch";      static Point3D offset = new Point3D(0.057, -0.192, -0.131);
//       static String jointName = "rightHipPitch";      static Point3D offset = new Point3D(-0.030, -0.192, -0.235);
//       static String jointName = "rightHipPitch";      static Point3D offset = new Point3D(0.144, -0.132, -0.208);
//       static String jointName = "rightHipPitch";      static Point3D offset = new Point3D(0.121, -0.172, -0.242);
//       static String jointName = "rightHipPitch";      static Point3D offset = new Point3D(0.048, -0.199, -0.289);
//       static String jointName = "leftHipPitch";       static Point3D offset = new Point3D(0.109, 0.083, -0.422);
//       static String jointName = "leftHipPitch";       static Point3D offset = new Point3D(0.043, 0.161, -0.444);
//       static String jointName = "leftKneePitch";      static Point3D offset = new Point3D(-0.044, 0.104, -0.127);
//       static String jointName = "leftKneePitch";      static Point3D offset = new Point3D(0.084, 0.058, -0.160);
//       static String jointName = "leftKneePitch";      static Point3D offset = new Point3D(0.079, -0.014, -0.348);
//       static String jointName = "rightHipPitch";      static Point3D offset = new Point3D(0.109, -0.083, -0.422);
//       static String jointName = "rightHipPitch";      static Point3D offset = new Point3D(0.043, -0.161, -0.444);
//       static String jointName = "rightKneePitch";     static Point3D offset = new Point3D(-0.044, -0.104, -0.127);
//       static String jointName = "rightKneePitch";     static Point3D offset = new Point3D(0.084, -0.058, -0.160);
//       static String jointName = "rightKneePitch";     static Point3D offset = new Point3D(0.079, 0.014, -0.348);

   public ValkyrieExternalContactEstimationSimulation()
   {
      super(jointName, offset, simDT, true);
   }

   /**
    * When using experimental physics engine
    */
   private static void addExternalForcePoints(DRCSimulationStarter simulationStarter, Pair<String, Tuple3DReadOnly>... externalForcePointsToAdd)
   {
      HumanoidFloatingRootJointRobot sdfRobot = simulationStarter.getSDFRobot();
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      YoGraphicsList efpGraphicsList = new YoGraphicsList("simulatedExternalForce");

      for (int i = 0; i < externalForcePointsToAdd.length; i++)
      {
         String efpName = "efp" + i;
         String jointName = externalForcePointsToAdd[i].getLeft();
         Tuple3DReadOnly offset = externalForcePointsToAdd[i].getRight();

         ExternalForcePoint externalForcePoint = new ExternalForcePoint(efpName, offset, sdfRobot);
         sdfRobot.getJoint(jointName).addExternalForcePoint(externalForcePoint);

         YoGraphicVector simulatedForceVector = new YoGraphicVector("force" + i,
                                                                    externalForcePoint.getYoPosition(),
                                                                    externalForcePoint.getYoForce(),
                                                                    0.05,
                                                                    YoAppearance.Red());
         YoGraphicPosition simulatedForcePoint = new YoGraphicPosition("forcePoint" + i, externalForcePoint.getYoPosition(), 0.025, YoAppearance.Red());
         efpGraphicsList.add(simulatedForceVector);
         efpGraphicsList.add(simulatedForcePoint);
      }

      graphicsListRegistry.registerYoGraphicsList(efpGraphicsList);
      simulationStarter.getSimulationConstructionSet().addYoGraphicsListRegistry(graphicsListRegistry);
   }

   @Override
   protected DRCRobotModel getRobotModel()
   {
      ValkyrieRobotVersion version = ValkyrieRobotVersion.FINGERLESS;
      return new ValkyrieRobotModel(RobotTarget.SCS, version);
   }

   public static void main(String[] args)
   {
      new ValkyrieExternalContactEstimationSimulation();
   }
}

