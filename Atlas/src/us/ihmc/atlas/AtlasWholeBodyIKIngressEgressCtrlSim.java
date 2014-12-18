package us.ihmc.atlas;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.atlas.AtlasRobotModel.AtlasTarget;
import us.ihmc.communication.net.KryoLocalObjectCommunicator;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.darpaRoboticsChallenge.WholeBodyIK.WholeBodyIKPacketCreator;
import us.ihmc.darpaRoboticsChallenge.WholeBodyIK.WholeBodyIkSolver;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.humanoidRobot.partNames.LimbName;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
//import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
//import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
//import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import wholeBodyInverseKinematicsSimulationController.WholeBodyIKIngressEgressControllerSimulation;

public class AtlasWholeBodyIKIngressEgressCtrlSim
{

   private final WholeBodyIkSolver wholeBodyIKSolver;
   private final WholeBodyIKPacketCreator wholeBodyIKPacketCreator;
   //   private final YoVariableRegistry registry;
   //   private DoubleYoVariable hik_arm_pos_x_d;
   //   private DoubleYoVariable hik_arm_pos_y_d;
   //   private DoubleYoVariable hik_arm_pos_z_d;
   private final SDFFullRobotModel fullRobotModel;
   private final KryoLocalObjectCommunicator fieldObjectCommunicator;
   private ArrayList<Object> packetsToSend = new ArrayList<Object>();
   private WholeBodyIKIngressEgressControllerSimulation hikIngEgCtrlSim;

   public AtlasWholeBodyIKIngressEgressCtrlSim() throws IOException
   {
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_DUAL_ROBOTIQ, AtlasTarget.SIM, false);

      this.hikIngEgCtrlSim = new WholeBodyIKIngressEgressControllerSimulation(robotModel);
      hikIngEgCtrlSim.getDRCSimulation().start();
      this.fullRobotModel = robotModel.createFullRobotModel();
      //      this.registry = ikExample.getControllerFactory().getRegistry();
      this.fieldObjectCommunicator = hikIngEgCtrlSim.getKryoLocalObjectCommunicator();
      this.wholeBodyIKSolver = new WholeBodyIkSolver(robotModel, fullRobotModel, "models/atlas_v4_dual.urdf");
      wholeBodyIKSolver.enableHandRotAndTranslation(RobotSide.LEFT, false);
      this.wholeBodyIKPacketCreator = new WholeBodyIKPacketCreator(robotModel);
      while (true)
      {
         if (ingressEgressModeActivated())
         {

            ThreadTools.sleep(10000);
            //         Random random = new Random();
            //         ComHeightPacket chp = new ComHeightPacket(random);
            //         System.out.println("Sending random ComHeightPacket.");
            //         fieldObjectCommunicator.consumeObject(chp);
            doControl();
         }
      }
   }

   public static void main(String[] args) throws IOException
   {
      new AtlasWholeBodyIKIngressEgressCtrlSim();
   }

   private void doControl()
   {
      Random rand = new Random();
      Point3d randomPoint = RandomTools.generateRandomPoint(rand, -0.2, -0.2, 0.2, 0.2, 1.0, 1.5);
      ReferenceFrame desiredReferenceFrame = fullRobotModel.getEndEffector(RobotSide.RIGHT, LimbName.ARM).getBodyFixedFrame();
//      desiredToWorldTransform.setTranslation(randomPoint.getX(), randomPoint.getY(), randomPoint.getZ());
      //      desiredToWorldTransform.setTranslation(0.0, 0.0, 1.0);
      //      desiredToWorldTransform.setTranslation(rand.nextDouble(), rand.nextDouble(), rand.nextDouble());
      //      desiredToWorldTransform.setTranslation(hik_arm_pos_x_d.getDoubleValue(), hik_arm_pos_y_d.getDoubleValue(), hik_arm_pos_z_d.getDoubleValue());
      //      System.out.println(hik_arm_pos_x_d.getDoubleValue() + ", " + hik_arm_pos_y_d.getDoubleValue() + ", " + hik_arm_pos_z_d.getDoubleValue());
      wholeBodyIKSolver.setHandTarget(RobotSide.LEFT, desiredReferenceFrame);
      wholeBodyIKSolver.compute(fullRobotModel);
      wholeBodyIKPacketCreator.createPackets(fullRobotModel, 3.0, packetsToSend);
      for (int i = 0; i < packetsToSend.size(); i++)
      {
         fieldObjectCommunicator.consumeObject(packetsToSend.get(i));
      }
      packetsToSend.clear();
   }

   private boolean ingressEgressModeActivated()
   {
      ArrayList<YoVariable<?>> yoVariables = hikIngEgCtrlSim.getDRCSimulation().getSimulationConstructionSet().getAllVariables();
      boolean bool = false;
      for (YoVariable<?> yoVariable : yoVariables)
      {
         if (yoVariable.getName() == "highLevelState")
         {

            @SuppressWarnings("unchecked")
            EnumYoVariable<HighLevelState> enumYoVariable = (EnumYoVariable<HighLevelState>) yoVariable;
            //            enumYoVariable.set(HighLevelState.INGRESS_EGRESS);
            if (enumYoVariable.getEnumValue() == HighLevelState.INGRESS_EGRESS)
            {
               bool = true;
            }
            else
            {
               bool = false;
            }
         }
      }
      return bool;
   }

}