package us.ihmc.atlas;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.atlas.AtlasRobotModel.AtlasTarget;
import us.ihmc.communication.net.KryoLocalObjectCommunicator;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.darpaRoboticsChallenge.WholeBodyIK.WholeBodyIKPacketCreator;
import us.ihmc.darpaRoboticsChallenge.WholeBodyIK.WholeBodyIkSolver;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNodeType;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.humanoidRobot.partNames.LimbName;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphic;
import us.ihmc.yoUtilities.graphics.YoGraphicShape;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
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
   private final boolean DEBUG = true;
   private boolean USE_INGRESS_ONLY = false;
   private final YoVariableRegistry registry;
   private final YoFramePoint framePoint;
   private final YoFrameOrientation frameOrientation;
   private final YoGraphicShape yoGraphicsShape;
   private final DoubleYoVariable hik_x_des, hik_y_des, hik_z_des;

   public AtlasWholeBodyIKIngressEgressCtrlSim() throws IOException
   {
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_DUAL_ROBOTIQ, AtlasTarget.SIM, false);

      this.hikIngEgCtrlSim = new WholeBodyIKIngressEgressControllerSimulation(robotModel);
      this.registry = hikIngEgCtrlSim.getControllerFactory().getRegistry();
      hik_x_des = new DoubleYoVariable("hik_x_des", registry);
      hik_y_des = new DoubleYoVariable("hik_y_des", registry);
      hik_z_des = new DoubleYoVariable("hik_z_des", registry);
      hik_x_des.set(0.3908);
      hik_y_des.set(-0.3445);
      hik_z_des.set(0.6438);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addSphere(0.05, YoAppearance.Blue());
      framePoint = new YoFramePoint("dontCarePoint", ReferenceFrame.getWorldFrame(), registry);
      frameOrientation = new YoFrameOrientation("orientiation", ReferenceFrame.getWorldFrame(), registry);
      yoGraphicsShape = new YoGraphicShape("dontCareMarker", linkGraphics, framePoint, frameOrientation, 1.0);
      hikIngEgCtrlSim.getSimulationConstructionSet().addYoGraphic(yoGraphicsShape);
      hikIngEgCtrlSim.getDRCSimulation().start();
      this.fullRobotModel = hikIngEgCtrlSim.getDRCSimulation().getThreadDataSynchronizer().getEstimatorFullRobotModel();
      //      this.fullRobotModel = robotModel.createFullRobotModel();
      //      this.registry = ikExample.getControllerFactory().getRegistry();
      this.fieldObjectCommunicator = hikIngEgCtrlSim.getKryoLocalObjectCommunicator();
      this.wholeBodyIKSolver = new WholeBodyIkSolver(robotModel, fullRobotModel, "models/atlas_v4_wb.urdf");
      
      // NEW API
     // wholeBodyIKSolver.enableHandRotation(RobotSide.RIGHT, true);
     // wholeBodyIKSolver.enableHandTranslation(RobotSide.RIGHT, true);
     // wholeBodyIKSolver.enableHandRotAndTranslation(RobotSide.LEFT, false);
      
      
      wholeBodyIKSolver.setNumberOfControlledDoF( RobotSide.RIGHT, WholeBodyIkSolver.ControlledDoF.DOF_3P3R );
      wholeBodyIKSolver.setNumberOfControlledDoF( RobotSide.LEFT,  WholeBodyIkSolver.ControlledDoF.DOF_NONE );
      
      this.wholeBodyIKPacketCreator = new WholeBodyIKPacketCreator(robotModel);

      while (true)
      {
         if (USE_INGRESS_ONLY)
         {
            if (ingressEgressModeActivated())
            {

               ThreadTools.sleep(10000);
               doControl();
            }
         }
         else
         {
            ThreadTools.sleep(10000);
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
      Random random = new Random();
      Point3d randomPoint = RandomTools.generateRandomPoint(random, -0.2, -0.2, 0.2, 0.2, 1.0, 1.5);
      FramePoint point = new FramePoint(ReferenceFrame.getWorldFrame(), randomPoint, "dontCareFramePoint");
      //      Vector3d translationVector = new Vector3d();
      //      ReferenceFrame desiredReferenceFrame = fullRobotModel.getEndEffector(RobotSide.LEFT, LimbName.ARM).getBodyFixedFrame();
//      FramePoint point = new FramePoint(ReferenceFrame.getWorldFrame(), hik_x_des.getDoubleValue(), hik_y_des.getDoubleValue(), hik_z_des.getDoubleValue());
      FrameVector zAxis = new FrameVector(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.0);
      ReferenceFrame desiredReferenceFrame = ReferenceFrame.constructReferenceFrameFromPointAndZAxis("dontCare", point, zAxis); //new ReferenceFrame("desriedFrame",ReferenceFrame.getWorldFrame(),transformToWorldFrame, false, false, true);
      yoGraphicsShape.setToReferenceFrame(desiredReferenceFrame);
      //      RigidBodyTransform transformToWorld = desiredReferenceFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      //      transformToWorld.getTranslation(translationVector);
      //      System.out.println("AtlasWholeBodyIKIngressEgressCtrlSim: x: " + translationVector.getX() + ", y: " + translationVector.getY() + ", z: "
      //            + translationVector.getZ());
      //      desiredToWorldTransform.setTranslation(randomPoint.getX(), randomPoint.getY(), randomPoint.getZ());
      //      desiredToWorldTransform.setTranslation(0.0, 0.0, 1.0);
      //      desiredToWorldTransform.setTranslation(rand.nextDouble(), rand.nextDouble(), rand.nextDouble());
      //      desiredToWorldTransform.setTranslation(hik_arm_pos_x_d.getDoubleValue(), hik_arm_pos_y_d.getDoubleValue(), hik_arm_pos_z_d.getDoubleValue());
      //      System.out.println(hik_arm_pos_x_d.getDoubleValue() + ", " + hik_arm_pos_y_d.getDoubleValue() + ", " + hik_arm_pos_z_d.getDoubleValue());
      wholeBodyIKSolver.setHandTarget(RobotSide.RIGHT, desiredReferenceFrame);
      wholeBodyIKSolver.compute(fullRobotModel);
      wholeBodyIKPacketCreator.createPackets(fullRobotModel, 3.0, packetsToSend);
      System.out.println("AtlasWholeBodyIKIngressEgressCtrlSim: Sending packets");
      for (int i = 0; i < packetsToSend.size(); i++)
      {
         if (DEBUG)
         {
            fieldObjectCommunicator.consumeObject(packetsToSend.get(i));
         }
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