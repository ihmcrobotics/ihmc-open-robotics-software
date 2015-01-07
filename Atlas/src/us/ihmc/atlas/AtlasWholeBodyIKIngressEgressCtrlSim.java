package us.ihmc.atlas;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.atlas.AtlasRobotModel.AtlasTarget;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.darpaRoboticsChallenge.WholeBodyIK.WholeBodyIKPacketCreator;
import us.ihmc.darpaRoboticsChallenge.WholeBodyIK.WholeBodyIkSolver;
import us.ihmc.darpaRoboticsChallenge.WholeBodyIK.WholeBodyIkSolver.ComputeOption;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.wholeBodyInverseKinematicsSimulationController.WholeBodyIKIngressEgressControllerSimulation;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
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
import us.ihmc.yoUtilities.graphics.YoGraphicShape;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;

public class AtlasWholeBodyIKIngressEgressCtrlSim
{
   private final WholeBodyIkSolver wholeBodyIKSolver;
   private final WholeBodyIKPacketCreator wholeBodyIKPacketCreator;
   private final SDFFullRobotModel fullRobotModel;
   private final KryoLocalPacketCommunicator fieldObjectCommunicator;
   private final ArrayList<Packet> packetsToSend = new ArrayList<Packet>();
   private final ArrayList<ReferenceFrame> desiredReferenceFrameList = new ArrayList<ReferenceFrame>();
   private WholeBodyIKIngressEgressControllerSimulation hikIngEgCtrlSim;
   private boolean USE_INGRESS_ONLY = false;
   private final YoVariableRegistry registry;
   private final YoFramePoint framePoint;
   private final YoFrameOrientation frameOrientation;
   private final YoGraphicShape yoGraphicsShape;
   private final DoubleYoVariable hik_x_des, hik_y_des, hik_z_des;
   private final boolean random = false;
   private final double ERROR_DISTANCE_TOLERANCE = 0.25;

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
      this.fieldObjectCommunicator = hikIngEgCtrlSim.getKryoLocalObjectCommunicator();
      this.wholeBodyIKSolver = new WholeBodyIkSolver( robotModel, fullRobotModel );
      wholeBodyIKSolver.setNumberOfControlledDoF(RobotSide.RIGHT, WholeBodyIkSolver.ControlledDoF.DOF_3P);
      wholeBodyIKSolver.setNumberOfControlledDoF(RobotSide.LEFT, WholeBodyIkSolver.ControlledDoF.DOF_NONE);
      wholeBodyIKSolver.getHierarchicalSolver().setVerbose(false);
      
      this.wholeBodyIKPacketCreator = new WholeBodyIKPacketCreator(robotModel);
      createDesiredFramesList();
      for (int i = 0; i < desiredReferenceFrameList.size(); i++)
      {
         if (USE_INGRESS_ONLY)
         {
            if (ingressEgressModeActivated())
            {
               ThreadTools.sleep(7000);
               doControl(i);
               ThreadTools.sleep(10000);
               checkIfTargetWasReached(i);
            }
         }
         else
         {
            ThreadTools.sleep(7000);
            doControl(i);
            ThreadTools.sleep(10000);
            checkIfTargetWasReached(i);
         }
      }
   }

   public static void main(String[] args) throws IOException
   {
      new AtlasWholeBodyIKIngressEgressCtrlSim();
   }

   private void createDesiredFramesList()
   {
      final double reachLength = 0.5;
      for (int i = 0; i < 4; i++)
      {
         FramePoint point = new FramePoint(ReferenceFrame.getWorldFrame(), reachLength * Math.cos(-i * Math.PI / 4), reachLength * Math.sin(-i * Math.PI / 4),
               0.3);
         FrameVector zAxis = new FrameVector(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.0);
         ReferenceFrame desiredReferenceFrame = ReferenceFrame.constructReferenceFrameFromPointAndZAxis("dontCareEither", point, zAxis);
         desiredReferenceFrameList.add(desiredReferenceFrame);
      }
   }

   private ReferenceFrame getNextDesiredReferenceFrame(int index)
   {
      ReferenceFrame desiredReferenceFrame;
      if (random)
      {
         Random random = new Random();
         Point3d randomPoint = RandomTools.generateRandomPoint(random, -0.2, -0.2, 0.2, 0.2, 1.0, 1.5);
         FramePoint point = new FramePoint(ReferenceFrame.getWorldFrame(), randomPoint, "dontCareFramePoint");
         FrameVector zAxis = new FrameVector(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.0);
         desiredReferenceFrame = ReferenceFrame.constructReferenceFrameFromPointAndZAxis("dontCare", point, zAxis);
      }
      else
      {
         desiredReferenceFrame = desiredReferenceFrameList.get(index);
      }
      return desiredReferenceFrame;
   }

   private void doControl(int index)
   {
      ReferenceFrame desiredReferenceFrame = getNextDesiredReferenceFrame(index);
      yoGraphicsShape.setToReferenceFrame(desiredReferenceFrame);
      wholeBodyIKSolver.setHandTarget(RobotSide.RIGHT, desiredReferenceFrame);
      wholeBodyIKSolver.compute(fullRobotModel);
      wholeBodyIKPacketCreator.createPackets(fullRobotModel, 3.0, packetsToSend);
      System.out.println("AtlasWholeBodyIKIngressEgressCtrlSim: Sending packets");
      for (int i = 0; i < packetsToSend.size(); i++)
      {
         fieldObjectCommunicator.send(packetsToSend.get(i));
      }
      packetsToSend.clear();
   }

   private void checkIfTargetWasReached(int index)
   {
      ReferenceFrame rightHandPosition = fullRobotModel.getEndEffectorFrame(RobotSide.RIGHT, LimbName.ARM);
      ReferenceFrame desiredReference = getNextDesiredReferenceFrame(index);
      RigidBodyTransform rBT = rightHandPosition.getTransformToDesiredFrame(desiredReference);
      Vector3d vector = new Vector3d();
      rBT.getTranslation(vector);
      System.out.println("error: \n" + vector);
      
      /*ReferenceFrame workingHandFrame = wholeBodyIKSolver.getHandFrame(RobotSide.RIGHT, ReferenceFrame.getWorldFrame());
      
      RigidBodyTransform tempTransform =  workingHandFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      System.out.println("whole body: \n" + tempTransform);
      
      tempTransform =  desiredReference.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      System.out.println("desired: \n" + tempTransform);*/
      
      ReferenceFrame workingFrame = wholeBodyIKSolver.getDesiredPelvisFrame( ReferenceFrame.getWorldFrame());
      RigidBodyTransform tempTransform =  workingFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      System.out.println("whole body pelvis: \n" + tempTransform);
      
      ReferenceFrame actualFrame = fullRobotModel.getRootJoint().getFrameAfterJoint();
      tempTransform =  actualFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      System.out.println("actual pelvis: \n" + tempTransform);
      
     
      workingFrame = wholeBodyIKSolver.getDesiredBodyFrame("r_foot", ReferenceFrame.getWorldFrame());
      tempTransform =  workingFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      System.out.println("whole body akle: \n" + tempTransform);
      
      actualFrame = fullRobotModel.getOneDoFJointByName("r_leg_akx").getFrameAfterJoint();
      tempTransform =  actualFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      System.out.println("actual akle: \n" + tempTransform);
      
      
      if (vector.length() > ERROR_DISTANCE_TOLERANCE)
      {
         System.out.println(this.getClass().getName() + ": FAILED TO REACH DESIRED POINT");
      }
      else
      {
         System.out.println(this.getClass().getName() + ": SUCCESFULLY REACHED POINT");
      }
   };

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
            // enumYoVariable.set(HighLevelState.INGRESS_EGRESS);
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
