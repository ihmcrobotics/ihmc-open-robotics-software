package us.ihmc.atlas;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.atlas.AtlasRobotModel.AtlasTarget;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.wholeBodyInverseKinematicsSimulationController.WholeBodyIKIngressEgressControllerSimulation;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.robotics.humanoidRobot.partNames.LimbName;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyIKPacketCreator;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ComputeOption;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ComputeResult;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicShape;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;

public class AtlasWholeBodyIKIngressEgressCtrlSim
{
   private final WholeBodyIkSolver wholeBodyIKSolver;
   private final WholeBodyIKPacketCreator wholeBodyIKPacketCreator;
   private final SDFFullRobotModel actualRobotModel;
   private final PacketCommunicator fieldObjectCommunicator;
   private final ArrayList<Packet> packetsToSend = new ArrayList<Packet>();
   private final ArrayList<FramePose> desiredReferenceFrameList = new ArrayList<FramePose>();
   private final WholeBodyIKIngressEgressControllerSimulation hikIngEgCtrlSim;
   private final boolean USE_INGRESS_ONLY = false;
   private final YoVariableRegistry registry;
   private final YoFramePoint framePoint;
   private final YoFrameOrientation frameOrientation;
   private final YoGraphicShape yoGraphicsShapeDesired;
   private final boolean random = false;
   private final double ERROR_DISTANCE_TOLERANCE = 0.005;
   private final SDFFullRobotModel desiredFullRobotModel;
   private final YoGraphicShape yoGraphicsShapeActual;
   private final YoFramePoint framePoint2;
   private final YoFrameOrientation frameOrientation2;
   private final double trajectoryTime = 2.0;
   private ComputeResult success;

   public AtlasWholeBodyIKIngressEgressCtrlSim() throws Exception
   {
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, AtlasTarget.SIM, false);
      this.desiredFullRobotModel = robotModel.createFullRobotModel();
      this.hikIngEgCtrlSim = new WholeBodyIKIngressEgressControllerSimulation(robotModel);
      this.registry = hikIngEgCtrlSim.getSimulationConstructionSet().getRootRegistry();
      Graphics3DObject linkGraphicsDesired= new Graphics3DObject();
      Graphics3DObject linkGraphicsActual = new Graphics3DObject();
      linkGraphicsDesired.addSphere(0.05, YoAppearance.Blue());
      linkGraphicsActual.addSphere(0.05, YoAppearance.Magenta());
      framePoint = new YoFramePoint("dontCarePoint", ReferenceFrame.getWorldFrame(), registry);
      frameOrientation = new YoFrameOrientation("orientiation", ReferenceFrame.getWorldFrame(), registry);
      framePoint2 = new YoFramePoint("dontCarePoint2", ReferenceFrame.getWorldFrame(), registry);
      frameOrientation2 = new YoFrameOrientation("orientiation2", ReferenceFrame.getWorldFrame(), registry);
      yoGraphicsShapeDesired = new YoGraphicShape("desiredBall", linkGraphicsDesired, framePoint, frameOrientation, 1.0);
      yoGraphicsShapeActual = new YoGraphicShape("actualBall", linkGraphicsActual, framePoint2, frameOrientation2, 1.0);
      hikIngEgCtrlSim.getSimulationConstructionSet().addYoGraphic(yoGraphicsShapeDesired);
      hikIngEgCtrlSim.getSimulationConstructionSet().addYoGraphic(yoGraphicsShapeActual);
      hikIngEgCtrlSim.getDRCSimulation().start();
      this.actualRobotModel = hikIngEgCtrlSim.getDRCSimulation().getThreadDataSynchronizer().getEstimatorFullRobotModel();
      
      this.fieldObjectCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT, new IHMCCommunicationKryoNetClassList());
      this.fieldObjectCommunicator.connect();
      
      
      this.wholeBodyIKSolver = robotModel.createWholeBodyIkSolver();
      wholeBodyIKSolver.getConfiguration().setNumberOfControlledDoF(RobotSide.RIGHT, WholeBodyIkSolver.ControlledDoF.DOF_3P);
      wholeBodyIKSolver.getConfiguration().setNumberOfControlledDoF(RobotSide.LEFT, WholeBodyIkSolver.ControlledDoF.DOF_NONE);
      wholeBodyIKSolver.getHierarchicalSolver().setVerbosityLevel(0);

      this.wholeBodyIKPacketCreator = new WholeBodyIKPacketCreator(robotModel);
      createDesiredFramesList();
      System.out.println(getClass().getSimpleName() + ": Starting sleep for 10 secs.");
      ThreadTools.sleep(10000);
      System.out.println(getClass().getSimpleName() + ": Attempting to start test.");
      
      boolean testNotStarted = true;
      while(testNotStarted){
         if(USE_INGRESS_ONLY){
            if(ingressEgressModeActivated()){
               executeTest();
               testNotStarted = false;
            }
         }else{
            executeTest();
            testNotStarted = false;
         }
      }
   }
   
   private void executeTest(){
      for (int i = 0; i < desiredReferenceFrameList.size(); i++)
      {
         ThreadTools.sleep(1000);
         doControl(i);
         ThreadTools.sleep((long) (3*trajectoryTime*1000.0));
         checkIfTargetWasReached(i);
      }
   }

   public static void main(String[] args) throws Exception
   {
      new AtlasWholeBodyIKIngressEgressCtrlSim();
   }

   private void createDesiredFramesList()
   {
      final double reachLength = 0.5;
      for (int i = 0; i < 4; i++)
      {
         Point3d point = new Point3d(reachLength * Math.cos(-i * Math.PI / 4), reachLength * Math.sin(-i * Math.PI / 4), 0.6);
         FramePose desiredPose = new FramePose( ReferenceFrame.getWorldFrame(), point , new Quat4d() );
         desiredReferenceFrameList.add(desiredPose);
      }
   }

   private FramePose getNextDesiredReferenceFrame(int index)
   {
      FramePose desiredPose;
      if (random)
      {
         Random random = new Random();
         Point3d randomPoint = RandomTools.generateRandomPoint(random, -0.2, -0.2, 0.2, 0.2, 1.0, 1.5);
         desiredPose =  new FramePose( ReferenceFrame.getWorldFrame(), randomPoint , new Quat4d() );
      }
      else
      {
         desiredPose = desiredReferenceFrameList.get(index);
      }
      return desiredPose;
   }

   private void doControl(int index)
   {
      FramePose desiredPose = getNextDesiredReferenceFrame(index);
      wholeBodyIKSolver.setGripperPalmTarget( RobotSide.RIGHT, desiredPose);
      try
      {
         success = wholeBodyIKSolver.compute(actualRobotModel, desiredFullRobotModel, ComputeOption.USE_ACTUAL_MODEL_JOINTS);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
      wholeBodyIKPacketCreator.createPackets(desiredFullRobotModel, trajectoryTime, packetsToSend);
      System.out.println("AtlasWholeBodyIKIngressEgressCtrlSim: Sending packets");
      for (int i = 0; i < packetsToSend.size(); i++)
      {
         fieldObjectCommunicator.send(packetsToSend.get(i));
      }
      packetsToSend.clear();
      //Visualize where wrist should be going
      ReferenceFrame desiredWristReference = wholeBodyIKSolver.getDesiredBodyFrame("r_hand", ReferenceFrame.getWorldFrame());
  //     ReferenceFrame desiredWristReference = wholeBodyIKSolver.getDesiredHandFrame(RobotSide.RIGHT, ReferenceFrame.getWorldFrame());
      yoGraphicsShapeDesired.setToReferenceFrame(desiredWristReference);
   }

   private void checkIfTargetWasReached(int index)
   {
      Vector3d vector = new Vector3d();
      RigidBodyTransform rBT;
      //--------------------------------------
      System.out.println("-----------------\n");


      ReferenceFrame desiredWristReference = wholeBodyIKSolver.getDesiredBodyFrame("r_hand", ReferenceFrame.getWorldFrame());
//      ReferenceFrame desiredWristReference = wholeBodyIKSolver.getDesiredHandFrame(RobotSide.RIGHT, ReferenceFrame.getWorldFrame());
      rBT = desiredWristReference.getTransformToDesiredFrame( ReferenceFrame.getWorldFrame());

      rBT.getTranslation(vector);
      System.out.format("Desired position : %.3f  %.3f  %.3f%n",
            vector.getX(), vector.getY(), vector.getZ());
      //--------------------------------------

//      ReferenceFrame rightHandPosition = actualFullRobotModel.getHandControlFrame(RobotSide.RIGHT);
      ReferenceFrame rightHandPosition = actualRobotModel.getEndEffectorFrame(RobotSide.RIGHT, LimbName.ARM);
      rBT = rightHandPosition.getTransformToDesiredFrame( ReferenceFrame.getWorldFrame());

      yoGraphicsShapeActual.setToReferenceFrame(rightHandPosition);
      
      rBT.getTranslation(vector);
      System.out.format("Actual position : %.3f  %.3f  %.3f%n%n",
            vector.getX(), vector.getY(), vector.getZ());
      //-------------------------------
      rBT = desiredFullRobotModel.getSoleFrame(RobotSide.LEFT).getTransformToDesiredFrame( ReferenceFrame.getWorldFrame());

      rBT.getTranslation(vector);
      System.out.format("Position of the LEFT Sole [%.3f] : %.3f  %.3f  %.3f%n",
            vector.length(), vector.getX(), vector.getY(), vector.getZ());
      
      rBT = desiredFullRobotModel.getSoleFrame(RobotSide.RIGHT).getTransformToDesiredFrame( ReferenceFrame.getWorldFrame());

      rBT.getTranslation(vector);
      System.out.format("Position of the RIGHT Sole [%.3f] : %.3f  %.3f  %.3f%n",
            vector.length(), vector.getX(), vector.getY(), vector.getZ());

      //-------------------------------
      rBT = rightHandPosition.getTransformToDesiredFrame(desiredWristReference);

      rBT.getTranslation(vector);
      System.out.format("Error in final position [%.3f] : %.3f  %.3f  %.3f%n",
            vector.length(), vector.getX(), vector.getY(), vector.getZ());

      //-------------------------------
//      String[] jointNames = {
//            "r_leg_akx", "r_leg_aky",
//            "r_leg_kny",
//            "r_leg_hpy", "r_leg_hpx", "r_leg_hpz",
//
//            "l_leg_akx", "l_leg_aky",
//            "l_leg_kny",
//            "l_leg_hpy", "l_leg_hpx", "l_leg_hpz",
//
//            "back_bkz", "back_bky",  "back_bkx",
//            "r_arm_shz", "r_arm_shx", "r_arm_ely",
//            "r_arm_elx",
//            "r_arm_wry", "r_arm_wrx" };

//      Vector3d A = new Vector3d();
//      Vector3d B = new Vector3d();

//      for(int i=0; i< jointNames.length; i++)
//      {    
//         System.out.print("-----------------\n" +  jointNames[i] );
//         ReferenceFrame workingFrame =  wholeBodyIKSolver.getDesiredAfterJointFrame( jointNames[i], ReferenceFrame.getWorldFrame());
//         RigidBodyTransform tempTransform =  workingFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
//
//         tempTransform.getTranslation(A);
//         System.out.format("\n ACTUAL:  %.3f   %.3f   %.3f\n" ,A.getX(), A.getY(), A.getZ() );
//
//         ReferenceFrame actualFrame = actualFullRobotModel.getOneDoFJointByName(jointNames[i]).getFrameAfterJoint();
//         tempTransform =  actualFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
//
//         tempTransform.getTranslation(B);
//         System.out.format(" DESIRED: %.3f   %.3f   %.3f\n" ,B.getX(), B.getY(), B.getZ() );
//
//         A.sub(B);
//         System.out.format(" angle error is %+.1f degrees\terror in location is  %.3f   %.3f   %.3f\n", 
//
//               (180.0/Math.PI)*(wholeBodyIKSolver.getDesiredJointAngle( jointNames[i] ) - 
//                     actualFullRobotModel.getOneDoFJointByName(jointNames[i]).getQ() ),
//                     A.getX(), A.getY(), A.getZ() );  
//      }

      if (vector.length() > ERROR_DISTANCE_TOLERANCE)
      {
         System.out.println(this.getClass().getName() + ": " + (success == ComputeResult.SUCCEEDED ? "HIK REPORTS POSSIBLE BUT" : "HIK REPORTS IMPOSSIBLE THUS") + " FAILED TO REACH DESIRED POINT " );
      }
      else
      {
         System.out.println(this.getClass().getName() + ": SUCCESFULLY REACHED POINT");
      }
   }

   private boolean ingressEgressModeActivated()
   {
      ArrayList<YoVariable<?>> yoVariables = hikIngEgCtrlSim.getDRCSimulation().getSimulationConstructionSet().getAllVariables();
      boolean bool = false;
      for (YoVariable<?> yoVariable : yoVariables)
      {
         if (yoVariable.getName().equals("highLevelState"))
         {
            @SuppressWarnings("unchecked")
            EnumYoVariable<HighLevelState> enumYoVariable = (EnumYoVariable<HighLevelState>) yoVariable;
            // enumYoVariable.set(HighLevelState.INGRESS_EGRESS);
            bool = (enumYoVariable.getEnumValue() == HighLevelState.INGRESS_EGRESS);
         }
      }
      return bool;
   }
}
