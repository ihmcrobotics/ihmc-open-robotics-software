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
import us.ihmc.wholeBodyController.WholeBodyIKPacketCreator;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
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
   private final double ERROR_DISTANCE_TOLERANCE = 0.03;

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
               0.6);
         FrameVector zAxis = new FrameVector(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.0);
         ReferenceFrame desiredReferenceFrame = ReferenceFrame.constructReferenceFrameFromPointAndZAxis("dontCareEither", point, zAxis);
         desiredReferenceFrameList.add(desiredReferenceFrame);
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
      Vector3d vector = new Vector3d();
      RigidBodyTransform rBT;
      //--------------------------------------
      System.out.println("-----------------\n");


      ReferenceFrame desiredReference = getNextDesiredReferenceFrame(index);
      rBT = desiredReference.getTransformToDesiredFrame( ReferenceFrame.getWorldFrame());

      rBT.getTranslation(vector);
      System.out.format("Desired position : %.3f  %.3f  %.3f\n",  
            vector.getX(), vector.getY(), vector.getZ());
      //--------------------------------------

      ReferenceFrame rightHandPosition =   fullRobotModel.getHandControlFrame(RobotSide.RIGHT);
      //  fullRobotModel.getEndEffectorFrame(RobotSide.RIGHT, LimbName.ARM);
      rBT = rightHandPosition.getTransformToDesiredFrame( ReferenceFrame.getWorldFrame());

      rBT.getTranslation(vector);
      System.out.format("Actual position : %.3f  %.3f  %.3f\n\n",  
            vector.getX(), vector.getY(), vector.getZ());
      //-------------------------------
      rBT = fullRobotModel.getSoleFrame(RobotSide.LEFT).getTransformToDesiredFrame( ReferenceFrame.getWorldFrame());

      rBT.getTranslation(vector);
      System.out.format("Position of the LEFT Sole [%.3f] : %.3f  %.3f  %.3f\n",  
            vector.length(), vector.getX(), vector.getY(), vector.getZ());
      
      rBT = fullRobotModel.getSoleFrame(RobotSide.RIGHT).getTransformToDesiredFrame( ReferenceFrame.getWorldFrame());

      rBT.getTranslation(vector);
      System.out.format("Position of the RIGHT Sole [%.3f] : %.3f  %.3f  %.3f\n",  
            vector.length(), vector.getX(), vector.getY(), vector.getZ());

      //-------------------------------
      rBT = rightHandPosition.getTransformToDesiredFrame(desiredReference);

      rBT.getTranslation(vector);
      System.out.format("Error in final position [%.3f] : %.3f  %.3f  %.3f\n",  
            vector.length(), vector.getX(), vector.getY(), vector.getZ());

      //-------------------------------
      String[] jointNames = { 
            "r_leg_akx", "r_leg_aky",
            "r_leg_kny",
            "r_leg_hpy", "r_leg_hpx", "r_leg_hpz",    

            "l_leg_akx", "l_leg_aky",
            "l_leg_kny",
            "l_leg_hpy", "l_leg_hpx", "l_leg_hpz",  

            "back_bkz", "back_bky",  "back_bkx",           
            "r_arm_shz", "r_arm_shx", "r_arm_ely",
            "r_arm_elx",
            "r_arm_wry", "r_arm_wrx" };

      Vector3d A = new Vector3d(); 
      Vector3d B = new Vector3d(); 

      for(int i=0; i< jointNames.length; i++)
      {    
         System.out.print("-----------------\n" +  jointNames[i] );
         ReferenceFrame workingFrame =  wholeBodyIKSolver.getDesiredAfterJointFrame( jointNames[i], ReferenceFrame.getWorldFrame());
         RigidBodyTransform tempTransform =  workingFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());

         tempTransform.getTranslation(A);
         System.out.format("\n ACTUAL:  %.3f   %.3f   %.3f\n" ,A.getX(), A.getY(), A.getZ() );

         ReferenceFrame actualFrame = fullRobotModel.getOneDoFJointByName(jointNames[i]).getFrameAfterJoint();
         tempTransform =  actualFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());

         tempTransform.getTranslation(B);
         System.out.format(" DESIRED: %.3f   %.3f   %.3f\n" ,B.getX(), B.getY(), B.getZ() );

         A.sub(B);
         System.out.format(" angle error is %+.1f degrees\terror in location is  %.3f   %.3f   %.3f\n", 

               (180.0/Math.PI)*(wholeBodyIKSolver.getDesiredJointAngle( jointNames[i] ) - 
                     fullRobotModel.getOneDoFJointByName(jointNames[i]).getQ() ),
                     A.getX(), A.getY(), A.getZ() );  
      }

      if (vector.length() > ERROR_DISTANCE_TOLERANCE)
      {
         System.out.println(this.getClass().getName() + ": FAILED TO REACH DESIRED POINT " );
      }
      else{
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
