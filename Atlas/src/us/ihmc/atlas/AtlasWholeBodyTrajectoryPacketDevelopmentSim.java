package us.ihmc.atlas;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.atlas.AtlasRobotModel.AtlasTarget;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.communication.packets.manipulation.ArmJointTrajectoryPacket;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryPacket;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.wholeBodyInverseKinematicsSimulationController.WholeBodyIKIngressEgressControllerSimulation;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyIKPacketCreator;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicShape;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;

public class AtlasWholeBodyTrajectoryPacketDevelopmentSim
{
   private static final double EPS = 1e-5;
   private final WholeBodyIkSolver wholeBodyIKSolver;
   private final WholeBodyIKPacketCreator wholeBodyIKPacketCreator;
   private final SDFFullRobotModel actualRobotModel;
   private final PacketCommunicator fieldObjectCommunicator;
   //   private final ArrayList<Packet> packetsToSend = new ArrayList<Packet>();
   private final ArrayList<FramePose> desiredPelvisFrameList = new ArrayList<FramePose>();
   private final WholeBodyIKIngressEgressControllerSimulation hikIngEgCtrlSim;
   private final boolean USE_INGRESS_ONLY = false;
   private final YoVariableRegistry registry;
   private final YoFramePoint framePoint;
   private final YoFrameOrientation frameOrientation;
   private final YoGraphicShape yoGraphicsShapeDesired;
   private final boolean useRandom = false;
   //   private final double ERROR_DISTANCE_TOLERANCE = 0.005;
   private final SDFFullRobotModel desiredFullRobotModel;
   //   private YoGraphicShape yoGraphicsShapeActual;
   //   private YoFramePoint framePoint2;
   //   private YoFrameOrientation frameOrientation2;
   //   private final double trajectoryTime = 2.0;
   //   private ComputeResult success;

   private AtlasRobotModel robotModel;
   private int numberOfArmJoints = 6;

   public AtlasWholeBodyTrajectoryPacketDevelopmentSim() throws Exception
   {
      this.robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, AtlasTarget.SIM, false);
      this.desiredFullRobotModel = robotModel.createFullRobotModel();
      this.hikIngEgCtrlSim = new WholeBodyIKIngressEgressControllerSimulation(robotModel);
      this.registry = hikIngEgCtrlSim.getSimulationConstructionSet().getRootRegistry();
      Graphics3DObject linkGraphicsDesired = new Graphics3DObject();
      //      URL fileURL = new URL(null);
      //      linkGraphicsDesired.addModelFile(fileURL);
      linkGraphicsDesired.addSphere(0.05);
      framePoint = new YoFramePoint("dontCarePoint", ReferenceFrame.getWorldFrame(), registry);
      frameOrientation = new YoFrameOrientation("orientiation", ReferenceFrame.getWorldFrame(), registry);
      yoGraphicsShapeDesired = new YoGraphicShape("desiredPelvis", linkGraphicsDesired, framePoint, frameOrientation, 1.0);
      hikIngEgCtrlSim.getSimulationConstructionSet().addYoGraphic(yoGraphicsShapeDesired);
      hikIngEgCtrlSim.getDRCSimulation().start();
      this.actualRobotModel = hikIngEgCtrlSim.getDRCSimulation().getThreadDataSynchronizer().getEstimatorFullRobotModel();
      
      this.fieldObjectCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT, new IHMCCommunicationKryoNetClassList());
      this.fieldObjectCommunicator.connect();
      
      this.wholeBodyIKSolver = robotModel.createWholeBodyIkSolver();
      wholeBodyIKSolver.getConfiguration().setNumberOfControlledDoF(RobotSide.RIGHT, WholeBodyIkSolver.ControlledDoF.DOF_3P);
      wholeBodyIKSolver.getConfiguration().setNumberOfControlledDoF(RobotSide.LEFT, WholeBodyIkSolver.ControlledDoF.DOF_NONE);
      wholeBodyIKSolver.getHierarchicalSolver().setVerbosityLevel(0);
      this.wholeBodyIKPacketCreator = new WholeBodyIKPacketCreator(robotModel);
      createPelvisDesiredFramesList();
      System.out.println(getClass().getSimpleName() + ": Starting sleep for 10 secs.");
      ThreadTools.sleep(10000);
      System.out.println(getClass().getSimpleName() + ": Attempting to start test.");

      boolean testNotStarted = true;
      while (testNotStarted)
      {
         if (USE_INGRESS_ONLY)
         {
            if (ingressEgressModeActivated())
            {
               executeTest();
               testNotStarted = false;
            }
         }
         else
         {
            executeTest();
            testNotStarted = false;
         }
      }
   }

//         private double[][] generateArmTrajectory(RobotSide side, int waypoints)
//         {
//      	   double[][] trajectoryAnglesToReturn = new double[6][waypoints];
//      	   for(int w = 0; w<waypoints; w++){
//      	      int joints = -1;
//      	      for (ArmJointName armJointName : actualRobotModel.getRobotSpecificJointNames().getArmJointNames()){
//      //		   for(ArmJointName armJointName : robotModel.getJointMap().getArmJointNames()){
//      			   OneDoFJoint armJoint = actualRobotModel.getOneDoFJointByName(robotModel.getJointMap().getArmJointName(side, armJointName));
//      			   double q = armJoint.getJointLimitLower() +  w*(armJoint.getJointLimitUpper() - armJoint.getJointLimitLower()) / waypoints;
//      			   trajectoryAnglesToReturn[++joints][w] = q;
//      		   }
//      	   }
//      	   return trajectoryAnglesToReturn;
//         }

   private ArmJointTrajectoryPacket generateArmTrajectory(RobotSide robotSide, int numOfWaypoints)
   {
      // Put the arm down
      //waypoints isnt used in this function
      double halfPi = Math.PI / 2.0;
      double[] armDown2 = ensureJointAnglesSize(new double[] { 0.0, -0.4, halfPi / 2.0, 0.0 });
      double[] armIntermediateOnWayUp = ensureJointAnglesSize(new double[] { 0.0, -halfPi, halfPi / 2.0, -halfPi / 2.0 });
      double[] armUp1 = ensureJointAnglesSize(new double[] { 0.0, -1.5 * halfPi, halfPi / 2.0, 0.0 });
      double[] armUp2 = ensureJointAnglesSize(new double[] { 0.0, -1.5 * halfPi, -halfPi / 2.0, 0.0 });
      double[] armIntermediateOnWayDown = ensureJointAnglesSize(new double[] { 0.0, -halfPi, -halfPi / 2.0, -halfPi / 2.0 });
      double[] armDown1 = ensureJointAnglesSize(new double[] { 0.0, -0.4, -halfPi / 2.0, 0.0 });

      ArmJointTrajectoryPacket armFlyingSequence = new ArmJointTrajectoryPacket(robotSide, numOfWaypoints, numberOfArmJoints);

      for (int jointIndex = 0; jointIndex < numberOfArmJoints; jointIndex++)
      {
         for (int poseIndex = 0; poseIndex < numOfWaypoints; poseIndex++)
         {
            double desiredJointAngle;
            switch (poseIndex % 6)
            {
            case 0:
               desiredJointAngle = armDown1[jointIndex];
               break;
            case 1:
               desiredJointAngle = armDown2[jointIndex];
               break;
            case 2:
               desiredJointAngle = armIntermediateOnWayUp[jointIndex];
               break;
            case 3:
               desiredJointAngle = armUp1[jointIndex];
               break;
            case 4:
               desiredJointAngle = armUp2[jointIndex];
               break;
            case 5:
               desiredJointAngle = armIntermediateOnWayDown[jointIndex];
               break;
            default:
               throw new RuntimeException("Should not get there!");
            }

            armFlyingSequence.trajectoryPoints[poseIndex].positions[jointIndex] = desiredJointAngle;
         }
      }
      return armFlyingSequence;
   }

   private double[] ensureJointAnglesSize(double[] desiredJointAngles)
   {
      double[] ret;
      double[] src = desiredJointAngles;
      if (numberOfArmJoints > src.length)
      {
         ret = new double[numberOfArmJoints];
         System.arraycopy(src, 0, ret, 0, src.length);
      }
      else
         ret = src;
      return ret;
   }

   private double[][] transposeDoubleArrayMatrix(double[][] doubleArrayToTranspose)
   {
      double[][] doubleArrayToReturn = new double[doubleArrayToTranspose[0].length][doubleArrayToTranspose.length];
      for (int i = 0; i < doubleArrayToTranspose.length; i++)
      {
         for (int j = 0; j < doubleArrayToTranspose[0].length; j++)
         {
            doubleArrayToReturn[j][i] = doubleArrayToTranspose[i][j];
         }
      }
      return doubleArrayToReturn;
   }

   private void executeTest()
   {
      int waypoints = 8;
      WholeBodyTrajectoryPacket packet = new WholeBodyTrajectoryPacket(waypoints, 6);
      try
      {
         
         for (int w=0; w<waypoints; w++ )
         {
            packet.timeAtWaypoint[w] = 2.0;
            if( w > 0 ) 
               packet.timeAtWaypoint[w] += packet.timeAtWaypoint[w-1];
 
            packet.pelvisWorldPosition[w] = new Point3d( -0.05, 0.0, 0.2*( w%2 )+0.6);      
            
            Quat4d rotation  =  new Quat4d();    
            rotation.set( new AxisAngle4d(0, 1, 0, 0.2* ( (w%2)*2 - 1) ));
            
            packet.chestWorldOrientation[w] = rotation;    
            packet.pelvisWorldOrientation[w] = new Quat4d(); 
         }
         
         packet.allocateArmTrajectories();
         packet.leftArmTrajectory = generateArmTrajectory(RobotSide.LEFT, waypoints);
         packet.rightArmTrajectory = generateArmTrajectory(RobotSide.RIGHT, waypoints);
         
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
      System.out.println(getClass().getSimpleName() + ": Sending trajectory packet");
      fieldObjectCommunicator.send(packet);
   }

   //   private void executeTest()
   //   {
   //      for (int i = 0; i < desiredPelvisFrameList.size(); i++)
   //      {
   //         ThreadTools.sleep(1000);
   //         doControl(i);
   //         ThreadTools.sleep((long) (3 * trajectoryTime * 1000.0));
   //         checkIfTargetWasReached(i);
   //      }
   //   }

   public static void main(String[] args) throws Exception
   {
      new AtlasWholeBodyTrajectoryPacketDevelopmentSim();
   }

   private void createPelvisDesiredFramesList()
   {
      Random rand = new Random();
      for (int i = 0; i < 4; i++)
      {
         Point3d point = new Point3d(0.0, 0.0, rand.nextDouble() + 0.4);
         FramePose desiredPose = new FramePose(ReferenceFrame.getWorldFrame(), point, new Quat4d());
         desiredPelvisFrameList.add(desiredPose);
      }
   }

   private FramePose getNextDesiredPelvisFrame(int index)
   {
      FramePose desiredPose;
      if (useRandom)
      {
         Random random = new Random();
         Point3d randomPoint = RandomTools.generateRandomPoint(random, -0.2, -0.2, 0.2, 0.2, 1.0, 1.5);
         desiredPose = new FramePose(ReferenceFrame.getWorldFrame(), randomPoint, new Quat4d());
      }
      else
      {
         desiredPose = desiredPelvisFrameList.get(index);
      }
      return desiredPose;
   }

   private void doControl(int index)
   {
      FramePose desiredPose = getNextDesiredPelvisFrame(index);
      WholeBodyTrajectoryPacket packet = new WholeBodyTrajectoryPacket();
      yoGraphicsShapeDesired.setPosition(desiredPose.getFramePointCopy());
      yoGraphicsShapeDesired.setOrientation(desiredPose.getFrameOrientationCopy());
      System.out.println(getClass().getSimpleName() + ": Sending WholeBodyTrajectoryPacket.");
      fieldObjectCommunicator.send(packet);
      //      wholeBodyIKSolver.setGripperPalmTarget(actualRobotModel, RobotSide.RIGHT, desiredPose);
      //      try
      //      {
      //         success = wholeBodyIKSolver.compute(actualRobotModel, desiredFullRobotModel, ComputeOption.USE_ACTUAL_MODEL_JOINTS);
      //      }
      //      catch (Exception e)
      //      {
      //         e.printStackTrace();
      //      }
      //      wholeBodyIKPacketCreator.createPackets(desiredFullRobotModel, trajectoryTime, packetsToSend);
      //      System.out.println("AtlasWholeBodyIKIngressEgressCtrlSim: Sending packets");
      //      for (int i = 0; i < packetsToSend.size(); i++)
      //      {
      //         fieldObjectCommunicator.send(packetsToSend.get(i));
      //      }
      //      packetsToSend.clear();
      //      ReferenceFrame desiredWristReference = wholeBodyIKSolver.getDesiredBodyFrame("r_hand", ReferenceFrame.getWorldFrame());
      //      yoGraphicsShapeDesired.setToReferenceFrame(desiredWristReference);
   }

   private void checkIfTargetWasReached(int index)
   {
      FramePose desiredPose = getNextDesiredPelvisFrame(index);
      RigidBodyTransform desiredRBT = new RigidBodyTransform();
      desiredPose.getPose(desiredRBT);
      RigidBodyTransform actualRBT = actualRobotModel.getPelvis().getBodyFixedFrame().getTransformToWorldFrame();
      @SuppressWarnings("static-access")
      Vector3d errorVector = actualRBT.getTranslationDifference(desiredRBT, actualRBT);
      if (errorVector.length() > EPS)
      {
         System.out.println(getClass().getName() + ": (" + index + ") FAIL - commanded position not reached.");
      }
      else
      {
         System.out.println(getClass().getName() + ":(" + index + ") SUCCESS - position reached.");
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
