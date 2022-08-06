package us.ihmc.commonWalkingControlModules.configurations;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Arrays;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import com.esotericsoftware.kryo.util.IntMap;

import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.JointMatrixIndexProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.linearAlgebra.NullspaceCalculator;
import us.ihmc.robotics.linearAlgebra.SVDNullspaceCalculator;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;

//Typical usage steps (work-in-progress):
//
//(0) Search below for all "URDF"/"urdf" occurrences to see which ROBOT's NP function is in use here. 
//Initialization:
//(1) Instantiate a 'new NadiaNaturalPosture(...)' for that ROBOT.
//(2) Call 'computeNaturalPosture(...)' with a nominal whole-ROBOT world pose (i.e. base included).
//  The pose should basically be within the joint limits as specified in the URDF.
//(3) Call (typically) 'getNaturalPostureQuaternionrtBase()' and see what quaternion is returned for 
//  NP relative to the Base (pelvis).
//(4) If desired, the NP quaternion can be arbitrarily offset (e.g. set to identity) using,
//  for instance, the initial NP quaternion returned in step (3).
//  Ex: call 'setNaturalPostureOffset(...)' with the .conjugate() of the NP quaternion from step (3).
//Every control spoke:    
//(5) Call 'computeNaturalPosture(...)' with a whole-ROBOT world pose (i.e. base included).
//  The pose should basically be within the joint limits as specified in the URDF.
//(6) Use the main getters:
//  'getNaturalPostureQuaternion()'
//  'getNaturalPostureJacobian()'
//(6a) If you are doing something special, you can get ROBOT NP relative to the pelvis also:
//  'getNaturalPostureQuaternionrtBase()'
//(7) Do control using NP as a whole-body representation of angular CoM.

public class NadiaNaturalPosture implements HumanoidRobotNaturalPosture
{
   private final int NumDoFs = 29; // excluding floating base joint

   private static String folderPath = "C:\\Users\\yumin\\OneDrive\\Documents\\workspace\\exported_data\\";

   // joint indices (the order of joint list from MultiBodySystemTools.collectSubtreeJoints(fullRobotModel.getPelvis()))
   private int i0; // LEFT_HIP_Z
   private int i1; // RIGHT_HIP_Z
   private int i2; // SPINE_Z
   private int i3; // LEFT_HIP_X
   private int i4; // RIGHT_HIP_X
   private int i5; // SPINE_X
   private int i6; // LEFT_HIP_Y
   private int i7; // RIGHT_HIP_Y
   private int i8; // SPINE_Y
   private int i9; // LEFT_KNEE_Y
   private int i10; // RIGHT_KNEE_Y
   private int i11; // LEFT_SHOULDER_Y
   private int i12; // RIGHT_SHOULDER_Y
   private int i13; // LEFT_ANKLE_Y
   private int i14; // RIGHT_ANKLE_Y
   private int i15; // LEFT_SHOULDER_X
   private int i16; // RIGHT_SHOULDER_X
   private int i17; // LEFT_ANKLE_X
   private int i18; // RIGHT_ANKLE_X
   private int i19; // LEFT_SHOULDER_Z
   private int i20; // RIGHT_SHOULDER_Z
   private int i21; // LEFT_ELBOW_Y
   private int i22; // RIGHT_ELBOW_Y
   private int i23; // LEFT_WRIST_Z
   private int i24; // RIGHT_WRIST_Z
   private int i25; // LEFT_WRIST_X
   private int i26; // RIGHT_WRIST_X
   private int i27; // LEFT_WRIST_Y
   private int i28; // RIGHT_WRIST_Y

   private final Quaternion npQoffset = new Quaternion(0, 0, 0, 1);

   // Various Natural Posture results available via getters:
   private final Quaternion Q_Base_NP = new Quaternion(0, 0, 0, 1); // Natural Posture rt the pelvis
   private final Quaternion Q_World_NP = new Quaternion(0, 0, 0, 1); // Natural Posture rt the world
   private final DMatrixRMaj jacobianNP = new DMatrixRMaj(3, 6 + NumDoFs);

   // For internal use:
   private final DMatrixRMaj jacobianQuaternionNPrtBase = new DMatrixRMaj(4, NumDoFs);
   private final DMatrixRMaj jacobianOmegaNPrtBase = new DMatrixRMaj(4, NumDoFs); // GMN: Would need 2.0* if used externally.

   private final YoFrameQuaternion yoQuaternionNPrtWorld; //, yoQuaternionNPrtBase;
   private final YoFrameQuaternion yoQuaternionIdent;
   private final YoFramePoint3D originNPpelvis;
   private final YoFramePoint3D originPelvis;
   //   private final YoFramePoint3D originWorld;
   private final YoGraphicCoordinateSystem naturalPostureVizPelvis;
   private final YoGraphicCoordinateSystem vizPelvis;
   //   private final YoGraphicCoordinateSystem naturalPostureVizWorld;
   private boolean doGraphics = false;

   private final JointMatrixIndexProvider jointMatrixIndexProvider;
   private final IntMap<OneDoFJointReadOnly> jointMap = new IntMap<>();

   // A nominal standing pose; stored in this class for convenience; upon calling 'initialize()' will create a nominal Qoffset, which has a getter:
   // TODO pull from the "initialConfiguration" class
   // Gabe said this should include all actuated joints
   double[] qNomStandingURDF = new double[] {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
   double[] qNomStanding = new double[NumDoFs];
   private final Quaternion QbaseNomStanding = new Quaternion(0, 0, 0, 1);
   private final Quaternion npNomQoffset = new Quaternion(0, 0, 0, 1);

   private FullHumanoidRobotModel fullRobotModel = null;

   private int[] jointIndexArray = null;
   private final double[] jointPositionArray = new double[NumDoFs];

   public NadiaNaturalPosture(FullHumanoidRobotModel robotModel,
                              boolean useURDFJointNumbering,
                              YoRegistry registry,
                              YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.fullRobotModel = robotModel;

      yoQuaternionNPrtWorld = new YoFrameQuaternion("NadiaNaturalPostureInWorld", ReferenceFrame.getWorldFrame(), registry);
      yoQuaternionIdent = new YoFrameQuaternion("NadiaPelvisFrameViz", ReferenceFrame.getWorldFrame(), registry);
      //      yoQuaternionNPrtBase = new YoFrameQuaternion("naturalPostureInBase", robotModel.getRootBody().getBodyFixedFrame(), registry);

      if (yoGraphicsListRegistry != null)
      {
         doGraphics = true;

         originNPpelvis = new YoFramePoint3D("NadiaOriginNPpelvis", ReferenceFrame.getWorldFrame(), registry);
         naturalPostureVizPelvis = new YoGraphicCoordinateSystem("NadiaNaturalPostureP", originNPpelvis, yoQuaternionNPrtWorld, 0.5);
         yoGraphicsListRegistry.registerYoGraphic("NadiaNaturalPostureP", naturalPostureVizPelvis);

         originPelvis = new YoFramePoint3D("NadiaOriginPelvis", ReferenceFrame.getWorldFrame(), registry);
         vizPelvis = new YoGraphicCoordinateSystem("NadiaPelvisframe", originPelvis, yoQuaternionIdent, 0.4);
         yoGraphicsListRegistry.registerYoGraphic("NadiaPelvisframe", vizPelvis);
      }
      else
      {
         originNPpelvis = null;
         originPelvis = null;
         //         originWorld = null;
         naturalPostureVizPelvis = null;
         vizPelvis = null;
         //         naturalPostureVizWorld = null;
      }

      if (fullRobotModel != null)
      {
         jointMatrixIndexProvider = JointMatrixIndexProvider.toIndexProvider(HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel));
      }
      else
      {
         jointMatrixIndexProvider = null;
      }

      if (useURDFJointNumbering) // Mostly for testing
      {
         // Note: Currently, some joints are not used (allowed to vary) in the NP estimation - e.g. hands, ankles, neck
         i0 = 0;
         i1 = 1;
         i2 = 2;
         i3 = 3;
         i4 = 4;
         i5 = 5;
         i6 = 6;
         i7 = 7;
         i8 = 8;
         i9 = 9;
         i10 = 10;
         i11 = 11;
         i12 = 12;
         i13 = 13;
         i14 = 14;
         i15 = 15;
         i16 = 16;
         i17 = 17;
         i18 = 18;
         i19 = 19;
         i20 = 20;
         i21 = 21;
         i22 = 22;
         i23 = 23;
         i24 = 24;
         i25 = 25;
         i26 = 26;
         i27 = 27;
         i28 = 28;

         qNomStanding = qNomStandingURDF;
      }
      else
      {

         int iBase = 6; // GMN: offset for pelvis DoF

         // Look up and set joint indices:
         i0 = getJointIndices("LEFT_HIP_Z")[0] - iBase;
         i1 = getJointIndices("RIGHT_HIP_Z")[0] - iBase;
         i2 = getJointIndices("SPINE_Z")[0] - iBase;
         i3 = getJointIndices("LEFT_HIP_X")[0] - iBase;
         i4 = getJointIndices("RIGHT_HIP_X")[0] - iBase;
         i5 = getJointIndices("SPINE_X")[0] - iBase;
         i6 = getJointIndices("LEFT_HIP_Y")[0] - iBase;
         i7 = getJointIndices("RIGHT_HIP_Y")[0] - iBase;
         i8 = getJointIndices("SPINE_Y")[0] - iBase;
         i9 = getJointIndices("LEFT_KNEE_Y")[0] - iBase;
         i10 = getJointIndices("RIGHT_KNEE_Y")[0] - iBase;
         i11 = getJointIndices("LEFT_SHOULDER_Y")[0] - iBase;
         i12 = getJointIndices("RIGHT_SHOULDER_Y")[0] - iBase;
         i13 = getJointIndices("LEFT_ANKLE_Y")[0] - iBase;
         i14 = getJointIndices("RIGHT_ANKLE_Y")[0] - iBase;
         i15 = getJointIndices("LEFT_SHOULDER_X")[0] - iBase;
         i16 = getJointIndices("RIGHT_SHOULDER_X")[0] - iBase;
         i17 = getJointIndices("LEFT_ANKLE_X")[0] - iBase;
         i18 = getJointIndices("RIGHT_ANKLE_X")[0] - iBase;
         i19 = getJointIndices("LEFT_SHOULDER_Z")[0] - iBase;
         i20 = getJointIndices("RIGHT_SHOULDER_Z")[0] - iBase;
         i21 = getJointIndices("LEFT_ELBOW_Y")[0] - iBase;
         i22 = getJointIndices("RIGHT_ELBOW_Y")[0] - iBase;
         i23 = getJointIndices("LEFT_WRIST_Z")[0] - iBase;
         i24 = getJointIndices("RIGHT_WRIST_Z")[0] - iBase;
         i25 = getJointIndices("LEFT_WRIST_X")[0] - iBase;
         i26 = getJointIndices("RIGHT_WRIST_X")[0] - iBase;
         i27 = getJointIndices("LEFT_WRIST_Y")[0] - iBase;
         i28 = getJointIndices("RIGHT_WRIST_Y")[0] - iBase;

         // Test the method
         System.out.println(i0);
         System.out.println(i1);
         System.out.println(i2);
         System.out.println(i3);
         System.out.println(i4);
         System.out.println(i5);
         System.out.println(i6);
         System.out.println(i7);
         System.out.println(i8);
         System.out.println(i9);
         System.out.println(i10);
         System.out.println(i11);
         System.out.println(i12);
         System.out.println(i13);
         System.out.println(i14);
         System.out.println(i15);
         System.out.println(i16);
         System.out.println(i17);
         System.out.println(i18);
         System.out.println(i19);
         System.out.println(i20);
         System.out.println(i21);
         System.out.println(i22);
         System.out.println(i23);
         System.out.println(i24);
         System.out.println(i25);
         System.out.println(i26);
         System.out.println(i27);
         System.out.println(i28);

         jointMap.put(i0, fullRobotModel.getOneDoFJointByName("LEFT_HIP_Z"));
         jointMap.put(i1, fullRobotModel.getOneDoFJointByName("RIGHT_HIP_Z"));
         jointMap.put(i2, fullRobotModel.getOneDoFJointByName("SPINE_Z"));
         jointMap.put(i3, fullRobotModel.getOneDoFJointByName("LEFT_HIP_X"));
         jointMap.put(i4, fullRobotModel.getOneDoFJointByName("RIGHT_HIP_X"));
         jointMap.put(i5, fullRobotModel.getOneDoFJointByName("SPINE_X"));
         jointMap.put(i6, fullRobotModel.getOneDoFJointByName("LEFT_HIP_Y"));
         jointMap.put(i7, fullRobotModel.getOneDoFJointByName("RIGHT_HIP_Y"));
         jointMap.put(i8, fullRobotModel.getOneDoFJointByName("SPINE_Y"));
         jointMap.put(i9, fullRobotModel.getOneDoFJointByName("LEFT_KNEE_Y"));
         jointMap.put(i10, fullRobotModel.getOneDoFJointByName("RIGHT_KNEE_Y"));
         jointMap.put(i11, fullRobotModel.getOneDoFJointByName("LEFT_SHOULDER_Y"));
         jointMap.put(i12, fullRobotModel.getOneDoFJointByName("RIGHT_SHOULDER_Y"));
         jointMap.put(i13, fullRobotModel.getOneDoFJointByName("LEFT_ANKLE_Y"));
         jointMap.put(i14, fullRobotModel.getOneDoFJointByName("RIGHT_ANKLE_Y"));
         jointMap.put(i15, fullRobotModel.getOneDoFJointByName("LEFT_SHOULDER_X"));
         jointMap.put(i16, fullRobotModel.getOneDoFJointByName("RIGHT_SHOULDER_X"));
         jointMap.put(i17, fullRobotModel.getOneDoFJointByName("LEFT_ANKLE_X"));
         jointMap.put(i18, fullRobotModel.getOneDoFJointByName("RIGHT_ANKLE_X"));
         jointMap.put(i19, fullRobotModel.getOneDoFJointByName("LEFT_SHOULDER_Z"));
         jointMap.put(i20, fullRobotModel.getOneDoFJointByName("RIGHT_SHOULDER_Z"));
         jointMap.put(i21, fullRobotModel.getOneDoFJointByName("LEFT_ELBOW_Y"));
         jointMap.put(i22, fullRobotModel.getOneDoFJointByName("RIGHT_ELBOW_Y"));
         jointMap.put(i23, fullRobotModel.getOneDoFJointByName("LEFT_WRIST_Z"));
         jointMap.put(i24, fullRobotModel.getOneDoFJointByName("RIGHT_WRIST_Z"));
         jointMap.put(i25, fullRobotModel.getOneDoFJointByName("LEFT_WRIST_X"));
         jointMap.put(i26, fullRobotModel.getOneDoFJointByName("RIGHT_WRIST_X"));
         jointMap.put(i27, fullRobotModel.getOneDoFJointByName("LEFT_WRIST_Y"));
         jointMap.put(i28, fullRobotModel.getOneDoFJointByName("RIGHT_WRIST_Y"));

         //      LogTools.info("-------------------------NP------------------------------------");
         //      LogTools.info("-------------------------NP------------------------------------");
         //      LogTools.info(EuclidCoreIOTools.getCollectionString(", ", indexProvider.getIndexedJointsInOrder(), j -> j.getName()));
         //      LogTools.info("-------------------------NP------------------------------------");
         //      LogTools.info("-------------------------NP------------------------------------");
         //      LogTools.info("-------------------------NP------------------------------------");

         // reorder qNomStanding based on joint ordering above:
         qNomStanding[i0] = qNomStandingURDF[0];
         qNomStanding[i1] = qNomStandingURDF[1];
         qNomStanding[i2] = qNomStandingURDF[2];
         qNomStanding[i3] = qNomStandingURDF[3];
         qNomStanding[i4] = qNomStandingURDF[4];
         qNomStanding[i5] = qNomStandingURDF[5];
         qNomStanding[i6] = qNomStandingURDF[6];
         qNomStanding[i7] = qNomStandingURDF[7];
         qNomStanding[i8] = qNomStandingURDF[8];
         qNomStanding[i9] = qNomStandingURDF[9];
         qNomStanding[i10] = qNomStandingURDF[10];
         qNomStanding[i11] = qNomStandingURDF[11];
         qNomStanding[i12] = qNomStandingURDF[12];
         qNomStanding[i13] = qNomStandingURDF[13];
         qNomStanding[i14] = qNomStandingURDF[14];
         qNomStanding[i15] = qNomStandingURDF[15];
         qNomStanding[i16] = qNomStandingURDF[16];
         qNomStanding[i17] = qNomStandingURDF[17];
         qNomStanding[i18] = qNomStandingURDF[18];
         qNomStanding[i19] = qNomStandingURDF[19];
         qNomStanding[i20] = qNomStandingURDF[20];
         qNomStanding[i21] = qNomStandingURDF[21];
         qNomStanding[i22] = qNomStandingURDF[22];
         qNomStanding[i23] = qNomStandingURDF[23];
         qNomStanding[i24] = qNomStandingURDF[24];
         qNomStanding[i25] = qNomStandingURDF[25];
         qNomStanding[i26] = qNomStandingURDF[26];
         qNomStanding[i27] = qNomStandingURDF[27];
         qNomStanding[i28] = qNomStandingURDF[28];
      }

      Integer[] jointsToFit = read1DIntCsvToIntArray(folderPath + "joints_to_fit.csv");
      jointIndexArray = new int[jointsToFit.length];
      for (int i = 0; i < jointsToFit.length; i++)
      {
         jointIndexArray[i] = jointsToFit[i];
      }
   }

   int[] getJointIndices(String jointName)
   {
      return getJointIndices(fullRobotModel.getOneDoFJointByName(jointName));
   }

   int[] getJointIndices(JointReadOnly joint)
   {
      return jointMatrixIndexProvider.getJointDoFIndices(joint);
   }

   public double[] getJointPositionArray()
   {
      Arrays.fill(jointPositionArray, 0.0);

      for (int jointIndex : jointIndexArray)
      {
         jointPositionArray[jointIndex] = jointMap.get(jointIndex).getQ();
      }

      return jointPositionArray;
   }

   @Override
   public void initialize()
   {
      computeNaturalPosture(this.qNomStanding, this.QbaseNomStanding);
      this.npNomQoffset.set(getNaturalPostureQuaternionrtBase());
      this.npNomQoffset.conjugate();
      this.npQoffset.set(this.npNomQoffset);
   }

   @Override
   public Quaternion getNominalStandingPoseQoffset()
   {
      return this.npNomQoffset;
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return this.fullRobotModel;
   }

   // Set a user-chosen orientation offset to apply to NP:
   @Override
   public void setNaturalPostureOffset(QuaternionReadOnly Qoffset)
   {
      // System.out.println(Qoffset);
      this.npQoffset.set(Qoffset);
   }

   @Override
   public Quaternion getNaturalPostureQuaternion()
   {
      return this.Q_World_NP;
   }

   @Override
   public Quaternion getNaturalPostureQuaternionrtBase()
   {
      return this.Q_Base_NP;
   }

   @Override
   public DMatrixRMaj getNaturalPostureJacobian()
   {
      return this.jacobianNP;
   }

   @Override
   public void compute(double[] q, Orientation3DReadOnly Q_world_base)
   {
      computeNaturalPosture(q, Q_world_base);
   }
   
   public void computeNaturalPosture(double[] q, Orientation3DReadOnly Q_world_base)
   {
      // Get the NP quaternion r.t. the base(pelvis) frame:
      computeQuaternionNPrtBase(q, this.Q_Base_NP);

      // Express the NP quaternion in the world-frame:
      this.Q_World_NP.set(Q_world_base);
      this.Q_World_NP.multiply(this.Q_Base_NP);

      // Get the NP quaternion jacobian r.t. the base(pelvis) frame:
      // + We need 'q' because, like quaternionNPrtBase, this jacobian is also an explicit function of q.
      // + We need 'quaternionNPrtBase` to deduce the 1st row of this jacobian from the unit-sphere constraint. 
      computeJacobianQuaternionNPrtBase(q, this.Q_Base_NP, this.jacobianQuaternionNPrtBase);
      // Convert the NP jacobian to map to NP omega r.t. world ewrt NP-frame:
      // + We need 'quaternionNPrtBase' to get two transforms: E (bring Qdot -> omega) & C_NP_Base (bring omega_ewrt_Base -> omega_ewrt_NP)
      // + We need 'jacobiandQuaternionNPrtBase' as once transformed, it forms all the joint-based portion of the jacobianNP.
      computeJacobianNP(this.Q_Base_NP, this.jacobianQuaternionNPrtBase, this.jacobianNP);
      
//      SVDNullspaceCalculator nullspacecalculator = new SVDNullspaceCalculator(this.jacobianNP.numCols, true /*makeLargestComponentPositive*/);
//      DMatrixRMaj nullspaceProjectorToPack = new DMatrixRMaj(this.jacobianNP.numCols, this.jacobianNP.numCols);
//      nullspacecalculator.computeNullspaceProjector(this.jacobianNP, nullspaceProjectorToPack);
//      System.out.println(nullspaceProjectorToPack);
      
      if (doGraphics == true)
      {
         FramePoint3D originPose = new FramePoint3D(this.fullRobotModel.getRootBody().getBodyFixedFrame());
         originPose.changeFrame(ReferenceFrame.getWorldFrame());
         yoQuaternionNPrtWorld.set(Q_World_NP);
         originNPpelvis.set(originPose);
         yoQuaternionIdent.set(Q_world_base);
         originPelvis.set(originPose);
      }
   }

   // Given full set of joint angles, returns NP quaternion rt the Base(i.e. pelvis)-frame (with user defined NP offset):
   private void computeQuaternionNPrtBase(double[] q, Quaternion quaternionToPack)
   {
      double[] Q = new double[4];
      Q[1] = getQx(q);
      Q[2] = getQy(q);
      Q[3] = getQz(q);
      Q[0] = Math.sqrt(1 - (Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3])); // Should return NaN if argument is negative

      quaternionToPack.set(Q[1], Q[2], Q[3], Q[0]); // IHMC "stores" quaternions in the order (Qx,Qy,Qz,Qs)
      quaternionToPack.multiply(this.npQoffset);
   }

   // NOTE ***: For internal use only.
   // Given full set of joint angles and NP quaternion rt Base, returns this [Jacobian] -> Qdot_NP_rt_Base = [Jacobian] * q_dot: 
   private void computeJacobianQuaternionNPrtBase(double[] q, QuaternionReadOnly quaternionNPrtBase, DMatrixRMaj jacobianToPack)
   {
      double[][] JQ = new double[4][NumDoFs]; // GMN: These do initialize to all zero, right??  That's what I read online...
      getJQx(q, JQ); // replaces row index 1 of JQ
      getJQy(q, JQ); // replaces row index 2 of JQ
      getJQz(q, JQ); // replaces row index 3 of JQ

      // GMN: I'll have to do further investigation to see if we have methods that can work on submatrices...
      //      For now, I will do this manually:
      for (int i = 0; i < NumDoFs; i++)
      {
         JQ[0][i] = quaternionNPrtBase.getX() * JQ[1][i] + quaternionNPrtBase.getY() * JQ[2][i] + quaternionNPrtBase.getZ() * JQ[3][i];
         JQ[0][i] /= (-quaternionNPrtBase.getS());
      }

      jacobianToPack.set(JQ);
   }

   // NOTE ***: For internal use only.
   // Given NP quaternion rt Base & Jacobian of the same, returns this [Jacobian] -> 
   //         omega_NP_rt_world_ewrt_NP = [Jacobian] * [omega_Base_rt_world_ewrt_Base; q_dot]
   private void computeJacobianNP(QuaternionReadOnly quaternionNPrtBase, DMatrixRMaj jacobianQuaternionNPrtBase, DMatrixRMaj jacobianToPack)
   {
      // Need 3x3 tranform from Base-frame back to NP-frame:
      RotationMatrix CnpBase = new RotationMatrix(quaternionNPrtBase);
      CnpBase.transpose();

      // Insert transformation to bring omega_Base_rt_world_ewrt_Base back to NP-frame:
      for (int i = 0; i < 3; i++)
         for (int j = 0; j < 3; j++)
         {
            jacobianToPack.set(i, j, CnpBase.getElement(i, j)); // GMN: Need to see if we have submatrix operators...
            jacobianToPack.set(i, j + 3, 0.0);
         }

      // Need transformation of Qdot -> omega  (note the required '2.0*' comes in below):
      DMatrixRMaj E = EuclidCoreMissingTools.quaternionDotToOmegaTransform(quaternionNPrtBase);

      // Transform the quaternion jacobian to omega jacobian, and insert it: 
      CommonOps_DDRM.mult(E, jacobianQuaternionNPrtBase, this.jacobianOmegaNPrtBase);
      for (int i = 0; i < 3; i++)
         for (int j = 0; j < NumDoFs; j++)
            jacobianToPack.set(i, j + 6, 2.0 * this.jacobianOmegaNPrtBase.get(i + 1, j)); // for omega NP rt & ewrt NP-frame
      
//      // [Testing] not use the x y of NP.
//      for (int i = 0; i < 2; i++)
//         for (int j = 0; j < 3 + NumDoFs; j++)
//         {
//            jacobianToPack.set(i, j, 0.0);
//         }
   }

   //==== CSV utils ===========================================================================================================================

   private static int getNumRowsOfCsvFile(String filePath)
   {
      // Ref: https://stackoverflow.com/questions/18009416/how-to-count-total-rows-in-csv-using-java
      int count = 0;
      try
      {
         BufferedReader bufferedReader;
         bufferedReader = new BufferedReader(new FileReader(filePath));
         String input;
         try
         {
            while ((input = bufferedReader.readLine()) != null)
            {
               count++;
            }
         }
         catch (IOException e)
         {
            // TODO Auto-generated catch block
            e.printStackTrace();
         }

         System.out.println("Number of lines: " + count);

      }
      catch (FileNotFoundException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      return count;
   }

   private static int getNumColsOfCsvFile(String filePath)
   {
      String[] split = null;
      try (BufferedReader br = new BufferedReader(new FileReader(filePath)))
      {
         String line;
         while ((line = br.readLine()) != null)
         {
            split = line.split(",");
            //use the data here
         }
      }
      catch (FileNotFoundException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      catch (IOException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }

      System.out.println("Number of columns: " + split.length);

      //      for (int i = 0; i < split.length ; i++) {
      //         System.out.println(split[i]);
      //      }

      return split.length;
   }

   private static Integer[] read1DIntCsvToIntArray(String filePath)
   {
      int numRows = getNumRowsOfCsvFile(filePath);
      int numCols = getNumColsOfCsvFile(filePath);

      //      String delimiter = (numRows == 1) ? "," : "\n";
      return read1DIntCsvToIntArray(filePath, numRows * numCols, numRows == 1);
   }

   private static Integer[] read1DIntCsvToIntArray(String filePath, int numElement, Boolean isRowVector)
   {

      Integer[] intArray = new Integer[numElement];
      int i = 0;

      String[] split = null;
      try (BufferedReader br = new BufferedReader(new FileReader(filePath)))
      {
         String line;
         while ((line = br.readLine()) != null)
         {
            if (isRowVector)
            {
               split = line.split(",");

               if (numElement != split.length)
               {
                  System.out.println("there is a bug somewhere! The vector length is not consisent");
               }

               for (int j = 0; j < numElement; j++)
               {
                  intArray[j] = Integer.parseInt(split[j]);
               }
            }
            else
            {
               if (numElement == i)
               {
                  System.out.println("there is a bug somewhere! The vector length is not consisent");
               }
               intArray[i] = Integer.parseInt(line);
               i++;
            }
         }
      }
      catch (FileNotFoundException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      catch (IOException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      return intArray;
   }

   //==========================================================================================================================================

   public double getQx(double[] q)
   {
      double Qx;
      Qx = (-1.011476e-02) * q[i0] + (-1.009222e-02) * q[i1] + (7.404472e-04) * q[i2] + (1.300567e-01) * q[i3] + (1.293915e-01) * q[i4] + (1.283252e-01) * q[i5]
            + (2.919479e-03) * q[i6] + (-2.870989e-03) * q[i7] + (-9.047945e-05) * q[i8] + (6.574986e-03) * q[i9] + (-6.456387e-03) * q[i10]
            + (2.399867e-03) * q[i11] + (2.716937e-03) * q[i12] + (-2.752179e-03) * q[i15] + (2.667112e-03) * q[i16] + (-1.550586e-03) * q[i19]
            + (1.492182e-03) * q[i20] + (-2.502136e-03) * q[i21] + (-2.531255e-03) * q[i22] + (-3.476910e-03) * q[i0] * q[i0] + (3.447616e-03) * q[i1] * q[i1]
            + (-2.882641e-05) * q[i2] * q[i2] + (1.057298e-02) * q[i3] * q[i3] + (-1.056696e-02) * q[i4] * q[i4] + (-3.585542e-05) * q[i5] * q[i5]
            + (1.914971e-02) * q[i6] * q[i6] + (-1.908874e-02) * q[i7] * q[i7] + (-9.276544e-06) * q[i8] * q[i8] + (4.524237e-03) * q[i9] * q[i9]
            + (-4.464385e-03) * q[i10] * q[i10] + (5.615755e-03) * q[i11] * q[i11] + (-5.770524e-03) * q[i12] * q[i12] + (5.437961e-03) * q[i15] * q[i15]
            + (-5.490711e-03) * q[i16] * q[i16] + (8.195139e-04) * q[i19] * q[i19] + (-8.182365e-04) * q[i20] * q[i20] + (1.955704e-03) * q[i21] * q[i21]
            + (-1.957783e-03) * q[i22] * q[i22] + (1.979567e-05) * q[i0] * q[i1] + (-3.901331e-04) * q[i0] * q[i2] + (-6.272712e-04) * q[i0] * q[i3]
            + (-2.647624e-03) * q[i0] * q[i4] + (7.964863e-04) * q[i0] * q[i5] + (9.116774e-02) * q[i0] * q[i6] + (-9.695121e-03) * q[i0] * q[i7]
            + (-2.481516e-03) * q[i0] * q[i8] + (2.963527e-02) * q[i0] * q[i9] + (-1.181848e-03) * q[i0] * q[i10] + (9.983218e-04) * q[i0] * q[i11]
            + (-1.750409e-03) * q[i0] * q[i12] + (1.361736e-03) * q[i0] * q[i15] + (1.880785e-03) * q[i0] * q[i16] + (-9.288888e-04) * q[i0] * q[i19]
            + (-1.328615e-06) * q[i0] * q[i20] + (1.501395e-03) * q[i0] * q[i21] + (7.039540e-05) * q[i0] * q[i22] + (3.615126e-04) * q[i1] * q[i2]
            + (2.650327e-03) * q[i1] * q[i3] + (5.945322e-04) * q[i1] * q[i4] + (-8.069447e-04) * q[i1] * q[i5] + (-9.712005e-03) * q[i1] * q[i6]
            + (9.064621e-02) * q[i1] * q[i7] + (-2.464220e-03) * q[i1] * q[i8] + (-1.196230e-03) * q[i1] * q[i9] + (2.931251e-02) * q[i1] * q[i10]
            + (1.726736e-03) * q[i1] * q[i11] + (-9.807593e-04) * q[i1] * q[i12] + (1.865620e-03) * q[i1] * q[i15] + (1.351955e-03) * q[i1] * q[i16]
            + (6.761685e-06) * q[i1] * q[i19] + (-9.199283e-04) * q[i1] * q[i20] + (-6.984603e-05) * q[i1] * q[i21] + (-1.504815e-03) * q[i1] * q[i22]
            + (-3.763725e-04) * q[i2] * q[i3] + (3.412098e-04) * q[i2] * q[i4] + (2.775281e-05) * q[i2] * q[i5] + (1.562451e-02) * q[i2] * q[i6]
            + (1.549539e-02) * q[i2] * q[i7] + (-6.203577e-02) * q[i2] * q[i8] + (1.601713e-03) * q[i2] * q[i9] + (1.589481e-03) * q[i2] * q[i10]
            + (5.238081e-03) * q[i2] * q[i11] + (-5.424067e-03) * q[i2] * q[i12] + (2.942736e-03) * q[i2] * q[i15] + (3.012659e-03) * q[i2] * q[i16]
            + (-1.191364e-03) * q[i2] * q[i19] + (-1.200393e-03) * q[i2] * q[i20] + (4.442213e-04) * q[i2] * q[i21] + (-4.390413e-04) * q[i2] * q[i22]
            + (1.192715e-05) * q[i3] * q[i4] + (-9.766321e-03) * q[i3] * q[i5] + (9.761725e-03) * q[i3] * q[i6] + (-4.358020e-03) * q[i3] * q[i7]
            + (1.316746e-03) * q[i3] * q[i8] + (-8.968284e-03) * q[i3] * q[i9] + (5.529124e-03) * q[i3] * q[i10] + (-1.025216e-03) * q[i3] * q[i11]
            + (1.850945e-03) * q[i3] * q[i12] + (4.404436e-03) * q[i3] * q[i15] + (4.746587e-03) * q[i3] * q[i16] + (5.955505e-05) * q[i3] * q[i19]
            + (4.284752e-04) * q[i3] * q[i20] + (-9.822362e-04) * q[i3] * q[i21] + (-7.616008e-04) * q[i3] * q[i22] + (9.779998e-03) * q[i4] * q[i5]
            + (-4.360963e-03) * q[i4] * q[i6] + (9.714756e-03) * q[i4] * q[i7] + (1.315402e-03) * q[i4] * q[i8] + (5.551080e-03) * q[i4] * q[i9]
            + (-8.885331e-03) * q[i4] * q[i10] + (-1.811008e-03) * q[i4] * q[i11] + (1.046608e-03) * q[i4] * q[i12] + (4.666836e-03) * q[i4] * q[i15]
            + (4.427214e-03) * q[i4] * q[i16] + (4.169070e-04) * q[i4] * q[i19] + (5.590000e-05) * q[i4] * q[i20] + (7.599458e-04) * q[i4] * q[i21]
            + (9.896619e-04) * q[i4] * q[i22] + (-2.953629e-04) * q[i5] * q[i6] + (-2.761619e-04) * q[i5] * q[i7] + (-1.736126e-04) * q[i5] * q[i8]
            + (2.606375e-03) * q[i5] * q[i9] + (2.578295e-03) * q[i5] * q[i10] + (6.094528e-03) * q[i5] * q[i11] + (-6.165370e-03) * q[i5] * q[i12]
            + (-1.554917e-02) * q[i5] * q[i15] + (-1.573297e-02) * q[i5] * q[i16] + (-9.544767e-04) * q[i5] * q[i19] + (-9.681243e-04) * q[i5] * q[i20]
            + (-9.370004e-04) * q[i5] * q[i21] + (9.305713e-04) * q[i5] * q[i22] + (-4.590787e-05) * q[i6] * q[i7] + (1.097971e-03) * q[i6] * q[i8]
            + (9.624614e-03) * q[i6] * q[i9] + (-2.292030e-03) * q[i6] * q[i10] + (2.714796e-03) * q[i6] * q[i11] + (4.332843e-03) * q[i6] * q[i12]
            + (-7.229995e-04) * q[i6] * q[i15] + (-1.151517e-03) * q[i6] * q[i16] + (4.443123e-04) * q[i6] * q[i19] + (-3.447312e-04) * q[i6] * q[i20]
            + (-3.226140e-04) * q[i6] * q[i21] + (-3.170217e-04) * q[i6] * q[i22] + (-1.113790e-03) * q[i7] * q[i8] + (2.292232e-03) * q[i7] * q[i9]
            + (-9.520259e-03) * q[i7] * q[i10] + (4.318452e-03) * q[i7] * q[i11] + (2.696326e-03) * q[i7] * q[i12] + (1.142265e-03) * q[i7] * q[i15]
            + (7.310233e-04) * q[i7] * q[i16] + (3.617413e-04) * q[i7] * q[i19] + (-4.178380e-04) * q[i7] * q[i20] + (-3.194301e-04) * q[i7] * q[i21]
            + (-3.214045e-04) * q[i7] * q[i22] + (1.359273e-03) * q[i8] * q[i9] + (-1.367268e-03) * q[i8] * q[i10] + (1.045162e-02) * q[i8] * q[i11]
            + (1.053923e-02) * q[i8] * q[i12] + (8.056936e-03) * q[i8] * q[i15] + (-8.161851e-03) * q[i8] * q[i16] + (2.349562e-04) * q[i8] * q[i19]
            + (-1.955376e-04) * q[i8] * q[i20] + (2.085960e-03) * q[i8] * q[i21] + (2.096886e-03) * q[i8] * q[i22] + (-3.349715e-06) * q[i9] * q[i10]
            + (1.107012e-04) * q[i9] * q[i11] + (5.910216e-04) * q[i9] * q[i12] + (-2.286462e-04) * q[i9] * q[i15] + (2.458141e-04) * q[i9] * q[i16]
            + (4.954136e-04) * q[i9] * q[i19] + (-6.306070e-05) * q[i9] * q[i20] + (6.873966e-05) * q[i9] * q[i21] + (-3.064132e-04) * q[i9] * q[i22]
            + (5.861028e-04) * q[i10] * q[i11] + (1.030351e-04) * q[i10] * q[i12] + (-2.416712e-04) * q[i10] * q[i15] + (2.268613e-04) * q[i10] * q[i16]
            + (6.531916e-05) * q[i10] * q[i19] + (-4.910622e-04) * q[i10] * q[i20] + (-2.976386e-04) * q[i10] * q[i21] + (6.265089e-05) * q[i10] * q[i22]
            + (7.525369e-06) * q[i11] * q[i12] + (2.331858e-03) * q[i11] * q[i15] + (-1.170935e-04) * q[i11] * q[i16] + (-8.968728e-04) * q[i11] * q[i19]
            + (9.723991e-05) * q[i11] * q[i20] + (9.362029e-04) * q[i11] * q[i21] + (6.837693e-05) * q[i11] * q[i22] + (-9.949890e-05) * q[i12] * q[i15]
            + (2.418913e-03) * q[i12] * q[i16] + (9.553798e-05) * q[i12] * q[i19] + (-9.368650e-04) * q[i12] * q[i20] + (-7.341442e-05) * q[i12] * q[i21]
            + (-9.423373e-04) * q[i12] * q[i22] + (-4.424392e-06) * q[i15] * q[i16] + (7.760659e-04) * q[i15] * q[i19] + (-2.762830e-04) * q[i15] * q[i20]
            + (2.007150e-03) * q[i15] * q[i21] + (1.439286e-04) * q[i15] * q[i22] + (2.703000e-04) * q[i16] * q[i19] + (-7.458541e-04) * q[i16] * q[i20]
            + (1.446155e-04) * q[i16] * q[i21] + (2.009953e-03) * q[i16] * q[i22] + (-3.662987e-06) * q[i19] * q[i20] + (-9.797321e-04) * q[i19] * q[i21]
            + (-1.793031e-04) * q[i19] * q[i22] + (-1.799038e-04) * q[i20] * q[i21] + (-9.564593e-04) * q[i20] * q[i22] + (1.749562e-07) * q[i21] * q[i22]
            + (1.866919e-03) * q[i0] * q[i0] * q[i0] + (-1.628516e-03) * q[i0] * q[i0] * q[i1] + (1.209551e-03) * q[i0] * q[i0] * q[i2]
            + (-3.015329e-02) * q[i0] * q[i0] * q[i3] + (-1.588441e-03) * q[i0] * q[i0] * q[i4] + (-2.144201e-03) * q[i0] * q[i0] * q[i5]
            + (4.955900e-04) * q[i0] * q[i0] * q[i6] + (-3.322631e-04) * q[i0] * q[i0] * q[i7] + (-6.276619e-04) * q[i0] * q[i0] * q[i8]
            + (2.210254e-03) * q[i0] * q[i0] * q[i9] + (-7.039792e-04) * q[i0] * q[i0] * q[i10] + (2.165368e-04) * q[i0] * q[i0] * q[i11]
            + (5.653810e-04) * q[i0] * q[i0] * q[i12] + (-4.210967e-04) * q[i0] * q[i0] * q[i15] + (4.122016e-04) * q[i0] * q[i0] * q[i16]
            + (3.381829e-04) * q[i0] * q[i0] * q[i19] + (-6.683916e-04) * q[i0] * q[i0] * q[i20] + (2.935931e-04) * q[i0] * q[i0] * q[i21]
            + (-1.106642e-05) * q[i0] * q[i0] * q[i22] + (-1.636837e-03) * q[i0] * q[i1] * q[i1] + (1.872652e-03) * q[i1] * q[i1] * q[i1]
            + (1.222637e-03) * q[i1] * q[i1] * q[i2] + (-1.570535e-03) * q[i1] * q[i1] * q[i3] + (-3.006810e-02) * q[i1] * q[i1] * q[i4]
            + (-2.120072e-03) * q[i1] * q[i1] * q[i5] + (3.361165e-04) * q[i1] * q[i1] * q[i6] + (-5.001626e-04) * q[i1] * q[i1] * q[i7]
            + (6.299754e-04) * q[i1] * q[i1] * q[i8] + (7.155024e-04) * q[i1] * q[i1] * q[i9] + (-2.193286e-03) * q[i1] * q[i1] * q[i10]
            + (5.702403e-04) * q[i1] * q[i1] * q[i11] + (2.120748e-04) * q[i1] * q[i1] * q[i12] + (-4.106177e-04) * q[i1] * q[i1] * q[i15]
            + (4.250499e-04) * q[i1] * q[i1] * q[i16] + (6.595327e-04) * q[i1] * q[i1] * q[i19] + (-3.295209e-04) * q[i1] * q[i1] * q[i20]
            + (-1.209851e-05) * q[i1] * q[i1] * q[i21] + (2.927667e-04) * q[i1] * q[i1] * q[i22] + (6.810815e-04) * q[i0] * q[i2] * q[i2]
            + (6.794451e-04) * q[i1] * q[i2] * q[i2] + (1.593451e-03) * q[i2] * q[i2] * q[i2] + (-8.389673e-04) * q[i2] * q[i2] * q[i3]
            + (-8.240218e-04) * q[i2] * q[i2] * q[i4] + (-3.117940e-02) * q[i2] * q[i2] * q[i5] + (-2.665032e-04) * q[i2] * q[i2] * q[i6]
            + (2.687083e-04) * q[i2] * q[i2] * q[i7] + (-1.720420e-05) * q[i2] * q[i2] * q[i8] + (-1.767681e-04) * q[i2] * q[i2] * q[i9]
            + (1.711918e-04) * q[i2] * q[i2] * q[i10] + (2.758351e-04) * q[i2] * q[i2] * q[i11] + (2.583368e-04) * q[i2] * q[i2] * q[i12]
            + (-2.525173e-03) * q[i2] * q[i2] * q[i15] + (2.560187e-03) * q[i2] * q[i2] * q[i16] + (-1.678943e-04) * q[i2] * q[i2] * q[i19]
            + (1.765466e-04) * q[i2] * q[i2] * q[i20] + (4.466233e-04) * q[i2] * q[i2] * q[i21] + (4.547228e-04) * q[i2] * q[i2] * q[i22]
            + (-2.313770e-03) * q[i0] * q[i3] * q[i3] + (-6.507853e-04) * q[i1] * q[i3] * q[i3] + (-1.292170e-03) * q[i2] * q[i3] * q[i3]
            + (-2.109365e-03) * q[i3] * q[i3] * q[i3] + (-1.137334e-03) * q[i3] * q[i3] * q[i4] + (3.668307e-03) * q[i3] * q[i3] * q[i5]
            + (6.596583e-04) * q[i3] * q[i3] * q[i6] + (-8.461025e-04) * q[i3] * q[i3] * q[i7] + (6.903501e-05) * q[i3] * q[i3] * q[i8]
            + (-3.207326e-04) * q[i3] * q[i3] * q[i9] + (-1.570220e-05) * q[i3] * q[i3] * q[i10] + (-2.580375e-04) * q[i3] * q[i3] * q[i11]
            + (-1.087573e-03) * q[i3] * q[i3] * q[i12] + (6.435341e-04) * q[i3] * q[i3] * q[i15] + (5.647602e-04) * q[i3] * q[i3] * q[i16]
            + (-1.532691e-04) * q[i3] * q[i3] * q[i19] + (-1.637586e-04) * q[i3] * q[i3] * q[i20] + (8.051092e-05) * q[i3] * q[i3] * q[i21]
            + (3.262428e-05) * q[i3] * q[i3] * q[i22] + (-6.343227e-04) * q[i0] * q[i4] * q[i4] + (-2.322417e-03) * q[i1] * q[i4] * q[i4]
            + (-1.300065e-03) * q[i2] * q[i4] * q[i4] + (-1.137239e-03) * q[i3] * q[i4] * q[i4] + (-2.122879e-03) * q[i4] * q[i4] * q[i4]
            + (3.662696e-03) * q[i4] * q[i4] * q[i5] + (8.388854e-04) * q[i4] * q[i4] * q[i6] + (-6.562441e-04) * q[i4] * q[i4] * q[i7]
            + (-7.354570e-05) * q[i4] * q[i4] * q[i8] + (8.988663e-06) * q[i4] * q[i4] * q[i9] + (3.175588e-04) * q[i4] * q[i4] * q[i10]
            + (-1.083834e-03) * q[i4] * q[i4] * q[i11] + (-2.591561e-04) * q[i4] * q[i4] * q[i12] + (-5.605486e-04) * q[i4] * q[i4] * q[i15]
            + (-6.537892e-04) * q[i4] * q[i4] * q[i16] + (1.698040e-04) * q[i4] * q[i4] * q[i19] + (1.458927e-04) * q[i4] * q[i4] * q[i20]
            + (3.258086e-05) * q[i4] * q[i4] * q[i21] + (7.933015e-05) * q[i4] * q[i4] * q[i22] + (-8.933798e-05) * q[i0] * q[i5] * q[i5]
            + (-9.501622e-05) * q[i1] * q[i5] * q[i5] + (-2.462116e-03) * q[i2] * q[i5] * q[i5] + (1.856386e-03) * q[i3] * q[i5] * q[i5]
            + (1.872767e-03) * q[i4] * q[i5] * q[i5] + (-3.547081e-03) * q[i5] * q[i5] * q[i5] + (8.198076e-04) * q[i5] * q[i5] * q[i6]
            + (-8.280944e-04) * q[i5] * q[i5] * q[i7] + (-1.637447e-05) * q[i5] * q[i5] * q[i8] + (-1.758869e-04) * q[i5] * q[i5] * q[i9]
            + (1.866823e-04) * q[i5] * q[i5] * q[i10] + (1.824046e-05) * q[i5] * q[i5] * q[i11] + (3.115890e-06) * q[i5] * q[i5] * q[i12]
            + (6.431482e-07) * q[i5] * q[i5] * q[i15] + (1.903344e-06) * q[i5] * q[i5] * q[i16] + (7.653206e-05) * q[i5] * q[i5] * q[i19]
            + (-7.396116e-05) * q[i5] * q[i5] * q[i20] + (-1.678756e-04) * q[i5] * q[i5] * q[i21] + (-1.628696e-04) * q[i5] * q[i5] * q[i22]
            + (4.884875e-03) * q[i0] * q[i6] * q[i6] + (-1.896338e-03) * q[i1] * q[i6] * q[i6] + (9.353275e-04) * q[i2] * q[i6] * q[i6]
            + (-2.187515e-02) * q[i3] * q[i6] * q[i6] + (2.909161e-03) * q[i4] * q[i6] * q[i6] + (6.457586e-03) * q[i5] * q[i6] * q[i6]
            + (3.453611e-04) * q[i6] * q[i6] * q[i6] + (-8.446166e-04) * q[i6] * q[i6] * q[i7] + (6.126582e-05) * q[i6] * q[i6] * q[i8]
            + (-2.393264e-03) * q[i6] * q[i6] * q[i9] + (5.680422e-05) * q[i6] * q[i6] * q[i10] + (1.943749e-04) * q[i6] * q[i6] * q[i11]
            + (-1.374691e-04) * q[i6] * q[i6] * q[i12] + (-4.760048e-05) * q[i6] * q[i6] * q[i15] + (4.921217e-04) * q[i6] * q[i6] * q[i16]
            + (8.821549e-05) * q[i6] * q[i6] * q[i19] + (9.985464e-05) * q[i6] * q[i6] * q[i20] + (-3.543190e-05) * q[i6] * q[i6] * q[i21]
            + (-1.175586e-04) * q[i6] * q[i6] * q[i22] + (-1.905565e-03) * q[i0] * q[i7] * q[i7] + (4.921606e-03) * q[i1] * q[i7] * q[i7]
            + (9.488334e-04) * q[i2] * q[i7] * q[i7] + (2.902209e-03) * q[i3] * q[i7] * q[i7] + (-2.181574e-02) * q[i4] * q[i7] * q[i7]
            + (6.444140e-03) * q[i5] * q[i7] * q[i7] + (8.407773e-04) * q[i6] * q[i7] * q[i7] + (-3.490474e-04) * q[i7] * q[i7] * q[i7]
            + (-5.956300e-05) * q[i7] * q[i7] * q[i8] + (-5.738007e-05) * q[i7] * q[i7] * q[i9] + (2.367750e-03) * q[i7] * q[i7] * q[i10]
            + (-1.432442e-04) * q[i7] * q[i7] * q[i11] + (1.929896e-04) * q[i7] * q[i7] * q[i12] + (-4.862562e-04) * q[i7] * q[i7] * q[i15]
            + (3.594144e-05) * q[i7] * q[i7] * q[i16] + (-9.765210e-05) * q[i7] * q[i7] * q[i19] + (-9.126872e-05) * q[i7] * q[i7] * q[i20]
            + (-1.198847e-04) * q[i7] * q[i7] * q[i21] + (-3.545673e-05) * q[i7] * q[i7] * q[i22] + (-1.376380e-04) * q[i0] * q[i8] * q[i8]
            + (-1.363484e-04) * q[i1] * q[i8] * q[i8] + (2.814786e-03) * q[i2] * q[i8] * q[i8] + (4.246645e-03) * q[i3] * q[i8] * q[i8]
            + (4.248400e-03) * q[i4] * q[i8] * q[i8] + (-2.342988e-02) * q[i5] * q[i8] * q[i8] + (-1.323027e-03) * q[i6] * q[i8] * q[i8]
            + (1.322252e-03) * q[i7] * q[i8] * q[i8] + (8.470253e-06) * q[i8] * q[i8] * q[i8] + (2.211882e-04) * q[i8] * q[i8] * q[i9]
            + (-2.241490e-04) * q[i8] * q[i8] * q[i10] + (-9.334205e-04) * q[i8] * q[i8] * q[i11] + (-9.748763e-04) * q[i8] * q[i8] * q[i12]
            + (4.136062e-03) * q[i8] * q[i8] * q[i15] + (-4.181311e-03) * q[i8] * q[i8] * q[i16] + (3.631669e-04) * q[i8] * q[i8] * q[i19]
            + (-3.565308e-04) * q[i8] * q[i8] * q[i20] + (-3.602408e-05) * q[i8] * q[i8] * q[i21] + (-4.186722e-05) * q[i8] * q[i8] * q[i22]
            + (-7.039191e-03) * q[i0] * q[i9] * q[i9] + (1.026084e-03) * q[i1] * q[i9] * q[i9] + (-1.823829e-03) * q[i2] * q[i9] * q[i9]
            + (-3.393812e-03) * q[i3] * q[i9] * q[i9] + (6.176945e-04) * q[i4] * q[i9] * q[i9] + (1.423103e-03) * q[i5] * q[i9] * q[i9]
            + (-2.416989e-03) * q[i6] * q[i9] * q[i9] + (6.068129e-04) * q[i7] * q[i9] * q[i9] + (2.875656e-04) * q[i8] * q[i9] * q[i9]
            + (-7.837559e-04) * q[i9] * q[i9] * q[i9] + (3.294217e-04) * q[i9] * q[i9] * q[i10] + (-1.004519e-04) * q[i9] * q[i9] * q[i11]
            + (-2.476464e-04) * q[i9] * q[i9] * q[i12] + (3.525948e-05) * q[i9] * q[i9] * q[i15] + (2.191663e-04) * q[i9] * q[i9] * q[i16]
            + (-1.888503e-04) * q[i9] * q[i9] * q[i19] + (6.264896e-05) * q[i9] * q[i9] * q[i20] + (-4.165742e-05) * q[i9] * q[i9] * q[i21]
            + (3.440476e-06) * q[i9] * q[i9] * q[i22] + (1.015281e-03) * q[i0] * q[i10] * q[i10] + (-6.959196e-03) * q[i1] * q[i10] * q[i10]
            + (-1.797081e-03) * q[i2] * q[i10] * q[i10] + (6.085741e-04) * q[i3] * q[i10] * q[i10] + (-3.353310e-03) * q[i4] * q[i10] * q[i10]
            + (1.408923e-03) * q[i5] * q[i10] * q[i10] + (-5.953001e-04) * q[i6] * q[i10] * q[i10] + (2.383832e-03) * q[i7] * q[i10] * q[i10]
            + (-2.821292e-04) * q[i8] * q[i10] * q[i10] + (-3.284458e-04) * q[i9] * q[i10] * q[i10] + (7.730562e-04) * q[i10] * q[i10] * q[i10]
            + (-2.487220e-04) * q[i10] * q[i10] * q[i11] + (-9.833280e-05) * q[i10] * q[i10] * q[i12] + (-2.159147e-04) * q[i10] * q[i10] * q[i15]
            + (-3.608072e-05) * q[i10] * q[i10] * q[i16] + (-6.291108e-05) * q[i10] * q[i10] * q[i19] + (1.849190e-04) * q[i10] * q[i10] * q[i20]
            + (3.249624e-06) * q[i10] * q[i10] * q[i21] + (-4.041151e-05) * q[i10] * q[i10] * q[i22] + (4.014348e-04) * q[i0] * q[i11] * q[i11]
            + (-2.445407e-04) * q[i1] * q[i11] * q[i11] + (2.331302e-04) * q[i2] * q[i11] * q[i11] + (-4.269491e-04) * q[i3] * q[i11] * q[i11]
            + (4.152397e-04) * q[i4] * q[i11] * q[i11] + (-1.022053e-03) * q[i5] * q[i11] * q[i11] + (-3.015865e-04) * q[i6] * q[i11] * q[i11]
            + (-3.222720e-04) * q[i7] * q[i11] * q[i11] + (4.575323e-04) * q[i8] * q[i11] * q[i11] + (6.008311e-05) * q[i9] * q[i11] * q[i11]
            + (2.155967e-04) * q[i10] * q[i11] * q[i11] + (9.294861e-05) * q[i11] * q[i11] * q[i11] + (-5.585157e-05) * q[i11] * q[i11] * q[i12]
            + (1.792949e-03) * q[i11] * q[i11] * q[i15] + (1.490305e-04) * q[i11] * q[i11] * q[i16] + (5.161947e-04) * q[i11] * q[i11] * q[i19]
            + (-1.333448e-04) * q[i11] * q[i11] * q[i20] + (9.612420e-04) * q[i11] * q[i11] * q[i21] + (-1.220038e-04) * q[i11] * q[i11] * q[i22]
            + (-2.714360e-04) * q[i0] * q[i12] * q[i12] + (3.989079e-04) * q[i1] * q[i12] * q[i12] + (1.398483e-04) * q[i2] * q[i12] * q[i12]
            + (4.375516e-04) * q[i3] * q[i12] * q[i12] + (-4.336171e-04) * q[i4] * q[i12] * q[i12] + (-1.094256e-03) * q[i5] * q[i12] * q[i12]
            + (3.419038e-04) * q[i6] * q[i12] * q[i12] + (3.116004e-04) * q[i7] * q[i12] * q[i12] + (-3.806106e-04) * q[i8] * q[i12] * q[i12]
            + (-2.273791e-04) * q[i9] * q[i12] * q[i12] + (-6.828272e-05) * q[i10] * q[i12] * q[i12] + (-5.184264e-05) * q[i11] * q[i12] * q[i12]
            + (4.199221e-05) * q[i12] * q[i12] * q[i12] + (-1.595047e-04) * q[i12] * q[i12] * q[i15] + (-1.814456e-03) * q[i12] * q[i12] * q[i16]
            + (1.325286e-04) * q[i12] * q[i12] * q[i19] + (-4.996383e-04) * q[i12] * q[i12] * q[i20] + (-1.226814e-04) * q[i12] * q[i12] * q[i21]
            + (9.664559e-04) * q[i12] * q[i12] * q[i22] + (2.282337e-04) * q[i0] * q[i15] * q[i15] + (1.897088e-04) * q[i1] * q[i15] * q[i15]
            + (1.852820e-03) * q[i2] * q[i15] * q[i15] + (2.706537e-04) * q[i3] * q[i15] * q[i15] + (5.376847e-04) * q[i4] * q[i15] * q[i15]
            + (-2.289652e-03) * q[i5] * q[i15] * q[i15] + (-1.665728e-05) * q[i6] * q[i15] * q[i15] + (-2.762074e-05) * q[i7] * q[i15] * q[i15]
            + (2.054755e-03) * q[i8] * q[i15] * q[i15] + (2.068098e-05) * q[i9] * q[i15] * q[i15] + (7.112707e-05) * q[i10] * q[i15] * q[i15]
            + (2.203379e-03) * q[i11] * q[i15] * q[i15] + (7.106387e-05) * q[i12] * q[i15] * q[i15] + (5.023342e-04) * q[i15] * q[i15] * q[i15]
            + (8.242057e-06) * q[i15] * q[i15] * q[i16] + (1.147565e-04) * q[i15] * q[i15] * q[i19] + (-3.720846e-05) * q[i15] * q[i15] * q[i20]
            + (5.641917e-04) * q[i15] * q[i15] * q[i21] + (4.040847e-05) * q[i15] * q[i15] * q[i22] + (1.901364e-04) * q[i0] * q[i16] * q[i16]
            + (2.275479e-04) * q[i1] * q[i16] * q[i16] + (1.876151e-03) * q[i2] * q[i16] * q[i16] + (5.383057e-04) * q[i3] * q[i16] * q[i16]
            + (2.743125e-04) * q[i4] * q[i16] * q[i16] + (-2.306933e-03) * q[i5] * q[i16] * q[i16] + (2.869172e-05) * q[i6] * q[i16] * q[i16]
            + (2.036815e-05) * q[i7] * q[i16] * q[i16] + (-2.070227e-03) * q[i8] * q[i16] * q[i16] + (-7.112827e-05) * q[i9] * q[i16] * q[i16]
            + (-2.127310e-05) * q[i10] * q[i16] * q[i16] + (6.585089e-05) * q[i11] * q[i16] * q[i16] + (2.237602e-03) * q[i12] * q[i16] * q[i16]
            + (-8.383803e-06) * q[i15] * q[i16] * q[i16] + (-5.035672e-04) * q[i16] * q[i16] * q[i16] + (3.819796e-05) * q[i16] * q[i16] * q[i19]
            + (-1.095250e-04) * q[i16] * q[i16] * q[i20] + (4.022276e-05) * q[i16] * q[i16] * q[i21] + (5.704893e-04) * q[i16] * q[i16] * q[i22]
            + (-1.025481e-04) * q[i0] * q[i19] * q[i19] + (-2.613781e-06) * q[i1] * q[i19] * q[i19] + (-1.497534e-04) * q[i2] * q[i19] * q[i19]
            + (-8.723253e-05) * q[i3] * q[i19] * q[i19] + (-1.017205e-06) * q[i4] * q[i19] * q[i19] + (-2.370621e-05) * q[i5] * q[i19] * q[i19]
            + (-7.604286e-05) * q[i6] * q[i19] * q[i19] + (4.005731e-05) * q[i7] * q[i19] * q[i19] + (9.644574e-05) * q[i8] * q[i19] * q[i19]
            + (-5.995082e-05) * q[i9] * q[i19] * q[i19] + (-7.298273e-05) * q[i10] * q[i19] * q[i19] + (5.587782e-05) * q[i11] * q[i19] * q[i19]
            + (-8.649770e-05) * q[i12] * q[i19] * q[i19] + (2.662236e-04) * q[i15] * q[i19] * q[i19] + (6.883622e-05) * q[i16] * q[i19] * q[i19]
            + (1.031562e-04) * q[i19] * q[i19] * q[i19] + (-2.962465e-05) * q[i19] * q[i19] * q[i20] + (7.039310e-04) * q[i19] * q[i19] * q[i21]
            + (-6.772213e-05) * q[i19] * q[i19] * q[i22] + (-1.444755e-06) * q[i0] * q[i20] * q[i20] + (-9.771895e-05) * q[i1] * q[i20] * q[i20]
            + (-1.462692e-04) * q[i2] * q[i20] * q[i20] + (-5.818832e-06) * q[i3] * q[i20] * q[i20] + (-8.774236e-05) * q[i4] * q[i20] * q[i20]
            + (-1.470971e-05) * q[i5] * q[i20] * q[i20] + (-4.128587e-05) * q[i6] * q[i20] * q[i20] + (7.549052e-05) * q[i7] * q[i20] * q[i20]
            + (-1.047269e-04) * q[i8] * q[i20] * q[i20] + (7.274886e-05) * q[i9] * q[i20] * q[i20] + (5.974990e-05) * q[i10] * q[i20] * q[i20]
            + (-8.713647e-05) * q[i11] * q[i20] * q[i20] + (7.128725e-05) * q[i12] * q[i20] * q[i20] + (-6.891444e-05) * q[i15] * q[i20] * q[i20]
            + (-2.606892e-04) * q[i16] * q[i20] * q[i20] + (2.959283e-05) * q[i19] * q[i20] * q[i20] + (-9.737891e-05) * q[i20] * q[i20] * q[i20]
            + (-6.756140e-05) * q[i20] * q[i20] * q[i21] + (7.037935e-04) * q[i20] * q[i20] * q[i22] + (1.070024e-04) * q[i0] * q[i21] * q[i21]
            + (-1.453267e-04) * q[i1] * q[i21] * q[i21] + (2.625060e-04) * q[i2] * q[i21] * q[i21] + (-6.384954e-05) * q[i3] * q[i21] * q[i21]
            + (6.024040e-04) * q[i4] * q[i21] * q[i21] + (-1.243868e-03) * q[i5] * q[i21] * q[i21] + (-5.489398e-05) * q[i6] * q[i21] * q[i21]
            + (-1.488944e-04) * q[i7] * q[i21] * q[i21] + (7.672063e-04) * q[i8] * q[i21] * q[i21] + (1.329937e-05) * q[i9] * q[i21] * q[i21]
            + (-4.647722e-05) * q[i10] * q[i21] * q[i21] + (4.934464e-04) * q[i11] * q[i21] * q[i21] + (-8.258681e-06) * q[i12] * q[i21] * q[i21]
            + (7.005584e-04) * q[i15] * q[i21] * q[i21] + (-1.700324e-05) * q[i16] * q[i21] * q[i21] + (-1.850370e-04) * q[i19] * q[i21] * q[i21]
            + (-3.964983e-05) * q[i20] * q[i21] * q[i21] + (3.685271e-04) * q[i21] * q[i21] * q[i21] + (3.164260e-05) * q[i21] * q[i21] * q[i22]
            + (-1.444598e-04) * q[i0] * q[i22] * q[i22] + (1.078829e-04) * q[i1] * q[i22] * q[i22] + (2.635518e-04) * q[i2] * q[i22] * q[i22]
            + (6.055682e-04) * q[i3] * q[i22] * q[i22] + (-6.108601e-05) * q[i4] * q[i22] * q[i22] + (-1.245705e-03) * q[i5] * q[i22] * q[i22]
            + (1.492691e-04) * q[i6] * q[i22] * q[i22] + (5.372295e-05) * q[i7] * q[i22] * q[i22] + (-7.750709e-04) * q[i8] * q[i22] * q[i22]
            + (4.793143e-05) * q[i9] * q[i22] * q[i22] + (-1.234508e-05) * q[i10] * q[i22] * q[i22] + (-9.397988e-06) * q[i11] * q[i22] * q[i22]
            + (4.990193e-04) * q[i12] * q[i22] * q[i22] + (1.858024e-05) * q[i15] * q[i22] * q[i22] + (-7.034394e-04) * q[i16] * q[i22] * q[i22]
            + (4.007287e-05) * q[i19] * q[i22] * q[i22] + (1.833804e-04) * q[i20] * q[i22] * q[i22] + (3.202463e-05) * q[i21] * q[i22] * q[i22]
            + (3.702771e-04) * q[i22] * q[i22] * q[i22] + (-1.194385e-03) * q[i0] * q[i1] * q[i2] + (7.610132e-03) * q[i0] * q[i1] * q[i3]
            + (7.626269e-03) * q[i0] * q[i1] * q[i4] + (-3.021663e-03) * q[i0] * q[i1] * q[i5] + (3.010431e-04) * q[i0] * q[i1] * q[i6]
            + (-2.984609e-04) * q[i0] * q[i1] * q[i7] + (7.173709e-06) * q[i0] * q[i1] * q[i8] + (-9.985536e-04) * q[i0] * q[i1] * q[i9]
            + (9.903339e-04) * q[i0] * q[i1] * q[i10] + (-6.081333e-04) * q[i0] * q[i1] * q[i11] + (-6.105035e-04) * q[i0] * q[i1] * q[i12]
            + (7.877042e-04) * q[i0] * q[i1] * q[i15] + (-7.890194e-04) * q[i0] * q[i1] * q[i16] + (-9.758276e-04) * q[i0] * q[i1] * q[i19]
            + (9.787422e-04) * q[i0] * q[i1] * q[i20] + (-2.022340e-06) * q[i0] * q[i1] * q[i21] + (-4.565074e-06) * q[i0] * q[i1] * q[i22]
            + (-1.244728e-02) * q[i0] * q[i2] * q[i3] + (-2.347755e-04) * q[i0] * q[i2] * q[i4] + (-8.450503e-03) * q[i0] * q[i2] * q[i5]
            + (3.948689e-04) * q[i0] * q[i2] * q[i6] + (-3.649659e-04) * q[i0] * q[i2] * q[i7] + (-2.402488e-03) * q[i0] * q[i2] * q[i8]
            + (1.025103e-03) * q[i0] * q[i2] * q[i9] + (-2.223396e-04) * q[i0] * q[i2] * q[i10] + (1.562511e-04) * q[i0] * q[i2] * q[i11]
            + (8.152518e-04) * q[i0] * q[i2] * q[i12] + (-7.861968e-04) * q[i0] * q[i2] * q[i15] + (2.308786e-03) * q[i0] * q[i2] * q[i16]
            + (3.991832e-05) * q[i0] * q[i2] * q[i19] + (8.805669e-04) * q[i0] * q[i2] * q[i20] + (2.581975e-04) * q[i0] * q[i2] * q[i21]
            + (-6.659603e-05) * q[i0] * q[i2] * q[i22] + (1.838885e-04) * q[i0] * q[i3] * q[i4] + (3.433614e-03) * q[i0] * q[i3] * q[i5]
            + (-1.054360e-02) * q[i0] * q[i3] * q[i6] + (2.862776e-03) * q[i0] * q[i3] * q[i7] + (2.243601e-03) * q[i0] * q[i3] * q[i8]
            + (-8.869268e-04) * q[i0] * q[i3] * q[i9] + (2.287754e-03) * q[i0] * q[i3] * q[i10] + (-1.845070e-03) * q[i0] * q[i3] * q[i11]
            + (-1.582897e-03) * q[i0] * q[i3] * q[i12] + (1.435510e-03) * q[i0] * q[i3] * q[i15] + (9.760053e-04) * q[i0] * q[i3] * q[i16]
            + (7.935073e-04) * q[i0] * q[i3] * q[i19] + (3.412249e-04) * q[i0] * q[i3] * q[i20] + (-1.788544e-04) * q[i0] * q[i3] * q[i21]
            + (-2.821919e-04) * q[i0] * q[i3] * q[i22] + (2.288322e-03) * q[i0] * q[i4] * q[i5] + (-2.369617e-03) * q[i0] * q[i4] * q[i6]
            + (-1.188181e-03) * q[i0] * q[i4] * q[i7] + (-1.034320e-03) * q[i0] * q[i4] * q[i8] + (5.048114e-04) * q[i0] * q[i4] * q[i9]
            + (3.749607e-04) * q[i0] * q[i4] * q[i10] + (3.848988e-04) * q[i0] * q[i4] * q[i11] + (1.873872e-04) * q[i0] * q[i4] * q[i12]
            + (4.058198e-04) * q[i0] * q[i4] * q[i15] + (9.716004e-05) * q[i0] * q[i4] * q[i16] + (4.639175e-04) * q[i0] * q[i4] * q[i19]
            + (-2.708977e-04) * q[i0] * q[i4] * q[i20] + (1.816487e-04) * q[i0] * q[i4] * q[i21] + (1.422256e-04) * q[i0] * q[i4] * q[i22]
            + (-7.721839e-03) * q[i0] * q[i5] * q[i6] + (-1.202393e-03) * q[i0] * q[i5] * q[i7] + (5.711739e-04) * q[i0] * q[i5] * q[i8]
            + (-5.320007e-04) * q[i0] * q[i5] * q[i9] + (4.327036e-05) * q[i0] * q[i5] * q[i10] + (1.515204e-03) * q[i0] * q[i5] * q[i11]
            + (-3.911966e-04) * q[i0] * q[i5] * q[i12] + (1.990528e-04) * q[i0] * q[i5] * q[i15] + (-1.860218e-04) * q[i0] * q[i5] * q[i16]
            + (-5.553259e-04) * q[i0] * q[i5] * q[i19] + (1.763750e-04) * q[i0] * q[i5] * q[i20] + (5.344639e-04) * q[i0] * q[i5] * q[i21]
            + (-1.737749e-04) * q[i0] * q[i5] * q[i22] + (-1.080780e-03) * q[i0] * q[i6] * q[i7] + (6.325835e-03) * q[i0] * q[i6] * q[i8]
            + (-1.465474e-02) * q[i0] * q[i6] * q[i9] + (3.719124e-03) * q[i0] * q[i6] * q[i10] + (-7.742778e-04) * q[i0] * q[i6] * q[i11]
            + (2.973193e-04) * q[i0] * q[i6] * q[i12] + (1.582378e-03) * q[i0] * q[i6] * q[i15] + (1.714890e-03) * q[i0] * q[i6] * q[i16]
            + (5.418329e-04) * q[i0] * q[i6] * q[i19] + (-3.649028e-04) * q[i0] * q[i6] * q[i20] + (-5.273413e-04) * q[i0] * q[i6] * q[i21]
            + (5.840159e-04) * q[i0] * q[i6] * q[i22] + (-1.829260e-04) * q[i0] * q[i7] * q[i8] + (-1.361720e-04) * q[i0] * q[i7] * q[i9]
            + (1.584395e-03) * q[i0] * q[i7] * q[i10] + (-2.102670e-04) * q[i0] * q[i7] * q[i11] + (-8.309921e-04) * q[i0] * q[i7] * q[i12]
            + (-4.744342e-04) * q[i0] * q[i7] * q[i15] + (-3.210253e-04) * q[i0] * q[i7] * q[i16] + (6.264215e-04) * q[i0] * q[i7] * q[i19]
            + (-7.089603e-05) * q[i0] * q[i7] * q[i20] + (-1.047804e-04) * q[i0] * q[i7] * q[i21] + (-2.431557e-04) * q[i0] * q[i7] * q[i22]
            + (5.752271e-04) * q[i0] * q[i8] * q[i9] + (-6.227911e-04) * q[i0] * q[i8] * q[i10] + (-5.047233e-04) * q[i0] * q[i8] * q[i11]
            + (6.275289e-04) * q[i0] * q[i8] * q[i12] + (4.118524e-04) * q[i0] * q[i8] * q[i15] + (1.008221e-03) * q[i0] * q[i8] * q[i16]
            + (4.729393e-04) * q[i0] * q[i8] * q[i19] + (3.312547e-04) * q[i0] * q[i8] * q[i20] + (2.722731e-05) * q[i0] * q[i8] * q[i21]
            + (-2.079227e-04) * q[i0] * q[i8] * q[i22] + (5.517986e-04) * q[i0] * q[i9] * q[i10] + (-8.545944e-05) * q[i0] * q[i9] * q[i11]
            + (3.097661e-04) * q[i0] * q[i9] * q[i12] + (3.770447e-04) * q[i0] * q[i9] * q[i15] + (2.400579e-04) * q[i0] * q[i9] * q[i16]
            + (4.301947e-04) * q[i0] * q[i9] * q[i19] + (1.408991e-04) * q[i0] * q[i9] * q[i20] + (-1.125004e-04) * q[i0] * q[i9] * q[i21]
            + (7.431093e-05) * q[i0] * q[i9] * q[i22] + (-6.850180e-06) * q[i0] * q[i10] * q[i11] + (4.021818e-06) * q[i0] * q[i10] * q[i12]
            + (2.035634e-04) * q[i0] * q[i10] * q[i15] + (-9.682506e-05) * q[i0] * q[i10] * q[i16] + (8.105425e-05) * q[i0] * q[i10] * q[i19]
            + (-3.296448e-04) * q[i0] * q[i10] * q[i20] + (-3.191838e-04) * q[i0] * q[i10] * q[i21] + (-1.513056e-04) * q[i0] * q[i10] * q[i22]
            + (1.621298e-04) * q[i0] * q[i11] * q[i12] + (1.406522e-04) * q[i0] * q[i11] * q[i15] + (3.678196e-04) * q[i0] * q[i11] * q[i16]
            + (1.275807e-04) * q[i0] * q[i11] * q[i19] + (6.317997e-05) * q[i0] * q[i11] * q[i20] + (2.023471e-04) * q[i0] * q[i11] * q[i21]
            + (-1.685427e-04) * q[i0] * q[i11] * q[i22] + (1.160218e-04) * q[i0] * q[i12] * q[i15] + (-5.135890e-04) * q[i0] * q[i12] * q[i16]
            + (6.372694e-05) * q[i0] * q[i12] * q[i19] + (-1.227215e-04) * q[i0] * q[i12] * q[i20] + (2.551483e-04) * q[i0] * q[i12] * q[i21]
            + (1.660179e-05) * q[i0] * q[i12] * q[i22] + (1.680045e-04) * q[i0] * q[i15] * q[i16] + (-1.696522e-04) * q[i0] * q[i15] * q[i19]
            + (-8.786154e-05) * q[i0] * q[i15] * q[i20] + (4.839724e-04) * q[i0] * q[i15] * q[i21] + (9.873012e-05) * q[i0] * q[i15] * q[i22]
            + (-2.713472e-04) * q[i0] * q[i16] * q[i19] + (2.116721e-04) * q[i0] * q[i16] * q[i20] + (-8.932882e-06) * q[i0] * q[i16] * q[i21]
            + (-7.559066e-05) * q[i0] * q[i16] * q[i22] + (-2.635676e-05) * q[i0] * q[i19] * q[i20] + (2.077649e-04) * q[i0] * q[i19] * q[i21]
            + (-4.681289e-05) * q[i0] * q[i19] * q[i22] + (1.602136e-04) * q[i0] * q[i20] * q[i21] + (1.849766e-04) * q[i0] * q[i20] * q[i22]
            + (7.868737e-05) * q[i0] * q[i21] * q[i22] + (-2.168192e-04) * q[i1] * q[i2] * q[i3] + (-1.237445e-02) * q[i1] * q[i2] * q[i4]
            + (-8.393336e-03) * q[i1] * q[i2] * q[i5] + (3.588026e-04) * q[i1] * q[i2] * q[i6] + (-3.874749e-04) * q[i1] * q[i2] * q[i7]
            + (2.394033e-03) * q[i1] * q[i2] * q[i8] + (2.287002e-04) * q[i1] * q[i2] * q[i9] + (-1.015867e-03) * q[i1] * q[i2] * q[i10]
            + (8.179783e-04) * q[i1] * q[i2] * q[i11] + (1.544878e-04) * q[i1] * q[i2] * q[i12] + (-2.282695e-03) * q[i1] * q[i2] * q[i15]
            + (7.919335e-04) * q[i1] * q[i2] * q[i16] + (-8.783529e-04) * q[i1] * q[i2] * q[i19] + (-3.859424e-05) * q[i1] * q[i2] * q[i20]
            + (-7.342616e-05) * q[i1] * q[i2] * q[i21] + (2.587410e-04) * q[i1] * q[i2] * q[i22] + (1.903197e-04) * q[i1] * q[i3] * q[i4]
            + (2.304678e-03) * q[i1] * q[i3] * q[i5] + (1.181236e-03) * q[i1] * q[i3] * q[i6] + (2.355013e-03) * q[i1] * q[i3] * q[i7]
            + (1.040977e-03) * q[i1] * q[i3] * q[i8] + (-3.836647e-04) * q[i1] * q[i3] * q[i9] + (-5.058233e-04) * q[i1] * q[i3] * q[i10]
            + (1.973673e-04) * q[i1] * q[i3] * q[i11] + (3.859845e-04) * q[i1] * q[i3] * q[i12] + (-1.024926e-04) * q[i1] * q[i3] * q[i15]
            + (-4.060882e-04) * q[i1] * q[i3] * q[i16] + (2.787432e-04) * q[i1] * q[i3] * q[i19] + (-4.741077e-04) * q[i1] * q[i3] * q[i20]
            + (1.423696e-04) * q[i1] * q[i3] * q[i21] + (1.807528e-04) * q[i1] * q[i3] * q[i22] + (3.417656e-03) * q[i1] * q[i4] * q[i5]
            + (-2.834066e-03) * q[i1] * q[i4] * q[i6] + (1.048470e-02) * q[i1] * q[i4] * q[i7] + (-2.226817e-03) * q[i1] * q[i4] * q[i8]
            + (-2.315400e-03) * q[i1] * q[i4] * q[i9] + (8.886449e-04) * q[i1] * q[i4] * q[i10] + (-1.600283e-03) * q[i1] * q[i4] * q[i11]
            + (-1.832126e-03) * q[i1] * q[i4] * q[i12] + (-9.721177e-04) * q[i1] * q[i4] * q[i15] + (-1.436457e-03) * q[i1] * q[i4] * q[i16]
            + (-3.493539e-04) * q[i1] * q[i4] * q[i19] + (-7.889787e-04) * q[i1] * q[i4] * q[i20] + (-2.771836e-04) * q[i1] * q[i4] * q[i21]
            + (-1.891432e-04) * q[i1] * q[i4] * q[i22] + (1.183537e-03) * q[i1] * q[i5] * q[i6] + (7.746703e-03) * q[i1] * q[i5] * q[i7]
            + (-5.622257e-04) * q[i1] * q[i5] * q[i8] + (-5.807911e-05) * q[i1] * q[i5] * q[i9] + (5.458980e-04) * q[i1] * q[i5] * q[i10]
            + (-3.894842e-04) * q[i1] * q[i5] * q[i11] + (1.525443e-03) * q[i1] * q[i5] * q[i12] + (1.882025e-04) * q[i1] * q[i5] * q[i15]
            + (-2.026340e-04) * q[i1] * q[i5] * q[i16] + (-1.674119e-04) * q[i1] * q[i5] * q[i19] + (5.510449e-04) * q[i1] * q[i5] * q[i20]
            + (-1.820496e-04) * q[i1] * q[i5] * q[i21] + (5.276585e-04) * q[i1] * q[i5] * q[i22] + (-1.056407e-03) * q[i1] * q[i6] * q[i7]
            + (-1.799223e-04) * q[i1] * q[i6] * q[i8] + (1.606109e-03) * q[i1] * q[i6] * q[i9] + (-1.269122e-04) * q[i1] * q[i6] * q[i10]
            + (8.367449e-04) * q[i1] * q[i6] * q[i11] + (2.208027e-04) * q[i1] * q[i6] * q[i12] + (-3.151395e-04) * q[i1] * q[i6] * q[i15]
            + (-4.786425e-04) * q[i1] * q[i6] * q[i16] + (-7.087169e-05) * q[i1] * q[i6] * q[i19] + (6.264196e-04) * q[i1] * q[i6] * q[i20]
            + (2.398719e-04) * q[i1] * q[i6] * q[i21] + (1.031743e-04) * q[i1] * q[i6] * q[i22] + (6.316686e-03) * q[i1] * q[i7] * q[i8]
            + (3.751410e-03) * q[i1] * q[i7] * q[i9] + (-1.448508e-02) * q[i1] * q[i7] * q[i10] + (-2.759820e-04) * q[i1] * q[i7] * q[i11]
            + (7.933086e-04) * q[i1] * q[i7] * q[i12] + (1.680669e-03) * q[i1] * q[i7] * q[i15] + (1.597703e-03) * q[i1] * q[i7] * q[i16]
            + (-3.730352e-04) * q[i1] * q[i7] * q[i19] + (5.390420e-04) * q[i1] * q[i7] * q[i20] + (-5.712384e-04) * q[i1] * q[i7] * q[i21]
            + (5.396924e-04) * q[i1] * q[i7] * q[i22] + (-6.288250e-04) * q[i1] * q[i8] * q[i9] + (5.758056e-04) * q[i1] * q[i8] * q[i10]
            + (-5.762647e-04) * q[i1] * q[i8] * q[i11] + (5.423433e-04) * q[i1] * q[i8] * q[i12] + (9.890983e-04) * q[i1] * q[i8] * q[i15]
            + (4.193216e-04) * q[i1] * q[i8] * q[i16] + (3.292585e-04) * q[i1] * q[i8] * q[i19] + (4.634254e-04) * q[i1] * q[i8] * q[i20]
            + (2.051705e-04) * q[i1] * q[i8] * q[i21] + (-2.760091e-05) * q[i1] * q[i8] * q[i22] + (5.497590e-04) * q[i1] * q[i9] * q[i10]
            + (-3.580780e-06) * q[i1] * q[i9] * q[i11] + (3.420878e-06) * q[i1] * q[i9] * q[i12] + (-9.988166e-05) * q[i1] * q[i9] * q[i15]
            + (2.043830e-04) * q[i1] * q[i9] * q[i16] + (-3.293673e-04) * q[i1] * q[i9] * q[i19] + (7.962626e-05) * q[i1] * q[i9] * q[i20]
            + (1.490205e-04) * q[i1] * q[i9] * q[i21] + (3.217950e-04) * q[i1] * q[i9] * q[i22] + (-3.052562e-04) * q[i1] * q[i10] * q[i11]
            + (8.281640e-05) * q[i1] * q[i10] * q[i12] + (2.340344e-04) * q[i1] * q[i10] * q[i15] + (3.797765e-04) * q[i1] * q[i10] * q[i16]
            + (1.399759e-04) * q[i1] * q[i10] * q[i19] + (4.300117e-04) * q[i1] * q[i10] * q[i20] + (-7.301478e-05) * q[i1] * q[i10] * q[i21]
            + (1.116875e-04) * q[i1] * q[i10] * q[i22] + (1.603131e-04) * q[i1] * q[i11] * q[i12] + (5.024914e-04) * q[i1] * q[i11] * q[i15]
            + (-1.164569e-04) * q[i1] * q[i11] * q[i16] + (1.243312e-04) * q[i1] * q[i11] * q[i19] + (-6.589012e-05) * q[i1] * q[i11] * q[i20]
            + (1.314783e-05) * q[i1] * q[i11] * q[i21] + (2.562168e-04) * q[i1] * q[i11] * q[i22] + (-3.718481e-04) * q[i1] * q[i12] * q[i15]
            + (-1.442976e-04) * q[i1] * q[i12] * q[i16] + (-6.553262e-05) * q[i1] * q[i12] * q[i19] + (-1.224903e-04) * q[i1] * q[i12] * q[i20]
            + (-1.676125e-04) * q[i1] * q[i12] * q[i21] + (2.016792e-04) * q[i1] * q[i12] * q[i22] + (1.628166e-04) * q[i1] * q[i15] * q[i16]
            + (2.174304e-04) * q[i1] * q[i15] * q[i19] + (-2.712922e-04) * q[i1] * q[i15] * q[i20] + (7.240107e-05) * q[i1] * q[i15] * q[i21]
            + (6.014123e-06) * q[i1] * q[i15] * q[i22] + (-8.246948e-05) * q[i1] * q[i16] * q[i19] + (-1.668326e-04) * q[i1] * q[i16] * q[i20]
            + (-9.617169e-05) * q[i1] * q[i16] * q[i21] + (-4.835072e-04) * q[i1] * q[i16] * q[i22] + (-3.140601e-05) * q[i1] * q[i19] * q[i20]
            + (-1.897953e-04) * q[i1] * q[i19] * q[i21] + (-1.584384e-04) * q[i1] * q[i19] * q[i22] + (4.808083e-05) * q[i1] * q[i20] * q[i21]
            + (-2.088448e-04) * q[i1] * q[i20] * q[i22] + (7.632382e-05) * q[i1] * q[i21] * q[i22] + (-6.316623e-05) * q[i2] * q[i3] * q[i4]
            + (3.768436e-03) * q[i2] * q[i3] * q[i5] + (-3.102689e-03) * q[i2] * q[i3] * q[i6] + (-2.698023e-04) * q[i2] * q[i3] * q[i7]
            + (3.073255e-03) * q[i2] * q[i3] * q[i8] + (-3.824442e-04) * q[i2] * q[i3] * q[i9] + (2.621603e-04) * q[i2] * q[i3] * q[i10]
            + (-3.181467e-05) * q[i2] * q[i3] * q[i11] + (9.610482e-05) * q[i2] * q[i3] * q[i12] + (1.055586e-03) * q[i2] * q[i3] * q[i15]
            + (-6.409353e-04) * q[i2] * q[i3] * q[i16] + (-5.450766e-04) * q[i2] * q[i3] * q[i19] + (-6.409211e-04) * q[i2] * q[i3] * q[i20]
            + (-3.747750e-05) * q[i2] * q[i3] * q[i21] + (-2.728672e-05) * q[i2] * q[i3] * q[i22] + (3.768350e-03) * q[i2] * q[i4] * q[i5]
            + (2.777308e-04) * q[i2] * q[i4] * q[i6] + (3.087575e-03) * q[i2] * q[i4] * q[i7] + (-3.044392e-03) * q[i2] * q[i4] * q[i8]
            + (-2.735006e-04) * q[i2] * q[i4] * q[i9] + (3.840404e-04) * q[i2] * q[i4] * q[i10] + (7.071592e-05) * q[i2] * q[i4] * q[i11]
            + (-7.793834e-06) * q[i2] * q[i4] * q[i12] + (6.335989e-04) * q[i2] * q[i4] * q[i15] + (-1.068054e-03) * q[i2] * q[i4] * q[i16]
            + (6.371539e-04) * q[i2] * q[i4] * q[i19] + (5.427403e-04) * q[i2] * q[i4] * q[i20] + (-2.653857e-05) * q[i2] * q[i4] * q[i21]
            + (-4.333677e-05) * q[i2] * q[i4] * q[i22] + (5.061598e-04) * q[i2] * q[i5] * q[i6] + (-5.146189e-04) * q[i2] * q[i5] * q[i7]
            + (-3.873137e-05) * q[i2] * q[i5] * q[i8] + (1.587581e-03) * q[i2] * q[i5] * q[i9] + (-1.582219e-03) * q[i2] * q[i5] * q[i10]
            + (2.228351e-03) * q[i2] * q[i5] * q[i11] + (2.197321e-03) * q[i2] * q[i5] * q[i12] + (-1.390519e-03) * q[i2] * q[i5] * q[i15]
            + (1.387809e-03) * q[i2] * q[i5] * q[i16] + (1.181564e-03) * q[i2] * q[i5] * q[i19] + (-1.176193e-03) * q[i2] * q[i5] * q[i20]
            + (6.264919e-04) * q[i2] * q[i5] * q[i21] + (6.305250e-04) * q[i2] * q[i5] * q[i22] + (-1.377426e-04) * q[i2] * q[i6] * q[i7]
            + (5.054400e-03) * q[i2] * q[i6] * q[i8] + (-3.439988e-03) * q[i2] * q[i6] * q[i9] + (1.431572e-03) * q[i2] * q[i6] * q[i10]
            + (-7.221575e-04) * q[i2] * q[i6] * q[i11] + (-2.130224e-05) * q[i2] * q[i6] * q[i12] + (6.749095e-04) * q[i2] * q[i6] * q[i15]
            + (-4.701494e-04) * q[i2] * q[i6] * q[i16] + (-1.506784e-04) * q[i2] * q[i6] * q[i19] + (2.140396e-04) * q[i2] * q[i6] * q[i20]
            + (2.169045e-04) * q[i2] * q[i6] * q[i21] + (-1.013324e-04) * q[i2] * q[i6] * q[i22] + (5.048879e-03) * q[i2] * q[i7] * q[i8]
            + (1.438721e-03) * q[i2] * q[i7] * q[i9] + (-3.384361e-03) * q[i2] * q[i7] * q[i10] + (3.092087e-05) * q[i2] * q[i7] * q[i11]
            + (7.329934e-04) * q[i2] * q[i7] * q[i12] + (-4.618498e-04) * q[i2] * q[i7] * q[i15] + (6.752372e-04) * q[i2] * q[i7] * q[i16]
            + (2.106647e-04) * q[i2] * q[i7] * q[i19] + (-1.452176e-04) * q[i2] * q[i7] * q[i20] + (1.059136e-04) * q[i2] * q[i7] * q[i21]
            + (-2.113594e-04) * q[i2] * q[i7] * q[i22] + (-3.521480e-03) * q[i2] * q[i8] * q[i9] + (-3.483055e-03) * q[i2] * q[i8] * q[i10]
            + (-9.728021e-04) * q[i2] * q[i8] * q[i11] + (1.148406e-03) * q[i2] * q[i8] * q[i12] + (5.609538e-03) * q[i2] * q[i8] * q[i15]
            + (5.688234e-03) * q[i2] * q[i8] * q[i16] + (2.465393e-04) * q[i2] * q[i8] * q[i19] + (2.382505e-04) * q[i2] * q[i8] * q[i20]
            + (-3.458079e-04) * q[i2] * q[i8] * q[i21] + (3.538491e-04) * q[i2] * q[i8] * q[i22] + (1.051741e-06) * q[i2] * q[i9] * q[i10]
            + (-1.846460e-04) * q[i2] * q[i9] * q[i11] + (-5.668092e-05) * q[i2] * q[i9] * q[i12] + (5.058564e-06) * q[i2] * q[i9] * q[i15]
            + (-1.346369e-04) * q[i2] * q[i9] * q[i16] + (1.978261e-05) * q[i2] * q[i9] * q[i19] + (9.105248e-05) * q[i2] * q[i9] * q[i20]
            + (-3.524261e-05) * q[i2] * q[i9] * q[i21] + (1.524644e-04) * q[i2] * q[i9] * q[i22] + (4.245635e-05) * q[i2] * q[i10] * q[i11]
            + (1.703269e-04) * q[i2] * q[i10] * q[i12] + (-1.328026e-04) * q[i2] * q[i10] * q[i15] + (9.072092e-06) * q[i2] * q[i10] * q[i16]
            + (9.332384e-05) * q[i2] * q[i10] * q[i19] + (2.200496e-05) * q[i2] * q[i10] * q[i20] + (-1.501000e-04) * q[i2] * q[i10] * q[i21]
            + (3.215402e-05) * q[i2] * q[i10] * q[i22] + (-3.372752e-04) * q[i2] * q[i11] * q[i12] + (3.213017e-03) * q[i2] * q[i11] * q[i15]
            + (-2.787331e-04) * q[i2] * q[i11] * q[i16] + (9.466732e-04) * q[i2] * q[i11] * q[i19] + (-2.194630e-04) * q[i2] * q[i11] * q[i20]
            + (7.741898e-04) * q[i2] * q[i11] * q[i21] + (-7.041495e-05) * q[i2] * q[i11] * q[i22] + (2.604640e-04) * q[i2] * q[i12] * q[i15]
            + (-3.231733e-03) * q[i2] * q[i12] * q[i16] + (2.152759e-04) * q[i2] * q[i12] * q[i19] + (-9.212270e-04) * q[i2] * q[i12] * q[i20]
            + (-7.210027e-05) * q[i2] * q[i12] * q[i21] + (7.730127e-04) * q[i2] * q[i12] * q[i22] + (1.514747e-05) * q[i2] * q[i15] * q[i16]
            + (-7.424004e-04) * q[i2] * q[i15] * q[i19] + (-4.808458e-04) * q[i2] * q[i15] * q[i20] + (1.176342e-05) * q[i2] * q[i15] * q[i21]
            + (2.144045e-04) * q[i2] * q[i15] * q[i22] + (-4.820720e-04) * q[i2] * q[i16] * q[i19] + (-7.416021e-04) * q[i2] * q[i16] * q[i20]
            + (-2.151514e-04) * q[i2] * q[i16] * q[i21] + (-1.013329e-05) * q[i2] * q[i16] * q[i22] + (7.708067e-05) * q[i2] * q[i19] * q[i20]
            + (-5.448979e-04) * q[i2] * q[i19] * q[i21] + (-2.062266e-04) * q[i2] * q[i19] * q[i22] + (2.072959e-04) * q[i2] * q[i20] * q[i21]
            + (5.365955e-04) * q[i2] * q[i20] * q[i22] + (-1.536732e-05) * q[i2] * q[i21] * q[i22] + (-5.909751e-04) * q[i3] * q[i4] * q[i5]
            + (1.084108e-03) * q[i3] * q[i4] * q[i6] + (-1.089899e-03) * q[i3] * q[i4] * q[i7] + (1.260295e-06) * q[i3] * q[i4] * q[i8]
            + (-4.328217e-04) * q[i3] * q[i4] * q[i9] + (4.310851e-04) * q[i3] * q[i4] * q[i10] + (2.821409e-04) * q[i3] * q[i4] * q[i11]
            + (2.781655e-04) * q[i3] * q[i4] * q[i12] + (3.733548e-04) * q[i3] * q[i4] * q[i15] + (-3.644859e-04) * q[i3] * q[i4] * q[i16]
            + (1.639802e-04) * q[i3] * q[i4] * q[i19] + (-1.619049e-04) * q[i3] * q[i4] * q[i20] + (-7.005385e-04) * q[i3] * q[i4] * q[i21]
            + (-7.129517e-04) * q[i3] * q[i4] * q[i22] + (-1.677210e-03) * q[i3] * q[i5] * q[i6] + (-1.603558e-04) * q[i3] * q[i5] * q[i7]
            + (-2.791834e-04) * q[i3] * q[i5] * q[i8] + (2.351077e-04) * q[i3] * q[i5] * q[i9] + (-9.716475e-05) * q[i3] * q[i5] * q[i10]
            + (-1.227979e-03) * q[i3] * q[i5] * q[i11] + (6.743793e-04) * q[i3] * q[i5] * q[i12] + (8.657755e-04) * q[i3] * q[i5] * q[i15]
            + (-3.537567e-04) * q[i3] * q[i5] * q[i16] + (3.344671e-04) * q[i3] * q[i5] * q[i19] + (-6.595843e-04) * q[i3] * q[i5] * q[i20]
            + (3.193782e-04) * q[i3] * q[i5] * q[i21] + (-7.617263e-06) * q[i3] * q[i5] * q[i22] + (-1.257061e-04) * q[i3] * q[i6] * q[i7]
            + (-1.520813e-03) * q[i3] * q[i6] * q[i8] + (-1.066384e-02) * q[i3] * q[i6] * q[i9] + (6.920992e-04) * q[i3] * q[i6] * q[i10]
            + (-2.072890e-04) * q[i3] * q[i6] * q[i11] + (-2.334922e-05) * q[i3] * q[i6] * q[i12] + (2.047130e-04) * q[i3] * q[i6] * q[i15]
            + (-9.212668e-04) * q[i3] * q[i6] * q[i16] + (-5.412548e-04) * q[i3] * q[i6] * q[i19] + (5.468548e-04) * q[i3] * q[i6] * q[i20]
            + (4.925865e-06) * q[i3] * q[i6] * q[i21] + (4.098744e-04) * q[i3] * q[i6] * q[i22] + (2.248550e-04) * q[i3] * q[i7] * q[i8]
            + (2.845153e-04) * q[i3] * q[i7] * q[i9] + (1.740196e-03) * q[i3] * q[i7] * q[i10] + (-3.172781e-04) * q[i3] * q[i7] * q[i11]
            + (6.034648e-04) * q[i3] * q[i7] * q[i12] + (-2.640983e-04) * q[i3] * q[i7] * q[i15] + (-3.420845e-04) * q[i3] * q[i7] * q[i16]
            + (-3.583018e-04) * q[i3] * q[i7] * q[i19] + (2.250667e-05) * q[i3] * q[i7] * q[i20] + (1.451721e-04) * q[i3] * q[i7] * q[i21]
            + (1.780640e-04) * q[i3] * q[i7] * q[i22] + (-9.315922e-05) * q[i3] * q[i8] * q[i9] + (4.078081e-04) * q[i3] * q[i8] * q[i10]
            + (-8.162157e-04) * q[i3] * q[i8] * q[i11] + (5.571022e-04) * q[i3] * q[i8] * q[i12] + (-5.793535e-04) * q[i3] * q[i8] * q[i15]
            + (-5.885023e-04) * q[i3] * q[i8] * q[i16] + (-2.736758e-04) * q[i3] * q[i8] * q[i19] + (-2.506133e-04) * q[i3] * q[i8] * q[i20]
            + (3.583714e-04) * q[i3] * q[i8] * q[i21] + (-3.718586e-05) * q[i3] * q[i8] * q[i22] + (-4.873547e-04) * q[i3] * q[i9] * q[i10]
            + (-1.778195e-04) * q[i3] * q[i9] * q[i11] + (-2.674895e-04) * q[i3] * q[i9] * q[i12] + (-2.490409e-04) * q[i3] * q[i9] * q[i15]
            + (-6.474482e-04) * q[i3] * q[i9] * q[i16] + (-2.530471e-04) * q[i3] * q[i9] * q[i19] + (5.058323e-05) * q[i3] * q[i9] * q[i20]
            + (1.621689e-04) * q[i3] * q[i9] * q[i21] + (-2.972290e-04) * q[i3] * q[i9] * q[i22] + (-1.247490e-04) * q[i3] * q[i10] * q[i11]
            + (3.478084e-04) * q[i3] * q[i10] * q[i12] + (1.607090e-04) * q[i3] * q[i10] * q[i15] + (1.095946e-04) * q[i3] * q[i10] * q[i16]
            + (6.130547e-05) * q[i3] * q[i10] * q[i19] + (1.567877e-04) * q[i3] * q[i10] * q[i20] + (-2.241620e-04) * q[i3] * q[i10] * q[i21]
            + (2.002785e-04) * q[i3] * q[i10] * q[i22] + (-1.237553e-05) * q[i3] * q[i11] * q[i12] + (-9.387109e-04) * q[i3] * q[i11] * q[i15]
            + (4.190322e-04) * q[i3] * q[i11] * q[i16] + (1.874694e-04) * q[i3] * q[i11] * q[i19] + (-1.119682e-04) * q[i3] * q[i11] * q[i20]
            + (-3.660009e-04) * q[i3] * q[i11] * q[i21] + (-9.408255e-05) * q[i3] * q[i11] * q[i22] + (5.437707e-04) * q[i3] * q[i12] * q[i15]
            + (-1.435395e-04) * q[i3] * q[i12] * q[i16] + (-1.542441e-04) * q[i3] * q[i12] * q[i19] + (3.180409e-04) * q[i3] * q[i12] * q[i20]
            + (-4.124738e-05) * q[i3] * q[i12] * q[i21] + (-2.224609e-04) * q[i3] * q[i12] * q[i22] + (2.630016e-04) * q[i3] * q[i15] * q[i16]
            + (-2.487628e-04) * q[i3] * q[i15] * q[i19] + (8.713109e-05) * q[i3] * q[i15] * q[i20] + (3.401230e-04) * q[i3] * q[i15] * q[i21]
            + (1.075916e-05) * q[i3] * q[i15] * q[i22] + (-2.486897e-05) * q[i3] * q[i16] * q[i19] + (1.707353e-04) * q[i3] * q[i16] * q[i20]
            + (-2.294937e-04) * q[i3] * q[i16] * q[i21] + (-6.333649e-04) * q[i3] * q[i16] * q[i22] + (2.075815e-05) * q[i3] * q[i19] * q[i20]
            + (3.289395e-04) * q[i3] * q[i19] * q[i21] + (-7.843576e-05) * q[i3] * q[i19] * q[i22] + (-1.167296e-04) * q[i3] * q[i20] * q[i21]
            + (-6.593001e-04) * q[i3] * q[i20] * q[i22] + (5.306985e-05) * q[i3] * q[i21] * q[i22] + (1.423295e-04) * q[i4] * q[i5] * q[i6]
            + (1.695603e-03) * q[i4] * q[i5] * q[i7] + (2.818257e-04) * q[i4] * q[i5] * q[i8] + (9.712108e-05) * q[i4] * q[i5] * q[i9]
            + (-2.354443e-04) * q[i4] * q[i5] * q[i10] + (6.574422e-04) * q[i4] * q[i5] * q[i11] + (-1.211321e-03) * q[i4] * q[i5] * q[i12]
            + (3.483358e-04) * q[i4] * q[i5] * q[i15] + (-8.730152e-04) * q[i4] * q[i5] * q[i16] + (6.492198e-04) * q[i4] * q[i5] * q[i19]
            + (-3.338799e-04) * q[i4] * q[i5] * q[i20] + (-1.473504e-05) * q[i4] * q[i5] * q[i21] + (3.263962e-04) * q[i4] * q[i5] * q[i22]
            + (-1.416828e-04) * q[i4] * q[i6] * q[i7] + (2.296334e-04) * q[i4] * q[i6] * q[i8] + (1.746406e-03) * q[i4] * q[i6] * q[i9]
            + (2.863477e-04) * q[i4] * q[i6] * q[i10] + (-5.990457e-04) * q[i4] * q[i6] * q[i11] + (3.041727e-04) * q[i4] * q[i6] * q[i12]
            + (-3.383068e-04) * q[i4] * q[i6] * q[i15] + (-2.718924e-04) * q[i4] * q[i6] * q[i16] + (2.185914e-05) * q[i4] * q[i6] * q[i19]
            + (-3.518379e-04) * q[i4] * q[i6] * q[i20] + (-1.800077e-04) * q[i4] * q[i6] * q[i21] + (-1.451111e-04) * q[i4] * q[i6] * q[i22]
            + (-1.525328e-03) * q[i4] * q[i7] * q[i8] + (7.122502e-04) * q[i4] * q[i7] * q[i9] + (-1.058271e-02) * q[i4] * q[i7] * q[i10]
            + (2.784186e-05) * q[i4] * q[i7] * q[i11] + (2.025499e-04) * q[i4] * q[i7] * q[i12] + (-9.273298e-04) * q[i4] * q[i7] * q[i15]
            + (2.176679e-04) * q[i4] * q[i7] * q[i16] + (5.446240e-04) * q[i4] * q[i7] * q[i19] + (-5.295054e-04) * q[i4] * q[i7] * q[i20]
            + (-4.123952e-04) * q[i4] * q[i7] * q[i21] + (4.186780e-06) * q[i4] * q[i7] * q[i22] + (4.080237e-04) * q[i4] * q[i8] * q[i9]
            + (-9.400196e-05) * q[i4] * q[i8] * q[i10] + (-5.216702e-04) * q[i4] * q[i8] * q[i11] + (8.343474e-04) * q[i4] * q[i8] * q[i12]
            + (-5.722809e-04) * q[i4] * q[i8] * q[i15] + (-5.842616e-04) * q[i4] * q[i8] * q[i16] + (-2.530987e-04) * q[i4] * q[i8] * q[i19]
            + (-2.619990e-04) * q[i4] * q[i8] * q[i20] + (3.014335e-05) * q[i4] * q[i8] * q[i21] + (-3.593076e-04) * q[i4] * q[i8] * q[i22]
            + (-4.884575e-04) * q[i4] * q[i9] * q[i10] + (-3.400631e-04) * q[i4] * q[i9] * q[i11] + (1.283953e-04) * q[i4] * q[i9] * q[i12]
            + (1.065911e-04) * q[i4] * q[i9] * q[i15] + (1.631917e-04) * q[i4] * q[i9] * q[i16] + (1.529120e-04) * q[i4] * q[i9] * q[i19]
            + (6.564241e-05) * q[i4] * q[i9] * q[i20] + (-2.000379e-04) * q[i4] * q[i9] * q[i21] + (2.217890e-04) * q[i4] * q[i9] * q[i22]
            + (2.572872e-04) * q[i4] * q[i10] * q[i11] + (1.697263e-04) * q[i4] * q[i10] * q[i12] + (-6.359112e-04) * q[i4] * q[i10] * q[i15]
            + (-2.492120e-04) * q[i4] * q[i10] * q[i16] + (4.953959e-05) * q[i4] * q[i10] * q[i19] + (-2.550075e-04) * q[i4] * q[i10] * q[i20]
            + (2.937772e-04) * q[i4] * q[i10] * q[i21] + (-1.621648e-04) * q[i4] * q[i10] * q[i22] + (-8.308391e-06) * q[i4] * q[i11] * q[i12]
            + (1.431981e-04) * q[i4] * q[i11] * q[i15] + (-5.401505e-04) * q[i4] * q[i11] * q[i16] + (-3.156915e-04) * q[i4] * q[i11] * q[i19]
            + (1.557434e-04) * q[i4] * q[i11] * q[i20] + (-2.194707e-04) * q[i4] * q[i11] * q[i21] + (-3.941864e-05) * q[i4] * q[i11] * q[i22]
            + (-4.132159e-04) * q[i4] * q[i12] * q[i15] + (9.410504e-04) * q[i4] * q[i12] * q[i16] + (1.068496e-04) * q[i4] * q[i12] * q[i19]
            + (-1.928976e-04) * q[i4] * q[i12] * q[i20] + (-9.924322e-05) * q[i4] * q[i12] * q[i21] + (-3.666046e-04) * q[i4] * q[i12] * q[i22]
            + (2.579163e-04) * q[i4] * q[i15] * q[i16] + (1.673949e-04) * q[i4] * q[i15] * q[i19] + (-2.553386e-05) * q[i4] * q[i15] * q[i20]
            + (6.246916e-04) * q[i4] * q[i15] * q[i21] + (2.248961e-04) * q[i4] * q[i15] * q[i22] + (8.093611e-05) * q[i4] * q[i16] * q[i19]
            + (-2.527976e-04) * q[i4] * q[i16] * q[i20] + (-9.233878e-06) * q[i4] * q[i16] * q[i21] + (-3.416751e-04) * q[i4] * q[i16] * q[i22]
            + (2.110874e-05) * q[i4] * q[i19] * q[i20] + (6.592995e-04) * q[i4] * q[i19] * q[i21] + (1.175819e-04) * q[i4] * q[i19] * q[i22]
            + (7.482038e-05) * q[i4] * q[i20] * q[i21] + (-3.308207e-04) * q[i4] * q[i20] * q[i22] + (5.319872e-05) * q[i4] * q[i21] * q[i22]
            + (1.751704e-04) * q[i5] * q[i6] * q[i7] + (7.352272e-05) * q[i5] * q[i6] * q[i8] + (2.487281e-03) * q[i5] * q[i6] * q[i9]
            + (-1.403109e-03) * q[i5] * q[i6] * q[i10] + (-2.555128e-04) * q[i5] * q[i6] * q[i11] + (-5.092444e-04) * q[i5] * q[i6] * q[i12]
            + (-1.719568e-05) * q[i5] * q[i6] * q[i15] + (1.081003e-04) * q[i5] * q[i6] * q[i16] + (1.491224e-04) * q[i5] * q[i6] * q[i19]
            + (6.341876e-04) * q[i5] * q[i6] * q[i20] + (-3.825242e-04) * q[i5] * q[i6] * q[i21] + (-2.843814e-04) * q[i5] * q[i6] * q[i22]
            + (7.377251e-05) * q[i5] * q[i7] * q[i8] + (-1.413646e-03) * q[i5] * q[i7] * q[i9] + (2.477301e-03) * q[i5] * q[i7] * q[i10]
            + (5.000242e-04) * q[i5] * q[i7] * q[i11] + (2.595822e-04) * q[i5] * q[i7] * q[i12] + (1.080666e-04) * q[i5] * q[i7] * q[i15]
            + (-2.874128e-05) * q[i5] * q[i7] * q[i16] + (6.361853e-04) * q[i5] * q[i7] * q[i19] + (1.472312e-04) * q[i5] * q[i7] * q[i20]
            + (2.908408e-04) * q[i5] * q[i7] * q[i21] + (3.741623e-04) * q[i5] * q[i7] * q[i22] + (-1.799927e-03) * q[i5] * q[i8] * q[i9]
            + (-1.784532e-03) * q[i5] * q[i8] * q[i10] + (2.327999e-03) * q[i5] * q[i8] * q[i11] + (-2.406813e-03) * q[i5] * q[i8] * q[i12]
            + (2.348362e-03) * q[i5] * q[i8] * q[i15] + (2.398485e-03) * q[i5] * q[i8] * q[i16] + (5.959096e-04) * q[i5] * q[i8] * q[i19]
            + (5.773746e-04) * q[i5] * q[i8] * q[i20] + (1.498457e-04) * q[i5] * q[i8] * q[i21] + (-1.440720e-04) * q[i5] * q[i8] * q[i22]
            + (-2.438288e-06) * q[i5] * q[i9] * q[i10] + (8.956663e-05) * q[i5] * q[i9] * q[i11] + (-2.005148e-04) * q[i5] * q[i9] * q[i12]
            + (-3.390928e-04) * q[i5] * q[i9] * q[i15] + (-2.962105e-04) * q[i5] * q[i9] * q[i16] + (3.594022e-04) * q[i5] * q[i9] * q[i19]
            + (4.230960e-05) * q[i5] * q[i9] * q[i20] + (4.788983e-05) * q[i5] * q[i9] * q[i21] + (1.136154e-06) * q[i5] * q[i9] * q[i22]
            + (1.925860e-04) * q[i5] * q[i10] * q[i11] + (-9.604946e-05) * q[i5] * q[i10] * q[i12] + (-2.919496e-04) * q[i5] * q[i10] * q[i15]
            + (-3.412088e-04) * q[i5] * q[i10] * q[i16] + (4.502975e-05) * q[i5] * q[i10] * q[i19] + (3.584510e-04) * q[i5] * q[i10] * q[i20]
            + (1.194787e-06) * q[i5] * q[i10] * q[i21] + (-5.287167e-05) * q[i5] * q[i10] * q[i22] + (4.851027e-05) * q[i5] * q[i11] * q[i12]
            + (1.888604e-03) * q[i5] * q[i11] * q[i15] + (4.896294e-05) * q[i5] * q[i11] * q[i16] + (8.125725e-04) * q[i5] * q[i11] * q[i19]
            + (-3.515285e-04) * q[i5] * q[i11] * q[i20] + (7.588162e-04) * q[i5] * q[i11] * q[i21] + (-1.558879e-04) * q[i5] * q[i11] * q[i22]
            + (-6.515054e-05) * q[i5] * q[i12] * q[i15] + (-1.907801e-03) * q[i5] * q[i12] * q[i16] + (3.512644e-04) * q[i5] * q[i12] * q[i19]
            + (-8.135706e-04) * q[i5] * q[i12] * q[i20] + (-1.486958e-04) * q[i5] * q[i12] * q[i21] + (7.760778e-04) * q[i5] * q[i12] * q[i22]
            + (-4.701687e-04) * q[i5] * q[i15] * q[i16] + (-1.713542e-04) * q[i5] * q[i15] * q[i19] + (-2.484216e-04) * q[i5] * q[i15] * q[i20]
            + (-1.837282e-03) * q[i5] * q[i15] * q[i21] + (-1.835453e-04) * q[i5] * q[i15] * q[i22] + (-2.437812e-04) * q[i5] * q[i16] * q[i19]
            + (-1.624382e-04) * q[i5] * q[i16] * q[i20] + (1.821656e-04) * q[i5] * q[i16] * q[i21] + (1.859405e-03) * q[i5] * q[i16] * q[i22]
            + (6.759754e-04) * q[i5] * q[i19] * q[i20] + (-1.579017e-03) * q[i5] * q[i19] * q[i21] + (-1.745701e-04) * q[i5] * q[i19] * q[i22]
            + (1.759727e-04) * q[i5] * q[i20] * q[i21] + (1.582590e-03) * q[i5] * q[i20] * q[i22] + (-4.213042e-04) * q[i5] * q[i21] * q[i22]
            + (3.050427e-06) * q[i6] * q[i7] * q[i8] + (8.933321e-04) * q[i6] * q[i7] * q[i9] + (-8.729645e-04) * q[i6] * q[i7] * q[i10]
            + (9.157569e-04) * q[i6] * q[i7] * q[i11] + (9.096246e-04) * q[i6] * q[i7] * q[i12] + (-2.486836e-04) * q[i6] * q[i7] * q[i15]
            + (2.563843e-04) * q[i6] * q[i7] * q[i16] + (4.420799e-04) * q[i6] * q[i7] * q[i19] + (-4.454843e-04) * q[i6] * q[i7] * q[i20]
            + (-1.594664e-05) * q[i6] * q[i7] * q[i21] + (-1.502534e-05) * q[i6] * q[i7] * q[i22] + (8.946651e-04) * q[i6] * q[i8] * q[i9]
            + (7.445128e-05) * q[i6] * q[i8] * q[i10] + (-3.951089e-04) * q[i6] * q[i8] * q[i11] + (-6.113200e-04) * q[i6] * q[i8] * q[i12]
            + (4.453109e-04) * q[i6] * q[i8] * q[i15] + (6.577481e-05) * q[i6] * q[i8] * q[i16] + (-3.950201e-05) * q[i6] * q[i8] * q[i19]
            + (4.931194e-04) * q[i6] * q[i8] * q[i20] + (-1.592634e-04) * q[i6] * q[i8] * q[i21] + (3.601620e-04) * q[i6] * q[i8] * q[i22]
            + (5.360598e-04) * q[i6] * q[i9] * q[i10] + (-1.914007e-04) * q[i6] * q[i9] * q[i11] + (-4.993365e-04) * q[i6] * q[i9] * q[i12]
            + (-1.714614e-04) * q[i6] * q[i9] * q[i15] + (3.639200e-04) * q[i6] * q[i9] * q[i16] + (-1.812739e-04) * q[i6] * q[i9] * q[i19]
            + (2.937345e-05) * q[i6] * q[i9] * q[i20] + (8.760108e-06) * q[i6] * q[i9] * q[i21] + (-1.099680e-04) * q[i6] * q[i9] * q[i22]
            + (2.801582e-04) * q[i6] * q[i10] * q[i11] + (2.759020e-04) * q[i6] * q[i10] * q[i12] + (-4.925413e-04) * q[i6] * q[i10] * q[i15]
            + (-1.564324e-04) * q[i6] * q[i10] * q[i16] + (-7.371924e-06) * q[i6] * q[i10] * q[i19] + (-3.897913e-05) * q[i6] * q[i10] * q[i20]
            + (8.674091e-05) * q[i6] * q[i10] * q[i21] + (1.694297e-04) * q[i6] * q[i10] * q[i22] + (3.259319e-04) * q[i6] * q[i11] * q[i12]
            + (8.257468e-04) * q[i6] * q[i11] * q[i15] + (8.527838e-05) * q[i6] * q[i11] * q[i16] + (5.202263e-05) * q[i6] * q[i11] * q[i19]
            + (-4.364655e-05) * q[i6] * q[i11] * q[i20] + (3.742519e-04) * q[i6] * q[i11] * q[i21] + (1.222552e-04) * q[i6] * q[i11] * q[i22]
            + (4.345391e-04) * q[i6] * q[i12] * q[i15] + (9.717923e-04) * q[i6] * q[i12] * q[i16] + (-6.807007e-05) * q[i6] * q[i12] * q[i19]
            + (5.282220e-05) * q[i6] * q[i12] * q[i20] + (-1.138935e-04) * q[i6] * q[i12] * q[i21] + (-1.274102e-04) * q[i6] * q[i12] * q[i22]
            + (-3.368392e-04) * q[i6] * q[i15] * q[i16] + (2.110082e-04) * q[i6] * q[i15] * q[i19] + (1.025149e-05) * q[i6] * q[i15] * q[i20]
            + (-4.249887e-05) * q[i6] * q[i15] * q[i21] + (-3.835844e-05) * q[i6] * q[i15] * q[i22] + (-2.116556e-04) * q[i6] * q[i16] * q[i19]
            + (-2.076262e-04) * q[i6] * q[i16] * q[i20] + (-1.229254e-04) * q[i6] * q[i16] * q[i21] + (1.434616e-04) * q[i6] * q[i16] * q[i22]
            + (5.515648e-05) * q[i6] * q[i19] * q[i20] + (-1.667539e-04) * q[i6] * q[i19] * q[i21] + (2.299836e-05) * q[i6] * q[i19] * q[i22]
            + (2.634986e-06) * q[i6] * q[i20] * q[i21] + (-3.036432e-04) * q[i6] * q[i20] * q[i22] + (-2.140628e-05) * q[i6] * q[i21] * q[i22]
            + (-7.573144e-05) * q[i7] * q[i8] * q[i9] + (-8.841607e-04) * q[i7] * q[i8] * q[i10] + (-5.749266e-04) * q[i7] * q[i8] * q[i11]
            + (-3.858998e-04) * q[i7] * q[i8] * q[i12] + (-6.792527e-05) * q[i7] * q[i8] * q[i15] + (-4.314481e-04) * q[i7] * q[i8] * q[i16]
            + (-4.847388e-04) * q[i7] * q[i8] * q[i19] + (3.692607e-05) * q[i7] * q[i8] * q[i20] + (3.624944e-04) * q[i7] * q[i8] * q[i21]
            + (-1.662362e-04) * q[i7] * q[i8] * q[i22] + (-5.349943e-04) * q[i7] * q[i9] * q[i10] + (2.795555e-04) * q[i7] * q[i9] * q[i11]
            + (2.772482e-04) * q[i7] * q[i9] * q[i12] + (1.556296e-04) * q[i7] * q[i9] * q[i15] + (4.976830e-04) * q[i7] * q[i9] * q[i16]
            + (3.774569e-05) * q[i7] * q[i9] * q[i19] + (4.779438e-06) * q[i7] * q[i9] * q[i20] + (1.699146e-04) * q[i7] * q[i9] * q[i21]
            + (8.824007e-05) * q[i7] * q[i9] * q[i22] + (-4.982814e-04) * q[i7] * q[i10] * q[i11] + (-1.927251e-04) * q[i7] * q[i10] * q[i12]
            + (-3.596411e-04) * q[i7] * q[i10] * q[i15] + (1.720243e-04) * q[i7] * q[i10] * q[i16] + (-3.155922e-05) * q[i7] * q[i10] * q[i19]
            + (1.732689e-04) * q[i7] * q[i10] * q[i20] + (-1.111959e-04) * q[i7] * q[i10] * q[i21] + (8.252590e-06) * q[i7] * q[i10] * q[i22]
            + (-3.273813e-04) * q[i7] * q[i11] * q[i12] + (9.646963e-04) * q[i7] * q[i11] * q[i15] + (4.343167e-04) * q[i7] * q[i11] * q[i16]
            + (5.168748e-05) * q[i7] * q[i11] * q[i19] + (-7.278678e-05) * q[i7] * q[i11] * q[i20] + (1.227451e-04) * q[i7] * q[i11] * q[i21]
            + (1.142882e-04) * q[i7] * q[i11] * q[i22] + (8.093829e-05) * q[i7] * q[i12] * q[i15] + (8.227340e-04) * q[i7] * q[i12] * q[i16]
            + (-4.373682e-05) * q[i7] * q[i12] * q[i19] + (5.232382e-05) * q[i7] * q[i12] * q[i20] + (-1.195996e-04) * q[i7] * q[i12] * q[i21]
            + (-3.711772e-04) * q[i7] * q[i12] * q[i22] + (3.378316e-04) * q[i7] * q[i15] * q[i16] + (2.100835e-04) * q[i7] * q[i15] * q[i19]
            + (2.107295e-04) * q[i7] * q[i15] * q[i20] + (1.401101e-04) * q[i7] * q[i15] * q[i21] + (-1.217494e-04) * q[i7] * q[i15] * q[i22]
            + (-9.969937e-06) * q[i7] * q[i16] * q[i19] + (-2.072429e-04) * q[i7] * q[i16] * q[i20] + (-3.814879e-05) * q[i7] * q[i16] * q[i21]
            + (-4.905897e-05) * q[i7] * q[i16] * q[i22] + (-5.782290e-05) * q[i7] * q[i19] * q[i20] + (-2.977827e-04) * q[i7] * q[i19] * q[i21]
            + (5.188520e-06) * q[i7] * q[i19] * q[i22] + (2.271721e-05) * q[i7] * q[i20] * q[i21] + (-1.667069e-04) * q[i7] * q[i20] * q[i22]
            + (2.078964e-05) * q[i7] * q[i21] * q[i22] + (-8.093102e-07) * q[i8] * q[i9] * q[i10] + (4.649929e-04) * q[i8] * q[i9] * q[i11]
            + (7.230965e-04) * q[i8] * q[i9] * q[i12] + (-2.590363e-04) * q[i8] * q[i9] * q[i15] + (7.364581e-05) * q[i8] * q[i9] * q[i16]
            + (6.685977e-05) * q[i8] * q[i9] * q[i19] + (-3.435974e-05) * q[i8] * q[i9] * q[i20] + (3.378134e-04) * q[i8] * q[i9] * q[i21]
            + (-2.450494e-04) * q[i8] * q[i9] * q[i22] + (7.121912e-04) * q[i8] * q[i10] * q[i11] + (4.641616e-04) * q[i8] * q[i10] * q[i12]
            + (-7.107874e-05) * q[i8] * q[i10] * q[i15] + (2.570242e-04) * q[i8] * q[i10] * q[i16] + (4.027034e-05) * q[i8] * q[i10] * q[i19]
            + (-6.536552e-05) * q[i8] * q[i10] * q[i20] + (-2.449227e-04) * q[i8] * q[i10] * q[i21] + (3.374044e-04) * q[i8] * q[i10] * q[i22]
            + (-3.659516e-06) * q[i8] * q[i11] * q[i12] + (3.760734e-03) * q[i8] * q[i11] * q[i15] + (1.553745e-04) * q[i8] * q[i11] * q[i16]
            + (3.075091e-04) * q[i8] * q[i11] * q[i19] + (4.150482e-05) * q[i8] * q[i11] * q[i20] + (1.772542e-03) * q[i8] * q[i11] * q[i21]
            + (8.801020e-05) * q[i8] * q[i11] * q[i22] + (1.540005e-04) * q[i8] * q[i12] * q[i15] + (3.777960e-03) * q[i8] * q[i12] * q[i16]
            + (4.182337e-05) * q[i8] * q[i12] * q[i19] + (2.916899e-04) * q[i8] * q[i12] * q[i20] + (-8.636645e-05) * q[i8] * q[i12] * q[i21]
            + (-1.784999e-03) * q[i8] * q[i12] * q[i22] + (6.105476e-07) * q[i8] * q[i15] * q[i16] + (3.855031e-04) * q[i8] * q[i15] * q[i19]
            + (-1.373722e-04) * q[i8] * q[i15] * q[i20] + (4.243812e-04) * q[i8] * q[i15] * q[i21] + (1.451907e-04) * q[i8] * q[i15] * q[i22]
            + (1.402156e-04) * q[i8] * q[i16] * q[i19] + (-3.603188e-04) * q[i8] * q[i16] * q[i20] + (1.460470e-04) * q[i8] * q[i16] * q[i21]
            + (4.367203e-04) * q[i8] * q[i16] * q[i22] + (-6.143433e-07) * q[i8] * q[i19] * q[i20] + (-8.813338e-04) * q[i8] * q[i19] * q[i21]
            + (1.426832e-05) * q[i8] * q[i19] * q[i22] + (1.982403e-05) * q[i8] * q[i20] * q[i21] + (-8.678889e-04) * q[i8] * q[i20] * q[i22]
            + (1.326477e-07) * q[i8] * q[i21] * q[i22] + (8.753649e-05) * q[i9] * q[i10] * q[i11] + (8.681458e-05) * q[i9] * q[i10] * q[i12]
            + (3.084963e-05) * q[i9] * q[i10] * q[i15] + (-3.222391e-05) * q[i9] * q[i10] * q[i16] + (-2.000294e-05) * q[i9] * q[i10] * q[i19]
            + (1.999417e-05) * q[i9] * q[i10] * q[i20] + (7.364560e-05) * q[i9] * q[i10] * q[i21] + (7.426748e-05) * q[i9] * q[i10] * q[i22]
            + (-5.540778e-05) * q[i9] * q[i11] * q[i12] + (-3.124145e-04) * q[i9] * q[i11] * q[i15] + (-2.319001e-05) * q[i9] * q[i11] * q[i16]
            + (1.114000e-05) * q[i9] * q[i11] * q[i19] + (-3.336551e-05) * q[i9] * q[i11] * q[i20] + (1.422593e-04) * q[i9] * q[i11] * q[i21]
            + (-7.166301e-05) * q[i9] * q[i11] * q[i22] + (-1.269955e-04) * q[i9] * q[i12] * q[i15] + (-1.029408e-04) * q[i9] * q[i12] * q[i16]
            + (-7.128775e-05) * q[i9] * q[i12] * q[i19] + (1.669239e-04) * q[i9] * q[i12] * q[i20] + (2.485172e-05) * q[i9] * q[i12] * q[i21]
            + (1.425158e-04) * q[i9] * q[i12] * q[i22] + (6.452166e-05) * q[i9] * q[i15] * q[i16] + (9.578845e-05) * q[i9] * q[i15] * q[i19]
            + (4.479746e-05) * q[i9] * q[i15] * q[i20] + (5.512411e-05) * q[i9] * q[i15] * q[i21] + (-1.941762e-05) * q[i9] * q[i15] * q[i22]
            + (-9.599487e-05) * q[i9] * q[i16] * q[i19] + (-6.383294e-05) * q[i9] * q[i16] * q[i20] + (-3.070896e-05) * q[i9] * q[i16] * q[i21]
            + (-3.057005e-05) * q[i9] * q[i16] * q[i22] + (1.254037e-04) * q[i9] * q[i19] * q[i20] + (6.051227e-05) * q[i9] * q[i19] * q[i21]
            + (7.515042e-05) * q[i9] * q[i19] * q[i22] + (-2.057594e-05) * q[i9] * q[i20] * q[i21] + (-1.112748e-04) * q[i9] * q[i20] * q[i22]
            + (-2.941916e-06) * q[i9] * q[i21] * q[i22] + (5.493271e-05) * q[i10] * q[i11] * q[i12] + (-1.025170e-04) * q[i10] * q[i11] * q[i15]
            + (-1.252333e-04) * q[i10] * q[i11] * q[i16] + (1.704197e-04) * q[i10] * q[i11] * q[i19] + (-7.218964e-05) * q[i10] * q[i11] * q[i20]
            + (-1.410711e-04) * q[i10] * q[i11] * q[i21] + (-2.435788e-05) * q[i10] * q[i11] * q[i22] + (-2.486485e-05) * q[i10] * q[i12] * q[i15]
            + (-3.109391e-04) * q[i10] * q[i12] * q[i16] + (-3.370367e-05) * q[i10] * q[i12] * q[i19] + (1.260359e-05) * q[i10] * q[i12] * q[i20]
            + (7.388342e-05) * q[i10] * q[i12] * q[i21] + (-1.428886e-04) * q[i10] * q[i12] * q[i22] + (-6.514242e-05) * q[i10] * q[i15] * q[i16]
            + (6.426175e-05) * q[i10] * q[i15] * q[i19] + (9.496067e-05) * q[i10] * q[i15] * q[i20] + (-2.781572e-05) * q[i10] * q[i15] * q[i21]
            + (-2.911934e-05) * q[i10] * q[i15] * q[i22] + (-4.382993e-05) * q[i10] * q[i16] * q[i19] + (-9.594302e-05) * q[i10] * q[i16] * q[i20]
            + (-1.803125e-05) * q[i10] * q[i16] * q[i21] + (5.298471e-05) * q[i10] * q[i16] * q[i22] + (-1.253923e-04) * q[i10] * q[i19] * q[i20]
            + (-1.100553e-04) * q[i10] * q[i19] * q[i21] + (-2.068554e-05) * q[i10] * q[i19] * q[i22] + (7.477975e-05) * q[i10] * q[i20] * q[i21]
            + (5.951271e-05) * q[i10] * q[i20] * q[i22] + (2.709701e-06) * q[i10] * q[i21] * q[i22] + (1.944444e-04) * q[i11] * q[i12] * q[i15]
            + (-1.939213e-04) * q[i11] * q[i12] * q[i16] + (3.515086e-04) * q[i11] * q[i12] * q[i19] + (-3.487330e-04) * q[i11] * q[i12] * q[i20]
            + (-1.346398e-04) * q[i11] * q[i12] * q[i21] + (-1.364356e-04) * q[i11] * q[i12] * q[i22] + (-2.880563e-04) * q[i11] * q[i15] * q[i16]
            + (-2.796700e-04) * q[i11] * q[i15] * q[i19] + (-8.792055e-05) * q[i11] * q[i15] * q[i20] + (-5.240005e-04) * q[i11] * q[i15] * q[i21]
            + (3.849008e-05) * q[i11] * q[i15] * q[i22] + (1.741379e-06) * q[i11] * q[i16] * q[i19] + (2.023173e-04) * q[i11] * q[i16] * q[i20]
            + (-1.639818e-04) * q[i11] * q[i16] * q[i21] + (-1.462666e-04) * q[i11] * q[i16] * q[i22] + (4.498088e-05) * q[i11] * q[i19] * q[i20]
            + (-1.951025e-03) * q[i11] * q[i19] * q[i21] + (6.040702e-05) * q[i11] * q[i19] * q[i22] + (-9.114553e-05) * q[i11] * q[i20] * q[i21]
            + (-3.234788e-05) * q[i11] * q[i20] * q[i22] + (2.329214e-05) * q[i11] * q[i21] * q[i22] + (-2.832443e-04) * q[i12] * q[i15] * q[i16]
            + (2.032436e-04) * q[i12] * q[i15] * q[i19] + (-7.861044e-07) * q[i12] * q[i15] * q[i20] + (1.456073e-04) * q[i12] * q[i15] * q[i21]
            + (1.656489e-04) * q[i12] * q[i15] * q[i22] + (-8.642033e-05) * q[i12] * q[i16] * q[i19] + (-2.879309e-04) * q[i12] * q[i16] * q[i20]
            + (-3.884873e-05) * q[i12] * q[i16] * q[i21] + (5.267210e-04) * q[i12] * q[i16] * q[i22] + (4.564215e-05) * q[i12] * q[i19] * q[i20]
            + (3.084258e-05) * q[i12] * q[i19] * q[i21] + (9.309488e-05) * q[i12] * q[i19] * q[i22] + (-6.337097e-05) * q[i12] * q[i20] * q[i21]
            + (1.943681e-03) * q[i12] * q[i20] * q[i22] + (2.400665e-05) * q[i12] * q[i21] * q[i22] + (3.736643e-06) * q[i15] * q[i16] * q[i19]
            + (-4.263416e-06) * q[i15] * q[i16] * q[i20] + (5.189304e-05) * q[i15] * q[i16] * q[i21] + (5.324950e-05) * q[i15] * q[i16] * q[i22]
            + (-5.533933e-05) * q[i15] * q[i19] * q[i20] + (2.051240e-04) * q[i15] * q[i19] * q[i21] + (-1.997200e-05) * q[i15] * q[i19] * q[i22]
            + (-3.264947e-05) * q[i15] * q[i20] * q[i21] + (4.061760e-05) * q[i15] * q[i20] * q[i22] + (4.119159e-05) * q[i15] * q[i21] * q[i22]
            + (5.549695e-05) * q[i16] * q[i19] * q[i20] + (3.903370e-05) * q[i16] * q[i19] * q[i21] + (-3.183414e-05) * q[i16] * q[i19] * q[i22]
            + (-1.827554e-05) * q[i16] * q[i20] * q[i21] + (2.108443e-04) * q[i16] * q[i20] * q[i22] + (-4.087610e-05) * q[i16] * q[i21] * q[i22]
            + (1.138070e-04) * q[i19] * q[i20] * q[i21] + (1.152920e-04) * q[i19] * q[i20] * q[i22] + (-2.884140e-05) * q[i19] * q[i21] * q[i22]
            + (2.995648e-05) * q[i20] * q[i21] * q[i22];

      return Qx;
   }

   public double getQy(double[] q)
   {
      double Qy;
      Qy = (-5.017620e-03) * q[i0] + (5.085403e-03) * q[i1] + (1.317187e-04) * q[i2] + (6.817190e-04) * q[i3] + (-7.032224e-04) * q[i4]
            + (-2.487388e-05) * q[i5] + (1.310088e-01) * q[i6] + (1.302818e-01) * q[i7] + (1.106335e-01) * q[i8] + (5.753591e-02) * q[i9]
            + (5.691600e-02) * q[i10] + (-6.730134e-03) * q[i11] + (7.177252e-03) * q[i12] + (6.601284e-03) * q[i15] + (6.513104e-03) * q[i16]
            + (-8.200116e-04) * q[i19] + (-7.776449e-04) * q[i20] + (3.284135e-03) * q[i21] + (-3.288472e-03) * q[i22] + (2.299577e-03) * q[i0] * q[i0]
            + (2.279791e-03) * q[i1] * q[i1] + (-1.288685e-03) * q[i2] * q[i2] + (3.154545e-04) * q[i3] * q[i3] + (3.037401e-04) * q[i4] * q[i4]
            + (-1.974648e-03) * q[i5] * q[i5] + (-4.170347e-04) * q[i6] * q[i6] + (-4.007243e-04) * q[i7] * q[i7] + (1.743072e-03) * q[i8] * q[i8]
            + (-3.601163e-03) * q[i9] * q[i9] + (-3.588062e-03) * q[i10] * q[i10] + (-3.357970e-03) * q[i11] * q[i11] + (-3.266432e-03) * q[i12] * q[i12]
            + (-1.411100e-03) * q[i15] * q[i15] + (-1.437953e-03) * q[i16] * q[i16] + (-2.374948e-04) * q[i19] * q[i19] + (-2.294355e-04) * q[i20] * q[i20]
            + (-7.242032e-04) * q[i21] * q[i21] + (-7.308113e-04) * q[i22] * q[i22] + (-1.639256e-04) * q[i0] * q[i1] + (6.040509e-05) * q[i0] * q[i2]
            + (-1.050023e-01) * q[i0] * q[i3] + (1.019550e-02) * q[i0] * q[i4] + (2.311095e-03) * q[i0] * q[i5] + (5.812794e-03) * q[i0] * q[i6]
            + (1.397899e-03) * q[i0] * q[i7] + (-8.777736e-04) * q[i0] * q[i8] + (8.871379e-04) * q[i0] * q[i9] + (8.983805e-04) * q[i0] * q[i10]
            + (-1.743768e-03) * q[i0] * q[i11] + (-4.475553e-04) * q[i0] * q[i12] + (3.125782e-03) * q[i0] * q[i15] + (-3.367573e-03) * q[i0] * q[i16]
            + (2.667678e-04) * q[i0] * q[i19] + (-8.810999e-04) * q[i0] * q[i20] + (6.470544e-04) * q[i0] * q[i21] + (6.914933e-04) * q[i0] * q[i22]
            + (5.298778e-05) * q[i1] * q[i2] + (1.020397e-02) * q[i1] * q[i3] + (-1.045263e-01) * q[i1] * q[i4] + (2.323533e-03) * q[i1] * q[i5]
            + (-1.376621e-03) * q[i1] * q[i6] + (-5.877031e-03) * q[i1] * q[i7] + (8.841392e-04) * q[i1] * q[i8] + (-8.937913e-04) * q[i1] * q[i9]
            + (-8.990074e-04) * q[i1] * q[i10] + (-4.216920e-04) * q[i1] * q[i11] + (-1.746531e-03) * q[i1] * q[i12] + (3.317269e-03) * q[i1] * q[i15]
            + (-3.137907e-03) * q[i1] * q[i16] + (8.714941e-04) * q[i1] * q[i19] + (-2.465857e-04) * q[i1] * q[i20] + (6.795261e-04) * q[i1] * q[i21]
            + (6.412982e-04) * q[i1] * q[i22] + (-2.072470e-02) * q[i2] * q[i3] + (-2.060847e-02) * q[i2] * q[i4] + (7.256947e-02) * q[i2] * q[i5]
            + (-6.440818e-04) * q[i2] * q[i6] + (6.527983e-04) * q[i2] * q[i7] + (-2.594993e-05) * q[i2] * q[i8] + (2.799744e-03) * q[i2] * q[i9]
            + (-2.752438e-03) * q[i2] * q[i10] + (-2.770685e-03) * q[i2] * q[i11] + (-2.708838e-03) * q[i2] * q[i12] + (6.361470e-03) * q[i2] * q[i15]
            + (-6.473874e-03) * q[i2] * q[i16] + (8.740099e-05) * q[i2] * q[i19] + (-9.883928e-05) * q[i2] * q[i20] + (4.279692e-04) * q[i2] * q[i21]
            + (4.093939e-04) * q[i2] * q[i22] + (-2.626730e-03) * q[i3] * q[i4] + (-7.588072e-04) * q[i3] * q[i5] + (-1.294811e-02) * q[i3] * q[i6]
            + (1.874437e-02) * q[i3] * q[i7] + (1.536486e-03) * q[i3] * q[i8] + (-3.155387e-03) * q[i3] * q[i9] + (5.113675e-03) * q[i3] * q[i10]
            + (-3.877735e-03) * q[i3] * q[i11] + (-5.651667e-03) * q[i3] * q[i12] + (-1.862611e-03) * q[i3] * q[i15] + (2.498941e-03) * q[i3] * q[i16]
            + (-1.422479e-03) * q[i3] * q[i19] + (3.632669e-04) * q[i3] * q[i20] + (-8.891952e-04) * q[i3] * q[i21] + (-2.091769e-04) * q[i3] * q[i22]
            + (-7.867142e-04) * q[i4] * q[i5] + (-1.874132e-02) * q[i4] * q[i6] + (1.288080e-02) * q[i4] * q[i7] + (-1.477931e-03) * q[i4] * q[i8]
            + (-5.158458e-03) * q[i4] * q[i9] + (3.126489e-03) * q[i4] * q[i10] + (-5.590388e-03) * q[i4] * q[i11] + (-3.875277e-03) * q[i4] * q[i12]
            + (-2.459107e-03) * q[i4] * q[i15] + (1.907826e-03) * q[i4] * q[i16] + (-3.897407e-04) * q[i4] * q[i19] + (1.405618e-03) * q[i4] * q[i20]
            + (-2.048008e-04) * q[i4] * q[i21] + (-8.828745e-04) * q[i4] * q[i22] + (-1.212218e-02) * q[i5] * q[i6] + (1.210702e-02) * q[i5] * q[i7]
            + (6.977615e-06) * q[i5] * q[i8] + (-4.796828e-03) * q[i5] * q[i9] + (4.750550e-03) * q[i5] * q[i10] + (-4.371294e-03) * q[i5] * q[i11]
            + (-4.297983e-03) * q[i5] * q[i12] + (2.815028e-03) * q[i5] * q[i15] + (-2.854398e-03) * q[i5] * q[i16] + (-5.138955e-04) * q[i5] * q[i19]
            + (4.998270e-04) * q[i5] * q[i20] + (1.153886e-03) * q[i5] * q[i21] + (1.139228e-03) * q[i5] * q[i22] + (-5.955809e-03) * q[i6] * q[i7]
            + (-3.563051e-05) * q[i6] * q[i8] + (-9.249520e-03) * q[i6] * q[i9] + (4.391855e-03) * q[i6] * q[i10] + (-7.010548e-04) * q[i6] * q[i11]
            + (8.597632e-04) * q[i6] * q[i12] + (2.545774e-03) * q[i6] * q[i15] + (6.446622e-03) * q[i6] * q[i16] + (-2.650868e-04) * q[i6] * q[i19]
            + (4.161348e-04) * q[i6] * q[i20] + (5.154993e-04) * q[i6] * q[i21] + (1.831471e-04) * q[i6] * q[i22] + (-5.774932e-05) * q[i7] * q[i8]
            + (4.408551e-03) * q[i7] * q[i9] + (-9.169212e-03) * q[i7] * q[i10] + (-8.042559e-04) * q[i7] * q[i11] + (7.330325e-04) * q[i7] * q[i12]
            + (6.341804e-03) * q[i7] * q[i15] + (2.538173e-03) * q[i7] * q[i16] + (4.083241e-04) * q[i7] * q[i19] + (-2.641031e-04) * q[i7] * q[i20]
            + (-1.823671e-04) * q[i7] * q[i21] + (-5.026373e-04) * q[i7] * q[i22] + (1.824121e-03) * q[i8] * q[i9] + (1.798970e-03) * q[i8] * q[i10]
            + (-9.114369e-04) * q[i8] * q[i11] + (7.988239e-04) * q[i8] * q[i12] + (-7.749465e-03) * q[i8] * q[i15] + (-7.820070e-03) * q[i8] * q[i16]
            + (-3.488025e-04) * q[i8] * q[i19] + (-3.333278e-04) * q[i8] * q[i20] + (-1.221320e-03) * q[i8] * q[i21] + (1.217945e-03) * q[i8] * q[i22]
            + (3.150382e-03) * q[i9] * q[i10] + (5.850180e-05) * q[i9] * q[i11] + (1.022610e-03) * q[i9] * q[i12] + (7.203772e-04) * q[i9] * q[i15]
            + (1.594974e-03) * q[i9] * q[i16] + (-1.511852e-04) * q[i9] * q[i19] + (-3.007639e-06) * q[i9] * q[i20] + (-1.003113e-04) * q[i9] * q[i21]
            + (5.577721e-04) * q[i9] * q[i22] + (-9.841551e-04) * q[i10] * q[i11] + (-2.625510e-05) * q[i10] * q[i12] + (1.558823e-03) * q[i10] * q[i15]
            + (7.085550e-04) * q[i10] * q[i16] + (-1.109468e-05) * q[i10] * q[i19] + (-1.568486e-04) * q[i10] * q[i20] + (-5.512975e-04) * q[i10] * q[i21]
            + (9.916410e-05) * q[i10] * q[i22] + (-3.692658e-04) * q[i11] * q[i12] + (1.224232e-03) * q[i11] * q[i15] + (8.886032e-04) * q[i11] * q[i16]
            + (-1.088796e-03) * q[i11] * q[i19] + (-4.647446e-04) * q[i11] * q[i20] + (-1.516041e-03) * q[i11] * q[i21] + (-1.487105e-04) * q[i11] * q[i22]
            + (-8.773765e-04) * q[i12] * q[i15] + (-1.290365e-03) * q[i12] * q[i16] + (4.731398e-04) * q[i12] * q[i19] + (1.054469e-03) * q[i12] * q[i20]
            + (-1.320978e-04) * q[i12] * q[i21] + (-1.535922e-03) * q[i12] * q[i22] + (5.665540e-04) * q[i15] * q[i16] + (1.849066e-04) * q[i15] * q[i19]
            + (3.171280e-05) * q[i15] * q[i20] + (7.157948e-04) * q[i15] * q[i21] + (-1.049503e-04) * q[i15] * q[i22] + (3.598342e-05) * q[i16] * q[i19]
            + (1.660364e-04) * q[i16] * q[i20] + (1.118701e-04) * q[i16] * q[i21] + (-7.094236e-04) * q[i16] * q[i22] + (-1.484084e-04) * q[i19] * q[i20]
            + (1.465471e-03) * q[i19] * q[i21] + (-2.425777e-04) * q[i19] * q[i22] + (2.482349e-04) * q[i20] * q[i21] + (-1.482029e-03) * q[i20] * q[i22]
            + (-4.247383e-04) * q[i21] * q[i22] + (2.816164e-03) * q[i0] * q[i0] * q[i0] + (3.268503e-04) * q[i0] * q[i0] * q[i1]
            + (2.064399e-03) * q[i0] * q[i0] * q[i2] + (-3.329575e-03) * q[i0] * q[i0] * q[i3] + (-1.948477e-03) * q[i0] * q[i0] * q[i4]
            + (-1.714863e-03) * q[i0] * q[i0] * q[i5] + (-2.474288e-02) * q[i0] * q[i0] * q[i6] + (-2.529714e-03) * q[i0] * q[i0] * q[i7]
            + (2.492421e-04) * q[i0] * q[i0] * q[i8] + (-3.616889e-03) * q[i0] * q[i0] * q[i9] + (1.348726e-04) * q[i0] * q[i0] * q[i10]
            + (2.516890e-04) * q[i0] * q[i0] * q[i11] + (1.310823e-04) * q[i0] * q[i0] * q[i12] + (2.843503e-04) * q[i0] * q[i0] * q[i15]
            + (3.868913e-04) * q[i0] * q[i0] * q[i16] + (2.950859e-04) * q[i0] * q[i0] * q[i19] + (2.163784e-04) * q[i0] * q[i0] * q[i20]
            + (3.546135e-04) * q[i0] * q[i0] * q[i21] + (-8.499914e-05) * q[i0] * q[i0] * q[i22] + (-3.029270e-04) * q[i0] * q[i1] * q[i1]
            + (-2.819707e-03) * q[i1] * q[i1] * q[i1] + (-2.060532e-03) * q[i1] * q[i1] * q[i2] + (1.952530e-03) * q[i1] * q[i1] * q[i3]
            + (3.350859e-03) * q[i1] * q[i1] * q[i4] + (1.726265e-03) * q[i1] * q[i1] * q[i5] + (-2.540755e-03) * q[i1] * q[i1] * q[i6]
            + (-2.469259e-02) * q[i1] * q[i1] * q[i7] + (2.526131e-04) * q[i1] * q[i1] * q[i8] + (1.274547e-04) * q[i1] * q[i1] * q[i9]
            + (-3.584321e-03) * q[i1] * q[i1] * q[i10] + (-1.337575e-04) * q[i1] * q[i1] * q[i11] + (-2.479423e-04) * q[i1] * q[i1] * q[i12]
            + (3.789879e-04) * q[i1] * q[i1] * q[i15] + (2.860883e-04) * q[i1] * q[i1] * q[i16] + (2.170462e-04) * q[i1] * q[i1] * q[i19]
            + (2.942254e-04) * q[i1] * q[i1] * q[i20] + (8.782667e-05) * q[i1] * q[i1] * q[i21] + (-3.527509e-04) * q[i1] * q[i1] * q[i22]
            + (1.145821e-03) * q[i0] * q[i2] * q[i2] + (-1.155402e-03) * q[i1] * q[i2] * q[i2] + (-2.214427e-05) * q[i2] * q[i2] * q[i2]
            + (-6.231101e-04) * q[i2] * q[i2] * q[i3] + (6.283564e-04) * q[i2] * q[i2] * q[i4] + (-2.788829e-06) * q[i2] * q[i2] * q[i5]
            + (-1.645044e-03) * q[i2] * q[i2] * q[i6] + (-1.659321e-03) * q[i2] * q[i2] * q[i7] + (-3.049899e-02) * q[i2] * q[i2] * q[i8]
            + (-1.123049e-03) * q[i2] * q[i2] * q[i9] + (-1.118759e-03) * q[i2] * q[i2] * q[i10] + (1.546000e-03) * q[i2] * q[i2] * q[i11]
            + (-1.606939e-03) * q[i2] * q[i2] * q[i12] + (3.446050e-04) * q[i2] * q[i2] * q[i15] + (3.625418e-04) * q[i2] * q[i2] * q[i16]
            + (9.646155e-04) * q[i2] * q[i2] * q[i19] + (9.561565e-04) * q[i2] * q[i2] * q[i20] + (9.184563e-06) * q[i2] * q[i2] * q[i21]
            + (-9.068572e-06) * q[i2] * q[i2] * q[i22] + (7.659209e-03) * q[i0] * q[i3] * q[i3] + (-1.703064e-03) * q[i1] * q[i3] * q[i3]
            + (1.515015e-03) * q[i2] * q[i3] * q[i3] + (1.310312e-04) * q[i3] * q[i3] * q[i3] + (1.318724e-03) * q[i3] * q[i3] * q[i4]
            + (5.198465e-04) * q[i3] * q[i3] * q[i5] + (1.017437e-02) * q[i3] * q[i3] * q[i6] + (-3.002781e-03) * q[i3] * q[i3] * q[i7]
            + (4.890949e-03) * q[i3] * q[i3] * q[i8] + (1.245209e-03) * q[i3] * q[i3] * q[i9] + (-6.931558e-04) * q[i3] * q[i3] * q[i10]
            + (-6.311628e-04) * q[i3] * q[i3] * q[i11] + (3.770896e-04) * q[i3] * q[i3] * q[i12] + (7.268971e-05) * q[i3] * q[i3] * q[i15]
            + (-2.084037e-04) * q[i3] * q[i3] * q[i16] + (-1.649753e-04) * q[i3] * q[i3] * q[i19] + (-3.124390e-04) * q[i3] * q[i3] * q[i20]
            + (-1.674498e-04) * q[i3] * q[i3] * q[i21] + (-1.204875e-04) * q[i3] * q[i3] * q[i22] + (1.713932e-03) * q[i0] * q[i4] * q[i4]
            + (-7.622494e-03) * q[i1] * q[i4] * q[i4] + (-1.485448e-03) * q[i2] * q[i4] * q[i4] + (-1.305402e-03) * q[i3] * q[i4] * q[i4]
            + (-1.340514e-04) * q[i4] * q[i4] * q[i4] + (-5.512622e-04) * q[i4] * q[i4] * q[i5] + (-2.999238e-03) * q[i4] * q[i4] * q[i6]
            + (1.018575e-02) * q[i4] * q[i4] * q[i7] + (4.897763e-03) * q[i4] * q[i4] * q[i8] + (-7.027872e-04) * q[i4] * q[i4] * q[i9]
            + (1.253167e-03) * q[i4] * q[i4] * q[i10] + (-3.640856e-04) * q[i4] * q[i4] * q[i11] + (6.490454e-04) * q[i4] * q[i4] * q[i12]
            + (-2.037655e-04) * q[i4] * q[i4] * q[i15] + (6.807479e-05) * q[i4] * q[i4] * q[i16] + (-3.064979e-04) * q[i4] * q[i4] * q[i19]
            + (-1.632086e-04) * q[i4] * q[i4] * q[i20] + (1.172331e-04) * q[i4] * q[i4] * q[i21] + (1.670597e-04) * q[i4] * q[i4] * q[i22]
            + (1.598485e-04) * q[i0] * q[i5] * q[i5] + (-1.496662e-04) * q[i1] * q[i5] * q[i5] + (-1.879650e-05) * q[i2] * q[i5] * q[i5]
            + (9.509695e-04) * q[i3] * q[i5] * q[i5] + (-9.547658e-04) * q[i4] * q[i5] * q[i5] + (1.624199e-05) * q[i5] * q[i5] * q[i5]
            + (1.201563e-04) * q[i5] * q[i5] * q[i6] + (1.129247e-04) * q[i5] * q[i5] * q[i7] + (3.681054e-03) * q[i5] * q[i5] * q[i8]
            + (1.445221e-04) * q[i5] * q[i5] * q[i9] + (1.495364e-04) * q[i5] * q[i5] * q[i10] + (2.935778e-04) * q[i5] * q[i5] * q[i11]
            + (-2.990441e-04) * q[i5] * q[i5] * q[i12] + (-1.269300e-03) * q[i5] * q[i5] * q[i15] + (-1.285440e-03) * q[i5] * q[i5] * q[i16]
            + (-7.327898e-04) * q[i5] * q[i5] * q[i19] + (-7.266314e-04) * q[i5] * q[i5] * q[i20] + (-3.969436e-04) * q[i5] * q[i5] * q[i21]
            + (3.976714e-04) * q[i5] * q[i5] * q[i22] + (-4.359479e-03) * q[i0] * q[i6] * q[i6] + (-4.147658e-04) * q[i1] * q[i6] * q[i6]
            + (-8.370063e-04) * q[i2] * q[i6] * q[i6] + (-1.337203e-03) * q[i3] * q[i6] * q[i6] + (3.797123e-04) * q[i4] * q[i6] * q[i6]
            + (5.578361e-04) * q[i5] * q[i6] * q[i6] + (-2.034647e-03) * q[i6] * q[i6] * q[i6] + (-1.443923e-03) * q[i6] * q[i6] * q[i7]
            + (1.496093e-03) * q[i6] * q[i6] * q[i8] + (-5.717403e-04) * q[i6] * q[i6] * q[i9] + (-3.134357e-04) * q[i6] * q[i6] * q[i10]
            + (-5.248814e-04) * q[i6] * q[i6] * q[i11] + (-2.601745e-04) * q[i6] * q[i6] * q[i12] + (-5.445084e-04) * q[i6] * q[i6] * q[i15]
            + (-2.344344e-04) * q[i6] * q[i6] * q[i16] + (-4.345726e-04) * q[i6] * q[i6] * q[i19] + (7.992784e-05) * q[i6] * q[i6] * q[i20]
            + (-1.263081e-04) * q[i6] * q[i6] * q[i21] + (2.285076e-04) * q[i6] * q[i6] * q[i22] + (4.180654e-04) * q[i0] * q[i7] * q[i7]
            + (4.344633e-03) * q[i1] * q[i7] * q[i7] + (8.489854e-04) * q[i2] * q[i7] * q[i7] + (-3.771542e-04) * q[i3] * q[i7] * q[i7]
            + (1.369226e-03) * q[i4] * q[i7] * q[i7] + (-5.615365e-04) * q[i5] * q[i7] * q[i7] + (-1.441506e-03) * q[i6] * q[i7] * q[i7]
            + (-2.043729e-03) * q[i7] * q[i7] * q[i7] + (1.520306e-03) * q[i7] * q[i7] * q[i8] + (-3.131878e-04) * q[i7] * q[i7] * q[i9]
            + (-5.660781e-04) * q[i7] * q[i7] * q[i10] + (2.586729e-04) * q[i7] * q[i7] * q[i11] + (5.256333e-04) * q[i7] * q[i7] * q[i12]
            + (-2.316011e-04) * q[i7] * q[i7] * q[i15] + (-5.468400e-04) * q[i7] * q[i7] * q[i16] + (8.015671e-05) * q[i7] * q[i7] * q[i19]
            + (-4.334767e-04) * q[i7] * q[i7] * q[i20] + (-2.268698e-04) * q[i7] * q[i7] * q[i21] + (1.228869e-04) * q[i7] * q[i7] * q[i22]
            + (-1.026171e-03) * q[i0] * q[i8] * q[i8] + (1.033403e-03) * q[i1] * q[i8] * q[i8] + (1.672647e-05) * q[i2] * q[i8] * q[i8]
            + (-3.937382e-04) * q[i3] * q[i8] * q[i8] + (3.832575e-04) * q[i4] * q[i8] * q[i8] + (5.006040e-06) * q[i5] * q[i8] * q[i8]
            + (2.038677e-03) * q[i6] * q[i8] * q[i8] + (2.041062e-03) * q[i7] * q[i8] * q[i8] + (-3.380286e-03) * q[i8] * q[i8] * q[i8]
            + (1.219217e-04) * q[i8] * q[i8] * q[i9] + (1.215441e-04) * q[i8] * q[i8] * q[i10] + (1.006271e-03) * q[i8] * q[i8] * q[i11]
            + (-1.040167e-03) * q[i8] * q[i8] * q[i12] + (2.766317e-04) * q[i8] * q[i8] * q[i15] + (2.802137e-04) * q[i8] * q[i8] * q[i16]
            + (1.835682e-04) * q[i8] * q[i8] * q[i19] + (1.768287e-04) * q[i8] * q[i8] * q[i20] + (1.380972e-05) * q[i8] * q[i8] * q[i21]
            + (-1.355391e-05) * q[i8] * q[i8] * q[i22] + (-4.753033e-04) * q[i0] * q[i9] * q[i9] + (-1.276147e-04) * q[i1] * q[i9] * q[i9]
            + (1.951560e-04) * q[i2] * q[i9] * q[i9] + (4.354672e-04) * q[i3] * q[i9] * q[i9] + (8.160768e-04) * q[i4] * q[i9] * q[i9]
            + (8.902593e-04) * q[i5] * q[i9] * q[i9] + (-2.425500e-03) * q[i6] * q[i9] * q[i9] + (3.724591e-04) * q[i7] * q[i9] * q[i9]
            + (9.753884e-04) * q[i8] * q[i9] * q[i9] + (-8.323603e-04) * q[i9] * q[i9] * q[i9] + (-2.172481e-04) * q[i9] * q[i9] * q[i10]
            + (-2.931694e-05) * q[i9] * q[i9] * q[i11] + (5.262958e-05) * q[i9] * q[i9] * q[i12] + (-2.901246e-05) * q[i9] * q[i9] * q[i15]
            + (-2.284468e-04) * q[i9] * q[i9] * q[i16] + (6.414339e-05) * q[i9] * q[i9] * q[i19] + (-2.279670e-06) * q[i9] * q[i9] * q[i20]
            + (7.424689e-05) * q[i9] * q[i9] * q[i21] + (-1.788403e-04) * q[i9] * q[i9] * q[i22] + (1.221015e-04) * q[i0] * q[i10] * q[i10]
            + (4.739604e-04) * q[i1] * q[i10] * q[i10] + (-1.928276e-04) * q[i2] * q[i10] * q[i10] + (-8.072043e-04) * q[i3] * q[i10] * q[i10]
            + (-4.293332e-04) * q[i4] * q[i10] * q[i10] + (-8.848187e-04) * q[i5] * q[i10] * q[i10] + (3.637730e-04) * q[i6] * q[i10] * q[i10]
            + (-2.398467e-03) * q[i7] * q[i10] * q[i10] + (9.696730e-04) * q[i8] * q[i10] * q[i10] + (-2.200224e-04) * q[i9] * q[i10] * q[i10]
            + (-8.206650e-04) * q[i10] * q[i10] * q[i10] + (-5.578485e-05) * q[i10] * q[i10] * q[i11] + (2.749652e-05) * q[i10] * q[i10] * q[i12]
            + (-2.242911e-04) * q[i10] * q[i10] * q[i15] + (-2.973345e-05) * q[i10] * q[i10] * q[i16] + (-1.276539e-06) * q[i10] * q[i10] * q[i19]
            + (6.250395e-05) * q[i10] * q[i10] * q[i20] + (1.764229e-04) * q[i10] * q[i10] * q[i21] + (-7.383322e-05) * q[i10] * q[i10] * q[i22]
            + (2.362361e-04) * q[i0] * q[i11] * q[i11] + (-1.418573e-04) * q[i1] * q[i11] * q[i11] + (2.309479e-03) * q[i2] * q[i11] * q[i11]
            + (3.976655e-05) * q[i3] * q[i11] * q[i11] + (-3.286597e-04) * q[i4] * q[i11] * q[i11] + (1.262623e-03) * q[i5] * q[i11] * q[i11]
            + (-4.280665e-04) * q[i6] * q[i11] * q[i11] + (-1.671712e-05) * q[i7] * q[i11] * q[i11] + (1.529249e-03) * q[i8] * q[i11] * q[i11]
            + (-1.026045e-04) * q[i9] * q[i11] * q[i11] + (6.209638e-05) * q[i10] * q[i11] * q[i11] + (1.094118e-03) * q[i11] * q[i11] * q[i11]
            + (-8.548084e-05) * q[i11] * q[i11] * q[i12] + (-1.956376e-03) * q[i11] * q[i11] * q[i15] + (-1.667675e-04) * q[i11] * q[i11] * q[i16]
            + (4.941124e-04) * q[i11] * q[i11] * q[i19] + (-2.970354e-05) * q[i11] * q[i11] * q[i20] + (-4.377671e-05) * q[i11] * q[i11] * q[i21]
            + (-4.816737e-05) * q[i11] * q[i11] * q[i22] + (1.346939e-04) * q[i0] * q[i12] * q[i12] + (-2.415135e-04) * q[i1] * q[i12] * q[i12]
            + (-2.434125e-03) * q[i2] * q[i12] * q[i12] + (3.180134e-04) * q[i3] * q[i12] * q[i12] + (-5.807313e-05) * q[i4] * q[i12] * q[i12]
            + (-1.318934e-03) * q[i5] * q[i12] * q[i12] + (-1.049009e-05) * q[i6] * q[i12] * q[i12] + (-4.457106e-04) * q[i7] * q[i12] * q[i12]
            + (1.567306e-03) * q[i8] * q[i12] * q[i12] + (7.635329e-05) * q[i9] * q[i12] * q[i12] + (-1.057925e-04) * q[i10] * q[i12] * q[i12]
            + (9.131174e-05) * q[i11] * q[i12] * q[i12] + (-1.147698e-03) * q[i12] * q[i12] * q[i12] + (-1.695134e-04) * q[i12] * q[i12] * q[i15]
            + (-1.947121e-03) * q[i12] * q[i12] * q[i16] + (-2.958783e-05) * q[i12] * q[i12] * q[i19] + (4.820150e-04) * q[i12] * q[i12] * q[i20]
            + (4.809768e-05) * q[i12] * q[i12] * q[i21] + (5.052362e-05) * q[i12] * q[i12] * q[i22] + (1.811154e-04) * q[i0] * q[i15] * q[i15]
            + (2.195069e-04) * q[i1] * q[i15] * q[i15] + (2.480727e-03) * q[i2] * q[i15] * q[i15] + (-6.305771e-04) * q[i3] * q[i15] * q[i15]
            + (-3.253032e-04) * q[i4] * q[i15] * q[i15] + (2.926691e-04) * q[i5] * q[i15] * q[i15] + (-3.336203e-05) * q[i6] * q[i15] * q[i15]
            + (7.962845e-04) * q[i7] * q[i15] * q[i15] + (1.056451e-04) * q[i8] * q[i15] * q[i15] + (-1.752264e-04) * q[i9] * q[i15] * q[i15]
            + (-5.884776e-05) * q[i10] * q[i15] * q[i15] + (2.137069e-03) * q[i11] * q[i15] * q[i15] + (2.019327e-05) * q[i12] * q[i15] * q[i15]
            + (2.280557e-04) * q[i15] * q[i15] * q[i15] + (-2.599556e-05) * q[i15] * q[i15] * q[i16] + (1.298486e-04) * q[i15] * q[i15] * q[i19]
            + (-6.822884e-05) * q[i15] * q[i15] * q[i20] + (-1.844805e-04) * q[i15] * q[i15] * q[i21] + (9.319952e-05) * q[i15] * q[i15] * q[i22]
            + (-2.201567e-04) * q[i0] * q[i16] * q[i16] + (-1.820508e-04) * q[i1] * q[i16] * q[i16] + (-2.510068e-03) * q[i2] * q[i16] * q[i16]
            + (3.251068e-04) * q[i3] * q[i16] * q[i16] + (6.351138e-04) * q[i4] * q[i16] * q[i16] + (-2.914479e-04) * q[i5] * q[i16] * q[i16]
            + (8.003635e-04) * q[i6] * q[i16] * q[i16] + (-3.535705e-05) * q[i7] * q[i16] * q[i16] + (1.140662e-04) * q[i8] * q[i16] * q[i16]
            + (-6.107686e-05) * q[i9] * q[i16] * q[i16] + (-1.768509e-04) * q[i10] * q[i16] * q[i16] + (-2.123403e-05) * q[i11] * q[i16] * q[i16]
            + (-2.171113e-03) * q[i12] * q[i16] * q[i16] + (-2.785503e-05) * q[i15] * q[i16] * q[i16] + (2.302472e-04) * q[i16] * q[i16] * q[i16]
            + (-6.792132e-05) * q[i16] * q[i16] * q[i19] + (1.229388e-04) * q[i16] * q[i16] * q[i20] + (-9.468958e-05) * q[i16] * q[i16] * q[i21]
            + (1.911788e-04) * q[i16] * q[i16] * q[i22] + (1.728568e-04) * q[i0] * q[i19] * q[i19] + (4.184498e-05) * q[i1] * q[i19] * q[i19]
            + (-2.042859e-04) * q[i2] * q[i19] * q[i19] + (-4.560208e-04) * q[i3] * q[i19] * q[i19] + (1.279469e-04) * q[i4] * q[i19] * q[i19]
            + (-2.332126e-04) * q[i5] * q[i19] * q[i19] + (-1.507277e-04) * q[i6] * q[i19] * q[i19] + (1.477554e-04) * q[i7] * q[i19] * q[i19]
            + (-1.688085e-04) * q[i8] * q[i19] * q[i19] + (1.884527e-05) * q[i9] * q[i19] * q[i19] + (-7.063182e-05) * q[i10] * q[i19] * q[i19]
            + (-4.597726e-04) * q[i11] * q[i19] * q[i19] + (6.349233e-05) * q[i12] * q[i19] * q[i19] + (-2.985337e-04) * q[i15] * q[i19] * q[i19]
            + (-7.985526e-05) * q[i16] * q[i19] * q[i19] + (6.091844e-05) * q[i19] * q[i19] * q[i19] + (-4.986784e-05) * q[i19] * q[i19] * q[i20]
            + (-1.143215e-04) * q[i19] * q[i19] * q[i21] + (3.806140e-05) * q[i19] * q[i19] * q[i22] + (-4.234846e-05) * q[i0] * q[i20] * q[i20]
            + (-1.711052e-04) * q[i1] * q[i20] * q[i20] + (2.057429e-04) * q[i2] * q[i20] * q[i20] + (-1.282047e-04) * q[i3] * q[i20] * q[i20]
            + (4.503998e-04) * q[i4] * q[i20] * q[i20] + (2.367955e-04) * q[i5] * q[i20] * q[i20] + (1.439755e-04) * q[i6] * q[i20] * q[i20]
            + (-1.466157e-04) * q[i7] * q[i20] * q[i20] + (-1.628132e-04) * q[i8] * q[i20] * q[i20] + (-7.237348e-05) * q[i9] * q[i20] * q[i20]
            + (1.950431e-05) * q[i10] * q[i20] * q[i20] + (-6.177688e-05) * q[i11] * q[i20] * q[i20] + (4.608820e-04) * q[i12] * q[i20] * q[i20]
            + (-8.019728e-05) * q[i15] * q[i20] * q[i20] + (-2.866588e-04) * q[i16] * q[i20] * q[i20] + (-5.008723e-05) * q[i19] * q[i20] * q[i20]
            + (5.911894e-05) * q[i20] * q[i20] * q[i20] + (-3.731444e-05) * q[i20] * q[i20] * q[i21] + (1.148807e-04) * q[i20] * q[i20] * q[i22]
            + (6.811269e-05) * q[i0] * q[i21] * q[i21] + (6.981324e-05) * q[i1] * q[i21] * q[i21] + (3.228096e-04) * q[i2] * q[i21] * q[i21]
            + (-1.587875e-06) * q[i3] * q[i21] * q[i21] + (-1.481153e-04) * q[i4] * q[i21] * q[i21] + (-4.019923e-04) * q[i5] * q[i21] * q[i21]
            + (9.304271e-05) * q[i6] * q[i21] * q[i21] + (4.622909e-04) * q[i7] * q[i21] * q[i21] + (-9.195516e-04) * q[i8] * q[i21] * q[i21]
            + (4.402841e-05) * q[i9] * q[i21] * q[i21] + (5.434041e-05) * q[i10] * q[i21] * q[i21] + (-4.426272e-04) * q[i11] * q[i21] * q[i21]
            + (-6.658309e-05) * q[i12] * q[i21] * q[i21] + (-8.546269e-05) * q[i15] * q[i21] * q[i21] + (-5.159170e-05) * q[i16] * q[i21] * q[i21]
            + (5.550159e-04) * q[i19] * q[i21] * q[i21] + (-6.873703e-05) * q[i20] * q[i21] * q[i21] + (-8.845787e-05) * q[i21] * q[i21] * q[i21]
            + (-4.517425e-05) * q[i21] * q[i21] * q[i22] + (-7.112266e-05) * q[i0] * q[i22] * q[i22] + (-6.529956e-05) * q[i1] * q[i22] * q[i22]
            + (-3.137231e-04) * q[i2] * q[i22] * q[i22] + (1.493617e-04) * q[i3] * q[i22] * q[i22] + (-1.566640e-06) * q[i4] * q[i22] * q[i22]
            + (4.060778e-04) * q[i5] * q[i22] * q[i22] + (4.638544e-04) * q[i6] * q[i22] * q[i22] + (9.348077e-05) * q[i7] * q[i22] * q[i22]
            + (-9.218395e-04) * q[i8] * q[i22] * q[i22] + (5.344678e-05) * q[i9] * q[i22] * q[i22] + (4.411843e-05) * q[i10] * q[i22] * q[i22]
            + (6.576752e-05) * q[i11] * q[i22] * q[i22] + (4.546156e-04) * q[i12] * q[i22] * q[i22] + (-5.206458e-05) * q[i15] * q[i22] * q[i22]
            + (-8.863382e-05) * q[i16] * q[i22] * q[i22] + (-6.935476e-05) * q[i19] * q[i22] * q[i22] + (5.637903e-04) * q[i20] * q[i22] * q[i22]
            + (4.528698e-05) * q[i21] * q[i22] * q[i22] + (9.090574e-05) * q[i22] * q[i22] * q[i22] + (3.179068e-06) * q[i0] * q[i1] * q[i2]
            + (-2.378489e-03) * q[i0] * q[i1] * q[i3] + (2.358891e-03) * q[i0] * q[i1] * q[i4] + (9.988349e-06) * q[i0] * q[i1] * q[i5]
            + (6.064606e-03) * q[i0] * q[i1] * q[i6] + (6.056071e-03) * q[i0] * q[i1] * q[i7] + (-1.745568e-04) * q[i0] * q[i1] * q[i8]
            + (9.218836e-04) * q[i0] * q[i1] * q[i9] + (9.146743e-04) * q[i0] * q[i1] * q[i10] + (-2.862139e-04) * q[i0] * q[i1] * q[i11]
            + (2.970770e-04) * q[i0] * q[i1] * q[i12] + (-1.059418e-04) * q[i0] * q[i1] * q[i15] + (-1.032743e-04) * q[i0] * q[i1] * q[i16]
            + (-1.413254e-04) * q[i0] * q[i1] * q[i19] + (-1.424201e-04) * q[i0] * q[i1] * q[i20] + (1.137842e-04) * q[i0] * q[i1] * q[i21]
            + (-1.097233e-04) * q[i0] * q[i1] * q[i22] + (-3.557220e-03) * q[i0] * q[i2] * q[i3] + (-1.882166e-03) * q[i0] * q[i2] * q[i4]
            + (-1.790790e-03) * q[i0] * q[i2] * q[i5] + (-1.163352e-02) * q[i0] * q[i2] * q[i6] + (7.894988e-04) * q[i0] * q[i2] * q[i7]
            + (-5.673798e-03) * q[i0] * q[i2] * q[i8] + (-1.526473e-03) * q[i0] * q[i2] * q[i9] + (1.848176e-03) * q[i0] * q[i2] * q[i10]
            + (1.045153e-03) * q[i0] * q[i2] * q[i11] + (6.411273e-05) * q[i0] * q[i2] * q[i12] + (3.562394e-04) * q[i0] * q[i2] * q[i15]
            + (7.656795e-04) * q[i0] * q[i2] * q[i16] + (6.379012e-04) * q[i0] * q[i2] * q[i19] + (3.685915e-05) * q[i0] * q[i2] * q[i20]
            + (1.027054e-04) * q[i0] * q[i2] * q[i21] + (-6.779181e-04) * q[i0] * q[i2] * q[i22] + (1.336112e-02) * q[i0] * q[i3] * q[i4]
            + (9.800715e-03) * q[i0] * q[i3] * q[i5] + (-1.615880e-02) * q[i0] * q[i3] * q[i6] + (3.259265e-03) * q[i0] * q[i3] * q[i7]
            + (-1.176956e-03) * q[i0] * q[i3] * q[i8] + (1.243164e-02) * q[i0] * q[i3] * q[i9] + (-3.773849e-03) * q[i0] * q[i3] * q[i10]
            + (3.946048e-04) * q[i0] * q[i3] * q[i11] + (-9.528819e-04) * q[i0] * q[i3] * q[i12] + (-1.674781e-03) * q[i0] * q[i3] * q[i15]
            + (-3.861911e-03) * q[i0] * q[i3] * q[i16] + (-7.626717e-04) * q[i0] * q[i3] * q[i19] + (4.278285e-05) * q[i0] * q[i3] * q[i20]
            + (-1.361292e-04) * q[i0] * q[i3] * q[i21] + (-8.498490e-04) * q[i0] * q[i3] * q[i22] + (3.740858e-03) * q[i0] * q[i4] * q[i5]
            + (-8.506806e-04) * q[i0] * q[i4] * q[i6] + (3.139579e-03) * q[i0] * q[i4] * q[i7] + (2.537469e-04) * q[i0] * q[i4] * q[i8]
            + (-1.366809e-04) * q[i0] * q[i4] * q[i9] + (-9.703830e-04) * q[i0] * q[i4] * q[i10] + (-2.479629e-05) * q[i0] * q[i4] * q[i11]
            + (-2.565737e-04) * q[i0] * q[i4] * q[i12] + (5.375513e-04) * q[i0] * q[i4] * q[i15] + (5.378835e-04) * q[i0] * q[i4] * q[i16]
            + (2.205854e-05) * q[i0] * q[i4] * q[i19] + (9.761715e-06) * q[i0] * q[i4] * q[i20] + (-2.597395e-04) * q[i0] * q[i4] * q[i21]
            + (-6.792909e-04) * q[i0] * q[i4] * q[i22] + (4.591833e-03) * q[i0] * q[i5] * q[i6] + (-9.150038e-04) * q[i0] * q[i5] * q[i7]
            + (-1.960694e-05) * q[i0] * q[i5] * q[i8] + (2.434291e-04) * q[i0] * q[i5] * q[i9] + (1.174825e-03) * q[i0] * q[i5] * q[i10]
            + (-1.086987e-03) * q[i0] * q[i5] * q[i11] + (1.086708e-03) * q[i0] * q[i5] * q[i12] + (-8.253056e-04) * q[i0] * q[i5] * q[i15]
            + (-1.200608e-03) * q[i0] * q[i5] * q[i16] + (-1.774556e-04) * q[i0] * q[i5] * q[i19] + (-3.916660e-04) * q[i0] * q[i5] * q[i20]
            + (-8.283292e-06) * q[i0] * q[i5] * q[i21] + (6.741703e-04) * q[i0] * q[i5] * q[i22] + (1.025195e-02) * q[i0] * q[i6] * q[i7]
            + (3.391187e-03) * q[i0] * q[i6] * q[i8] + (-1.594058e-03) * q[i0] * q[i6] * q[i9] + (2.157548e-03) * q[i0] * q[i6] * q[i10]
            + (-1.685007e-03) * q[i0] * q[i6] * q[i11] + (-1.091461e-03) * q[i0] * q[i6] * q[i12] + (-4.290466e-04) * q[i0] * q[i6] * q[i15]
            + (1.370662e-03) * q[i0] * q[i6] * q[i16] + (-1.298252e-03) * q[i0] * q[i6] * q[i19] + (9.889740e-04) * q[i0] * q[i6] * q[i20]
            + (-1.969670e-04) * q[i0] * q[i6] * q[i21] + (8.597856e-06) * q[i0] * q[i6] * q[i22] + (-5.919886e-04) * q[i0] * q[i7] * q[i8]
            + (1.357902e-03) * q[i0] * q[i7] * q[i9] + (-2.642911e-05) * q[i0] * q[i7] * q[i10] + (4.746056e-05) * q[i0] * q[i7] * q[i11]
            + (-7.667034e-05) * q[i0] * q[i7] * q[i12] + (4.276688e-04) * q[i0] * q[i7] * q[i15] + (-3.514662e-05) * q[i0] * q[i7] * q[i16]
            + (3.153959e-04) * q[i0] * q[i7] * q[i19] + (-1.302443e-04) * q[i0] * q[i7] * q[i20] + (-1.596513e-05) * q[i0] * q[i7] * q[i21]
            + (-4.932519e-04) * q[i0] * q[i7] * q[i22] + (1.484234e-03) * q[i0] * q[i8] * q[i9] + (-1.658021e-04) * q[i0] * q[i8] * q[i10]
            + (-4.435071e-04) * q[i0] * q[i8] * q[i11] + (-2.355451e-04) * q[i0] * q[i8] * q[i12] + (-6.521984e-05) * q[i0] * q[i8] * q[i15]
            + (2.012094e-04) * q[i0] * q[i8] * q[i16] + (-2.381760e-04) * q[i0] * q[i8] * q[i19] + (1.675113e-04) * q[i0] * q[i8] * q[i20]
            + (3.856355e-04) * q[i0] * q[i8] * q[i21] + (5.524433e-05) * q[i0] * q[i8] * q[i22] + (3.607942e-04) * q[i0] * q[i9] * q[i10]
            + (1.239044e-04) * q[i0] * q[i9] * q[i11] + (-3.238291e-04) * q[i0] * q[i9] * q[i12] + (-2.437090e-04) * q[i0] * q[i9] * q[i15]
            + (3.510747e-04) * q[i0] * q[i9] * q[i16] + (-2.967470e-04) * q[i0] * q[i9] * q[i19] + (3.917039e-04) * q[i0] * q[i9] * q[i20]
            + (2.764603e-05) * q[i0] * q[i9] * q[i21] + (-1.361217e-04) * q[i0] * q[i9] * q[i22] + (5.471964e-04) * q[i0] * q[i10] * q[i11]
            + (2.231798e-04) * q[i0] * q[i10] * q[i12] + (3.978931e-04) * q[i0] * q[i10] * q[i15] + (-2.885660e-04) * q[i0] * q[i10] * q[i16]
            + (2.560510e-04) * q[i0] * q[i10] * q[i19] + (1.447837e-04) * q[i0] * q[i10] * q[i20] + (1.370656e-04) * q[i0] * q[i10] * q[i21]
            + (3.005937e-05) * q[i0] * q[i10] * q[i22] + (1.395562e-04) * q[i0] * q[i11] * q[i12] + (1.694093e-04) * q[i0] * q[i11] * q[i15]
            + (-2.069843e-04) * q[i0] * q[i11] * q[i16] + (6.329313e-04) * q[i0] * q[i11] * q[i19] + (-2.262147e-04) * q[i0] * q[i11] * q[i20]
            + (1.627175e-05) * q[i0] * q[i11] * q[i21] + (8.850492e-05) * q[i0] * q[i11] * q[i22] + (-1.615810e-04) * q[i0] * q[i12] * q[i15]
            + (-1.409281e-05) * q[i0] * q[i12] * q[i16] + (1.480147e-04) * q[i0] * q[i12] * q[i19] + (-1.575840e-04) * q[i0] * q[i12] * q[i20]
            + (8.082817e-05) * q[i0] * q[i12] * q[i21] + (-1.832422e-04) * q[i0] * q[i12] * q[i22] + (1.832028e-04) * q[i0] * q[i15] * q[i16]
            + (-8.622976e-06) * q[i0] * q[i15] * q[i19] + (-1.083230e-04) * q[i0] * q[i15] * q[i20] + (3.299489e-04) * q[i0] * q[i15] * q[i21]
            + (1.040241e-05) * q[i0] * q[i15] * q[i22] + (1.609813e-04) * q[i0] * q[i16] * q[i19] + (-1.584302e-04) * q[i0] * q[i16] * q[i20]
            + (3.315816e-05) * q[i0] * q[i16] * q[i21] + (7.259586e-04) * q[i0] * q[i16] * q[i22] + (2.514656e-05) * q[i0] * q[i19] * q[i20]
            + (-1.606318e-06) * q[i0] * q[i19] * q[i21] + (-4.005700e-04) * q[i0] * q[i19] * q[i22] + (4.732295e-05) * q[i0] * q[i20] * q[i21]
            + (1.434421e-04) * q[i0] * q[i20] * q[i22] + (-2.342242e-04) * q[i0] * q[i21] * q[i22] + (1.905476e-03) * q[i1] * q[i2] * q[i3]
            + (3.555067e-03) * q[i1] * q[i2] * q[i4] + (1.789451e-03) * q[i1] * q[i2] * q[i5] + (7.748543e-04) * q[i1] * q[i2] * q[i6]
            + (-1.163635e-02) * q[i1] * q[i2] * q[i7] + (-5.671491e-03) * q[i1] * q[i2] * q[i8] + (1.846815e-03) * q[i1] * q[i2] * q[i9]
            + (-1.523589e-03) * q[i1] * q[i2] * q[i10] + (-6.976181e-05) * q[i1] * q[i2] * q[i11] + (-1.039166e-03) * q[i1] * q[i2] * q[i12]
            + (7.528875e-04) * q[i1] * q[i2] * q[i15] + (3.601205e-04) * q[i1] * q[i2] * q[i16] + (3.645932e-05) * q[i1] * q[i2] * q[i19]
            + (6.373758e-04) * q[i1] * q[i2] * q[i20] + (6.805839e-04) * q[i1] * q[i2] * q[i21] + (-1.026114e-04) * q[i1] * q[i2] * q[i22]
            + (-1.336134e-02) * q[i1] * q[i3] * q[i4] + (-3.748302e-03) * q[i1] * q[i3] * q[i5] + (3.136272e-03) * q[i1] * q[i3] * q[i6]
            + (-8.651054e-04) * q[i1] * q[i3] * q[i7] + (2.425740e-04) * q[i1] * q[i3] * q[i8] + (-9.725343e-04) * q[i1] * q[i3] * q[i9]
            + (-1.391450e-04) * q[i1] * q[i3] * q[i10] + (2.653701e-04) * q[i1] * q[i3] * q[i11] + (3.474245e-05) * q[i1] * q[i3] * q[i12]
            + (5.455054e-04) * q[i1] * q[i3] * q[i15] + (5.488424e-04) * q[i1] * q[i3] * q[i16] + (1.523011e-05) * q[i1] * q[i3] * q[i19]
            + (2.528618e-05) * q[i1] * q[i3] * q[i20] + (6.851692e-04) * q[i1] * q[i3] * q[i21] + (2.598052e-04) * q[i1] * q[i3] * q[i22]
            + (-9.843163e-03) * q[i1] * q[i4] * q[i5] + (3.240810e-03) * q[i1] * q[i4] * q[i6] + (-1.622585e-02) * q[i1] * q[i4] * q[i7]
            + (-1.161245e-03) * q[i1] * q[i4] * q[i8] + (-3.814247e-03) * q[i1] * q[i4] * q[i9] + (1.230341e-02) * q[i1] * q[i4] * q[i10]
            + (9.170136e-04) * q[i1] * q[i4] * q[i11] + (-4.105944e-04) * q[i1] * q[i4] * q[i12] + (-3.807639e-03) * q[i1] * q[i4] * q[i15]
            + (-1.680345e-03) * q[i1] * q[i4] * q[i16] + (4.173312e-05) * q[i1] * q[i4] * q[i19] + (-7.562183e-04) * q[i1] * q[i4] * q[i20]
            + (8.366162e-04) * q[i1] * q[i4] * q[i21] + (1.205473e-04) * q[i1] * q[i4] * q[i22] + (-9.311442e-04) * q[i1] * q[i5] * q[i6]
            + (4.571576e-03) * q[i1] * q[i5] * q[i7] + (8.817743e-06) * q[i1] * q[i5] * q[i8] + (1.174705e-03) * q[i1] * q[i5] * q[i9]
            + (2.354629e-04) * q[i1] * q[i5] * q[i10] + (-1.123989e-03) * q[i1] * q[i5] * q[i11] + (1.038978e-03) * q[i1] * q[i5] * q[i12]
            + (-1.199295e-03) * q[i1] * q[i5] * q[i15] + (-8.293675e-04) * q[i1] * q[i5] * q[i16] + (-3.957794e-04) * q[i1] * q[i5] * q[i19]
            + (-1.755813e-04) * q[i1] * q[i5] * q[i20] + (-6.676990e-04) * q[i1] * q[i5] * q[i21] + (1.252290e-05) * q[i1] * q[i5] * q[i22]
            + (-1.024657e-02) * q[i1] * q[i6] * q[i7] + (5.875240e-04) * q[i1] * q[i6] * q[i8] + (2.119731e-05) * q[i1] * q[i6] * q[i9]
            + (-1.360581e-03) * q[i1] * q[i6] * q[i10] + (-7.651621e-05) * q[i1] * q[i6] * q[i11] + (4.752027e-05) * q[i1] * q[i6] * q[i12]
            + (3.144774e-05) * q[i1] * q[i6] * q[i15] + (-4.277297e-04) * q[i1] * q[i6] * q[i16] + (1.368596e-04) * q[i1] * q[i6] * q[i19]
            + (-3.205544e-04) * q[i1] * q[i6] * q[i20] + (-4.913701e-04) * q[i1] * q[i6] * q[i21] + (-8.847691e-06) * q[i1] * q[i6] * q[i22]
            + (-3.389789e-03) * q[i1] * q[i7] * q[i8] + (-2.176429e-03) * q[i1] * q[i7] * q[i9] + (1.605034e-03) * q[i1] * q[i7] * q[i10]
            + (-1.080001e-03) * q[i1] * q[i7] * q[i11] + (-1.693575e-03) * q[i1] * q[i7] * q[i12] + (-1.361213e-03) * q[i1] * q[i7] * q[i15]
            + (4.450985e-04) * q[i1] * q[i7] * q[i16] + (-9.905888e-04) * q[i1] * q[i7] * q[i19] + (1.288409e-03) * q[i1] * q[i7] * q[i20]
            + (1.105648e-05) * q[i1] * q[i7] * q[i21] + (-1.934229e-04) * q[i1] * q[i7] * q[i22] + (1.679392e-04) * q[i1] * q[i8] * q[i9]
            + (-1.470540e-03) * q[i1] * q[i8] * q[i10] + (-2.058888e-04) * q[i1] * q[i8] * q[i11] + (-4.676848e-04) * q[i1] * q[i8] * q[i12]
            + (-1.883926e-04) * q[i1] * q[i8] * q[i15] + (6.772655e-05) * q[i1] * q[i8] * q[i16] + (-1.747253e-04) * q[i1] * q[i8] * q[i19]
            + (2.498898e-04) * q[i1] * q[i8] * q[i20] + (4.661306e-05) * q[i1] * q[i8] * q[i21] + (3.830143e-04) * q[i1] * q[i8] * q[i22]
            + (-3.642427e-04) * q[i1] * q[i9] * q[i10] + (2.217855e-04) * q[i1] * q[i9] * q[i11] + (5.530140e-04) * q[i1] * q[i9] * q[i12]
            + (2.875109e-04) * q[i1] * q[i9] * q[i15] + (-3.991676e-04) * q[i1] * q[i9] * q[i16] + (-1.422766e-04) * q[i1] * q[i9] * q[i19]
            + (-2.535342e-04) * q[i1] * q[i9] * q[i20] + (3.084981e-05) * q[i1] * q[i9] * q[i21] + (1.378833e-04) * q[i1] * q[i9] * q[i22]
            + (-3.287098e-04) * q[i1] * q[i10] * q[i11] + (1.165157e-04) * q[i1] * q[i10] * q[i12] + (-3.468384e-04) * q[i1] * q[i10] * q[i15]
            + (2.473510e-04) * q[i1] * q[i10] * q[i16] + (-3.908049e-04) * q[i1] * q[i10] * q[i19] + (2.920327e-04) * q[i1] * q[i10] * q[i20]
            + (-1.361306e-04) * q[i1] * q[i10] * q[i21] + (3.006953e-05) * q[i1] * q[i10] * q[i22] + (-1.408358e-04) * q[i1] * q[i11] * q[i12]
            + (-1.825324e-05) * q[i1] * q[i11] * q[i15] + (-1.518464e-04) * q[i1] * q[i11] * q[i16] + (-1.618743e-04) * q[i1] * q[i11] * q[i19]
            + (1.495978e-04) * q[i1] * q[i11] * q[i20] + (1.855161e-04) * q[i1] * q[i11] * q[i21] + (-7.675297e-05) * q[i1] * q[i11] * q[i22]
            + (-2.055615e-04) * q[i1] * q[i12] * q[i15] + (1.753839e-04) * q[i1] * q[i12] * q[i16] + (-2.282897e-04) * q[i1] * q[i12] * q[i19]
            + (6.312069e-04) * q[i1] * q[i12] * q[i20] + (-8.990631e-05) * q[i1] * q[i12] * q[i21] + (-1.564901e-05) * q[i1] * q[i12] * q[i22]
            + (-1.795702e-04) * q[i1] * q[i15] * q[i16] + (1.611291e-04) * q[i1] * q[i15] * q[i19] + (-1.568737e-04) * q[i1] * q[i15] * q[i20]
            + (7.180507e-04) * q[i1] * q[i15] * q[i21] + (3.390483e-05) * q[i1] * q[i15] * q[i22] + (1.016426e-04) * q[i1] * q[i16] * q[i19]
            + (1.760141e-05) * q[i1] * q[i16] * q[i20] + (7.808667e-06) * q[i1] * q[i16] * q[i21] + (3.334762e-04) * q[i1] * q[i16] * q[i22]
            + (-2.081722e-05) * q[i1] * q[i19] * q[i20] + (1.391243e-04) * q[i1] * q[i19] * q[i21] + (4.763852e-05) * q[i1] * q[i19] * q[i22]
            + (-3.954753e-04) * q[i1] * q[i20] * q[i21] + (-3.753872e-06) * q[i1] * q[i20] * q[i22] + (2.334830e-04) * q[i1] * q[i21] * q[i22]
            + (-1.073665e-05) * q[i2] * q[i3] * q[i4] + (8.591752e-03) * q[i2] * q[i3] * q[i5] + (-3.653709e-03) * q[i2] * q[i3] * q[i6]
            + (1.358948e-03) * q[i2] * q[i3] * q[i7] + (1.127051e-04) * q[i2] * q[i3] * q[i8] + (2.158290e-03) * q[i2] * q[i3] * q[i9]
            + (-1.252840e-03) * q[i2] * q[i3] * q[i10] + (-2.804176e-04) * q[i2] * q[i3] * q[i11] + (3.965130e-04) * q[i2] * q[i3] * q[i12]
            + (4.431432e-04) * q[i2] * q[i3] * q[i15] + (-9.113398e-04) * q[i2] * q[i3] * q[i16] + (-4.642063e-04) * q[i2] * q[i3] * q[i19]
            + (-2.037979e-04) * q[i2] * q[i3] * q[i20] + (-4.673228e-04) * q[i2] * q[i3] * q[i21] + (2.693041e-04) * q[i2] * q[i3] * q[i22]
            + (-8.593283e-03) * q[i2] * q[i4] * q[i5] + (1.356865e-03) * q[i2] * q[i4] * q[i6] + (-3.675611e-03) * q[i2] * q[i4] * q[i7]
            + (1.230089e-04) * q[i2] * q[i4] * q[i8] + (-1.264543e-03) * q[i2] * q[i4] * q[i9] + (2.128636e-03) * q[i2] * q[i4] * q[i10]
            + (-4.179273e-04) * q[i2] * q[i4] * q[i11] + (2.623375e-04) * q[i2] * q[i4] * q[i12] + (-9.084603e-04) * q[i2] * q[i4] * q[i15]
            + (4.579803e-04) * q[i2] * q[i4] * q[i16] + (-2.047420e-04) * q[i2] * q[i4] * q[i19] + (-4.625191e-04) * q[i2] * q[i4] * q[i20]
            + (-2.657561e-04) * q[i2] * q[i4] * q[i21] + (4.650251e-04) * q[i2] * q[i4] * q[i22] + (-2.200894e-03) * q[i2] * q[i5] * q[i6]
            + (-2.212931e-03) * q[i2] * q[i5] * q[i7] + (-1.136204e-02) * q[i2] * q[i5] * q[i8] + (6.123080e-03) * q[i2] * q[i5] * q[i9]
            + (6.063427e-03) * q[i2] * q[i5] * q[i10] + (1.802504e-03) * q[i2] * q[i5] * q[i11] + (-1.910099e-03) * q[i2] * q[i5] * q[i12]
            + (-6.389843e-03) * q[i2] * q[i5] * q[i15] + (-6.479207e-03) * q[i2] * q[i5] * q[i16] + (2.116533e-05) * q[i2] * q[i5] * q[i19]
            + (2.157559e-05) * q[i2] * q[i5] * q[i20] + (7.172603e-04) * q[i2] * q[i5] * q[i21] + (-7.400227e-04) * q[i2] * q[i5] * q[i22]
            + (-1.284721e-05) * q[i2] * q[i6] * q[i7] + (8.949380e-03) * q[i2] * q[i6] * q[i8] + (4.654102e-04) * q[i2] * q[i6] * q[i9]
            + (-2.886947e-04) * q[i2] * q[i6] * q[i10] + (-1.117621e-03) * q[i2] * q[i6] * q[i11] + (2.174589e-04) * q[i2] * q[i6] * q[i12]
            + (-6.428325e-04) * q[i2] * q[i6] * q[i15] + (6.249175e-04) * q[i2] * q[i6] * q[i16] + (-7.928593e-04) * q[i2] * q[i6] * q[i19]
            + (9.333862e-05) * q[i2] * q[i6] * q[i20] + (2.855538e-05) * q[i2] * q[i6] * q[i21] + (8.983287e-05) * q[i2] * q[i6] * q[i22]
            + (-8.968141e-03) * q[i2] * q[i7] * q[i8] + (2.865255e-04) * q[i2] * q[i7] * q[i9] + (-4.609410e-04) * q[i2] * q[i7] * q[i10]
            + (2.363002e-04) * q[i2] * q[i7] * q[i11] + (-1.152232e-03) * q[i2] * q[i7] * q[i12] + (-6.230010e-04) * q[i2] * q[i7] * q[i15]
            + (6.615263e-04) * q[i2] * q[i7] * q[i16] + (-9.679730e-05) * q[i2] * q[i7] * q[i19] + (7.850361e-04) * q[i2] * q[i7] * q[i20]
            + (8.112065e-05) * q[i2] * q[i7] * q[i21] + (2.716826e-05) * q[i2] * q[i7] * q[i22] + (2.838627e-03) * q[i2] * q[i8] * q[i9]
            + (-2.816410e-03) * q[i2] * q[i8] * q[i10] + (4.622125e-03) * q[i2] * q[i8] * q[i11] + (4.647709e-03) * q[i2] * q[i8] * q[i12]
            + (3.739850e-04) * q[i2] * q[i8] * q[i15] + (-4.028890e-04) * q[i2] * q[i8] * q[i16] + (-1.238277e-04) * q[i2] * q[i8] * q[i19]
            + (1.423855e-04) * q[i2] * q[i8] * q[i20] + (4.697497e-04) * q[i2] * q[i8] * q[i21] + (4.659396e-04) * q[i2] * q[i8] * q[i22]
            + (-1.552921e-06) * q[i2] * q[i9] * q[i10] + (6.676637e-05) * q[i2] * q[i9] * q[i11] + (9.903040e-05) * q[i2] * q[i9] * q[i12]
            + (-1.966923e-04) * q[i2] * q[i9] * q[i15] + (-1.519043e-04) * q[i2] * q[i9] * q[i16] + (-5.151900e-04) * q[i2] * q[i9] * q[i19]
            + (5.072482e-05) * q[i2] * q[i9] * q[i20] + (8.193177e-06) * q[i2] * q[i9] * q[i21] + (1.973894e-05) * q[i2] * q[i9] * q[i22]
            + (8.644235e-05) * q[i2] * q[i10] * q[i11] + (5.846824e-05) * q[i2] * q[i10] * q[i12] + (1.523461e-04) * q[i2] * q[i10] * q[i15]
            + (1.977923e-04) * q[i2] * q[i10] * q[i16] + (-5.029466e-05) * q[i2] * q[i10] * q[i19] + (5.113233e-04) * q[i2] * q[i10] * q[i20]
            + (1.997862e-05) * q[i2] * q[i10] * q[i21] + (9.079683e-06) * q[i2] * q[i10] * q[i22] + (5.027884e-06) * q[i2] * q[i11] * q[i12]
            + (-2.437054e-03) * q[i2] * q[i11] * q[i15] + (-6.090809e-04) * q[i2] * q[i11] * q[i16] + (1.210145e-03) * q[i2] * q[i11] * q[i19]
            + (-3.686003e-04) * q[i2] * q[i11] * q[i20] + (4.179207e-04) * q[i2] * q[i11] * q[i21] + (-2.085002e-05) * q[i2] * q[i11] * q[i22]
            + (-6.126193e-04) * q[i2] * q[i12] * q[i15] + (-2.427449e-03) * q[i2] * q[i12] * q[i16] + (-3.721341e-04) * q[i2] * q[i12] * q[i19]
            + (1.194359e-03) * q[i2] * q[i12] * q[i20] + (1.978979e-05) * q[i2] * q[i12] * q[i21] + (-4.119226e-04) * q[i2] * q[i12] * q[i22]
            + (-1.065679e-06) * q[i2] * q[i15] * q[i16] + (8.714605e-05) * q[i2] * q[i15] * q[i19] + (-9.551340e-05) * q[i2] * q[i15] * q[i20]
            + (6.314654e-04) * q[i2] * q[i15] * q[i21] + (1.855685e-04) * q[i2] * q[i15] * q[i22] + (9.087750e-05) * q[i2] * q[i16] * q[i19]
            + (-6.702515e-05) * q[i2] * q[i16] * q[i20] + (1.824016e-04) * q[i2] * q[i16] * q[i21] + (6.283451e-04) * q[i2] * q[i16] * q[i22]
            + (4.114824e-06) * q[i2] * q[i19] * q[i20] + (-2.210598e-04) * q[i2] * q[i19] * q[i21] + (-4.203312e-04) * q[i2] * q[i19] * q[i22]
            + (-4.167799e-04) * q[i2] * q[i20] * q[i21] + (-2.152634e-04) * q[i2] * q[i20] * q[i22] + (-6.924897e-07) * q[i2] * q[i21] * q[i22]
            + (1.721855e-05) * q[i3] * q[i4] * q[i5] + (-4.119390e-03) * q[i3] * q[i4] * q[i6] + (-4.095398e-03) * q[i3] * q[i4] * q[i7]
            + (-2.612025e-03) * q[i3] * q[i4] * q[i8] + (-1.389606e-03) * q[i3] * q[i4] * q[i9] + (-1.383767e-03) * q[i3] * q[i4] * q[i10]
            + (1.854835e-04) * q[i3] * q[i4] * q[i11] + (-2.100593e-04) * q[i3] * q[i4] * q[i12] + (-1.056834e-04) * q[i3] * q[i4] * q[i15]
            + (-1.009872e-04) * q[i3] * q[i4] * q[i16] + (4.109163e-04) * q[i3] * q[i4] * q[i19] + (4.027741e-04) * q[i3] * q[i4] * q[i20]
            + (-1.784566e-04) * q[i3] * q[i4] * q[i21] + (1.765279e-04) * q[i3] * q[i4] * q[i22] + (-5.860166e-03) * q[i3] * q[i5] * q[i6]
            + (-3.410167e-04) * q[i3] * q[i5] * q[i7] + (-5.676358e-03) * q[i3] * q[i5] * q[i8] + (-9.961027e-04) * q[i3] * q[i5] * q[i9]
            + (-6.296811e-04) * q[i3] * q[i5] * q[i10] + (-6.531907e-04) * q[i3] * q[i5] * q[i11] + (-4.466833e-04) * q[i3] * q[i5] * q[i12]
            + (2.362346e-04) * q[i3] * q[i5] * q[i15] + (5.074723e-05) * q[i3] * q[i5] * q[i16] + (1.175286e-04) * q[i3] * q[i5] * q[i19]
            + (-8.921760e-04) * q[i3] * q[i5] * q[i20] + (-2.468295e-04) * q[i3] * q[i5] * q[i21] + (-4.020164e-04) * q[i3] * q[i5] * q[i22]
            + (1.879310e-03) * q[i3] * q[i6] * q[i7] + (8.413058e-04) * q[i3] * q[i6] * q[i8] + (7.022871e-04) * q[i3] * q[i6] * q[i9]
            + (-4.007765e-04) * q[i3] * q[i6] * q[i10] + (-4.024529e-04) * q[i3] * q[i6] * q[i11] + (-1.035896e-03) * q[i3] * q[i6] * q[i12]
            + (-9.391136e-04) * q[i3] * q[i6] * q[i15] + (-1.784537e-05) * q[i3] * q[i6] * q[i16] + (5.723820e-05) * q[i3] * q[i6] * q[i19]
            + (5.647931e-04) * q[i3] * q[i6] * q[i20] + (-2.184872e-05) * q[i3] * q[i6] * q[i21] + (-4.243971e-04) * q[i3] * q[i6] * q[i22]
            + (2.840954e-04) * q[i3] * q[i7] * q[i8] + (-1.541094e-03) * q[i3] * q[i7] * q[i9] + (-1.745114e-03) * q[i3] * q[i7] * q[i10]
            + (-3.231969e-06) * q[i3] * q[i7] * q[i11] + (-4.333243e-04) * q[i3] * q[i7] * q[i12] + (9.098021e-04) * q[i3] * q[i7] * q[i15]
            + (8.037877e-04) * q[i3] * q[i7] * q[i16] + (-8.243394e-04) * q[i3] * q[i7] * q[i19] + (3.347674e-04) * q[i3] * q[i7] * q[i20]
            + (-5.588223e-04) * q[i3] * q[i7] * q[i21] + (4.399826e-04) * q[i3] * q[i7] * q[i22] + (-9.853283e-04) * q[i3] * q[i8] * q[i9]
            + (9.389851e-04) * q[i3] * q[i8] * q[i10] + (8.302458e-04) * q[i3] * q[i8] * q[i11] + (3.707437e-04) * q[i3] * q[i8] * q[i12]
            + (-1.874093e-03) * q[i3] * q[i8] * q[i15] + (1.586166e-03) * q[i3] * q[i8] * q[i16] + (-1.489504e-04) * q[i3] * q[i8] * q[i19]
            + (-5.408535e-05) * q[i3] * q[i8] * q[i20] + (7.900402e-05) * q[i3] * q[i8] * q[i21] + (4.263331e-04) * q[i3] * q[i8] * q[i22]
            + (-4.769213e-04) * q[i3] * q[i9] * q[i10] + (3.395112e-04) * q[i3] * q[i9] * q[i11] + (3.349317e-04) * q[i3] * q[i9] * q[i12]
            + (-1.930683e-04) * q[i3] * q[i9] * q[i15] + (-1.237888e-04) * q[i3] * q[i9] * q[i16] + (1.574528e-04) * q[i3] * q[i9] * q[i19]
            + (9.604014e-05) * q[i3] * q[i9] * q[i20] + (1.093087e-04) * q[i3] * q[i9] * q[i21] + (-4.633238e-05) * q[i3] * q[i9] * q[i22]
            + (-4.832726e-04) * q[i3] * q[i10] * q[i11] + (9.182299e-05) * q[i3] * q[i10] * q[i12] + (3.303860e-05) * q[i3] * q[i10] * q[i15]
            + (2.333993e-04) * q[i3] * q[i10] * q[i16] + (6.864690e-05) * q[i3] * q[i10] * q[i19] + (2.429382e-04) * q[i3] * q[i10] * q[i20]
            + (-1.674627e-04) * q[i3] * q[i10] * q[i21] + (-9.695850e-05) * q[i3] * q[i10] * q[i22] + (-2.917590e-04) * q[i3] * q[i11] * q[i12]
            + (-1.024490e-03) * q[i3] * q[i11] * q[i15] + (-5.020910e-05) * q[i3] * q[i11] * q[i16] + (-4.190742e-05) * q[i3] * q[i11] * q[i19]
            + (1.218354e-04) * q[i3] * q[i11] * q[i20] + (-7.313836e-04) * q[i3] * q[i11] * q[i21] + (-8.641867e-05) * q[i3] * q[i11] * q[i22]
            + (-2.073332e-04) * q[i3] * q[i12] * q[i15] + (-1.170318e-03) * q[i3] * q[i12] * q[i16] + (2.762868e-04) * q[i3] * q[i12] * q[i19]
            + (-4.230714e-04) * q[i3] * q[i12] * q[i20] + (-3.968007e-05) * q[i3] * q[i12] * q[i21] + (2.359377e-04) * q[i3] * q[i12] * q[i22]
            + (2.405502e-04) * q[i3] * q[i15] * q[i16] + (-1.154513e-04) * q[i3] * q[i15] * q[i19] + (8.314693e-05) * q[i3] * q[i15] * q[i20]
            + (-3.249219e-04) * q[i3] * q[i15] * q[i21] + (-2.351795e-04) * q[i3] * q[i15] * q[i22] + (-1.910909e-04) * q[i3] * q[i16] * q[i19]
            + (3.031278e-04) * q[i3] * q[i16] * q[i20] + (9.856968e-05) * q[i3] * q[i16] * q[i21] + (-2.787869e-04) * q[i3] * q[i16] * q[i22]
            + (1.303709e-04) * q[i3] * q[i19] * q[i20] + (-2.283146e-04) * q[i3] * q[i19] * q[i21] + (2.080684e-04) * q[i3] * q[i19] * q[i22]
            + (-2.150225e-04) * q[i3] * q[i20] * q[i21] + (3.425700e-04) * q[i3] * q[i20] * q[i22] + (4.145291e-04) * q[i3] * q[i21] * q[i22]
            + (-3.586758e-04) * q[i4] * q[i5] * q[i6] + (-5.851652e-03) * q[i4] * q[i5] * q[i7] + (-5.683570e-03) * q[i4] * q[i5] * q[i8]
            + (-6.389476e-04) * q[i4] * q[i5] * q[i9] + (-9.861556e-04) * q[i4] * q[i5] * q[i10] + (4.378300e-04) * q[i4] * q[i5] * q[i11]
            + (6.221890e-04) * q[i4] * q[i5] * q[i12] + (5.443202e-05) * q[i4] * q[i5] * q[i15] + (2.385429e-04) * q[i4] * q[i5] * q[i16]
            + (-9.016316e-04) * q[i4] * q[i5] * q[i19] + (1.081047e-04) * q[i4] * q[i5] * q[i20] + (4.077822e-04) * q[i4] * q[i5] * q[i21]
            + (2.557705e-04) * q[i4] * q[i5] * q[i22] + (-1.896358e-03) * q[i4] * q[i6] * q[i7] + (-2.832570e-04) * q[i4] * q[i6] * q[i8]
            + (1.757509e-03) * q[i4] * q[i6] * q[i9] + (1.530174e-03) * q[i4] * q[i6] * q[i10] + (-4.370129e-04) * q[i4] * q[i6] * q[i11]
            + (2.431725e-06) * q[i4] * q[i6] * q[i12] + (-7.902417e-04) * q[i4] * q[i6] * q[i15] + (-9.180732e-04) * q[i4] * q[i6] * q[i16]
            + (-3.333828e-04) * q[i4] * q[i6] * q[i19] + (8.223252e-04) * q[i4] * q[i6] * q[i20] + (4.330354e-04) * q[i4] * q[i6] * q[i21]
            + (-5.600775e-04) * q[i4] * q[i6] * q[i22] + (-8.515030e-04) * q[i4] * q[i7] * q[i8] + (3.995365e-04) * q[i4] * q[i7] * q[i9]
            + (-6.864024e-04) * q[i4] * q[i7] * q[i10] + (-1.038503e-03) * q[i4] * q[i7] * q[i11] + (-4.192667e-04) * q[i4] * q[i7] * q[i12]
            + (2.146712e-05) * q[i4] * q[i7] * q[i15] + (9.376283e-04) * q[i4] * q[i7] * q[i16] + (-5.666982e-04) * q[i4] * q[i7] * q[i19]
            + (-5.485476e-05) * q[i4] * q[i7] * q[i20] + (-4.266081e-04) * q[i4] * q[i7] * q[i21] + (-2.061593e-05) * q[i4] * q[i7] * q[i22]
            + (-9.450100e-04) * q[i4] * q[i8] * q[i9] + (9.708365e-04) * q[i4] * q[i8] * q[i10] + (3.666690e-04) * q[i4] * q[i8] * q[i11]
            + (8.563020e-04) * q[i4] * q[i8] * q[i12] + (-1.573262e-03) * q[i4] * q[i8] * q[i15] + (1.892670e-03) * q[i4] * q[i8] * q[i16]
            + (5.975865e-05) * q[i4] * q[i8] * q[i19] + (1.447929e-04) * q[i4] * q[i8] * q[i20] + (4.272613e-04) * q[i4] * q[i8] * q[i21]
            + (8.697111e-05) * q[i4] * q[i8] * q[i22] + (4.768357e-04) * q[i4] * q[i9] * q[i10] + (8.569621e-05) * q[i4] * q[i9] * q[i11]
            + (-4.901619e-04) * q[i4] * q[i9] * q[i12] + (-2.303278e-04) * q[i4] * q[i9] * q[i15] + (-3.853204e-05) * q[i4] * q[i9] * q[i16]
            + (-2.364868e-04) * q[i4] * q[i9] * q[i19] + (-6.932773e-05) * q[i4] * q[i9] * q[i20] + (-9.635595e-05) * q[i4] * q[i9] * q[i21]
            + (-1.695343e-04) * q[i4] * q[i9] * q[i22] + (3.251140e-04) * q[i4] * q[i10] * q[i11] + (3.369470e-04) * q[i4] * q[i10] * q[i12]
            + (1.181967e-04) * q[i4] * q[i10] * q[i15] + (1.877344e-04) * q[i4] * q[i10] * q[i16] + (-9.532546e-05) * q[i4] * q[i10] * q[i19]
            + (-1.563696e-04) * q[i4] * q[i10] * q[i20] + (-4.528867e-05) * q[i4] * q[i10] * q[i21] + (1.107078e-04) * q[i4] * q[i10] * q[i22]
            + (2.936496e-04) * q[i4] * q[i11] * q[i12] + (-1.160440e-03) * q[i4] * q[i11] * q[i15] + (-2.101134e-04) * q[i4] * q[i11] * q[i16]
            + (-4.214670e-04) * q[i4] * q[i11] * q[i19] + (2.725329e-04) * q[i4] * q[i11] * q[i20] + (-2.377437e-04) * q[i4] * q[i11] * q[i21]
            + (3.716635e-05) * q[i4] * q[i11] * q[i22] + (-4.993484e-05) * q[i4] * q[i12] * q[i15] + (-1.028767e-03) * q[i4] * q[i12] * q[i16]
            + (1.145429e-04) * q[i4] * q[i12] * q[i19] + (-3.911072e-05) * q[i4] * q[i12] * q[i20] + (9.036042e-05) * q[i4] * q[i12] * q[i21]
            + (7.318424e-04) * q[i4] * q[i12] * q[i22] + (-2.399481e-04) * q[i4] * q[i15] * q[i16] + (-3.102687e-04) * q[i4] * q[i15] * q[i19]
            + (1.892475e-04) * q[i4] * q[i15] * q[i20] + (-2.721388e-04) * q[i4] * q[i15] * q[i21] + (9.849923e-05) * q[i4] * q[i15] * q[i22]
            + (-8.271905e-05) * q[i4] * q[i16] * q[i19] + (1.078302e-04) * q[i4] * q[i16] * q[i20] + (-2.327941e-04) * q[i4] * q[i16] * q[i21]
            + (-3.290808e-04) * q[i4] * q[i16] * q[i22] + (-1.338756e-04) * q[i4] * q[i19] * q[i20] + (3.439801e-04) * q[i4] * q[i19] * q[i21]
            + (-2.058941e-04) * q[i4] * q[i19] * q[i22] + (2.046500e-04) * q[i4] * q[i20] * q[i21] + (-2.295565e-04) * q[i4] * q[i20] * q[i22]
            + (-4.134249e-04) * q[i4] * q[i21] * q[i22] + (5.752728e-06) * q[i5] * q[i6] * q[i7] + (1.156394e-03) * q[i5] * q[i6] * q[i8]
            + (2.235833e-03) * q[i5] * q[i6] * q[i9] + (-1.380465e-03) * q[i5] * q[i6] * q[i10] + (-5.280932e-04) * q[i5] * q[i6] * q[i11]
            + (-9.891163e-04) * q[i5] * q[i6] * q[i12] + (1.541645e-03) * q[i5] * q[i6] * q[i15] + (2.314176e-04) * q[i5] * q[i6] * q[i16]
            + (1.809299e-04) * q[i5] * q[i6] * q[i19] + (-3.780468e-04) * q[i5] * q[i6] * q[i20] + (-4.964292e-05) * q[i5] * q[i6] * q[i21]
            + (8.301751e-05) * q[i5] * q[i6] * q[i22] + (-1.156110e-03) * q[i5] * q[i7] * q[i8] + (1.387223e-03) * q[i5] * q[i7] * q[i9]
            + (-2.220273e-03) * q[i5] * q[i7] * q[i10] + (-1.002169e-03) * q[i5] * q[i7] * q[i11] + (-5.294600e-04) * q[i5] * q[i7] * q[i12]
            + (-2.394354e-04) * q[i5] * q[i7] * q[i15] + (-1.548307e-03) * q[i5] * q[i7] * q[i16] + (3.610782e-04) * q[i5] * q[i7] * q[i19]
            + (-1.750359e-04) * q[i5] * q[i7] * q[i20] + (7.739277e-05) * q[i5] * q[i7] * q[i21] + (-4.801370e-05) * q[i5] * q[i7] * q[i22]
            + (1.313991e-03) * q[i5] * q[i8] * q[i9] + (-1.302686e-03) * q[i5] * q[i8] * q[i10] + (5.523023e-04) * q[i5] * q[i8] * q[i11]
            + (5.779302e-04) * q[i5] * q[i8] * q[i12] + (1.110366e-03) * q[i5] * q[i8] * q[i15] + (-1.141076e-03) * q[i5] * q[i8] * q[i16]
            + (-4.558482e-04) * q[i5] * q[i8] * q[i19] + (4.501319e-04) * q[i5] * q[i8] * q[i20] + (1.165022e-04) * q[i5] * q[i8] * q[i21]
            + (1.116974e-04) * q[i5] * q[i8] * q[i22] + (7.806473e-07) * q[i5] * q[i9] * q[i10] + (-3.886678e-04) * q[i5] * q[i9] * q[i11]
            + (-3.133345e-04) * q[i5] * q[i9] * q[i12] + (7.187954e-04) * q[i5] * q[i9] * q[i15] + (-1.300566e-04) * q[i5] * q[i9] * q[i16]
            + (2.208338e-04) * q[i5] * q[i9] * q[i19] + (-1.474719e-04) * q[i5] * q[i9] * q[i20] + (-5.418967e-04) * q[i5] * q[i9] * q[i21]
            + (-2.203100e-04) * q[i5] * q[i9] * q[i22] + (-3.202338e-04) * q[i5] * q[i10] * q[i11] + (-3.828427e-04) * q[i5] * q[i10] * q[i12]
            + (1.317935e-04) * q[i5] * q[i10] * q[i15] + (-7.160931e-04) * q[i5] * q[i10] * q[i16] + (1.491292e-04) * q[i5] * q[i10] * q[i19]
            + (-2.241152e-04) * q[i5] * q[i10] * q[i20] + (-2.214388e-04) * q[i5] * q[i10] * q[i21] + (-5.331750e-04) * q[i5] * q[i10] * q[i22]
            + (1.470583e-05) * q[i5] * q[i11] * q[i12] + (-2.237288e-03) * q[i5] * q[i11] * q[i15] + (-2.229771e-05) * q[i5] * q[i11] * q[i16]
            + (2.494105e-04) * q[i5] * q[i11] * q[i19] + (7.149178e-05) * q[i5] * q[i11] * q[i20] + (-2.693622e-04) * q[i5] * q[i11] * q[i21]
            + (1.497267e-05) * q[i5] * q[i11] * q[i22] + (-1.129334e-05) * q[i5] * q[i12] * q[i15] + (-2.231427e-03) * q[i5] * q[i12] * q[i16]
            + (6.981440e-05) * q[i5] * q[i12] * q[i19] + (2.474242e-04) * q[i5] * q[i12] * q[i20] + (-1.618050e-05) * q[i5] * q[i12] * q[i21]
            + (2.787426e-04) * q[i5] * q[i12] * q[i22] + (-3.547791e-06) * q[i5] * q[i15] * q[i16] + (-6.461589e-04) * q[i5] * q[i15] * q[i19]
            + (-2.655420e-04) * q[i5] * q[i15] * q[i20] + (7.849938e-04) * q[i5] * q[i15] * q[i21] + (2.021711e-04) * q[i5] * q[i15] * q[i22]
            + (2.734731e-04) * q[i5] * q[i16] * q[i19] + (6.321716e-04) * q[i5] * q[i16] * q[i20] + (2.034900e-04) * q[i5] * q[i16] * q[i21]
            + (7.961114e-04) * q[i5] * q[i16] * q[i22] + (4.417112e-06) * q[i5] * q[i19] * q[i20] + (6.504661e-04) * q[i5] * q[i19] * q[i21]
            + (-1.864600e-04) * q[i5] * q[i19] * q[i22] + (-1.819648e-04) * q[i5] * q[i20] * q[i21] + (6.460300e-04) * q[i5] * q[i20] * q[i22]
            + (-3.876848e-06) * q[i5] * q[i21] * q[i22] + (-2.277266e-03) * q[i6] * q[i7] * q[i8] + (3.240125e-04) * q[i6] * q[i7] * q[i9]
            + (3.225811e-04) * q[i6] * q[i7] * q[i10] + (-3.633376e-04) * q[i6] * q[i7] * q[i11] + (3.594762e-04) * q[i6] * q[i7] * q[i12]
            + (5.321298e-04) * q[i6] * q[i7] * q[i15] + (5.348429e-04) * q[i6] * q[i7] * q[i16] + (3.668577e-04) * q[i6] * q[i7] * q[i19]
            + (3.663251e-04) * q[i6] * q[i7] * q[i20] + (7.693029e-05) * q[i6] * q[i7] * q[i21] + (-8.031516e-05) * q[i6] * q[i7] * q[i22]
            + (-6.389555e-04) * q[i6] * q[i8] * q[i9] + (-8.771073e-04) * q[i6] * q[i8] * q[i10] + (-5.018576e-04) * q[i6] * q[i8] * q[i11]
            + (-5.343117e-05) * q[i6] * q[i8] * q[i12] + (-3.231139e-04) * q[i6] * q[i8] * q[i15] + (-3.754556e-04) * q[i6] * q[i8] * q[i16]
            + (-1.490583e-04) * q[i6] * q[i8] * q[i19] + (-1.835712e-04) * q[i6] * q[i8] * q[i20] + (-5.335340e-04) * q[i6] * q[i8] * q[i21]
            + (3.416559e-05) * q[i6] * q[i8] * q[i22] + (-6.427358e-04) * q[i6] * q[i9] * q[i10] + (-1.433083e-04) * q[i6] * q[i9] * q[i11]
            + (-7.875434e-05) * q[i6] * q[i9] * q[i12] + (-2.687738e-04) * q[i6] * q[i9] * q[i15] + (-4.103087e-04) * q[i6] * q[i9] * q[i16]
            + (1.042731e-04) * q[i6] * q[i9] * q[i19] + (-6.404165e-05) * q[i6] * q[i9] * q[i20] + (-1.674403e-04) * q[i6] * q[i9] * q[i21]
            + (-2.159308e-04) * q[i6] * q[i9] * q[i22] + (-1.313475e-04) * q[i6] * q[i10] * q[i11] + (-1.012554e-05) * q[i6] * q[i10] * q[i12]
            + (-5.974720e-05) * q[i6] * q[i10] * q[i15] + (2.814061e-04) * q[i6] * q[i10] * q[i16] + (-1.477668e-04) * q[i6] * q[i10] * q[i19]
            + (5.322371e-05) * q[i6] * q[i10] * q[i20] + (-2.659831e-04) * q[i6] * q[i10] * q[i21] + (-9.791022e-05) * q[i6] * q[i10] * q[i22]
            + (-2.782700e-04) * q[i6] * q[i11] * q[i12] + (-1.034194e-04) * q[i6] * q[i11] * q[i15] + (5.382410e-05) * q[i6] * q[i11] * q[i16]
            + (-2.237864e-04) * q[i6] * q[i11] * q[i19] + (5.266348e-05) * q[i6] * q[i11] * q[i20] + (-1.794102e-04) * q[i6] * q[i11] * q[i21]
            + (2.074214e-04) * q[i6] * q[i11] * q[i22] + (-7.543038e-05) * q[i6] * q[i12] * q[i15] + (4.898942e-05) * q[i6] * q[i12] * q[i16]
            + (-1.779052e-04) * q[i6] * q[i12] * q[i19] + (2.987062e-04) * q[i6] * q[i12] * q[i20] + (-3.023907e-05) * q[i6] * q[i12] * q[i21]
            + (-2.987766e-04) * q[i6] * q[i12] * q[i22] + (-2.468928e-05) * q[i6] * q[i15] * q[i16] + (-2.205203e-04) * q[i6] * q[i15] * q[i19]
            + (9.431270e-05) * q[i6] * q[i15] * q[i20] + (3.049960e-04) * q[i6] * q[i15] * q[i21] + (-4.658677e-05) * q[i6] * q[i15] * q[i22]
            + (-7.431485e-05) * q[i6] * q[i16] * q[i19] + (-1.600898e-04) * q[i6] * q[i16] * q[i20] + (-2.786086e-04) * q[i6] * q[i16] * q[i21]
            + (-4.603793e-04) * q[i6] * q[i16] * q[i22] + (-4.335967e-05) * q[i6] * q[i19] * q[i20] + (2.546358e-04) * q[i6] * q[i19] * q[i21]
            + (2.654591e-05) * q[i6] * q[i19] * q[i22] + (4.148696e-05) * q[i6] * q[i20] * q[i21] + (-2.644389e-04) * q[i6] * q[i20] * q[i22]
            + (-1.885125e-05) * q[i6] * q[i21] * q[i22] + (-8.894445e-04) * q[i7] * q[i8] * q[i9] + (-6.230244e-04) * q[i7] * q[i8] * q[i10]
            + (8.977087e-05) * q[i7] * q[i8] * q[i11] + (5.125980e-04) * q[i7] * q[i8] * q[i12] + (-3.710050e-04) * q[i7] * q[i8] * q[i15]
            + (-3.285802e-04) * q[i7] * q[i8] * q[i16] + (-1.816022e-04) * q[i7] * q[i8] * q[i19] + (-1.430783e-04) * q[i7] * q[i8] * q[i20]
            + (-3.633953e-05) * q[i7] * q[i8] * q[i21] + (5.422506e-04) * q[i7] * q[i8] * q[i22] + (-6.463066e-04) * q[i7] * q[i9] * q[i10]
            + (8.946800e-06) * q[i7] * q[i9] * q[i11] + (1.347187e-04) * q[i7] * q[i9] * q[i12] + (2.835824e-04) * q[i7] * q[i9] * q[i15]
            + (-5.507066e-05) * q[i7] * q[i9] * q[i16] + (5.549223e-05) * q[i7] * q[i9] * q[i19] + (-1.455974e-04) * q[i7] * q[i9] * q[i20]
            + (9.839484e-05) * q[i7] * q[i9] * q[i21] + (2.637625e-04) * q[i7] * q[i9] * q[i22] + (6.824602e-05) * q[i7] * q[i10] * q[i11]
            + (1.358580e-04) * q[i7] * q[i10] * q[i12] + (-4.023069e-04) * q[i7] * q[i10] * q[i15] + (-2.686026e-04) * q[i7] * q[i10] * q[i16]
            + (-6.344870e-05) * q[i7] * q[i10] * q[i19] + (1.039055e-04) * q[i7] * q[i10] * q[i20] + (2.147215e-04) * q[i7] * q[i10] * q[i21]
            + (1.668709e-04) * q[i7] * q[i10] * q[i22] + (-2.844112e-04) * q[i7] * q[i11] * q[i12] + (-5.275637e-05) * q[i7] * q[i11] * q[i15]
            + (8.334291e-05) * q[i7] * q[i11] * q[i16] + (-3.031969e-04) * q[i7] * q[i11] * q[i19] + (1.806968e-04) * q[i7] * q[i11] * q[i20]
            + (-3.003335e-04) * q[i7] * q[i11] * q[i21] + (-3.141856e-05) * q[i7] * q[i11] * q[i22] + (-5.183897e-05) * q[i7] * q[i12] * q[i15]
            + (1.050108e-04) * q[i7] * q[i12] * q[i16] + (-5.069996e-05) * q[i7] * q[i12] * q[i19] + (2.194537e-04) * q[i7] * q[i12] * q[i20]
            + (2.042993e-04) * q[i7] * q[i12] * q[i21] + (-1.771800e-04) * q[i7] * q[i12] * q[i22] + (-2.721343e-05) * q[i7] * q[i15] * q[i16]
            + (-1.581387e-04) * q[i7] * q[i15] * q[i19] + (-7.403503e-05) * q[i7] * q[i15] * q[i20] + (4.517717e-04) * q[i7] * q[i15] * q[i21]
            + (2.796963e-04) * q[i7] * q[i15] * q[i22] + (9.455737e-05) * q[i7] * q[i16] * q[i19] + (-2.170218e-04) * q[i7] * q[i16] * q[i20]
            + (4.784558e-05) * q[i7] * q[i16] * q[i21] + (-3.036729e-04) * q[i7] * q[i16] * q[i22] + (-4.366264e-05) * q[i7] * q[i19] * q[i20]
            + (2.590795e-04) * q[i7] * q[i19] * q[i21] + (-4.059630e-05) * q[i7] * q[i19] * q[i22] + (-2.406731e-05) * q[i7] * q[i20] * q[i21]
            + (-2.534018e-04) * q[i7] * q[i20] * q[i22] + (-1.582395e-05) * q[i7] * q[i21] * q[i22] + (3.466968e-04) * q[i8] * q[i9] * q[i10]
            + (-5.878640e-04) * q[i8] * q[i9] * q[i11] + (-1.250041e-04) * q[i8] * q[i9] * q[i12] + (-2.177621e-04) * q[i8] * q[i9] * q[i15]
            + (-5.823431e-04) * q[i8] * q[i9] * q[i16] + (4.886623e-06) * q[i8] * q[i9] * q[i19] + (6.850970e-05) * q[i8] * q[i9] * q[i20]
            + (-1.462135e-04) * q[i8] * q[i9] * q[i21] + (-1.807617e-04) * q[i8] * q[i9] * q[i22] + (1.243089e-04) * q[i8] * q[i10] * q[i11]
            + (5.872017e-04) * q[i8] * q[i10] * q[i12] + (-5.728400e-04) * q[i8] * q[i10] * q[i15] + (-2.213568e-04) * q[i8] * q[i10] * q[i16]
            + (7.077076e-05) * q[i8] * q[i10] * q[i19] + (5.437111e-06) * q[i8] * q[i10] * q[i20] + (1.820278e-04) * q[i8] * q[i10] * q[i21]
            + (1.433013e-04) * q[i8] * q[i10] * q[i22] + (-2.186374e-04) * q[i8] * q[i11] * q[i12] + (-6.202402e-04) * q[i8] * q[i11] * q[i15]
            + (-2.278058e-04) * q[i8] * q[i11] * q[i16] + (4.967597e-04) * q[i8] * q[i11] * q[i19] + (1.391907e-04) * q[i8] * q[i11] * q[i20]
            + (7.869919e-06) * q[i8] * q[i11] * q[i21] + (-7.596144e-05) * q[i8] * q[i11] * q[i22] + (2.198356e-04) * q[i8] * q[i12] * q[i15]
            + (6.076149e-04) * q[i8] * q[i12] * q[i16] + (-1.399631e-04) * q[i8] * q[i12] * q[i19] + (-4.876793e-04) * q[i8] * q[i12] * q[i20]
            + (-7.232507e-05) * q[i8] * q[i12] * q[i21] + (1.357240e-06) * q[i8] * q[i12] * q[i22] + (-2.887256e-05) * q[i8] * q[i15] * q[i16]
            + (2.942123e-04) * q[i8] * q[i15] * q[i19] + (1.343018e-05) * q[i8] * q[i15] * q[i20] + (-8.182625e-04) * q[i8] * q[i15] * q[i21]
            + (7.254698e-05) * q[i8] * q[i15] * q[i22] + (1.246325e-05) * q[i8] * q[i16] * q[i19] + (2.942755e-04) * q[i8] * q[i16] * q[i20]
            + (-7.242451e-05) * q[i8] * q[i16] * q[i21] + (8.305395e-04) * q[i8] * q[i16] * q[i22] + (-2.221943e-04) * q[i8] * q[i19] * q[i20]
            + (-5.539763e-04) * q[i8] * q[i19] * q[i21] + (-3.164284e-05) * q[i8] * q[i19] * q[i22] + (3.068988e-05) * q[i8] * q[i20] * q[i21]
            + (5.498027e-04) * q[i8] * q[i20] * q[i22] + (-1.903642e-04) * q[i8] * q[i21] * q[i22] + (4.886757e-05) * q[i9] * q[i10] * q[i11]
            + (-4.815101e-05) * q[i9] * q[i10] * q[i12] + (6.594137e-05) * q[i9] * q[i10] * q[i15] + (6.633137e-05) * q[i9] * q[i10] * q[i16]
            + (-3.746738e-05) * q[i9] * q[i10] * q[i19] + (-3.693607e-05) * q[i9] * q[i10] * q[i20] + (-6.211473e-05) * q[i9] * q[i10] * q[i21]
            + (6.165970e-05) * q[i9] * q[i10] * q[i22] + (1.294106e-04) * q[i9] * q[i11] * q[i12] + (3.956050e-06) * q[i9] * q[i11] * q[i15]
            + (1.615459e-04) * q[i9] * q[i11] * q[i16] + (-2.660218e-04) * q[i9] * q[i11] * q[i19] + (2.055716e-04) * q[i9] * q[i11] * q[i20]
            + (-5.655255e-05) * q[i9] * q[i11] * q[i21] + (1.690360e-04) * q[i9] * q[i11] * q[i22] + (1.577575e-04) * q[i9] * q[i12] * q[i15]
            + (5.806691e-04) * q[i9] * q[i12] * q[i16] + (-6.097590e-05) * q[i9] * q[i12] * q[i19] + (5.071179e-05) * q[i9] * q[i12] * q[i20]
            + (-7.381588e-05) * q[i9] * q[i12] * q[i21] + (-1.232102e-04) * q[i9] * q[i12] * q[i22] + (-4.069744e-05) * q[i9] * q[i15] * q[i16]
            + (-5.244398e-05) * q[i9] * q[i15] * q[i19] + (8.511117e-05) * q[i9] * q[i15] * q[i20] + (8.364882e-05) * q[i9] * q[i15] * q[i21]
            + (1.024389e-04) * q[i9] * q[i15] * q[i22] + (3.970330e-05) * q[i9] * q[i16] * q[i19] + (-2.986617e-05) * q[i9] * q[i16] * q[i20]
            + (-4.268491e-05) * q[i9] * q[i16] * q[i21] + (-1.365421e-04) * q[i9] * q[i16] * q[i22] + (3.019187e-05) * q[i9] * q[i19] * q[i20]
            + (6.170221e-06) * q[i9] * q[i19] * q[i21] + (7.599913e-05) * q[i9] * q[i19] * q[i22] + (-5.544617e-05) * q[i9] * q[i20] * q[i21]
            + (-1.504658e-04) * q[i9] * q[i20] * q[i22] + (-1.829503e-05) * q[i9] * q[i21] * q[i22] + (1.306873e-04) * q[i10] * q[i11] * q[i12]
            + (-5.741643e-04) * q[i10] * q[i11] * q[i15] + (-1.549752e-04) * q[i10] * q[i11] * q[i16] + (-5.359431e-05) * q[i10] * q[i11] * q[i19]
            + (6.115402e-05) * q[i10] * q[i11] * q[i20] + (-1.222902e-04) * q[i10] * q[i11] * q[i21] + (-7.195368e-05) * q[i10] * q[i11] * q[i22]
            + (-1.558511e-04) * q[i10] * q[i12] * q[i15] + (-2.731546e-06) * q[i10] * q[i12] * q[i16] + (-2.062423e-04) * q[i10] * q[i12] * q[i19]
            + (2.616168e-04) * q[i10] * q[i12] * q[i20] + (1.687286e-04) * q[i10] * q[i12] * q[i21] + (-5.460698e-05) * q[i10] * q[i12] * q[i22]
            + (-4.238001e-05) * q[i10] * q[i15] * q[i16] + (-3.179155e-05) * q[i10] * q[i15] * q[i19] + (3.929254e-05) * q[i10] * q[i15] * q[i20]
            + (1.331861e-04) * q[i10] * q[i15] * q[i21] + (4.119188e-05) * q[i10] * q[i15] * q[i22] + (8.554462e-05) * q[i10] * q[i16] * q[i19]
            + (-5.567756e-05) * q[i10] * q[i16] * q[i20] + (-1.035764e-04) * q[i10] * q[i16] * q[i21] + (-8.357313e-05) * q[i10] * q[i16] * q[i22]
            + (3.192201e-05) * q[i10] * q[i19] * q[i20] + (1.496858e-04) * q[i10] * q[i19] * q[i21] + (5.384536e-05) * q[i10] * q[i19] * q[i22]
            + (-7.447196e-05) * q[i10] * q[i20] * q[i21] + (-4.256861e-06) * q[i10] * q[i20] * q[i22] + (-1.805391e-05) * q[i10] * q[i21] * q[i22]
            + (1.660088e-05) * q[i11] * q[i12] * q[i15] + (1.707712e-05) * q[i11] * q[i12] * q[i16] + (-2.223878e-04) * q[i11] * q[i12] * q[i19]
            + (-2.200995e-04) * q[i11] * q[i12] * q[i20] + (3.935233e-06) * q[i11] * q[i12] * q[i21] + (-4.086977e-06) * q[i11] * q[i12] * q[i22]
            + (2.246258e-04) * q[i11] * q[i15] * q[i16] + (-2.731911e-04) * q[i11] * q[i15] * q[i19] + (-1.157003e-04) * q[i11] * q[i15] * q[i20]
            + (5.362887e-05) * q[i11] * q[i15] * q[i21] + (7.960830e-05) * q[i11] * q[i15] * q[i22] + (1.197865e-05) * q[i11] * q[i16] * q[i19]
            + (7.699104e-06) * q[i11] * q[i16] * q[i20] + (4.936564e-05) * q[i11] * q[i16] * q[i21] + (-1.514886e-04) * q[i11] * q[i16] * q[i22]
            + (5.760783e-05) * q[i11] * q[i19] * q[i20] + (7.847570e-05) * q[i11] * q[i19] * q[i21] + (-6.953426e-05) * q[i11] * q[i19] * q[i22]
            + (-1.020223e-04) * q[i11] * q[i20] * q[i21] + (-4.386798e-05) * q[i11] * q[i20] * q[i22] + (-2.507574e-05) * q[i11] * q[i21] * q[i22]
            + (-2.242077e-04) * q[i12] * q[i15] * q[i16] + (-9.356121e-06) * q[i12] * q[i15] * q[i19] + (-1.318325e-05) * q[i12] * q[i15] * q[i20]
            + (-1.468120e-04) * q[i12] * q[i15] * q[i21] + (4.998225e-05) * q[i12] * q[i15] * q[i22] + (1.185705e-04) * q[i12] * q[i16] * q[i19]
            + (2.802190e-04) * q[i12] * q[i16] * q[i20] + (8.148644e-05) * q[i12] * q[i16] * q[i21] + (3.854881e-05) * q[i12] * q[i16] * q[i22]
            + (-5.788815e-05) * q[i12] * q[i19] * q[i20] + (-4.247808e-05) * q[i12] * q[i19] * q[i21] + (-1.049382e-04) * q[i12] * q[i19] * q[i22]
            + (-6.778826e-05) * q[i12] * q[i20] * q[i21] + (8.675031e-05) * q[i12] * q[i20] * q[i22] + (2.284150e-05) * q[i12] * q[i21] * q[i22]
            + (7.088559e-05) * q[i15] * q[i16] * q[i19] + (6.764808e-05) * q[i15] * q[i16] * q[i20] + (1.638822e-04) * q[i15] * q[i16] * q[i21]
            + (-1.640155e-04) * q[i15] * q[i16] * q[i22] + (-9.041658e-06) * q[i15] * q[i19] * q[i20] + (-7.785493e-04) * q[i15] * q[i19] * q[i21]
            + (-6.918400e-05) * q[i15] * q[i19] * q[i22] + (7.900249e-05) * q[i15] * q[i20] * q[i21] + (-8.887574e-06) * q[i15] * q[i20] * q[i22]
            + (-7.131537e-05) * q[i15] * q[i21] * q[i22] + (-9.552562e-06) * q[i16] * q[i19] * q[i20] + (9.281681e-06) * q[i16] * q[i19] * q[i21]
            + (-7.924950e-05) * q[i16] * q[i19] * q[i22] + (7.053497e-05) * q[i16] * q[i20] * q[i21] + (7.728345e-04) * q[i16] * q[i20] * q[i22]
            + (-7.306663e-05) * q[i16] * q[i21] * q[i22] + (-2.281735e-05) * q[i19] * q[i20] * q[i21] + (2.105748e-05) * q[i19] * q[i20] * q[i22]
            + (7.189642e-05) * q[i19] * q[i21] * q[i22] + (7.142724e-05) * q[i20] * q[i21] * q[i22];

      return Qy;
   }

   public double getQz(double[] q)
   {
      double Qz;
      Qz = (-1.109815e-01) * q[i0] + (-1.105000e-01) * q[i1] + (1.993570e-01) * q[i2] + (1.557260e-02) * q[i3] + (1.556921e-02) * q[i4]
            + (-1.256203e-02) * q[i5] + (1.076226e-01) * q[i6] + (-1.073752e-01) * q[i7] + (-2.310491e-04) * q[i8] + (3.732829e-02) * q[i9]
            + (-3.705800e-02) * q[i10] + (4.345893e-02) * q[i11] + (4.368614e-02) * q[i12] + (8.570411e-03) * q[i15] + (-8.773902e-03) * q[i16]
            + (8.838663e-03) * q[i19] + (-8.644365e-03) * q[i20] + (8.061264e-03) * q[i21] + (8.054589e-03) * q[i22] + (2.186380e-03) * q[i0] * q[i0]
            + (-2.156062e-03) * q[i1] * q[i1] + (-1.236494e-06) * q[i2] * q[i2] + (-2.297659e-03) * q[i3] * q[i3] + (2.283697e-03) * q[i4] * q[i4]
            + (-2.374712e-05) * q[i5] * q[i5] + (3.803345e-03) * q[i6] * q[i6] + (-3.797777e-03) * q[i7] * q[i7] + (7.957113e-05) * q[i8] * q[i8]
            + (-3.575926e-03) * q[i9] * q[i9] + (3.552622e-03) * q[i10] * q[i10] + (5.601448e-05) * q[i11] * q[i11] + (1.634584e-04) * q[i12] * q[i12]
            + (3.095329e-03) * q[i15] * q[i15] + (-3.092488e-03) * q[i16] * q[i16] + (-6.549195e-04) * q[i19] * q[i19] + (6.533052e-04) * q[i20] * q[i20]
            + (5.896429e-04) * q[i21] * q[i21] + (-6.045764e-04) * q[i22] * q[i22] + (1.298186e-05) * q[i0] * q[i1] + (1.331273e-03) * q[i0] * q[i2]
            + (-7.220598e-02) * q[i0] * q[i3] + (-3.559331e-02) * q[i0] * q[i4] + (-5.699832e-03) * q[i0] * q[i5] + (2.130645e-02) * q[i0] * q[i6]
            + (-4.250287e-03) * q[i0] * q[i7] + (4.216772e-03) * q[i0] * q[i8] + (-3.284959e-03) * q[i0] * q[i9] + (2.338755e-03) * q[i0] * q[i10]
            + (7.367676e-04) * q[i0] * q[i11] + (-8.091329e-05) * q[i0] * q[i12] + (-9.541826e-03) * q[i0] * q[i15] + (-9.139200e-03) * q[i0] * q[i16]
            + (1.184830e-03) * q[i0] * q[i19] + (-1.494506e-03) * q[i0] * q[i20] + (-1.608200e-03) * q[i0] * q[i21] + (-8.171787e-04) * q[i0] * q[i22]
            + (-1.305400e-03) * q[i1] * q[i2] + (3.548523e-02) * q[i1] * q[i3] + (7.220449e-02) * q[i1] * q[i4] + (5.617093e-03) * q[i1] * q[i5]
            + (-4.244224e-03) * q[i1] * q[i6] + (2.127899e-02) * q[i1] * q[i7] + (4.222046e-03) * q[i1] * q[i8] + (2.326091e-03) * q[i1] * q[i9]
            + (-3.166851e-03) * q[i1] * q[i10] + (1.854969e-04) * q[i1] * q[i11] + (-6.749129e-04) * q[i1] * q[i12] + (-9.011323e-03) * q[i1] * q[i15]
            + (-9.624126e-03) * q[i1] * q[i16] + (-1.508031e-03) * q[i1] * q[i19] + (1.164486e-03) * q[i1] * q[i20] + (7.894374e-04) * q[i1] * q[i21]
            + (1.582445e-03) * q[i1] * q[i22] + (-2.337207e-02) * q[i2] * q[i3] + (2.327494e-02) * q[i2] * q[i4] + (-7.763668e-05) * q[i2] * q[i5]
            + (9.364764e-03) * q[i2] * q[i6] + (9.353732e-03) * q[i2] * q[i7] + (9.573565e-03) * q[i2] * q[i8] + (1.805297e-03) * q[i2] * q[i9]
            + (1.865104e-03) * q[i2] * q[i10] + (1.561177e-03) * q[i2] * q[i11] + (-1.403663e-03) * q[i2] * q[i12] + (-1.970557e-02) * q[i2] * q[i15]
            + (-1.993201e-02) * q[i2] * q[i16] + (1.275394e-03) * q[i2] * q[i19] + (1.266208e-03) * q[i2] * q[i20] + (6.967254e-04) * q[i2] * q[i21]
            + (-7.466201e-04) * q[i2] * q[i22] + (-5.903601e-05) * q[i3] * q[i4] + (-2.861027e-03) * q[i3] * q[i5] + (7.357195e-03) * q[i3] * q[i6]
            + (2.883714e-02) * q[i3] * q[i7] + (-2.371876e-03) * q[i3] * q[i8] + (1.781216e-03) * q[i3] * q[i9] + (1.244188e-02) * q[i3] * q[i10]
            + (-2.343839e-05) * q[i3] * q[i11] + (8.395165e-04) * q[i3] * q[i12] + (4.887112e-03) * q[i3] * q[i15] + (1.684864e-04) * q[i3] * q[i16]
            + (-2.726175e-03) * q[i3] * q[i19] + (3.994531e-03) * q[i3] * q[i20] + (4.071305e-03) * q[i3] * q[i21] + (3.704013e-04) * q[i3] * q[i22]
            + (2.906511e-03) * q[i4] * q[i5] + (2.883853e-02) * q[i4] * q[i6] + (7.226012e-03) * q[i4] * q[i7] + (-2.295192e-03) * q[i4] * q[i8]
            + (1.254799e-02) * q[i4] * q[i9] + (1.785154e-03) * q[i4] * q[i10] + (-8.096186e-04) * q[i4] * q[i11] + (2.268258e-05) * q[i4] * q[i12]
            + (1.618154e-04) * q[i4] * q[i15] + (4.859843e-03) * q[i4] * q[i16] + (3.983855e-03) * q[i4] * q[i19] + (-2.744435e-03) * q[i4] * q[i20]
            + (-3.607526e-04) * q[i4] * q[i21] + (-4.038619e-03) * q[i4] * q[i22] + (2.228666e-02) * q[i5] * q[i6] + (2.211254e-02) * q[i5] * q[i7]
            + (4.436577e-02) * q[i5] * q[i8] + (8.223239e-03) * q[i5] * q[i9] + (8.111889e-03) * q[i5] * q[i10] + (-3.910134e-03) * q[i5] * q[i11]
            + (4.062875e-03) * q[i5] * q[i12] + (-3.040717e-03) * q[i5] * q[i15] + (-3.044449e-03) * q[i5] * q[i16] + (2.821675e-03) * q[i5] * q[i19]
            + (2.839825e-03) * q[i5] * q[i20] + (-1.049549e-03) * q[i5] * q[i21] + (1.032526e-03) * q[i5] * q[i22] + (1.644721e-06) * q[i6] * q[i7]
            + (-5.708182e-03) * q[i6] * q[i8] + (-5.801340e-03) * q[i6] * q[i9] + (-1.367266e-03) * q[i6] * q[i10] + (9.259903e-04) * q[i6] * q[i11]
            + (1.502845e-03) * q[i6] * q[i12] + (5.385031e-04) * q[i6] * q[i15] + (9.371454e-03) * q[i6] * q[i16] + (-7.744288e-04) * q[i6] * q[i19]
            + (-2.307807e-03) * q[i6] * q[i20] + (-1.719988e-03) * q[i6] * q[i21] + (-2.084573e-04) * q[i6] * q[i22] + (5.638196e-03) * q[i7] * q[i8]
            + (1.361736e-03) * q[i7] * q[i9] + (5.718622e-03) * q[i7] * q[i10] + (1.568874e-03) * q[i7] * q[i11] + (9.662553e-04) * q[i7] * q[i12]
            + (-9.219974e-03) * q[i7] * q[i15] + (-5.866709e-04) * q[i7] * q[i16] + (2.318131e-03) * q[i7] * q[i19] + (7.661167e-04) * q[i7] * q[i20]
            + (-2.265384e-04) * q[i7] * q[i21] + (-1.728481e-03) * q[i7] * q[i22] + (1.373567e-03) * q[i8] * q[i9] + (-1.386535e-03) * q[i8] * q[i10]
            + (-3.113403e-03) * q[i8] * q[i11] + (-3.324383e-03) * q[i8] * q[i12] + (1.215025e-02) * q[i8] * q[i15] + (-1.220530e-02) * q[i8] * q[i16]
            + (1.431593e-03) * q[i8] * q[i19] + (-1.407848e-03) * q[i8] * q[i20] + (3.691870e-03) * q[i8] * q[i21] + (3.724218e-03) * q[i8] * q[i22]
            + (-4.851862e-06) * q[i9] * q[i10] + (1.881100e-03) * q[i9] * q[i11] + (2.079217e-03) * q[i9] * q[i12] + (-5.601694e-04) * q[i9] * q[i15]
            + (3.954755e-04) * q[i9] * q[i16] + (-9.137787e-04) * q[i9] * q[i19] + (-1.001970e-03) * q[i9] * q[i20] + (-6.894929e-04) * q[i9] * q[i21]
            + (3.273732e-04) * q[i9] * q[i22] + (2.076850e-03) * q[i10] * q[i11] + (1.907630e-03) * q[i10] * q[i12] + (-3.640950e-04) * q[i10] * q[i15]
            + (5.470199e-04) * q[i10] * q[i16] + (1.006726e-03) * q[i10] * q[i19] + (9.125165e-04) * q[i10] * q[i20] + (3.067238e-04) * q[i10] * q[i21]
            + (-6.889521e-04) * q[i10] * q[i22] + (-4.592529e-05) * q[i11] * q[i12] + (-3.116551e-04) * q[i11] * q[i15] + (8.965799e-04) * q[i11] * q[i16]
            + (2.130627e-03) * q[i11] * q[i19] + (9.533648e-04) * q[i11] * q[i20] + (5.293681e-03) * q[i11] * q[i21] + (6.641784e-04) * q[i11] * q[i22]
            + (8.579139e-04) * q[i12] * q[i15] + (-4.133825e-04) * q[i12] * q[i16] + (9.631199e-04) * q[i12] * q[i19] + (2.098065e-03) * q[i12] * q[i20]
            + (-6.710139e-04) * q[i12] * q[i21] + (-5.392673e-03) * q[i12] * q[i22] + (3.477047e-06) * q[i15] * q[i16] + (4.399609e-03) * q[i15] * q[i19]
            + (-1.002143e-03) * q[i15] * q[i20] + (2.999294e-03) * q[i15] * q[i21] + (4.596933e-04) * q[i15] * q[i22] + (1.017425e-03) * q[i16] * q[i19]
            + (-4.327457e-03) * q[i16] * q[i20] + (4.671434e-04) * q[i16] * q[i21] + (3.021687e-03) * q[i16] * q[i22] + (3.845273e-06) * q[i19] * q[i20]
            + (-4.411577e-03) * q[i19] * q[i21] + (-8.428399e-04) * q[i19] * q[i22] + (-8.348530e-04) * q[i20] * q[i21] + (-4.439633e-03) * q[i20] * q[i22]
            + (-4.130537e-06) * q[i21] * q[i22] + (3.091546e-03) * q[i0] * q[i0] * q[i0] + (3.516880e-04) * q[i0] * q[i0] * q[i1]
            + (-1.628473e-03) * q[i0] * q[i0] * q[i2] + (-2.554699e-03) * q[i0] * q[i0] * q[i3] + (-1.489368e-03) * q[i0] * q[i0] * q[i4]
            + (-1.309733e-03) * q[i0] * q[i0] * q[i5] + (-1.297379e-02) * q[i0] * q[i0] * q[i6] + (-8.947360e-04) * q[i0] * q[i0] * q[i7]
            + (-1.421258e-03) * q[i0] * q[i0] * q[i8] + (-1.484376e-03) * q[i0] * q[i0] * q[i9] + (-1.081645e-03) * q[i0] * q[i0] * q[i10]
            + (-9.726582e-04) * q[i0] * q[i0] * q[i11] + (2.299991e-04) * q[i0] * q[i0] * q[i12] + (2.123896e-04) * q[i0] * q[i0] * q[i15]
            + (-8.236463e-04) * q[i0] * q[i0] * q[i16] + (-6.370901e-05) * q[i0] * q[i0] * q[i19] + (1.103751e-03) * q[i0] * q[i0] * q[i20]
            + (-6.953435e-05) * q[i0] * q[i0] * q[i21] + (6.636358e-05) * q[i0] * q[i0] * q[i22] + (3.066759e-04) * q[i0] * q[i1] * q[i1]
            + (3.085430e-03) * q[i1] * q[i1] * q[i1] + (-1.600676e-03) * q[i1] * q[i1] * q[i2] + (-1.489914e-03) * q[i1] * q[i1] * q[i3]
            + (-2.534778e-03) * q[i1] * q[i1] * q[i4] + (-1.299480e-03) * q[i1] * q[i1] * q[i5] + (9.132872e-04) * q[i1] * q[i1] * q[i6]
            + (1.300026e-02) * q[i1] * q[i1] * q[i7] + (1.391285e-03) * q[i1] * q[i1] * q[i8] + (1.083900e-03) * q[i1] * q[i1] * q[i9]
            + (1.475178e-03) * q[i1] * q[i1] * q[i10] + (2.308146e-04) * q[i1] * q[i1] * q[i11] + (-9.689934e-04) * q[i1] * q[i1] * q[i12]
            + (8.221723e-04) * q[i1] * q[i1] * q[i15] + (-2.117819e-04) * q[i1] * q[i1] * q[i16] + (-1.095282e-03) * q[i1] * q[i1] * q[i19]
            + (6.888944e-05) * q[i1] * q[i1] * q[i20] + (7.867434e-05) * q[i1] * q[i1] * q[i21] + (-8.686967e-05) * q[i1] * q[i1] * q[i22]
            + (2.141207e-03) * q[i0] * q[i2] * q[i2] + (2.145444e-03) * q[i1] * q[i2] * q[i2] + (-2.549352e-03) * q[i2] * q[i2] * q[i2]
            + (-6.756562e-04) * q[i2] * q[i2] * q[i3] + (-6.928833e-04) * q[i2] * q[i2] * q[i4] + (5.646531e-03) * q[i2] * q[i2] * q[i5]
            + (-2.140038e-03) * q[i2] * q[i2] * q[i6] + (2.125828e-03) * q[i2] * q[i2] * q[i7] + (-7.652074e-06) * q[i2] * q[i2] * q[i8]
            + (-1.039342e-03) * q[i2] * q[i2] * q[i9] + (1.028019e-03) * q[i2] * q[i2] * q[i10] + (-1.988575e-04) * q[i2] * q[i2] * q[i11]
            + (-1.941948e-04) * q[i2] * q[i2] * q[i12] + (-2.909282e-04) * q[i2] * q[i2] * q[i15] + (2.973533e-04) * q[i2] * q[i2] * q[i16]
            + (-5.778378e-04) * q[i2] * q[i2] * q[i19] + (5.646483e-04) * q[i2] * q[i2] * q[i20] + (-3.514461e-05) * q[i2] * q[i2] * q[i21]
            + (-3.549139e-05) * q[i2] * q[i2] * q[i22] + (-5.565556e-03) * q[i0] * q[i3] * q[i3] + (2.957839e-03) * q[i1] * q[i3] * q[i3]
            + (-3.204087e-03) * q[i2] * q[i3] * q[i3] + (1.182319e-03) * q[i3] * q[i3] * q[i3] + (1.306708e-03) * q[i3] * q[i3] * q[i4]
            + (8.595789e-04) * q[i3] * q[i3] * q[i5] + (8.180236e-03) * q[i3] * q[i3] * q[i6] + (2.244172e-03) * q[i3] * q[i3] * q[i7]
            + (4.152577e-03) * q[i3] * q[i3] * q[i8] + (1.871842e-03) * q[i3] * q[i3] * q[i9] + (-1.640236e-04) * q[i3] * q[i3] * q[i10]
            + (1.521167e-04) * q[i3] * q[i3] * q[i11] + (-1.310267e-03) * q[i3] * q[i3] * q[i12] + (-1.179013e-03) * q[i3] * q[i3] * q[i15]
            + (-9.294659e-05) * q[i3] * q[i3] * q[i16] + (1.117891e-03) * q[i3] * q[i3] * q[i19] + (-6.584318e-04) * q[i3] * q[i3] * q[i20]
            + (6.239800e-04) * q[i3] * q[i3] * q[i21] + (2.862283e-04) * q[i3] * q[i3] * q[i22] + (2.936765e-03) * q[i0] * q[i4] * q[i4]
            + (-5.573877e-03) * q[i1] * q[i4] * q[i4] + (-3.201652e-03) * q[i2] * q[i4] * q[i4] + (1.289784e-03) * q[i3] * q[i4] * q[i4]
            + (1.173673e-03) * q[i4] * q[i4] * q[i4] + (8.479721e-04) * q[i4] * q[i4] * q[i5] + (-2.258470e-03) * q[i4] * q[i4] * q[i6]
            + (-8.213598e-03) * q[i4] * q[i4] * q[i7] + (-4.135429e-03) * q[i4] * q[i4] * q[i8] + (1.618644e-04) * q[i4] * q[i4] * q[i9]
            + (-1.866949e-03) * q[i4] * q[i4] * q[i10] + (-1.317034e-03) * q[i4] * q[i4] * q[i11] + (1.310350e-04) * q[i4] * q[i4] * q[i12]
            + (9.697352e-05) * q[i4] * q[i4] * q[i15] + (1.172453e-03) * q[i4] * q[i4] * q[i16] + (6.466786e-04) * q[i4] * q[i4] * q[i19]
            + (-1.116481e-03) * q[i4] * q[i4] * q[i20] + (2.800301e-04) * q[i4] * q[i4] * q[i21] + (6.291117e-04) * q[i4] * q[i4] * q[i22]
            + (8.127943e-04) * q[i0] * q[i5] * q[i5] + (7.662143e-04) * q[i1] * q[i5] * q[i5] + (-6.251040e-03) * q[i2] * q[i5] * q[i5]
            + (-8.961304e-04) * q[i3] * q[i5] * q[i5] + (-8.772800e-04) * q[i4] * q[i5] * q[i5] + (2.084497e-04) * q[i5] * q[i5] * q[i5]
            + (-6.685046e-04) * q[i5] * q[i5] * q[i6] + (6.582352e-04) * q[i5] * q[i5] * q[i7] + (3.616921e-05) * q[i5] * q[i5] * q[i8]
            + (-1.680639e-03) * q[i5] * q[i5] * q[i9] + (1.669961e-03) * q[i5] * q[i5] * q[i10] + (1.658584e-03) * q[i5] * q[i5] * q[i11]
            + (1.766628e-03) * q[i5] * q[i5] * q[i12] + (3.443046e-03) * q[i5] * q[i5] * q[i15] + (-3.470037e-03) * q[i5] * q[i5] * q[i16]
            + (-3.159070e-04) * q[i5] * q[i5] * q[i19] + (3.362770e-04) * q[i5] * q[i5] * q[i20] + (5.841197e-04) * q[i5] * q[i5] * q[i21]
            + (5.764229e-04) * q[i5] * q[i5] * q[i22] + (-1.193443e-02) * q[i0] * q[i6] * q[i6] + (7.437457e-03) * q[i1] * q[i6] * q[i6]
            + (-5.773167e-03) * q[i2] * q[i6] * q[i6] + (-9.116238e-03) * q[i3] * q[i6] * q[i6] + (1.010245e-03) * q[i4] * q[i6] * q[i6]
            + (7.471263e-03) * q[i5] * q[i6] * q[i6] + (-4.578857e-03) * q[i6] * q[i6] * q[i6] + (5.240790e-03) * q[i6] * q[i6] * q[i7]
            + (4.802060e-03) * q[i6] * q[i6] * q[i8] + (-9.200232e-04) * q[i6] * q[i6] * q[i9] + (2.026387e-03) * q[i6] * q[i6] * q[i10]
            + (-1.200776e-03) * q[i6] * q[i6] * q[i11] + (5.813818e-04) * q[i6] * q[i6] * q[i12] + (1.371895e-03) * q[i6] * q[i6] * q[i15]
            + (7.176423e-04) * q[i6] * q[i6] * q[i16] + (-7.310683e-04) * q[i6] * q[i6] * q[i19] + (-8.019595e-04) * q[i6] * q[i6] * q[i20]
            + (-1.001196e-04) * q[i6] * q[i6] * q[i21] + (-1.661576e-03) * q[i6] * q[i6] * q[i22] + (7.442525e-03) * q[i0] * q[i7] * q[i7]
            + (-1.192737e-02) * q[i1] * q[i7] * q[i7] + (-5.751154e-03) * q[i2] * q[i7] * q[i7] + (1.026232e-03) * q[i3] * q[i7] * q[i7]
            + (-9.138727e-03) * q[i4] * q[i7] * q[i7] + (7.479982e-03) * q[i5] * q[i7] * q[i7] + (-5.244531e-03) * q[i6] * q[i7] * q[i7]
            + (4.588625e-03) * q[i7] * q[i7] * q[i7] + (-4.807721e-03) * q[i7] * q[i7] * q[i8] + (-2.036214e-03) * q[i7] * q[i7] * q[i9]
            + (9.137079e-04) * q[i7] * q[i7] * q[i10] + (5.780431e-04) * q[i7] * q[i7] * q[i11] + (-1.196945e-03) * q[i7] * q[i7] * q[i12]
            + (-7.038177e-04) * q[i7] * q[i7] * q[i15] + (-1.368118e-03) * q[i7] * q[i7] * q[i16] + (8.031525e-04) * q[i7] * q[i7] * q[i19]
            + (7.309260e-04) * q[i7] * q[i7] * q[i20] + (-1.652416e-03) * q[i7] * q[i7] * q[i21] + (-1.002680e-04) * q[i7] * q[i7] * q[i22]
            + (1.993809e-03) * q[i0] * q[i8] * q[i8] + (1.982408e-03) * q[i1] * q[i8] * q[i8] + (1.203593e-03) * q[i2] * q[i8] * q[i8]
            + (-5.386114e-04) * q[i3] * q[i8] * q[i8] + (-5.371954e-04) * q[i4] * q[i8] * q[i8] + (4.209572e-03) * q[i5] * q[i8] * q[i8]
            + (-1.768166e-03) * q[i6] * q[i8] * q[i8] + (1.768702e-03) * q[i7] * q[i8] * q[i8] + (-1.080768e-05) * q[i8] * q[i8] * q[i8]
            + (-5.393390e-04) * q[i8] * q[i8] * q[i9] + (5.418722e-04) * q[i8] * q[i8] * q[i10] + (-3.460304e-03) * q[i8] * q[i8] * q[i11]
            + (-3.547921e-03) * q[i8] * q[i8] * q[i12] + (-3.006602e-03) * q[i8] * q[i8] * q[i15] + (3.066301e-03) * q[i8] * q[i8] * q[i16]
            + (6.014736e-04) * q[i8] * q[i8] * q[i19] + (-6.122044e-04) * q[i8] * q[i8] * q[i20] + (9.352912e-04) * q[i8] * q[i8] * q[i21]
            + (9.451885e-04) * q[i8] * q[i8] * q[i22] + (3.640691e-03) * q[i0] * q[i9] * q[i9] + (-4.503962e-04) * q[i1] * q[i9] * q[i9]
            + (1.087245e-03) * q[i2] * q[i9] * q[i9] + (6.405877e-04) * q[i3] * q[i9] * q[i9] + (-1.436498e-03) * q[i4] * q[i9] * q[i9]
            + (-1.565213e-03) * q[i5] * q[i9] * q[i9] + (-2.235646e-03) * q[i6] * q[i9] * q[i9] + (-2.395308e-04) * q[i7] * q[i9] * q[i9]
            + (2.340922e-04) * q[i8] * q[i9] * q[i9] + (-9.357534e-04) * q[i9] * q[i9] * q[i9] + (1.209485e-04) * q[i9] * q[i9] * q[i10]
            + (1.281909e-05) * q[i9] * q[i9] * q[i11] + (9.194117e-05) * q[i9] * q[i9] * q[i12] + (4.820502e-05) * q[i9] * q[i9] * q[i15]
            + (-3.311503e-04) * q[i9] * q[i9] * q[i16] + (-1.175025e-04) * q[i9] * q[i9] * q[i19] + (3.687722e-04) * q[i9] * q[i9] * q[i20]
            + (1.736911e-04) * q[i9] * q[i9] * q[i21] + (-1.672486e-04) * q[i9] * q[i9] * q[i22] + (-4.561813e-04) * q[i0] * q[i10] * q[i10]
            + (3.589453e-03) * q[i1] * q[i10] * q[i10] + (1.063092e-03) * q[i2] * q[i10] * q[i10] + (-1.418208e-03) * q[i3] * q[i10] * q[i10]
            + (5.974724e-04) * q[i4] * q[i10] * q[i10] + (-1.534836e-03) * q[i5] * q[i10] * q[i10] + (2.415019e-04) * q[i6] * q[i10] * q[i10]
            + (2.215410e-03) * q[i7] * q[i10] * q[i10] + (-2.270851e-04) * q[i8] * q[i10] * q[i10] + (-1.185146e-04) * q[i9] * q[i10] * q[i10]
            + (9.230751e-04) * q[i10] * q[i10] * q[i10] + (8.811553e-05) * q[i10] * q[i10] * q[i11] + (6.403050e-06) * q[i10] * q[i10] * q[i12]
            + (3.212800e-04) * q[i10] * q[i10] * q[i15] + (-4.669194e-05) * q[i10] * q[i10] * q[i16] + (-3.655092e-04) * q[i10] * q[i10] * q[i19]
            + (1.150544e-04) * q[i10] * q[i10] * q[i20] + (-1.624001e-04) * q[i10] * q[i10] * q[i21] + (1.740462e-04) * q[i10] * q[i10] * q[i22]
            + (-5.592799e-04) * q[i0] * q[i11] * q[i11] + (1.304813e-05) * q[i1] * q[i11] * q[i11] + (-6.774947e-04) * q[i2] * q[i11] * q[i11]
            + (-5.152622e-04) * q[i3] * q[i11] * q[i11] + (-8.731155e-04) * q[i4] * q[i11] * q[i11] + (-4.852332e-04) * q[i5] * q[i11] * q[i11]
            + (1.388729e-03) * q[i6] * q[i11] * q[i11] + (-1.134499e-03) * q[i7] * q[i11] * q[i11] + (-2.875865e-03) * q[i8] * q[i11] * q[i11]
            + (6.788440e-05) * q[i9] * q[i11] * q[i11] + (-2.069287e-04) * q[i10] * q[i11] * q[i11] + (-1.067481e-03) * q[i11] * q[i11] * q[i11]
            + (3.223710e-04) * q[i11] * q[i11] * q[i12] + (1.537403e-03) * q[i11] * q[i11] * q[i15] + (-4.745409e-05) * q[i11] * q[i11] * q[i16]
            + (-9.906457e-04) * q[i11] * q[i11] * q[i19] + (5.648588e-05) * q[i11] * q[i11] * q[i20] + (-7.417927e-04) * q[i11] * q[i11] * q[i21]
            + (-7.896716e-05) * q[i11] * q[i11] * q[i22] + (1.959393e-06) * q[i0] * q[i12] * q[i12] + (-6.008980e-04) * q[i1] * q[i12] * q[i12]
            + (-7.279767e-04) * q[i2] * q[i12] * q[i12] + (-9.127081e-04) * q[i3] * q[i12] * q[i12] + (-4.720268e-04) * q[i4] * q[i12] * q[i12]
            + (-4.685750e-04) * q[i5] * q[i12] * q[i12] + (1.156984e-03) * q[i6] * q[i12] * q[i12] + (-1.382291e-03) * q[i7] * q[i12] * q[i12]
            + (2.941201e-03) * q[i8] * q[i12] * q[i12] + (2.177714e-04) * q[i9] * q[i12] * q[i12] + (-6.095788e-05) * q[i10] * q[i12] * q[i12]
            + (3.280248e-04) * q[i11] * q[i12] * q[i12] + (-1.105121e-03) * q[i12] * q[i12] * q[i12] + (4.546132e-05) * q[i12] * q[i12] * q[i15]
            + (-1.535927e-03) * q[i12] * q[i12] * q[i16] + (-5.515674e-05) * q[i12] * q[i12] * q[i19] + (9.596390e-04) * q[i12] * q[i12] * q[i20]
            + (-8.149968e-05) * q[i12] * q[i12] * q[i21] + (-7.300253e-04) * q[i12] * q[i12] * q[i22] + (-2.262076e-03) * q[i0] * q[i15] * q[i15]
            + (-2.313700e-03) * q[i1] * q[i15] * q[i15] + (-4.514467e-03) * q[i2] * q[i15] * q[i15] + (5.518776e-05) * q[i3] * q[i15] * q[i15]
            + (4.612167e-04) * q[i4] * q[i15] * q[i15] + (-2.098538e-03) * q[i5] * q[i15] * q[i15] + (1.037642e-03) * q[i6] * q[i15] * q[i15]
            + (-1.213451e-03) * q[i7] * q[i15] * q[i15] + (-2.416944e-03) * q[i8] * q[i15] * q[i15] + (1.615056e-04) * q[i9] * q[i15] * q[i15]
            + (3.116321e-04) * q[i10] * q[i15] * q[i15] + (-6.175304e-03) * q[i11] * q[i15] * q[i15] + (2.057077e-04) * q[i12] * q[i15] * q[i15]
            + (-2.868432e-04) * q[i15] * q[i15] * q[i15] + (-2.078434e-04) * q[i15] * q[i15] * q[i16] + (6.294158e-04) * q[i15] * q[i15] * q[i19]
            + (1.619278e-04) * q[i15] * q[i15] * q[i20] + (1.051880e-03) * q[i15] * q[i15] * q[i21] + (8.621996e-05) * q[i15] * q[i15] * q[i22]
            + (-2.350594e-03) * q[i0] * q[i16] * q[i16] + (-2.272204e-03) * q[i1] * q[i16] * q[i16] + (-4.559418e-03) * q[i2] * q[i16] * q[i16]
            + (4.668928e-04) * q[i3] * q[i16] * q[i16] + (5.548368e-05) * q[i4] * q[i16] * q[i16] + (-2.123313e-03) * q[i5] * q[i16] * q[i16]
            + (1.226560e-03) * q[i6] * q[i16] * q[i16] + (-1.050098e-03) * q[i7] * q[i16] * q[i16] + (2.450313e-03) * q[i8] * q[i16] * q[i16]
            + (-3.178814e-04) * q[i9] * q[i16] * q[i16] + (-1.635291e-04) * q[i10] * q[i16] * q[i16] + (2.060527e-04) * q[i11] * q[i16] * q[i16]
            + (-6.247284e-03) * q[i12] * q[i16] * q[i16] + (2.072165e-04) * q[i15] * q[i16] * q[i16] + (2.937789e-04) * q[i16] * q[i16] * q[i16]
            + (-1.587213e-04) * q[i16] * q[i16] * q[i19] + (-6.310745e-04) * q[i16] * q[i16] * q[i20] + (8.688550e-05) * q[i16] * q[i16] * q[i21]
            + (1.061528e-03) * q[i16] * q[i16] * q[i22] + (1.288298e-04) * q[i0] * q[i19] * q[i19] + (-3.503810e-04) * q[i1] * q[i19] * q[i19]
            + (-9.231063e-06) * q[i2] * q[i19] * q[i19] + (-4.202966e-04) * q[i3] * q[i19] * q[i19] + (-2.580627e-04) * q[i4] * q[i19] * q[i19]
            + (1.300974e-03) * q[i5] * q[i19] * q[i19] + (5.782373e-04) * q[i6] * q[i19] * q[i19] + (-3.725240e-04) * q[i7] * q[i19] * q[i19]
            + (9.640285e-04) * q[i8] * q[i19] * q[i19] + (1.008393e-04) * q[i9] * q[i19] * q[i19] + (-4.304783e-05) * q[i10] * q[i19] * q[i19]
            + (8.530024e-04) * q[i11] * q[i19] * q[i19] + (3.809914e-05) * q[i12] * q[i19] * q[i19] + (4.075845e-04) * q[i15] * q[i19] * q[i19]
            + (1.511783e-05) * q[i16] * q[i19] * q[i19] + (-3.517564e-04) * q[i19] * q[i19] * q[i19] + (-1.211001e-05) * q[i19] * q[i19] * q[i20]
            + (-1.911368e-03) * q[i19] * q[i19] * q[i21] + (9.038099e-06) * q[i19] * q[i19] * q[i22] + (-3.383102e-04) * q[i0] * q[i20] * q[i20]
            + (1.353930e-04) * q[i1] * q[i20] * q[i20] + (3.831992e-06) * q[i2] * q[i20] * q[i20] + (-2.618625e-04) * q[i3] * q[i20] * q[i20]
            + (-4.215867e-04) * q[i4] * q[i20] * q[i20] + (1.304845e-03) * q[i5] * q[i20] * q[i20] + (3.643780e-04) * q[i6] * q[i20] * q[i20]
            + (-5.816897e-04) * q[i7] * q[i20] * q[i20] + (-9.433013e-04) * q[i8] * q[i20] * q[i20] + (3.953414e-05) * q[i9] * q[i20] * q[i20]
            + (-1.005608e-04) * q[i10] * q[i20] * q[i20] + (3.857706e-05) * q[i11] * q[i20] * q[i20] + (8.428841e-04) * q[i12] * q[i20] * q[i20]
            + (-1.337070e-05) * q[i15] * q[i20] * q[i20] + (-3.944168e-04) * q[i16] * q[i20] * q[i20] + (7.361915e-06) * q[i19] * q[i20] * q[i20]
            + (3.434237e-04) * q[i20] * q[i20] * q[i20] + (1.125180e-05) * q[i20] * q[i20] * q[i21] + (-1.910928e-03) * q[i20] * q[i20] * q[i22]
            + (-1.336771e-03) * q[i0] * q[i21] * q[i21] + (-9.860480e-06) * q[i1] * q[i21] * q[i21] + (-1.364698e-03) * q[i2] * q[i21] * q[i21]
            + (4.875567e-04) * q[i3] * q[i21] * q[i21] + (4.514942e-06) * q[i4] * q[i21] * q[i21] + (1.230496e-04) * q[i5] * q[i21] * q[i21]
            + (6.467126e-04) * q[i6] * q[i21] * q[i21] + (-7.341577e-04) * q[i7] * q[i21] * q[i21] + (2.487787e-04) * q[i8] * q[i21] * q[i21]
            + (1.739053e-04) * q[i9] * q[i21] * q[i21] + (-6.830081e-07) * q[i10] * q[i21] * q[i21] + (-3.345501e-04) * q[i11] * q[i21] * q[i21]
            + (1.356424e-04) * q[i12] * q[i21] * q[i21] + (1.073933e-04) * q[i15] * q[i21] * q[i21] + (-3.701730e-05) * q[i16] * q[i21] * q[i21]
            + (-1.211949e-03) * q[i19] * q[i21] * q[i21] + (-5.956123e-05) * q[i20] * q[i21] * q[i21] + (-2.655636e-04) * q[i21] * q[i21] * q[i21]
            + (8.961552e-05) * q[i21] * q[i21] * q[i22] + (-7.529093e-06) * q[i0] * q[i22] * q[i22] + (-1.331682e-03) * q[i1] * q[i22] * q[i22]
            + (-1.357232e-03) * q[i2] * q[i22] * q[i22] + (3.564695e-06) * q[i3] * q[i22] * q[i22] + (4.814324e-04) * q[i4] * q[i22] * q[i22]
            + (1.300223e-04) * q[i5] * q[i22] * q[i22] + (7.316595e-04) * q[i6] * q[i22] * q[i22] + (-6.525707e-04) * q[i7] * q[i22] * q[i22]
            + (-2.487447e-04) * q[i8] * q[i22] * q[i22] + (1.112554e-06) * q[i9] * q[i22] * q[i22] + (-1.751883e-04) * q[i10] * q[i22] * q[i22]
            + (1.367414e-04) * q[i11] * q[i22] * q[i22] + (-3.197375e-04) * q[i12] * q[i22] * q[i22] + (3.600029e-05) * q[i15] * q[i22] * q[i22]
            + (-1.055802e-04) * q[i16] * q[i22] * q[i22] + (5.989050e-05) * q[i19] * q[i22] * q[i22] + (1.230489e-03) * q[i20] * q[i22] * q[i22]
            + (8.952621e-05) * q[i21] * q[i22] * q[i22] + (-2.655773e-04) * q[i22] * q[i22] * q[i22] + (-2.472957e-04) * q[i0] * q[i1] * q[i2]
            + (-2.775416e-03) * q[i0] * q[i1] * q[i3] + (-2.774729e-03) * q[i0] * q[i1] * q[i4] + (1.397100e-04) * q[i0] * q[i1] * q[i5]
            + (5.577832e-03) * q[i0] * q[i1] * q[i6] + (-5.614291e-03) * q[i0] * q[i1] * q[i7] + (1.491949e-05) * q[i0] * q[i1] * q[i8]
            + (-5.789151e-04) * q[i0] * q[i1] * q[i9] + (5.807603e-04) * q[i0] * q[i1] * q[i10] + (4.939440e-04) * q[i0] * q[i1] * q[i11]
            + (4.886039e-04) * q[i0] * q[i1] * q[i12] + (-1.055647e-03) * q[i0] * q[i1] * q[i15] + (1.063611e-03) * q[i0] * q[i1] * q[i16]
            + (8.518128e-04) * q[i0] * q[i1] * q[i19] + (-8.652112e-04) * q[i0] * q[i1] * q[i20] + (4.364268e-04) * q[i0] * q[i1] * q[i21]
            + (4.455604e-04) * q[i0] * q[i1] * q[i22] + (1.754163e-03) * q[i0] * q[i2] * q[i3] + (-2.729223e-03) * q[i0] * q[i2] * q[i4]
            + (5.637483e-03) * q[i0] * q[i2] * q[i5] + (-9.571732e-03) * q[i0] * q[i2] * q[i6] + (-3.346360e-03) * q[i0] * q[i2] * q[i7]
            + (1.531502e-03) * q[i0] * q[i2] * q[i8] + (-1.145506e-03) * q[i0] * q[i2] * q[i9] + (-7.645814e-04) * q[i0] * q[i2] * q[i10]
            + (3.099205e-04) * q[i0] * q[i2] * q[i11] + (8.559853e-04) * q[i0] * q[i2] * q[i12] + (-1.028915e-04) * q[i0] * q[i2] * q[i15]
            + (-4.698589e-04) * q[i0] * q[i2] * q[i16] + (-7.537860e-04) * q[i0] * q[i2] * q[i19] + (-8.232012e-04) * q[i0] * q[i2] * q[i20]
            + (-1.085080e-04) * q[i0] * q[i2] * q[i21] + (-3.301443e-04) * q[i0] * q[i2] * q[i22] + (6.036308e-03) * q[i0] * q[i3] * q[i4]
            + (-9.588020e-03) * q[i0] * q[i3] * q[i5] + (-1.332572e-02) * q[i0] * q[i3] * q[i6] + (-2.780377e-03) * q[i0] * q[i3] * q[i7]
            + (2.669265e-03) * q[i0] * q[i3] * q[i8] + (5.972616e-03) * q[i0] * q[i3] * q[i9] + (4.885023e-04) * q[i0] * q[i3] * q[i10]
            + (-3.955337e-04) * q[i0] * q[i3] * q[i11] + (-1.783963e-03) * q[i0] * q[i3] * q[i12] + (3.448991e-04) * q[i0] * q[i3] * q[i15]
            + (-3.403611e-03) * q[i0] * q[i3] * q[i16] + (-7.449361e-04) * q[i0] * q[i3] * q[i19] + (-1.838031e-04) * q[i0] * q[i3] * q[i20]
            + (4.444859e-04) * q[i0] * q[i3] * q[i21] + (-1.090024e-03) * q[i0] * q[i3] * q[i22] + (8.595402e-04) * q[i0] * q[i4] * q[i5]
            + (4.970552e-04) * q[i0] * q[i4] * q[i6] + (-3.318059e-03) * q[i0] * q[i4] * q[i7] + (2.519785e-03) * q[i0] * q[i4] * q[i8]
            + (4.115776e-03) * q[i0] * q[i4] * q[i9] + (2.123625e-03) * q[i0] * q[i4] * q[i10] + (-5.837825e-04) * q[i0] * q[i4] * q[i11]
            + (1.205081e-04) * q[i0] * q[i4] * q[i12] + (-3.152945e-03) * q[i0] * q[i4] * q[i15] + (-1.142440e-03) * q[i0] * q[i4] * q[i16]
            + (1.462930e-06) * q[i0] * q[i4] * q[i19] + (-1.045449e-04) * q[i0] * q[i4] * q[i20] + (5.186598e-04) * q[i0] * q[i4] * q[i21]
            + (-2.030472e-03) * q[i0] * q[i4] * q[i22] + (6.621142e-04) * q[i0] * q[i5] * q[i6] + (2.755521e-04) * q[i0] * q[i5] * q[i7]
            + (2.677214e-03) * q[i0] * q[i5] * q[i8] + (9.345901e-04) * q[i0] * q[i5] * q[i9] + (8.060825e-05) * q[i0] * q[i5] * q[i10]
            + (4.402940e-04) * q[i0] * q[i5] * q[i11] + (1.904859e-04) * q[i0] * q[i5] * q[i12] + (3.505028e-03) * q[i0] * q[i5] * q[i15]
            + (-3.198981e-03) * q[i0] * q[i5] * q[i16] + (4.599974e-04) * q[i0] * q[i5] * q[i19] + (5.631766e-04) * q[i0] * q[i5] * q[i20]
            + (-9.084618e-04) * q[i0] * q[i5] * q[i21] + (-2.016757e-04) * q[i0] * q[i5] * q[i22] + (1.005908e-02) * q[i0] * q[i6] * q[i7]
            + (2.764363e-03) * q[i0] * q[i6] * q[i8] + (-7.348027e-03) * q[i0] * q[i6] * q[i9] + (4.699217e-03) * q[i0] * q[i6] * q[i10]
            + (2.115287e-04) * q[i0] * q[i6] * q[i11] + (4.855654e-04) * q[i0] * q[i6] * q[i12] + (5.780722e-04) * q[i0] * q[i6] * q[i15]
            + (9.414677e-04) * q[i0] * q[i6] * q[i16] + (-5.449188e-04) * q[i0] * q[i6] * q[i19] + (8.165496e-04) * q[i0] * q[i6] * q[i20]
            + (1.794742e-03) * q[i0] * q[i6] * q[i21] + (6.902083e-04) * q[i0] * q[i6] * q[i22] + (5.301472e-04) * q[i0] * q[i7] * q[i8]
            + (1.497076e-03) * q[i0] * q[i7] * q[i9] + (4.225340e-03) * q[i0] * q[i7] * q[i10] + (-6.099110e-04) * q[i0] * q[i7] * q[i11]
            + (2.767898e-04) * q[i0] * q[i7] * q[i12] + (7.127504e-04) * q[i0] * q[i7] * q[i15] + (-6.352356e-04) * q[i0] * q[i7] * q[i16]
            + (-1.611498e-03) * q[i0] * q[i7] * q[i19] + (-1.202997e-03) * q[i0] * q[i7] * q[i20] + (-1.377346e-04) * q[i0] * q[i7] * q[i21]
            + (-2.210035e-04) * q[i0] * q[i7] * q[i22] + (5.055425e-06) * q[i0] * q[i8] * q[i9] + (-5.337817e-04) * q[i0] * q[i8] * q[i10]
            + (-4.324909e-04) * q[i0] * q[i8] * q[i11] + (-8.340509e-04) * q[i0] * q[i8] * q[i12] + (-1.039356e-03) * q[i0] * q[i8] * q[i15]
            + (-2.300283e-03) * q[i0] * q[i8] * q[i16] + (-1.829503e-04) * q[i0] * q[i8] * q[i19] + (2.763084e-04) * q[i0] * q[i8] * q[i20]
            + (-3.341720e-05) * q[i0] * q[i8] * q[i21] + (2.267855e-04) * q[i0] * q[i8] * q[i22] + (6.205401e-04) * q[i0] * q[i9] * q[i10]
            + (-1.484126e-04) * q[i0] * q[i9] * q[i11] + (1.019386e-04) * q[i0] * q[i9] * q[i12] + (3.218487e-05) * q[i0] * q[i9] * q[i15]
            + (2.249269e-05) * q[i0] * q[i9] * q[i16] + (4.460672e-05) * q[i0] * q[i9] * q[i19] + (1.743876e-05) * q[i0] * q[i9] * q[i20]
            + (1.217458e-04) * q[i0] * q[i9] * q[i21] + (1.699462e-04) * q[i0] * q[i9] * q[i22] + (3.592033e-04) * q[i0] * q[i10] * q[i11]
            + (2.376944e-04) * q[i0] * q[i10] * q[i12] + (1.978951e-04) * q[i0] * q[i10] * q[i15] + (4.330330e-05) * q[i0] * q[i10] * q[i16]
            + (-5.757650e-04) * q[i0] * q[i10] * q[i19] + (3.177573e-04) * q[i0] * q[i10] * q[i20] + (1.299127e-04) * q[i0] * q[i10] * q[i21]
            + (-1.408020e-04) * q[i0] * q[i10] * q[i22] + (5.993503e-04) * q[i0] * q[i11] * q[i12] + (1.081469e-03) * q[i0] * q[i11] * q[i15]
            + (1.854720e-04) * q[i0] * q[i11] * q[i16] + (2.979992e-04) * q[i0] * q[i11] * q[i19] + (-1.001125e-03) * q[i0] * q[i11] * q[i20]
            + (2.566488e-04) * q[i0] * q[i11] * q[i21] + (3.796646e-04) * q[i0] * q[i11] * q[i22] + (4.838951e-04) * q[i0] * q[i12] * q[i15]
            + (-1.300320e-03) * q[i0] * q[i12] * q[i16] + (3.978069e-05) * q[i0] * q[i12] * q[i19] + (1.015993e-03) * q[i0] * q[i12] * q[i20]
            + (-1.147600e-04) * q[i0] * q[i12] * q[i21] + (-5.634708e-04) * q[i0] * q[i12] * q[i22] + (3.132837e-07) * q[i0] * q[i15] * q[i16]
            + (1.295461e-04) * q[i0] * q[i15] * q[i19] + (3.283325e-05) * q[i0] * q[i15] * q[i20] + (-1.167079e-03) * q[i0] * q[i15] * q[i21]
            + (-1.190509e-04) * q[i0] * q[i15] * q[i22] + (3.027109e-04) * q[i0] * q[i16] * q[i19] + (-3.938751e-04) * q[i0] * q[i16] * q[i20]
            + (-4.024322e-04) * q[i0] * q[i16] * q[i21] + (7.784233e-04) * q[i0] * q[i16] * q[i22] + (-1.269053e-04) * q[i0] * q[i19] * q[i20]
            + (-6.506993e-04) * q[i0] * q[i19] * q[i21] + (1.077289e-05) * q[i0] * q[i19] * q[i22] + (-1.982301e-04) * q[i0] * q[i20] * q[i21]
            + (2.989038e-04) * q[i0] * q[i20] * q[i22] + (2.354593e-04) * q[i0] * q[i21] * q[i22] + (-2.719367e-03) * q[i1] * q[i2] * q[i3]
            + (1.711951e-03) * q[i1] * q[i2] * q[i4] + (5.671669e-03) * q[i1] * q[i2] * q[i5] + (3.342473e-03) * q[i1] * q[i2] * q[i6]
            + (9.583511e-03) * q[i1] * q[i2] * q[i7] + (-1.557901e-03) * q[i1] * q[i2] * q[i8] + (7.736552e-04) * q[i1] * q[i2] * q[i9]
            + (1.135680e-03) * q[i1] * q[i2] * q[i10] + (8.498896e-04) * q[i1] * q[i2] * q[i11] + (3.347439e-04) * q[i1] * q[i2] * q[i12]
            + (4.658728e-04) * q[i1] * q[i2] * q[i15] + (1.066038e-04) * q[i1] * q[i2] * q[i16] + (8.139548e-04) * q[i1] * q[i2] * q[i19]
            + (7.591803e-04) * q[i1] * q[i2] * q[i20] + (-3.285522e-04) * q[i1] * q[i2] * q[i21] + (-1.144321e-04) * q[i1] * q[i2] * q[i22]
            + (6.036870e-03) * q[i1] * q[i3] * q[i4] + (8.587630e-04) * q[i1] * q[i3] * q[i5] + (3.224087e-03) * q[i1] * q[i3] * q[i6]
            + (-5.326879e-04) * q[i1] * q[i3] * q[i7] + (-2.519406e-03) * q[i1] * q[i3] * q[i8] + (-2.124975e-03) * q[i1] * q[i3] * q[i9]
            + (-4.112262e-03) * q[i1] * q[i3] * q[i10] + (7.834184e-05) * q[i1] * q[i3] * q[i11] + (-6.064450e-04) * q[i1] * q[i3] * q[i12]
            + (1.111182e-03) * q[i1] * q[i3] * q[i15] + (3.177512e-03) * q[i1] * q[i3] * q[i16] + (1.178857e-04) * q[i1] * q[i3] * q[i19]
            + (-6.184635e-07) * q[i1] * q[i3] * q[i20] + (-1.995427e-03) * q[i1] * q[i3] * q[i21] + (5.225465e-04) * q[i1] * q[i3] * q[i22]
            + (-9.516510e-03) * q[i1] * q[i4] * q[i5] + (2.779804e-03) * q[i1] * q[i4] * q[i6] + (1.346045e-02) * q[i1] * q[i4] * q[i7]
            + (-2.673250e-03) * q[i1] * q[i4] * q[i8] + (-4.901910e-04) * q[i1] * q[i4] * q[i9] + (-5.909641e-03) * q[i1] * q[i4] * q[i10]
            + (-1.796502e-03) * q[i1] * q[i4] * q[i11] + (-4.375266e-04) * q[i1] * q[i4] * q[i12] + (3.354312e-03) * q[i1] * q[i4] * q[i15]
            + (-3.226464e-04) * q[i1] * q[i4] * q[i16] + (1.749162e-04) * q[i1] * q[i4] * q[i19] + (7.257399e-04) * q[i1] * q[i4] * q[i20]
            + (-1.075580e-03) * q[i1] * q[i4] * q[i21] + (4.590103e-04) * q[i1] * q[i4] * q[i22] + (-2.830539e-04) * q[i1] * q[i5] * q[i6]
            + (-6.403782e-04) * q[i1] * q[i5] * q[i7] + (-2.662097e-03) * q[i1] * q[i5] * q[i8] + (-6.748973e-05) * q[i1] * q[i5] * q[i9]
            + (-9.173541e-04) * q[i1] * q[i5] * q[i10] + (2.033053e-04) * q[i1] * q[i5] * q[i11] + (4.199434e-04) * q[i1] * q[i5] * q[i12]
            + (3.178904e-03) * q[i1] * q[i5] * q[i15] + (-3.545824e-03) * q[i1] * q[i5] * q[i16] + (-5.702509e-04) * q[i1] * q[i5] * q[i19]
            + (-4.714147e-04) * q[i1] * q[i5] * q[i20] + (-1.942308e-04) * q[i1] * q[i5] * q[i21] + (-8.835536e-04) * q[i1] * q[i5] * q[i22]
            + (1.008619e-02) * q[i1] * q[i6] * q[i7] + (5.266155e-04) * q[i1] * q[i6] * q[i8] + (4.249609e-03) * q[i1] * q[i6] * q[i9]
            + (1.493330e-03) * q[i1] * q[i6] * q[i10] + (-2.573820e-04) * q[i1] * q[i6] * q[i11] + (6.048126e-04) * q[i1] * q[i6] * q[i12]
            + (-6.323197e-04) * q[i1] * q[i6] * q[i15] + (6.999986e-04) * q[i1] * q[i6] * q[i16] + (-1.192923e-03) * q[i1] * q[i6] * q[i19]
            + (-1.608027e-03) * q[i1] * q[i6] * q[i20] + (2.265587e-04) * q[i1] * q[i6] * q[i21] + (1.453181e-04) * q[i1] * q[i6] * q[i22]
            + (2.738656e-03) * q[i1] * q[i7] * q[i8] + (4.749150e-03) * q[i1] * q[i7] * q[i9] + (-7.327260e-03) * q[i1] * q[i7] * q[i10]
            + (-4.702745e-04) * q[i1] * q[i7] * q[i11] + (-2.142420e-04) * q[i1] * q[i7] * q[i12] + (9.428895e-04) * q[i1] * q[i7] * q[i15]
            + (5.748451e-04) * q[i1] * q[i7] * q[i16] + (8.248072e-04) * q[i1] * q[i7] * q[i19] + (-5.374547e-04) * q[i1] * q[i7] * q[i20]
            + (-6.836386e-04) * q[i1] * q[i7] * q[i21] + (-1.779180e-03) * q[i1] * q[i7] * q[i22] + (-5.361324e-04) * q[i1] * q[i8] * q[i9]
            + (-3.204259e-06) * q[i1] * q[i8] * q[i10] + (8.293673e-04) * q[i1] * q[i8] * q[i11] + (4.379223e-04) * q[i1] * q[i8] * q[i12]
            + (-2.278029e-03) * q[i1] * q[i8] * q[i15] + (-1.046418e-03) * q[i1] * q[i8] * q[i16] + (2.850417e-04) * q[i1] * q[i8] * q[i19]
            + (-1.726717e-04) * q[i1] * q[i8] * q[i20] + (-2.283093e-04) * q[i1] * q[i8] * q[i21] + (1.446450e-05) * q[i1] * q[i8] * q[i22]
            + (6.272113e-04) * q[i1] * q[i9] * q[i10] + (-2.220626e-04) * q[i1] * q[i9] * q[i11] + (-3.510685e-04) * q[i1] * q[i9] * q[i12]
            + (4.011463e-05) * q[i1] * q[i9] * q[i15] + (1.947114e-04) * q[i1] * q[i9] * q[i16] + (3.131939e-04) * q[i1] * q[i9] * q[i19]
            + (-5.745873e-04) * q[i1] * q[i9] * q[i20] + (1.450506e-04) * q[i1] * q[i9] * q[i21] + (-1.327543e-04) * q[i1] * q[i9] * q[i22]
            + (-1.021766e-04) * q[i1] * q[i10] * q[i11] + (1.458899e-04) * q[i1] * q[i10] * q[i12] + (2.651569e-05) * q[i1] * q[i10] * q[i15]
            + (3.412348e-05) * q[i1] * q[i10] * q[i16] + (1.504097e-05) * q[i1] * q[i10] * q[i19] + (4.715358e-05) * q[i1] * q[i10] * q[i20]
            + (-1.679596e-04) * q[i1] * q[i10] * q[i21] + (-1.153232e-04) * q[i1] * q[i10] * q[i22] + (5.996905e-04) * q[i1] * q[i11] * q[i12]
            + (1.303593e-03) * q[i1] * q[i11] * q[i15] + (-4.838039e-04) * q[i1] * q[i11] * q[i16] + (-1.021959e-03) * q[i1] * q[i11] * q[i19]
            + (-3.769764e-05) * q[i1] * q[i11] * q[i20] + (-5.641109e-04) * q[i1] * q[i11] * q[i21] + (-1.102424e-04) * q[i1] * q[i11] * q[i22]
            + (-1.792892e-04) * q[i1] * q[i12] * q[i15] + (-1.087714e-03) * q[i1] * q[i12] * q[i16] + (9.876071e-04) * q[i1] * q[i12] * q[i19]
            + (-3.163449e-04) * q[i1] * q[i12] * q[i20] + (3.741781e-04) * q[i1] * q[i12] * q[i21] + (2.672405e-04) * q[i1] * q[i12] * q[i22]
            + (-5.778187e-06) * q[i1] * q[i15] * q[i16] + (-4.077494e-04) * q[i1] * q[i15] * q[i19] + (3.002936e-04) * q[i1] * q[i15] * q[i20]
            + (-7.800937e-04) * q[i1] * q[i15] * q[i21] + (3.931238e-04) * q[i1] * q[i15] * q[i22] + (2.687331e-05) * q[i1] * q[i16] * q[i19]
            + (1.281700e-04) * q[i1] * q[i16] * q[i20] + (1.202486e-04) * q[i1] * q[i16] * q[i21] + (1.171665e-03) * q[i1] * q[i16] * q[i22]
            + (-1.275865e-04) * q[i1] * q[i19] * q[i20] + (-3.065236e-04) * q[i1] * q[i19] * q[i21] + (1.984812e-04) * q[i1] * q[i19] * q[i22]
            + (-1.011552e-05) * q[i1] * q[i20] * q[i21] + (6.525594e-04) * q[i1] * q[i20] * q[i22] + (2.246849e-04) * q[i1] * q[i21] * q[i22]
            + (-1.562891e-03) * q[i2] * q[i3] * q[i4] + (-1.571061e-02) * q[i2] * q[i3] * q[i5] + (-4.895144e-03) * q[i2] * q[i3] * q[i6]
            + (-2.724645e-03) * q[i2] * q[i3] * q[i7] + (-2.313675e-03) * q[i2] * q[i3] * q[i8] + (1.834456e-03) * q[i2] * q[i3] * q[i9]
            + (-1.719263e-03) * q[i2] * q[i3] * q[i10] + (-7.970426e-04) * q[i2] * q[i3] * q[i11] + (-1.430941e-03) * q[i2] * q[i3] * q[i12]
            + (2.462569e-03) * q[i2] * q[i3] * q[i15] + (-9.440405e-04) * q[i2] * q[i3] * q[i16] + (3.471882e-04) * q[i2] * q[i3] * q[i19]
            + (3.399666e-04) * q[i2] * q[i3] * q[i20] + (-4.242770e-04) * q[i2] * q[i3] * q[i21] + (1.596645e-04) * q[i2] * q[i3] * q[i22]
            + (-1.563477e-02) * q[i2] * q[i4] * q[i5] + (2.686297e-03) * q[i2] * q[i4] * q[i6] + (4.905160e-03) * q[i2] * q[i4] * q[i7]
            + (2.307384e-03) * q[i2] * q[i4] * q[i8] + (1.725919e-03) * q[i2] * q[i4] * q[i9] + (-1.793558e-03) * q[i2] * q[i4] * q[i10]
            + (-1.389473e-03) * q[i2] * q[i4] * q[i11] + (-8.046657e-04) * q[i2] * q[i4] * q[i12] + (9.279543e-04) * q[i2] * q[i4] * q[i15]
            + (-2.494502e-03) * q[i2] * q[i4] * q[i16] + (-3.463028e-04) * q[i2] * q[i4] * q[i19] + (-3.384693e-04) * q[i2] * q[i4] * q[i20]
            + (1.623750e-04) * q[i2] * q[i4] * q[i21] + (-4.368106e-04) * q[i2] * q[i4] * q[i22] + (-6.240699e-03) * q[i2] * q[i5] * q[i6]
            + (6.249056e-03) * q[i2] * q[i5] * q[i7] + (-1.735343e-06) * q[i2] * q[i5] * q[i8] + (2.875813e-03) * q[i2] * q[i5] * q[i9]
            + (-2.851083e-03) * q[i2] * q[i5] * q[i10] + (-7.976358e-04) * q[i2] * q[i5] * q[i11] + (-8.642104e-04) * q[i2] * q[i5] * q[i12]
            + (5.182532e-03) * q[i2] * q[i5] * q[i15] + (-5.230026e-03) * q[i2] * q[i5] * q[i16] + (-1.092606e-03) * q[i2] * q[i5] * q[i19]
            + (1.088875e-03) * q[i2] * q[i5] * q[i20] + (1.804426e-04) * q[i2] * q[i5] * q[i21] + (1.878957e-04) * q[i2] * q[i5] * q[i22]
            + (9.552667e-03) * q[i2] * q[i6] * q[i7] + (-5.275663e-03) * q[i2] * q[i6] * q[i8] + (-4.297177e-03) * q[i2] * q[i6] * q[i9]
            + (1.693568e-03) * q[i2] * q[i6] * q[i10] + (-9.012448e-04) * q[i2] * q[i6] * q[i11] + (-2.685933e-04) * q[i2] * q[i6] * q[i12]
            + (-1.495753e-04) * q[i2] * q[i6] * q[i15] + (-7.048554e-04) * q[i2] * q[i6] * q[i16] + (1.051281e-03) * q[i2] * q[i6] * q[i19]
            + (2.332729e-04) * q[i2] * q[i6] * q[i20] + (-1.001613e-03) * q[i2] * q[i6] * q[i21] + (-5.206706e-05) * q[i2] * q[i6] * q[i22]
            + (-5.243947e-03) * q[i2] * q[i7] * q[i8] + (1.719453e-03) * q[i2] * q[i7] * q[i9] + (-4.273448e-03) * q[i2] * q[i7] * q[i10]
            + (2.739008e-04) * q[i2] * q[i7] * q[i11] + (9.051833e-04) * q[i2] * q[i7] * q[i12] + (-6.872632e-04) * q[i2] * q[i7] * q[i15]
            + (-1.709576e-04) * q[i2] * q[i7] * q[i16] + (2.468784e-04) * q[i2] * q[i7] * q[i19] + (1.040407e-03) * q[i2] * q[i7] * q[i20]
            + (6.353159e-05) * q[i2] * q[i7] * q[i21] + (9.994085e-04) * q[i2] * q[i7] * q[i22] + (-1.988901e-03) * q[i2] * q[i8] * q[i9]
            + (-1.960195e-03) * q[i2] * q[i8] * q[i10] + (-1.958002e-04) * q[i2] * q[i8] * q[i11] + (1.600792e-04) * q[i2] * q[i8] * q[i12]
            + (-2.168129e-03) * q[i2] * q[i8] * q[i15] + (-2.187043e-03) * q[i2] * q[i8] * q[i16] + (-7.860911e-04) * q[i2] * q[i8] * q[i19]
            + (-7.718952e-04) * q[i2] * q[i8] * q[i20] + (8.458875e-04) * q[i2] * q[i8] * q[i21] + (-8.524062e-04) * q[i2] * q[i8] * q[i22]
            + (-3.042158e-05) * q[i2] * q[i9] * q[i10] + (-5.052321e-04) * q[i2] * q[i9] * q[i11] + (-5.898781e-04) * q[i2] * q[i9] * q[i12]
            + (2.126291e-04) * q[i2] * q[i9] * q[i15] + (-2.350800e-04) * q[i2] * q[i9] * q[i16] + (7.958861e-04) * q[i2] * q[i9] * q[i19]
            + (-7.146794e-04) * q[i2] * q[i9] * q[i20] + (-2.137513e-04) * q[i2] * q[i9] * q[i21] + (3.476444e-05) * q[i2] * q[i9] * q[i22]
            + (5.983864e-04) * q[i2] * q[i10] * q[i11] + (5.127200e-04) * q[i2] * q[i10] * q[i12] + (-2.269576e-04) * q[i2] * q[i10] * q[i15]
            + (2.115800e-04) * q[i2] * q[i10] * q[i16] + (-7.141847e-04) * q[i2] * q[i10] * q[i19] + (7.970314e-04) * q[i2] * q[i10] * q[i20]
            + (-3.824529e-05) * q[i2] * q[i10] * q[i21] + (2.139062e-04) * q[i2] * q[i10] * q[i22] + (1.333949e-03) * q[i2] * q[i11] * q[i12]
            + (2.093869e-03) * q[i2] * q[i11] * q[i15] + (-4.612780e-04) * q[i2] * q[i11] * q[i16] + (-8.422558e-04) * q[i2] * q[i11] * q[i19]
            + (-3.052536e-04) * q[i2] * q[i11] * q[i20] + (-6.046811e-04) * q[i2] * q[i11] * q[i21] + (-1.341003e-04) * q[i2] * q[i11] * q[i22]
            + (4.733281e-04) * q[i2] * q[i12] * q[i15] + (-2.107401e-03) * q[i2] * q[i12] * q[i16] + (2.915829e-04) * q[i2] * q[i12] * q[i19]
            + (8.216694e-04) * q[i2] * q[i12] * q[i20] + (-1.392476e-04) * q[i2] * q[i12] * q[i21] + (-5.861205e-04) * q[i2] * q[i12] * q[i22]
            + (-2.080770e-04) * q[i2] * q[i15] * q[i16] + (-1.433018e-04) * q[i2] * q[i15] * q[i19] + (4.225078e-04) * q[i2] * q[i15] * q[i20]
            + (-1.630412e-03) * q[i2] * q[i15] * q[i21] + (-3.246137e-04) * q[i2] * q[i15] * q[i22] + (4.207257e-04) * q[i2] * q[i16] * q[i19]
            + (-1.308262e-04) * q[i2] * q[i16] * q[i20] + (3.203257e-04) * q[i2] * q[i16] * q[i21] + (1.635022e-03) * q[i2] * q[i16] * q[i22]
            + (-5.132617e-04) * q[i2] * q[i19] * q[i20] + (-1.510791e-03) * q[i2] * q[i19] * q[i21] + (-1.495248e-04) * q[i2] * q[i19] * q[i22]
            + (1.482476e-04) * q[i2] * q[i20] * q[i21] + (1.505349e-03) * q[i2] * q[i20] * q[i22] + (7.342545e-05) * q[i2] * q[i21] * q[i22]
            + (1.959007e-03) * q[i3] * q[i4] * q[i5] + (4.694904e-03) * q[i3] * q[i4] * q[i6] + (-4.638914e-03) * q[i3] * q[i4] * q[i7]
            + (-1.029825e-05) * q[i3] * q[i4] * q[i8] + (4.459834e-04) * q[i3] * q[i4] * q[i9] + (-4.160438e-04) * q[i3] * q[i4] * q[i10]
            + (3.414564e-03) * q[i3] * q[i4] * q[i11] + (3.431121e-03) * q[i3] * q[i4] * q[i12] + (9.870022e-04) * q[i3] * q[i4] * q[i15]
            + (-9.768833e-04) * q[i3] * q[i4] * q[i16] + (-2.270218e-03) * q[i3] * q[i4] * q[i19] + (2.276730e-03) * q[i3] * q[i4] * q[i20]
            + (1.190653e-03) * q[i3] * q[i4] * q[i21] + (1.195199e-03) * q[i3] * q[i4] * q[i22] + (-6.271087e-03) * q[i3] * q[i5] * q[i6]
            + (-2.619309e-03) * q[i3] * q[i5] * q[i7] + (-1.224074e-02) * q[i3] * q[i5] * q[i8] + (5.931976e-04) * q[i3] * q[i5] * q[i9]
            + (1.127468e-03) * q[i3] * q[i5] * q[i10] + (-1.901310e-03) * q[i3] * q[i5] * q[i11] + (-1.684910e-03) * q[i3] * q[i5] * q[i12]
            + (3.770076e-04) * q[i3] * q[i5] * q[i15] + (2.227935e-03) * q[i3] * q[i5] * q[i16] + (-9.872696e-04) * q[i3] * q[i5] * q[i19]
            + (-2.079203e-03) * q[i3] * q[i5] * q[i20] + (1.540285e-03) * q[i3] * q[i5] * q[i21] + (-1.739445e-03) * q[i3] * q[i5] * q[i22]
            + (2.388319e-03) * q[i3] * q[i6] * q[i7] + (1.302527e-03) * q[i3] * q[i6] * q[i8] + (-2.525796e-04) * q[i3] * q[i6] * q[i9]
            + (1.481257e-03) * q[i3] * q[i6] * q[i10] + (-5.606625e-04) * q[i3] * q[i6] * q[i11] + (6.150328e-04) * q[i3] * q[i6] * q[i12]
            + (2.190919e-03) * q[i3] * q[i6] * q[i15] + (1.475704e-03) * q[i3] * q[i6] * q[i16] + (-2.351642e-03) * q[i3] * q[i6] * q[i19]
            + (-1.440157e-03) * q[i3] * q[i6] * q[i20] + (-2.486855e-03) * q[i3] * q[i6] * q[i21] + (1.476708e-04) * q[i3] * q[i6] * q[i22]
            + (-3.731833e-03) * q[i3] * q[i7] * q[i8] + (-1.921714e-03) * q[i3] * q[i7] * q[i9] + (-2.387680e-03) * q[i3] * q[i7] * q[i10]
            + (2.047629e-03) * q[i3] * q[i7] * q[i11] + (-1.334634e-04) * q[i3] * q[i7] * q[i12] + (8.216297e-04) * q[i3] * q[i7] * q[i15]
            + (-9.833456e-05) * q[i3] * q[i7] * q[i16] + (2.829038e-03) * q[i3] * q[i7] * q[i19] + (1.855864e-03) * q[i3] * q[i7] * q[i20]
            + (2.294031e-03) * q[i3] * q[i7] * q[i21] + (-1.392027e-03) * q[i3] * q[i7] * q[i22] + (-3.652426e-04) * q[i3] * q[i8] * q[i9]
            + (6.855023e-04) * q[i3] * q[i8] * q[i10] + (1.223695e-03) * q[i3] * q[i8] * q[i11] + (6.380943e-04) * q[i3] * q[i8] * q[i12]
            + (-1.681315e-03) * q[i3] * q[i8] * q[i15] + (7.726770e-04) * q[i3] * q[i8] * q[i16] + (1.835695e-03) * q[i3] * q[i8] * q[i19]
            + (-1.592012e-03) * q[i3] * q[i8] * q[i20] + (4.773394e-04) * q[i3] * q[i8] * q[i21] + (-1.072421e-04) * q[i3] * q[i8] * q[i22]
            + (-5.715158e-04) * q[i3] * q[i9] * q[i10] + (-4.639914e-04) * q[i3] * q[i9] * q[i11] + (-8.660618e-04) * q[i3] * q[i9] * q[i12]
            + (-3.666855e-04) * q[i3] * q[i9] * q[i15] + (6.472386e-04) * q[i3] * q[i9] * q[i16] + (6.488512e-04) * q[i3] * q[i9] * q[i19]
            + (-4.009139e-04) * q[i3] * q[i9] * q[i20] + (-1.182874e-03) * q[i3] * q[i9] * q[i21] + (6.093263e-04) * q[i3] * q[i9] * q[i22]
            + (-2.395844e-04) * q[i3] * q[i10] * q[i11] + (-3.029864e-04) * q[i3] * q[i10] * q[i12] + (-2.730054e-04) * q[i3] * q[i10] * q[i15]
            + (-3.540169e-04) * q[i3] * q[i10] * q[i16] + (8.387708e-05) * q[i3] * q[i10] * q[i19] + (-6.046693e-04) * q[i3] * q[i10] * q[i20]
            + (2.872147e-04) * q[i3] * q[i10] * q[i21] + (-3.128102e-04) * q[i3] * q[i10] * q[i22] + (-6.716568e-04) * q[i3] * q[i11] * q[i12]
            + (-9.556197e-04) * q[i3] * q[i11] * q[i15] + (-8.723860e-04) * q[i3] * q[i11] * q[i16] + (-2.145832e-04) * q[i3] * q[i11] * q[i19]
            + (-1.368810e-03) * q[i3] * q[i11] * q[i20] + (-8.578729e-04) * q[i3] * q[i11] * q[i21] + (-5.538045e-04) * q[i3] * q[i11] * q[i22]
            + (-2.005103e-04) * q[i3] * q[i12] * q[i15] + (-1.512364e-03) * q[i3] * q[i12] * q[i16] + (-1.993557e-03) * q[i3] * q[i12] * q[i19]
            + (-3.306156e-04) * q[i3] * q[i12] * q[i20] + (2.418505e-04) * q[i3] * q[i12] * q[i21] + (9.448845e-04) * q[i3] * q[i12] * q[i22]
            + (2.289200e-04) * q[i3] * q[i15] * q[i16] + (1.308130e-04) * q[i3] * q[i15] * q[i19] + (4.418298e-04) * q[i3] * q[i15] * q[i20]
            + (2.398466e-04) * q[i3] * q[i15] * q[i21] + (1.323404e-04) * q[i3] * q[i15] * q[i22] + (4.027790e-06) * q[i3] * q[i16] * q[i19]
            + (8.806233e-04) * q[i3] * q[i16] * q[i20] + (7.408101e-04) * q[i3] * q[i16] * q[i21] + (5.767705e-04) * q[i3] * q[i16] * q[i22]
            + (-1.007803e-04) * q[i3] * q[i19] * q[i20] + (9.749011e-04) * q[i3] * q[i19] * q[i21] + (-2.244236e-06) * q[i3] * q[i19] * q[i22]
            + (6.515640e-04) * q[i3] * q[i20] * q[i21] + (8.760701e-04) * q[i3] * q[i20] * q[i22] + (2.058444e-04) * q[i3] * q[i21] * q[i22]
            + (2.613924e-03) * q[i4] * q[i5] * q[i6] + (6.279395e-03) * q[i4] * q[i5] * q[i7] + (1.221304e-02) * q[i4] * q[i5] * q[i8]
            + (-1.123052e-03) * q[i4] * q[i5] * q[i9] + (-6.099268e-04) * q[i4] * q[i5] * q[i10] + (-1.607223e-03) * q[i4] * q[i5] * q[i11]
            + (-1.913452e-03) * q[i4] * q[i5] * q[i12] + (-2.212603e-03) * q[i4] * q[i5] * q[i15] + (-3.527507e-04) * q[i4] * q[i5] * q[i16]
            + (2.051015e-03) * q[i4] * q[i5] * q[i19] + (9.827736e-04) * q[i4] * q[i5] * q[i20] + (-1.737532e-03) * q[i4] * q[i5] * q[i21]
            + (1.528462e-03) * q[i4] * q[i5] * q[i22] + (2.431741e-03) * q[i4] * q[i6] * q[i7] + (-3.741767e-03) * q[i4] * q[i6] * q[i8]
            + (-2.433371e-03) * q[i4] * q[i6] * q[i9] + (-1.892674e-03) * q[i4] * q[i6] * q[i10] + (1.127119e-04) * q[i4] * q[i6] * q[i11]
            + (-2.077972e-03) * q[i4] * q[i6] * q[i12] + (-1.289673e-04) * q[i4] * q[i6] * q[i15] + (8.370493e-04) * q[i4] * q[i6] * q[i16]
            + (1.840922e-03) * q[i4] * q[i6] * q[i19] + (2.832568e-03) * q[i4] * q[i6] * q[i20] + (1.400710e-03) * q[i4] * q[i6] * q[i21]
            + (-2.287192e-03) * q[i4] * q[i6] * q[i22] + (1.331434e-03) * q[i4] * q[i7] * q[i8] + (1.500690e-03) * q[i4] * q[i7] * q[i9]
            + (-3.217646e-04) * q[i4] * q[i7] * q[i10] + (-6.173099e-04) * q[i4] * q[i7] * q[i11] + (5.386931e-04) * q[i4] * q[i7] * q[i12]
            + (1.469082e-03) * q[i4] * q[i7] * q[i15] + (2.197919e-03) * q[i4] * q[i7] * q[i16] + (-1.443071e-03) * q[i4] * q[i7] * q[i19]
            + (-2.349355e-03) * q[i4] * q[i7] * q[i20] + (-1.470133e-04) * q[i4] * q[i7] * q[i21] + (2.475617e-03) * q[i4] * q[i7] * q[i22]
            + (6.757443e-04) * q[i4] * q[i8] * q[i9] + (-3.792716e-04) * q[i4] * q[i8] * q[i10] + (-6.633114e-04) * q[i4] * q[i8] * q[i11]
            + (-1.297709e-03) * q[i4] * q[i8] * q[i12] + (7.776032e-04) * q[i4] * q[i8] * q[i15] + (-1.691448e-03) * q[i4] * q[i8] * q[i16]
            + (-1.577424e-03) * q[i4] * q[i8] * q[i19] + (1.827705e-03) * q[i4] * q[i8] * q[i20] + (1.110036e-04) * q[i4] * q[i8] * q[i21]
            + (-4.623798e-04) * q[i4] * q[i8] * q[i22] + (-5.691556e-04) * q[i4] * q[i9] * q[i10] + (3.008557e-04) * q[i4] * q[i9] * q[i11]
            + (2.362237e-04) * q[i4] * q[i9] * q[i12] + (-3.479641e-04) * q[i4] * q[i9] * q[i15] + (-2.630869e-04) * q[i4] * q[i9] * q[i16]
            + (-6.041358e-04) * q[i4] * q[i9] * q[i19] + (9.788540e-05) * q[i4] * q[i9] * q[i20] + (3.185083e-04) * q[i4] * q[i9] * q[i21]
            + (-2.816227e-04) * q[i4] * q[i9] * q[i22] + (8.557228e-04) * q[i4] * q[i10] * q[i11] + (4.610310e-04) * q[i4] * q[i10] * q[i12]
            + (6.379221e-04) * q[i4] * q[i10] * q[i15] + (-3.588534e-04) * q[i4] * q[i10] * q[i16] + (-4.020045e-04) * q[i4] * q[i10] * q[i19]
            + (6.473136e-04) * q[i4] * q[i10] * q[i20] + (-6.081948e-04) * q[i4] * q[i10] * q[i21] + (1.174813e-03) * q[i4] * q[i10] * q[i22]
            + (-6.650560e-04) * q[i4] * q[i11] * q[i12] + (1.505921e-03) * q[i4] * q[i11] * q[i15] + (2.099040e-04) * q[i4] * q[i11] * q[i16]
            + (3.381856e-04) * q[i4] * q[i11] * q[i19] + (1.983930e-03) * q[i4] * q[i11] * q[i20] + (9.509409e-04) * q[i4] * q[i11] * q[i21]
            + (2.386574e-04) * q[i4] * q[i11] * q[i22] + (8.698201e-04) * q[i4] * q[i12] * q[i15] + (9.665469e-04) * q[i4] * q[i12] * q[i16]
            + (1.368546e-03) * q[i4] * q[i12] * q[i19] + (2.078426e-04) * q[i4] * q[i12] * q[i20] + (-5.561184e-04) * q[i4] * q[i12] * q[i21]
            + (-8.644089e-04) * q[i4] * q[i12] * q[i22] + (2.368711e-04) * q[i4] * q[i15] * q[i16] + (8.908017e-04) * q[i4] * q[i15] * q[i19]
            + (-2.271586e-06) * q[i4] * q[i15] * q[i20] + (-5.666974e-04) * q[i4] * q[i15] * q[i21] + (-7.345003e-04) * q[i4] * q[i15] * q[i22]
            + (4.364204e-04) * q[i4] * q[i16] * q[i19] + (1.314712e-04) * q[i4] * q[i16] * q[i20] + (-1.290802e-04) * q[i4] * q[i16] * q[i21]
            + (-2.341977e-04) * q[i4] * q[i16] * q[i22] + (-9.948025e-05) * q[i4] * q[i19] * q[i20] + (-8.673770e-04) * q[i4] * q[i19] * q[i21]
            + (-6.473718e-04) * q[i4] * q[i19] * q[i22] + (2.535387e-06) * q[i4] * q[i20] * q[i21] + (-9.697348e-04) * q[i4] * q[i20] * q[i22]
            + (2.030181e-04) * q[i4] * q[i21] * q[i22] + (-5.953039e-03) * q[i5] * q[i6] * q[i7] + (1.560364e-03) * q[i5] * q[i6] * q[i8]
            + (-4.389303e-03) * q[i5] * q[i6] * q[i9] + (2.530828e-03) * q[i5] * q[i6] * q[i10] + (1.066903e-03) * q[i5] * q[i6] * q[i11]
            + (-1.394643e-04) * q[i5] * q[i6] * q[i12] + (-2.992203e-03) * q[i5] * q[i6] * q[i15] + (-1.591540e-03) * q[i5] * q[i6] * q[i16]
            + (2.117505e-03) * q[i5] * q[i6] * q[i19] + (-1.682309e-03) * q[i5] * q[i6] * q[i20] + (9.124768e-04) * q[i5] * q[i6] * q[i21]
            + (-1.664968e-04) * q[i5] * q[i6] * q[i22] + (1.567247e-03) * q[i5] * q[i7] * q[i8] + (2.547313e-03) * q[i5] * q[i7] * q[i9]
            + (-4.329624e-03) * q[i5] * q[i7] * q[i10] + (1.694044e-04) * q[i5] * q[i7] * q[i11] + (-1.035892e-03) * q[i5] * q[i7] * q[i12]
            + (-1.588615e-03) * q[i5] * q[i7] * q[i15] + (-3.005338e-03) * q[i5] * q[i7] * q[i16] + (-1.678776e-03) * q[i5] * q[i7] * q[i19]
            + (2.085572e-03) * q[i5] * q[i7] * q[i20] + (1.582883e-04) * q[i5] * q[i7] * q[i21] + (-9.148670e-04) * q[i5] * q[i7] * q[i22]
            + (3.370415e-03) * q[i5] * q[i8] * q[i9] + (3.358581e-03) * q[i5] * q[i8] * q[i10] + (-2.579714e-03) * q[i5] * q[i8] * q[i11]
            + (2.749042e-03) * q[i5] * q[i8] * q[i12] + (-2.099753e-03) * q[i5] * q[i8] * q[i15] + (-2.120832e-03) * q[i5] * q[i8] * q[i16]
            + (3.321299e-04) * q[i5] * q[i8] * q[i19] + (3.494283e-04) * q[i5] * q[i8] * q[i20] + (-1.047609e-04) * q[i5] * q[i8] * q[i21]
            + (6.351661e-05) * q[i5] * q[i8] * q[i22] + (1.795244e-03) * q[i5] * q[i9] * q[i10] + (-2.721587e-04) * q[i5] * q[i9] * q[i11]
            + (6.765033e-05) * q[i5] * q[i9] * q[i12] + (6.044531e-04) * q[i5] * q[i9] * q[i15] + (-8.888490e-05) * q[i5] * q[i9] * q[i16]
            + (-4.683551e-04) * q[i5] * q[i9] * q[i19] + (-2.614614e-04) * q[i5] * q[i9] * q[i20] + (5.513357e-04) * q[i5] * q[i9] * q[i21]
            + (-5.134596e-04) * q[i5] * q[i9] * q[i22] + (-5.485953e-05) * q[i5] * q[i10] * q[i11] + (2.937166e-04) * q[i5] * q[i10] * q[i12]
            + (-7.002994e-05) * q[i5] * q[i10] * q[i15] + (6.086860e-04) * q[i5] * q[i10] * q[i16] + (-2.530374e-04) * q[i5] * q[i10] * q[i19]
            + (-4.645845e-04) * q[i5] * q[i10] * q[i20] + (5.093617e-04) * q[i5] * q[i10] * q[i21] + (-5.545135e-04) * q[i5] * q[i10] * q[i22]
            + (1.140986e-03) * q[i5] * q[i11] * q[i12] + (-2.709833e-03) * q[i5] * q[i11] * q[i15] + (4.243776e-04) * q[i5] * q[i11] * q[i16]
            + (-1.004278e-03) * q[i5] * q[i11] * q[i19] + (-2.755955e-04) * q[i5] * q[i11] * q[i20] + (-1.054120e-03) * q[i5] * q[i11] * q[i21]
            + (4.779538e-04) * q[i5] * q[i11] * q[i22] + (-4.287751e-04) * q[i5] * q[i12] * q[i15] + (2.717488e-03) * q[i5] * q[i12] * q[i16]
            + (2.775053e-04) * q[i5] * q[i12] * q[i19] + (9.851661e-04) * q[i5] * q[i12] * q[i20] + (4.795968e-04) * q[i5] * q[i12] * q[i21]
            + (-1.037047e-03) * q[i5] * q[i12] * q[i22] + (1.007694e-03) * q[i5] * q[i15] * q[i16] + (5.705245e-04) * q[i5] * q[i15] * q[i19]
            + (-3.208123e-04) * q[i5] * q[i15] * q[i20] + (1.573328e-04) * q[i5] * q[i15] * q[i21] + (-1.446802e-04) * q[i5] * q[i15] * q[i22]
            + (-3.259581e-04) * q[i5] * q[i16] * q[i19] + (5.767805e-04) * q[i5] * q[i16] * q[i20] + (1.445475e-04) * q[i5] * q[i16] * q[i21]
            + (-1.700077e-04) * q[i5] * q[i16] * q[i22] + (5.075397e-04) * q[i5] * q[i19] * q[i20] + (3.316346e-04) * q[i5] * q[i19] * q[i21]
            + (9.389452e-05) * q[i5] * q[i19] * q[i22] + (-9.564362e-05) * q[i5] * q[i20] * q[i21] + (-3.390023e-04) * q[i5] * q[i20] * q[i22]
            + (-1.336004e-04) * q[i5] * q[i21] * q[i22] + (-1.143098e-05) * q[i6] * q[i7] * q[i8] + (2.577368e-03) * q[i6] * q[i7] * q[i9]
            + (-2.564323e-03) * q[i6] * q[i7] * q[i10] + (-8.060847e-04) * q[i6] * q[i7] * q[i11] + (-8.094762e-04) * q[i6] * q[i7] * q[i12]
            + (-1.212214e-03) * q[i6] * q[i7] * q[i15] + (1.210781e-03) * q[i6] * q[i7] * q[i16] + (8.441643e-04) * q[i6] * q[i7] * q[i19]
            + (-8.377542e-04) * q[i6] * q[i7] * q[i20] + (-3.850392e-04) * q[i6] * q[i7] * q[i21] + (-3.897134e-04) * q[i6] * q[i7] * q[i22]
            + (4.222792e-04) * q[i6] * q[i8] * q[i9] + (-3.622831e-04) * q[i6] * q[i8] * q[i10] + (1.840413e-03) * q[i6] * q[i8] * q[i11]
            + (5.693488e-04) * q[i6] * q[i8] * q[i12] + (6.057846e-04) * q[i6] * q[i8] * q[i15] + (-2.253623e-03) * q[i6] * q[i8] * q[i16]
            + (-2.846423e-04) * q[i6] * q[i8] * q[i19] + (1.106294e-03) * q[i6] * q[i8] * q[i20] + (-1.004595e-03) * q[i6] * q[i8] * q[i21]
            + (1.344199e-03) * q[i6] * q[i8] * q[i22] + (8.809378e-04) * q[i6] * q[i9] * q[i10] + (-5.482627e-04) * q[i6] * q[i9] * q[i11]
            + (-6.646076e-04) * q[i6] * q[i9] * q[i12] + (5.592634e-04) * q[i6] * q[i9] * q[i15] + (-4.138171e-04) * q[i6] * q[i9] * q[i16]
            + (-5.808050e-04) * q[i6] * q[i9] * q[i19] + (8.196789e-04) * q[i6] * q[i9] * q[i20] + (2.981797e-04) * q[i6] * q[i9] * q[i21]
            + (-7.568454e-04) * q[i6] * q[i9] * q[i22] + (7.645532e-04) * q[i6] * q[i10] * q[i11] + (-4.659777e-05) * q[i6] * q[i10] * q[i12]
            + (4.928168e-05) * q[i6] * q[i10] * q[i15] + (5.564520e-04) * q[i6] * q[i10] * q[i16] + (1.102690e-03) * q[i6] * q[i10] * q[i19]
            + (4.810234e-04) * q[i6] * q[i10] * q[i20] + (7.280991e-04) * q[i6] * q[i10] * q[i21] + (1.866174e-04) * q[i6] * q[i10] * q[i22]
            + (-2.188075e-03) * q[i6] * q[i11] * q[i12] + (-8.049555e-05) * q[i6] * q[i11] * q[i15] + (1.236653e-03) * q[i6] * q[i11] * q[i16]
            + (5.555949e-05) * q[i6] * q[i11] * q[i19] + (-6.663878e-04) * q[i6] * q[i11] * q[i20] + (8.746138e-04) * q[i6] * q[i11] * q[i21]
            + (2.380237e-04) * q[i6] * q[i11] * q[i22] + (-9.200743e-04) * q[i6] * q[i12] * q[i15] + (4.833148e-04) * q[i6] * q[i12] * q[i16]
            + (6.442033e-04) * q[i6] * q[i12] * q[i19] + (-7.282235e-04) * q[i6] * q[i12] * q[i20] + (1.442067e-04) * q[i6] * q[i12] * q[i21]
            + (2.225201e-04) * q[i6] * q[i12] * q[i22] + (7.888703e-04) * q[i6] * q[i15] * q[i16] + (6.141175e-05) * q[i6] * q[i15] * q[i19]
            + (-2.685348e-04) * q[i6] * q[i15] * q[i20] + (-1.138220e-03) * q[i6] * q[i15] * q[i21] + (5.901552e-04) * q[i6] * q[i15] * q[i22]
            + (8.896395e-05) * q[i6] * q[i16] * q[i19] + (6.421954e-04) * q[i6] * q[i16] * q[i20] + (2.359060e-04) * q[i6] * q[i16] * q[i21]
            + (2.283668e-05) * q[i6] * q[i16] * q[i22] + (6.021812e-04) * q[i6] * q[i19] * q[i20] + (-5.092972e-04) * q[i6] * q[i19] * q[i21]
            + (4.299740e-04) * q[i6] * q[i19] * q[i22] + (-4.971938e-04) * q[i6] * q[i20] * q[i21] + (-8.179393e-04) * q[i6] * q[i20] * q[i22]
            + (3.492630e-04) * q[i6] * q[i21] * q[i22] + (3.671713e-04) * q[i7] * q[i8] * q[i9] + (-4.161039e-04) * q[i7] * q[i8] * q[i10]
            + (5.344024e-04) * q[i7] * q[i8] * q[i11] + (1.838683e-03) * q[i7] * q[i8] * q[i12] + (2.249842e-03) * q[i7] * q[i8] * q[i15]
            + (-6.333248e-04) * q[i7] * q[i8] * q[i16] + (-1.102029e-03) * q[i7] * q[i8] * q[i19] + (2.936866e-04) * q[i7] * q[i8] * q[i20]
            + (1.345406e-03) * q[i7] * q[i8] * q[i21] + (-1.009421e-03) * q[i7] * q[i8] * q[i22] + (-8.760234e-04) * q[i7] * q[i9] * q[i10]
            + (-4.594280e-05) * q[i7] * q[i9] * q[i11] + (7.649316e-04) * q[i7] * q[i9] * q[i12] + (-5.560971e-04) * q[i7] * q[i9] * q[i15]
            + (-4.782029e-05) * q[i7] * q[i9] * q[i16] + (-4.730683e-04) * q[i7] * q[i9] * q[i19] + (-1.099627e-03) * q[i7] * q[i9] * q[i20]
            + (1.866575e-04) * q[i7] * q[i9] * q[i21] + (7.260578e-04) * q[i7] * q[i9] * q[i22] + (-6.667777e-04) * q[i7] * q[i10] * q[i11]
            + (-5.476878e-04) * q[i7] * q[i10] * q[i12] + (3.970188e-04) * q[i7] * q[i10] * q[i15] + (-5.558833e-04) * q[i7] * q[i10] * q[i16]
            + (-8.172558e-04) * q[i7] * q[i10] * q[i19] + (5.768088e-04) * q[i7] * q[i10] * q[i20] + (-7.450826e-04) * q[i7] * q[i10] * q[i21]
            + (3.008438e-04) * q[i7] * q[i10] * q[i22] + (2.182957e-03) * q[i7] * q[i11] * q[i12] + (4.914696e-04) * q[i7] * q[i11] * q[i15]
            + (-9.167755e-04) * q[i7] * q[i11] * q[i16] + (-7.213827e-04) * q[i7] * q[i11] * q[i19] + (6.426055e-04) * q[i7] * q[i11] * q[i20]
            + (-2.241782e-04) * q[i7] * q[i11] * q[i21] + (-1.502159e-04) * q[i7] * q[i11] * q[i22] + (1.238090e-03) * q[i7] * q[i12] * q[i15]
            + (-8.871230e-05) * q[i7] * q[i12] * q[i16] + (-6.713509e-04) * q[i7] * q[i12] * q[i19] + (4.993456e-05) * q[i7] * q[i12] * q[i20]
            + (-2.370325e-04) * q[i7] * q[i12] * q[i21] + (-8.710689e-04) * q[i7] * q[i12] * q[i22] + (-7.862352e-04) * q[i7] * q[i15] * q[i16]
            + (-6.389798e-04) * q[i7] * q[i15] * q[i19] + (-9.214458e-05) * q[i7] * q[i15] * q[i20] + (2.972375e-05) * q[i7] * q[i15] * q[i21]
            + (2.369613e-04) * q[i7] * q[i15] * q[i22] + (2.711572e-04) * q[i7] * q[i16] * q[i19] + (-6.498497e-05) * q[i7] * q[i16] * q[i20]
            + (5.920154e-04) * q[i7] * q[i16] * q[i21] + (-1.133731e-03) * q[i7] * q[i16] * q[i22] + (-6.020368e-04) * q[i7] * q[i19] * q[i20]
            + (-8.144087e-04) * q[i7] * q[i19] * q[i21] + (-4.996268e-04) * q[i7] * q[i19] * q[i22] + (4.308491e-04) * q[i7] * q[i20] * q[i21]
            + (-5.089145e-04) * q[i7] * q[i20] * q[i22] + (-3.487774e-04) * q[i7] * q[i21] * q[i22] + (-2.654165e-06) * q[i8] * q[i9] * q[i10]
            + (9.049542e-04) * q[i8] * q[i9] * q[i11] + (1.520027e-04) * q[i8] * q[i9] * q[i12] + (-9.073218e-06) * q[i8] * q[i9] * q[i15]
            + (-1.216992e-03) * q[i8] * q[i9] * q[i16] + (1.033928e-04) * q[i8] * q[i9] * q[i19] + (5.290930e-04) * q[i8] * q[i9] * q[i20]
            + (-2.171296e-04) * q[i8] * q[i9] * q[i21] + (-2.532998e-04) * q[i8] * q[i9] * q[i22] + (1.550690e-04) * q[i8] * q[i10] * q[i11]
            + (9.047453e-04) * q[i8] * q[i10] * q[i12] + (1.210958e-03) * q[i8] * q[i10] * q[i15] + (7.722964e-06) * q[i8] * q[i10] * q[i16]
            + (-5.215299e-04) * q[i8] * q[i10] * q[i19] + (-1.035603e-04) * q[i8] * q[i10] * q[i20] + (-2.553434e-04) * q[i8] * q[i10] * q[i21]
            + (-2.204404e-04) * q[i8] * q[i10] * q[i22] + (6.327379e-06) * q[i8] * q[i11] * q[i12] + (2.458364e-03) * q[i8] * q[i11] * q[i15]
            + (2.508030e-04) * q[i8] * q[i11] * q[i16] + (-6.835541e-04) * q[i8] * q[i11] * q[i19] + (1.607269e-04) * q[i8] * q[i11] * q[i20]
            + (-8.277569e-04) * q[i8] * q[i11] * q[i21] + (-1.381067e-04) * q[i8] * q[i11] * q[i22] + (2.457640e-04) * q[i8] * q[i12] * q[i15]
            + (2.461207e-03) * q[i8] * q[i12] * q[i16] + (1.680650e-04) * q[i8] * q[i12] * q[i19] + (-6.569697e-04) * q[i8] * q[i12] * q[i20]
            + (1.356376e-04) * q[i8] * q[i12] * q[i21] + (8.294795e-04) * q[i8] * q[i12] * q[i22] + (8.123149e-06) * q[i8] * q[i15] * q[i16]
            + (7.750505e-04) * q[i8] * q[i15] * q[i19] + (1.638484e-04) * q[i8] * q[i15] * q[i20] + (2.465250e-03) * q[i8] * q[i15] * q[i21]
            + (-1.715073e-04) * q[i8] * q[i15] * q[i22] + (-1.567777e-04) * q[i8] * q[i16] * q[i19] + (-7.861975e-04) * q[i8] * q[i16] * q[i20]
            + (-1.693606e-04) * q[i8] * q[i16] * q[i21] + (2.504864e-03) * q[i8] * q[i16] * q[i22] + (5.985790e-06) * q[i8] * q[i19] * q[i20]
            + (1.998452e-03) * q[i8] * q[i19] * q[i21] + (-2.105038e-04) * q[i8] * q[i19] * q[i22] + (-2.078082e-04) * q[i8] * q[i20] * q[i21]
            + (1.988446e-03) * q[i8] * q[i20] * q[i22] + (1.711823e-06) * q[i8] * q[i21] * q[i22] + (1.286065e-04) * q[i9] * q[i10] * q[i11]
            + (1.250044e-04) * q[i9] * q[i10] * q[i12] + (-3.606545e-04) * q[i9] * q[i10] * q[i15] + (3.622643e-04) * q[i9] * q[i10] * q[i16]
            + (8.369679e-05) * q[i9] * q[i10] * q[i19] + (-8.075254e-05) * q[i9] * q[i10] * q[i20] + (3.387799e-04) * q[i9] * q[i10] * q[i21]
            + (3.377193e-04) * q[i9] * q[i10] * q[i22] + (-1.895063e-04) * q[i9] * q[i11] * q[i12] + (5.412181e-04) * q[i9] * q[i11] * q[i15]
            + (1.706533e-04) * q[i9] * q[i11] * q[i16] + (1.998468e-04) * q[i9] * q[i11] * q[i19] + (-1.656583e-04) * q[i9] * q[i11] * q[i20]
            + (4.284419e-04) * q[i9] * q[i11] * q[i21] + (1.575985e-04) * q[i9] * q[i11] * q[i22] + (-4.133039e-05) * q[i9] * q[i12] * q[i15]
            + (8.252603e-04) * q[i9] * q[i12] * q[i16] + (-8.015074e-05) * q[i9] * q[i12] * q[i19] + (-2.905708e-04) * q[i9] * q[i12] * q[i20]
            + (3.768899e-05) * q[i9] * q[i12] * q[i21] + (-2.169850e-04) * q[i9] * q[i12] * q[i22] + (-2.230831e-04) * q[i9] * q[i15] * q[i16]
            + (-5.647189e-05) * q[i9] * q[i15] * q[i19] + (1.652257e-04) * q[i9] * q[i15] * q[i20] + (-4.322648e-04) * q[i9] * q[i15] * q[i21]
            + (5.285697e-05) * q[i9] * q[i15] * q[i22] + (-3.671280e-05) * q[i9] * q[i16] * q[i19] + (1.781724e-04) * q[i9] * q[i16] * q[i20]
            + (-2.725853e-05) * q[i9] * q[i16] * q[i21] + (4.789656e-05) * q[i9] * q[i16] * q[i22] + (1.186678e-04) * q[i9] * q[i19] * q[i20]
            + (-3.199455e-04) * q[i9] * q[i19] * q[i21] + (1.559101e-04) * q[i9] * q[i19] * q[i22] + (-7.904078e-05) * q[i9] * q[i20] * q[i21]
            + (-1.632341e-04) * q[i9] * q[i20] * q[i22] + (4.792738e-05) * q[i9] * q[i21] * q[i22] + (1.848216e-04) * q[i10] * q[i11] * q[i12]
            + (8.219186e-04) * q[i10] * q[i11] * q[i15] + (-3.676150e-05) * q[i10] * q[i11] * q[i16] + (-2.833224e-04) * q[i10] * q[i11] * q[i19]
            + (-7.525957e-05) * q[i10] * q[i11] * q[i20] + (2.175368e-04) * q[i10] * q[i11] * q[i21] + (-3.807429e-05) * q[i10] * q[i11] * q[i22]
            + (1.745728e-04) * q[i10] * q[i12] * q[i15] + (5.422234e-04) * q[i10] * q[i12] * q[i16] + (-1.673986e-04) * q[i10] * q[i12] * q[i19]
            + (2.034290e-04) * q[i10] * q[i12] * q[i20] + (-1.576923e-04) * q[i10] * q[i12] * q[i21] + (-4.287859e-04) * q[i10] * q[i12] * q[i22]
            + (2.259030e-04) * q[i10] * q[i15] * q[i16] + (-1.702933e-04) * q[i10] * q[i15] * q[i19] + (3.417873e-05) * q[i10] * q[i15] * q[i20]
            + (4.745517e-05) * q[i10] * q[i15] * q[i21] + (-2.623439e-05) * q[i10] * q[i15] * q[i22] + (-1.618793e-04) * q[i10] * q[i16] * q[i19]
            + (5.807324e-05) * q[i10] * q[i16] * q[i20] + (5.007479e-05) * q[i10] * q[i16] * q[i21] + (-4.296329e-04) * q[i10] * q[i16] * q[i22]
            + (-1.181828e-04) * q[i10] * q[i19] * q[i20] + (-1.620898e-04) * q[i10] * q[i19] * q[i21] + (-7.845373e-05) * q[i10] * q[i19] * q[i22]
            + (1.556146e-04) * q[i10] * q[i20] * q[i21] + (-3.133288e-04) * q[i10] * q[i20] * q[i22] + (-4.459758e-05) * q[i10] * q[i21] * q[i22]
            + (-9.738384e-05) * q[i11] * q[i12] * q[i15] + (7.708771e-05) * q[i11] * q[i12] * q[i16] + (2.126881e-04) * q[i11] * q[i12] * q[i19]
            + (-2.092707e-04) * q[i11] * q[i12] * q[i20] + (2.520394e-04) * q[i11] * q[i12] * q[i21] + (2.587395e-04) * q[i11] * q[i12] * q[i22]
            + (4.304277e-06) * q[i11] * q[i15] * q[i16] + (4.307798e-04) * q[i11] * q[i15] * q[i19] + (-2.607958e-04) * q[i11] * q[i15] * q[i20]
            + (1.367234e-03) * q[i11] * q[i15] * q[i21] + (2.065659e-04) * q[i11] * q[i15] * q[i22] + (2.303527e-05) * q[i11] * q[i16] * q[i19]
            + (-4.123443e-05) * q[i11] * q[i16] * q[i20] + (1.331680e-04) * q[i11] * q[i16] * q[i21] + (2.692128e-05) * q[i11] * q[i16] * q[i22]
            + (6.704706e-05) * q[i11] * q[i19] * q[i20] + (1.175299e-03) * q[i11] * q[i19] * q[i21] + (6.339400e-05) * q[i11] * q[i19] * q[i22]
            + (9.226390e-05) * q[i11] * q[i20] * q[i21] + (-1.246492e-04) * q[i11] * q[i20] * q[i22] + (3.448838e-05) * q[i11] * q[i21] * q[i22]
            + (-4.345753e-06) * q[i12] * q[i15] * q[i16] + (-4.131142e-05) * q[i12] * q[i15] * q[i19] + (2.314826e-05) * q[i12] * q[i15] * q[i20]
            + (-3.017616e-05) * q[i12] * q[i15] * q[i21] + (-1.395952e-04) * q[i12] * q[i15] * q[i22] + (-2.640523e-04) * q[i12] * q[i16] * q[i19]
            + (4.473115e-04) * q[i12] * q[i16] * q[i20] + (-2.065151e-04) * q[i12] * q[i16] * q[i21] + (-1.411102e-03) * q[i12] * q[i16] * q[i22]
            + (7.135416e-05) * q[i12] * q[i19] * q[i20] + (1.233256e-04) * q[i12] * q[i19] * q[i21] + (-9.640700e-05) * q[i12] * q[i19] * q[i22]
            + (-6.256477e-05) * q[i12] * q[i20] * q[i21] + (-1.167479e-03) * q[i12] * q[i20] * q[i22] + (3.444092e-05) * q[i12] * q[i21] * q[i22]
            + (2.386171e-04) * q[i15] * q[i16] * q[i19] + (-2.361487e-04) * q[i15] * q[i16] * q[i20] + (-2.004290e-04) * q[i15] * q[i16] * q[i21]
            + (-2.019045e-04) * q[i15] * q[i16] * q[i22] + (3.712005e-06) * q[i15] * q[i19] * q[i20] + (4.275000e-04) * q[i15] * q[i19] * q[i21]
            + (4.808193e-05) * q[i15] * q[i19] * q[i22] + (-1.281918e-04) * q[i15] * q[i20] * q[i21] + (9.076848e-05) * q[i15] * q[i20] * q[i22]
            + (1.820690e-04) * q[i15] * q[i21] * q[i22] + (-2.982123e-06) * q[i16] * q[i19] * q[i20] + (9.156044e-05) * q[i16] * q[i19] * q[i21]
            + (-1.290174e-04) * q[i16] * q[i19] * q[i22] + (5.082644e-05) * q[i16] * q[i20] * q[i21] + (4.165926e-04) * q[i16] * q[i20] * q[i22]
            + (-1.806560e-04) * q[i16] * q[i21] * q[i22] + (-1.008673e-04) * q[i19] * q[i20] * q[i21] + (-1.018229e-04) * q[i19] * q[i20] * q[i22]
            + (-1.219829e-04) * q[i19] * q[i21] * q[i22] + (1.231277e-04) * q[i20] * q[i21] * q[i22];
      return Qz;
   }

   //==========================================================================================================================================

   public void getJQx(double[] q, double[][] JQ)
   {
      getJQx0(q, JQ);
      getJQx1(q, JQ);
      getJQx2(q, JQ);
      getJQx3(q, JQ);
      getJQx4(q, JQ);
      getJQx5(q, JQ);
      getJQx6(q, JQ);
      getJQx7(q, JQ);
      getJQx8(q, JQ);
      getJQx9(q, JQ);
      getJQx10(q, JQ);
      getJQx11(q, JQ);
      getJQx12(q, JQ);
      getJQx15(q, JQ);
      getJQx16(q, JQ);
      getJQx19(q, JQ);
      getJQx20(q, JQ);
      getJQx21(q, JQ);
      getJQx22(q, JQ);
   }

   public void getJQx0(double[] q, double[][] JQ)
   {
      JQ[1][i0] = (-1.091470e-02) * (1) + (-4.763695e-03) * ((2) * q[i0]) + (-3.072205e-05) * (q[i1]) + (-2.215087e-04) * (q[i2]) + (-1.614664e-03) * (q[i3])
            + (-4.964491e-03) * (q[i4]) + (2.972583e-03) * (q[i5]) + (9.063135e-02) * (q[i6]) + (-1.051662e-02) * (q[i7]) + (-3.638438e-03) * (q[i8])
            + (2.931941e-02) * (q[i9]) + (-1.566182e-03) * (q[i10]) + (2.431821e-03) * (q[i11]) + (-2.645071e-04) * (q[i12]) + (-3.495047e-04) * (q[i15])
            + (2.440634e-03) * (q[i16]) + (-2.003494e-03) * (q[i19]) + (6.647019e-04) * (q[i20]) + (1.479821e-03) * (q[i21]) + (3.335293e-04) * (q[i22])
            + (1.626362e-03) * ((3) * q[i0] * q[i0]) + (-3.464843e-03) * ((2) * q[i0] * q[i1]) + (2.556321e-03) * ((2) * q[i0] * q[i2])
            + (-3.012961e-02) * ((2) * q[i0] * q[i3]) + (-6.835978e-05) * ((2) * q[i0] * q[i4]) + (-3.507112e-03) * ((2) * q[i0] * q[i5])
            + (1.253647e-03) * ((2) * q[i0] * q[i6]) + (2.032812e-03) * ((2) * q[i0] * q[i7]) + (-1.493391e-03) * ((2) * q[i0] * q[i8])
            + (2.952615e-03) * ((2) * q[i0] * q[i9]) + (-2.826019e-04) * ((2) * q[i0] * q[i10]) + (7.179850e-04) * ((2) * q[i0] * q[i11])
            + (4.998808e-06) * ((2) * q[i0] * q[i12]) + (-1.196572e-03) * ((2) * q[i0] * q[i15]) + (-2.972802e-04) * ((2) * q[i0] * q[i16])
            + (-4.202603e-05) * ((2) * q[i0] * q[i19]) + (-3.548803e-04) * ((2) * q[i0] * q[i20]) + (6.331864e-04) * ((2) * q[i0] * q[i21])
            + (-2.889568e-04) * ((2) * q[i0] * q[i22]) + (-3.444680e-03) * (q[i1] * q[i1]) + (1.645876e-04) * (q[i2] * q[i2])
            + (-2.153409e-03) * (q[i3] * q[i3]) + (1.433173e-03) * (q[i4] * q[i4]) + (-2.019421e-03) * (q[i5] * q[i5]) + (3.867929e-03) * (q[i6] * q[i6])
            + (-3.453669e-03) * (q[i7] * q[i7]) + (1.790137e-04) * (q[i8] * q[i8]) + (-6.786792e-03) * (q[i9] * q[i9]) + (1.268991e-03) * (q[i10] * q[i10])
            + (2.634201e-04) * (q[i11] * q[i11]) + (2.812292e-04) * (q[i12] * q[i12]) + (-9.181986e-05) * (q[i15] * q[i15]) + (6.728841e-05) * (q[i16] * q[i16])
            + (-1.738111e-05) * (q[i19] * q[i19]) + (-1.050280e-04) * (q[i20] * q[i20]) + (2.005327e-04) * (q[i21] * q[i21])
            + (3.563537e-04) * (q[i22] * q[i22]) + (-4.256968e-03) * (q[i1] * q[i2]) + (7.202945e-03) * (q[i1] * q[i3]) + (7.154852e-03) * (q[i1] * q[i4])
            + (-5.894587e-03) * (q[i1] * q[i5]) + (1.950107e-03) * (q[i1] * q[i6]) + (-1.925395e-03) * (q[i1] * q[i7]) + (4.442005e-06) * (q[i1] * q[i8])
            + (-1.418768e-03) * (q[i1] * q[i9]) + (1.421309e-03) * (q[i1] * q[i10]) + (-8.110965e-04) * (q[i1] * q[i11]) + (-8.088057e-04) * (q[i1] * q[i12])
            + (1.151046e-03) * (q[i1] * q[i15]) + (-1.169626e-03) * (q[i1] * q[i16]) + (-5.858271e-04) * (q[i1] * q[i19]) + (5.868450e-04) * (q[i1] * q[i20])
            + (2.626669e-04) * (q[i1] * q[i21]) + (2.594540e-04) * (q[i1] * q[i22]) + (-1.283325e-02) * (q[i2] * q[i3]) + (1.855208e-03) * (q[i2] * q[i4])
            + (-1.194723e-02) * (q[i2] * q[i5]) + (1.345896e-03) * (q[i2] * q[i6]) + (-2.001371e-03) * (q[i2] * q[i7]) + (-3.725796e-03) * (q[i2] * q[i8])
            + (1.292606e-03) * (q[i2] * q[i9]) + (5.500446e-04) * (q[i2] * q[i10]) + (6.652454e-04) * (q[i2] * q[i11]) + (1.396673e-03) * (q[i2] * q[i12])
            + (-6.568085e-05) * (q[i2] * q[i15]) + (1.546440e-03) * (q[i2] * q[i16]) + (4.345169e-04) * (q[i2] * q[i19]) + (1.349169e-03) * (q[i2] * q[i20])
            + (5.814716e-04) * (q[i2] * q[i21]) + (1.058040e-04) * (q[i2] * q[i22]) + (-1.104364e-05) * (q[i3] * q[i4]) + (6.232324e-04) * (q[i3] * q[i5])
            + (-1.220277e-02) * (q[i3] * q[i6]) + (4.599825e-04) * (q[i3] * q[i7]) + (2.879513e-03) * (q[i3] * q[i8]) + (-1.020871e-03) * (q[i3] * q[i9])
            + (1.436438e-03) * (q[i3] * q[i10]) + (4.059029e-05) * (q[i3] * q[i11]) + (-1.685471e-03) * (q[i3] * q[i12]) + (2.231693e-03) * (q[i3] * q[i15])
            + (8.670147e-04) * (q[i3] * q[i16]) + (1.522527e-03) * (q[i3] * q[i19]) + (1.245394e-03) * (q[i3] * q[i20]) + (-8.057844e-04) * (q[i3] * q[i21])
            + (-7.652511e-04) * (q[i3] * q[i22]) + (4.044206e-03) * (q[i4] * q[i5]) + (-3.852007e-03) * (q[i4] * q[i6]) + (-2.147826e-04) * (q[i4] * q[i7])
            + (-1.860052e-03) * (q[i4] * q[i8]) + (-6.762930e-04) * (q[i4] * q[i9]) + (7.799416e-04) * (q[i4] * q[i10]) + (1.098684e-03) * (q[i4] * q[i11])
            + (-1.804400e-03) * (q[i4] * q[i12]) + (-5.569488e-04) * (q[i4] * q[i15]) + (-3.701069e-04) * (q[i4] * q[i16]) + (-1.016211e-03) * (q[i4] * q[i19])
            + (-9.938613e-04) * (q[i4] * q[i20]) + (-3.788516e-04) * (q[i4] * q[i21]) + (6.018258e-04) * (q[i4] * q[i22]) + (-8.060198e-03) * (q[i5] * q[i6])
            + (-4.824738e-04) * (q[i5] * q[i7]) + (-3.618220e-04) * (q[i5] * q[i8]) + (4.046716e-04) * (q[i5] * q[i9]) + (-8.990503e-04) * (q[i5] * q[i10])
            + (-6.948002e-05) * (q[i5] * q[i11]) + (2.869565e-04) * (q[i5] * q[i12]) + (7.623461e-04) * (q[i5] * q[i15]) + (-1.635196e-03) * (q[i5] * q[i16])
            + (-8.694627e-04) * (q[i5] * q[i19]) + (1.593632e-04) * (q[i5] * q[i20]) + (6.680767e-04) * (q[i5] * q[i21]) + (-2.400176e-04) * (q[i5] * q[i22])
            + (-4.657516e-04) * (q[i6] * q[i7]) + (7.409014e-03) * (q[i6] * q[i8]) + (-1.360672e-02) * (q[i6] * q[i9]) + (1.714814e-03) * (q[i6] * q[i10])
            + (-1.493674e-03) * (q[i6] * q[i11]) + (3.205563e-04) * (q[i6] * q[i12]) + (3.511050e-04) * (q[i6] * q[i15]) + (2.069774e-03) * (q[i6] * q[i16])
            + (1.222475e-03) * (q[i6] * q[i19]) + (1.359705e-03) * (q[i6] * q[i20]) + (-8.987898e-04) * (q[i6] * q[i21]) + (6.300372e-04) * (q[i6] * q[i22])
            + (-1.297956e-03) * (q[i7] * q[i8]) + (-1.327951e-04) * (q[i7] * q[i9]) + (3.413640e-03) * (q[i7] * q[i10]) + (-9.047856e-04) * (q[i7] * q[i11])
            + (-1.669194e-03) * (q[i7] * q[i12]) + (-6.133359e-04) * (q[i7] * q[i15]) + (7.881685e-04) * (q[i7] * q[i16]) + (-6.159552e-04) * (q[i7] * q[i19])
            + (-9.066067e-04) * (q[i7] * q[i20]) + (-2.006549e-04) * (q[i7] * q[i21]) + (-8.392349e-04) * (q[i7] * q[i22]) + (7.407671e-04) * (q[i8] * q[i9])
            + (-1.721679e-04) * (q[i8] * q[i10]) + (-5.959634e-04) * (q[i8] * q[i11]) + (1.649874e-04) * (q[i8] * q[i12]) + (9.418310e-04) * (q[i8] * q[i15])
            + (2.296953e-04) * (q[i8] * q[i16]) + (1.349733e-03) * (q[i8] * q[i19]) + (1.054646e-03) * (q[i8] * q[i20]) + (3.697073e-04) * (q[i8] * q[i21])
            + (-3.461998e-04) * (q[i8] * q[i22]) + (7.379308e-04) * (q[i9] * q[i10]) + (1.292349e-05) * (q[i9] * q[i11]) + (-5.578639e-04) * (q[i9] * q[i12])
            + (7.242263e-04) * (q[i9] * q[i15]) + (2.030050e-04) * (q[i9] * q[i16]) + (8.202329e-04) * (q[i9] * q[i19]) + (5.663061e-04) * (q[i9] * q[i20])
            + (-4.827697e-04) * (q[i9] * q[i21]) + (-5.376229e-05) * (q[i9] * q[i22]) + (-8.957915e-04) * (q[i10] * q[i11])
            + (-5.991964e-05) * (q[i10] * q[i12]) + (-8.095689e-05) * (q[i10] * q[i15]) + (1.579682e-04) * (q[i10] * q[i16])
            + (-3.351315e-04) * (q[i10] * q[i19]) + (-6.638931e-04) * (q[i10] * q[i20]) + (-6.733610e-04) * (q[i10] * q[i21])
            + (-4.308028e-04) * (q[i10] * q[i22]) + (1.037433e-03) * (q[i11] * q[i12]) + (1.111723e-03) * (q[i11] * q[i15])
            + (-6.870321e-05) * (q[i11] * q[i16]) + (6.155529e-04) * (q[i11] * q[i19]) + (-2.593303e-04) * (q[i11] * q[i20])
            + (7.605467e-04) * (q[i11] * q[i21]) + (3.498287e-04) * (q[i11] * q[i22]) + (-3.508971e-04) * (q[i12] * q[i15])
            + (-3.126500e-04) * (q[i12] * q[i16]) + (1.308050e-04) * (q[i12] * q[i19]) + (-5.335765e-04) * (q[i12] * q[i20])
            + (4.240182e-04) * (q[i12] * q[i21]) + (-7.301080e-04) * (q[i12] * q[i22]) + (5.148387e-04) * (q[i15] * q[i16])
            + (-1.717268e-04) * (q[i15] * q[i19]) + (4.410306e-04) * (q[i15] * q[i20]) + (2.398073e-04) * (q[i15] * q[i21]) + (5.474682e-04) * (q[i15] * q[i22])
            + (-1.048530e-03) * (q[i16] * q[i19]) + (8.003445e-04) * (q[i16] * q[i20]) + (-2.354564e-04) * (q[i16] * q[i21])
            + (-2.929430e-05) * (q[i16] * q[i22]) + (4.840219e-04) * (q[i19] * q[i20]) + (-2.430971e-04) * (q[i19] * q[i21])
            + (-5.005325e-04) * (q[i19] * q[i22]) + (7.464442e-04) * (q[i20] * q[i21]) + (8.370573e-04) * (q[i20] * q[i22])
            + (1.707779e-04) * (q[i21] * q[i22]);
   }

   public void getJQx1(double[] q, double[][] JQ)
   {
      JQ[1][i1] = (-1.075516e-02) * (1) + (4.731760e-03) * ((2) * q[i1]) + (-3.072205e-05) * (q[i0]) + (1.546791e-04) * (q[i2]) + (4.911325e-03) * (q[i3])
            + (1.639490e-03) * (q[i4]) + (-2.945678e-03) * (q[i5]) + (-1.054391e-02) * (q[i6]) + (9.010674e-02) * (q[i7]) + (-3.676940e-03) * (q[i8])
            + (-1.585410e-03) * (q[i9]) + (2.896086e-02) * (q[i10]) + (2.549366e-04) * (q[i11]) + (-2.437312e-03) * (q[i12]) + (2.488392e-03) * (q[i15])
            + (-3.292644e-04) * (q[i16]) + (7.010237e-04) * (q[i19]) + (-1.956615e-03) * (q[i20]) + (-3.335710e-04) * (q[i21]) + (-1.478069e-03) * (q[i22])
            + (-3.464843e-03) * (q[i0] * q[i0]) + (-3.444680e-03) * ((2) * q[i0] * q[i1]) + (1.609090e-03) * ((3) * q[i1] * q[i1])
            + (2.557233e-03) * ((2) * q[i1] * q[i2]) + (-6.150220e-05) * ((2) * q[i1] * q[i3]) + (-3.004084e-02) * ((2) * q[i1] * q[i4])
            + (-3.462911e-03) * ((2) * q[i1] * q[i5]) + (-2.010742e-03) * ((2) * q[i1] * q[i6]) + (-1.266027e-03) * ((2) * q[i1] * q[i7])
            + (1.472293e-03) * ((2) * q[i1] * q[i8]) + (3.075456e-04) * ((2) * q[i1] * q[i9]) + (-2.936930e-03) * ((2) * q[i1] * q[i10])
            + (2.325739e-05) * ((2) * q[i1] * q[i11]) + (7.082192e-04) * ((2) * q[i1] * q[i12]) + (2.994997e-04) * ((2) * q[i1] * q[i15])
            + (1.192890e-03) * ((2) * q[i1] * q[i16]) + (3.554884e-04) * ((2) * q[i1] * q[i19]) + (5.501578e-05) * ((2) * q[i1] * q[i20])
            + (-2.780366e-04) * ((2) * q[i1] * q[i21]) + (6.328589e-04) * ((2) * q[i1] * q[i22]) + (1.444696e-04) * (q[i2] * q[i2])
            + (1.442191e-03) * (q[i3] * q[i3]) + (-2.141928e-03) * (q[i4] * q[i4]) + (-1.986907e-03) * (q[i5] * q[i5]) + (-3.455323e-03) * (q[i6] * q[i6])
            + (3.902302e-03) * (q[i7] * q[i7]) + (2.022371e-04) * (q[i8] * q[i8]) + (1.277800e-03) * (q[i9] * q[i9]) + (-6.708023e-03) * (q[i10] * q[i10])
            + (3.093027e-04) * (q[i11] * q[i11]) + (2.557378e-04) * (q[i12] * q[i12]) + (7.592240e-05) * (q[i15] * q[i15]) + (-9.215216e-05) * (q[i16] * q[i16])
            + (-1.020428e-04) * (q[i19] * q[i19]) + (-2.351599e-05) * (q[i20] * q[i20]) + (3.551545e-04) * (q[i21] * q[i21])
            + (1.959782e-04) * (q[i22] * q[i22]) + (-4.256968e-03) * (q[i0] * q[i2]) + (7.202945e-03) * (q[i0] * q[i3]) + (7.154852e-03) * (q[i0] * q[i4])
            + (-5.894587e-03) * (q[i0] * q[i5]) + (1.950107e-03) * (q[i0] * q[i6]) + (-1.925395e-03) * (q[i0] * q[i7]) + (4.442005e-06) * (q[i0] * q[i8])
            + (-1.418768e-03) * (q[i0] * q[i9]) + (1.421309e-03) * (q[i0] * q[i10]) + (-8.110965e-04) * (q[i0] * q[i11]) + (-8.088057e-04) * (q[i0] * q[i12])
            + (1.151046e-03) * (q[i0] * q[i15]) + (-1.169626e-03) * (q[i0] * q[i16]) + (-5.858271e-04) * (q[i0] * q[i19]) + (5.868450e-04) * (q[i0] * q[i20])
            + (2.626669e-04) * (q[i0] * q[i21]) + (2.594540e-04) * (q[i0] * q[i22]) + (1.897785e-03) * (q[i2] * q[i3]) + (-1.277614e-02) * (q[i2] * q[i4])
            + (-1.187656e-02) * (q[i2] * q[i5]) + (2.015449e-03) * (q[i2] * q[i6]) + (-1.331022e-03) * (q[i2] * q[i7]) + (3.702176e-03) * (q[i2] * q[i8])
            + (-5.259012e-04) * (q[i2] * q[i9]) + (-1.279786e-03) * (q[i2] * q[i10]) + (1.404683e-03) * (q[i2] * q[i11]) + (6.646431e-04) * (q[i2] * q[i12])
            + (-1.531927e-03) * (q[i2] * q[i15]) + (6.903362e-05) * (q[i2] * q[i16]) + (-1.356130e-03) * (q[i2] * q[i19]) + (-4.233245e-04) * (q[i2] * q[i20])
            + (1.109319e-04) * (q[i2] * q[i21]) + (5.835975e-04) * (q[i2] * q[i22]) + (6.927884e-06) * (q[i3] * q[i4]) + (4.044748e-03) * (q[i3] * q[i5])
            + (1.776476e-04) * (q[i3] * q[i6]) + (3.785884e-03) * (q[i3] * q[i7]) + (1.894027e-03) * (q[i3] * q[i8]) + (-7.905932e-04) * (q[i3] * q[i9])
            + (6.715230e-04) * (q[i3] * q[i10]) + (-1.809140e-03) * (q[i3] * q[i11]) + (1.062638e-03) * (q[i3] * q[i12]) + (3.459426e-04) * (q[i3] * q[i15])
            + (5.483259e-04) * (q[i3] * q[i16]) + (9.862217e-04) * (q[i3] * q[i19]) + (1.000061e-03) * (q[i3] * q[i20]) + (5.962383e-04) * (q[i3] * q[i21])
            + (-3.858473e-04) * (q[i3] * q[i22]) + (6.049404e-04) * (q[i4] * q[i5]) + (-4.309909e-04) * (q[i4] * q[i6]) + (1.215547e-02) * (q[i4] * q[i7])
            + (-2.884023e-03) * (q[i4] * q[i8]) + (-1.469909e-03) * (q[i4] * q[i9]) + (1.007366e-03) * (q[i4] * q[i10]) + (-1.716275e-03) * (q[i4] * q[i11])
            + (6.743792e-05) * (q[i4] * q[i12]) + (-8.506545e-04) * (q[i4] * q[i15]) + (-2.229472e-03) * (q[i4] * q[i16]) + (-1.241991e-03) * (q[i4] * q[i19])
            + (-1.501474e-03) * (q[i4] * q[i20]) + (-7.710065e-04) * (q[i4] * q[i21]) + (-8.268980e-04) * (q[i4] * q[i22]) + (4.644945e-04) * (q[i5] * q[i6])
            + (8.116384e-03) * (q[i5] * q[i7]) + (3.976265e-04) * (q[i5] * q[i8]) + (8.545363e-04) * (q[i5] * q[i9]) + (-3.880431e-04) * (q[i5] * q[i10])
            + (3.387693e-04) * (q[i5] * q[i11]) + (-6.398775e-05) * (q[i5] * q[i12]) + (1.632663e-03) * (q[i5] * q[i15]) + (-7.627753e-04) * (q[i5] * q[i16])
            + (-1.608649e-04) * (q[i5] * q[i19]) + (8.323787e-04) * (q[i5] * q[i20]) + (-2.455253e-04) * (q[i5] * q[i21]) + (6.741223e-04) * (q[i5] * q[i22])
            + (-5.072719e-04) * (q[i6] * q[i7]) + (-1.312707e-03) * (q[i6] * q[i8]) + (3.417191e-03) * (q[i6] * q[i9]) + (-1.111062e-04) * (q[i6] * q[i10])
            + (1.669093e-03) * (q[i6] * q[i11]) + (9.281863e-04) * (q[i6] * q[i12]) + (7.933511e-04) * (q[i6] * q[i15]) + (-6.163040e-04) * (q[i6] * q[i16])
            + (-9.142841e-04) * (q[i6] * q[i19]) + (-6.114105e-04) * (q[i6] * q[i20]) + (8.439397e-04) * (q[i6] * q[i21]) + (1.852810e-04) * (q[i6] * q[i22])
            + (7.388077e-03) * (q[i7] * q[i8]) + (1.747766e-03) * (q[i7] * q[i9]) + (-1.345148e-02) * (q[i7] * q[i10]) + (-2.929429e-04) * (q[i7] * q[i11])
            + (1.519737e-03) * (q[i7] * q[i12]) + (2.038911e-03) * (q[i7] * q[i15]) + (3.758379e-04) * (q[i7] * q[i16]) + (1.349245e-03) * (q[i7] * q[i19])
            + (1.199835e-03) * (q[i7] * q[i20]) + (-6.180849e-04) * (q[i7] * q[i21]) + (9.134953e-04) * (q[i7] * q[i22]) + (-1.896148e-04) * (q[i8] * q[i9])
            + (7.417262e-04) * (q[i8] * q[i10]) + (-1.253281e-04) * (q[i8] * q[i11]) + (6.184059e-04) * (q[i8] * q[i12]) + (2.099563e-04) * (q[i8] * q[i15])
            + (9.333350e-04) * (q[i8] * q[i16]) + (1.060160e-03) * (q[i8] * q[i19]) + (1.348211e-03) * (q[i8] * q[i20]) + (3.351103e-04) * (q[i8] * q[i21])
            + (-3.669231e-04) * (q[i8] * q[i22]) + (7.396080e-04) * (q[i9] * q[i10]) + (6.116578e-05) * (q[i9] * q[i11]) + (9.128566e-04) * (q[i9] * q[i12])
            + (1.494251e-04) * (q[i9] * q[i15]) + (-7.386152e-05) * (q[i9] * q[i16]) + (-6.724032e-04) * (q[i9] * q[i19]) + (-3.448627e-04) * (q[i9] * q[i20])
            + (4.375698e-04) * (q[i9] * q[i21]) + (6.739413e-04) * (q[i9] * q[i22]) + (5.542927e-04) * (q[i10] * q[i11]) + (-1.430565e-05) * (q[i10] * q[i12])
            + (1.871632e-04) * (q[i10] * q[i15]) + (7.150911e-04) * (q[i10] * q[i16]) + (5.671184e-04) * (q[i10] * q[i19]) + (8.148392e-04) * (q[i10] * q[i20])
            + (5.603105e-05) * (q[i10] * q[i21]) + (4.743153e-04) * (q[i10] * q[i22]) + (1.050391e-03) * (q[i11] * q[i12]) + (2.987142e-04) * (q[i11] * q[i15])
            + (3.494695e-04) * (q[i11] * q[i16]) + (5.263009e-04) * (q[i11] * q[i19]) + (-1.317697e-04) * (q[i11] * q[i20])
            + (-7.379106e-04) * (q[i11] * q[i21]) + (4.227531e-04) * (q[i11] * q[i22]) + (5.891167e-05) * (q[i12] * q[i15])
            + (-1.117033e-03) * (q[i12] * q[i16]) + (2.595780e-04) * (q[i12] * q[i19]) + (-6.142505e-04) * (q[i12] * q[i20])
            + (3.496778e-04) * (q[i12] * q[i21]) + (7.630297e-04) * (q[i12] * q[i22]) + (5.088894e-04) * (q[i15] * q[i16]) + (8.008115e-04) * (q[i15] * q[i19])
            + (-1.042180e-03) * (q[i15] * q[i20]) + (2.924900e-05) * (q[i15] * q[i21]) + (2.277305e-04) * (q[i15] * q[i22]) + (4.527292e-04) * (q[i16] * q[i19])
            + (-1.634519e-04) * (q[i16] * q[i20]) + (-5.340071e-04) * (q[i16] * q[i21]) + (-2.454685e-04) * (q[i16] * q[i22])
            + (4.608439e-04) * (q[i19] * q[i20]) + (-8.342090e-04) * (q[i19] * q[i21]) + (-7.396063e-04) * (q[i19] * q[i22])
            + (5.044497e-04) * (q[i20] * q[i21]) + (2.404725e-04) * (q[i20] * q[i22]) + (1.760598e-04) * (q[i21] * q[i22]);
   }

   public void getJQx2(double[] q, double[][] JQ)
   {
      JQ[1][i2] = (-6.493713e-04) * (1) + (-1.342401e-05) * ((2) * q[i2]) + (-2.215087e-04) * (q[i0]) + (1.546791e-04) * (q[i1]) + (9.295948e-04) * (q[i3])
            + (-1.007355e-03) * (q[i4]) + (9.527315e-05) * (q[i5]) + (1.591417e-02) * (q[i6]) + (1.577288e-02) * (q[i7]) + (-6.770789e-02) * (q[i8])
            + (3.152888e-03) * (q[i9]) + (3.116881e-03) * (q[i10]) + (5.278294e-03) * (q[i11]) + (-5.454492e-03) * (q[i12]) + (2.469027e-03) * (q[i15])
            + (2.522821e-03) * (q[i16]) + (-2.095669e-03) * (q[i19]) + (-2.100913e-03) * (q[i20]) + (1.403467e-03) * (q[i21]) + (-1.403050e-03) * (q[i22])
            + (2.556321e-03) * (q[i0] * q[i0]) + (2.557233e-03) * (q[i1] * q[i1]) + (1.645876e-04) * ((2) * q[i0] * q[i2])
            + (1.444696e-04) * ((2) * q[i1] * q[i2]) + (2.998311e-03) * ((3) * q[i2] * q[i2]) + (-1.410196e-03) * ((2) * q[i2] * q[i3])
            + (-1.390624e-03) * ((2) * q[i2] * q[i4]) + (-2.711721e-02) * ((2) * q[i2] * q[i5]) + (9.419021e-04) * ((2) * q[i2] * q[i6])
            + (-9.400876e-04) * ((2) * q[i2] * q[i7]) + (-1.694320e-05) * ((2) * q[i2] * q[i8]) + (1.309976e-04) * ((2) * q[i2] * q[i9])
            + (-1.301474e-04) * ((2) * q[i2] * q[i10]) + (6.982843e-04) * ((2) * q[i2] * q[i11]) + (6.892704e-04) * ((2) * q[i2] * q[i12])
            + (-2.546112e-03) * ((2) * q[i2] * q[i15]) + (2.587146e-03) * ((2) * q[i2] * q[i16]) + (1.521570e-04) * ((2) * q[i2] * q[i19])
            + (-1.591219e-04) * ((2) * q[i2] * q[i20]) + (1.112668e-03) * ((2) * q[i2] * q[i21]) + (1.105582e-03) * ((2) * q[i2] * q[i22])
            + (-2.028160e-03) * (q[i3] * q[i3]) + (-2.034666e-03) * (q[i4] * q[i4]) + (-4.393894e-04) * (q[i5] * q[i5]) + (-4.848208e-04) * (q[i6] * q[i6])
            + (-4.773915e-04) * (q[i7] * q[i7]) + (1.764850e-03) * (q[i8] * q[i8]) + (-2.022016e-03) * (q[i9] * q[i9]) + (-1.993552e-03) * (q[i10] * q[i10])
            + (-4.927059e-05) * (q[i11] * q[i11]) + (-1.504999e-04) * (q[i12] * q[i12]) + (1.748959e-03) * (q[i15] * q[i15])
            + (1.766318e-03) * (q[i16] * q[i16]) + (-2.030081e-04) * (q[i19] * q[i19]) + (-2.080122e-04) * (q[i20] * q[i20])
            + (6.122352e-04) * (q[i21] * q[i21]) + (6.068116e-04) * (q[i22] * q[i22]) + (-4.256968e-03) * (q[i0] * q[i1]) + (-1.283325e-02) * (q[i0] * q[i3])
            + (1.855208e-03) * (q[i0] * q[i4]) + (-1.194723e-02) * (q[i0] * q[i5]) + (1.345896e-03) * (q[i0] * q[i6]) + (-2.001371e-03) * (q[i0] * q[i7])
            + (-3.725796e-03) * (q[i0] * q[i8]) + (1.292606e-03) * (q[i0] * q[i9]) + (5.500446e-04) * (q[i0] * q[i10]) + (6.652454e-04) * (q[i0] * q[i11])
            + (1.396673e-03) * (q[i0] * q[i12]) + (-6.568085e-05) * (q[i0] * q[i15]) + (1.546440e-03) * (q[i0] * q[i16]) + (4.345169e-04) * (q[i0] * q[i19])
            + (1.349169e-03) * (q[i0] * q[i20]) + (5.814716e-04) * (q[i0] * q[i21]) + (1.058040e-04) * (q[i0] * q[i22]) + (1.897785e-03) * (q[i1] * q[i3])
            + (-1.277614e-02) * (q[i1] * q[i4]) + (-1.187656e-02) * (q[i1] * q[i5]) + (2.015449e-03) * (q[i1] * q[i6]) + (-1.331022e-03) * (q[i1] * q[i7])
            + (3.702176e-03) * (q[i1] * q[i8]) + (-5.259012e-04) * (q[i1] * q[i9]) + (-1.279786e-03) * (q[i1] * q[i10]) + (1.404683e-03) * (q[i1] * q[i11])
            + (6.646431e-04) * (q[i1] * q[i12]) + (-1.531927e-03) * (q[i1] * q[i15]) + (6.903362e-05) * (q[i1] * q[i16]) + (-1.356130e-03) * (q[i1] * q[i19])
            + (-4.233245e-04) * (q[i1] * q[i20]) + (1.109319e-04) * (q[i1] * q[i21]) + (5.835975e-04) * (q[i1] * q[i22]) + (2.120604e-03) * (q[i3] * q[i4])
            + (5.379433e-03) * (q[i3] * q[i5]) + (-3.754849e-03) * (q[i3] * q[i6]) + (1.078962e-03) * (q[i3] * q[i7]) + (3.084937e-03) * (q[i3] * q[i8])
            + (1.821206e-04) * (q[i3] * q[i9]) + (-4.290648e-04) * (q[i3] * q[i10]) + (5.062225e-04) * (q[i3] * q[i11]) + (-4.700165e-04) * (q[i3] * q[i12])
            + (1.418867e-03) * (q[i3] * q[i15]) + (1.015846e-04) * (q[i3] * q[i16]) + (-9.577654e-04) * (q[i3] * q[i19]) + (5.090413e-04) * (q[i3] * q[i20])
            + (-7.309653e-04) * (q[i3] * q[i21]) + (1.000350e-03) * (q[i3] * q[i22]) + (5.443058e-03) * (q[i4] * q[i5]) + (-1.066613e-03) * (q[i4] * q[i6])
            + (3.717713e-03) * (q[i4] * q[i7]) + (-3.042462e-03) * (q[i4] * q[i8]) + (4.203955e-04) * (q[i4] * q[i9]) + (-1.730749e-04) * (q[i4] * q[i10])
            + (-4.936573e-04) * (q[i4] * q[i11]) + (5.365713e-04) * (q[i4] * q[i12]) + (-9.815592e-05) * (q[i4] * q[i15]) + (-1.455408e-03) * (q[i4] * q[i16])
            + (-5.021187e-04) * (q[i4] * q[i19]) + (9.312876e-04) * (q[i4] * q[i20]) + (9.853955e-04) * (q[i4] * q[i21]) + (-7.527671e-04) * (q[i4] * q[i22])
            + (-1.897680e-04) * (q[i5] * q[i6]) + (1.789531e-04) * (q[i5] * q[i7]) + (-2.629788e-05) * (q[i5] * q[i8]) + (2.116555e-03) * (q[i5] * q[i9])
            + (-2.136809e-03) * (q[i5] * q[i10]) + (1.737642e-03) * (q[i5] * q[i11]) + (1.689365e-03) * (q[i5] * q[i12]) + (-1.444219e-03) * (q[i5] * q[i15])
            + (1.441259e-03) * (q[i5] * q[i16]) + (1.186527e-03) * (q[i5] * q[i19]) + (-1.194357e-03) * (q[i5] * q[i20]) + (5.688177e-04) * (q[i5] * q[i21])
            + (5.556571e-04) * (q[i5] * q[i22]) + (3.344318e-03) * (q[i6] * q[i7]) + (5.734035e-03) * (q[i6] * q[i8]) + (-2.505272e-03) * (q[i6] * q[i9])
            + (7.022326e-04) * (q[i6] * q[i10]) + (-7.407081e-04) * (q[i6] * q[i11]) + (-3.079661e-04) * (q[i6] * q[i12]) + (5.709624e-04) * (q[i6] * q[i15])
            + (2.251452e-04) * (q[i6] * q[i16]) + (1.268388e-03) * (q[i6] * q[i19]) + (3.018136e-04) * (q[i6] * q[i20]) + (1.285496e-04) * (q[i6] * q[i21])
            + (3.242108e-04) * (q[i6] * q[i22]) + (5.755823e-03) * (q[i7] * q[i8]) + (7.057201e-04) * (q[i7] * q[i9]) + (-2.443772e-03) * (q[i7] * q[i10])
            + (3.318719e-04) * (q[i7] * q[i11]) + (7.383186e-04) * (q[i7] * q[i12]) + (2.397672e-04) * (q[i7] * q[i15]) + (5.739001e-04) * (q[i7] * q[i16])
            + (2.983729e-04) * (q[i7] * q[i19]) + (1.274549e-03) * (q[i7] * q[i20]) + (-3.183364e-04) * (q[i7] * q[i21]) + (-1.257341e-04) * (q[i7] * q[i22])
            + (-2.458956e-03) * (q[i8] * q[i9]) + (-2.408560e-03) * (q[i8] * q[i10]) + (-1.244990e-03) * (q[i8] * q[i11]) + (1.411772e-03) * (q[i8] * q[i12])
            + (4.348435e-03) * (q[i8] * q[i15]) + (4.428257e-03) * (q[i8] * q[i16]) + (1.071222e-03) * (q[i8] * q[i19]) + (1.062139e-03) * (q[i8] * q[i20])
            + (-5.368134e-04) * (q[i8] * q[i21]) + (5.417197e-04) * (q[i8] * q[i22]) + (-1.138271e-04) * (q[i9] * q[i10]) + (-4.428682e-04) * (q[i9] * q[i11])
            + (1.445852e-04) * (q[i9] * q[i12]) + (6.495961e-04) * (q[i9] * q[i15]) + (-4.232450e-04) * (q[i9] * q[i16]) + (1.164111e-04) * (q[i9] * q[i19])
            + (3.097910e-04) * (q[i9] * q[i20]) + (-3.177849e-04) * (q[i9] * q[i21]) + (2.041463e-04) * (q[i9] * q[i22]) + (-1.591874e-04) * (q[i10] * q[i11])
            + (4.314493e-04) * (q[i10] * q[i12]) + (-4.261787e-04) * (q[i10] * q[i15]) + (6.458251e-04) * (q[i10] * q[i16]) + (3.252194e-04) * (q[i10] * q[i19])
            + (1.234765e-04) * (q[i10] * q[i20]) + (-2.055229e-04) * (q[i10] * q[i21]) + (3.180438e-04) * (q[i10] * q[i22]) + (1.565082e-03) * (q[i11] * q[i12])
            + (3.600388e-03) * (q[i11] * q[i15]) + (-6.828608e-04) * (q[i11] * q[i16]) + (1.696311e-03) * (q[i11] * q[i19]) + (5.165242e-05) * (q[i11] * q[i20])
            + (8.125435e-05) * (q[i11] * q[i21]) + (-1.375290e-04) * (q[i11] * q[i22]) + (6.583024e-04) * (q[i12] * q[i15])
            + (-3.614837e-03) * (q[i12] * q[i16]) + (-6.191380e-05) * (q[i12] * q[i19]) + (-1.678354e-03) * (q[i12] * q[i20])
            + (-1.404150e-04) * (q[i12] * q[i21]) + (8.041627e-05) * (q[i12] * q[i22]) + (4.283540e-05) * (q[i15] * q[i16])
            + (-9.625853e-04) * (q[i15] * q[i19]) + (-8.625585e-04) * (q[i15] * q[i20]) + (3.715050e-04) * (q[i15] * q[i21])
            + (3.675351e-04) * (q[i15] * q[i22]) + (-8.608036e-04) * (q[i16] * q[i19]) + (-9.489442e-04) * (q[i16] * q[i20])
            + (-3.632431e-04) * (q[i16] * q[i21]) + (-3.792889e-04) * (q[i16] * q[i22]) + (1.081195e-03) * (q[i19] * q[i20])
            + (-1.188958e-03) * (q[i19] * q[i21]) + (-5.315273e-04) * (q[i19] * q[i22]) + (5.348327e-04) * (q[i20] * q[i21])
            + (1.183849e-03) * (q[i20] * q[i22]) + (2.080626e-04) * (q[i21] * q[i22]);
   }

   public void getJQx3(double[] q, double[][] JQ)
   {
      JQ[1][i3] = (1.312319e-01) * (1) + (1.348478e-02) * ((2) * q[i3]) + (-1.614664e-03) * (q[i0]) + (4.911325e-03) * (q[i1]) + (9.295948e-04) * (q[i2])
            + (-2.876676e-06) * (q[i4]) + (-9.794577e-03) * (q[i5]) + (1.055381e-02) * (q[i6]) + (-4.554090e-03) * (q[i7]) + (2.986873e-03) * (q[i8])
            + (-7.460434e-03) * (q[i9]) + (4.861489e-03) * (q[i10]) + (-8.971683e-04) * (q[i11]) + (2.443215e-03) * (q[i12]) + (5.579206e-03) * (q[i15])
            + (5.510126e-03) * (q[i16]) + (-2.364092e-03) * (q[i19]) + (3.470420e-03) * (q[i20]) + (-9.036793e-04) * (q[i21]) + (5.059900e-04) * (q[i22])
            + (-3.012961e-02) * (q[i0] * q[i0]) + (-6.150220e-05) * (q[i1] * q[i1]) + (-1.410196e-03) * (q[i2] * q[i2])
            + (-2.153409e-03) * ((2) * q[i0] * q[i3]) + (1.442191e-03) * ((2) * q[i1] * q[i3]) + (-2.028160e-03) * ((2) * q[i2] * q[i3])
            + (-3.734343e-03) * ((3) * q[i3] * q[i3]) + (-1.103080e-03) * ((2) * q[i3] * q[i4]) + (7.210215e-03) * ((2) * q[i3] * q[i5])
            + (1.880440e-03) * ((2) * q[i3] * q[i6]) + (2.994120e-04) * ((2) * q[i3] * q[i7]) + (7.210461e-04) * ((2) * q[i3] * q[i8])
            + (-1.857075e-04) * ((2) * q[i3] * q[i9]) + (-2.367540e-04) * ((2) * q[i3] * q[i10]) + (-1.402094e-03) * ((2) * q[i3] * q[i11])
            + (-2.855950e-03) * ((2) * q[i3] * q[i12]) + (7.552389e-04) * ((2) * q[i3] * q[i15]) + (3.199199e-04) * ((2) * q[i3] * q[i16])
            + (-1.116370e-03) * ((2) * q[i3] * q[i19]) + (-2.797243e-04) * ((2) * q[i3] * q[i20]) + (1.536098e-04) * ((2) * q[i3] * q[i21])
            + (3.081561e-04) * ((2) * q[i3] * q[i22]) + (-1.147295e-03) * (q[i4] * q[i4]) + (-4.753110e-04) * (q[i5] * q[i5])
            + (-2.187062e-02) * (q[i6] * q[i6]) + (3.528850e-03) * (q[i7] * q[i7]) + (5.135405e-03) * (q[i8] * q[i8]) + (-3.546787e-03) * (q[i9] * q[i9])
            + (4.730181e-04) * (q[i10] * q[i10]) + (-3.952900e-04) * (q[i11] * q[i11]) + (-9.821072e-04) * (q[i12] * q[i12])
            + (6.068730e-04) * (q[i15] * q[i15]) + (7.318281e-04) * (q[i16] * q[i16]) + (1.567991e-04) * (q[i19] * q[i19]) + (1.332810e-04) * (q[i20] * q[i20])
            + (2.876675e-04) * (q[i21] * q[i21]) + (1.916338e-04) * (q[i22] * q[i22]) + (7.202945e-03) * (q[i0] * q[i1]) + (-1.283325e-02) * (q[i0] * q[i2])
            + (-1.104364e-05) * (q[i0] * q[i4]) + (6.232324e-04) * (q[i0] * q[i5]) + (-1.220277e-02) * (q[i0] * q[i6]) + (4.599825e-04) * (q[i0] * q[i7])
            + (2.879513e-03) * (q[i0] * q[i8]) + (-1.020871e-03) * (q[i0] * q[i9]) + (1.436438e-03) * (q[i0] * q[i10]) + (4.059029e-05) * (q[i0] * q[i11])
            + (-1.685471e-03) * (q[i0] * q[i12]) + (2.231693e-03) * (q[i0] * q[i15]) + (8.670147e-04) * (q[i0] * q[i16]) + (1.522527e-03) * (q[i0] * q[i19])
            + (1.245394e-03) * (q[i0] * q[i20]) + (-8.057844e-04) * (q[i0] * q[i21]) + (-7.652511e-04) * (q[i0] * q[i22]) + (1.897785e-03) * (q[i1] * q[i2])
            + (6.927884e-06) * (q[i1] * q[i4]) + (4.044748e-03) * (q[i1] * q[i5]) + (1.776476e-04) * (q[i1] * q[i6]) + (3.785884e-03) * (q[i1] * q[i7])
            + (1.894027e-03) * (q[i1] * q[i8]) + (-7.905932e-04) * (q[i1] * q[i9]) + (6.715230e-04) * (q[i1] * q[i10]) + (-1.809140e-03) * (q[i1] * q[i11])
            + (1.062638e-03) * (q[i1] * q[i12]) + (3.459426e-04) * (q[i1] * q[i15]) + (5.483259e-04) * (q[i1] * q[i16]) + (9.862217e-04) * (q[i1] * q[i19])
            + (1.000061e-03) * (q[i1] * q[i20]) + (5.962383e-04) * (q[i1] * q[i21]) + (-3.858473e-04) * (q[i1] * q[i22]) + (2.120604e-03) * (q[i2] * q[i4])
            + (5.379433e-03) * (q[i2] * q[i5]) + (-3.754849e-03) * (q[i2] * q[i6]) + (1.078962e-03) * (q[i2] * q[i7]) + (3.084937e-03) * (q[i2] * q[i8])
            + (1.821206e-04) * (q[i2] * q[i9]) + (-4.290648e-04) * (q[i2] * q[i10]) + (5.062225e-04) * (q[i2] * q[i11]) + (-4.700165e-04) * (q[i2] * q[i12])
            + (1.418867e-03) * (q[i2] * q[i15]) + (1.015846e-04) * (q[i2] * q[i16]) + (-9.577654e-04) * (q[i2] * q[i19]) + (5.090413e-04) * (q[i2] * q[i20])
            + (-7.309653e-04) * (q[i2] * q[i21]) + (1.000350e-03) * (q[i2] * q[i22]) + (-1.625450e-03) * (q[i4] * q[i5]) + (2.142900e-03) * (q[i4] * q[i6])
            + (-2.125483e-03) * (q[i4] * q[i7]) + (-2.209751e-05) * (q[i4] * q[i8]) + (-4.865914e-05) * (q[i4] * q[i9]) + (5.663865e-05) * (q[i4] * q[i10])
            + (1.984605e-03) * (q[i4] * q[i11]) + (1.988030e-03) * (q[i4] * q[i12]) + (-6.454528e-05) * (q[i4] * q[i15]) + (7.420731e-05) * (q[i4] * q[i16])
            + (-7.927073e-04) * (q[i4] * q[i19]) + (7.792435e-04) * (q[i4] * q[i20]) + (-7.415448e-04) * (q[i4] * q[i21]) + (-7.459352e-04) * (q[i4] * q[i22])
            + (-6.349387e-03) * (q[i5] * q[i6]) + (5.954575e-04) * (q[i5] * q[i7]) + (-4.838856e-04) * (q[i5] * q[i8]) + (-1.903444e-04) * (q[i5] * q[i9])
            + (-1.450961e-03) * (q[i5] * q[i10]) + (-8.940210e-04) * (q[i5] * q[i11]) + (-3.053467e-04) * (q[i5] * q[i12]) + (3.455274e-04) * (q[i5] * q[i15])
            + (9.330827e-04) * (q[i5] * q[i16]) + (-8.253939e-04) * (q[i5] * q[i19]) + (-1.240502e-03) * (q[i5] * q[i20]) + (3.996984e-05) * (q[i5] * q[i21])
            + (1.160825e-03) * (q[i5] * q[i22]) + (3.245163e-04) * (q[i6] * q[i7]) + (-3.734237e-03) * (q[i6] * q[i8]) + (-1.030080e-02) * (q[i6] * q[i9])
            + (5.297648e-04) * (q[i6] * q[i10]) + (1.117085e-04) * (q[i6] * q[i11]) + (1.625795e-04) * (q[i6] * q[i12]) + (1.331850e-03) * (q[i6] * q[i15])
            + (-1.027732e-03) * (q[i6] * q[i16]) + (-2.949664e-03) * (q[i6] * q[i19]) + (-1.961914e-04) * (q[i6] * q[i20]) + (-8.116925e-04) * (q[i6] * q[i21])
            + (3.394242e-04) * (q[i6] * q[i22]) + (-3.796529e-04) * (q[i7] * q[i8]) + (1.223988e-04) * (q[i7] * q[i9]) + (1.490958e-03) * (q[i7] * q[i10])
            + (-9.071750e-04) * (q[i7] * q[i11]) + (7.903289e-04) * (q[i7] * q[i12]) + (-7.167150e-04) * (q[i7] * q[i15]) + (-1.838161e-03) * (q[i7] * q[i16])
            + (-1.571947e-04) * (q[i7] * q[i19]) + (1.872840e-03) * (q[i7] * q[i20]) + (1.340993e-03) * (q[i7] * q[i21]) + (7.346822e-04) * (q[i7] * q[i22])
            + (-7.091580e-04) * (q[i8] * q[i9]) + (9.674446e-04) * (q[i8] * q[i10]) + (-8.620245e-04) * (q[i8] * q[i11]) + (1.817324e-03) * (q[i8] * q[i12])
            + (4.639511e-04) * (q[i8] * q[i15]) + (-2.946867e-04) * (q[i8] * q[i16]) + (-1.082728e-03) * (q[i8] * q[i19]) + (4.997872e-05) * (q[i8] * q[i20])
            + (1.835052e-04) * (q[i8] * q[i21]) + (-1.137578e-04) * (q[i8] * q[i22]) + (-1.125415e-03) * (q[i9] * q[i10]) + (-5.818825e-04) * (q[i9] * q[i11])
            + (2.452206e-04) * (q[i9] * q[i12]) + (-2.710467e-04) * (q[i9] * q[i15]) + (-4.947617e-04) * (q[i9] * q[i16]) + (2.575008e-04) * (q[i9] * q[i19])
            + (-1.099831e-03) * (q[i9] * q[i20]) + (2.429874e-04) * (q[i9] * q[i21]) + (-1.711662e-04) * (q[i9] * q[i22]) + (-7.761758e-05) * (q[i10] * q[i11])
            + (2.547123e-04) * (q[i10] * q[i12]) + (-6.593369e-04) * (q[i10] * q[i15]) + (-5.319852e-05) * (q[i10] * q[i16])
            + (9.528878e-04) * (q[i10] * q[i19]) + (-1.118422e-04) * (q[i10] * q[i20]) + (-2.816300e-04) * (q[i10] * q[i21])
            + (3.857699e-04) * (q[i10] * q[i22]) + (-9.431192e-04) * (q[i11] * q[i12]) + (-2.399187e-03) * (q[i11] * q[i15])
            + (9.598698e-04) * (q[i11] * q[i16]) + (3.549022e-04) * (q[i11] * q[i19]) + (-6.758842e-05) * (q[i11] * q[i20])
            + (-8.200449e-04) * (q[i11] * q[i21]) + (-1.065079e-03) * (q[i11] * q[i22]) + (3.537319e-04) * (q[i12] * q[i15])
            + (-4.212583e-04) * (q[i12] * q[i16]) + (-5.466593e-04) * (q[i12] * q[i19]) + (8.499335e-04) * (q[i12] * q[i20])
            + (8.870416e-04) * (q[i12] * q[i21]) + (-2.562754e-04) * (q[i12] * q[i22]) + (6.317741e-04) * (q[i15] * q[i16])
            + (-4.963041e-04) * (q[i15] * q[i19]) + (-5.463849e-04) * (q[i15] * q[i20]) + (-1.765358e-04) * (q[i15] * q[i21])
            + (2.002703e-04) * (q[i15] * q[i22]) + (7.135509e-04) * (q[i16] * q[i19]) + (4.429594e-04) * (q[i16] * q[i20]) + (-2.275339e-05) * (q[i16] * q[i21])
            + (-5.864281e-04) * (q[i16] * q[i22]) + (6.337695e-04) * (q[i19] * q[i20]) + (1.284728e-03) * (q[i19] * q[i21])
            + (-3.871210e-04) * (q[i19] * q[i22]) + (-3.009105e-04) * (q[i20] * q[i21]) + (-1.722683e-04) * (q[i20] * q[i22])
            + (2.200795e-04) * (q[i21] * q[i22]);
   }

   public void getJQx4(double[] q, double[][] JQ)
   {
      JQ[1][i4] = (1.305468e-01) * (1) + (-1.337858e-02) * ((2) * q[i4]) + (-4.964491e-03) * (q[i0]) + (1.639490e-03) * (q[i1]) + (-1.007355e-03) * (q[i2])
            + (-2.876676e-06) * (q[i3]) + (9.743681e-03) * (q[i5]) + (-4.531510e-03) * (q[i6]) + (1.051373e-02) * (q[i7]) + (3.061437e-03) * (q[i8])
            + (4.908594e-03) * (q[i9]) + (-7.410230e-03) * (q[i10]) + (-2.383645e-03) * (q[i11]) + (9.079738e-04) * (q[i12]) + (5.425509e-03) * (q[i15])
            + (5.592667e-03) * (q[i16]) + (3.453573e-03) * (q[i19]) + (-2.334264e-03) * (q[i20]) + (-5.053012e-04) * (q[i21]) + (8.770941e-04) * (q[i22])
            + (-6.835978e-05) * (q[i0] * q[i0]) + (-3.004084e-02) * (q[i1] * q[i1]) + (-1.390624e-03) * (q[i2] * q[i2]) + (-1.103080e-03) * (q[i3] * q[i3])
            + (1.433173e-03) * ((2) * q[i0] * q[i4]) + (-2.141928e-03) * ((2) * q[i1] * q[i4]) + (-2.034666e-03) * ((2) * q[i2] * q[i4])
            + (-1.147295e-03) * ((2) * q[i3] * q[i4]) + (-3.714140e-03) * ((3) * q[i4] * q[i4]) + (7.177278e-03) * ((2) * q[i4] * q[i5])
            + (-2.780751e-04) * ((2) * q[i4] * q[i6]) + (-1.868758e-03) * ((2) * q[i4] * q[i7]) + (-6.687702e-04) * ((2) * q[i4] * q[i8])
            + (2.262692e-04) * ((2) * q[i4] * q[i9]) + (1.723745e-04) * ((2) * q[i4] * q[i10]) + (-2.837358e-03) * ((2) * q[i4] * q[i11])
            + (-1.394569e-03) * ((2) * q[i4] * q[i12]) + (-3.144852e-04) * ((2) * q[i4] * q[i15]) + (-7.444036e-04) * ((2) * q[i4] * q[i16])
            + (2.920938e-04) * ((2) * q[i4] * q[i19]) + (1.091014e-03) * ((2) * q[i4] * q[i20]) + (3.029220e-04) * ((2) * q[i4] * q[i21])
            + (1.453614e-04) * ((2) * q[i4] * q[i22]) + (-4.325963e-04) * (q[i5] * q[i5]) + (3.542552e-03) * (q[i6] * q[i6]) + (-2.179241e-02) * (q[i7] * q[i7])
            + (5.120441e-03) * (q[i8] * q[i8]) + (4.736603e-04) * (q[i9] * q[i9]) + (-3.499427e-03) * (q[i10] * q[i10]) + (-1.005719e-03) * (q[i11] * q[i11])
            + (-3.937433e-04) * (q[i12] * q[i12]) + (7.297165e-04) * (q[i15] * q[i15]) + (6.037906e-04) * (q[i16] * q[i16]) + (1.412430e-04) * (q[i19] * q[i19])
            + (1.504221e-04) * (q[i20] * q[i20]) + (1.871116e-04) * (q[i21] * q[i21]) + (2.904623e-04) * (q[i22] * q[i22]) + (7.154852e-03) * (q[i0] * q[i1])
            + (1.855208e-03) * (q[i0] * q[i2]) + (-1.104364e-05) * (q[i0] * q[i3]) + (4.044206e-03) * (q[i0] * q[i5]) + (-3.852007e-03) * (q[i0] * q[i6])
            + (-2.147826e-04) * (q[i0] * q[i7]) + (-1.860052e-03) * (q[i0] * q[i8]) + (-6.762930e-04) * (q[i0] * q[i9]) + (7.799416e-04) * (q[i0] * q[i10])
            + (1.098684e-03) * (q[i0] * q[i11]) + (-1.804400e-03) * (q[i0] * q[i12]) + (-5.569488e-04) * (q[i0] * q[i15]) + (-3.701069e-04) * (q[i0] * q[i16])
            + (-1.016211e-03) * (q[i0] * q[i19]) + (-9.938613e-04) * (q[i0] * q[i20]) + (-3.788516e-04) * (q[i0] * q[i21]) + (6.018258e-04) * (q[i0] * q[i22])
            + (-1.277614e-02) * (q[i1] * q[i2]) + (6.927884e-06) * (q[i1] * q[i3]) + (6.049404e-04) * (q[i1] * q[i5]) + (-4.309909e-04) * (q[i1] * q[i6])
            + (1.215547e-02) * (q[i1] * q[i7]) + (-2.884023e-03) * (q[i1] * q[i8]) + (-1.469909e-03) * (q[i1] * q[i9]) + (1.007366e-03) * (q[i1] * q[i10])
            + (-1.716275e-03) * (q[i1] * q[i11]) + (6.743792e-05) * (q[i1] * q[i12]) + (-8.506545e-04) * (q[i1] * q[i15]) + (-2.229472e-03) * (q[i1] * q[i16])
            + (-1.241991e-03) * (q[i1] * q[i19]) + (-1.501474e-03) * (q[i1] * q[i20]) + (-7.710065e-04) * (q[i1] * q[i21]) + (-8.268980e-04) * (q[i1] * q[i22])
            + (2.120604e-03) * (q[i2] * q[i3]) + (5.443058e-03) * (q[i2] * q[i5]) + (-1.066613e-03) * (q[i2] * q[i6]) + (3.717713e-03) * (q[i2] * q[i7])
            + (-3.042462e-03) * (q[i2] * q[i8]) + (4.203955e-04) * (q[i2] * q[i9]) + (-1.730749e-04) * (q[i2] * q[i10]) + (-4.936573e-04) * (q[i2] * q[i11])
            + (5.365713e-04) * (q[i2] * q[i12]) + (-9.815592e-05) * (q[i2] * q[i15]) + (-1.455408e-03) * (q[i2] * q[i16]) + (-5.021187e-04) * (q[i2] * q[i19])
            + (9.312876e-04) * (q[i2] * q[i20]) + (9.853955e-04) * (q[i2] * q[i21]) + (-7.527671e-04) * (q[i2] * q[i22]) + (-1.625450e-03) * (q[i3] * q[i5])
            + (2.142900e-03) * (q[i3] * q[i6]) + (-2.125483e-03) * (q[i3] * q[i7]) + (-2.209751e-05) * (q[i3] * q[i8]) + (-4.865914e-05) * (q[i3] * q[i9])
            + (5.663865e-05) * (q[i3] * q[i10]) + (1.984605e-03) * (q[i3] * q[i11]) + (1.988030e-03) * (q[i3] * q[i12]) + (-6.454528e-05) * (q[i3] * q[i15])
            + (7.420731e-05) * (q[i3] * q[i16]) + (-7.927073e-04) * (q[i3] * q[i19]) + (7.792435e-04) * (q[i3] * q[i20]) + (-7.415448e-04) * (q[i3] * q[i21])
            + (-7.459352e-04) * (q[i3] * q[i22]) + (-5.931727e-04) * (q[i5] * q[i6]) + (6.315469e-03) * (q[i5] * q[i7]) + (4.671618e-04) * (q[i5] * q[i8])
            + (1.461121e-03) * (q[i5] * q[i9]) + (1.876810e-04) * (q[i5] * q[i10]) + (-3.199444e-04) * (q[i5] * q[i11]) + (-8.964191e-04) * (q[i5] * q[i12])
            + (-9.330154e-04) * (q[i5] * q[i15]) + (-3.794327e-04) * (q[i5] * q[i16]) + (1.241729e-03) * (q[i5] * q[i19]) + (8.317277e-04) * (q[i5] * q[i20])
            + (1.183960e-03) * (q[i5] * q[i21]) + (7.428462e-05) * (q[i5] * q[i22]) + (3.288686e-04) * (q[i6] * q[i7]) + (-3.885931e-04) * (q[i6] * q[i8])
            + (1.505838e-03) * (q[i6] * q[i9]) + (1.077158e-04) * (q[i6] * q[i10]) + (-7.801497e-04) * (q[i6] * q[i11]) + (8.819036e-04) * (q[i6] * q[i12])
            + (-1.836521e-03) * (q[i6] * q[i15]) + (-7.226764e-04) * (q[i6] * q[i16]) + (1.863697e-03) * (q[i6] * q[i19]) + (-1.470723e-04) * (q[i6] * q[i20])
            + (-7.265601e-04) * (q[i6] * q[i21]) + (-1.339614e-03) * (q[i6] * q[i22]) + (-3.720154e-03) * (q[i7] * q[i8]) + (5.630538e-04) * (q[i7] * q[i9])
            + (-1.020911e-02) * (q[i7] * q[i10]) + (-1.659809e-04) * (q[i7] * q[i11]) + (-1.034888e-04) * (q[i7] * q[i12]) + (-1.031517e-03) * (q[i7] * q[i15])
            + (1.346195e-03) * (q[i7] * q[i16]) + (-1.888206e-04) * (q[i7] * q[i19]) + (-2.927266e-03) * (q[i7] * q[i20]) + (-3.443914e-04) * (q[i7] * q[i21])
            + (8.115579e-04) * (q[i7] * q[i22]) + (9.636456e-04) * (q[i8] * q[i9]) + (-7.088078e-04) * (q[i8] * q[i10]) + (-1.784835e-03) * (q[i8] * q[i11])
            + (8.717079e-04) * (q[i8] * q[i12]) + (-2.745696e-04) * (q[i8] * q[i15]) + (4.798022e-04) * (q[i8] * q[i16]) + (5.460890e-05) * (q[i8] * q[i19])
            + (-1.055922e-03) * (q[i8] * q[i20]) + (1.136042e-04) * (q[i8] * q[i21]) + (-1.860377e-04) * (q[i8] * q[i22]) + (-1.116048e-03) * (q[i9] * q[i10])
            + (-2.538559e-04) * (q[i9] * q[i11]) + (7.054069e-05) * (q[i9] * q[i12]) + (-6.173504e-05) * (q[i9] * q[i15]) + (-6.497450e-04) * (q[i9] * q[i16])
            + (-1.113940e-04) * (q[i9] * q[i19]) + (9.484636e-04) * (q[i9] * q[i20]) + (-3.795851e-04) * (q[i9] * q[i21]) + (2.792880e-04) * (q[i9] * q[i22])
            + (-2.539604e-04) * (q[i10] * q[i11]) + (5.829149e-04) * (q[i10] * q[i12]) + (-4.795333e-04) * (q[i10] * q[i15])
            + (-2.686769e-04) * (q[i10] * q[i16]) + (-1.089708e-03) * (q[i10] * q[i19]) + (2.507049e-04) * (q[i10] * q[i20])
            + (1.679941e-04) * (q[i10] * q[i21]) + (-2.440550e-04) * (q[i10] * q[i22]) + (-9.341547e-04) * (q[i11] * q[i12])
            + (4.198854e-04) * (q[i11] * q[i15]) + (-3.551721e-04) * (q[i11] * q[i16]) + (-8.331422e-04) * (q[i11] * q[i19])
            + (5.438664e-04) * (q[i11] * q[i20]) + (-2.493820e-04) * (q[i11] * q[i21]) + (8.925204e-04) * (q[i11] * q[i22])
            + (-9.618192e-04) * (q[i12] * q[i15]) + (2.398704e-03) * (q[i12] * q[i16]) + (5.877753e-05) * (q[i12] * q[i19])
            + (-3.604102e-04) * (q[i12] * q[i20]) + (-1.056132e-03) * (q[i12] * q[i21]) + (-8.202490e-04) * (q[i12] * q[i22])
            + (6.267090e-04) * (q[i15] * q[i16]) + (4.461442e-04) * (q[i15] * q[i19]) + (7.146477e-04) * (q[i15] * q[i20]) + (5.753389e-04) * (q[i15] * q[i21])
            + (1.739075e-05) * (q[i15] * q[i22]) + (-5.510075e-04) * (q[i16] * q[i19]) + (-4.883223e-04) * (q[i16] * q[i20])
            + (-1.986130e-04) * (q[i16] * q[i21]) + (1.665167e-04) * (q[i16] * q[i22]) + (6.201209e-04) * (q[i19] * q[i20]) + (1.730511e-04) * (q[i19] * q[i21])
            + (3.040057e-04) * (q[i19] * q[i22]) + (3.878512e-04) * (q[i20] * q[i21]) + (-1.278620e-03) * (q[i20] * q[i22])
            + (2.073554e-04) * (q[i21] * q[i22]);
   }

   public void getJQx5(double[] q, double[][] JQ)
   {
      JQ[1][i5] = (1.272279e-01) * (1) + (-1.522872e-05) * ((2) * q[i5]) + (2.972583e-03) * (q[i0]) + (-2.945678e-03) * (q[i1]) + (9.527315e-05) * (q[i2])
            + (-9.794577e-03) * (q[i3]) + (9.743681e-03) * (q[i4]) + (-1.444991e-03) * (q[i6]) + (-1.434103e-03) * (q[i7]) + (4.079506e-04) * (q[i8])
            + (2.276604e-03) * (q[i9]) + (2.289996e-03) * (q[i10]) + (5.436482e-03) * (q[i11]) + (-5.465597e-03) * (q[i12]) + (-1.788624e-02) * (q[i15])
            + (-1.806450e-02) * (q[i16]) + (-5.391021e-04) * (q[i19]) + (-5.439486e-04) * (q[i20]) + (-8.173700e-04) * (q[i21]) + (8.332752e-04) * (q[i22])
            + (-3.507112e-03) * (q[i0] * q[i0]) + (-3.462911e-03) * (q[i1] * q[i1]) + (-2.711721e-02) * (q[i2] * q[i2]) + (7.210215e-03) * (q[i3] * q[i3])
            + (7.177278e-03) * (q[i4] * q[i4]) + (-2.019421e-03) * ((2) * q[i0] * q[i5]) + (-1.986907e-03) * ((2) * q[i1] * q[i5])
            + (-4.393894e-04) * ((2) * q[i2] * q[i5]) + (-4.753110e-04) * ((2) * q[i3] * q[i5]) + (-4.325963e-04) * ((2) * q[i4] * q[i5])
            + (-4.778403e-03) * ((3) * q[i5] * q[i5]) + (-1.737774e-04) * ((2) * q[i5] * q[i6]) + (1.739067e-04) * ((2) * q[i5] * q[i7])
            + (8.966249e-06) * ((2) * q[i5] * q[i8]) + (1.513727e-04) * ((2) * q[i5] * q[i9]) + (-1.422913e-04) * ((2) * q[i5] * q[i10])
            + (2.879716e-04) * ((2) * q[i5] * q[i11]) + (2.853626e-04) * ((2) * q[i5] * q[i12]) + (1.071036e-04) * ((2) * q[i5] * q[i15])
            + (-9.213145e-05) * ((2) * q[i5] * q[i16]) + (4.846585e-04) * ((2) * q[i5] * q[i19]) + (-4.678912e-04) * ((2) * q[i5] * q[i20])
            + (-4.744346e-04) * ((2) * q[i5] * q[i21]) + (-4.877765e-04) * ((2) * q[i5] * q[i22]) + (4.401184e-03) * (q[i6] * q[i6])
            + (4.394210e-03) * (q[i7] * q[i7]) + (-2.642023e-02) * (q[i8] * q[i8]) + (1.952825e-03) * (q[i9] * q[i9]) + (1.924849e-03) * (q[i10] * q[i10])
            + (-1.257756e-03) * (q[i11] * q[i11]) + (-1.331503e-03) * (q[i12] * q[i12]) + (-3.172480e-03) * (q[i15] * q[i15])
            + (-3.193741e-03) * (q[i16] * q[i16]) + (3.073737e-04) * (q[i19] * q[i19]) + (3.262443e-04) * (q[i20] * q[i20])
            + (-1.374830e-03) * (q[i21] * q[i21]) + (-1.379575e-03) * (q[i22] * q[i22]) + (-5.894587e-03) * (q[i0] * q[i1]) + (-1.194723e-02) * (q[i0] * q[i2])
            + (6.232324e-04) * (q[i0] * q[i3]) + (4.044206e-03) * (q[i0] * q[i4]) + (-8.060198e-03) * (q[i0] * q[i6]) + (-4.824738e-04) * (q[i0] * q[i7])
            + (-3.618220e-04) * (q[i0] * q[i8]) + (4.046716e-04) * (q[i0] * q[i9]) + (-8.990503e-04) * (q[i0] * q[i10]) + (-6.948002e-05) * (q[i0] * q[i11])
            + (2.869565e-04) * (q[i0] * q[i12]) + (7.623461e-04) * (q[i0] * q[i15]) + (-1.635196e-03) * (q[i0] * q[i16]) + (-8.694627e-04) * (q[i0] * q[i19])
            + (1.593632e-04) * (q[i0] * q[i20]) + (6.680767e-04) * (q[i0] * q[i21]) + (-2.400176e-04) * (q[i0] * q[i22]) + (-1.187656e-02) * (q[i1] * q[i2])
            + (4.044748e-03) * (q[i1] * q[i3]) + (6.049404e-04) * (q[i1] * q[i4]) + (4.644945e-04) * (q[i1] * q[i6]) + (8.116384e-03) * (q[i1] * q[i7])
            + (3.976265e-04) * (q[i1] * q[i8]) + (8.545363e-04) * (q[i1] * q[i9]) + (-3.880431e-04) * (q[i1] * q[i10]) + (3.387693e-04) * (q[i1] * q[i11])
            + (-6.398775e-05) * (q[i1] * q[i12]) + (1.632663e-03) * (q[i1] * q[i15]) + (-7.627753e-04) * (q[i1] * q[i16]) + (-1.608649e-04) * (q[i1] * q[i19])
            + (8.323787e-04) * (q[i1] * q[i20]) + (-2.455253e-04) * (q[i1] * q[i21]) + (6.741223e-04) * (q[i1] * q[i22]) + (5.379433e-03) * (q[i2] * q[i3])
            + (5.443058e-03) * (q[i2] * q[i4]) + (-1.897680e-04) * (q[i2] * q[i6]) + (1.789531e-04) * (q[i2] * q[i7]) + (-2.629788e-05) * (q[i2] * q[i8])
            + (2.116555e-03) * (q[i2] * q[i9]) + (-2.136809e-03) * (q[i2] * q[i10]) + (1.737642e-03) * (q[i2] * q[i11]) + (1.689365e-03) * (q[i2] * q[i12])
            + (-1.444219e-03) * (q[i2] * q[i15]) + (1.441259e-03) * (q[i2] * q[i16]) + (1.186527e-03) * (q[i2] * q[i19]) + (-1.194357e-03) * (q[i2] * q[i20])
            + (5.688177e-04) * (q[i2] * q[i21]) + (5.556571e-04) * (q[i2] * q[i22]) + (-1.625450e-03) * (q[i3] * q[i4]) + (-6.349387e-03) * (q[i3] * q[i6])
            + (5.954575e-04) * (q[i3] * q[i7]) + (-4.838856e-04) * (q[i3] * q[i8]) + (-1.903444e-04) * (q[i3] * q[i9]) + (-1.450961e-03) * (q[i3] * q[i10])
            + (-8.940210e-04) * (q[i3] * q[i11]) + (-3.053467e-04) * (q[i3] * q[i12]) + (3.455274e-04) * (q[i3] * q[i15]) + (9.330827e-04) * (q[i3] * q[i16])
            + (-8.253939e-04) * (q[i3] * q[i19]) + (-1.240502e-03) * (q[i3] * q[i20]) + (3.996984e-05) * (q[i3] * q[i21]) + (1.160825e-03) * (q[i3] * q[i22])
            + (-5.931727e-04) * (q[i4] * q[i6]) + (6.315469e-03) * (q[i4] * q[i7]) + (4.671618e-04) * (q[i4] * q[i8]) + (1.461121e-03) * (q[i4] * q[i9])
            + (1.876810e-04) * (q[i4] * q[i10]) + (-3.199444e-04) * (q[i4] * q[i11]) + (-8.964191e-04) * (q[i4] * q[i12]) + (-9.330154e-04) * (q[i4] * q[i15])
            + (-3.794327e-04) * (q[i4] * q[i16]) + (1.241729e-03) * (q[i4] * q[i19]) + (8.317277e-04) * (q[i4] * q[i20]) + (1.183960e-03) * (q[i4] * q[i21])
            + (7.428462e-05) * (q[i4] * q[i22]) + (2.967315e-03) * (q[i6] * q[i7]) + (1.843466e-03) * (q[i6] * q[i8]) + (3.600806e-03) * (q[i6] * q[i9])
            + (-1.788734e-03) * (q[i6] * q[i10]) + (1.185197e-04) * (q[i6] * q[i11]) + (-1.385588e-03) * (q[i6] * q[i12]) + (-2.008343e-04) * (q[i6] * q[i15])
            + (3.957857e-04) * (q[i6] * q[i16]) + (2.459765e-04) * (q[i6] * q[i19]) + (6.275868e-04) * (q[i6] * q[i20]) + (4.400469e-04) * (q[i6] * q[i21])
            + (5.221971e-04) * (q[i6] * q[i22]) + (1.807979e-03) * (q[i7] * q[i8]) + (-1.811998e-03) * (q[i7] * q[i9]) + (3.594889e-03) * (q[i7] * q[i10])
            + (1.385137e-03) * (q[i7] * q[i11]) + (-1.108449e-04) * (q[i7] * q[i12]) + (3.823836e-04) * (q[i7] * q[i15]) + (-2.080655e-04) * (q[i7] * q[i16])
            + (6.288068e-04) * (q[i7] * q[i19]) + (2.373423e-04) * (q[i7] * q[i20]) + (-5.079405e-04) * (q[i7] * q[i21]) + (-4.616325e-04) * (q[i7] * q[i22])
            + (-1.520993e-03) * (q[i8] * q[i9]) + (-1.520071e-03) * (q[i8] * q[i10]) + (1.744742e-03) * (q[i8] * q[i11]) + (-1.778833e-03) * (q[i8] * q[i12])
            + (1.608008e-03) * (q[i8] * q[i15]) + (1.625924e-03) * (q[i8] * q[i16]) + (-1.509317e-03) * (q[i8] * q[i19]) + (-1.517377e-03) * (q[i8] * q[i20])
            + (6.891492e-04) * (q[i8] * q[i21]) + (-7.023000e-04) * (q[i8] * q[i22]) + (3.736259e-04) * (q[i9] * q[i10]) + (-3.088273e-04) * (q[i9] * q[i11])
            + (-1.763527e-04) * (q[i9] * q[i12]) + (-1.271194e-05) * (q[i9] * q[i15]) + (-2.967064e-06) * (q[i9] * q[i16]) + (-1.319905e-04) * (q[i9] * q[i19])
            + (6.840200e-05) * (q[i9] * q[i20]) + (5.699121e-05) * (q[i9] * q[i21]) + (-1.983944e-04) * (q[i9] * q[i22]) + (1.690130e-04) * (q[i10] * q[i11])
            + (3.018424e-04) * (q[i10] * q[i12]) + (1.603742e-06) * (q[i10] * q[i15]) + (-2.413245e-05) * (q[i10] * q[i16]) + (6.552438e-05) * (q[i10] * q[i19])
            + (-1.415325e-04) * (q[i10] * q[i20]) + (1.988570e-04) * (q[i10] * q[i21]) + (-7.148407e-05) * (q[i10] * q[i22])
            + (1.094104e-03) * (q[i11] * q[i12]) + (2.062101e-03) * (q[i11] * q[i15]) + (-6.159860e-04) * (q[i11] * q[i16]) + (6.998644e-04) * (q[i11] * q[i19])
            + (-4.212765e-04) * (q[i11] * q[i20]) + (-2.158314e-04) * (q[i11] * q[i21]) + (-3.102334e-04) * (q[i11] * q[i22])
            + (6.111405e-04) * (q[i12] * q[i15]) + (-2.064967e-03) * (q[i12] * q[i16]) + (4.324835e-04) * (q[i12] * q[i19])
            + (-7.160581e-04) * (q[i12] * q[i20]) + (-3.110414e-04) * (q[i12] * q[i21]) + (-2.057889e-04) * (q[i12] * q[i22])
            + (-1.431438e-04) * (q[i15] * q[i16]) + (-7.355210e-04) * (q[i15] * q[i19]) + (-3.359705e-04) * (q[i15] * q[i20])
            + (-1.399082e-03) * (q[i15] * q[i21]) + (1.192572e-04) * (q[i15] * q[i22]) + (-3.321778e-04) * (q[i16] * q[i19])
            + (-7.404865e-04) * (q[i16] * q[i20]) + (-1.216028e-04) * (q[i16] * q[i21]) + (1.415512e-03) * (q[i16] * q[i22])
            + (-5.472268e-04) * (q[i19] * q[i20]) + (-1.777132e-03) * (q[i19] * q[i21]) + (5.059876e-04) * (q[i19] * q[i22])
            + (-5.014894e-04) * (q[i20] * q[i21]) + (1.779544e-03) * (q[i20] * q[i22]) + (-5.758126e-04) * (q[i21] * q[i22]);
   }

   public void getJQx6(double[] q, double[][] JQ)
   {
      JQ[1][i6] = (3.137786e-03) * (1) + (1.824246e-02) * ((2) * q[i6]) + (9.063135e-02) * (q[i0]) + (-1.054391e-02) * (q[i1]) + (1.591417e-02) * (q[i2])
            + (1.055381e-02) * (q[i3]) + (-4.531510e-03) * (q[i4]) + (-1.444991e-03) * (q[i5]) + (-5.334098e-05) * (q[i7]) + (2.546911e-03) * (q[i8])
            + (8.509535e-03) * (q[i9]) + (-4.073500e-04) * (q[i10]) + (3.803170e-03) * (q[i11]) + (4.725349e-03) * (q[i12]) + (-9.283013e-04) * (q[i15])
            + (-7.215826e-04) * (q[i16]) + (2.066508e-03) * (q[i19]) + (1.200329e-03) * (q[i20]) + (-7.083634e-04) * (q[i21]) + (-1.230140e-03) * (q[i22])
            + (1.253647e-03) * (q[i0] * q[i0]) + (-2.010742e-03) * (q[i1] * q[i1]) + (9.419021e-04) * (q[i2] * q[i2]) + (1.880440e-03) * (q[i3] * q[i3])
            + (-2.780751e-04) * (q[i4] * q[i4]) + (-1.737774e-04) * (q[i5] * q[i5]) + (3.867929e-03) * ((2) * q[i0] * q[i6])
            + (-3.455323e-03) * ((2) * q[i1] * q[i6]) + (-4.848208e-04) * ((2) * q[i2] * q[i6]) + (-2.187062e-02) * ((2) * q[i3] * q[i6])
            + (3.542552e-03) * ((2) * q[i4] * q[i6]) + (4.401184e-03) * ((2) * q[i5] * q[i6]) + (-6.368184e-04) * ((3) * q[i6] * q[i6])
            + (-1.625084e-03) * ((2) * q[i6] * q[i7]) + (-3.501911e-04) * ((2) * q[i6] * q[i8]) + (-2.486045e-03) * ((2) * q[i6] * q[i9])
            + (1.582213e-04) * ((2) * q[i6] * q[i10]) + (5.171001e-04) * ((2) * q[i6] * q[i11]) + (-6.927001e-04) * ((2) * q[i6] * q[i12])
            + (2.228535e-04) * ((2) * q[i6] * q[i15]) + (1.106866e-03) * ((2) * q[i6] * q[i16]) + (-1.366856e-04) * ((2) * q[i6] * q[i19])
            + (4.183499e-04) * ((2) * q[i6] * q[i20]) + (2.497640e-04) * ((2) * q[i6] * q[i21]) + (-6.035456e-05) * ((2) * q[i6] * q[i22])
            + (1.623807e-03) * (q[i7] * q[i7]) + (-1.543149e-04) * (q[i8] * q[i8]) + (-2.439674e-03) * (q[i9] * q[i9]) + (-9.914628e-04) * (q[i10] * q[i10])
            + (-5.734439e-05) * (q[i11] * q[i11]) + (4.222588e-05) * (q[i12] * q[i12]) + (8.926814e-05) * (q[i15] * q[i15]) + (5.766638e-05) * (q[i16] * q[i16])
            + (-1.450210e-04) * (q[i19] * q[i19]) + (-3.249408e-04) * (q[i20] * q[i20]) + (-1.487235e-04) * (q[i21] * q[i21])
            + (3.754935e-04) * (q[i22] * q[i22]) + (1.950107e-03) * (q[i0] * q[i1]) + (1.345896e-03) * (q[i0] * q[i2]) + (-1.220277e-02) * (q[i0] * q[i3])
            + (-3.852007e-03) * (q[i0] * q[i4]) + (-8.060198e-03) * (q[i0] * q[i5]) + (-4.657516e-04) * (q[i0] * q[i7]) + (7.409014e-03) * (q[i0] * q[i8])
            + (-1.360672e-02) * (q[i0] * q[i9]) + (1.714814e-03) * (q[i0] * q[i10]) + (-1.493674e-03) * (q[i0] * q[i11]) + (3.205563e-04) * (q[i0] * q[i12])
            + (3.511050e-04) * (q[i0] * q[i15]) + (2.069774e-03) * (q[i0] * q[i16]) + (1.222475e-03) * (q[i0] * q[i19]) + (1.359705e-03) * (q[i0] * q[i20])
            + (-8.987898e-04) * (q[i0] * q[i21]) + (6.300372e-04) * (q[i0] * q[i22]) + (2.015449e-03) * (q[i1] * q[i2]) + (1.776476e-04) * (q[i1] * q[i3])
            + (-4.309909e-04) * (q[i1] * q[i4]) + (4.644945e-04) * (q[i1] * q[i5]) + (-5.072719e-04) * (q[i1] * q[i7]) + (-1.312707e-03) * (q[i1] * q[i8])
            + (3.417191e-03) * (q[i1] * q[i9]) + (-1.111062e-04) * (q[i1] * q[i10]) + (1.669093e-03) * (q[i1] * q[i11]) + (9.281863e-04) * (q[i1] * q[i12])
            + (7.933511e-04) * (q[i1] * q[i15]) + (-6.163040e-04) * (q[i1] * q[i16]) + (-9.142841e-04) * (q[i1] * q[i19]) + (-6.114105e-04) * (q[i1] * q[i20])
            + (8.439397e-04) * (q[i1] * q[i21]) + (1.852810e-04) * (q[i1] * q[i22]) + (-3.754849e-03) * (q[i2] * q[i3]) + (-1.066613e-03) * (q[i2] * q[i4])
            + (-1.897680e-04) * (q[i2] * q[i5]) + (3.344318e-03) * (q[i2] * q[i7]) + (5.734035e-03) * (q[i2] * q[i8]) + (-2.505272e-03) * (q[i2] * q[i9])
            + (7.022326e-04) * (q[i2] * q[i10]) + (-7.407081e-04) * (q[i2] * q[i11]) + (-3.079661e-04) * (q[i2] * q[i12]) + (5.709624e-04) * (q[i2] * q[i15])
            + (2.251452e-04) * (q[i2] * q[i16]) + (1.268388e-03) * (q[i2] * q[i19]) + (3.018136e-04) * (q[i2] * q[i20]) + (1.285496e-04) * (q[i2] * q[i21])
            + (3.242108e-04) * (q[i2] * q[i22]) + (2.142900e-03) * (q[i3] * q[i4]) + (-6.349387e-03) * (q[i3] * q[i5]) + (3.245163e-04) * (q[i3] * q[i7])
            + (-3.734237e-03) * (q[i3] * q[i8]) + (-1.030080e-02) * (q[i3] * q[i9]) + (5.297648e-04) * (q[i3] * q[i10]) + (1.117085e-04) * (q[i3] * q[i11])
            + (1.625795e-04) * (q[i3] * q[i12]) + (1.331850e-03) * (q[i3] * q[i15]) + (-1.027732e-03) * (q[i3] * q[i16]) + (-2.949664e-03) * (q[i3] * q[i19])
            + (-1.961914e-04) * (q[i3] * q[i20]) + (-8.116925e-04) * (q[i3] * q[i21]) + (3.394242e-04) * (q[i3] * q[i22]) + (-5.931727e-04) * (q[i4] * q[i5])
            + (3.288686e-04) * (q[i4] * q[i7]) + (-3.885931e-04) * (q[i4] * q[i8]) + (1.505838e-03) * (q[i4] * q[i9]) + (1.077158e-04) * (q[i4] * q[i10])
            + (-7.801497e-04) * (q[i4] * q[i11]) + (8.819036e-04) * (q[i4] * q[i12]) + (-1.836521e-03) * (q[i4] * q[i15]) + (-7.226764e-04) * (q[i4] * q[i16])
            + (1.863697e-03) * (q[i4] * q[i19]) + (-1.470723e-04) * (q[i4] * q[i20]) + (-7.265601e-04) * (q[i4] * q[i21]) + (-1.339614e-03) * (q[i4] * q[i22])
            + (2.967315e-03) * (q[i5] * q[i7]) + (1.843466e-03) * (q[i5] * q[i8]) + (3.600806e-03) * (q[i5] * q[i9]) + (-1.788734e-03) * (q[i5] * q[i10])
            + (1.185197e-04) * (q[i5] * q[i11]) + (-1.385588e-03) * (q[i5] * q[i12]) + (-2.008343e-04) * (q[i5] * q[i15]) + (3.957857e-04) * (q[i5] * q[i16])
            + (2.459765e-04) * (q[i5] * q[i19]) + (6.275868e-04) * (q[i5] * q[i20]) + (4.400469e-04) * (q[i5] * q[i21]) + (5.221971e-04) * (q[i5] * q[i22])
            + (8.144014e-06) * (q[i7] * q[i8]) + (1.094756e-03) * (q[i7] * q[i9]) + (-1.072949e-03) * (q[i7] * q[i10]) + (9.749898e-04) * (q[i7] * q[i11])
            + (9.681896e-04) * (q[i7] * q[i12]) + (-1.479991e-04) * (q[i7] * q[i15]) + (1.486869e-04) * (q[i7] * q[i16]) + (3.730544e-04) * (q[i7] * q[i19])
            + (-3.841827e-04) * (q[i7] * q[i20]) + (-2.921076e-04) * (q[i7] * q[i21]) + (-2.994227e-04) * (q[i7] * q[i22]) + (2.919645e-04) * (q[i8] * q[i9])
            + (5.735390e-05) * (q[i8] * q[i10]) + (3.118483e-04) * (q[i8] * q[i11]) + (3.574590e-04) * (q[i8] * q[i12]) + (1.204213e-05) * (q[i8] * q[i15])
            + (8.807517e-04) * (q[i8] * q[i16]) + (-3.218953e-04) * (q[i8] * q[i19]) + (-2.926902e-06) * (q[i8] * q[i20]) + (3.936861e-05) * (q[i8] * q[i21])
            + (6.272907e-04) * (q[i8] * q[i22]) + (1.057842e-03) * (q[i9] * q[i10]) + (-8.023481e-04) * (q[i9] * q[i11]) + (-6.623801e-05) * (q[i9] * q[i12])
            + (-5.787308e-05) * (q[i9] * q[i15]) + (5.530778e-04) * (q[i9] * q[i16]) + (-5.147763e-04) * (q[i9] * q[i19]) + (-2.811633e-04) * (q[i9] * q[i20])
            + (-5.704405e-06) * (q[i9] * q[i21]) + (6.697351e-06) * (q[i9] * q[i22]) + (1.632620e-04) * (q[i10] * q[i11]) + (1.932459e-04) * (q[i10] * q[i12])
            + (5.898438e-05) * (q[i10] * q[i15]) + (-3.333471e-05) * (q[i10] * q[i16]) + (-3.367955e-06) * (q[i10] * q[i19])
            + (-6.403651e-05) * (q[i10] * q[i20]) + (6.740169e-04) * (q[i10] * q[i21]) + (3.724830e-04) * (q[i10] * q[i22])
            + (-9.789896e-05) * (q[i11] * q[i12]) + (1.189101e-03) * (q[i11] * q[i15]) + (1.504720e-04) * (q[i11] * q[i16]) + (2.453448e-05) * (q[i11] * q[i19])
            + (1.247018e-03) * (q[i11] * q[i20]) + (-3.542402e-04) * (q[i11] * q[i21]) + (-3.928276e-04) * (q[i11] * q[i22])
            + (1.037932e-03) * (q[i12] * q[i15]) + (3.039606e-04) * (q[i12] * q[i16]) + (1.155972e-04) * (q[i12] * q[i19]) + (-1.390614e-04) * (q[i12] * q[i20])
            + (9.065239e-05) * (q[i12] * q[i21]) + (-5.720250e-04) * (q[i12] * q[i22]) + (9.620974e-05) * (q[i15] * q[i16]) + (2.382661e-04) * (q[i15] * q[i19])
            + (-2.391586e-04) * (q[i15] * q[i20]) + (-5.152226e-05) * (q[i15] * q[i21]) + (2.113318e-04) * (q[i15] * q[i22])
            + (-4.223975e-04) * (q[i16] * q[i19]) + (1.109683e-05) * (q[i16] * q[i20]) + (1.089463e-04) * (q[i16] * q[i21]) + (1.466666e-04) * (q[i16] * q[i22])
            + (1.838184e-04) * (q[i19] * q[i20]) + (-1.257420e-04) * (q[i19] * q[i21]) + (-7.666956e-05) * (q[i19] * q[i22])
            + (8.421887e-04) * (q[i20] * q[i21]) + (-6.182464e-04) * (q[i20] * q[i22]) + (-1.593029e-04) * (q[i21] * q[i22]);
   }

   public void getJQx7(double[] q, double[][] JQ)
   {
      JQ[1][i7] = (-3.106478e-03) * (1) + (-1.817250e-02) * ((2) * q[i7]) + (-1.051662e-02) * (q[i0]) + (9.010674e-02) * (q[i1]) + (1.577288e-02) * (q[i2])
            + (-4.554090e-03) * (q[i3]) + (1.051373e-02) * (q[i4]) + (-1.434103e-03) * (q[i5]) + (-5.334098e-05) * (q[i6]) + (-2.560472e-03) * (q[i8])
            + (3.818315e-04) * (q[i9]) + (-8.383535e-03) * (q[i10]) + (4.708322e-03) * (q[i11]) + (3.765473e-03) * (q[i12]) + (7.197612e-04) * (q[i15])
            + (9.171318e-04) * (q[i16]) + (-1.185100e-03) * (q[i19]) + (-2.023647e-03) * (q[i20]) + (-1.223041e-03) * (q[i21]) + (-6.863549e-04) * (q[i22])
            + (2.032812e-03) * (q[i0] * q[i0]) + (-1.266027e-03) * (q[i1] * q[i1]) + (-9.400876e-04) * (q[i2] * q[i2]) + (2.994120e-04) * (q[i3] * q[i3])
            + (-1.868758e-03) * (q[i4] * q[i4]) + (1.739067e-04) * (q[i5] * q[i5]) + (-1.625084e-03) * (q[i6] * q[i6]) + (-3.453669e-03) * ((2) * q[i0] * q[i7])
            + (3.902302e-03) * ((2) * q[i1] * q[i7]) + (-4.773915e-04) * ((2) * q[i2] * q[i7]) + (3.528850e-03) * ((2) * q[i3] * q[i7])
            + (-2.179241e-02) * ((2) * q[i4] * q[i7]) + (4.394210e-03) * ((2) * q[i5] * q[i7]) + (1.623807e-03) * ((2) * q[i6] * q[i7])
            + (6.311920e-04) * ((3) * q[i7] * q[i7]) + (3.571365e-04) * ((2) * q[i7] * q[i8]) + (-1.603254e-04) * ((2) * q[i7] * q[i9])
            + (2.457586e-03) * ((2) * q[i7] * q[i10]) + (-6.901101e-04) * ((2) * q[i7] * q[i11]) + (5.123069e-04) * ((2) * q[i7] * q[i12])
            + (-1.107298e-03) * ((2) * q[i7] * q[i15]) + (-2.299380e-04) * ((2) * q[i7] * q[i16]) + (-4.174993e-04) * ((2) * q[i7] * q[i19])
            + (1.400944e-04) * ((2) * q[i7] * q[i20]) + (-5.819971e-05) * ((2) * q[i7] * q[i21]) + (2.482698e-04) * ((2) * q[i7] * q[i22])
            + (1.455535e-04) * (q[i8] * q[i8]) + (1.011449e-03) * (q[i9] * q[i9]) + (2.404803e-03) * (q[i10] * q[i10]) + (-2.701751e-05) * (q[i11] * q[i11])
            + (6.581599e-05) * (q[i12] * q[i12]) + (-5.139503e-05) * (q[i15] * q[i15]) + (-8.659144e-05) * (q[i16] * q[i16])
            + (3.239674e-04) * (q[i19] * q[i19]) + (1.465960e-04) * (q[i20] * q[i20]) + (-3.780352e-04) * (q[i21] * q[i21]) + (1.424931e-04) * (q[i22] * q[i22])
            + (-1.925395e-03) * (q[i0] * q[i1]) + (-2.001371e-03) * (q[i0] * q[i2]) + (4.599825e-04) * (q[i0] * q[i3]) + (-2.147826e-04) * (q[i0] * q[i4])
            + (-4.824738e-04) * (q[i0] * q[i5]) + (-4.657516e-04) * (q[i0] * q[i6]) + (-1.297956e-03) * (q[i0] * q[i8]) + (-1.327951e-04) * (q[i0] * q[i9])
            + (3.413640e-03) * (q[i0] * q[i10]) + (-9.047856e-04) * (q[i0] * q[i11]) + (-1.669194e-03) * (q[i0] * q[i12]) + (-6.133359e-04) * (q[i0] * q[i15])
            + (7.881685e-04) * (q[i0] * q[i16]) + (-6.159552e-04) * (q[i0] * q[i19]) + (-9.066067e-04) * (q[i0] * q[i20]) + (-2.006549e-04) * (q[i0] * q[i21])
            + (-8.392349e-04) * (q[i0] * q[i22]) + (-1.331022e-03) * (q[i1] * q[i2]) + (3.785884e-03) * (q[i1] * q[i3]) + (1.215547e-02) * (q[i1] * q[i4])
            + (8.116384e-03) * (q[i1] * q[i5]) + (-5.072719e-04) * (q[i1] * q[i6]) + (7.388077e-03) * (q[i1] * q[i8]) + (1.747766e-03) * (q[i1] * q[i9])
            + (-1.345148e-02) * (q[i1] * q[i10]) + (-2.929429e-04) * (q[i1] * q[i11]) + (1.519737e-03) * (q[i1] * q[i12]) + (2.038911e-03) * (q[i1] * q[i15])
            + (3.758379e-04) * (q[i1] * q[i16]) + (1.349245e-03) * (q[i1] * q[i19]) + (1.199835e-03) * (q[i1] * q[i20]) + (-6.180849e-04) * (q[i1] * q[i21])
            + (9.134953e-04) * (q[i1] * q[i22]) + (1.078962e-03) * (q[i2] * q[i3]) + (3.717713e-03) * (q[i2] * q[i4]) + (1.789531e-04) * (q[i2] * q[i5])
            + (3.344318e-03) * (q[i2] * q[i6]) + (5.755823e-03) * (q[i2] * q[i8]) + (7.057201e-04) * (q[i2] * q[i9]) + (-2.443772e-03) * (q[i2] * q[i10])
            + (3.318719e-04) * (q[i2] * q[i11]) + (7.383186e-04) * (q[i2] * q[i12]) + (2.397672e-04) * (q[i2] * q[i15]) + (5.739001e-04) * (q[i2] * q[i16])
            + (2.983729e-04) * (q[i2] * q[i19]) + (1.274549e-03) * (q[i2] * q[i20]) + (-3.183364e-04) * (q[i2] * q[i21]) + (-1.257341e-04) * (q[i2] * q[i22])
            + (-2.125483e-03) * (q[i3] * q[i4]) + (5.954575e-04) * (q[i3] * q[i5]) + (3.245163e-04) * (q[i3] * q[i6]) + (-3.796529e-04) * (q[i3] * q[i8])
            + (1.223988e-04) * (q[i3] * q[i9]) + (1.490958e-03) * (q[i3] * q[i10]) + (-9.071750e-04) * (q[i3] * q[i11]) + (7.903289e-04) * (q[i3] * q[i12])
            + (-7.167150e-04) * (q[i3] * q[i15]) + (-1.838161e-03) * (q[i3] * q[i16]) + (-1.571947e-04) * (q[i3] * q[i19]) + (1.872840e-03) * (q[i3] * q[i20])
            + (1.340993e-03) * (q[i3] * q[i21]) + (7.346822e-04) * (q[i3] * q[i22]) + (6.315469e-03) * (q[i4] * q[i5]) + (3.288686e-04) * (q[i4] * q[i6])
            + (-3.720154e-03) * (q[i4] * q[i8]) + (5.630538e-04) * (q[i4] * q[i9]) + (-1.020911e-02) * (q[i4] * q[i10]) + (-1.659809e-04) * (q[i4] * q[i11])
            + (-1.034888e-04) * (q[i4] * q[i12]) + (-1.031517e-03) * (q[i4] * q[i15]) + (1.346195e-03) * (q[i4] * q[i16]) + (-1.888206e-04) * (q[i4] * q[i19])
            + (-2.927266e-03) * (q[i4] * q[i20]) + (-3.443914e-04) * (q[i4] * q[i21]) + (8.115579e-04) * (q[i4] * q[i22]) + (2.967315e-03) * (q[i5] * q[i6])
            + (1.807979e-03) * (q[i5] * q[i8]) + (-1.811998e-03) * (q[i5] * q[i9]) + (3.594889e-03) * (q[i5] * q[i10]) + (1.385137e-03) * (q[i5] * q[i11])
            + (-1.108449e-04) * (q[i5] * q[i12]) + (3.823836e-04) * (q[i5] * q[i15]) + (-2.080655e-04) * (q[i5] * q[i16]) + (6.288068e-04) * (q[i5] * q[i19])
            + (2.373423e-04) * (q[i5] * q[i20]) + (-5.079405e-04) * (q[i5] * q[i21]) + (-4.616325e-04) * (q[i5] * q[i22]) + (8.144014e-06) * (q[i6] * q[i8])
            + (1.094756e-03) * (q[i6] * q[i9]) + (-1.072949e-03) * (q[i6] * q[i10]) + (9.749898e-04) * (q[i6] * q[i11]) + (9.681896e-04) * (q[i6] * q[i12])
            + (-1.479991e-04) * (q[i6] * q[i15]) + (1.486869e-04) * (q[i6] * q[i16]) + (3.730544e-04) * (q[i6] * q[i19]) + (-3.841827e-04) * (q[i6] * q[i20])
            + (-2.921076e-04) * (q[i6] * q[i21]) + (-2.994227e-04) * (q[i6] * q[i22]) + (-5.487595e-05) * (q[i8] * q[i9]) + (-2.853329e-04) * (q[i8] * q[i10])
            + (3.848970e-04) * (q[i8] * q[i11]) + (3.219046e-04) * (q[i8] * q[i12]) + (-8.874344e-04) * (q[i8] * q[i15]) + (-6.459206e-07) * (q[i8] * q[i16])
            + (2.425172e-06) * (q[i8] * q[i19]) + (3.094587e-04) * (q[i8] * q[i20]) + (6.312095e-04) * (q[i8] * q[i21]) + (3.965294e-05) * (q[i8] * q[i22])
            + (-1.062174e-03) * (q[i9] * q[i10]) + (2.032087e-04) * (q[i9] * q[i11]) + (1.531247e-04) * (q[i9] * q[i12]) + (3.024924e-05) * (q[i9] * q[i15])
            + (-5.499585e-05) * (q[i9] * q[i16]) + (6.235673e-05) * (q[i9] * q[i19]) + (-3.490015e-07) * (q[i9] * q[i20]) + (3.703065e-04) * (q[i9] * q[i21])
            + (6.726312e-04) * (q[i9] * q[i22]) + (-6.176399e-05) * (q[i10] * q[i11]) + (-7.972379e-04) * (q[i10] * q[i12])
            + (-5.512347e-04) * (q[i10] * q[i15]) + (6.889978e-05) * (q[i10] * q[i16]) + (2.787552e-04) * (q[i10] * q[i19]) + (5.044036e-04) * (q[i10] * q[i20])
            + (6.170180e-06) * (q[i10] * q[i21]) + (-1.011493e-05) * (q[i10] * q[i22]) + (1.092357e-04) * (q[i11] * q[i12]) + (3.001006e-04) * (q[i11] * q[i15])
            + (1.036485e-03) * (q[i11] * q[i16]) + (-1.439265e-04) * (q[i11] * q[i19]) + (1.173055e-04) * (q[i11] * q[i20]) + (5.647104e-04) * (q[i11] * q[i21])
            + (-8.827581e-05) * (q[i11] * q[i22]) + (1.521089e-04) * (q[i12] * q[i15]) + (1.174695e-03) * (q[i12] * q[i16]) + (1.244186e-03) * (q[i12] * q[i19])
            + (2.493281e-05) * (q[i12] * q[i20]) + (3.914591e-04) * (q[i12] * q[i21]) + (3.562947e-04) * (q[i12] * q[i22]) + (-9.605176e-05) * (q[i15] * q[i16])
            + (-1.144394e-05) * (q[i15] * q[i19]) + (4.214893e-04) * (q[i15] * q[i20]) + (1.458426e-04) * (q[i15] * q[i21]) + (1.105229e-04) * (q[i15] * q[i22])
            + (2.379539e-04) * (q[i16] * q[i19]) + (-2.380913e-04) * (q[i16] * q[i20]) + (2.063762e-04) * (q[i16] * q[i21])
            + (-5.774850e-05) * (q[i16] * q[i22]) + (-1.807043e-04) * (q[i19] * q[i20]) + (-6.118894e-04) * (q[i19] * q[i21])
            + (8.430038e-04) * (q[i19] * q[i22]) + (-8.372730e-05) * (q[i20] * q[i21]) + (-1.313692e-04) * (q[i20] * q[i22])
            + (1.575562e-04) * (q[i21] * q[i22]);
   }

   public void getJQx8(double[] q, double[][] JQ)
   {
      JQ[1][i8] = (-1.146618e-04) * (1) + (-7.790112e-06) * ((2) * q[i8]) + (-3.638438e-03) * (q[i0]) + (-3.676940e-03) * (q[i1]) + (-6.770789e-02) * (q[i2])
            + (2.986873e-03) * (q[i3]) + (3.061437e-03) * (q[i4]) + (4.079506e-04) * (q[i5]) + (2.546911e-03) * (q[i6]) + (-2.560472e-03) * (q[i7])
            + (1.827642e-03) * (q[i9]) + (-1.850971e-03) * (q[i10]) + (1.037983e-02) * (q[i11]) + (1.046921e-02) * (q[i12]) + (9.172387e-03) * (q[i15])
            + (-9.271249e-03) * (q[i16]) + (-4.194115e-04) * (q[i19]) + (4.509890e-04) * (q[i20]) + (2.241619e-03) * (q[i21]) + (2.269841e-03) * (q[i22])
            + (-1.493391e-03) * (q[i0] * q[i0]) + (1.472293e-03) * (q[i1] * q[i1]) + (-1.694320e-05) * (q[i2] * q[i2]) + (7.210461e-04) * (q[i3] * q[i3])
            + (-6.687702e-04) * (q[i4] * q[i4]) + (8.966249e-06) * (q[i5] * q[i5]) + (-3.501911e-04) * (q[i6] * q[i6]) + (3.571365e-04) * (q[i7] * q[i7])
            + (1.790137e-04) * ((2) * q[i0] * q[i8]) + (2.022371e-04) * ((2) * q[i1] * q[i8]) + (1.764850e-03) * ((2) * q[i2] * q[i8])
            + (5.135405e-03) * ((2) * q[i3] * q[i8]) + (5.120441e-03) * ((2) * q[i4] * q[i8]) + (-2.642023e-02) * ((2) * q[i5] * q[i8])
            + (-1.543149e-04) * ((2) * q[i6] * q[i8]) + (1.455535e-04) * ((2) * q[i7] * q[i8]) + (1.545282e-05) * ((3) * q[i8] * q[i8])
            + (-4.521382e-04) * ((2) * q[i8] * q[i9]) + (4.395179e-04) * ((2) * q[i8] * q[i10]) + (-1.781949e-03) * ((2) * q[i8] * q[i11])
            + (-1.816467e-03) * ((2) * q[i8] * q[i12]) + (4.086490e-03) * ((2) * q[i8] * q[i15]) + (-4.139522e-03) * ((2) * q[i8] * q[i16])
            + (-2.545432e-05) * ((2) * q[i8] * q[i19]) + (2.835757e-05) * ((2) * q[i8] * q[i20]) + (-6.908828e-04) * ((2) * q[i8] * q[i21])
            + (-6.994664e-04) * ((2) * q[i8] * q[i22]) + (3.629383e-04) * (q[i9] * q[i9]) + (-3.565921e-04) * (q[i10] * q[i10])
            + (4.277118e-04) * (q[i11] * q[i11]) + (-3.407250e-04) * (q[i12] * q[i12]) + (2.670214e-03) * (q[i15] * q[i15])
            + (-2.682642e-03) * (q[i16] * q[i16]) + (8.407745e-04) * (q[i19] * q[i19]) + (-8.367389e-04) * (q[i20] * q[i20])
            + (6.811219e-04) * (q[i21] * q[i21]) + (-6.902480e-04) * (q[i22] * q[i22]) + (4.442005e-06) * (q[i0] * q[i1]) + (-3.725796e-03) * (q[i0] * q[i2])
            + (2.879513e-03) * (q[i0] * q[i3]) + (-1.860052e-03) * (q[i0] * q[i4]) + (-3.618220e-04) * (q[i0] * q[i5]) + (7.409014e-03) * (q[i0] * q[i6])
            + (-1.297956e-03) * (q[i0] * q[i7]) + (7.407671e-04) * (q[i0] * q[i9]) + (-1.721679e-04) * (q[i0] * q[i10]) + (-5.959634e-04) * (q[i0] * q[i11])
            + (1.649874e-04) * (q[i0] * q[i12]) + (9.418310e-04) * (q[i0] * q[i15]) + (2.296953e-04) * (q[i0] * q[i16]) + (1.349733e-03) * (q[i0] * q[i19])
            + (1.054646e-03) * (q[i0] * q[i20]) + (3.697073e-04) * (q[i0] * q[i21]) + (-3.461998e-04) * (q[i0] * q[i22]) + (3.702176e-03) * (q[i1] * q[i2])
            + (1.894027e-03) * (q[i1] * q[i3]) + (-2.884023e-03) * (q[i1] * q[i4]) + (3.976265e-04) * (q[i1] * q[i5]) + (-1.312707e-03) * (q[i1] * q[i6])
            + (7.388077e-03) * (q[i1] * q[i7]) + (-1.896148e-04) * (q[i1] * q[i9]) + (7.417262e-04) * (q[i1] * q[i10]) + (-1.253281e-04) * (q[i1] * q[i11])
            + (6.184059e-04) * (q[i1] * q[i12]) + (2.099563e-04) * (q[i1] * q[i15]) + (9.333350e-04) * (q[i1] * q[i16]) + (1.060160e-03) * (q[i1] * q[i19])
            + (1.348211e-03) * (q[i1] * q[i20]) + (3.351103e-04) * (q[i1] * q[i21]) + (-3.669231e-04) * (q[i1] * q[i22]) + (3.084937e-03) * (q[i2] * q[i3])
            + (-3.042462e-03) * (q[i2] * q[i4]) + (-2.629788e-05) * (q[i2] * q[i5]) + (5.734035e-03) * (q[i2] * q[i6]) + (5.755823e-03) * (q[i2] * q[i7])
            + (-2.458956e-03) * (q[i2] * q[i9]) + (-2.408560e-03) * (q[i2] * q[i10]) + (-1.244990e-03) * (q[i2] * q[i11]) + (1.411772e-03) * (q[i2] * q[i12])
            + (4.348435e-03) * (q[i2] * q[i15]) + (4.428257e-03) * (q[i2] * q[i16]) + (1.071222e-03) * (q[i2] * q[i19]) + (1.062139e-03) * (q[i2] * q[i20])
            + (-5.368134e-04) * (q[i2] * q[i21]) + (5.417197e-04) * (q[i2] * q[i22]) + (-2.209751e-05) * (q[i3] * q[i4]) + (-4.838856e-04) * (q[i3] * q[i5])
            + (-3.734237e-03) * (q[i3] * q[i6]) + (-3.796529e-04) * (q[i3] * q[i7]) + (-7.091580e-04) * (q[i3] * q[i9]) + (9.674446e-04) * (q[i3] * q[i10])
            + (-8.620245e-04) * (q[i3] * q[i11]) + (1.817324e-03) * (q[i3] * q[i12]) + (4.639511e-04) * (q[i3] * q[i15]) + (-2.946867e-04) * (q[i3] * q[i16])
            + (-1.082728e-03) * (q[i3] * q[i19]) + (4.997872e-05) * (q[i3] * q[i20]) + (1.835052e-04) * (q[i3] * q[i21]) + (-1.137578e-04) * (q[i3] * q[i22])
            + (4.671618e-04) * (q[i4] * q[i5]) + (-3.885931e-04) * (q[i4] * q[i6]) + (-3.720154e-03) * (q[i4] * q[i7]) + (9.636456e-04) * (q[i4] * q[i9])
            + (-7.088078e-04) * (q[i4] * q[i10]) + (-1.784835e-03) * (q[i4] * q[i11]) + (8.717079e-04) * (q[i4] * q[i12]) + (-2.745696e-04) * (q[i4] * q[i15])
            + (4.798022e-04) * (q[i4] * q[i16]) + (5.460890e-05) * (q[i4] * q[i19]) + (-1.055922e-03) * (q[i4] * q[i20]) + (1.136042e-04) * (q[i4] * q[i21])
            + (-1.860377e-04) * (q[i4] * q[i22]) + (1.843466e-03) * (q[i5] * q[i6]) + (1.807979e-03) * (q[i5] * q[i7]) + (-1.520993e-03) * (q[i5] * q[i9])
            + (-1.520071e-03) * (q[i5] * q[i10]) + (1.744742e-03) * (q[i5] * q[i11]) + (-1.778833e-03) * (q[i5] * q[i12]) + (1.608008e-03) * (q[i5] * q[i15])
            + (1.625924e-03) * (q[i5] * q[i16]) + (-1.509317e-03) * (q[i5] * q[i19]) + (-1.517377e-03) * (q[i5] * q[i20]) + (6.891492e-04) * (q[i5] * q[i21])
            + (-7.023000e-04) * (q[i5] * q[i22]) + (8.144014e-06) * (q[i6] * q[i7]) + (2.919645e-04) * (q[i6] * q[i9]) + (5.735390e-05) * (q[i6] * q[i10])
            + (3.118483e-04) * (q[i6] * q[i11]) + (3.574590e-04) * (q[i6] * q[i12]) + (1.204213e-05) * (q[i6] * q[i15]) + (8.807517e-04) * (q[i6] * q[i16])
            + (-3.218953e-04) * (q[i6] * q[i19]) + (-2.926902e-06) * (q[i6] * q[i20]) + (3.936861e-05) * (q[i6] * q[i21]) + (6.272907e-04) * (q[i6] * q[i22])
            + (-5.487595e-05) * (q[i7] * q[i9]) + (-2.853329e-04) * (q[i7] * q[i10]) + (3.848970e-04) * (q[i7] * q[i11]) + (3.219046e-04) * (q[i7] * q[i12])
            + (-8.874344e-04) * (q[i7] * q[i15]) + (-6.459206e-07) * (q[i7] * q[i16]) + (2.425172e-06) * (q[i7] * q[i19]) + (3.094587e-04) * (q[i7] * q[i20])
            + (6.312095e-04) * (q[i7] * q[i21]) + (3.965294e-05) * (q[i7] * q[i22]) + (3.051791e-06) * (q[i9] * q[i10]) + (3.713144e-04) * (q[i9] * q[i11])
            + (1.890128e-03) * (q[i9] * q[i12]) + (-6.984873e-04) * (q[i9] * q[i15]) + (5.208003e-04) * (q[i9] * q[i16]) + (-2.432863e-04) * (q[i9] * q[i19])
            + (5.502630e-05) * (q[i9] * q[i20]) + (1.687497e-04) * (q[i9] * q[i21]) + (-6.421777e-04) * (q[i9] * q[i22]) + (1.871301e-03) * (q[i10] * q[i11])
            + (3.728237e-04) * (q[i10] * q[i12]) + (-5.211538e-04) * (q[i10] * q[i15]) + (6.973763e-04) * (q[i10] * q[i16])
            + (-4.806814e-05) * (q[i10] * q[i19]) + (2.439533e-04) * (q[i10] * q[i20]) + (-6.443793e-04) * (q[i10] * q[i21])
            + (1.681949e-04) * (q[i10] * q[i22]) + (-1.148201e-05) * (q[i11] * q[i12]) + (3.140654e-03) * (q[i11] * q[i15]) + (9.446172e-04) * (q[i11] * q[i16])
            + (1.077931e-03) * (q[i11] * q[i19]) + (-1.831442e-04) * (q[i11] * q[i20]) + (1.827731e-03) * (q[i11] * q[i21])
            + (-3.900571e-05) * (q[i11] * q[i22]) + (9.501223e-04) * (q[i12] * q[i15]) + (3.159565e-03) * (q[i12] * q[i16])
            + (-1.796733e-04) * (q[i12] * q[i19]) + (1.059739e-03) * (q[i12] * q[i20]) + (4.025228e-05) * (q[i12] * q[i21])
            + (-1.843622e-03) * (q[i12] * q[i22]) + (5.080928e-06) * (q[i15] * q[i16]) + (4.603370e-04) * (q[i15] * q[i19]) + (2.050454e-04) * (q[i15] * q[i20])
            + (-1.066225e-04) * (q[i15] * q[i21]) + (-4.518506e-04) * (q[i15] * q[i22]) + (-2.102962e-04) * (q[i16] * q[i19])
            + (-4.268456e-04) * (q[i16] * q[i20]) + (-4.533561e-04) * (q[i16] * q[i21]) + (-8.338577e-05) * (q[i16] * q[i22])
            + (5.811766e-08) * (q[i19] * q[i20]) + (-1.353064e-03) * (q[i19] * q[i21]) + (-1.799147e-05) * (q[i19] * q[i22])
            + (-8.841363e-06) * (q[i20] * q[i21]) + (-1.338323e-03) * (q[i20] * q[i22]) + (-1.803648e-06) * (q[i21] * q[i22]);
   }

   public void getJQx9(double[] q, double[][] JQ)
   {
      JQ[1][i9] = (4.889873e-03) * (1) + (4.028609e-03) * ((2) * q[i9]) + (2.931941e-02) * (q[i0]) + (-1.585410e-03) * (q[i1]) + (3.152888e-03) * (q[i2])
            + (-7.460434e-03) * (q[i3]) + (4.908594e-03) * (q[i4]) + (2.276604e-03) * (q[i5]) + (8.509535e-03) * (q[i6]) + (3.818315e-04) * (q[i7])
            + (1.827642e-03) * (q[i8]) + (3.718957e-06) * (q[i10]) + (1.113186e-03) * (q[i11]) + (2.158386e-03) * (q[i12]) + (-1.509882e-03) * (q[i15])
            + (3.377185e-04) * (q[i16]) + (1.013386e-04) * (q[i19]) + (1.035450e-03) * (q[i20]) + (2.236456e-04) * (q[i21]) + (-5.577125e-05) * (q[i22])
            + (2.952615e-03) * (q[i0] * q[i0]) + (3.075456e-04) * (q[i1] * q[i1]) + (1.309976e-04) * (q[i2] * q[i2]) + (-1.857075e-04) * (q[i3] * q[i3])
            + (2.262692e-04) * (q[i4] * q[i4]) + (1.513727e-04) * (q[i5] * q[i5]) + (-2.486045e-03) * (q[i6] * q[i6]) + (-1.603254e-04) * (q[i7] * q[i7])
            + (-4.521382e-04) * (q[i8] * q[i8]) + (-6.786792e-03) * ((2) * q[i0] * q[i9]) + (1.277800e-03) * ((2) * q[i1] * q[i9])
            + (-2.022016e-03) * ((2) * q[i2] * q[i9]) + (-3.546787e-03) * ((2) * q[i3] * q[i9]) + (4.736603e-04) * ((2) * q[i4] * q[i9])
            + (1.952825e-03) * ((2) * q[i5] * q[i9]) + (-2.439674e-03) * ((2) * q[i6] * q[i9]) + (1.011449e-03) * ((2) * q[i7] * q[i9])
            + (3.629383e-04) * ((2) * q[i8] * q[i9]) + (-5.922928e-04) * ((3) * q[i9] * q[i9]) + (2.615357e-04) * ((2) * q[i9] * q[i10])
            + (-7.666270e-05) * ((2) * q[i9] * q[i11]) + (-6.388348e-04) * ((2) * q[i9] * q[i12]) + (2.176835e-04) * ((2) * q[i9] * q[i15])
            + (1.270710e-04) * ((2) * q[i9] * q[i16]) + (-1.505523e-04) * ((2) * q[i9] * q[i19]) + (-1.073877e-05) * ((2) * q[i9] * q[i20])
            + (-2.193992e-05) * ((2) * q[i9] * q[i21]) + (-1.976300e-05) * ((2) * q[i9] * q[i22]) + (-2.601219e-04) * (q[i10] * q[i10])
            + (-2.190168e-04) * (q[i11] * q[i11]) + (-5.664863e-04) * (q[i12] * q[i12]) + (-7.293982e-05) * (q[i15] * q[i15])
            + (2.559521e-05) * (q[i16] * q[i16]) + (-3.021958e-05) * (q[i19] * q[i19]) + (-4.193547e-05) * (q[i20] * q[i20])
            + (1.256556e-04) * (q[i21] * q[i21]) + (1.046439e-04) * (q[i22] * q[i22]) + (-1.418768e-03) * (q[i0] * q[i1]) + (1.292606e-03) * (q[i0] * q[i2])
            + (-1.020871e-03) * (q[i0] * q[i3]) + (-6.762930e-04) * (q[i0] * q[i4]) + (4.046716e-04) * (q[i0] * q[i5]) + (-1.360672e-02) * (q[i0] * q[i6])
            + (-1.327951e-04) * (q[i0] * q[i7]) + (7.407671e-04) * (q[i0] * q[i8]) + (7.379308e-04) * (q[i0] * q[i10]) + (1.292349e-05) * (q[i0] * q[i11])
            + (-5.578639e-04) * (q[i0] * q[i12]) + (7.242263e-04) * (q[i0] * q[i15]) + (2.030050e-04) * (q[i0] * q[i16]) + (8.202329e-04) * (q[i0] * q[i19])
            + (5.663061e-04) * (q[i0] * q[i20]) + (-4.827697e-04) * (q[i0] * q[i21]) + (-5.376229e-05) * (q[i0] * q[i22]) + (-5.259012e-04) * (q[i1] * q[i2])
            + (-7.905932e-04) * (q[i1] * q[i3]) + (-1.469909e-03) * (q[i1] * q[i4]) + (8.545363e-04) * (q[i1] * q[i5]) + (3.417191e-03) * (q[i1] * q[i6])
            + (1.747766e-03) * (q[i1] * q[i7]) + (-1.896148e-04) * (q[i1] * q[i8]) + (7.396080e-04) * (q[i1] * q[i10]) + (6.116578e-05) * (q[i1] * q[i11])
            + (9.128566e-04) * (q[i1] * q[i12]) + (1.494251e-04) * (q[i1] * q[i15]) + (-7.386152e-05) * (q[i1] * q[i16]) + (-6.724032e-04) * (q[i1] * q[i19])
            + (-3.448627e-04) * (q[i1] * q[i20]) + (4.375698e-04) * (q[i1] * q[i21]) + (6.739413e-04) * (q[i1] * q[i22]) + (1.821206e-04) * (q[i2] * q[i3])
            + (4.203955e-04) * (q[i2] * q[i4]) + (2.116555e-03) * (q[i2] * q[i5]) + (-2.505272e-03) * (q[i2] * q[i6]) + (7.057201e-04) * (q[i2] * q[i7])
            + (-2.458956e-03) * (q[i2] * q[i8]) + (-1.138271e-04) * (q[i2] * q[i10]) + (-4.428682e-04) * (q[i2] * q[i11]) + (1.445852e-04) * (q[i2] * q[i12])
            + (6.495961e-04) * (q[i2] * q[i15]) + (-4.232450e-04) * (q[i2] * q[i16]) + (1.164111e-04) * (q[i2] * q[i19]) + (3.097910e-04) * (q[i2] * q[i20])
            + (-3.177849e-04) * (q[i2] * q[i21]) + (2.041463e-04) * (q[i2] * q[i22]) + (-4.865914e-05) * (q[i3] * q[i4]) + (-1.903444e-04) * (q[i3] * q[i5])
            + (-1.030080e-02) * (q[i3] * q[i6]) + (1.223988e-04) * (q[i3] * q[i7]) + (-7.091580e-04) * (q[i3] * q[i8]) + (-1.125415e-03) * (q[i3] * q[i10])
            + (-5.818825e-04) * (q[i3] * q[i11]) + (2.452206e-04) * (q[i3] * q[i12]) + (-2.710467e-04) * (q[i3] * q[i15]) + (-4.947617e-04) * (q[i3] * q[i16])
            + (2.575008e-04) * (q[i3] * q[i19]) + (-1.099831e-03) * (q[i3] * q[i20]) + (2.429874e-04) * (q[i3] * q[i21]) + (-1.711662e-04) * (q[i3] * q[i22])
            + (1.461121e-03) * (q[i4] * q[i5]) + (1.505838e-03) * (q[i4] * q[i6]) + (5.630538e-04) * (q[i4] * q[i7]) + (9.636456e-04) * (q[i4] * q[i8])
            + (-1.116048e-03) * (q[i4] * q[i10]) + (-2.538559e-04) * (q[i4] * q[i11]) + (7.054069e-05) * (q[i4] * q[i12]) + (-6.173504e-05) * (q[i4] * q[i15])
            + (-6.497450e-04) * (q[i4] * q[i16]) + (-1.113940e-04) * (q[i4] * q[i19]) + (9.484636e-04) * (q[i4] * q[i20]) + (-3.795851e-04) * (q[i4] * q[i21])
            + (2.792880e-04) * (q[i4] * q[i22]) + (3.600806e-03) * (q[i5] * q[i6]) + (-1.811998e-03) * (q[i5] * q[i7]) + (-1.520993e-03) * (q[i5] * q[i8])
            + (3.736259e-04) * (q[i5] * q[i10]) + (-3.088273e-04) * (q[i5] * q[i11]) + (-1.763527e-04) * (q[i5] * q[i12]) + (-1.271194e-05) * (q[i5] * q[i15])
            + (-2.967064e-06) * (q[i5] * q[i16]) + (-1.319905e-04) * (q[i5] * q[i19]) + (6.840200e-05) * (q[i5] * q[i20]) + (5.699121e-05) * (q[i5] * q[i21])
            + (-1.983944e-04) * (q[i5] * q[i22]) + (1.094756e-03) * (q[i6] * q[i7]) + (2.919645e-04) * (q[i6] * q[i8]) + (1.057842e-03) * (q[i6] * q[i10])
            + (-8.023481e-04) * (q[i6] * q[i11]) + (-6.623801e-05) * (q[i6] * q[i12]) + (-5.787308e-05) * (q[i6] * q[i15]) + (5.530778e-04) * (q[i6] * q[i16])
            + (-5.147763e-04) * (q[i6] * q[i19]) + (-2.811633e-04) * (q[i6] * q[i20]) + (-5.704405e-06) * (q[i6] * q[i21]) + (6.697351e-06) * (q[i6] * q[i22])
            + (-5.487595e-05) * (q[i7] * q[i8]) + (-1.062174e-03) * (q[i7] * q[i10]) + (2.032087e-04) * (q[i7] * q[i11]) + (1.531247e-04) * (q[i7] * q[i12])
            + (3.024924e-05) * (q[i7] * q[i15]) + (-5.499585e-05) * (q[i7] * q[i16]) + (6.235673e-05) * (q[i7] * q[i19]) + (-3.490015e-07) * (q[i7] * q[i20])
            + (3.703065e-04) * (q[i7] * q[i21]) + (6.726312e-04) * (q[i7] * q[i22]) + (3.051791e-06) * (q[i8] * q[i10]) + (3.713144e-04) * (q[i8] * q[i11])
            + (1.890128e-03) * (q[i8] * q[i12]) + (-6.984873e-04) * (q[i8] * q[i15]) + (5.208003e-04) * (q[i8] * q[i16]) + (-2.432863e-04) * (q[i8] * q[i19])
            + (5.502630e-05) * (q[i8] * q[i20]) + (1.687497e-04) * (q[i8] * q[i21]) + (-6.421777e-04) * (q[i8] * q[i22]) + (-1.215391e-04) * (q[i10] * q[i11])
            + (-1.253624e-04) * (q[i10] * q[i12]) + (3.373777e-04) * (q[i10] * q[i15]) + (-3.369648e-04) * (q[i10] * q[i16])
            + (-4.452825e-05) * (q[i10] * q[i19]) + (4.880631e-05) * (q[i10] * q[i20]) + (1.934615e-04) * (q[i10] * q[i21]) + (1.904536e-04) * (q[i10] * q[i22])
            + (-2.414728e-04) * (q[i11] * q[i12]) + (1.564612e-04) * (q[i11] * q[i15]) + (1.517742e-04) * (q[i11] * q[i16]) + (4.364411e-05) * (q[i11] * q[i19])
            + (2.088469e-04) * (q[i11] * q[i20]) + (2.569153e-05) * (q[i11] * q[i21]) + (-1.682093e-04) * (q[i11] * q[i22])
            + (-1.483531e-04) * (q[i12] * q[i15]) + (6.375883e-05) * (q[i12] * q[i16]) + (-8.314378e-05) * (q[i12] * q[i19])
            + (-5.902861e-05) * (q[i12] * q[i20]) + (2.179966e-04) * (q[i12] * q[i21]) + (9.555884e-05) * (q[i12] * q[i22])
            + (-2.402656e-04) * (q[i15] * q[i16]) + (1.483290e-04) * (q[i15] * q[i19]) + (1.634168e-04) * (q[i15] * q[i20]) + (6.501105e-05) * (q[i15] * q[i21])
            + (-3.990033e-05) * (q[i15] * q[i22]) + (-2.577399e-04) * (q[i16] * q[i19]) + (1.908896e-04) * (q[i16] * q[i20])
            + (4.393048e-05) * (q[i16] * q[i21]) + (-1.016466e-04) * (q[i16] * q[i22]) + (1.544749e-04) * (q[i19] * q[i20])
            + (-2.061993e-05) * (q[i19] * q[i21]) + (1.277229e-04) * (q[i19] * q[i22]) + (1.741792e-04) * (q[i20] * q[i21])
            + (-3.629460e-04) * (q[i20] * q[i22]) + (-1.726096e-05) * (q[i21] * q[i22]);
   }

   public void getJQx10(double[] q, double[][] JQ)
   {
      JQ[1][i10] = (-4.803546e-03) * (1) + (-3.969694e-03) * ((2) * q[i10]) + (-1.566182e-03) * (q[i0]) + (2.896086e-02) * (q[i1]) + (3.116881e-03) * (q[i2])
            + (4.861489e-03) * (q[i3]) + (-7.410230e-03) * (q[i4]) + (2.289996e-03) * (q[i5]) + (-4.073500e-04) * (q[i6]) + (-8.383535e-03) * (q[i7])
            + (-1.850971e-03) * (q[i8]) + (3.718957e-06) * (q[i9]) + (2.136864e-03) * (q[i11]) + (1.103518e-03) * (q[i12]) + (-3.305814e-04) * (q[i15])
            + (1.497577e-03) * (q[i16]) + (-1.040316e-03) * (q[i19]) + (-8.931737e-05) * (q[i20]) + (-5.361554e-05) * (q[i21]) + (2.301890e-04) * (q[i22])
            + (-2.826019e-04) * (q[i0] * q[i0]) + (-2.936930e-03) * (q[i1] * q[i1]) + (-1.301474e-04) * (q[i2] * q[i2]) + (-2.367540e-04) * (q[i3] * q[i3])
            + (1.723745e-04) * (q[i4] * q[i4]) + (-1.422913e-04) * (q[i5] * q[i5]) + (1.582213e-04) * (q[i6] * q[i6]) + (2.457586e-03) * (q[i7] * q[i7])
            + (4.395179e-04) * (q[i8] * q[i8]) + (2.615357e-04) * (q[i9] * q[i9]) + (1.268991e-03) * ((2) * q[i0] * q[i10])
            + (-6.708023e-03) * ((2) * q[i1] * q[i10]) + (-1.993552e-03) * ((2) * q[i2] * q[i10]) + (4.730181e-04) * ((2) * q[i3] * q[i10])
            + (-3.499427e-03) * ((2) * q[i4] * q[i10]) + (1.924849e-03) * ((2) * q[i5] * q[i10]) + (-9.914628e-04) * ((2) * q[i6] * q[i10])
            + (2.404803e-03) * ((2) * q[i7] * q[i10]) + (-3.565921e-04) * ((2) * q[i8] * q[i10]) + (-2.601219e-04) * ((2) * q[i9] * q[i10])
            + (5.824171e-04) * ((3) * q[i10] * q[i10]) + (-6.364222e-04) * ((2) * q[i10] * q[i11]) + (-7.248281e-05) * ((2) * q[i10] * q[i12])
            + (-1.262826e-04) * ((2) * q[i10] * q[i15]) + (-2.161482e-04) * ((2) * q[i10] * q[i16]) + (1.223015e-05) * ((2) * q[i10] * q[i19])
            + (1.445431e-04) * ((2) * q[i10] * q[i20]) + (-2.028236e-05) * ((2) * q[i10] * q[i21]) + (-2.286918e-05) * ((2) * q[i10] * q[i22])
            + (5.507668e-04) * (q[i11] * q[i11]) + (2.103634e-04) * (q[i12] * q[i12]) + (-2.545667e-05) * (q[i15] * q[i15]) + (7.266301e-05) * (q[i16] * q[i16])
            + (4.162951e-05) * (q[i19] * q[i19]) + (2.972484e-05) * (q[i20] * q[i20]) + (-1.027291e-04) * (q[i21] * q[i21])
            + (-1.250614e-04) * (q[i22] * q[i22]) + (1.421309e-03) * (q[i0] * q[i1]) + (5.500446e-04) * (q[i0] * q[i2]) + (1.436438e-03) * (q[i0] * q[i3])
            + (7.799416e-04) * (q[i0] * q[i4]) + (-8.990503e-04) * (q[i0] * q[i5]) + (1.714814e-03) * (q[i0] * q[i6]) + (3.413640e-03) * (q[i0] * q[i7])
            + (-1.721679e-04) * (q[i0] * q[i8]) + (7.379308e-04) * (q[i0] * q[i9]) + (-8.957915e-04) * (q[i0] * q[i11]) + (-5.991964e-05) * (q[i0] * q[i12])
            + (-8.095689e-05) * (q[i0] * q[i15]) + (1.579682e-04) * (q[i0] * q[i16]) + (-3.351315e-04) * (q[i0] * q[i19]) + (-6.638931e-04) * (q[i0] * q[i20])
            + (-6.733610e-04) * (q[i0] * q[i21]) + (-4.308028e-04) * (q[i0] * q[i22]) + (-1.279786e-03) * (q[i1] * q[i2]) + (6.715230e-04) * (q[i1] * q[i3])
            + (1.007366e-03) * (q[i1] * q[i4]) + (-3.880431e-04) * (q[i1] * q[i5]) + (-1.111062e-04) * (q[i1] * q[i6]) + (-1.345148e-02) * (q[i1] * q[i7])
            + (7.417262e-04) * (q[i1] * q[i8]) + (7.396080e-04) * (q[i1] * q[i9]) + (5.542927e-04) * (q[i1] * q[i11]) + (-1.430565e-05) * (q[i1] * q[i12])
            + (1.871632e-04) * (q[i1] * q[i15]) + (7.150911e-04) * (q[i1] * q[i16]) + (5.671184e-04) * (q[i1] * q[i19]) + (8.148392e-04) * (q[i1] * q[i20])
            + (5.603105e-05) * (q[i1] * q[i21]) + (4.743153e-04) * (q[i1] * q[i22]) + (-4.290648e-04) * (q[i2] * q[i3]) + (-1.730749e-04) * (q[i2] * q[i4])
            + (-2.136809e-03) * (q[i2] * q[i5]) + (7.022326e-04) * (q[i2] * q[i6]) + (-2.443772e-03) * (q[i2] * q[i7]) + (-2.408560e-03) * (q[i2] * q[i8])
            + (-1.138271e-04) * (q[i2] * q[i9]) + (-1.591874e-04) * (q[i2] * q[i11]) + (4.314493e-04) * (q[i2] * q[i12]) + (-4.261787e-04) * (q[i2] * q[i15])
            + (6.458251e-04) * (q[i2] * q[i16]) + (3.252194e-04) * (q[i2] * q[i19]) + (1.234765e-04) * (q[i2] * q[i20]) + (-2.055229e-04) * (q[i2] * q[i21])
            + (3.180438e-04) * (q[i2] * q[i22]) + (5.663865e-05) * (q[i3] * q[i4]) + (-1.450961e-03) * (q[i3] * q[i5]) + (5.297648e-04) * (q[i3] * q[i6])
            + (1.490958e-03) * (q[i3] * q[i7]) + (9.674446e-04) * (q[i3] * q[i8]) + (-1.125415e-03) * (q[i3] * q[i9]) + (-7.761758e-05) * (q[i3] * q[i11])
            + (2.547123e-04) * (q[i3] * q[i12]) + (-6.593369e-04) * (q[i3] * q[i15]) + (-5.319852e-05) * (q[i3] * q[i16]) + (9.528878e-04) * (q[i3] * q[i19])
            + (-1.118422e-04) * (q[i3] * q[i20]) + (-2.816300e-04) * (q[i3] * q[i21]) + (3.857699e-04) * (q[i3] * q[i22]) + (1.876810e-04) * (q[i4] * q[i5])
            + (1.077158e-04) * (q[i4] * q[i6]) + (-1.020911e-02) * (q[i4] * q[i7]) + (-7.088078e-04) * (q[i4] * q[i8]) + (-1.116048e-03) * (q[i4] * q[i9])
            + (-2.539604e-04) * (q[i4] * q[i11]) + (5.829149e-04) * (q[i4] * q[i12]) + (-4.795333e-04) * (q[i4] * q[i15]) + (-2.686769e-04) * (q[i4] * q[i16])
            + (-1.089708e-03) * (q[i4] * q[i19]) + (2.507049e-04) * (q[i4] * q[i20]) + (1.679941e-04) * (q[i4] * q[i21]) + (-2.440550e-04) * (q[i4] * q[i22])
            + (-1.788734e-03) * (q[i5] * q[i6]) + (3.594889e-03) * (q[i5] * q[i7]) + (-1.520071e-03) * (q[i5] * q[i8]) + (3.736259e-04) * (q[i5] * q[i9])
            + (1.690130e-04) * (q[i5] * q[i11]) + (3.018424e-04) * (q[i5] * q[i12]) + (1.603742e-06) * (q[i5] * q[i15]) + (-2.413245e-05) * (q[i5] * q[i16])
            + (6.552438e-05) * (q[i5] * q[i19]) + (-1.415325e-04) * (q[i5] * q[i20]) + (1.988570e-04) * (q[i5] * q[i21]) + (-7.148407e-05) * (q[i5] * q[i22])
            + (-1.072949e-03) * (q[i6] * q[i7]) + (5.735390e-05) * (q[i6] * q[i8]) + (1.057842e-03) * (q[i6] * q[i9]) + (1.632620e-04) * (q[i6] * q[i11])
            + (1.932459e-04) * (q[i6] * q[i12]) + (5.898438e-05) * (q[i6] * q[i15]) + (-3.333471e-05) * (q[i6] * q[i16]) + (-3.367955e-06) * (q[i6] * q[i19])
            + (-6.403651e-05) * (q[i6] * q[i20]) + (6.740169e-04) * (q[i6] * q[i21]) + (3.724830e-04) * (q[i6] * q[i22]) + (-2.853329e-04) * (q[i7] * q[i8])
            + (-1.062174e-03) * (q[i7] * q[i9]) + (-6.176399e-05) * (q[i7] * q[i11]) + (-7.972379e-04) * (q[i7] * q[i12]) + (-5.512347e-04) * (q[i7] * q[i15])
            + (6.889978e-05) * (q[i7] * q[i16]) + (2.787552e-04) * (q[i7] * q[i19]) + (5.044036e-04) * (q[i7] * q[i20]) + (6.170180e-06) * (q[i7] * q[i21])
            + (-1.011493e-05) * (q[i7] * q[i22]) + (3.051791e-06) * (q[i8] * q[i9]) + (1.871301e-03) * (q[i8] * q[i11]) + (3.728237e-04) * (q[i8] * q[i12])
            + (-5.211538e-04) * (q[i8] * q[i15]) + (6.973763e-04) * (q[i8] * q[i16]) + (-4.806814e-05) * (q[i8] * q[i19]) + (2.439533e-04) * (q[i8] * q[i20])
            + (-6.443793e-04) * (q[i8] * q[i21]) + (1.681949e-04) * (q[i8] * q[i22]) + (-1.215391e-04) * (q[i9] * q[i11]) + (-1.253624e-04) * (q[i9] * q[i12])
            + (3.373777e-04) * (q[i9] * q[i15]) + (-3.369648e-04) * (q[i9] * q[i16]) + (-4.452825e-05) * (q[i9] * q[i19]) + (4.880631e-05) * (q[i9] * q[i20])
            + (1.934615e-04) * (q[i9] * q[i21]) + (1.904536e-04) * (q[i9] * q[i22]) + (2.459750e-04) * (q[i11] * q[i12]) + (6.393224e-05) * (q[i11] * q[i15])
            + (-1.507483e-04) * (q[i11] * q[i16]) + (-4.681708e-05) * (q[i11] * q[i19]) + (-8.202952e-05) * (q[i11] * q[i20])
            + (-9.864996e-05) * (q[i11] * q[i21]) + (-2.166452e-04) * (q[i11] * q[i22]) + (1.453329e-04) * (q[i12] * q[i15])
            + (1.589430e-04) * (q[i12] * q[i16]) + (2.078339e-04) * (q[i12] * q[i19]) + (4.660277e-05) * (q[i12] * q[i20]) + (1.714616e-04) * (q[i12] * q[i21])
            + (-2.717697e-05) * (q[i12] * q[i22]) + (2.381822e-04) * (q[i15] * q[i16]) + (-1.906830e-04) * (q[i15] * q[i19])
            + (2.572036e-04) * (q[i15] * q[i20]) + (-9.423758e-05) * (q[i15] * q[i21]) + (4.601349e-05) * (q[i15] * q[i22])
            + (-1.615727e-04) * (q[i16] * q[i19]) + (-1.470767e-04) * (q[i16] * q[i20]) + (-4.275113e-05) * (q[i16] * q[i21])
            + (6.190540e-05) * (q[i16] * q[i22]) + (-1.553178e-04) * (q[i19] * q[i20]) + (-3.572332e-04) * (q[i19] * q[i21])
            + (1.728515e-04) * (q[i19] * q[i22]) + (1.273184e-04) * (q[i20] * q[i21]) + (-2.134184e-05) * (q[i20] * q[i22])
            + (1.970438e-05) * (q[i21] * q[i22]);
   }

   public void getJQx11(double[] q, double[][] JQ)
   {
      JQ[1][i11] = (2.683704e-03) * (1) + (5.184729e-03) * ((2) * q[i11]) + (2.431821e-03) * (q[i0]) + (2.549366e-04) * (q[i1]) + (5.278294e-03) * (q[i2])
            + (-8.971683e-04) * (q[i3]) + (-2.383645e-03) * (q[i4]) + (5.436482e-03) * (q[i5]) + (3.803170e-03) * (q[i6]) + (4.708322e-03) * (q[i7])
            + (1.037983e-02) * (q[i8]) + (1.113186e-03) * (q[i9]) + (2.136864e-03) * (q[i10]) + (1.641208e-05) * (q[i12]) + (1.869733e-03) * (q[i15])
            + (-5.634301e-05) * (q[i16]) + (-1.497320e-03) * (q[i19]) + (1.693188e-03) * (q[i20]) + (4.262676e-04) * (q[i21]) + (-2.681229e-04) * (q[i22])
            + (7.179850e-04) * (q[i0] * q[i0]) + (2.325739e-05) * (q[i1] * q[i1]) + (6.982843e-04) * (q[i2] * q[i2]) + (-1.402094e-03) * (q[i3] * q[i3])
            + (-2.837358e-03) * (q[i4] * q[i4]) + (2.879716e-04) * (q[i5] * q[i5]) + (5.171001e-04) * (q[i6] * q[i6]) + (-6.901101e-04) * (q[i7] * q[i7])
            + (-1.781949e-03) * (q[i8] * q[i8]) + (-7.666270e-05) * (q[i9] * q[i9]) + (-6.364222e-04) * (q[i10] * q[i10])
            + (2.634201e-04) * ((2) * q[i0] * q[i11]) + (3.093027e-04) * ((2) * q[i1] * q[i11]) + (-4.927059e-05) * ((2) * q[i2] * q[i11])
            + (-3.952900e-04) * ((2) * q[i3] * q[i11]) + (-1.005719e-03) * ((2) * q[i4] * q[i11]) + (-1.257756e-03) * ((2) * q[i5] * q[i11])
            + (-5.734439e-05) * ((2) * q[i6] * q[i11]) + (-2.701751e-05) * ((2) * q[i7] * q[i11]) + (4.277118e-04) * ((2) * q[i8] * q[i11])
            + (-2.190168e-04) * ((2) * q[i9] * q[i11]) + (5.507668e-04) * ((2) * q[i10] * q[i11]) + (-7.581977e-05) * ((3) * q[i11] * q[i11])
            + (2.780140e-04) * ((2) * q[i11] * q[i12]) + (1.788992e-03) * ((2) * q[i11] * q[i15]) + (1.087262e-04) * ((2) * q[i11] * q[i16])
            + (4.134884e-04) * ((2) * q[i11] * q[i19]) + (-1.291444e-04) * ((2) * q[i11] * q[i20]) + (8.513405e-04) * ((2) * q[i11] * q[i21])
            + (6.335773e-05) * ((2) * q[i11] * q[i22]) + (2.928184e-04) * (q[i12] * q[i12]) + (2.021889e-03) * (q[i15] * q[i15])
            + (1.245777e-04) * (q[i16] * q[i16]) + (-1.188472e-04) * (q[i19] * q[i19]) + (-3.707643e-05) * (q[i20] * q[i20])
            + (4.836974e-04) * (q[i21] * q[i21]) + (1.762423e-04) * (q[i22] * q[i22]) + (-8.110965e-04) * (q[i0] * q[i1]) + (6.652454e-04) * (q[i0] * q[i2])
            + (4.059029e-05) * (q[i0] * q[i3]) + (1.098684e-03) * (q[i0] * q[i4]) + (-6.948002e-05) * (q[i0] * q[i5]) + (-1.493674e-03) * (q[i0] * q[i6])
            + (-9.047856e-04) * (q[i0] * q[i7]) + (-5.959634e-04) * (q[i0] * q[i8]) + (1.292349e-05) * (q[i0] * q[i9]) + (-8.957915e-04) * (q[i0] * q[i10])
            + (1.037433e-03) * (q[i0] * q[i12]) + (1.111723e-03) * (q[i0] * q[i15]) + (-6.870321e-05) * (q[i0] * q[i16]) + (6.155529e-04) * (q[i0] * q[i19])
            + (-2.593303e-04) * (q[i0] * q[i20]) + (7.605467e-04) * (q[i0] * q[i21]) + (3.498287e-04) * (q[i0] * q[i22]) + (1.404683e-03) * (q[i1] * q[i2])
            + (-1.809140e-03) * (q[i1] * q[i3]) + (-1.716275e-03) * (q[i1] * q[i4]) + (3.387693e-04) * (q[i1] * q[i5]) + (1.669093e-03) * (q[i1] * q[i6])
            + (-2.929429e-04) * (q[i1] * q[i7]) + (-1.253281e-04) * (q[i1] * q[i8]) + (6.116578e-05) * (q[i1] * q[i9]) + (5.542927e-04) * (q[i1] * q[i10])
            + (1.050391e-03) * (q[i1] * q[i12]) + (2.987142e-04) * (q[i1] * q[i15]) + (3.494695e-04) * (q[i1] * q[i16]) + (5.263009e-04) * (q[i1] * q[i19])
            + (-1.317697e-04) * (q[i1] * q[i20]) + (-7.379106e-04) * (q[i1] * q[i21]) + (4.227531e-04) * (q[i1] * q[i22]) + (5.062225e-04) * (q[i2] * q[i3])
            + (-4.936573e-04) * (q[i2] * q[i4]) + (1.737642e-03) * (q[i2] * q[i5]) + (-7.407081e-04) * (q[i2] * q[i6]) + (3.318719e-04) * (q[i2] * q[i7])
            + (-1.244990e-03) * (q[i2] * q[i8]) + (-4.428682e-04) * (q[i2] * q[i9]) + (-1.591874e-04) * (q[i2] * q[i10]) + (1.565082e-03) * (q[i2] * q[i12])
            + (3.600388e-03) * (q[i2] * q[i15]) + (-6.828608e-04) * (q[i2] * q[i16]) + (1.696311e-03) * (q[i2] * q[i19]) + (5.165242e-05) * (q[i2] * q[i20])
            + (8.125435e-05) * (q[i2] * q[i21]) + (-1.375290e-04) * (q[i2] * q[i22]) + (1.984605e-03) * (q[i3] * q[i4]) + (-8.940210e-04) * (q[i3] * q[i5])
            + (1.117085e-04) * (q[i3] * q[i6]) + (-9.071750e-04) * (q[i3] * q[i7]) + (-8.620245e-04) * (q[i3] * q[i8]) + (-5.818825e-04) * (q[i3] * q[i9])
            + (-7.761758e-05) * (q[i3] * q[i10]) + (-9.431192e-04) * (q[i3] * q[i12]) + (-2.399187e-03) * (q[i3] * q[i15]) + (9.598698e-04) * (q[i3] * q[i16])
            + (3.549022e-04) * (q[i3] * q[i19]) + (-6.758842e-05) * (q[i3] * q[i20]) + (-8.200449e-04) * (q[i3] * q[i21]) + (-1.065079e-03) * (q[i3] * q[i22])
            + (-3.199444e-04) * (q[i4] * q[i5]) + (-7.801497e-04) * (q[i4] * q[i6]) + (-1.659809e-04) * (q[i4] * q[i7]) + (-1.784835e-03) * (q[i4] * q[i8])
            + (-2.538559e-04) * (q[i4] * q[i9]) + (-2.539604e-04) * (q[i4] * q[i10]) + (-9.341547e-04) * (q[i4] * q[i12]) + (4.198854e-04) * (q[i4] * q[i15])
            + (-3.551721e-04) * (q[i4] * q[i16]) + (-8.331422e-04) * (q[i4] * q[i19]) + (5.438664e-04) * (q[i4] * q[i20]) + (-2.493820e-04) * (q[i4] * q[i21])
            + (8.925204e-04) * (q[i4] * q[i22]) + (1.185197e-04) * (q[i5] * q[i6]) + (1.385137e-03) * (q[i5] * q[i7]) + (1.744742e-03) * (q[i5] * q[i8])
            + (-3.088273e-04) * (q[i5] * q[i9]) + (1.690130e-04) * (q[i5] * q[i10]) + (1.094104e-03) * (q[i5] * q[i12]) + (2.062101e-03) * (q[i5] * q[i15])
            + (-6.159860e-04) * (q[i5] * q[i16]) + (6.998644e-04) * (q[i5] * q[i19]) + (-4.212765e-04) * (q[i5] * q[i20]) + (-2.158314e-04) * (q[i5] * q[i21])
            + (-3.102334e-04) * (q[i5] * q[i22]) + (9.749898e-04) * (q[i6] * q[i7]) + (3.118483e-04) * (q[i6] * q[i8]) + (-8.023481e-04) * (q[i6] * q[i9])
            + (1.632620e-04) * (q[i6] * q[i10]) + (-9.789896e-05) * (q[i6] * q[i12]) + (1.189101e-03) * (q[i6] * q[i15]) + (1.504720e-04) * (q[i6] * q[i16])
            + (2.453448e-05) * (q[i6] * q[i19]) + (1.247018e-03) * (q[i6] * q[i20]) + (-3.542402e-04) * (q[i6] * q[i21]) + (-3.928276e-04) * (q[i6] * q[i22])
            + (3.848970e-04) * (q[i7] * q[i8]) + (2.032087e-04) * (q[i7] * q[i9]) + (-6.176399e-05) * (q[i7] * q[i10]) + (1.092357e-04) * (q[i7] * q[i12])
            + (3.001006e-04) * (q[i7] * q[i15]) + (1.036485e-03) * (q[i7] * q[i16]) + (-1.439265e-04) * (q[i7] * q[i19]) + (1.173055e-04) * (q[i7] * q[i20])
            + (5.647104e-04) * (q[i7] * q[i21]) + (-8.827581e-05) * (q[i7] * q[i22]) + (3.713144e-04) * (q[i8] * q[i9]) + (1.871301e-03) * (q[i8] * q[i10])
            + (-1.148201e-05) * (q[i8] * q[i12]) + (3.140654e-03) * (q[i8] * q[i15]) + (9.446172e-04) * (q[i8] * q[i16]) + (1.077931e-03) * (q[i8] * q[i19])
            + (-1.831442e-04) * (q[i8] * q[i20]) + (1.827731e-03) * (q[i8] * q[i21]) + (-3.900571e-05) * (q[i8] * q[i22]) + (-1.215391e-04) * (q[i9] * q[i10])
            + (-2.414728e-04) * (q[i9] * q[i12]) + (1.564612e-04) * (q[i9] * q[i15]) + (1.517742e-04) * (q[i9] * q[i16]) + (4.364411e-05) * (q[i9] * q[i19])
            + (2.088469e-04) * (q[i9] * q[i20]) + (2.569153e-05) * (q[i9] * q[i21]) + (-1.682093e-04) * (q[i9] * q[i22]) + (2.459750e-04) * (q[i10] * q[i12])
            + (6.393224e-05) * (q[i10] * q[i15]) + (-1.507483e-04) * (q[i10] * q[i16]) + (-4.681708e-05) * (q[i10] * q[i19])
            + (-8.202952e-05) * (q[i10] * q[i20]) + (-9.864996e-05) * (q[i10] * q[i21]) + (-2.166452e-04) * (q[i10] * q[i22])
            + (3.146763e-04) * (q[i12] * q[i15]) + (-3.128072e-04) * (q[i12] * q[i16]) + (3.545202e-04) * (q[i12] * q[i19])
            + (-3.485009e-04) * (q[i12] * q[i20]) + (-4.512187e-04) * (q[i12] * q[i21]) + (-4.540651e-04) * (q[i12] * q[i22])
            + (-1.783143e-04) * (q[i15] * q[i16]) + (-4.017164e-04) * (q[i15] * q[i19]) + (-2.733872e-05) * (q[i15] * q[i20])
            + (-4.815746e-04) * (q[i15] * q[i21]) + (-1.275106e-04) * (q[i15] * q[i22]) + (5.782590e-05) * (q[i16] * q[i19])
            + (1.390104e-04) * (q[i16] * q[i20]) + (-6.533038e-04) * (q[i16] * q[i21]) + (-3.496984e-04) * (q[i16] * q[i22])
            + (-2.405631e-04) * (q[i19] * q[i20]) + (-2.057011e-03) * (q[i19] * q[i21]) + (6.084827e-05) * (q[i19] * q[i22])
            + (1.702902e-04) * (q[i20] * q[i21]) + (-2.498516e-04) * (q[i20] * q[i22]) + (8.413113e-05) * (q[i21] * q[i22]);
   }

   public void getJQx12(double[] q, double[][] JQ)
   {
      JQ[1][i12] = (3.009467e-03) * (1) + (-5.326149e-03) * ((2) * q[i12]) + (-2.645071e-04) * (q[i0]) + (-2.437312e-03) * (q[i1]) + (-5.454492e-03) * (q[i2])
            + (2.443215e-03) * (q[i3]) + (9.079738e-04) * (q[i4]) + (-5.465597e-03) * (q[i5]) + (4.725349e-03) * (q[i6]) + (3.765473e-03) * (q[i7])
            + (1.046921e-02) * (q[i8]) + (2.158386e-03) * (q[i9]) + (1.103518e-03) * (q[i10]) + (1.641208e-05) * (q[i11]) + (-3.818853e-05) * (q[i15])
            + (1.944719e-03) * (q[i16]) + (1.699568e-03) * (q[i19]) + (-1.523914e-03) * (q[i20]) + (2.662284e-04) * (q[i21]) + (-4.255399e-04) * (q[i22])
            + (4.998808e-06) * (q[i0] * q[i0]) + (7.082192e-04) * (q[i1] * q[i1]) + (6.892704e-04) * (q[i2] * q[i2]) + (-2.855950e-03) * (q[i3] * q[i3])
            + (-1.394569e-03) * (q[i4] * q[i4]) + (2.853626e-04) * (q[i5] * q[i5]) + (-6.927001e-04) * (q[i6] * q[i6]) + (5.123069e-04) * (q[i7] * q[i7])
            + (-1.816467e-03) * (q[i8] * q[i8]) + (-6.388348e-04) * (q[i9] * q[i9]) + (-7.248281e-05) * (q[i10] * q[i10]) + (2.780140e-04) * (q[i11] * q[i11])
            + (2.812292e-04) * ((2) * q[i0] * q[i12]) + (2.557378e-04) * ((2) * q[i1] * q[i12]) + (-1.504999e-04) * ((2) * q[i2] * q[i12])
            + (-9.821072e-04) * ((2) * q[i3] * q[i12]) + (-3.937433e-04) * ((2) * q[i4] * q[i12]) + (-1.331503e-03) * ((2) * q[i5] * q[i12])
            + (4.222588e-05) * ((2) * q[i6] * q[i12]) + (6.581599e-05) * ((2) * q[i7] * q[i12]) + (-3.407250e-04) * ((2) * q[i8] * q[i12])
            + (-5.664863e-04) * ((2) * q[i9] * q[i12]) + (2.103634e-04) * ((2) * q[i10] * q[i12]) + (2.928184e-04) * ((2) * q[i11] * q[i12])
            + (-1.318355e-04) * ((3) * q[i12] * q[i12]) + (-1.176538e-04) * ((2) * q[i12] * q[i15]) + (-1.806532e-03) * ((2) * q[i12] * q[i16])
            + (1.260357e-04) * ((2) * q[i12] * q[i19]) + (-4.017068e-04) * ((2) * q[i12] * q[i20]) + (5.719601e-05) * ((2) * q[i12] * q[i21])
            + (8.551246e-04) * ((2) * q[i12] * q[i22]) + (1.280547e-04) * (q[i15] * q[i15]) + (2.050267e-03) * (q[i16] * q[i16])
            + (-3.760187e-05) * (q[i19] * q[i19]) + (-1.051264e-04) * (q[i20] * q[i20]) + (1.767875e-04) * (q[i21] * q[i21])
            + (4.884731e-04) * (q[i22] * q[i22]) + (-8.088057e-04) * (q[i0] * q[i1]) + (1.396673e-03) * (q[i0] * q[i2]) + (-1.685471e-03) * (q[i0] * q[i3])
            + (-1.804400e-03) * (q[i0] * q[i4]) + (2.869565e-04) * (q[i0] * q[i5]) + (3.205563e-04) * (q[i0] * q[i6]) + (-1.669194e-03) * (q[i0] * q[i7])
            + (1.649874e-04) * (q[i0] * q[i8]) + (-5.578639e-04) * (q[i0] * q[i9]) + (-5.991964e-05) * (q[i0] * q[i10]) + (1.037433e-03) * (q[i0] * q[i11])
            + (-3.508971e-04) * (q[i0] * q[i15]) + (-3.126500e-04) * (q[i0] * q[i16]) + (1.308050e-04) * (q[i0] * q[i19]) + (-5.335765e-04) * (q[i0] * q[i20])
            + (4.240182e-04) * (q[i0] * q[i21]) + (-7.301080e-04) * (q[i0] * q[i22]) + (6.646431e-04) * (q[i1] * q[i2]) + (1.062638e-03) * (q[i1] * q[i3])
            + (6.743792e-05) * (q[i1] * q[i4]) + (-6.398775e-05) * (q[i1] * q[i5]) + (9.281863e-04) * (q[i1] * q[i6]) + (1.519737e-03) * (q[i1] * q[i7])
            + (6.184059e-04) * (q[i1] * q[i8]) + (9.128566e-04) * (q[i1] * q[i9]) + (-1.430565e-05) * (q[i1] * q[i10]) + (1.050391e-03) * (q[i1] * q[i11])
            + (5.891167e-05) * (q[i1] * q[i15]) + (-1.117033e-03) * (q[i1] * q[i16]) + (2.595780e-04) * (q[i1] * q[i19]) + (-6.142505e-04) * (q[i1] * q[i20])
            + (3.496778e-04) * (q[i1] * q[i21]) + (7.630297e-04) * (q[i1] * q[i22]) + (-4.700165e-04) * (q[i2] * q[i3]) + (5.365713e-04) * (q[i2] * q[i4])
            + (1.689365e-03) * (q[i2] * q[i5]) + (-3.079661e-04) * (q[i2] * q[i6]) + (7.383186e-04) * (q[i2] * q[i7]) + (1.411772e-03) * (q[i2] * q[i8])
            + (1.445852e-04) * (q[i2] * q[i9]) + (4.314493e-04) * (q[i2] * q[i10]) + (1.565082e-03) * (q[i2] * q[i11]) + (6.583024e-04) * (q[i2] * q[i15])
            + (-3.614837e-03) * (q[i2] * q[i16]) + (-6.191380e-05) * (q[i2] * q[i19]) + (-1.678354e-03) * (q[i2] * q[i20]) + (-1.404150e-04) * (q[i2] * q[i21])
            + (8.041627e-05) * (q[i2] * q[i22]) + (1.988030e-03) * (q[i3] * q[i4]) + (-3.053467e-04) * (q[i3] * q[i5]) + (1.625795e-04) * (q[i3] * q[i6])
            + (7.903289e-04) * (q[i3] * q[i7]) + (1.817324e-03) * (q[i3] * q[i8]) + (2.452206e-04) * (q[i3] * q[i9]) + (2.547123e-04) * (q[i3] * q[i10])
            + (-9.431192e-04) * (q[i3] * q[i11]) + (3.537319e-04) * (q[i3] * q[i15]) + (-4.212583e-04) * (q[i3] * q[i16]) + (-5.466593e-04) * (q[i3] * q[i19])
            + (8.499335e-04) * (q[i3] * q[i20]) + (8.870416e-04) * (q[i3] * q[i21]) + (-2.562754e-04) * (q[i3] * q[i22]) + (-8.964191e-04) * (q[i4] * q[i5])
            + (8.819036e-04) * (q[i4] * q[i6]) + (-1.034888e-04) * (q[i4] * q[i7]) + (8.717079e-04) * (q[i4] * q[i8]) + (7.054069e-05) * (q[i4] * q[i9])
            + (5.829149e-04) * (q[i4] * q[i10]) + (-9.341547e-04) * (q[i4] * q[i11]) + (-9.618192e-04) * (q[i4] * q[i15]) + (2.398704e-03) * (q[i4] * q[i16])
            + (5.877753e-05) * (q[i4] * q[i19]) + (-3.604102e-04) * (q[i4] * q[i20]) + (-1.056132e-03) * (q[i4] * q[i21]) + (-8.202490e-04) * (q[i4] * q[i22])
            + (-1.385588e-03) * (q[i5] * q[i6]) + (-1.108449e-04) * (q[i5] * q[i7]) + (-1.778833e-03) * (q[i5] * q[i8]) + (-1.763527e-04) * (q[i5] * q[i9])
            + (3.018424e-04) * (q[i5] * q[i10]) + (1.094104e-03) * (q[i5] * q[i11]) + (6.111405e-04) * (q[i5] * q[i15]) + (-2.064967e-03) * (q[i5] * q[i16])
            + (4.324835e-04) * (q[i5] * q[i19]) + (-7.160581e-04) * (q[i5] * q[i20]) + (-3.110414e-04) * (q[i5] * q[i21]) + (-2.057889e-04) * (q[i5] * q[i22])
            + (9.681896e-04) * (q[i6] * q[i7]) + (3.574590e-04) * (q[i6] * q[i8]) + (-6.623801e-05) * (q[i6] * q[i9]) + (1.932459e-04) * (q[i6] * q[i10])
            + (-9.789896e-05) * (q[i6] * q[i11]) + (1.037932e-03) * (q[i6] * q[i15]) + (3.039606e-04) * (q[i6] * q[i16]) + (1.155972e-04) * (q[i6] * q[i19])
            + (-1.390614e-04) * (q[i6] * q[i20]) + (9.065239e-05) * (q[i6] * q[i21]) + (-5.720250e-04) * (q[i6] * q[i22]) + (3.219046e-04) * (q[i7] * q[i8])
            + (1.531247e-04) * (q[i7] * q[i9]) + (-7.972379e-04) * (q[i7] * q[i10]) + (1.092357e-04) * (q[i7] * q[i11]) + (1.521089e-04) * (q[i7] * q[i15])
            + (1.174695e-03) * (q[i7] * q[i16]) + (1.244186e-03) * (q[i7] * q[i19]) + (2.493281e-05) * (q[i7] * q[i20]) + (3.914591e-04) * (q[i7] * q[i21])
            + (3.562947e-04) * (q[i7] * q[i22]) + (1.890128e-03) * (q[i8] * q[i9]) + (3.728237e-04) * (q[i8] * q[i10]) + (-1.148201e-05) * (q[i8] * q[i11])
            + (9.501223e-04) * (q[i8] * q[i15]) + (3.159565e-03) * (q[i8] * q[i16]) + (-1.796733e-04) * (q[i8] * q[i19]) + (1.059739e-03) * (q[i8] * q[i20])
            + (4.025228e-05) * (q[i8] * q[i21]) + (-1.843622e-03) * (q[i8] * q[i22]) + (-1.253624e-04) * (q[i9] * q[i10]) + (-2.414728e-04) * (q[i9] * q[i11])
            + (-1.483531e-04) * (q[i9] * q[i15]) + (6.375883e-05) * (q[i9] * q[i16]) + (-8.314378e-05) * (q[i9] * q[i19]) + (-5.902861e-05) * (q[i9] * q[i20])
            + (2.179966e-04) * (q[i9] * q[i21]) + (9.555884e-05) * (q[i9] * q[i22]) + (2.459750e-04) * (q[i10] * q[i11]) + (1.453329e-04) * (q[i10] * q[i15])
            + (1.589430e-04) * (q[i10] * q[i16]) + (2.078339e-04) * (q[i10] * q[i19]) + (4.660277e-05) * (q[i10] * q[i20]) + (1.714616e-04) * (q[i10] * q[i21])
            + (-2.717697e-05) * (q[i10] * q[i22]) + (3.146763e-04) * (q[i11] * q[i15]) + (-3.128072e-04) * (q[i11] * q[i16])
            + (3.545202e-04) * (q[i11] * q[i19]) + (-3.485009e-04) * (q[i11] * q[i20]) + (-4.512187e-04) * (q[i11] * q[i21])
            + (-4.540651e-04) * (q[i11] * q[i22]) + (-1.691708e-04) * (q[i15] * q[i16]) + (1.354376e-04) * (q[i15] * q[i19])
            + (6.039360e-05) * (q[i15] * q[i20]) + (3.469436e-04) * (q[i15] * q[i21]) + (6.541224e-04) * (q[i15] * q[i22]) + (-2.004387e-05) * (q[i16] * q[i19])
            + (-4.167561e-04) * (q[i16] * q[i20]) + (1.295322e-04) * (q[i16] * q[i21]) + (4.850914e-04) * (q[i16] * q[i22])
            + (-2.413345e-04) * (q[i19] * q[i20]) + (2.520685e-04) * (q[i19] * q[i21]) + (-1.723724e-04) * (q[i19] * q[i22])
            + (-6.266256e-05) * (q[i20] * q[i21]) + (2.050566e-03) * (q[i20] * q[i22]) + (8.656308e-05) * (q[i21] * q[i22]);
   }

   public void getJQx15(double[] q, double[][] JQ)
   {
      JQ[1][i15] = (-1.968976e-03) * (1) + (6.490308e-03) * ((2) * q[i15]) + (-3.495047e-04) * (q[i0]) + (2.488392e-03) * (q[i1]) + (2.469027e-03) * (q[i2])
            + (5.579206e-03) * (q[i3]) + (5.425509e-03) * (q[i4]) + (-1.788624e-02) * (q[i5]) + (-9.283013e-04) * (q[i6]) + (7.197612e-04) * (q[i7])
            + (9.172387e-03) * (q[i8]) + (-1.509882e-03) * (q[i9]) + (-3.305814e-04) * (q[i10]) + (1.869733e-03) * (q[i11]) + (-3.818853e-05) * (q[i12])
            + (1.381008e-06) * (q[i16]) + (1.443820e-03) * (q[i19]) + (-1.050440e-04) * (q[i20]) + (2.450143e-03) * (q[i21]) + (2.765729e-04) * (q[i22])
            + (-1.196572e-03) * (q[i0] * q[i0]) + (2.994997e-04) * (q[i1] * q[i1]) + (-2.546112e-03) * (q[i2] * q[i2]) + (7.552389e-04) * (q[i3] * q[i3])
            + (-3.144852e-04) * (q[i4] * q[i4]) + (1.071036e-04) * (q[i5] * q[i5]) + (2.228535e-04) * (q[i6] * q[i6]) + (-1.107298e-03) * (q[i7] * q[i7])
            + (4.086490e-03) * (q[i8] * q[i8]) + (2.176835e-04) * (q[i9] * q[i9]) + (-1.262826e-04) * (q[i10] * q[i10]) + (1.788992e-03) * (q[i11] * q[i11])
            + (-1.176538e-04) * (q[i12] * q[i12]) + (-9.181986e-05) * ((2) * q[i0] * q[i15]) + (7.592240e-05) * ((2) * q[i1] * q[i15])
            + (1.748959e-03) * ((2) * q[i2] * q[i15]) + (6.068730e-04) * ((2) * q[i3] * q[i15]) + (7.297165e-04) * ((2) * q[i4] * q[i15])
            + (-3.172480e-03) * ((2) * q[i5] * q[i15]) + (8.926814e-05) * ((2) * q[i6] * q[i15]) + (-5.139503e-05) * ((2) * q[i7] * q[i15])
            + (2.670214e-03) * ((2) * q[i8] * q[i15]) + (-7.293982e-05) * ((2) * q[i9] * q[i15]) + (-2.545667e-05) * ((2) * q[i10] * q[i15])
            + (2.021889e-03) * ((2) * q[i11] * q[i15]) + (1.280547e-04) * ((2) * q[i12] * q[i15]) + (7.975157e-04) * ((3) * q[i15] * q[i15])
            + (1.442044e-04) * ((2) * q[i15] * q[i16]) + (2.168470e-04) * ((2) * q[i15] * q[i19]) + (1.325622e-04) * ((2) * q[i15] * q[i20])
            + (3.968051e-04) * ((2) * q[i15] * q[i21]) + (-1.268897e-05) * ((2) * q[i15] * q[i22]) + (-1.452361e-04) * (q[i16] * q[i16])
            + (3.503035e-04) * (q[i19] * q[i19]) + (-1.661363e-04) * (q[i20] * q[i20]) + (7.926800e-04) * (q[i21] * q[i21]) + (1.745091e-05) * (q[i22] * q[i22])
            + (1.151046e-03) * (q[i0] * q[i1]) + (-6.568085e-05) * (q[i0] * q[i2]) + (2.231693e-03) * (q[i0] * q[i3]) + (-5.569488e-04) * (q[i0] * q[i4])
            + (7.623461e-04) * (q[i0] * q[i5]) + (3.511050e-04) * (q[i0] * q[i6]) + (-6.133359e-04) * (q[i0] * q[i7]) + (9.418310e-04) * (q[i0] * q[i8])
            + (7.242263e-04) * (q[i0] * q[i9]) + (-8.095689e-05) * (q[i0] * q[i10]) + (1.111723e-03) * (q[i0] * q[i11]) + (-3.508971e-04) * (q[i0] * q[i12])
            + (5.148387e-04) * (q[i0] * q[i16]) + (-1.717268e-04) * (q[i0] * q[i19]) + (4.410306e-04) * (q[i0] * q[i20]) + (2.398073e-04) * (q[i0] * q[i21])
            + (5.474682e-04) * (q[i0] * q[i22]) + (-1.531927e-03) * (q[i1] * q[i2]) + (3.459426e-04) * (q[i1] * q[i3]) + (-8.506545e-04) * (q[i1] * q[i4])
            + (1.632663e-03) * (q[i1] * q[i5]) + (7.933511e-04) * (q[i1] * q[i6]) + (2.038911e-03) * (q[i1] * q[i7]) + (2.099563e-04) * (q[i1] * q[i8])
            + (1.494251e-04) * (q[i1] * q[i9]) + (1.871632e-04) * (q[i1] * q[i10]) + (2.987142e-04) * (q[i1] * q[i11]) + (5.891167e-05) * (q[i1] * q[i12])
            + (5.088894e-04) * (q[i1] * q[i16]) + (8.008115e-04) * (q[i1] * q[i19]) + (-1.042180e-03) * (q[i1] * q[i20]) + (2.924900e-05) * (q[i1] * q[i21])
            + (2.277305e-04) * (q[i1] * q[i22]) + (1.418867e-03) * (q[i2] * q[i3]) + (-9.815592e-05) * (q[i2] * q[i4]) + (-1.444219e-03) * (q[i2] * q[i5])
            + (5.709624e-04) * (q[i2] * q[i6]) + (2.397672e-04) * (q[i2] * q[i7]) + (4.348435e-03) * (q[i2] * q[i8]) + (6.495961e-04) * (q[i2] * q[i9])
            + (-4.261787e-04) * (q[i2] * q[i10]) + (3.600388e-03) * (q[i2] * q[i11]) + (6.583024e-04) * (q[i2] * q[i12]) + (4.283540e-05) * (q[i2] * q[i16])
            + (-9.625853e-04) * (q[i2] * q[i19]) + (-8.625585e-04) * (q[i2] * q[i20]) + (3.715050e-04) * (q[i2] * q[i21]) + (3.675351e-04) * (q[i2] * q[i22])
            + (-6.454528e-05) * (q[i3] * q[i4]) + (3.455274e-04) * (q[i3] * q[i5]) + (1.331850e-03) * (q[i3] * q[i6]) + (-7.167150e-04) * (q[i3] * q[i7])
            + (4.639511e-04) * (q[i3] * q[i8]) + (-2.710467e-04) * (q[i3] * q[i9]) + (-6.593369e-04) * (q[i3] * q[i10]) + (-2.399187e-03) * (q[i3] * q[i11])
            + (3.537319e-04) * (q[i3] * q[i12]) + (6.317741e-04) * (q[i3] * q[i16]) + (-4.963041e-04) * (q[i3] * q[i19]) + (-5.463849e-04) * (q[i3] * q[i20])
            + (-1.765358e-04) * (q[i3] * q[i21]) + (2.002703e-04) * (q[i3] * q[i22]) + (-9.330154e-04) * (q[i4] * q[i5]) + (-1.836521e-03) * (q[i4] * q[i6])
            + (-1.031517e-03) * (q[i4] * q[i7]) + (-2.745696e-04) * (q[i4] * q[i8]) + (-6.173504e-05) * (q[i4] * q[i9]) + (-4.795333e-04) * (q[i4] * q[i10])
            + (4.198854e-04) * (q[i4] * q[i11]) + (-9.618192e-04) * (q[i4] * q[i12]) + (6.267090e-04) * (q[i4] * q[i16]) + (4.461442e-04) * (q[i4] * q[i19])
            + (7.146477e-04) * (q[i4] * q[i20]) + (5.753389e-04) * (q[i4] * q[i21]) + (1.739075e-05) * (q[i4] * q[i22]) + (-2.008343e-04) * (q[i5] * q[i6])
            + (3.823836e-04) * (q[i5] * q[i7]) + (1.608008e-03) * (q[i5] * q[i8]) + (-1.271194e-05) * (q[i5] * q[i9]) + (1.603742e-06) * (q[i5] * q[i10])
            + (2.062101e-03) * (q[i5] * q[i11]) + (6.111405e-04) * (q[i5] * q[i12]) + (-1.431438e-04) * (q[i5] * q[i16]) + (-7.355210e-04) * (q[i5] * q[i19])
            + (-3.359705e-04) * (q[i5] * q[i20]) + (-1.399082e-03) * (q[i5] * q[i21]) + (1.192572e-04) * (q[i5] * q[i22]) + (-1.479991e-04) * (q[i6] * q[i7])
            + (1.204213e-05) * (q[i6] * q[i8]) + (-5.787308e-05) * (q[i6] * q[i9]) + (5.898438e-05) * (q[i6] * q[i10]) + (1.189101e-03) * (q[i6] * q[i11])
            + (1.037932e-03) * (q[i6] * q[i12]) + (9.620974e-05) * (q[i6] * q[i16]) + (2.382661e-04) * (q[i6] * q[i19]) + (-2.391586e-04) * (q[i6] * q[i20])
            + (-5.152226e-05) * (q[i6] * q[i21]) + (2.113318e-04) * (q[i6] * q[i22]) + (-8.874344e-04) * (q[i7] * q[i8]) + (3.024924e-05) * (q[i7] * q[i9])
            + (-5.512347e-04) * (q[i7] * q[i10]) + (3.001006e-04) * (q[i7] * q[i11]) + (1.521089e-04) * (q[i7] * q[i12]) + (-9.605176e-05) * (q[i7] * q[i16])
            + (-1.144394e-05) * (q[i7] * q[i19]) + (4.214893e-04) * (q[i7] * q[i20]) + (1.458426e-04) * (q[i7] * q[i21]) + (1.105229e-04) * (q[i7] * q[i22])
            + (-6.984873e-04) * (q[i8] * q[i9]) + (-5.211538e-04) * (q[i8] * q[i10]) + (3.140654e-03) * (q[i8] * q[i11]) + (9.501223e-04) * (q[i8] * q[i12])
            + (5.080928e-06) * (q[i8] * q[i16]) + (4.603370e-04) * (q[i8] * q[i19]) + (2.050454e-04) * (q[i8] * q[i20]) + (-1.066225e-04) * (q[i8] * q[i21])
            + (-4.518506e-04) * (q[i8] * q[i22]) + (3.373777e-04) * (q[i9] * q[i10]) + (1.564612e-04) * (q[i9] * q[i11]) + (-1.483531e-04) * (q[i9] * q[i12])
            + (-2.402656e-04) * (q[i9] * q[i16]) + (1.483290e-04) * (q[i9] * q[i19]) + (1.634168e-04) * (q[i9] * q[i20]) + (6.501105e-05) * (q[i9] * q[i21])
            + (-3.990033e-05) * (q[i9] * q[i22]) + (6.393224e-05) * (q[i10] * q[i11]) + (1.453329e-04) * (q[i10] * q[i12]) + (2.381822e-04) * (q[i10] * q[i16])
            + (-1.906830e-04) * (q[i10] * q[i19]) + (2.572036e-04) * (q[i10] * q[i20]) + (-9.423758e-05) * (q[i10] * q[i21])
            + (4.601349e-05) * (q[i10] * q[i22]) + (3.146763e-04) * (q[i11] * q[i12]) + (-1.783143e-04) * (q[i11] * q[i16])
            + (-4.017164e-04) * (q[i11] * q[i19]) + (-2.733872e-05) * (q[i11] * q[i20]) + (-4.815746e-04) * (q[i11] * q[i21])
            + (-1.275106e-04) * (q[i11] * q[i22]) + (-1.691708e-04) * (q[i12] * q[i16]) + (1.354376e-04) * (q[i12] * q[i19])
            + (6.039360e-05) * (q[i12] * q[i20]) + (3.469436e-04) * (q[i12] * q[i21]) + (6.541224e-04) * (q[i12] * q[i22]) + (2.134674e-04) * (q[i16] * q[i19])
            + (-2.091147e-04) * (q[i16] * q[i20]) + (1.579448e-04) * (q[i16] * q[i21]) + (1.535920e-04) * (q[i16] * q[i22]) + (1.092114e-05) * (q[i19] * q[i20])
            + (5.399830e-05) * (q[i19] * q[i21]) + (-8.669724e-05) * (q[i19] * q[i22]) + (1.848389e-06) * (q[i20] * q[i21])
            + (-2.399921e-05) * (q[i20] * q[i22]) + (-6.705985e-05) * (q[i21] * q[i22]);
   }

   public void getJQx16(double[] q, double[][] JQ)
   {
      JQ[1][i16] = (1.890989e-03) * (1) + (-6.544746e-03) * ((2) * q[i16]) + (2.440634e-03) * (q[i0]) + (-3.292644e-04) * (q[i1]) + (2.522821e-03) * (q[i2])
            + (5.510126e-03) * (q[i3]) + (5.592667e-03) * (q[i4]) + (-1.806450e-02) * (q[i5]) + (-7.215826e-04) * (q[i6]) + (9.171318e-04) * (q[i7])
            + (-9.271249e-03) * (q[i8]) + (3.377185e-04) * (q[i9]) + (1.497577e-03) * (q[i10]) + (-5.634301e-05) * (q[i11]) + (1.944719e-03) * (q[i12])
            + (1.381008e-06) * (q[i15]) + (9.614661e-05) * (q[i19]) + (-1.418129e-03) * (q[i20]) + (2.959218e-04) * (q[i21]) + (2.443170e-03) * (q[i22])
            + (-2.972802e-04) * (q[i0] * q[i0]) + (1.192890e-03) * (q[i1] * q[i1]) + (2.587146e-03) * (q[i2] * q[i2]) + (3.199199e-04) * (q[i3] * q[i3])
            + (-7.444036e-04) * (q[i4] * q[i4]) + (-9.213145e-05) * (q[i5] * q[i5]) + (1.106866e-03) * (q[i6] * q[i6]) + (-2.299380e-04) * (q[i7] * q[i7])
            + (-4.139522e-03) * (q[i8] * q[i8]) + (1.270710e-04) * (q[i9] * q[i9]) + (-2.161482e-04) * (q[i10] * q[i10]) + (1.087262e-04) * (q[i11] * q[i11])
            + (-1.806532e-03) * (q[i12] * q[i12]) + (1.442044e-04) * (q[i15] * q[i15]) + (6.728841e-05) * ((2) * q[i0] * q[i16])
            + (-9.215216e-05) * ((2) * q[i1] * q[i16]) + (1.766318e-03) * ((2) * q[i2] * q[i16]) + (7.318281e-04) * ((2) * q[i3] * q[i16])
            + (6.037906e-04) * ((2) * q[i4] * q[i16]) + (-3.193741e-03) * ((2) * q[i5] * q[i16]) + (5.766638e-05) * ((2) * q[i6] * q[i16])
            + (-8.659144e-05) * ((2) * q[i7] * q[i16]) + (-2.682642e-03) * ((2) * q[i8] * q[i16]) + (2.559521e-05) * ((2) * q[i9] * q[i16])
            + (7.266301e-05) * ((2) * q[i10] * q[i16]) + (1.245777e-04) * ((2) * q[i11] * q[i16]) + (2.050267e-03) * ((2) * q[i12] * q[i16])
            + (-1.452361e-04) * ((2) * q[i15] * q[i16]) + (-7.998110e-04) * ((3) * q[i16] * q[i16]) + (-1.343704e-04) * ((2) * q[i16] * q[i19])
            + (-2.153955e-04) * ((2) * q[i16] * q[i20]) + (-1.105465e-05) * ((2) * q[i16] * q[i21]) + (4.009891e-04) * ((2) * q[i16] * q[i22])
            + (1.705391e-04) * (q[i19] * q[i19]) + (-3.447732e-04) * (q[i20] * q[i20]) + (-1.365182e-05) * (q[i21] * q[i21])
            + (-7.943027e-04) * (q[i22] * q[i22]) + (-1.169626e-03) * (q[i0] * q[i1]) + (1.546440e-03) * (q[i0] * q[i2]) + (8.670147e-04) * (q[i0] * q[i3])
            + (-3.701069e-04) * (q[i0] * q[i4]) + (-1.635196e-03) * (q[i0] * q[i5]) + (2.069774e-03) * (q[i0] * q[i6]) + (7.881685e-04) * (q[i0] * q[i7])
            + (2.296953e-04) * (q[i0] * q[i8]) + (2.030050e-04) * (q[i0] * q[i9]) + (1.579682e-04) * (q[i0] * q[i10]) + (-6.870321e-05) * (q[i0] * q[i11])
            + (-3.126500e-04) * (q[i0] * q[i12]) + (5.148387e-04) * (q[i0] * q[i15]) + (-1.048530e-03) * (q[i0] * q[i19]) + (8.003445e-04) * (q[i0] * q[i20])
            + (-2.354564e-04) * (q[i0] * q[i21]) + (-2.929430e-05) * (q[i0] * q[i22]) + (6.903362e-05) * (q[i1] * q[i2]) + (5.483259e-04) * (q[i1] * q[i3])
            + (-2.229472e-03) * (q[i1] * q[i4]) + (-7.627753e-04) * (q[i1] * q[i5]) + (-6.163040e-04) * (q[i1] * q[i6]) + (3.758379e-04) * (q[i1] * q[i7])
            + (9.333350e-04) * (q[i1] * q[i8]) + (-7.386152e-05) * (q[i1] * q[i9]) + (7.150911e-04) * (q[i1] * q[i10]) + (3.494695e-04) * (q[i1] * q[i11])
            + (-1.117033e-03) * (q[i1] * q[i12]) + (5.088894e-04) * (q[i1] * q[i15]) + (4.527292e-04) * (q[i1] * q[i19]) + (-1.634519e-04) * (q[i1] * q[i20])
            + (-5.340071e-04) * (q[i1] * q[i21]) + (-2.454685e-04) * (q[i1] * q[i22]) + (1.015846e-04) * (q[i2] * q[i3]) + (-1.455408e-03) * (q[i2] * q[i4])
            + (1.441259e-03) * (q[i2] * q[i5]) + (2.251452e-04) * (q[i2] * q[i6]) + (5.739001e-04) * (q[i2] * q[i7]) + (4.428257e-03) * (q[i2] * q[i8])
            + (-4.232450e-04) * (q[i2] * q[i9]) + (6.458251e-04) * (q[i2] * q[i10]) + (-6.828608e-04) * (q[i2] * q[i11]) + (-3.614837e-03) * (q[i2] * q[i12])
            + (4.283540e-05) * (q[i2] * q[i15]) + (-8.608036e-04) * (q[i2] * q[i19]) + (-9.489442e-04) * (q[i2] * q[i20]) + (-3.632431e-04) * (q[i2] * q[i21])
            + (-3.792889e-04) * (q[i2] * q[i22]) + (7.420731e-05) * (q[i3] * q[i4]) + (9.330827e-04) * (q[i3] * q[i5]) + (-1.027732e-03) * (q[i3] * q[i6])
            + (-1.838161e-03) * (q[i3] * q[i7]) + (-2.946867e-04) * (q[i3] * q[i8]) + (-4.947617e-04) * (q[i3] * q[i9]) + (-5.319852e-05) * (q[i3] * q[i10])
            + (9.598698e-04) * (q[i3] * q[i11]) + (-4.212583e-04) * (q[i3] * q[i12]) + (6.317741e-04) * (q[i3] * q[i15]) + (7.135509e-04) * (q[i3] * q[i19])
            + (4.429594e-04) * (q[i3] * q[i20]) + (-2.275339e-05) * (q[i3] * q[i21]) + (-5.864281e-04) * (q[i3] * q[i22]) + (-3.794327e-04) * (q[i4] * q[i5])
            + (-7.226764e-04) * (q[i4] * q[i6]) + (1.346195e-03) * (q[i4] * q[i7]) + (4.798022e-04) * (q[i4] * q[i8]) + (-6.497450e-04) * (q[i4] * q[i9])
            + (-2.686769e-04) * (q[i4] * q[i10]) + (-3.551721e-04) * (q[i4] * q[i11]) + (2.398704e-03) * (q[i4] * q[i12]) + (6.267090e-04) * (q[i4] * q[i15])
            + (-5.510075e-04) * (q[i4] * q[i19]) + (-4.883223e-04) * (q[i4] * q[i20]) + (-1.986130e-04) * (q[i4] * q[i21]) + (1.665167e-04) * (q[i4] * q[i22])
            + (3.957857e-04) * (q[i5] * q[i6]) + (-2.080655e-04) * (q[i5] * q[i7]) + (1.625924e-03) * (q[i5] * q[i8]) + (-2.967064e-06) * (q[i5] * q[i9])
            + (-2.413245e-05) * (q[i5] * q[i10]) + (-6.159860e-04) * (q[i5] * q[i11]) + (-2.064967e-03) * (q[i5] * q[i12]) + (-1.431438e-04) * (q[i5] * q[i15])
            + (-3.321778e-04) * (q[i5] * q[i19]) + (-7.404865e-04) * (q[i5] * q[i20]) + (-1.216028e-04) * (q[i5] * q[i21]) + (1.415512e-03) * (q[i5] * q[i22])
            + (1.486869e-04) * (q[i6] * q[i7]) + (8.807517e-04) * (q[i6] * q[i8]) + (5.530778e-04) * (q[i6] * q[i9]) + (-3.333471e-05) * (q[i6] * q[i10])
            + (1.504720e-04) * (q[i6] * q[i11]) + (3.039606e-04) * (q[i6] * q[i12]) + (9.620974e-05) * (q[i6] * q[i15]) + (-4.223975e-04) * (q[i6] * q[i19])
            + (1.109683e-05) * (q[i6] * q[i20]) + (1.089463e-04) * (q[i6] * q[i21]) + (1.466666e-04) * (q[i6] * q[i22]) + (-6.459206e-07) * (q[i7] * q[i8])
            + (-5.499585e-05) * (q[i7] * q[i9]) + (6.889978e-05) * (q[i7] * q[i10]) + (1.036485e-03) * (q[i7] * q[i11]) + (1.174695e-03) * (q[i7] * q[i12])
            + (-9.605176e-05) * (q[i7] * q[i15]) + (2.379539e-04) * (q[i7] * q[i19]) + (-2.380913e-04) * (q[i7] * q[i20]) + (2.063762e-04) * (q[i7] * q[i21])
            + (-5.774850e-05) * (q[i7] * q[i22]) + (5.208003e-04) * (q[i8] * q[i9]) + (6.973763e-04) * (q[i8] * q[i10]) + (9.446172e-04) * (q[i8] * q[i11])
            + (3.159565e-03) * (q[i8] * q[i12]) + (5.080928e-06) * (q[i8] * q[i15]) + (-2.102962e-04) * (q[i8] * q[i19]) + (-4.268456e-04) * (q[i8] * q[i20])
            + (-4.533561e-04) * (q[i8] * q[i21]) + (-8.338577e-05) * (q[i8] * q[i22]) + (-3.369648e-04) * (q[i9] * q[i10]) + (1.517742e-04) * (q[i9] * q[i11])
            + (6.375883e-05) * (q[i9] * q[i12]) + (-2.402656e-04) * (q[i9] * q[i15]) + (-2.577399e-04) * (q[i9] * q[i19]) + (1.908896e-04) * (q[i9] * q[i20])
            + (4.393048e-05) * (q[i9] * q[i21]) + (-1.016466e-04) * (q[i9] * q[i22]) + (-1.507483e-04) * (q[i10] * q[i11]) + (1.589430e-04) * (q[i10] * q[i12])
            + (2.381822e-04) * (q[i10] * q[i15]) + (-1.615727e-04) * (q[i10] * q[i19]) + (-1.470767e-04) * (q[i10] * q[i20])
            + (-4.275113e-05) * (q[i10] * q[i21]) + (6.190540e-05) * (q[i10] * q[i22]) + (-3.128072e-04) * (q[i11] * q[i12])
            + (-1.783143e-04) * (q[i11] * q[i15]) + (5.782590e-05) * (q[i11] * q[i19]) + (1.390104e-04) * (q[i11] * q[i20])
            + (-6.533038e-04) * (q[i11] * q[i21]) + (-3.496984e-04) * (q[i11] * q[i22]) + (-1.691708e-04) * (q[i12] * q[i15])
            + (-2.004387e-05) * (q[i12] * q[i19]) + (-4.167561e-04) * (q[i12] * q[i20]) + (1.295322e-04) * (q[i12] * q[i21])
            + (4.850914e-04) * (q[i12] * q[i22]) + (2.134674e-04) * (q[i15] * q[i19]) + (-2.091147e-04) * (q[i15] * q[i20]) + (1.579448e-04) * (q[i15] * q[i21])
            + (1.535920e-04) * (q[i15] * q[i22]) + (-7.888197e-06) * (q[i19] * q[i20]) + (-2.416558e-05) * (q[i19] * q[i21])
            + (5.949074e-06) * (q[i19] * q[i22]) + (-8.441539e-05) * (q[i20] * q[i21]) + (6.254680e-05) * (q[i20] * q[i22])
            + (6.619962e-05) * (q[i21] * q[i22]);
   }

   public void getJQx19(double[] q, double[][] JQ)
   {
      JQ[1][i19] = (9.177392e-04) * (1) + (7.924415e-04) * ((2) * q[i19]) + (-2.003494e-03) * (q[i0]) + (7.010237e-04) * (q[i1]) + (-2.095669e-03) * (q[i2])
            + (-2.364092e-03) * (q[i3]) + (3.453573e-03) * (q[i4]) + (-5.391021e-04) * (q[i5]) + (2.066508e-03) * (q[i6]) + (-1.185100e-03) * (q[i7])
            + (-4.194115e-04) * (q[i8]) + (1.013386e-04) * (q[i9]) + (-1.040316e-03) * (q[i10]) + (-1.497320e-03) * (q[i11]) + (1.699568e-03) * (q[i12])
            + (1.443820e-03) * (q[i15]) + (9.614661e-05) * (q[i16]) + (6.413053e-06) * (q[i20]) + (-9.128984e-04) * (q[i21]) + (-9.662913e-05) * (q[i22])
            + (-4.202603e-05) * (q[i0] * q[i0]) + (3.554884e-04) * (q[i1] * q[i1]) + (1.521570e-04) * (q[i2] * q[i2]) + (-1.116370e-03) * (q[i3] * q[i3])
            + (2.920938e-04) * (q[i4] * q[i4]) + (4.846585e-04) * (q[i5] * q[i5]) + (-1.366856e-04) * (q[i6] * q[i6]) + (-4.174993e-04) * (q[i7] * q[i7])
            + (-2.545432e-05) * (q[i8] * q[i8]) + (-1.505523e-04) * (q[i9] * q[i9]) + (1.223015e-05) * (q[i10] * q[i10]) + (4.134884e-04) * (q[i11] * q[i11])
            + (1.260357e-04) * (q[i12] * q[i12]) + (2.168470e-04) * (q[i15] * q[i15]) + (-1.343704e-04) * (q[i16] * q[i16])
            + (-1.738111e-05) * ((2) * q[i0] * q[i19]) + (-1.020428e-04) * ((2) * q[i1] * q[i19]) + (-2.030081e-04) * ((2) * q[i2] * q[i19])
            + (1.567991e-04) * ((2) * q[i3] * q[i19]) + (1.412430e-04) * ((2) * q[i4] * q[i19]) + (3.073737e-04) * ((2) * q[i5] * q[i19])
            + (-1.450210e-04) * ((2) * q[i6] * q[i19]) + (3.239674e-04) * ((2) * q[i7] * q[i19]) + (8.407745e-04) * ((2) * q[i8] * q[i19])
            + (-3.021958e-05) * ((2) * q[i9] * q[i19]) + (4.162951e-05) * ((2) * q[i10] * q[i19]) + (-1.188472e-04) * ((2) * q[i11] * q[i19])
            + (-3.760187e-05) * ((2) * q[i12] * q[i19]) + (3.503035e-04) * ((2) * q[i15] * q[i19]) + (1.705391e-04) * ((2) * q[i16] * q[i19])
            + (1.431031e-04) * ((3) * q[i19] * q[i19]) + (-5.105214e-06) * ((2) * q[i19] * q[i20]) + (6.685921e-04) * ((2) * q[i19] * q[i21])
            + (-7.300996e-05) * ((2) * q[i19] * q[i22]) + (4.827284e-06) * (q[i20] * q[i20]) + (-9.116763e-05) * (q[i21] * q[i21])
            + (1.245651e-04) * (q[i22] * q[i22]) + (-5.858271e-04) * (q[i0] * q[i1]) + (4.345169e-04) * (q[i0] * q[i2]) + (1.522527e-03) * (q[i0] * q[i3])
            + (-1.016211e-03) * (q[i0] * q[i4]) + (-8.694627e-04) * (q[i0] * q[i5]) + (1.222475e-03) * (q[i0] * q[i6]) + (-6.159552e-04) * (q[i0] * q[i7])
            + (1.349733e-03) * (q[i0] * q[i8]) + (8.202329e-04) * (q[i0] * q[i9]) + (-3.351315e-04) * (q[i0] * q[i10]) + (6.155529e-04) * (q[i0] * q[i11])
            + (1.308050e-04) * (q[i0] * q[i12]) + (-1.717268e-04) * (q[i0] * q[i15]) + (-1.048530e-03) * (q[i0] * q[i16]) + (4.840219e-04) * (q[i0] * q[i20])
            + (-2.430971e-04) * (q[i0] * q[i21]) + (-5.005325e-04) * (q[i0] * q[i22]) + (-1.356130e-03) * (q[i1] * q[i2]) + (9.862217e-04) * (q[i1] * q[i3])
            + (-1.241991e-03) * (q[i1] * q[i4]) + (-1.608649e-04) * (q[i1] * q[i5]) + (-9.142841e-04) * (q[i1] * q[i6]) + (1.349245e-03) * (q[i1] * q[i7])
            + (1.060160e-03) * (q[i1] * q[i8]) + (-6.724032e-04) * (q[i1] * q[i9]) + (5.671184e-04) * (q[i1] * q[i10]) + (5.263009e-04) * (q[i1] * q[i11])
            + (2.595780e-04) * (q[i1] * q[i12]) + (8.008115e-04) * (q[i1] * q[i15]) + (4.527292e-04) * (q[i1] * q[i16]) + (4.608439e-04) * (q[i1] * q[i20])
            + (-8.342090e-04) * (q[i1] * q[i21]) + (-7.396063e-04) * (q[i1] * q[i22]) + (-9.577654e-04) * (q[i2] * q[i3]) + (-5.021187e-04) * (q[i2] * q[i4])
            + (1.186527e-03) * (q[i2] * q[i5]) + (1.268388e-03) * (q[i2] * q[i6]) + (2.983729e-04) * (q[i2] * q[i7]) + (1.071222e-03) * (q[i2] * q[i8])
            + (1.164111e-04) * (q[i2] * q[i9]) + (3.252194e-04) * (q[i2] * q[i10]) + (1.696311e-03) * (q[i2] * q[i11]) + (-6.191380e-05) * (q[i2] * q[i12])
            + (-9.625853e-04) * (q[i2] * q[i15]) + (-8.608036e-04) * (q[i2] * q[i16]) + (1.081195e-03) * (q[i2] * q[i20]) + (-1.188958e-03) * (q[i2] * q[i21])
            + (-5.315273e-04) * (q[i2] * q[i22]) + (-7.927073e-04) * (q[i3] * q[i4]) + (-8.253939e-04) * (q[i3] * q[i5]) + (-2.949664e-03) * (q[i3] * q[i6])
            + (-1.571947e-04) * (q[i3] * q[i7]) + (-1.082728e-03) * (q[i3] * q[i8]) + (2.575008e-04) * (q[i3] * q[i9]) + (9.528878e-04) * (q[i3] * q[i10])
            + (3.549022e-04) * (q[i3] * q[i11]) + (-5.466593e-04) * (q[i3] * q[i12]) + (-4.963041e-04) * (q[i3] * q[i15]) + (7.135509e-04) * (q[i3] * q[i16])
            + (6.337695e-04) * (q[i3] * q[i20]) + (1.284728e-03) * (q[i3] * q[i21]) + (-3.871210e-04) * (q[i3] * q[i22]) + (1.241729e-03) * (q[i4] * q[i5])
            + (1.863697e-03) * (q[i4] * q[i6]) + (-1.888206e-04) * (q[i4] * q[i7]) + (5.460890e-05) * (q[i4] * q[i8]) + (-1.113940e-04) * (q[i4] * q[i9])
            + (-1.089708e-03) * (q[i4] * q[i10]) + (-8.331422e-04) * (q[i4] * q[i11]) + (5.877753e-05) * (q[i4] * q[i12]) + (4.461442e-04) * (q[i4] * q[i15])
            + (-5.510075e-04) * (q[i4] * q[i16]) + (6.201209e-04) * (q[i4] * q[i20]) + (1.730511e-04) * (q[i4] * q[i21]) + (3.040057e-04) * (q[i4] * q[i22])
            + (2.459765e-04) * (q[i5] * q[i6]) + (6.288068e-04) * (q[i5] * q[i7]) + (-1.509317e-03) * (q[i5] * q[i8]) + (-1.319905e-04) * (q[i5] * q[i9])
            + (6.552438e-05) * (q[i5] * q[i10]) + (6.998644e-04) * (q[i5] * q[i11]) + (4.324835e-04) * (q[i5] * q[i12]) + (-7.355210e-04) * (q[i5] * q[i15])
            + (-3.321778e-04) * (q[i5] * q[i16]) + (-5.472268e-04) * (q[i5] * q[i20]) + (-1.777132e-03) * (q[i5] * q[i21]) + (5.059876e-04) * (q[i5] * q[i22])
            + (3.730544e-04) * (q[i6] * q[i7]) + (-3.218953e-04) * (q[i6] * q[i8]) + (-5.147763e-04) * (q[i6] * q[i9]) + (-3.367955e-06) * (q[i6] * q[i10])
            + (2.453448e-05) * (q[i6] * q[i11]) + (1.155972e-04) * (q[i6] * q[i12]) + (2.382661e-04) * (q[i6] * q[i15]) + (-4.223975e-04) * (q[i6] * q[i16])
            + (1.838184e-04) * (q[i6] * q[i20]) + (-1.257420e-04) * (q[i6] * q[i21]) + (-7.666956e-05) * (q[i6] * q[i22]) + (2.425172e-06) * (q[i7] * q[i8])
            + (6.235673e-05) * (q[i7] * q[i9]) + (2.787552e-04) * (q[i7] * q[i10]) + (-1.439265e-04) * (q[i7] * q[i11]) + (1.244186e-03) * (q[i7] * q[i12])
            + (-1.144394e-05) * (q[i7] * q[i15]) + (2.379539e-04) * (q[i7] * q[i16]) + (-1.807043e-04) * (q[i7] * q[i20]) + (-6.118894e-04) * (q[i7] * q[i21])
            + (8.430038e-04) * (q[i7] * q[i22]) + (-2.432863e-04) * (q[i8] * q[i9]) + (-4.806814e-05) * (q[i8] * q[i10]) + (1.077931e-03) * (q[i8] * q[i11])
            + (-1.796733e-04) * (q[i8] * q[i12]) + (4.603370e-04) * (q[i8] * q[i15]) + (-2.102962e-04) * (q[i8] * q[i16]) + (5.811766e-08) * (q[i8] * q[i20])
            + (-1.353064e-03) * (q[i8] * q[i21]) + (-1.799147e-05) * (q[i8] * q[i22]) + (-4.452825e-05) * (q[i9] * q[i10]) + (4.364411e-05) * (q[i9] * q[i11])
            + (-8.314378e-05) * (q[i9] * q[i12]) + (1.483290e-04) * (q[i9] * q[i15]) + (-2.577399e-04) * (q[i9] * q[i16]) + (1.544749e-04) * (q[i9] * q[i20])
            + (-2.061993e-05) * (q[i9] * q[i21]) + (1.277229e-04) * (q[i9] * q[i22]) + (-4.681708e-05) * (q[i10] * q[i11]) + (2.078339e-04) * (q[i10] * q[i12])
            + (-1.906830e-04) * (q[i10] * q[i15]) + (-1.615727e-04) * (q[i10] * q[i16]) + (-1.553178e-04) * (q[i10] * q[i20])
            + (-3.572332e-04) * (q[i10] * q[i21]) + (1.728515e-04) * (q[i10] * q[i22]) + (3.545202e-04) * (q[i11] * q[i12])
            + (-4.017164e-04) * (q[i11] * q[i15]) + (5.782590e-05) * (q[i11] * q[i16]) + (-2.405631e-04) * (q[i11] * q[i20])
            + (-2.057011e-03) * (q[i11] * q[i21]) + (6.084827e-05) * (q[i11] * q[i22]) + (1.354376e-04) * (q[i12] * q[i15])
            + (-2.004387e-05) * (q[i12] * q[i16]) + (-2.413345e-04) * (q[i12] * q[i20]) + (2.520685e-04) * (q[i12] * q[i21])
            + (-1.723724e-04) * (q[i12] * q[i22]) + (2.134674e-04) * (q[i15] * q[i16]) + (1.092114e-05) * (q[i15] * q[i20]) + (5.399830e-05) * (q[i15] * q[i21])
            + (-8.669724e-05) * (q[i15] * q[i22]) + (-7.888197e-06) * (q[i16] * q[i20]) + (-2.416558e-05) * (q[i16] * q[i21])
            + (5.949074e-06) * (q[i16] * q[i22]) + (1.641668e-04) * (q[i20] * q[i21]) + (1.603013e-04) * (q[i20] * q[i22]) + (6.385282e-05) * (q[i21] * q[i22]);
   }

   public void getJQx20(double[] q, double[][] JQ)
   {
      JQ[1][i20] = (-9.533365e-04) * (1) + (-7.808437e-04) * ((2) * q[i20]) + (6.647019e-04) * (q[i0]) + (-1.956615e-03) * (q[i1]) + (-2.100913e-03) * (q[i2])
            + (3.470420e-03) * (q[i3]) + (-2.334264e-03) * (q[i4]) + (-5.439486e-04) * (q[i5]) + (1.200329e-03) * (q[i6]) + (-2.023647e-03) * (q[i7])
            + (4.509890e-04) * (q[i8]) + (1.035450e-03) * (q[i9]) + (-8.931737e-05) * (q[i10]) + (1.693188e-03) * (q[i11]) + (-1.523914e-03) * (q[i12])
            + (-1.050440e-04) * (q[i15]) + (-1.418129e-03) * (q[i16]) + (6.413053e-06) * (q[i19]) + (-1.065760e-04) * (q[i21]) + (-8.800162e-04) * (q[i22])
            + (-3.548803e-04) * (q[i0] * q[i0]) + (5.501578e-05) * (q[i1] * q[i1]) + (-1.591219e-04) * (q[i2] * q[i2]) + (-2.797243e-04) * (q[i3] * q[i3])
            + (1.091014e-03) * (q[i4] * q[i4]) + (-4.678912e-04) * (q[i5] * q[i5]) + (4.183499e-04) * (q[i6] * q[i6]) + (1.400944e-04) * (q[i7] * q[i7])
            + (2.835757e-05) * (q[i8] * q[i8]) + (-1.073877e-05) * (q[i9] * q[i9]) + (1.445431e-04) * (q[i10] * q[i10]) + (-1.291444e-04) * (q[i11] * q[i11])
            + (-4.017068e-04) * (q[i12] * q[i12]) + (1.325622e-04) * (q[i15] * q[i15]) + (-2.153955e-04) * (q[i16] * q[i16])
            + (-5.105214e-06) * (q[i19] * q[i19]) + (-1.050280e-04) * ((2) * q[i0] * q[i20]) + (-2.351599e-05) * ((2) * q[i1] * q[i20])
            + (-2.080122e-04) * ((2) * q[i2] * q[i20]) + (1.332810e-04) * ((2) * q[i3] * q[i20]) + (1.504221e-04) * ((2) * q[i4] * q[i20])
            + (3.262443e-04) * ((2) * q[i5] * q[i20]) + (-3.249408e-04) * ((2) * q[i6] * q[i20]) + (1.465960e-04) * ((2) * q[i7] * q[i20])
            + (-8.367389e-04) * ((2) * q[i8] * q[i20]) + (-4.193547e-05) * ((2) * q[i9] * q[i20]) + (2.972484e-05) * ((2) * q[i10] * q[i20])
            + (-3.707643e-05) * ((2) * q[i11] * q[i20]) + (-1.051264e-04) * ((2) * q[i12] * q[i20]) + (-1.661363e-04) * ((2) * q[i15] * q[i20])
            + (-3.447732e-04) * ((2) * q[i16] * q[i20]) + (4.827284e-06) * ((2) * q[i19] * q[i20]) + (-1.374386e-04) * ((3) * q[i20] * q[i20])
            + (-7.287472e-05) * ((2) * q[i20] * q[i21]) + (6.647343e-04) * ((2) * q[i20] * q[i22]) + (-1.229703e-04) * (q[i21] * q[i21])
            + (8.810742e-05) * (q[i22] * q[i22]) + (5.868450e-04) * (q[i0] * q[i1]) + (1.349169e-03) * (q[i0] * q[i2]) + (1.245394e-03) * (q[i0] * q[i3])
            + (-9.938613e-04) * (q[i0] * q[i4]) + (1.593632e-04) * (q[i0] * q[i5]) + (1.359705e-03) * (q[i0] * q[i6]) + (-9.066067e-04) * (q[i0] * q[i7])
            + (1.054646e-03) * (q[i0] * q[i8]) + (5.663061e-04) * (q[i0] * q[i9]) + (-6.638931e-04) * (q[i0] * q[i10]) + (-2.593303e-04) * (q[i0] * q[i11])
            + (-5.335765e-04) * (q[i0] * q[i12]) + (4.410306e-04) * (q[i0] * q[i15]) + (8.003445e-04) * (q[i0] * q[i16]) + (4.840219e-04) * (q[i0] * q[i19])
            + (7.464442e-04) * (q[i0] * q[i21]) + (8.370573e-04) * (q[i0] * q[i22]) + (-4.233245e-04) * (q[i1] * q[i2]) + (1.000061e-03) * (q[i1] * q[i3])
            + (-1.501474e-03) * (q[i1] * q[i4]) + (8.323787e-04) * (q[i1] * q[i5]) + (-6.114105e-04) * (q[i1] * q[i6]) + (1.199835e-03) * (q[i1] * q[i7])
            + (1.348211e-03) * (q[i1] * q[i8]) + (-3.448627e-04) * (q[i1] * q[i9]) + (8.148392e-04) * (q[i1] * q[i10]) + (-1.317697e-04) * (q[i1] * q[i11])
            + (-6.142505e-04) * (q[i1] * q[i12]) + (-1.042180e-03) * (q[i1] * q[i15]) + (-1.634519e-04) * (q[i1] * q[i16]) + (4.608439e-04) * (q[i1] * q[i19])
            + (5.044497e-04) * (q[i1] * q[i21]) + (2.404725e-04) * (q[i1] * q[i22]) + (5.090413e-04) * (q[i2] * q[i3]) + (9.312876e-04) * (q[i2] * q[i4])
            + (-1.194357e-03) * (q[i2] * q[i5]) + (3.018136e-04) * (q[i2] * q[i6]) + (1.274549e-03) * (q[i2] * q[i7]) + (1.062139e-03) * (q[i2] * q[i8])
            + (3.097910e-04) * (q[i2] * q[i9]) + (1.234765e-04) * (q[i2] * q[i10]) + (5.165242e-05) * (q[i2] * q[i11]) + (-1.678354e-03) * (q[i2] * q[i12])
            + (-8.625585e-04) * (q[i2] * q[i15]) + (-9.489442e-04) * (q[i2] * q[i16]) + (1.081195e-03) * (q[i2] * q[i19]) + (5.348327e-04) * (q[i2] * q[i21])
            + (1.183849e-03) * (q[i2] * q[i22]) + (7.792435e-04) * (q[i3] * q[i4]) + (-1.240502e-03) * (q[i3] * q[i5]) + (-1.961914e-04) * (q[i3] * q[i6])
            + (1.872840e-03) * (q[i3] * q[i7]) + (4.997872e-05) * (q[i3] * q[i8]) + (-1.099831e-03) * (q[i3] * q[i9]) + (-1.118422e-04) * (q[i3] * q[i10])
            + (-6.758842e-05) * (q[i3] * q[i11]) + (8.499335e-04) * (q[i3] * q[i12]) + (-5.463849e-04) * (q[i3] * q[i15]) + (4.429594e-04) * (q[i3] * q[i16])
            + (6.337695e-04) * (q[i3] * q[i19]) + (-3.009105e-04) * (q[i3] * q[i21]) + (-1.722683e-04) * (q[i3] * q[i22]) + (8.317277e-04) * (q[i4] * q[i5])
            + (-1.470723e-04) * (q[i4] * q[i6]) + (-2.927266e-03) * (q[i4] * q[i7]) + (-1.055922e-03) * (q[i4] * q[i8]) + (9.484636e-04) * (q[i4] * q[i9])
            + (2.507049e-04) * (q[i4] * q[i10]) + (5.438664e-04) * (q[i4] * q[i11]) + (-3.604102e-04) * (q[i4] * q[i12]) + (7.146477e-04) * (q[i4] * q[i15])
            + (-4.883223e-04) * (q[i4] * q[i16]) + (6.201209e-04) * (q[i4] * q[i19]) + (3.878512e-04) * (q[i4] * q[i21]) + (-1.278620e-03) * (q[i4] * q[i22])
            + (6.275868e-04) * (q[i5] * q[i6]) + (2.373423e-04) * (q[i5] * q[i7]) + (-1.517377e-03) * (q[i5] * q[i8]) + (6.840200e-05) * (q[i5] * q[i9])
            + (-1.415325e-04) * (q[i5] * q[i10]) + (-4.212765e-04) * (q[i5] * q[i11]) + (-7.160581e-04) * (q[i5] * q[i12]) + (-3.359705e-04) * (q[i5] * q[i15])
            + (-7.404865e-04) * (q[i5] * q[i16]) + (-5.472268e-04) * (q[i5] * q[i19]) + (-5.014894e-04) * (q[i5] * q[i21]) + (1.779544e-03) * (q[i5] * q[i22])
            + (-3.841827e-04) * (q[i6] * q[i7]) + (-2.926902e-06) * (q[i6] * q[i8]) + (-2.811633e-04) * (q[i6] * q[i9]) + (-6.403651e-05) * (q[i6] * q[i10])
            + (1.247018e-03) * (q[i6] * q[i11]) + (-1.390614e-04) * (q[i6] * q[i12]) + (-2.391586e-04) * (q[i6] * q[i15]) + (1.109683e-05) * (q[i6] * q[i16])
            + (1.838184e-04) * (q[i6] * q[i19]) + (8.421887e-04) * (q[i6] * q[i21]) + (-6.182464e-04) * (q[i6] * q[i22]) + (3.094587e-04) * (q[i7] * q[i8])
            + (-3.490015e-07) * (q[i7] * q[i9]) + (5.044036e-04) * (q[i7] * q[i10]) + (1.173055e-04) * (q[i7] * q[i11]) + (2.493281e-05) * (q[i7] * q[i12])
            + (4.214893e-04) * (q[i7] * q[i15]) + (-2.380913e-04) * (q[i7] * q[i16]) + (-1.807043e-04) * (q[i7] * q[i19]) + (-8.372730e-05) * (q[i7] * q[i21])
            + (-1.313692e-04) * (q[i7] * q[i22]) + (5.502630e-05) * (q[i8] * q[i9]) + (2.439533e-04) * (q[i8] * q[i10]) + (-1.831442e-04) * (q[i8] * q[i11])
            + (1.059739e-03) * (q[i8] * q[i12]) + (2.050454e-04) * (q[i8] * q[i15]) + (-4.268456e-04) * (q[i8] * q[i16]) + (5.811766e-08) * (q[i8] * q[i19])
            + (-8.841363e-06) * (q[i8] * q[i21]) + (-1.338323e-03) * (q[i8] * q[i22]) + (4.880631e-05) * (q[i9] * q[i10]) + (2.088469e-04) * (q[i9] * q[i11])
            + (-5.902861e-05) * (q[i9] * q[i12]) + (1.634168e-04) * (q[i9] * q[i15]) + (1.908896e-04) * (q[i9] * q[i16]) + (1.544749e-04) * (q[i9] * q[i19])
            + (1.741792e-04) * (q[i9] * q[i21]) + (-3.629460e-04) * (q[i9] * q[i22]) + (-8.202952e-05) * (q[i10] * q[i11]) + (4.660277e-05) * (q[i10] * q[i12])
            + (2.572036e-04) * (q[i10] * q[i15]) + (-1.470767e-04) * (q[i10] * q[i16]) + (-1.553178e-04) * (q[i10] * q[i19])
            + (1.273184e-04) * (q[i10] * q[i21]) + (-2.134184e-05) * (q[i10] * q[i22]) + (-3.485009e-04) * (q[i11] * q[i12])
            + (-2.733872e-05) * (q[i11] * q[i15]) + (1.390104e-04) * (q[i11] * q[i16]) + (-2.405631e-04) * (q[i11] * q[i19])
            + (1.702902e-04) * (q[i11] * q[i21]) + (-2.498516e-04) * (q[i11] * q[i22]) + (6.039360e-05) * (q[i12] * q[i15])
            + (-4.167561e-04) * (q[i12] * q[i16]) + (-2.413345e-04) * (q[i12] * q[i19]) + (-6.266256e-05) * (q[i12] * q[i21])
            + (2.050566e-03) * (q[i12] * q[i22]) + (-2.091147e-04) * (q[i15] * q[i16]) + (1.092114e-05) * (q[i15] * q[i19]) + (1.848389e-06) * (q[i15] * q[i21])
            + (-2.399921e-05) * (q[i15] * q[i22]) + (-7.888197e-06) * (q[i16] * q[i19]) + (-8.441539e-05) * (q[i16] * q[i21])
            + (6.254680e-05) * (q[i16] * q[i22]) + (1.641668e-04) * (q[i19] * q[i21]) + (1.603013e-04) * (q[i19] * q[i22])
            + (-5.825105e-05) * (q[i21] * q[i22]);
   }

   public void getJQx21(double[] q, double[][] JQ)
   {
      JQ[1][i21] = (-2.718647e-03) * (1) + (1.668478e-03) * ((2) * q[i21]) + (1.479821e-03) * (q[i0]) + (-3.335710e-04) * (q[i1]) + (1.403467e-03) * (q[i2])
            + (-9.036793e-04) * (q[i3]) + (-5.053012e-04) * (q[i4]) + (-8.173700e-04) * (q[i5]) + (-7.083634e-04) * (q[i6]) + (-1.223041e-03) * (q[i7])
            + (2.241619e-03) * (q[i8]) + (2.236456e-04) * (q[i9]) + (-5.361554e-05) * (q[i10]) + (4.262676e-04) * (q[i11]) + (2.662284e-04) * (q[i12])
            + (2.450143e-03) * (q[i15]) + (2.959218e-04) * (q[i16]) + (-9.128984e-04) * (q[i19]) + (-1.065760e-04) * (q[i20]) + (-9.178308e-06) * (q[i22])
            + (6.331864e-04) * (q[i0] * q[i0]) + (-2.780366e-04) * (q[i1] * q[i1]) + (1.112668e-03) * (q[i2] * q[i2]) + (1.536098e-04) * (q[i3] * q[i3])
            + (3.029220e-04) * (q[i4] * q[i4]) + (-4.744346e-04) * (q[i5] * q[i5]) + (2.497640e-04) * (q[i6] * q[i6]) + (-5.819971e-05) * (q[i7] * q[i7])
            + (-6.908828e-04) * (q[i8] * q[i8]) + (-2.193992e-05) * (q[i9] * q[i9]) + (-2.028236e-05) * (q[i10] * q[i10]) + (8.513405e-04) * (q[i11] * q[i11])
            + (5.719601e-05) * (q[i12] * q[i12]) + (3.968051e-04) * (q[i15] * q[i15]) + (-1.105465e-05) * (q[i16] * q[i16]) + (6.685921e-04) * (q[i19] * q[i19])
            + (-7.287472e-05) * (q[i20] * q[i20]) + (2.005327e-04) * ((2) * q[i0] * q[i21]) + (3.551545e-04) * ((2) * q[i1] * q[i21])
            + (6.122352e-04) * ((2) * q[i2] * q[i21]) + (2.876675e-04) * ((2) * q[i3] * q[i21]) + (1.871116e-04) * ((2) * q[i4] * q[i21])
            + (-1.374830e-03) * ((2) * q[i5] * q[i21]) + (-1.487235e-04) * ((2) * q[i6] * q[i21]) + (-3.780352e-04) * ((2) * q[i7] * q[i21])
            + (6.811219e-04) * ((2) * q[i8] * q[i21]) + (1.256556e-04) * ((2) * q[i9] * q[i21]) + (-1.027291e-04) * ((2) * q[i10] * q[i21])
            + (4.836974e-04) * ((2) * q[i11] * q[i21]) + (1.767875e-04) * ((2) * q[i12] * q[i21]) + (7.926800e-04) * ((2) * q[i15] * q[i21])
            + (-1.365182e-05) * ((2) * q[i16] * q[i21]) + (-9.116763e-05) * ((2) * q[i19] * q[i21]) + (-1.229703e-04) * ((2) * q[i20] * q[i21])
            + (3.620149e-04) * ((3) * q[i21] * q[i21]) + (3.931188e-05) * ((2) * q[i21] * q[i22]) + (4.055755e-05) * (q[i22] * q[i22])
            + (2.626669e-04) * (q[i0] * q[i1]) + (5.814716e-04) * (q[i0] * q[i2]) + (-8.057844e-04) * (q[i0] * q[i3]) + (-3.788516e-04) * (q[i0] * q[i4])
            + (6.680767e-04) * (q[i0] * q[i5]) + (-8.987898e-04) * (q[i0] * q[i6]) + (-2.006549e-04) * (q[i0] * q[i7]) + (3.697073e-04) * (q[i0] * q[i8])
            + (-4.827697e-04) * (q[i0] * q[i9]) + (-6.733610e-04) * (q[i0] * q[i10]) + (7.605467e-04) * (q[i0] * q[i11]) + (4.240182e-04) * (q[i0] * q[i12])
            + (2.398073e-04) * (q[i0] * q[i15]) + (-2.354564e-04) * (q[i0] * q[i16]) + (-2.430971e-04) * (q[i0] * q[i19]) + (7.464442e-04) * (q[i0] * q[i20])
            + (1.707779e-04) * (q[i0] * q[i22]) + (1.109319e-04) * (q[i1] * q[i2]) + (5.962383e-04) * (q[i1] * q[i3]) + (-7.710065e-04) * (q[i1] * q[i4])
            + (-2.455253e-04) * (q[i1] * q[i5]) + (8.439397e-04) * (q[i1] * q[i6]) + (-6.180849e-04) * (q[i1] * q[i7]) + (3.351103e-04) * (q[i1] * q[i8])
            + (4.375698e-04) * (q[i1] * q[i9]) + (5.603105e-05) * (q[i1] * q[i10]) + (-7.379106e-04) * (q[i1] * q[i11]) + (3.496778e-04) * (q[i1] * q[i12])
            + (2.924900e-05) * (q[i1] * q[i15]) + (-5.340071e-04) * (q[i1] * q[i16]) + (-8.342090e-04) * (q[i1] * q[i19]) + (5.044497e-04) * (q[i1] * q[i20])
            + (1.760598e-04) * (q[i1] * q[i22]) + (-7.309653e-04) * (q[i2] * q[i3]) + (9.853955e-04) * (q[i2] * q[i4]) + (5.688177e-04) * (q[i2] * q[i5])
            + (1.285496e-04) * (q[i2] * q[i6]) + (-3.183364e-04) * (q[i2] * q[i7]) + (-5.368134e-04) * (q[i2] * q[i8]) + (-3.177849e-04) * (q[i2] * q[i9])
            + (-2.055229e-04) * (q[i2] * q[i10]) + (8.125435e-05) * (q[i2] * q[i11]) + (-1.404150e-04) * (q[i2] * q[i12]) + (3.715050e-04) * (q[i2] * q[i15])
            + (-3.632431e-04) * (q[i2] * q[i16]) + (-1.188958e-03) * (q[i2] * q[i19]) + (5.348327e-04) * (q[i2] * q[i20]) + (2.080626e-04) * (q[i2] * q[i22])
            + (-7.415448e-04) * (q[i3] * q[i4]) + (3.996984e-05) * (q[i3] * q[i5]) + (-8.116925e-04) * (q[i3] * q[i6]) + (1.340993e-03) * (q[i3] * q[i7])
            + (1.835052e-04) * (q[i3] * q[i8]) + (2.429874e-04) * (q[i3] * q[i9]) + (-2.816300e-04) * (q[i3] * q[i10]) + (-8.200449e-04) * (q[i3] * q[i11])
            + (8.870416e-04) * (q[i3] * q[i12]) + (-1.765358e-04) * (q[i3] * q[i15]) + (-2.275339e-05) * (q[i3] * q[i16]) + (1.284728e-03) * (q[i3] * q[i19])
            + (-3.009105e-04) * (q[i3] * q[i20]) + (2.200795e-04) * (q[i3] * q[i22]) + (1.183960e-03) * (q[i4] * q[i5]) + (-7.265601e-04) * (q[i4] * q[i6])
            + (-3.443914e-04) * (q[i4] * q[i7]) + (1.136042e-04) * (q[i4] * q[i8]) + (-3.795851e-04) * (q[i4] * q[i9]) + (1.679941e-04) * (q[i4] * q[i10])
            + (-2.493820e-04) * (q[i4] * q[i11]) + (-1.056132e-03) * (q[i4] * q[i12]) + (5.753389e-04) * (q[i4] * q[i15]) + (-1.986130e-04) * (q[i4] * q[i16])
            + (1.730511e-04) * (q[i4] * q[i19]) + (3.878512e-04) * (q[i4] * q[i20]) + (2.073554e-04) * (q[i4] * q[i22]) + (4.400469e-04) * (q[i5] * q[i6])
            + (-5.079405e-04) * (q[i5] * q[i7]) + (6.891492e-04) * (q[i5] * q[i8]) + (5.699121e-05) * (q[i5] * q[i9]) + (1.988570e-04) * (q[i5] * q[i10])
            + (-2.158314e-04) * (q[i5] * q[i11]) + (-3.110414e-04) * (q[i5] * q[i12]) + (-1.399082e-03) * (q[i5] * q[i15]) + (-1.216028e-04) * (q[i5] * q[i16])
            + (-1.777132e-03) * (q[i5] * q[i19]) + (-5.014894e-04) * (q[i5] * q[i20]) + (-5.758126e-04) * (q[i5] * q[i22]) + (-2.921076e-04) * (q[i6] * q[i7])
            + (3.936861e-05) * (q[i6] * q[i8]) + (-5.704405e-06) * (q[i6] * q[i9]) + (6.740169e-04) * (q[i6] * q[i10]) + (-3.542402e-04) * (q[i6] * q[i11])
            + (9.065239e-05) * (q[i6] * q[i12]) + (-5.152226e-05) * (q[i6] * q[i15]) + (1.089463e-04) * (q[i6] * q[i16]) + (-1.257420e-04) * (q[i6] * q[i19])
            + (8.421887e-04) * (q[i6] * q[i20]) + (-1.593029e-04) * (q[i6] * q[i22]) + (6.312095e-04) * (q[i7] * q[i8]) + (3.703065e-04) * (q[i7] * q[i9])
            + (6.170180e-06) * (q[i7] * q[i10]) + (5.647104e-04) * (q[i7] * q[i11]) + (3.914591e-04) * (q[i7] * q[i12]) + (1.458426e-04) * (q[i7] * q[i15])
            + (2.063762e-04) * (q[i7] * q[i16]) + (-6.118894e-04) * (q[i7] * q[i19]) + (-8.372730e-05) * (q[i7] * q[i20]) + (1.575562e-04) * (q[i7] * q[i22])
            + (1.687497e-04) * (q[i8] * q[i9]) + (-6.443793e-04) * (q[i8] * q[i10]) + (1.827731e-03) * (q[i8] * q[i11]) + (4.025228e-05) * (q[i8] * q[i12])
            + (-1.066225e-04) * (q[i8] * q[i15]) + (-4.533561e-04) * (q[i8] * q[i16]) + (-1.353064e-03) * (q[i8] * q[i19]) + (-8.841363e-06) * (q[i8] * q[i20])
            + (-1.803648e-06) * (q[i8] * q[i22]) + (1.934615e-04) * (q[i9] * q[i10]) + (2.569153e-05) * (q[i9] * q[i11]) + (2.179966e-04) * (q[i9] * q[i12])
            + (6.501105e-05) * (q[i9] * q[i15]) + (4.393048e-05) * (q[i9] * q[i16]) + (-2.061993e-05) * (q[i9] * q[i19]) + (1.741792e-04) * (q[i9] * q[i20])
            + (-1.726096e-05) * (q[i9] * q[i22]) + (-9.864996e-05) * (q[i10] * q[i11]) + (1.714616e-04) * (q[i10] * q[i12])
            + (-9.423758e-05) * (q[i10] * q[i15]) + (-4.275113e-05) * (q[i10] * q[i16]) + (-3.572332e-04) * (q[i10] * q[i19])
            + (1.273184e-04) * (q[i10] * q[i20]) + (1.970438e-05) * (q[i10] * q[i22]) + (-4.512187e-04) * (q[i11] * q[i12])
            + (-4.815746e-04) * (q[i11] * q[i15]) + (-6.533038e-04) * (q[i11] * q[i16]) + (-2.057011e-03) * (q[i11] * q[i19])
            + (1.702902e-04) * (q[i11] * q[i20]) + (8.413113e-05) * (q[i11] * q[i22]) + (3.469436e-04) * (q[i12] * q[i15]) + (1.295322e-04) * (q[i12] * q[i16])
            + (2.520685e-04) * (q[i12] * q[i19]) + (-6.266256e-05) * (q[i12] * q[i20]) + (8.656308e-05) * (q[i12] * q[i22]) + (1.579448e-04) * (q[i15] * q[i16])
            + (5.399830e-05) * (q[i15] * q[i19]) + (1.848389e-06) * (q[i15] * q[i20]) + (-6.705985e-05) * (q[i15] * q[i22])
            + (-2.416558e-05) * (q[i16] * q[i19]) + (-8.441539e-05) * (q[i16] * q[i20]) + (6.619962e-05) * (q[i16] * q[i22])
            + (1.641668e-04) * (q[i19] * q[i20]) + (6.385282e-05) * (q[i19] * q[i22]) + (-5.825105e-05) * (q[i20] * q[i22]);
   }

   public void getJQx22(double[] q, double[][] JQ)
   {
      JQ[1][i22] = (-2.768467e-03) * (1) + (-1.674249e-03) * ((2) * q[i22]) + (3.335293e-04) * (q[i0]) + (-1.478069e-03) * (q[i1]) + (-1.403050e-03) * (q[i2])
            + (5.059900e-04) * (q[i3]) + (8.770941e-04) * (q[i4]) + (8.332752e-04) * (q[i5]) + (-1.230140e-03) * (q[i6]) + (-6.863549e-04) * (q[i7])
            + (2.269841e-03) * (q[i8]) + (-5.577125e-05) * (q[i9]) + (2.301890e-04) * (q[i10]) + (-2.681229e-04) * (q[i11]) + (-4.255399e-04) * (q[i12])
            + (2.765729e-04) * (q[i15]) + (2.443170e-03) * (q[i16]) + (-9.662913e-05) * (q[i19]) + (-8.800162e-04) * (q[i20]) + (-9.178308e-06) * (q[i21])
            + (-2.889568e-04) * (q[i0] * q[i0]) + (6.328589e-04) * (q[i1] * q[i1]) + (1.105582e-03) * (q[i2] * q[i2]) + (3.081561e-04) * (q[i3] * q[i3])
            + (1.453614e-04) * (q[i4] * q[i4]) + (-4.877765e-04) * (q[i5] * q[i5]) + (-6.035456e-05) * (q[i6] * q[i6]) + (2.482698e-04) * (q[i7] * q[i7])
            + (-6.994664e-04) * (q[i8] * q[i8]) + (-1.976300e-05) * (q[i9] * q[i9]) + (-2.286918e-05) * (q[i10] * q[i10]) + (6.335773e-05) * (q[i11] * q[i11])
            + (8.551246e-04) * (q[i12] * q[i12]) + (-1.268897e-05) * (q[i15] * q[i15]) + (4.009891e-04) * (q[i16] * q[i16])
            + (-7.300996e-05) * (q[i19] * q[i19]) + (6.647343e-04) * (q[i20] * q[i20]) + (3.931188e-05) * (q[i21] * q[i21])
            + (3.563537e-04) * ((2) * q[i0] * q[i22]) + (1.959782e-04) * ((2) * q[i1] * q[i22]) + (6.068116e-04) * ((2) * q[i2] * q[i22])
            + (1.916338e-04) * ((2) * q[i3] * q[i22]) + (2.904623e-04) * ((2) * q[i4] * q[i22]) + (-1.379575e-03) * ((2) * q[i5] * q[i22])
            + (3.754935e-04) * ((2) * q[i6] * q[i22]) + (1.424931e-04) * ((2) * q[i7] * q[i22]) + (-6.902480e-04) * ((2) * q[i8] * q[i22])
            + (1.046439e-04) * ((2) * q[i9] * q[i22]) + (-1.250614e-04) * ((2) * q[i10] * q[i22]) + (1.762423e-04) * ((2) * q[i11] * q[i22])
            + (4.884731e-04) * ((2) * q[i12] * q[i22]) + (1.745091e-05) * ((2) * q[i15] * q[i22]) + (-7.943027e-04) * ((2) * q[i16] * q[i22])
            + (1.245651e-04) * ((2) * q[i19] * q[i22]) + (8.810742e-05) * ((2) * q[i20] * q[i22]) + (4.055755e-05) * ((2) * q[i21] * q[i22])
            + (3.661811e-04) * ((3) * q[i22] * q[i22]) + (2.594540e-04) * (q[i0] * q[i1]) + (1.058040e-04) * (q[i0] * q[i2]) + (-7.652511e-04) * (q[i0] * q[i3])
            + (6.018258e-04) * (q[i0] * q[i4]) + (-2.400176e-04) * (q[i0] * q[i5]) + (6.300372e-04) * (q[i0] * q[i6]) + (-8.392349e-04) * (q[i0] * q[i7])
            + (-3.461998e-04) * (q[i0] * q[i8]) + (-5.376229e-05) * (q[i0] * q[i9]) + (-4.308028e-04) * (q[i0] * q[i10]) + (3.498287e-04) * (q[i0] * q[i11])
            + (-7.301080e-04) * (q[i0] * q[i12]) + (5.474682e-04) * (q[i0] * q[i15]) + (-2.929430e-05) * (q[i0] * q[i16]) + (-5.005325e-04) * (q[i0] * q[i19])
            + (8.370573e-04) * (q[i0] * q[i20]) + (1.707779e-04) * (q[i0] * q[i21]) + (5.835975e-04) * (q[i1] * q[i2]) + (-3.858473e-04) * (q[i1] * q[i3])
            + (-8.268980e-04) * (q[i1] * q[i4]) + (6.741223e-04) * (q[i1] * q[i5]) + (1.852810e-04) * (q[i1] * q[i6]) + (9.134953e-04) * (q[i1] * q[i7])
            + (-3.669231e-04) * (q[i1] * q[i8]) + (6.739413e-04) * (q[i1] * q[i9]) + (4.743153e-04) * (q[i1] * q[i10]) + (4.227531e-04) * (q[i1] * q[i11])
            + (7.630297e-04) * (q[i1] * q[i12]) + (2.277305e-04) * (q[i1] * q[i15]) + (-2.454685e-04) * (q[i1] * q[i16]) + (-7.396063e-04) * (q[i1] * q[i19])
            + (2.404725e-04) * (q[i1] * q[i20]) + (1.760598e-04) * (q[i1] * q[i21]) + (1.000350e-03) * (q[i2] * q[i3]) + (-7.527671e-04) * (q[i2] * q[i4])
            + (5.556571e-04) * (q[i2] * q[i5]) + (3.242108e-04) * (q[i2] * q[i6]) + (-1.257341e-04) * (q[i2] * q[i7]) + (5.417197e-04) * (q[i2] * q[i8])
            + (2.041463e-04) * (q[i2] * q[i9]) + (3.180438e-04) * (q[i2] * q[i10]) + (-1.375290e-04) * (q[i2] * q[i11]) + (8.041627e-05) * (q[i2] * q[i12])
            + (3.675351e-04) * (q[i2] * q[i15]) + (-3.792889e-04) * (q[i2] * q[i16]) + (-5.315273e-04) * (q[i2] * q[i19]) + (1.183849e-03) * (q[i2] * q[i20])
            + (2.080626e-04) * (q[i2] * q[i21]) + (-7.459352e-04) * (q[i3] * q[i4]) + (1.160825e-03) * (q[i3] * q[i5]) + (3.394242e-04) * (q[i3] * q[i6])
            + (7.346822e-04) * (q[i3] * q[i7]) + (-1.137578e-04) * (q[i3] * q[i8]) + (-1.711662e-04) * (q[i3] * q[i9]) + (3.857699e-04) * (q[i3] * q[i10])
            + (-1.065079e-03) * (q[i3] * q[i11]) + (-2.562754e-04) * (q[i3] * q[i12]) + (2.002703e-04) * (q[i3] * q[i15]) + (-5.864281e-04) * (q[i3] * q[i16])
            + (-3.871210e-04) * (q[i3] * q[i19]) + (-1.722683e-04) * (q[i3] * q[i20]) + (2.200795e-04) * (q[i3] * q[i21]) + (7.428462e-05) * (q[i4] * q[i5])
            + (-1.339614e-03) * (q[i4] * q[i6]) + (8.115579e-04) * (q[i4] * q[i7]) + (-1.860377e-04) * (q[i4] * q[i8]) + (2.792880e-04) * (q[i4] * q[i9])
            + (-2.440550e-04) * (q[i4] * q[i10]) + (8.925204e-04) * (q[i4] * q[i11]) + (-8.202490e-04) * (q[i4] * q[i12]) + (1.739075e-05) * (q[i4] * q[i15])
            + (1.665167e-04) * (q[i4] * q[i16]) + (3.040057e-04) * (q[i4] * q[i19]) + (-1.278620e-03) * (q[i4] * q[i20]) + (2.073554e-04) * (q[i4] * q[i21])
            + (5.221971e-04) * (q[i5] * q[i6]) + (-4.616325e-04) * (q[i5] * q[i7]) + (-7.023000e-04) * (q[i5] * q[i8]) + (-1.983944e-04) * (q[i5] * q[i9])
            + (-7.148407e-05) * (q[i5] * q[i10]) + (-3.102334e-04) * (q[i5] * q[i11]) + (-2.057889e-04) * (q[i5] * q[i12]) + (1.192572e-04) * (q[i5] * q[i15])
            + (1.415512e-03) * (q[i5] * q[i16]) + (5.059876e-04) * (q[i5] * q[i19]) + (1.779544e-03) * (q[i5] * q[i20]) + (-5.758126e-04) * (q[i5] * q[i21])
            + (-2.994227e-04) * (q[i6] * q[i7]) + (6.272907e-04) * (q[i6] * q[i8]) + (6.697351e-06) * (q[i6] * q[i9]) + (3.724830e-04) * (q[i6] * q[i10])
            + (-3.928276e-04) * (q[i6] * q[i11]) + (-5.720250e-04) * (q[i6] * q[i12]) + (2.113318e-04) * (q[i6] * q[i15]) + (1.466666e-04) * (q[i6] * q[i16])
            + (-7.666956e-05) * (q[i6] * q[i19]) + (-6.182464e-04) * (q[i6] * q[i20]) + (-1.593029e-04) * (q[i6] * q[i21]) + (3.965294e-05) * (q[i7] * q[i8])
            + (6.726312e-04) * (q[i7] * q[i9]) + (-1.011493e-05) * (q[i7] * q[i10]) + (-8.827581e-05) * (q[i7] * q[i11]) + (3.562947e-04) * (q[i7] * q[i12])
            + (1.105229e-04) * (q[i7] * q[i15]) + (-5.774850e-05) * (q[i7] * q[i16]) + (8.430038e-04) * (q[i7] * q[i19]) + (-1.313692e-04) * (q[i7] * q[i20])
            + (1.575562e-04) * (q[i7] * q[i21]) + (-6.421777e-04) * (q[i8] * q[i9]) + (1.681949e-04) * (q[i8] * q[i10]) + (-3.900571e-05) * (q[i8] * q[i11])
            + (-1.843622e-03) * (q[i8] * q[i12]) + (-4.518506e-04) * (q[i8] * q[i15]) + (-8.338577e-05) * (q[i8] * q[i16]) + (-1.799147e-05) * (q[i8] * q[i19])
            + (-1.338323e-03) * (q[i8] * q[i20]) + (-1.803648e-06) * (q[i8] * q[i21]) + (1.904536e-04) * (q[i9] * q[i10]) + (-1.682093e-04) * (q[i9] * q[i11])
            + (9.555884e-05) * (q[i9] * q[i12]) + (-3.990033e-05) * (q[i9] * q[i15]) + (-1.016466e-04) * (q[i9] * q[i16]) + (1.277229e-04) * (q[i9] * q[i19])
            + (-3.629460e-04) * (q[i9] * q[i20]) + (-1.726096e-05) * (q[i9] * q[i21]) + (-2.166452e-04) * (q[i10] * q[i11])
            + (-2.717697e-05) * (q[i10] * q[i12]) + (4.601349e-05) * (q[i10] * q[i15]) + (6.190540e-05) * (q[i10] * q[i16]) + (1.728515e-04) * (q[i10] * q[i19])
            + (-2.134184e-05) * (q[i10] * q[i20]) + (1.970438e-05) * (q[i10] * q[i21]) + (-4.540651e-04) * (q[i11] * q[i12])
            + (-1.275106e-04) * (q[i11] * q[i15]) + (-3.496984e-04) * (q[i11] * q[i16]) + (6.084827e-05) * (q[i11] * q[i19])
            + (-2.498516e-04) * (q[i11] * q[i20]) + (8.413113e-05) * (q[i11] * q[i21]) + (6.541224e-04) * (q[i12] * q[i15]) + (4.850914e-04) * (q[i12] * q[i16])
            + (-1.723724e-04) * (q[i12] * q[i19]) + (2.050566e-03) * (q[i12] * q[i20]) + (8.656308e-05) * (q[i12] * q[i21]) + (1.535920e-04) * (q[i15] * q[i16])
            + (-8.669724e-05) * (q[i15] * q[i19]) + (-2.399921e-05) * (q[i15] * q[i20]) + (-6.705985e-05) * (q[i15] * q[i21])
            + (5.949074e-06) * (q[i16] * q[i19]) + (6.254680e-05) * (q[i16] * q[i20]) + (6.619962e-05) * (q[i16] * q[i21]) + (1.603013e-04) * (q[i19] * q[i20])
            + (6.385282e-05) * (q[i19] * q[i21]) + (-5.825105e-05) * (q[i20] * q[i21]);
   }

   public void getJQy(double[] q, double[][] JQ)
   {
      getJQy0(q, JQ);
      getJQy1(q, JQ);
      getJQy2(q, JQ);
      getJQy3(q, JQ);
      getJQy4(q, JQ);
      getJQy5(q, JQ);
      getJQy6(q, JQ);
      getJQy7(q, JQ);
      getJQy8(q, JQ);
      getJQy9(q, JQ);
      getJQy10(q, JQ);
      getJQy11(q, JQ);
      getJQy12(q, JQ);
      getJQy15(q, JQ);
      getJQy16(q, JQ);
      getJQy19(q, JQ);
      getJQy20(q, JQ);
      getJQy21(q, JQ);
      getJQy22(q, JQ);
   }

   public void getJQy0(double[] q, double[][] JQ)
   {
      JQ[2][i0] = (-1.118106e-03) * (1) + (4.467133e-03) * ((2) * q[i0]) + (7.115423e-05) * (q[i1]) + (2.760604e-03) * (q[i2]) + (-1.082041e-01) * (q[i3])
            + (1.091409e-02) * (q[i4]) + (1.464916e-03) * (q[i5]) + (6.008783e-03) * (q[i6]) + (5.937328e-04) * (q[i7]) + (1.223880e-04) * (q[i8])
            + (-1.577200e-03) * (q[i9]) + (-2.364382e-03) * (q[i10]) + (-2.101461e-03) * (q[i11]) + (-6.247272e-04) * (q[i12]) + (1.763563e-03) * (q[i15])
            + (-5.522394e-03) * (q[i16]) + (-1.921777e-03) * (q[i19]) + (8.267937e-04) * (q[i20]) + (2.222365e-03) * (q[i21]) + (-8.863129e-04) * (q[i22])
            + (2.699103e-03) * ((3) * q[i0] * q[i0]) + (4.217759e-04) * ((2) * q[i0] * q[i1]) + (2.148421e-03) * ((2) * q[i0] * q[i2])
            + (-2.877567e-03) * ((2) * q[i0] * q[i3]) + (-1.957202e-03) * ((2) * q[i0] * q[i4]) + (-4.799782e-03) * ((2) * q[i0] * q[i5])
            + (-2.172771e-02) * ((2) * q[i0] * q[i6]) + (-2.006883e-03) * ((2) * q[i0] * q[i7]) + (1.524819e-03) * ((2) * q[i0] * q[i8])
            + (-3.620341e-03) * ((2) * q[i0] * q[i9]) + (-4.255330e-04) * ((2) * q[i0] * q[i10]) + (1.261540e-03) * ((2) * q[i0] * q[i11])
            + (-2.336911e-04) * ((2) * q[i0] * q[i12]) + (7.602174e-04) * ((2) * q[i0] * q[i15]) + (-1.045320e-04) * ((2) * q[i0] * q[i16])
            + (8.855494e-05) * ((2) * q[i0] * q[i19]) + (7.927038e-04) * ((2) * q[i0] * q[i20]) + (1.487935e-04) * ((2) * q[i0] * q[i21])
            + (2.358285e-04) * ((2) * q[i0] * q[i22]) + (-3.711844e-04) * (q[i1] * q[i1]) + (1.270791e-03) * (q[i2] * q[i2]) + (7.799925e-03) * (q[i3] * q[i3])
            + (2.965714e-03) * (q[i4] * q[i4]) + (-3.637586e-04) * (q[i5] * q[i5]) + (-4.331391e-03) * (q[i6] * q[i6]) + (1.095570e-03) * (q[i7] * q[i7])
            + (-1.652844e-03) * (q[i8] * q[i8]) + (1.141431e-04) * (q[i9] * q[i9]) + (5.209430e-04) * (q[i10] * q[i10]) + (2.709956e-04) * (q[i11] * q[i11])
            + (4.285072e-04) * (q[i12] * q[i12]) + (4.538791e-05) * (q[i15] * q[i15]) + (-7.146619e-04) * (q[i16] * q[i16])
            + (-1.345468e-04) * (q[i19] * q[i19]) + (2.372272e-04) * (q[i20] * q[i20]) + (1.887070e-04) * (q[i21] * q[i21]) + (4.910673e-05) * (q[i22] * q[i22])
            + (2.041900e-05) * (q[i1] * q[i2]) + (-2.203863e-03) * (q[i1] * q[i3]) + (2.176018e-03) * (q[i1] * q[i4]) + (-3.023284e-06) * (q[i1] * q[i5])
            + (4.607573e-03) * (q[i1] * q[i6]) + (4.585382e-03) * (q[i1] * q[i7]) + (3.606869e-03) * (q[i1] * q[i8]) + (3.022323e-04) * (q[i1] * q[i9])
            + (2.846155e-04) * (q[i1] * q[i10]) + (-7.803878e-04) * (q[i1] * q[i11]) + (7.788289e-04) * (q[i1] * q[i12]) + (-5.317338e-04) * (q[i1] * q[i15])
            + (-5.364147e-04) * (q[i1] * q[i16]) + (-2.149220e-03) * (q[i1] * q[i19]) + (-2.164970e-03) * (q[i1] * q[i20]) + (1.059114e-03) * (q[i1] * q[i21])
            + (-1.058629e-03) * (q[i1] * q[i22]) + (-4.830283e-03) * (q[i2] * q[i3]) + (-3.669630e-04) * (q[i2] * q[i4]) + (-1.346757e-03) * (q[i2] * q[i5])
            + (-8.850569e-03) * (q[i2] * q[i6]) + (1.509462e-03) * (q[i2] * q[i7]) + (-2.242384e-03) * (q[i2] * q[i8]) + (-1.992049e-03) * (q[i2] * q[i9])
            + (2.438050e-03) * (q[i2] * q[i10]) + (6.523406e-04) * (q[i2] * q[i11]) + (8.603355e-04) * (q[i2] * q[i12]) + (9.454998e-05) * (q[i2] * q[i15])
            + (1.685530e-04) * (q[i2] * q[i16]) + (-5.308325e-04) * (q[i2] * q[i19]) + (-7.462804e-04) * (q[i2] * q[i20]) + (1.166162e-03) * (q[i2] * q[i21])
            + (-1.663098e-03) * (q[i2] * q[i22]) + (1.577433e-02) * (q[i3] * q[i4]) + (8.064864e-03) * (q[i3] * q[i5]) + (-1.744554e-02) * (q[i3] * q[i6])
            + (7.487355e-04) * (q[i3] * q[i7]) + (-6.385192e-04) * (q[i3] * q[i8]) + (1.275901e-02) * (q[i3] * q[i9]) + (-2.628810e-03) * (q[i3] * q[i10])
            + (3.415507e-04) * (q[i3] * q[i11]) + (-1.892927e-03) * (q[i3] * q[i12]) + (-2.736937e-04) * (q[i3] * q[i15]) + (-4.429769e-03) * (q[i3] * q[i16])
            + (-1.283668e-03) * (q[i3] * q[i19]) + (-1.352818e-03) * (q[i3] * q[i20]) + (-4.281597e-05) * (q[i3] * q[i21]) + (-7.913839e-04) * (q[i3] * q[i22])
            + (2.766852e-03) * (q[i4] * q[i5]) + (-6.211916e-04) * (q[i4] * q[i6]) + (5.575730e-03) * (q[i4] * q[i7]) + (-2.558086e-03) * (q[i4] * q[i8])
            + (-9.015293e-04) * (q[i4] * q[i9]) + (-4.478219e-04) * (q[i4] * q[i10]) + (-2.682264e-04) * (q[i4] * q[i11]) + (-1.597540e-03) * (q[i4] * q[i12])
            + (-3.911409e-04) * (q[i4] * q[i15]) + (2.809789e-04) * (q[i4] * q[i16]) + (-5.715602e-06) * (q[i4] * q[i19]) + (-6.549558e-04) * (q[i4] * q[i20])
            + (-1.410528e-04) * (q[i4] * q[i21]) + (-6.179272e-04) * (q[i4] * q[i22]) + (1.989546e-03) * (q[i5] * q[i6]) + (1.368311e-03) * (q[i5] * q[i7])
            + (1.635292e-04) * (q[i5] * q[i8]) + (2.272778e-03) * (q[i5] * q[i9]) + (-3.349013e-04) * (q[i5] * q[i10]) + (-2.742440e-03) * (q[i5] * q[i11])
            + (2.213568e-03) * (q[i5] * q[i12]) + (-1.032333e-03) * (q[i5] * q[i15]) + (-1.635496e-03) * (q[i5] * q[i16]) + (6.490070e-04) * (q[i5] * q[i19])
            + (-3.652971e-05) * (q[i5] * q[i20]) + (-4.205140e-04) * (q[i5] * q[i21]) + (6.642214e-04) * (q[i5] * q[i22]) + (1.017963e-02) * (q[i6] * q[i7])
            + (1.651949e-03) * (q[i6] * q[i8]) + (-5.399697e-04) * (q[i6] * q[i9]) + (1.844408e-03) * (q[i6] * q[i10]) + (-1.342392e-03) * (q[i6] * q[i11])
            + (-1.443070e-03) * (q[i6] * q[i12]) + (-3.174323e-04) * (q[i6] * q[i15]) + (1.159499e-03) * (q[i6] * q[i16]) + (-1.498094e-03) * (q[i6] * q[i19])
            + (7.182522e-04) * (q[i6] * q[i20]) + (5.736209e-04) * (q[i6] * q[i21]) + (-8.699801e-04) * (q[i6] * q[i22]) + (2.109935e-04) * (q[i7] * q[i8])
            + (1.644994e-03) * (q[i7] * q[i9]) + (5.517188e-04) * (q[i7] * q[i10]) + (8.710504e-05) * (q[i7] * q[i11]) + (2.285169e-04) * (q[i7] * q[i12])
            + (7.248336e-04) * (q[i7] * q[i15]) + (1.887704e-04) * (q[i7] * q[i16]) + (-1.286637e-03) * (q[i7] * q[i19]) + (3.037250e-04) * (q[i7] * q[i20])
            + (-4.422425e-04) * (q[i7] * q[i21]) + (-3.798863e-04) * (q[i7] * q[i22]) + (7.119842e-04) * (q[i8] * q[i9]) + (2.040536e-04) * (q[i8] * q[i10])
            + (-9.079045e-04) * (q[i8] * q[i11]) + (9.702281e-04) * (q[i8] * q[i12]) + (-7.466604e-04) * (q[i8] * q[i15]) + (2.160150e-03) * (q[i8] * q[i16])
            + (-3.694046e-04) * (q[i8] * q[i19]) + (-2.317022e-04) * (q[i8] * q[i20]) + (2.642182e-05) * (q[i8] * q[i21]) + (-4.572423e-04) * (q[i8] * q[i22])
            + (1.360263e-03) * (q[i9] * q[i10]) + (4.597602e-05) * (q[i9] * q[i11]) + (-1.002433e-03) * (q[i9] * q[i12]) + (1.017340e-04) * (q[i9] * q[i15])
            + (1.127961e-03) * (q[i9] * q[i16]) + (-1.799168e-04) * (q[i9] * q[i19]) + (5.617884e-04) * (q[i9] * q[i20]) + (-7.818176e-05) * (q[i9] * q[i21])
            + (-6.660250e-05) * (q[i9] * q[i22]) + (6.985196e-04) * (q[i10] * q[i11]) + (5.734440e-04) * (q[i10] * q[i12]) + (5.303826e-04) * (q[i10] * q[i15])
            + (-5.940487e-04) * (q[i10] * q[i16]) + (2.148813e-04) * (q[i10] * q[i19]) + (-4.319454e-04) * (q[i10] * q[i20])
            + (-1.584458e-04) * (q[i10] * q[i21]) + (-2.132184e-04) * (q[i10] * q[i22]) + (-5.393562e-04) * (q[i11] * q[i12])
            + (1.859815e-04) * (q[i11] * q[i15]) + (-7.142625e-05) * (q[i11] * q[i16]) + (5.389142e-04) * (q[i11] * q[i19]) + (4.192410e-04) * (q[i11] * q[i20])
            + (2.893217e-04) * (q[i11] * q[i21]) + (3.909004e-04) * (q[i11] * q[i22]) + (-2.571111e-05) * (q[i12] * q[i15])
            + (-5.853968e-05) * (q[i12] * q[i16]) + (-5.457165e-04) * (q[i12] * q[i19]) + (6.830868e-05) * (q[i12] * q[i20])
            + (-5.167213e-04) * (q[i12] * q[i21]) + (-3.713889e-04) * (q[i12] * q[i22]) + (-2.180080e-04) * (q[i15] * q[i16])
            + (-9.731131e-05) * (q[i15] * q[i19]) + (-1.329828e-04) * (q[i15] * q[i20]) + (3.266881e-04) * (q[i15] * q[i21])
            + (1.098482e-04) * (q[i15] * q[i22]) + (-5.863536e-04) * (q[i16] * q[i19]) + (1.775150e-04) * (q[i16] * q[i20])
            + (-2.073675e-04) * (q[i16] * q[i21]) + (1.769337e-04) * (q[i16] * q[i22]) + (3.099871e-04) * (q[i19] * q[i20]) + (2.827223e-04) * (q[i19] * q[i21])
            + (2.580421e-04) * (q[i19] * q[i22]) + (-1.022213e-04) * (q[i20] * q[i21]) + (-2.220662e-05) * (q[i20] * q[i22])
            + (-5.595093e-04) * (q[i21] * q[i22]);
   }

   public void getJQy1(double[] q, double[][] JQ)
   {
      JQ[2][i1] = (1.171536e-03) * (1) + (4.412114e-03) * ((2) * q[i1]) + (7.115423e-05) * (q[i0]) + (2.773411e-03) * (q[i2]) + (1.089383e-02) * (q[i3])
            + (-1.076574e-01) * (q[i4]) + (1.596455e-03) * (q[i5]) + (-6.207352e-04) * (q[i6]) + (-6.060779e-03) * (q[i7]) + (-1.238358e-04) * (q[i8])
            + (2.346357e-03) * (q[i9]) + (1.575809e-03) * (q[i10]) + (-6.405035e-04) * (q[i11]) + (-2.114922e-03) * (q[i12]) + (5.440769e-03) * (q[i15])
            + (-1.795247e-03) * (q[i16]) + (-8.235971e-04) * (q[i19]) + (1.904894e-03) * (q[i20]) + (-8.990263e-04) * (q[i21]) + (2.234868e-03) * (q[i22])
            + (4.217759e-04) * (q[i0] * q[i0]) + (-3.711844e-04) * ((2) * q[i0] * q[i1]) + (-2.713918e-03) * ((3) * q[i1] * q[i1])
            + (-2.176803e-03) * ((2) * q[i1] * q[i2]) + (1.966242e-03) * ((2) * q[i1] * q[i3]) + (2.900782e-03) * ((2) * q[i1] * q[i4])
            + (4.795869e-03) * ((2) * q[i1] * q[i5]) + (-2.025674e-03) * ((2) * q[i1] * q[i6]) + (-2.168669e-02) * ((2) * q[i1] * q[i7])
            + (1.522468e-03) * ((2) * q[i1] * q[i8]) + (-4.279621e-04) * ((2) * q[i1] * q[i9]) + (-3.573820e-03) * ((2) * q[i1] * q[i10])
            + (2.313198e-04) * ((2) * q[i1] * q[i11]) + (-1.247452e-03) * ((2) * q[i1] * q[i12]) + (-1.123010e-04) * ((2) * q[i1] * q[i15])
            + (7.653473e-04) * ((2) * q[i1] * q[i16]) + (7.919462e-04) * ((2) * q[i1] * q[i19]) + (9.471634e-05) * ((2) * q[i1] * q[i20])
            + (-2.343892e-04) * ((2) * q[i1] * q[i21]) + (-1.491548e-04) * ((2) * q[i1] * q[i22]) + (-1.279295e-03) * (q[i2] * q[i2])
            + (-2.959506e-03) * (q[i3] * q[i3]) + (-7.743154e-03) * (q[i4] * q[i4]) + (3.673930e-04) * (q[i5] * q[i5]) + (-1.108751e-03) * (q[i6] * q[i6])
            + (4.318104e-03) * (q[i7] * q[i7]) + (1.667796e-03) * (q[i8] * q[i8]) + (-5.281776e-04) * (q[i9] * q[i9]) + (-1.151267e-04) * (q[i10] * q[i10])
            + (-4.310596e-04) * (q[i11] * q[i11]) + (-2.801488e-04) * (q[i12] * q[i12]) + (7.123550e-04) * (q[i15] * q[i15])
            + (-4.658708e-05) * (q[i16] * q[i16]) + (-2.431014e-04) * (q[i19] * q[i19]) + (1.360576e-04) * (q[i20] * q[i20])
            + (-6.073279e-05) * (q[i21] * q[i21]) + (-1.891935e-04) * (q[i22] * q[i22]) + (2.041900e-05) * (q[i0] * q[i2]) + (-2.203863e-03) * (q[i0] * q[i3])
            + (2.176018e-03) * (q[i0] * q[i4]) + (-3.023284e-06) * (q[i0] * q[i5]) + (4.607573e-03) * (q[i0] * q[i6]) + (4.585382e-03) * (q[i0] * q[i7])
            + (3.606869e-03) * (q[i0] * q[i8]) + (3.022323e-04) * (q[i0] * q[i9]) + (2.846155e-04) * (q[i0] * q[i10]) + (-7.803878e-04) * (q[i0] * q[i11])
            + (7.788289e-04) * (q[i0] * q[i12]) + (-5.317338e-04) * (q[i0] * q[i15]) + (-5.364147e-04) * (q[i0] * q[i16]) + (-2.149220e-03) * (q[i0] * q[i19])
            + (-2.164970e-03) * (q[i0] * q[i20]) + (1.059114e-03) * (q[i0] * q[i21]) + (-1.058629e-03) * (q[i0] * q[i22]) + (3.625418e-04) * (q[i2] * q[i3])
            + (4.829084e-03) * (q[i2] * q[i4]) + (1.352036e-03) * (q[i2] * q[i5]) + (1.486211e-03) * (q[i2] * q[i6]) + (-8.826749e-03) * (q[i2] * q[i7])
            + (-2.254740e-03) * (q[i2] * q[i8]) + (2.449361e-03) * (q[i2] * q[i9]) + (-1.976517e-03) * (q[i2] * q[i10]) + (-8.432124e-04) * (q[i2] * q[i11])
            + (-6.395182e-04) * (q[i2] * q[i12]) + (1.760426e-04) * (q[i2] * q[i15]) + (1.001514e-04) * (q[i2] * q[i16]) + (-7.448692e-04) * (q[i2] * q[i19])
            + (-5.387324e-04) * (q[i2] * q[i20]) + (1.664055e-03) * (q[i2] * q[i21]) + (-1.179484e-03) * (q[i2] * q[i22]) + (-1.580378e-02) * (q[i3] * q[i4])
            + (-2.793112e-03) * (q[i3] * q[i5]) + (5.600800e-03) * (q[i3] * q[i6]) + (-6.311696e-04) * (q[i3] * q[i7]) + (-2.588719e-03) * (q[i3] * q[i8])
            + (-4.320600e-04) * (q[i3] * q[i9]) + (-8.867219e-04) * (q[i3] * q[i10]) + (1.609103e-03) * (q[i3] * q[i11]) + (2.617932e-04) * (q[i3] * q[i12])
            + (2.837263e-04) * (q[i3] * q[i15]) + (-3.677597e-04) * (q[i3] * q[i16]) + (-6.588636e-04) * (q[i3] * q[i19]) + (1.403780e-05) * (q[i3] * q[i20])
            + (6.214166e-04) * (q[i3] * q[i21]) + (1.400097e-04) * (q[i3] * q[i22]) + (-8.075424e-03) * (q[i4] * q[i5]) + (7.475188e-04) * (q[i4] * q[i6])
            + (-1.749236e-02) * (q[i4] * q[i7]) + (-6.287449e-04) * (q[i4] * q[i8]) + (-2.675081e-03) * (q[i4] * q[i9]) + (1.261796e-02) * (q[i4] * q[i10])
            + (1.852015e-03) * (q[i4] * q[i11]) + (-3.495028e-04) * (q[i4] * q[i12]) + (-4.364390e-03) * (q[i4] * q[i15]) + (-2.788492e-04) * (q[i4] * q[i16])
            + (-1.337410e-03) * (q[i4] * q[i19]) + (-1.289726e-03) * (q[i4] * q[i20]) + (7.876661e-04) * (q[i4] * q[i21]) + (4.832829e-05) * (q[i4] * q[i22])
            + (1.352922e-03) * (q[i5] * q[i6]) + (1.951753e-03) * (q[i5] * q[i7]) + (1.637739e-04) * (q[i5] * q[i8]) + (-3.407700e-04) * (q[i5] * q[i9])
            + (2.248592e-03) * (q[i5] * q[i10]) + (-2.247199e-03) * (q[i5] * q[i11]) + (2.677469e-03) * (q[i5] * q[i12]) + (-1.592579e-03) * (q[i5] * q[i15])
            + (-1.026977e-03) * (q[i5] * q[i16]) + (-5.729034e-05) * (q[i5] * q[i19]) + (6.226303e-04) * (q[i5] * q[i20]) + (-6.467642e-04) * (q[i5] * q[i21])
            + (4.217495e-04) * (q[i5] * q[i22]) + (-1.015857e-02) * (q[i6] * q[i7]) + (-2.377159e-04) * (q[i6] * q[i8]) + (-5.646921e-04) * (q[i6] * q[i9])
            + (-1.639768e-03) * (q[i6] * q[i10]) + (2.123385e-04) * (q[i6] * q[i11]) + (7.670935e-05) * (q[i6] * q[i12]) + (-1.923527e-04) * (q[i6] * q[i15])
            + (-7.395194e-04) * (q[i6] * q[i16]) + (-2.937084e-04) * (q[i6] * q[i19]) + (1.270226e-03) * (q[i6] * q[i20]) + (-3.794178e-04) * (q[i6] * q[i21])
            + (-4.370185e-04) * (q[i6] * q[i22]) + (-1.668724e-03) * (q[i7] * q[i8]) + (-1.866107e-03) * (q[i7] * q[i9]) + (5.542534e-04) * (q[i7] * q[i10])
            + (-1.426140e-03) * (q[i7] * q[i11]) + (-1.341927e-03) * (q[i7] * q[i12]) + (-1.151666e-03) * (q[i7] * q[i15]) + (3.387974e-04) * (q[i7] * q[i16])
            + (-7.119287e-04) * (q[i7] * q[i19]) + (1.484588e-03) * (q[i7] * q[i20]) + (-8.684813e-04) * (q[i7] * q[i21]) + (5.826807e-04) * (q[i7] * q[i22])
            + (-2.056903e-04) * (q[i8] * q[i9]) + (-7.043732e-04) * (q[i8] * q[i10]) + (1.002922e-03) * (q[i8] * q[i11]) + (-9.243151e-04) * (q[i8] * q[i12])
            + (-2.146160e-03) * (q[i8] * q[i15]) + (7.556071e-04) * (q[i8] * q[i16]) + (2.228837e-04) * (q[i8] * q[i19]) + (3.807387e-04) * (q[i8] * q[i20])
            + (-4.667108e-04) * (q[i8] * q[i21]) + (3.449386e-05) * (q[i8] * q[i22]) + (-1.360802e-03) * (q[i9] * q[i10]) + (5.642974e-04) * (q[i9] * q[i11])
            + (7.066151e-04) * (q[i9] * q[i12]) + (5.905837e-04) * (q[i9] * q[i15]) + (-5.296953e-04) * (q[i9] * q[i16]) + (4.403208e-04) * (q[i9] * q[i19])
            + (-2.157064e-04) * (q[i9] * q[i20]) + (-2.141346e-04) * (q[i9] * q[i21]) + (-1.572557e-04) * (q[i9] * q[i22]) + (-9.983918e-04) * (q[i10] * q[i11])
            + (3.945032e-05) * (q[i10] * q[i12]) + (-1.115424e-03) * (q[i10] * q[i15]) + (-9.232939e-05) * (q[i10] * q[i16])
            + (-5.613018e-04) * (q[i10] * q[i19]) + (1.733368e-04) * (q[i10] * q[i20]) + (-6.546301e-05) * (q[i10] * q[i21])
            + (-7.665282e-05) * (q[i10] * q[i22]) + (5.435814e-04) * (q[i11] * q[i12]) + (-6.513442e-05) * (q[i11] * q[i15])
            + (-2.026858e-05) * (q[i11] * q[i16]) + (6.352125e-05) * (q[i11] * q[i19]) + (-5.394431e-04) * (q[i11] * q[i20])
            + (3.606199e-04) * (q[i11] * q[i21]) + (5.179043e-04) * (q[i11] * q[i22]) + (-7.006303e-05) * (q[i12] * q[i15]) + (1.941888e-04) * (q[i12] * q[i16])
            + (4.266644e-04) * (q[i12] * q[i19]) + (5.463806e-04) * (q[i12] * q[i20]) + (-3.924730e-04) * (q[i12] * q[i21])
            + (-2.773281e-04) * (q[i12] * q[i22]) + (2.186881e-04) * (q[i15] * q[i16]) + (-1.743718e-04) * (q[i15] * q[i19])
            + (5.877674e-04) * (q[i15] * q[i20]) + (1.723671e-04) * (q[i15] * q[i21]) + (-2.020610e-04) * (q[i15] * q[i22]) + (1.262092e-04) * (q[i16] * q[i19])
            + (8.947500e-05) * (q[i16] * q[i20]) + (1.137587e-04) * (q[i16] * q[i21]) + (3.327511e-04) * (q[i16] * q[i22]) + (-3.021009e-04) * (q[i19] * q[i20])
            + (-2.477809e-05) * (q[i19] * q[i21]) + (-1.043606e-04) * (q[i19] * q[i22]) + (2.473619e-04) * (q[i20] * q[i21])
            + (2.811799e-04) * (q[i20] * q[i22]) + (5.531782e-04) * (q[i21] * q[i22]);
   }

   public void getJQy2(double[] q, double[][] JQ)
   {
      JQ[2][i2] = (1.371481e-04) * (1) + (3.642564e-04) * ((2) * q[i2]) + (2.760604e-03) * (q[i0]) + (2.773411e-03) * (q[i1]) + (-2.528163e-02) * (q[i3])
            + (-2.511238e-02) * (q[i4]) + (7.861535e-02) * (q[i5]) + (-9.848709e-05) * (q[i6]) + (9.317147e-05) * (q[i7]) + (-7.650917e-06) * (q[i8])
            + (1.460264e-03) * (q[i9]) + (-1.435032e-03) * (q[i10]) + (-2.575455e-03) * (q[i11]) + (-2.486851e-03) * (q[i12]) + (5.981859e-03) * (q[i15])
            + (-6.084513e-03) * (q[i16]) + (-2.725771e-03) * (q[i19]) + (2.709918e-03) * (q[i20]) + (1.988143e-03) * (q[i21]) + (1.965185e-03) * (q[i22])
            + (2.148421e-03) * (q[i0] * q[i0]) + (-2.176803e-03) * (q[i1] * q[i1]) + (1.270791e-03) * ((2) * q[i0] * q[i2])
            + (-1.279295e-03) * ((2) * q[i1] * q[i2]) + (-9.223137e-06) * ((3) * q[i2] * q[i2]) + (1.510165e-04) * ((2) * q[i2] * q[i3])
            + (-1.471769e-04) * ((2) * q[i2] * q[i4]) + (-1.566487e-05) * ((2) * q[i2] * q[i5]) + (2.143538e-05) * ((2) * q[i2] * q[i6])
            + (2.407642e-05) * ((2) * q[i2] * q[i7]) + (-3.070144e-02) * ((2) * q[i2] * q[i8]) + (-9.801745e-04) * ((2) * q[i2] * q[i9])
            + (-9.776732e-04) * ((2) * q[i2] * q[i10]) + (8.314828e-05) * ((2) * q[i2] * q[i11]) + (-1.286932e-04) * ((2) * q[i2] * q[i12])
            + (-2.236801e-04) * ((2) * q[i2] * q[i15]) + (-2.076048e-04) * ((2) * q[i2] * q[i16]) + (-6.277662e-04) * ((2) * q[i2] * q[i19])
            + (-6.472335e-04) * ((2) * q[i2] * q[i20]) + (7.020029e-04) * ((2) * q[i2] * q[i21]) + (-7.202685e-04) * ((2) * q[i2] * q[i22])
            + (1.746452e-03) * (q[i3] * q[i3]) + (-1.715255e-03) * (q[i4] * q[i4]) + (-3.329987e-05) * (q[i5] * q[i5]) + (-3.326144e-04) * (q[i6] * q[i6])
            + (3.392017e-04) * (q[i7] * q[i7]) + (2.976896e-05) * (q[i8] * q[i8]) + (7.392941e-04) * (q[i9] * q[i9]) + (-7.362072e-04) * (q[i10] * q[i10])
            + (1.572873e-03) * (q[i11] * q[i11]) + (-1.692362e-03) * (q[i12] * q[i12]) + (2.563287e-03) * (q[i15] * q[i15])
            + (-2.586976e-03) * (q[i16] * q[i16]) + (3.508930e-06) * (q[i19] * q[i19]) + (-2.837281e-06) * (q[i20] * q[i20])
            + (3.209924e-04) * (q[i21] * q[i21]) + (-3.183774e-04) * (q[i22] * q[i22]) + (2.041900e-05) * (q[i0] * q[i1]) + (-4.830283e-03) * (q[i0] * q[i3])
            + (-3.669630e-04) * (q[i0] * q[i4]) + (-1.346757e-03) * (q[i0] * q[i5]) + (-8.850569e-03) * (q[i0] * q[i6]) + (1.509462e-03) * (q[i0] * q[i7])
            + (-2.242384e-03) * (q[i0] * q[i8]) + (-1.992049e-03) * (q[i0] * q[i9]) + (2.438050e-03) * (q[i0] * q[i10]) + (6.523406e-04) * (q[i0] * q[i11])
            + (8.603355e-04) * (q[i0] * q[i12]) + (9.454998e-05) * (q[i0] * q[i15]) + (1.685530e-04) * (q[i0] * q[i16]) + (-5.308325e-04) * (q[i0] * q[i19])
            + (-7.462804e-04) * (q[i0] * q[i20]) + (1.166162e-03) * (q[i0] * q[i21]) + (-1.663098e-03) * (q[i0] * q[i22]) + (3.625418e-04) * (q[i1] * q[i3])
            + (4.829084e-03) * (q[i1] * q[i4]) + (1.352036e-03) * (q[i1] * q[i5]) + (1.486211e-03) * (q[i1] * q[i6]) + (-8.826749e-03) * (q[i1] * q[i7])
            + (-2.254740e-03) * (q[i1] * q[i8]) + (2.449361e-03) * (q[i1] * q[i9]) + (-1.976517e-03) * (q[i1] * q[i10]) + (-8.432124e-04) * (q[i1] * q[i11])
            + (-6.395182e-04) * (q[i1] * q[i12]) + (1.760426e-04) * (q[i1] * q[i15]) + (1.001514e-04) * (q[i1] * q[i16]) + (-7.448692e-04) * (q[i1] * q[i19])
            + (-5.387324e-04) * (q[i1] * q[i20]) + (1.664055e-03) * (q[i1] * q[i21]) + (-1.179484e-03) * (q[i1] * q[i22]) + (8.644762e-06) * (q[i3] * q[i4])
            + (8.699480e-03) * (q[i3] * q[i5]) + (-5.663162e-03) * (q[i3] * q[i6]) + (6.717905e-04) * (q[i3] * q[i7]) + (-3.571442e-04) * (q[i3] * q[i8])
            + (2.309691e-03) * (q[i3] * q[i9]) + (-2.988670e-04) * (q[i3] * q[i10]) + (7.361710e-04) * (q[i3] * q[i11]) + (5.600232e-04) * (q[i3] * q[i12])
            + (9.927942e-04) * (q[i3] * q[i15]) + (-2.515278e-03) * (q[i3] * q[i16]) + (1.276545e-04) * (q[i3] * q[i19]) + (6.979532e-05) * (q[i3] * q[i20])
            + (-6.013862e-04) * (q[i3] * q[i21]) + (6.002039e-04) * (q[i3] * q[i22]) + (-8.697755e-03) * (q[i4] * q[i5]) + (6.851293e-04) * (q[i4] * q[i6])
            + (-5.699442e-03) * (q[i4] * q[i7]) + (-3.418379e-04) * (q[i4] * q[i8]) + (-3.234076e-04) * (q[i4] * q[i9]) + (2.270634e-03) * (q[i4] * q[i10])
            + (-5.738576e-04) * (q[i4] * q[i11]) + (-7.830249e-04) * (q[i4] * q[i12]) + (-2.498836e-03) * (q[i4] * q[i15]) + (1.011946e-03) * (q[i4] * q[i16])
            + (8.152563e-05) * (q[i4] * q[i19]) + (1.180434e-04) * (q[i4] * q[i20]) + (-5.871365e-04) * (q[i4] * q[i21]) + (5.962369e-04) * (q[i4] * q[i22])
            + (3.143789e-04) * (q[i5] * q[i6]) + (3.189315e-04) * (q[i5] * q[i7]) + (-1.046169e-02) * (q[i5] * q[i8]) + (5.292999e-03) * (q[i5] * q[i9])
            + (5.223961e-03) * (q[i5] * q[i10]) + (1.171327e-03) * (q[i5] * q[i11]) + (-1.285937e-03) * (q[i5] * q[i12]) + (-6.544554e-03) * (q[i5] * q[i15])
            + (-6.624475e-03) * (q[i5] * q[i16]) + (1.439647e-04) * (q[i5] * q[i19]) + (1.530245e-04) * (q[i5] * q[i20]) + (2.058515e-03) * (q[i5] * q[i21])
            + (-2.067730e-03) * (q[i5] * q[i22]) + (-1.321266e-06) * (q[i6] * q[i7]) + (9.315958e-03) * (q[i6] * q[i8]) + (1.297394e-03) * (q[i6] * q[i9])
            + (-4.481669e-04) * (q[i6] * q[i10]) + (-7.785610e-04) * (q[i6] * q[i11]) + (4.258846e-04) * (q[i6] * q[i12]) + (-1.313394e-03) * (q[i6] * q[i15])
            + (4.201134e-04) * (q[i6] * q[i16]) + (-3.549272e-04) * (q[i6] * q[i19]) + (7.624220e-04) * (q[i6] * q[i20]) + (4.757560e-04) * (q[i6] * q[i21])
            + (-7.125799e-04) * (q[i6] * q[i22]) + (-9.328918e-03) * (q[i7] * q[i8]) + (4.524172e-04) * (q[i7] * q[i9]) + (-1.291531e-03) * (q[i7] * q[i10])
            + (4.293556e-04) * (q[i7] * q[i11]) + (-8.119320e-04) * (q[i7] * q[i12]) + (-4.246611e-04) * (q[i7] * q[i15]) + (1.335803e-03) * (q[i7] * q[i16])
            + (-7.598168e-04) * (q[i7] * q[i19]) + (3.408859e-04) * (q[i7] * q[i20]) + (-7.253175e-04) * (q[i7] * q[i21]) + (4.696598e-04) * (q[i7] * q[i22])
            + (2.149226e-03) * (q[i8] * q[i9]) + (-2.143734e-03) * (q[i8] * q[i10]) + (4.147071e-03) * (q[i8] * q[i11]) + (4.154181e-03) * (q[i8] * q[i12])
            + (-9.564607e-04) * (q[i8] * q[i15]) + (9.360359e-04) * (q[i8] * q[i16]) + (1.023466e-03) * (q[i8] * q[i19]) + (-1.027806e-03) * (q[i8] * q[i20])
            + (4.401218e-04) * (q[i8] * q[i21]) + (4.437775e-04) * (q[i8] * q[i22]) + (2.516672e-06) * (q[i9] * q[i10]) + (3.620506e-06) * (q[i9] * q[i11])
            + (-1.799722e-04) * (q[i9] * q[i12]) + (4.049757e-04) * (q[i9] * q[i15]) + (-5.789794e-05) * (q[i9] * q[i16]) + (-2.610005e-04) * (q[i9] * q[i19])
            + (9.740003e-05) * (q[i9] * q[i20]) + (-4.083199e-04) * (q[i9] * q[i21]) + (-3.429924e-04) * (q[i9] * q[i22]) + (-1.926262e-04) * (q[i10] * q[i11])
            + (1.820085e-06) * (q[i10] * q[i12]) + (6.286513e-05) * (q[i10] * q[i15]) + (-4.083522e-04) * (q[i10] * q[i16])
            + (-1.011537e-04) * (q[i10] * q[i19]) + (2.619118e-04) * (q[i10] * q[i20]) + (-3.418190e-04) * (q[i10] * q[i21])
            + (-4.035359e-04) * (q[i10] * q[i22]) + (6.764880e-06) * (q[i11] * q[i12]) + (-2.362006e-03) * (q[i11] * q[i15])
            + (-5.016997e-04) * (q[i11] * q[i16]) + (6.703634e-04) * (q[i11] * q[i19]) + (-4.443000e-04) * (q[i11] * q[i20])
            + (9.345787e-04) * (q[i11] * q[i21]) + (1.211605e-03) * (q[i11] * q[i22]) + (-5.111858e-04) * (q[i12] * q[i15])
            + (-2.336272e-03) * (q[i12] * q[i16]) + (-4.444471e-04) * (q[i12] * q[i19]) + (6.501908e-04) * (q[i12] * q[i20])
            + (-1.204995e-03) * (q[i12] * q[i21]) + (-9.273983e-04) * (q[i12] * q[i22]) + (-4.108729e-06) * (q[i15] * q[i16])
            + (-3.347044e-04) * (q[i15] * q[i19]) + (1.346695e-03) * (q[i15] * q[i20]) + (7.605788e-04) * (q[i15] * q[i21]) + (3.999834e-05) * (q[i15] * q[i22])
            + (-1.355870e-03) * (q[i16] * q[i19]) + (3.529748e-04) * (q[i16] * q[i20]) + (4.025737e-05) * (q[i16] * q[i21]) + (7.495992e-04) * (q[i16] * q[i22])
            + (9.810085e-06) * (q[i19] * q[i20]) + (-6.799983e-04) * (q[i19] * q[i21]) + (-8.525243e-05) * (q[i19] * q[i22])
            + (-9.238074e-05) * (q[i20] * q[i21]) + (-6.741689e-04) * (q[i20] * q[i22]) + (-1.328435e-06) * (q[i21] * q[i22]);
   }

   public void getJQy3(double[] q, double[][] JQ)
   {
      JQ[2][i3] = (-2.286804e-03) * (1) + (-2.063380e-04) * ((2) * q[i3]) + (-1.082041e-01) * (q[i0]) + (1.089383e-02) * (q[i1]) + (-2.528163e-02) * (q[i2])
            + (-4.914697e-03) * (q[i4]) + (5.148517e-04) * (q[i5]) + (-1.325262e-02) * (q[i6]) + (1.496705e-02) * (q[i7]) + (2.286157e-03) * (q[i8])
            + (-6.041502e-03) * (q[i9]) + (5.639904e-03) * (q[i10]) + (-4.951514e-03) * (q[i11]) + (-2.852589e-03) * (q[i12]) + (1.701516e-04) * (q[i15])
            + (1.983357e-03) * (q[i16]) + (-3.041974e-03) * (q[i19]) + (-1.632224e-03) * (q[i20]) + (-2.054358e-03) * (q[i21]) + (1.763675e-03) * (q[i22])
            + (-2.877567e-03) * (q[i0] * q[i0]) + (1.966242e-03) * (q[i1] * q[i1]) + (1.510165e-04) * (q[i2] * q[i2]) + (7.799925e-03) * ((2) * q[i0] * q[i3])
            + (-2.959506e-03) * ((2) * q[i1] * q[i3]) + (1.746452e-03) * ((2) * q[i2] * q[i3]) + (9.593527e-04) * ((3) * q[i3] * q[i3])
            + (1.781395e-03) * ((2) * q[i3] * q[i4]) + (6.591798e-04) * ((2) * q[i3] * q[i5]) + (1.200284e-02) * ((2) * q[i3] * q[i6])
            + (-1.176808e-03) * ((2) * q[i3] * q[i7]) + (3.685066e-03) * ((2) * q[i3] * q[i8]) + (2.280216e-03) * ((2) * q[i3] * q[i9])
            + (-9.699651e-04) * ((2) * q[i3] * q[i10]) + (2.476817e-04) * ((2) * q[i3] * q[i11]) + (7.545558e-04) * ((2) * q[i3] * q[i12])
            + (3.150266e-04) * ((2) * q[i3] * q[i15]) + (1.793124e-04) * ((2) * q[i3] * q[i16]) + (1.215654e-03) * ((2) * q[i3] * q[i19])
            + (-4.721584e-04) * ((2) * q[i3] * q[i20]) + (-7.272412e-04) * ((2) * q[i3] * q[i21]) + (-6.555301e-04) * ((2) * q[i3] * q[i22])
            + (-1.787267e-03) * (q[i4] * q[i4]) + (-1.147605e-03) * (q[i5] * q[i5]) + (-6.037617e-04) * (q[i6] * q[i6]) + (-8.564796e-04) * (q[i7] * q[i7])
            + (-5.313342e-04) * (q[i8] * q[i8]) + (1.189732e-03) * (q[i9] * q[i9]) + (-1.155973e-03) * (q[i10] * q[i10]) + (1.445009e-03) * (q[i11] * q[i11])
            + (6.728427e-04) * (q[i12] * q[i12]) + (-2.638679e-05) * (q[i15] * q[i15]) + (4.245840e-04) * (q[i16] * q[i16])
            + (-3.874298e-04) * (q[i19] * q[i19]) + (-1.862724e-04) * (q[i20] * q[i20]) + (-2.119539e-04) * (q[i21] * q[i21])
            + (-5.071852e-04) * (q[i22] * q[i22]) + (-2.203863e-03) * (q[i0] * q[i1]) + (-4.830283e-03) * (q[i0] * q[i2]) + (1.577433e-02) * (q[i0] * q[i4])
            + (8.064864e-03) * (q[i0] * q[i5]) + (-1.744554e-02) * (q[i0] * q[i6]) + (7.487355e-04) * (q[i0] * q[i7]) + (-6.385192e-04) * (q[i0] * q[i8])
            + (1.275901e-02) * (q[i0] * q[i9]) + (-2.628810e-03) * (q[i0] * q[i10]) + (3.415507e-04) * (q[i0] * q[i11]) + (-1.892927e-03) * (q[i0] * q[i12])
            + (-2.736937e-04) * (q[i0] * q[i15]) + (-4.429769e-03) * (q[i0] * q[i16]) + (-1.283668e-03) * (q[i0] * q[i19]) + (-1.352818e-03) * (q[i0] * q[i20])
            + (-4.281597e-05) * (q[i0] * q[i21]) + (-7.913839e-04) * (q[i0] * q[i22]) + (3.625418e-04) * (q[i1] * q[i2]) + (-1.580378e-02) * (q[i1] * q[i4])
            + (-2.793112e-03) * (q[i1] * q[i5]) + (5.600800e-03) * (q[i1] * q[i6]) + (-6.311696e-04) * (q[i1] * q[i7]) + (-2.588719e-03) * (q[i1] * q[i8])
            + (-4.320600e-04) * (q[i1] * q[i9]) + (-8.867219e-04) * (q[i1] * q[i10]) + (1.609103e-03) * (q[i1] * q[i11]) + (2.617932e-04) * (q[i1] * q[i12])
            + (2.837263e-04) * (q[i1] * q[i15]) + (-3.677597e-04) * (q[i1] * q[i16]) + (-6.588636e-04) * (q[i1] * q[i19]) + (1.403780e-05) * (q[i1] * q[i20])
            + (6.214166e-04) * (q[i1] * q[i21]) + (1.400097e-04) * (q[i1] * q[i22]) + (8.644762e-06) * (q[i2] * q[i4]) + (8.699480e-03) * (q[i2] * q[i5])
            + (-5.663162e-03) * (q[i2] * q[i6]) + (6.717905e-04) * (q[i2] * q[i7]) + (-3.571442e-04) * (q[i2] * q[i8]) + (2.309691e-03) * (q[i2] * q[i9])
            + (-2.988670e-04) * (q[i2] * q[i10]) + (7.361710e-04) * (q[i2] * q[i11]) + (5.600232e-04) * (q[i2] * q[i12]) + (9.927942e-04) * (q[i2] * q[i15])
            + (-2.515278e-03) * (q[i2] * q[i16]) + (1.276545e-04) * (q[i2] * q[i19]) + (6.979532e-05) * (q[i2] * q[i20]) + (-6.013862e-04) * (q[i2] * q[i21])
            + (6.002039e-04) * (q[i2] * q[i22]) + (-1.528038e-05) * (q[i4] * q[i5]) + (-4.692084e-03) * (q[i4] * q[i6]) + (-4.669405e-03) * (q[i4] * q[i7])
            + (-2.187009e-03) * (q[i4] * q[i8]) + (-1.861590e-03) * (q[i4] * q[i9]) + (-1.869920e-03) * (q[i4] * q[i10]) + (1.272945e-03) * (q[i4] * q[i11])
            + (-1.294124e-03) * (q[i4] * q[i12]) + (-3.552360e-04) * (q[i4] * q[i15]) + (-3.478787e-04) * (q[i4] * q[i16]) + (1.436047e-03) * (q[i4] * q[i19])
            + (1.413824e-03) * (q[i4] * q[i20]) + (-1.400512e-03) * (q[i4] * q[i21]) + (1.412147e-03) * (q[i4] * q[i22]) + (-9.070857e-03) * (q[i5] * q[i6])
            + (1.810798e-03) * (q[i5] * q[i7]) + (-3.838593e-03) * (q[i5] * q[i8]) + (-1.723064e-03) * (q[i5] * q[i9]) + (-5.797220e-04) * (q[i5] * q[i10])
            + (-1.448399e-03) * (q[i5] * q[i11]) + (-1.207768e-03) * (q[i5] * q[i12]) + (-1.703704e-03) * (q[i5] * q[i15]) + (9.957671e-04) * (q[i5] * q[i16])
            + (-1.447273e-03) * (q[i5] * q[i19]) + (2.859915e-04) * (q[i5] * q[i20]) + (-1.096375e-03) * (q[i5] * q[i21]) + (-2.078576e-03) * (q[i5] * q[i22])
            + (1.678006e-04) * (q[i6] * q[i7]) + (5.232857e-04) * (q[i6] * q[i8]) + (1.313509e-03) * (q[i6] * q[i9]) + (-8.018915e-04) * (q[i6] * q[i10])
            + (-2.051880e-03) * (q[i6] * q[i11]) + (-2.301127e-04) * (q[i6] * q[i12]) + (-6.109421e-04) * (q[i6] * q[i15]) + (1.322964e-03) * (q[i6] * q[i16])
            + (5.983288e-04) * (q[i6] * q[i19]) + (-5.325464e-04) * (q[i6] * q[i20]) + (-1.193203e-03) * (q[i6] * q[i21]) + (-2.469359e-04) * (q[i6] * q[i22])
            + (8.731107e-04) * (q[i7] * q[i8]) + (-1.450256e-03) * (q[i7] * q[i9]) + (-2.740255e-03) * (q[i7] * q[i10]) + (5.274222e-04) * (q[i7] * q[i11])
            + (2.305726e-04) * (q[i7] * q[i12]) + (1.450320e-03) * (q[i7] * q[i15]) + (-7.606796e-04) * (q[i7] * q[i16]) + (2.271741e-04) * (q[i7] * q[i19])
            + (1.320482e-04) * (q[i7] * q[i20]) + (-7.691139e-04) * (q[i7] * q[i21]) + (1.421559e-03) * (q[i7] * q[i22]) + (-1.037956e-03) * (q[i8] * q[i9])
            + (2.746779e-04) * (q[i8] * q[i10]) + (1.522556e-03) * (q[i8] * q[i11]) + (-3.970226e-05) * (q[i8] * q[i12]) + (-2.196074e-03) * (q[i8] * q[i15])
            + (1.745948e-03) * (q[i8] * q[i16]) + (-1.824331e-04) * (q[i8] * q[i19]) + (9.336729e-05) * (q[i8] * q[i20]) + (2.121110e-04) * (q[i8] * q[i21])
            + (4.277614e-04) * (q[i8] * q[i22]) + (-8.110467e-04) * (q[i9] * q[i10]) + (7.615759e-04) * (q[i9] * q[i11]) + (6.839198e-04) * (q[i9] * q[i12])
            + (-2.617285e-04) * (q[i9] * q[i15]) + (-7.010736e-04) * (q[i9] * q[i16]) + (3.544132e-04) * (q[i9] * q[i19]) + (-6.354895e-04) * (q[i9] * q[i20])
            + (-2.459064e-04) * (q[i9] * q[i21]) + (-3.224698e-04) * (q[i9] * q[i22]) + (-1.161962e-03) * (q[i10] * q[i11]) + (7.719139e-05) * (q[i10] * q[i12])
            + (-4.930728e-04) * (q[i10] * q[i15]) + (4.747305e-04) * (q[i10] * q[i16]) + (2.506987e-04) * (q[i10] * q[i19]) + (6.488983e-04) * (q[i10] * q[i20])
            + (1.378146e-05) * (q[i10] * q[i21]) + (2.411005e-04) * (q[i10] * q[i22]) + (-7.578032e-04) * (q[i11] * q[i12])
            + (-9.027708e-04) * (q[i11] * q[i15]) + (-3.870236e-05) * (q[i11] * q[i16]) + (-1.712573e-04) * (q[i11] * q[i19])
            + (-8.019534e-04) * (q[i11] * q[i20]) + (-8.460064e-04) * (q[i11] * q[i21]) + (4.242010e-04) * (q[i11] * q[i22])
            + (1.071918e-03) * (q[i12] * q[i15]) + (-4.166864e-04) * (q[i12] * q[i16]) + (8.904234e-05) * (q[i12] * q[i19])
            + (-1.091851e-04) * (q[i12] * q[i20]) + (-2.769233e-05) * (q[i12] * q[i21]) + (-1.330506e-04) * (q[i12] * q[i22])
            + (-4.291182e-04) * (q[i15] * q[i16]) + (-1.090363e-03) * (q[i15] * q[i19]) + (-3.773423e-04) * (q[i15] * q[i20])
            + (-7.025679e-04) * (q[i15] * q[i21]) + (-7.839491e-04) * (q[i15] * q[i22]) + (-4.696699e-04) * (q[i16] * q[i19])
            + (1.838969e-04) * (q[i16] * q[i20]) + (-4.494096e-05) * (q[i16] * q[i21]) + (2.849656e-04) * (q[i16] * q[i22]) + (7.589383e-04) * (q[i19] * q[i20])
            + (7.291109e-05) * (q[i19] * q[i21]) + (2.006097e-04) * (q[i19] * q[i22]) + (-7.413924e-04) * (q[i20] * q[i21]) + (8.525907e-04) * (q[i20] * q[i22])
            + (1.085187e-04) * (q[i21] * q[i22]);
   }

   public void getJQy4(double[] q, double[][] JQ)
   {
      JQ[2][i4] = (2.272458e-03) * (1) + (-2.102849e-04) * ((2) * q[i4]) + (1.091409e-02) * (q[i0]) + (-1.076574e-01) * (q[i1]) + (-2.511238e-02) * (q[i2])
            + (-4.914697e-03) * (q[i3]) + (4.679285e-04) * (q[i5]) + (-1.500024e-02) * (q[i6]) + (1.316982e-02) * (q[i7]) + (-2.209310e-03) * (q[i8])
            + (-5.699578e-03) * (q[i9]) + (5.987383e-03) * (q[i10]) + (-2.811232e-03) * (q[i11]) + (-4.944247e-03) * (q[i12]) + (-1.936538e-03) * (q[i15])
            + (-1.070322e-04) * (q[i16]) + (1.584654e-03) * (q[i19]) + (3.021591e-03) * (q[i20]) + (1.778881e-03) * (q[i21]) + (-2.025126e-03) * (q[i22])
            + (-1.957202e-03) * (q[i0] * q[i0]) + (2.900782e-03) * (q[i1] * q[i1]) + (-1.471769e-04) * (q[i2] * q[i2]) + (1.781395e-03) * (q[i3] * q[i3])
            + (2.965714e-03) * ((2) * q[i0] * q[i4]) + (-7.743154e-03) * ((2) * q[i1] * q[i4]) + (-1.715255e-03) * ((2) * q[i2] * q[i4])
            + (-1.787267e-03) * ((2) * q[i3] * q[i4]) + (-9.534190e-04) * ((3) * q[i4] * q[i4]) + (-6.716760e-04) * ((2) * q[i4] * q[i5])
            + (-1.198562e-03) * ((2) * q[i4] * q[i6]) + (1.202639e-02) * ((2) * q[i4] * q[i7]) + (3.701925e-03) * ((2) * q[i4] * q[i8])
            + (-9.772404e-04) * ((2) * q[i4] * q[i9]) + (2.273967e-03) * ((2) * q[i4] * q[i10]) + (-7.619221e-04) * ((2) * q[i4] * q[i11])
            + (-2.208289e-04) * ((2) * q[i4] * q[i12]) + (1.659287e-04) * ((2) * q[i4] * q[i15]) + (2.896900e-04) * ((2) * q[i4] * q[i16])
            + (-4.610605e-04) * ((2) * q[i4] * q[i19]) + (1.217230e-03) * ((2) * q[i4] * q[i20]) + (6.508448e-04) * ((2) * q[i4] * q[i21])
            + (7.148945e-04) * ((2) * q[i4] * q[i22]) + (1.138473e-03) * (q[i5] * q[i5]) + (8.606353e-04) * (q[i6] * q[i6]) + (6.288183e-04) * (q[i7] * q[i7])
            + (5.051829e-04) * (q[i8] * q[i8]) + (1.167596e-03) * (q[i9] * q[i9]) + (-1.170908e-03) * (q[i10] * q[i10]) + (-6.828332e-04) * (q[i11] * q[i11])
            + (-1.457032e-03) * (q[i12] * q[i12]) + (-4.245545e-04) * (q[i15] * q[i15]) + (4.041991e-05) * (q[i16] * q[i16])
            + (1.838183e-04) * (q[i19] * q[i19]) + (3.691705e-04) * (q[i20] * q[i20]) + (5.086029e-04) * (q[i21] * q[i21]) + (2.075443e-04) * (q[i22] * q[i22])
            + (2.176018e-03) * (q[i0] * q[i1]) + (-3.669630e-04) * (q[i0] * q[i2]) + (1.577433e-02) * (q[i0] * q[i3]) + (2.766852e-03) * (q[i0] * q[i5])
            + (-6.211916e-04) * (q[i0] * q[i6]) + (5.575730e-03) * (q[i0] * q[i7]) + (-2.558086e-03) * (q[i0] * q[i8]) + (-9.015293e-04) * (q[i0] * q[i9])
            + (-4.478219e-04) * (q[i0] * q[i10]) + (-2.682264e-04) * (q[i0] * q[i11]) + (-1.597540e-03) * (q[i0] * q[i12]) + (-3.911409e-04) * (q[i0] * q[i15])
            + (2.809789e-04) * (q[i0] * q[i16]) + (-5.715602e-06) * (q[i0] * q[i19]) + (-6.549558e-04) * (q[i0] * q[i20]) + (-1.410528e-04) * (q[i0] * q[i21])
            + (-6.179272e-04) * (q[i0] * q[i22]) + (4.829084e-03) * (q[i1] * q[i2]) + (-1.580378e-02) * (q[i1] * q[i3]) + (-8.075424e-03) * (q[i1] * q[i5])
            + (7.475188e-04) * (q[i1] * q[i6]) + (-1.749236e-02) * (q[i1] * q[i7]) + (-6.287449e-04) * (q[i1] * q[i8]) + (-2.675081e-03) * (q[i1] * q[i9])
            + (1.261796e-02) * (q[i1] * q[i10]) + (1.852015e-03) * (q[i1] * q[i11]) + (-3.495028e-04) * (q[i1] * q[i12]) + (-4.364390e-03) * (q[i1] * q[i15])
            + (-2.788492e-04) * (q[i1] * q[i16]) + (-1.337410e-03) * (q[i1] * q[i19]) + (-1.289726e-03) * (q[i1] * q[i20]) + (7.876661e-04) * (q[i1] * q[i21])
            + (4.832829e-05) * (q[i1] * q[i22]) + (8.644762e-06) * (q[i2] * q[i3]) + (-8.697755e-03) * (q[i2] * q[i5]) + (6.851293e-04) * (q[i2] * q[i6])
            + (-5.699442e-03) * (q[i2] * q[i7]) + (-3.418379e-04) * (q[i2] * q[i8]) + (-3.234076e-04) * (q[i2] * q[i9]) + (2.270634e-03) * (q[i2] * q[i10])
            + (-5.738576e-04) * (q[i2] * q[i11]) + (-7.830249e-04) * (q[i2] * q[i12]) + (-2.498836e-03) * (q[i2] * q[i15]) + (1.011946e-03) * (q[i2] * q[i16])
            + (8.152563e-05) * (q[i2] * q[i19]) + (1.180434e-04) * (q[i2] * q[i20]) + (-5.871365e-04) * (q[i2] * q[i21]) + (5.962369e-04) * (q[i2] * q[i22])
            + (-1.528038e-05) * (q[i3] * q[i5]) + (-4.692084e-03) * (q[i3] * q[i6]) + (-4.669405e-03) * (q[i3] * q[i7]) + (-2.187009e-03) * (q[i3] * q[i8])
            + (-1.861590e-03) * (q[i3] * q[i9]) + (-1.869920e-03) * (q[i3] * q[i10]) + (1.272945e-03) * (q[i3] * q[i11]) + (-1.294124e-03) * (q[i3] * q[i12])
            + (-3.552360e-04) * (q[i3] * q[i15]) + (-3.478787e-04) * (q[i3] * q[i16]) + (1.436047e-03) * (q[i3] * q[i19]) + (1.413824e-03) * (q[i3] * q[i20])
            + (-1.400512e-03) * (q[i3] * q[i21]) + (1.412147e-03) * (q[i3] * q[i22]) + (1.822253e-03) * (q[i5] * q[i6]) + (-9.067011e-03) * (q[i5] * q[i7])
            + (-3.841741e-03) * (q[i5] * q[i8]) + (-5.934737e-04) * (q[i5] * q[i9]) + (-1.696389e-03) * (q[i5] * q[i10]) + (1.201051e-03) * (q[i5] * q[i11])
            + (1.420428e-03) * (q[i5] * q[i12]) + (9.956928e-04) * (q[i5] * q[i15]) + (-1.694699e-03) * (q[i5] * q[i16]) + (2.752440e-04) * (q[i5] * q[i19])
            + (-1.445958e-03) * (q[i5] * q[i20]) + (2.063944e-03) * (q[i5] * q[i21]) + (1.112460e-03) * (q[i5] * q[i22]) + (-1.860803e-04) * (q[i6] * q[i7])
            + (-8.465525e-04) * (q[i6] * q[i8]) + (2.745881e-03) * (q[i6] * q[i9]) + (1.436831e-03) * (q[i6] * q[i10]) + (2.222197e-04) * (q[i6] * q[i11])
            + (5.484947e-04) * (q[i6] * q[i12]) + (7.600604e-04) * (q[i6] * q[i15]) + (-1.475898e-03) * (q[i6] * q[i16]) + (-1.344769e-04) * (q[i6] * q[i19])
            + (-2.211296e-04) * (q[i6] * q[i20]) + (1.419563e-03) * (q[i6] * q[i21]) + (-7.808593e-04) * (q[i6] * q[i22]) + (-5.403478e-04) * (q[i7] * q[i8])
            + (8.076992e-04) * (q[i7] * q[i9]) + (-1.282671e-03) * (q[i7] * q[i10]) + (-2.309113e-04) * (q[i7] * q[i11]) + (-2.075818e-03) * (q[i7] * q[i12])
            + (-1.302155e-03) * (q[i7] * q[i15]) + (6.214028e-04) * (q[i7] * q[i16]) + (5.267683e-04) * (q[i7] * q[i19]) + (-6.071990e-04) * (q[i7] * q[i20])
            + (-2.688400e-04) * (q[i7] * q[i21]) + (-1.188431e-03) * (q[i7] * q[i22]) + (-2.740939e-04) * (q[i8] * q[i9]) + (1.022948e-03) * (q[i8] * q[i10])
            + (-3.970672e-05) * (q[i8] * q[i11]) + (1.552861e-03) * (q[i8] * q[i12]) + (-1.735568e-03) * (q[i8] * q[i15]) + (2.215468e-03) * (q[i8] * q[i16])
            + (-9.381795e-05) * (q[i8] * q[i19]) + (1.747690e-04) * (q[i8] * q[i20]) + (4.350437e-04) * (q[i8] * q[i21]) + (2.177448e-04) * (q[i8] * q[i22])
            + (8.112286e-04) * (q[i9] * q[i10]) + (7.421642e-05) * (q[i9] * q[i11]) + (-1.158998e-03) * (q[i9] * q[i12]) + (-4.751135e-04) * (q[i9] * q[i15])
            + (4.904249e-04) * (q[i9] * q[i16]) + (-6.228559e-04) * (q[i9] * q[i19]) + (-2.543523e-04) * (q[i9] * q[i20]) + (2.289478e-04) * (q[i9] * q[i21])
            + (-4.351189e-08) * (q[i9] * q[i22]) + (6.648302e-04) * (q[i10] * q[i11]) + (7.633476e-04) * (q[i10] * q[i12]) + (6.980060e-04) * (q[i10] * q[i15])
            + (2.553292e-04) * (q[i10] * q[i16]) + (6.283218e-04) * (q[i10] * q[i19]) + (-3.471171e-04) * (q[i10] * q[i20])
            + (-3.230970e-04) * (q[i10] * q[i21]) + (-2.538713e-04) * (q[i10] * q[i22]) + (7.642985e-04) * (q[i11] * q[i12])
            + (-4.090085e-04) * (q[i11] * q[i15]) + (1.070191e-03) * (q[i11] * q[i16]) + (-1.119821e-04) * (q[i11] * q[i19])
            + (9.979069e-05) * (q[i11] * q[i20]) + (1.227838e-04) * (q[i11] * q[i21]) + (2.703891e-05) * (q[i11] * q[i22]) + (-2.736292e-05) * (q[i12] * q[i15])
            + (-9.062632e-04) * (q[i12] * q[i16]) + (-7.938514e-04) * (q[i12] * q[i19]) + (-1.668851e-04) * (q[i12] * q[i20])
            + (-4.133740e-04) * (q[i12] * q[i21]) + (8.567602e-04) * (q[i12] * q[i22]) + (4.269459e-04) * (q[i15] * q[i16])
            + (-1.936850e-04) * (q[i15] * q[i19]) + (4.801148e-04) * (q[i15] * q[i20]) + (2.930438e-04) * (q[i15] * q[i21])
            + (-5.318361e-05) * (q[i15] * q[i22]) + (3.716815e-04) * (q[i16] * q[i19]) + (1.078294e-03) * (q[i16] * q[i20])
            + (-7.797041e-04) * (q[i16] * q[i21]) + (-7.098311e-04) * (q[i16] * q[i22]) + (-7.567033e-04) * (q[i19] * q[i20])
            + (8.571854e-04) * (q[i19] * q[i21]) + (-7.366768e-04) * (q[i19] * q[i22]) + (1.948996e-04) * (q[i20] * q[i21]) + (7.178784e-05) * (q[i20] * q[i22])
            + (-1.024263e-04) * (q[i21] * q[i22]);
   }

   public void getJQy5(double[] q, double[][] JQ)
   {
      JQ[2][i5] = (-5.235205e-05) * (1) + (-2.271242e-03) * ((2) * q[i5]) + (1.464916e-03) * (q[i0]) + (1.596455e-03) * (q[i1]) + (7.861535e-02) * (q[i2])
            + (5.148517e-04) * (q[i3]) + (4.679285e-04) * (q[i4]) + (-1.586590e-02) * (q[i6]) + (1.586884e-02) * (q[i7]) + (-4.203808e-05) * (q[i8])
            + (-5.528472e-03) * (q[i9]) + (5.516264e-03) * (q[i10]) + (-4.741971e-03) * (q[i11]) + (-4.693143e-03) * (q[i12]) + (1.072417e-03) * (q[i15])
            + (-1.156109e-03) * (q[i16]) + (-6.688310e-04) * (q[i19]) + (6.817503e-04) * (q[i20]) + (1.007175e-03) * (q[i21]) + (1.007538e-03) * (q[i22])
            + (-4.799782e-03) * (q[i0] * q[i0]) + (4.795869e-03) * (q[i1] * q[i1]) + (-1.566487e-05) * (q[i2] * q[i2]) + (6.591798e-04) * (q[i3] * q[i3])
            + (-6.716760e-04) * (q[i4] * q[i4]) + (-3.637586e-04) * ((2) * q[i0] * q[i5]) + (3.673930e-04) * ((2) * q[i1] * q[i5])
            + (-3.329987e-05) * ((2) * q[i2] * q[i5]) + (-1.147605e-03) * ((2) * q[i3] * q[i5]) + (1.138473e-03) * ((2) * q[i4] * q[i5])
            + (1.971547e-05) * ((3) * q[i5] * q[i5]) + (1.523657e-03) * ((2) * q[i5] * q[i6]) + (1.525435e-03) * ((2) * q[i5] * q[i7])
            + (2.662022e-03) * ((2) * q[i5] * q[i8]) + (-2.407482e-04) * ((2) * q[i5] * q[i9]) + (-2.454196e-04) * ((2) * q[i5] * q[i10])
            + (6.884126e-05) * ((2) * q[i5] * q[i11]) + (-9.857491e-05) * ((2) * q[i5] * q[i12]) + (-2.111881e-03) * ((2) * q[i5] * q[i15])
            + (-2.132539e-03) * ((2) * q[i5] * q[i16]) + (-1.718998e-03) * ((2) * q[i5] * q[i19]) + (-1.704766e-03) * ((2) * q[i5] * q[i20])
            + (-1.473637e-03) * ((2) * q[i5] * q[i21]) + (1.495075e-03) * ((2) * q[i5] * q[i22]) + (-3.680953e-03) * (q[i6] * q[i6])
            + (3.688227e-03) * (q[i7] * q[i7]) + (-1.978580e-07) * (q[i8] * q[i8]) + (8.578853e-04) * (q[i9] * q[i9]) + (-8.626645e-04) * (q[i10] * q[i10])
            + (3.701298e-04) * (q[i11] * q[i11]) + (-4.291887e-04) * (q[i12] * q[i12]) + (-5.541777e-04) * (q[i15] * q[i15])
            + (5.385013e-04) * (q[i16] * q[i16]) + (-8.537189e-04) * (q[i19] * q[i19]) + (8.604417e-04) * (q[i20] * q[i20])
            + (-4.984050e-04) * (q[i21] * q[i21]) + (5.071164e-04) * (q[i22] * q[i22]) + (-3.023284e-06) * (q[i0] * q[i1]) + (-1.346757e-03) * (q[i0] * q[i2])
            + (8.064864e-03) * (q[i0] * q[i3]) + (2.766852e-03) * (q[i0] * q[i4]) + (1.989546e-03) * (q[i0] * q[i6]) + (1.368311e-03) * (q[i0] * q[i7])
            + (1.635292e-04) * (q[i0] * q[i8]) + (2.272778e-03) * (q[i0] * q[i9]) + (-3.349013e-04) * (q[i0] * q[i10]) + (-2.742440e-03) * (q[i0] * q[i11])
            + (2.213568e-03) * (q[i0] * q[i12]) + (-1.032333e-03) * (q[i0] * q[i15]) + (-1.635496e-03) * (q[i0] * q[i16]) + (6.490070e-04) * (q[i0] * q[i19])
            + (-3.652971e-05) * (q[i0] * q[i20]) + (-4.205140e-04) * (q[i0] * q[i21]) + (6.642214e-04) * (q[i0] * q[i22]) + (1.352036e-03) * (q[i1] * q[i2])
            + (-2.793112e-03) * (q[i1] * q[i3]) + (-8.075424e-03) * (q[i1] * q[i4]) + (1.352922e-03) * (q[i1] * q[i6]) + (1.951753e-03) * (q[i1] * q[i7])
            + (1.637739e-04) * (q[i1] * q[i8]) + (-3.407700e-04) * (q[i1] * q[i9]) + (2.248592e-03) * (q[i1] * q[i10]) + (-2.247199e-03) * (q[i1] * q[i11])
            + (2.677469e-03) * (q[i1] * q[i12]) + (-1.592579e-03) * (q[i1] * q[i15]) + (-1.026977e-03) * (q[i1] * q[i16]) + (-5.729034e-05) * (q[i1] * q[i19])
            + (6.226303e-04) * (q[i1] * q[i20]) + (-6.467642e-04) * (q[i1] * q[i21]) + (4.217495e-04) * (q[i1] * q[i22]) + (8.699480e-03) * (q[i2] * q[i3])
            + (-8.697755e-03) * (q[i2] * q[i4]) + (3.143789e-04) * (q[i2] * q[i6]) + (3.189315e-04) * (q[i2] * q[i7]) + (-1.046169e-02) * (q[i2] * q[i8])
            + (5.292999e-03) * (q[i2] * q[i9]) + (5.223961e-03) * (q[i2] * q[i10]) + (1.171327e-03) * (q[i2] * q[i11]) + (-1.285937e-03) * (q[i2] * q[i12])
            + (-6.544554e-03) * (q[i2] * q[i15]) + (-6.624475e-03) * (q[i2] * q[i16]) + (1.439647e-04) * (q[i2] * q[i19]) + (1.530245e-04) * (q[i2] * q[i20])
            + (2.058515e-03) * (q[i2] * q[i21]) + (-2.067730e-03) * (q[i2] * q[i22]) + (-1.528038e-05) * (q[i3] * q[i4]) + (-9.070857e-03) * (q[i3] * q[i6])
            + (1.810798e-03) * (q[i3] * q[i7]) + (-3.838593e-03) * (q[i3] * q[i8]) + (-1.723064e-03) * (q[i3] * q[i9]) + (-5.797220e-04) * (q[i3] * q[i10])
            + (-1.448399e-03) * (q[i3] * q[i11]) + (-1.207768e-03) * (q[i3] * q[i12]) + (-1.703704e-03) * (q[i3] * q[i15]) + (9.957671e-04) * (q[i3] * q[i16])
            + (-1.447273e-03) * (q[i3] * q[i19]) + (2.859915e-04) * (q[i3] * q[i20]) + (-1.096375e-03) * (q[i3] * q[i21]) + (-2.078576e-03) * (q[i3] * q[i22])
            + (1.822253e-03) * (q[i4] * q[i6]) + (-9.067011e-03) * (q[i4] * q[i7]) + (-3.841741e-03) * (q[i4] * q[i8]) + (-5.934737e-04) * (q[i4] * q[i9])
            + (-1.696389e-03) * (q[i4] * q[i10]) + (1.201051e-03) * (q[i4] * q[i11]) + (1.420428e-03) * (q[i4] * q[i12]) + (9.956928e-04) * (q[i4] * q[i15])
            + (-1.694699e-03) * (q[i4] * q[i16]) + (2.752440e-04) * (q[i4] * q[i19]) + (-1.445958e-03) * (q[i4] * q[i20]) + (2.063944e-03) * (q[i4] * q[i21])
            + (1.112460e-03) * (q[i4] * q[i22]) + (1.324561e-05) * (q[i6] * q[i7]) + (1.535462e-03) * (q[i6] * q[i8]) + (2.186654e-03) * (q[i6] * q[i9])
            + (-2.477935e-03) * (q[i6] * q[i10]) + (4.159708e-04) * (q[i6] * q[i11]) + (-1.840572e-03) * (q[i6] * q[i12]) + (6.391077e-04) * (q[i6] * q[i15])
            + (-3.613898e-04) * (q[i6] * q[i16]) + (-9.828295e-04) * (q[i6] * q[i19]) + (-2.404216e-04) * (q[i6] * q[i20]) + (-2.500503e-04) * (q[i6] * q[i21])
            + (2.925324e-04) * (q[i6] * q[i22]) + (-1.558072e-03) * (q[i7] * q[i8]) + (2.477884e-03) * (q[i7] * q[i9]) + (-2.164833e-03) * (q[i7] * q[i10])
            + (-1.864377e-03) * (q[i7] * q[i11]) + (4.200824e-04) * (q[i7] * q[i12]) + (3.664296e-04) * (q[i7] * q[i15]) + (-6.492493e-04) * (q[i7] * q[i16])
            + (2.210279e-04) * (q[i7] * q[i19]) + (9.826121e-04) * (q[i7] * q[i20]) + (2.803336e-04) * (q[i7] * q[i21]) + (-2.304339e-04) * (q[i7] * q[i22])
            + (2.484928e-03) * (q[i8] * q[i9]) + (-2.482312e-03) * (q[i8] * q[i10]) + (2.648866e-03) * (q[i8] * q[i11]) + (2.660963e-03) * (q[i8] * q[i12])
            + (2.435700e-03) * (q[i8] * q[i15]) + (-2.476418e-03) * (q[i8] * q[i16]) + (1.505927e-04) * (q[i8] * q[i19]) + (-1.445753e-04) * (q[i8] * q[i20])
            + (-5.209222e-05) * (q[i8] * q[i21]) + (-4.250972e-05) * (q[i8] * q[i22]) + (-2.280379e-06) * (q[i9] * q[i10]) + (-9.996856e-04) * (q[i9] * q[i11])
            + (2.664558e-04) * (q[i9] * q[i12]) + (4.065399e-04) * (q[i9] * q[i15]) + (-8.526925e-04) * (q[i9] * q[i16]) + (6.817273e-04) * (q[i9] * q[i19])
            + (2.758747e-04) * (q[i9] * q[i20]) + (-5.875879e-04) * (q[i9] * q[i21]) + (-2.189059e-04) * (q[i9] * q[i22]) + (2.569866e-04) * (q[i10] * q[i11])
            + (-9.864429e-04) * (q[i10] * q[i12]) + (8.599808e-04) * (q[i10] * q[i15]) + (-4.068580e-04) * (q[i10] * q[i16])
            + (-2.698364e-04) * (q[i10] * q[i19]) + (-7.060816e-04) * (q[i10] * q[i20]) + (-2.235920e-04) * (q[i10] * q[i21])
            + (-5.826465e-04) * (q[i10] * q[i22]) + (7.873494e-06) * (q[i11] * q[i12]) + (-2.545878e-03) * (q[i11] * q[i15])
            + (-3.355602e-04) * (q[i11] * q[i16]) + (-3.721245e-04) * (q[i11] * q[i19]) + (3.566828e-04) * (q[i11] * q[i20])
            + (-5.596208e-04) * (q[i11] * q[i21]) + (-2.649668e-04) * (q[i11] * q[i22]) + (-3.395477e-04) * (q[i12] * q[i15])
            + (-2.540585e-03) * (q[i12] * q[i16]) + (3.623418e-04) * (q[i12] * q[i19]) + (-3.867339e-04) * (q[i12] * q[i20])
            + (2.657216e-04) * (q[i12] * q[i21]) + (5.715204e-04) * (q[i12] * q[i22]) + (-1.584383e-06) * (q[i15] * q[i16])
            + (-4.229998e-04) * (q[i15] * q[i19]) + (-4.258683e-04) * (q[i15] * q[i20]) + (6.511432e-04) * (q[i15] * q[i21])
            + (4.829936e-04) * (q[i15] * q[i22]) + (4.420311e-04) * (q[i16] * q[i19]) + (4.188070e-04) * (q[i16] * q[i20]) + (4.852346e-04) * (q[i16] * q[i21])
            + (6.692534e-04) * (q[i16] * q[i22]) + (2.482564e-06) * (q[i19] * q[i20]) + (6.058628e-04) * (q[i19] * q[i21]) + (9.127125e-05) * (q[i19] * q[i22])
            + (9.610038e-05) * (q[i20] * q[i21]) + (6.030247e-04) * (q[i20] * q[i22]) + (-2.869781e-06) * (q[i21] * q[i22]);
   }

   public void getJQy6(double[] q, double[][] JQ)
   {
      JQ[2][i6] = (1.286961e-01) * (1) + (-1.276062e-03) * ((2) * q[i6]) + (6.008783e-03) * (q[i0]) + (-6.207352e-04) * (q[i1]) + (-9.848709e-05) * (q[i2])
            + (-1.325262e-02) * (q[i3]) + (-1.500024e-02) * (q[i4]) + (-1.586590e-02) * (q[i5]) + (-3.116376e-03) * (q[i7]) + (9.608001e-04) * (q[i8])
            + (-7.158658e-03) * (q[i9]) + (5.731393e-03) * (q[i10]) + (2.074065e-03) * (q[i11]) + (9.387117e-04) * (q[i12]) + (2.506067e-03) * (q[i15])
            + (6.238223e-03) * (q[i16]) + (-5.073938e-04) * (q[i19]) + (-9.342910e-04) * (q[i20]) + (-2.525261e-04) * (q[i21]) + (3.042374e-04) * (q[i22])
            + (-2.172771e-02) * (q[i0] * q[i0]) + (-2.025674e-03) * (q[i1] * q[i1]) + (2.143538e-05) * (q[i2] * q[i2]) + (1.200284e-02) * (q[i3] * q[i3])
            + (-1.198562e-03) * (q[i4] * q[i4]) + (1.523657e-03) * (q[i5] * q[i5]) + (-4.331391e-03) * ((2) * q[i0] * q[i6])
            + (-1.108751e-03) * ((2) * q[i1] * q[i6]) + (-3.326144e-04) * ((2) * q[i2] * q[i6]) + (-6.037617e-04) * ((2) * q[i3] * q[i6])
            + (8.606353e-04) * ((2) * q[i4] * q[i6]) + (-3.680953e-03) * ((2) * q[i5] * q[i6]) + (-2.277140e-03) * ((3) * q[i6] * q[i6])
            + (-8.911033e-04) * ((2) * q[i6] * q[i7]) + (3.097585e-03) * ((2) * q[i6] * q[i8]) + (-6.738246e-04) * ((2) * q[i6] * q[i9])
            + (-4.169451e-04) * ((2) * q[i6] * q[i10]) + (-2.863740e-04) * ((2) * q[i6] * q[i11]) + (3.608378e-04) * ((2) * q[i6] * q[i12])
            + (-5.963555e-04) * ((2) * q[i6] * q[i15]) + (-4.876350e-04) * ((2) * q[i6] * q[i16]) + (-1.054806e-03) * ((2) * q[i6] * q[i19])
            + (-5.433964e-05) * ((2) * q[i6] * q[i20]) + (-6.000541e-04) * ((2) * q[i6] * q[i21]) + (1.043639e-03) * ((2) * q[i6] * q[i22])
            + (-8.986901e-04) * (q[i7] * q[i7]) + (1.549358e-03) * (q[i8] * q[i8]) + (-3.082180e-03) * (q[i9] * q[i9]) + (-1.009958e-04) * (q[i10] * q[i10])
            + (-8.192230e-04) * (q[i11] * q[i11]) + (1.147318e-05) * (q[i12] * q[i12]) + (4.019567e-05) * (q[i15] * q[i15]) + (8.658253e-04) * (q[i16] * q[i16])
            + (-6.359441e-04) * (q[i19] * q[i19]) + (3.957655e-04) * (q[i20] * q[i20]) + (1.102790e-04) * (q[i21] * q[i21]) + (6.934395e-04) * (q[i22] * q[i22])
            + (4.607573e-03) * (q[i0] * q[i1]) + (-8.850569e-03) * (q[i0] * q[i2]) + (-1.744554e-02) * (q[i0] * q[i3]) + (-6.211916e-04) * (q[i0] * q[i4])
            + (1.989546e-03) * (q[i0] * q[i5]) + (1.017963e-02) * (q[i0] * q[i7]) + (1.651949e-03) * (q[i0] * q[i8]) + (-5.399697e-04) * (q[i0] * q[i9])
            + (1.844408e-03) * (q[i0] * q[i10]) + (-1.342392e-03) * (q[i0] * q[i11]) + (-1.443070e-03) * (q[i0] * q[i12]) + (-3.174323e-04) * (q[i0] * q[i15])
            + (1.159499e-03) * (q[i0] * q[i16]) + (-1.498094e-03) * (q[i0] * q[i19]) + (7.182522e-04) * (q[i0] * q[i20]) + (5.736209e-04) * (q[i0] * q[i21])
            + (-8.699801e-04) * (q[i0] * q[i22]) + (1.486211e-03) * (q[i1] * q[i2]) + (5.600800e-03) * (q[i1] * q[i3]) + (7.475188e-04) * (q[i1] * q[i4])
            + (1.352922e-03) * (q[i1] * q[i5]) + (-1.015857e-02) * (q[i1] * q[i7]) + (-2.377159e-04) * (q[i1] * q[i8]) + (-5.646921e-04) * (q[i1] * q[i9])
            + (-1.639768e-03) * (q[i1] * q[i10]) + (2.123385e-04) * (q[i1] * q[i11]) + (7.670935e-05) * (q[i1] * q[i12]) + (-1.923527e-04) * (q[i1] * q[i15])
            + (-7.395194e-04) * (q[i1] * q[i16]) + (-2.937084e-04) * (q[i1] * q[i19]) + (1.270226e-03) * (q[i1] * q[i20]) + (-3.794178e-04) * (q[i1] * q[i21])
            + (-4.370185e-04) * (q[i1] * q[i22]) + (-5.663162e-03) * (q[i2] * q[i3]) + (6.851293e-04) * (q[i2] * q[i4]) + (3.143789e-04) * (q[i2] * q[i5])
            + (-1.321266e-06) * (q[i2] * q[i7]) + (9.315958e-03) * (q[i2] * q[i8]) + (1.297394e-03) * (q[i2] * q[i9]) + (-4.481669e-04) * (q[i2] * q[i10])
            + (-7.785610e-04) * (q[i2] * q[i11]) + (4.258846e-04) * (q[i2] * q[i12]) + (-1.313394e-03) * (q[i2] * q[i15]) + (4.201134e-04) * (q[i2] * q[i16])
            + (-3.549272e-04) * (q[i2] * q[i19]) + (7.624220e-04) * (q[i2] * q[i20]) + (4.757560e-04) * (q[i2] * q[i21]) + (-7.125799e-04) * (q[i2] * q[i22])
            + (-4.692084e-03) * (q[i3] * q[i4]) + (-9.070857e-03) * (q[i3] * q[i5]) + (1.678006e-04) * (q[i3] * q[i7]) + (5.232857e-04) * (q[i3] * q[i8])
            + (1.313509e-03) * (q[i3] * q[i9]) + (-8.018915e-04) * (q[i3] * q[i10]) + (-2.051880e-03) * (q[i3] * q[i11]) + (-2.301127e-04) * (q[i3] * q[i12])
            + (-6.109421e-04) * (q[i3] * q[i15]) + (1.322964e-03) * (q[i3] * q[i16]) + (5.983288e-04) * (q[i3] * q[i19]) + (-5.325464e-04) * (q[i3] * q[i20])
            + (-1.193203e-03) * (q[i3] * q[i21]) + (-2.469359e-04) * (q[i3] * q[i22]) + (1.822253e-03) * (q[i4] * q[i5]) + (-1.860803e-04) * (q[i4] * q[i7])
            + (-8.465525e-04) * (q[i4] * q[i8]) + (2.745881e-03) * (q[i4] * q[i9]) + (1.436831e-03) * (q[i4] * q[i10]) + (2.222197e-04) * (q[i4] * q[i11])
            + (5.484947e-04) * (q[i4] * q[i12]) + (7.600604e-04) * (q[i4] * q[i15]) + (-1.475898e-03) * (q[i4] * q[i16]) + (-1.344769e-04) * (q[i4] * q[i19])
            + (-2.211296e-04) * (q[i4] * q[i20]) + (1.419563e-03) * (q[i4] * q[i21]) + (-7.808593e-04) * (q[i4] * q[i22]) + (1.324561e-05) * (q[i5] * q[i7])
            + (1.535462e-03) * (q[i5] * q[i8]) + (2.186654e-03) * (q[i5] * q[i9]) + (-2.477935e-03) * (q[i5] * q[i10]) + (4.159708e-04) * (q[i5] * q[i11])
            + (-1.840572e-03) * (q[i5] * q[i12]) + (6.391077e-04) * (q[i5] * q[i15]) + (-3.613898e-04) * (q[i5] * q[i16]) + (-9.828295e-04) * (q[i5] * q[i19])
            + (-2.404216e-04) * (q[i5] * q[i20]) + (-2.500503e-04) * (q[i5] * q[i21]) + (2.925324e-04) * (q[i5] * q[i22]) + (-7.091101e-03) * (q[i7] * q[i8])
            + (5.066186e-04) * (q[i7] * q[i9]) + (5.131238e-04) * (q[i7] * q[i10]) + (1.122814e-03) * (q[i7] * q[i11]) + (-1.119746e-03) * (q[i7] * q[i12])
            + (6.383422e-04) * (q[i7] * q[i15]) + (6.339042e-04) * (q[i7] * q[i16]) + (2.586580e-04) * (q[i7] * q[i19]) + (2.497187e-04) * (q[i7] * q[i20])
            + (-2.658031e-04) * (q[i7] * q[i21]) + (2.470120e-04) * (q[i7] * q[i22]) + (-3.254459e-04) * (q[i8] * q[i9]) + (-1.566180e-03) * (q[i8] * q[i10])
            + (-8.828370e-04) * (q[i8] * q[i11]) + (-1.065433e-03) * (q[i8] * q[i12]) + (7.782932e-04) * (q[i8] * q[i15]) + (-7.840902e-04) * (q[i8] * q[i16])
            + (-5.448643e-05) * (q[i8] * q[i19]) + (-2.203541e-04) * (q[i8] * q[i20]) + (-4.420825e-04) * (q[i8] * q[i21]) + (-1.667029e-04) * (q[i8] * q[i22])
            + (-3.222617e-04) * (q[i9] * q[i10]) + (3.018579e-04) * (q[i9] * q[i11]) + (3.321788e-04) * (q[i9] * q[i12]) + (-4.425578e-04) * (q[i9] * q[i15])
            + (-8.358086e-05) * (q[i9] * q[i16]) + (1.934901e-04) * (q[i9] * q[i19]) + (-2.956798e-04) * (q[i9] * q[i20]) + (-2.630103e-04) * (q[i9] * q[i21])
            + (-2.043316e-04) * (q[i9] * q[i22]) + (3.046552e-05) * (q[i10] * q[i11]) + (-5.482079e-04) * (q[i10] * q[i12]) + (7.382468e-06) * (q[i10] * q[i15])
            + (2.798608e-04) * (q[i10] * q[i16]) + (-1.757855e-05) * (q[i10] * q[i19]) + (4.630654e-04) * (q[i10] * q[i20])
            + (-3.136931e-04) * (q[i10] * q[i21]) + (-3.502427e-06) * (q[i10] * q[i22]) + (-6.285653e-05) * (q[i11] * q[i12])
            + (4.187876e-04) * (q[i11] * q[i15]) + (3.723862e-04) * (q[i11] * q[i16]) + (-1.640755e-04) * (q[i11] * q[i19])
            + (-2.434679e-04) * (q[i11] * q[i20]) + (-2.957810e-04) * (q[i11] * q[i21]) + (-6.079118e-04) * (q[i11] * q[i22])
            + (3.061818e-05) * (q[i12] * q[i15]) + (-5.408422e-04) * (q[i12] * q[i16]) + (-9.819785e-04) * (q[i12] * q[i19])
            + (8.291197e-05) * (q[i12] * q[i20]) + (6.580356e-04) * (q[i12] * q[i21]) + (3.425389e-07) * (q[i12] * q[i22]) + (-2.138078e-05) * (q[i15] * q[i16])
            + (-6.097696e-05) * (q[i15] * q[i19]) + (-4.208301e-04) * (q[i15] * q[i20]) + (-3.006825e-04) * (q[i15] * q[i21])
            + (6.121058e-05) * (q[i15] * q[i22]) + (1.010537e-03) * (q[i16] * q[i19]) + (-2.576356e-04) * (q[i16] * q[i20]) + (1.806325e-04) * (q[i16] * q[i21])
            + (-7.857355e-04) * (q[i16] * q[i22]) + (-3.891080e-04) * (q[i19] * q[i20]) + (3.961905e-04) * (q[i19] * q[i21])
            + (-1.491139e-04) * (q[i19] * q[i22]) + (1.049546e-04) * (q[i20] * q[i21]) + (2.197945e-04) * (q[i20] * q[i22])
            + (3.274916e-04) * (q[i21] * q[i22]);
   }

   public void getJQy7(double[] q, double[][] JQ)
   {
      JQ[2][i7] = (1.279249e-01) * (1) + (-1.230130e-03) * ((2) * q[i7]) + (5.937328e-04) * (q[i0]) + (-6.060779e-03) * (q[i1]) + (9.317147e-05) * (q[i2])
            + (1.496705e-02) * (q[i3]) + (1.316982e-02) * (q[i4]) + (1.586884e-02) * (q[i5]) + (-3.116376e-03) * (q[i6]) + (9.322740e-04) * (q[i8])
            + (5.781920e-03) * (q[i9]) + (-7.079973e-03) * (q[i10]) + (-9.094966e-04) * (q[i11]) + (-2.033062e-03) * (q[i12]) + (6.173416e-03) * (q[i15])
            + (2.507167e-03) * (q[i16]) + (-9.229792e-04) * (q[i19]) + (-4.789234e-04) * (q[i20]) + (-3.041800e-04) * (q[i21]) + (2.464662e-04) * (q[i22])
            + (-2.006883e-03) * (q[i0] * q[i0]) + (-2.168669e-02) * (q[i1] * q[i1]) + (2.407642e-05) * (q[i2] * q[i2]) + (-1.176808e-03) * (q[i3] * q[i3])
            + (1.202639e-02) * (q[i4] * q[i4]) + (1.525435e-03) * (q[i5] * q[i5]) + (-8.911033e-04) * (q[i6] * q[i6]) + (1.095570e-03) * ((2) * q[i0] * q[i7])
            + (4.318104e-03) * ((2) * q[i1] * q[i7]) + (3.392017e-04) * ((2) * q[i2] * q[i7]) + (-8.564796e-04) * ((2) * q[i3] * q[i7])
            + (6.288183e-04) * ((2) * q[i4] * q[i7]) + (3.688227e-03) * ((2) * q[i5] * q[i7]) + (-8.986901e-04) * ((2) * q[i6] * q[i7])
            + (-2.277373e-03) * ((3) * q[i7] * q[i7]) + (3.125767e-03) * ((2) * q[i7] * q[i8]) + (-4.165958e-04) * ((2) * q[i7] * q[i9])
            + (-6.674327e-04) * ((2) * q[i7] * q[i10]) + (-3.531736e-04) * ((2) * q[i7] * q[i11]) + (3.045578e-04) * ((2) * q[i7] * q[i12])
            + (-4.750761e-04) * ((2) * q[i7] * q[i15]) + (-5.980053e-04) * ((2) * q[i7] * q[i16]) + (-4.924212e-05) * ((2) * q[i7] * q[i19])
            + (-1.049889e-03) * ((2) * q[i7] * q[i20]) + (-1.035763e-03) * ((2) * q[i7] * q[i21]) + (5.882127e-04) * ((2) * q[i7] * q[i22])
            + (1.578614e-03) * (q[i8] * q[i8]) + (-9.292043e-05) * (q[i9] * q[i9]) + (-3.045815e-03) * (q[i10] * q[i10]) + (5.161338e-06) * (q[i11] * q[i11])
            + (-8.323333e-04) * (q[i12] * q[i12]) + (8.646733e-04) * (q[i15] * q[i15]) + (3.910849e-05) * (q[i16] * q[i16]) + (3.946263e-04) * (q[i19] * q[i19])
            + (-6.289636e-04) * (q[i20] * q[i20]) + (6.938603e-04) * (q[i21] * q[i21]) + (1.140094e-04) * (q[i22] * q[i22]) + (4.585382e-03) * (q[i0] * q[i1])
            + (1.509462e-03) * (q[i0] * q[i2]) + (7.487355e-04) * (q[i0] * q[i3]) + (5.575730e-03) * (q[i0] * q[i4]) + (1.368311e-03) * (q[i0] * q[i5])
            + (1.017963e-02) * (q[i0] * q[i6]) + (2.109935e-04) * (q[i0] * q[i8]) + (1.644994e-03) * (q[i0] * q[i9]) + (5.517188e-04) * (q[i0] * q[i10])
            + (8.710504e-05) * (q[i0] * q[i11]) + (2.285169e-04) * (q[i0] * q[i12]) + (7.248336e-04) * (q[i0] * q[i15]) + (1.887704e-04) * (q[i0] * q[i16])
            + (-1.286637e-03) * (q[i0] * q[i19]) + (3.037250e-04) * (q[i0] * q[i20]) + (-4.422425e-04) * (q[i0] * q[i21]) + (-3.798863e-04) * (q[i0] * q[i22])
            + (-8.826749e-03) * (q[i1] * q[i2]) + (-6.311696e-04) * (q[i1] * q[i3]) + (-1.749236e-02) * (q[i1] * q[i4]) + (1.951753e-03) * (q[i1] * q[i5])
            + (-1.015857e-02) * (q[i1] * q[i6]) + (-1.668724e-03) * (q[i1] * q[i8]) + (-1.866107e-03) * (q[i1] * q[i9]) + (5.542534e-04) * (q[i1] * q[i10])
            + (-1.426140e-03) * (q[i1] * q[i11]) + (-1.341927e-03) * (q[i1] * q[i12]) + (-1.151666e-03) * (q[i1] * q[i15]) + (3.387974e-04) * (q[i1] * q[i16])
            + (-7.119287e-04) * (q[i1] * q[i19]) + (1.484588e-03) * (q[i1] * q[i20]) + (-8.684813e-04) * (q[i1] * q[i21]) + (5.826807e-04) * (q[i1] * q[i22])
            + (6.717905e-04) * (q[i2] * q[i3]) + (-5.699442e-03) * (q[i2] * q[i4]) + (3.189315e-04) * (q[i2] * q[i5]) + (-1.321266e-06) * (q[i2] * q[i6])
            + (-9.328918e-03) * (q[i2] * q[i8]) + (4.524172e-04) * (q[i2] * q[i9]) + (-1.291531e-03) * (q[i2] * q[i10]) + (4.293556e-04) * (q[i2] * q[i11])
            + (-8.119320e-04) * (q[i2] * q[i12]) + (-4.246611e-04) * (q[i2] * q[i15]) + (1.335803e-03) * (q[i2] * q[i16]) + (-7.598168e-04) * (q[i2] * q[i19])
            + (3.408859e-04) * (q[i2] * q[i20]) + (-7.253175e-04) * (q[i2] * q[i21]) + (4.696598e-04) * (q[i2] * q[i22]) + (-4.669405e-03) * (q[i3] * q[i4])
            + (1.810798e-03) * (q[i3] * q[i5]) + (1.678006e-04) * (q[i3] * q[i6]) + (8.731107e-04) * (q[i3] * q[i8]) + (-1.450256e-03) * (q[i3] * q[i9])
            + (-2.740255e-03) * (q[i3] * q[i10]) + (5.274222e-04) * (q[i3] * q[i11]) + (2.305726e-04) * (q[i3] * q[i12]) + (1.450320e-03) * (q[i3] * q[i15])
            + (-7.606796e-04) * (q[i3] * q[i16]) + (2.271741e-04) * (q[i3] * q[i19]) + (1.320482e-04) * (q[i3] * q[i20]) + (-7.691139e-04) * (q[i3] * q[i21])
            + (1.421559e-03) * (q[i3] * q[i22]) + (-9.067011e-03) * (q[i4] * q[i5]) + (-1.860803e-04) * (q[i4] * q[i6]) + (-5.403478e-04) * (q[i4] * q[i8])
            + (8.076992e-04) * (q[i4] * q[i9]) + (-1.282671e-03) * (q[i4] * q[i10]) + (-2.309113e-04) * (q[i4] * q[i11]) + (-2.075818e-03) * (q[i4] * q[i12])
            + (-1.302155e-03) * (q[i4] * q[i15]) + (6.214028e-04) * (q[i4] * q[i16]) + (5.267683e-04) * (q[i4] * q[i19]) + (-6.071990e-04) * (q[i4] * q[i20])
            + (-2.688400e-04) * (q[i4] * q[i21]) + (-1.188431e-03) * (q[i4] * q[i22]) + (1.324561e-05) * (q[i5] * q[i6]) + (-1.558072e-03) * (q[i5] * q[i8])
            + (2.477884e-03) * (q[i5] * q[i9]) + (-2.164833e-03) * (q[i5] * q[i10]) + (-1.864377e-03) * (q[i5] * q[i11]) + (4.200824e-04) * (q[i5] * q[i12])
            + (3.664296e-04) * (q[i5] * q[i15]) + (-6.492493e-04) * (q[i5] * q[i16]) + (2.210279e-04) * (q[i5] * q[i19]) + (9.826121e-04) * (q[i5] * q[i20])
            + (2.803336e-04) * (q[i5] * q[i21]) + (-2.304339e-04) * (q[i5] * q[i22]) + (-7.091101e-03) * (q[i6] * q[i8]) + (5.066186e-04) * (q[i6] * q[i9])
            + (5.131238e-04) * (q[i6] * q[i10]) + (1.122814e-03) * (q[i6] * q[i11]) + (-1.119746e-03) * (q[i6] * q[i12]) + (6.383422e-04) * (q[i6] * q[i15])
            + (6.339042e-04) * (q[i6] * q[i16]) + (2.586580e-04) * (q[i6] * q[i19]) + (2.497187e-04) * (q[i6] * q[i20]) + (-2.658031e-04) * (q[i6] * q[i21])
            + (2.470120e-04) * (q[i6] * q[i22]) + (-1.582883e-03) * (q[i8] * q[i9]) + (-3.117434e-04) * (q[i8] * q[i10]) + (1.097291e-03) * (q[i8] * q[i11])
            + (8.750085e-04) * (q[i8] * q[i12]) + (-7.901275e-04) * (q[i8] * q[i15]) + (7.769723e-04) * (q[i8] * q[i16]) + (-2.197780e-04) * (q[i8] * q[i19])
            + (-3.898825e-05) * (q[i8] * q[i20]) + (1.665584e-04) * (q[i8] * q[i21]) + (4.488181e-04) * (q[i8] * q[i22]) + (-3.334738e-04) * (q[i9] * q[i10])
            + (5.548080e-04) * (q[i9] * q[i11]) + (-2.256889e-05) * (q[i9] * q[i12]) + (2.860588e-04) * (q[i9] * q[i15]) + (1.170306e-05) * (q[i9] * q[i16])
            + (4.661810e-04) * (q[i9] * q[i19]) + (-2.378912e-05) * (q[i9] * q[i20]) + (2.457164e-06) * (q[i9] * q[i21]) + (3.049735e-04) * (q[i9] * q[i22])
            + (-3.371485e-04) * (q[i10] * q[i11]) + (-3.108714e-04) * (q[i10] * q[i12]) + (-7.809335e-05) * (q[i10] * q[i15])
            + (-4.451027e-04) * (q[i10] * q[i16]) + (-2.948753e-04) * (q[i10] * q[i19]) + (1.864975e-04) * (q[i10] * q[i20])
            + (2.096182e-04) * (q[i10] * q[i21]) + (2.627882e-04) * (q[i10] * q[i22]) + (-8.108071e-05) * (q[i11] * q[i12]) + (5.273374e-04) * (q[i11] * q[i15])
            + (-3.163262e-05) * (q[i11] * q[i16]) + (-7.524544e-05) * (q[i11] * q[i19]) + (9.860693e-04) * (q[i11] * q[i20])
            + (-1.315688e-06) * (q[i11] * q[i21]) + (6.556906e-04) * (q[i11] * q[i22]) + (-3.739448e-04) * (q[i12] * q[i15])
            + (-4.156540e-04) * (q[i12] * q[i16]) + (2.504454e-04) * (q[i12] * q[i19]) + (1.631307e-04) * (q[i12] * q[i20])
            + (-6.068143e-04) * (q[i12] * q[i21]) + (-2.887615e-04) * (q[i12] * q[i22]) + (-2.752398e-05) * (q[i15] * q[i16])
            + (-2.515684e-04) * (q[i15] * q[i19]) + (1.008449e-03) * (q[i15] * q[i20]) + (7.773756e-04) * (q[i15] * q[i21])
            + (-1.900134e-04) * (q[i15] * q[i22]) + (-4.187241e-04) * (q[i16] * q[i19]) + (-4.536671e-05) * (q[i16] * q[i20])
            + (-4.937533e-05) * (q[i16] * q[i21]) + (3.086174e-04) * (q[i16] * q[i22]) + (-3.844589e-04) * (q[i19] * q[i20])
            + (-2.231712e-04) * (q[i19] * q[i21]) + (-1.014804e-04) * (q[i19] * q[i22]) + (1.437196e-04) * (q[i20] * q[i21])
            + (-3.856598e-04) * (q[i20] * q[i22]) + (3.294441e-04) * (q[i21] * q[i22]);
   }

   public void getJQy8(double[] q, double[][] JQ)
   {
      JQ[2][i8] = (1.062151e-01) * (1) + (-2.514359e-03) * ((2) * q[i8]) + (1.223880e-04) * (q[i0]) + (-1.238358e-04) * (q[i1]) + (-7.650917e-06) * (q[i2])
            + (2.286157e-03) * (q[i3]) + (-2.209310e-03) * (q[i4]) + (-4.203808e-05) * (q[i5]) + (9.608001e-04) * (q[i6]) + (9.322740e-04) * (q[i7])
            + (2.419856e-03) * (q[i9]) + (2.390816e-03) * (q[i10]) + (-2.250525e-03) * (q[i11]) + (2.137495e-03) * (q[i12]) + (-9.332737e-03) * (q[i15])
            + (-9.383865e-03) * (q[i16]) + (-2.190352e-03) * (q[i19]) + (-2.185457e-03) * (q[i20]) + (-2.085760e-03) * (q[i21]) + (2.088335e-03) * (q[i22])
            + (1.524819e-03) * (q[i0] * q[i0]) + (1.522468e-03) * (q[i1] * q[i1]) + (-3.070144e-02) * (q[i2] * q[i2]) + (3.685066e-03) * (q[i3] * q[i3])
            + (3.701925e-03) * (q[i4] * q[i4]) + (2.662022e-03) * (q[i5] * q[i5]) + (3.097585e-03) * (q[i6] * q[i6]) + (3.125767e-03) * (q[i7] * q[i7])
            + (-1.652844e-03) * ((2) * q[i0] * q[i8]) + (1.667796e-03) * ((2) * q[i1] * q[i8]) + (2.976896e-05) * ((2) * q[i2] * q[i8])
            + (-5.313342e-04) * ((2) * q[i3] * q[i8]) + (5.051829e-04) * ((2) * q[i4] * q[i8]) + (-1.978580e-07) * ((2) * q[i5] * q[i8])
            + (1.549358e-03) * ((2) * q[i6] * q[i8]) + (1.578614e-03) * ((2) * q[i7] * q[i8]) + (-3.481256e-03) * ((3) * q[i8] * q[i8])
            + (8.673059e-04) * ((2) * q[i8] * q[i9]) + (8.654743e-04) * ((2) * q[i8] * q[i10]) + (7.407764e-04) * ((2) * q[i8] * q[i11])
            + (-7.625813e-04) * ((2) * q[i8] * q[i12]) + (4.076422e-05) * ((2) * q[i8] * q[i15]) + (5.069054e-05) * ((2) * q[i8] * q[i16])
            + (-1.430933e-05) * ((2) * q[i8] * q[i19]) + (-2.542223e-05) * ((2) * q[i8] * q[i20]) + (-5.663762e-04) * ((2) * q[i8] * q[i21])
            + (5.723610e-04) * ((2) * q[i8] * q[i22]) + (1.081172e-03) * (q[i9] * q[i9]) + (1.069369e-03) * (q[i10] * q[i10])
            + (1.676774e-03) * (q[i11] * q[i11]) + (1.706114e-03) * (q[i12] * q[i12]) + (6.044488e-05) * (q[i15] * q[i15]) + (7.343281e-05) * (q[i16] * q[i16])
            + (-4.900689e-04) * (q[i19] * q[i19]) + (-4.881255e-04) * (q[i20] * q[i20]) + (-7.177874e-04) * (q[i21] * q[i21])
            + (-7.183218e-04) * (q[i22] * q[i22]) + (3.606869e-03) * (q[i0] * q[i1]) + (-2.242384e-03) * (q[i0] * q[i2]) + (-6.385192e-04) * (q[i0] * q[i3])
            + (-2.558086e-03) * (q[i0] * q[i4]) + (1.635292e-04) * (q[i0] * q[i5]) + (1.651949e-03) * (q[i0] * q[i6]) + (2.109935e-04) * (q[i0] * q[i7])
            + (7.119842e-04) * (q[i0] * q[i9]) + (2.040536e-04) * (q[i0] * q[i10]) + (-9.079045e-04) * (q[i0] * q[i11]) + (9.702281e-04) * (q[i0] * q[i12])
            + (-7.466604e-04) * (q[i0] * q[i15]) + (2.160150e-03) * (q[i0] * q[i16]) + (-3.694046e-04) * (q[i0] * q[i19]) + (-2.317022e-04) * (q[i0] * q[i20])
            + (2.642182e-05) * (q[i0] * q[i21]) + (-4.572423e-04) * (q[i0] * q[i22]) + (-2.254740e-03) * (q[i1] * q[i2]) + (-2.588719e-03) * (q[i1] * q[i3])
            + (-6.287449e-04) * (q[i1] * q[i4]) + (1.637739e-04) * (q[i1] * q[i5]) + (-2.377159e-04) * (q[i1] * q[i6]) + (-1.668724e-03) * (q[i1] * q[i7])
            + (-2.056903e-04) * (q[i1] * q[i9]) + (-7.043732e-04) * (q[i1] * q[i10]) + (1.002922e-03) * (q[i1] * q[i11]) + (-9.243151e-04) * (q[i1] * q[i12])
            + (-2.146160e-03) * (q[i1] * q[i15]) + (7.556071e-04) * (q[i1] * q[i16]) + (2.228837e-04) * (q[i1] * q[i19]) + (3.807387e-04) * (q[i1] * q[i20])
            + (-4.667108e-04) * (q[i1] * q[i21]) + (3.449386e-05) * (q[i1] * q[i22]) + (-3.571442e-04) * (q[i2] * q[i3]) + (-3.418379e-04) * (q[i2] * q[i4])
            + (-1.046169e-02) * (q[i2] * q[i5]) + (9.315958e-03) * (q[i2] * q[i6]) + (-9.328918e-03) * (q[i2] * q[i7]) + (2.149226e-03) * (q[i2] * q[i9])
            + (-2.143734e-03) * (q[i2] * q[i10]) + (4.147071e-03) * (q[i2] * q[i11]) + (4.154181e-03) * (q[i2] * q[i12]) + (-9.564607e-04) * (q[i2] * q[i15])
            + (9.360359e-04) * (q[i2] * q[i16]) + (1.023466e-03) * (q[i2] * q[i19]) + (-1.027806e-03) * (q[i2] * q[i20]) + (4.401218e-04) * (q[i2] * q[i21])
            + (4.437775e-04) * (q[i2] * q[i22]) + (-2.187009e-03) * (q[i3] * q[i4]) + (-3.838593e-03) * (q[i3] * q[i5]) + (5.232857e-04) * (q[i3] * q[i6])
            + (8.731107e-04) * (q[i3] * q[i7]) + (-1.037956e-03) * (q[i3] * q[i9]) + (2.746779e-04) * (q[i3] * q[i10]) + (1.522556e-03) * (q[i3] * q[i11])
            + (-3.970226e-05) * (q[i3] * q[i12]) + (-2.196074e-03) * (q[i3] * q[i15]) + (1.745948e-03) * (q[i3] * q[i16]) + (-1.824331e-04) * (q[i3] * q[i19])
            + (9.336729e-05) * (q[i3] * q[i20]) + (2.121110e-04) * (q[i3] * q[i21]) + (4.277614e-04) * (q[i3] * q[i22]) + (-3.841741e-03) * (q[i4] * q[i5])
            + (-8.465525e-04) * (q[i4] * q[i6]) + (-5.403478e-04) * (q[i4] * q[i7]) + (-2.740939e-04) * (q[i4] * q[i9]) + (1.022948e-03) * (q[i4] * q[i10])
            + (-3.970672e-05) * (q[i4] * q[i11]) + (1.552861e-03) * (q[i4] * q[i12]) + (-1.735568e-03) * (q[i4] * q[i15]) + (2.215468e-03) * (q[i4] * q[i16])
            + (-9.381795e-05) * (q[i4] * q[i19]) + (1.747690e-04) * (q[i4] * q[i20]) + (4.350437e-04) * (q[i4] * q[i21]) + (2.177448e-04) * (q[i4] * q[i22])
            + (1.535462e-03) * (q[i5] * q[i6]) + (-1.558072e-03) * (q[i5] * q[i7]) + (2.484928e-03) * (q[i5] * q[i9]) + (-2.482312e-03) * (q[i5] * q[i10])
            + (2.648866e-03) * (q[i5] * q[i11]) + (2.660963e-03) * (q[i5] * q[i12]) + (2.435700e-03) * (q[i5] * q[i15]) + (-2.476418e-03) * (q[i5] * q[i16])
            + (1.505927e-04) * (q[i5] * q[i19]) + (-1.445753e-04) * (q[i5] * q[i20]) + (-5.209222e-05) * (q[i5] * q[i21]) + (-4.250972e-05) * (q[i5] * q[i22])
            + (-7.091101e-03) * (q[i6] * q[i7]) + (-3.254459e-04) * (q[i6] * q[i9]) + (-1.566180e-03) * (q[i6] * q[i10]) + (-8.828370e-04) * (q[i6] * q[i11])
            + (-1.065433e-03) * (q[i6] * q[i12]) + (7.782932e-04) * (q[i6] * q[i15]) + (-7.840902e-04) * (q[i6] * q[i16]) + (-5.448643e-05) * (q[i6] * q[i19])
            + (-2.203541e-04) * (q[i6] * q[i20]) + (-4.420825e-04) * (q[i6] * q[i21]) + (-1.667029e-04) * (q[i6] * q[i22]) + (-1.582883e-03) * (q[i7] * q[i9])
            + (-3.117434e-04) * (q[i7] * q[i10]) + (1.097291e-03) * (q[i7] * q[i11]) + (8.750085e-04) * (q[i7] * q[i12]) + (-7.901275e-04) * (q[i7] * q[i15])
            + (7.769723e-04) * (q[i7] * q[i16]) + (-2.197780e-04) * (q[i7] * q[i19]) + (-3.898825e-05) * (q[i7] * q[i20]) + (1.665584e-04) * (q[i7] * q[i21])
            + (4.488181e-04) * (q[i7] * q[i22]) + (-1.025028e-04) * (q[i9] * q[i10]) + (-4.504523e-04) * (q[i9] * q[i11]) + (-9.541455e-04) * (q[i9] * q[i12])
            + (-3.011738e-04) * (q[i9] * q[i15]) + (-1.652432e-04) * (q[i9] * q[i16]) + (-1.399525e-04) * (q[i9] * q[i19]) + (8.570201e-04) * (q[i9] * q[i20])
            + (9.173531e-05) * (q[i9] * q[i21]) + (-3.030985e-04) * (q[i9] * q[i22]) + (9.523357e-04) * (q[i10] * q[i11]) + (4.452269e-04) * (q[i10] * q[i12])
            + (-1.570818e-04) * (q[i10] * q[i15]) + (-3.077152e-04) * (q[i10] * q[i16]) + (8.625016e-04) * (q[i10] * q[i19])
            + (-1.244961e-04) * (q[i10] * q[i20]) + (2.951818e-04) * (q[i10] * q[i21]) + (-9.019046e-05) * (q[i10] * q[i22])
            + (-1.421608e-03) * (q[i11] * q[i12]) + (-9.667942e-04) * (q[i11] * q[i15]) + (2.049559e-04) * (q[i11] * q[i16])
            + (7.583491e-04) * (q[i11] * q[i19]) + (1.177453e-03) * (q[i11] * q[i20]) + (-3.791023e-04) * (q[i11] * q[i21])
            + (-2.098501e-04) * (q[i11] * q[i22]) + (-2.133103e-04) * (q[i12] * q[i15]) + (9.615786e-04) * (q[i12] * q[i16])
            + (-1.190358e-03) * (q[i12] * q[i19]) + (-7.548895e-04) * (q[i12] * q[i20]) + (-2.082552e-04) * (q[i12] * q[i21])
            + (-3.871480e-04) * (q[i12] * q[i22]) + (-2.463027e-04) * (q[i15] * q[i16]) + (1.797364e-04) * (q[i15] * q[i19])
            + (-7.647453e-04) * (q[i15] * q[i20]) + (-1.503288e-03) * (q[i15] * q[i21]) + (2.291151e-04) * (q[i15] * q[i22])
            + (-7.659774e-04) * (q[i16] * q[i19]) + (1.855496e-04) * (q[i16] * q[i20]) + (-2.341260e-04) * (q[i16] * q[i21])
            + (1.516622e-03) * (q[i16] * q[i22]) + (6.328515e-04) * (q[i19] * q[i20]) + (-1.120134e-03) * (q[i19] * q[i21])
            + (-4.248339e-04) * (q[i19] * q[i22]) + (4.221432e-04) * (q[i20] * q[i21]) + (1.111237e-03) * (q[i20] * q[i22])
            + (-4.557013e-04) * (q[i21] * q[i22]);
   }

   public void getJQy9(double[] q, double[][] JQ)
   {
      JQ[2][i9] = (5.879427e-02) * (1) + (-3.273431e-03) * ((2) * q[i9]) + (-1.577200e-03) * (q[i0]) + (2.346357e-03) * (q[i1]) + (1.460264e-03) * (q[i2])
            + (-6.041502e-03) * (q[i3]) + (-5.699578e-03) * (q[i4]) + (-5.528472e-03) * (q[i5]) + (-7.158658e-03) * (q[i6]) + (5.781920e-03) * (q[i7])
            + (2.419856e-03) * (q[i8]) + (4.848303e-03) * (q[i10]) + (-2.832155e-04) * (q[i11]) + (3.941600e-04) * (q[i12]) + (1.616846e-03) * (q[i15])
            + (2.078583e-03) * (q[i16]) + (7.876256e-05) * (q[i19]) + (1.164829e-03) * (q[i20]) + (-7.003847e-04) * (q[i21]) + (4.554292e-04) * (q[i22])
            + (-3.620341e-03) * (q[i0] * q[i0]) + (-4.279621e-04) * (q[i1] * q[i1]) + (-9.801745e-04) * (q[i2] * q[i2]) + (2.280216e-03) * (q[i3] * q[i3])
            + (-9.772404e-04) * (q[i4] * q[i4]) + (-2.407482e-04) * (q[i5] * q[i5]) + (-6.738246e-04) * (q[i6] * q[i6]) + (-4.165958e-04) * (q[i7] * q[i7])
            + (8.673059e-04) * (q[i8] * q[i8]) + (1.141431e-04) * ((2) * q[i0] * q[i9]) + (-5.281776e-04) * ((2) * q[i1] * q[i9])
            + (7.392941e-04) * ((2) * q[i2] * q[i9]) + (1.189732e-03) * ((2) * q[i3] * q[i9]) + (1.167596e-03) * ((2) * q[i4] * q[i9])
            + (8.578853e-04) * ((2) * q[i5] * q[i9]) + (-3.082180e-03) * ((2) * q[i6] * q[i9]) + (-9.292043e-05) * ((2) * q[i7] * q[i9])
            + (1.081172e-03) * ((2) * q[i8] * q[i9]) + (-1.031580e-03) * ((3) * q[i9] * q[i9]) + (-3.716021e-04) * ((2) * q[i9] * q[i10])
            + (1.059143e-04) * ((2) * q[i9] * q[i11]) + (1.338060e-04) * ((2) * q[i9] * q[i12]) + (-2.008965e-04) * ((2) * q[i9] * q[i15])
            + (-2.215306e-04) * ((2) * q[i9] * q[i16]) + (1.068099e-04) * ((2) * q[i9] * q[i19]) + (-2.559619e-04) * ((2) * q[i9] * q[i20])
            + (5.934105e-06) * ((2) * q[i9] * q[i21]) + (-2.138896e-04) * ((2) * q[i9] * q[i22]) + (-3.751170e-04) * (q[i10] * q[i10])
            + (-1.130603e-04) * (q[i11] * q[i11]) + (6.798928e-05) * (q[i12] * q[i12]) + (-9.121269e-06) * (q[i15] * q[i15])
            + (-1.646033e-04) * (q[i16] * q[i16]) + (-1.551092e-04) * (q[i19] * q[i19]) + (-1.402449e-04) * (q[i20] * q[i20])
            + (-3.988740e-05) * (q[i21] * q[i21]) + (1.021671e-04) * (q[i22] * q[i22]) + (3.022323e-04) * (q[i0] * q[i1]) + (-1.992049e-03) * (q[i0] * q[i2])
            + (1.275901e-02) * (q[i0] * q[i3]) + (-9.015293e-04) * (q[i0] * q[i4]) + (2.272778e-03) * (q[i0] * q[i5]) + (-5.399697e-04) * (q[i0] * q[i6])
            + (1.644994e-03) * (q[i0] * q[i7]) + (7.119842e-04) * (q[i0] * q[i8]) + (1.360263e-03) * (q[i0] * q[i10]) + (4.597602e-05) * (q[i0] * q[i11])
            + (-1.002433e-03) * (q[i0] * q[i12]) + (1.017340e-04) * (q[i0] * q[i15]) + (1.127961e-03) * (q[i0] * q[i16]) + (-1.799168e-04) * (q[i0] * q[i19])
            + (5.617884e-04) * (q[i0] * q[i20]) + (-7.818176e-05) * (q[i0] * q[i21]) + (-6.660250e-05) * (q[i0] * q[i22]) + (2.449361e-03) * (q[i1] * q[i2])
            + (-4.320600e-04) * (q[i1] * q[i3]) + (-2.675081e-03) * (q[i1] * q[i4]) + (-3.407700e-04) * (q[i1] * q[i5]) + (-5.646921e-04) * (q[i1] * q[i6])
            + (-1.866107e-03) * (q[i1] * q[i7]) + (-2.056903e-04) * (q[i1] * q[i8]) + (-1.360802e-03) * (q[i1] * q[i10]) + (5.642974e-04) * (q[i1] * q[i11])
            + (7.066151e-04) * (q[i1] * q[i12]) + (5.905837e-04) * (q[i1] * q[i15]) + (-5.296953e-04) * (q[i1] * q[i16]) + (4.403208e-04) * (q[i1] * q[i19])
            + (-2.157064e-04) * (q[i1] * q[i20]) + (-2.141346e-04) * (q[i1] * q[i21]) + (-1.572557e-04) * (q[i1] * q[i22]) + (2.309691e-03) * (q[i2] * q[i3])
            + (-3.234076e-04) * (q[i2] * q[i4]) + (5.292999e-03) * (q[i2] * q[i5]) + (1.297394e-03) * (q[i2] * q[i6]) + (4.524172e-04) * (q[i2] * q[i7])
            + (2.149226e-03) * (q[i2] * q[i8]) + (2.516672e-06) * (q[i2] * q[i10]) + (3.620506e-06) * (q[i2] * q[i11]) + (-1.799722e-04) * (q[i2] * q[i12])
            + (4.049757e-04) * (q[i2] * q[i15]) + (-5.789794e-05) * (q[i2] * q[i16]) + (-2.610005e-04) * (q[i2] * q[i19]) + (9.740003e-05) * (q[i2] * q[i20])
            + (-4.083199e-04) * (q[i2] * q[i21]) + (-3.429924e-04) * (q[i2] * q[i22]) + (-1.861590e-03) * (q[i3] * q[i4]) + (-1.723064e-03) * (q[i3] * q[i5])
            + (1.313509e-03) * (q[i3] * q[i6]) + (-1.450256e-03) * (q[i3] * q[i7]) + (-1.037956e-03) * (q[i3] * q[i8]) + (-8.110467e-04) * (q[i3] * q[i10])
            + (7.615759e-04) * (q[i3] * q[i11]) + (6.839198e-04) * (q[i3] * q[i12]) + (-2.617285e-04) * (q[i3] * q[i15]) + (-7.010736e-04) * (q[i3] * q[i16])
            + (3.544132e-04) * (q[i3] * q[i19]) + (-6.354895e-04) * (q[i3] * q[i20]) + (-2.459064e-04) * (q[i3] * q[i21]) + (-3.224698e-04) * (q[i3] * q[i22])
            + (-5.934737e-04) * (q[i4] * q[i5]) + (2.745881e-03) * (q[i4] * q[i6]) + (8.076992e-04) * (q[i4] * q[i7]) + (-2.740939e-04) * (q[i4] * q[i8])
            + (8.112286e-04) * (q[i4] * q[i10]) + (7.421642e-05) * (q[i4] * q[i11]) + (-1.158998e-03) * (q[i4] * q[i12]) + (-4.751135e-04) * (q[i4] * q[i15])
            + (4.904249e-04) * (q[i4] * q[i16]) + (-6.228559e-04) * (q[i4] * q[i19]) + (-2.543523e-04) * (q[i4] * q[i20]) + (2.289478e-04) * (q[i4] * q[i21])
            + (-4.351189e-08) * (q[i4] * q[i22]) + (2.186654e-03) * (q[i5] * q[i6]) + (2.477884e-03) * (q[i5] * q[i7]) + (2.484928e-03) * (q[i5] * q[i8])
            + (-2.280379e-06) * (q[i5] * q[i10]) + (-9.996856e-04) * (q[i5] * q[i11]) + (2.664558e-04) * (q[i5] * q[i12]) + (4.065399e-04) * (q[i5] * q[i15])
            + (-8.526925e-04) * (q[i5] * q[i16]) + (6.817273e-04) * (q[i5] * q[i19]) + (2.758747e-04) * (q[i5] * q[i20]) + (-5.875879e-04) * (q[i5] * q[i21])
            + (-2.189059e-04) * (q[i5] * q[i22]) + (5.066186e-04) * (q[i6] * q[i7]) + (-3.254459e-04) * (q[i6] * q[i8]) + (-3.222617e-04) * (q[i6] * q[i10])
            + (3.018579e-04) * (q[i6] * q[i11]) + (3.321788e-04) * (q[i6] * q[i12]) + (-4.425578e-04) * (q[i6] * q[i15]) + (-8.358086e-05) * (q[i6] * q[i16])
            + (1.934901e-04) * (q[i6] * q[i19]) + (-2.956798e-04) * (q[i6] * q[i20]) + (-2.630103e-04) * (q[i6] * q[i21]) + (-2.043316e-04) * (q[i6] * q[i22])
            + (-1.582883e-03) * (q[i7] * q[i8]) + (-3.334738e-04) * (q[i7] * q[i10]) + (5.548080e-04) * (q[i7] * q[i11]) + (-2.256889e-05) * (q[i7] * q[i12])
            + (2.860588e-04) * (q[i7] * q[i15]) + (1.170306e-05) * (q[i7] * q[i16]) + (4.661810e-04) * (q[i7] * q[i19]) + (-2.378912e-05) * (q[i7] * q[i20])
            + (2.457164e-06) * (q[i7] * q[i21]) + (3.049735e-04) * (q[i7] * q[i22]) + (-1.025028e-04) * (q[i8] * q[i10]) + (-4.504523e-04) * (q[i8] * q[i11])
            + (-9.541455e-04) * (q[i8] * q[i12]) + (-3.011738e-04) * (q[i8] * q[i15]) + (-1.652432e-04) * (q[i8] * q[i16]) + (-1.399525e-04) * (q[i8] * q[i19])
            + (8.570201e-04) * (q[i8] * q[i20]) + (9.173531e-05) * (q[i8] * q[i21]) + (-3.030985e-04) * (q[i8] * q[i22]) + (1.670375e-04) * (q[i10] * q[i11])
            + (-1.631895e-04) * (q[i10] * q[i12]) + (2.964690e-04) * (q[i10] * q[i15]) + (2.970809e-04) * (q[i10] * q[i16]) + (3.917550e-05) * (q[i10] * q[i19])
            + (4.137510e-05) * (q[i10] * q[i20]) + (7.523850e-05) * (q[i10] * q[i21]) + (-7.457916e-05) * (q[i10] * q[i22]) + (2.691510e-04) * (q[i11] * q[i12])
            + (1.353202e-04) * (q[i11] * q[i15]) + (-1.070767e-04) * (q[i11] * q[i16]) + (1.100914e-04) * (q[i11] * q[i19]) + (5.449230e-04) * (q[i11] * q[i20])
            + (-5.803486e-05) * (q[i11] * q[i21]) + (8.276334e-05) * (q[i11] * q[i22]) + (2.551192e-04) * (q[i12] * q[i15]) + (2.425242e-04) * (q[i12] * q[i16])
            + (1.687963e-04) * (q[i12] * q[i19]) + (3.931066e-04) * (q[i12] * q[i20]) + (-2.334586e-05) * (q[i12] * q[i21]) + (3.840462e-07) * (q[i12] * q[i22])
            + (2.926568e-04) * (q[i15] * q[i16]) + (-1.205072e-04) * (q[i15] * q[i19]) + (2.208531e-04) * (q[i15] * q[i20]) + (5.189756e-06) * (q[i15] * q[i21])
            + (-3.313817e-05) * (q[i15] * q[i22]) + (4.823402e-04) * (q[i16] * q[i19]) + (2.434512e-04) * (q[i16] * q[i20])
            + (-8.190503e-05) * (q[i16] * q[i21]) + (-1.549280e-04) * (q[i16] * q[i22]) + (-1.338109e-04) * (q[i19] * q[i20])
            + (1.220038e-04) * (q[i19] * q[i21]) + (1.010255e-04) * (q[i19] * q[i22]) + (1.195072e-04) * (q[i20] * q[i21]) + (-1.742876e-04) * (q[i20] * q[i22])
            + (6.391691e-05) * (q[i21] * q[i22]);
   }

   public void getJQy10(double[] q, double[][] JQ)
   {
      JQ[2][i10] = (5.814686e-02) * (1) + (-3.258967e-03) * ((2) * q[i10]) + (-2.364382e-03) * (q[i0]) + (1.575809e-03) * (q[i1]) + (-1.435032e-03) * (q[i2])
            + (5.639904e-03) * (q[i3]) + (5.987383e-03) * (q[i4]) + (5.516264e-03) * (q[i5]) + (5.731393e-03) * (q[i6]) + (-7.079973e-03) * (q[i7])
            + (2.390816e-03) * (q[i8]) + (4.848303e-03) * (q[i9]) + (-3.697813e-04) * (q[i11]) + (3.171618e-04) * (q[i12]) + (2.032891e-03) * (q[i15])
            + (1.586893e-03) * (q[i16]) + (1.160841e-03) * (q[i19]) + (6.927503e-05) * (q[i20]) + (-4.558118e-04) * (q[i21]) + (6.953218e-04) * (q[i22])
            + (-4.255330e-04) * (q[i0] * q[i0]) + (-3.573820e-03) * (q[i1] * q[i1]) + (-9.776732e-04) * (q[i2] * q[i2]) + (-9.699651e-04) * (q[i3] * q[i3])
            + (2.273967e-03) * (q[i4] * q[i4]) + (-2.454196e-04) * (q[i5] * q[i5]) + (-4.169451e-04) * (q[i6] * q[i6]) + (-6.674327e-04) * (q[i7] * q[i7])
            + (8.654743e-04) * (q[i8] * q[i8]) + (-3.716021e-04) * (q[i9] * q[i9]) + (5.209430e-04) * ((2) * q[i0] * q[i10])
            + (-1.151267e-04) * ((2) * q[i1] * q[i10]) + (-7.362072e-04) * ((2) * q[i2] * q[i10]) + (-1.155973e-03) * ((2) * q[i3] * q[i10])
            + (-1.170908e-03) * ((2) * q[i4] * q[i10]) + (-8.626645e-04) * ((2) * q[i5] * q[i10]) + (-1.009958e-04) * ((2) * q[i6] * q[i10])
            + (-3.045815e-03) * ((2) * q[i7] * q[i10]) + (1.069369e-03) * ((2) * q[i8] * q[i10]) + (-3.751170e-04) * ((2) * q[i9] * q[i10])
            + (-1.017551e-03) * ((3) * q[i10] * q[i10]) + (-1.380678e-04) * ((2) * q[i10] * q[i11]) + (-1.062532e-04) * ((2) * q[i10] * q[i12])
            + (-2.173750e-04) * ((2) * q[i10] * q[i15]) + (-1.998722e-04) * ((2) * q[i10] * q[i16]) + (-2.521337e-04) * ((2) * q[i10] * q[i19])
            + (1.029168e-04) * ((2) * q[i10] * q[i20]) + (2.119792e-04) * ((2) * q[i10] * q[i21]) + (-6.543407e-06) * ((2) * q[i10] * q[i22])
            + (5.397174e-05) * (q[i11] * q[i11]) + (-1.124623e-04) * (q[i12] * q[i12]) + (-1.647609e-04) * (q[i15] * q[i15])
            + (-1.332429e-05) * (q[i16] * q[i16]) + (-1.400187e-04) * (q[i19] * q[i19]) + (-1.564151e-04) * (q[i20] * q[i20])
            + (1.020322e-04) * (q[i21] * q[i21]) + (-4.066959e-05) * (q[i22] * q[i22]) + (2.846155e-04) * (q[i0] * q[i1]) + (2.438050e-03) * (q[i0] * q[i2])
            + (-2.628810e-03) * (q[i0] * q[i3]) + (-4.478219e-04) * (q[i0] * q[i4]) + (-3.349013e-04) * (q[i0] * q[i5]) + (1.844408e-03) * (q[i0] * q[i6])
            + (5.517188e-04) * (q[i0] * q[i7]) + (2.040536e-04) * (q[i0] * q[i8]) + (1.360263e-03) * (q[i0] * q[i9]) + (6.985196e-04) * (q[i0] * q[i11])
            + (5.734440e-04) * (q[i0] * q[i12]) + (5.303826e-04) * (q[i0] * q[i15]) + (-5.940487e-04) * (q[i0] * q[i16]) + (2.148813e-04) * (q[i0] * q[i19])
            + (-4.319454e-04) * (q[i0] * q[i20]) + (-1.584458e-04) * (q[i0] * q[i21]) + (-2.132184e-04) * (q[i0] * q[i22]) + (-1.976517e-03) * (q[i1] * q[i2])
            + (-8.867219e-04) * (q[i1] * q[i3]) + (1.261796e-02) * (q[i1] * q[i4]) + (2.248592e-03) * (q[i1] * q[i5]) + (-1.639768e-03) * (q[i1] * q[i6])
            + (5.542534e-04) * (q[i1] * q[i7]) + (-7.043732e-04) * (q[i1] * q[i8]) + (-1.360802e-03) * (q[i1] * q[i9]) + (-9.983918e-04) * (q[i1] * q[i11])
            + (3.945032e-05) * (q[i1] * q[i12]) + (-1.115424e-03) * (q[i1] * q[i15]) + (-9.232939e-05) * (q[i1] * q[i16]) + (-5.613018e-04) * (q[i1] * q[i19])
            + (1.733368e-04) * (q[i1] * q[i20]) + (-6.546301e-05) * (q[i1] * q[i21]) + (-7.665282e-05) * (q[i1] * q[i22]) + (-2.988670e-04) * (q[i2] * q[i3])
            + (2.270634e-03) * (q[i2] * q[i4]) + (5.223961e-03) * (q[i2] * q[i5]) + (-4.481669e-04) * (q[i2] * q[i6]) + (-1.291531e-03) * (q[i2] * q[i7])
            + (-2.143734e-03) * (q[i2] * q[i8]) + (2.516672e-06) * (q[i2] * q[i9]) + (-1.926262e-04) * (q[i2] * q[i11]) + (1.820085e-06) * (q[i2] * q[i12])
            + (6.286513e-05) * (q[i2] * q[i15]) + (-4.083522e-04) * (q[i2] * q[i16]) + (-1.011537e-04) * (q[i2] * q[i19]) + (2.619118e-04) * (q[i2] * q[i20])
            + (-3.418190e-04) * (q[i2] * q[i21]) + (-4.035359e-04) * (q[i2] * q[i22]) + (-1.869920e-03) * (q[i3] * q[i4]) + (-5.797220e-04) * (q[i3] * q[i5])
            + (-8.018915e-04) * (q[i3] * q[i6]) + (-2.740255e-03) * (q[i3] * q[i7]) + (2.746779e-04) * (q[i3] * q[i8]) + (-8.110467e-04) * (q[i3] * q[i9])
            + (-1.161962e-03) * (q[i3] * q[i11]) + (7.719139e-05) * (q[i3] * q[i12]) + (-4.930728e-04) * (q[i3] * q[i15]) + (4.747305e-04) * (q[i3] * q[i16])
            + (2.506987e-04) * (q[i3] * q[i19]) + (6.488983e-04) * (q[i3] * q[i20]) + (1.378146e-05) * (q[i3] * q[i21]) + (2.411005e-04) * (q[i3] * q[i22])
            + (-1.696389e-03) * (q[i4] * q[i5]) + (1.436831e-03) * (q[i4] * q[i6]) + (-1.282671e-03) * (q[i4] * q[i7]) + (1.022948e-03) * (q[i4] * q[i8])
            + (8.112286e-04) * (q[i4] * q[i9]) + (6.648302e-04) * (q[i4] * q[i11]) + (7.633476e-04) * (q[i4] * q[i12]) + (6.980060e-04) * (q[i4] * q[i15])
            + (2.553292e-04) * (q[i4] * q[i16]) + (6.283218e-04) * (q[i4] * q[i19]) + (-3.471171e-04) * (q[i4] * q[i20]) + (-3.230970e-04) * (q[i4] * q[i21])
            + (-2.538713e-04) * (q[i4] * q[i22]) + (-2.477935e-03) * (q[i5] * q[i6]) + (-2.164833e-03) * (q[i5] * q[i7]) + (-2.482312e-03) * (q[i5] * q[i8])
            + (-2.280379e-06) * (q[i5] * q[i9]) + (2.569866e-04) * (q[i5] * q[i11]) + (-9.864429e-04) * (q[i5] * q[i12]) + (8.599808e-04) * (q[i5] * q[i15])
            + (-4.068580e-04) * (q[i5] * q[i16]) + (-2.698364e-04) * (q[i5] * q[i19]) + (-7.060816e-04) * (q[i5] * q[i20]) + (-2.235920e-04) * (q[i5] * q[i21])
            + (-5.826465e-04) * (q[i5] * q[i22]) + (5.131238e-04) * (q[i6] * q[i7]) + (-1.566180e-03) * (q[i6] * q[i8]) + (-3.222617e-04) * (q[i6] * q[i9])
            + (3.046552e-05) * (q[i6] * q[i11]) + (-5.482079e-04) * (q[i6] * q[i12]) + (7.382468e-06) * (q[i6] * q[i15]) + (2.798608e-04) * (q[i6] * q[i16])
            + (-1.757855e-05) * (q[i6] * q[i19]) + (4.630654e-04) * (q[i6] * q[i20]) + (-3.136931e-04) * (q[i6] * q[i21]) + (-3.502427e-06) * (q[i6] * q[i22])
            + (-3.117434e-04) * (q[i7] * q[i8]) + (-3.334738e-04) * (q[i7] * q[i9]) + (-3.371485e-04) * (q[i7] * q[i11]) + (-3.108714e-04) * (q[i7] * q[i12])
            + (-7.809335e-05) * (q[i7] * q[i15]) + (-4.451027e-04) * (q[i7] * q[i16]) + (-2.948753e-04) * (q[i7] * q[i19]) + (1.864975e-04) * (q[i7] * q[i20])
            + (2.096182e-04) * (q[i7] * q[i21]) + (2.627882e-04) * (q[i7] * q[i22]) + (-1.025028e-04) * (q[i8] * q[i9]) + (9.523357e-04) * (q[i8] * q[i11])
            + (4.452269e-04) * (q[i8] * q[i12]) + (-1.570818e-04) * (q[i8] * q[i15]) + (-3.077152e-04) * (q[i8] * q[i16]) + (8.625016e-04) * (q[i8] * q[i19])
            + (-1.244961e-04) * (q[i8] * q[i20]) + (2.951818e-04) * (q[i8] * q[i21]) + (-9.019046e-05) * (q[i8] * q[i22]) + (1.670375e-04) * (q[i9] * q[i11])
            + (-1.631895e-04) * (q[i9] * q[i12]) + (2.964690e-04) * (q[i9] * q[i15]) + (2.970809e-04) * (q[i9] * q[i16]) + (3.917550e-05) * (q[i9] * q[i19])
            + (4.137510e-05) * (q[i9] * q[i20]) + (7.523850e-05) * (q[i9] * q[i21]) + (-7.457916e-05) * (q[i9] * q[i22]) + (2.763474e-04) * (q[i11] * q[i12])
            + (-2.427197e-04) * (q[i11] * q[i15]) + (-2.551507e-04) * (q[i11] * q[i16]) + (-3.946347e-04) * (q[i11] * q[i19])
            + (-1.611441e-04) * (q[i11] * q[i20]) + (-1.973640e-06) * (q[i11] * q[i21]) + (-1.972338e-05) * (q[i11] * q[i22])
            + (1.152339e-04) * (q[i12] * q[i15]) + (-1.365703e-04) * (q[i12] * q[i16]) + (-5.485815e-04) * (q[i12] * q[i19])
            + (-1.178500e-04) * (q[i12] * q[i20]) + (8.537272e-05) * (q[i12] * q[i21]) + (-5.462330e-05) * (q[i12] * q[i22])
            + (2.893244e-04) * (q[i15] * q[i16]) + (2.416269e-04) * (q[i15] * q[i19]) + (4.802103e-04) * (q[i15] * q[i20]) + (1.516216e-04) * (q[i15] * q[i21])
            + (8.125625e-05) * (q[i15] * q[i22]) + (2.265096e-04) * (q[i16] * q[i19]) + (-1.238974e-04) * (q[i16] * q[i20]) + (3.186902e-05) * (q[i16] * q[i21])
            + (-2.368354e-06) * (q[i16] * q[i22]) + (-1.316243e-04) * (q[i19] * q[i20]) + (1.747631e-04) * (q[i19] * q[i21])
            + (-1.190671e-04) * (q[i19] * q[i22]) + (-9.656492e-05) * (q[i20] * q[i21]) + (-1.132204e-04) * (q[i20] * q[i22])
            + (6.457753e-05) * (q[i21] * q[i22]);
   }

   public void getJQy11(double[] q, double[][] JQ)
   {
      JQ[2][i11] = (-5.222022e-03) * (1) + (-3.690845e-03) * ((2) * q[i11]) + (-2.101461e-03) * (q[i0]) + (-6.405035e-04) * (q[i1]) + (-2.575455e-03) * (q[i2])
            + (-4.951514e-03) * (q[i3]) + (-2.811232e-03) * (q[i4]) + (-4.741971e-03) * (q[i5]) + (2.074065e-03) * (q[i6]) + (-9.094966e-04) * (q[i7])
            + (-2.250525e-03) * (q[i8]) + (-2.832155e-04) * (q[i9]) + (-3.697813e-04) * (q[i10]) + (-1.794156e-04) * (q[i12]) + (8.438747e-04) * (q[i15])
            + (7.972360e-04) * (q[i16]) + (-9.842817e-04) * (q[i19]) + (-7.222546e-04) * (q[i20]) + (-1.187627e-03) * (q[i21]) + (1.108089e-05) * (q[i22])
            + (1.261540e-03) * (q[i0] * q[i0]) + (2.313198e-04) * (q[i1] * q[i1]) + (8.314828e-05) * (q[i2] * q[i2]) + (2.476817e-04) * (q[i3] * q[i3])
            + (-7.619221e-04) * (q[i4] * q[i4]) + (6.884126e-05) * (q[i5] * q[i5]) + (-2.863740e-04) * (q[i6] * q[i6]) + (-3.531736e-04) * (q[i7] * q[i7])
            + (7.407764e-04) * (q[i8] * q[i8]) + (1.059143e-04) * (q[i9] * q[i9]) + (-1.380678e-04) * (q[i10] * q[i10])
            + (2.709956e-04) * ((2) * q[i0] * q[i11]) + (-4.310596e-04) * ((2) * q[i1] * q[i11]) + (1.572873e-03) * ((2) * q[i2] * q[i11])
            + (1.445009e-03) * ((2) * q[i3] * q[i11]) + (-6.828332e-04) * ((2) * q[i4] * q[i11]) + (3.701298e-04) * ((2) * q[i5] * q[i11])
            + (-8.192230e-04) * ((2) * q[i6] * q[i11]) + (5.161338e-06) * ((2) * q[i7] * q[i11]) + (1.676774e-03) * ((2) * q[i8] * q[i11])
            + (-1.130603e-04) * ((2) * q[i9] * q[i11]) + (5.397174e-05) * ((2) * q[i10] * q[i11]) + (8.726313e-04) * ((3) * q[i11] * q[i11])
            + (-5.712128e-04) * ((2) * q[i11] * q[i12]) + (-2.055705e-03) * ((2) * q[i11] * q[i15]) + (-1.399818e-04) * ((2) * q[i11] * q[i16])
            + (6.309429e-04) * ((2) * q[i11] * q[i19]) + (-2.318441e-05) * ((2) * q[i11] * q[i20]) + (1.754330e-04) * ((2) * q[i11] * q[i21])
            + (1.907818e-04) * ((2) * q[i11] * q[i22]) + (5.751741e-04) * (q[i12] * q[i12]) + (2.137627e-03) * (q[i15] * q[i15])
            + (-2.187742e-04) * (q[i16] * q[i16]) + (-3.553330e-04) * (q[i19] * q[i19]) + (-1.814366e-04) * (q[i20] * q[i20])
            + (-4.051638e-04) * (q[i21] * q[i21]) + (2.422408e-05) * (q[i22] * q[i22]) + (-7.803878e-04) * (q[i0] * q[i1]) + (6.523406e-04) * (q[i0] * q[i2])
            + (3.415507e-04) * (q[i0] * q[i3]) + (-2.682264e-04) * (q[i0] * q[i4]) + (-2.742440e-03) * (q[i0] * q[i5]) + (-1.342392e-03) * (q[i0] * q[i6])
            + (8.710504e-05) * (q[i0] * q[i7]) + (-9.079045e-04) * (q[i0] * q[i8]) + (4.597602e-05) * (q[i0] * q[i9]) + (6.985196e-04) * (q[i0] * q[i10])
            + (-5.393562e-04) * (q[i0] * q[i12]) + (1.859815e-04) * (q[i0] * q[i15]) + (-7.142625e-05) * (q[i0] * q[i16]) + (5.389142e-04) * (q[i0] * q[i19])
            + (4.192410e-04) * (q[i0] * q[i20]) + (2.893217e-04) * (q[i0] * q[i21]) + (3.909004e-04) * (q[i0] * q[i22]) + (-8.432124e-04) * (q[i1] * q[i2])
            + (1.609103e-03) * (q[i1] * q[i3]) + (1.852015e-03) * (q[i1] * q[i4]) + (-2.247199e-03) * (q[i1] * q[i5]) + (2.123385e-04) * (q[i1] * q[i6])
            + (-1.426140e-03) * (q[i1] * q[i7]) + (1.002922e-03) * (q[i1] * q[i8]) + (5.642974e-04) * (q[i1] * q[i9]) + (-9.983918e-04) * (q[i1] * q[i10])
            + (5.435814e-04) * (q[i1] * q[i12]) + (-6.513442e-05) * (q[i1] * q[i15]) + (-2.026858e-05) * (q[i1] * q[i16]) + (6.352125e-05) * (q[i1] * q[i19])
            + (-5.394431e-04) * (q[i1] * q[i20]) + (3.606199e-04) * (q[i1] * q[i21]) + (5.179043e-04) * (q[i1] * q[i22]) + (7.361710e-04) * (q[i2] * q[i3])
            + (-5.738576e-04) * (q[i2] * q[i4]) + (1.171327e-03) * (q[i2] * q[i5]) + (-7.785610e-04) * (q[i2] * q[i6]) + (4.293556e-04) * (q[i2] * q[i7])
            + (4.147071e-03) * (q[i2] * q[i8]) + (3.620506e-06) * (q[i2] * q[i9]) + (-1.926262e-04) * (q[i2] * q[i10]) + (6.764880e-06) * (q[i2] * q[i12])
            + (-2.362006e-03) * (q[i2] * q[i15]) + (-5.016997e-04) * (q[i2] * q[i16]) + (6.703634e-04) * (q[i2] * q[i19]) + (-4.443000e-04) * (q[i2] * q[i20])
            + (9.345787e-04) * (q[i2] * q[i21]) + (1.211605e-03) * (q[i2] * q[i22]) + (1.272945e-03) * (q[i3] * q[i4]) + (-1.448399e-03) * (q[i3] * q[i5])
            + (-2.051880e-03) * (q[i3] * q[i6]) + (5.274222e-04) * (q[i3] * q[i7]) + (1.522556e-03) * (q[i3] * q[i8]) + (7.615759e-04) * (q[i3] * q[i9])
            + (-1.161962e-03) * (q[i3] * q[i10]) + (-7.578032e-04) * (q[i3] * q[i12]) + (-9.027708e-04) * (q[i3] * q[i15]) + (-3.870236e-05) * (q[i3] * q[i16])
            + (-1.712573e-04) * (q[i3] * q[i19]) + (-8.019534e-04) * (q[i3] * q[i20]) + (-8.460064e-04) * (q[i3] * q[i21]) + (4.242010e-04) * (q[i3] * q[i22])
            + (1.201051e-03) * (q[i4] * q[i5]) + (2.222197e-04) * (q[i4] * q[i6]) + (-2.309113e-04) * (q[i4] * q[i7]) + (-3.970672e-05) * (q[i4] * q[i8])
            + (7.421642e-05) * (q[i4] * q[i9]) + (6.648302e-04) * (q[i4] * q[i10]) + (7.642985e-04) * (q[i4] * q[i12]) + (-4.090085e-04) * (q[i4] * q[i15])
            + (1.070191e-03) * (q[i4] * q[i16]) + (-1.119821e-04) * (q[i4] * q[i19]) + (9.979069e-05) * (q[i4] * q[i20]) + (1.227838e-04) * (q[i4] * q[i21])
            + (2.703891e-05) * (q[i4] * q[i22]) + (4.159708e-04) * (q[i5] * q[i6]) + (-1.864377e-03) * (q[i5] * q[i7]) + (2.648866e-03) * (q[i5] * q[i8])
            + (-9.996856e-04) * (q[i5] * q[i9]) + (2.569866e-04) * (q[i5] * q[i10]) + (7.873494e-06) * (q[i5] * q[i12]) + (-2.545878e-03) * (q[i5] * q[i15])
            + (-3.355602e-04) * (q[i5] * q[i16]) + (-3.721245e-04) * (q[i5] * q[i19]) + (3.566828e-04) * (q[i5] * q[i20]) + (-5.596208e-04) * (q[i5] * q[i21])
            + (-2.649668e-04) * (q[i5] * q[i22]) + (1.122814e-03) * (q[i6] * q[i7]) + (-8.828370e-04) * (q[i6] * q[i8]) + (3.018579e-04) * (q[i6] * q[i9])
            + (3.046552e-05) * (q[i6] * q[i10]) + (-6.285653e-05) * (q[i6] * q[i12]) + (4.187876e-04) * (q[i6] * q[i15]) + (3.723862e-04) * (q[i6] * q[i16])
            + (-1.640755e-04) * (q[i6] * q[i19]) + (-2.434679e-04) * (q[i6] * q[i20]) + (-2.957810e-04) * (q[i6] * q[i21]) + (-6.079118e-04) * (q[i6] * q[i22])
            + (1.097291e-03) * (q[i7] * q[i8]) + (5.548080e-04) * (q[i7] * q[i9]) + (-3.371485e-04) * (q[i7] * q[i10]) + (-8.108071e-05) * (q[i7] * q[i12])
            + (5.273374e-04) * (q[i7] * q[i15]) + (-3.163262e-05) * (q[i7] * q[i16]) + (-7.524544e-05) * (q[i7] * q[i19]) + (9.860693e-04) * (q[i7] * q[i20])
            + (-1.315688e-06) * (q[i7] * q[i21]) + (6.556906e-04) * (q[i7] * q[i22]) + (-4.504523e-04) * (q[i8] * q[i9]) + (9.523357e-04) * (q[i8] * q[i10])
            + (-1.421608e-03) * (q[i8] * q[i12]) + (-9.667942e-04) * (q[i8] * q[i15]) + (2.049559e-04) * (q[i8] * q[i16]) + (7.583491e-04) * (q[i8] * q[i19])
            + (1.177453e-03) * (q[i8] * q[i20]) + (-3.791023e-04) * (q[i8] * q[i21]) + (-2.098501e-04) * (q[i8] * q[i22]) + (1.670375e-04) * (q[i9] * q[i10])
            + (2.691510e-04) * (q[i9] * q[i12]) + (1.353202e-04) * (q[i9] * q[i15]) + (-1.070767e-04) * (q[i9] * q[i16]) + (1.100914e-04) * (q[i9] * q[i19])
            + (5.449230e-04) * (q[i9] * q[i20]) + (-5.803486e-05) * (q[i9] * q[i21]) + (8.276334e-05) * (q[i9] * q[i22]) + (2.763474e-04) * (q[i10] * q[i12])
            + (-2.427197e-04) * (q[i10] * q[i15]) + (-2.551507e-04) * (q[i10] * q[i16]) + (-3.946347e-04) * (q[i10] * q[i19])
            + (-1.611441e-04) * (q[i10] * q[i20]) + (-1.973640e-06) * (q[i10] * q[i21]) + (-1.972338e-05) * (q[i10] * q[i22])
            + (1.804308e-05) * (q[i12] * q[i15]) + (2.901597e-05) * (q[i12] * q[i16]) + (-2.141351e-04) * (q[i12] * q[i19])
            + (-2.115579e-04) * (q[i12] * q[i20]) + (3.290286e-06) * (q[i12] * q[i21]) + (9.393283e-08) * (q[i12] * q[i22])
            + (-1.103244e-04) * (q[i15] * q[i16]) + (1.222465e-04) * (q[i15] * q[i19]) + (-5.286889e-04) * (q[i15] * q[i20])
            + (-7.061340e-05) * (q[i15] * q[i21]) + (1.524228e-04) * (q[i15] * q[i22]) + (-5.783379e-04) * (q[i16] * q[i19])
            + (-1.003911e-04) * (q[i16] * q[i20]) + (1.560478e-04) * (q[i16] * q[i21]) + (-5.117911e-05) * (q[i16] * q[i22])
            + (1.460629e-04) * (q[i19] * q[i20]) + (1.258352e-04) * (q[i19] * q[i21]) + (-9.818217e-05) * (q[i19] * q[i22])
            + (-8.726795e-05) * (q[i20] * q[i21]) + (-1.764124e-04) * (q[i20] * q[i22]) + (-1.611880e-05) * (q[i21] * q[i22]);
   }

   public void getJQy12(double[] q, double[][] JQ)
   {
      JQ[2][i12] = (5.619843e-03) * (1) + (-3.610286e-03) * ((2) * q[i12]) + (-6.247272e-04) * (q[i0]) + (-2.114922e-03) * (q[i1]) + (-2.486851e-03) * (q[i2])
            + (-2.852589e-03) * (q[i3]) + (-4.944247e-03) * (q[i4]) + (-4.693143e-03) * (q[i5]) + (9.387117e-04) * (q[i6]) + (-2.033062e-03) * (q[i7])
            + (2.137495e-03) * (q[i8]) + (3.941600e-04) * (q[i9]) + (3.171618e-04) * (q[i10]) + (-1.794156e-04) * (q[i11]) + (-7.939226e-04) * (q[i15])
            + (-9.109884e-04) * (q[i16]) + (7.347940e-04) * (q[i19]) + (9.576683e-04) * (q[i20]) + (3.132506e-05) * (q[i21]) + (-1.201508e-03) * (q[i22])
            + (-2.336911e-04) * (q[i0] * q[i0]) + (-1.247452e-03) * (q[i1] * q[i1]) + (-1.286932e-04) * (q[i2] * q[i2]) + (7.545558e-04) * (q[i3] * q[i3])
            + (-2.208289e-04) * (q[i4] * q[i4]) + (-9.857491e-05) * (q[i5] * q[i5]) + (3.608378e-04) * (q[i6] * q[i6]) + (3.045578e-04) * (q[i7] * q[i7])
            + (-7.625813e-04) * (q[i8] * q[i8]) + (1.338060e-04) * (q[i9] * q[i9]) + (-1.062532e-04) * (q[i10] * q[i10]) + (-5.712128e-04) * (q[i11] * q[i11])
            + (4.285072e-04) * ((2) * q[i0] * q[i12]) + (-2.801488e-04) * ((2) * q[i1] * q[i12]) + (-1.692362e-03) * ((2) * q[i2] * q[i12])
            + (6.728427e-04) * ((2) * q[i3] * q[i12]) + (-1.457032e-03) * ((2) * q[i4] * q[i12]) + (-4.291887e-04) * ((2) * q[i5] * q[i12])
            + (1.147318e-05) * ((2) * q[i6] * q[i12]) + (-8.323333e-04) * ((2) * q[i7] * q[i12]) + (1.706114e-03) * ((2) * q[i8] * q[i12])
            + (6.798928e-05) * ((2) * q[i9] * q[i12]) + (-1.124623e-04) * ((2) * q[i10] * q[i12]) + (5.751741e-04) * ((2) * q[i11] * q[i12])
            + (-9.228392e-04) * ((3) * q[i12] * q[i12]) + (-1.452196e-04) * ((2) * q[i12] * q[i15]) + (-2.048617e-03) * ((2) * q[i12] * q[i16])
            + (-1.360159e-05) * ((2) * q[i12] * q[i19]) + (6.134458e-04) * ((2) * q[i12] * q[i20]) + (-1.951116e-04) * ((2) * q[i12] * q[i21])
            + (-1.700646e-04) * ((2) * q[i12] * q[i22]) + (2.182402e-04) * (q[i15] * q[i15]) + (-2.170813e-03) * (q[i16] * q[i16])
            + (1.844954e-04) * (q[i19] * q[i19]) + (3.580127e-04) * (q[i20] * q[i20]) + (-2.526193e-05) * (q[i21] * q[i21]) + (4.178399e-04) * (q[i22] * q[i22])
            + (7.788289e-04) * (q[i0] * q[i1]) + (8.603355e-04) * (q[i0] * q[i2]) + (-1.892927e-03) * (q[i0] * q[i3]) + (-1.597540e-03) * (q[i0] * q[i4])
            + (2.213568e-03) * (q[i0] * q[i5]) + (-1.443070e-03) * (q[i0] * q[i6]) + (2.285169e-04) * (q[i0] * q[i7]) + (9.702281e-04) * (q[i0] * q[i8])
            + (-1.002433e-03) * (q[i0] * q[i9]) + (5.734440e-04) * (q[i0] * q[i10]) + (-5.393562e-04) * (q[i0] * q[i11]) + (-2.571111e-05) * (q[i0] * q[i15])
            + (-5.853968e-05) * (q[i0] * q[i16]) + (-5.457165e-04) * (q[i0] * q[i19]) + (6.830868e-05) * (q[i0] * q[i20]) + (-5.167213e-04) * (q[i0] * q[i21])
            + (-3.713889e-04) * (q[i0] * q[i22]) + (-6.395182e-04) * (q[i1] * q[i2]) + (2.617932e-04) * (q[i1] * q[i3]) + (-3.495028e-04) * (q[i1] * q[i4])
            + (2.677469e-03) * (q[i1] * q[i5]) + (7.670935e-05) * (q[i1] * q[i6]) + (-1.341927e-03) * (q[i1] * q[i7]) + (-9.243151e-04) * (q[i1] * q[i8])
            + (7.066151e-04) * (q[i1] * q[i9]) + (3.945032e-05) * (q[i1] * q[i10]) + (5.435814e-04) * (q[i1] * q[i11]) + (-7.006303e-05) * (q[i1] * q[i15])
            + (1.941888e-04) * (q[i1] * q[i16]) + (4.266644e-04) * (q[i1] * q[i19]) + (5.463806e-04) * (q[i1] * q[i20]) + (-3.924730e-04) * (q[i1] * q[i21])
            + (-2.773281e-04) * (q[i1] * q[i22]) + (5.600232e-04) * (q[i2] * q[i3]) + (-7.830249e-04) * (q[i2] * q[i4]) + (-1.285937e-03) * (q[i2] * q[i5])
            + (4.258846e-04) * (q[i2] * q[i6]) + (-8.119320e-04) * (q[i2] * q[i7]) + (4.154181e-03) * (q[i2] * q[i8]) + (-1.799722e-04) * (q[i2] * q[i9])
            + (1.820085e-06) * (q[i2] * q[i10]) + (6.764880e-06) * (q[i2] * q[i11]) + (-5.111858e-04) * (q[i2] * q[i15]) + (-2.336272e-03) * (q[i2] * q[i16])
            + (-4.444471e-04) * (q[i2] * q[i19]) + (6.501908e-04) * (q[i2] * q[i20]) + (-1.204995e-03) * (q[i2] * q[i21]) + (-9.273983e-04) * (q[i2] * q[i22])
            + (-1.294124e-03) * (q[i3] * q[i4]) + (-1.207768e-03) * (q[i3] * q[i5]) + (-2.301127e-04) * (q[i3] * q[i6]) + (2.305726e-04) * (q[i3] * q[i7])
            + (-3.970226e-05) * (q[i3] * q[i8]) + (6.839198e-04) * (q[i3] * q[i9]) + (7.719139e-05) * (q[i3] * q[i10]) + (-7.578032e-04) * (q[i3] * q[i11])
            + (1.071918e-03) * (q[i3] * q[i15]) + (-4.166864e-04) * (q[i3] * q[i16]) + (8.904234e-05) * (q[i3] * q[i19]) + (-1.091851e-04) * (q[i3] * q[i20])
            + (-2.769233e-05) * (q[i3] * q[i21]) + (-1.330506e-04) * (q[i3] * q[i22]) + (1.420428e-03) * (q[i4] * q[i5]) + (5.484947e-04) * (q[i4] * q[i6])
            + (-2.075818e-03) * (q[i4] * q[i7]) + (1.552861e-03) * (q[i4] * q[i8]) + (-1.158998e-03) * (q[i4] * q[i9]) + (7.633476e-04) * (q[i4] * q[i10])
            + (7.642985e-04) * (q[i4] * q[i11]) + (-2.736292e-05) * (q[i4] * q[i15]) + (-9.062632e-04) * (q[i4] * q[i16]) + (-7.938514e-04) * (q[i4] * q[i19])
            + (-1.668851e-04) * (q[i4] * q[i20]) + (-4.133740e-04) * (q[i4] * q[i21]) + (8.567602e-04) * (q[i4] * q[i22]) + (-1.840572e-03) * (q[i5] * q[i6])
            + (4.200824e-04) * (q[i5] * q[i7]) + (2.660963e-03) * (q[i5] * q[i8]) + (2.664558e-04) * (q[i5] * q[i9]) + (-9.864429e-04) * (q[i5] * q[i10])
            + (7.873494e-06) * (q[i5] * q[i11]) + (-3.395477e-04) * (q[i5] * q[i15]) + (-2.540585e-03) * (q[i5] * q[i16]) + (3.623418e-04) * (q[i5] * q[i19])
            + (-3.867339e-04) * (q[i5] * q[i20]) + (2.657216e-04) * (q[i5] * q[i21]) + (5.715204e-04) * (q[i5] * q[i22]) + (-1.119746e-03) * (q[i6] * q[i7])
            + (-1.065433e-03) * (q[i6] * q[i8]) + (3.321788e-04) * (q[i6] * q[i9]) + (-5.482079e-04) * (q[i6] * q[i10]) + (-6.285653e-05) * (q[i6] * q[i11])
            + (3.061818e-05) * (q[i6] * q[i15]) + (-5.408422e-04) * (q[i6] * q[i16]) + (-9.819785e-04) * (q[i6] * q[i19]) + (8.291197e-05) * (q[i6] * q[i20])
            + (6.580356e-04) * (q[i6] * q[i21]) + (3.425389e-07) * (q[i6] * q[i22]) + (8.750085e-04) * (q[i7] * q[i8]) + (-2.256889e-05) * (q[i7] * q[i9])
            + (-3.108714e-04) * (q[i7] * q[i10]) + (-8.108071e-05) * (q[i7] * q[i11]) + (-3.739448e-04) * (q[i7] * q[i15]) + (-4.156540e-04) * (q[i7] * q[i16])
            + (2.504454e-04) * (q[i7] * q[i19]) + (1.631307e-04) * (q[i7] * q[i20]) + (-6.068143e-04) * (q[i7] * q[i21]) + (-2.887615e-04) * (q[i7] * q[i22])
            + (-9.541455e-04) * (q[i8] * q[i9]) + (4.452269e-04) * (q[i8] * q[i10]) + (-1.421608e-03) * (q[i8] * q[i11]) + (-2.133103e-04) * (q[i8] * q[i15])
            + (9.615786e-04) * (q[i8] * q[i16]) + (-1.190358e-03) * (q[i8] * q[i19]) + (-7.548895e-04) * (q[i8] * q[i20]) + (-2.082552e-04) * (q[i8] * q[i21])
            + (-3.871480e-04) * (q[i8] * q[i22]) + (-1.631895e-04) * (q[i9] * q[i10]) + (2.691510e-04) * (q[i9] * q[i11]) + (2.551192e-04) * (q[i9] * q[i15])
            + (2.425242e-04) * (q[i9] * q[i16]) + (1.687963e-04) * (q[i9] * q[i19]) + (3.931066e-04) * (q[i9] * q[i20]) + (-2.334586e-05) * (q[i9] * q[i21])
            + (3.840462e-07) * (q[i9] * q[i22]) + (2.763474e-04) * (q[i10] * q[i11]) + (1.152339e-04) * (q[i10] * q[i15]) + (-1.365703e-04) * (q[i10] * q[i16])
            + (-5.485815e-04) * (q[i10] * q[i19]) + (-1.178500e-04) * (q[i10] * q[i20]) + (8.537272e-05) * (q[i10] * q[i21])
            + (-5.462330e-05) * (q[i10] * q[i22]) + (1.804308e-05) * (q[i11] * q[i15]) + (2.901597e-05) * (q[i11] * q[i16])
            + (-2.141351e-04) * (q[i11] * q[i19]) + (-2.115579e-04) * (q[i11] * q[i20]) + (3.290286e-06) * (q[i11] * q[i21])
            + (9.393283e-08) * (q[i11] * q[i22]) + (1.079393e-04) * (q[i15] * q[i16]) + (9.948236e-05) * (q[i15] * q[i19]) + (5.750484e-04) * (q[i15] * q[i20])
            + (-4.713812e-05) * (q[i15] * q[i21]) + (1.608407e-04) * (q[i15] * q[i22]) + (5.335886e-04) * (q[i16] * q[i19])
            + (-1.164256e-04) * (q[i16] * q[i20]) + (1.596939e-04) * (q[i16] * q[i21]) + (-8.637346e-05) * (q[i16] * q[i22])
            + (-1.457333e-04) * (q[i19] * q[i20]) + (-1.774228e-04) * (q[i19] * q[i21]) + (-8.433190e-05) * (q[i19] * q[i22])
            + (-9.988231e-05) * (q[i20] * q[i21]) + (1.307902e-04) * (q[i20] * q[i22]) + (1.312943e-05) * (q[i21] * q[i22]);
   }

   public void getJQy15(double[] q, double[][] JQ)
   {
      JQ[2][i15] = (5.040684e-03) * (1) + (-2.841511e-03) * ((2) * q[i15]) + (1.763563e-03) * (q[i0]) + (5.440769e-03) * (q[i1]) + (5.981859e-03) * (q[i2])
            + (1.701516e-04) * (q[i3]) + (-1.936538e-03) * (q[i4]) + (1.072417e-03) * (q[i5]) + (2.506067e-03) * (q[i6]) + (6.173416e-03) * (q[i7])
            + (-9.332737e-03) * (q[i8]) + (1.616846e-03) * (q[i9]) + (2.032891e-03) * (q[i10]) + (8.438747e-04) * (q[i11]) + (-7.939226e-04) * (q[i12])
            + (-9.050529e-04) * (q[i16]) + (6.894907e-04) * (q[i19]) + (-7.798828e-04) * (q[i20]) + (6.160006e-04) * (q[i21]) + (-3.030791e-04) * (q[i22])
            + (7.602174e-04) * (q[i0] * q[i0]) + (-1.123010e-04) * (q[i1] * q[i1]) + (-2.236801e-04) * (q[i2] * q[i2]) + (3.150266e-04) * (q[i3] * q[i3])
            + (1.659287e-04) * (q[i4] * q[i4]) + (-2.111881e-03) * (q[i5] * q[i5]) + (-5.963555e-04) * (q[i6] * q[i6]) + (-4.750761e-04) * (q[i7] * q[i7])
            + (4.076422e-05) * (q[i8] * q[i8]) + (-2.008965e-04) * (q[i9] * q[i9]) + (-2.173750e-04) * (q[i10] * q[i10]) + (-2.055705e-03) * (q[i11] * q[i11])
            + (-1.452196e-04) * (q[i12] * q[i12]) + (4.538791e-05) * ((2) * q[i0] * q[i15]) + (7.123550e-04) * ((2) * q[i1] * q[i15])
            + (2.563287e-03) * ((2) * q[i2] * q[i15]) + (-2.638679e-05) * ((2) * q[i3] * q[i15]) + (-4.245545e-04) * ((2) * q[i4] * q[i15])
            + (-5.541777e-04) * ((2) * q[i5] * q[i15]) + (4.019567e-05) * ((2) * q[i6] * q[i15]) + (8.646733e-04) * ((2) * q[i7] * q[i15])
            + (6.044488e-05) * ((2) * q[i8] * q[i15]) + (-9.121269e-06) * ((2) * q[i9] * q[i15]) + (-1.647609e-04) * ((2) * q[i10] * q[i15])
            + (2.137627e-03) * ((2) * q[i11] * q[i15]) + (2.182402e-04) * ((2) * q[i12] * q[i15]) + (5.182965e-05) * ((3) * q[i15] * q[i15])
            + (-3.212224e-04) * ((2) * q[i15] * q[i16]) + (1.617819e-04) * ((2) * q[i15] * q[i19]) + (-3.049154e-04) * ((2) * q[i15] * q[i20])
            + (-1.976457e-04) * ((2) * q[i15] * q[i21]) + (9.307695e-07) * ((2) * q[i15] * q[i22]) + (-3.227798e-04) * (q[i16] * q[i16])
            + (-1.633945e-04) * (q[i19] * q[i19]) + (-5.054305e-05) * (q[i20] * q[i20]) + (-1.613204e-04) * (q[i21] * q[i21])
            + (1.252246e-04) * (q[i22] * q[i22]) + (-5.317338e-04) * (q[i0] * q[i1]) + (9.454998e-05) * (q[i0] * q[i2]) + (-2.736937e-04) * (q[i0] * q[i3])
            + (-3.911409e-04) * (q[i0] * q[i4]) + (-1.032333e-03) * (q[i0] * q[i5]) + (-3.174323e-04) * (q[i0] * q[i6]) + (7.248336e-04) * (q[i0] * q[i7])
            + (-7.466604e-04) * (q[i0] * q[i8]) + (1.017340e-04) * (q[i0] * q[i9]) + (5.303826e-04) * (q[i0] * q[i10]) + (1.859815e-04) * (q[i0] * q[i11])
            + (-2.571111e-05) * (q[i0] * q[i12]) + (-2.180080e-04) * (q[i0] * q[i16]) + (-9.731131e-05) * (q[i0] * q[i19]) + (-1.329828e-04) * (q[i0] * q[i20])
            + (3.266881e-04) * (q[i0] * q[i21]) + (1.098482e-04) * (q[i0] * q[i22]) + (1.760426e-04) * (q[i1] * q[i2]) + (2.837263e-04) * (q[i1] * q[i3])
            + (-4.364390e-03) * (q[i1] * q[i4]) + (-1.592579e-03) * (q[i1] * q[i5]) + (-1.923527e-04) * (q[i1] * q[i6]) + (-1.151666e-03) * (q[i1] * q[i7])
            + (-2.146160e-03) * (q[i1] * q[i8]) + (5.905837e-04) * (q[i1] * q[i9]) + (-1.115424e-03) * (q[i1] * q[i10]) + (-6.513442e-05) * (q[i1] * q[i11])
            + (-7.006303e-05) * (q[i1] * q[i12]) + (2.186881e-04) * (q[i1] * q[i16]) + (-1.743718e-04) * (q[i1] * q[i19]) + (5.877674e-04) * (q[i1] * q[i20])
            + (1.723671e-04) * (q[i1] * q[i21]) + (-2.020610e-04) * (q[i1] * q[i22]) + (9.927942e-04) * (q[i2] * q[i3]) + (-2.498836e-03) * (q[i2] * q[i4])
            + (-6.544554e-03) * (q[i2] * q[i5]) + (-1.313394e-03) * (q[i2] * q[i6]) + (-4.246611e-04) * (q[i2] * q[i7]) + (-9.564607e-04) * (q[i2] * q[i8])
            + (4.049757e-04) * (q[i2] * q[i9]) + (6.286513e-05) * (q[i2] * q[i10]) + (-2.362006e-03) * (q[i2] * q[i11]) + (-5.111858e-04) * (q[i2] * q[i12])
            + (-4.108729e-06) * (q[i2] * q[i16]) + (-3.347044e-04) * (q[i2] * q[i19]) + (1.346695e-03) * (q[i2] * q[i20]) + (7.605788e-04) * (q[i2] * q[i21])
            + (3.999834e-05) * (q[i2] * q[i22]) + (-3.552360e-04) * (q[i3] * q[i4]) + (-1.703704e-03) * (q[i3] * q[i5]) + (-6.109421e-04) * (q[i3] * q[i6])
            + (1.450320e-03) * (q[i3] * q[i7]) + (-2.196074e-03) * (q[i3] * q[i8]) + (-2.617285e-04) * (q[i3] * q[i9]) + (-4.930728e-04) * (q[i3] * q[i10])
            + (-9.027708e-04) * (q[i3] * q[i11]) + (1.071918e-03) * (q[i3] * q[i12]) + (-4.291182e-04) * (q[i3] * q[i16]) + (-1.090363e-03) * (q[i3] * q[i19])
            + (-3.773423e-04) * (q[i3] * q[i20]) + (-7.025679e-04) * (q[i3] * q[i21]) + (-7.839491e-04) * (q[i3] * q[i22]) + (9.956928e-04) * (q[i4] * q[i5])
            + (7.600604e-04) * (q[i4] * q[i6]) + (-1.302155e-03) * (q[i4] * q[i7]) + (-1.735568e-03) * (q[i4] * q[i8]) + (-4.751135e-04) * (q[i4] * q[i9])
            + (6.980060e-04) * (q[i4] * q[i10]) + (-4.090085e-04) * (q[i4] * q[i11]) + (-2.736292e-05) * (q[i4] * q[i12]) + (4.269459e-04) * (q[i4] * q[i16])
            + (-1.936850e-04) * (q[i4] * q[i19]) + (4.801148e-04) * (q[i4] * q[i20]) + (2.930438e-04) * (q[i4] * q[i21]) + (-5.318361e-05) * (q[i4] * q[i22])
            + (6.391077e-04) * (q[i5] * q[i6]) + (3.664296e-04) * (q[i5] * q[i7]) + (2.435700e-03) * (q[i5] * q[i8]) + (4.065399e-04) * (q[i5] * q[i9])
            + (8.599808e-04) * (q[i5] * q[i10]) + (-2.545878e-03) * (q[i5] * q[i11]) + (-3.395477e-04) * (q[i5] * q[i12]) + (-1.584383e-06) * (q[i5] * q[i16])
            + (-4.229998e-04) * (q[i5] * q[i19]) + (-4.258683e-04) * (q[i5] * q[i20]) + (6.511432e-04) * (q[i5] * q[i21]) + (4.829936e-04) * (q[i5] * q[i22])
            + (6.383422e-04) * (q[i6] * q[i7]) + (7.782932e-04) * (q[i6] * q[i8]) + (-4.425578e-04) * (q[i6] * q[i9]) + (7.382468e-06) * (q[i6] * q[i10])
            + (4.187876e-04) * (q[i6] * q[i11]) + (3.061818e-05) * (q[i6] * q[i12]) + (-2.138078e-05) * (q[i6] * q[i16]) + (-6.097696e-05) * (q[i6] * q[i19])
            + (-4.208301e-04) * (q[i6] * q[i20]) + (-3.006825e-04) * (q[i6] * q[i21]) + (6.121058e-05) * (q[i6] * q[i22]) + (-7.901275e-04) * (q[i7] * q[i8])
            + (2.860588e-04) * (q[i7] * q[i9]) + (-7.809335e-05) * (q[i7] * q[i10]) + (5.273374e-04) * (q[i7] * q[i11]) + (-3.739448e-04) * (q[i7] * q[i12])
            + (-2.752398e-05) * (q[i7] * q[i16]) + (-2.515684e-04) * (q[i7] * q[i19]) + (1.008449e-03) * (q[i7] * q[i20]) + (7.773756e-04) * (q[i7] * q[i21])
            + (-1.900134e-04) * (q[i7] * q[i22]) + (-3.011738e-04) * (q[i8] * q[i9]) + (-1.570818e-04) * (q[i8] * q[i10]) + (-9.667942e-04) * (q[i8] * q[i11])
            + (-2.133103e-04) * (q[i8] * q[i12]) + (-2.463027e-04) * (q[i8] * q[i16]) + (1.797364e-04) * (q[i8] * q[i19]) + (-7.647453e-04) * (q[i8] * q[i20])
            + (-1.503288e-03) * (q[i8] * q[i21]) + (2.291151e-04) * (q[i8] * q[i22]) + (2.964690e-04) * (q[i9] * q[i10]) + (1.353202e-04) * (q[i9] * q[i11])
            + (2.551192e-04) * (q[i9] * q[i12]) + (2.926568e-04) * (q[i9] * q[i16]) + (-1.205072e-04) * (q[i9] * q[i19]) + (2.208531e-04) * (q[i9] * q[i20])
            + (5.189756e-06) * (q[i9] * q[i21]) + (-3.313817e-05) * (q[i9] * q[i22]) + (-2.427197e-04) * (q[i10] * q[i11]) + (1.152339e-04) * (q[i10] * q[i12])
            + (2.893244e-04) * (q[i10] * q[i16]) + (2.416269e-04) * (q[i10] * q[i19]) + (4.802103e-04) * (q[i10] * q[i20]) + (1.516216e-04) * (q[i10] * q[i21])
            + (8.125625e-05) * (q[i10] * q[i22]) + (1.804308e-05) * (q[i11] * q[i12]) + (-1.103244e-04) * (q[i11] * q[i16]) + (1.222465e-04) * (q[i11] * q[i19])
            + (-5.286889e-04) * (q[i11] * q[i20]) + (-7.061340e-05) * (q[i11] * q[i21]) + (1.524228e-04) * (q[i11] * q[i22])
            + (1.079393e-04) * (q[i12] * q[i16]) + (9.948236e-05) * (q[i12] * q[i19]) + (5.750484e-04) * (q[i12] * q[i20]) + (-4.713812e-05) * (q[i12] * q[i21])
            + (1.608407e-04) * (q[i12] * q[i22]) + (2.676551e-04) * (q[i16] * q[i19]) + (2.627945e-04) * (q[i16] * q[i20]) + (6.172918e-05) * (q[i16] * q[i21])
            + (-6.089486e-05) * (q[i16] * q[i22]) + (9.631664e-05) * (q[i19] * q[i20]) + (-8.679254e-04) * (q[i19] * q[i21])
            + (-2.501875e-04) * (q[i19] * q[i22]) + (-1.314745e-04) * (q[i20] * q[i21]) + (-9.374580e-05) * (q[i20] * q[i22])
            + (-4.223805e-05) * (q[i21] * q[i22]);
   }

   public void getJQy16(double[] q, double[][] JQ)
   {
      JQ[2][i16] = (4.959764e-03) * (1) + (-2.872103e-03) * ((2) * q[i16]) + (-5.522394e-03) * (q[i0]) + (-1.795247e-03) * (q[i1]) + (-6.084513e-03) * (q[i2])
            + (1.983357e-03) * (q[i3]) + (-1.070322e-04) * (q[i4]) + (-1.156109e-03) * (q[i5]) + (6.238223e-03) * (q[i6]) + (2.507167e-03) * (q[i7])
            + (-9.383865e-03) * (q[i8]) + (2.078583e-03) * (q[i9]) + (1.586893e-03) * (q[i10]) + (7.972360e-04) * (q[i11]) + (-9.109884e-04) * (q[i12])
            + (-9.050529e-04) * (q[i15]) + (-8.045054e-04) * (q[i19]) + (6.796655e-04) * (q[i20]) + (2.968374e-04) * (q[i21]) + (-6.132847e-04) * (q[i22])
            + (-1.045320e-04) * (q[i0] * q[i0]) + (7.653473e-04) * (q[i1] * q[i1]) + (-2.076048e-04) * (q[i2] * q[i2]) + (1.793124e-04) * (q[i3] * q[i3])
            + (2.896900e-04) * (q[i4] * q[i4]) + (-2.132539e-03) * (q[i5] * q[i5]) + (-4.876350e-04) * (q[i6] * q[i6]) + (-5.980053e-04) * (q[i7] * q[i7])
            + (5.069054e-05) * (q[i8] * q[i8]) + (-2.215306e-04) * (q[i9] * q[i9]) + (-1.998722e-04) * (q[i10] * q[i10]) + (-1.399818e-04) * (q[i11] * q[i11])
            + (-2.048617e-03) * (q[i12] * q[i12]) + (-3.212224e-04) * (q[i15] * q[i15]) + (-7.146619e-04) * ((2) * q[i0] * q[i16])
            + (-4.658708e-05) * ((2) * q[i1] * q[i16]) + (-2.586976e-03) * ((2) * q[i2] * q[i16]) + (4.245840e-04) * ((2) * q[i3] * q[i16])
            + (4.041991e-05) * ((2) * q[i4] * q[i16]) + (5.385013e-04) * ((2) * q[i5] * q[i16]) + (8.658253e-04) * ((2) * q[i6] * q[i16])
            + (3.910849e-05) * ((2) * q[i7] * q[i16]) + (7.343281e-05) * ((2) * q[i8] * q[i16]) + (-1.646033e-04) * ((2) * q[i9] * q[i16])
            + (-1.332429e-05) * ((2) * q[i10] * q[i16]) + (-2.187742e-04) * ((2) * q[i11] * q[i16]) + (-2.170813e-03) * ((2) * q[i12] * q[i16])
            + (-3.227798e-04) * ((2) * q[i15] * q[i16]) + (5.263756e-05) * ((3) * q[i16] * q[i16]) + (-3.098556e-04) * ((2) * q[i16] * q[i19])
            + (1.564144e-04) * ((2) * q[i16] * q[i20]) + (-3.724932e-06) * ((2) * q[i16] * q[i21]) + (2.048794e-04) * ((2) * q[i16] * q[i22])
            + (-4.874503e-05) * (q[i19] * q[i19]) + (-1.522340e-04) * (q[i20] * q[i20]) + (1.226978e-04) * (q[i21] * q[i21])
            + (-1.634418e-04) * (q[i22] * q[i22]) + (-5.364147e-04) * (q[i0] * q[i1]) + (1.685530e-04) * (q[i0] * q[i2]) + (-4.429769e-03) * (q[i0] * q[i3])
            + (2.809789e-04) * (q[i0] * q[i4]) + (-1.635496e-03) * (q[i0] * q[i5]) + (1.159499e-03) * (q[i0] * q[i6]) + (1.887704e-04) * (q[i0] * q[i7])
            + (2.160150e-03) * (q[i0] * q[i8]) + (1.127961e-03) * (q[i0] * q[i9]) + (-5.940487e-04) * (q[i0] * q[i10]) + (-7.142625e-05) * (q[i0] * q[i11])
            + (-5.853968e-05) * (q[i0] * q[i12]) + (-2.180080e-04) * (q[i0] * q[i15]) + (-5.863536e-04) * (q[i0] * q[i19]) + (1.775150e-04) * (q[i0] * q[i20])
            + (-2.073675e-04) * (q[i0] * q[i21]) + (1.769337e-04) * (q[i0] * q[i22]) + (1.001514e-04) * (q[i1] * q[i2]) + (-3.677597e-04) * (q[i1] * q[i3])
            + (-2.788492e-04) * (q[i1] * q[i4]) + (-1.026977e-03) * (q[i1] * q[i5]) + (-7.395194e-04) * (q[i1] * q[i6]) + (3.387974e-04) * (q[i1] * q[i7])
            + (7.556071e-04) * (q[i1] * q[i8]) + (-5.296953e-04) * (q[i1] * q[i9]) + (-9.232939e-05) * (q[i1] * q[i10]) + (-2.026858e-05) * (q[i1] * q[i11])
            + (1.941888e-04) * (q[i1] * q[i12]) + (2.186881e-04) * (q[i1] * q[i15]) + (1.262092e-04) * (q[i1] * q[i19]) + (8.947500e-05) * (q[i1] * q[i20])
            + (1.137587e-04) * (q[i1] * q[i21]) + (3.327511e-04) * (q[i1] * q[i22]) + (-2.515278e-03) * (q[i2] * q[i3]) + (1.011946e-03) * (q[i2] * q[i4])
            + (-6.624475e-03) * (q[i2] * q[i5]) + (4.201134e-04) * (q[i2] * q[i6]) + (1.335803e-03) * (q[i2] * q[i7]) + (9.360359e-04) * (q[i2] * q[i8])
            + (-5.789794e-05) * (q[i2] * q[i9]) + (-4.083522e-04) * (q[i2] * q[i10]) + (-5.016997e-04) * (q[i2] * q[i11]) + (-2.336272e-03) * (q[i2] * q[i12])
            + (-4.108729e-06) * (q[i2] * q[i15]) + (-1.355870e-03) * (q[i2] * q[i19]) + (3.529748e-04) * (q[i2] * q[i20]) + (4.025737e-05) * (q[i2] * q[i21])
            + (7.495992e-04) * (q[i2] * q[i22]) + (-3.478787e-04) * (q[i3] * q[i4]) + (9.957671e-04) * (q[i3] * q[i5]) + (1.322964e-03) * (q[i3] * q[i6])
            + (-7.606796e-04) * (q[i3] * q[i7]) + (1.745948e-03) * (q[i3] * q[i8]) + (-7.010736e-04) * (q[i3] * q[i9]) + (4.747305e-04) * (q[i3] * q[i10])
            + (-3.870236e-05) * (q[i3] * q[i11]) + (-4.166864e-04) * (q[i3] * q[i12]) + (-4.291182e-04) * (q[i3] * q[i15]) + (-4.696699e-04) * (q[i3] * q[i19])
            + (1.838969e-04) * (q[i3] * q[i20]) + (-4.494096e-05) * (q[i3] * q[i21]) + (2.849656e-04) * (q[i3] * q[i22]) + (-1.694699e-03) * (q[i4] * q[i5])
            + (-1.475898e-03) * (q[i4] * q[i6]) + (6.214028e-04) * (q[i4] * q[i7]) + (2.215468e-03) * (q[i4] * q[i8]) + (4.904249e-04) * (q[i4] * q[i9])
            + (2.553292e-04) * (q[i4] * q[i10]) + (1.070191e-03) * (q[i4] * q[i11]) + (-9.062632e-04) * (q[i4] * q[i12]) + (4.269459e-04) * (q[i4] * q[i15])
            + (3.716815e-04) * (q[i4] * q[i19]) + (1.078294e-03) * (q[i4] * q[i20]) + (-7.797041e-04) * (q[i4] * q[i21]) + (-7.098311e-04) * (q[i4] * q[i22])
            + (-3.613898e-04) * (q[i5] * q[i6]) + (-6.492493e-04) * (q[i5] * q[i7]) + (-2.476418e-03) * (q[i5] * q[i8]) + (-8.526925e-04) * (q[i5] * q[i9])
            + (-4.068580e-04) * (q[i5] * q[i10]) + (-3.355602e-04) * (q[i5] * q[i11]) + (-2.540585e-03) * (q[i5] * q[i12]) + (-1.584383e-06) * (q[i5] * q[i15])
            + (4.420311e-04) * (q[i5] * q[i19]) + (4.188070e-04) * (q[i5] * q[i20]) + (4.852346e-04) * (q[i5] * q[i21]) + (6.692534e-04) * (q[i5] * q[i22])
            + (6.339042e-04) * (q[i6] * q[i7]) + (-7.840902e-04) * (q[i6] * q[i8]) + (-8.358086e-05) * (q[i6] * q[i9]) + (2.798608e-04) * (q[i6] * q[i10])
            + (3.723862e-04) * (q[i6] * q[i11]) + (-5.408422e-04) * (q[i6] * q[i12]) + (-2.138078e-05) * (q[i6] * q[i15]) + (1.010537e-03) * (q[i6] * q[i19])
            + (-2.576356e-04) * (q[i6] * q[i20]) + (1.806325e-04) * (q[i6] * q[i21]) + (-7.857355e-04) * (q[i6] * q[i22]) + (7.769723e-04) * (q[i7] * q[i8])
            + (1.170306e-05) * (q[i7] * q[i9]) + (-4.451027e-04) * (q[i7] * q[i10]) + (-3.163262e-05) * (q[i7] * q[i11]) + (-4.156540e-04) * (q[i7] * q[i12])
            + (-2.752398e-05) * (q[i7] * q[i15]) + (-4.187241e-04) * (q[i7] * q[i19]) + (-4.536671e-05) * (q[i7] * q[i20]) + (-4.937533e-05) * (q[i7] * q[i21])
            + (3.086174e-04) * (q[i7] * q[i22]) + (-1.652432e-04) * (q[i8] * q[i9]) + (-3.077152e-04) * (q[i8] * q[i10]) + (2.049559e-04) * (q[i8] * q[i11])
            + (9.615786e-04) * (q[i8] * q[i12]) + (-2.463027e-04) * (q[i8] * q[i15]) + (-7.659774e-04) * (q[i8] * q[i19]) + (1.855496e-04) * (q[i8] * q[i20])
            + (-2.341260e-04) * (q[i8] * q[i21]) + (1.516622e-03) * (q[i8] * q[i22]) + (2.970809e-04) * (q[i9] * q[i10]) + (-1.070767e-04) * (q[i9] * q[i11])
            + (2.425242e-04) * (q[i9] * q[i12]) + (2.926568e-04) * (q[i9] * q[i15]) + (4.823402e-04) * (q[i9] * q[i19]) + (2.434512e-04) * (q[i9] * q[i20])
            + (-8.190503e-05) * (q[i9] * q[i21]) + (-1.549280e-04) * (q[i9] * q[i22]) + (-2.551507e-04) * (q[i10] * q[i11])
            + (-1.365703e-04) * (q[i10] * q[i12]) + (2.893244e-04) * (q[i10] * q[i15]) + (2.265096e-04) * (q[i10] * q[i19])
            + (-1.238974e-04) * (q[i10] * q[i20]) + (3.186902e-05) * (q[i10] * q[i21]) + (-2.368354e-06) * (q[i10] * q[i22])
            + (2.901597e-05) * (q[i11] * q[i12]) + (-1.103244e-04) * (q[i11] * q[i15]) + (-5.783379e-04) * (q[i11] * q[i19])
            + (-1.003911e-04) * (q[i11] * q[i20]) + (1.560478e-04) * (q[i11] * q[i21]) + (-5.117911e-05) * (q[i11] * q[i22])
            + (1.079393e-04) * (q[i12] * q[i15]) + (5.335886e-04) * (q[i12] * q[i19]) + (-1.164256e-04) * (q[i12] * q[i20]) + (1.596939e-04) * (q[i12] * q[i21])
            + (-8.637346e-05) * (q[i12] * q[i22]) + (2.676551e-04) * (q[i15] * q[i19]) + (2.627945e-04) * (q[i15] * q[i20]) + (6.172918e-05) * (q[i15] * q[i21])
            + (-6.089486e-05) * (q[i15] * q[i22]) + (9.661418e-05) * (q[i19] * q[i20]) + (9.483944e-05) * (q[i19] * q[i21]) + (1.345618e-04) * (q[i19] * q[i22])
            + (2.487113e-04) * (q[i20] * q[i21]) + (8.598867e-04) * (q[i20] * q[i22]) + (-4.249707e-05) * (q[i21] * q[i22]);
   }

   public void getJQy19(double[] q, double[][] JQ)
   {
      JQ[2][i19] = (-1.357233e-03) * (1) + (-2.122095e-05) * ((2) * q[i19]) + (-1.921777e-03) * (q[i0]) + (-8.235971e-04) * (q[i1]) + (-2.725771e-03) * (q[i2])
            + (-3.041974e-03) * (q[i3]) + (1.584654e-03) * (q[i4]) + (-6.688310e-04) * (q[i5]) + (-5.073938e-04) * (q[i6]) + (-9.229792e-04) * (q[i7])
            + (-2.190352e-03) * (q[i8]) + (7.876256e-05) * (q[i9]) + (1.160841e-03) * (q[i10]) + (-9.842817e-04) * (q[i11]) + (7.347940e-04) * (q[i12])
            + (6.894907e-04) * (q[i15]) + (-8.045054e-04) * (q[i16]) + (5.521720e-04) * (q[i20]) + (1.012404e-03) * (q[i21]) + (-1.535273e-04) * (q[i22])
            + (8.855494e-05) * (q[i0] * q[i0]) + (7.919462e-04) * (q[i1] * q[i1]) + (-6.277662e-04) * (q[i2] * q[i2]) + (1.215654e-03) * (q[i3] * q[i3])
            + (-4.610605e-04) * (q[i4] * q[i4]) + (-1.718998e-03) * (q[i5] * q[i5]) + (-1.054806e-03) * (q[i6] * q[i6]) + (-4.924212e-05) * (q[i7] * q[i7])
            + (-1.430933e-05) * (q[i8] * q[i8]) + (1.068099e-04) * (q[i9] * q[i9]) + (-2.521337e-04) * (q[i10] * q[i10]) + (6.309429e-04) * (q[i11] * q[i11])
            + (-1.360159e-05) * (q[i12] * q[i12]) + (1.617819e-04) * (q[i15] * q[i15]) + (-3.098556e-04) * (q[i16] * q[i16])
            + (-1.345468e-04) * ((2) * q[i0] * q[i19]) + (-2.431014e-04) * ((2) * q[i1] * q[i19]) + (3.508930e-06) * ((2) * q[i2] * q[i19])
            + (-3.874298e-04) * ((2) * q[i3] * q[i19]) + (1.838183e-04) * ((2) * q[i4] * q[i19]) + (-8.537189e-04) * ((2) * q[i5] * q[i19])
            + (-6.359441e-04) * ((2) * q[i6] * q[i19]) + (3.946263e-04) * ((2) * q[i7] * q[i19]) + (-4.900689e-04) * ((2) * q[i8] * q[i19])
            + (-1.551092e-04) * ((2) * q[i9] * q[i19]) + (-1.400187e-04) * ((2) * q[i10] * q[i19]) + (-3.553330e-04) * ((2) * q[i11] * q[i19])
            + (1.844954e-04) * ((2) * q[i12] * q[i19]) + (-1.633945e-04) * ((2) * q[i15] * q[i19]) + (-4.874503e-05) * ((2) * q[i16] * q[i19])
            + (2.056915e-04) * ((3) * q[i19] * q[i19]) + (5.381839e-05) * ((2) * q[i19] * q[i20]) + (-3.067863e-04) * ((2) * q[i19] * q[i21])
            + (4.524680e-05) * ((2) * q[i19] * q[i22]) + (5.310574e-05) * (q[i20] * q[i20]) + (5.306266e-04) * (q[i21] * q[i21])
            + (-1.313933e-04) * (q[i22] * q[i22]) + (-2.149220e-03) * (q[i0] * q[i1]) + (-5.308325e-04) * (q[i0] * q[i2]) + (-1.283668e-03) * (q[i0] * q[i3])
            + (-5.715602e-06) * (q[i0] * q[i4]) + (6.490070e-04) * (q[i0] * q[i5]) + (-1.498094e-03) * (q[i0] * q[i6]) + (-1.286637e-03) * (q[i0] * q[i7])
            + (-3.694046e-04) * (q[i0] * q[i8]) + (-1.799168e-04) * (q[i0] * q[i9]) + (2.148813e-04) * (q[i0] * q[i10]) + (5.389142e-04) * (q[i0] * q[i11])
            + (-5.457165e-04) * (q[i0] * q[i12]) + (-9.731131e-05) * (q[i0] * q[i15]) + (-5.863536e-04) * (q[i0] * q[i16]) + (3.099871e-04) * (q[i0] * q[i20])
            + (2.827223e-04) * (q[i0] * q[i21]) + (2.580421e-04) * (q[i0] * q[i22]) + (-7.448692e-04) * (q[i1] * q[i2]) + (-6.588636e-04) * (q[i1] * q[i3])
            + (-1.337410e-03) * (q[i1] * q[i4]) + (-5.729034e-05) * (q[i1] * q[i5]) + (-2.937084e-04) * (q[i1] * q[i6]) + (-7.119287e-04) * (q[i1] * q[i7])
            + (2.228837e-04) * (q[i1] * q[i8]) + (4.403208e-04) * (q[i1] * q[i9]) + (-5.613018e-04) * (q[i1] * q[i10]) + (6.352125e-05) * (q[i1] * q[i11])
            + (4.266644e-04) * (q[i1] * q[i12]) + (-1.743718e-04) * (q[i1] * q[i15]) + (1.262092e-04) * (q[i1] * q[i16]) + (-3.021009e-04) * (q[i1] * q[i20])
            + (-2.477809e-05) * (q[i1] * q[i21]) + (-1.043606e-04) * (q[i1] * q[i22]) + (1.276545e-04) * (q[i2] * q[i3]) + (8.152563e-05) * (q[i2] * q[i4])
            + (1.439647e-04) * (q[i2] * q[i5]) + (-3.549272e-04) * (q[i2] * q[i6]) + (-7.598168e-04) * (q[i2] * q[i7]) + (1.023466e-03) * (q[i2] * q[i8])
            + (-2.610005e-04) * (q[i2] * q[i9]) + (-1.011537e-04) * (q[i2] * q[i10]) + (6.703634e-04) * (q[i2] * q[i11]) + (-4.444471e-04) * (q[i2] * q[i12])
            + (-3.347044e-04) * (q[i2] * q[i15]) + (-1.355870e-03) * (q[i2] * q[i16]) + (9.810085e-06) * (q[i2] * q[i20]) + (-6.799983e-04) * (q[i2] * q[i21])
            + (-8.525243e-05) * (q[i2] * q[i22]) + (1.436047e-03) * (q[i3] * q[i4]) + (-1.447273e-03) * (q[i3] * q[i5]) + (5.983288e-04) * (q[i3] * q[i6])
            + (2.271741e-04) * (q[i3] * q[i7]) + (-1.824331e-04) * (q[i3] * q[i8]) + (3.544132e-04) * (q[i3] * q[i9]) + (2.506987e-04) * (q[i3] * q[i10])
            + (-1.712573e-04) * (q[i3] * q[i11]) + (8.904234e-05) * (q[i3] * q[i12]) + (-1.090363e-03) * (q[i3] * q[i15]) + (-4.696699e-04) * (q[i3] * q[i16])
            + (7.589383e-04) * (q[i3] * q[i20]) + (7.291109e-05) * (q[i3] * q[i21]) + (2.006097e-04) * (q[i3] * q[i22]) + (2.752440e-04) * (q[i4] * q[i5])
            + (-1.344769e-04) * (q[i4] * q[i6]) + (5.267683e-04) * (q[i4] * q[i7]) + (-9.381795e-05) * (q[i4] * q[i8]) + (-6.228559e-04) * (q[i4] * q[i9])
            + (6.283218e-04) * (q[i4] * q[i10]) + (-1.119821e-04) * (q[i4] * q[i11]) + (-7.938514e-04) * (q[i4] * q[i12]) + (-1.936850e-04) * (q[i4] * q[i15])
            + (3.716815e-04) * (q[i4] * q[i16]) + (-7.567033e-04) * (q[i4] * q[i20]) + (8.571854e-04) * (q[i4] * q[i21]) + (-7.366768e-04) * (q[i4] * q[i22])
            + (-9.828295e-04) * (q[i5] * q[i6]) + (2.210279e-04) * (q[i5] * q[i7]) + (1.505927e-04) * (q[i5] * q[i8]) + (6.817273e-04) * (q[i5] * q[i9])
            + (-2.698364e-04) * (q[i5] * q[i10]) + (-3.721245e-04) * (q[i5] * q[i11]) + (3.623418e-04) * (q[i5] * q[i12]) + (-4.229998e-04) * (q[i5] * q[i15])
            + (4.420311e-04) * (q[i5] * q[i16]) + (2.482564e-06) * (q[i5] * q[i20]) + (6.058628e-04) * (q[i5] * q[i21]) + (9.127125e-05) * (q[i5] * q[i22])
            + (2.586580e-04) * (q[i6] * q[i7]) + (-5.448643e-05) * (q[i6] * q[i8]) + (1.934901e-04) * (q[i6] * q[i9]) + (-1.757855e-05) * (q[i6] * q[i10])
            + (-1.640755e-04) * (q[i6] * q[i11]) + (-9.819785e-04) * (q[i6] * q[i12]) + (-6.097696e-05) * (q[i6] * q[i15]) + (1.010537e-03) * (q[i6] * q[i16])
            + (-3.891080e-04) * (q[i6] * q[i20]) + (3.961905e-04) * (q[i6] * q[i21]) + (-1.491139e-04) * (q[i6] * q[i22]) + (-2.197780e-04) * (q[i7] * q[i8])
            + (4.661810e-04) * (q[i7] * q[i9]) + (-2.948753e-04) * (q[i7] * q[i10]) + (-7.524544e-05) * (q[i7] * q[i11]) + (2.504454e-04) * (q[i7] * q[i12])
            + (-2.515684e-04) * (q[i7] * q[i15]) + (-4.187241e-04) * (q[i7] * q[i16]) + (-3.844589e-04) * (q[i7] * q[i20]) + (-2.231712e-04) * (q[i7] * q[i21])
            + (-1.014804e-04) * (q[i7] * q[i22]) + (-1.399525e-04) * (q[i8] * q[i9]) + (8.625016e-04) * (q[i8] * q[i10]) + (7.583491e-04) * (q[i8] * q[i11])
            + (-1.190358e-03) * (q[i8] * q[i12]) + (1.797364e-04) * (q[i8] * q[i15]) + (-7.659774e-04) * (q[i8] * q[i16]) + (6.328515e-04) * (q[i8] * q[i20])
            + (-1.120134e-03) * (q[i8] * q[i21]) + (-4.248339e-04) * (q[i8] * q[i22]) + (3.917550e-05) * (q[i9] * q[i10]) + (1.100914e-04) * (q[i9] * q[i11])
            + (1.687963e-04) * (q[i9] * q[i12]) + (-1.205072e-04) * (q[i9] * q[i15]) + (4.823402e-04) * (q[i9] * q[i16]) + (-1.338109e-04) * (q[i9] * q[i20])
            + (1.220038e-04) * (q[i9] * q[i21]) + (1.010255e-04) * (q[i9] * q[i22]) + (-3.946347e-04) * (q[i10] * q[i11]) + (-5.485815e-04) * (q[i10] * q[i12])
            + (2.416269e-04) * (q[i10] * q[i15]) + (2.265096e-04) * (q[i10] * q[i16]) + (-1.316243e-04) * (q[i10] * q[i20]) + (1.747631e-04) * (q[i10] * q[i21])
            + (-1.190671e-04) * (q[i10] * q[i22]) + (-2.141351e-04) * (q[i11] * q[i12]) + (1.222465e-04) * (q[i11] * q[i15])
            + (-5.783379e-04) * (q[i11] * q[i16]) + (1.460629e-04) * (q[i11] * q[i20]) + (1.258352e-04) * (q[i11] * q[i21])
            + (-9.818217e-05) * (q[i11] * q[i22]) + (9.948236e-05) * (q[i12] * q[i15]) + (5.335886e-04) * (q[i12] * q[i16])
            + (-1.457333e-04) * (q[i12] * q[i20]) + (-1.774228e-04) * (q[i12] * q[i21]) + (-8.433190e-05) * (q[i12] * q[i22])
            + (2.676551e-04) * (q[i15] * q[i16]) + (9.631664e-05) * (q[i15] * q[i20]) + (-8.679254e-04) * (q[i15] * q[i21])
            + (-2.501875e-04) * (q[i15] * q[i22]) + (9.661418e-05) * (q[i16] * q[i20]) + (9.483944e-05) * (q[i16] * q[i21]) + (1.345618e-04) * (q[i16] * q[i22])
            + (3.544925e-04) * (q[i20] * q[i21]) + (-3.544988e-04) * (q[i20] * q[i22]) + (3.561850e-06) * (q[i21] * q[i22]);
   }

   public void getJQy20(double[] q, double[][] JQ)
   {
      JQ[2][i20] = (-1.260901e-03) * (1) + (-7.609279e-06) * ((2) * q[i20]) + (8.267937e-04) * (q[i0]) + (1.904894e-03) * (q[i1]) + (2.709918e-03) * (q[i2])
            + (-1.632224e-03) * (q[i3]) + (3.021591e-03) * (q[i4]) + (6.817503e-04) * (q[i5]) + (-9.342910e-04) * (q[i6]) + (-4.789234e-04) * (q[i7])
            + (-2.185457e-03) * (q[i8]) + (1.164829e-03) * (q[i9]) + (6.927503e-05) * (q[i10]) + (-7.222546e-04) * (q[i11]) + (9.576683e-04) * (q[i12])
            + (-7.798828e-04) * (q[i15]) + (6.796655e-04) * (q[i16]) + (5.521720e-04) * (q[i19]) + (1.546839e-04) * (q[i21]) + (-1.037990e-03) * (q[i22])
            + (7.927038e-04) * (q[i0] * q[i0]) + (9.471634e-05) * (q[i1] * q[i1]) + (-6.472335e-04) * (q[i2] * q[i2]) + (-4.721584e-04) * (q[i3] * q[i3])
            + (1.217230e-03) * (q[i4] * q[i4]) + (-1.704766e-03) * (q[i5] * q[i5]) + (-5.433964e-05) * (q[i6] * q[i6]) + (-1.049889e-03) * (q[i7] * q[i7])
            + (-2.542223e-05) * (q[i8] * q[i8]) + (-2.559619e-04) * (q[i9] * q[i9]) + (1.029168e-04) * (q[i10] * q[i10]) + (-2.318441e-05) * (q[i11] * q[i11])
            + (6.134458e-04) * (q[i12] * q[i12]) + (-3.049154e-04) * (q[i15] * q[i15]) + (1.564144e-04) * (q[i16] * q[i16]) + (5.381839e-05) * (q[i19] * q[i19])
            + (2.372272e-04) * ((2) * q[i0] * q[i20]) + (1.360576e-04) * ((2) * q[i1] * q[i20]) + (-2.837281e-06) * ((2) * q[i2] * q[i20])
            + (-1.862724e-04) * ((2) * q[i3] * q[i20]) + (3.691705e-04) * ((2) * q[i4] * q[i20]) + (8.604417e-04) * ((2) * q[i5] * q[i20])
            + (3.957655e-04) * ((2) * q[i6] * q[i20]) + (-6.289636e-04) * ((2) * q[i7] * q[i20]) + (-4.881255e-04) * ((2) * q[i8] * q[i20])
            + (-1.402449e-04) * ((2) * q[i9] * q[i20]) + (-1.564151e-04) * ((2) * q[i10] * q[i20]) + (-1.814366e-04) * ((2) * q[i11] * q[i20])
            + (3.580127e-04) * ((2) * q[i12] * q[i20]) + (-5.054305e-05) * ((2) * q[i15] * q[i20]) + (-1.522340e-04) * ((2) * q[i16] * q[i20])
            + (5.310574e-05) * ((2) * q[i19] * q[i20]) + (2.032525e-04) * ((3) * q[i20] * q[i20]) + (-4.372817e-05) * ((2) * q[i20] * q[i21])
            + (3.035206e-04) * ((2) * q[i20] * q[i22]) + (-1.304610e-04) * (q[i21] * q[i21]) + (5.409582e-04) * (q[i22] * q[i22])
            + (-2.164970e-03) * (q[i0] * q[i1]) + (-7.462804e-04) * (q[i0] * q[i2]) + (-1.352818e-03) * (q[i0] * q[i3]) + (-6.549558e-04) * (q[i0] * q[i4])
            + (-3.652971e-05) * (q[i0] * q[i5]) + (7.182522e-04) * (q[i0] * q[i6]) + (3.037250e-04) * (q[i0] * q[i7]) + (-2.317022e-04) * (q[i0] * q[i8])
            + (5.617884e-04) * (q[i0] * q[i9]) + (-4.319454e-04) * (q[i0] * q[i10]) + (4.192410e-04) * (q[i0] * q[i11]) + (6.830868e-05) * (q[i0] * q[i12])
            + (-1.329828e-04) * (q[i0] * q[i15]) + (1.775150e-04) * (q[i0] * q[i16]) + (3.099871e-04) * (q[i0] * q[i19]) + (-1.022213e-04) * (q[i0] * q[i21])
            + (-2.220662e-05) * (q[i0] * q[i22]) + (-5.387324e-04) * (q[i1] * q[i2]) + (1.403780e-05) * (q[i1] * q[i3]) + (-1.289726e-03) * (q[i1] * q[i4])
            + (6.226303e-04) * (q[i1] * q[i5]) + (1.270226e-03) * (q[i1] * q[i6]) + (1.484588e-03) * (q[i1] * q[i7]) + (3.807387e-04) * (q[i1] * q[i8])
            + (-2.157064e-04) * (q[i1] * q[i9]) + (1.733368e-04) * (q[i1] * q[i10]) + (-5.394431e-04) * (q[i1] * q[i11]) + (5.463806e-04) * (q[i1] * q[i12])
            + (5.877674e-04) * (q[i1] * q[i15]) + (8.947500e-05) * (q[i1] * q[i16]) + (-3.021009e-04) * (q[i1] * q[i19]) + (2.473619e-04) * (q[i1] * q[i21])
            + (2.811799e-04) * (q[i1] * q[i22]) + (6.979532e-05) * (q[i2] * q[i3]) + (1.180434e-04) * (q[i2] * q[i4]) + (1.530245e-04) * (q[i2] * q[i5])
            + (7.624220e-04) * (q[i2] * q[i6]) + (3.408859e-04) * (q[i2] * q[i7]) + (-1.027806e-03) * (q[i2] * q[i8]) + (9.740003e-05) * (q[i2] * q[i9])
            + (2.619118e-04) * (q[i2] * q[i10]) + (-4.443000e-04) * (q[i2] * q[i11]) + (6.501908e-04) * (q[i2] * q[i12]) + (1.346695e-03) * (q[i2] * q[i15])
            + (3.529748e-04) * (q[i2] * q[i16]) + (9.810085e-06) * (q[i2] * q[i19]) + (-9.238074e-05) * (q[i2] * q[i21]) + (-6.741689e-04) * (q[i2] * q[i22])
            + (1.413824e-03) * (q[i3] * q[i4]) + (2.859915e-04) * (q[i3] * q[i5]) + (-5.325464e-04) * (q[i3] * q[i6]) + (1.320482e-04) * (q[i3] * q[i7])
            + (9.336729e-05) * (q[i3] * q[i8]) + (-6.354895e-04) * (q[i3] * q[i9]) + (6.488983e-04) * (q[i3] * q[i10]) + (-8.019534e-04) * (q[i3] * q[i11])
            + (-1.091851e-04) * (q[i3] * q[i12]) + (-3.773423e-04) * (q[i3] * q[i15]) + (1.838969e-04) * (q[i3] * q[i16]) + (7.589383e-04) * (q[i3] * q[i19])
            + (-7.413924e-04) * (q[i3] * q[i21]) + (8.525907e-04) * (q[i3] * q[i22]) + (-1.445958e-03) * (q[i4] * q[i5]) + (-2.211296e-04) * (q[i4] * q[i6])
            + (-6.071990e-04) * (q[i4] * q[i7]) + (1.747690e-04) * (q[i4] * q[i8]) + (-2.543523e-04) * (q[i4] * q[i9]) + (-3.471171e-04) * (q[i4] * q[i10])
            + (9.979069e-05) * (q[i4] * q[i11]) + (-1.668851e-04) * (q[i4] * q[i12]) + (4.801148e-04) * (q[i4] * q[i15]) + (1.078294e-03) * (q[i4] * q[i16])
            + (-7.567033e-04) * (q[i4] * q[i19]) + (1.948996e-04) * (q[i4] * q[i21]) + (7.178784e-05) * (q[i4] * q[i22]) + (-2.404216e-04) * (q[i5] * q[i6])
            + (9.826121e-04) * (q[i5] * q[i7]) + (-1.445753e-04) * (q[i5] * q[i8]) + (2.758747e-04) * (q[i5] * q[i9]) + (-7.060816e-04) * (q[i5] * q[i10])
            + (3.566828e-04) * (q[i5] * q[i11]) + (-3.867339e-04) * (q[i5] * q[i12]) + (-4.258683e-04) * (q[i5] * q[i15]) + (4.188070e-04) * (q[i5] * q[i16])
            + (2.482564e-06) * (q[i5] * q[i19]) + (9.610038e-05) * (q[i5] * q[i21]) + (6.030247e-04) * (q[i5] * q[i22]) + (2.497187e-04) * (q[i6] * q[i7])
            + (-2.203541e-04) * (q[i6] * q[i8]) + (-2.956798e-04) * (q[i6] * q[i9]) + (4.630654e-04) * (q[i6] * q[i10]) + (-2.434679e-04) * (q[i6] * q[i11])
            + (8.291197e-05) * (q[i6] * q[i12]) + (-4.208301e-04) * (q[i6] * q[i15]) + (-2.576356e-04) * (q[i6] * q[i16]) + (-3.891080e-04) * (q[i6] * q[i19])
            + (1.049546e-04) * (q[i6] * q[i21]) + (2.197945e-04) * (q[i6] * q[i22]) + (-3.898825e-05) * (q[i7] * q[i8]) + (-2.378912e-05) * (q[i7] * q[i9])
            + (1.864975e-04) * (q[i7] * q[i10]) + (9.860693e-04) * (q[i7] * q[i11]) + (1.631307e-04) * (q[i7] * q[i12]) + (1.008449e-03) * (q[i7] * q[i15])
            + (-4.536671e-05) * (q[i7] * q[i16]) + (-3.844589e-04) * (q[i7] * q[i19]) + (1.437196e-04) * (q[i7] * q[i21]) + (-3.856598e-04) * (q[i7] * q[i22])
            + (8.570201e-04) * (q[i8] * q[i9]) + (-1.244961e-04) * (q[i8] * q[i10]) + (1.177453e-03) * (q[i8] * q[i11]) + (-7.548895e-04) * (q[i8] * q[i12])
            + (-7.647453e-04) * (q[i8] * q[i15]) + (1.855496e-04) * (q[i8] * q[i16]) + (6.328515e-04) * (q[i8] * q[i19]) + (4.221432e-04) * (q[i8] * q[i21])
            + (1.111237e-03) * (q[i8] * q[i22]) + (4.137510e-05) * (q[i9] * q[i10]) + (5.449230e-04) * (q[i9] * q[i11]) + (3.931066e-04) * (q[i9] * q[i12])
            + (2.208531e-04) * (q[i9] * q[i15]) + (2.434512e-04) * (q[i9] * q[i16]) + (-1.338109e-04) * (q[i9] * q[i19]) + (1.195072e-04) * (q[i9] * q[i21])
            + (-1.742876e-04) * (q[i9] * q[i22]) + (-1.611441e-04) * (q[i10] * q[i11]) + (-1.178500e-04) * (q[i10] * q[i12])
            + (4.802103e-04) * (q[i10] * q[i15]) + (-1.238974e-04) * (q[i10] * q[i16]) + (-1.316243e-04) * (q[i10] * q[i19])
            + (-9.656492e-05) * (q[i10] * q[i21]) + (-1.132204e-04) * (q[i10] * q[i22]) + (-2.115579e-04) * (q[i11] * q[i12])
            + (-5.286889e-04) * (q[i11] * q[i15]) + (-1.003911e-04) * (q[i11] * q[i16]) + (1.460629e-04) * (q[i11] * q[i19])
            + (-8.726795e-05) * (q[i11] * q[i21]) + (-1.764124e-04) * (q[i11] * q[i22]) + (5.750484e-04) * (q[i12] * q[i15])
            + (-1.164256e-04) * (q[i12] * q[i16]) + (-1.457333e-04) * (q[i12] * q[i19]) + (-9.988231e-05) * (q[i12] * q[i21])
            + (1.307902e-04) * (q[i12] * q[i22]) + (2.627945e-04) * (q[i15] * q[i16]) + (9.631664e-05) * (q[i15] * q[i19]) + (-1.314745e-04) * (q[i15] * q[i21])
            + (-9.374580e-05) * (q[i15] * q[i22]) + (9.661418e-05) * (q[i16] * q[i19]) + (2.487113e-04) * (q[i16] * q[i21]) + (8.598867e-04) * (q[i16] * q[i22])
            + (3.544925e-04) * (q[i19] * q[i21]) + (-3.544988e-04) * (q[i19] * q[i22]) + (3.144747e-06) * (q[i21] * q[i22]);
   }

   public void getJQy21(double[] q, double[][] JQ)
   {
      JQ[2][i21] = (5.083198e-03) * (1) + (-1.589365e-04) * ((2) * q[i21]) + (2.222365e-03) * (q[i0]) + (-8.990263e-04) * (q[i1]) + (1.988143e-03) * (q[i2])
            + (-2.054358e-03) * (q[i3]) + (1.778881e-03) * (q[i4]) + (1.007175e-03) * (q[i5]) + (-2.525261e-04) * (q[i6]) + (-3.041800e-04) * (q[i7])
            + (-2.085760e-03) * (q[i8]) + (-7.003847e-04) * (q[i9]) + (-4.558118e-04) * (q[i10]) + (-1.187627e-03) * (q[i11]) + (3.132506e-05) * (q[i12])
            + (6.160006e-04) * (q[i15]) + (2.968374e-04) * (q[i16]) + (1.012404e-03) * (q[i19]) + (1.546839e-04) * (q[i20]) + (2.020928e-04) * (q[i22])
            + (1.487935e-04) * (q[i0] * q[i0]) + (-2.343892e-04) * (q[i1] * q[i1]) + (7.020029e-04) * (q[i2] * q[i2]) + (-7.272412e-04) * (q[i3] * q[i3])
            + (6.508448e-04) * (q[i4] * q[i4]) + (-1.473637e-03) * (q[i5] * q[i5]) + (-6.000541e-04) * (q[i6] * q[i6]) + (-1.035763e-03) * (q[i7] * q[i7])
            + (-5.663762e-04) * (q[i8] * q[i8]) + (5.934105e-06) * (q[i9] * q[i9]) + (2.119792e-04) * (q[i10] * q[i10]) + (1.754330e-04) * (q[i11] * q[i11])
            + (-1.951116e-04) * (q[i12] * q[i12]) + (-1.976457e-04) * (q[i15] * q[i15]) + (-3.724932e-06) * (q[i16] * q[i16])
            + (-3.067863e-04) * (q[i19] * q[i19]) + (-4.372817e-05) * (q[i20] * q[i20]) + (1.887070e-04) * ((2) * q[i0] * q[i21])
            + (-6.073279e-05) * ((2) * q[i1] * q[i21]) + (3.209924e-04) * ((2) * q[i2] * q[i21]) + (-2.119539e-04) * ((2) * q[i3] * q[i21])
            + (5.086029e-04) * ((2) * q[i4] * q[i21]) + (-4.984050e-04) * ((2) * q[i5] * q[i21]) + (1.102790e-04) * ((2) * q[i6] * q[i21])
            + (6.938603e-04) * ((2) * q[i7] * q[i21]) + (-7.177874e-04) * ((2) * q[i8] * q[i21]) + (-3.988740e-05) * ((2) * q[i9] * q[i21])
            + (1.020322e-04) * ((2) * q[i10] * q[i21]) + (-4.051638e-04) * ((2) * q[i11] * q[i21]) + (-2.526193e-05) * ((2) * q[i12] * q[i21])
            + (-1.613204e-04) * ((2) * q[i15] * q[i21]) + (1.226978e-04) * ((2) * q[i16] * q[i21]) + (5.306266e-04) * ((2) * q[i19] * q[i21])
            + (-1.304610e-04) * ((2) * q[i20] * q[i21]) + (-8.453739e-05) * ((3) * q[i21] * q[i21]) + (5.480708e-05) * ((2) * q[i21] * q[i22])
            + (-5.651128e-05) * (q[i22] * q[i22]) + (1.059114e-03) * (q[i0] * q[i1]) + (1.166162e-03) * (q[i0] * q[i2]) + (-4.281597e-05) * (q[i0] * q[i3])
            + (-1.410528e-04) * (q[i0] * q[i4]) + (-4.205140e-04) * (q[i0] * q[i5]) + (5.736209e-04) * (q[i0] * q[i6]) + (-4.422425e-04) * (q[i0] * q[i7])
            + (2.642182e-05) * (q[i0] * q[i8]) + (-7.818176e-05) * (q[i0] * q[i9]) + (-1.584458e-04) * (q[i0] * q[i10]) + (2.893217e-04) * (q[i0] * q[i11])
            + (-5.167213e-04) * (q[i0] * q[i12]) + (3.266881e-04) * (q[i0] * q[i15]) + (-2.073675e-04) * (q[i0] * q[i16]) + (2.827223e-04) * (q[i0] * q[i19])
            + (-1.022213e-04) * (q[i0] * q[i20]) + (-5.595093e-04) * (q[i0] * q[i22]) + (1.664055e-03) * (q[i1] * q[i2]) + (6.214166e-04) * (q[i1] * q[i3])
            + (7.876661e-04) * (q[i1] * q[i4]) + (-6.467642e-04) * (q[i1] * q[i5]) + (-3.794178e-04) * (q[i1] * q[i6]) + (-8.684813e-04) * (q[i1] * q[i7])
            + (-4.667108e-04) * (q[i1] * q[i8]) + (-2.141346e-04) * (q[i1] * q[i9]) + (-6.546301e-05) * (q[i1] * q[i10]) + (3.606199e-04) * (q[i1] * q[i11])
            + (-3.924730e-04) * (q[i1] * q[i12]) + (1.723671e-04) * (q[i1] * q[i15]) + (1.137587e-04) * (q[i1] * q[i16]) + (-2.477809e-05) * (q[i1] * q[i19])
            + (2.473619e-04) * (q[i1] * q[i20]) + (5.531782e-04) * (q[i1] * q[i22]) + (-6.013862e-04) * (q[i2] * q[i3]) + (-5.871365e-04) * (q[i2] * q[i4])
            + (2.058515e-03) * (q[i2] * q[i5]) + (4.757560e-04) * (q[i2] * q[i6]) + (-7.253175e-04) * (q[i2] * q[i7]) + (4.401218e-04) * (q[i2] * q[i8])
            + (-4.083199e-04) * (q[i2] * q[i9]) + (-3.418190e-04) * (q[i2] * q[i10]) + (9.345787e-04) * (q[i2] * q[i11]) + (-1.204995e-03) * (q[i2] * q[i12])
            + (7.605788e-04) * (q[i2] * q[i15]) + (4.025737e-05) * (q[i2] * q[i16]) + (-6.799983e-04) * (q[i2] * q[i19]) + (-9.238074e-05) * (q[i2] * q[i20])
            + (-1.328435e-06) * (q[i2] * q[i22]) + (-1.400512e-03) * (q[i3] * q[i4]) + (-1.096375e-03) * (q[i3] * q[i5]) + (-1.193203e-03) * (q[i3] * q[i6])
            + (-7.691139e-04) * (q[i3] * q[i7]) + (2.121110e-04) * (q[i3] * q[i8]) + (-2.459064e-04) * (q[i3] * q[i9]) + (1.378146e-05) * (q[i3] * q[i10])
            + (-8.460064e-04) * (q[i3] * q[i11]) + (-2.769233e-05) * (q[i3] * q[i12]) + (-7.025679e-04) * (q[i3] * q[i15]) + (-4.494096e-05) * (q[i3] * q[i16])
            + (7.291109e-05) * (q[i3] * q[i19]) + (-7.413924e-04) * (q[i3] * q[i20]) + (1.085187e-04) * (q[i3] * q[i22]) + (2.063944e-03) * (q[i4] * q[i5])
            + (1.419563e-03) * (q[i4] * q[i6]) + (-2.688400e-04) * (q[i4] * q[i7]) + (4.350437e-04) * (q[i4] * q[i8]) + (2.289478e-04) * (q[i4] * q[i9])
            + (-3.230970e-04) * (q[i4] * q[i10]) + (1.227838e-04) * (q[i4] * q[i11]) + (-4.133740e-04) * (q[i4] * q[i12]) + (2.930438e-04) * (q[i4] * q[i15])
            + (-7.797041e-04) * (q[i4] * q[i16]) + (8.571854e-04) * (q[i4] * q[i19]) + (1.948996e-04) * (q[i4] * q[i20]) + (-1.024263e-04) * (q[i4] * q[i22])
            + (-2.500503e-04) * (q[i5] * q[i6]) + (2.803336e-04) * (q[i5] * q[i7]) + (-5.209222e-05) * (q[i5] * q[i8]) + (-5.875879e-04) * (q[i5] * q[i9])
            + (-2.235920e-04) * (q[i5] * q[i10]) + (-5.596208e-04) * (q[i5] * q[i11]) + (2.657216e-04) * (q[i5] * q[i12]) + (6.511432e-04) * (q[i5] * q[i15])
            + (4.852346e-04) * (q[i5] * q[i16]) + (6.058628e-04) * (q[i5] * q[i19]) + (9.610038e-05) * (q[i5] * q[i20]) + (-2.869781e-06) * (q[i5] * q[i22])
            + (-2.658031e-04) * (q[i6] * q[i7]) + (-4.420825e-04) * (q[i6] * q[i8]) + (-2.630103e-04) * (q[i6] * q[i9]) + (-3.136931e-04) * (q[i6] * q[i10])
            + (-2.957810e-04) * (q[i6] * q[i11]) + (6.580356e-04) * (q[i6] * q[i12]) + (-3.006825e-04) * (q[i6] * q[i15]) + (1.806325e-04) * (q[i6] * q[i16])
            + (3.961905e-04) * (q[i6] * q[i19]) + (1.049546e-04) * (q[i6] * q[i20]) + (3.274916e-04) * (q[i6] * q[i22]) + (1.665584e-04) * (q[i7] * q[i8])
            + (2.457164e-06) * (q[i7] * q[i9]) + (2.096182e-04) * (q[i7] * q[i10]) + (-1.315688e-06) * (q[i7] * q[i11]) + (-6.068143e-04) * (q[i7] * q[i12])
            + (7.773756e-04) * (q[i7] * q[i15]) + (-4.937533e-05) * (q[i7] * q[i16]) + (-2.231712e-04) * (q[i7] * q[i19]) + (1.437196e-04) * (q[i7] * q[i20])
            + (3.294441e-04) * (q[i7] * q[i22]) + (9.173531e-05) * (q[i8] * q[i9]) + (2.951818e-04) * (q[i8] * q[i10]) + (-3.791023e-04) * (q[i8] * q[i11])
            + (-2.082552e-04) * (q[i8] * q[i12]) + (-1.503288e-03) * (q[i8] * q[i15]) + (-2.341260e-04) * (q[i8] * q[i16]) + (-1.120134e-03) * (q[i8] * q[i19])
            + (4.221432e-04) * (q[i8] * q[i20]) + (-4.557013e-04) * (q[i8] * q[i22]) + (7.523850e-05) * (q[i9] * q[i10]) + (-5.803486e-05) * (q[i9] * q[i11])
            + (-2.334586e-05) * (q[i9] * q[i12]) + (5.189756e-06) * (q[i9] * q[i15]) + (-8.190503e-05) * (q[i9] * q[i16]) + (1.220038e-04) * (q[i9] * q[i19])
            + (1.195072e-04) * (q[i9] * q[i20]) + (6.391691e-05) * (q[i9] * q[i22]) + (-1.973640e-06) * (q[i10] * q[i11]) + (8.537272e-05) * (q[i10] * q[i12])
            + (1.516216e-04) * (q[i10] * q[i15]) + (3.186902e-05) * (q[i10] * q[i16]) + (1.747631e-04) * (q[i10] * q[i19]) + (-9.656492e-05) * (q[i10] * q[i20])
            + (6.457753e-05) * (q[i10] * q[i22]) + (3.290286e-06) * (q[i11] * q[i12]) + (-7.061340e-05) * (q[i11] * q[i15]) + (1.560478e-04) * (q[i11] * q[i16])
            + (1.258352e-04) * (q[i11] * q[i19]) + (-8.726795e-05) * (q[i11] * q[i20]) + (-1.611880e-05) * (q[i11] * q[i22])
            + (-4.713812e-05) * (q[i12] * q[i15]) + (1.596939e-04) * (q[i12] * q[i16]) + (-1.774228e-04) * (q[i12] * q[i19])
            + (-9.988231e-05) * (q[i12] * q[i20]) + (1.312943e-05) * (q[i12] * q[i22]) + (6.172918e-05) * (q[i15] * q[i16])
            + (-8.679254e-04) * (q[i15] * q[i19]) + (-1.314745e-04) * (q[i15] * q[i20]) + (-4.223805e-05) * (q[i15] * q[i22])
            + (9.483944e-05) * (q[i16] * q[i19]) + (2.487113e-04) * (q[i16] * q[i20]) + (-4.249707e-05) * (q[i16] * q[i22]) + (3.544925e-04) * (q[i19] * q[i20])
            + (3.561850e-06) * (q[i19] * q[i22]) + (3.144747e-06) * (q[i20] * q[i22]);
   }

   public void getJQy22(double[] q, double[][] JQ)
   {
      JQ[2][i22] = (-5.078924e-03) * (1) + (-1.608665e-04) * ((2) * q[i22]) + (-8.863129e-04) * (q[i0]) + (2.234868e-03) * (q[i1]) + (1.965185e-03) * (q[i2])
            + (1.763675e-03) * (q[i3]) + (-2.025126e-03) * (q[i4]) + (1.007538e-03) * (q[i5]) + (3.042374e-04) * (q[i6]) + (2.464662e-04) * (q[i7])
            + (2.088335e-03) * (q[i8]) + (4.554292e-04) * (q[i9]) + (6.953218e-04) * (q[i10]) + (1.108089e-05) * (q[i11]) + (-1.201508e-03) * (q[i12])
            + (-3.030791e-04) * (q[i15]) + (-6.132847e-04) * (q[i16]) + (-1.535273e-04) * (q[i19]) + (-1.037990e-03) * (q[i20]) + (2.020928e-04) * (q[i21])
            + (2.358285e-04) * (q[i0] * q[i0]) + (-1.491548e-04) * (q[i1] * q[i1]) + (-7.202685e-04) * (q[i2] * q[i2]) + (-6.555301e-04) * (q[i3] * q[i3])
            + (7.148945e-04) * (q[i4] * q[i4]) + (1.495075e-03) * (q[i5] * q[i5]) + (1.043639e-03) * (q[i6] * q[i6]) + (5.882127e-04) * (q[i7] * q[i7])
            + (5.723610e-04) * (q[i8] * q[i8]) + (-2.138896e-04) * (q[i9] * q[i9]) + (-6.543407e-06) * (q[i10] * q[i10]) + (1.907818e-04) * (q[i11] * q[i11])
            + (-1.700646e-04) * (q[i12] * q[i12]) + (9.307695e-07) * (q[i15] * q[i15]) + (2.048794e-04) * (q[i16] * q[i16]) + (4.524680e-05) * (q[i19] * q[i19])
            + (3.035206e-04) * (q[i20] * q[i20]) + (5.480708e-05) * (q[i21] * q[i21]) + (4.910673e-05) * ((2) * q[i0] * q[i22])
            + (-1.891935e-04) * ((2) * q[i1] * q[i22]) + (-3.183774e-04) * ((2) * q[i2] * q[i22]) + (-5.071852e-04) * ((2) * q[i3] * q[i22])
            + (2.075443e-04) * ((2) * q[i4] * q[i22]) + (5.071164e-04) * ((2) * q[i5] * q[i22]) + (6.934395e-04) * ((2) * q[i6] * q[i22])
            + (1.140094e-04) * ((2) * q[i7] * q[i22]) + (-7.183218e-04) * ((2) * q[i8] * q[i22]) + (1.021671e-04) * ((2) * q[i9] * q[i22])
            + (-4.066959e-05) * ((2) * q[i10] * q[i22]) + (2.422408e-05) * ((2) * q[i11] * q[i22]) + (4.178399e-04) * ((2) * q[i12] * q[i22])
            + (1.252246e-04) * ((2) * q[i15] * q[i22]) + (-1.634418e-04) * ((2) * q[i16] * q[i22]) + (-1.313933e-04) * ((2) * q[i19] * q[i22])
            + (5.409582e-04) * ((2) * q[i20] * q[i22]) + (-5.651128e-05) * ((2) * q[i21] * q[i22]) + (8.735746e-05) * ((3) * q[i22] * q[i22])
            + (-1.058629e-03) * (q[i0] * q[i1]) + (-1.663098e-03) * (q[i0] * q[i2]) + (-7.913839e-04) * (q[i0] * q[i3]) + (-6.179272e-04) * (q[i0] * q[i4])
            + (6.642214e-04) * (q[i0] * q[i5]) + (-8.699801e-04) * (q[i0] * q[i6]) + (-3.798863e-04) * (q[i0] * q[i7]) + (-4.572423e-04) * (q[i0] * q[i8])
            + (-6.660250e-05) * (q[i0] * q[i9]) + (-2.132184e-04) * (q[i0] * q[i10]) + (3.909004e-04) * (q[i0] * q[i11]) + (-3.713889e-04) * (q[i0] * q[i12])
            + (1.098482e-04) * (q[i0] * q[i15]) + (1.769337e-04) * (q[i0] * q[i16]) + (2.580421e-04) * (q[i0] * q[i19]) + (-2.220662e-05) * (q[i0] * q[i20])
            + (-5.595093e-04) * (q[i0] * q[i21]) + (-1.179484e-03) * (q[i1] * q[i2]) + (1.400097e-04) * (q[i1] * q[i3]) + (4.832829e-05) * (q[i1] * q[i4])
            + (4.217495e-04) * (q[i1] * q[i5]) + (-4.370185e-04) * (q[i1] * q[i6]) + (5.826807e-04) * (q[i1] * q[i7]) + (3.449386e-05) * (q[i1] * q[i8])
            + (-1.572557e-04) * (q[i1] * q[i9]) + (-7.665282e-05) * (q[i1] * q[i10]) + (5.179043e-04) * (q[i1] * q[i11]) + (-2.773281e-04) * (q[i1] * q[i12])
            + (-2.020610e-04) * (q[i1] * q[i15]) + (3.327511e-04) * (q[i1] * q[i16]) + (-1.043606e-04) * (q[i1] * q[i19]) + (2.811799e-04) * (q[i1] * q[i20])
            + (5.531782e-04) * (q[i1] * q[i21]) + (6.002039e-04) * (q[i2] * q[i3]) + (5.962369e-04) * (q[i2] * q[i4]) + (-2.067730e-03) * (q[i2] * q[i5])
            + (-7.125799e-04) * (q[i2] * q[i6]) + (4.696598e-04) * (q[i2] * q[i7]) + (4.437775e-04) * (q[i2] * q[i8]) + (-3.429924e-04) * (q[i2] * q[i9])
            + (-4.035359e-04) * (q[i2] * q[i10]) + (1.211605e-03) * (q[i2] * q[i11]) + (-9.273983e-04) * (q[i2] * q[i12]) + (3.999834e-05) * (q[i2] * q[i15])
            + (7.495992e-04) * (q[i2] * q[i16]) + (-8.525243e-05) * (q[i2] * q[i19]) + (-6.741689e-04) * (q[i2] * q[i20]) + (-1.328435e-06) * (q[i2] * q[i21])
            + (1.412147e-03) * (q[i3] * q[i4]) + (-2.078576e-03) * (q[i3] * q[i5]) + (-2.469359e-04) * (q[i3] * q[i6]) + (1.421559e-03) * (q[i3] * q[i7])
            + (4.277614e-04) * (q[i3] * q[i8]) + (-3.224698e-04) * (q[i3] * q[i9]) + (2.411005e-04) * (q[i3] * q[i10]) + (4.242010e-04) * (q[i3] * q[i11])
            + (-1.330506e-04) * (q[i3] * q[i12]) + (-7.839491e-04) * (q[i3] * q[i15]) + (2.849656e-04) * (q[i3] * q[i16]) + (2.006097e-04) * (q[i3] * q[i19])
            + (8.525907e-04) * (q[i3] * q[i20]) + (1.085187e-04) * (q[i3] * q[i21]) + (1.112460e-03) * (q[i4] * q[i5]) + (-7.808593e-04) * (q[i4] * q[i6])
            + (-1.188431e-03) * (q[i4] * q[i7]) + (2.177448e-04) * (q[i4] * q[i8]) + (-4.351189e-08) * (q[i4] * q[i9]) + (-2.538713e-04) * (q[i4] * q[i10])
            + (2.703891e-05) * (q[i4] * q[i11]) + (8.567602e-04) * (q[i4] * q[i12]) + (-5.318361e-05) * (q[i4] * q[i15]) + (-7.098311e-04) * (q[i4] * q[i16])
            + (-7.366768e-04) * (q[i4] * q[i19]) + (7.178784e-05) * (q[i4] * q[i20]) + (-1.024263e-04) * (q[i4] * q[i21]) + (2.925324e-04) * (q[i5] * q[i6])
            + (-2.304339e-04) * (q[i5] * q[i7]) + (-4.250972e-05) * (q[i5] * q[i8]) + (-2.189059e-04) * (q[i5] * q[i9]) + (-5.826465e-04) * (q[i5] * q[i10])
            + (-2.649668e-04) * (q[i5] * q[i11]) + (5.715204e-04) * (q[i5] * q[i12]) + (4.829936e-04) * (q[i5] * q[i15]) + (6.692534e-04) * (q[i5] * q[i16])
            + (9.127125e-05) * (q[i5] * q[i19]) + (6.030247e-04) * (q[i5] * q[i20]) + (-2.869781e-06) * (q[i5] * q[i21]) + (2.470120e-04) * (q[i6] * q[i7])
            + (-1.667029e-04) * (q[i6] * q[i8]) + (-2.043316e-04) * (q[i6] * q[i9]) + (-3.502427e-06) * (q[i6] * q[i10]) + (-6.079118e-04) * (q[i6] * q[i11])
            + (3.425389e-07) * (q[i6] * q[i12]) + (6.121058e-05) * (q[i6] * q[i15]) + (-7.857355e-04) * (q[i6] * q[i16]) + (-1.491139e-04) * (q[i6] * q[i19])
            + (2.197945e-04) * (q[i6] * q[i20]) + (3.274916e-04) * (q[i6] * q[i21]) + (4.488181e-04) * (q[i7] * q[i8]) + (3.049735e-04) * (q[i7] * q[i9])
            + (2.627882e-04) * (q[i7] * q[i10]) + (6.556906e-04) * (q[i7] * q[i11]) + (-2.887615e-04) * (q[i7] * q[i12]) + (-1.900134e-04) * (q[i7] * q[i15])
            + (3.086174e-04) * (q[i7] * q[i16]) + (-1.014804e-04) * (q[i7] * q[i19]) + (-3.856598e-04) * (q[i7] * q[i20]) + (3.294441e-04) * (q[i7] * q[i21])
            + (-3.030985e-04) * (q[i8] * q[i9]) + (-9.019046e-05) * (q[i8] * q[i10]) + (-2.098501e-04) * (q[i8] * q[i11]) + (-3.871480e-04) * (q[i8] * q[i12])
            + (2.291151e-04) * (q[i8] * q[i15]) + (1.516622e-03) * (q[i8] * q[i16]) + (-4.248339e-04) * (q[i8] * q[i19]) + (1.111237e-03) * (q[i8] * q[i20])
            + (-4.557013e-04) * (q[i8] * q[i21]) + (-7.457916e-05) * (q[i9] * q[i10]) + (8.276334e-05) * (q[i9] * q[i11]) + (3.840462e-07) * (q[i9] * q[i12])
            + (-3.313817e-05) * (q[i9] * q[i15]) + (-1.549280e-04) * (q[i9] * q[i16]) + (1.010255e-04) * (q[i9] * q[i19]) + (-1.742876e-04) * (q[i9] * q[i20])
            + (6.391691e-05) * (q[i9] * q[i21]) + (-1.972338e-05) * (q[i10] * q[i11]) + (-5.462330e-05) * (q[i10] * q[i12]) + (8.125625e-05) * (q[i10] * q[i15])
            + (-2.368354e-06) * (q[i10] * q[i16]) + (-1.190671e-04) * (q[i10] * q[i19]) + (-1.132204e-04) * (q[i10] * q[i20])
            + (6.457753e-05) * (q[i10] * q[i21]) + (9.393283e-08) * (q[i11] * q[i12]) + (1.524228e-04) * (q[i11] * q[i15]) + (-5.117911e-05) * (q[i11] * q[i16])
            + (-9.818217e-05) * (q[i11] * q[i19]) + (-1.764124e-04) * (q[i11] * q[i20]) + (-1.611880e-05) * (q[i11] * q[i21])
            + (1.608407e-04) * (q[i12] * q[i15]) + (-8.637346e-05) * (q[i12] * q[i16]) + (-8.433190e-05) * (q[i12] * q[i19])
            + (1.307902e-04) * (q[i12] * q[i20]) + (1.312943e-05) * (q[i12] * q[i21]) + (-6.089486e-05) * (q[i15] * q[i16])
            + (-2.501875e-04) * (q[i15] * q[i19]) + (-9.374580e-05) * (q[i15] * q[i20]) + (-4.223805e-05) * (q[i15] * q[i21])
            + (1.345618e-04) * (q[i16] * q[i19]) + (8.598867e-04) * (q[i16] * q[i20]) + (-4.249707e-05) * (q[i16] * q[i21])
            + (-3.544988e-04) * (q[i19] * q[i20]) + (3.561850e-06) * (q[i19] * q[i21]) + (3.144747e-06) * (q[i20] * q[i21]);
   }

   public void getJQz(double[] q, double[][] JQ)
   {
      getJQz0(q, JQ);
      getJQz1(q, JQ);
      getJQz2(q, JQ);
      getJQz3(q, JQ);
      getJQz4(q, JQ);
      getJQz5(q, JQ);
      getJQz6(q, JQ);
      getJQz7(q, JQ);
      getJQz8(q, JQ);
      getJQz9(q, JQ);
      getJQz10(q, JQ);
      getJQz11(q, JQ);
      getJQz12(q, JQ);
      getJQz15(q, JQ);
      getJQz16(q, JQ);
      getJQz19(q, JQ);
      getJQz20(q, JQ);
      getJQz21(q, JQ);
      getJQz22(q, JQ);
   }

   public void getJQz0(double[] q, double[][] JQ)
   {
      JQ[3][i0] = (-1.133065e-01) * (1) + (6.937143e-03) * ((2) * q[i0]) + (6.714252e-06) * (q[i1]) + (-3.677595e-03) * (q[i2]) + (-6.042340e-02) * (q[i3])
            + (-5.138662e-02) * (q[i4]) + (-8.213849e-03) * (q[i5]) + (1.570326e-02) * (q[i6]) + (-1.074346e-02) * (q[i7]) + (8.653143e-03) * (q[i8])
            + (-5.261070e-03) * (q[i9]) + (-1.270144e-03) * (q[i10]) + (6.303760e-04) * (q[i11]) + (-3.051713e-04) * (q[i12]) + (-8.068139e-03) * (q[i15])
            + (-7.002062e-03) * (q[i16]) + (-3.021311e-03) * (q[i19]) + (-2.484627e-05) * (q[i20]) + (-6.986138e-03) * (q[i21]) + (1.111550e-03) * (q[i22])
            + (4.892620e-03) * ((3) * q[i0] * q[i0]) + (-1.963697e-03) * ((2) * q[i0] * q[i1]) + (-1.779915e-03) * ((2) * q[i0] * q[i2])
            + (-7.543770e-03) * ((2) * q[i0] * q[i3]) + (-2.024621e-03) * ((2) * q[i0] * q[i4]) + (1.598918e-04) * ((2) * q[i0] * q[i5])
            + (-1.235529e-02) * ((2) * q[i0] * q[i6]) + (-3.272551e-03) * ((2) * q[i0] * q[i7]) + (-9.077013e-04) * ((2) * q[i0] * q[i8])
            + (-2.269364e-03) * ((2) * q[i0] * q[i9]) + (-2.136750e-03) * ((2) * q[i0] * q[i10]) + (-6.309729e-05) * ((2) * q[i0] * q[i11])
            + (2.519247e-03) * ((2) * q[i0] * q[i12]) + (6.291848e-04) * ((2) * q[i0] * q[i15]) + (-1.373595e-06) * ((2) * q[i0] * q[i16])
            + (-6.879212e-05) * ((2) * q[i0] * q[i19]) + (3.310978e-04) * ((2) * q[i0] * q[i20]) + (-1.531392e-04) * ((2) * q[i0] * q[i21])
            + (4.324533e-04) * ((2) * q[i0] * q[i22]) + (-1.922747e-03) * (q[i1] * q[i1]) + (3.389921e-03) * (q[i2] * q[i2]) + (-6.693411e-03) * (q[i3] * q[i3])
            + (-2.229401e-03) * (q[i4] * q[i4]) + (4.455479e-05) * (q[i5] * q[i5]) + (-1.078988e-02) * (q[i6] * q[i6]) + (6.715776e-03) * (q[i7] * q[i7])
            + (8.763863e-04) * (q[i8] * q[i8]) + (3.571000e-03) * (q[i9] * q[i9]) + (2.137082e-04) * (q[i10] * q[i10]) + (-3.013394e-04) * (q[i11] * q[i11])
            + (-4.926891e-06) * (q[i12] * q[i12]) + (-1.610159e-03) * (q[i15] * q[i15]) + (-2.629202e-03) * (q[i16] * q[i16])
            + (9.614484e-04) * (q[i19] * q[i19]) + (-8.809152e-04) * (q[i20] * q[i20]) + (-1.759633e-03) * (q[i21] * q[i21])
            + (-4.010723e-04) * (q[i22] * q[i22]) + (-1.034438e-03) * (q[i1] * q[i2]) + (1.107523e-03) * (q[i1] * q[i3]) + (1.092826e-03) * (q[i1] * q[i4])
            + (5.936415e-03) * (q[i1] * q[i5]) + (-3.535111e-04) * (q[i1] * q[i6]) + (2.757222e-04) * (q[i1] * q[i7]) + (5.311938e-05) * (q[i1] * q[i8])
            + (-1.650176e-03) * (q[i1] * q[i9]) + (1.641943e-03) * (q[i1] * q[i10]) + (-6.978984e-04) * (q[i1] * q[i11]) + (-6.819984e-04) * (q[i1] * q[i12])
            + (8.307751e-05) * (q[i1] * q[i15]) + (-8.990329e-05) * (q[i1] * q[i16]) + (2.970379e-04) * (q[i1] * q[i19]) + (-3.106547e-04) * (q[i1] * q[i20])
            + (1.096464e-03) * (q[i1] * q[i21]) + (1.088216e-03) * (q[i1] * q[i22]) + (2.092116e-03) * (q[i2] * q[i3]) + (-5.079142e-03) * (q[i2] * q[i4])
            + (7.131349e-03) * (q[i2] * q[i5]) + (-8.584065e-03) * (q[i2] * q[i6]) + (-2.635861e-03) * (q[i2] * q[i7]) + (5.935676e-03) * (q[i2] * q[i8])
            + (-1.593483e-03) * (q[i2] * q[i9]) + (-1.246070e-03) * (q[i2] * q[i10]) + (5.106428e-04) * (q[i2] * q[i11]) + (-8.160733e-04) * (q[i2] * q[i12])
            + (-8.816348e-04) * (q[i2] * q[i15]) + (-1.612249e-03) * (q[i2] * q[i16]) + (-1.778900e-03) * (q[i2] * q[i19]) + (3.204858e-04) * (q[i2] * q[i20])
            + (-7.543829e-05) * (q[i2] * q[i21]) + (1.559269e-03) * (q[i2] * q[i22]) + (1.157804e-02) * (q[i3] * q[i4]) + (-1.175620e-02) * (q[i3] * q[i5])
            + (-1.011579e-02) * (q[i3] * q[i6]) + (3.106750e-03) * (q[i3] * q[i7]) + (5.752914e-03) * (q[i3] * q[i8]) + (6.880723e-03) * (q[i3] * q[i9])
            + (4.104927e-04) * (q[i3] * q[i10]) + (-1.905806e-03) * (q[i3] * q[i11]) + (-4.130384e-03) * (q[i3] * q[i12]) + (3.737213e-03) * (q[i3] * q[i15])
            + (-4.549942e-03) * (q[i3] * q[i16]) + (-7.239892e-04) * (q[i3] * q[i19]) + (-4.420090e-04) * (q[i3] * q[i20]) + (2.352586e-03) * (q[i3] * q[i21])
            + (-2.251364e-03) * (q[i3] * q[i22]) + (3.113038e-04) * (q[i4] * q[i5]) + (2.345123e-03) * (q[i4] * q[i6]) + (-3.091155e-03) * (q[i4] * q[i7])
            + (2.799478e-03) * (q[i4] * q[i8]) + (5.130394e-03) * (q[i4] * q[i9]) + (6.354973e-03) * (q[i4] * q[i10]) + (-4.735293e-04) * (q[i4] * q[i11])
            + (3.386984e-03) * (q[i4] * q[i12]) + (-3.231341e-03) * (q[i4] * q[i15]) + (-1.271616e-04) * (q[i4] * q[i16]) + (-1.119306e-05) * (q[i4] * q[i19])
            + (-5.171096e-04) * (q[i4] * q[i20]) + (-2.774221e-03) * (q[i4] * q[i21]) + (-7.930453e-04) * (q[i4] * q[i22]) + (2.328631e-03) * (q[i5] * q[i6])
            + (-2.363745e-04) * (q[i5] * q[i7]) + (3.065318e-03) * (q[i5] * q[i8]) + (2.392677e-03) * (q[i5] * q[i9]) + (1.078840e-03) * (q[i5] * q[i10])
            + (1.322023e-03) * (q[i5] * q[i11]) + (1.282937e-03) * (q[i5] * q[i12]) + (3.452045e-03) * (q[i5] * q[i15]) + (-2.146369e-03) * (q[i5] * q[i16])
            + (-1.380481e-03) * (q[i5] * q[i19]) + (-5.019998e-04) * (q[i5] * q[i20]) + (-2.871135e-03) * (q[i5] * q[i21]) + (-8.908884e-04) * (q[i5] * q[i22])
            + (7.590054e-03) * (q[i6] * q[i7]) + (3.540452e-03) * (q[i6] * q[i8]) + (-6.566775e-03) * (q[i6] * q[i9]) + (6.778875e-03) * (q[i6] * q[i10])
            + (2.362114e-03) * (q[i6] * q[i11]) + (1.859035e-03) * (q[i6] * q[i12]) + (-3.302023e-04) * (q[i6] * q[i15]) + (2.531603e-03) * (q[i6] * q[i16])
            + (-1.968210e-03) * (q[i6] * q[i19]) + (-7.278929e-04) * (q[i6] * q[i20]) + (-4.359618e-04) * (q[i6] * q[i21]) + (1.950349e-03) * (q[i6] * q[i22])
            + (-1.160031e-03) * (q[i7] * q[i8]) + (2.473974e-04) * (q[i7] * q[i9]) + (5.886241e-03) * (q[i7] * q[i10]) + (-8.944924e-04) * (q[i7] * q[i11])
            + (-6.621534e-04) * (q[i7] * q[i12]) + (1.730123e-03) * (q[i7] * q[i15]) + (-2.987341e-03) * (q[i7] * q[i16]) + (-2.717691e-03) * (q[i7] * q[i19])
            + (-1.157077e-04) * (q[i7] * q[i20]) + (4.351924e-04) * (q[i7] * q[i21]) + (-6.437164e-04) * (q[i7] * q[i22]) + (3.435307e-04) * (q[i8] * q[i9])
            + (-1.449731e-03) * (q[i8] * q[i10]) + (-1.501626e-04) * (q[i8] * q[i11]) + (3.196186e-04) * (q[i8] * q[i12]) + (-1.777304e-03) * (q[i8] * q[i15])
            + (-1.266048e-03) * (q[i8] * q[i16]) + (-9.278645e-04) * (q[i8] * q[i19]) + (2.572781e-03) * (q[i8] * q[i20]) + (5.334901e-04) * (q[i8] * q[i21])
            + (-1.699471e-03) * (q[i8] * q[i22]) + (1.333336e-03) * (q[i9] * q[i10]) + (9.486425e-06) * (q[i9] * q[i11]) + (-4.151017e-04) * (q[i9] * q[i12])
            + (1.080065e-04) * (q[i9] * q[i15]) + (-8.717527e-04) * (q[i9] * q[i16]) + (1.619438e-03) * (q[i9] * q[i19]) + (-3.476660e-04) * (q[i9] * q[i20])
            + (1.984431e-04) * (q[i9] * q[i21]) + (4.594173e-05) * (q[i9] * q[i22]) + (-1.415883e-03) * (q[i10] * q[i11]) + (1.398713e-03) * (q[i10] * q[i12])
            + (-1.365183e-03) * (q[i10] * q[i15]) + (-2.779088e-04) * (q[i10] * q[i16]) + (1.148780e-03) * (q[i10] * q[i19])
            + (-6.231768e-04) * (q[i10] * q[i20]) + (6.428626e-04) * (q[i10] * q[i21]) + (-1.778341e-04) * (q[i10] * q[i22])
            + (5.165199e-04) * (q[i11] * q[i12]) + (-1.382556e-03) * (q[i11] * q[i15]) + (-2.685371e-04) * (q[i11] * q[i16])
            + (1.617007e-03) * (q[i11] * q[i19]) + (-1.578691e-03) * (q[i11] * q[i20]) + (-8.476260e-04) * (q[i11] * q[i21])
            + (-1.043071e-03) * (q[i11] * q[i22]) + (1.106490e-03) * (q[i12] * q[i15]) + (-3.871843e-03) * (q[i12] * q[i16])
            + (6.751019e-04) * (q[i12] * q[i19]) + (2.247609e-03) * (q[i12] * q[i20]) + (1.695129e-03) * (q[i12] * q[i21]) + (6.040034e-04) * (q[i12] * q[i22])
            + (8.868801e-05) * (q[i15] * q[i16]) + (1.068404e-03) * (q[i15] * q[i19]) + (-6.770629e-05) * (q[i15] * q[i20])
            + (-2.242571e-03) * (q[i15] * q[i21]) + (2.860531e-04) * (q[i15] * q[i22]) + (1.225508e-03) * (q[i16] * q[i19])
            + (-9.726985e-04) * (q[i16] * q[i20]) + (8.158969e-05) * (q[i16] * q[i21]) + (2.233752e-04) * (q[i16] * q[i22])
            + (-1.029540e-04) * (q[i19] * q[i20]) + (7.949083e-04) * (q[i19] * q[i21]) + (-6.577718e-04) * (q[i19] * q[i22])
            + (-8.140101e-04) * (q[i20] * q[i21]) + (1.086136e-04) * (q[i20] * q[i22]) + (4.544889e-04) * (q[i21] * q[i22]);
   }

   public void getJQz1(double[] q, double[][] JQ)
   {
      JQ[3][i1] = (-1.127153e-01) * (1) + (-6.826124e-03) * ((2) * q[i1]) + (6.714252e-06) * (q[i0]) + (3.762040e-03) * (q[i2]) + (5.143739e-02) * (q[i3])
            + (6.048093e-02) * (q[i4]) + (8.116596e-03) * (q[i5]) + (-1.079350e-02) * (q[i6]) + (1.575324e-02) * (q[i7]) + (8.756454e-03) * (q[i8])
            + (-1.332474e-03) * (q[i9]) + (-5.206619e-03) * (q[i10]) + (4.869941e-04) * (q[i11]) + (-5.109633e-04) * (q[i12]) + (-6.918457e-03) * (q[i15])
            + (-8.223930e-03) * (q[i16]) + (-1.285072e-05) * (q[i19]) + (-3.012531e-03) * (q[i20]) + (-1.117623e-03) * (q[i21]) + (6.876052e-03) * (q[i22])
            + (-1.963697e-03) * (q[i0] * q[i0]) + (-1.922747e-03) * ((2) * q[i0] * q[i1]) + (4.864441e-03) * ((3) * q[i1] * q[i1])
            + (-1.710495e-03) * ((2) * q[i1] * q[i2]) + (-2.026980e-03) * ((2) * q[i1] * q[i3]) + (-7.512135e-03) * ((2) * q[i1] * q[i4])
            + (1.282382e-04) * ((2) * q[i1] * q[i5]) + (3.284873e-03) * ((2) * q[i1] * q[i6]) + (1.242606e-02) * ((2) * q[i1] * q[i7])
            + (8.847795e-04) * ((2) * q[i1] * q[i8]) + (2.143130e-03) * ((2) * q[i1] * q[i9]) + (2.239776e-03) * ((2) * q[i1] * q[i10])
            + (2.519575e-03) * ((2) * q[i1] * q[i11]) + (-6.138320e-05) * ((2) * q[i1] * q[i12]) + (3.195453e-05) * ((2) * q[i1] * q[i15])
            + (-6.272335e-04) * ((2) * q[i1] * q[i16]) + (-3.195466e-04) * ((2) * q[i1] * q[i19]) + (1.054223e-04) * ((2) * q[i1] * q[i20])
            + (4.403735e-04) * ((2) * q[i1] * q[i21]) + (-1.664605e-04) * ((2) * q[i1] * q[i22]) + (3.420126e-03) * (q[i2] * q[i2])
            + (-2.267421e-03) * (q[i3] * q[i3]) + (-6.662712e-03) * (q[i4] * q[i4]) + (-6.971971e-05) * (q[i5] * q[i5]) + (6.680487e-03) * (q[i6] * q[i6])
            + (-1.079080e-02) * (q[i7] * q[i7]) + (8.621596e-04) * (q[i8] * q[i8]) + (2.252412e-04) * (q[i9] * q[i9]) + (3.526691e-03) * (q[i10] * q[i10])
            + (-1.769956e-05) * (q[i11] * q[i11]) + (-3.530106e-04) * (q[i12] * q[i12]) + (-2.590570e-03) * (q[i15] * q[i15])
            + (-1.635661e-03) * (q[i16] * q[i16]) + (-8.908750e-04) * (q[i19] * q[i19]) + (9.579766e-04) * (q[i20] * q[i20])
            + (-3.901743e-04) * (q[i21] * q[i21]) + (-1.742359e-03) * (q[i22] * q[i22]) + (-1.034438e-03) * (q[i0] * q[i2]) + (1.107523e-03) * (q[i0] * q[i3])
            + (1.092826e-03) * (q[i0] * q[i4]) + (5.936415e-03) * (q[i0] * q[i5]) + (-3.535111e-04) * (q[i0] * q[i6]) + (2.757222e-04) * (q[i0] * q[i7])
            + (5.311938e-05) * (q[i0] * q[i8]) + (-1.650176e-03) * (q[i0] * q[i9]) + (1.641943e-03) * (q[i0] * q[i10]) + (-6.978984e-04) * (q[i0] * q[i11])
            + (-6.819984e-04) * (q[i0] * q[i12]) + (8.307751e-05) * (q[i0] * q[i15]) + (-8.990329e-05) * (q[i0] * q[i16]) + (2.970379e-04) * (q[i0] * q[i19])
            + (-3.106547e-04) * (q[i0] * q[i20]) + (1.096464e-03) * (q[i0] * q[i21]) + (1.088216e-03) * (q[i0] * q[i22]) + (-5.128699e-03) * (q[i2] * q[i3])
            + (2.120396e-03) * (q[i2] * q[i4]) + (7.188995e-03) * (q[i2] * q[i5]) + (2.627176e-03) * (q[i2] * q[i6]) + (8.588029e-03) * (q[i2] * q[i7])
            + (-5.944304e-03) * (q[i2] * q[i8]) + (1.251727e-03) * (q[i2] * q[i9]) + (1.589570e-03) * (q[i2] * q[i10]) + (-8.452529e-04) * (q[i2] * q[i11])
            + (5.137758e-04) * (q[i2] * q[i12]) + (1.617850e-03) * (q[i2] * q[i15]) + (8.778651e-04) * (q[i2] * q[i16]) + (-3.047581e-04) * (q[i2] * q[i19])
            + (1.812038e-03) * (q[i2] * q[i20]) + (1.553031e-03) * (q[i2] * q[i21]) + (-1.092656e-04) * (q[i2] * q[i22]) + (1.164925e-02) * (q[i3] * q[i4])
            + (3.613486e-04) * (q[i3] * q[i5]) + (3.009775e-03) * (q[i3] * q[i6]) + (-2.379827e-03) * (q[i3] * q[i7]) + (-2.802561e-03) * (q[i3] * q[i8])
            + (-6.364877e-03) * (q[i3] * q[i9]) + (-5.111334e-03) * (q[i3] * q[i10]) + (3.375875e-03) * (q[i3] * q[i11]) + (-5.158804e-04) * (q[i3] * q[i12])
            + (1.375503e-04) * (q[i3] * q[i15]) + (3.280968e-03) * (q[i3] * q[i16]) + (5.168160e-04) * (q[i3] * q[i19]) + (-2.998582e-05) * (q[i3] * q[i20])
            + (-7.713824e-04) * (q[i3] * q[i21]) + (-2.770689e-03) * (q[i3] * q[i22]) + (-1.174509e-02) * (q[i4] * q[i5]) + (-3.075405e-03) * (q[i4] * q[i6])
            + (1.026329e-02) * (q[i4] * q[i7]) + (-5.652658e-03) * (q[i4] * q[i8]) + (-4.481237e-04) * (q[i4] * q[i9]) + (-6.823310e-03) * (q[i4] * q[i10])
            + (-4.153900e-03) * (q[i4] * q[i11]) + (-1.935043e-03) * (q[i4] * q[i12]) + (4.452672e-03) * (q[i4] * q[i15]) + (-3.699263e-03) * (q[i4] * q[i16])
            + (4.557667e-04) * (q[i4] * q[i19]) + (6.645825e-04) * (q[i4] * q[i20]) + (-2.249974e-03) * (q[i4] * q[i21]) + (2.335218e-03) * (q[i4] * q[i22])
            + (2.449185e-04) * (q[i5] * q[i6]) + (-2.324987e-03) * (q[i5] * q[i7]) + (-3.085534e-03) * (q[i5] * q[i8]) + (-1.058509e-03) * (q[i5] * q[i9])
            + (-2.381350e-03) * (q[i5] * q[i10]) + (1.337867e-03) * (q[i5] * q[i11]) + (1.235463e-03) * (q[i5] * q[i12]) + (2.119109e-03) * (q[i5] * q[i15])
            + (-3.452865e-03) * (q[i5] * q[i16]) + (5.274652e-04) * (q[i5] * q[i19]) + (1.376363e-03) * (q[i5] * q[i20]) + (-8.784839e-04) * (q[i5] * q[i21])
            + (-2.824797e-03) * (q[i5] * q[i22]) + (7.598896e-03) * (q[i6] * q[i7]) + (-1.137300e-03) * (q[i6] * q[i8]) + (5.926933e-03) * (q[i6] * q[i9])
            + (2.451999e-04) * (q[i6] * q[i10]) + (7.141237e-04) * (q[i6] * q[i11]) + (8.925012e-04) * (q[i6] * q[i12]) + (-2.981604e-03) * (q[i6] * q[i15])
            + (1.727913e-03) * (q[i6] * q[i16]) + (-1.552690e-04) * (q[i6] * q[i19]) + (-2.713295e-03) * (q[i6] * q[i20]) + (6.585911e-04) * (q[i6] * q[i21])
            + (-4.110103e-04) * (q[i6] * q[i22]) + (3.540632e-03) * (q[i7] * q[i8]) + (6.815199e-03) * (q[i7] * q[i9]) + (-6.555638e-03) * (q[i7] * q[i10])
            + (-1.815133e-03) * (q[i7] * q[i11]) + (-2.364231e-03) * (q[i7] * q[i12]) + (2.516036e-03) * (q[i7] * q[i15]) + (-2.919010e-04) * (q[i7] * q[i16])
            + (-7.084220e-04) * (q[i7] * q[i19]) + (-1.945346e-03) * (q[i7] * q[i20]) + (-1.962824e-03) * (q[i7] * q[i21]) + (4.281154e-04) * (q[i7] * q[i22])
            + (-1.458352e-03) * (q[i8] * q[i9]) + (3.259798e-04) * (q[i8] * q[i10]) + (-3.038879e-04) * (q[i8] * q[i11]) + (1.257303e-04) * (q[i8] * q[i12])
            + (-1.220725e-03) * (q[i8] * q[i15]) + (-1.803216e-03) * (q[i8] * q[i16]) + (2.591051e-03) * (q[i8] * q[i19]) + (-9.210163e-04) * (q[i8] * q[i20])
            + (1.685769e-03) * (q[i8] * q[i21]) + (-5.526813e-04) * (q[i8] * q[i22]) + (1.366859e-03) * (q[i9] * q[i10]) + (-1.395787e-03) * (q[i9] * q[i11])
            + (1.418468e-03) * (q[i9] * q[i12]) + (-2.835336e-04) * (q[i9] * q[i15]) + (-1.345912e-03) * (q[i9] * q[i16]) + (-6.331721e-04) * (q[i9] * q[i19])
            + (1.136149e-03) * (q[i9] * q[i20]) + (1.872351e-04) * (q[i9] * q[i21]) + (-6.430545e-04) * (q[i9] * q[i22]) + (4.045033e-04) * (q[i10] * q[i11])
            + (-1.472401e-05) * (q[i10] * q[i12]) + (-8.608620e-04) * (q[i10] * q[i15]) + (1.288937e-04) * (q[i10] * q[i16])
            + (-3.646995e-04) * (q[i10] * q[i19]) + (1.601052e-03) * (q[i10] * q[i20]) + (-4.788341e-05) * (q[i10] * q[i21])
            + (-1.747739e-04) * (q[i10] * q[i22]) + (5.198115e-04) * (q[i11] * q[i12]) + (3.872995e-03) * (q[i11] * q[i15])
            + (-1.095867e-03) * (q[i11] * q[i16]) + (-2.231571e-03) * (q[i11] * q[i19]) + (-6.580470e-04) * (q[i11] * q[i20])
            + (6.031111e-04) * (q[i11] * q[i21]) + (1.702132e-03) * (q[i11] * q[i22]) + (2.860590e-04) * (q[i12] * q[i15]) + (1.377429e-03) * (q[i12] * q[i16])
            + (1.575255e-03) * (q[i12] * q[i19]) + (-1.639693e-03) * (q[i12] * q[i20]) + (-1.023231e-03) * (q[i12] * q[i21])
            + (-8.373231e-04) * (q[i12] * q[i22]) + (8.015678e-05) * (q[i15] * q[i16]) + (-9.882359e-04) * (q[i15] * q[i19])
            + (1.207770e-03) * (q[i15] * q[i20]) + (-2.136392e-04) * (q[i15] * q[i21]) + (-6.772398e-05) * (q[i15] * q[i22])
            + (-6.929032e-05) * (q[i16] * q[i19]) + (1.071461e-03) * (q[i16] * q[i20]) + (-3.002668e-04) * (q[i16] * q[i21])
            + (2.228063e-03) * (q[i16] * q[i22]) + (-1.173045e-04) * (q[i19] * q[i20]) + (-1.194085e-04) * (q[i19] * q[i21])
            + (8.183928e-04) * (q[i19] * q[i22]) + (6.573443e-04) * (q[i20] * q[i21]) + (-7.683510e-04) * (q[i20] * q[i22])
            + (4.280431e-04) * (q[i21] * q[i22]);
   }

   public void getJQz2(double[] q, double[][] JQ)
   {
      JQ[3][i2] = (2.139773e-01) * (1) + (2.270333e-06) * ((2) * q[i2]) + (-3.677595e-03) * (q[i0]) + (3.762040e-03) * (q[i1]) + (-1.297012e-02) * (q[i3])
            + (1.287635e-02) * (q[i4]) + (-2.055079e-05) * (q[i5]) + (1.407401e-02) * (q[i6]) + (1.409484e-02) * (q[i7]) + (-1.008699e-03) * (q[i8])
            + (-1.280027e-03) * (q[i9]) + (-1.263228e-03) * (q[i10]) + (2.007064e-03) * (q[i11]) + (-1.752413e-03) * (q[i12]) + (-1.796744e-02) * (q[i15])
            + (-1.816952e-02) * (q[i16]) + (2.002263e-03) * (q[i19]) + (1.964953e-03) * (q[i20]) + (-3.676468e-03) * (q[i21]) + (3.596080e-03) * (q[i22])
            + (-1.779915e-03) * (q[i0] * q[i0]) + (-1.710495e-03) * (q[i1] * q[i1]) + (3.389921e-03) * ((2) * q[i0] * q[i2])
            + (3.420126e-03) * ((2) * q[i1] * q[i2]) + (-2.802144e-03) * ((3) * q[i2] * q[i2]) + (-3.984365e-03) * ((2) * q[i2] * q[i3])
            + (-3.960318e-03) * ((2) * q[i2] * q[i4]) + (5.685596e-03) * ((2) * q[i2] * q[i5]) + (-2.160606e-04) * ((2) * q[i2] * q[i6])
            + (2.160540e-04) * ((2) * q[i2] * q[i7]) + (6.674590e-06) * ((2) * q[i2] * q[i8]) + (4.256555e-04) * ((2) * q[i2] * q[i9])
            + (-4.375921e-04) * ((2) * q[i2] * q[i10]) + (-1.959227e-03) * ((2) * q[i2] * q[i11]) + (-1.950016e-03) * ((2) * q[i2] * q[i12])
            + (-1.460510e-03) * ((2) * q[i2] * q[i15]) + (1.450669e-03) * ((2) * q[i2] * q[i16]) + (-1.100006e-03) * ((2) * q[i2] * q[i19])
            + (1.085563e-03) * ((2) * q[i2] * q[i20]) + (5.330095e-04) * ((2) * q[i2] * q[i21]) + (5.261333e-04) * ((2) * q[i2] * q[i22])
            + (-6.304685e-03) * (q[i3] * q[i3]) + (-6.295773e-03) * (q[i4] * q[i4]) + (-1.450116e-02) * (q[i5] * q[i5]) + (-4.822411e-03) * (q[i6] * q[i6])
            + (-4.756895e-03) * (q[i7] * q[i7]) + (6.311364e-03) * (q[i8] * q[i8]) + (9.906023e-04) * (q[i9] * q[i9]) + (9.744991e-04) * (q[i10] * q[i10])
            + (-2.203007e-03) * (q[i11] * q[i11]) + (-2.230185e-03) * (q[i12] * q[i12]) + (-4.875913e-03) * (q[i15] * q[i15])
            + (-4.926560e-03) * (q[i16] * q[i16]) + (-3.407559e-04) * (q[i19] * q[i19]) + (-3.275157e-04) * (q[i20] * q[i20])
            + (-2.045029e-03) * (q[i21] * q[i21]) + (-2.031400e-03) * (q[i22] * q[i22]) + (-1.034438e-03) * (q[i0] * q[i1]) + (2.092116e-03) * (q[i0] * q[i3])
            + (-5.079142e-03) * (q[i0] * q[i4]) + (7.131349e-03) * (q[i0] * q[i5]) + (-8.584065e-03) * (q[i0] * q[i6]) + (-2.635861e-03) * (q[i0] * q[i7])
            + (5.935676e-03) * (q[i0] * q[i8]) + (-1.593483e-03) * (q[i0] * q[i9]) + (-1.246070e-03) * (q[i0] * q[i10]) + (5.106428e-04) * (q[i0] * q[i11])
            + (-8.160733e-04) * (q[i0] * q[i12]) + (-8.816348e-04) * (q[i0] * q[i15]) + (-1.612249e-03) * (q[i0] * q[i16]) + (-1.778900e-03) * (q[i0] * q[i19])
            + (3.204858e-04) * (q[i0] * q[i20]) + (-7.543829e-05) * (q[i0] * q[i21]) + (1.559269e-03) * (q[i0] * q[i22]) + (-5.128699e-03) * (q[i1] * q[i3])
            + (2.120396e-03) * (q[i1] * q[i4]) + (7.188995e-03) * (q[i1] * q[i5]) + (2.627176e-03) * (q[i1] * q[i6]) + (8.588029e-03) * (q[i1] * q[i7])
            + (-5.944304e-03) * (q[i1] * q[i8]) + (1.251727e-03) * (q[i1] * q[i9]) + (1.589570e-03) * (q[i1] * q[i10]) + (-8.452529e-04) * (q[i1] * q[i11])
            + (5.137758e-04) * (q[i1] * q[i12]) + (1.617850e-03) * (q[i1] * q[i15]) + (8.778651e-04) * (q[i1] * q[i16]) + (-3.047581e-04) * (q[i1] * q[i19])
            + (1.812038e-03) * (q[i1] * q[i20]) + (1.553031e-03) * (q[i1] * q[i21]) + (-1.092656e-04) * (q[i1] * q[i22]) + (8.438957e-04) * (q[i3] * q[i4])
            + (-1.295654e-02) * (q[i3] * q[i5]) + (-3.436195e-03) * (q[i3] * q[i6]) + (-5.176599e-05) * (q[i3] * q[i7]) + (3.351840e-03) * (q[i3] * q[i8])
            + (1.625624e-03) * (q[i3] * q[i9]) + (-2.978439e-03) * (q[i3] * q[i10]) + (1.251982e-03) * (q[i3] * q[i11]) + (-3.162736e-03) * (q[i3] * q[i12])
            + (7.214720e-06) * (q[i3] * q[i15]) + (6.397862e-04) * (q[i3] * q[i16]) + (-9.618666e-04) * (q[i3] * q[i19]) + (-2.681924e-03) * (q[i3] * q[i20])
            + (1.918705e-03) * (q[i3] * q[i21]) + (-3.860117e-03) * (q[i3] * q[i22]) + (-1.289630e-02) * (q[i4] * q[i5]) + (8.420013e-06) * (q[i4] * q[i6])
            + (3.457939e-03) * (q[i4] * q[i7]) + (-3.298082e-03) * (q[i4] * q[i8]) + (2.980178e-03) * (q[i4] * q[i9]) + (-1.570757e-03) * (q[i4] * q[i10])
            + (-3.083760e-03) * (q[i4] * q[i11]) + (1.235821e-03) * (q[i4] * q[i12]) + (-6.401135e-04) * (q[i4] * q[i15]) + (-3.348564e-05) * (q[i4] * q[i16])
            + (2.656238e-03) * (q[i4] * q[i19]) + (9.512730e-04) * (q[i4] * q[i20]) + (-3.841436e-03) * (q[i4] * q[i21]) + (1.912853e-03) * (q[i4] * q[i22])
            + (-8.390429e-03) * (q[i5] * q[i6]) + (8.471359e-03) * (q[i5] * q[i7]) + (-1.490702e-05) * (q[i5] * q[i8]) + (2.289242e-03) * (q[i5] * q[i9])
            + (-2.286014e-03) * (q[i5] * q[i10]) + (-1.072867e-03) * (q[i5] * q[i11]) + (-1.137662e-03) * (q[i5] * q[i12]) + (7.764108e-03) * (q[i5] * q[i15])
            + (-7.806409e-03) * (q[i5] * q[i16]) + (-3.188673e-03) * (q[i5] * q[i19]) + (3.189714e-03) * (q[i5] * q[i20]) + (-2.336499e-03) * (q[i5] * q[i21])
            + (-2.288843e-03) * (q[i5] * q[i22]) + (1.407930e-02) * (q[i6] * q[i7]) + (-9.756797e-03) * (q[i6] * q[i8]) + (-4.292913e-03) * (q[i6] * q[i9])
            + (1.744585e-03) * (q[i6] * q[i10]) + (2.110339e-03) * (q[i6] * q[i11]) + (3.404679e-03) * (q[i6] * q[i12]) + (-2.437519e-03) * (q[i6] * q[i15])
            + (1.432669e-03) * (q[i6] * q[i16]) + (-9.689233e-04) * (q[i6] * q[i19]) + (1.426482e-03) * (q[i6] * q[i20]) + (-3.636610e-04) * (q[i6] * q[i21])
            + (4.758287e-04) * (q[i6] * q[i22]) + (-9.730376e-03) * (q[i7] * q[i8]) + (1.755898e-03) * (q[i7] * q[i9]) + (-4.274821e-03) * (q[i7] * q[i10])
            + (-3.373453e-03) * (q[i7] * q[i11]) + (-2.075903e-03) * (q[i7] * q[i12]) + (1.418427e-03) * (q[i7] * q[i15]) + (-2.459923e-03) * (q[i7] * q[i16])
            + (1.409915e-03) * (q[i7] * q[i19]) + (-9.379687e-04) * (q[i7] * q[i20]) + (-4.657147e-04) * (q[i7] * q[i21]) + (3.561437e-04) * (q[i7] * q[i22])
            + (3.126228e-04) * (q[i8] * q[i9]) + (3.500331e-04) * (q[i8] * q[i10]) + (6.417629e-05) * (q[i8] * q[i11]) + (-1.014369e-04) * (q[i8] * q[i12])
            + (-6.401808e-04) * (q[i8] * q[i15]) + (-6.618732e-04) * (q[i8] * q[i16]) + (-2.143070e-03) * (q[i8] * q[i19]) + (-2.164839e-03) * (q[i8] * q[i20])
            + (7.322682e-04) * (q[i8] * q[i21]) + (-7.450876e-04) * (q[i8] * q[i22]) + (1.046591e-03) * (q[i9] * q[i10]) + (-1.559192e-03) * (q[i9] * q[i11])
            + (7.825747e-04) * (q[i9] * q[i12]) + (1.132058e-03) * (q[i9] * q[i15]) + (-1.885767e-03) * (q[i9] * q[i16]) + (-1.062082e-04) * (q[i9] * q[i19])
            + (3.898031e-04) * (q[i9] * q[i20]) + (9.727644e-05) * (q[i9] * q[i21]) + (-3.653917e-04) * (q[i9] * q[i22]) + (-7.718939e-04) * (q[i10] * q[i11])
            + (1.535351e-03) * (q[i10] * q[i12]) + (-1.879207e-03) * (q[i10] * q[i15]) + (1.131607e-03) * (q[i10] * q[i16]) + (3.717942e-04) * (q[i10] * q[i19])
            + (-1.153407e-04) * (q[i10] * q[i20]) + (3.553253e-04) * (q[i10] * q[i21]) + (-8.714664e-05) * (q[i10] * q[i22])
            + (2.987426e-03) * (q[i11] * q[i12]) + (2.351462e-03) * (q[i11] * q[i15]) + (-1.103160e-03) * (q[i11] * q[i16])
            + (-7.304907e-04) * (q[i11] * q[i19]) + (6.020709e-04) * (q[i11] * q[i20]) + (-8.595518e-04) * (q[i11] * q[i21])
            + (-6.161720e-04) * (q[i11] * q[i22]) + (1.115856e-03) * (q[i12] * q[i15]) + (-2.369277e-03) * (q[i12] * q[i16])
            + (-6.144859e-04) * (q[i12] * q[i19]) + (7.187429e-04) * (q[i12] * q[i20]) + (-6.115261e-04) * (q[i12] * q[i21])
            + (-8.317397e-04) * (q[i12] * q[i22]) + (5.453522e-04) * (q[i15] * q[i16]) + (5.335550e-04) * (q[i15] * q[i19]) + (9.489542e-04) * (q[i15] * q[i20])
            + (-1.693832e-03) * (q[i15] * q[i21]) + (-3.798725e-04) * (q[i15] * q[i22]) + (9.717815e-04) * (q[i16] * q[i19])
            + (5.450892e-04) * (q[i16] * q[i20]) + (3.833704e-04) * (q[i16] * q[i21]) + (1.684649e-03) * (q[i16] * q[i22]) + (-8.298723e-04) * (q[i19] * q[i20])
            + (-6.527569e-04) * (q[i19] * q[i21]) + (3.099660e-04) * (q[i19] * q[i22]) + (-3.104171e-04) * (q[i20] * q[i21])
            + (6.695144e-04) * (q[i20] * q[i22]) + (1.096783e-03) * (q[i21] * q[i22]);
   }

   public void getJQz3(double[] q, double[][] JQ)
   {
      JQ[3][i3] = (2.223118e-02) * (1) + (-6.406715e-03) * ((2) * q[i3]) + (-6.042340e-02) * (q[i0]) + (5.143739e-02) * (q[i1]) + (-1.297012e-02) * (q[i2])
            + (-6.414578e-05) * (q[i4]) + (3.490649e-03) * (q[i5]) + (1.711444e-02) * (q[i6]) + (1.131108e-02) * (q[i7]) + (-9.928663e-04) * (q[i8])
            + (5.648699e-03) * (q[i9]) + (1.150907e-02) * (q[i10]) + (-1.591195e-03) * (q[i11]) + (-2.040305e-03) * (q[i12]) + (7.821852e-03) * (q[i15])
            + (-2.114044e-04) * (q[i16]) + (-1.242713e-02) * (q[i19]) + (1.146731e-02) * (q[i20]) + (1.072986e-02) * (q[i21]) + (3.548932e-03) * (q[i22])
            + (-7.543770e-03) * (q[i0] * q[i0]) + (-2.026980e-03) * (q[i1] * q[i1]) + (-3.984365e-03) * (q[i2] * q[i2])
            + (-6.693411e-03) * ((2) * q[i0] * q[i3]) + (-2.267421e-03) * ((2) * q[i1] * q[i3]) + (-6.304685e-03) * ((2) * q[i2] * q[i3])
            + (3.357610e-03) * ((3) * q[i3] * q[i3]) + (1.782037e-03) * ((2) * q[i3] * q[i4]) + (-4.691578e-03) * ((2) * q[i3] * q[i5])
            + (5.830212e-03) * ((2) * q[i3] * q[i6]) + (9.049718e-03) * ((2) * q[i3] * q[i7]) + (4.594219e-03) * ((2) * q[i3] * q[i8])
            + (8.430913e-04) * ((2) * q[i3] * q[i9]) + (-2.710877e-03) * ((2) * q[i3] * q[i10]) + (3.105474e-03) * ((2) * q[i3] * q[i11])
            + (-3.467011e-04) * ((2) * q[i3] * q[i12]) + (-6.010023e-03) * ((2) * q[i3] * q[i15]) + (-2.747068e-03) * ((2) * q[i3] * q[i16])
            + (8.414819e-04) * ((2) * q[i3] * q[i19]) + (-2.612225e-04) * ((2) * q[i3] * q[i20]) + (-1.876124e-03) * ((2) * q[i3] * q[i21])
            + (1.019498e-03) * ((2) * q[i3] * q[i22]) + (1.758108e-03) * (q[i4] * q[i4]) + (3.354428e-03) * (q[i5] * q[i5]) + (-5.211335e-03) * (q[i6] * q[i6])
            + (-3.817495e-03) * (q[i7] * q[i7]) + (-4.006531e-03) * (q[i8] * q[i8]) + (-1.039539e-03) * (q[i9] * q[i9]) + (-1.109231e-03) * (q[i10] * q[i10])
            + (-3.986162e-04) * (q[i11] * q[i11]) + (-2.886119e-03) * (q[i12] * q[i12]) + (-1.798421e-03) * (q[i15] * q[i15])
            + (1.128739e-03) * (q[i16] * q[i16]) + (-1.618046e-03) * (q[i19] * q[i19]) + (9.940037e-04) * (q[i20] * q[i20])
            + (-7.426308e-04) * (q[i21] * q[i21]) + (1.324916e-03) * (q[i22] * q[i22]) + (1.107523e-03) * (q[i0] * q[i1]) + (2.092116e-03) * (q[i0] * q[i2])
            + (1.157804e-02) * (q[i0] * q[i4]) + (-1.175620e-02) * (q[i0] * q[i5]) + (-1.011579e-02) * (q[i0] * q[i6]) + (3.106750e-03) * (q[i0] * q[i7])
            + (5.752914e-03) * (q[i0] * q[i8]) + (6.880723e-03) * (q[i0] * q[i9]) + (4.104927e-04) * (q[i0] * q[i10]) + (-1.905806e-03) * (q[i0] * q[i11])
            + (-4.130384e-03) * (q[i0] * q[i12]) + (3.737213e-03) * (q[i0] * q[i15]) + (-4.549942e-03) * (q[i0] * q[i16]) + (-7.239892e-04) * (q[i0] * q[i19])
            + (-4.420090e-04) * (q[i0] * q[i20]) + (2.352586e-03) * (q[i0] * q[i21]) + (-2.251364e-03) * (q[i0] * q[i22]) + (-5.128699e-03) * (q[i1] * q[i2])
            + (1.164925e-02) * (q[i1] * q[i4]) + (3.613486e-04) * (q[i1] * q[i5]) + (3.009775e-03) * (q[i1] * q[i6]) + (-2.379827e-03) * (q[i1] * q[i7])
            + (-2.802561e-03) * (q[i1] * q[i8]) + (-6.364877e-03) * (q[i1] * q[i9]) + (-5.111334e-03) * (q[i1] * q[i10]) + (3.375875e-03) * (q[i1] * q[i11])
            + (-5.158804e-04) * (q[i1] * q[i12]) + (1.375503e-04) * (q[i1] * q[i15]) + (3.280968e-03) * (q[i1] * q[i16]) + (5.168160e-04) * (q[i1] * q[i19])
            + (-2.998582e-05) * (q[i1] * q[i20]) + (-7.713824e-04) * (q[i1] * q[i21]) + (-2.770689e-03) * (q[i1] * q[i22]) + (8.438957e-04) * (q[i2] * q[i4])
            + (-1.295654e-02) * (q[i2] * q[i5]) + (-3.436195e-03) * (q[i2] * q[i6]) + (-5.176599e-05) * (q[i2] * q[i7]) + (3.351840e-03) * (q[i2] * q[i8])
            + (1.625624e-03) * (q[i2] * q[i9]) + (-2.978439e-03) * (q[i2] * q[i10]) + (1.251982e-03) * (q[i2] * q[i11]) + (-3.162736e-03) * (q[i2] * q[i12])
            + (7.214720e-06) * (q[i2] * q[i15]) + (6.397862e-04) * (q[i2] * q[i16]) + (-9.618666e-04) * (q[i2] * q[i19]) + (-2.681924e-03) * (q[i2] * q[i20])
            + (1.918705e-03) * (q[i2] * q[i21]) + (-3.860117e-03) * (q[i2] * q[i22]) + (8.470272e-03) * (q[i4] * q[i5]) + (8.451358e-03) * (q[i4] * q[i6])
            + (-8.385159e-03) * (q[i4] * q[i7]) + (-5.215263e-05) * (q[i4] * q[i8]) + (-2.546138e-03) * (q[i4] * q[i9]) + (2.573182e-03) * (q[i4] * q[i10])
            + (1.651983e-04) * (q[i4] * q[i11]) + (2.118888e-04) * (q[i4] * q[i12]) + (1.928381e-03) * (q[i4] * q[i15]) + (-1.965659e-03) * (q[i4] * q[i16])
            + (-1.061026e-03) * (q[i4] * q[i19]) + (1.098279e-03) * (q[i4] * q[i20]) + (-1.279848e-03) * (q[i4] * q[i21]) + (-1.279522e-03) * (q[i4] * q[i22])
            + (-6.583003e-03) * (q[i5] * q[i6]) + (-6.469581e-03) * (q[i5] * q[i7]) + (-2.029328e-02) * (q[i5] * q[i8]) + (1.136585e-03) * (q[i5] * q[i9])
            + (-4.226047e-05) * (q[i5] * q[i10]) + (1.932446e-04) * (q[i5] * q[i11]) + (-4.053934e-03) * (q[i5] * q[i12]) + (-8.369577e-04) * (q[i5] * q[i15])
            + (1.036362e-03) * (q[i5] * q[i16]) + (-3.405769e-03) * (q[i5] * q[i19]) + (-2.451930e-03) * (q[i5] * q[i20]) + (2.088894e-03) * (q[i5] * q[i21])
            + (-2.087590e-03) * (q[i5] * q[i22]) + (9.574128e-05) * (q[i6] * q[i7]) + (1.185074e-04) * (q[i6] * q[i8]) + (-1.053298e-03) * (q[i6] * q[i9])
            + (1.761238e-03) * (q[i6] * q[i10]) + (-3.321811e-03) * (q[i6] * q[i11]) + (5.265804e-03) * (q[i6] * q[i12]) + (4.697587e-03) * (q[i6] * q[i15])
            + (1.186916e-03) * (q[i6] * q[i16]) + (-1.181307e-03) * (q[i6] * q[i19]) + (2.162745e-03) * (q[i6] * q[i20]) + (-1.000557e-03) * (q[i6] * q[i21])
            + (3.790578e-03) * (q[i6] * q[i22]) + (8.310662e-04) * (q[i7] * q[i8]) + (-5.500530e-04) * (q[i7] * q[i9]) + (-2.524895e-03) * (q[i7] * q[i10])
            + (5.338200e-03) * (q[i7] * q[i11]) + (-3.673652e-03) * (q[i7] * q[i12]) + (3.929025e-03) * (q[i7] * q[i15]) + (-4.584537e-03) * (q[i7] * q[i16])
            + (-2.230980e-04) * (q[i7] * q[i19]) + (5.809542e-03) * (q[i7] * q[i20]) + (1.440824e-03) * (q[i7] * q[i21]) + (-3.218778e-04) * (q[i7] * q[i22])
            + (8.914568e-04) * (q[i8] * q[i9]) + (-4.275390e-04) * (q[i8] * q[i10]) + (2.691313e-03) * (q[i8] * q[i11]) + (2.354166e-03) * (q[i8] * q[i12])
            + (-8.223876e-04) * (q[i8] * q[i15]) + (3.525317e-06) * (q[i8] * q[i16]) + (5.242117e-03) * (q[i8] * q[i19]) + (-2.827921e-03) * (q[i8] * q[i20])
            + (1.435959e-03) * (q[i8] * q[i21]) + (2.509093e-03) * (q[i8] * q[i22]) + (4.041426e-04) * (q[i9] * q[i10]) + (-1.194464e-03) * (q[i9] * q[i11])
            + (-6.590604e-04) * (q[i9] * q[i12]) + (-2.520779e-04) * (q[i9] * q[i15]) + (1.079600e-03) * (q[i9] * q[i16]) + (1.489055e-03) * (q[i9] * q[i19])
            + (6.169341e-04) * (q[i9] * q[i20]) + (-1.580515e-03) * (q[i9] * q[i21]) + (4.432069e-04) * (q[i9] * q[i22]) + (7.508213e-04) * (q[i10] * q[i11])
            + (2.924787e-04) * (q[i10] * q[i12]) + (-4.046427e-05) * (q[i10] * q[i15]) + (-2.314218e-03) * (q[i10] * q[i16])
            + (1.010646e-03) * (q[i10] * q[i19]) + (-1.256187e-03) * (q[i10] * q[i20]) + (-2.090453e-03) * (q[i10] * q[i21])
            + (-1.610804e-03) * (q[i10] * q[i22]) + (4.219337e-04) * (q[i11] * q[i12]) + (-1.423635e-03) * (q[i11] * q[i15])
            + (4.422790e-04) * (q[i11] * q[i16]) + (-1.174579e-03) * (q[i11] * q[i19]) + (-2.797302e-03) * (q[i11] * q[i20])
            + (-1.517272e-03) * (q[i11] * q[i21]) + (-7.618473e-04) * (q[i11] * q[i22]) + (-4.134908e-04) * (q[i12] * q[i15])
            + (-4.669852e-04) * (q[i12] * q[i16]) + (-4.828984e-03) * (q[i12] * q[i19]) + (-2.117439e-03) * (q[i12] * q[i20])
            + (-5.185118e-04) * (q[i12] * q[i21]) + (9.435594e-05) * (q[i12] * q[i22]) + (2.937128e-04) * (q[i15] * q[i16])
            + (-1.359044e-03) * (q[i15] * q[i19]) + (1.892597e-03) * (q[i15] * q[i20]) + (2.977472e-04) * (q[i15] * q[i21]) + (1.001613e-03) * (q[i15] * q[i22])
            + (-2.043554e-03) * (q[i16] * q[i19]) + (2.998594e-03) * (q[i16] * q[i20]) + (9.347620e-04) * (q[i16] * q[i21]) + (1.107827e-03) * (q[i16] * q[i22])
            + (-2.845084e-04) * (q[i19] * q[i20]) + (2.922209e-04) * (q[i19] * q[i21]) + (-7.987878e-04) * (q[i19] * q[i22])
            + (8.213946e-04) * (q[i20] * q[i21]) + (1.826411e-03) * (q[i20] * q[i22]) + (7.898597e-05) * (q[i21] * q[i22]);
   }

   public void getJQz4(double[] q, double[][] JQ)
   {
      JQ[3][i4] = (2.211737e-02) * (1) + (6.368734e-03) * ((2) * q[i4]) + (-5.138662e-02) * (q[i0]) + (6.048093e-02) * (q[i1]) + (1.287635e-02) * (q[i2])
            + (-6.414578e-05) * (q[i3]) + (-3.470244e-03) * (q[i5]) + (1.139101e-02) * (q[i6]) + (1.688534e-02) * (q[i7]) + (-9.773458e-04) * (q[i8])
            + (1.158554e-02) * (q[i9]) + (5.640228e-03) * (q[i10]) + (2.082612e-03) * (q[i11]) + (1.588879e-03) * (q[i12]) + (-2.693480e-04) * (q[i15])
            + (7.795655e-03) * (q[i16]) + (1.143341e-02) * (q[i19]) + (-1.241368e-02) * (q[i20]) + (-3.541732e-03) * (q[i21]) + (-1.061346e-02) * (q[i22])
            + (-2.024621e-03) * (q[i0] * q[i0]) + (-7.512135e-03) * (q[i1] * q[i1]) + (-3.960318e-03) * (q[i2] * q[i2]) + (1.782037e-03) * (q[i3] * q[i3])
            + (-2.229401e-03) * ((2) * q[i0] * q[i4]) + (-6.662712e-03) * ((2) * q[i1] * q[i4]) + (-6.295773e-03) * ((2) * q[i2] * q[i4])
            + (1.758108e-03) * ((2) * q[i3] * q[i4]) + (3.280418e-03) * ((3) * q[i4] * q[i4]) + (-4.701985e-03) * ((2) * q[i4] * q[i5])
            + (-9.034951e-03) * ((2) * q[i4] * q[i6]) + (-5.931101e-03) * ((2) * q[i4] * q[i7]) + (-4.616553e-03) * ((2) * q[i4] * q[i8])
            + (2.690506e-03) * ((2) * q[i4] * q[i9]) + (-8.586939e-04) * ((2) * q[i4] * q[i10]) + (-3.515045e-04) * ((2) * q[i4] * q[i11])
            + (3.047201e-03) * ((2) * q[i4] * q[i12]) + (2.749408e-03) * ((2) * q[i4] * q[i15]) + (6.002643e-03) * ((2) * q[i4] * q[i16])
            + (2.218625e-04) * ((2) * q[i4] * q[i19]) + (-8.482367e-04) * ((2) * q[i4] * q[i20]) + (1.014072e-03) * ((2) * q[i4] * q[i21])
            + (-1.839932e-03) * ((2) * q[i4] * q[i22]) + (3.387415e-03) * (q[i5] * q[i5]) + (-3.818273e-03) * (q[i6] * q[i6])
            + (-5.239347e-03) * (q[i7] * q[i7]) + (-4.035328e-03) * (q[i8] * q[i8]) + (-1.127005e-03) * (q[i9] * q[i9]) + (-1.073550e-03) * (q[i10] * q[i10])
            + (-2.803565e-03) * (q[i11] * q[i11]) + (-3.361134e-04) * (q[i12] * q[i12]) + (1.108226e-03) * (q[i15] * q[i15])
            + (-1.787982e-03) * (q[i16] * q[i16]) + (9.882835e-04) * (q[i19] * q[i19]) + (-1.590706e-03) * (q[i20] * q[i20])
            + (1.312973e-03) * (q[i21] * q[i21]) + (-7.465614e-04) * (q[i22] * q[i22]) + (1.092826e-03) * (q[i0] * q[i1]) + (-5.079142e-03) * (q[i0] * q[i2])
            + (1.157804e-02) * (q[i0] * q[i3]) + (3.113038e-04) * (q[i0] * q[i5]) + (2.345123e-03) * (q[i0] * q[i6]) + (-3.091155e-03) * (q[i0] * q[i7])
            + (2.799478e-03) * (q[i0] * q[i8]) + (5.130394e-03) * (q[i0] * q[i9]) + (6.354973e-03) * (q[i0] * q[i10]) + (-4.735293e-04) * (q[i0] * q[i11])
            + (3.386984e-03) * (q[i0] * q[i12]) + (-3.231341e-03) * (q[i0] * q[i15]) + (-1.271616e-04) * (q[i0] * q[i16]) + (-1.119306e-05) * (q[i0] * q[i19])
            + (-5.171096e-04) * (q[i0] * q[i20]) + (-2.774221e-03) * (q[i0] * q[i21]) + (-7.930453e-04) * (q[i0] * q[i22]) + (2.120396e-03) * (q[i1] * q[i2])
            + (1.164925e-02) * (q[i1] * q[i3]) + (-1.174509e-02) * (q[i1] * q[i5]) + (-3.075405e-03) * (q[i1] * q[i6]) + (1.026329e-02) * (q[i1] * q[i7])
            + (-5.652658e-03) * (q[i1] * q[i8]) + (-4.481237e-04) * (q[i1] * q[i9]) + (-6.823310e-03) * (q[i1] * q[i10]) + (-4.153900e-03) * (q[i1] * q[i11])
            + (-1.935043e-03) * (q[i1] * q[i12]) + (4.452672e-03) * (q[i1] * q[i15]) + (-3.699263e-03) * (q[i1] * q[i16]) + (4.557667e-04) * (q[i1] * q[i19])
            + (6.645825e-04) * (q[i1] * q[i20]) + (-2.249974e-03) * (q[i1] * q[i21]) + (2.335218e-03) * (q[i1] * q[i22]) + (8.438957e-04) * (q[i2] * q[i3])
            + (-1.289630e-02) * (q[i2] * q[i5]) + (8.420013e-06) * (q[i2] * q[i6]) + (3.457939e-03) * (q[i2] * q[i7]) + (-3.298082e-03) * (q[i2] * q[i8])
            + (2.980178e-03) * (q[i2] * q[i9]) + (-1.570757e-03) * (q[i2] * q[i10]) + (-3.083760e-03) * (q[i2] * q[i11]) + (1.235821e-03) * (q[i2] * q[i12])
            + (-6.401135e-04) * (q[i2] * q[i15]) + (-3.348564e-05) * (q[i2] * q[i16]) + (2.656238e-03) * (q[i2] * q[i19]) + (9.512730e-04) * (q[i2] * q[i20])
            + (-3.841436e-03) * (q[i2] * q[i21]) + (1.912853e-03) * (q[i2] * q[i22]) + (8.470272e-03) * (q[i3] * q[i5]) + (8.451358e-03) * (q[i3] * q[i6])
            + (-8.385159e-03) * (q[i3] * q[i7]) + (-5.215263e-05) * (q[i3] * q[i8]) + (-2.546138e-03) * (q[i3] * q[i9]) + (2.573182e-03) * (q[i3] * q[i10])
            + (1.651983e-04) * (q[i3] * q[i11]) + (2.118888e-04) * (q[i3] * q[i12]) + (1.928381e-03) * (q[i3] * q[i15]) + (-1.965659e-03) * (q[i3] * q[i16])
            + (-1.061026e-03) * (q[i3] * q[i19]) + (1.098279e-03) * (q[i3] * q[i20]) + (-1.279848e-03) * (q[i3] * q[i21]) + (-1.279522e-03) * (q[i3] * q[i22])
            + (6.493612e-03) * (q[i5] * q[i6]) + (6.544851e-03) * (q[i5] * q[i7]) + (2.026581e-02) * (q[i5] * q[i8]) + (7.047726e-05) * (q[i5] * q[i9])
            + (-1.138860e-03) * (q[i5] * q[i10]) + (-3.948614e-03) * (q[i5] * q[i11]) + (1.697960e-04) * (q[i5] * q[i12]) + (-9.759214e-04) * (q[i5] * q[i15])
            + (8.420715e-04) * (q[i5] * q[i16]) + (2.421059e-03) * (q[i5] * q[i19]) + (3.417518e-03) * (q[i5] * q[i20]) + (-2.063205e-03) * (q[i5] * q[i21])
            + (2.065826e-03) * (q[i5] * q[i22]) + (1.566079e-04) * (q[i6] * q[i7]) + (8.183180e-04) * (q[i6] * q[i8]) + (-2.593748e-03) * (q[i6] * q[i9])
            + (-5.370685e-04) * (q[i6] * q[i10]) + (3.676719e-03) * (q[i6] * q[i11]) + (-5.365699e-03) * (q[i6] * q[i12]) + (-4.621407e-03) * (q[i6] * q[i15])
            + (3.924413e-03) * (q[i6] * q[i16]) + (5.798120e-03) * (q[i6] * q[i19]) + (-2.300427e-04) * (q[i6] * q[i20]) + (3.513589e-04) * (q[i6] * q[i21])
            + (-1.427510e-03) * (q[i6] * q[i22]) + (1.354132e-04) * (q[i7] * q[i8]) + (1.765688e-03) * (q[i7] * q[i9]) + (-1.119688e-03) * (q[i7] * q[i10])
            + (-5.271561e-03) * (q[i7] * q[i11]) + (3.306854e-03) * (q[i7] * q[i12]) + (1.152578e-03) * (q[i7] * q[i15]) + (4.677970e-03) * (q[i7] * q[i16])
            + (2.130440e-03) * (q[i7] * q[i19]) + (-1.210903e-03) * (q[i7] * q[i20]) + (-3.757145e-03) * (q[i7] * q[i21]) + (1.006111e-03) * (q[i7] * q[i22])
            + (-4.158017e-04) * (q[i8] * q[i9]) + (8.800150e-04) * (q[i8] * q[i10]) + (-2.375399e-03) * (q[i8] * q[i11]) + (-2.759669e-03) * (q[i8] * q[i12])
            + (1.167909e-05) * (q[i8] * q[i15]) + (-8.594415e-04) * (q[i8] * q[i16]) + (-2.804724e-03) * (q[i8] * q[i19]) + (5.203166e-03) * (q[i8] * q[i20])
            + (-2.477220e-03) * (q[i8] * q[i21]) + (-1.455301e-03) * (q[i8] * q[i22]) + (3.924239e-04) * (q[i9] * q[i10]) + (-3.106594e-04) * (q[i9] * q[i11])
            + (-7.626711e-04) * (q[i9] * q[i12]) + (-2.307338e-03) * (q[i9] * q[i15]) + (-3.622675e-05) * (q[i9] * q[i16]) + (-1.238296e-03) * (q[i9] * q[i19])
            + (1.006603e-03) * (q[i9] * q[i20]) + (1.596703e-03) * (q[i9] * q[i21]) + (2.096791e-03) * (q[i9] * q[i22]) + (6.662096e-04) * (q[i10] * q[i11])
            + (1.201299e-03) * (q[i10] * q[i12]) + (1.070567e-03) * (q[i10] * q[i15]) + (-2.303241e-04) * (q[i10] * q[i16]) + (5.875405e-04) * (q[i10] * q[i19])
            + (1.476427e-03) * (q[i10] * q[i20]) + (-4.333095e-04) * (q[i10] * q[i21]) + (1.577342e-03) * (q[i10] * q[i22]) + (4.096850e-04) * (q[i11] * q[i12])
            + (4.705882e-04) * (q[i11] * q[i15]) + (4.229579e-04) * (q[i11] * q[i16]) + (2.112294e-03) * (q[i11] * q[i19]) + (4.808062e-03) * (q[i11] * q[i20])
            + (1.044082e-04) * (q[i11] * q[i21]) + (-5.197775e-04) * (q[i11] * q[i22]) + (-4.336719e-04) * (q[i12] * q[i15])
            + (1.438830e-03) * (q[i12] * q[i16]) + (2.819507e-03) * (q[i12] * q[i19]) + (1.160681e-03) * (q[i12] * q[i20]) + (-7.572880e-04) * (q[i12] * q[i21])
            + (-1.530939e-03) * (q[i12] * q[i22]) + (3.093467e-04) * (q[i15] * q[i16]) + (3.023984e-03) * (q[i15] * q[i19])
            + (-2.037393e-03) * (q[i15] * q[i20]) + (-1.102679e-03) * (q[i15] * q[i21]) + (-9.009462e-04) * (q[i15] * q[i22])
            + (1.869637e-03) * (q[i16] * q[i19]) + (-1.350580e-03) * (q[i16] * q[i20]) + (-1.002016e-03) * (q[i16] * q[i21])
            + (-2.786692e-04) * (q[i16] * q[i22]) + (-2.449044e-04) * (q[i19] * q[i20]) + (-1.807287e-03) * (q[i19] * q[i21])
            + (-8.107735e-04) * (q[i19] * q[i22]) + (7.983853e-04) * (q[i20] * q[i21]) + (-2.967091e-04) * (q[i20] * q[i22])
            + (6.475461e-05) * (q[i21] * q[i22]);
   }

   public void getJQz5(double[] q, double[][] JQ)
   {
      JQ[3][i5] = (-1.718942e-02) * (1) + (1.956306e-05) * ((2) * q[i5]) + (-8.213849e-03) * (q[i0]) + (8.116596e-03) * (q[i1]) + (-2.055079e-05) * (q[i2])
            + (3.490649e-03) * (q[i3]) + (-3.470244e-03) * (q[i4]) + (2.939048e-02) * (q[i6]) + (2.907652e-02) * (q[i7]) + (5.116424e-02) * (q[i8])
            + (8.933518e-03) * (q[i9]) + (8.822487e-03) * (q[i10]) + (-3.697470e-03) * (q[i11]) + (3.888892e-03) * (q[i12]) + (-1.199885e-03) * (q[i15])
            + (-1.234830e-03) * (q[i16]) + (5.678138e-03) * (q[i19]) + (5.708390e-03) * (q[i20]) + (-3.162355e-03) * (q[i21]) + (3.131660e-03) * (q[i22])
            + (1.598918e-04) * (q[i0] * q[i0]) + (1.282382e-04) * (q[i1] * q[i1]) + (5.685596e-03) * (q[i2] * q[i2]) + (-4.691578e-03) * (q[i3] * q[i3])
            + (-4.701985e-03) * (q[i4] * q[i4]) + (4.455479e-05) * ((2) * q[i0] * q[i5]) + (-6.971971e-05) * ((2) * q[i1] * q[i5])
            + (-1.450116e-02) * ((2) * q[i2] * q[i5]) + (3.354428e-03) * ((2) * q[i3] * q[i5]) + (3.387415e-03) * ((2) * q[i4] * q[i5])
            + (2.352317e-03) * ((3) * q[i5] * q[i5]) + (2.022121e-04) * ((2) * q[i5] * q[i6]) + (-2.879824e-04) * ((2) * q[i5] * q[i7])
            + (7.539750e-05) * ((2) * q[i5] * q[i8]) + (4.889481e-05) * ((2) * q[i5] * q[i9]) + (-8.529623e-05) * ((2) * q[i5] * q[i10])
            + (3.295042e-03) * ((2) * q[i5] * q[i11]) + (3.401737e-03) * ((2) * q[i5] * q[i12]) + (9.894973e-04) * ((2) * q[i5] * q[i15])
            + (-9.961190e-04) * ((2) * q[i5] * q[i16]) + (2.131419e-03) * ((2) * q[i5] * q[i19]) + (-2.149900e-03) * ((2) * q[i5] * q[i20])
            + (4.273690e-04) * ((2) * q[i5] * q[i21]) + (4.067289e-04) * ((2) * q[i5] * q[i22]) + (8.587558e-03) * (q[i6] * q[i6])
            + (8.560408e-03) * (q[i7] * q[i7]) + (6.413173e-03) * (q[i8] * q[i8]) + (-1.496355e-03) * (q[i9] * q[i9]) + (-1.465258e-03) * (q[i10] * q[i10])
            + (-4.641605e-04) * (q[i11] * q[i11]) + (-4.418924e-04) * (q[i12] * q[i12]) + (-1.275408e-03) * (q[i15] * q[i15])
            + (-1.308459e-03) * (q[i16] * q[i16]) + (5.672238e-04) * (q[i19] * q[i19]) + (5.569911e-04) * (q[i20] * q[i20])
            + (-3.217971e-04) * (q[i21] * q[i21]) + (-3.075707e-04) * (q[i22] * q[i22]) + (5.936415e-03) * (q[i0] * q[i1]) + (7.131349e-03) * (q[i0] * q[i2])
            + (-1.175620e-02) * (q[i0] * q[i3]) + (3.113038e-04) * (q[i0] * q[i4]) + (2.328631e-03) * (q[i0] * q[i6]) + (-2.363745e-04) * (q[i0] * q[i7])
            + (3.065318e-03) * (q[i0] * q[i8]) + (2.392677e-03) * (q[i0] * q[i9]) + (1.078840e-03) * (q[i0] * q[i10]) + (1.322023e-03) * (q[i0] * q[i11])
            + (1.282937e-03) * (q[i0] * q[i12]) + (3.452045e-03) * (q[i0] * q[i15]) + (-2.146369e-03) * (q[i0] * q[i16]) + (-1.380481e-03) * (q[i0] * q[i19])
            + (-5.019998e-04) * (q[i0] * q[i20]) + (-2.871135e-03) * (q[i0] * q[i21]) + (-8.908884e-04) * (q[i0] * q[i22]) + (7.188995e-03) * (q[i1] * q[i2])
            + (3.613486e-04) * (q[i1] * q[i3]) + (-1.174509e-02) * (q[i1] * q[i4]) + (2.449185e-04) * (q[i1] * q[i6]) + (-2.324987e-03) * (q[i1] * q[i7])
            + (-3.085534e-03) * (q[i1] * q[i8]) + (-1.058509e-03) * (q[i1] * q[i9]) + (-2.381350e-03) * (q[i1] * q[i10]) + (1.337867e-03) * (q[i1] * q[i11])
            + (1.235463e-03) * (q[i1] * q[i12]) + (2.119109e-03) * (q[i1] * q[i15]) + (-3.452865e-03) * (q[i1] * q[i16]) + (5.274652e-04) * (q[i1] * q[i19])
            + (1.376363e-03) * (q[i1] * q[i20]) + (-8.784839e-04) * (q[i1] * q[i21]) + (-2.824797e-03) * (q[i1] * q[i22]) + (-1.295654e-02) * (q[i2] * q[i3])
            + (-1.289630e-02) * (q[i2] * q[i4]) + (-8.390429e-03) * (q[i2] * q[i6]) + (8.471359e-03) * (q[i2] * q[i7]) + (-1.490702e-05) * (q[i2] * q[i8])
            + (2.289242e-03) * (q[i2] * q[i9]) + (-2.286014e-03) * (q[i2] * q[i10]) + (-1.072867e-03) * (q[i2] * q[i11]) + (-1.137662e-03) * (q[i2] * q[i12])
            + (7.764108e-03) * (q[i2] * q[i15]) + (-7.806409e-03) * (q[i2] * q[i16]) + (-3.188673e-03) * (q[i2] * q[i19]) + (3.189714e-03) * (q[i2] * q[i20])
            + (-2.336499e-03) * (q[i2] * q[i21]) + (-2.288843e-03) * (q[i2] * q[i22]) + (8.470272e-03) * (q[i3] * q[i4]) + (-6.583003e-03) * (q[i3] * q[i6])
            + (-6.469581e-03) * (q[i3] * q[i7]) + (-2.029328e-02) * (q[i3] * q[i8]) + (1.136585e-03) * (q[i3] * q[i9]) + (-4.226047e-05) * (q[i3] * q[i10])
            + (1.932446e-04) * (q[i3] * q[i11]) + (-4.053934e-03) * (q[i3] * q[i12]) + (-8.369577e-04) * (q[i3] * q[i15]) + (1.036362e-03) * (q[i3] * q[i16])
            + (-3.405769e-03) * (q[i3] * q[i19]) + (-2.451930e-03) * (q[i3] * q[i20]) + (2.088894e-03) * (q[i3] * q[i21]) + (-2.087590e-03) * (q[i3] * q[i22])
            + (6.493612e-03) * (q[i4] * q[i6]) + (6.544851e-03) * (q[i4] * q[i7]) + (2.026581e-02) * (q[i4] * q[i8]) + (7.047726e-05) * (q[i4] * q[i9])
            + (-1.138860e-03) * (q[i4] * q[i10]) + (-3.948614e-03) * (q[i4] * q[i11]) + (1.697960e-04) * (q[i4] * q[i12]) + (-9.759214e-04) * (q[i4] * q[i15])
            + (8.420715e-04) * (q[i4] * q[i16]) + (2.421059e-03) * (q[i4] * q[i19]) + (3.417518e-03) * (q[i4] * q[i20]) + (-2.063205e-03) * (q[i4] * q[i21])
            + (2.065826e-03) * (q[i4] * q[i22]) + (-1.077427e-02) * (q[i6] * q[i7]) + (2.130050e-04) * (q[i6] * q[i8]) + (-3.894283e-03) * (q[i6] * q[i9])
            + (-1.495500e-03) * (q[i6] * q[i10]) + (-1.098048e-03) * (q[i6] * q[i11]) + (-1.625085e-03) * (q[i6] * q[i12]) + (-3.531705e-03) * (q[i6] * q[i15])
            + (-2.354698e-03) * (q[i6] * q[i16]) + (2.702330e-03) * (q[i6] * q[i19]) + (-9.051232e-04) * (q[i6] * q[i20]) + (1.591960e-03) * (q[i6] * q[i21])
            + (-1.461438e-03) * (q[i6] * q[i22]) + (2.489370e-04) * (q[i7] * q[i8]) + (-1.484852e-03) * (q[i7] * q[i9]) + (-3.813284e-03) * (q[i7] * q[i10])
            + (1.649069e-03) * (q[i7] * q[i11]) + (1.139513e-03) * (q[i7] * q[i12]) + (-2.396054e-03) * (q[i7] * q[i15]) + (-3.510798e-03) * (q[i7] * q[i16])
            + (-8.926671e-04) * (q[i7] * q[i19]) + (2.701717e-03) * (q[i7] * q[i20]) + (1.449493e-03) * (q[i7] * q[i21]) + (-1.590293e-03) * (q[i7] * q[i22])
            + (2.075229e-03) * (q[i8] * q[i9]) + (2.087639e-03) * (q[i8] * q[i10]) + (-5.032207e-03) * (q[i8] * q[i11]) + (5.177861e-03) * (q[i8] * q[i12])
            + (-3.516862e-03) * (q[i8] * q[i15]) + (-3.532130e-03) * (q[i8] * q[i16]) + (-1.052659e-03) * (q[i8] * q[i19]) + (-1.009716e-03) * (q[i8] * q[i20])
            + (1.641170e-03) * (q[i8] * q[i21]) + (-1.689967e-03) * (q[i8] * q[i22]) + (3.383062e-06) * (q[i9] * q[i10]) + (-4.488976e-04) * (q[i9] * q[i11])
            + (3.316518e-04) * (q[i9] * q[i12]) + (6.361504e-04) * (q[i9] * q[i15]) + (-8.130769e-04) * (q[i9] * q[i16]) + (3.115368e-04) * (q[i9] * q[i19])
            + (-1.091896e-03) * (q[i9] * q[i20]) + (7.568795e-04) * (q[i9] * q[i21]) + (-1.920676e-03) * (q[i9] * q[i22]) + (-3.314254e-04) * (q[i10] * q[i11])
            + (4.846041e-04) * (q[i10] * q[i12]) + (-7.985036e-04) * (q[i10] * q[i15]) + (6.463539e-04) * (q[i10] * q[i16])
            + (-1.069672e-03) * (q[i10] * q[i19]) + (3.255600e-04) * (q[i10] * q[i20]) + (1.911686e-03) * (q[i10] * q[i21])
            + (-7.536486e-04) * (q[i10] * q[i22]) + (-5.596421e-05) * (q[i11] * q[i12]) + (-3.651541e-03) * (q[i11] * q[i15])
            + (1.199525e-03) * (q[i11] * q[i16]) + (-7.826358e-04) * (q[i11] * q[i19]) + (8.982612e-04) * (q[i11] * q[i20])
            + (-2.270249e-04) * (q[i11] * q[i21]) + (7.702120e-04) * (q[i11] * q[i22]) + (-1.202181e-03) * (q[i12] * q[i15])
            + (3.666847e-03) * (q[i12] * q[i16]) + (-9.107160e-04) * (q[i12] * q[i19]) + (7.582725e-04) * (q[i12] * q[i20]) + (7.650291e-04) * (q[i12] * q[i21])
            + (-2.058770e-04) * (q[i12] * q[i22]) + (1.537212e-03) * (q[i15] * q[i16]) + (7.910750e-04) * (q[i15] * q[i19]) + (3.173976e-05) * (q[i15] * q[i20])
            + (-1.798743e-04) * (q[i15] * q[i21]) + (1.713108e-04) * (q[i15] * q[i22]) + (5.531803e-05) * (q[i16] * q[i19]) + (8.106571e-04) * (q[i16] * q[i20])
            + (-1.722371e-04) * (q[i16] * q[i21]) + (1.665068e-04) * (q[i16] * q[i22]) + (4.123524e-06) * (q[i19] * q[i20]) + (2.630843e-04) * (q[i19] * q[i21])
            + (5.709591e-04) * (q[i19] * q[i22]) + (-5.749362e-04) * (q[i20] * q[i21]) + (-2.760606e-04) * (q[i20] * q[i22])
            + (-9.237874e-04) * (q[i21] * q[i22]);
   }

   public void getJQz6(double[] q, double[][] JQ)
   {
      JQ[3][i6] = (8.517761e-02) * (1) + (-6.686235e-03) * ((2) * q[i6]) + (1.570326e-02) * (q[i0]) + (-1.079350e-02) * (q[i1]) + (1.407401e-02) * (q[i2])
            + (1.711444e-02) * (q[i3]) + (1.139101e-02) * (q[i4]) + (2.939048e-02) * (q[i5]) + (8.185942e-06) * (q[i7]) + (-8.877688e-03) * (q[i8])
            + (-6.607333e-03) * (q[i9]) + (1.429997e-03) * (q[i10]) + (8.965253e-04) * (q[i11]) + (1.560899e-03) * (q[i12]) + (2.112994e-03) * (q[i15])
            + (1.119554e-02) * (q[i16]) + (-8.683908e-03) * (q[i19]) + (-5.856227e-03) * (q[i20]) + (-1.736492e-03) * (q[i21]) + (2.301098e-03) * (q[i22])
            + (-1.235529e-02) * (q[i0] * q[i0]) + (3.284873e-03) * (q[i1] * q[i1]) + (-2.160606e-04) * (q[i2] * q[i2]) + (5.830212e-03) * (q[i3] * q[i3])
            + (-9.034951e-03) * (q[i4] * q[i4]) + (2.022121e-04) * (q[i5] * q[i5]) + (-1.078988e-02) * ((2) * q[i0] * q[i6])
            + (6.680487e-03) * ((2) * q[i1] * q[i6]) + (-4.822411e-03) * ((2) * q[i2] * q[i6]) + (-5.211335e-03) * ((2) * q[i3] * q[i6])
            + (-3.818273e-03) * ((2) * q[i4] * q[i6]) + (8.587558e-03) * ((2) * q[i5] * q[i6]) + (-3.909764e-03) * ((3) * q[i6] * q[i6])
            + (1.289841e-03) * ((2) * q[i6] * q[i7]) + (7.130810e-03) * ((2) * q[i6] * q[i8]) + (-2.098500e-04) * ((2) * q[i6] * q[i9])
            + (2.881432e-03) * ((2) * q[i6] * q[i10]) + (-4.684032e-03) * ((2) * q[i6] * q[i11]) + (1.484473e-03) * ((2) * q[i6] * q[i12])
            + (2.561257e-03) * ((2) * q[i6] * q[i15]) + (-1.554538e-03) * ((2) * q[i6] * q[i16]) + (2.031403e-04) * ((2) * q[i6] * q[i19])
            + (-8.974494e-04) * ((2) * q[i6] * q[i20]) + (-1.225484e-03) * ((2) * q[i6] * q[i21]) + (-1.812007e-03) * ((2) * q[i6] * q[i22])
            + (-1.309745e-03) * (q[i7] * q[i7]) + (1.706365e-03) * (q[i8] * q[i8]) + (-1.563730e-03) * (q[i9] * q[i9]) + (1.977798e-04) * (q[i10] * q[i10])
            + (2.021229e-03) * (q[i11] * q[i11]) + (2.741810e-03) * (q[i12] * q[i12]) + (1.350029e-03) * (q[i15] * q[i15]) + (1.732938e-03) * (q[i16] * q[i16])
            + (1.277015e-03) * (q[i19] * q[i19]) + (1.285111e-03) * (q[i20] * q[i20]) + (1.607276e-03) * (q[i21] * q[i21]) + (1.739386e-03) * (q[i22] * q[i22])
            + (-3.535111e-04) * (q[i0] * q[i1]) + (-8.584065e-03) * (q[i0] * q[i2]) + (-1.011579e-02) * (q[i0] * q[i3]) + (2.345123e-03) * (q[i0] * q[i4])
            + (2.328631e-03) * (q[i0] * q[i5]) + (7.590054e-03) * (q[i0] * q[i7]) + (3.540452e-03) * (q[i0] * q[i8]) + (-6.566775e-03) * (q[i0] * q[i9])
            + (6.778875e-03) * (q[i0] * q[i10]) + (2.362114e-03) * (q[i0] * q[i11]) + (1.859035e-03) * (q[i0] * q[i12]) + (-3.302023e-04) * (q[i0] * q[i15])
            + (2.531603e-03) * (q[i0] * q[i16]) + (-1.968210e-03) * (q[i0] * q[i19]) + (-7.278929e-04) * (q[i0] * q[i20]) + (-4.359618e-04) * (q[i0] * q[i21])
            + (1.950349e-03) * (q[i0] * q[i22]) + (2.627176e-03) * (q[i1] * q[i2]) + (3.009775e-03) * (q[i1] * q[i3]) + (-3.075405e-03) * (q[i1] * q[i4])
            + (2.449185e-04) * (q[i1] * q[i5]) + (7.598896e-03) * (q[i1] * q[i7]) + (-1.137300e-03) * (q[i1] * q[i8]) + (5.926933e-03) * (q[i1] * q[i9])
            + (2.451999e-04) * (q[i1] * q[i10]) + (7.141237e-04) * (q[i1] * q[i11]) + (8.925012e-04) * (q[i1] * q[i12]) + (-2.981604e-03) * (q[i1] * q[i15])
            + (1.727913e-03) * (q[i1] * q[i16]) + (-1.552690e-04) * (q[i1] * q[i19]) + (-2.713295e-03) * (q[i1] * q[i20]) + (6.585911e-04) * (q[i1] * q[i21])
            + (-4.110103e-04) * (q[i1] * q[i22]) + (-3.436195e-03) * (q[i2] * q[i3]) + (8.420013e-06) * (q[i2] * q[i4]) + (-8.390429e-03) * (q[i2] * q[i5])
            + (1.407930e-02) * (q[i2] * q[i7]) + (-9.756797e-03) * (q[i2] * q[i8]) + (-4.292913e-03) * (q[i2] * q[i9]) + (1.744585e-03) * (q[i2] * q[i10])
            + (2.110339e-03) * (q[i2] * q[i11]) + (3.404679e-03) * (q[i2] * q[i12]) + (-2.437519e-03) * (q[i2] * q[i15]) + (1.432669e-03) * (q[i2] * q[i16])
            + (-9.689233e-04) * (q[i2] * q[i19]) + (1.426482e-03) * (q[i2] * q[i20]) + (-3.636610e-04) * (q[i2] * q[i21]) + (4.758287e-04) * (q[i2] * q[i22])
            + (8.451358e-03) * (q[i3] * q[i4]) + (-6.583003e-03) * (q[i3] * q[i5]) + (9.574128e-05) * (q[i3] * q[i7]) + (1.185074e-04) * (q[i3] * q[i8])
            + (-1.053298e-03) * (q[i3] * q[i9]) + (1.761238e-03) * (q[i3] * q[i10]) + (-3.321811e-03) * (q[i3] * q[i11]) + (5.265804e-03) * (q[i3] * q[i12])
            + (4.697587e-03) * (q[i3] * q[i15]) + (1.186916e-03) * (q[i3] * q[i16]) + (-1.181307e-03) * (q[i3] * q[i19]) + (2.162745e-03) * (q[i3] * q[i20])
            + (-1.000557e-03) * (q[i3] * q[i21]) + (3.790578e-03) * (q[i3] * q[i22]) + (6.493612e-03) * (q[i4] * q[i5]) + (1.566079e-04) * (q[i4] * q[i7])
            + (8.183180e-04) * (q[i4] * q[i8]) + (-2.593748e-03) * (q[i4] * q[i9]) + (-5.370685e-04) * (q[i4] * q[i10]) + (3.676719e-03) * (q[i4] * q[i11])
            + (-5.365699e-03) * (q[i4] * q[i12]) + (-4.621407e-03) * (q[i4] * q[i15]) + (3.924413e-03) * (q[i4] * q[i16]) + (5.798120e-03) * (q[i4] * q[i19])
            + (-2.300427e-04) * (q[i4] * q[i20]) + (3.513589e-04) * (q[i4] * q[i21]) + (-1.427510e-03) * (q[i4] * q[i22]) + (-1.077427e-02) * (q[i5] * q[i7])
            + (2.130050e-04) * (q[i5] * q[i8]) + (-3.894283e-03) * (q[i5] * q[i9]) + (-1.495500e-03) * (q[i5] * q[i10]) + (-1.098048e-03) * (q[i5] * q[i11])
            + (-1.625085e-03) * (q[i5] * q[i12]) + (-3.531705e-03) * (q[i5] * q[i15]) + (-2.354698e-03) * (q[i5] * q[i16]) + (2.702330e-03) * (q[i5] * q[i19])
            + (-9.051232e-04) * (q[i5] * q[i20]) + (1.591960e-03) * (q[i5] * q[i21]) + (-1.461438e-03) * (q[i5] * q[i22]) + (1.471681e-06) * (q[i7] * q[i8])
            + (3.538223e-03) * (q[i7] * q[i9]) + (-3.533052e-03) * (q[i7] * q[i10]) + (7.670973e-04) * (q[i7] * q[i11]) + (7.360295e-04) * (q[i7] * q[i12])
            + (-4.667196e-03) * (q[i7] * q[i15]) + (4.670192e-03) * (q[i7] * q[i16]) + (1.347722e-03) * (q[i7] * q[i19]) + (-1.363415e-03) * (q[i7] * q[i20])
            + (1.620153e-03) * (q[i7] * q[i21]) + (1.609069e-03) * (q[i7] * q[i22]) + (1.728767e-04) * (q[i8] * q[i9]) + (3.467390e-04) * (q[i8] * q[i10])
            + (2.328102e-03) * (q[i8] * q[i11]) + (-6.499768e-04) * (q[i8] * q[i12]) + (-1.649729e-03) * (q[i8] * q[i15]) + (-2.953595e-03) * (q[i8] * q[i16])
            + (9.782341e-04) * (q[i8] * q[i19]) + (3.762517e-03) * (q[i8] * q[i20]) + (-3.744837e-04) * (q[i8] * q[i21]) + (5.308637e-04) * (q[i8] * q[i22])
            + (-1.387681e-04) * (q[i9] * q[i10]) + (9.282152e-04) * (q[i9] * q[i11]) + (-1.665129e-03) * (q[i9] * q[i12]) + (-4.085242e-04) * (q[i9] * q[i15])
            + (-1.681619e-03) * (q[i9] * q[i16]) + (1.523656e-03) * (q[i9] * q[i19]) + (1.277208e-03) * (q[i9] * q[i20]) + (1.961747e-04) * (q[i9] * q[i21])
            + (-2.454917e-03) * (q[i9] * q[i22]) + (1.092517e-03) * (q[i10] * q[i11]) + (-3.319301e-03) * (q[i10] * q[i12])
            + (-3.119390e-03) * (q[i10] * q[i15]) + (8.119270e-04) * (q[i10] * q[i16]) + (1.324738e-03) * (q[i10] * q[i19]) + (7.252227e-04) * (q[i10] * q[i20])
            + (1.626460e-03) * (q[i10] * q[i21]) + (-4.929493e-04) * (q[i10] * q[i22]) + (-6.797917e-03) * (q[i11] * q[i12])
            + (-2.988266e-05) * (q[i11] * q[i15]) + (2.932153e-03) * (q[i11] * q[i16]) + (1.671965e-03) * (q[i11] * q[i19])
            + (-7.149938e-04) * (q[i11] * q[i20]) + (1.916454e-03) * (q[i11] * q[i21]) + (1.473500e-03) * (q[i11] * q[i22])
            + (-3.450668e-03) * (q[i12] * q[i15]) + (9.277737e-04) * (q[i12] * q[i16]) + (-3.422618e-04) * (q[i12] * q[i19])
            + (-1.686519e-03) * (q[i12] * q[i20]) + (-1.191898e-03) * (q[i12] * q[i21]) + (1.967394e-03) * (q[i12] * q[i22])
            + (-3.686725e-04) * (q[i15] * q[i16]) + (-1.345635e-03) * (q[i15] * q[i19]) + (-1.620104e-03) * (q[i15] * q[i20])
            + (-1.247835e-03) * (q[i15] * q[i21]) + (9.407677e-04) * (q[i15] * q[i22]) + (-3.332313e-03) * (q[i16] * q[i19])
            + (5.609957e-04) * (q[i16] * q[i20]) + (5.767641e-04) * (q[i16] * q[i21]) + (-1.211639e-05) * (q[i16] * q[i22]) + (1.521665e-03) * (q[i19] * q[i20])
            + (-1.122185e-03) * (q[i19] * q[i21]) + (1.952267e-04) * (q[i19] * q[i22]) + (-4.661550e-05) * (q[i20] * q[i21])
            + (-1.946770e-03) * (q[i20] * q[i22]) + (-3.455159e-04) * (q[i21] * q[i22]);
   }

   public void getJQz7(double[] q, double[][] JQ)
   {
      JQ[3][i7] = (-8.492880e-02) * (1) + (6.668476e-03) * ((2) * q[i7]) + (-1.074346e-02) * (q[i0]) + (1.575324e-02) * (q[i1]) + (1.409484e-02) * (q[i2])
            + (1.131108e-02) * (q[i3]) + (1.688534e-02) * (q[i4]) + (2.907652e-02) * (q[i5]) + (8.185942e-06) * (q[i6]) + (8.758807e-03) * (q[i8])
            + (-1.487626e-03) * (q[i9]) + (6.514480e-03) * (q[i10]) + (1.726801e-03) * (q[i11]) + (9.740498e-04) * (q[i12]) + (-1.104024e-02) * (q[i15])
            + (-2.191571e-03) * (q[i16]) + (5.833061e-03) * (q[i19]) + (8.629763e-03) * (q[i20]) + (2.335451e-03) * (q[i21]) + (-1.764857e-03) * (q[i22])
            + (-3.272551e-03) * (q[i0] * q[i0]) + (1.242606e-02) * (q[i1] * q[i1]) + (2.160540e-04) * (q[i2] * q[i2]) + (9.049718e-03) * (q[i3] * q[i3])
            + (-5.931101e-03) * (q[i4] * q[i4]) + (-2.879824e-04) * (q[i5] * q[i5]) + (1.289841e-03) * (q[i6] * q[i6]) + (6.715776e-03) * ((2) * q[i0] * q[i7])
            + (-1.079080e-02) * ((2) * q[i1] * q[i7]) + (-4.756895e-03) * ((2) * q[i2] * q[i7]) + (-3.817495e-03) * ((2) * q[i3] * q[i7])
            + (-5.239347e-03) * ((2) * q[i4] * q[i7]) + (8.560408e-03) * ((2) * q[i5] * q[i7]) + (-1.309745e-03) * ((2) * q[i6] * q[i7])
            + (3.920256e-03) * ((3) * q[i7] * q[i7]) + (-7.145583e-03) * ((2) * q[i7] * q[i8]) + (-2.885807e-03) * ((2) * q[i7] * q[i9])
            + (2.142158e-04) * ((2) * q[i7] * q[i10]) + (1.484495e-03) * ((2) * q[i7] * q[i11]) + (-4.665546e-03) * ((2) * q[i7] * q[i12])
            + (1.562721e-03) * ((2) * q[i7] * q[i15]) + (-2.543950e-03) * ((2) * q[i7] * q[i16]) + (8.816005e-04) * ((2) * q[i7] * q[i19])
            + (-2.149406e-04) * ((2) * q[i7] * q[i20]) + (-1.779634e-03) * ((2) * q[i7] * q[i21]) + (-1.211107e-03) * ((2) * q[i7] * q[i22])
            + (-1.682221e-03) * (q[i8] * q[i8]) + (-1.879608e-04) * (q[i9] * q[i9]) + (1.547509e-03) * (q[i10] * q[i10]) + (-2.718147e-03) * (q[i11] * q[i11])
            + (-1.990822e-03) * (q[i12] * q[i12]) + (-1.718359e-03) * (q[i15] * q[i15]) + (-1.365205e-03) * (q[i16] * q[i16])
            + (-1.300125e-03) * (q[i19] * q[i19]) + (-1.281165e-03) * (q[i20] * q[i20]) + (-1.741215e-03) * (q[i21] * q[i21])
            + (-1.603073e-03) * (q[i22] * q[i22]) + (2.757222e-04) * (q[i0] * q[i1]) + (-2.635861e-03) * (q[i0] * q[i2]) + (3.106750e-03) * (q[i0] * q[i3])
            + (-3.091155e-03) * (q[i0] * q[i4]) + (-2.363745e-04) * (q[i0] * q[i5]) + (7.590054e-03) * (q[i0] * q[i6]) + (-1.160031e-03) * (q[i0] * q[i8])
            + (2.473974e-04) * (q[i0] * q[i9]) + (5.886241e-03) * (q[i0] * q[i10]) + (-8.944924e-04) * (q[i0] * q[i11]) + (-6.621534e-04) * (q[i0] * q[i12])
            + (1.730123e-03) * (q[i0] * q[i15]) + (-2.987341e-03) * (q[i0] * q[i16]) + (-2.717691e-03) * (q[i0] * q[i19]) + (-1.157077e-04) * (q[i0] * q[i20])
            + (4.351924e-04) * (q[i0] * q[i21]) + (-6.437164e-04) * (q[i0] * q[i22]) + (8.588029e-03) * (q[i1] * q[i2]) + (-2.379827e-03) * (q[i1] * q[i3])
            + (1.026329e-02) * (q[i1] * q[i4]) + (-2.324987e-03) * (q[i1] * q[i5]) + (7.598896e-03) * (q[i1] * q[i6]) + (3.540632e-03) * (q[i1] * q[i8])
            + (6.815199e-03) * (q[i1] * q[i9]) + (-6.555638e-03) * (q[i1] * q[i10]) + (-1.815133e-03) * (q[i1] * q[i11]) + (-2.364231e-03) * (q[i1] * q[i12])
            + (2.516036e-03) * (q[i1] * q[i15]) + (-2.919010e-04) * (q[i1] * q[i16]) + (-7.084220e-04) * (q[i1] * q[i19]) + (-1.945346e-03) * (q[i1] * q[i20])
            + (-1.962824e-03) * (q[i1] * q[i21]) + (4.281154e-04) * (q[i1] * q[i22]) + (-5.176599e-05) * (q[i2] * q[i3]) + (3.457939e-03) * (q[i2] * q[i4])
            + (8.471359e-03) * (q[i2] * q[i5]) + (1.407930e-02) * (q[i2] * q[i6]) + (-9.730376e-03) * (q[i2] * q[i8]) + (1.755898e-03) * (q[i2] * q[i9])
            + (-4.274821e-03) * (q[i2] * q[i10]) + (-3.373453e-03) * (q[i2] * q[i11]) + (-2.075903e-03) * (q[i2] * q[i12]) + (1.418427e-03) * (q[i2] * q[i15])
            + (-2.459923e-03) * (q[i2] * q[i16]) + (1.409915e-03) * (q[i2] * q[i19]) + (-9.379687e-04) * (q[i2] * q[i20]) + (-4.657147e-04) * (q[i2] * q[i21])
            + (3.561437e-04) * (q[i2] * q[i22]) + (-8.385159e-03) * (q[i3] * q[i4]) + (-6.469581e-03) * (q[i3] * q[i5]) + (9.574128e-05) * (q[i3] * q[i6])
            + (8.310662e-04) * (q[i3] * q[i8]) + (-5.500530e-04) * (q[i3] * q[i9]) + (-2.524895e-03) * (q[i3] * q[i10]) + (5.338200e-03) * (q[i3] * q[i11])
            + (-3.673652e-03) * (q[i3] * q[i12]) + (3.929025e-03) * (q[i3] * q[i15]) + (-4.584537e-03) * (q[i3] * q[i16]) + (-2.230980e-04) * (q[i3] * q[i19])
            + (5.809542e-03) * (q[i3] * q[i20]) + (1.440824e-03) * (q[i3] * q[i21]) + (-3.218778e-04) * (q[i3] * q[i22]) + (6.544851e-03) * (q[i4] * q[i5])
            + (1.566079e-04) * (q[i4] * q[i6]) + (1.354132e-04) * (q[i4] * q[i8]) + (1.765688e-03) * (q[i4] * q[i9]) + (-1.119688e-03) * (q[i4] * q[i10])
            + (-5.271561e-03) * (q[i4] * q[i11]) + (3.306854e-03) * (q[i4] * q[i12]) + (1.152578e-03) * (q[i4] * q[i15]) + (4.677970e-03) * (q[i4] * q[i16])
            + (2.130440e-03) * (q[i4] * q[i19]) + (-1.210903e-03) * (q[i4] * q[i20]) + (-3.757145e-03) * (q[i4] * q[i21]) + (1.006111e-03) * (q[i4] * q[i22])
            + (-1.077427e-02) * (q[i5] * q[i6]) + (2.489370e-04) * (q[i5] * q[i8]) + (-1.484852e-03) * (q[i5] * q[i9]) + (-3.813284e-03) * (q[i5] * q[i10])
            + (1.649069e-03) * (q[i5] * q[i11]) + (1.139513e-03) * (q[i5] * q[i12]) + (-2.396054e-03) * (q[i5] * q[i15]) + (-3.510798e-03) * (q[i5] * q[i16])
            + (-8.926671e-04) * (q[i5] * q[i19]) + (2.701717e-03) * (q[i5] * q[i20]) + (1.449493e-03) * (q[i5] * q[i21]) + (-1.590293e-03) * (q[i5] * q[i22])
            + (1.471681e-06) * (q[i6] * q[i8]) + (3.538223e-03) * (q[i6] * q[i9]) + (-3.533052e-03) * (q[i6] * q[i10]) + (7.670973e-04) * (q[i6] * q[i11])
            + (7.360295e-04) * (q[i6] * q[i12]) + (-4.667196e-03) * (q[i6] * q[i15]) + (4.670192e-03) * (q[i6] * q[i16]) + (1.347722e-03) * (q[i6] * q[i19])
            + (-1.363415e-03) * (q[i6] * q[i20]) + (1.620153e-03) * (q[i6] * q[i21]) + (1.609069e-03) * (q[i6] * q[i22]) + (-3.371606e-04) * (q[i8] * q[i9])
            + (-1.569436e-04) * (q[i8] * q[i10]) + (-6.914101e-04) * (q[i8] * q[i11]) + (2.296906e-03) * (q[i8] * q[i12]) + (2.943766e-03) * (q[i8] * q[i15])
            + (1.613064e-03) * (q[i8] * q[i16]) + (-3.737186e-03) * (q[i8] * q[i19]) + (-9.455939e-04) * (q[i8] * q[i20]) + (5.372617e-04) * (q[i8] * q[i21])
            + (-3.697157e-04) * (q[i8] * q[i22]) + (1.400217e-04) * (q[i9] * q[i10]) + (-3.324927e-03) * (q[i9] * q[i11]) + (1.092796e-03) * (q[i9] * q[i12])
            + (-8.232981e-04) * (q[i9] * q[i15]) + (3.118214e-03) * (q[i9] * q[i16]) + (-7.282846e-04) * (q[i9] * q[i19]) + (-1.328406e-03) * (q[i9] * q[i20])
            + (-5.005814e-04) * (q[i9] * q[i21]) + (1.636119e-03) * (q[i9] * q[i22]) + (-1.687183e-03) * (q[i10] * q[i11]) + (9.381162e-04) * (q[i10] * q[i12])
            + (1.665228e-03) * (q[i10] * q[i15]) + (4.276289e-04) * (q[i10] * q[i16]) + (-1.287692e-03) * (q[i10] * q[i19])
            + (-1.523878e-03) * (q[i10] * q[i20]) + (-2.451621e-03) * (q[i10] * q[i21]) + (1.984686e-04) * (q[i10] * q[i22])
            + (6.788398e-03) * (q[i11] * q[i12]) + (9.438830e-04) * (q[i11] * q[i15]) + (-3.429429e-03) * (q[i11] * q[i16])
            + (-1.694066e-03) * (q[i11] * q[i19]) + (-3.149649e-04) * (q[i11] * q[i20]) + (-1.967779e-03) * (q[i11] * q[i21])
            + (1.176254e-03) * (q[i11] * q[i22]) + (2.947093e-03) * (q[i12] * q[i15]) + (-3.053283e-05) * (q[i12] * q[i16])
            + (-7.212440e-04) * (q[i12] * q[i19]) + (1.663596e-03) * (q[i12] * q[i20]) + (-1.454366e-03) * (q[i12] * q[i21])
            + (-1.900401e-03) * (q[i12] * q[i22]) + (3.657008e-04) * (q[i15] * q[i16]) + (-5.565482e-04) * (q[i15] * q[i19])
            + (3.309427e-03) * (q[i15] * q[i20]) + (-8.648373e-06) * (q[i15] * q[i21]) + (5.829073e-04) * (q[i15] * q[i22]) + (1.611454e-03) * (q[i16] * q[i19])
            + (1.343040e-03) * (q[i16] * q[i20]) + (9.408520e-04) * (q[i16] * q[i21]) + (-1.249415e-03) * (q[i16] * q[i22])
            + (-1.519264e-03) * (q[i19] * q[i20]) + (-1.940345e-03) * (q[i19] * q[i21]) + (-4.046829e-05) * (q[i19] * q[i22])
            + (2.033601e-04) * (q[i20] * q[i21]) + (-1.112846e-03) * (q[i20] * q[i22]) + (3.555269e-04) * (q[i21] * q[i22]);
   }

   public void getJQz8(double[] q, double[][] JQ)
   {
      JQ[3][i8] = (-2.080289e-04) * (1) + (9.495748e-05) * ((2) * q[i8]) + (8.653143e-03) * (q[i0]) + (8.756454e-03) * (q[i1]) + (-1.008699e-03) * (q[i2])
            + (-9.928663e-04) * (q[i3]) + (-9.773458e-04) * (q[i4]) + (5.116424e-02) * (q[i5]) + (-8.877688e-03) * (q[i6]) + (8.758807e-03) * (q[i7])
            + (-2.700869e-03) * (q[i9]) + (2.641316e-03) * (q[i10]) + (-3.691522e-03) * (q[i11]) + (-3.862373e-03) * (q[i12]) + (1.443567e-02) * (q[i15])
            + (-1.453026e-02) * (q[i16]) + (4.252995e-03) * (q[i19]) + (-4.199044e-03) * (q[i20]) + (3.168813e-03) * (q[i21]) + (3.195922e-03) * (q[i22])
            + (-9.077013e-04) * (q[i0] * q[i0]) + (8.847795e-04) * (q[i1] * q[i1]) + (6.674590e-06) * (q[i2] * q[i2]) + (4.594219e-03) * (q[i3] * q[i3])
            + (-4.616553e-03) * (q[i4] * q[i4]) + (7.539750e-05) * (q[i5] * q[i5]) + (7.130810e-03) * (q[i6] * q[i6]) + (-7.145583e-03) * (q[i7] * q[i7])
            + (8.763863e-04) * ((2) * q[i0] * q[i8]) + (8.621596e-04) * ((2) * q[i1] * q[i8]) + (6.311364e-03) * ((2) * q[i2] * q[i8])
            + (-4.006531e-03) * ((2) * q[i3] * q[i8]) + (-4.035328e-03) * ((2) * q[i4] * q[i8]) + (6.413173e-03) * ((2) * q[i5] * q[i8])
            + (1.706365e-03) * ((2) * q[i6] * q[i8]) + (-1.682221e-03) * ((2) * q[i7] * q[i8]) + (-2.481958e-05) * ((3) * q[i8] * q[i8])
            + (3.472965e-04) * ((2) * q[i8] * q[i9]) + (-3.438137e-04) * ((2) * q[i8] * q[i10]) + (-5.372922e-03) * ((2) * q[i8] * q[i11])
            + (-5.461197e-03) * ((2) * q[i8] * q[i12]) + (-1.147863e-03) * ((2) * q[i8] * q[i15]) + (1.224891e-03) * ((2) * q[i8] * q[i16])
            + (-1.183642e-03) * ((2) * q[i8] * q[i19]) + (1.153215e-03) * ((2) * q[i8] * q[i20]) + (1.220470e-03) * ((2) * q[i8] * q[i21])
            + (1.243642e-03) * ((2) * q[i8] * q[i22]) + (-5.020887e-04) * (q[i9] * q[i9]) + (5.061291e-04) * (q[i10] * q[i10])
            + (-2.565567e-03) * (q[i11] * q[i11]) + (2.620844e-03) * (q[i12] * q[i12]) + (-1.623978e-03) * (q[i15] * q[i15])
            + (1.643932e-03) * (q[i16] * q[i16]) + (-2.297524e-04) * (q[i19] * q[i19]) + (2.460478e-04) * (q[i20] * q[i20]) + (4.810772e-04) * (q[i21] * q[i21])
            + (-4.754923e-04) * (q[i22] * q[i22]) + (5.311938e-05) * (q[i0] * q[i1]) + (5.935676e-03) * (q[i0] * q[i2]) + (5.752914e-03) * (q[i0] * q[i3])
            + (2.799478e-03) * (q[i0] * q[i4]) + (3.065318e-03) * (q[i0] * q[i5]) + (3.540452e-03) * (q[i0] * q[i6]) + (-1.160031e-03) * (q[i0] * q[i7])
            + (3.435307e-04) * (q[i0] * q[i9]) + (-1.449731e-03) * (q[i0] * q[i10]) + (-1.501626e-04) * (q[i0] * q[i11]) + (3.196186e-04) * (q[i0] * q[i12])
            + (-1.777304e-03) * (q[i0] * q[i15]) + (-1.266048e-03) * (q[i0] * q[i16]) + (-9.278645e-04) * (q[i0] * q[i19]) + (2.572781e-03) * (q[i0] * q[i20])
            + (5.334901e-04) * (q[i0] * q[i21]) + (-1.699471e-03) * (q[i0] * q[i22]) + (-5.944304e-03) * (q[i1] * q[i2]) + (-2.802561e-03) * (q[i1] * q[i3])
            + (-5.652658e-03) * (q[i1] * q[i4]) + (-3.085534e-03) * (q[i1] * q[i5]) + (-1.137300e-03) * (q[i1] * q[i6]) + (3.540632e-03) * (q[i1] * q[i7])
            + (-1.458352e-03) * (q[i1] * q[i9]) + (3.259798e-04) * (q[i1] * q[i10]) + (-3.038879e-04) * (q[i1] * q[i11]) + (1.257303e-04) * (q[i1] * q[i12])
            + (-1.220725e-03) * (q[i1] * q[i15]) + (-1.803216e-03) * (q[i1] * q[i16]) + (2.591051e-03) * (q[i1] * q[i19]) + (-9.210163e-04) * (q[i1] * q[i20])
            + (1.685769e-03) * (q[i1] * q[i21]) + (-5.526813e-04) * (q[i1] * q[i22]) + (3.351840e-03) * (q[i2] * q[i3]) + (-3.298082e-03) * (q[i2] * q[i4])
            + (-1.490702e-05) * (q[i2] * q[i5]) + (-9.756797e-03) * (q[i2] * q[i6]) + (-9.730376e-03) * (q[i2] * q[i7]) + (3.126228e-04) * (q[i2] * q[i9])
            + (3.500331e-04) * (q[i2] * q[i10]) + (6.417629e-05) * (q[i2] * q[i11]) + (-1.014369e-04) * (q[i2] * q[i12]) + (-6.401808e-04) * (q[i2] * q[i15])
            + (-6.618732e-04) * (q[i2] * q[i16]) + (-2.143070e-03) * (q[i2] * q[i19]) + (-2.164839e-03) * (q[i2] * q[i20]) + (7.322682e-04) * (q[i2] * q[i21])
            + (-7.450876e-04) * (q[i2] * q[i22]) + (-5.215263e-05) * (q[i3] * q[i4]) + (-2.029328e-02) * (q[i3] * q[i5]) + (1.185074e-04) * (q[i3] * q[i6])
            + (8.310662e-04) * (q[i3] * q[i7]) + (8.914568e-04) * (q[i3] * q[i9]) + (-4.275390e-04) * (q[i3] * q[i10]) + (2.691313e-03) * (q[i3] * q[i11])
            + (2.354166e-03) * (q[i3] * q[i12]) + (-8.223876e-04) * (q[i3] * q[i15]) + (3.525317e-06) * (q[i3] * q[i16]) + (5.242117e-03) * (q[i3] * q[i19])
            + (-2.827921e-03) * (q[i3] * q[i20]) + (1.435959e-03) * (q[i3] * q[i21]) + (2.509093e-03) * (q[i3] * q[i22]) + (2.026581e-02) * (q[i4] * q[i5])
            + (8.183180e-04) * (q[i4] * q[i6]) + (1.354132e-04) * (q[i4] * q[i7]) + (-4.158017e-04) * (q[i4] * q[i9]) + (8.800150e-04) * (q[i4] * q[i10])
            + (-2.375399e-03) * (q[i4] * q[i11]) + (-2.759669e-03) * (q[i4] * q[i12]) + (1.167909e-05) * (q[i4] * q[i15]) + (-8.594415e-04) * (q[i4] * q[i16])
            + (-2.804724e-03) * (q[i4] * q[i19]) + (5.203166e-03) * (q[i4] * q[i20]) + (-2.477220e-03) * (q[i4] * q[i21]) + (-1.455301e-03) * (q[i4] * q[i22])
            + (2.130050e-04) * (q[i5] * q[i6]) + (2.489370e-04) * (q[i5] * q[i7]) + (2.075229e-03) * (q[i5] * q[i9]) + (2.087639e-03) * (q[i5] * q[i10])
            + (-5.032207e-03) * (q[i5] * q[i11]) + (5.177861e-03) * (q[i5] * q[i12]) + (-3.516862e-03) * (q[i5] * q[i15]) + (-3.532130e-03) * (q[i5] * q[i16])
            + (-1.052659e-03) * (q[i5] * q[i19]) + (-1.009716e-03) * (q[i5] * q[i20]) + (1.641170e-03) * (q[i5] * q[i21]) + (-1.689967e-03) * (q[i5] * q[i22])
            + (1.471681e-06) * (q[i6] * q[i7]) + (1.728767e-04) * (q[i6] * q[i9]) + (3.467390e-04) * (q[i6] * q[i10]) + (2.328102e-03) * (q[i6] * q[i11])
            + (-6.499768e-04) * (q[i6] * q[i12]) + (-1.649729e-03) * (q[i6] * q[i15]) + (-2.953595e-03) * (q[i6] * q[i16]) + (9.782341e-04) * (q[i6] * q[i19])
            + (3.762517e-03) * (q[i6] * q[i20]) + (-3.744837e-04) * (q[i6] * q[i21]) + (5.308637e-04) * (q[i6] * q[i22]) + (-3.371606e-04) * (q[i7] * q[i9])
            + (-1.569436e-04) * (q[i7] * q[i10]) + (-6.914101e-04) * (q[i7] * q[i11]) + (2.296906e-03) * (q[i7] * q[i12]) + (2.943766e-03) * (q[i7] * q[i15])
            + (1.613064e-03) * (q[i7] * q[i16]) + (-3.737186e-03) * (q[i7] * q[i19]) + (-9.455939e-04) * (q[i7] * q[i20]) + (5.372617e-04) * (q[i7] * q[i21])
            + (-3.697157e-04) * (q[i7] * q[i22]) + (7.190316e-06) * (q[i9] * q[i10]) + (1.505537e-03) * (q[i9] * q[i11]) + (7.128823e-04) * (q[i9] * q[i12])
            + (-2.079523e-03) * (q[i9] * q[i15]) + (-2.149540e-03) * (q[i9] * q[i16]) + (8.208506e-04) * (q[i9] * q[i19]) + (2.952076e-03) * (q[i9] * q[i20])
            + (-1.144188e-03) * (q[i9] * q[i21]) + (4.128965e-04) * (q[i9] * q[i22]) + (7.206284e-04) * (q[i10] * q[i11]) + (1.499905e-03) * (q[i10] * q[i12])
            + (2.132471e-03) * (q[i10] * q[i15]) + (2.067358e-03) * (q[i10] * q[i16]) + (-2.938725e-03) * (q[i10] * q[i19])
            + (-8.147241e-04) * (q[i10] * q[i20]) + (4.143830e-04) * (q[i10] * q[i21]) + (-1.149477e-03) * (q[i10] * q[i22])
            + (5.213333e-07) * (q[i11] * q[i12]) + (2.826673e-03) * (q[i11] * q[i15]) + (1.193063e-03) * (q[i11] * q[i16]) + (-1.131070e-03) * (q[i11] * q[i19])
            + (4.758059e-04) * (q[i11] * q[i20]) + (-3.066098e-04) * (q[i11] * q[i21]) + (-1.142485e-03) * (q[i11] * q[i22])
            + (1.212328e-03) * (q[i12] * q[i15]) + (2.828037e-03) * (q[i12] * q[i16]) + (4.809914e-04) * (q[i12] * q[i19]) + (-1.072583e-03) * (q[i12] * q[i20])
            + (1.149213e-03) * (q[i12] * q[i21]) + (3.046039e-04) * (q[i12] * q[i22]) + (7.098969e-07) * (q[i15] * q[i16]) + (1.069179e-03) * (q[i15] * q[i19])
            + (-7.574531e-04) * (q[i15] * q[i20]) + (2.629756e-03) * (q[i15] * q[i21]) + (-3.028679e-04) * (q[i15] * q[i22])
            + (7.779851e-04) * (q[i16] * q[i19]) + (-1.079222e-03) * (q[i16] * q[i20]) + (-2.999376e-04) * (q[i16] * q[i21])
            + (2.667583e-03) * (q[i16] * q[i22]) + (5.116259e-06) * (q[i19] * q[i20]) + (3.348708e-03) * (q[i19] * q[i21]) + (2.725898e-04) * (q[i19] * q[i22])
            + (2.678443e-04) * (q[i20] * q[i21]) + (3.337514e-03) * (q[i20] * q[i22]) + (7.139614e-06) * (q[i21] * q[i22]);
   }

   public void getJQz9(double[] q, double[][] JQ)
   {
      JQ[3][i9] = (3.172784e-02) * (1) + (-3.316302e-03) * ((2) * q[i9]) + (-5.261070e-03) * (q[i0]) + (-1.332474e-03) * (q[i1]) + (-1.280027e-03) * (q[i2])
            + (5.648699e-03) * (q[i3]) + (1.158554e-02) * (q[i4]) + (8.933518e-03) * (q[i5]) + (-6.607333e-03) * (q[i6]) + (-1.487626e-03) * (q[i7])
            + (-2.700869e-03) * (q[i8]) + (-3.860355e-06) * (q[i10]) + (-1.635497e-03) * (q[i11]) + (1.497003e-03) * (q[i12]) + (-2.044194e-03) * (q[i15])
            + (1.961958e-03) * (q[i16]) + (-4.272644e-03) * (q[i19]) + (-1.320874e-03) * (q[i20]) + (-4.451931e-04) * (q[i21]) + (2.513326e-03) * (q[i22])
            + (-2.269364e-03) * (q[i0] * q[i0]) + (2.143130e-03) * (q[i1] * q[i1]) + (4.256555e-04) * (q[i2] * q[i2]) + (8.430913e-04) * (q[i3] * q[i3])
            + (2.690506e-03) * (q[i4] * q[i4]) + (4.889481e-05) * (q[i5] * q[i5]) + (-2.098500e-04) * (q[i6] * q[i6]) + (-2.885807e-03) * (q[i7] * q[i7])
            + (3.472965e-04) * (q[i8] * q[i8]) + (3.571000e-03) * ((2) * q[i0] * q[i9]) + (2.252412e-04) * ((2) * q[i1] * q[i9])
            + (9.906023e-04) * ((2) * q[i2] * q[i9]) + (-1.039539e-03) * ((2) * q[i3] * q[i9]) + (-1.127005e-03) * ((2) * q[i4] * q[i9])
            + (-1.496355e-03) * ((2) * q[i5] * q[i9]) + (-1.563730e-03) * ((2) * q[i6] * q[i9]) + (-1.879608e-04) * ((2) * q[i7] * q[i9])
            + (-5.020887e-04) * ((2) * q[i8] * q[i9]) + (-7.815466e-04) * ((3) * q[i9] * q[i9]) + (-4.161969e-04) * ((2) * q[i9] * q[i10])
            + (7.893895e-04) * ((2) * q[i9] * q[i11]) + (-3.654373e-04) * ((2) * q[i9] * q[i12]) + (-1.776470e-04) * ((2) * q[i9] * q[i15])
            + (-8.181243e-04) * ((2) * q[i9] * q[i16]) + (5.688618e-04) * ((2) * q[i9] * q[i19]) + (5.050609e-04) * ((2) * q[i9] * q[i20])
            + (1.956634e-04) * ((2) * q[i9] * q[i21]) + (-7.894434e-04) * ((2) * q[i9] * q[i22]) + (4.154889e-04) * (q[i10] * q[i10])
            + (3.303996e-05) * (q[i11] * q[i11]) + (8.229866e-04) * (q[i12] * q[i12]) + (-9.476405e-05) * (q[i15] * q[i15])
            + (-4.486444e-04) * (q[i16] * q[i16]) + (3.851531e-04) * (q[i19] * q[i19]) + (-2.195414e-04) * (q[i20] * q[i20])
            + (3.766099e-04) * (q[i21] * q[i21]) + (-6.016814e-05) * (q[i22] * q[i22]) + (-1.650176e-03) * (q[i0] * q[i1]) + (-1.593483e-03) * (q[i0] * q[i2])
            + (6.880723e-03) * (q[i0] * q[i3]) + (5.130394e-03) * (q[i0] * q[i4]) + (2.392677e-03) * (q[i0] * q[i5]) + (-6.566775e-03) * (q[i0] * q[i6])
            + (2.473974e-04) * (q[i0] * q[i7]) + (3.435307e-04) * (q[i0] * q[i8]) + (1.333336e-03) * (q[i0] * q[i10]) + (9.486425e-06) * (q[i0] * q[i11])
            + (-4.151017e-04) * (q[i0] * q[i12]) + (1.080065e-04) * (q[i0] * q[i15]) + (-8.717527e-04) * (q[i0] * q[i16]) + (1.619438e-03) * (q[i0] * q[i19])
            + (-3.476660e-04) * (q[i0] * q[i20]) + (1.984431e-04) * (q[i0] * q[i21]) + (4.594173e-05) * (q[i0] * q[i22]) + (1.251727e-03) * (q[i1] * q[i2])
            + (-6.364877e-03) * (q[i1] * q[i3]) + (-4.481237e-04) * (q[i1] * q[i4]) + (-1.058509e-03) * (q[i1] * q[i5]) + (5.926933e-03) * (q[i1] * q[i6])
            + (6.815199e-03) * (q[i1] * q[i7]) + (-1.458352e-03) * (q[i1] * q[i8]) + (1.366859e-03) * (q[i1] * q[i10]) + (-1.395787e-03) * (q[i1] * q[i11])
            + (1.418468e-03) * (q[i1] * q[i12]) + (-2.835336e-04) * (q[i1] * q[i15]) + (-1.345912e-03) * (q[i1] * q[i16]) + (-6.331721e-04) * (q[i1] * q[i19])
            + (1.136149e-03) * (q[i1] * q[i20]) + (1.872351e-04) * (q[i1] * q[i21]) + (-6.430545e-04) * (q[i1] * q[i22]) + (1.625624e-03) * (q[i2] * q[i3])
            + (2.980178e-03) * (q[i2] * q[i4]) + (2.289242e-03) * (q[i2] * q[i5]) + (-4.292913e-03) * (q[i2] * q[i6]) + (1.755898e-03) * (q[i2] * q[i7])
            + (3.126228e-04) * (q[i2] * q[i8]) + (1.046591e-03) * (q[i2] * q[i10]) + (-1.559192e-03) * (q[i2] * q[i11]) + (7.825747e-04) * (q[i2] * q[i12])
            + (1.132058e-03) * (q[i2] * q[i15]) + (-1.885767e-03) * (q[i2] * q[i16]) + (-1.062082e-04) * (q[i2] * q[i19]) + (3.898031e-04) * (q[i2] * q[i20])
            + (9.727644e-05) * (q[i2] * q[i21]) + (-3.653917e-04) * (q[i2] * q[i22]) + (-2.546138e-03) * (q[i3] * q[i4]) + (1.136585e-03) * (q[i3] * q[i5])
            + (-1.053298e-03) * (q[i3] * q[i6]) + (-5.500530e-04) * (q[i3] * q[i7]) + (8.914568e-04) * (q[i3] * q[i8]) + (4.041426e-04) * (q[i3] * q[i10])
            + (-1.194464e-03) * (q[i3] * q[i11]) + (-6.590604e-04) * (q[i3] * q[i12]) + (-2.520779e-04) * (q[i3] * q[i15]) + (1.079600e-03) * (q[i3] * q[i16])
            + (1.489055e-03) * (q[i3] * q[i19]) + (6.169341e-04) * (q[i3] * q[i20]) + (-1.580515e-03) * (q[i3] * q[i21]) + (4.432069e-04) * (q[i3] * q[i22])
            + (7.047726e-05) * (q[i4] * q[i5]) + (-2.593748e-03) * (q[i4] * q[i6]) + (1.765688e-03) * (q[i4] * q[i7]) + (-4.158017e-04) * (q[i4] * q[i8])
            + (3.924239e-04) * (q[i4] * q[i10]) + (-3.106594e-04) * (q[i4] * q[i11]) + (-7.626711e-04) * (q[i4] * q[i12]) + (-2.307338e-03) * (q[i4] * q[i15])
            + (-3.622675e-05) * (q[i4] * q[i16]) + (-1.238296e-03) * (q[i4] * q[i19]) + (1.006603e-03) * (q[i4] * q[i20]) + (1.596703e-03) * (q[i4] * q[i21])
            + (2.096791e-03) * (q[i4] * q[i22]) + (-3.894283e-03) * (q[i5] * q[i6]) + (-1.484852e-03) * (q[i5] * q[i7]) + (2.075229e-03) * (q[i5] * q[i8])
            + (3.383062e-06) * (q[i5] * q[i10]) + (-4.488976e-04) * (q[i5] * q[i11]) + (3.316518e-04) * (q[i5] * q[i12]) + (6.361504e-04) * (q[i5] * q[i15])
            + (-8.130769e-04) * (q[i5] * q[i16]) + (3.115368e-04) * (q[i5] * q[i19]) + (-1.091896e-03) * (q[i5] * q[i20]) + (7.568795e-04) * (q[i5] * q[i21])
            + (-1.920676e-03) * (q[i5] * q[i22]) + (3.538223e-03) * (q[i6] * q[i7]) + (1.728767e-04) * (q[i6] * q[i8]) + (-1.387681e-04) * (q[i6] * q[i10])
            + (9.282152e-04) * (q[i6] * q[i11]) + (-1.665129e-03) * (q[i6] * q[i12]) + (-4.085242e-04) * (q[i6] * q[i15]) + (-1.681619e-03) * (q[i6] * q[i16])
            + (1.523656e-03) * (q[i6] * q[i19]) + (1.277208e-03) * (q[i6] * q[i20]) + (1.961747e-04) * (q[i6] * q[i21]) + (-2.454917e-03) * (q[i6] * q[i22])
            + (-3.371606e-04) * (q[i7] * q[i8]) + (1.400217e-04) * (q[i7] * q[i10]) + (-3.324927e-03) * (q[i7] * q[i11]) + (1.092796e-03) * (q[i7] * q[i12])
            + (-8.232981e-04) * (q[i7] * q[i15]) + (3.118214e-03) * (q[i7] * q[i16]) + (-7.282846e-04) * (q[i7] * q[i19]) + (-1.328406e-03) * (q[i7] * q[i20])
            + (-5.005814e-04) * (q[i7] * q[i21]) + (1.636119e-03) * (q[i7] * q[i22]) + (7.190316e-06) * (q[i8] * q[i10]) + (1.505537e-03) * (q[i8] * q[i11])
            + (7.128823e-04) * (q[i8] * q[i12]) + (-2.079523e-03) * (q[i8] * q[i15]) + (-2.149540e-03) * (q[i8] * q[i16]) + (8.208506e-04) * (q[i8] * q[i19])
            + (2.952076e-03) * (q[i8] * q[i20]) + (-1.144188e-03) * (q[i8] * q[i21]) + (4.128965e-04) * (q[i8] * q[i22]) + (-3.432840e-04) * (q[i10] * q[i11])
            + (-3.509618e-04) * (q[i10] * q[i12]) + (-7.261380e-04) * (q[i10] * q[i15]) + (7.210693e-04) * (q[i10] * q[i16])
            + (3.068323e-04) * (q[i10] * q[i19]) + (-3.114489e-04) * (q[i10] * q[i20]) + (2.671950e-04) * (q[i10] * q[i21]) + (2.713456e-04) * (q[i10] * q[i22])
            + (-1.383689e-03) * (q[i11] * q[i12]) + (4.185205e-04) * (q[i11] * q[i15]) + (6.748590e-05) * (q[i11] * q[i16]) + (2.570354e-04) * (q[i11] * q[i19])
            + (-2.698155e-04) * (q[i11] * q[i20]) + (5.498505e-04) * (q[i11] * q[i21]) + (5.739907e-04) * (q[i11] * q[i22])
            + (-5.104271e-04) * (q[i12] * q[i15]) + (8.176412e-04) * (q[i12] * q[i16]) + (8.131269e-05) * (q[i12] * q[i19])
            + (-9.948959e-06) * (q[i12] * q[i20]) + (-7.537221e-04) * (q[i12] * q[i21]) + (-2.366834e-04) * (q[i12] * q[i22])
            + (-6.006669e-04) * (q[i15] * q[i16]) + (-1.487320e-04) * (q[i15] * q[i19]) + (-2.653334e-05) * (q[i15] * q[i20])
            + (-6.297149e-04) * (q[i15] * q[i21]) + (-3.685585e-05) * (q[i15] * q[i22]) + (-9.418895e-04) * (q[i16] * q[i19])
            + (4.169658e-04) * (q[i16] * q[i20]) + (1.721087e-05) * (q[i16] * q[i21]) + (2.033018e-05) * (q[i16] * q[i22]) + (6.983687e-04) * (q[i19] * q[i20])
            + (-4.695622e-04) * (q[i19] * q[i21]) + (2.972330e-04) * (q[i19] * q[i22]) + (-1.456776e-05) * (q[i20] * q[i21])
            + (-3.028418e-04) * (q[i20] * q[i22]) + (-1.130490e-04) * (q[i21] * q[i22]);
   }

   public void getJQz10(double[] q, double[][] JQ)
   {
      JQ[3][i10] = (-3.150416e-02) * (1) + (3.293977e-03) * ((2) * q[i10]) + (-1.270144e-03) * (q[i0]) + (-5.206619e-03) * (q[i1]) + (-1.263228e-03) * (q[i2])
            + (1.150907e-02) * (q[i3]) + (5.640228e-03) * (q[i4]) + (8.822487e-03) * (q[i5]) + (1.429997e-03) * (q[i6]) + (6.514480e-03) * (q[i7])
            + (2.641316e-03) * (q[i8]) + (-3.860355e-06) * (q[i9]) + (1.510507e-03) * (q[i11]) + (-1.588390e-03) * (q[i12]) + (-1.900916e-03) * (q[i15])
            + (2.036227e-03) * (q[i16]) + (1.303528e-03) * (q[i19]) + (4.239309e-03) * (q[i20]) + (2.496257e-03) * (q[i21]) + (-4.460657e-04) * (q[i22])
            + (-2.136750e-03) * (q[i0] * q[i0]) + (2.239776e-03) * (q[i1] * q[i1]) + (-4.375921e-04) * (q[i2] * q[i2]) + (-2.710877e-03) * (q[i3] * q[i3])
            + (-8.586939e-04) * (q[i4] * q[i4]) + (-8.529623e-05) * (q[i5] * q[i5]) + (2.881432e-03) * (q[i6] * q[i6]) + (2.142158e-04) * (q[i7] * q[i7])
            + (-3.438137e-04) * (q[i8] * q[i8]) + (-4.161969e-04) * (q[i9] * q[i9]) + (2.137082e-04) * ((2) * q[i0] * q[i10])
            + (3.526691e-03) * ((2) * q[i1] * q[i10]) + (9.744991e-04) * ((2) * q[i2] * q[i10]) + (-1.109231e-03) * ((2) * q[i3] * q[i10])
            + (-1.073550e-03) * ((2) * q[i4] * q[i10]) + (-1.465258e-03) * ((2) * q[i5] * q[i10]) + (1.977798e-04) * ((2) * q[i6] * q[i10])
            + (1.547509e-03) * ((2) * q[i7] * q[i10]) + (5.061291e-04) * ((2) * q[i8] * q[i10]) + (4.154889e-04) * ((2) * q[i9] * q[i10])
            + (7.701880e-04) * ((3) * q[i10] * q[i10]) + (-3.646132e-04) * ((2) * q[i10] * q[i11]) + (7.796672e-04) * ((2) * q[i10] * q[i12])
            + (8.040850e-04) * ((2) * q[i10] * q[i15]) + (1.798451e-04) * ((2) * q[i10] * q[i16]) + (-5.029924e-04) * ((2) * q[i10] * q[i19])
            + (-5.685368e-04) * ((2) * q[i10] * q[i20]) + (-7.848734e-04) * ((2) * q[i10] * q[i21]) + (1.943379e-04) * ((2) * q[i10] * q[i22])
            + (-8.027005e-04) * (q[i11] * q[i11]) + (-1.848996e-05) * (q[i12] * q[i12]) + (4.446822e-04) * (q[i15] * q[i15])
            + (9.360250e-05) * (q[i16] * q[i16]) + (2.144442e-04) * (q[i19] * q[i19]) + (-3.820262e-04) * (q[i20] * q[i20]) + (6.369769e-05) * (q[i21] * q[i21])
            + (-3.702790e-04) * (q[i22] * q[i22]) + (1.641943e-03) * (q[i0] * q[i1]) + (-1.246070e-03) * (q[i0] * q[i2]) + (4.104927e-04) * (q[i0] * q[i3])
            + (6.354973e-03) * (q[i0] * q[i4]) + (1.078840e-03) * (q[i0] * q[i5]) + (6.778875e-03) * (q[i0] * q[i6]) + (5.886241e-03) * (q[i0] * q[i7])
            + (-1.449731e-03) * (q[i0] * q[i8]) + (1.333336e-03) * (q[i0] * q[i9]) + (-1.415883e-03) * (q[i0] * q[i11]) + (1.398713e-03) * (q[i0] * q[i12])
            + (-1.365183e-03) * (q[i0] * q[i15]) + (-2.779088e-04) * (q[i0] * q[i16]) + (1.148780e-03) * (q[i0] * q[i19]) + (-6.231768e-04) * (q[i0] * q[i20])
            + (6.428626e-04) * (q[i0] * q[i21]) + (-1.778341e-04) * (q[i0] * q[i22]) + (1.589570e-03) * (q[i1] * q[i2]) + (-5.111334e-03) * (q[i1] * q[i3])
            + (-6.823310e-03) * (q[i1] * q[i4]) + (-2.381350e-03) * (q[i1] * q[i5]) + (2.451999e-04) * (q[i1] * q[i6]) + (-6.555638e-03) * (q[i1] * q[i7])
            + (3.259798e-04) * (q[i1] * q[i8]) + (1.366859e-03) * (q[i1] * q[i9]) + (4.045033e-04) * (q[i1] * q[i11]) + (-1.472401e-05) * (q[i1] * q[i12])
            + (-8.608620e-04) * (q[i1] * q[i15]) + (1.288937e-04) * (q[i1] * q[i16]) + (-3.646995e-04) * (q[i1] * q[i19]) + (1.601052e-03) * (q[i1] * q[i20])
            + (-4.788341e-05) * (q[i1] * q[i21]) + (-1.747739e-04) * (q[i1] * q[i22]) + (-2.978439e-03) * (q[i2] * q[i3]) + (-1.570757e-03) * (q[i2] * q[i4])
            + (-2.286014e-03) * (q[i2] * q[i5]) + (1.744585e-03) * (q[i2] * q[i6]) + (-4.274821e-03) * (q[i2] * q[i7]) + (3.500331e-04) * (q[i2] * q[i8])
            + (1.046591e-03) * (q[i2] * q[i9]) + (-7.718939e-04) * (q[i2] * q[i11]) + (1.535351e-03) * (q[i2] * q[i12]) + (-1.879207e-03) * (q[i2] * q[i15])
            + (1.131607e-03) * (q[i2] * q[i16]) + (3.717942e-04) * (q[i2] * q[i19]) + (-1.153407e-04) * (q[i2] * q[i20]) + (3.553253e-04) * (q[i2] * q[i21])
            + (-8.714664e-05) * (q[i2] * q[i22]) + (2.573182e-03) * (q[i3] * q[i4]) + (-4.226047e-05) * (q[i3] * q[i5]) + (1.761238e-03) * (q[i3] * q[i6])
            + (-2.524895e-03) * (q[i3] * q[i7]) + (-4.275390e-04) * (q[i3] * q[i8]) + (4.041426e-04) * (q[i3] * q[i9]) + (7.508213e-04) * (q[i3] * q[i11])
            + (2.924787e-04) * (q[i3] * q[i12]) + (-4.046427e-05) * (q[i3] * q[i15]) + (-2.314218e-03) * (q[i3] * q[i16]) + (1.010646e-03) * (q[i3] * q[i19])
            + (-1.256187e-03) * (q[i3] * q[i20]) + (-2.090453e-03) * (q[i3] * q[i21]) + (-1.610804e-03) * (q[i3] * q[i22]) + (-1.138860e-03) * (q[i4] * q[i5])
            + (-5.370685e-04) * (q[i4] * q[i6]) + (-1.119688e-03) * (q[i4] * q[i7]) + (8.800150e-04) * (q[i4] * q[i8]) + (3.924239e-04) * (q[i4] * q[i9])
            + (6.662096e-04) * (q[i4] * q[i11]) + (1.201299e-03) * (q[i4] * q[i12]) + (1.070567e-03) * (q[i4] * q[i15]) + (-2.303241e-04) * (q[i4] * q[i16])
            + (5.875405e-04) * (q[i4] * q[i19]) + (1.476427e-03) * (q[i4] * q[i20]) + (-4.333095e-04) * (q[i4] * q[i21]) + (1.577342e-03) * (q[i4] * q[i22])
            + (-1.495500e-03) * (q[i5] * q[i6]) + (-3.813284e-03) * (q[i5] * q[i7]) + (2.087639e-03) * (q[i5] * q[i8]) + (3.383062e-06) * (q[i5] * q[i9])
            + (-3.314254e-04) * (q[i5] * q[i11]) + (4.846041e-04) * (q[i5] * q[i12]) + (-7.985036e-04) * (q[i5] * q[i15]) + (6.463539e-04) * (q[i5] * q[i16])
            + (-1.069672e-03) * (q[i5] * q[i19]) + (3.255600e-04) * (q[i5] * q[i20]) + (1.911686e-03) * (q[i5] * q[i21]) + (-7.536486e-04) * (q[i5] * q[i22])
            + (-3.533052e-03) * (q[i6] * q[i7]) + (3.467390e-04) * (q[i6] * q[i8]) + (-1.387681e-04) * (q[i6] * q[i9]) + (1.092517e-03) * (q[i6] * q[i11])
            + (-3.319301e-03) * (q[i6] * q[i12]) + (-3.119390e-03) * (q[i6] * q[i15]) + (8.119270e-04) * (q[i6] * q[i16]) + (1.324738e-03) * (q[i6] * q[i19])
            + (7.252227e-04) * (q[i6] * q[i20]) + (1.626460e-03) * (q[i6] * q[i21]) + (-4.929493e-04) * (q[i6] * q[i22]) + (-1.569436e-04) * (q[i7] * q[i8])
            + (1.400217e-04) * (q[i7] * q[i9]) + (-1.687183e-03) * (q[i7] * q[i11]) + (9.381162e-04) * (q[i7] * q[i12]) + (1.665228e-03) * (q[i7] * q[i15])
            + (4.276289e-04) * (q[i7] * q[i16]) + (-1.287692e-03) * (q[i7] * q[i19]) + (-1.523878e-03) * (q[i7] * q[i20]) + (-2.451621e-03) * (q[i7] * q[i21])
            + (1.984686e-04) * (q[i7] * q[i22]) + (7.190316e-06) * (q[i8] * q[i9]) + (7.206284e-04) * (q[i8] * q[i11]) + (1.499905e-03) * (q[i8] * q[i12])
            + (2.132471e-03) * (q[i8] * q[i15]) + (2.067358e-03) * (q[i8] * q[i16]) + (-2.938725e-03) * (q[i8] * q[i19]) + (-8.147241e-04) * (q[i8] * q[i20])
            + (4.143830e-04) * (q[i8] * q[i21]) + (-1.149477e-03) * (q[i8] * q[i22]) + (-3.432840e-04) * (q[i9] * q[i11]) + (-3.509618e-04) * (q[i9] * q[i12])
            + (-7.261380e-04) * (q[i9] * q[i15]) + (7.210693e-04) * (q[i9] * q[i16]) + (3.068323e-04) * (q[i9] * q[i19]) + (-3.114489e-04) * (q[i9] * q[i20])
            + (2.671950e-04) * (q[i9] * q[i21]) + (2.713456e-04) * (q[i9] * q[i22]) + (1.351670e-03) * (q[i11] * q[i12]) + (8.191657e-04) * (q[i11] * q[i15])
            + (-4.993969e-04) * (q[i11] * q[i16]) + (-6.430745e-06) * (q[i11] * q[i19]) + (9.173832e-05) * (q[i11] * q[i20])
            + (2.451832e-04) * (q[i11] * q[i21]) + (7.502987e-04) * (q[i11] * q[i22]) + (6.865590e-05) * (q[i12] * q[i15]) + (4.180841e-04) * (q[i12] * q[i16])
            + (-2.767815e-04) * (q[i12] * q[i19]) + (2.536174e-04) * (q[i12] * q[i20]) + (-5.718205e-04) * (q[i12] * q[i21])
            + (-5.451014e-04) * (q[i12] * q[i22]) + (6.058846e-04) * (q[i15] * q[i16]) + (-4.135366e-04) * (q[i15] * q[i19])
            + (9.230676e-04) * (q[i15] * q[i20]) + (2.476633e-05) * (q[i15] * q[i21]) + (2.225760e-05) * (q[i15] * q[i22]) + (2.947857e-05) * (q[i16] * q[i19])
            + (1.432046e-04) * (q[i16] * q[i20]) + (-4.318072e-05) * (q[i16] * q[i21]) + (-6.301378e-04) * (q[i16] * q[i22])
            + (-6.971758e-04) * (q[i19] * q[i20]) + (-3.065525e-04) * (q[i19] * q[i21]) + (-1.014092e-05) * (q[i19] * q[i22])
            + (2.987073e-04) * (q[i20] * q[i21]) + (-4.575649e-04) * (q[i20] * q[i22]) + (1.193717e-04) * (q[i21] * q[i22]);
   }

   public void getJQz11(double[] q, double[][] JQ)
   {
      JQ[3][i11] = (4.990228e-02) * (1) + (-9.116594e-04) * ((2) * q[i11]) + (6.303760e-04) * (q[i0]) + (4.869941e-04) * (q[i1]) + (2.007064e-03) * (q[i2])
            + (-1.591195e-03) * (q[i3]) + (2.082612e-03) * (q[i4]) + (-3.697470e-03) * (q[i5]) + (8.965253e-04) * (q[i6]) + (1.726801e-03) * (q[i7])
            + (-3.691522e-03) * (q[i8]) + (-1.635497e-03) * (q[i9]) + (1.510507e-03) * (q[i10]) + (2.681928e-06) * (q[i12]) + (2.564447e-03) * (q[i15])
            + (2.571983e-03) * (q[i16]) + (4.182586e-03) * (q[i19]) + (3.689563e-04) * (q[i20]) + (4.161639e-03) * (q[i21]) + (4.512438e-04) * (q[i22])
            + (-6.309729e-05) * (q[i0] * q[i0]) + (2.519575e-03) * (q[i1] * q[i1]) + (-1.959227e-03) * (q[i2] * q[i2]) + (3.105474e-03) * (q[i3] * q[i3])
            + (-3.515045e-04) * (q[i4] * q[i4]) + (3.295042e-03) * (q[i5] * q[i5]) + (-4.684032e-03) * (q[i6] * q[i6]) + (1.484495e-03) * (q[i7] * q[i7])
            + (-5.372922e-03) * (q[i8] * q[i8]) + (7.893895e-04) * (q[i9] * q[i9]) + (-3.646132e-04) * (q[i10] * q[i10])
            + (-3.013394e-04) * ((2) * q[i0] * q[i11]) + (-1.769956e-05) * ((2) * q[i1] * q[i11]) + (-2.203007e-03) * ((2) * q[i2] * q[i11])
            + (-3.986162e-04) * ((2) * q[i3] * q[i11]) + (-2.803565e-03) * ((2) * q[i4] * q[i11]) + (-4.641605e-04) * ((2) * q[i5] * q[i11])
            + (2.021229e-03) * ((2) * q[i6] * q[i11]) + (-2.718147e-03) * ((2) * q[i7] * q[i11]) + (-2.565567e-03) * ((2) * q[i8] * q[i11])
            + (3.303996e-05) * ((2) * q[i9] * q[i11]) + (-8.027005e-04) * ((2) * q[i10] * q[i11]) + (-8.028001e-04) * ((3) * q[i11] * q[i11])
            + (5.829736e-04) * ((2) * q[i11] * q[i12]) + (1.444886e-03) * ((2) * q[i11] * q[i15]) + (-5.429432e-04) * ((2) * q[i11] * q[i16])
            + (-1.210977e-03) * ((2) * q[i11] * q[i19]) + (-2.343149e-05) * ((2) * q[i11] * q[i20]) + (-1.088728e-03) * ((2) * q[i11] * q[i21])
            + (2.058726e-04) * ((2) * q[i11] * q[i22]) + (5.870360e-04) * (q[i12] * q[i12]) + (-5.868728e-03) * (q[i15] * q[i15])
            + (4.659210e-04) * (q[i16] * q[i16]) + (6.159593e-04) * (q[i19] * q[i19]) + (-1.191432e-04) * (q[i20] * q[i20])
            + (-6.023134e-04) * (q[i21] * q[i21]) + (-4.069585e-04) * (q[i22] * q[i22]) + (-6.978984e-04) * (q[i0] * q[i1]) + (5.106428e-04) * (q[i0] * q[i2])
            + (-1.905806e-03) * (q[i0] * q[i3]) + (-4.735293e-04) * (q[i0] * q[i4]) + (1.322023e-03) * (q[i0] * q[i5]) + (2.362114e-03) * (q[i0] * q[i6])
            + (-8.944924e-04) * (q[i0] * q[i7]) + (-1.501626e-04) * (q[i0] * q[i8]) + (9.486425e-06) * (q[i0] * q[i9]) + (-1.415883e-03) * (q[i0] * q[i10])
            + (5.165199e-04) * (q[i0] * q[i12]) + (-1.382556e-03) * (q[i0] * q[i15]) + (-2.685371e-04) * (q[i0] * q[i16]) + (1.617007e-03) * (q[i0] * q[i19])
            + (-1.578691e-03) * (q[i0] * q[i20]) + (-8.476260e-04) * (q[i0] * q[i21]) + (-1.043071e-03) * (q[i0] * q[i22]) + (-8.452529e-04) * (q[i1] * q[i2])
            + (3.375875e-03) * (q[i1] * q[i3]) + (-4.153900e-03) * (q[i1] * q[i4]) + (1.337867e-03) * (q[i1] * q[i5]) + (7.141237e-04) * (q[i1] * q[i6])
            + (-1.815133e-03) * (q[i1] * q[i7]) + (-3.038879e-04) * (q[i1] * q[i8]) + (-1.395787e-03) * (q[i1] * q[i9]) + (4.045033e-04) * (q[i1] * q[i10])
            + (5.198115e-04) * (q[i1] * q[i12]) + (3.872995e-03) * (q[i1] * q[i15]) + (-1.095867e-03) * (q[i1] * q[i16]) + (-2.231571e-03) * (q[i1] * q[i19])
            + (-6.580470e-04) * (q[i1] * q[i20]) + (6.031111e-04) * (q[i1] * q[i21]) + (1.702132e-03) * (q[i1] * q[i22]) + (1.251982e-03) * (q[i2] * q[i3])
            + (-3.083760e-03) * (q[i2] * q[i4]) + (-1.072867e-03) * (q[i2] * q[i5]) + (2.110339e-03) * (q[i2] * q[i6]) + (-3.373453e-03) * (q[i2] * q[i7])
            + (6.417629e-05) * (q[i2] * q[i8]) + (-1.559192e-03) * (q[i2] * q[i9]) + (-7.718939e-04) * (q[i2] * q[i10]) + (2.987426e-03) * (q[i2] * q[i12])
            + (2.351462e-03) * (q[i2] * q[i15]) + (-1.103160e-03) * (q[i2] * q[i16]) + (-7.304907e-04) * (q[i2] * q[i19]) + (6.020709e-04) * (q[i2] * q[i20])
            + (-8.595518e-04) * (q[i2] * q[i21]) + (-6.161720e-04) * (q[i2] * q[i22]) + (1.651983e-04) * (q[i3] * q[i4]) + (1.932446e-04) * (q[i3] * q[i5])
            + (-3.321811e-03) * (q[i3] * q[i6]) + (5.338200e-03) * (q[i3] * q[i7]) + (2.691313e-03) * (q[i3] * q[i8]) + (-1.194464e-03) * (q[i3] * q[i9])
            + (7.508213e-04) * (q[i3] * q[i10]) + (4.219337e-04) * (q[i3] * q[i12]) + (-1.423635e-03) * (q[i3] * q[i15]) + (4.422790e-04) * (q[i3] * q[i16])
            + (-1.174579e-03) * (q[i3] * q[i19]) + (-2.797302e-03) * (q[i3] * q[i20]) + (-1.517272e-03) * (q[i3] * q[i21]) + (-7.618473e-04) * (q[i3] * q[i22])
            + (-3.948614e-03) * (q[i4] * q[i5]) + (3.676719e-03) * (q[i4] * q[i6]) + (-5.271561e-03) * (q[i4] * q[i7]) + (-2.375399e-03) * (q[i4] * q[i8])
            + (-3.106594e-04) * (q[i4] * q[i9]) + (6.662096e-04) * (q[i4] * q[i10]) + (4.096850e-04) * (q[i4] * q[i12]) + (4.705882e-04) * (q[i4] * q[i15])
            + (4.229579e-04) * (q[i4] * q[i16]) + (2.112294e-03) * (q[i4] * q[i19]) + (4.808062e-03) * (q[i4] * q[i20]) + (1.044082e-04) * (q[i4] * q[i21])
            + (-5.197775e-04) * (q[i4] * q[i22]) + (-1.098048e-03) * (q[i5] * q[i6]) + (1.649069e-03) * (q[i5] * q[i7]) + (-5.032207e-03) * (q[i5] * q[i8])
            + (-4.488976e-04) * (q[i5] * q[i9]) + (-3.314254e-04) * (q[i5] * q[i10]) + (-5.596421e-05) * (q[i5] * q[i12]) + (-3.651541e-03) * (q[i5] * q[i15])
            + (1.199525e-03) * (q[i5] * q[i16]) + (-7.826358e-04) * (q[i5] * q[i19]) + (8.982612e-04) * (q[i5] * q[i20]) + (-2.270249e-04) * (q[i5] * q[i21])
            + (7.702120e-04) * (q[i5] * q[i22]) + (7.670973e-04) * (q[i6] * q[i7]) + (2.328102e-03) * (q[i6] * q[i8]) + (9.282152e-04) * (q[i6] * q[i9])
            + (1.092517e-03) * (q[i6] * q[i10]) + (-6.797917e-03) * (q[i6] * q[i12]) + (-2.988266e-05) * (q[i6] * q[i15]) + (2.932153e-03) * (q[i6] * q[i16])
            + (1.671965e-03) * (q[i6] * q[i19]) + (-7.149938e-04) * (q[i6] * q[i20]) + (1.916454e-03) * (q[i6] * q[i21]) + (1.473500e-03) * (q[i6] * q[i22])
            + (-6.914101e-04) * (q[i7] * q[i8]) + (-3.324927e-03) * (q[i7] * q[i9]) + (-1.687183e-03) * (q[i7] * q[i10]) + (6.788398e-03) * (q[i7] * q[i12])
            + (9.438830e-04) * (q[i7] * q[i15]) + (-3.429429e-03) * (q[i7] * q[i16]) + (-1.694066e-03) * (q[i7] * q[i19]) + (-3.149649e-04) * (q[i7] * q[i20])
            + (-1.967779e-03) * (q[i7] * q[i21]) + (1.176254e-03) * (q[i7] * q[i22]) + (1.505537e-03) * (q[i8] * q[i9]) + (7.206284e-04) * (q[i8] * q[i10])
            + (5.213333e-07) * (q[i8] * q[i12]) + (2.826673e-03) * (q[i8] * q[i15]) + (1.193063e-03) * (q[i8] * q[i16]) + (-1.131070e-03) * (q[i8] * q[i19])
            + (4.758059e-04) * (q[i8] * q[i20]) + (-3.066098e-04) * (q[i8] * q[i21]) + (-1.142485e-03) * (q[i8] * q[i22]) + (-3.432840e-04) * (q[i9] * q[i10])
            + (-1.383689e-03) * (q[i9] * q[i12]) + (4.185205e-04) * (q[i9] * q[i15]) + (6.748590e-05) * (q[i9] * q[i16]) + (2.570354e-04) * (q[i9] * q[i19])
            + (-2.698155e-04) * (q[i9] * q[i20]) + (5.498505e-04) * (q[i9] * q[i21]) + (5.739907e-04) * (q[i9] * q[i22]) + (1.351670e-03) * (q[i10] * q[i12])
            + (8.191657e-04) * (q[i10] * q[i15]) + (-4.993969e-04) * (q[i10] * q[i16]) + (-6.430745e-06) * (q[i10] * q[i19])
            + (9.173832e-05) * (q[i10] * q[i20]) + (2.451832e-04) * (q[i10] * q[i21]) + (7.502987e-04) * (q[i10] * q[i22]) + (-3.824057e-04) * (q[i12] * q[i15])
            + (3.722676e-04) * (q[i12] * q[i16]) + (-5.943896e-04) * (q[i12] * q[i19]) + (6.070909e-04) * (q[i12] * q[i20]) + (1.004132e-04) * (q[i12] * q[i21])
            + (1.110204e-04) * (q[i12] * q[i22]) + (7.387768e-04) * (q[i15] * q[i16]) + (8.488030e-04) * (q[i15] * q[i19]) + (-1.645095e-04) * (q[i15] * q[i20])
            + (1.516783e-03) * (q[i15] * q[i21]) + (-1.931456e-04) * (q[i15] * q[i22]) + (5.898340e-04) * (q[i16] * q[i19])
            + (-9.083006e-04) * (q[i16] * q[i20]) + (1.304451e-04) * (q[i16] * q[i21]) + (-9.794195e-05) * (q[i16] * q[i22])
            + (3.248382e-04) * (q[i19] * q[i20]) + (1.886006e-03) * (q[i19] * q[i21]) + (7.798466e-04) * (q[i19] * q[i22]) + (3.111280e-04) * (q[i20] * q[i21])
            + (-3.240401e-04) * (q[i20] * q[i22]) + (2.629995e-04) * (q[i21] * q[i22]);
   }

   public void getJQz12(double[] q, double[][] JQ)
   {
      JQ[3][i12] = (5.017613e-02) * (1) + (1.148862e-03) * ((2) * q[i12]) + (-3.051713e-04) * (q[i0]) + (-5.109633e-04) * (q[i1]) + (-1.752413e-03) * (q[i2])
            + (-2.040305e-03) * (q[i3]) + (1.588879e-03) * (q[i4]) + (3.888892e-03) * (q[i5]) + (1.560899e-03) * (q[i6]) + (9.740498e-04) * (q[i7])
            + (-3.862373e-03) * (q[i8]) + (1.497003e-03) * (q[i9]) + (-1.588390e-03) * (q[i10]) + (2.681928e-06) * (q[i11]) + (2.559983e-03) * (q[i15])
            + (2.497307e-03) * (q[i16]) + (3.930770e-04) * (q[i19]) + (4.151972e-03) * (q[i20]) + (-4.494994e-04) * (q[i21]) + (-4.286199e-03) * (q[i22])
            + (2.519247e-03) * (q[i0] * q[i0]) + (-6.138320e-05) * (q[i1] * q[i1]) + (-1.950016e-03) * (q[i2] * q[i2]) + (-3.467011e-04) * (q[i3] * q[i3])
            + (3.047201e-03) * (q[i4] * q[i4]) + (3.401737e-03) * (q[i5] * q[i5]) + (1.484473e-03) * (q[i6] * q[i6]) + (-4.665546e-03) * (q[i7] * q[i7])
            + (-5.461197e-03) * (q[i8] * q[i8]) + (-3.654373e-04) * (q[i9] * q[i9]) + (7.796672e-04) * (q[i10] * q[i10]) + (5.829736e-04) * (q[i11] * q[i11])
            + (-4.926891e-06) * ((2) * q[i0] * q[i12]) + (-3.530106e-04) * ((2) * q[i1] * q[i12]) + (-2.230185e-03) * ((2) * q[i2] * q[i12])
            + (-2.886119e-03) * ((2) * q[i3] * q[i12]) + (-3.361134e-04) * ((2) * q[i4] * q[i12]) + (-4.418924e-04) * ((2) * q[i5] * q[i12])
            + (2.741810e-03) * ((2) * q[i6] * q[i12]) + (-1.990822e-03) * ((2) * q[i7] * q[i12]) + (2.620844e-03) * ((2) * q[i8] * q[i12])
            + (8.229866e-04) * ((2) * q[i9] * q[i12]) + (-1.848996e-05) * ((2) * q[i10] * q[i12]) + (5.870360e-04) * ((2) * q[i11] * q[i12])
            + (-8.365510e-04) * ((3) * q[i12] * q[i12]) + (5.440826e-04) * ((2) * q[i12] * q[i15]) + (-1.440262e-03) * ((2) * q[i12] * q[i16])
            + (4.530601e-06) * ((2) * q[i12] * q[i19]) + (1.180369e-03) * ((2) * q[i12] * q[i20]) + (2.075158e-04) * ((2) * q[i12] * q[i21])
            + (-1.071907e-03) * ((2) * q[i12] * q[i22]) + (4.680595e-04) * (q[i15] * q[i15]) + (-5.938069e-03) * (q[i16] * q[i16])
            + (-1.264439e-04) * (q[i19] * q[i19]) + (6.084688e-04) * (q[i20] * q[i20]) + (-4.094983e-04) * (q[i21] * q[i21])
            + (-5.864783e-04) * (q[i22] * q[i22]) + (-6.819984e-04) * (q[i0] * q[i1]) + (-8.160733e-04) * (q[i0] * q[i2]) + (-4.130384e-03) * (q[i0] * q[i3])
            + (3.386984e-03) * (q[i0] * q[i4]) + (1.282937e-03) * (q[i0] * q[i5]) + (1.859035e-03) * (q[i0] * q[i6]) + (-6.621534e-04) * (q[i0] * q[i7])
            + (3.196186e-04) * (q[i0] * q[i8]) + (-4.151017e-04) * (q[i0] * q[i9]) + (1.398713e-03) * (q[i0] * q[i10]) + (5.165199e-04) * (q[i0] * q[i11])
            + (1.106490e-03) * (q[i0] * q[i15]) + (-3.871843e-03) * (q[i0] * q[i16]) + (6.751019e-04) * (q[i0] * q[i19]) + (2.247609e-03) * (q[i0] * q[i20])
            + (1.695129e-03) * (q[i0] * q[i21]) + (6.040034e-04) * (q[i0] * q[i22]) + (5.137758e-04) * (q[i1] * q[i2]) + (-5.158804e-04) * (q[i1] * q[i3])
            + (-1.935043e-03) * (q[i1] * q[i4]) + (1.235463e-03) * (q[i1] * q[i5]) + (8.925012e-04) * (q[i1] * q[i6]) + (-2.364231e-03) * (q[i1] * q[i7])
            + (1.257303e-04) * (q[i1] * q[i8]) + (1.418468e-03) * (q[i1] * q[i9]) + (-1.472401e-05) * (q[i1] * q[i10]) + (5.198115e-04) * (q[i1] * q[i11])
            + (2.860590e-04) * (q[i1] * q[i15]) + (1.377429e-03) * (q[i1] * q[i16]) + (1.575255e-03) * (q[i1] * q[i19]) + (-1.639693e-03) * (q[i1] * q[i20])
            + (-1.023231e-03) * (q[i1] * q[i21]) + (-8.373231e-04) * (q[i1] * q[i22]) + (-3.162736e-03) * (q[i2] * q[i3]) + (1.235821e-03) * (q[i2] * q[i4])
            + (-1.137662e-03) * (q[i2] * q[i5]) + (3.404679e-03) * (q[i2] * q[i6]) + (-2.075903e-03) * (q[i2] * q[i7]) + (-1.014369e-04) * (q[i2] * q[i8])
            + (7.825747e-04) * (q[i2] * q[i9]) + (1.535351e-03) * (q[i2] * q[i10]) + (2.987426e-03) * (q[i2] * q[i11]) + (1.115856e-03) * (q[i2] * q[i15])
            + (-2.369277e-03) * (q[i2] * q[i16]) + (-6.144859e-04) * (q[i2] * q[i19]) + (7.187429e-04) * (q[i2] * q[i20]) + (-6.115261e-04) * (q[i2] * q[i21])
            + (-8.317397e-04) * (q[i2] * q[i22]) + (2.118888e-04) * (q[i3] * q[i4]) + (-4.053934e-03) * (q[i3] * q[i5]) + (5.265804e-03) * (q[i3] * q[i6])
            + (-3.673652e-03) * (q[i3] * q[i7]) + (2.354166e-03) * (q[i3] * q[i8]) + (-6.590604e-04) * (q[i3] * q[i9]) + (2.924787e-04) * (q[i3] * q[i10])
            + (4.219337e-04) * (q[i3] * q[i11]) + (-4.134908e-04) * (q[i3] * q[i15]) + (-4.669852e-04) * (q[i3] * q[i16]) + (-4.828984e-03) * (q[i3] * q[i19])
            + (-2.117439e-03) * (q[i3] * q[i20]) + (-5.185118e-04) * (q[i3] * q[i21]) + (9.435594e-05) * (q[i3] * q[i22]) + (1.697960e-04) * (q[i4] * q[i5])
            + (-5.365699e-03) * (q[i4] * q[i6]) + (3.306854e-03) * (q[i4] * q[i7]) + (-2.759669e-03) * (q[i4] * q[i8]) + (-7.626711e-04) * (q[i4] * q[i9])
            + (1.201299e-03) * (q[i4] * q[i10]) + (4.096850e-04) * (q[i4] * q[i11]) + (-4.336719e-04) * (q[i4] * q[i15]) + (1.438830e-03) * (q[i4] * q[i16])
            + (2.819507e-03) * (q[i4] * q[i19]) + (1.160681e-03) * (q[i4] * q[i20]) + (-7.572880e-04) * (q[i4] * q[i21]) + (-1.530939e-03) * (q[i4] * q[i22])
            + (-1.625085e-03) * (q[i5] * q[i6]) + (1.139513e-03) * (q[i5] * q[i7]) + (5.177861e-03) * (q[i5] * q[i8]) + (3.316518e-04) * (q[i5] * q[i9])
            + (4.846041e-04) * (q[i5] * q[i10]) + (-5.596421e-05) * (q[i5] * q[i11]) + (-1.202181e-03) * (q[i5] * q[i15]) + (3.666847e-03) * (q[i5] * q[i16])
            + (-9.107160e-04) * (q[i5] * q[i19]) + (7.582725e-04) * (q[i5] * q[i20]) + (7.650291e-04) * (q[i5] * q[i21]) + (-2.058770e-04) * (q[i5] * q[i22])
            + (7.360295e-04) * (q[i6] * q[i7]) + (-6.499768e-04) * (q[i6] * q[i8]) + (-1.665129e-03) * (q[i6] * q[i9]) + (-3.319301e-03) * (q[i6] * q[i10])
            + (-6.797917e-03) * (q[i6] * q[i11]) + (-3.450668e-03) * (q[i6] * q[i15]) + (9.277737e-04) * (q[i6] * q[i16]) + (-3.422618e-04) * (q[i6] * q[i19])
            + (-1.686519e-03) * (q[i6] * q[i20]) + (-1.191898e-03) * (q[i6] * q[i21]) + (1.967394e-03) * (q[i6] * q[i22]) + (2.296906e-03) * (q[i7] * q[i8])
            + (1.092796e-03) * (q[i7] * q[i9]) + (9.381162e-04) * (q[i7] * q[i10]) + (6.788398e-03) * (q[i7] * q[i11]) + (2.947093e-03) * (q[i7] * q[i15])
            + (-3.053283e-05) * (q[i7] * q[i16]) + (-7.212440e-04) * (q[i7] * q[i19]) + (1.663596e-03) * (q[i7] * q[i20]) + (-1.454366e-03) * (q[i7] * q[i21])
            + (-1.900401e-03) * (q[i7] * q[i22]) + (7.128823e-04) * (q[i8] * q[i9]) + (1.499905e-03) * (q[i8] * q[i10]) + (5.213333e-07) * (q[i8] * q[i11])
            + (1.212328e-03) * (q[i8] * q[i15]) + (2.828037e-03) * (q[i8] * q[i16]) + (4.809914e-04) * (q[i8] * q[i19]) + (-1.072583e-03) * (q[i8] * q[i20])
            + (1.149213e-03) * (q[i8] * q[i21]) + (3.046039e-04) * (q[i8] * q[i22]) + (-3.509618e-04) * (q[i9] * q[i10]) + (-1.383689e-03) * (q[i9] * q[i11])
            + (-5.104271e-04) * (q[i9] * q[i15]) + (8.176412e-04) * (q[i9] * q[i16]) + (8.131269e-05) * (q[i9] * q[i19]) + (-9.948959e-06) * (q[i9] * q[i20])
            + (-7.537221e-04) * (q[i9] * q[i21]) + (-2.366834e-04) * (q[i9] * q[i22]) + (1.351670e-03) * (q[i10] * q[i11]) + (6.865590e-05) * (q[i10] * q[i15])
            + (4.180841e-04) * (q[i10] * q[i16]) + (-2.767815e-04) * (q[i10] * q[i19]) + (2.536174e-04) * (q[i10] * q[i20])
            + (-5.718205e-04) * (q[i10] * q[i21]) + (-5.451014e-04) * (q[i10] * q[i22]) + (-3.824057e-04) * (q[i11] * q[i15])
            + (3.722676e-04) * (q[i11] * q[i16]) + (-5.943896e-04) * (q[i11] * q[i19]) + (6.070909e-04) * (q[i11] * q[i20]) + (1.004132e-04) * (q[i11] * q[i21])
            + (1.110204e-04) * (q[i11] * q[i22]) + (7.343239e-04) * (q[i15] * q[i16]) + (-9.047756e-04) * (q[i15] * q[i19]) + (5.885397e-04) * (q[i15] * q[i20])
            + (9.203270e-05) * (q[i15] * q[i21]) + (-1.425435e-04) * (q[i15] * q[i22]) + (-1.627775e-04) * (q[i16] * q[i19])
            + (8.694204e-04) * (q[i16] * q[i20]) + (1.930178e-04) * (q[i16] * q[i21]) + (-1.559937e-03) * (q[i16] * q[i22]) + (3.196399e-04) * (q[i19] * q[i20])
            + (3.213397e-04) * (q[i19] * q[i21]) + (-3.151055e-04) * (q[i19] * q[i22]) + (-7.816191e-04) * (q[i20] * q[i21])
            + (-1.866132e-03) * (q[i20] * q[i22]) + (2.602461e-04) * (q[i21] * q[i22]);
   }

   public void getJQz15(double[] q, double[][] JQ)
   {
      JQ[3][i15] = (1.203367e-02) * (1) + (4.929370e-03) * ((2) * q[i15]) + (-8.068139e-03) * (q[i0]) + (-6.918457e-03) * (q[i1]) + (-1.796744e-02) * (q[i2])
            + (7.821852e-03) * (q[i3]) + (-2.693480e-04) * (q[i4]) + (-1.199885e-03) * (q[i5]) + (2.112994e-03) * (q[i6]) + (-1.104024e-02) * (q[i7])
            + (1.443567e-02) * (q[i8]) + (-2.044194e-03) * (q[i9]) + (-1.900916e-03) * (q[i10]) + (2.564447e-03) * (q[i11]) + (2.559983e-03) * (q[i12])
            + (6.650289e-06) * (q[i16]) + (4.703209e-03) * (q[i19]) + (-1.427964e-03) * (q[i20]) + (4.141272e-03) * (q[i21]) + (3.035838e-04) * (q[i22])
            + (6.291848e-04) * (q[i0] * q[i0]) + (3.195453e-05) * (q[i1] * q[i1]) + (-1.460510e-03) * (q[i2] * q[i2]) + (-6.010023e-03) * (q[i3] * q[i3])
            + (2.749408e-03) * (q[i4] * q[i4]) + (9.894973e-04) * (q[i5] * q[i5]) + (2.561257e-03) * (q[i6] * q[i6]) + (1.562721e-03) * (q[i7] * q[i7])
            + (-1.147863e-03) * (q[i8] * q[i8]) + (-1.776470e-04) * (q[i9] * q[i9]) + (8.040850e-04) * (q[i10] * q[i10]) + (1.444886e-03) * (q[i11] * q[i11])
            + (5.440826e-04) * (q[i12] * q[i12]) + (-1.610159e-03) * ((2) * q[i0] * q[i15]) + (-2.590570e-03) * ((2) * q[i1] * q[i15])
            + (-4.875913e-03) * ((2) * q[i2] * q[i15]) + (-1.798421e-03) * ((2) * q[i3] * q[i15]) + (1.108226e-03) * ((2) * q[i4] * q[i15])
            + (-1.275408e-03) * ((2) * q[i5] * q[i15]) + (1.350029e-03) * ((2) * q[i6] * q[i15]) + (-1.718359e-03) * ((2) * q[i7] * q[i15])
            + (-1.623978e-03) * ((2) * q[i8] * q[i15]) + (-9.476405e-05) * ((2) * q[i9] * q[i15]) + (4.446822e-04) * ((2) * q[i10] * q[i15])
            + (-5.868728e-03) * ((2) * q[i11] * q[i15]) + (4.680595e-04) * ((2) * q[i12] * q[i15]) + (-2.844297e-04) * ((3) * q[i15] * q[i15])
            + (-1.891718e-04) * ((2) * q[i15] * q[i16]) + (8.950920e-04) * ((2) * q[i15] * q[i19]) + (-3.238596e-05) * ((2) * q[i15] * q[i20])
            + (1.447443e-03) * ((2) * q[i15] * q[i21]) + (-2.898229e-04) * ((2) * q[i15] * q[i22]) + (1.895777e-04) * (q[i16] * q[i16])
            + (3.317757e-04) * (q[i19] * q[i19]) + (1.876722e-04) * (q[i20] * q[i20]) + (2.459007e-04) * (q[i21] * q[i21]) + (1.223636e-04) * (q[i22] * q[i22])
            + (8.307751e-05) * (q[i0] * q[i1]) + (-8.816348e-04) * (q[i0] * q[i2]) + (3.737213e-03) * (q[i0] * q[i3]) + (-3.231341e-03) * (q[i0] * q[i4])
            + (3.452045e-03) * (q[i0] * q[i5]) + (-3.302023e-04) * (q[i0] * q[i6]) + (1.730123e-03) * (q[i0] * q[i7]) + (-1.777304e-03) * (q[i0] * q[i8])
            + (1.080065e-04) * (q[i0] * q[i9]) + (-1.365183e-03) * (q[i0] * q[i10]) + (-1.382556e-03) * (q[i0] * q[i11]) + (1.106490e-03) * (q[i0] * q[i12])
            + (8.868801e-05) * (q[i0] * q[i16]) + (1.068404e-03) * (q[i0] * q[i19]) + (-6.770629e-05) * (q[i0] * q[i20]) + (-2.242571e-03) * (q[i0] * q[i21])
            + (2.860531e-04) * (q[i0] * q[i22]) + (1.617850e-03) * (q[i1] * q[i2]) + (1.375503e-04) * (q[i1] * q[i3]) + (4.452672e-03) * (q[i1] * q[i4])
            + (2.119109e-03) * (q[i1] * q[i5]) + (-2.981604e-03) * (q[i1] * q[i6]) + (2.516036e-03) * (q[i1] * q[i7]) + (-1.220725e-03) * (q[i1] * q[i8])
            + (-2.835336e-04) * (q[i1] * q[i9]) + (-8.608620e-04) * (q[i1] * q[i10]) + (3.872995e-03) * (q[i1] * q[i11]) + (2.860590e-04) * (q[i1] * q[i12])
            + (8.015678e-05) * (q[i1] * q[i16]) + (-9.882359e-04) * (q[i1] * q[i19]) + (1.207770e-03) * (q[i1] * q[i20]) + (-2.136392e-04) * (q[i1] * q[i21])
            + (-6.772398e-05) * (q[i1] * q[i22]) + (7.214720e-06) * (q[i2] * q[i3]) + (-6.401135e-04) * (q[i2] * q[i4]) + (7.764108e-03) * (q[i2] * q[i5])
            + (-2.437519e-03) * (q[i2] * q[i6]) + (1.418427e-03) * (q[i2] * q[i7]) + (-6.401808e-04) * (q[i2] * q[i8]) + (1.132058e-03) * (q[i2] * q[i9])
            + (-1.879207e-03) * (q[i2] * q[i10]) + (2.351462e-03) * (q[i2] * q[i11]) + (1.115856e-03) * (q[i2] * q[i12]) + (5.453522e-04) * (q[i2] * q[i16])
            + (5.335550e-04) * (q[i2] * q[i19]) + (9.489542e-04) * (q[i2] * q[i20]) + (-1.693832e-03) * (q[i2] * q[i21]) + (-3.798725e-04) * (q[i2] * q[i22])
            + (1.928381e-03) * (q[i3] * q[i4]) + (-8.369577e-04) * (q[i3] * q[i5]) + (4.697587e-03) * (q[i3] * q[i6]) + (3.929025e-03) * (q[i3] * q[i7])
            + (-8.223876e-04) * (q[i3] * q[i8]) + (-2.520779e-04) * (q[i3] * q[i9]) + (-4.046427e-05) * (q[i3] * q[i10]) + (-1.423635e-03) * (q[i3] * q[i11])
            + (-4.134908e-04) * (q[i3] * q[i12]) + (2.937128e-04) * (q[i3] * q[i16]) + (-1.359044e-03) * (q[i3] * q[i19]) + (1.892597e-03) * (q[i3] * q[i20])
            + (2.977472e-04) * (q[i3] * q[i21]) + (1.001613e-03) * (q[i3] * q[i22]) + (-9.759214e-04) * (q[i4] * q[i5]) + (-4.621407e-03) * (q[i4] * q[i6])
            + (1.152578e-03) * (q[i4] * q[i7]) + (1.167909e-05) * (q[i4] * q[i8]) + (-2.307338e-03) * (q[i4] * q[i9]) + (1.070567e-03) * (q[i4] * q[i10])
            + (4.705882e-04) * (q[i4] * q[i11]) + (-4.336719e-04) * (q[i4] * q[i12]) + (3.093467e-04) * (q[i4] * q[i16]) + (3.023984e-03) * (q[i4] * q[i19])
            + (-2.037393e-03) * (q[i4] * q[i20]) + (-1.102679e-03) * (q[i4] * q[i21]) + (-9.009462e-04) * (q[i4] * q[i22]) + (-3.531705e-03) * (q[i5] * q[i6])
            + (-2.396054e-03) * (q[i5] * q[i7]) + (-3.516862e-03) * (q[i5] * q[i8]) + (6.361504e-04) * (q[i5] * q[i9]) + (-7.985036e-04) * (q[i5] * q[i10])
            + (-3.651541e-03) * (q[i5] * q[i11]) + (-1.202181e-03) * (q[i5] * q[i12]) + (1.537212e-03) * (q[i5] * q[i16]) + (7.910750e-04) * (q[i5] * q[i19])
            + (3.173976e-05) * (q[i5] * q[i20]) + (-1.798743e-04) * (q[i5] * q[i21]) + (1.713108e-04) * (q[i5] * q[i22]) + (-4.667196e-03) * (q[i6] * q[i7])
            + (-1.649729e-03) * (q[i6] * q[i8]) + (-4.085242e-04) * (q[i6] * q[i9]) + (-3.119390e-03) * (q[i6] * q[i10]) + (-2.988266e-05) * (q[i6] * q[i11])
            + (-3.450668e-03) * (q[i6] * q[i12]) + (-3.686725e-04) * (q[i6] * q[i16]) + (-1.345635e-03) * (q[i6] * q[i19]) + (-1.620104e-03) * (q[i6] * q[i20])
            + (-1.247835e-03) * (q[i6] * q[i21]) + (9.407677e-04) * (q[i6] * q[i22]) + (2.943766e-03) * (q[i7] * q[i8]) + (-8.232981e-04) * (q[i7] * q[i9])
            + (1.665228e-03) * (q[i7] * q[i10]) + (9.438830e-04) * (q[i7] * q[i11]) + (2.947093e-03) * (q[i7] * q[i12]) + (3.657008e-04) * (q[i7] * q[i16])
            + (-5.565482e-04) * (q[i7] * q[i19]) + (3.309427e-03) * (q[i7] * q[i20]) + (-8.648373e-06) * (q[i7] * q[i21]) + (5.829073e-04) * (q[i7] * q[i22])
            + (-2.079523e-03) * (q[i8] * q[i9]) + (2.132471e-03) * (q[i8] * q[i10]) + (2.826673e-03) * (q[i8] * q[i11]) + (1.212328e-03) * (q[i8] * q[i12])
            + (7.098969e-07) * (q[i8] * q[i16]) + (1.069179e-03) * (q[i8] * q[i19]) + (-7.574531e-04) * (q[i8] * q[i20]) + (2.629756e-03) * (q[i8] * q[i21])
            + (-3.028679e-04) * (q[i8] * q[i22]) + (-7.261380e-04) * (q[i9] * q[i10]) + (4.185205e-04) * (q[i9] * q[i11]) + (-5.104271e-04) * (q[i9] * q[i12])
            + (-6.006669e-04) * (q[i9] * q[i16]) + (-1.487320e-04) * (q[i9] * q[i19]) + (-2.653334e-05) * (q[i9] * q[i20]) + (-6.297149e-04) * (q[i9] * q[i21])
            + (-3.685585e-05) * (q[i9] * q[i22]) + (8.191657e-04) * (q[i10] * q[i11]) + (6.865590e-05) * (q[i10] * q[i12]) + (6.058846e-04) * (q[i10] * q[i16])
            + (-4.135366e-04) * (q[i10] * q[i19]) + (9.230676e-04) * (q[i10] * q[i20]) + (2.476633e-05) * (q[i10] * q[i21]) + (2.225760e-05) * (q[i10] * q[i22])
            + (-3.824057e-04) * (q[i11] * q[i12]) + (7.387768e-04) * (q[i11] * q[i16]) + (8.488030e-04) * (q[i11] * q[i19])
            + (-1.645095e-04) * (q[i11] * q[i20]) + (1.516783e-03) * (q[i11] * q[i21]) + (-1.931456e-04) * (q[i11] * q[i22])
            + (7.343239e-04) * (q[i12] * q[i16]) + (-9.047756e-04) * (q[i12] * q[i19]) + (5.885397e-04) * (q[i12] * q[i20]) + (9.203270e-05) * (q[i12] * q[i21])
            + (-1.425435e-04) * (q[i12] * q[i22]) + (-7.513823e-05) * (q[i16] * q[i19]) + (7.803911e-05) * (q[i16] * q[i20])
            + (-2.143389e-04) * (q[i16] * q[i21]) + (-2.106631e-04) * (q[i16] * q[i22]) + (-5.097665e-04) * (q[i19] * q[i20])
            + (1.186223e-04) * (q[i19] * q[i21]) + (-1.350526e-06) * (q[i19] * q[i22]) + (-2.515901e-04) * (q[i20] * q[i21])
            + (6.296997e-05) * (q[i20] * q[i22]) + (1.336803e-04) * (q[i21] * q[i22]);
   }

   public void getJQz16(double[] q, double[][] JQ)
   {
      JQ[3][i16] = (-1.233942e-02) * (1) + (-4.954336e-03) * ((2) * q[i16]) + (-7.002062e-03) * (q[i0]) + (-8.223930e-03) * (q[i1]) + (-1.816952e-02) * (q[i2])
            + (-2.114044e-04) * (q[i3]) + (7.795655e-03) * (q[i4]) + (-1.234830e-03) * (q[i5]) + (1.119554e-02) * (q[i6]) + (-2.191571e-03) * (q[i7])
            + (-1.453026e-02) * (q[i8]) + (1.961958e-03) * (q[i9]) + (2.036227e-03) * (q[i10]) + (2.571983e-03) * (q[i11]) + (2.497307e-03) * (q[i12])
            + (6.650289e-06) * (q[i15]) + (1.448494e-03) * (q[i19]) + (-4.613598e-03) * (q[i20]) + (3.120678e-04) * (q[i21]) + (4.203977e-03) * (q[i22])
            + (-1.373595e-06) * (q[i0] * q[i0]) + (-6.272335e-04) * (q[i1] * q[i1]) + (1.450669e-03) * (q[i2] * q[i2]) + (-2.747068e-03) * (q[i3] * q[i3])
            + (6.002643e-03) * (q[i4] * q[i4]) + (-9.961190e-04) * (q[i5] * q[i5]) + (-1.554538e-03) * (q[i6] * q[i6]) + (-2.543950e-03) * (q[i7] * q[i7])
            + (1.224891e-03) * (q[i8] * q[i8]) + (-8.181243e-04) * (q[i9] * q[i9]) + (1.798451e-04) * (q[i10] * q[i10]) + (-5.429432e-04) * (q[i11] * q[i11])
            + (-1.440262e-03) * (q[i12] * q[i12]) + (-1.891718e-04) * (q[i15] * q[i15]) + (-2.629202e-03) * ((2) * q[i0] * q[i16])
            + (-1.635661e-03) * ((2) * q[i1] * q[i16]) + (-4.926560e-03) * ((2) * q[i2] * q[i16]) + (1.128739e-03) * ((2) * q[i3] * q[i16])
            + (-1.787982e-03) * ((2) * q[i4] * q[i16]) + (-1.308459e-03) * ((2) * q[i5] * q[i16]) + (1.732938e-03) * ((2) * q[i6] * q[i16])
            + (-1.365205e-03) * ((2) * q[i7] * q[i16]) + (1.643932e-03) * ((2) * q[i8] * q[i16]) + (-4.486444e-04) * ((2) * q[i9] * q[i16])
            + (9.360250e-05) * ((2) * q[i10] * q[i16]) + (4.659210e-04) * ((2) * q[i11] * q[i16]) + (-5.938069e-03) * ((2) * q[i12] * q[i16])
            + (1.895777e-04) * ((2) * q[i15] * q[i16]) + (2.887173e-04) * ((3) * q[i16] * q[i16]) + (3.629187e-05) * ((2) * q[i16] * q[i19])
            + (-9.031755e-04) * ((2) * q[i16] * q[i20]) + (-2.888105e-04) * ((2) * q[i16] * q[i21]) + (1.464243e-03) * ((2) * q[i16] * q[i22])
            + (-1.910473e-04) * (q[i19] * q[i19]) + (-3.153356e-04) * (q[i20] * q[i20]) + (-1.195869e-04) * (q[i21] * q[i21])
            + (-2.405727e-04) * (q[i22] * q[i22]) + (-8.990329e-05) * (q[i0] * q[i1]) + (-1.612249e-03) * (q[i0] * q[i2]) + (-4.549942e-03) * (q[i0] * q[i3])
            + (-1.271616e-04) * (q[i0] * q[i4]) + (-2.146369e-03) * (q[i0] * q[i5]) + (2.531603e-03) * (q[i0] * q[i6]) + (-2.987341e-03) * (q[i0] * q[i7])
            + (-1.266048e-03) * (q[i0] * q[i8]) + (-8.717527e-04) * (q[i0] * q[i9]) + (-2.779088e-04) * (q[i0] * q[i10]) + (-2.685371e-04) * (q[i0] * q[i11])
            + (-3.871843e-03) * (q[i0] * q[i12]) + (8.868801e-05) * (q[i0] * q[i15]) + (1.225508e-03) * (q[i0] * q[i19]) + (-9.726985e-04) * (q[i0] * q[i20])
            + (8.158969e-05) * (q[i0] * q[i21]) + (2.233752e-04) * (q[i0] * q[i22]) + (8.778651e-04) * (q[i1] * q[i2]) + (3.280968e-03) * (q[i1] * q[i3])
            + (-3.699263e-03) * (q[i1] * q[i4]) + (-3.452865e-03) * (q[i1] * q[i5]) + (1.727913e-03) * (q[i1] * q[i6]) + (-2.919010e-04) * (q[i1] * q[i7])
            + (-1.803216e-03) * (q[i1] * q[i8]) + (-1.345912e-03) * (q[i1] * q[i9]) + (1.288937e-04) * (q[i1] * q[i10]) + (-1.095867e-03) * (q[i1] * q[i11])
            + (1.377429e-03) * (q[i1] * q[i12]) + (8.015678e-05) * (q[i1] * q[i15]) + (-6.929032e-05) * (q[i1] * q[i19]) + (1.071461e-03) * (q[i1] * q[i20])
            + (-3.002668e-04) * (q[i1] * q[i21]) + (2.228063e-03) * (q[i1] * q[i22]) + (6.397862e-04) * (q[i2] * q[i3]) + (-3.348564e-05) * (q[i2] * q[i4])
            + (-7.806409e-03) * (q[i2] * q[i5]) + (1.432669e-03) * (q[i2] * q[i6]) + (-2.459923e-03) * (q[i2] * q[i7]) + (-6.618732e-04) * (q[i2] * q[i8])
            + (-1.885767e-03) * (q[i2] * q[i9]) + (1.131607e-03) * (q[i2] * q[i10]) + (-1.103160e-03) * (q[i2] * q[i11]) + (-2.369277e-03) * (q[i2] * q[i12])
            + (5.453522e-04) * (q[i2] * q[i15]) + (9.717815e-04) * (q[i2] * q[i19]) + (5.450892e-04) * (q[i2] * q[i20]) + (3.833704e-04) * (q[i2] * q[i21])
            + (1.684649e-03) * (q[i2] * q[i22]) + (-1.965659e-03) * (q[i3] * q[i4]) + (1.036362e-03) * (q[i3] * q[i5]) + (1.186916e-03) * (q[i3] * q[i6])
            + (-4.584537e-03) * (q[i3] * q[i7]) + (3.525317e-06) * (q[i3] * q[i8]) + (1.079600e-03) * (q[i3] * q[i9]) + (-2.314218e-03) * (q[i3] * q[i10])
            + (4.422790e-04) * (q[i3] * q[i11]) + (-4.669852e-04) * (q[i3] * q[i12]) + (2.937128e-04) * (q[i3] * q[i15]) + (-2.043554e-03) * (q[i3] * q[i19])
            + (2.998594e-03) * (q[i3] * q[i20]) + (9.347620e-04) * (q[i3] * q[i21]) + (1.107827e-03) * (q[i3] * q[i22]) + (8.420715e-04) * (q[i4] * q[i5])
            + (3.924413e-03) * (q[i4] * q[i6]) + (4.677970e-03) * (q[i4] * q[i7]) + (-8.594415e-04) * (q[i4] * q[i8]) + (-3.622675e-05) * (q[i4] * q[i9])
            + (-2.303241e-04) * (q[i4] * q[i10]) + (4.229579e-04) * (q[i4] * q[i11]) + (1.438830e-03) * (q[i4] * q[i12]) + (3.093467e-04) * (q[i4] * q[i15])
            + (1.869637e-03) * (q[i4] * q[i19]) + (-1.350580e-03) * (q[i4] * q[i20]) + (-1.002016e-03) * (q[i4] * q[i21]) + (-2.786692e-04) * (q[i4] * q[i22])
            + (-2.354698e-03) * (q[i5] * q[i6]) + (-3.510798e-03) * (q[i5] * q[i7]) + (-3.532130e-03) * (q[i5] * q[i8]) + (-8.130769e-04) * (q[i5] * q[i9])
            + (6.463539e-04) * (q[i5] * q[i10]) + (1.199525e-03) * (q[i5] * q[i11]) + (3.666847e-03) * (q[i5] * q[i12]) + (1.537212e-03) * (q[i5] * q[i15])
            + (5.531803e-05) * (q[i5] * q[i19]) + (8.106571e-04) * (q[i5] * q[i20]) + (-1.722371e-04) * (q[i5] * q[i21]) + (1.665068e-04) * (q[i5] * q[i22])
            + (4.670192e-03) * (q[i6] * q[i7]) + (-2.953595e-03) * (q[i6] * q[i8]) + (-1.681619e-03) * (q[i6] * q[i9]) + (8.119270e-04) * (q[i6] * q[i10])
            + (2.932153e-03) * (q[i6] * q[i11]) + (9.277737e-04) * (q[i6] * q[i12]) + (-3.686725e-04) * (q[i6] * q[i15]) + (-3.332313e-03) * (q[i6] * q[i19])
            + (5.609957e-04) * (q[i6] * q[i20]) + (5.767641e-04) * (q[i6] * q[i21]) + (-1.211639e-05) * (q[i6] * q[i22]) + (1.613064e-03) * (q[i7] * q[i8])
            + (3.118214e-03) * (q[i7] * q[i9]) + (4.276289e-04) * (q[i7] * q[i10]) + (-3.429429e-03) * (q[i7] * q[i11]) + (-3.053283e-05) * (q[i7] * q[i12])
            + (3.657008e-04) * (q[i7] * q[i15]) + (1.611454e-03) * (q[i7] * q[i19]) + (1.343040e-03) * (q[i7] * q[i20]) + (9.408520e-04) * (q[i7] * q[i21])
            + (-1.249415e-03) * (q[i7] * q[i22]) + (-2.149540e-03) * (q[i8] * q[i9]) + (2.067358e-03) * (q[i8] * q[i10]) + (1.193063e-03) * (q[i8] * q[i11])
            + (2.828037e-03) * (q[i8] * q[i12]) + (7.098969e-07) * (q[i8] * q[i15]) + (7.779851e-04) * (q[i8] * q[i19]) + (-1.079222e-03) * (q[i8] * q[i20])
            + (-2.999376e-04) * (q[i8] * q[i21]) + (2.667583e-03) * (q[i8] * q[i22]) + (7.210693e-04) * (q[i9] * q[i10]) + (6.748590e-05) * (q[i9] * q[i11])
            + (8.176412e-04) * (q[i9] * q[i12]) + (-6.006669e-04) * (q[i9] * q[i15]) + (-9.418895e-04) * (q[i9] * q[i19]) + (4.169658e-04) * (q[i9] * q[i20])
            + (1.721087e-05) * (q[i9] * q[i21]) + (2.033018e-05) * (q[i9] * q[i22]) + (-4.993969e-04) * (q[i10] * q[i11]) + (4.180841e-04) * (q[i10] * q[i12])
            + (6.058846e-04) * (q[i10] * q[i15]) + (2.947857e-05) * (q[i10] * q[i19]) + (1.432046e-04) * (q[i10] * q[i20]) + (-4.318072e-05) * (q[i10] * q[i21])
            + (-6.301378e-04) * (q[i10] * q[i22]) + (3.722676e-04) * (q[i11] * q[i12]) + (7.387768e-04) * (q[i11] * q[i15]) + (5.898340e-04) * (q[i11] * q[i19])
            + (-9.083006e-04) * (q[i11] * q[i20]) + (1.304451e-04) * (q[i11] * q[i21]) + (-9.794195e-05) * (q[i11] * q[i22])
            + (7.343239e-04) * (q[i12] * q[i15]) + (-1.627775e-04) * (q[i12] * q[i19]) + (8.694204e-04) * (q[i12] * q[i20]) + (1.930178e-04) * (q[i12] * q[i21])
            + (-1.559937e-03) * (q[i12] * q[i22]) + (-7.513823e-05) * (q[i15] * q[i19]) + (7.803911e-05) * (q[i15] * q[i20])
            + (-2.143389e-04) * (q[i15] * q[i21]) + (-2.106631e-04) * (q[i15] * q[i22]) + (5.075237e-04) * (q[i19] * q[i20])
            + (6.470507e-05) * (q[i19] * q[i21]) + (-2.544436e-04) * (q[i19] * q[i22]) + (5.950866e-06) * (q[i20] * q[i21]) + (1.024102e-04) * (q[i20] * q[i22])
            + (-1.343905e-04) * (q[i21] * q[i22]);
   }

   public void getJQz19(double[] q, double[][] JQ)
   {
      JQ[3][i19] = (9.749392e-03) * (1) + (-1.456905e-03) * ((2) * q[i19]) + (-3.021311e-03) * (q[i0]) + (-1.285072e-05) * (q[i1]) + (2.002263e-03) * (q[i2])
            + (-1.242713e-02) * (q[i3]) + (1.143341e-02) * (q[i4]) + (5.678138e-03) * (q[i5]) + (-8.683908e-03) * (q[i6]) + (5.833061e-03) * (q[i7])
            + (4.252995e-03) * (q[i8]) + (-4.272644e-03) * (q[i9]) + (1.303528e-03) * (q[i10]) + (4.182586e-03) * (q[i11]) + (3.930770e-04) * (q[i12])
            + (4.703209e-03) * (q[i15]) + (1.448494e-03) * (q[i16]) + (7.985600e-06) * (q[i20]) + (-5.571742e-03) * (q[i21]) + (-7.097554e-04) * (q[i22])
            + (-6.879212e-05) * (q[i0] * q[i0]) + (-3.195466e-04) * (q[i1] * q[i1]) + (-1.100006e-03) * (q[i2] * q[i2]) + (8.414819e-04) * (q[i3] * q[i3])
            + (2.218625e-04) * (q[i4] * q[i4]) + (2.131419e-03) * (q[i5] * q[i5]) + (2.031403e-04) * (q[i6] * q[i6]) + (8.816005e-04) * (q[i7] * q[i7])
            + (-1.183642e-03) * (q[i8] * q[i8]) + (5.688618e-04) * (q[i9] * q[i9]) + (-5.029924e-04) * (q[i10] * q[i10]) + (-1.210977e-03) * (q[i11] * q[i11])
            + (4.530601e-06) * (q[i12] * q[i12]) + (8.950920e-04) * (q[i15] * q[i15]) + (3.629187e-05) * (q[i16] * q[i16])
            + (9.614484e-04) * ((2) * q[i0] * q[i19]) + (-8.908750e-04) * ((2) * q[i1] * q[i19]) + (-3.407559e-04) * ((2) * q[i2] * q[i19])
            + (-1.618046e-03) * ((2) * q[i3] * q[i19]) + (9.882835e-04) * ((2) * q[i4] * q[i19]) + (5.672238e-04) * ((2) * q[i5] * q[i19])
            + (1.277015e-03) * ((2) * q[i6] * q[i19]) + (-1.300125e-03) * ((2) * q[i7] * q[i19]) + (-2.297524e-04) * ((2) * q[i8] * q[i19])
            + (3.851531e-04) * ((2) * q[i9] * q[i19]) + (2.144442e-04) * ((2) * q[i10] * q[i19]) + (6.159593e-04) * ((2) * q[i11] * q[i19])
            + (-1.264439e-04) * ((2) * q[i12] * q[i19]) + (3.317757e-04) * ((2) * q[i15] * q[i19]) + (-1.910473e-04) * ((2) * q[i16] * q[i19])
            + (-1.966820e-04) * ((3) * q[i19] * q[i19]) + (-3.966958e-04) * ((2) * q[i19] * q[i20]) + (-2.099350e-03) * ((2) * q[i19] * q[i21])
            + (-1.815490e-05) * ((2) * q[i19] * q[i22]) + (3.836269e-04) * (q[i20] * q[i20]) + (-1.030272e-03) * (q[i21] * q[i21])
            + (-1.440290e-04) * (q[i22] * q[i22]) + (2.970379e-04) * (q[i0] * q[i1]) + (-1.778900e-03) * (q[i0] * q[i2]) + (-7.239892e-04) * (q[i0] * q[i3])
            + (-1.119306e-05) * (q[i0] * q[i4]) + (-1.380481e-03) * (q[i0] * q[i5]) + (-1.968210e-03) * (q[i0] * q[i6]) + (-2.717691e-03) * (q[i0] * q[i7])
            + (-9.278645e-04) * (q[i0] * q[i8]) + (1.619438e-03) * (q[i0] * q[i9]) + (1.148780e-03) * (q[i0] * q[i10]) + (1.617007e-03) * (q[i0] * q[i11])
            + (6.751019e-04) * (q[i0] * q[i12]) + (1.068404e-03) * (q[i0] * q[i15]) + (1.225508e-03) * (q[i0] * q[i16]) + (-1.029540e-04) * (q[i0] * q[i20])
            + (7.949083e-04) * (q[i0] * q[i21]) + (-6.577718e-04) * (q[i0] * q[i22]) + (-3.047581e-04) * (q[i1] * q[i2]) + (5.168160e-04) * (q[i1] * q[i3])
            + (4.557667e-04) * (q[i1] * q[i4]) + (5.274652e-04) * (q[i1] * q[i5]) + (-1.552690e-04) * (q[i1] * q[i6]) + (-7.084220e-04) * (q[i1] * q[i7])
            + (2.591051e-03) * (q[i1] * q[i8]) + (-6.331721e-04) * (q[i1] * q[i9]) + (-3.646995e-04) * (q[i1] * q[i10]) + (-2.231571e-03) * (q[i1] * q[i11])
            + (1.575255e-03) * (q[i1] * q[i12]) + (-9.882359e-04) * (q[i1] * q[i15]) + (-6.929032e-05) * (q[i1] * q[i16]) + (-1.173045e-04) * (q[i1] * q[i20])
            + (-1.194085e-04) * (q[i1] * q[i21]) + (8.183928e-04) * (q[i1] * q[i22]) + (-9.618666e-04) * (q[i2] * q[i3]) + (2.656238e-03) * (q[i2] * q[i4])
            + (-3.188673e-03) * (q[i2] * q[i5]) + (-9.689233e-04) * (q[i2] * q[i6]) + (1.409915e-03) * (q[i2] * q[i7]) + (-2.143070e-03) * (q[i2] * q[i8])
            + (-1.062082e-04) * (q[i2] * q[i9]) + (3.717942e-04) * (q[i2] * q[i10]) + (-7.304907e-04) * (q[i2] * q[i11]) + (-6.144859e-04) * (q[i2] * q[i12])
            + (5.335550e-04) * (q[i2] * q[i15]) + (9.717815e-04) * (q[i2] * q[i16]) + (-8.298723e-04) * (q[i2] * q[i20]) + (-6.527569e-04) * (q[i2] * q[i21])
            + (3.099660e-04) * (q[i2] * q[i22]) + (-1.061026e-03) * (q[i3] * q[i4]) + (-3.405769e-03) * (q[i3] * q[i5]) + (-1.181307e-03) * (q[i3] * q[i6])
            + (-2.230980e-04) * (q[i3] * q[i7]) + (5.242117e-03) * (q[i3] * q[i8]) + (1.489055e-03) * (q[i3] * q[i9]) + (1.010646e-03) * (q[i3] * q[i10])
            + (-1.174579e-03) * (q[i3] * q[i11]) + (-4.828984e-03) * (q[i3] * q[i12]) + (-1.359044e-03) * (q[i3] * q[i15]) + (-2.043554e-03) * (q[i3] * q[i16])
            + (-2.845084e-04) * (q[i3] * q[i20]) + (2.922209e-04) * (q[i3] * q[i21]) + (-7.987878e-04) * (q[i3] * q[i22]) + (2.421059e-03) * (q[i4] * q[i5])
            + (5.798120e-03) * (q[i4] * q[i6]) + (2.130440e-03) * (q[i4] * q[i7]) + (-2.804724e-03) * (q[i4] * q[i8]) + (-1.238296e-03) * (q[i4] * q[i9])
            + (5.875405e-04) * (q[i4] * q[i10]) + (2.112294e-03) * (q[i4] * q[i11]) + (2.819507e-03) * (q[i4] * q[i12]) + (3.023984e-03) * (q[i4] * q[i15])
            + (1.869637e-03) * (q[i4] * q[i16]) + (-2.449044e-04) * (q[i4] * q[i20]) + (-1.807287e-03) * (q[i4] * q[i21]) + (-8.107735e-04) * (q[i4] * q[i22])
            + (2.702330e-03) * (q[i5] * q[i6]) + (-8.926671e-04) * (q[i5] * q[i7]) + (-1.052659e-03) * (q[i5] * q[i8]) + (3.115368e-04) * (q[i5] * q[i9])
            + (-1.069672e-03) * (q[i5] * q[i10]) + (-7.826358e-04) * (q[i5] * q[i11]) + (-9.107160e-04) * (q[i5] * q[i12]) + (7.910750e-04) * (q[i5] * q[i15])
            + (5.531803e-05) * (q[i5] * q[i16]) + (4.123524e-06) * (q[i5] * q[i20]) + (2.630843e-04) * (q[i5] * q[i21]) + (5.709591e-04) * (q[i5] * q[i22])
            + (1.347722e-03) * (q[i6] * q[i7]) + (9.782341e-04) * (q[i6] * q[i8]) + (1.523656e-03) * (q[i6] * q[i9]) + (1.324738e-03) * (q[i6] * q[i10])
            + (1.671965e-03) * (q[i6] * q[i11]) + (-3.422618e-04) * (q[i6] * q[i12]) + (-1.345635e-03) * (q[i6] * q[i15]) + (-3.332313e-03) * (q[i6] * q[i16])
            + (1.521665e-03) * (q[i6] * q[i20]) + (-1.122185e-03) * (q[i6] * q[i21]) + (1.952267e-04) * (q[i6] * q[i22]) + (-3.737186e-03) * (q[i7] * q[i8])
            + (-7.282846e-04) * (q[i7] * q[i9]) + (-1.287692e-03) * (q[i7] * q[i10]) + (-1.694066e-03) * (q[i7] * q[i11]) + (-7.212440e-04) * (q[i7] * q[i12])
            + (-5.565482e-04) * (q[i7] * q[i15]) + (1.611454e-03) * (q[i7] * q[i16]) + (-1.519264e-03) * (q[i7] * q[i20]) + (-1.940345e-03) * (q[i7] * q[i21])
            + (-4.046829e-05) * (q[i7] * q[i22]) + (8.208506e-04) * (q[i8] * q[i9]) + (-2.938725e-03) * (q[i8] * q[i10]) + (-1.131070e-03) * (q[i8] * q[i11])
            + (4.809914e-04) * (q[i8] * q[i12]) + (1.069179e-03) * (q[i8] * q[i15]) + (7.779851e-04) * (q[i8] * q[i16]) + (5.116259e-06) * (q[i8] * q[i20])
            + (3.348708e-03) * (q[i8] * q[i21]) + (2.725898e-04) * (q[i8] * q[i22]) + (3.068323e-04) * (q[i9] * q[i10]) + (2.570354e-04) * (q[i9] * q[i11])
            + (8.131269e-05) * (q[i9] * q[i12]) + (-1.487320e-04) * (q[i9] * q[i15]) + (-9.418895e-04) * (q[i9] * q[i16]) + (6.983687e-04) * (q[i9] * q[i20])
            + (-4.695622e-04) * (q[i9] * q[i21]) + (2.972330e-04) * (q[i9] * q[i22]) + (-6.430745e-06) * (q[i10] * q[i11]) + (-2.767815e-04) * (q[i10] * q[i12])
            + (-4.135366e-04) * (q[i10] * q[i15]) + (2.947857e-05) * (q[i10] * q[i16]) + (-6.971758e-04) * (q[i10] * q[i20])
            + (-3.065525e-04) * (q[i10] * q[i21]) + (-1.014092e-05) * (q[i10] * q[i22]) + (-5.943896e-04) * (q[i11] * q[i12])
            + (8.488030e-04) * (q[i11] * q[i15]) + (5.898340e-04) * (q[i11] * q[i16]) + (3.248382e-04) * (q[i11] * q[i20]) + (1.886006e-03) * (q[i11] * q[i21])
            + (7.798466e-04) * (q[i11] * q[i22]) + (-9.047756e-04) * (q[i12] * q[i15]) + (-1.627775e-04) * (q[i12] * q[i16])
            + (3.196399e-04) * (q[i12] * q[i20]) + (3.213397e-04) * (q[i12] * q[i21]) + (-3.151055e-04) * (q[i12] * q[i22])
            + (-7.513823e-05) * (q[i15] * q[i16]) + (-5.097665e-04) * (q[i15] * q[i20]) + (1.186223e-04) * (q[i15] * q[i21])
            + (-1.350526e-06) * (q[i15] * q[i22]) + (5.075237e-04) * (q[i16] * q[i20]) + (6.470507e-05) * (q[i16] * q[i21])
            + (-2.544436e-04) * (q[i16] * q[i22]) + (-2.439649e-05) * (q[i20] * q[i21]) + (-2.701687e-05) * (q[i20] * q[i22])
            + (8.061874e-05) * (q[i21] * q[i22]);
   }

   public void getJQz20(double[] q, double[][] JQ)
   {
      JQ[3][i20] = (-9.463679e-03) * (1) + (1.449502e-03) * ((2) * q[i20]) + (-2.484627e-05) * (q[i0]) + (-3.012531e-03) * (q[i1]) + (1.964953e-03) * (q[i2])
            + (1.146731e-02) * (q[i3]) + (-1.241368e-02) * (q[i4]) + (5.708390e-03) * (q[i5]) + (-5.856227e-03) * (q[i6]) + (8.629763e-03) * (q[i7])
            + (-4.199044e-03) * (q[i8]) + (-1.320874e-03) * (q[i9]) + (4.239309e-03) * (q[i10]) + (3.689563e-04) * (q[i11]) + (4.151972e-03) * (q[i12])
            + (-1.427964e-03) * (q[i15]) + (-4.613598e-03) * (q[i16]) + (7.985600e-06) * (q[i19]) + (-6.905349e-04) * (q[i21]) + (-5.626379e-03) * (q[i22])
            + (3.310978e-04) * (q[i0] * q[i0]) + (1.054223e-04) * (q[i1] * q[i1]) + (1.085563e-03) * (q[i2] * q[i2]) + (-2.612225e-04) * (q[i3] * q[i3])
            + (-8.482367e-04) * (q[i4] * q[i4]) + (-2.149900e-03) * (q[i5] * q[i5]) + (-8.974494e-04) * (q[i6] * q[i6]) + (-2.149406e-04) * (q[i7] * q[i7])
            + (1.153215e-03) * (q[i8] * q[i8]) + (5.050609e-04) * (q[i9] * q[i9]) + (-5.685368e-04) * (q[i10] * q[i10]) + (-2.343149e-05) * (q[i11] * q[i11])
            + (1.180369e-03) * (q[i12] * q[i12]) + (-3.238596e-05) * (q[i15] * q[i15]) + (-9.031755e-04) * (q[i16] * q[i16])
            + (-3.966958e-04) * (q[i19] * q[i19]) + (-8.809152e-04) * ((2) * q[i0] * q[i20]) + (9.579766e-04) * ((2) * q[i1] * q[i20])
            + (-3.275157e-04) * ((2) * q[i2] * q[i20]) + (9.940037e-04) * ((2) * q[i3] * q[i20]) + (-1.590706e-03) * ((2) * q[i4] * q[i20])
            + (5.569911e-04) * ((2) * q[i5] * q[i20]) + (1.285111e-03) * ((2) * q[i6] * q[i20]) + (-1.281165e-03) * ((2) * q[i7] * q[i20])
            + (2.460478e-04) * ((2) * q[i8] * q[i20]) + (-2.195414e-04) * ((2) * q[i9] * q[i20]) + (-3.820262e-04) * ((2) * q[i10] * q[i20])
            + (-1.191432e-04) * ((2) * q[i11] * q[i20]) + (6.084688e-04) * ((2) * q[i12] * q[i20]) + (1.876722e-04) * ((2) * q[i15] * q[i20])
            + (-3.153356e-04) * ((2) * q[i16] * q[i20]) + (3.836269e-04) * ((2) * q[i19] * q[i20]) + (1.954735e-04) * ((3) * q[i20] * q[i20])
            + (-1.573420e-05) * ((2) * q[i20] * q[i21]) + (-2.097085e-03) * ((2) * q[i20] * q[i22]) + (1.407896e-04) * (q[i21] * q[i21])
            + (1.047033e-03) * (q[i22] * q[i22]) + (-3.106547e-04) * (q[i0] * q[i1]) + (3.204858e-04) * (q[i0] * q[i2]) + (-4.420090e-04) * (q[i0] * q[i3])
            + (-5.171096e-04) * (q[i0] * q[i4]) + (-5.019998e-04) * (q[i0] * q[i5]) + (-7.278929e-04) * (q[i0] * q[i6]) + (-1.157077e-04) * (q[i0] * q[i7])
            + (2.572781e-03) * (q[i0] * q[i8]) + (-3.476660e-04) * (q[i0] * q[i9]) + (-6.231768e-04) * (q[i0] * q[i10]) + (-1.578691e-03) * (q[i0] * q[i11])
            + (2.247609e-03) * (q[i0] * q[i12]) + (-6.770629e-05) * (q[i0] * q[i15]) + (-9.726985e-04) * (q[i0] * q[i16]) + (-1.029540e-04) * (q[i0] * q[i19])
            + (-8.140101e-04) * (q[i0] * q[i21]) + (1.086136e-04) * (q[i0] * q[i22]) + (1.812038e-03) * (q[i1] * q[i2]) + (-2.998582e-05) * (q[i1] * q[i3])
            + (6.645825e-04) * (q[i1] * q[i4]) + (1.376363e-03) * (q[i1] * q[i5]) + (-2.713295e-03) * (q[i1] * q[i6]) + (-1.945346e-03) * (q[i1] * q[i7])
            + (-9.210163e-04) * (q[i1] * q[i8]) + (1.136149e-03) * (q[i1] * q[i9]) + (1.601052e-03) * (q[i1] * q[i10]) + (-6.580470e-04) * (q[i1] * q[i11])
            + (-1.639693e-03) * (q[i1] * q[i12]) + (1.207770e-03) * (q[i1] * q[i15]) + (1.071461e-03) * (q[i1] * q[i16]) + (-1.173045e-04) * (q[i1] * q[i19])
            + (6.573443e-04) * (q[i1] * q[i21]) + (-7.683510e-04) * (q[i1] * q[i22]) + (-2.681924e-03) * (q[i2] * q[i3]) + (9.512730e-04) * (q[i2] * q[i4])
            + (3.189714e-03) * (q[i2] * q[i5]) + (1.426482e-03) * (q[i2] * q[i6]) + (-9.379687e-04) * (q[i2] * q[i7]) + (-2.164839e-03) * (q[i2] * q[i8])
            + (3.898031e-04) * (q[i2] * q[i9]) + (-1.153407e-04) * (q[i2] * q[i10]) + (6.020709e-04) * (q[i2] * q[i11]) + (7.187429e-04) * (q[i2] * q[i12])
            + (9.489542e-04) * (q[i2] * q[i15]) + (5.450892e-04) * (q[i2] * q[i16]) + (-8.298723e-04) * (q[i2] * q[i19]) + (-3.104171e-04) * (q[i2] * q[i21])
            + (6.695144e-04) * (q[i2] * q[i22]) + (1.098279e-03) * (q[i3] * q[i4]) + (-2.451930e-03) * (q[i3] * q[i5]) + (2.162745e-03) * (q[i3] * q[i6])
            + (5.809542e-03) * (q[i3] * q[i7]) + (-2.827921e-03) * (q[i3] * q[i8]) + (6.169341e-04) * (q[i3] * q[i9]) + (-1.256187e-03) * (q[i3] * q[i10])
            + (-2.797302e-03) * (q[i3] * q[i11]) + (-2.117439e-03) * (q[i3] * q[i12]) + (1.892597e-03) * (q[i3] * q[i15]) + (2.998594e-03) * (q[i3] * q[i16])
            + (-2.845084e-04) * (q[i3] * q[i19]) + (8.213946e-04) * (q[i3] * q[i21]) + (1.826411e-03) * (q[i3] * q[i22]) + (3.417518e-03) * (q[i4] * q[i5])
            + (-2.300427e-04) * (q[i4] * q[i6]) + (-1.210903e-03) * (q[i4] * q[i7]) + (5.203166e-03) * (q[i4] * q[i8]) + (1.006603e-03) * (q[i4] * q[i9])
            + (1.476427e-03) * (q[i4] * q[i10]) + (4.808062e-03) * (q[i4] * q[i11]) + (1.160681e-03) * (q[i4] * q[i12]) + (-2.037393e-03) * (q[i4] * q[i15])
            + (-1.350580e-03) * (q[i4] * q[i16]) + (-2.449044e-04) * (q[i4] * q[i19]) + (7.983853e-04) * (q[i4] * q[i21]) + (-2.967091e-04) * (q[i4] * q[i22])
            + (-9.051232e-04) * (q[i5] * q[i6]) + (2.701717e-03) * (q[i5] * q[i7]) + (-1.009716e-03) * (q[i5] * q[i8]) + (-1.091896e-03) * (q[i5] * q[i9])
            + (3.255600e-04) * (q[i5] * q[i10]) + (8.982612e-04) * (q[i5] * q[i11]) + (7.582725e-04) * (q[i5] * q[i12]) + (3.173976e-05) * (q[i5] * q[i15])
            + (8.106571e-04) * (q[i5] * q[i16]) + (4.123524e-06) * (q[i5] * q[i19]) + (-5.749362e-04) * (q[i5] * q[i21]) + (-2.760606e-04) * (q[i5] * q[i22])
            + (-1.363415e-03) * (q[i6] * q[i7]) + (3.762517e-03) * (q[i6] * q[i8]) + (1.277208e-03) * (q[i6] * q[i9]) + (7.252227e-04) * (q[i6] * q[i10])
            + (-7.149938e-04) * (q[i6] * q[i11]) + (-1.686519e-03) * (q[i6] * q[i12]) + (-1.620104e-03) * (q[i6] * q[i15]) + (5.609957e-04) * (q[i6] * q[i16])
            + (1.521665e-03) * (q[i6] * q[i19]) + (-4.661550e-05) * (q[i6] * q[i21]) + (-1.946770e-03) * (q[i6] * q[i22]) + (-9.455939e-04) * (q[i7] * q[i8])
            + (-1.328406e-03) * (q[i7] * q[i9]) + (-1.523878e-03) * (q[i7] * q[i10]) + (-3.149649e-04) * (q[i7] * q[i11]) + (1.663596e-03) * (q[i7] * q[i12])
            + (3.309427e-03) * (q[i7] * q[i15]) + (1.343040e-03) * (q[i7] * q[i16]) + (-1.519264e-03) * (q[i7] * q[i19]) + (2.033601e-04) * (q[i7] * q[i21])
            + (-1.112846e-03) * (q[i7] * q[i22]) + (2.952076e-03) * (q[i8] * q[i9]) + (-8.147241e-04) * (q[i8] * q[i10]) + (4.758059e-04) * (q[i8] * q[i11])
            + (-1.072583e-03) * (q[i8] * q[i12]) + (-7.574531e-04) * (q[i8] * q[i15]) + (-1.079222e-03) * (q[i8] * q[i16]) + (5.116259e-06) * (q[i8] * q[i19])
            + (2.678443e-04) * (q[i8] * q[i21]) + (3.337514e-03) * (q[i8] * q[i22]) + (-3.114489e-04) * (q[i9] * q[i10]) + (-2.698155e-04) * (q[i9] * q[i11])
            + (-9.948959e-06) * (q[i9] * q[i12]) + (-2.653334e-05) * (q[i9] * q[i15]) + (4.169658e-04) * (q[i9] * q[i16]) + (6.983687e-04) * (q[i9] * q[i19])
            + (-1.456776e-05) * (q[i9] * q[i21]) + (-3.028418e-04) * (q[i9] * q[i22]) + (9.173832e-05) * (q[i10] * q[i11]) + (2.536174e-04) * (q[i10] * q[i12])
            + (9.230676e-04) * (q[i10] * q[i15]) + (1.432046e-04) * (q[i10] * q[i16]) + (-6.971758e-04) * (q[i10] * q[i19]) + (2.987073e-04) * (q[i10] * q[i21])
            + (-4.575649e-04) * (q[i10] * q[i22]) + (6.070909e-04) * (q[i11] * q[i12]) + (-1.645095e-04) * (q[i11] * q[i15])
            + (-9.083006e-04) * (q[i11] * q[i16]) + (3.248382e-04) * (q[i11] * q[i19]) + (3.111280e-04) * (q[i11] * q[i21])
            + (-3.240401e-04) * (q[i11] * q[i22]) + (5.885397e-04) * (q[i12] * q[i15]) + (8.694204e-04) * (q[i12] * q[i16]) + (3.196399e-04) * (q[i12] * q[i19])
            + (-7.816191e-04) * (q[i12] * q[i21]) + (-1.866132e-03) * (q[i12] * q[i22]) + (7.803911e-05) * (q[i15] * q[i16])
            + (-5.097665e-04) * (q[i15] * q[i19]) + (-2.515901e-04) * (q[i15] * q[i21]) + (6.296997e-05) * (q[i15] * q[i22])
            + (5.075237e-04) * (q[i16] * q[i19]) + (5.950866e-06) * (q[i16] * q[i21]) + (1.024102e-04) * (q[i16] * q[i22]) + (-2.439649e-05) * (q[i19] * q[i21])
            + (-2.701687e-05) * (q[i19] * q[i22]) + (-8.116803e-05) * (q[i21] * q[i22]);
   }

   public void getJQz21(double[] q, double[][] JQ)
   {
      JQ[3][i21] = (8.252879e-03) * (1) + (5.218888e-05) * ((2) * q[i21]) + (-6.986138e-03) * (q[i0]) + (-1.117623e-03) * (q[i1]) + (-3.676468e-03) * (q[i2])
            + (1.072986e-02) * (q[i3]) + (-3.541732e-03) * (q[i4]) + (-3.162355e-03) * (q[i5]) + (-1.736492e-03) * (q[i6]) + (2.335451e-03) * (q[i7])
            + (3.168813e-03) * (q[i8]) + (-4.451931e-04) * (q[i9]) + (2.496257e-03) * (q[i10]) + (4.161639e-03) * (q[i11]) + (-4.494994e-04) * (q[i12])
            + (4.141272e-03) * (q[i15]) + (3.120678e-04) * (q[i16]) + (-5.571742e-03) * (q[i19]) + (-6.905349e-04) * (q[i20]) + (-3.384314e-06) * (q[i22])
            + (-1.531392e-04) * (q[i0] * q[i0]) + (4.403735e-04) * (q[i1] * q[i1]) + (5.330095e-04) * (q[i2] * q[i2]) + (-1.876124e-03) * (q[i3] * q[i3])
            + (1.014072e-03) * (q[i4] * q[i4]) + (4.273690e-04) * (q[i5] * q[i5]) + (-1.225484e-03) * (q[i6] * q[i6]) + (-1.779634e-03) * (q[i7] * q[i7])
            + (1.220470e-03) * (q[i8] * q[i8]) + (1.956634e-04) * (q[i9] * q[i9]) + (-7.848734e-04) * (q[i10] * q[i10]) + (-1.088728e-03) * (q[i11] * q[i11])
            + (2.075158e-04) * (q[i12] * q[i12]) + (1.447443e-03) * (q[i15] * q[i15]) + (-2.888105e-04) * (q[i16] * q[i16])
            + (-2.099350e-03) * (q[i19] * q[i19]) + (-1.573420e-05) * (q[i20] * q[i20]) + (-1.759633e-03) * ((2) * q[i0] * q[i21])
            + (-3.901743e-04) * ((2) * q[i1] * q[i21]) + (-2.045029e-03) * ((2) * q[i2] * q[i21]) + (-7.426308e-04) * ((2) * q[i3] * q[i21])
            + (1.312973e-03) * ((2) * q[i4] * q[i21]) + (-3.217971e-04) * ((2) * q[i5] * q[i21]) + (1.607276e-03) * ((2) * q[i6] * q[i21])
            + (-1.741215e-03) * ((2) * q[i7] * q[i21]) + (4.810772e-04) * ((2) * q[i8] * q[i21]) + (3.766099e-04) * ((2) * q[i9] * q[i21])
            + (6.369769e-05) * ((2) * q[i10] * q[i21]) + (-6.023134e-04) * ((2) * q[i11] * q[i21]) + (-4.094983e-04) * ((2) * q[i12] * q[i21])
            + (2.459007e-04) * ((2) * q[i15] * q[i21]) + (-1.195869e-04) * ((2) * q[i16] * q[i21]) + (-1.030272e-03) * ((2) * q[i19] * q[i21])
            + (1.407896e-04) * ((2) * q[i20] * q[i21]) + (-5.500735e-04) * ((3) * q[i21] * q[i21]) + (4.184537e-05) * ((2) * q[i21] * q[i22])
            + (3.682205e-05) * (q[i22] * q[i22]) + (1.096464e-03) * (q[i0] * q[i1]) + (-7.543829e-05) * (q[i0] * q[i2]) + (2.352586e-03) * (q[i0] * q[i3])
            + (-2.774221e-03) * (q[i0] * q[i4]) + (-2.871135e-03) * (q[i0] * q[i5]) + (-4.359618e-04) * (q[i0] * q[i6]) + (4.351924e-04) * (q[i0] * q[i7])
            + (5.334901e-04) * (q[i0] * q[i8]) + (1.984431e-04) * (q[i0] * q[i9]) + (6.428626e-04) * (q[i0] * q[i10]) + (-8.476260e-04) * (q[i0] * q[i11])
            + (1.695129e-03) * (q[i0] * q[i12]) + (-2.242571e-03) * (q[i0] * q[i15]) + (8.158969e-05) * (q[i0] * q[i16]) + (7.949083e-04) * (q[i0] * q[i19])
            + (-8.140101e-04) * (q[i0] * q[i20]) + (4.544889e-04) * (q[i0] * q[i22]) + (1.553031e-03) * (q[i1] * q[i2]) + (-7.713824e-04) * (q[i1] * q[i3])
            + (-2.249974e-03) * (q[i1] * q[i4]) + (-8.784839e-04) * (q[i1] * q[i5]) + (6.585911e-04) * (q[i1] * q[i6]) + (-1.962824e-03) * (q[i1] * q[i7])
            + (1.685769e-03) * (q[i1] * q[i8]) + (1.872351e-04) * (q[i1] * q[i9]) + (-4.788341e-05) * (q[i1] * q[i10]) + (6.031111e-04) * (q[i1] * q[i11])
            + (-1.023231e-03) * (q[i1] * q[i12]) + (-2.136392e-04) * (q[i1] * q[i15]) + (-3.002668e-04) * (q[i1] * q[i16]) + (-1.194085e-04) * (q[i1] * q[i19])
            + (6.573443e-04) * (q[i1] * q[i20]) + (4.280431e-04) * (q[i1] * q[i22]) + (1.918705e-03) * (q[i2] * q[i3]) + (-3.841436e-03) * (q[i2] * q[i4])
            + (-2.336499e-03) * (q[i2] * q[i5]) + (-3.636610e-04) * (q[i2] * q[i6]) + (-4.657147e-04) * (q[i2] * q[i7]) + (7.322682e-04) * (q[i2] * q[i8])
            + (9.727644e-05) * (q[i2] * q[i9]) + (3.553253e-04) * (q[i2] * q[i10]) + (-8.595518e-04) * (q[i2] * q[i11]) + (-6.115261e-04) * (q[i2] * q[i12])
            + (-1.693832e-03) * (q[i2] * q[i15]) + (3.833704e-04) * (q[i2] * q[i16]) + (-6.527569e-04) * (q[i2] * q[i19]) + (-3.104171e-04) * (q[i2] * q[i20])
            + (1.096783e-03) * (q[i2] * q[i22]) + (-1.279848e-03) * (q[i3] * q[i4]) + (2.088894e-03) * (q[i3] * q[i5]) + (-1.000557e-03) * (q[i3] * q[i6])
            + (1.440824e-03) * (q[i3] * q[i7]) + (1.435959e-03) * (q[i3] * q[i8]) + (-1.580515e-03) * (q[i3] * q[i9]) + (-2.090453e-03) * (q[i3] * q[i10])
            + (-1.517272e-03) * (q[i3] * q[i11]) + (-5.185118e-04) * (q[i3] * q[i12]) + (2.977472e-04) * (q[i3] * q[i15]) + (9.347620e-04) * (q[i3] * q[i16])
            + (2.922209e-04) * (q[i3] * q[i19]) + (8.213946e-04) * (q[i3] * q[i20]) + (7.898597e-05) * (q[i3] * q[i22]) + (-2.063205e-03) * (q[i4] * q[i5])
            + (3.513589e-04) * (q[i4] * q[i6]) + (-3.757145e-03) * (q[i4] * q[i7]) + (-2.477220e-03) * (q[i4] * q[i8]) + (1.596703e-03) * (q[i4] * q[i9])
            + (-4.333095e-04) * (q[i4] * q[i10]) + (1.044082e-04) * (q[i4] * q[i11]) + (-7.572880e-04) * (q[i4] * q[i12]) + (-1.102679e-03) * (q[i4] * q[i15])
            + (-1.002016e-03) * (q[i4] * q[i16]) + (-1.807287e-03) * (q[i4] * q[i19]) + (7.983853e-04) * (q[i4] * q[i20]) + (6.475461e-05) * (q[i4] * q[i22])
            + (1.591960e-03) * (q[i5] * q[i6]) + (1.449493e-03) * (q[i5] * q[i7]) + (1.641170e-03) * (q[i5] * q[i8]) + (7.568795e-04) * (q[i5] * q[i9])
            + (1.911686e-03) * (q[i5] * q[i10]) + (-2.270249e-04) * (q[i5] * q[i11]) + (7.650291e-04) * (q[i5] * q[i12]) + (-1.798743e-04) * (q[i5] * q[i15])
            + (-1.722371e-04) * (q[i5] * q[i16]) + (2.630843e-04) * (q[i5] * q[i19]) + (-5.749362e-04) * (q[i5] * q[i20]) + (-9.237874e-04) * (q[i5] * q[i22])
            + (1.620153e-03) * (q[i6] * q[i7]) + (-3.744837e-04) * (q[i6] * q[i8]) + (1.961747e-04) * (q[i6] * q[i9]) + (1.626460e-03) * (q[i6] * q[i10])
            + (1.916454e-03) * (q[i6] * q[i11]) + (-1.191898e-03) * (q[i6] * q[i12]) + (-1.247835e-03) * (q[i6] * q[i15]) + (5.767641e-04) * (q[i6] * q[i16])
            + (-1.122185e-03) * (q[i6] * q[i19]) + (-4.661550e-05) * (q[i6] * q[i20]) + (-3.455159e-04) * (q[i6] * q[i22]) + (5.372617e-04) * (q[i7] * q[i8])
            + (-5.005814e-04) * (q[i7] * q[i9]) + (-2.451621e-03) * (q[i7] * q[i10]) + (-1.967779e-03) * (q[i7] * q[i11]) + (-1.454366e-03) * (q[i7] * q[i12])
            + (-8.648373e-06) * (q[i7] * q[i15]) + (9.408520e-04) * (q[i7] * q[i16]) + (-1.940345e-03) * (q[i7] * q[i19]) + (2.033601e-04) * (q[i7] * q[i20])
            + (3.555269e-04) * (q[i7] * q[i22]) + (-1.144188e-03) * (q[i8] * q[i9]) + (4.143830e-04) * (q[i8] * q[i10]) + (-3.066098e-04) * (q[i8] * q[i11])
            + (1.149213e-03) * (q[i8] * q[i12]) + (2.629756e-03) * (q[i8] * q[i15]) + (-2.999376e-04) * (q[i8] * q[i16]) + (3.348708e-03) * (q[i8] * q[i19])
            + (2.678443e-04) * (q[i8] * q[i20]) + (7.139614e-06) * (q[i8] * q[i22]) + (2.671950e-04) * (q[i9] * q[i10]) + (5.498505e-04) * (q[i9] * q[i11])
            + (-7.537221e-04) * (q[i9] * q[i12]) + (-6.297149e-04) * (q[i9] * q[i15]) + (1.721087e-05) * (q[i9] * q[i16]) + (-4.695622e-04) * (q[i9] * q[i19])
            + (-1.456776e-05) * (q[i9] * q[i20]) + (-1.130490e-04) * (q[i9] * q[i22]) + (2.451832e-04) * (q[i10] * q[i11]) + (-5.718205e-04) * (q[i10] * q[i12])
            + (2.476633e-05) * (q[i10] * q[i15]) + (-4.318072e-05) * (q[i10] * q[i16]) + (-3.065525e-04) * (q[i10] * q[i19])
            + (2.987073e-04) * (q[i10] * q[i20]) + (1.193717e-04) * (q[i10] * q[i22]) + (1.004132e-04) * (q[i11] * q[i12]) + (1.516783e-03) * (q[i11] * q[i15])
            + (1.304451e-04) * (q[i11] * q[i16]) + (1.886006e-03) * (q[i11] * q[i19]) + (3.111280e-04) * (q[i11] * q[i20]) + (2.629995e-04) * (q[i11] * q[i22])
            + (9.203270e-05) * (q[i12] * q[i15]) + (1.930178e-04) * (q[i12] * q[i16]) + (3.213397e-04) * (q[i12] * q[i19]) + (-7.816191e-04) * (q[i12] * q[i20])
            + (2.602461e-04) * (q[i12] * q[i22]) + (-2.143389e-04) * (q[i15] * q[i16]) + (1.186223e-04) * (q[i15] * q[i19])
            + (-2.515901e-04) * (q[i15] * q[i20]) + (1.336803e-04) * (q[i15] * q[i22]) + (6.470507e-05) * (q[i16] * q[i19]) + (5.950866e-06) * (q[i16] * q[i20])
            + (-1.343905e-04) * (q[i16] * q[i22]) + (-2.439649e-05) * (q[i19] * q[i20]) + (8.061874e-05) * (q[i19] * q[i22])
            + (-8.116803e-05) * (q[i20] * q[i22]);
   }

   public void getJQz22(double[] q, double[][] JQ)
   {
      JQ[3][i22] = (8.274845e-03) * (1) + (-7.309604e-05) * ((2) * q[i22]) + (1.111550e-03) * (q[i0]) + (6.876052e-03) * (q[i1]) + (3.596080e-03) * (q[i2])
            + (3.548932e-03) * (q[i3]) + (-1.061346e-02) * (q[i4]) + (3.131660e-03) * (q[i5]) + (2.301098e-03) * (q[i6]) + (-1.764857e-03) * (q[i7])
            + (3.195922e-03) * (q[i8]) + (2.513326e-03) * (q[i9]) + (-4.460657e-04) * (q[i10]) + (4.512438e-04) * (q[i11]) + (-4.286199e-03) * (q[i12])
            + (3.035838e-04) * (q[i15]) + (4.203977e-03) * (q[i16]) + (-7.097554e-04) * (q[i19]) + (-5.626379e-03) * (q[i20]) + (-3.384314e-06) * (q[i21])
            + (4.324533e-04) * (q[i0] * q[i0]) + (-1.664605e-04) * (q[i1] * q[i1]) + (5.261333e-04) * (q[i2] * q[i2]) + (1.019498e-03) * (q[i3] * q[i3])
            + (-1.839932e-03) * (q[i4] * q[i4]) + (4.067289e-04) * (q[i5] * q[i5]) + (-1.812007e-03) * (q[i6] * q[i6]) + (-1.211107e-03) * (q[i7] * q[i7])
            + (1.243642e-03) * (q[i8] * q[i8]) + (-7.894434e-04) * (q[i9] * q[i9]) + (1.943379e-04) * (q[i10] * q[i10]) + (2.058726e-04) * (q[i11] * q[i11])
            + (-1.071907e-03) * (q[i12] * q[i12]) + (-2.898229e-04) * (q[i15] * q[i15]) + (1.464243e-03) * (q[i16] * q[i16])
            + (-1.815490e-05) * (q[i19] * q[i19]) + (-2.097085e-03) * (q[i20] * q[i20]) + (4.184537e-05) * (q[i21] * q[i21])
            + (-4.010723e-04) * ((2) * q[i0] * q[i22]) + (-1.742359e-03) * ((2) * q[i1] * q[i22]) + (-2.031400e-03) * ((2) * q[i2] * q[i22])
            + (1.324916e-03) * ((2) * q[i3] * q[i22]) + (-7.465614e-04) * ((2) * q[i4] * q[i22]) + (-3.075707e-04) * ((2) * q[i5] * q[i22])
            + (1.739386e-03) * ((2) * q[i6] * q[i22]) + (-1.603073e-03) * ((2) * q[i7] * q[i22]) + (-4.754923e-04) * ((2) * q[i8] * q[i22])
            + (-6.016814e-05) * ((2) * q[i9] * q[i22]) + (-3.702790e-04) * ((2) * q[i10] * q[i22]) + (-4.069585e-04) * ((2) * q[i11] * q[i22])
            + (-5.864783e-04) * ((2) * q[i12] * q[i22]) + (1.223636e-04) * ((2) * q[i15] * q[i22]) + (-2.405727e-04) * ((2) * q[i16] * q[i22])
            + (-1.440290e-04) * ((2) * q[i19] * q[i22]) + (1.047033e-03) * ((2) * q[i20] * q[i22]) + (3.682205e-05) * ((2) * q[i21] * q[i22])
            + (-5.527782e-04) * ((3) * q[i22] * q[i22]) + (1.088216e-03) * (q[i0] * q[i1]) + (1.559269e-03) * (q[i0] * q[i2])
            + (-2.251364e-03) * (q[i0] * q[i3]) + (-7.930453e-04) * (q[i0] * q[i4]) + (-8.908884e-04) * (q[i0] * q[i5]) + (1.950349e-03) * (q[i0] * q[i6])
            + (-6.437164e-04) * (q[i0] * q[i7]) + (-1.699471e-03) * (q[i0] * q[i8]) + (4.594173e-05) * (q[i0] * q[i9]) + (-1.778341e-04) * (q[i0] * q[i10])
            + (-1.043071e-03) * (q[i0] * q[i11]) + (6.040034e-04) * (q[i0] * q[i12]) + (2.860531e-04) * (q[i0] * q[i15]) + (2.233752e-04) * (q[i0] * q[i16])
            + (-6.577718e-04) * (q[i0] * q[i19]) + (1.086136e-04) * (q[i0] * q[i20]) + (4.544889e-04) * (q[i0] * q[i21]) + (-1.092656e-04) * (q[i1] * q[i2])
            + (-2.770689e-03) * (q[i1] * q[i3]) + (2.335218e-03) * (q[i1] * q[i4]) + (-2.824797e-03) * (q[i1] * q[i5]) + (-4.110103e-04) * (q[i1] * q[i6])
            + (4.281154e-04) * (q[i1] * q[i7]) + (-5.526813e-04) * (q[i1] * q[i8]) + (-6.430545e-04) * (q[i1] * q[i9]) + (-1.747739e-04) * (q[i1] * q[i10])
            + (1.702132e-03) * (q[i1] * q[i11]) + (-8.373231e-04) * (q[i1] * q[i12]) + (-6.772398e-05) * (q[i1] * q[i15]) + (2.228063e-03) * (q[i1] * q[i16])
            + (8.183928e-04) * (q[i1] * q[i19]) + (-7.683510e-04) * (q[i1] * q[i20]) + (4.280431e-04) * (q[i1] * q[i21]) + (-3.860117e-03) * (q[i2] * q[i3])
            + (1.912853e-03) * (q[i2] * q[i4]) + (-2.288843e-03) * (q[i2] * q[i5]) + (4.758287e-04) * (q[i2] * q[i6]) + (3.561437e-04) * (q[i2] * q[i7])
            + (-7.450876e-04) * (q[i2] * q[i8]) + (-3.653917e-04) * (q[i2] * q[i9]) + (-8.714664e-05) * (q[i2] * q[i10]) + (-6.161720e-04) * (q[i2] * q[i11])
            + (-8.317397e-04) * (q[i2] * q[i12]) + (-3.798725e-04) * (q[i2] * q[i15]) + (1.684649e-03) * (q[i2] * q[i16]) + (3.099660e-04) * (q[i2] * q[i19])
            + (6.695144e-04) * (q[i2] * q[i20]) + (1.096783e-03) * (q[i2] * q[i21]) + (-1.279522e-03) * (q[i3] * q[i4]) + (-2.087590e-03) * (q[i3] * q[i5])
            + (3.790578e-03) * (q[i3] * q[i6]) + (-3.218778e-04) * (q[i3] * q[i7]) + (2.509093e-03) * (q[i3] * q[i8]) + (4.432069e-04) * (q[i3] * q[i9])
            + (-1.610804e-03) * (q[i3] * q[i10]) + (-7.618473e-04) * (q[i3] * q[i11]) + (9.435594e-05) * (q[i3] * q[i12]) + (1.001613e-03) * (q[i3] * q[i15])
            + (1.107827e-03) * (q[i3] * q[i16]) + (-7.987878e-04) * (q[i3] * q[i19]) + (1.826411e-03) * (q[i3] * q[i20]) + (7.898597e-05) * (q[i3] * q[i21])
            + (2.065826e-03) * (q[i4] * q[i5]) + (-1.427510e-03) * (q[i4] * q[i6]) + (1.006111e-03) * (q[i4] * q[i7]) + (-1.455301e-03) * (q[i4] * q[i8])
            + (2.096791e-03) * (q[i4] * q[i9]) + (1.577342e-03) * (q[i4] * q[i10]) + (-5.197775e-04) * (q[i4] * q[i11]) + (-1.530939e-03) * (q[i4] * q[i12])
            + (-9.009462e-04) * (q[i4] * q[i15]) + (-2.786692e-04) * (q[i4] * q[i16]) + (-8.107735e-04) * (q[i4] * q[i19]) + (-2.967091e-04) * (q[i4] * q[i20])
            + (6.475461e-05) * (q[i4] * q[i21]) + (-1.461438e-03) * (q[i5] * q[i6]) + (-1.590293e-03) * (q[i5] * q[i7]) + (-1.689967e-03) * (q[i5] * q[i8])
            + (-1.920676e-03) * (q[i5] * q[i9]) + (-7.536486e-04) * (q[i5] * q[i10]) + (7.702120e-04) * (q[i5] * q[i11]) + (-2.058770e-04) * (q[i5] * q[i12])
            + (1.713108e-04) * (q[i5] * q[i15]) + (1.665068e-04) * (q[i5] * q[i16]) + (5.709591e-04) * (q[i5] * q[i19]) + (-2.760606e-04) * (q[i5] * q[i20])
            + (-9.237874e-04) * (q[i5] * q[i21]) + (1.609069e-03) * (q[i6] * q[i7]) + (5.308637e-04) * (q[i6] * q[i8]) + (-2.454917e-03) * (q[i6] * q[i9])
            + (-4.929493e-04) * (q[i6] * q[i10]) + (1.473500e-03) * (q[i6] * q[i11]) + (1.967394e-03) * (q[i6] * q[i12]) + (9.407677e-04) * (q[i6] * q[i15])
            + (-1.211639e-05) * (q[i6] * q[i16]) + (1.952267e-04) * (q[i6] * q[i19]) + (-1.946770e-03) * (q[i6] * q[i20]) + (-3.455159e-04) * (q[i6] * q[i21])
            + (-3.697157e-04) * (q[i7] * q[i8]) + (1.636119e-03) * (q[i7] * q[i9]) + (1.984686e-04) * (q[i7] * q[i10]) + (1.176254e-03) * (q[i7] * q[i11])
            + (-1.900401e-03) * (q[i7] * q[i12]) + (5.829073e-04) * (q[i7] * q[i15]) + (-1.249415e-03) * (q[i7] * q[i16]) + (-4.046829e-05) * (q[i7] * q[i19])
            + (-1.112846e-03) * (q[i7] * q[i20]) + (3.555269e-04) * (q[i7] * q[i21]) + (4.128965e-04) * (q[i8] * q[i9]) + (-1.149477e-03) * (q[i8] * q[i10])
            + (-1.142485e-03) * (q[i8] * q[i11]) + (3.046039e-04) * (q[i8] * q[i12]) + (-3.028679e-04) * (q[i8] * q[i15]) + (2.667583e-03) * (q[i8] * q[i16])
            + (2.725898e-04) * (q[i8] * q[i19]) + (3.337514e-03) * (q[i8] * q[i20]) + (7.139614e-06) * (q[i8] * q[i21]) + (2.713456e-04) * (q[i9] * q[i10])
            + (5.739907e-04) * (q[i9] * q[i11]) + (-2.366834e-04) * (q[i9] * q[i12]) + (-3.685585e-05) * (q[i9] * q[i15]) + (2.033018e-05) * (q[i9] * q[i16])
            + (2.972330e-04) * (q[i9] * q[i19]) + (-3.028418e-04) * (q[i9] * q[i20]) + (-1.130490e-04) * (q[i9] * q[i21]) + (7.502987e-04) * (q[i10] * q[i11])
            + (-5.451014e-04) * (q[i10] * q[i12]) + (2.225760e-05) * (q[i10] * q[i15]) + (-6.301378e-04) * (q[i10] * q[i16])
            + (-1.014092e-05) * (q[i10] * q[i19]) + (-4.575649e-04) * (q[i10] * q[i20]) + (1.193717e-04) * (q[i10] * q[i21])
            + (1.110204e-04) * (q[i11] * q[i12]) + (-1.931456e-04) * (q[i11] * q[i15]) + (-9.794195e-05) * (q[i11] * q[i16])
            + (7.798466e-04) * (q[i11] * q[i19]) + (-3.240401e-04) * (q[i11] * q[i20]) + (2.629995e-04) * (q[i11] * q[i21])
            + (-1.425435e-04) * (q[i12] * q[i15]) + (-1.559937e-03) * (q[i12] * q[i16]) + (-3.151055e-04) * (q[i12] * q[i19])
            + (-1.866132e-03) * (q[i12] * q[i20]) + (2.602461e-04) * (q[i12] * q[i21]) + (-2.106631e-04) * (q[i15] * q[i16])
            + (-1.350526e-06) * (q[i15] * q[i19]) + (6.296997e-05) * (q[i15] * q[i20]) + (1.336803e-04) * (q[i15] * q[i21])
            + (-2.544436e-04) * (q[i16] * q[i19]) + (1.024102e-04) * (q[i16] * q[i20]) + (-1.343905e-04) * (q[i16] * q[i21])
            + (-2.701687e-05) * (q[i19] * q[i20]) + (8.061874e-05) * (q[i19] * q[i21]) + (-8.116803e-05) * (q[i20] * q[i21]);
   }

}
