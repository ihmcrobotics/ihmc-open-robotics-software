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
   private final int NumDoFs = 29;  // excluding floating base joint

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
   private final Quaternion quaternionNPrtBase = new Quaternion(0, 0, 0, 1); // Natural Posture rt the pelvis
   private final Quaternion quaternionNPrtWorld = new Quaternion(0, 0, 0, 1); // Natural Posture rt the world
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
   double[] qNomStandingURDF = new double[] {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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

      yoQuaternionNPrtWorld = new YoFrameQuaternion("naturalPostureInWorld", ReferenceFrame.getWorldFrame(), registry);
      yoQuaternionIdent = new YoFrameQuaternion("pelvisFrameViz", ReferenceFrame.getWorldFrame(), registry);
      //      yoQuaternionNPrtBase = new YoFrameQuaternion("naturalPostureInBase", robotModel.getRootBody().getBodyFixedFrame(), registry);

      if (yoGraphicsListRegistry != null)
      {
         doGraphics = true;

         originNPpelvis = new YoFramePoint3D("originNPpelvis", ReferenceFrame.getWorldFrame(), registry);
         naturalPostureVizPelvis = new YoGraphicCoordinateSystem("NaturalPostureP", originNPpelvis, yoQuaternionNPrtWorld, 0.5);
         yoGraphicsListRegistry.registerYoGraphic("NaturalPostureP", naturalPostureVizPelvis);

         originPelvis = new YoFramePoint3D("originPelvis", ReferenceFrame.getWorldFrame(), registry);
         vizPelvis = new YoGraphicCoordinateSystem("Pelvisframe", originPelvis, yoQuaternionIdent, 0.4);
         yoGraphicsListRegistry.registerYoGraphic("Pelvisframe", vizPelvis);
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

      useURDFJointNumbering = true;
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
      this.npQoffset.set(Qoffset);
   }

   @Override
   public Quaternion getNaturalPostureQuaternion()
   {
      return this.quaternionNPrtWorld;
   }

   @Override
   public Quaternion getNaturalPostureQuaternionrtBase()
   {
      return this.quaternionNPrtBase;
   }

   @Override
   public DMatrixRMaj getNaturalPostureJacobian()
   {
      return this.jacobianNP;
   }

   @Override
   public void compute(double[] q, Orientation3DReadOnly Qbase)
   {
      computeNaturalPosture(q, Qbase);
   }

   public void computeNaturalPosture(double[] q, Orientation3DReadOnly Qbase)
   {
      // Get the NP quaternion r.t. the base(pelvis) frame:
      computeQuaternionNPrtBase(q, this.quaternionNPrtBase);

      // Express the NP quaternion in the world-frame:
      this.quaternionNPrtWorld.set(Qbase);
      this.quaternionNPrtWorld.multiply(this.quaternionNPrtBase);

      // Get the NP quaternion jacobian r.t. the base(pelvis) frame:
      // + We need 'q' because, like quaternionNPrtBase, this jacobian is also an explicit function of q.
      // + We need 'quaternionNPrtBase` to deduce the 1st row of this jacobian from the unit-sphere constraint. 
      computeJacobianQuaternionNPrtBase(q, this.quaternionNPrtBase, this.jacobianQuaternionNPrtBase);
      // Convert the NP jacobian to map to NP omega r.t. world ewrt NP-frame:
      // + We need 'quaternionNPrtBase' to get two transforms: E (bring Qdot -> omega) & C_NP_Base (bring omega_ewrt_Base -> omega_ewrt_NP)
      // + We need 'jacobiandQuaternionNPrtBase' as once transformed, it forms all the joint-based portion of the jacobianNP.
      computeJacobianNP(this.quaternionNPrtBase, this.jacobianQuaternionNPrtBase, this.jacobianNP);

      if (doGraphics == true)
      {
         FramePoint3D originPose = new FramePoint3D(this.fullRobotModel.getRootBody().getBodyFixedFrame());
         originPose.changeFrame(ReferenceFrame.getWorldFrame());
         yoQuaternionNPrtWorld.set(quaternionNPrtWorld);
         originNPpelvis.set(originPose);
         yoQuaternionIdent.set(Qbase);
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
   }

   //==== CSV utils ====

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
      JQ[1][i0] = (-1.011476e-02) * (1) + (-3.476910e-03) * ((2) * q[i0]) + (1.979567e-05) * (q[i1]) + (-3.901331e-04) * (q[i2]) + (-6.272712e-04) * (q[i3])
            + (-2.647624e-03) * (q[i4]) + (7.964863e-04) * (q[i5]) + (9.116774e-02) * (q[i6]) + (-9.695121e-03) * (q[i7]) + (-2.481516e-03) * (q[i8])
            + (2.963527e-02) * (q[i9]) + (-1.181848e-03) * (q[i10]) + (9.983218e-04) * (q[i11]) + (-1.750409e-03) * (q[i12]) + (1.361736e-03) * (q[i15])
            + (1.880785e-03) * (q[i16]) + (-9.288888e-04) * (q[i19]) + (-1.328615e-06) * (q[i20]) + (1.501395e-03) * (q[i21]) + (7.039540e-05) * (q[i22])
            + (1.866919e-03) * ((3) * q[i0] * q[i0]) + (-1.628516e-03) * ((2) * q[i0] * q[i1]) + (1.209551e-03) * ((2) * q[i0] * q[i2])
            + (-3.015329e-02) * ((2) * q[i0] * q[i3]) + (-1.588441e-03) * ((2) * q[i0] * q[i4]) + (-2.144201e-03) * ((2) * q[i0] * q[i5])
            + (4.955900e-04) * ((2) * q[i0] * q[i6]) + (-3.322631e-04) * ((2) * q[i0] * q[i7]) + (-6.276619e-04) * ((2) * q[i0] * q[i8])
            + (2.210254e-03) * ((2) * q[i0] * q[i9]) + (-7.039792e-04) * ((2) * q[i0] * q[i10]) + (2.165368e-04) * ((2) * q[i0] * q[i11])
            + (5.653810e-04) * ((2) * q[i0] * q[i12]) + (-4.210967e-04) * ((2) * q[i0] * q[i15]) + (4.122016e-04) * ((2) * q[i0] * q[i16])
            + (3.381829e-04) * ((2) * q[i0] * q[i19]) + (-6.683916e-04) * ((2) * q[i0] * q[i20]) + (2.935931e-04) * ((2) * q[i0] * q[i21])
            + (-1.106642e-05) * ((2) * q[i0] * q[i22]) + (-1.636837e-03) * (q[i1] * q[i1]) + (6.810815e-04) * (q[i2] * q[i2])
            + (-2.313770e-03) * (q[i3] * q[i3]) + (-6.343227e-04) * (q[i4] * q[i4]) + (-8.933798e-05) * (q[i5] * q[i5]) + (4.884875e-03) * (q[i6] * q[i6])
            + (-1.905565e-03) * (q[i7] * q[i7]) + (-1.376380e-04) * (q[i8] * q[i8]) + (-7.039191e-03) * (q[i9] * q[i9]) + (1.015281e-03) * (q[i10] * q[i10])
            + (4.014348e-04) * (q[i11] * q[i11]) + (-2.714360e-04) * (q[i12] * q[i12]) + (2.282337e-04) * (q[i15] * q[i15]) + (1.901364e-04) * (q[i16] * q[i16])
            + (-1.025481e-04) * (q[i19] * q[i19]) + (-1.444755e-06) * (q[i20] * q[i20]) + (1.070024e-04) * (q[i21] * q[i21])
            + (-1.444598e-04) * (q[i22] * q[i22]) + (-1.194385e-03) * (q[i1] * q[i2]) + (7.610132e-03) * (q[i1] * q[i3]) + (7.626269e-03) * (q[i1] * q[i4])
            + (-3.021663e-03) * (q[i1] * q[i5]) + (3.010431e-04) * (q[i1] * q[i6]) + (-2.984609e-04) * (q[i1] * q[i7]) + (7.173709e-06) * (q[i1] * q[i8])
            + (-9.985536e-04) * (q[i1] * q[i9]) + (9.903339e-04) * (q[i1] * q[i10]) + (-6.081333e-04) * (q[i1] * q[i11]) + (-6.105035e-04) * (q[i1] * q[i12])
            + (7.877042e-04) * (q[i1] * q[i15]) + (-7.890194e-04) * (q[i1] * q[i16]) + (-9.758276e-04) * (q[i1] * q[i19]) + (9.787422e-04) * (q[i1] * q[i20])
            + (-2.022340e-06) * (q[i1] * q[i21]) + (-4.565074e-06) * (q[i1] * q[i22]) + (-1.244728e-02) * (q[i2] * q[i3]) + (-2.347755e-04) * (q[i2] * q[i4])
            + (-8.450503e-03) * (q[i2] * q[i5]) + (3.948689e-04) * (q[i2] * q[i6]) + (-3.649659e-04) * (q[i2] * q[i7]) + (-2.402488e-03) * (q[i2] * q[i8])
            + (1.025103e-03) * (q[i2] * q[i9]) + (-2.223396e-04) * (q[i2] * q[i10]) + (1.562511e-04) * (q[i2] * q[i11]) + (8.152518e-04) * (q[i2] * q[i12])
            + (-7.861968e-04) * (q[i2] * q[i15]) + (2.308786e-03) * (q[i2] * q[i16]) + (3.991832e-05) * (q[i2] * q[i19]) + (8.805669e-04) * (q[i2] * q[i20])
            + (2.581975e-04) * (q[i2] * q[i21]) + (-6.659603e-05) * (q[i2] * q[i22]) + (1.838885e-04) * (q[i3] * q[i4]) + (3.433614e-03) * (q[i3] * q[i5])
            + (-1.054360e-02) * (q[i3] * q[i6]) + (2.862776e-03) * (q[i3] * q[i7]) + (2.243601e-03) * (q[i3] * q[i8]) + (-8.869268e-04) * (q[i3] * q[i9])
            + (2.287754e-03) * (q[i3] * q[i10]) + (-1.845070e-03) * (q[i3] * q[i11]) + (-1.582897e-03) * (q[i3] * q[i12]) + (1.435510e-03) * (q[i3] * q[i15])
            + (9.760053e-04) * (q[i3] * q[i16]) + (7.935073e-04) * (q[i3] * q[i19]) + (3.412249e-04) * (q[i3] * q[i20]) + (-1.788544e-04) * (q[i3] * q[i21])
            + (-2.821919e-04) * (q[i3] * q[i22]) + (2.288322e-03) * (q[i4] * q[i5]) + (-2.369617e-03) * (q[i4] * q[i6]) + (-1.188181e-03) * (q[i4] * q[i7])
            + (-1.034320e-03) * (q[i4] * q[i8]) + (5.048114e-04) * (q[i4] * q[i9]) + (3.749607e-04) * (q[i4] * q[i10]) + (3.848988e-04) * (q[i4] * q[i11])
            + (1.873872e-04) * (q[i4] * q[i12]) + (4.058198e-04) * (q[i4] * q[i15]) + (9.716004e-05) * (q[i4] * q[i16]) + (4.639175e-04) * (q[i4] * q[i19])
            + (-2.708977e-04) * (q[i4] * q[i20]) + (1.816487e-04) * (q[i4] * q[i21]) + (1.422256e-04) * (q[i4] * q[i22]) + (-7.721839e-03) * (q[i5] * q[i6])
            + (-1.202393e-03) * (q[i5] * q[i7]) + (5.711739e-04) * (q[i5] * q[i8]) + (-5.320007e-04) * (q[i5] * q[i9]) + (4.327036e-05) * (q[i5] * q[i10])
            + (1.515204e-03) * (q[i5] * q[i11]) + (-3.911966e-04) * (q[i5] * q[i12]) + (1.990528e-04) * (q[i5] * q[i15]) + (-1.860218e-04) * (q[i5] * q[i16])
            + (-5.553259e-04) * (q[i5] * q[i19]) + (1.763750e-04) * (q[i5] * q[i20]) + (5.344639e-04) * (q[i5] * q[i21]) + (-1.737749e-04) * (q[i5] * q[i22])
            + (-1.080780e-03) * (q[i6] * q[i7]) + (6.325835e-03) * (q[i6] * q[i8]) + (-1.465474e-02) * (q[i6] * q[i9]) + (3.719124e-03) * (q[i6] * q[i10])
            + (-7.742778e-04) * (q[i6] * q[i11]) + (2.973193e-04) * (q[i6] * q[i12]) + (1.582378e-03) * (q[i6] * q[i15]) + (1.714890e-03) * (q[i6] * q[i16])
            + (5.418329e-04) * (q[i6] * q[i19]) + (-3.649028e-04) * (q[i6] * q[i20]) + (-5.273413e-04) * (q[i6] * q[i21]) + (5.840159e-04) * (q[i6] * q[i22])
            + (-1.829260e-04) * (q[i7] * q[i8]) + (-1.361720e-04) * (q[i7] * q[i9]) + (1.584395e-03) * (q[i7] * q[i10]) + (-2.102670e-04) * (q[i7] * q[i11])
            + (-8.309921e-04) * (q[i7] * q[i12]) + (-4.744342e-04) * (q[i7] * q[i15]) + (-3.210253e-04) * (q[i7] * q[i16]) + (6.264215e-04) * (q[i7] * q[i19])
            + (-7.089603e-05) * (q[i7] * q[i20]) + (-1.047804e-04) * (q[i7] * q[i21]) + (-2.431557e-04) * (q[i7] * q[i22]) + (5.752271e-04) * (q[i8] * q[i9])
            + (-6.227911e-04) * (q[i8] * q[i10]) + (-5.047233e-04) * (q[i8] * q[i11]) + (6.275289e-04) * (q[i8] * q[i12]) + (4.118524e-04) * (q[i8] * q[i15])
            + (1.008221e-03) * (q[i8] * q[i16]) + (4.729393e-04) * (q[i8] * q[i19]) + (3.312547e-04) * (q[i8] * q[i20]) + (2.722731e-05) * (q[i8] * q[i21])
            + (-2.079227e-04) * (q[i8] * q[i22]) + (5.517986e-04) * (q[i9] * q[i10]) + (-8.545944e-05) * (q[i9] * q[i11]) + (3.097661e-04) * (q[i9] * q[i12])
            + (3.770447e-04) * (q[i9] * q[i15]) + (2.400579e-04) * (q[i9] * q[i16]) + (4.301947e-04) * (q[i9] * q[i19]) + (1.408991e-04) * (q[i9] * q[i20])
            + (-1.125004e-04) * (q[i9] * q[i21]) + (7.431093e-05) * (q[i9] * q[i22]) + (-6.850180e-06) * (q[i10] * q[i11]) + (4.021818e-06) * (q[i10] * q[i12])
            + (2.035634e-04) * (q[i10] * q[i15]) + (-9.682506e-05) * (q[i10] * q[i16]) + (8.105425e-05) * (q[i10] * q[i19])
            + (-3.296448e-04) * (q[i10] * q[i20]) + (-3.191838e-04) * (q[i10] * q[i21]) + (-1.513056e-04) * (q[i10] * q[i22])
            + (1.621298e-04) * (q[i11] * q[i12]) + (1.406522e-04) * (q[i11] * q[i15]) + (3.678196e-04) * (q[i11] * q[i16]) + (1.275807e-04) * (q[i11] * q[i19])
            + (6.317997e-05) * (q[i11] * q[i20]) + (2.023471e-04) * (q[i11] * q[i21]) + (-1.685427e-04) * (q[i11] * q[i22]) + (1.160218e-04) * (q[i12] * q[i15])
            + (-5.135890e-04) * (q[i12] * q[i16]) + (6.372694e-05) * (q[i12] * q[i19]) + (-1.227215e-04) * (q[i12] * q[i20])
            + (2.551483e-04) * (q[i12] * q[i21]) + (1.660179e-05) * (q[i12] * q[i22]) + (1.680045e-04) * (q[i15] * q[i16]) + (-1.696522e-04) * (q[i15] * q[i19])
            + (-8.786154e-05) * (q[i15] * q[i20]) + (4.839724e-04) * (q[i15] * q[i21]) + (9.873012e-05) * (q[i15] * q[i22])
            + (-2.713472e-04) * (q[i16] * q[i19]) + (2.116721e-04) * (q[i16] * q[i20]) + (-8.932882e-06) * (q[i16] * q[i21])
            + (-7.559066e-05) * (q[i16] * q[i22]) + (-2.635676e-05) * (q[i19] * q[i20]) + (2.077649e-04) * (q[i19] * q[i21])
            + (-4.681289e-05) * (q[i19] * q[i22]) + (1.602136e-04) * (q[i20] * q[i21]) + (1.849766e-04) * (q[i20] * q[i22])
            + (7.868737e-05) * (q[i21] * q[i22]);
      JQ[1][i1] = (-1.009222e-02) * (1) + (3.447616e-03) * ((2) * q[i1]) + (1.979567e-05) * (q[i0]) + (3.615126e-04) * (q[i2]) + (2.650327e-03) * (q[i3])
            + (5.945322e-04) * (q[i4]) + (-8.069447e-04) * (q[i5]) + (-9.712005e-03) * (q[i6]) + (9.064621e-02) * (q[i7]) + (-2.464220e-03) * (q[i8])
            + (-1.196230e-03) * (q[i9]) + (2.931251e-02) * (q[i10]) + (1.726736e-03) * (q[i11]) + (-9.807593e-04) * (q[i12]) + (1.865620e-03) * (q[i15])
            + (1.351955e-03) * (q[i16]) + (6.761685e-06) * (q[i19]) + (-9.199283e-04) * (q[i20]) + (-6.984603e-05) * (q[i21]) + (-1.504815e-03) * (q[i22])
            + (-1.628516e-03) * (q[i0] * q[i0]) + (-1.636837e-03) * ((2) * q[i0] * q[i1]) + (1.872652e-03) * ((3) * q[i1] * q[i1])
            + (1.222637e-03) * ((2) * q[i1] * q[i2]) + (-1.570535e-03) * ((2) * q[i1] * q[i3]) + (-3.006810e-02) * ((2) * q[i1] * q[i4])
            + (-2.120072e-03) * ((2) * q[i1] * q[i5]) + (3.361165e-04) * ((2) * q[i1] * q[i6]) + (-5.001626e-04) * ((2) * q[i1] * q[i7])
            + (6.299754e-04) * ((2) * q[i1] * q[i8]) + (7.155024e-04) * ((2) * q[i1] * q[i9]) + (-2.193286e-03) * ((2) * q[i1] * q[i10])
            + (5.702403e-04) * ((2) * q[i1] * q[i11]) + (2.120748e-04) * ((2) * q[i1] * q[i12]) + (-4.106177e-04) * ((2) * q[i1] * q[i15])
            + (4.250499e-04) * ((2) * q[i1] * q[i16]) + (6.595327e-04) * ((2) * q[i1] * q[i19]) + (-3.295209e-04) * ((2) * q[i1] * q[i20])
            + (-1.209851e-05) * ((2) * q[i1] * q[i21]) + (2.927667e-04) * ((2) * q[i1] * q[i22]) + (6.794451e-04) * (q[i2] * q[i2])
            + (-6.507853e-04) * (q[i3] * q[i3]) + (-2.322417e-03) * (q[i4] * q[i4]) + (-9.501622e-05) * (q[i5] * q[i5]) + (-1.896338e-03) * (q[i6] * q[i6])
            + (4.921606e-03) * (q[i7] * q[i7]) + (-1.363484e-04) * (q[i8] * q[i8]) + (1.026084e-03) * (q[i9] * q[i9]) + (-6.959196e-03) * (q[i10] * q[i10])
            + (-2.445407e-04) * (q[i11] * q[i11]) + (3.989079e-04) * (q[i12] * q[i12]) + (1.897088e-04) * (q[i15] * q[i15]) + (2.275479e-04) * (q[i16] * q[i16])
            + (-2.613781e-06) * (q[i19] * q[i19]) + (-9.771895e-05) * (q[i20] * q[i20]) + (-1.453267e-04) * (q[i21] * q[i21])
            + (1.078829e-04) * (q[i22] * q[i22]) + (-1.194385e-03) * (q[i0] * q[i2]) + (7.610132e-03) * (q[i0] * q[i3]) + (7.626269e-03) * (q[i0] * q[i4])
            + (-3.021663e-03) * (q[i0] * q[i5]) + (3.010431e-04) * (q[i0] * q[i6]) + (-2.984609e-04) * (q[i0] * q[i7]) + (7.173709e-06) * (q[i0] * q[i8])
            + (-9.985536e-04) * (q[i0] * q[i9]) + (9.903339e-04) * (q[i0] * q[i10]) + (-6.081333e-04) * (q[i0] * q[i11]) + (-6.105035e-04) * (q[i0] * q[i12])
            + (7.877042e-04) * (q[i0] * q[i15]) + (-7.890194e-04) * (q[i0] * q[i16]) + (-9.758276e-04) * (q[i0] * q[i19]) + (9.787422e-04) * (q[i0] * q[i20])
            + (-2.022340e-06) * (q[i0] * q[i21]) + (-4.565074e-06) * (q[i0] * q[i22]) + (-2.168192e-04) * (q[i2] * q[i3]) + (-1.237445e-02) * (q[i2] * q[i4])
            + (-8.393336e-03) * (q[i2] * q[i5]) + (3.588026e-04) * (q[i2] * q[i6]) + (-3.874749e-04) * (q[i2] * q[i7]) + (2.394033e-03) * (q[i2] * q[i8])
            + (2.287002e-04) * (q[i2] * q[i9]) + (-1.015867e-03) * (q[i2] * q[i10]) + (8.179783e-04) * (q[i2] * q[i11]) + (1.544878e-04) * (q[i2] * q[i12])
            + (-2.282695e-03) * (q[i2] * q[i15]) + (7.919335e-04) * (q[i2] * q[i16]) + (-8.783529e-04) * (q[i2] * q[i19]) + (-3.859424e-05) * (q[i2] * q[i20])
            + (-7.342616e-05) * (q[i2] * q[i21]) + (2.587410e-04) * (q[i2] * q[i22]) + (1.903197e-04) * (q[i3] * q[i4]) + (2.304678e-03) * (q[i3] * q[i5])
            + (1.181236e-03) * (q[i3] * q[i6]) + (2.355013e-03) * (q[i3] * q[i7]) + (1.040977e-03) * (q[i3] * q[i8]) + (-3.836647e-04) * (q[i3] * q[i9])
            + (-5.058233e-04) * (q[i3] * q[i10]) + (1.973673e-04) * (q[i3] * q[i11]) + (3.859845e-04) * (q[i3] * q[i12]) + (-1.024926e-04) * (q[i3] * q[i15])
            + (-4.060882e-04) * (q[i3] * q[i16]) + (2.787432e-04) * (q[i3] * q[i19]) + (-4.741077e-04) * (q[i3] * q[i20]) + (1.423696e-04) * (q[i3] * q[i21])
            + (1.807528e-04) * (q[i3] * q[i22]) + (3.417656e-03) * (q[i4] * q[i5]) + (-2.834066e-03) * (q[i4] * q[i6]) + (1.048470e-02) * (q[i4] * q[i7])
            + (-2.226817e-03) * (q[i4] * q[i8]) + (-2.315400e-03) * (q[i4] * q[i9]) + (8.886449e-04) * (q[i4] * q[i10]) + (-1.600283e-03) * (q[i4] * q[i11])
            + (-1.832126e-03) * (q[i4] * q[i12]) + (-9.721177e-04) * (q[i4] * q[i15]) + (-1.436457e-03) * (q[i4] * q[i16]) + (-3.493539e-04) * (q[i4] * q[i19])
            + (-7.889787e-04) * (q[i4] * q[i20]) + (-2.771836e-04) * (q[i4] * q[i21]) + (-1.891432e-04) * (q[i4] * q[i22]) + (1.183537e-03) * (q[i5] * q[i6])
            + (7.746703e-03) * (q[i5] * q[i7]) + (-5.622257e-04) * (q[i5] * q[i8]) + (-5.807911e-05) * (q[i5] * q[i9]) + (5.458980e-04) * (q[i5] * q[i10])
            + (-3.894842e-04) * (q[i5] * q[i11]) + (1.525443e-03) * (q[i5] * q[i12]) + (1.882025e-04) * (q[i5] * q[i15]) + (-2.026340e-04) * (q[i5] * q[i16])
            + (-1.674119e-04) * (q[i5] * q[i19]) + (5.510449e-04) * (q[i5] * q[i20]) + (-1.820496e-04) * (q[i5] * q[i21]) + (5.276585e-04) * (q[i5] * q[i22])
            + (-1.056407e-03) * (q[i6] * q[i7]) + (-1.799223e-04) * (q[i6] * q[i8]) + (1.606109e-03) * (q[i6] * q[i9]) + (-1.269122e-04) * (q[i6] * q[i10])
            + (8.367449e-04) * (q[i6] * q[i11]) + (2.208027e-04) * (q[i6] * q[i12]) + (-3.151395e-04) * (q[i6] * q[i15]) + (-4.786425e-04) * (q[i6] * q[i16])
            + (-7.087169e-05) * (q[i6] * q[i19]) + (6.264196e-04) * (q[i6] * q[i20]) + (2.398719e-04) * (q[i6] * q[i21]) + (1.031743e-04) * (q[i6] * q[i22])
            + (6.316686e-03) * (q[i7] * q[i8]) + (3.751410e-03) * (q[i7] * q[i9]) + (-1.448508e-02) * (q[i7] * q[i10]) + (-2.759820e-04) * (q[i7] * q[i11])
            + (7.933086e-04) * (q[i7] * q[i12]) + (1.680669e-03) * (q[i7] * q[i15]) + (1.597703e-03) * (q[i7] * q[i16]) + (-3.730352e-04) * (q[i7] * q[i19])
            + (5.390420e-04) * (q[i7] * q[i20]) + (-5.712384e-04) * (q[i7] * q[i21]) + (5.396924e-04) * (q[i7] * q[i22]) + (-6.288250e-04) * (q[i8] * q[i9])
            + (5.758056e-04) * (q[i8] * q[i10]) + (-5.762647e-04) * (q[i8] * q[i11]) + (5.423433e-04) * (q[i8] * q[i12]) + (9.890983e-04) * (q[i8] * q[i15])
            + (4.193216e-04) * (q[i8] * q[i16]) + (3.292585e-04) * (q[i8] * q[i19]) + (4.634254e-04) * (q[i8] * q[i20]) + (2.051705e-04) * (q[i8] * q[i21])
            + (-2.760091e-05) * (q[i8] * q[i22]) + (5.497590e-04) * (q[i9] * q[i10]) + (-3.580780e-06) * (q[i9] * q[i11]) + (3.420878e-06) * (q[i9] * q[i12])
            + (-9.988166e-05) * (q[i9] * q[i15]) + (2.043830e-04) * (q[i9] * q[i16]) + (-3.293673e-04) * (q[i9] * q[i19]) + (7.962626e-05) * (q[i9] * q[i20])
            + (1.490205e-04) * (q[i9] * q[i21]) + (3.217950e-04) * (q[i9] * q[i22]) + (-3.052562e-04) * (q[i10] * q[i11]) + (8.281640e-05) * (q[i10] * q[i12])
            + (2.340344e-04) * (q[i10] * q[i15]) + (3.797765e-04) * (q[i10] * q[i16]) + (1.399759e-04) * (q[i10] * q[i19]) + (4.300117e-04) * (q[i10] * q[i20])
            + (-7.301478e-05) * (q[i10] * q[i21]) + (1.116875e-04) * (q[i10] * q[i22]) + (1.603131e-04) * (q[i11] * q[i12]) + (5.024914e-04) * (q[i11] * q[i15])
            + (-1.164569e-04) * (q[i11] * q[i16]) + (1.243312e-04) * (q[i11] * q[i19]) + (-6.589012e-05) * (q[i11] * q[i20])
            + (1.314783e-05) * (q[i11] * q[i21]) + (2.562168e-04) * (q[i11] * q[i22]) + (-3.718481e-04) * (q[i12] * q[i15])
            + (-1.442976e-04) * (q[i12] * q[i16]) + (-6.553262e-05) * (q[i12] * q[i19]) + (-1.224903e-04) * (q[i12] * q[i20])
            + (-1.676125e-04) * (q[i12] * q[i21]) + (2.016792e-04) * (q[i12] * q[i22]) + (1.628166e-04) * (q[i15] * q[i16]) + (2.174304e-04) * (q[i15] * q[i19])
            + (-2.712922e-04) * (q[i15] * q[i20]) + (7.240107e-05) * (q[i15] * q[i21]) + (6.014123e-06) * (q[i15] * q[i22])
            + (-8.246948e-05) * (q[i16] * q[i19]) + (-1.668326e-04) * (q[i16] * q[i20]) + (-9.617169e-05) * (q[i16] * q[i21])
            + (-4.835072e-04) * (q[i16] * q[i22]) + (-3.140601e-05) * (q[i19] * q[i20]) + (-1.897953e-04) * (q[i19] * q[i21])
            + (-1.584384e-04) * (q[i19] * q[i22]) + (4.808083e-05) * (q[i20] * q[i21]) + (-2.088448e-04) * (q[i20] * q[i22])
            + (7.632382e-05) * (q[i21] * q[i22]);
      JQ[1][i2] = (7.404472e-04) * (1) + (-2.882641e-05) * ((2) * q[i2]) + (-3.901331e-04) * (q[i0]) + (3.615126e-04) * (q[i1]) + (-3.763725e-04) * (q[i3])
            + (3.412098e-04) * (q[i4]) + (2.775281e-05) * (q[i5]) + (1.562451e-02) * (q[i6]) + (1.549539e-02) * (q[i7]) + (-6.203577e-02) * (q[i8])
            + (1.601713e-03) * (q[i9]) + (1.589481e-03) * (q[i10]) + (5.238081e-03) * (q[i11]) + (-5.424067e-03) * (q[i12]) + (2.942736e-03) * (q[i15])
            + (3.012659e-03) * (q[i16]) + (-1.191364e-03) * (q[i19]) + (-1.200393e-03) * (q[i20]) + (4.442213e-04) * (q[i21]) + (-4.390413e-04) * (q[i22])
            + (1.209551e-03) * (q[i0] * q[i0]) + (1.222637e-03) * (q[i1] * q[i1]) + (6.810815e-04) * ((2) * q[i0] * q[i2])
            + (6.794451e-04) * ((2) * q[i1] * q[i2]) + (1.593451e-03) * ((3) * q[i2] * q[i2]) + (-8.389673e-04) * ((2) * q[i2] * q[i3])
            + (-8.240218e-04) * ((2) * q[i2] * q[i4]) + (-3.117940e-02) * ((2) * q[i2] * q[i5]) + (-2.665032e-04) * ((2) * q[i2] * q[i6])
            + (2.687083e-04) * ((2) * q[i2] * q[i7]) + (-1.720420e-05) * ((2) * q[i2] * q[i8]) + (-1.767681e-04) * ((2) * q[i2] * q[i9])
            + (1.711918e-04) * ((2) * q[i2] * q[i10]) + (2.758351e-04) * ((2) * q[i2] * q[i11]) + (2.583368e-04) * ((2) * q[i2] * q[i12])
            + (-2.525173e-03) * ((2) * q[i2] * q[i15]) + (2.560187e-03) * ((2) * q[i2] * q[i16]) + (-1.678943e-04) * ((2) * q[i2] * q[i19])
            + (1.765466e-04) * ((2) * q[i2] * q[i20]) + (4.466233e-04) * ((2) * q[i2] * q[i21]) + (4.547228e-04) * ((2) * q[i2] * q[i22])
            + (-1.292170e-03) * (q[i3] * q[i3]) + (-1.300065e-03) * (q[i4] * q[i4]) + (-2.462116e-03) * (q[i5] * q[i5]) + (9.353275e-04) * (q[i6] * q[i6])
            + (9.488334e-04) * (q[i7] * q[i7]) + (2.814786e-03) * (q[i8] * q[i8]) + (-1.823829e-03) * (q[i9] * q[i9]) + (-1.797081e-03) * (q[i10] * q[i10])
            + (2.331302e-04) * (q[i11] * q[i11]) + (1.398483e-04) * (q[i12] * q[i12]) + (1.852820e-03) * (q[i15] * q[i15]) + (1.876151e-03) * (q[i16] * q[i16])
            + (-1.497534e-04) * (q[i19] * q[i19]) + (-1.462692e-04) * (q[i20] * q[i20]) + (2.625060e-04) * (q[i21] * q[i21])
            + (2.635518e-04) * (q[i22] * q[i22]) + (-1.194385e-03) * (q[i0] * q[i1]) + (-1.244728e-02) * (q[i0] * q[i3]) + (-2.347755e-04) * (q[i0] * q[i4])
            + (-8.450503e-03) * (q[i0] * q[i5]) + (3.948689e-04) * (q[i0] * q[i6]) + (-3.649659e-04) * (q[i0] * q[i7]) + (-2.402488e-03) * (q[i0] * q[i8])
            + (1.025103e-03) * (q[i0] * q[i9]) + (-2.223396e-04) * (q[i0] * q[i10]) + (1.562511e-04) * (q[i0] * q[i11]) + (8.152518e-04) * (q[i0] * q[i12])
            + (-7.861968e-04) * (q[i0] * q[i15]) + (2.308786e-03) * (q[i0] * q[i16]) + (3.991832e-05) * (q[i0] * q[i19]) + (8.805669e-04) * (q[i0] * q[i20])
            + (2.581975e-04) * (q[i0] * q[i21]) + (-6.659603e-05) * (q[i0] * q[i22]) + (-2.168192e-04) * (q[i1] * q[i3]) + (-1.237445e-02) * (q[i1] * q[i4])
            + (-8.393336e-03) * (q[i1] * q[i5]) + (3.588026e-04) * (q[i1] * q[i6]) + (-3.874749e-04) * (q[i1] * q[i7]) + (2.394033e-03) * (q[i1] * q[i8])
            + (2.287002e-04) * (q[i1] * q[i9]) + (-1.015867e-03) * (q[i1] * q[i10]) + (8.179783e-04) * (q[i1] * q[i11]) + (1.544878e-04) * (q[i1] * q[i12])
            + (-2.282695e-03) * (q[i1] * q[i15]) + (7.919335e-04) * (q[i1] * q[i16]) + (-8.783529e-04) * (q[i1] * q[i19]) + (-3.859424e-05) * (q[i1] * q[i20])
            + (-7.342616e-05) * (q[i1] * q[i21]) + (2.587410e-04) * (q[i1] * q[i22]) + (-6.316623e-05) * (q[i3] * q[i4]) + (3.768436e-03) * (q[i3] * q[i5])
            + (-3.102689e-03) * (q[i3] * q[i6]) + (-2.698023e-04) * (q[i3] * q[i7]) + (3.073255e-03) * (q[i3] * q[i8]) + (-3.824442e-04) * (q[i3] * q[i9])
            + (2.621603e-04) * (q[i3] * q[i10]) + (-3.181467e-05) * (q[i3] * q[i11]) + (9.610482e-05) * (q[i3] * q[i12]) + (1.055586e-03) * (q[i3] * q[i15])
            + (-6.409353e-04) * (q[i3] * q[i16]) + (-5.450766e-04) * (q[i3] * q[i19]) + (-6.409211e-04) * (q[i3] * q[i20]) + (-3.747750e-05) * (q[i3] * q[i21])
            + (-2.728672e-05) * (q[i3] * q[i22]) + (3.768350e-03) * (q[i4] * q[i5]) + (2.777308e-04) * (q[i4] * q[i6]) + (3.087575e-03) * (q[i4] * q[i7])
            + (-3.044392e-03) * (q[i4] * q[i8]) + (-2.735006e-04) * (q[i4] * q[i9]) + (3.840404e-04) * (q[i4] * q[i10]) + (7.071592e-05) * (q[i4] * q[i11])
            + (-7.793834e-06) * (q[i4] * q[i12]) + (6.335989e-04) * (q[i4] * q[i15]) + (-1.068054e-03) * (q[i4] * q[i16]) + (6.371539e-04) * (q[i4] * q[i19])
            + (5.427403e-04) * (q[i4] * q[i20]) + (-2.653857e-05) * (q[i4] * q[i21]) + (-4.333677e-05) * (q[i4] * q[i22]) + (5.061598e-04) * (q[i5] * q[i6])
            + (-5.146189e-04) * (q[i5] * q[i7]) + (-3.873137e-05) * (q[i5] * q[i8]) + (1.587581e-03) * (q[i5] * q[i9]) + (-1.582219e-03) * (q[i5] * q[i10])
            + (2.228351e-03) * (q[i5] * q[i11]) + (2.197321e-03) * (q[i5] * q[i12]) + (-1.390519e-03) * (q[i5] * q[i15]) + (1.387809e-03) * (q[i5] * q[i16])
            + (1.181564e-03) * (q[i5] * q[i19]) + (-1.176193e-03) * (q[i5] * q[i20]) + (6.264919e-04) * (q[i5] * q[i21]) + (6.305250e-04) * (q[i5] * q[i22])
            + (-1.377426e-04) * (q[i6] * q[i7]) + (5.054400e-03) * (q[i6] * q[i8]) + (-3.439988e-03) * (q[i6] * q[i9]) + (1.431572e-03) * (q[i6] * q[i10])
            + (-7.221575e-04) * (q[i6] * q[i11]) + (-2.130224e-05) * (q[i6] * q[i12]) + (6.749095e-04) * (q[i6] * q[i15]) + (-4.701494e-04) * (q[i6] * q[i16])
            + (-1.506784e-04) * (q[i6] * q[i19]) + (2.140396e-04) * (q[i6] * q[i20]) + (2.169045e-04) * (q[i6] * q[i21]) + (-1.013324e-04) * (q[i6] * q[i22])
            + (5.048879e-03) * (q[i7] * q[i8]) + (1.438721e-03) * (q[i7] * q[i9]) + (-3.384361e-03) * (q[i7] * q[i10]) + (3.092087e-05) * (q[i7] * q[i11])
            + (7.329934e-04) * (q[i7] * q[i12]) + (-4.618498e-04) * (q[i7] * q[i15]) + (6.752372e-04) * (q[i7] * q[i16]) + (2.106647e-04) * (q[i7] * q[i19])
            + (-1.452176e-04) * (q[i7] * q[i20]) + (1.059136e-04) * (q[i7] * q[i21]) + (-2.113594e-04) * (q[i7] * q[i22]) + (-3.521480e-03) * (q[i8] * q[i9])
            + (-3.483055e-03) * (q[i8] * q[i10]) + (-9.728021e-04) * (q[i8] * q[i11]) + (1.148406e-03) * (q[i8] * q[i12]) + (5.609538e-03) * (q[i8] * q[i15])
            + (5.688234e-03) * (q[i8] * q[i16]) + (2.465393e-04) * (q[i8] * q[i19]) + (2.382505e-04) * (q[i8] * q[i20]) + (-3.458079e-04) * (q[i8] * q[i21])
            + (3.538491e-04) * (q[i8] * q[i22]) + (1.051741e-06) * (q[i9] * q[i10]) + (-1.846460e-04) * (q[i9] * q[i11]) + (-5.668092e-05) * (q[i9] * q[i12])
            + (5.058564e-06) * (q[i9] * q[i15]) + (-1.346369e-04) * (q[i9] * q[i16]) + (1.978261e-05) * (q[i9] * q[i19]) + (9.105248e-05) * (q[i9] * q[i20])
            + (-3.524261e-05) * (q[i9] * q[i21]) + (1.524644e-04) * (q[i9] * q[i22]) + (4.245635e-05) * (q[i10] * q[i11]) + (1.703269e-04) * (q[i10] * q[i12])
            + (-1.328026e-04) * (q[i10] * q[i15]) + (9.072092e-06) * (q[i10] * q[i16]) + (9.332384e-05) * (q[i10] * q[i19]) + (2.200496e-05) * (q[i10] * q[i20])
            + (-1.501000e-04) * (q[i10] * q[i21]) + (3.215402e-05) * (q[i10] * q[i22]) + (-3.372752e-04) * (q[i11] * q[i12])
            + (3.213017e-03) * (q[i11] * q[i15]) + (-2.787331e-04) * (q[i11] * q[i16]) + (9.466732e-04) * (q[i11] * q[i19])
            + (-2.194630e-04) * (q[i11] * q[i20]) + (7.741898e-04) * (q[i11] * q[i21]) + (-7.041495e-05) * (q[i11] * q[i22])
            + (2.604640e-04) * (q[i12] * q[i15]) + (-3.231733e-03) * (q[i12] * q[i16]) + (2.152759e-04) * (q[i12] * q[i19])
            + (-9.212270e-04) * (q[i12] * q[i20]) + (-7.210027e-05) * (q[i12] * q[i21]) + (7.730127e-04) * (q[i12] * q[i22])
            + (1.514747e-05) * (q[i15] * q[i16]) + (-7.424004e-04) * (q[i15] * q[i19]) + (-4.808458e-04) * (q[i15] * q[i20])
            + (1.176342e-05) * (q[i15] * q[i21]) + (2.144045e-04) * (q[i15] * q[i22]) + (-4.820720e-04) * (q[i16] * q[i19])
            + (-7.416021e-04) * (q[i16] * q[i20]) + (-2.151514e-04) * (q[i16] * q[i21]) + (-1.013329e-05) * (q[i16] * q[i22])
            + (7.708067e-05) * (q[i19] * q[i20]) + (-5.448979e-04) * (q[i19] * q[i21]) + (-2.062266e-04) * (q[i19] * q[i22])
            + (2.072959e-04) * (q[i20] * q[i21]) + (5.365955e-04) * (q[i20] * q[i22]) + (-1.536732e-05) * (q[i21] * q[i22]);
      JQ[1][i3] = (1.300567e-01) * (1) + (1.057298e-02) * ((2) * q[i3]) + (-6.272712e-04) * (q[i0]) + (2.650327e-03) * (q[i1]) + (-3.763725e-04) * (q[i2])
            + (1.192715e-05) * (q[i4]) + (-9.766321e-03) * (q[i5]) + (9.761725e-03) * (q[i6]) + (-4.358020e-03) * (q[i7]) + (1.316746e-03) * (q[i8])
            + (-8.968284e-03) * (q[i9]) + (5.529124e-03) * (q[i10]) + (-1.025216e-03) * (q[i11]) + (1.850945e-03) * (q[i12]) + (4.404436e-03) * (q[i15])
            + (4.746587e-03) * (q[i16]) + (5.955505e-05) * (q[i19]) + (4.284752e-04) * (q[i20]) + (-9.822362e-04) * (q[i21]) + (-7.616008e-04) * (q[i22])
            + (-3.015329e-02) * (q[i0] * q[i0]) + (-1.570535e-03) * (q[i1] * q[i1]) + (-8.389673e-04) * (q[i2] * q[i2])
            + (-2.313770e-03) * ((2) * q[i0] * q[i3]) + (-6.507853e-04) * ((2) * q[i1] * q[i3]) + (-1.292170e-03) * ((2) * q[i2] * q[i3])
            + (-2.109365e-03) * ((3) * q[i3] * q[i3]) + (-1.137334e-03) * ((2) * q[i3] * q[i4]) + (3.668307e-03) * ((2) * q[i3] * q[i5])
            + (6.596583e-04) * ((2) * q[i3] * q[i6]) + (-8.461025e-04) * ((2) * q[i3] * q[i7]) + (6.903501e-05) * ((2) * q[i3] * q[i8])
            + (-3.207326e-04) * ((2) * q[i3] * q[i9]) + (-1.570220e-05) * ((2) * q[i3] * q[i10]) + (-2.580375e-04) * ((2) * q[i3] * q[i11])
            + (-1.087573e-03) * ((2) * q[i3] * q[i12]) + (6.435341e-04) * ((2) * q[i3] * q[i15]) + (5.647602e-04) * ((2) * q[i3] * q[i16])
            + (-1.532691e-04) * ((2) * q[i3] * q[i19]) + (-1.637586e-04) * ((2) * q[i3] * q[i20]) + (8.051092e-05) * ((2) * q[i3] * q[i21])
            + (3.262428e-05) * ((2) * q[i3] * q[i22]) + (-1.137239e-03) * (q[i4] * q[i4]) + (1.856386e-03) * (q[i5] * q[i5]) + (-2.187515e-02) * (q[i6] * q[i6])
            + (2.902209e-03) * (q[i7] * q[i7]) + (4.246645e-03) * (q[i8] * q[i8]) + (-3.393812e-03) * (q[i9] * q[i9]) + (6.085741e-04) * (q[i10] * q[i10])
            + (-4.269491e-04) * (q[i11] * q[i11]) + (4.375516e-04) * (q[i12] * q[i12]) + (2.706537e-04) * (q[i15] * q[i15]) + (5.383057e-04) * (q[i16] * q[i16])
            + (-8.723253e-05) * (q[i19] * q[i19]) + (-5.818832e-06) * (q[i20] * q[i20]) + (-6.384954e-05) * (q[i21] * q[i21])
            + (6.055682e-04) * (q[i22] * q[i22]) + (7.610132e-03) * (q[i0] * q[i1]) + (-1.244728e-02) * (q[i0] * q[i2]) + (1.838885e-04) * (q[i0] * q[i4])
            + (3.433614e-03) * (q[i0] * q[i5]) + (-1.054360e-02) * (q[i0] * q[i6]) + (2.862776e-03) * (q[i0] * q[i7]) + (2.243601e-03) * (q[i0] * q[i8])
            + (-8.869268e-04) * (q[i0] * q[i9]) + (2.287754e-03) * (q[i0] * q[i10]) + (-1.845070e-03) * (q[i0] * q[i11]) + (-1.582897e-03) * (q[i0] * q[i12])
            + (1.435510e-03) * (q[i0] * q[i15]) + (9.760053e-04) * (q[i0] * q[i16]) + (7.935073e-04) * (q[i0] * q[i19]) + (3.412249e-04) * (q[i0] * q[i20])
            + (-1.788544e-04) * (q[i0] * q[i21]) + (-2.821919e-04) * (q[i0] * q[i22]) + (-2.168192e-04) * (q[i1] * q[i2]) + (1.903197e-04) * (q[i1] * q[i4])
            + (2.304678e-03) * (q[i1] * q[i5]) + (1.181236e-03) * (q[i1] * q[i6]) + (2.355013e-03) * (q[i1] * q[i7]) + (1.040977e-03) * (q[i1] * q[i8])
            + (-3.836647e-04) * (q[i1] * q[i9]) + (-5.058233e-04) * (q[i1] * q[i10]) + (1.973673e-04) * (q[i1] * q[i11]) + (3.859845e-04) * (q[i1] * q[i12])
            + (-1.024926e-04) * (q[i1] * q[i15]) + (-4.060882e-04) * (q[i1] * q[i16]) + (2.787432e-04) * (q[i1] * q[i19]) + (-4.741077e-04) * (q[i1] * q[i20])
            + (1.423696e-04) * (q[i1] * q[i21]) + (1.807528e-04) * (q[i1] * q[i22]) + (-6.316623e-05) * (q[i2] * q[i4]) + (3.768436e-03) * (q[i2] * q[i5])
            + (-3.102689e-03) * (q[i2] * q[i6]) + (-2.698023e-04) * (q[i2] * q[i7]) + (3.073255e-03) * (q[i2] * q[i8]) + (-3.824442e-04) * (q[i2] * q[i9])
            + (2.621603e-04) * (q[i2] * q[i10]) + (-3.181467e-05) * (q[i2] * q[i11]) + (9.610482e-05) * (q[i2] * q[i12]) + (1.055586e-03) * (q[i2] * q[i15])
            + (-6.409353e-04) * (q[i2] * q[i16]) + (-5.450766e-04) * (q[i2] * q[i19]) + (-6.409211e-04) * (q[i2] * q[i20]) + (-3.747750e-05) * (q[i2] * q[i21])
            + (-2.728672e-05) * (q[i2] * q[i22]) + (-5.909751e-04) * (q[i4] * q[i5]) + (1.084108e-03) * (q[i4] * q[i6]) + (-1.089899e-03) * (q[i4] * q[i7])
            + (1.260295e-06) * (q[i4] * q[i8]) + (-4.328217e-04) * (q[i4] * q[i9]) + (4.310851e-04) * (q[i4] * q[i10]) + (2.821409e-04) * (q[i4] * q[i11])
            + (2.781655e-04) * (q[i4] * q[i12]) + (3.733548e-04) * (q[i4] * q[i15]) + (-3.644859e-04) * (q[i4] * q[i16]) + (1.639802e-04) * (q[i4] * q[i19])
            + (-1.619049e-04) * (q[i4] * q[i20]) + (-7.005385e-04) * (q[i4] * q[i21]) + (-7.129517e-04) * (q[i4] * q[i22]) + (-1.677210e-03) * (q[i5] * q[i6])
            + (-1.603558e-04) * (q[i5] * q[i7]) + (-2.791834e-04) * (q[i5] * q[i8]) + (2.351077e-04) * (q[i5] * q[i9]) + (-9.716475e-05) * (q[i5] * q[i10])
            + (-1.227979e-03) * (q[i5] * q[i11]) + (6.743793e-04) * (q[i5] * q[i12]) + (8.657755e-04) * (q[i5] * q[i15]) + (-3.537567e-04) * (q[i5] * q[i16])
            + (3.344671e-04) * (q[i5] * q[i19]) + (-6.595843e-04) * (q[i5] * q[i20]) + (3.193782e-04) * (q[i5] * q[i21]) + (-7.617263e-06) * (q[i5] * q[i22])
            + (-1.257061e-04) * (q[i6] * q[i7]) + (-1.520813e-03) * (q[i6] * q[i8]) + (-1.066384e-02) * (q[i6] * q[i9]) + (6.920992e-04) * (q[i6] * q[i10])
            + (-2.072890e-04) * (q[i6] * q[i11]) + (-2.334922e-05) * (q[i6] * q[i12]) + (2.047130e-04) * (q[i6] * q[i15]) + (-9.212668e-04) * (q[i6] * q[i16])
            + (-5.412548e-04) * (q[i6] * q[i19]) + (5.468548e-04) * (q[i6] * q[i20]) + (4.925865e-06) * (q[i6] * q[i21]) + (4.098744e-04) * (q[i6] * q[i22])
            + (2.248550e-04) * (q[i7] * q[i8]) + (2.845153e-04) * (q[i7] * q[i9]) + (1.740196e-03) * (q[i7] * q[i10]) + (-3.172781e-04) * (q[i7] * q[i11])
            + (6.034648e-04) * (q[i7] * q[i12]) + (-2.640983e-04) * (q[i7] * q[i15]) + (-3.420845e-04) * (q[i7] * q[i16]) + (-3.583018e-04) * (q[i7] * q[i19])
            + (2.250667e-05) * (q[i7] * q[i20]) + (1.451721e-04) * (q[i7] * q[i21]) + (1.780640e-04) * (q[i7] * q[i22]) + (-9.315922e-05) * (q[i8] * q[i9])
            + (4.078081e-04) * (q[i8] * q[i10]) + (-8.162157e-04) * (q[i8] * q[i11]) + (5.571022e-04) * (q[i8] * q[i12]) + (-5.793535e-04) * (q[i8] * q[i15])
            + (-5.885023e-04) * (q[i8] * q[i16]) + (-2.736758e-04) * (q[i8] * q[i19]) + (-2.506133e-04) * (q[i8] * q[i20]) + (3.583714e-04) * (q[i8] * q[i21])
            + (-3.718586e-05) * (q[i8] * q[i22]) + (-4.873547e-04) * (q[i9] * q[i10]) + (-1.778195e-04) * (q[i9] * q[i11]) + (-2.674895e-04) * (q[i9] * q[i12])
            + (-2.490409e-04) * (q[i9] * q[i15]) + (-6.474482e-04) * (q[i9] * q[i16]) + (-2.530471e-04) * (q[i9] * q[i19]) + (5.058323e-05) * (q[i9] * q[i20])
            + (1.621689e-04) * (q[i9] * q[i21]) + (-2.972290e-04) * (q[i9] * q[i22]) + (-1.247490e-04) * (q[i10] * q[i11]) + (3.478084e-04) * (q[i10] * q[i12])
            + (1.607090e-04) * (q[i10] * q[i15]) + (1.095946e-04) * (q[i10] * q[i16]) + (6.130547e-05) * (q[i10] * q[i19]) + (1.567877e-04) * (q[i10] * q[i20])
            + (-2.241620e-04) * (q[i10] * q[i21]) + (2.002785e-04) * (q[i10] * q[i22]) + (-1.237553e-05) * (q[i11] * q[i12])
            + (-9.387109e-04) * (q[i11] * q[i15]) + (4.190322e-04) * (q[i11] * q[i16]) + (1.874694e-04) * (q[i11] * q[i19])
            + (-1.119682e-04) * (q[i11] * q[i20]) + (-3.660009e-04) * (q[i11] * q[i21]) + (-9.408255e-05) * (q[i11] * q[i22])
            + (5.437707e-04) * (q[i12] * q[i15]) + (-1.435395e-04) * (q[i12] * q[i16]) + (-1.542441e-04) * (q[i12] * q[i19])
            + (3.180409e-04) * (q[i12] * q[i20]) + (-4.124738e-05) * (q[i12] * q[i21]) + (-2.224609e-04) * (q[i12] * q[i22])
            + (2.630016e-04) * (q[i15] * q[i16]) + (-2.487628e-04) * (q[i15] * q[i19]) + (8.713109e-05) * (q[i15] * q[i20]) + (3.401230e-04) * (q[i15] * q[i21])
            + (1.075916e-05) * (q[i15] * q[i22]) + (-2.486897e-05) * (q[i16] * q[i19]) + (1.707353e-04) * (q[i16] * q[i20])
            + (-2.294937e-04) * (q[i16] * q[i21]) + (-6.333649e-04) * (q[i16] * q[i22]) + (2.075815e-05) * (q[i19] * q[i20])
            + (3.289395e-04) * (q[i19] * q[i21]) + (-7.843576e-05) * (q[i19] * q[i22]) + (-1.167296e-04) * (q[i20] * q[i21])
            + (-6.593001e-04) * (q[i20] * q[i22]) + (5.306985e-05) * (q[i21] * q[i22]);
      JQ[1][i4] = (1.293915e-01) * (1) + (-1.056696e-02) * ((2) * q[i4]) + (-2.647624e-03) * (q[i0]) + (5.945322e-04) * (q[i1]) + (3.412098e-04) * (q[i2])
            + (1.192715e-05) * (q[i3]) + (9.779998e-03) * (q[i5]) + (-4.360963e-03) * (q[i6]) + (9.714756e-03) * (q[i7]) + (1.315402e-03) * (q[i8])
            + (5.551080e-03) * (q[i9]) + (-8.885331e-03) * (q[i10]) + (-1.811008e-03) * (q[i11]) + (1.046608e-03) * (q[i12]) + (4.666836e-03) * (q[i15])
            + (4.427214e-03) * (q[i16]) + (4.169070e-04) * (q[i19]) + (5.590000e-05) * (q[i20]) + (7.599458e-04) * (q[i21]) + (9.896619e-04) * (q[i22])
            + (-1.588441e-03) * (q[i0] * q[i0]) + (-3.006810e-02) * (q[i1] * q[i1]) + (-8.240218e-04) * (q[i2] * q[i2]) + (-1.137334e-03) * (q[i3] * q[i3])
            + (-6.343227e-04) * ((2) * q[i0] * q[i4]) + (-2.322417e-03) * ((2) * q[i1] * q[i4]) + (-1.300065e-03) * ((2) * q[i2] * q[i4])
            + (-1.137239e-03) * ((2) * q[i3] * q[i4]) + (-2.122879e-03) * ((3) * q[i4] * q[i4]) + (3.662696e-03) * ((2) * q[i4] * q[i5])
            + (8.388854e-04) * ((2) * q[i4] * q[i6]) + (-6.562441e-04) * ((2) * q[i4] * q[i7]) + (-7.354570e-05) * ((2) * q[i4] * q[i8])
            + (8.988663e-06) * ((2) * q[i4] * q[i9]) + (3.175588e-04) * ((2) * q[i4] * q[i10]) + (-1.083834e-03) * ((2) * q[i4] * q[i11])
            + (-2.591561e-04) * ((2) * q[i4] * q[i12]) + (-5.605486e-04) * ((2) * q[i4] * q[i15]) + (-6.537892e-04) * ((2) * q[i4] * q[i16])
            + (1.698040e-04) * ((2) * q[i4] * q[i19]) + (1.458927e-04) * ((2) * q[i4] * q[i20]) + (3.258086e-05) * ((2) * q[i4] * q[i21])
            + (7.933015e-05) * ((2) * q[i4] * q[i22]) + (1.872767e-03) * (q[i5] * q[i5]) + (2.909161e-03) * (q[i6] * q[i6]) + (-2.181574e-02) * (q[i7] * q[i7])
            + (4.248400e-03) * (q[i8] * q[i8]) + (6.176945e-04) * (q[i9] * q[i9]) + (-3.353310e-03) * (q[i10] * q[i10]) + (4.152397e-04) * (q[i11] * q[i11])
            + (-4.336171e-04) * (q[i12] * q[i12]) + (5.376847e-04) * (q[i15] * q[i15]) + (2.743125e-04) * (q[i16] * q[i16])
            + (-1.017205e-06) * (q[i19] * q[i19]) + (-8.774236e-05) * (q[i20] * q[i20]) + (6.024040e-04) * (q[i21] * q[i21])
            + (-6.108601e-05) * (q[i22] * q[i22]) + (7.626269e-03) * (q[i0] * q[i1]) + (-2.347755e-04) * (q[i0] * q[i2]) + (1.838885e-04) * (q[i0] * q[i3])
            + (2.288322e-03) * (q[i0] * q[i5]) + (-2.369617e-03) * (q[i0] * q[i6]) + (-1.188181e-03) * (q[i0] * q[i7]) + (-1.034320e-03) * (q[i0] * q[i8])
            + (5.048114e-04) * (q[i0] * q[i9]) + (3.749607e-04) * (q[i0] * q[i10]) + (3.848988e-04) * (q[i0] * q[i11]) + (1.873872e-04) * (q[i0] * q[i12])
            + (4.058198e-04) * (q[i0] * q[i15]) + (9.716004e-05) * (q[i0] * q[i16]) + (4.639175e-04) * (q[i0] * q[i19]) + (-2.708977e-04) * (q[i0] * q[i20])
            + (1.816487e-04) * (q[i0] * q[i21]) + (1.422256e-04) * (q[i0] * q[i22]) + (-1.237445e-02) * (q[i1] * q[i2]) + (1.903197e-04) * (q[i1] * q[i3])
            + (3.417656e-03) * (q[i1] * q[i5]) + (-2.834066e-03) * (q[i1] * q[i6]) + (1.048470e-02) * (q[i1] * q[i7]) + (-2.226817e-03) * (q[i1] * q[i8])
            + (-2.315400e-03) * (q[i1] * q[i9]) + (8.886449e-04) * (q[i1] * q[i10]) + (-1.600283e-03) * (q[i1] * q[i11]) + (-1.832126e-03) * (q[i1] * q[i12])
            + (-9.721177e-04) * (q[i1] * q[i15]) + (-1.436457e-03) * (q[i1] * q[i16]) + (-3.493539e-04) * (q[i1] * q[i19]) + (-7.889787e-04) * (q[i1] * q[i20])
            + (-2.771836e-04) * (q[i1] * q[i21]) + (-1.891432e-04) * (q[i1] * q[i22]) + (-6.316623e-05) * (q[i2] * q[i3]) + (3.768350e-03) * (q[i2] * q[i5])
            + (2.777308e-04) * (q[i2] * q[i6]) + (3.087575e-03) * (q[i2] * q[i7]) + (-3.044392e-03) * (q[i2] * q[i8]) + (-2.735006e-04) * (q[i2] * q[i9])
            + (3.840404e-04) * (q[i2] * q[i10]) + (7.071592e-05) * (q[i2] * q[i11]) + (-7.793834e-06) * (q[i2] * q[i12]) + (6.335989e-04) * (q[i2] * q[i15])
            + (-1.068054e-03) * (q[i2] * q[i16]) + (6.371539e-04) * (q[i2] * q[i19]) + (5.427403e-04) * (q[i2] * q[i20]) + (-2.653857e-05) * (q[i2] * q[i21])
            + (-4.333677e-05) * (q[i2] * q[i22]) + (-5.909751e-04) * (q[i3] * q[i5]) + (1.084108e-03) * (q[i3] * q[i6]) + (-1.089899e-03) * (q[i3] * q[i7])
            + (1.260295e-06) * (q[i3] * q[i8]) + (-4.328217e-04) * (q[i3] * q[i9]) + (4.310851e-04) * (q[i3] * q[i10]) + (2.821409e-04) * (q[i3] * q[i11])
            + (2.781655e-04) * (q[i3] * q[i12]) + (3.733548e-04) * (q[i3] * q[i15]) + (-3.644859e-04) * (q[i3] * q[i16]) + (1.639802e-04) * (q[i3] * q[i19])
            + (-1.619049e-04) * (q[i3] * q[i20]) + (-7.005385e-04) * (q[i3] * q[i21]) + (-7.129517e-04) * (q[i3] * q[i22]) + (1.423295e-04) * (q[i5] * q[i6])
            + (1.695603e-03) * (q[i5] * q[i7]) + (2.818257e-04) * (q[i5] * q[i8]) + (9.712108e-05) * (q[i5] * q[i9]) + (-2.354443e-04) * (q[i5] * q[i10])
            + (6.574422e-04) * (q[i5] * q[i11]) + (-1.211321e-03) * (q[i5] * q[i12]) + (3.483358e-04) * (q[i5] * q[i15]) + (-8.730152e-04) * (q[i5] * q[i16])
            + (6.492198e-04) * (q[i5] * q[i19]) + (-3.338799e-04) * (q[i5] * q[i20]) + (-1.473504e-05) * (q[i5] * q[i21]) + (3.263962e-04) * (q[i5] * q[i22])
            + (-1.416828e-04) * (q[i6] * q[i7]) + (2.296334e-04) * (q[i6] * q[i8]) + (1.746406e-03) * (q[i6] * q[i9]) + (2.863477e-04) * (q[i6] * q[i10])
            + (-5.990457e-04) * (q[i6] * q[i11]) + (3.041727e-04) * (q[i6] * q[i12]) + (-3.383068e-04) * (q[i6] * q[i15]) + (-2.718924e-04) * (q[i6] * q[i16])
            + (2.185914e-05) * (q[i6] * q[i19]) + (-3.518379e-04) * (q[i6] * q[i20]) + (-1.800077e-04) * (q[i6] * q[i21]) + (-1.451111e-04) * (q[i6] * q[i22])
            + (-1.525328e-03) * (q[i7] * q[i8]) + (7.122502e-04) * (q[i7] * q[i9]) + (-1.058271e-02) * (q[i7] * q[i10]) + (2.784186e-05) * (q[i7] * q[i11])
            + (2.025499e-04) * (q[i7] * q[i12]) + (-9.273298e-04) * (q[i7] * q[i15]) + (2.176679e-04) * (q[i7] * q[i16]) + (5.446240e-04) * (q[i7] * q[i19])
            + (-5.295054e-04) * (q[i7] * q[i20]) + (-4.123952e-04) * (q[i7] * q[i21]) + (4.186780e-06) * (q[i7] * q[i22]) + (4.080237e-04) * (q[i8] * q[i9])
            + (-9.400196e-05) * (q[i8] * q[i10]) + (-5.216702e-04) * (q[i8] * q[i11]) + (8.343474e-04) * (q[i8] * q[i12]) + (-5.722809e-04) * (q[i8] * q[i15])
            + (-5.842616e-04) * (q[i8] * q[i16]) + (-2.530987e-04) * (q[i8] * q[i19]) + (-2.619990e-04) * (q[i8] * q[i20]) + (3.014335e-05) * (q[i8] * q[i21])
            + (-3.593076e-04) * (q[i8] * q[i22]) + (-4.884575e-04) * (q[i9] * q[i10]) + (-3.400631e-04) * (q[i9] * q[i11]) + (1.283953e-04) * (q[i9] * q[i12])
            + (1.065911e-04) * (q[i9] * q[i15]) + (1.631917e-04) * (q[i9] * q[i16]) + (1.529120e-04) * (q[i9] * q[i19]) + (6.564241e-05) * (q[i9] * q[i20])
            + (-2.000379e-04) * (q[i9] * q[i21]) + (2.217890e-04) * (q[i9] * q[i22]) + (2.572872e-04) * (q[i10] * q[i11]) + (1.697263e-04) * (q[i10] * q[i12])
            + (-6.359112e-04) * (q[i10] * q[i15]) + (-2.492120e-04) * (q[i10] * q[i16]) + (4.953959e-05) * (q[i10] * q[i19])
            + (-2.550075e-04) * (q[i10] * q[i20]) + (2.937772e-04) * (q[i10] * q[i21]) + (-1.621648e-04) * (q[i10] * q[i22])
            + (-8.308391e-06) * (q[i11] * q[i12]) + (1.431981e-04) * (q[i11] * q[i15]) + (-5.401505e-04) * (q[i11] * q[i16])
            + (-3.156915e-04) * (q[i11] * q[i19]) + (1.557434e-04) * (q[i11] * q[i20]) + (-2.194707e-04) * (q[i11] * q[i21])
            + (-3.941864e-05) * (q[i11] * q[i22]) + (-4.132159e-04) * (q[i12] * q[i15]) + (9.410504e-04) * (q[i12] * q[i16])
            + (1.068496e-04) * (q[i12] * q[i19]) + (-1.928976e-04) * (q[i12] * q[i20]) + (-9.924322e-05) * (q[i12] * q[i21])
            + (-3.666046e-04) * (q[i12] * q[i22]) + (2.579163e-04) * (q[i15] * q[i16]) + (1.673949e-04) * (q[i15] * q[i19])
            + (-2.553386e-05) * (q[i15] * q[i20]) + (6.246916e-04) * (q[i15] * q[i21]) + (2.248961e-04) * (q[i15] * q[i22]) + (8.093611e-05) * (q[i16] * q[i19])
            + (-2.527976e-04) * (q[i16] * q[i20]) + (-9.233878e-06) * (q[i16] * q[i21]) + (-3.416751e-04) * (q[i16] * q[i22])
            + (2.110874e-05) * (q[i19] * q[i20]) + (6.592995e-04) * (q[i19] * q[i21]) + (1.175819e-04) * (q[i19] * q[i22]) + (7.482038e-05) * (q[i20] * q[i21])
            + (-3.308207e-04) * (q[i20] * q[i22]) + (5.319872e-05) * (q[i21] * q[i22]);
      JQ[1][i5] = (1.283252e-01) * (1) + (-3.585542e-05) * ((2) * q[i5]) + (7.964863e-04) * (q[i0]) + (-8.069447e-04) * (q[i1]) + (2.775281e-05) * (q[i2])
            + (-9.766321e-03) * (q[i3]) + (9.779998e-03) * (q[i4]) + (-2.953629e-04) * (q[i6]) + (-2.761619e-04) * (q[i7]) + (-1.736126e-04) * (q[i8])
            + (2.606375e-03) * (q[i9]) + (2.578295e-03) * (q[i10]) + (6.094528e-03) * (q[i11]) + (-6.165370e-03) * (q[i12]) + (-1.554917e-02) * (q[i15])
            + (-1.573297e-02) * (q[i16]) + (-9.544767e-04) * (q[i19]) + (-9.681243e-04) * (q[i20]) + (-9.370004e-04) * (q[i21]) + (9.305713e-04) * (q[i22])
            + (-2.144201e-03) * (q[i0] * q[i0]) + (-2.120072e-03) * (q[i1] * q[i1]) + (-3.117940e-02) * (q[i2] * q[i2]) + (3.668307e-03) * (q[i3] * q[i3])
            + (3.662696e-03) * (q[i4] * q[i4]) + (-8.933798e-05) * ((2) * q[i0] * q[i5]) + (-9.501622e-05) * ((2) * q[i1] * q[i5])
            + (-2.462116e-03) * ((2) * q[i2] * q[i5]) + (1.856386e-03) * ((2) * q[i3] * q[i5]) + (1.872767e-03) * ((2) * q[i4] * q[i5])
            + (-3.547081e-03) * ((3) * q[i5] * q[i5]) + (8.198076e-04) * ((2) * q[i5] * q[i6]) + (-8.280944e-04) * ((2) * q[i5] * q[i7])
            + (-1.637447e-05) * ((2) * q[i5] * q[i8]) + (-1.758869e-04) * ((2) * q[i5] * q[i9]) + (1.866823e-04) * ((2) * q[i5] * q[i10])
            + (1.824046e-05) * ((2) * q[i5] * q[i11]) + (3.115890e-06) * ((2) * q[i5] * q[i12]) + (6.431482e-07) * ((2) * q[i5] * q[i15])
            + (1.903344e-06) * ((2) * q[i5] * q[i16]) + (7.653206e-05) * ((2) * q[i5] * q[i19]) + (-7.396116e-05) * ((2) * q[i5] * q[i20])
            + (-1.678756e-04) * ((2) * q[i5] * q[i21]) + (-1.628696e-04) * ((2) * q[i5] * q[i22]) + (6.457586e-03) * (q[i6] * q[i6])
            + (6.444140e-03) * (q[i7] * q[i7]) + (-2.342988e-02) * (q[i8] * q[i8]) + (1.423103e-03) * (q[i9] * q[i9]) + (1.408923e-03) * (q[i10] * q[i10])
            + (-1.022053e-03) * (q[i11] * q[i11]) + (-1.094256e-03) * (q[i12] * q[i12]) + (-2.289652e-03) * (q[i15] * q[i15])
            + (-2.306933e-03) * (q[i16] * q[i16]) + (-2.370621e-05) * (q[i19] * q[i19]) + (-1.470971e-05) * (q[i20] * q[i20])
            + (-1.243868e-03) * (q[i21] * q[i21]) + (-1.245705e-03) * (q[i22] * q[i22]) + (-3.021663e-03) * (q[i0] * q[i1]) + (-8.450503e-03) * (q[i0] * q[i2])
            + (3.433614e-03) * (q[i0] * q[i3]) + (2.288322e-03) * (q[i0] * q[i4]) + (-7.721839e-03) * (q[i0] * q[i6]) + (-1.202393e-03) * (q[i0] * q[i7])
            + (5.711739e-04) * (q[i0] * q[i8]) + (-5.320007e-04) * (q[i0] * q[i9]) + (4.327036e-05) * (q[i0] * q[i10]) + (1.515204e-03) * (q[i0] * q[i11])
            + (-3.911966e-04) * (q[i0] * q[i12]) + (1.990528e-04) * (q[i0] * q[i15]) + (-1.860218e-04) * (q[i0] * q[i16]) + (-5.553259e-04) * (q[i0] * q[i19])
            + (1.763750e-04) * (q[i0] * q[i20]) + (5.344639e-04) * (q[i0] * q[i21]) + (-1.737749e-04) * (q[i0] * q[i22]) + (-8.393336e-03) * (q[i1] * q[i2])
            + (2.304678e-03) * (q[i1] * q[i3]) + (3.417656e-03) * (q[i1] * q[i4]) + (1.183537e-03) * (q[i1] * q[i6]) + (7.746703e-03) * (q[i1] * q[i7])
            + (-5.622257e-04) * (q[i1] * q[i8]) + (-5.807911e-05) * (q[i1] * q[i9]) + (5.458980e-04) * (q[i1] * q[i10]) + (-3.894842e-04) * (q[i1] * q[i11])
            + (1.525443e-03) * (q[i1] * q[i12]) + (1.882025e-04) * (q[i1] * q[i15]) + (-2.026340e-04) * (q[i1] * q[i16]) + (-1.674119e-04) * (q[i1] * q[i19])
            + (5.510449e-04) * (q[i1] * q[i20]) + (-1.820496e-04) * (q[i1] * q[i21]) + (5.276585e-04) * (q[i1] * q[i22]) + (3.768436e-03) * (q[i2] * q[i3])
            + (3.768350e-03) * (q[i2] * q[i4]) + (5.061598e-04) * (q[i2] * q[i6]) + (-5.146189e-04) * (q[i2] * q[i7]) + (-3.873137e-05) * (q[i2] * q[i8])
            + (1.587581e-03) * (q[i2] * q[i9]) + (-1.582219e-03) * (q[i2] * q[i10]) + (2.228351e-03) * (q[i2] * q[i11]) + (2.197321e-03) * (q[i2] * q[i12])
            + (-1.390519e-03) * (q[i2] * q[i15]) + (1.387809e-03) * (q[i2] * q[i16]) + (1.181564e-03) * (q[i2] * q[i19]) + (-1.176193e-03) * (q[i2] * q[i20])
            + (6.264919e-04) * (q[i2] * q[i21]) + (6.305250e-04) * (q[i2] * q[i22]) + (-5.909751e-04) * (q[i3] * q[i4]) + (-1.677210e-03) * (q[i3] * q[i6])
            + (-1.603558e-04) * (q[i3] * q[i7]) + (-2.791834e-04) * (q[i3] * q[i8]) + (2.351077e-04) * (q[i3] * q[i9]) + (-9.716475e-05) * (q[i3] * q[i10])
            + (-1.227979e-03) * (q[i3] * q[i11]) + (6.743793e-04) * (q[i3] * q[i12]) + (8.657755e-04) * (q[i3] * q[i15]) + (-3.537567e-04) * (q[i3] * q[i16])
            + (3.344671e-04) * (q[i3] * q[i19]) + (-6.595843e-04) * (q[i3] * q[i20]) + (3.193782e-04) * (q[i3] * q[i21]) + (-7.617263e-06) * (q[i3] * q[i22])
            + (1.423295e-04) * (q[i4] * q[i6]) + (1.695603e-03) * (q[i4] * q[i7]) + (2.818257e-04) * (q[i4] * q[i8]) + (9.712108e-05) * (q[i4] * q[i9])
            + (-2.354443e-04) * (q[i4] * q[i10]) + (6.574422e-04) * (q[i4] * q[i11]) + (-1.211321e-03) * (q[i4] * q[i12]) + (3.483358e-04) * (q[i4] * q[i15])
            + (-8.730152e-04) * (q[i4] * q[i16]) + (6.492198e-04) * (q[i4] * q[i19]) + (-3.338799e-04) * (q[i4] * q[i20]) + (-1.473504e-05) * (q[i4] * q[i21])
            + (3.263962e-04) * (q[i4] * q[i22]) + (1.751704e-04) * (q[i6] * q[i7]) + (7.352272e-05) * (q[i6] * q[i8]) + (2.487281e-03) * (q[i6] * q[i9])
            + (-1.403109e-03) * (q[i6] * q[i10]) + (-2.555128e-04) * (q[i6] * q[i11]) + (-5.092444e-04) * (q[i6] * q[i12]) + (-1.719568e-05) * (q[i6] * q[i15])
            + (1.081003e-04) * (q[i6] * q[i16]) + (1.491224e-04) * (q[i6] * q[i19]) + (6.341876e-04) * (q[i6] * q[i20]) + (-3.825242e-04) * (q[i6] * q[i21])
            + (-2.843814e-04) * (q[i6] * q[i22]) + (7.377251e-05) * (q[i7] * q[i8]) + (-1.413646e-03) * (q[i7] * q[i9]) + (2.477301e-03) * (q[i7] * q[i10])
            + (5.000242e-04) * (q[i7] * q[i11]) + (2.595822e-04) * (q[i7] * q[i12]) + (1.080666e-04) * (q[i7] * q[i15]) + (-2.874128e-05) * (q[i7] * q[i16])
            + (6.361853e-04) * (q[i7] * q[i19]) + (1.472312e-04) * (q[i7] * q[i20]) + (2.908408e-04) * (q[i7] * q[i21]) + (3.741623e-04) * (q[i7] * q[i22])
            + (-1.799927e-03) * (q[i8] * q[i9]) + (-1.784532e-03) * (q[i8] * q[i10]) + (2.327999e-03) * (q[i8] * q[i11]) + (-2.406813e-03) * (q[i8] * q[i12])
            + (2.348362e-03) * (q[i8] * q[i15]) + (2.398485e-03) * (q[i8] * q[i16]) + (5.959096e-04) * (q[i8] * q[i19]) + (5.773746e-04) * (q[i8] * q[i20])
            + (1.498457e-04) * (q[i8] * q[i21]) + (-1.440720e-04) * (q[i8] * q[i22]) + (-2.438288e-06) * (q[i9] * q[i10]) + (8.956663e-05) * (q[i9] * q[i11])
            + (-2.005148e-04) * (q[i9] * q[i12]) + (-3.390928e-04) * (q[i9] * q[i15]) + (-2.962105e-04) * (q[i9] * q[i16]) + (3.594022e-04) * (q[i9] * q[i19])
            + (4.230960e-05) * (q[i9] * q[i20]) + (4.788983e-05) * (q[i9] * q[i21]) + (1.136154e-06) * (q[i9] * q[i22]) + (1.925860e-04) * (q[i10] * q[i11])
            + (-9.604946e-05) * (q[i10] * q[i12]) + (-2.919496e-04) * (q[i10] * q[i15]) + (-3.412088e-04) * (q[i10] * q[i16])
            + (4.502975e-05) * (q[i10] * q[i19]) + (3.584510e-04) * (q[i10] * q[i20]) + (1.194787e-06) * (q[i10] * q[i21]) + (-5.287167e-05) * (q[i10] * q[i22])
            + (4.851027e-05) * (q[i11] * q[i12]) + (1.888604e-03) * (q[i11] * q[i15]) + (4.896294e-05) * (q[i11] * q[i16]) + (8.125725e-04) * (q[i11] * q[i19])
            + (-3.515285e-04) * (q[i11] * q[i20]) + (7.588162e-04) * (q[i11] * q[i21]) + (-1.558879e-04) * (q[i11] * q[i22])
            + (-6.515054e-05) * (q[i12] * q[i15]) + (-1.907801e-03) * (q[i12] * q[i16]) + (3.512644e-04) * (q[i12] * q[i19])
            + (-8.135706e-04) * (q[i12] * q[i20]) + (-1.486958e-04) * (q[i12] * q[i21]) + (7.760778e-04) * (q[i12] * q[i22])
            + (-4.701687e-04) * (q[i15] * q[i16]) + (-1.713542e-04) * (q[i15] * q[i19]) + (-2.484216e-04) * (q[i15] * q[i20])
            + (-1.837282e-03) * (q[i15] * q[i21]) + (-1.835453e-04) * (q[i15] * q[i22]) + (-2.437812e-04) * (q[i16] * q[i19])
            + (-1.624382e-04) * (q[i16] * q[i20]) + (1.821656e-04) * (q[i16] * q[i21]) + (1.859405e-03) * (q[i16] * q[i22]) + (6.759754e-04) * (q[i19] * q[i20])
            + (-1.579017e-03) * (q[i19] * q[i21]) + (-1.745701e-04) * (q[i19] * q[i22]) + (1.759727e-04) * (q[i20] * q[i21])
            + (1.582590e-03) * (q[i20] * q[i22]) + (-4.213042e-04) * (q[i21] * q[i22]);
      JQ[1][i6] = (2.919479e-03) * (1) + (1.914971e-02) * ((2) * q[i6]) + (9.116774e-02) * (q[i0]) + (-9.712005e-03) * (q[i1]) + (1.562451e-02) * (q[i2])
            + (9.761725e-03) * (q[i3]) + (-4.360963e-03) * (q[i4]) + (-2.953629e-04) * (q[i5]) + (-4.590787e-05) * (q[i7]) + (1.097971e-03) * (q[i8])
            + (9.624614e-03) * (q[i9]) + (-2.292030e-03) * (q[i10]) + (2.714796e-03) * (q[i11]) + (4.332843e-03) * (q[i12]) + (-7.229995e-04) * (q[i15])
            + (-1.151517e-03) * (q[i16]) + (4.443123e-04) * (q[i19]) + (-3.447312e-04) * (q[i20]) + (-3.226140e-04) * (q[i21]) + (-3.170217e-04) * (q[i22])
            + (4.955900e-04) * (q[i0] * q[i0]) + (3.361165e-04) * (q[i1] * q[i1]) + (-2.665032e-04) * (q[i2] * q[i2]) + (6.596583e-04) * (q[i3] * q[i3])
            + (8.388854e-04) * (q[i4] * q[i4]) + (8.198076e-04) * (q[i5] * q[i5]) + (4.884875e-03) * ((2) * q[i0] * q[i6])
            + (-1.896338e-03) * ((2) * q[i1] * q[i6]) + (9.353275e-04) * ((2) * q[i2] * q[i6]) + (-2.187515e-02) * ((2) * q[i3] * q[i6])
            + (2.909161e-03) * ((2) * q[i4] * q[i6]) + (6.457586e-03) * ((2) * q[i5] * q[i6]) + (3.453611e-04) * ((3) * q[i6] * q[i6])
            + (-8.446166e-04) * ((2) * q[i6] * q[i7]) + (6.126582e-05) * ((2) * q[i6] * q[i8]) + (-2.393264e-03) * ((2) * q[i6] * q[i9])
            + (5.680422e-05) * ((2) * q[i6] * q[i10]) + (1.943749e-04) * ((2) * q[i6] * q[i11]) + (-1.374691e-04) * ((2) * q[i6] * q[i12])
            + (-4.760048e-05) * ((2) * q[i6] * q[i15]) + (4.921217e-04) * ((2) * q[i6] * q[i16]) + (8.821549e-05) * ((2) * q[i6] * q[i19])
            + (9.985464e-05) * ((2) * q[i6] * q[i20]) + (-3.543190e-05) * ((2) * q[i6] * q[i21]) + (-1.175586e-04) * ((2) * q[i6] * q[i22])
            + (8.407773e-04) * (q[i7] * q[i7]) + (-1.323027e-03) * (q[i8] * q[i8]) + (-2.416989e-03) * (q[i9] * q[i9]) + (-5.953001e-04) * (q[i10] * q[i10])
            + (-3.015865e-04) * (q[i11] * q[i11]) + (3.419038e-04) * (q[i12] * q[i12]) + (-1.665728e-05) * (q[i15] * q[i15])
            + (2.869172e-05) * (q[i16] * q[i16]) + (-7.604286e-05) * (q[i19] * q[i19]) + (-4.128587e-05) * (q[i20] * q[i20])
            + (-5.489398e-05) * (q[i21] * q[i21]) + (1.492691e-04) * (q[i22] * q[i22]) + (3.010431e-04) * (q[i0] * q[i1]) + (3.948689e-04) * (q[i0] * q[i2])
            + (-1.054360e-02) * (q[i0] * q[i3]) + (-2.369617e-03) * (q[i0] * q[i4]) + (-7.721839e-03) * (q[i0] * q[i5]) + (-1.080780e-03) * (q[i0] * q[i7])
            + (6.325835e-03) * (q[i0] * q[i8]) + (-1.465474e-02) * (q[i0] * q[i9]) + (3.719124e-03) * (q[i0] * q[i10]) + (-7.742778e-04) * (q[i0] * q[i11])
            + (2.973193e-04) * (q[i0] * q[i12]) + (1.582378e-03) * (q[i0] * q[i15]) + (1.714890e-03) * (q[i0] * q[i16]) + (5.418329e-04) * (q[i0] * q[i19])
            + (-3.649028e-04) * (q[i0] * q[i20]) + (-5.273413e-04) * (q[i0] * q[i21]) + (5.840159e-04) * (q[i0] * q[i22]) + (3.588026e-04) * (q[i1] * q[i2])
            + (1.181236e-03) * (q[i1] * q[i3]) + (-2.834066e-03) * (q[i1] * q[i4]) + (1.183537e-03) * (q[i1] * q[i5]) + (-1.056407e-03) * (q[i1] * q[i7])
            + (-1.799223e-04) * (q[i1] * q[i8]) + (1.606109e-03) * (q[i1] * q[i9]) + (-1.269122e-04) * (q[i1] * q[i10]) + (8.367449e-04) * (q[i1] * q[i11])
            + (2.208027e-04) * (q[i1] * q[i12]) + (-3.151395e-04) * (q[i1] * q[i15]) + (-4.786425e-04) * (q[i1] * q[i16]) + (-7.087169e-05) * (q[i1] * q[i19])
            + (6.264196e-04) * (q[i1] * q[i20]) + (2.398719e-04) * (q[i1] * q[i21]) + (1.031743e-04) * (q[i1] * q[i22]) + (-3.102689e-03) * (q[i2] * q[i3])
            + (2.777308e-04) * (q[i2] * q[i4]) + (5.061598e-04) * (q[i2] * q[i5]) + (-1.377426e-04) * (q[i2] * q[i7]) + (5.054400e-03) * (q[i2] * q[i8])
            + (-3.439988e-03) * (q[i2] * q[i9]) + (1.431572e-03) * (q[i2] * q[i10]) + (-7.221575e-04) * (q[i2] * q[i11]) + (-2.130224e-05) * (q[i2] * q[i12])
            + (6.749095e-04) * (q[i2] * q[i15]) + (-4.701494e-04) * (q[i2] * q[i16]) + (-1.506784e-04) * (q[i2] * q[i19]) + (2.140396e-04) * (q[i2] * q[i20])
            + (2.169045e-04) * (q[i2] * q[i21]) + (-1.013324e-04) * (q[i2] * q[i22]) + (1.084108e-03) * (q[i3] * q[i4]) + (-1.677210e-03) * (q[i3] * q[i5])
            + (-1.257061e-04) * (q[i3] * q[i7]) + (-1.520813e-03) * (q[i3] * q[i8]) + (-1.066384e-02) * (q[i3] * q[i9]) + (6.920992e-04) * (q[i3] * q[i10])
            + (-2.072890e-04) * (q[i3] * q[i11]) + (-2.334922e-05) * (q[i3] * q[i12]) + (2.047130e-04) * (q[i3] * q[i15]) + (-9.212668e-04) * (q[i3] * q[i16])
            + (-5.412548e-04) * (q[i3] * q[i19]) + (5.468548e-04) * (q[i3] * q[i20]) + (4.925865e-06) * (q[i3] * q[i21]) + (4.098744e-04) * (q[i3] * q[i22])
            + (1.423295e-04) * (q[i4] * q[i5]) + (-1.416828e-04) * (q[i4] * q[i7]) + (2.296334e-04) * (q[i4] * q[i8]) + (1.746406e-03) * (q[i4] * q[i9])
            + (2.863477e-04) * (q[i4] * q[i10]) + (-5.990457e-04) * (q[i4] * q[i11]) + (3.041727e-04) * (q[i4] * q[i12]) + (-3.383068e-04) * (q[i4] * q[i15])
            + (-2.718924e-04) * (q[i4] * q[i16]) + (2.185914e-05) * (q[i4] * q[i19]) + (-3.518379e-04) * (q[i4] * q[i20]) + (-1.800077e-04) * (q[i4] * q[i21])
            + (-1.451111e-04) * (q[i4] * q[i22]) + (1.751704e-04) * (q[i5] * q[i7]) + (7.352272e-05) * (q[i5] * q[i8]) + (2.487281e-03) * (q[i5] * q[i9])
            + (-1.403109e-03) * (q[i5] * q[i10]) + (-2.555128e-04) * (q[i5] * q[i11]) + (-5.092444e-04) * (q[i5] * q[i12]) + (-1.719568e-05) * (q[i5] * q[i15])
            + (1.081003e-04) * (q[i5] * q[i16]) + (1.491224e-04) * (q[i5] * q[i19]) + (6.341876e-04) * (q[i5] * q[i20]) + (-3.825242e-04) * (q[i5] * q[i21])
            + (-2.843814e-04) * (q[i5] * q[i22]) + (3.050427e-06) * (q[i7] * q[i8]) + (8.933321e-04) * (q[i7] * q[i9]) + (-8.729645e-04) * (q[i7] * q[i10])
            + (9.157569e-04) * (q[i7] * q[i11]) + (9.096246e-04) * (q[i7] * q[i12]) + (-2.486836e-04) * (q[i7] * q[i15]) + (2.563843e-04) * (q[i7] * q[i16])
            + (4.420799e-04) * (q[i7] * q[i19]) + (-4.454843e-04) * (q[i7] * q[i20]) + (-1.594664e-05) * (q[i7] * q[i21]) + (-1.502534e-05) * (q[i7] * q[i22])
            + (8.946651e-04) * (q[i8] * q[i9]) + (7.445128e-05) * (q[i8] * q[i10]) + (-3.951089e-04) * (q[i8] * q[i11]) + (-6.113200e-04) * (q[i8] * q[i12])
            + (4.453109e-04) * (q[i8] * q[i15]) + (6.577481e-05) * (q[i8] * q[i16]) + (-3.950201e-05) * (q[i8] * q[i19]) + (4.931194e-04) * (q[i8] * q[i20])
            + (-1.592634e-04) * (q[i8] * q[i21]) + (3.601620e-04) * (q[i8] * q[i22]) + (5.360598e-04) * (q[i9] * q[i10]) + (-1.914007e-04) * (q[i9] * q[i11])
            + (-4.993365e-04) * (q[i9] * q[i12]) + (-1.714614e-04) * (q[i9] * q[i15]) + (3.639200e-04) * (q[i9] * q[i16]) + (-1.812739e-04) * (q[i9] * q[i19])
            + (2.937345e-05) * (q[i9] * q[i20]) + (8.760108e-06) * (q[i9] * q[i21]) + (-1.099680e-04) * (q[i9] * q[i22]) + (2.801582e-04) * (q[i10] * q[i11])
            + (2.759020e-04) * (q[i10] * q[i12]) + (-4.925413e-04) * (q[i10] * q[i15]) + (-1.564324e-04) * (q[i10] * q[i16])
            + (-7.371924e-06) * (q[i10] * q[i19]) + (-3.897913e-05) * (q[i10] * q[i20]) + (8.674091e-05) * (q[i10] * q[i21])
            + (1.694297e-04) * (q[i10] * q[i22]) + (3.259319e-04) * (q[i11] * q[i12]) + (8.257468e-04) * (q[i11] * q[i15]) + (8.527838e-05) * (q[i11] * q[i16])
            + (5.202263e-05) * (q[i11] * q[i19]) + (-4.364655e-05) * (q[i11] * q[i20]) + (3.742519e-04) * (q[i11] * q[i21]) + (1.222552e-04) * (q[i11] * q[i22])
            + (4.345391e-04) * (q[i12] * q[i15]) + (9.717923e-04) * (q[i12] * q[i16]) + (-6.807007e-05) * (q[i12] * q[i19]) + (5.282220e-05) * (q[i12] * q[i20])
            + (-1.138935e-04) * (q[i12] * q[i21]) + (-1.274102e-04) * (q[i12] * q[i22]) + (-3.368392e-04) * (q[i15] * q[i16])
            + (2.110082e-04) * (q[i15] * q[i19]) + (1.025149e-05) * (q[i15] * q[i20]) + (-4.249887e-05) * (q[i15] * q[i21])
            + (-3.835844e-05) * (q[i15] * q[i22]) + (-2.116556e-04) * (q[i16] * q[i19]) + (-2.076262e-04) * (q[i16] * q[i20])
            + (-1.229254e-04) * (q[i16] * q[i21]) + (1.434616e-04) * (q[i16] * q[i22]) + (5.515648e-05) * (q[i19] * q[i20])
            + (-1.667539e-04) * (q[i19] * q[i21]) + (2.299836e-05) * (q[i19] * q[i22]) + (2.634986e-06) * (q[i20] * q[i21])
            + (-3.036432e-04) * (q[i20] * q[i22]) + (-2.140628e-05) * (q[i21] * q[i22]);
      JQ[1][i7] = (-2.870989e-03) * (1) + (-1.908874e-02) * ((2) * q[i7]) + (-9.695121e-03) * (q[i0]) + (9.064621e-02) * (q[i1]) + (1.549539e-02) * (q[i2])
            + (-4.358020e-03) * (q[i3]) + (9.714756e-03) * (q[i4]) + (-2.761619e-04) * (q[i5]) + (-4.590787e-05) * (q[i6]) + (-1.113790e-03) * (q[i8])
            + (2.292232e-03) * (q[i9]) + (-9.520259e-03) * (q[i10]) + (4.318452e-03) * (q[i11]) + (2.696326e-03) * (q[i12]) + (1.142265e-03) * (q[i15])
            + (7.310233e-04) * (q[i16]) + (3.617413e-04) * (q[i19]) + (-4.178380e-04) * (q[i20]) + (-3.194301e-04) * (q[i21]) + (-3.214045e-04) * (q[i22])
            + (-3.322631e-04) * (q[i0] * q[i0]) + (-5.001626e-04) * (q[i1] * q[i1]) + (2.687083e-04) * (q[i2] * q[i2]) + (-8.461025e-04) * (q[i3] * q[i3])
            + (-6.562441e-04) * (q[i4] * q[i4]) + (-8.280944e-04) * (q[i5] * q[i5]) + (-8.446166e-04) * (q[i6] * q[i6])
            + (-1.905565e-03) * ((2) * q[i0] * q[i7]) + (4.921606e-03) * ((2) * q[i1] * q[i7]) + (9.488334e-04) * ((2) * q[i2] * q[i7])
            + (2.902209e-03) * ((2) * q[i3] * q[i7]) + (-2.181574e-02) * ((2) * q[i4] * q[i7]) + (6.444140e-03) * ((2) * q[i5] * q[i7])
            + (8.407773e-04) * ((2) * q[i6] * q[i7]) + (-3.490474e-04) * ((3) * q[i7] * q[i7]) + (-5.956300e-05) * ((2) * q[i7] * q[i8])
            + (-5.738007e-05) * ((2) * q[i7] * q[i9]) + (2.367750e-03) * ((2) * q[i7] * q[i10]) + (-1.432442e-04) * ((2) * q[i7] * q[i11])
            + (1.929896e-04) * ((2) * q[i7] * q[i12]) + (-4.862562e-04) * ((2) * q[i7] * q[i15]) + (3.594144e-05) * ((2) * q[i7] * q[i16])
            + (-9.765210e-05) * ((2) * q[i7] * q[i19]) + (-9.126872e-05) * ((2) * q[i7] * q[i20]) + (-1.198847e-04) * ((2) * q[i7] * q[i21])
            + (-3.545673e-05) * ((2) * q[i7] * q[i22]) + (1.322252e-03) * (q[i8] * q[i8]) + (6.068129e-04) * (q[i9] * q[i9])
            + (2.383832e-03) * (q[i10] * q[i10]) + (-3.222720e-04) * (q[i11] * q[i11]) + (3.116004e-04) * (q[i12] * q[i12])
            + (-2.762074e-05) * (q[i15] * q[i15]) + (2.036815e-05) * (q[i16] * q[i16]) + (4.005731e-05) * (q[i19] * q[i19]) + (7.549052e-05) * (q[i20] * q[i20])
            + (-1.488944e-04) * (q[i21] * q[i21]) + (5.372295e-05) * (q[i22] * q[i22]) + (-2.984609e-04) * (q[i0] * q[i1]) + (-3.649659e-04) * (q[i0] * q[i2])
            + (2.862776e-03) * (q[i0] * q[i3]) + (-1.188181e-03) * (q[i0] * q[i4]) + (-1.202393e-03) * (q[i0] * q[i5]) + (-1.080780e-03) * (q[i0] * q[i6])
            + (-1.829260e-04) * (q[i0] * q[i8]) + (-1.361720e-04) * (q[i0] * q[i9]) + (1.584395e-03) * (q[i0] * q[i10]) + (-2.102670e-04) * (q[i0] * q[i11])
            + (-8.309921e-04) * (q[i0] * q[i12]) + (-4.744342e-04) * (q[i0] * q[i15]) + (-3.210253e-04) * (q[i0] * q[i16]) + (6.264215e-04) * (q[i0] * q[i19])
            + (-7.089603e-05) * (q[i0] * q[i20]) + (-1.047804e-04) * (q[i0] * q[i21]) + (-2.431557e-04) * (q[i0] * q[i22]) + (-3.874749e-04) * (q[i1] * q[i2])
            + (2.355013e-03) * (q[i1] * q[i3]) + (1.048470e-02) * (q[i1] * q[i4]) + (7.746703e-03) * (q[i1] * q[i5]) + (-1.056407e-03) * (q[i1] * q[i6])
            + (6.316686e-03) * (q[i1] * q[i8]) + (3.751410e-03) * (q[i1] * q[i9]) + (-1.448508e-02) * (q[i1] * q[i10]) + (-2.759820e-04) * (q[i1] * q[i11])
            + (7.933086e-04) * (q[i1] * q[i12]) + (1.680669e-03) * (q[i1] * q[i15]) + (1.597703e-03) * (q[i1] * q[i16]) + (-3.730352e-04) * (q[i1] * q[i19])
            + (5.390420e-04) * (q[i1] * q[i20]) + (-5.712384e-04) * (q[i1] * q[i21]) + (5.396924e-04) * (q[i1] * q[i22]) + (-2.698023e-04) * (q[i2] * q[i3])
            + (3.087575e-03) * (q[i2] * q[i4]) + (-5.146189e-04) * (q[i2] * q[i5]) + (-1.377426e-04) * (q[i2] * q[i6]) + (5.048879e-03) * (q[i2] * q[i8])
            + (1.438721e-03) * (q[i2] * q[i9]) + (-3.384361e-03) * (q[i2] * q[i10]) + (3.092087e-05) * (q[i2] * q[i11]) + (7.329934e-04) * (q[i2] * q[i12])
            + (-4.618498e-04) * (q[i2] * q[i15]) + (6.752372e-04) * (q[i2] * q[i16]) + (2.106647e-04) * (q[i2] * q[i19]) + (-1.452176e-04) * (q[i2] * q[i20])
            + (1.059136e-04) * (q[i2] * q[i21]) + (-2.113594e-04) * (q[i2] * q[i22]) + (-1.089899e-03) * (q[i3] * q[i4]) + (-1.603558e-04) * (q[i3] * q[i5])
            + (-1.257061e-04) * (q[i3] * q[i6]) + (2.248550e-04) * (q[i3] * q[i8]) + (2.845153e-04) * (q[i3] * q[i9]) + (1.740196e-03) * (q[i3] * q[i10])
            + (-3.172781e-04) * (q[i3] * q[i11]) + (6.034648e-04) * (q[i3] * q[i12]) + (-2.640983e-04) * (q[i3] * q[i15]) + (-3.420845e-04) * (q[i3] * q[i16])
            + (-3.583018e-04) * (q[i3] * q[i19]) + (2.250667e-05) * (q[i3] * q[i20]) + (1.451721e-04) * (q[i3] * q[i21]) + (1.780640e-04) * (q[i3] * q[i22])
            + (1.695603e-03) * (q[i4] * q[i5]) + (-1.416828e-04) * (q[i4] * q[i6]) + (-1.525328e-03) * (q[i4] * q[i8]) + (7.122502e-04) * (q[i4] * q[i9])
            + (-1.058271e-02) * (q[i4] * q[i10]) + (2.784186e-05) * (q[i4] * q[i11]) + (2.025499e-04) * (q[i4] * q[i12]) + (-9.273298e-04) * (q[i4] * q[i15])
            + (2.176679e-04) * (q[i4] * q[i16]) + (5.446240e-04) * (q[i4] * q[i19]) + (-5.295054e-04) * (q[i4] * q[i20]) + (-4.123952e-04) * (q[i4] * q[i21])
            + (4.186780e-06) * (q[i4] * q[i22]) + (1.751704e-04) * (q[i5] * q[i6]) + (7.377251e-05) * (q[i5] * q[i8]) + (-1.413646e-03) * (q[i5] * q[i9])
            + (2.477301e-03) * (q[i5] * q[i10]) + (5.000242e-04) * (q[i5] * q[i11]) + (2.595822e-04) * (q[i5] * q[i12]) + (1.080666e-04) * (q[i5] * q[i15])
            + (-2.874128e-05) * (q[i5] * q[i16]) + (6.361853e-04) * (q[i5] * q[i19]) + (1.472312e-04) * (q[i5] * q[i20]) + (2.908408e-04) * (q[i5] * q[i21])
            + (3.741623e-04) * (q[i5] * q[i22]) + (3.050427e-06) * (q[i6] * q[i8]) + (8.933321e-04) * (q[i6] * q[i9]) + (-8.729645e-04) * (q[i6] * q[i10])
            + (9.157569e-04) * (q[i6] * q[i11]) + (9.096246e-04) * (q[i6] * q[i12]) + (-2.486836e-04) * (q[i6] * q[i15]) + (2.563843e-04) * (q[i6] * q[i16])
            + (4.420799e-04) * (q[i6] * q[i19]) + (-4.454843e-04) * (q[i6] * q[i20]) + (-1.594664e-05) * (q[i6] * q[i21]) + (-1.502534e-05) * (q[i6] * q[i22])
            + (-7.573144e-05) * (q[i8] * q[i9]) + (-8.841607e-04) * (q[i8] * q[i10]) + (-5.749266e-04) * (q[i8] * q[i11]) + (-3.858998e-04) * (q[i8] * q[i12])
            + (-6.792527e-05) * (q[i8] * q[i15]) + (-4.314481e-04) * (q[i8] * q[i16]) + (-4.847388e-04) * (q[i8] * q[i19]) + (3.692607e-05) * (q[i8] * q[i20])
            + (3.624944e-04) * (q[i8] * q[i21]) + (-1.662362e-04) * (q[i8] * q[i22]) + (-5.349943e-04) * (q[i9] * q[i10]) + (2.795555e-04) * (q[i9] * q[i11])
            + (2.772482e-04) * (q[i9] * q[i12]) + (1.556296e-04) * (q[i9] * q[i15]) + (4.976830e-04) * (q[i9] * q[i16]) + (3.774569e-05) * (q[i9] * q[i19])
            + (4.779438e-06) * (q[i9] * q[i20]) + (1.699146e-04) * (q[i9] * q[i21]) + (8.824007e-05) * (q[i9] * q[i22]) + (-4.982814e-04) * (q[i10] * q[i11])
            + (-1.927251e-04) * (q[i10] * q[i12]) + (-3.596411e-04) * (q[i10] * q[i15]) + (1.720243e-04) * (q[i10] * q[i16])
            + (-3.155922e-05) * (q[i10] * q[i19]) + (1.732689e-04) * (q[i10] * q[i20]) + (-1.111959e-04) * (q[i10] * q[i21])
            + (8.252590e-06) * (q[i10] * q[i22]) + (-3.273813e-04) * (q[i11] * q[i12]) + (9.646963e-04) * (q[i11] * q[i15]) + (4.343167e-04) * (q[i11] * q[i16])
            + (5.168748e-05) * (q[i11] * q[i19]) + (-7.278678e-05) * (q[i11] * q[i20]) + (1.227451e-04) * (q[i11] * q[i21]) + (1.142882e-04) * (q[i11] * q[i22])
            + (8.093829e-05) * (q[i12] * q[i15]) + (8.227340e-04) * (q[i12] * q[i16]) + (-4.373682e-05) * (q[i12] * q[i19]) + (5.232382e-05) * (q[i12] * q[i20])
            + (-1.195996e-04) * (q[i12] * q[i21]) + (-3.711772e-04) * (q[i12] * q[i22]) + (3.378316e-04) * (q[i15] * q[i16])
            + (2.100835e-04) * (q[i15] * q[i19]) + (2.107295e-04) * (q[i15] * q[i20]) + (1.401101e-04) * (q[i15] * q[i21]) + (-1.217494e-04) * (q[i15] * q[i22])
            + (-9.969937e-06) * (q[i16] * q[i19]) + (-2.072429e-04) * (q[i16] * q[i20]) + (-3.814879e-05) * (q[i16] * q[i21])
            + (-4.905897e-05) * (q[i16] * q[i22]) + (-5.782290e-05) * (q[i19] * q[i20]) + (-2.977827e-04) * (q[i19] * q[i21])
            + (5.188520e-06) * (q[i19] * q[i22]) + (2.271721e-05) * (q[i20] * q[i21]) + (-1.667069e-04) * (q[i20] * q[i22])
            + (2.078964e-05) * (q[i21] * q[i22]);
      JQ[1][i8] = (-9.047945e-05) * (1) + (-9.276544e-06) * ((2) * q[i8]) + (-2.481516e-03) * (q[i0]) + (-2.464220e-03) * (q[i1]) + (-6.203577e-02) * (q[i2])
            + (1.316746e-03) * (q[i3]) + (1.315402e-03) * (q[i4]) + (-1.736126e-04) * (q[i5]) + (1.097971e-03) * (q[i6]) + (-1.113790e-03) * (q[i7])
            + (1.359273e-03) * (q[i9]) + (-1.367268e-03) * (q[i10]) + (1.045162e-02) * (q[i11]) + (1.053923e-02) * (q[i12]) + (8.056936e-03) * (q[i15])
            + (-8.161851e-03) * (q[i16]) + (2.349562e-04) * (q[i19]) + (-1.955376e-04) * (q[i20]) + (2.085960e-03) * (q[i21]) + (2.096886e-03) * (q[i22])
            + (-6.276619e-04) * (q[i0] * q[i0]) + (6.299754e-04) * (q[i1] * q[i1]) + (-1.720420e-05) * (q[i2] * q[i2]) + (6.903501e-05) * (q[i3] * q[i3])
            + (-7.354570e-05) * (q[i4] * q[i4]) + (-1.637447e-05) * (q[i5] * q[i5]) + (6.126582e-05) * (q[i6] * q[i6]) + (-5.956300e-05) * (q[i7] * q[i7])
            + (-1.376380e-04) * ((2) * q[i0] * q[i8]) + (-1.363484e-04) * ((2) * q[i1] * q[i8]) + (2.814786e-03) * ((2) * q[i2] * q[i8])
            + (4.246645e-03) * ((2) * q[i3] * q[i8]) + (4.248400e-03) * ((2) * q[i4] * q[i8]) + (-2.342988e-02) * ((2) * q[i5] * q[i8])
            + (-1.323027e-03) * ((2) * q[i6] * q[i8]) + (1.322252e-03) * ((2) * q[i7] * q[i8]) + (8.470253e-06) * ((3) * q[i8] * q[i8])
            + (2.211882e-04) * ((2) * q[i8] * q[i9]) + (-2.241490e-04) * ((2) * q[i8] * q[i10]) + (-9.334205e-04) * ((2) * q[i8] * q[i11])
            + (-9.748763e-04) * ((2) * q[i8] * q[i12]) + (4.136062e-03) * ((2) * q[i8] * q[i15]) + (-4.181311e-03) * ((2) * q[i8] * q[i16])
            + (3.631669e-04) * ((2) * q[i8] * q[i19]) + (-3.565308e-04) * ((2) * q[i8] * q[i20]) + (-3.602408e-05) * ((2) * q[i8] * q[i21])
            + (-4.186722e-05) * ((2) * q[i8] * q[i22]) + (2.875656e-04) * (q[i9] * q[i9]) + (-2.821292e-04) * (q[i10] * q[i10])
            + (4.575323e-04) * (q[i11] * q[i11]) + (-3.806106e-04) * (q[i12] * q[i12]) + (2.054755e-03) * (q[i15] * q[i15])
            + (-2.070227e-03) * (q[i16] * q[i16]) + (9.644574e-05) * (q[i19] * q[i19]) + (-1.047269e-04) * (q[i20] * q[i20])
            + (7.672063e-04) * (q[i21] * q[i21]) + (-7.750709e-04) * (q[i22] * q[i22]) + (7.173709e-06) * (q[i0] * q[i1]) + (-2.402488e-03) * (q[i0] * q[i2])
            + (2.243601e-03) * (q[i0] * q[i3]) + (-1.034320e-03) * (q[i0] * q[i4]) + (5.711739e-04) * (q[i0] * q[i5]) + (6.325835e-03) * (q[i0] * q[i6])
            + (-1.829260e-04) * (q[i0] * q[i7]) + (5.752271e-04) * (q[i0] * q[i9]) + (-6.227911e-04) * (q[i0] * q[i10]) + (-5.047233e-04) * (q[i0] * q[i11])
            + (6.275289e-04) * (q[i0] * q[i12]) + (4.118524e-04) * (q[i0] * q[i15]) + (1.008221e-03) * (q[i0] * q[i16]) + (4.729393e-04) * (q[i0] * q[i19])
            + (3.312547e-04) * (q[i0] * q[i20]) + (2.722731e-05) * (q[i0] * q[i21]) + (-2.079227e-04) * (q[i0] * q[i22]) + (2.394033e-03) * (q[i1] * q[i2])
            + (1.040977e-03) * (q[i1] * q[i3]) + (-2.226817e-03) * (q[i1] * q[i4]) + (-5.622257e-04) * (q[i1] * q[i5]) + (-1.799223e-04) * (q[i1] * q[i6])
            + (6.316686e-03) * (q[i1] * q[i7]) + (-6.288250e-04) * (q[i1] * q[i9]) + (5.758056e-04) * (q[i1] * q[i10]) + (-5.762647e-04) * (q[i1] * q[i11])
            + (5.423433e-04) * (q[i1] * q[i12]) + (9.890983e-04) * (q[i1] * q[i15]) + (4.193216e-04) * (q[i1] * q[i16]) + (3.292585e-04) * (q[i1] * q[i19])
            + (4.634254e-04) * (q[i1] * q[i20]) + (2.051705e-04) * (q[i1] * q[i21]) + (-2.760091e-05) * (q[i1] * q[i22]) + (3.073255e-03) * (q[i2] * q[i3])
            + (-3.044392e-03) * (q[i2] * q[i4]) + (-3.873137e-05) * (q[i2] * q[i5]) + (5.054400e-03) * (q[i2] * q[i6]) + (5.048879e-03) * (q[i2] * q[i7])
            + (-3.521480e-03) * (q[i2] * q[i9]) + (-3.483055e-03) * (q[i2] * q[i10]) + (-9.728021e-04) * (q[i2] * q[i11]) + (1.148406e-03) * (q[i2] * q[i12])
            + (5.609538e-03) * (q[i2] * q[i15]) + (5.688234e-03) * (q[i2] * q[i16]) + (2.465393e-04) * (q[i2] * q[i19]) + (2.382505e-04) * (q[i2] * q[i20])
            + (-3.458079e-04) * (q[i2] * q[i21]) + (3.538491e-04) * (q[i2] * q[i22]) + (1.260295e-06) * (q[i3] * q[i4]) + (-2.791834e-04) * (q[i3] * q[i5])
            + (-1.520813e-03) * (q[i3] * q[i6]) + (2.248550e-04) * (q[i3] * q[i7]) + (-9.315922e-05) * (q[i3] * q[i9]) + (4.078081e-04) * (q[i3] * q[i10])
            + (-8.162157e-04) * (q[i3] * q[i11]) + (5.571022e-04) * (q[i3] * q[i12]) + (-5.793535e-04) * (q[i3] * q[i15]) + (-5.885023e-04) * (q[i3] * q[i16])
            + (-2.736758e-04) * (q[i3] * q[i19]) + (-2.506133e-04) * (q[i3] * q[i20]) + (3.583714e-04) * (q[i3] * q[i21]) + (-3.718586e-05) * (q[i3] * q[i22])
            + (2.818257e-04) * (q[i4] * q[i5]) + (2.296334e-04) * (q[i4] * q[i6]) + (-1.525328e-03) * (q[i4] * q[i7]) + (4.080237e-04) * (q[i4] * q[i9])
            + (-9.400196e-05) * (q[i4] * q[i10]) + (-5.216702e-04) * (q[i4] * q[i11]) + (8.343474e-04) * (q[i4] * q[i12]) + (-5.722809e-04) * (q[i4] * q[i15])
            + (-5.842616e-04) * (q[i4] * q[i16]) + (-2.530987e-04) * (q[i4] * q[i19]) + (-2.619990e-04) * (q[i4] * q[i20]) + (3.014335e-05) * (q[i4] * q[i21])
            + (-3.593076e-04) * (q[i4] * q[i22]) + (7.352272e-05) * (q[i5] * q[i6]) + (7.377251e-05) * (q[i5] * q[i7]) + (-1.799927e-03) * (q[i5] * q[i9])
            + (-1.784532e-03) * (q[i5] * q[i10]) + (2.327999e-03) * (q[i5] * q[i11]) + (-2.406813e-03) * (q[i5] * q[i12]) + (2.348362e-03) * (q[i5] * q[i15])
            + (2.398485e-03) * (q[i5] * q[i16]) + (5.959096e-04) * (q[i5] * q[i19]) + (5.773746e-04) * (q[i5] * q[i20]) + (1.498457e-04) * (q[i5] * q[i21])
            + (-1.440720e-04) * (q[i5] * q[i22]) + (3.050427e-06) * (q[i6] * q[i7]) + (8.946651e-04) * (q[i6] * q[i9]) + (7.445128e-05) * (q[i6] * q[i10])
            + (-3.951089e-04) * (q[i6] * q[i11]) + (-6.113200e-04) * (q[i6] * q[i12]) + (4.453109e-04) * (q[i6] * q[i15]) + (6.577481e-05) * (q[i6] * q[i16])
            + (-3.950201e-05) * (q[i6] * q[i19]) + (4.931194e-04) * (q[i6] * q[i20]) + (-1.592634e-04) * (q[i6] * q[i21]) + (3.601620e-04) * (q[i6] * q[i22])
            + (-7.573144e-05) * (q[i7] * q[i9]) + (-8.841607e-04) * (q[i7] * q[i10]) + (-5.749266e-04) * (q[i7] * q[i11]) + (-3.858998e-04) * (q[i7] * q[i12])
            + (-6.792527e-05) * (q[i7] * q[i15]) + (-4.314481e-04) * (q[i7] * q[i16]) + (-4.847388e-04) * (q[i7] * q[i19]) + (3.692607e-05) * (q[i7] * q[i20])
            + (3.624944e-04) * (q[i7] * q[i21]) + (-1.662362e-04) * (q[i7] * q[i22]) + (-8.093102e-07) * (q[i9] * q[i10]) + (4.649929e-04) * (q[i9] * q[i11])
            + (7.230965e-04) * (q[i9] * q[i12]) + (-2.590363e-04) * (q[i9] * q[i15]) + (7.364581e-05) * (q[i9] * q[i16]) + (6.685977e-05) * (q[i9] * q[i19])
            + (-3.435974e-05) * (q[i9] * q[i20]) + (3.378134e-04) * (q[i9] * q[i21]) + (-2.450494e-04) * (q[i9] * q[i22]) + (7.121912e-04) * (q[i10] * q[i11])
            + (4.641616e-04) * (q[i10] * q[i12]) + (-7.107874e-05) * (q[i10] * q[i15]) + (2.570242e-04) * (q[i10] * q[i16]) + (4.027034e-05) * (q[i10] * q[i19])
            + (-6.536552e-05) * (q[i10] * q[i20]) + (-2.449227e-04) * (q[i10] * q[i21]) + (3.374044e-04) * (q[i10] * q[i22])
            + (-3.659516e-06) * (q[i11] * q[i12]) + (3.760734e-03) * (q[i11] * q[i15]) + (1.553745e-04) * (q[i11] * q[i16]) + (3.075091e-04) * (q[i11] * q[i19])
            + (4.150482e-05) * (q[i11] * q[i20]) + (1.772542e-03) * (q[i11] * q[i21]) + (8.801020e-05) * (q[i11] * q[i22]) + (1.540005e-04) * (q[i12] * q[i15])
            + (3.777960e-03) * (q[i12] * q[i16]) + (4.182337e-05) * (q[i12] * q[i19]) + (2.916899e-04) * (q[i12] * q[i20]) + (-8.636645e-05) * (q[i12] * q[i21])
            + (-1.784999e-03) * (q[i12] * q[i22]) + (6.105476e-07) * (q[i15] * q[i16]) + (3.855031e-04) * (q[i15] * q[i19])
            + (-1.373722e-04) * (q[i15] * q[i20]) + (4.243812e-04) * (q[i15] * q[i21]) + (1.451907e-04) * (q[i15] * q[i22]) + (1.402156e-04) * (q[i16] * q[i19])
            + (-3.603188e-04) * (q[i16] * q[i20]) + (1.460470e-04) * (q[i16] * q[i21]) + (4.367203e-04) * (q[i16] * q[i22])
            + (-6.143433e-07) * (q[i19] * q[i20]) + (-8.813338e-04) * (q[i19] * q[i21]) + (1.426832e-05) * (q[i19] * q[i22])
            + (1.982403e-05) * (q[i20] * q[i21]) + (-8.678889e-04) * (q[i20] * q[i22]) + (1.326477e-07) * (q[i21] * q[i22]);
      JQ[1][i9] = (6.574986e-03) * (1) + (4.524237e-03) * ((2) * q[i9]) + (2.963527e-02) * (q[i0]) + (-1.196230e-03) * (q[i1]) + (1.601713e-03) * (q[i2])
            + (-8.968284e-03) * (q[i3]) + (5.551080e-03) * (q[i4]) + (2.606375e-03) * (q[i5]) + (9.624614e-03) * (q[i6]) + (2.292232e-03) * (q[i7])
            + (1.359273e-03) * (q[i8]) + (-3.349715e-06) * (q[i10]) + (1.107012e-04) * (q[i11]) + (5.910216e-04) * (q[i12]) + (-2.286462e-04) * (q[i15])
            + (2.458141e-04) * (q[i16]) + (4.954136e-04) * (q[i19]) + (-6.306070e-05) * (q[i20]) + (6.873966e-05) * (q[i21]) + (-3.064132e-04) * (q[i22])
            + (2.210254e-03) * (q[i0] * q[i0]) + (7.155024e-04) * (q[i1] * q[i1]) + (-1.767681e-04) * (q[i2] * q[i2]) + (-3.207326e-04) * (q[i3] * q[i3])
            + (8.988663e-06) * (q[i4] * q[i4]) + (-1.758869e-04) * (q[i5] * q[i5]) + (-2.393264e-03) * (q[i6] * q[i6]) + (-5.738007e-05) * (q[i7] * q[i7])
            + (2.211882e-04) * (q[i8] * q[i8]) + (-7.039191e-03) * ((2) * q[i0] * q[i9]) + (1.026084e-03) * ((2) * q[i1] * q[i9])
            + (-1.823829e-03) * ((2) * q[i2] * q[i9]) + (-3.393812e-03) * ((2) * q[i3] * q[i9]) + (6.176945e-04) * ((2) * q[i4] * q[i9])
            + (1.423103e-03) * ((2) * q[i5] * q[i9]) + (-2.416989e-03) * ((2) * q[i6] * q[i9]) + (6.068129e-04) * ((2) * q[i7] * q[i9])
            + (2.875656e-04) * ((2) * q[i8] * q[i9]) + (-7.837559e-04) * ((3) * q[i9] * q[i9]) + (3.294217e-04) * ((2) * q[i9] * q[i10])
            + (-1.004519e-04) * ((2) * q[i9] * q[i11]) + (-2.476464e-04) * ((2) * q[i9] * q[i12]) + (3.525948e-05) * ((2) * q[i9] * q[i15])
            + (2.191663e-04) * ((2) * q[i9] * q[i16]) + (-1.888503e-04) * ((2) * q[i9] * q[i19]) + (6.264896e-05) * ((2) * q[i9] * q[i20])
            + (-4.165742e-05) * ((2) * q[i9] * q[i21]) + (3.440476e-06) * ((2) * q[i9] * q[i22]) + (-3.284458e-04) * (q[i10] * q[i10])
            + (6.008311e-05) * (q[i11] * q[i11]) + (-2.273791e-04) * (q[i12] * q[i12]) + (2.068098e-05) * (q[i15] * q[i15])
            + (-7.112827e-05) * (q[i16] * q[i16]) + (-5.995082e-05) * (q[i19] * q[i19]) + (7.274886e-05) * (q[i20] * q[i20])
            + (1.329937e-05) * (q[i21] * q[i21]) + (4.793143e-05) * (q[i22] * q[i22]) + (-9.985536e-04) * (q[i0] * q[i1]) + (1.025103e-03) * (q[i0] * q[i2])
            + (-8.869268e-04) * (q[i0] * q[i3]) + (5.048114e-04) * (q[i0] * q[i4]) + (-5.320007e-04) * (q[i0] * q[i5]) + (-1.465474e-02) * (q[i0] * q[i6])
            + (-1.361720e-04) * (q[i0] * q[i7]) + (5.752271e-04) * (q[i0] * q[i8]) + (5.517986e-04) * (q[i0] * q[i10]) + (-8.545944e-05) * (q[i0] * q[i11])
            + (3.097661e-04) * (q[i0] * q[i12]) + (3.770447e-04) * (q[i0] * q[i15]) + (2.400579e-04) * (q[i0] * q[i16]) + (4.301947e-04) * (q[i0] * q[i19])
            + (1.408991e-04) * (q[i0] * q[i20]) + (-1.125004e-04) * (q[i0] * q[i21]) + (7.431093e-05) * (q[i0] * q[i22]) + (2.287002e-04) * (q[i1] * q[i2])
            + (-3.836647e-04) * (q[i1] * q[i3]) + (-2.315400e-03) * (q[i1] * q[i4]) + (-5.807911e-05) * (q[i1] * q[i5]) + (1.606109e-03) * (q[i1] * q[i6])
            + (3.751410e-03) * (q[i1] * q[i7]) + (-6.288250e-04) * (q[i1] * q[i8]) + (5.497590e-04) * (q[i1] * q[i10]) + (-3.580780e-06) * (q[i1] * q[i11])
            + (3.420878e-06) * (q[i1] * q[i12]) + (-9.988166e-05) * (q[i1] * q[i15]) + (2.043830e-04) * (q[i1] * q[i16]) + (-3.293673e-04) * (q[i1] * q[i19])
            + (7.962626e-05) * (q[i1] * q[i20]) + (1.490205e-04) * (q[i1] * q[i21]) + (3.217950e-04) * (q[i1] * q[i22]) + (-3.824442e-04) * (q[i2] * q[i3])
            + (-2.735006e-04) * (q[i2] * q[i4]) + (1.587581e-03) * (q[i2] * q[i5]) + (-3.439988e-03) * (q[i2] * q[i6]) + (1.438721e-03) * (q[i2] * q[i7])
            + (-3.521480e-03) * (q[i2] * q[i8]) + (1.051741e-06) * (q[i2] * q[i10]) + (-1.846460e-04) * (q[i2] * q[i11]) + (-5.668092e-05) * (q[i2] * q[i12])
            + (5.058564e-06) * (q[i2] * q[i15]) + (-1.346369e-04) * (q[i2] * q[i16]) + (1.978261e-05) * (q[i2] * q[i19]) + (9.105248e-05) * (q[i2] * q[i20])
            + (-3.524261e-05) * (q[i2] * q[i21]) + (1.524644e-04) * (q[i2] * q[i22]) + (-4.328217e-04) * (q[i3] * q[i4]) + (2.351077e-04) * (q[i3] * q[i5])
            + (-1.066384e-02) * (q[i3] * q[i6]) + (2.845153e-04) * (q[i3] * q[i7]) + (-9.315922e-05) * (q[i3] * q[i8]) + (-4.873547e-04) * (q[i3] * q[i10])
            + (-1.778195e-04) * (q[i3] * q[i11]) + (-2.674895e-04) * (q[i3] * q[i12]) + (-2.490409e-04) * (q[i3] * q[i15]) + (-6.474482e-04) * (q[i3] * q[i16])
            + (-2.530471e-04) * (q[i3] * q[i19]) + (5.058323e-05) * (q[i3] * q[i20]) + (1.621689e-04) * (q[i3] * q[i21]) + (-2.972290e-04) * (q[i3] * q[i22])
            + (9.712108e-05) * (q[i4] * q[i5]) + (1.746406e-03) * (q[i4] * q[i6]) + (7.122502e-04) * (q[i4] * q[i7]) + (4.080237e-04) * (q[i4] * q[i8])
            + (-4.884575e-04) * (q[i4] * q[i10]) + (-3.400631e-04) * (q[i4] * q[i11]) + (1.283953e-04) * (q[i4] * q[i12]) + (1.065911e-04) * (q[i4] * q[i15])
            + (1.631917e-04) * (q[i4] * q[i16]) + (1.529120e-04) * (q[i4] * q[i19]) + (6.564241e-05) * (q[i4] * q[i20]) + (-2.000379e-04) * (q[i4] * q[i21])
            + (2.217890e-04) * (q[i4] * q[i22]) + (2.487281e-03) * (q[i5] * q[i6]) + (-1.413646e-03) * (q[i5] * q[i7]) + (-1.799927e-03) * (q[i5] * q[i8])
            + (-2.438288e-06) * (q[i5] * q[i10]) + (8.956663e-05) * (q[i5] * q[i11]) + (-2.005148e-04) * (q[i5] * q[i12]) + (-3.390928e-04) * (q[i5] * q[i15])
            + (-2.962105e-04) * (q[i5] * q[i16]) + (3.594022e-04) * (q[i5] * q[i19]) + (4.230960e-05) * (q[i5] * q[i20]) + (4.788983e-05) * (q[i5] * q[i21])
            + (1.136154e-06) * (q[i5] * q[i22]) + (8.933321e-04) * (q[i6] * q[i7]) + (8.946651e-04) * (q[i6] * q[i8]) + (5.360598e-04) * (q[i6] * q[i10])
            + (-1.914007e-04) * (q[i6] * q[i11]) + (-4.993365e-04) * (q[i6] * q[i12]) + (-1.714614e-04) * (q[i6] * q[i15]) + (3.639200e-04) * (q[i6] * q[i16])
            + (-1.812739e-04) * (q[i6] * q[i19]) + (2.937345e-05) * (q[i6] * q[i20]) + (8.760108e-06) * (q[i6] * q[i21]) + (-1.099680e-04) * (q[i6] * q[i22])
            + (-7.573144e-05) * (q[i7] * q[i8]) + (-5.349943e-04) * (q[i7] * q[i10]) + (2.795555e-04) * (q[i7] * q[i11]) + (2.772482e-04) * (q[i7] * q[i12])
            + (1.556296e-04) * (q[i7] * q[i15]) + (4.976830e-04) * (q[i7] * q[i16]) + (3.774569e-05) * (q[i7] * q[i19]) + (4.779438e-06) * (q[i7] * q[i20])
            + (1.699146e-04) * (q[i7] * q[i21]) + (8.824007e-05) * (q[i7] * q[i22]) + (-8.093102e-07) * (q[i8] * q[i10]) + (4.649929e-04) * (q[i8] * q[i11])
            + (7.230965e-04) * (q[i8] * q[i12]) + (-2.590363e-04) * (q[i8] * q[i15]) + (7.364581e-05) * (q[i8] * q[i16]) + (6.685977e-05) * (q[i8] * q[i19])
            + (-3.435974e-05) * (q[i8] * q[i20]) + (3.378134e-04) * (q[i8] * q[i21]) + (-2.450494e-04) * (q[i8] * q[i22]) + (8.753649e-05) * (q[i10] * q[i11])
            + (8.681458e-05) * (q[i10] * q[i12]) + (3.084963e-05) * (q[i10] * q[i15]) + (-3.222391e-05) * (q[i10] * q[i16])
            + (-2.000294e-05) * (q[i10] * q[i19]) + (1.999417e-05) * (q[i10] * q[i20]) + (7.364560e-05) * (q[i10] * q[i21]) + (7.426748e-05) * (q[i10] * q[i22])
            + (-5.540778e-05) * (q[i11] * q[i12]) + (-3.124145e-04) * (q[i11] * q[i15]) + (-2.319001e-05) * (q[i11] * q[i16])
            + (1.114000e-05) * (q[i11] * q[i19]) + (-3.336551e-05) * (q[i11] * q[i20]) + (1.422593e-04) * (q[i11] * q[i21])
            + (-7.166301e-05) * (q[i11] * q[i22]) + (-1.269955e-04) * (q[i12] * q[i15]) + (-1.029408e-04) * (q[i12] * q[i16])
            + (-7.128775e-05) * (q[i12] * q[i19]) + (1.669239e-04) * (q[i12] * q[i20]) + (2.485172e-05) * (q[i12] * q[i21]) + (1.425158e-04) * (q[i12] * q[i22])
            + (6.452166e-05) * (q[i15] * q[i16]) + (9.578845e-05) * (q[i15] * q[i19]) + (4.479746e-05) * (q[i15] * q[i20]) + (5.512411e-05) * (q[i15] * q[i21])
            + (-1.941762e-05) * (q[i15] * q[i22]) + (-9.599487e-05) * (q[i16] * q[i19]) + (-6.383294e-05) * (q[i16] * q[i20])
            + (-3.070896e-05) * (q[i16] * q[i21]) + (-3.057005e-05) * (q[i16] * q[i22]) + (1.254037e-04) * (q[i19] * q[i20])
            + (6.051227e-05) * (q[i19] * q[i21]) + (7.515042e-05) * (q[i19] * q[i22]) + (-2.057594e-05) * (q[i20] * q[i21])
            + (-1.112748e-04) * (q[i20] * q[i22]) + (-2.941916e-06) * (q[i21] * q[i22]);
      JQ[1][i10] = (-6.456387e-03) * (1) + (-4.464385e-03) * ((2) * q[i10]) + (-1.181848e-03) * (q[i0]) + (2.931251e-02) * (q[i1]) + (1.589481e-03) * (q[i2])
            + (5.529124e-03) * (q[i3]) + (-8.885331e-03) * (q[i4]) + (2.578295e-03) * (q[i5]) + (-2.292030e-03) * (q[i6]) + (-9.520259e-03) * (q[i7])
            + (-1.367268e-03) * (q[i8]) + (-3.349715e-06) * (q[i9]) + (5.861028e-04) * (q[i11]) + (1.030351e-04) * (q[i12]) + (-2.416712e-04) * (q[i15])
            + (2.268613e-04) * (q[i16]) + (6.531916e-05) * (q[i19]) + (-4.910622e-04) * (q[i20]) + (-2.976386e-04) * (q[i21]) + (6.265089e-05) * (q[i22])
            + (-7.039792e-04) * (q[i0] * q[i0]) + (-2.193286e-03) * (q[i1] * q[i1]) + (1.711918e-04) * (q[i2] * q[i2]) + (-1.570220e-05) * (q[i3] * q[i3])
            + (3.175588e-04) * (q[i4] * q[i4]) + (1.866823e-04) * (q[i5] * q[i5]) + (5.680422e-05) * (q[i6] * q[i6]) + (2.367750e-03) * (q[i7] * q[i7])
            + (-2.241490e-04) * (q[i8] * q[i8]) + (3.294217e-04) * (q[i9] * q[i9]) + (1.015281e-03) * ((2) * q[i0] * q[i10])
            + (-6.959196e-03) * ((2) * q[i1] * q[i10]) + (-1.797081e-03) * ((2) * q[i2] * q[i10]) + (6.085741e-04) * ((2) * q[i3] * q[i10])
            + (-3.353310e-03) * ((2) * q[i4] * q[i10]) + (1.408923e-03) * ((2) * q[i5] * q[i10]) + (-5.953001e-04) * ((2) * q[i6] * q[i10])
            + (2.383832e-03) * ((2) * q[i7] * q[i10]) + (-2.821292e-04) * ((2) * q[i8] * q[i10]) + (-3.284458e-04) * ((2) * q[i9] * q[i10])
            + (7.730562e-04) * ((3) * q[i10] * q[i10]) + (-2.487220e-04) * ((2) * q[i10] * q[i11]) + (-9.833280e-05) * ((2) * q[i10] * q[i12])
            + (-2.159147e-04) * ((2) * q[i10] * q[i15]) + (-3.608072e-05) * ((2) * q[i10] * q[i16]) + (-6.291108e-05) * ((2) * q[i10] * q[i19])
            + (1.849190e-04) * ((2) * q[i10] * q[i20]) + (3.249624e-06) * ((2) * q[i10] * q[i21]) + (-4.041151e-05) * ((2) * q[i10] * q[i22])
            + (2.155967e-04) * (q[i11] * q[i11]) + (-6.828272e-05) * (q[i12] * q[i12]) + (7.112707e-05) * (q[i15] * q[i15])
            + (-2.127310e-05) * (q[i16] * q[i16]) + (-7.298273e-05) * (q[i19] * q[i19]) + (5.974990e-05) * (q[i20] * q[i20])
            + (-4.647722e-05) * (q[i21] * q[i21]) + (-1.234508e-05) * (q[i22] * q[i22]) + (9.903339e-04) * (q[i0] * q[i1]) + (-2.223396e-04) * (q[i0] * q[i2])
            + (2.287754e-03) * (q[i0] * q[i3]) + (3.749607e-04) * (q[i0] * q[i4]) + (4.327036e-05) * (q[i0] * q[i5]) + (3.719124e-03) * (q[i0] * q[i6])
            + (1.584395e-03) * (q[i0] * q[i7]) + (-6.227911e-04) * (q[i0] * q[i8]) + (5.517986e-04) * (q[i0] * q[i9]) + (-6.850180e-06) * (q[i0] * q[i11])
            + (4.021818e-06) * (q[i0] * q[i12]) + (2.035634e-04) * (q[i0] * q[i15]) + (-9.682506e-05) * (q[i0] * q[i16]) + (8.105425e-05) * (q[i0] * q[i19])
            + (-3.296448e-04) * (q[i0] * q[i20]) + (-3.191838e-04) * (q[i0] * q[i21]) + (-1.513056e-04) * (q[i0] * q[i22]) + (-1.015867e-03) * (q[i1] * q[i2])
            + (-5.058233e-04) * (q[i1] * q[i3]) + (8.886449e-04) * (q[i1] * q[i4]) + (5.458980e-04) * (q[i1] * q[i5]) + (-1.269122e-04) * (q[i1] * q[i6])
            + (-1.448508e-02) * (q[i1] * q[i7]) + (5.758056e-04) * (q[i1] * q[i8]) + (5.497590e-04) * (q[i1] * q[i9]) + (-3.052562e-04) * (q[i1] * q[i11])
            + (8.281640e-05) * (q[i1] * q[i12]) + (2.340344e-04) * (q[i1] * q[i15]) + (3.797765e-04) * (q[i1] * q[i16]) + (1.399759e-04) * (q[i1] * q[i19])
            + (4.300117e-04) * (q[i1] * q[i20]) + (-7.301478e-05) * (q[i1] * q[i21]) + (1.116875e-04) * (q[i1] * q[i22]) + (2.621603e-04) * (q[i2] * q[i3])
            + (3.840404e-04) * (q[i2] * q[i4]) + (-1.582219e-03) * (q[i2] * q[i5]) + (1.431572e-03) * (q[i2] * q[i6]) + (-3.384361e-03) * (q[i2] * q[i7])
            + (-3.483055e-03) * (q[i2] * q[i8]) + (1.051741e-06) * (q[i2] * q[i9]) + (4.245635e-05) * (q[i2] * q[i11]) + (1.703269e-04) * (q[i2] * q[i12])
            + (-1.328026e-04) * (q[i2] * q[i15]) + (9.072092e-06) * (q[i2] * q[i16]) + (9.332384e-05) * (q[i2] * q[i19]) + (2.200496e-05) * (q[i2] * q[i20])
            + (-1.501000e-04) * (q[i2] * q[i21]) + (3.215402e-05) * (q[i2] * q[i22]) + (4.310851e-04) * (q[i3] * q[i4]) + (-9.716475e-05) * (q[i3] * q[i5])
            + (6.920992e-04) * (q[i3] * q[i6]) + (1.740196e-03) * (q[i3] * q[i7]) + (4.078081e-04) * (q[i3] * q[i8]) + (-4.873547e-04) * (q[i3] * q[i9])
            + (-1.247490e-04) * (q[i3] * q[i11]) + (3.478084e-04) * (q[i3] * q[i12]) + (1.607090e-04) * (q[i3] * q[i15]) + (1.095946e-04) * (q[i3] * q[i16])
            + (6.130547e-05) * (q[i3] * q[i19]) + (1.567877e-04) * (q[i3] * q[i20]) + (-2.241620e-04) * (q[i3] * q[i21]) + (2.002785e-04) * (q[i3] * q[i22])
            + (-2.354443e-04) * (q[i4] * q[i5]) + (2.863477e-04) * (q[i4] * q[i6]) + (-1.058271e-02) * (q[i4] * q[i7]) + (-9.400196e-05) * (q[i4] * q[i8])
            + (-4.884575e-04) * (q[i4] * q[i9]) + (2.572872e-04) * (q[i4] * q[i11]) + (1.697263e-04) * (q[i4] * q[i12]) + (-6.359112e-04) * (q[i4] * q[i15])
            + (-2.492120e-04) * (q[i4] * q[i16]) + (4.953959e-05) * (q[i4] * q[i19]) + (-2.550075e-04) * (q[i4] * q[i20]) + (2.937772e-04) * (q[i4] * q[i21])
            + (-1.621648e-04) * (q[i4] * q[i22]) + (-1.403109e-03) * (q[i5] * q[i6]) + (2.477301e-03) * (q[i5] * q[i7]) + (-1.784532e-03) * (q[i5] * q[i8])
            + (-2.438288e-06) * (q[i5] * q[i9]) + (1.925860e-04) * (q[i5] * q[i11]) + (-9.604946e-05) * (q[i5] * q[i12]) + (-2.919496e-04) * (q[i5] * q[i15])
            + (-3.412088e-04) * (q[i5] * q[i16]) + (4.502975e-05) * (q[i5] * q[i19]) + (3.584510e-04) * (q[i5] * q[i20]) + (1.194787e-06) * (q[i5] * q[i21])
            + (-5.287167e-05) * (q[i5] * q[i22]) + (-8.729645e-04) * (q[i6] * q[i7]) + (7.445128e-05) * (q[i6] * q[i8]) + (5.360598e-04) * (q[i6] * q[i9])
            + (2.801582e-04) * (q[i6] * q[i11]) + (2.759020e-04) * (q[i6] * q[i12]) + (-4.925413e-04) * (q[i6] * q[i15]) + (-1.564324e-04) * (q[i6] * q[i16])
            + (-7.371924e-06) * (q[i6] * q[i19]) + (-3.897913e-05) * (q[i6] * q[i20]) + (8.674091e-05) * (q[i6] * q[i21]) + (1.694297e-04) * (q[i6] * q[i22])
            + (-8.841607e-04) * (q[i7] * q[i8]) + (-5.349943e-04) * (q[i7] * q[i9]) + (-4.982814e-04) * (q[i7] * q[i11]) + (-1.927251e-04) * (q[i7] * q[i12])
            + (-3.596411e-04) * (q[i7] * q[i15]) + (1.720243e-04) * (q[i7] * q[i16]) + (-3.155922e-05) * (q[i7] * q[i19]) + (1.732689e-04) * (q[i7] * q[i20])
            + (-1.111959e-04) * (q[i7] * q[i21]) + (8.252590e-06) * (q[i7] * q[i22]) + (-8.093102e-07) * (q[i8] * q[i9]) + (7.121912e-04) * (q[i8] * q[i11])
            + (4.641616e-04) * (q[i8] * q[i12]) + (-7.107874e-05) * (q[i8] * q[i15]) + (2.570242e-04) * (q[i8] * q[i16]) + (4.027034e-05) * (q[i8] * q[i19])
            + (-6.536552e-05) * (q[i8] * q[i20]) + (-2.449227e-04) * (q[i8] * q[i21]) + (3.374044e-04) * (q[i8] * q[i22]) + (8.753649e-05) * (q[i9] * q[i11])
            + (8.681458e-05) * (q[i9] * q[i12]) + (3.084963e-05) * (q[i9] * q[i15]) + (-3.222391e-05) * (q[i9] * q[i16]) + (-2.000294e-05) * (q[i9] * q[i19])
            + (1.999417e-05) * (q[i9] * q[i20]) + (7.364560e-05) * (q[i9] * q[i21]) + (7.426748e-05) * (q[i9] * q[i22]) + (5.493271e-05) * (q[i11] * q[i12])
            + (-1.025170e-04) * (q[i11] * q[i15]) + (-1.252333e-04) * (q[i11] * q[i16]) + (1.704197e-04) * (q[i11] * q[i19])
            + (-7.218964e-05) * (q[i11] * q[i20]) + (-1.410711e-04) * (q[i11] * q[i21]) + (-2.435788e-05) * (q[i11] * q[i22])
            + (-2.486485e-05) * (q[i12] * q[i15]) + (-3.109391e-04) * (q[i12] * q[i16]) + (-3.370367e-05) * (q[i12] * q[i19])
            + (1.260359e-05) * (q[i12] * q[i20]) + (7.388342e-05) * (q[i12] * q[i21]) + (-1.428886e-04) * (q[i12] * q[i22])
            + (-6.514242e-05) * (q[i15] * q[i16]) + (6.426175e-05) * (q[i15] * q[i19]) + (9.496067e-05) * (q[i15] * q[i20])
            + (-2.781572e-05) * (q[i15] * q[i21]) + (-2.911934e-05) * (q[i15] * q[i22]) + (-4.382993e-05) * (q[i16] * q[i19])
            + (-9.594302e-05) * (q[i16] * q[i20]) + (-1.803125e-05) * (q[i16] * q[i21]) + (5.298471e-05) * (q[i16] * q[i22])
            + (-1.253923e-04) * (q[i19] * q[i20]) + (-1.100553e-04) * (q[i19] * q[i21]) + (-2.068554e-05) * (q[i19] * q[i22])
            + (7.477975e-05) * (q[i20] * q[i21]) + (5.951271e-05) * (q[i20] * q[i22]) + (2.709701e-06) * (q[i21] * q[i22]);
      JQ[1][i11] = (2.399867e-03) * (1) + (5.615755e-03) * ((2) * q[i11]) + (9.983218e-04) * (q[i0]) + (1.726736e-03) * (q[i1]) + (5.238081e-03) * (q[i2])
            + (-1.025216e-03) * (q[i3]) + (-1.811008e-03) * (q[i4]) + (6.094528e-03) * (q[i5]) + (2.714796e-03) * (q[i6]) + (4.318452e-03) * (q[i7])
            + (1.045162e-02) * (q[i8]) + (1.107012e-04) * (q[i9]) + (5.861028e-04) * (q[i10]) + (7.525369e-06) * (q[i12]) + (2.331858e-03) * (q[i15])
            + (-1.170935e-04) * (q[i16]) + (-8.968728e-04) * (q[i19]) + (9.723991e-05) * (q[i20]) + (9.362029e-04) * (q[i21]) + (6.837693e-05) * (q[i22])
            + (2.165368e-04) * (q[i0] * q[i0]) + (5.702403e-04) * (q[i1] * q[i1]) + (2.758351e-04) * (q[i2] * q[i2]) + (-2.580375e-04) * (q[i3] * q[i3])
            + (-1.083834e-03) * (q[i4] * q[i4]) + (1.824046e-05) * (q[i5] * q[i5]) + (1.943749e-04) * (q[i6] * q[i6]) + (-1.432442e-04) * (q[i7] * q[i7])
            + (-9.334205e-04) * (q[i8] * q[i8]) + (-1.004519e-04) * (q[i9] * q[i9]) + (-2.487220e-04) * (q[i10] * q[i10])
            + (4.014348e-04) * ((2) * q[i0] * q[i11]) + (-2.445407e-04) * ((2) * q[i1] * q[i11]) + (2.331302e-04) * ((2) * q[i2] * q[i11])
            + (-4.269491e-04) * ((2) * q[i3] * q[i11]) + (4.152397e-04) * ((2) * q[i4] * q[i11]) + (-1.022053e-03) * ((2) * q[i5] * q[i11])
            + (-3.015865e-04) * ((2) * q[i6] * q[i11]) + (-3.222720e-04) * ((2) * q[i7] * q[i11]) + (4.575323e-04) * ((2) * q[i8] * q[i11])
            + (6.008311e-05) * ((2) * q[i9] * q[i11]) + (2.155967e-04) * ((2) * q[i10] * q[i11]) + (9.294861e-05) * ((3) * q[i11] * q[i11])
            + (-5.585157e-05) * ((2) * q[i11] * q[i12]) + (1.792949e-03) * ((2) * q[i11] * q[i15]) + (1.490305e-04) * ((2) * q[i11] * q[i16])
            + (5.161947e-04) * ((2) * q[i11] * q[i19]) + (-1.333448e-04) * ((2) * q[i11] * q[i20]) + (9.612420e-04) * ((2) * q[i11] * q[i21])
            + (-1.220038e-04) * ((2) * q[i11] * q[i22]) + (-5.184264e-05) * (q[i12] * q[i12]) + (2.203379e-03) * (q[i15] * q[i15])
            + (6.585089e-05) * (q[i16] * q[i16]) + (5.587782e-05) * (q[i19] * q[i19]) + (-8.713647e-05) * (q[i20] * q[i20]) + (4.934464e-04) * (q[i21] * q[i21])
            + (-9.397988e-06) * (q[i22] * q[i22]) + (-6.081333e-04) * (q[i0] * q[i1]) + (1.562511e-04) * (q[i0] * q[i2]) + (-1.845070e-03) * (q[i0] * q[i3])
            + (3.848988e-04) * (q[i0] * q[i4]) + (1.515204e-03) * (q[i0] * q[i5]) + (-7.742778e-04) * (q[i0] * q[i6]) + (-2.102670e-04) * (q[i0] * q[i7])
            + (-5.047233e-04) * (q[i0] * q[i8]) + (-8.545944e-05) * (q[i0] * q[i9]) + (-6.850180e-06) * (q[i0] * q[i10]) + (1.621298e-04) * (q[i0] * q[i12])
            + (1.406522e-04) * (q[i0] * q[i15]) + (3.678196e-04) * (q[i0] * q[i16]) + (1.275807e-04) * (q[i0] * q[i19]) + (6.317997e-05) * (q[i0] * q[i20])
            + (2.023471e-04) * (q[i0] * q[i21]) + (-1.685427e-04) * (q[i0] * q[i22]) + (8.179783e-04) * (q[i1] * q[i2]) + (1.973673e-04) * (q[i1] * q[i3])
            + (-1.600283e-03) * (q[i1] * q[i4]) + (-3.894842e-04) * (q[i1] * q[i5]) + (8.367449e-04) * (q[i1] * q[i6]) + (-2.759820e-04) * (q[i1] * q[i7])
            + (-5.762647e-04) * (q[i1] * q[i8]) + (-3.580780e-06) * (q[i1] * q[i9]) + (-3.052562e-04) * (q[i1] * q[i10]) + (1.603131e-04) * (q[i1] * q[i12])
            + (5.024914e-04) * (q[i1] * q[i15]) + (-1.164569e-04) * (q[i1] * q[i16]) + (1.243312e-04) * (q[i1] * q[i19]) + (-6.589012e-05) * (q[i1] * q[i20])
            + (1.314783e-05) * (q[i1] * q[i21]) + (2.562168e-04) * (q[i1] * q[i22]) + (-3.181467e-05) * (q[i2] * q[i3]) + (7.071592e-05) * (q[i2] * q[i4])
            + (2.228351e-03) * (q[i2] * q[i5]) + (-7.221575e-04) * (q[i2] * q[i6]) + (3.092087e-05) * (q[i2] * q[i7]) + (-9.728021e-04) * (q[i2] * q[i8])
            + (-1.846460e-04) * (q[i2] * q[i9]) + (4.245635e-05) * (q[i2] * q[i10]) + (-3.372752e-04) * (q[i2] * q[i12]) + (3.213017e-03) * (q[i2] * q[i15])
            + (-2.787331e-04) * (q[i2] * q[i16]) + (9.466732e-04) * (q[i2] * q[i19]) + (-2.194630e-04) * (q[i2] * q[i20]) + (7.741898e-04) * (q[i2] * q[i21])
            + (-7.041495e-05) * (q[i2] * q[i22]) + (2.821409e-04) * (q[i3] * q[i4]) + (-1.227979e-03) * (q[i3] * q[i5]) + (-2.072890e-04) * (q[i3] * q[i6])
            + (-3.172781e-04) * (q[i3] * q[i7]) + (-8.162157e-04) * (q[i3] * q[i8]) + (-1.778195e-04) * (q[i3] * q[i9]) + (-1.247490e-04) * (q[i3] * q[i10])
            + (-1.237553e-05) * (q[i3] * q[i12]) + (-9.387109e-04) * (q[i3] * q[i15]) + (4.190322e-04) * (q[i3] * q[i16]) + (1.874694e-04) * (q[i3] * q[i19])
            + (-1.119682e-04) * (q[i3] * q[i20]) + (-3.660009e-04) * (q[i3] * q[i21]) + (-9.408255e-05) * (q[i3] * q[i22]) + (6.574422e-04) * (q[i4] * q[i5])
            + (-5.990457e-04) * (q[i4] * q[i6]) + (2.784186e-05) * (q[i4] * q[i7]) + (-5.216702e-04) * (q[i4] * q[i8]) + (-3.400631e-04) * (q[i4] * q[i9])
            + (2.572872e-04) * (q[i4] * q[i10]) + (-8.308391e-06) * (q[i4] * q[i12]) + (1.431981e-04) * (q[i4] * q[i15]) + (-5.401505e-04) * (q[i4] * q[i16])
            + (-3.156915e-04) * (q[i4] * q[i19]) + (1.557434e-04) * (q[i4] * q[i20]) + (-2.194707e-04) * (q[i4] * q[i21]) + (-3.941864e-05) * (q[i4] * q[i22])
            + (-2.555128e-04) * (q[i5] * q[i6]) + (5.000242e-04) * (q[i5] * q[i7]) + (2.327999e-03) * (q[i5] * q[i8]) + (8.956663e-05) * (q[i5] * q[i9])
            + (1.925860e-04) * (q[i5] * q[i10]) + (4.851027e-05) * (q[i5] * q[i12]) + (1.888604e-03) * (q[i5] * q[i15]) + (4.896294e-05) * (q[i5] * q[i16])
            + (8.125725e-04) * (q[i5] * q[i19]) + (-3.515285e-04) * (q[i5] * q[i20]) + (7.588162e-04) * (q[i5] * q[i21]) + (-1.558879e-04) * (q[i5] * q[i22])
            + (9.157569e-04) * (q[i6] * q[i7]) + (-3.951089e-04) * (q[i6] * q[i8]) + (-1.914007e-04) * (q[i6] * q[i9]) + (2.801582e-04) * (q[i6] * q[i10])
            + (3.259319e-04) * (q[i6] * q[i12]) + (8.257468e-04) * (q[i6] * q[i15]) + (8.527838e-05) * (q[i6] * q[i16]) + (5.202263e-05) * (q[i6] * q[i19])
            + (-4.364655e-05) * (q[i6] * q[i20]) + (3.742519e-04) * (q[i6] * q[i21]) + (1.222552e-04) * (q[i6] * q[i22]) + (-5.749266e-04) * (q[i7] * q[i8])
            + (2.795555e-04) * (q[i7] * q[i9]) + (-4.982814e-04) * (q[i7] * q[i10]) + (-3.273813e-04) * (q[i7] * q[i12]) + (9.646963e-04) * (q[i7] * q[i15])
            + (4.343167e-04) * (q[i7] * q[i16]) + (5.168748e-05) * (q[i7] * q[i19]) + (-7.278678e-05) * (q[i7] * q[i20]) + (1.227451e-04) * (q[i7] * q[i21])
            + (1.142882e-04) * (q[i7] * q[i22]) + (4.649929e-04) * (q[i8] * q[i9]) + (7.121912e-04) * (q[i8] * q[i10]) + (-3.659516e-06) * (q[i8] * q[i12])
            + (3.760734e-03) * (q[i8] * q[i15]) + (1.553745e-04) * (q[i8] * q[i16]) + (3.075091e-04) * (q[i8] * q[i19]) + (4.150482e-05) * (q[i8] * q[i20])
            + (1.772542e-03) * (q[i8] * q[i21]) + (8.801020e-05) * (q[i8] * q[i22]) + (8.753649e-05) * (q[i9] * q[i10]) + (-5.540778e-05) * (q[i9] * q[i12])
            + (-3.124145e-04) * (q[i9] * q[i15]) + (-2.319001e-05) * (q[i9] * q[i16]) + (1.114000e-05) * (q[i9] * q[i19]) + (-3.336551e-05) * (q[i9] * q[i20])
            + (1.422593e-04) * (q[i9] * q[i21]) + (-7.166301e-05) * (q[i9] * q[i22]) + (5.493271e-05) * (q[i10] * q[i12]) + (-1.025170e-04) * (q[i10] * q[i15])
            + (-1.252333e-04) * (q[i10] * q[i16]) + (1.704197e-04) * (q[i10] * q[i19]) + (-7.218964e-05) * (q[i10] * q[i20])
            + (-1.410711e-04) * (q[i10] * q[i21]) + (-2.435788e-05) * (q[i10] * q[i22]) + (1.944444e-04) * (q[i12] * q[i15])
            + (-1.939213e-04) * (q[i12] * q[i16]) + (3.515086e-04) * (q[i12] * q[i19]) + (-3.487330e-04) * (q[i12] * q[i20])
            + (-1.346398e-04) * (q[i12] * q[i21]) + (-1.364356e-04) * (q[i12] * q[i22]) + (-2.880563e-04) * (q[i15] * q[i16])
            + (-2.796700e-04) * (q[i15] * q[i19]) + (-8.792055e-05) * (q[i15] * q[i20]) + (-5.240005e-04) * (q[i15] * q[i21])
            + (3.849008e-05) * (q[i15] * q[i22]) + (1.741379e-06) * (q[i16] * q[i19]) + (2.023173e-04) * (q[i16] * q[i20]) + (-1.639818e-04) * (q[i16] * q[i21])
            + (-1.462666e-04) * (q[i16] * q[i22]) + (4.498088e-05) * (q[i19] * q[i20]) + (-1.951025e-03) * (q[i19] * q[i21])
            + (6.040702e-05) * (q[i19] * q[i22]) + (-9.114553e-05) * (q[i20] * q[i21]) + (-3.234788e-05) * (q[i20] * q[i22])
            + (2.329214e-05) * (q[i21] * q[i22]);
      JQ[1][i12] = (2.716937e-03) * (1) + (-5.770524e-03) * ((2) * q[i12]) + (-1.750409e-03) * (q[i0]) + (-9.807593e-04) * (q[i1]) + (-5.424067e-03) * (q[i2])
            + (1.850945e-03) * (q[i3]) + (1.046608e-03) * (q[i4]) + (-6.165370e-03) * (q[i5]) + (4.332843e-03) * (q[i6]) + (2.696326e-03) * (q[i7])
            + (1.053923e-02) * (q[i8]) + (5.910216e-04) * (q[i9]) + (1.030351e-04) * (q[i10]) + (7.525369e-06) * (q[i11]) + (-9.949890e-05) * (q[i15])
            + (2.418913e-03) * (q[i16]) + (9.553798e-05) * (q[i19]) + (-9.368650e-04) * (q[i20]) + (-7.341442e-05) * (q[i21]) + (-9.423373e-04) * (q[i22])
            + (5.653810e-04) * (q[i0] * q[i0]) + (2.120748e-04) * (q[i1] * q[i1]) + (2.583368e-04) * (q[i2] * q[i2]) + (-1.087573e-03) * (q[i3] * q[i3])
            + (-2.591561e-04) * (q[i4] * q[i4]) + (3.115890e-06) * (q[i5] * q[i5]) + (-1.374691e-04) * (q[i6] * q[i6]) + (1.929896e-04) * (q[i7] * q[i7])
            + (-9.748763e-04) * (q[i8] * q[i8]) + (-2.476464e-04) * (q[i9] * q[i9]) + (-9.833280e-05) * (q[i10] * q[i10]) + (-5.585157e-05) * (q[i11] * q[i11])
            + (-2.714360e-04) * ((2) * q[i0] * q[i12]) + (3.989079e-04) * ((2) * q[i1] * q[i12]) + (1.398483e-04) * ((2) * q[i2] * q[i12])
            + (4.375516e-04) * ((2) * q[i3] * q[i12]) + (-4.336171e-04) * ((2) * q[i4] * q[i12]) + (-1.094256e-03) * ((2) * q[i5] * q[i12])
            + (3.419038e-04) * ((2) * q[i6] * q[i12]) + (3.116004e-04) * ((2) * q[i7] * q[i12]) + (-3.806106e-04) * ((2) * q[i8] * q[i12])
            + (-2.273791e-04) * ((2) * q[i9] * q[i12]) + (-6.828272e-05) * ((2) * q[i10] * q[i12]) + (-5.184264e-05) * ((2) * q[i11] * q[i12])
            + (4.199221e-05) * ((3) * q[i12] * q[i12]) + (-1.595047e-04) * ((2) * q[i12] * q[i15]) + (-1.814456e-03) * ((2) * q[i12] * q[i16])
            + (1.325286e-04) * ((2) * q[i12] * q[i19]) + (-4.996383e-04) * ((2) * q[i12] * q[i20]) + (-1.226814e-04) * ((2) * q[i12] * q[i21])
            + (9.664559e-04) * ((2) * q[i12] * q[i22]) + (7.106387e-05) * (q[i15] * q[i15]) + (2.237602e-03) * (q[i16] * q[i16])
            + (-8.649770e-05) * (q[i19] * q[i19]) + (7.128725e-05) * (q[i20] * q[i20]) + (-8.258681e-06) * (q[i21] * q[i21])
            + (4.990193e-04) * (q[i22] * q[i22]) + (-6.105035e-04) * (q[i0] * q[i1]) + (8.152518e-04) * (q[i0] * q[i2]) + (-1.582897e-03) * (q[i0] * q[i3])
            + (1.873872e-04) * (q[i0] * q[i4]) + (-3.911966e-04) * (q[i0] * q[i5]) + (2.973193e-04) * (q[i0] * q[i6]) + (-8.309921e-04) * (q[i0] * q[i7])
            + (6.275289e-04) * (q[i0] * q[i8]) + (3.097661e-04) * (q[i0] * q[i9]) + (4.021818e-06) * (q[i0] * q[i10]) + (1.621298e-04) * (q[i0] * q[i11])
            + (1.160218e-04) * (q[i0] * q[i15]) + (-5.135890e-04) * (q[i0] * q[i16]) + (6.372694e-05) * (q[i0] * q[i19]) + (-1.227215e-04) * (q[i0] * q[i20])
            + (2.551483e-04) * (q[i0] * q[i21]) + (1.660179e-05) * (q[i0] * q[i22]) + (1.544878e-04) * (q[i1] * q[i2]) + (3.859845e-04) * (q[i1] * q[i3])
            + (-1.832126e-03) * (q[i1] * q[i4]) + (1.525443e-03) * (q[i1] * q[i5]) + (2.208027e-04) * (q[i1] * q[i6]) + (7.933086e-04) * (q[i1] * q[i7])
            + (5.423433e-04) * (q[i1] * q[i8]) + (3.420878e-06) * (q[i1] * q[i9]) + (8.281640e-05) * (q[i1] * q[i10]) + (1.603131e-04) * (q[i1] * q[i11])
            + (-3.718481e-04) * (q[i1] * q[i15]) + (-1.442976e-04) * (q[i1] * q[i16]) + (-6.553262e-05) * (q[i1] * q[i19]) + (-1.224903e-04) * (q[i1] * q[i20])
            + (-1.676125e-04) * (q[i1] * q[i21]) + (2.016792e-04) * (q[i1] * q[i22]) + (9.610482e-05) * (q[i2] * q[i3]) + (-7.793834e-06) * (q[i2] * q[i4])
            + (2.197321e-03) * (q[i2] * q[i5]) + (-2.130224e-05) * (q[i2] * q[i6]) + (7.329934e-04) * (q[i2] * q[i7]) + (1.148406e-03) * (q[i2] * q[i8])
            + (-5.668092e-05) * (q[i2] * q[i9]) + (1.703269e-04) * (q[i2] * q[i10]) + (-3.372752e-04) * (q[i2] * q[i11]) + (2.604640e-04) * (q[i2] * q[i15])
            + (-3.231733e-03) * (q[i2] * q[i16]) + (2.152759e-04) * (q[i2] * q[i19]) + (-9.212270e-04) * (q[i2] * q[i20]) + (-7.210027e-05) * (q[i2] * q[i21])
            + (7.730127e-04) * (q[i2] * q[i22]) + (2.781655e-04) * (q[i3] * q[i4]) + (6.743793e-04) * (q[i3] * q[i5]) + (-2.334922e-05) * (q[i3] * q[i6])
            + (6.034648e-04) * (q[i3] * q[i7]) + (5.571022e-04) * (q[i3] * q[i8]) + (-2.674895e-04) * (q[i3] * q[i9]) + (3.478084e-04) * (q[i3] * q[i10])
            + (-1.237553e-05) * (q[i3] * q[i11]) + (5.437707e-04) * (q[i3] * q[i15]) + (-1.435395e-04) * (q[i3] * q[i16]) + (-1.542441e-04) * (q[i3] * q[i19])
            + (3.180409e-04) * (q[i3] * q[i20]) + (-4.124738e-05) * (q[i3] * q[i21]) + (-2.224609e-04) * (q[i3] * q[i22]) + (-1.211321e-03) * (q[i4] * q[i5])
            + (3.041727e-04) * (q[i4] * q[i6]) + (2.025499e-04) * (q[i4] * q[i7]) + (8.343474e-04) * (q[i4] * q[i8]) + (1.283953e-04) * (q[i4] * q[i9])
            + (1.697263e-04) * (q[i4] * q[i10]) + (-8.308391e-06) * (q[i4] * q[i11]) + (-4.132159e-04) * (q[i4] * q[i15]) + (9.410504e-04) * (q[i4] * q[i16])
            + (1.068496e-04) * (q[i4] * q[i19]) + (-1.928976e-04) * (q[i4] * q[i20]) + (-9.924322e-05) * (q[i4] * q[i21]) + (-3.666046e-04) * (q[i4] * q[i22])
            + (-5.092444e-04) * (q[i5] * q[i6]) + (2.595822e-04) * (q[i5] * q[i7]) + (-2.406813e-03) * (q[i5] * q[i8]) + (-2.005148e-04) * (q[i5] * q[i9])
            + (-9.604946e-05) * (q[i5] * q[i10]) + (4.851027e-05) * (q[i5] * q[i11]) + (-6.515054e-05) * (q[i5] * q[i15]) + (-1.907801e-03) * (q[i5] * q[i16])
            + (3.512644e-04) * (q[i5] * q[i19]) + (-8.135706e-04) * (q[i5] * q[i20]) + (-1.486958e-04) * (q[i5] * q[i21]) + (7.760778e-04) * (q[i5] * q[i22])
            + (9.096246e-04) * (q[i6] * q[i7]) + (-6.113200e-04) * (q[i6] * q[i8]) + (-4.993365e-04) * (q[i6] * q[i9]) + (2.759020e-04) * (q[i6] * q[i10])
            + (3.259319e-04) * (q[i6] * q[i11]) + (4.345391e-04) * (q[i6] * q[i15]) + (9.717923e-04) * (q[i6] * q[i16]) + (-6.807007e-05) * (q[i6] * q[i19])
            + (5.282220e-05) * (q[i6] * q[i20]) + (-1.138935e-04) * (q[i6] * q[i21]) + (-1.274102e-04) * (q[i6] * q[i22]) + (-3.858998e-04) * (q[i7] * q[i8])
            + (2.772482e-04) * (q[i7] * q[i9]) + (-1.927251e-04) * (q[i7] * q[i10]) + (-3.273813e-04) * (q[i7] * q[i11]) + (8.093829e-05) * (q[i7] * q[i15])
            + (8.227340e-04) * (q[i7] * q[i16]) + (-4.373682e-05) * (q[i7] * q[i19]) + (5.232382e-05) * (q[i7] * q[i20]) + (-1.195996e-04) * (q[i7] * q[i21])
            + (-3.711772e-04) * (q[i7] * q[i22]) + (7.230965e-04) * (q[i8] * q[i9]) + (4.641616e-04) * (q[i8] * q[i10]) + (-3.659516e-06) * (q[i8] * q[i11])
            + (1.540005e-04) * (q[i8] * q[i15]) + (3.777960e-03) * (q[i8] * q[i16]) + (4.182337e-05) * (q[i8] * q[i19]) + (2.916899e-04) * (q[i8] * q[i20])
            + (-8.636645e-05) * (q[i8] * q[i21]) + (-1.784999e-03) * (q[i8] * q[i22]) + (8.681458e-05) * (q[i9] * q[i10]) + (-5.540778e-05) * (q[i9] * q[i11])
            + (-1.269955e-04) * (q[i9] * q[i15]) + (-1.029408e-04) * (q[i9] * q[i16]) + (-7.128775e-05) * (q[i9] * q[i19]) + (1.669239e-04) * (q[i9] * q[i20])
            + (2.485172e-05) * (q[i9] * q[i21]) + (1.425158e-04) * (q[i9] * q[i22]) + (5.493271e-05) * (q[i10] * q[i11]) + (-2.486485e-05) * (q[i10] * q[i15])
            + (-3.109391e-04) * (q[i10] * q[i16]) + (-3.370367e-05) * (q[i10] * q[i19]) + (1.260359e-05) * (q[i10] * q[i20])
            + (7.388342e-05) * (q[i10] * q[i21]) + (-1.428886e-04) * (q[i10] * q[i22]) + (1.944444e-04) * (q[i11] * q[i15])
            + (-1.939213e-04) * (q[i11] * q[i16]) + (3.515086e-04) * (q[i11] * q[i19]) + (-3.487330e-04) * (q[i11] * q[i20])
            + (-1.346398e-04) * (q[i11] * q[i21]) + (-1.364356e-04) * (q[i11] * q[i22]) + (-2.832443e-04) * (q[i15] * q[i16])
            + (2.032436e-04) * (q[i15] * q[i19]) + (-7.861044e-07) * (q[i15] * q[i20]) + (1.456073e-04) * (q[i15] * q[i21]) + (1.656489e-04) * (q[i15] * q[i22])
            + (-8.642033e-05) * (q[i16] * q[i19]) + (-2.879309e-04) * (q[i16] * q[i20]) + (-3.884873e-05) * (q[i16] * q[i21])
            + (5.267210e-04) * (q[i16] * q[i22]) + (4.564215e-05) * (q[i19] * q[i20]) + (3.084258e-05) * (q[i19] * q[i21]) + (9.309488e-05) * (q[i19] * q[i22])
            + (-6.337097e-05) * (q[i20] * q[i21]) + (1.943681e-03) * (q[i20] * q[i22]) + (2.400665e-05) * (q[i21] * q[i22]);
      JQ[1][i15] = (-2.752179e-03) * (1) + (5.437961e-03) * ((2) * q[i15]) + (1.361736e-03) * (q[i0]) + (1.865620e-03) * (q[i1]) + (2.942736e-03) * (q[i2])
            + (4.404436e-03) * (q[i3]) + (4.666836e-03) * (q[i4]) + (-1.554917e-02) * (q[i5]) + (-7.229995e-04) * (q[i6]) + (1.142265e-03) * (q[i7])
            + (8.056936e-03) * (q[i8]) + (-2.286462e-04) * (q[i9]) + (-2.416712e-04) * (q[i10]) + (2.331858e-03) * (q[i11]) + (-9.949890e-05) * (q[i12])
            + (-4.424392e-06) * (q[i16]) + (7.760659e-04) * (q[i19]) + (-2.762830e-04) * (q[i20]) + (2.007150e-03) * (q[i21]) + (1.439286e-04) * (q[i22])
            + (-4.210967e-04) * (q[i0] * q[i0]) + (-4.106177e-04) * (q[i1] * q[i1]) + (-2.525173e-03) * (q[i2] * q[i2]) + (6.435341e-04) * (q[i3] * q[i3])
            + (-5.605486e-04) * (q[i4] * q[i4]) + (6.431482e-07) * (q[i5] * q[i5]) + (-4.760048e-05) * (q[i6] * q[i6]) + (-4.862562e-04) * (q[i7] * q[i7])
            + (4.136062e-03) * (q[i8] * q[i8]) + (3.525948e-05) * (q[i9] * q[i9]) + (-2.159147e-04) * (q[i10] * q[i10]) + (1.792949e-03) * (q[i11] * q[i11])
            + (-1.595047e-04) * (q[i12] * q[i12]) + (2.282337e-04) * ((2) * q[i0] * q[i15]) + (1.897088e-04) * ((2) * q[i1] * q[i15])
            + (1.852820e-03) * ((2) * q[i2] * q[i15]) + (2.706537e-04) * ((2) * q[i3] * q[i15]) + (5.376847e-04) * ((2) * q[i4] * q[i15])
            + (-2.289652e-03) * ((2) * q[i5] * q[i15]) + (-1.665728e-05) * ((2) * q[i6] * q[i15]) + (-2.762074e-05) * ((2) * q[i7] * q[i15])
            + (2.054755e-03) * ((2) * q[i8] * q[i15]) + (2.068098e-05) * ((2) * q[i9] * q[i15]) + (7.112707e-05) * ((2) * q[i10] * q[i15])
            + (2.203379e-03) * ((2) * q[i11] * q[i15]) + (7.106387e-05) * ((2) * q[i12] * q[i15]) + (5.023342e-04) * ((3) * q[i15] * q[i15])
            + (8.242057e-06) * ((2) * q[i15] * q[i16]) + (1.147565e-04) * ((2) * q[i15] * q[i19]) + (-3.720846e-05) * ((2) * q[i15] * q[i20])
            + (5.641917e-04) * ((2) * q[i15] * q[i21]) + (4.040847e-05) * ((2) * q[i15] * q[i22]) + (-8.383803e-06) * (q[i16] * q[i16])
            + (2.662236e-04) * (q[i19] * q[i19]) + (-6.891444e-05) * (q[i20] * q[i20]) + (7.005584e-04) * (q[i21] * q[i21]) + (1.858024e-05) * (q[i22] * q[i22])
            + (7.877042e-04) * (q[i0] * q[i1]) + (-7.861968e-04) * (q[i0] * q[i2]) + (1.435510e-03) * (q[i0] * q[i3]) + (4.058198e-04) * (q[i0] * q[i4])
            + (1.990528e-04) * (q[i0] * q[i5]) + (1.582378e-03) * (q[i0] * q[i6]) + (-4.744342e-04) * (q[i0] * q[i7]) + (4.118524e-04) * (q[i0] * q[i8])
            + (3.770447e-04) * (q[i0] * q[i9]) + (2.035634e-04) * (q[i0] * q[i10]) + (1.406522e-04) * (q[i0] * q[i11]) + (1.160218e-04) * (q[i0] * q[i12])
            + (1.680045e-04) * (q[i0] * q[i16]) + (-1.696522e-04) * (q[i0] * q[i19]) + (-8.786154e-05) * (q[i0] * q[i20]) + (4.839724e-04) * (q[i0] * q[i21])
            + (9.873012e-05) * (q[i0] * q[i22]) + (-2.282695e-03) * (q[i1] * q[i2]) + (-1.024926e-04) * (q[i1] * q[i3]) + (-9.721177e-04) * (q[i1] * q[i4])
            + (1.882025e-04) * (q[i1] * q[i5]) + (-3.151395e-04) * (q[i1] * q[i6]) + (1.680669e-03) * (q[i1] * q[i7]) + (9.890983e-04) * (q[i1] * q[i8])
            + (-9.988166e-05) * (q[i1] * q[i9]) + (2.340344e-04) * (q[i1] * q[i10]) + (5.024914e-04) * (q[i1] * q[i11]) + (-3.718481e-04) * (q[i1] * q[i12])
            + (1.628166e-04) * (q[i1] * q[i16]) + (2.174304e-04) * (q[i1] * q[i19]) + (-2.712922e-04) * (q[i1] * q[i20]) + (7.240107e-05) * (q[i1] * q[i21])
            + (6.014123e-06) * (q[i1] * q[i22]) + (1.055586e-03) * (q[i2] * q[i3]) + (6.335989e-04) * (q[i2] * q[i4]) + (-1.390519e-03) * (q[i2] * q[i5])
            + (6.749095e-04) * (q[i2] * q[i6]) + (-4.618498e-04) * (q[i2] * q[i7]) + (5.609538e-03) * (q[i2] * q[i8]) + (5.058564e-06) * (q[i2] * q[i9])
            + (-1.328026e-04) * (q[i2] * q[i10]) + (3.213017e-03) * (q[i2] * q[i11]) + (2.604640e-04) * (q[i2] * q[i12]) + (1.514747e-05) * (q[i2] * q[i16])
            + (-7.424004e-04) * (q[i2] * q[i19]) + (-4.808458e-04) * (q[i2] * q[i20]) + (1.176342e-05) * (q[i2] * q[i21]) + (2.144045e-04) * (q[i2] * q[i22])
            + (3.733548e-04) * (q[i3] * q[i4]) + (8.657755e-04) * (q[i3] * q[i5]) + (2.047130e-04) * (q[i3] * q[i6]) + (-2.640983e-04) * (q[i3] * q[i7])
            + (-5.793535e-04) * (q[i3] * q[i8]) + (-2.490409e-04) * (q[i3] * q[i9]) + (1.607090e-04) * (q[i3] * q[i10]) + (-9.387109e-04) * (q[i3] * q[i11])
            + (5.437707e-04) * (q[i3] * q[i12]) + (2.630016e-04) * (q[i3] * q[i16]) + (-2.487628e-04) * (q[i3] * q[i19]) + (8.713109e-05) * (q[i3] * q[i20])
            + (3.401230e-04) * (q[i3] * q[i21]) + (1.075916e-05) * (q[i3] * q[i22]) + (3.483358e-04) * (q[i4] * q[i5]) + (-3.383068e-04) * (q[i4] * q[i6])
            + (-9.273298e-04) * (q[i4] * q[i7]) + (-5.722809e-04) * (q[i4] * q[i8]) + (1.065911e-04) * (q[i4] * q[i9]) + (-6.359112e-04) * (q[i4] * q[i10])
            + (1.431981e-04) * (q[i4] * q[i11]) + (-4.132159e-04) * (q[i4] * q[i12]) + (2.579163e-04) * (q[i4] * q[i16]) + (1.673949e-04) * (q[i4] * q[i19])
            + (-2.553386e-05) * (q[i4] * q[i20]) + (6.246916e-04) * (q[i4] * q[i21]) + (2.248961e-04) * (q[i4] * q[i22]) + (-1.719568e-05) * (q[i5] * q[i6])
            + (1.080666e-04) * (q[i5] * q[i7]) + (2.348362e-03) * (q[i5] * q[i8]) + (-3.390928e-04) * (q[i5] * q[i9]) + (-2.919496e-04) * (q[i5] * q[i10])
            + (1.888604e-03) * (q[i5] * q[i11]) + (-6.515054e-05) * (q[i5] * q[i12]) + (-4.701687e-04) * (q[i5] * q[i16]) + (-1.713542e-04) * (q[i5] * q[i19])
            + (-2.484216e-04) * (q[i5] * q[i20]) + (-1.837282e-03) * (q[i5] * q[i21]) + (-1.835453e-04) * (q[i5] * q[i22]) + (-2.486836e-04) * (q[i6] * q[i7])
            + (4.453109e-04) * (q[i6] * q[i8]) + (-1.714614e-04) * (q[i6] * q[i9]) + (-4.925413e-04) * (q[i6] * q[i10]) + (8.257468e-04) * (q[i6] * q[i11])
            + (4.345391e-04) * (q[i6] * q[i12]) + (-3.368392e-04) * (q[i6] * q[i16]) + (2.110082e-04) * (q[i6] * q[i19]) + (1.025149e-05) * (q[i6] * q[i20])
            + (-4.249887e-05) * (q[i6] * q[i21]) + (-3.835844e-05) * (q[i6] * q[i22]) + (-6.792527e-05) * (q[i7] * q[i8]) + (1.556296e-04) * (q[i7] * q[i9])
            + (-3.596411e-04) * (q[i7] * q[i10]) + (9.646963e-04) * (q[i7] * q[i11]) + (8.093829e-05) * (q[i7] * q[i12]) + (3.378316e-04) * (q[i7] * q[i16])
            + (2.100835e-04) * (q[i7] * q[i19]) + (2.107295e-04) * (q[i7] * q[i20]) + (1.401101e-04) * (q[i7] * q[i21]) + (-1.217494e-04) * (q[i7] * q[i22])
            + (-2.590363e-04) * (q[i8] * q[i9]) + (-7.107874e-05) * (q[i8] * q[i10]) + (3.760734e-03) * (q[i8] * q[i11]) + (1.540005e-04) * (q[i8] * q[i12])
            + (6.105476e-07) * (q[i8] * q[i16]) + (3.855031e-04) * (q[i8] * q[i19]) + (-1.373722e-04) * (q[i8] * q[i20]) + (4.243812e-04) * (q[i8] * q[i21])
            + (1.451907e-04) * (q[i8] * q[i22]) + (3.084963e-05) * (q[i9] * q[i10]) + (-3.124145e-04) * (q[i9] * q[i11]) + (-1.269955e-04) * (q[i9] * q[i12])
            + (6.452166e-05) * (q[i9] * q[i16]) + (9.578845e-05) * (q[i9] * q[i19]) + (4.479746e-05) * (q[i9] * q[i20]) + (5.512411e-05) * (q[i9] * q[i21])
            + (-1.941762e-05) * (q[i9] * q[i22]) + (-1.025170e-04) * (q[i10] * q[i11]) + (-2.486485e-05) * (q[i10] * q[i12])
            + (-6.514242e-05) * (q[i10] * q[i16]) + (6.426175e-05) * (q[i10] * q[i19]) + (9.496067e-05) * (q[i10] * q[i20])
            + (-2.781572e-05) * (q[i10] * q[i21]) + (-2.911934e-05) * (q[i10] * q[i22]) + (1.944444e-04) * (q[i11] * q[i12])
            + (-2.880563e-04) * (q[i11] * q[i16]) + (-2.796700e-04) * (q[i11] * q[i19]) + (-8.792055e-05) * (q[i11] * q[i20])
            + (-5.240005e-04) * (q[i11] * q[i21]) + (3.849008e-05) * (q[i11] * q[i22]) + (-2.832443e-04) * (q[i12] * q[i16])
            + (2.032436e-04) * (q[i12] * q[i19]) + (-7.861044e-07) * (q[i12] * q[i20]) + (1.456073e-04) * (q[i12] * q[i21]) + (1.656489e-04) * (q[i12] * q[i22])
            + (3.736643e-06) * (q[i16] * q[i19]) + (-4.263416e-06) * (q[i16] * q[i20]) + (5.189304e-05) * (q[i16] * q[i21]) + (5.324950e-05) * (q[i16] * q[i22])
            + (-5.533933e-05) * (q[i19] * q[i20]) + (2.051240e-04) * (q[i19] * q[i21]) + (-1.997200e-05) * (q[i19] * q[i22])
            + (-3.264947e-05) * (q[i20] * q[i21]) + (4.061760e-05) * (q[i20] * q[i22]) + (4.119159e-05) * (q[i21] * q[i22]);
      JQ[1][i16] = (2.667112e-03) * (1) + (-5.490711e-03) * ((2) * q[i16]) + (1.880785e-03) * (q[i0]) + (1.351955e-03) * (q[i1]) + (3.012659e-03) * (q[i2])
            + (4.746587e-03) * (q[i3]) + (4.427214e-03) * (q[i4]) + (-1.573297e-02) * (q[i5]) + (-1.151517e-03) * (q[i6]) + (7.310233e-04) * (q[i7])
            + (-8.161851e-03) * (q[i8]) + (2.458141e-04) * (q[i9]) + (2.268613e-04) * (q[i10]) + (-1.170935e-04) * (q[i11]) + (2.418913e-03) * (q[i12])
            + (-4.424392e-06) * (q[i15]) + (2.703000e-04) * (q[i19]) + (-7.458541e-04) * (q[i20]) + (1.446155e-04) * (q[i21]) + (2.009953e-03) * (q[i22])
            + (4.122016e-04) * (q[i0] * q[i0]) + (4.250499e-04) * (q[i1] * q[i1]) + (2.560187e-03) * (q[i2] * q[i2]) + (5.647602e-04) * (q[i3] * q[i3])
            + (-6.537892e-04) * (q[i4] * q[i4]) + (1.903344e-06) * (q[i5] * q[i5]) + (4.921217e-04) * (q[i6] * q[i6]) + (3.594144e-05) * (q[i7] * q[i7])
            + (-4.181311e-03) * (q[i8] * q[i8]) + (2.191663e-04) * (q[i9] * q[i9]) + (-3.608072e-05) * (q[i10] * q[i10]) + (1.490305e-04) * (q[i11] * q[i11])
            + (-1.814456e-03) * (q[i12] * q[i12]) + (8.242057e-06) * (q[i15] * q[i15]) + (1.901364e-04) * ((2) * q[i0] * q[i16])
            + (2.275479e-04) * ((2) * q[i1] * q[i16]) + (1.876151e-03) * ((2) * q[i2] * q[i16]) + (5.383057e-04) * ((2) * q[i3] * q[i16])
            + (2.743125e-04) * ((2) * q[i4] * q[i16]) + (-2.306933e-03) * ((2) * q[i5] * q[i16]) + (2.869172e-05) * ((2) * q[i6] * q[i16])
            + (2.036815e-05) * ((2) * q[i7] * q[i16]) + (-2.070227e-03) * ((2) * q[i8] * q[i16]) + (-7.112827e-05) * ((2) * q[i9] * q[i16])
            + (-2.127310e-05) * ((2) * q[i10] * q[i16]) + (6.585089e-05) * ((2) * q[i11] * q[i16]) + (2.237602e-03) * ((2) * q[i12] * q[i16])
            + (-8.383803e-06) * ((2) * q[i15] * q[i16]) + (-5.035672e-04) * ((3) * q[i16] * q[i16]) + (3.819796e-05) * ((2) * q[i16] * q[i19])
            + (-1.095250e-04) * ((2) * q[i16] * q[i20]) + (4.022276e-05) * ((2) * q[i16] * q[i21]) + (5.704893e-04) * ((2) * q[i16] * q[i22])
            + (6.883622e-05) * (q[i19] * q[i19]) + (-2.606892e-04) * (q[i20] * q[i20]) + (-1.700324e-05) * (q[i21] * q[i21])
            + (-7.034394e-04) * (q[i22] * q[i22]) + (-7.890194e-04) * (q[i0] * q[i1]) + (2.308786e-03) * (q[i0] * q[i2]) + (9.760053e-04) * (q[i0] * q[i3])
            + (9.716004e-05) * (q[i0] * q[i4]) + (-1.860218e-04) * (q[i0] * q[i5]) + (1.714890e-03) * (q[i0] * q[i6]) + (-3.210253e-04) * (q[i0] * q[i7])
            + (1.008221e-03) * (q[i0] * q[i8]) + (2.400579e-04) * (q[i0] * q[i9]) + (-9.682506e-05) * (q[i0] * q[i10]) + (3.678196e-04) * (q[i0] * q[i11])
            + (-5.135890e-04) * (q[i0] * q[i12]) + (1.680045e-04) * (q[i0] * q[i15]) + (-2.713472e-04) * (q[i0] * q[i19]) + (2.116721e-04) * (q[i0] * q[i20])
            + (-8.932882e-06) * (q[i0] * q[i21]) + (-7.559066e-05) * (q[i0] * q[i22]) + (7.919335e-04) * (q[i1] * q[i2]) + (-4.060882e-04) * (q[i1] * q[i3])
            + (-1.436457e-03) * (q[i1] * q[i4]) + (-2.026340e-04) * (q[i1] * q[i5]) + (-4.786425e-04) * (q[i1] * q[i6]) + (1.597703e-03) * (q[i1] * q[i7])
            + (4.193216e-04) * (q[i1] * q[i8]) + (2.043830e-04) * (q[i1] * q[i9]) + (3.797765e-04) * (q[i1] * q[i10]) + (-1.164569e-04) * (q[i1] * q[i11])
            + (-1.442976e-04) * (q[i1] * q[i12]) + (1.628166e-04) * (q[i1] * q[i15]) + (-8.246948e-05) * (q[i1] * q[i19]) + (-1.668326e-04) * (q[i1] * q[i20])
            + (-9.617169e-05) * (q[i1] * q[i21]) + (-4.835072e-04) * (q[i1] * q[i22]) + (-6.409353e-04) * (q[i2] * q[i3]) + (-1.068054e-03) * (q[i2] * q[i4])
            + (1.387809e-03) * (q[i2] * q[i5]) + (-4.701494e-04) * (q[i2] * q[i6]) + (6.752372e-04) * (q[i2] * q[i7]) + (5.688234e-03) * (q[i2] * q[i8])
            + (-1.346369e-04) * (q[i2] * q[i9]) + (9.072092e-06) * (q[i2] * q[i10]) + (-2.787331e-04) * (q[i2] * q[i11]) + (-3.231733e-03) * (q[i2] * q[i12])
            + (1.514747e-05) * (q[i2] * q[i15]) + (-4.820720e-04) * (q[i2] * q[i19]) + (-7.416021e-04) * (q[i2] * q[i20]) + (-2.151514e-04) * (q[i2] * q[i21])
            + (-1.013329e-05) * (q[i2] * q[i22]) + (-3.644859e-04) * (q[i3] * q[i4]) + (-3.537567e-04) * (q[i3] * q[i5]) + (-9.212668e-04) * (q[i3] * q[i6])
            + (-3.420845e-04) * (q[i3] * q[i7]) + (-5.885023e-04) * (q[i3] * q[i8]) + (-6.474482e-04) * (q[i3] * q[i9]) + (1.095946e-04) * (q[i3] * q[i10])
            + (4.190322e-04) * (q[i3] * q[i11]) + (-1.435395e-04) * (q[i3] * q[i12]) + (2.630016e-04) * (q[i3] * q[i15]) + (-2.486897e-05) * (q[i3] * q[i19])
            + (1.707353e-04) * (q[i3] * q[i20]) + (-2.294937e-04) * (q[i3] * q[i21]) + (-6.333649e-04) * (q[i3] * q[i22]) + (-8.730152e-04) * (q[i4] * q[i5])
            + (-2.718924e-04) * (q[i4] * q[i6]) + (2.176679e-04) * (q[i4] * q[i7]) + (-5.842616e-04) * (q[i4] * q[i8]) + (1.631917e-04) * (q[i4] * q[i9])
            + (-2.492120e-04) * (q[i4] * q[i10]) + (-5.401505e-04) * (q[i4] * q[i11]) + (9.410504e-04) * (q[i4] * q[i12]) + (2.579163e-04) * (q[i4] * q[i15])
            + (8.093611e-05) * (q[i4] * q[i19]) + (-2.527976e-04) * (q[i4] * q[i20]) + (-9.233878e-06) * (q[i4] * q[i21]) + (-3.416751e-04) * (q[i4] * q[i22])
            + (1.081003e-04) * (q[i5] * q[i6]) + (-2.874128e-05) * (q[i5] * q[i7]) + (2.398485e-03) * (q[i5] * q[i8]) + (-2.962105e-04) * (q[i5] * q[i9])
            + (-3.412088e-04) * (q[i5] * q[i10]) + (4.896294e-05) * (q[i5] * q[i11]) + (-1.907801e-03) * (q[i5] * q[i12]) + (-4.701687e-04) * (q[i5] * q[i15])
            + (-2.437812e-04) * (q[i5] * q[i19]) + (-1.624382e-04) * (q[i5] * q[i20]) + (1.821656e-04) * (q[i5] * q[i21]) + (1.859405e-03) * (q[i5] * q[i22])
            + (2.563843e-04) * (q[i6] * q[i7]) + (6.577481e-05) * (q[i6] * q[i8]) + (3.639200e-04) * (q[i6] * q[i9]) + (-1.564324e-04) * (q[i6] * q[i10])
            + (8.527838e-05) * (q[i6] * q[i11]) + (9.717923e-04) * (q[i6] * q[i12]) + (-3.368392e-04) * (q[i6] * q[i15]) + (-2.116556e-04) * (q[i6] * q[i19])
            + (-2.076262e-04) * (q[i6] * q[i20]) + (-1.229254e-04) * (q[i6] * q[i21]) + (1.434616e-04) * (q[i6] * q[i22]) + (-4.314481e-04) * (q[i7] * q[i8])
            + (4.976830e-04) * (q[i7] * q[i9]) + (1.720243e-04) * (q[i7] * q[i10]) + (4.343167e-04) * (q[i7] * q[i11]) + (8.227340e-04) * (q[i7] * q[i12])
            + (3.378316e-04) * (q[i7] * q[i15]) + (-9.969937e-06) * (q[i7] * q[i19]) + (-2.072429e-04) * (q[i7] * q[i20]) + (-3.814879e-05) * (q[i7] * q[i21])
            + (-4.905897e-05) * (q[i7] * q[i22]) + (7.364581e-05) * (q[i8] * q[i9]) + (2.570242e-04) * (q[i8] * q[i10]) + (1.553745e-04) * (q[i8] * q[i11])
            + (3.777960e-03) * (q[i8] * q[i12]) + (6.105476e-07) * (q[i8] * q[i15]) + (1.402156e-04) * (q[i8] * q[i19]) + (-3.603188e-04) * (q[i8] * q[i20])
            + (1.460470e-04) * (q[i8] * q[i21]) + (4.367203e-04) * (q[i8] * q[i22]) + (-3.222391e-05) * (q[i9] * q[i10]) + (-2.319001e-05) * (q[i9] * q[i11])
            + (-1.029408e-04) * (q[i9] * q[i12]) + (6.452166e-05) * (q[i9] * q[i15]) + (-9.599487e-05) * (q[i9] * q[i19]) + (-6.383294e-05) * (q[i9] * q[i20])
            + (-3.070896e-05) * (q[i9] * q[i21]) + (-3.057005e-05) * (q[i9] * q[i22]) + (-1.252333e-04) * (q[i10] * q[i11])
            + (-3.109391e-04) * (q[i10] * q[i12]) + (-6.514242e-05) * (q[i10] * q[i15]) + (-4.382993e-05) * (q[i10] * q[i19])
            + (-9.594302e-05) * (q[i10] * q[i20]) + (-1.803125e-05) * (q[i10] * q[i21]) + (5.298471e-05) * (q[i10] * q[i22])
            + (-1.939213e-04) * (q[i11] * q[i12]) + (-2.880563e-04) * (q[i11] * q[i15]) + (1.741379e-06) * (q[i11] * q[i19])
            + (2.023173e-04) * (q[i11] * q[i20]) + (-1.639818e-04) * (q[i11] * q[i21]) + (-1.462666e-04) * (q[i11] * q[i22])
            + (-2.832443e-04) * (q[i12] * q[i15]) + (-8.642033e-05) * (q[i12] * q[i19]) + (-2.879309e-04) * (q[i12] * q[i20])
            + (-3.884873e-05) * (q[i12] * q[i21]) + (5.267210e-04) * (q[i12] * q[i22]) + (3.736643e-06) * (q[i15] * q[i19])
            + (-4.263416e-06) * (q[i15] * q[i20]) + (5.189304e-05) * (q[i15] * q[i21]) + (5.324950e-05) * (q[i15] * q[i22]) + (5.549695e-05) * (q[i19] * q[i20])
            + (3.903370e-05) * (q[i19] * q[i21]) + (-3.183414e-05) * (q[i19] * q[i22]) + (-1.827554e-05) * (q[i20] * q[i21])
            + (2.108443e-04) * (q[i20] * q[i22]) + (-4.087610e-05) * (q[i21] * q[i22]);
      JQ[1][i19] = (-1.550586e-03) * (1) + (8.195139e-04) * ((2) * q[i19]) + (-9.288888e-04) * (q[i0]) + (6.761685e-06) * (q[i1]) + (-1.191364e-03) * (q[i2])
            + (5.955505e-05) * (q[i3]) + (4.169070e-04) * (q[i4]) + (-9.544767e-04) * (q[i5]) + (4.443123e-04) * (q[i6]) + (3.617413e-04) * (q[i7])
            + (2.349562e-04) * (q[i8]) + (4.954136e-04) * (q[i9]) + (6.531916e-05) * (q[i10]) + (-8.968728e-04) * (q[i11]) + (9.553798e-05) * (q[i12])
            + (7.760659e-04) * (q[i15]) + (2.703000e-04) * (q[i16]) + (-3.662987e-06) * (q[i20]) + (-9.797321e-04) * (q[i21]) + (-1.793031e-04) * (q[i22])
            + (3.381829e-04) * (q[i0] * q[i0]) + (6.595327e-04) * (q[i1] * q[i1]) + (-1.678943e-04) * (q[i2] * q[i2]) + (-1.532691e-04) * (q[i3] * q[i3])
            + (1.698040e-04) * (q[i4] * q[i4]) + (7.653206e-05) * (q[i5] * q[i5]) + (8.821549e-05) * (q[i6] * q[i6]) + (-9.765210e-05) * (q[i7] * q[i7])
            + (3.631669e-04) * (q[i8] * q[i8]) + (-1.888503e-04) * (q[i9] * q[i9]) + (-6.291108e-05) * (q[i10] * q[i10]) + (5.161947e-04) * (q[i11] * q[i11])
            + (1.325286e-04) * (q[i12] * q[i12]) + (1.147565e-04) * (q[i15] * q[i15]) + (3.819796e-05) * (q[i16] * q[i16])
            + (-1.025481e-04) * ((2) * q[i0] * q[i19]) + (-2.613781e-06) * ((2) * q[i1] * q[i19]) + (-1.497534e-04) * ((2) * q[i2] * q[i19])
            + (-8.723253e-05) * ((2) * q[i3] * q[i19]) + (-1.017205e-06) * ((2) * q[i4] * q[i19]) + (-2.370621e-05) * ((2) * q[i5] * q[i19])
            + (-7.604286e-05) * ((2) * q[i6] * q[i19]) + (4.005731e-05) * ((2) * q[i7] * q[i19]) + (9.644574e-05) * ((2) * q[i8] * q[i19])
            + (-5.995082e-05) * ((2) * q[i9] * q[i19]) + (-7.298273e-05) * ((2) * q[i10] * q[i19]) + (5.587782e-05) * ((2) * q[i11] * q[i19])
            + (-8.649770e-05) * ((2) * q[i12] * q[i19]) + (2.662236e-04) * ((2) * q[i15] * q[i19]) + (6.883622e-05) * ((2) * q[i16] * q[i19])
            + (1.031562e-04) * ((3) * q[i19] * q[i19]) + (-2.962465e-05) * ((2) * q[i19] * q[i20]) + (7.039310e-04) * ((2) * q[i19] * q[i21])
            + (-6.772213e-05) * ((2) * q[i19] * q[i22]) + (2.959283e-05) * (q[i20] * q[i20]) + (-1.850370e-04) * (q[i21] * q[i21])
            + (4.007287e-05) * (q[i22] * q[i22]) + (-9.758276e-04) * (q[i0] * q[i1]) + (3.991832e-05) * (q[i0] * q[i2]) + (7.935073e-04) * (q[i0] * q[i3])
            + (4.639175e-04) * (q[i0] * q[i4]) + (-5.553259e-04) * (q[i0] * q[i5]) + (5.418329e-04) * (q[i0] * q[i6]) + (6.264215e-04) * (q[i0] * q[i7])
            + (4.729393e-04) * (q[i0] * q[i8]) + (4.301947e-04) * (q[i0] * q[i9]) + (8.105425e-05) * (q[i0] * q[i10]) + (1.275807e-04) * (q[i0] * q[i11])
            + (6.372694e-05) * (q[i0] * q[i12]) + (-1.696522e-04) * (q[i0] * q[i15]) + (-2.713472e-04) * (q[i0] * q[i16]) + (-2.635676e-05) * (q[i0] * q[i20])
            + (2.077649e-04) * (q[i0] * q[i21]) + (-4.681289e-05) * (q[i0] * q[i22]) + (-8.783529e-04) * (q[i1] * q[i2]) + (2.787432e-04) * (q[i1] * q[i3])
            + (-3.493539e-04) * (q[i1] * q[i4]) + (-1.674119e-04) * (q[i1] * q[i5]) + (-7.087169e-05) * (q[i1] * q[i6]) + (-3.730352e-04) * (q[i1] * q[i7])
            + (3.292585e-04) * (q[i1] * q[i8]) + (-3.293673e-04) * (q[i1] * q[i9]) + (1.399759e-04) * (q[i1] * q[i10]) + (1.243312e-04) * (q[i1] * q[i11])
            + (-6.553262e-05) * (q[i1] * q[i12]) + (2.174304e-04) * (q[i1] * q[i15]) + (-8.246948e-05) * (q[i1] * q[i16]) + (-3.140601e-05) * (q[i1] * q[i20])
            + (-1.897953e-04) * (q[i1] * q[i21]) + (-1.584384e-04) * (q[i1] * q[i22]) + (-5.450766e-04) * (q[i2] * q[i3]) + (6.371539e-04) * (q[i2] * q[i4])
            + (1.181564e-03) * (q[i2] * q[i5]) + (-1.506784e-04) * (q[i2] * q[i6]) + (2.106647e-04) * (q[i2] * q[i7]) + (2.465393e-04) * (q[i2] * q[i8])
            + (1.978261e-05) * (q[i2] * q[i9]) + (9.332384e-05) * (q[i2] * q[i10]) + (9.466732e-04) * (q[i2] * q[i11]) + (2.152759e-04) * (q[i2] * q[i12])
            + (-7.424004e-04) * (q[i2] * q[i15]) + (-4.820720e-04) * (q[i2] * q[i16]) + (7.708067e-05) * (q[i2] * q[i20]) + (-5.448979e-04) * (q[i2] * q[i21])
            + (-2.062266e-04) * (q[i2] * q[i22]) + (1.639802e-04) * (q[i3] * q[i4]) + (3.344671e-04) * (q[i3] * q[i5]) + (-5.412548e-04) * (q[i3] * q[i6])
            + (-3.583018e-04) * (q[i3] * q[i7]) + (-2.736758e-04) * (q[i3] * q[i8]) + (-2.530471e-04) * (q[i3] * q[i9]) + (6.130547e-05) * (q[i3] * q[i10])
            + (1.874694e-04) * (q[i3] * q[i11]) + (-1.542441e-04) * (q[i3] * q[i12]) + (-2.487628e-04) * (q[i3] * q[i15]) + (-2.486897e-05) * (q[i3] * q[i16])
            + (2.075815e-05) * (q[i3] * q[i20]) + (3.289395e-04) * (q[i3] * q[i21]) + (-7.843576e-05) * (q[i3] * q[i22]) + (6.492198e-04) * (q[i4] * q[i5])
            + (2.185914e-05) * (q[i4] * q[i6]) + (5.446240e-04) * (q[i4] * q[i7]) + (-2.530987e-04) * (q[i4] * q[i8]) + (1.529120e-04) * (q[i4] * q[i9])
            + (4.953959e-05) * (q[i4] * q[i10]) + (-3.156915e-04) * (q[i4] * q[i11]) + (1.068496e-04) * (q[i4] * q[i12]) + (1.673949e-04) * (q[i4] * q[i15])
            + (8.093611e-05) * (q[i4] * q[i16]) + (2.110874e-05) * (q[i4] * q[i20]) + (6.592995e-04) * (q[i4] * q[i21]) + (1.175819e-04) * (q[i4] * q[i22])
            + (1.491224e-04) * (q[i5] * q[i6]) + (6.361853e-04) * (q[i5] * q[i7]) + (5.959096e-04) * (q[i5] * q[i8]) + (3.594022e-04) * (q[i5] * q[i9])
            + (4.502975e-05) * (q[i5] * q[i10]) + (8.125725e-04) * (q[i5] * q[i11]) + (3.512644e-04) * (q[i5] * q[i12]) + (-1.713542e-04) * (q[i5] * q[i15])
            + (-2.437812e-04) * (q[i5] * q[i16]) + (6.759754e-04) * (q[i5] * q[i20]) + (-1.579017e-03) * (q[i5] * q[i21]) + (-1.745701e-04) * (q[i5] * q[i22])
            + (4.420799e-04) * (q[i6] * q[i7]) + (-3.950201e-05) * (q[i6] * q[i8]) + (-1.812739e-04) * (q[i6] * q[i9]) + (-7.371924e-06) * (q[i6] * q[i10])
            + (5.202263e-05) * (q[i6] * q[i11]) + (-6.807007e-05) * (q[i6] * q[i12]) + (2.110082e-04) * (q[i6] * q[i15]) + (-2.116556e-04) * (q[i6] * q[i16])
            + (5.515648e-05) * (q[i6] * q[i20]) + (-1.667539e-04) * (q[i6] * q[i21]) + (2.299836e-05) * (q[i6] * q[i22]) + (-4.847388e-04) * (q[i7] * q[i8])
            + (3.774569e-05) * (q[i7] * q[i9]) + (-3.155922e-05) * (q[i7] * q[i10]) + (5.168748e-05) * (q[i7] * q[i11]) + (-4.373682e-05) * (q[i7] * q[i12])
            + (2.100835e-04) * (q[i7] * q[i15]) + (-9.969937e-06) * (q[i7] * q[i16]) + (-5.782290e-05) * (q[i7] * q[i20]) + (-2.977827e-04) * (q[i7] * q[i21])
            + (5.188520e-06) * (q[i7] * q[i22]) + (6.685977e-05) * (q[i8] * q[i9]) + (4.027034e-05) * (q[i8] * q[i10]) + (3.075091e-04) * (q[i8] * q[i11])
            + (4.182337e-05) * (q[i8] * q[i12]) + (3.855031e-04) * (q[i8] * q[i15]) + (1.402156e-04) * (q[i8] * q[i16]) + (-6.143433e-07) * (q[i8] * q[i20])
            + (-8.813338e-04) * (q[i8] * q[i21]) + (1.426832e-05) * (q[i8] * q[i22]) + (-2.000294e-05) * (q[i9] * q[i10]) + (1.114000e-05) * (q[i9] * q[i11])
            + (-7.128775e-05) * (q[i9] * q[i12]) + (9.578845e-05) * (q[i9] * q[i15]) + (-9.599487e-05) * (q[i9] * q[i16]) + (1.254037e-04) * (q[i9] * q[i20])
            + (6.051227e-05) * (q[i9] * q[i21]) + (7.515042e-05) * (q[i9] * q[i22]) + (1.704197e-04) * (q[i10] * q[i11]) + (-3.370367e-05) * (q[i10] * q[i12])
            + (6.426175e-05) * (q[i10] * q[i15]) + (-4.382993e-05) * (q[i10] * q[i16]) + (-1.253923e-04) * (q[i10] * q[i20])
            + (-1.100553e-04) * (q[i10] * q[i21]) + (-2.068554e-05) * (q[i10] * q[i22]) + (3.515086e-04) * (q[i11] * q[i12])
            + (-2.796700e-04) * (q[i11] * q[i15]) + (1.741379e-06) * (q[i11] * q[i16]) + (4.498088e-05) * (q[i11] * q[i20])
            + (-1.951025e-03) * (q[i11] * q[i21]) + (6.040702e-05) * (q[i11] * q[i22]) + (2.032436e-04) * (q[i12] * q[i15])
            + (-8.642033e-05) * (q[i12] * q[i16]) + (4.564215e-05) * (q[i12] * q[i20]) + (3.084258e-05) * (q[i12] * q[i21]) + (9.309488e-05) * (q[i12] * q[i22])
            + (3.736643e-06) * (q[i15] * q[i16]) + (-5.533933e-05) * (q[i15] * q[i20]) + (2.051240e-04) * (q[i15] * q[i21])
            + (-1.997200e-05) * (q[i15] * q[i22]) + (5.549695e-05) * (q[i16] * q[i20]) + (3.903370e-05) * (q[i16] * q[i21])
            + (-3.183414e-05) * (q[i16] * q[i22]) + (1.138070e-04) * (q[i20] * q[i21]) + (1.152920e-04) * (q[i20] * q[i22])
            + (-2.884140e-05) * (q[i21] * q[i22]);
      JQ[1][i20] = (1.492182e-03) * (1) + (-8.182365e-04) * ((2) * q[i20]) + (-1.328615e-06) * (q[i0]) + (-9.199283e-04) * (q[i1]) + (-1.200393e-03) * (q[i2])
            + (4.284752e-04) * (q[i3]) + (5.590000e-05) * (q[i4]) + (-9.681243e-04) * (q[i5]) + (-3.447312e-04) * (q[i6]) + (-4.178380e-04) * (q[i7])
            + (-1.955376e-04) * (q[i8]) + (-6.306070e-05) * (q[i9]) + (-4.910622e-04) * (q[i10]) + (9.723991e-05) * (q[i11]) + (-9.368650e-04) * (q[i12])
            + (-2.762830e-04) * (q[i15]) + (-7.458541e-04) * (q[i16]) + (-3.662987e-06) * (q[i19]) + (-1.799038e-04) * (q[i21]) + (-9.564593e-04) * (q[i22])
            + (-6.683916e-04) * (q[i0] * q[i0]) + (-3.295209e-04) * (q[i1] * q[i1]) + (1.765466e-04) * (q[i2] * q[i2]) + (-1.637586e-04) * (q[i3] * q[i3])
            + (1.458927e-04) * (q[i4] * q[i4]) + (-7.396116e-05) * (q[i5] * q[i5]) + (9.985464e-05) * (q[i6] * q[i6]) + (-9.126872e-05) * (q[i7] * q[i7])
            + (-3.565308e-04) * (q[i8] * q[i8]) + (6.264896e-05) * (q[i9] * q[i9]) + (1.849190e-04) * (q[i10] * q[i10]) + (-1.333448e-04) * (q[i11] * q[i11])
            + (-4.996383e-04) * (q[i12] * q[i12]) + (-3.720846e-05) * (q[i15] * q[i15]) + (-1.095250e-04) * (q[i16] * q[i16])
            + (-2.962465e-05) * (q[i19] * q[i19]) + (-1.444755e-06) * ((2) * q[i0] * q[i20]) + (-9.771895e-05) * ((2) * q[i1] * q[i20])
            + (-1.462692e-04) * ((2) * q[i2] * q[i20]) + (-5.818832e-06) * ((2) * q[i3] * q[i20]) + (-8.774236e-05) * ((2) * q[i4] * q[i20])
            + (-1.470971e-05) * ((2) * q[i5] * q[i20]) + (-4.128587e-05) * ((2) * q[i6] * q[i20]) + (7.549052e-05) * ((2) * q[i7] * q[i20])
            + (-1.047269e-04) * ((2) * q[i8] * q[i20]) + (7.274886e-05) * ((2) * q[i9] * q[i20]) + (5.974990e-05) * ((2) * q[i10] * q[i20])
            + (-8.713647e-05) * ((2) * q[i11] * q[i20]) + (7.128725e-05) * ((2) * q[i12] * q[i20]) + (-6.891444e-05) * ((2) * q[i15] * q[i20])
            + (-2.606892e-04) * ((2) * q[i16] * q[i20]) + (2.959283e-05) * ((2) * q[i19] * q[i20]) + (-9.737891e-05) * ((3) * q[i20] * q[i20])
            + (-6.756140e-05) * ((2) * q[i20] * q[i21]) + (7.037935e-04) * ((2) * q[i20] * q[i22]) + (-3.964983e-05) * (q[i21] * q[i21])
            + (1.833804e-04) * (q[i22] * q[i22]) + (9.787422e-04) * (q[i0] * q[i1]) + (8.805669e-04) * (q[i0] * q[i2]) + (3.412249e-04) * (q[i0] * q[i3])
            + (-2.708977e-04) * (q[i0] * q[i4]) + (1.763750e-04) * (q[i0] * q[i5]) + (-3.649028e-04) * (q[i0] * q[i6]) + (-7.089603e-05) * (q[i0] * q[i7])
            + (3.312547e-04) * (q[i0] * q[i8]) + (1.408991e-04) * (q[i0] * q[i9]) + (-3.296448e-04) * (q[i0] * q[i10]) + (6.317997e-05) * (q[i0] * q[i11])
            + (-1.227215e-04) * (q[i0] * q[i12]) + (-8.786154e-05) * (q[i0] * q[i15]) + (2.116721e-04) * (q[i0] * q[i16]) + (-2.635676e-05) * (q[i0] * q[i19])
            + (1.602136e-04) * (q[i0] * q[i21]) + (1.849766e-04) * (q[i0] * q[i22]) + (-3.859424e-05) * (q[i1] * q[i2]) + (-4.741077e-04) * (q[i1] * q[i3])
            + (-7.889787e-04) * (q[i1] * q[i4]) + (5.510449e-04) * (q[i1] * q[i5]) + (6.264196e-04) * (q[i1] * q[i6]) + (5.390420e-04) * (q[i1] * q[i7])
            + (4.634254e-04) * (q[i1] * q[i8]) + (7.962626e-05) * (q[i1] * q[i9]) + (4.300117e-04) * (q[i1] * q[i10]) + (-6.589012e-05) * (q[i1] * q[i11])
            + (-1.224903e-04) * (q[i1] * q[i12]) + (-2.712922e-04) * (q[i1] * q[i15]) + (-1.668326e-04) * (q[i1] * q[i16]) + (-3.140601e-05) * (q[i1] * q[i19])
            + (4.808083e-05) * (q[i1] * q[i21]) + (-2.088448e-04) * (q[i1] * q[i22]) + (-6.409211e-04) * (q[i2] * q[i3]) + (5.427403e-04) * (q[i2] * q[i4])
            + (-1.176193e-03) * (q[i2] * q[i5]) + (2.140396e-04) * (q[i2] * q[i6]) + (-1.452176e-04) * (q[i2] * q[i7]) + (2.382505e-04) * (q[i2] * q[i8])
            + (9.105248e-05) * (q[i2] * q[i9]) + (2.200496e-05) * (q[i2] * q[i10]) + (-2.194630e-04) * (q[i2] * q[i11]) + (-9.212270e-04) * (q[i2] * q[i12])
            + (-4.808458e-04) * (q[i2] * q[i15]) + (-7.416021e-04) * (q[i2] * q[i16]) + (7.708067e-05) * (q[i2] * q[i19]) + (2.072959e-04) * (q[i2] * q[i21])
            + (5.365955e-04) * (q[i2] * q[i22]) + (-1.619049e-04) * (q[i3] * q[i4]) + (-6.595843e-04) * (q[i3] * q[i5]) + (5.468548e-04) * (q[i3] * q[i6])
            + (2.250667e-05) * (q[i3] * q[i7]) + (-2.506133e-04) * (q[i3] * q[i8]) + (5.058323e-05) * (q[i3] * q[i9]) + (1.567877e-04) * (q[i3] * q[i10])
            + (-1.119682e-04) * (q[i3] * q[i11]) + (3.180409e-04) * (q[i3] * q[i12]) + (8.713109e-05) * (q[i3] * q[i15]) + (1.707353e-04) * (q[i3] * q[i16])
            + (2.075815e-05) * (q[i3] * q[i19]) + (-1.167296e-04) * (q[i3] * q[i21]) + (-6.593001e-04) * (q[i3] * q[i22]) + (-3.338799e-04) * (q[i4] * q[i5])
            + (-3.518379e-04) * (q[i4] * q[i6]) + (-5.295054e-04) * (q[i4] * q[i7]) + (-2.619990e-04) * (q[i4] * q[i8]) + (6.564241e-05) * (q[i4] * q[i9])
            + (-2.550075e-04) * (q[i4] * q[i10]) + (1.557434e-04) * (q[i4] * q[i11]) + (-1.928976e-04) * (q[i4] * q[i12]) + (-2.553386e-05) * (q[i4] * q[i15])
            + (-2.527976e-04) * (q[i4] * q[i16]) + (2.110874e-05) * (q[i4] * q[i19]) + (7.482038e-05) * (q[i4] * q[i21]) + (-3.308207e-04) * (q[i4] * q[i22])
            + (6.341876e-04) * (q[i5] * q[i6]) + (1.472312e-04) * (q[i5] * q[i7]) + (5.773746e-04) * (q[i5] * q[i8]) + (4.230960e-05) * (q[i5] * q[i9])
            + (3.584510e-04) * (q[i5] * q[i10]) + (-3.515285e-04) * (q[i5] * q[i11]) + (-8.135706e-04) * (q[i5] * q[i12]) + (-2.484216e-04) * (q[i5] * q[i15])
            + (-1.624382e-04) * (q[i5] * q[i16]) + (6.759754e-04) * (q[i5] * q[i19]) + (1.759727e-04) * (q[i5] * q[i21]) + (1.582590e-03) * (q[i5] * q[i22])
            + (-4.454843e-04) * (q[i6] * q[i7]) + (4.931194e-04) * (q[i6] * q[i8]) + (2.937345e-05) * (q[i6] * q[i9]) + (-3.897913e-05) * (q[i6] * q[i10])
            + (-4.364655e-05) * (q[i6] * q[i11]) + (5.282220e-05) * (q[i6] * q[i12]) + (1.025149e-05) * (q[i6] * q[i15]) + (-2.076262e-04) * (q[i6] * q[i16])
            + (5.515648e-05) * (q[i6] * q[i19]) + (2.634986e-06) * (q[i6] * q[i21]) + (-3.036432e-04) * (q[i6] * q[i22]) + (3.692607e-05) * (q[i7] * q[i8])
            + (4.779438e-06) * (q[i7] * q[i9]) + (1.732689e-04) * (q[i7] * q[i10]) + (-7.278678e-05) * (q[i7] * q[i11]) + (5.232382e-05) * (q[i7] * q[i12])
            + (2.107295e-04) * (q[i7] * q[i15]) + (-2.072429e-04) * (q[i7] * q[i16]) + (-5.782290e-05) * (q[i7] * q[i19]) + (2.271721e-05) * (q[i7] * q[i21])
            + (-1.667069e-04) * (q[i7] * q[i22]) + (-3.435974e-05) * (q[i8] * q[i9]) + (-6.536552e-05) * (q[i8] * q[i10]) + (4.150482e-05) * (q[i8] * q[i11])
            + (2.916899e-04) * (q[i8] * q[i12]) + (-1.373722e-04) * (q[i8] * q[i15]) + (-3.603188e-04) * (q[i8] * q[i16]) + (-6.143433e-07) * (q[i8] * q[i19])
            + (1.982403e-05) * (q[i8] * q[i21]) + (-8.678889e-04) * (q[i8] * q[i22]) + (1.999417e-05) * (q[i9] * q[i10]) + (-3.336551e-05) * (q[i9] * q[i11])
            + (1.669239e-04) * (q[i9] * q[i12]) + (4.479746e-05) * (q[i9] * q[i15]) + (-6.383294e-05) * (q[i9] * q[i16]) + (1.254037e-04) * (q[i9] * q[i19])
            + (-2.057594e-05) * (q[i9] * q[i21]) + (-1.112748e-04) * (q[i9] * q[i22]) + (-7.218964e-05) * (q[i10] * q[i11]) + (1.260359e-05) * (q[i10] * q[i12])
            + (9.496067e-05) * (q[i10] * q[i15]) + (-9.594302e-05) * (q[i10] * q[i16]) + (-1.253923e-04) * (q[i10] * q[i19])
            + (7.477975e-05) * (q[i10] * q[i21]) + (5.951271e-05) * (q[i10] * q[i22]) + (-3.487330e-04) * (q[i11] * q[i12])
            + (-8.792055e-05) * (q[i11] * q[i15]) + (2.023173e-04) * (q[i11] * q[i16]) + (4.498088e-05) * (q[i11] * q[i19])
            + (-9.114553e-05) * (q[i11] * q[i21]) + (-3.234788e-05) * (q[i11] * q[i22]) + (-7.861044e-07) * (q[i12] * q[i15])
            + (-2.879309e-04) * (q[i12] * q[i16]) + (4.564215e-05) * (q[i12] * q[i19]) + (-6.337097e-05) * (q[i12] * q[i21])
            + (1.943681e-03) * (q[i12] * q[i22]) + (-4.263416e-06) * (q[i15] * q[i16]) + (-5.533933e-05) * (q[i15] * q[i19])
            + (-3.264947e-05) * (q[i15] * q[i21]) + (4.061760e-05) * (q[i15] * q[i22]) + (5.549695e-05) * (q[i16] * q[i19])
            + (-1.827554e-05) * (q[i16] * q[i21]) + (2.108443e-04) * (q[i16] * q[i22]) + (1.138070e-04) * (q[i19] * q[i21]) + (1.152920e-04) * (q[i19] * q[i22])
            + (2.995648e-05) * (q[i21] * q[i22]);
      JQ[1][i21] = (-2.502136e-03) * (1) + (1.955704e-03) * ((2) * q[i21]) + (1.501395e-03) * (q[i0]) + (-6.984603e-05) * (q[i1]) + (4.442213e-04) * (q[i2])
            + (-9.822362e-04) * (q[i3]) + (7.599458e-04) * (q[i4]) + (-9.370004e-04) * (q[i5]) + (-3.226140e-04) * (q[i6]) + (-3.194301e-04) * (q[i7])
            + (2.085960e-03) * (q[i8]) + (6.873966e-05) * (q[i9]) + (-2.976386e-04) * (q[i10]) + (9.362029e-04) * (q[i11]) + (-7.341442e-05) * (q[i12])
            + (2.007150e-03) * (q[i15]) + (1.446155e-04) * (q[i16]) + (-9.797321e-04) * (q[i19]) + (-1.799038e-04) * (q[i20]) + (1.749562e-07) * (q[i22])
            + (2.935931e-04) * (q[i0] * q[i0]) + (-1.209851e-05) * (q[i1] * q[i1]) + (4.466233e-04) * (q[i2] * q[i2]) + (8.051092e-05) * (q[i3] * q[i3])
            + (3.258086e-05) * (q[i4] * q[i4]) + (-1.678756e-04) * (q[i5] * q[i5]) + (-3.543190e-05) * (q[i6] * q[i6]) + (-1.198847e-04) * (q[i7] * q[i7])
            + (-3.602408e-05) * (q[i8] * q[i8]) + (-4.165742e-05) * (q[i9] * q[i9]) + (3.249624e-06) * (q[i10] * q[i10]) + (9.612420e-04) * (q[i11] * q[i11])
            + (-1.226814e-04) * (q[i12] * q[i12]) + (5.641917e-04) * (q[i15] * q[i15]) + (4.022276e-05) * (q[i16] * q[i16]) + (7.039310e-04) * (q[i19] * q[i19])
            + (-6.756140e-05) * (q[i20] * q[i20]) + (1.070024e-04) * ((2) * q[i0] * q[i21]) + (-1.453267e-04) * ((2) * q[i1] * q[i21])
            + (2.625060e-04) * ((2) * q[i2] * q[i21]) + (-6.384954e-05) * ((2) * q[i3] * q[i21]) + (6.024040e-04) * ((2) * q[i4] * q[i21])
            + (-1.243868e-03) * ((2) * q[i5] * q[i21]) + (-5.489398e-05) * ((2) * q[i6] * q[i21]) + (-1.488944e-04) * ((2) * q[i7] * q[i21])
            + (7.672063e-04) * ((2) * q[i8] * q[i21]) + (1.329937e-05) * ((2) * q[i9] * q[i21]) + (-4.647722e-05) * ((2) * q[i10] * q[i21])
            + (4.934464e-04) * ((2) * q[i11] * q[i21]) + (-8.258681e-06) * ((2) * q[i12] * q[i21]) + (7.005584e-04) * ((2) * q[i15] * q[i21])
            + (-1.700324e-05) * ((2) * q[i16] * q[i21]) + (-1.850370e-04) * ((2) * q[i19] * q[i21]) + (-3.964983e-05) * ((2) * q[i20] * q[i21])
            + (3.685271e-04) * ((3) * q[i21] * q[i21]) + (3.164260e-05) * ((2) * q[i21] * q[i22]) + (3.202463e-05) * (q[i22] * q[i22])
            + (-2.022340e-06) * (q[i0] * q[i1]) + (2.581975e-04) * (q[i0] * q[i2]) + (-1.788544e-04) * (q[i0] * q[i3]) + (1.816487e-04) * (q[i0] * q[i4])
            + (5.344639e-04) * (q[i0] * q[i5]) + (-5.273413e-04) * (q[i0] * q[i6]) + (-1.047804e-04) * (q[i0] * q[i7]) + (2.722731e-05) * (q[i0] * q[i8])
            + (-1.125004e-04) * (q[i0] * q[i9]) + (-3.191838e-04) * (q[i0] * q[i10]) + (2.023471e-04) * (q[i0] * q[i11]) + (2.551483e-04) * (q[i0] * q[i12])
            + (4.839724e-04) * (q[i0] * q[i15]) + (-8.932882e-06) * (q[i0] * q[i16]) + (2.077649e-04) * (q[i0] * q[i19]) + (1.602136e-04) * (q[i0] * q[i20])
            + (7.868737e-05) * (q[i0] * q[i22]) + (-7.342616e-05) * (q[i1] * q[i2]) + (1.423696e-04) * (q[i1] * q[i3]) + (-2.771836e-04) * (q[i1] * q[i4])
            + (-1.820496e-04) * (q[i1] * q[i5]) + (2.398719e-04) * (q[i1] * q[i6]) + (-5.712384e-04) * (q[i1] * q[i7]) + (2.051705e-04) * (q[i1] * q[i8])
            + (1.490205e-04) * (q[i1] * q[i9]) + (-7.301478e-05) * (q[i1] * q[i10]) + (1.314783e-05) * (q[i1] * q[i11]) + (-1.676125e-04) * (q[i1] * q[i12])
            + (7.240107e-05) * (q[i1] * q[i15]) + (-9.617169e-05) * (q[i1] * q[i16]) + (-1.897953e-04) * (q[i1] * q[i19]) + (4.808083e-05) * (q[i1] * q[i20])
            + (7.632382e-05) * (q[i1] * q[i22]) + (-3.747750e-05) * (q[i2] * q[i3]) + (-2.653857e-05) * (q[i2] * q[i4]) + (6.264919e-04) * (q[i2] * q[i5])
            + (2.169045e-04) * (q[i2] * q[i6]) + (1.059136e-04) * (q[i2] * q[i7]) + (-3.458079e-04) * (q[i2] * q[i8]) + (-3.524261e-05) * (q[i2] * q[i9])
            + (-1.501000e-04) * (q[i2] * q[i10]) + (7.741898e-04) * (q[i2] * q[i11]) + (-7.210027e-05) * (q[i2] * q[i12]) + (1.176342e-05) * (q[i2] * q[i15])
            + (-2.151514e-04) * (q[i2] * q[i16]) + (-5.448979e-04) * (q[i2] * q[i19]) + (2.072959e-04) * (q[i2] * q[i20]) + (-1.536732e-05) * (q[i2] * q[i22])
            + (-7.005385e-04) * (q[i3] * q[i4]) + (3.193782e-04) * (q[i3] * q[i5]) + (4.925865e-06) * (q[i3] * q[i6]) + (1.451721e-04) * (q[i3] * q[i7])
            + (3.583714e-04) * (q[i3] * q[i8]) + (1.621689e-04) * (q[i3] * q[i9]) + (-2.241620e-04) * (q[i3] * q[i10]) + (-3.660009e-04) * (q[i3] * q[i11])
            + (-4.124738e-05) * (q[i3] * q[i12]) + (3.401230e-04) * (q[i3] * q[i15]) + (-2.294937e-04) * (q[i3] * q[i16]) + (3.289395e-04) * (q[i3] * q[i19])
            + (-1.167296e-04) * (q[i3] * q[i20]) + (5.306985e-05) * (q[i3] * q[i22]) + (-1.473504e-05) * (q[i4] * q[i5]) + (-1.800077e-04) * (q[i4] * q[i6])
            + (-4.123952e-04) * (q[i4] * q[i7]) + (3.014335e-05) * (q[i4] * q[i8]) + (-2.000379e-04) * (q[i4] * q[i9]) + (2.937772e-04) * (q[i4] * q[i10])
            + (-2.194707e-04) * (q[i4] * q[i11]) + (-9.924322e-05) * (q[i4] * q[i12]) + (6.246916e-04) * (q[i4] * q[i15]) + (-9.233878e-06) * (q[i4] * q[i16])
            + (6.592995e-04) * (q[i4] * q[i19]) + (7.482038e-05) * (q[i4] * q[i20]) + (5.319872e-05) * (q[i4] * q[i22]) + (-3.825242e-04) * (q[i5] * q[i6])
            + (2.908408e-04) * (q[i5] * q[i7]) + (1.498457e-04) * (q[i5] * q[i8]) + (4.788983e-05) * (q[i5] * q[i9]) + (1.194787e-06) * (q[i5] * q[i10])
            + (7.588162e-04) * (q[i5] * q[i11]) + (-1.486958e-04) * (q[i5] * q[i12]) + (-1.837282e-03) * (q[i5] * q[i15]) + (1.821656e-04) * (q[i5] * q[i16])
            + (-1.579017e-03) * (q[i5] * q[i19]) + (1.759727e-04) * (q[i5] * q[i20]) + (-4.213042e-04) * (q[i5] * q[i22]) + (-1.594664e-05) * (q[i6] * q[i7])
            + (-1.592634e-04) * (q[i6] * q[i8]) + (8.760108e-06) * (q[i6] * q[i9]) + (8.674091e-05) * (q[i6] * q[i10]) + (3.742519e-04) * (q[i6] * q[i11])
            + (-1.138935e-04) * (q[i6] * q[i12]) + (-4.249887e-05) * (q[i6] * q[i15]) + (-1.229254e-04) * (q[i6] * q[i16]) + (-1.667539e-04) * (q[i6] * q[i19])
            + (2.634986e-06) * (q[i6] * q[i20]) + (-2.140628e-05) * (q[i6] * q[i22]) + (3.624944e-04) * (q[i7] * q[i8]) + (1.699146e-04) * (q[i7] * q[i9])
            + (-1.111959e-04) * (q[i7] * q[i10]) + (1.227451e-04) * (q[i7] * q[i11]) + (-1.195996e-04) * (q[i7] * q[i12]) + (1.401101e-04) * (q[i7] * q[i15])
            + (-3.814879e-05) * (q[i7] * q[i16]) + (-2.977827e-04) * (q[i7] * q[i19]) + (2.271721e-05) * (q[i7] * q[i20]) + (2.078964e-05) * (q[i7] * q[i22])
            + (3.378134e-04) * (q[i8] * q[i9]) + (-2.449227e-04) * (q[i8] * q[i10]) + (1.772542e-03) * (q[i8] * q[i11]) + (-8.636645e-05) * (q[i8] * q[i12])
            + (4.243812e-04) * (q[i8] * q[i15]) + (1.460470e-04) * (q[i8] * q[i16]) + (-8.813338e-04) * (q[i8] * q[i19]) + (1.982403e-05) * (q[i8] * q[i20])
            + (1.326477e-07) * (q[i8] * q[i22]) + (7.364560e-05) * (q[i9] * q[i10]) + (1.422593e-04) * (q[i9] * q[i11]) + (2.485172e-05) * (q[i9] * q[i12])
            + (5.512411e-05) * (q[i9] * q[i15]) + (-3.070896e-05) * (q[i9] * q[i16]) + (6.051227e-05) * (q[i9] * q[i19]) + (-2.057594e-05) * (q[i9] * q[i20])
            + (-2.941916e-06) * (q[i9] * q[i22]) + (-1.410711e-04) * (q[i10] * q[i11]) + (7.388342e-05) * (q[i10] * q[i12])
            + (-2.781572e-05) * (q[i10] * q[i15]) + (-1.803125e-05) * (q[i10] * q[i16]) + (-1.100553e-04) * (q[i10] * q[i19])
            + (7.477975e-05) * (q[i10] * q[i20]) + (2.709701e-06) * (q[i10] * q[i22]) + (-1.346398e-04) * (q[i11] * q[i12])
            + (-5.240005e-04) * (q[i11] * q[i15]) + (-1.639818e-04) * (q[i11] * q[i16]) + (-1.951025e-03) * (q[i11] * q[i19])
            + (-9.114553e-05) * (q[i11] * q[i20]) + (2.329214e-05) * (q[i11] * q[i22]) + (1.456073e-04) * (q[i12] * q[i15])
            + (-3.884873e-05) * (q[i12] * q[i16]) + (3.084258e-05) * (q[i12] * q[i19]) + (-6.337097e-05) * (q[i12] * q[i20])
            + (2.400665e-05) * (q[i12] * q[i22]) + (5.189304e-05) * (q[i15] * q[i16]) + (2.051240e-04) * (q[i15] * q[i19]) + (-3.264947e-05) * (q[i15] * q[i20])
            + (4.119159e-05) * (q[i15] * q[i22]) + (3.903370e-05) * (q[i16] * q[i19]) + (-1.827554e-05) * (q[i16] * q[i20])
            + (-4.087610e-05) * (q[i16] * q[i22]) + (1.138070e-04) * (q[i19] * q[i20]) + (-2.884140e-05) * (q[i19] * q[i22])
            + (2.995648e-05) * (q[i20] * q[i22]);
      JQ[1][i22] = (-2.531255e-03) * (1) + (-1.957783e-03) * ((2) * q[i22]) + (7.039540e-05) * (q[i0]) + (-1.504815e-03) * (q[i1]) + (-4.390413e-04) * (q[i2])
            + (-7.616008e-04) * (q[i3]) + (9.896619e-04) * (q[i4]) + (9.305713e-04) * (q[i5]) + (-3.170217e-04) * (q[i6]) + (-3.214045e-04) * (q[i7])
            + (2.096886e-03) * (q[i8]) + (-3.064132e-04) * (q[i9]) + (6.265089e-05) * (q[i10]) + (6.837693e-05) * (q[i11]) + (-9.423373e-04) * (q[i12])
            + (1.439286e-04) * (q[i15]) + (2.009953e-03) * (q[i16]) + (-1.793031e-04) * (q[i19]) + (-9.564593e-04) * (q[i20]) + (1.749562e-07) * (q[i21])
            + (-1.106642e-05) * (q[i0] * q[i0]) + (2.927667e-04) * (q[i1] * q[i1]) + (4.547228e-04) * (q[i2] * q[i2]) + (3.262428e-05) * (q[i3] * q[i3])
            + (7.933015e-05) * (q[i4] * q[i4]) + (-1.628696e-04) * (q[i5] * q[i5]) + (-1.175586e-04) * (q[i6] * q[i6]) + (-3.545673e-05) * (q[i7] * q[i7])
            + (-4.186722e-05) * (q[i8] * q[i8]) + (3.440476e-06) * (q[i9] * q[i9]) + (-4.041151e-05) * (q[i10] * q[i10]) + (-1.220038e-04) * (q[i11] * q[i11])
            + (9.664559e-04) * (q[i12] * q[i12]) + (4.040847e-05) * (q[i15] * q[i15]) + (5.704893e-04) * (q[i16] * q[i16]) + (-6.772213e-05) * (q[i19] * q[i19])
            + (7.037935e-04) * (q[i20] * q[i20]) + (3.164260e-05) * (q[i21] * q[i21]) + (-1.444598e-04) * ((2) * q[i0] * q[i22])
            + (1.078829e-04) * ((2) * q[i1] * q[i22]) + (2.635518e-04) * ((2) * q[i2] * q[i22]) + (6.055682e-04) * ((2) * q[i3] * q[i22])
            + (-6.108601e-05) * ((2) * q[i4] * q[i22]) + (-1.245705e-03) * ((2) * q[i5] * q[i22]) + (1.492691e-04) * ((2) * q[i6] * q[i22])
            + (5.372295e-05) * ((2) * q[i7] * q[i22]) + (-7.750709e-04) * ((2) * q[i8] * q[i22]) + (4.793143e-05) * ((2) * q[i9] * q[i22])
            + (-1.234508e-05) * ((2) * q[i10] * q[i22]) + (-9.397988e-06) * ((2) * q[i11] * q[i22]) + (4.990193e-04) * ((2) * q[i12] * q[i22])
            + (1.858024e-05) * ((2) * q[i15] * q[i22]) + (-7.034394e-04) * ((2) * q[i16] * q[i22]) + (4.007287e-05) * ((2) * q[i19] * q[i22])
            + (1.833804e-04) * ((2) * q[i20] * q[i22]) + (3.202463e-05) * ((2) * q[i21] * q[i22]) + (3.702771e-04) * ((3) * q[i22] * q[i22])
            + (-4.565074e-06) * (q[i0] * q[i1]) + (-6.659603e-05) * (q[i0] * q[i2]) + (-2.821919e-04) * (q[i0] * q[i3]) + (1.422256e-04) * (q[i0] * q[i4])
            + (-1.737749e-04) * (q[i0] * q[i5]) + (5.840159e-04) * (q[i0] * q[i6]) + (-2.431557e-04) * (q[i0] * q[i7]) + (-2.079227e-04) * (q[i0] * q[i8])
            + (7.431093e-05) * (q[i0] * q[i9]) + (-1.513056e-04) * (q[i0] * q[i10]) + (-1.685427e-04) * (q[i0] * q[i11]) + (1.660179e-05) * (q[i0] * q[i12])
            + (9.873012e-05) * (q[i0] * q[i15]) + (-7.559066e-05) * (q[i0] * q[i16]) + (-4.681289e-05) * (q[i0] * q[i19]) + (1.849766e-04) * (q[i0] * q[i20])
            + (7.868737e-05) * (q[i0] * q[i21]) + (2.587410e-04) * (q[i1] * q[i2]) + (1.807528e-04) * (q[i1] * q[i3]) + (-1.891432e-04) * (q[i1] * q[i4])
            + (5.276585e-04) * (q[i1] * q[i5]) + (1.031743e-04) * (q[i1] * q[i6]) + (5.396924e-04) * (q[i1] * q[i7]) + (-2.760091e-05) * (q[i1] * q[i8])
            + (3.217950e-04) * (q[i1] * q[i9]) + (1.116875e-04) * (q[i1] * q[i10]) + (2.562168e-04) * (q[i1] * q[i11]) + (2.016792e-04) * (q[i1] * q[i12])
            + (6.014123e-06) * (q[i1] * q[i15]) + (-4.835072e-04) * (q[i1] * q[i16]) + (-1.584384e-04) * (q[i1] * q[i19]) + (-2.088448e-04) * (q[i1] * q[i20])
            + (7.632382e-05) * (q[i1] * q[i21]) + (-2.728672e-05) * (q[i2] * q[i3]) + (-4.333677e-05) * (q[i2] * q[i4]) + (6.305250e-04) * (q[i2] * q[i5])
            + (-1.013324e-04) * (q[i2] * q[i6]) + (-2.113594e-04) * (q[i2] * q[i7]) + (3.538491e-04) * (q[i2] * q[i8]) + (1.524644e-04) * (q[i2] * q[i9])
            + (3.215402e-05) * (q[i2] * q[i10]) + (-7.041495e-05) * (q[i2] * q[i11]) + (7.730127e-04) * (q[i2] * q[i12]) + (2.144045e-04) * (q[i2] * q[i15])
            + (-1.013329e-05) * (q[i2] * q[i16]) + (-2.062266e-04) * (q[i2] * q[i19]) + (5.365955e-04) * (q[i2] * q[i20]) + (-1.536732e-05) * (q[i2] * q[i21])
            + (-7.129517e-04) * (q[i3] * q[i4]) + (-7.617263e-06) * (q[i3] * q[i5]) + (4.098744e-04) * (q[i3] * q[i6]) + (1.780640e-04) * (q[i3] * q[i7])
            + (-3.718586e-05) * (q[i3] * q[i8]) + (-2.972290e-04) * (q[i3] * q[i9]) + (2.002785e-04) * (q[i3] * q[i10]) + (-9.408255e-05) * (q[i3] * q[i11])
            + (-2.224609e-04) * (q[i3] * q[i12]) + (1.075916e-05) * (q[i3] * q[i15]) + (-6.333649e-04) * (q[i3] * q[i16]) + (-7.843576e-05) * (q[i3] * q[i19])
            + (-6.593001e-04) * (q[i3] * q[i20]) + (5.306985e-05) * (q[i3] * q[i21]) + (3.263962e-04) * (q[i4] * q[i5]) + (-1.451111e-04) * (q[i4] * q[i6])
            + (4.186780e-06) * (q[i4] * q[i7]) + (-3.593076e-04) * (q[i4] * q[i8]) + (2.217890e-04) * (q[i4] * q[i9]) + (-1.621648e-04) * (q[i4] * q[i10])
            + (-3.941864e-05) * (q[i4] * q[i11]) + (-3.666046e-04) * (q[i4] * q[i12]) + (2.248961e-04) * (q[i4] * q[i15]) + (-3.416751e-04) * (q[i4] * q[i16])
            + (1.175819e-04) * (q[i4] * q[i19]) + (-3.308207e-04) * (q[i4] * q[i20]) + (5.319872e-05) * (q[i4] * q[i21]) + (-2.843814e-04) * (q[i5] * q[i6])
            + (3.741623e-04) * (q[i5] * q[i7]) + (-1.440720e-04) * (q[i5] * q[i8]) + (1.136154e-06) * (q[i5] * q[i9]) + (-5.287167e-05) * (q[i5] * q[i10])
            + (-1.558879e-04) * (q[i5] * q[i11]) + (7.760778e-04) * (q[i5] * q[i12]) + (-1.835453e-04) * (q[i5] * q[i15]) + (1.859405e-03) * (q[i5] * q[i16])
            + (-1.745701e-04) * (q[i5] * q[i19]) + (1.582590e-03) * (q[i5] * q[i20]) + (-4.213042e-04) * (q[i5] * q[i21]) + (-1.502534e-05) * (q[i6] * q[i7])
            + (3.601620e-04) * (q[i6] * q[i8]) + (-1.099680e-04) * (q[i6] * q[i9]) + (1.694297e-04) * (q[i6] * q[i10]) + (1.222552e-04) * (q[i6] * q[i11])
            + (-1.274102e-04) * (q[i6] * q[i12]) + (-3.835844e-05) * (q[i6] * q[i15]) + (1.434616e-04) * (q[i6] * q[i16]) + (2.299836e-05) * (q[i6] * q[i19])
            + (-3.036432e-04) * (q[i6] * q[i20]) + (-2.140628e-05) * (q[i6] * q[i21]) + (-1.662362e-04) * (q[i7] * q[i8]) + (8.824007e-05) * (q[i7] * q[i9])
            + (8.252590e-06) * (q[i7] * q[i10]) + (1.142882e-04) * (q[i7] * q[i11]) + (-3.711772e-04) * (q[i7] * q[i12]) + (-1.217494e-04) * (q[i7] * q[i15])
            + (-4.905897e-05) * (q[i7] * q[i16]) + (5.188520e-06) * (q[i7] * q[i19]) + (-1.667069e-04) * (q[i7] * q[i20]) + (2.078964e-05) * (q[i7] * q[i21])
            + (-2.450494e-04) * (q[i8] * q[i9]) + (3.374044e-04) * (q[i8] * q[i10]) + (8.801020e-05) * (q[i8] * q[i11]) + (-1.784999e-03) * (q[i8] * q[i12])
            + (1.451907e-04) * (q[i8] * q[i15]) + (4.367203e-04) * (q[i8] * q[i16]) + (1.426832e-05) * (q[i8] * q[i19]) + (-8.678889e-04) * (q[i8] * q[i20])
            + (1.326477e-07) * (q[i8] * q[i21]) + (7.426748e-05) * (q[i9] * q[i10]) + (-7.166301e-05) * (q[i9] * q[i11]) + (1.425158e-04) * (q[i9] * q[i12])
            + (-1.941762e-05) * (q[i9] * q[i15]) + (-3.057005e-05) * (q[i9] * q[i16]) + (7.515042e-05) * (q[i9] * q[i19]) + (-1.112748e-04) * (q[i9] * q[i20])
            + (-2.941916e-06) * (q[i9] * q[i21]) + (-2.435788e-05) * (q[i10] * q[i11]) + (-1.428886e-04) * (q[i10] * q[i12])
            + (-2.911934e-05) * (q[i10] * q[i15]) + (5.298471e-05) * (q[i10] * q[i16]) + (-2.068554e-05) * (q[i10] * q[i19])
            + (5.951271e-05) * (q[i10] * q[i20]) + (2.709701e-06) * (q[i10] * q[i21]) + (-1.364356e-04) * (q[i11] * q[i12]) + (3.849008e-05) * (q[i11] * q[i15])
            + (-1.462666e-04) * (q[i11] * q[i16]) + (6.040702e-05) * (q[i11] * q[i19]) + (-3.234788e-05) * (q[i11] * q[i20])
            + (2.329214e-05) * (q[i11] * q[i21]) + (1.656489e-04) * (q[i12] * q[i15]) + (5.267210e-04) * (q[i12] * q[i16]) + (9.309488e-05) * (q[i12] * q[i19])
            + (1.943681e-03) * (q[i12] * q[i20]) + (2.400665e-05) * (q[i12] * q[i21]) + (5.324950e-05) * (q[i15] * q[i16]) + (-1.997200e-05) * (q[i15] * q[i19])
            + (4.061760e-05) * (q[i15] * q[i20]) + (4.119159e-05) * (q[i15] * q[i21]) + (-3.183414e-05) * (q[i16] * q[i19]) + (2.108443e-04) * (q[i16] * q[i20])
            + (-4.087610e-05) * (q[i16] * q[i21]) + (1.152920e-04) * (q[i19] * q[i20]) + (-2.884140e-05) * (q[i19] * q[i21])
            + (2.995648e-05) * (q[i20] * q[i21]);
   }

   public void getJQy(double[] q, double[][] JQ)
   {
      JQ[2][i0] = (-5.017620e-03) * (1) + (2.299577e-03) * ((2) * q[i0]) + (-1.639256e-04) * (q[i1]) + (6.040509e-05) * (q[i2]) + (-1.050023e-01) * (q[i3])
            + (1.019550e-02) * (q[i4]) + (2.311095e-03) * (q[i5]) + (5.812794e-03) * (q[i6]) + (1.397899e-03) * (q[i7]) + (-8.777736e-04) * (q[i8])
            + (8.871379e-04) * (q[i9]) + (8.983805e-04) * (q[i10]) + (-1.743768e-03) * (q[i11]) + (-4.475553e-04) * (q[i12]) + (3.125782e-03) * (q[i15])
            + (-3.367573e-03) * (q[i16]) + (2.667678e-04) * (q[i19]) + (-8.810999e-04) * (q[i20]) + (6.470544e-04) * (q[i21]) + (6.914933e-04) * (q[i22])
            + (2.816164e-03) * ((3) * q[i0] * q[i0]) + (3.268503e-04) * ((2) * q[i0] * q[i1]) + (2.064399e-03) * ((2) * q[i0] * q[i2])
            + (-3.329575e-03) * ((2) * q[i0] * q[i3]) + (-1.948477e-03) * ((2) * q[i0] * q[i4]) + (-1.714863e-03) * ((2) * q[i0] * q[i5])
            + (-2.474288e-02) * ((2) * q[i0] * q[i6]) + (-2.529714e-03) * ((2) * q[i0] * q[i7]) + (2.492421e-04) * ((2) * q[i0] * q[i8])
            + (-3.616889e-03) * ((2) * q[i0] * q[i9]) + (1.348726e-04) * ((2) * q[i0] * q[i10]) + (2.516890e-04) * ((2) * q[i0] * q[i11])
            + (1.310823e-04) * ((2) * q[i0] * q[i12]) + (2.843503e-04) * ((2) * q[i0] * q[i15]) + (3.868913e-04) * ((2) * q[i0] * q[i16])
            + (2.950859e-04) * ((2) * q[i0] * q[i19]) + (2.163784e-04) * ((2) * q[i0] * q[i20]) + (3.546135e-04) * ((2) * q[i0] * q[i21])
            + (-8.499914e-05) * ((2) * q[i0] * q[i22]) + (-3.029270e-04) * (q[i1] * q[i1]) + (1.145821e-03) * (q[i2] * q[i2]) + (7.659209e-03) * (q[i3] * q[i3])
            + (1.713932e-03) * (q[i4] * q[i4]) + (1.598485e-04) * (q[i5] * q[i5]) + (-4.359479e-03) * (q[i6] * q[i6]) + (4.180654e-04) * (q[i7] * q[i7])
            + (-1.026171e-03) * (q[i8] * q[i8]) + (-4.753033e-04) * (q[i9] * q[i9]) + (1.221015e-04) * (q[i10] * q[i10]) + (2.362361e-04) * (q[i11] * q[i11])
            + (1.346939e-04) * (q[i12] * q[i12]) + (1.811154e-04) * (q[i15] * q[i15]) + (-2.201567e-04) * (q[i16] * q[i16]) + (1.728568e-04) * (q[i19] * q[i19])
            + (-4.234846e-05) * (q[i20] * q[i20]) + (6.811269e-05) * (q[i21] * q[i21]) + (-7.112266e-05) * (q[i22] * q[i22]) + (3.179068e-06) * (q[i1] * q[i2])
            + (-2.378489e-03) * (q[i1] * q[i3]) + (2.358891e-03) * (q[i1] * q[i4]) + (9.988349e-06) * (q[i1] * q[i5]) + (6.064606e-03) * (q[i1] * q[i6])
            + (6.056071e-03) * (q[i1] * q[i7]) + (-1.745568e-04) * (q[i1] * q[i8]) + (9.218836e-04) * (q[i1] * q[i9]) + (9.146743e-04) * (q[i1] * q[i10])
            + (-2.862139e-04) * (q[i1] * q[i11]) + (2.970770e-04) * (q[i1] * q[i12]) + (-1.059418e-04) * (q[i1] * q[i15]) + (-1.032743e-04) * (q[i1] * q[i16])
            + (-1.413254e-04) * (q[i1] * q[i19]) + (-1.424201e-04) * (q[i1] * q[i20]) + (1.137842e-04) * (q[i1] * q[i21]) + (-1.097233e-04) * (q[i1] * q[i22])
            + (-3.557220e-03) * (q[i2] * q[i3]) + (-1.882166e-03) * (q[i2] * q[i4]) + (-1.790790e-03) * (q[i2] * q[i5]) + (-1.163352e-02) * (q[i2] * q[i6])
            + (7.894988e-04) * (q[i2] * q[i7]) + (-5.673798e-03) * (q[i2] * q[i8]) + (-1.526473e-03) * (q[i2] * q[i9]) + (1.848176e-03) * (q[i2] * q[i10])
            + (1.045153e-03) * (q[i2] * q[i11]) + (6.411273e-05) * (q[i2] * q[i12]) + (3.562394e-04) * (q[i2] * q[i15]) + (7.656795e-04) * (q[i2] * q[i16])
            + (6.379012e-04) * (q[i2] * q[i19]) + (3.685915e-05) * (q[i2] * q[i20]) + (1.027054e-04) * (q[i2] * q[i21]) + (-6.779181e-04) * (q[i2] * q[i22])
            + (1.336112e-02) * (q[i3] * q[i4]) + (9.800715e-03) * (q[i3] * q[i5]) + (-1.615880e-02) * (q[i3] * q[i6]) + (3.259265e-03) * (q[i3] * q[i7])
            + (-1.176956e-03) * (q[i3] * q[i8]) + (1.243164e-02) * (q[i3] * q[i9]) + (-3.773849e-03) * (q[i3] * q[i10]) + (3.946048e-04) * (q[i3] * q[i11])
            + (-9.528819e-04) * (q[i3] * q[i12]) + (-1.674781e-03) * (q[i3] * q[i15]) + (-3.861911e-03) * (q[i3] * q[i16]) + (-7.626717e-04) * (q[i3] * q[i19])
            + (4.278285e-05) * (q[i3] * q[i20]) + (-1.361292e-04) * (q[i3] * q[i21]) + (-8.498490e-04) * (q[i3] * q[i22]) + (3.740858e-03) * (q[i4] * q[i5])
            + (-8.506806e-04) * (q[i4] * q[i6]) + (3.139579e-03) * (q[i4] * q[i7]) + (2.537469e-04) * (q[i4] * q[i8]) + (-1.366809e-04) * (q[i4] * q[i9])
            + (-9.703830e-04) * (q[i4] * q[i10]) + (-2.479629e-05) * (q[i4] * q[i11]) + (-2.565737e-04) * (q[i4] * q[i12]) + (5.375513e-04) * (q[i4] * q[i15])
            + (5.378835e-04) * (q[i4] * q[i16]) + (2.205854e-05) * (q[i4] * q[i19]) + (9.761715e-06) * (q[i4] * q[i20]) + (-2.597395e-04) * (q[i4] * q[i21])
            + (-6.792909e-04) * (q[i4] * q[i22]) + (4.591833e-03) * (q[i5] * q[i6]) + (-9.150038e-04) * (q[i5] * q[i7]) + (-1.960694e-05) * (q[i5] * q[i8])
            + (2.434291e-04) * (q[i5] * q[i9]) + (1.174825e-03) * (q[i5] * q[i10]) + (-1.086987e-03) * (q[i5] * q[i11]) + (1.086708e-03) * (q[i5] * q[i12])
            + (-8.253056e-04) * (q[i5] * q[i15]) + (-1.200608e-03) * (q[i5] * q[i16]) + (-1.774556e-04) * (q[i5] * q[i19]) + (-3.916660e-04) * (q[i5] * q[i20])
            + (-8.283292e-06) * (q[i5] * q[i21]) + (6.741703e-04) * (q[i5] * q[i22]) + (1.025195e-02) * (q[i6] * q[i7]) + (3.391187e-03) * (q[i6] * q[i8])
            + (-1.594058e-03) * (q[i6] * q[i9]) + (2.157548e-03) * (q[i6] * q[i10]) + (-1.685007e-03) * (q[i6] * q[i11]) + (-1.091461e-03) * (q[i6] * q[i12])
            + (-4.290466e-04) * (q[i6] * q[i15]) + (1.370662e-03) * (q[i6] * q[i16]) + (-1.298252e-03) * (q[i6] * q[i19]) + (9.889740e-04) * (q[i6] * q[i20])
            + (-1.969670e-04) * (q[i6] * q[i21]) + (8.597856e-06) * (q[i6] * q[i22]) + (-5.919886e-04) * (q[i7] * q[i8]) + (1.357902e-03) * (q[i7] * q[i9])
            + (-2.642911e-05) * (q[i7] * q[i10]) + (4.746056e-05) * (q[i7] * q[i11]) + (-7.667034e-05) * (q[i7] * q[i12]) + (4.276688e-04) * (q[i7] * q[i15])
            + (-3.514662e-05) * (q[i7] * q[i16]) + (3.153959e-04) * (q[i7] * q[i19]) + (-1.302443e-04) * (q[i7] * q[i20]) + (-1.596513e-05) * (q[i7] * q[i21])
            + (-4.932519e-04) * (q[i7] * q[i22]) + (1.484234e-03) * (q[i8] * q[i9]) + (-1.658021e-04) * (q[i8] * q[i10]) + (-4.435071e-04) * (q[i8] * q[i11])
            + (-2.355451e-04) * (q[i8] * q[i12]) + (-6.521984e-05) * (q[i8] * q[i15]) + (2.012094e-04) * (q[i8] * q[i16]) + (-2.381760e-04) * (q[i8] * q[i19])
            + (1.675113e-04) * (q[i8] * q[i20]) + (3.856355e-04) * (q[i8] * q[i21]) + (5.524433e-05) * (q[i8] * q[i22]) + (3.607942e-04) * (q[i9] * q[i10])
            + (1.239044e-04) * (q[i9] * q[i11]) + (-3.238291e-04) * (q[i9] * q[i12]) + (-2.437090e-04) * (q[i9] * q[i15]) + (3.510747e-04) * (q[i9] * q[i16])
            + (-2.967470e-04) * (q[i9] * q[i19]) + (3.917039e-04) * (q[i9] * q[i20]) + (2.764603e-05) * (q[i9] * q[i21]) + (-1.361217e-04) * (q[i9] * q[i22])
            + (5.471964e-04) * (q[i10] * q[i11]) + (2.231798e-04) * (q[i10] * q[i12]) + (3.978931e-04) * (q[i10] * q[i15]) + (-2.885660e-04) * (q[i10] * q[i16])
            + (2.560510e-04) * (q[i10] * q[i19]) + (1.447837e-04) * (q[i10] * q[i20]) + (1.370656e-04) * (q[i10] * q[i21]) + (3.005937e-05) * (q[i10] * q[i22])
            + (1.395562e-04) * (q[i11] * q[i12]) + (1.694093e-04) * (q[i11] * q[i15]) + (-2.069843e-04) * (q[i11] * q[i16]) + (6.329313e-04) * (q[i11] * q[i19])
            + (-2.262147e-04) * (q[i11] * q[i20]) + (1.627175e-05) * (q[i11] * q[i21]) + (8.850492e-05) * (q[i11] * q[i22])
            + (-1.615810e-04) * (q[i12] * q[i15]) + (-1.409281e-05) * (q[i12] * q[i16]) + (1.480147e-04) * (q[i12] * q[i19])
            + (-1.575840e-04) * (q[i12] * q[i20]) + (8.082817e-05) * (q[i12] * q[i21]) + (-1.832422e-04) * (q[i12] * q[i22])
            + (1.832028e-04) * (q[i15] * q[i16]) + (-8.622976e-06) * (q[i15] * q[i19]) + (-1.083230e-04) * (q[i15] * q[i20])
            + (3.299489e-04) * (q[i15] * q[i21]) + (1.040241e-05) * (q[i15] * q[i22]) + (1.609813e-04) * (q[i16] * q[i19]) + (-1.584302e-04) * (q[i16] * q[i20])
            + (3.315816e-05) * (q[i16] * q[i21]) + (7.259586e-04) * (q[i16] * q[i22]) + (2.514656e-05) * (q[i19] * q[i20]) + (-1.606318e-06) * (q[i19] * q[i21])
            + (-4.005700e-04) * (q[i19] * q[i22]) + (4.732295e-05) * (q[i20] * q[i21]) + (1.434421e-04) * (q[i20] * q[i22])
            + (-2.342242e-04) * (q[i21] * q[i22]);
      JQ[2][i1] = (5.085403e-03) * (1) + (2.279791e-03) * ((2) * q[i1]) + (-1.639256e-04) * (q[i0]) + (5.298778e-05) * (q[i2]) + (1.020397e-02) * (q[i3])
            + (-1.045263e-01) * (q[i4]) + (2.323533e-03) * (q[i5]) + (-1.376621e-03) * (q[i6]) + (-5.877031e-03) * (q[i7]) + (8.841392e-04) * (q[i8])
            + (-8.937913e-04) * (q[i9]) + (-8.990074e-04) * (q[i10]) + (-4.216920e-04) * (q[i11]) + (-1.746531e-03) * (q[i12]) + (3.317269e-03) * (q[i15])
            + (-3.137907e-03) * (q[i16]) + (8.714941e-04) * (q[i19]) + (-2.465857e-04) * (q[i20]) + (6.795261e-04) * (q[i21]) + (6.412982e-04) * (q[i22])
            + (3.268503e-04) * (q[i0] * q[i0]) + (-3.029270e-04) * ((2) * q[i0] * q[i1]) + (-2.819707e-03) * ((3) * q[i1] * q[i1])
            + (-2.060532e-03) * ((2) * q[i1] * q[i2]) + (1.952530e-03) * ((2) * q[i1] * q[i3]) + (3.350859e-03) * ((2) * q[i1] * q[i4])
            + (1.726265e-03) * ((2) * q[i1] * q[i5]) + (-2.540755e-03) * ((2) * q[i1] * q[i6]) + (-2.469259e-02) * ((2) * q[i1] * q[i7])
            + (2.526131e-04) * ((2) * q[i1] * q[i8]) + (1.274547e-04) * ((2) * q[i1] * q[i9]) + (-3.584321e-03) * ((2) * q[i1] * q[i10])
            + (-1.337575e-04) * ((2) * q[i1] * q[i11]) + (-2.479423e-04) * ((2) * q[i1] * q[i12]) + (3.789879e-04) * ((2) * q[i1] * q[i15])
            + (2.860883e-04) * ((2) * q[i1] * q[i16]) + (2.170462e-04) * ((2) * q[i1] * q[i19]) + (2.942254e-04) * ((2) * q[i1] * q[i20])
            + (8.782667e-05) * ((2) * q[i1] * q[i21]) + (-3.527509e-04) * ((2) * q[i1] * q[i22]) + (-1.155402e-03) * (q[i2] * q[i2])
            + (-1.703064e-03) * (q[i3] * q[i3]) + (-7.622494e-03) * (q[i4] * q[i4]) + (-1.496662e-04) * (q[i5] * q[i5]) + (-4.147658e-04) * (q[i6] * q[i6])
            + (4.344633e-03) * (q[i7] * q[i7]) + (1.033403e-03) * (q[i8] * q[i8]) + (-1.276147e-04) * (q[i9] * q[i9]) + (4.739604e-04) * (q[i10] * q[i10])
            + (-1.418573e-04) * (q[i11] * q[i11]) + (-2.415135e-04) * (q[i12] * q[i12]) + (2.195069e-04) * (q[i15] * q[i15])
            + (-1.820508e-04) * (q[i16] * q[i16]) + (4.184498e-05) * (q[i19] * q[i19]) + (-1.711052e-04) * (q[i20] * q[i20])
            + (6.981324e-05) * (q[i21] * q[i21]) + (-6.529956e-05) * (q[i22] * q[i22]) + (3.179068e-06) * (q[i0] * q[i2]) + (-2.378489e-03) * (q[i0] * q[i3])
            + (2.358891e-03) * (q[i0] * q[i4]) + (9.988349e-06) * (q[i0] * q[i5]) + (6.064606e-03) * (q[i0] * q[i6]) + (6.056071e-03) * (q[i0] * q[i7])
            + (-1.745568e-04) * (q[i0] * q[i8]) + (9.218836e-04) * (q[i0] * q[i9]) + (9.146743e-04) * (q[i0] * q[i10]) + (-2.862139e-04) * (q[i0] * q[i11])
            + (2.970770e-04) * (q[i0] * q[i12]) + (-1.059418e-04) * (q[i0] * q[i15]) + (-1.032743e-04) * (q[i0] * q[i16]) + (-1.413254e-04) * (q[i0] * q[i19])
            + (-1.424201e-04) * (q[i0] * q[i20]) + (1.137842e-04) * (q[i0] * q[i21]) + (-1.097233e-04) * (q[i0] * q[i22]) + (1.905476e-03) * (q[i2] * q[i3])
            + (3.555067e-03) * (q[i2] * q[i4]) + (1.789451e-03) * (q[i2] * q[i5]) + (7.748543e-04) * (q[i2] * q[i6]) + (-1.163635e-02) * (q[i2] * q[i7])
            + (-5.671491e-03) * (q[i2] * q[i8]) + (1.846815e-03) * (q[i2] * q[i9]) + (-1.523589e-03) * (q[i2] * q[i10]) + (-6.976181e-05) * (q[i2] * q[i11])
            + (-1.039166e-03) * (q[i2] * q[i12]) + (7.528875e-04) * (q[i2] * q[i15]) + (3.601205e-04) * (q[i2] * q[i16]) + (3.645932e-05) * (q[i2] * q[i19])
            + (6.373758e-04) * (q[i2] * q[i20]) + (6.805839e-04) * (q[i2] * q[i21]) + (-1.026114e-04) * (q[i2] * q[i22]) + (-1.336134e-02) * (q[i3] * q[i4])
            + (-3.748302e-03) * (q[i3] * q[i5]) + (3.136272e-03) * (q[i3] * q[i6]) + (-8.651054e-04) * (q[i3] * q[i7]) + (2.425740e-04) * (q[i3] * q[i8])
            + (-9.725343e-04) * (q[i3] * q[i9]) + (-1.391450e-04) * (q[i3] * q[i10]) + (2.653701e-04) * (q[i3] * q[i11]) + (3.474245e-05) * (q[i3] * q[i12])
            + (5.455054e-04) * (q[i3] * q[i15]) + (5.488424e-04) * (q[i3] * q[i16]) + (1.523011e-05) * (q[i3] * q[i19]) + (2.528618e-05) * (q[i3] * q[i20])
            + (6.851692e-04) * (q[i3] * q[i21]) + (2.598052e-04) * (q[i3] * q[i22]) + (-9.843163e-03) * (q[i4] * q[i5]) + (3.240810e-03) * (q[i4] * q[i6])
            + (-1.622585e-02) * (q[i4] * q[i7]) + (-1.161245e-03) * (q[i4] * q[i8]) + (-3.814247e-03) * (q[i4] * q[i9]) + (1.230341e-02) * (q[i4] * q[i10])
            + (9.170136e-04) * (q[i4] * q[i11]) + (-4.105944e-04) * (q[i4] * q[i12]) + (-3.807639e-03) * (q[i4] * q[i15]) + (-1.680345e-03) * (q[i4] * q[i16])
            + (4.173312e-05) * (q[i4] * q[i19]) + (-7.562183e-04) * (q[i4] * q[i20]) + (8.366162e-04) * (q[i4] * q[i21]) + (1.205473e-04) * (q[i4] * q[i22])
            + (-9.311442e-04) * (q[i5] * q[i6]) + (4.571576e-03) * (q[i5] * q[i7]) + (8.817743e-06) * (q[i5] * q[i8]) + (1.174705e-03) * (q[i5] * q[i9])
            + (2.354629e-04) * (q[i5] * q[i10]) + (-1.123989e-03) * (q[i5] * q[i11]) + (1.038978e-03) * (q[i5] * q[i12]) + (-1.199295e-03) * (q[i5] * q[i15])
            + (-8.293675e-04) * (q[i5] * q[i16]) + (-3.957794e-04) * (q[i5] * q[i19]) + (-1.755813e-04) * (q[i5] * q[i20]) + (-6.676990e-04) * (q[i5] * q[i21])
            + (1.252290e-05) * (q[i5] * q[i22]) + (-1.024657e-02) * (q[i6] * q[i7]) + (5.875240e-04) * (q[i6] * q[i8]) + (2.119731e-05) * (q[i6] * q[i9])
            + (-1.360581e-03) * (q[i6] * q[i10]) + (-7.651621e-05) * (q[i6] * q[i11]) + (4.752027e-05) * (q[i6] * q[i12]) + (3.144774e-05) * (q[i6] * q[i15])
            + (-4.277297e-04) * (q[i6] * q[i16]) + (1.368596e-04) * (q[i6] * q[i19]) + (-3.205544e-04) * (q[i6] * q[i20]) + (-4.913701e-04) * (q[i6] * q[i21])
            + (-8.847691e-06) * (q[i6] * q[i22]) + (-3.389789e-03) * (q[i7] * q[i8]) + (-2.176429e-03) * (q[i7] * q[i9]) + (1.605034e-03) * (q[i7] * q[i10])
            + (-1.080001e-03) * (q[i7] * q[i11]) + (-1.693575e-03) * (q[i7] * q[i12]) + (-1.361213e-03) * (q[i7] * q[i15]) + (4.450985e-04) * (q[i7] * q[i16])
            + (-9.905888e-04) * (q[i7] * q[i19]) + (1.288409e-03) * (q[i7] * q[i20]) + (1.105648e-05) * (q[i7] * q[i21]) + (-1.934229e-04) * (q[i7] * q[i22])
            + (1.679392e-04) * (q[i8] * q[i9]) + (-1.470540e-03) * (q[i8] * q[i10]) + (-2.058888e-04) * (q[i8] * q[i11]) + (-4.676848e-04) * (q[i8] * q[i12])
            + (-1.883926e-04) * (q[i8] * q[i15]) + (6.772655e-05) * (q[i8] * q[i16]) + (-1.747253e-04) * (q[i8] * q[i19]) + (2.498898e-04) * (q[i8] * q[i20])
            + (4.661306e-05) * (q[i8] * q[i21]) + (3.830143e-04) * (q[i8] * q[i22]) + (-3.642427e-04) * (q[i9] * q[i10]) + (2.217855e-04) * (q[i9] * q[i11])
            + (5.530140e-04) * (q[i9] * q[i12]) + (2.875109e-04) * (q[i9] * q[i15]) + (-3.991676e-04) * (q[i9] * q[i16]) + (-1.422766e-04) * (q[i9] * q[i19])
            + (-2.535342e-04) * (q[i9] * q[i20]) + (3.084981e-05) * (q[i9] * q[i21]) + (1.378833e-04) * (q[i9] * q[i22]) + (-3.287098e-04) * (q[i10] * q[i11])
            + (1.165157e-04) * (q[i10] * q[i12]) + (-3.468384e-04) * (q[i10] * q[i15]) + (2.473510e-04) * (q[i10] * q[i16])
            + (-3.908049e-04) * (q[i10] * q[i19]) + (2.920327e-04) * (q[i10] * q[i20]) + (-1.361306e-04) * (q[i10] * q[i21])
            + (3.006953e-05) * (q[i10] * q[i22]) + (-1.408358e-04) * (q[i11] * q[i12]) + (-1.825324e-05) * (q[i11] * q[i15])
            + (-1.518464e-04) * (q[i11] * q[i16]) + (-1.618743e-04) * (q[i11] * q[i19]) + (1.495978e-04) * (q[i11] * q[i20])
            + (1.855161e-04) * (q[i11] * q[i21]) + (-7.675297e-05) * (q[i11] * q[i22]) + (-2.055615e-04) * (q[i12] * q[i15])
            + (1.753839e-04) * (q[i12] * q[i16]) + (-2.282897e-04) * (q[i12] * q[i19]) + (6.312069e-04) * (q[i12] * q[i20])
            + (-8.990631e-05) * (q[i12] * q[i21]) + (-1.564901e-05) * (q[i12] * q[i22]) + (-1.795702e-04) * (q[i15] * q[i16])
            + (1.611291e-04) * (q[i15] * q[i19]) + (-1.568737e-04) * (q[i15] * q[i20]) + (7.180507e-04) * (q[i15] * q[i21]) + (3.390483e-05) * (q[i15] * q[i22])
            + (1.016426e-04) * (q[i16] * q[i19]) + (1.760141e-05) * (q[i16] * q[i20]) + (7.808667e-06) * (q[i16] * q[i21]) + (3.334762e-04) * (q[i16] * q[i22])
            + (-2.081722e-05) * (q[i19] * q[i20]) + (1.391243e-04) * (q[i19] * q[i21]) + (4.763852e-05) * (q[i19] * q[i22])
            + (-3.954753e-04) * (q[i20] * q[i21]) + (-3.753872e-06) * (q[i20] * q[i22]) + (2.334830e-04) * (q[i21] * q[i22]);
      JQ[2][i2] = (1.317187e-04) * (1) + (-1.288685e-03) * ((2) * q[i2]) + (6.040509e-05) * (q[i0]) + (5.298778e-05) * (q[i1]) + (-2.072470e-02) * (q[i3])
            + (-2.060847e-02) * (q[i4]) + (7.256947e-02) * (q[i5]) + (-6.440818e-04) * (q[i6]) + (6.527983e-04) * (q[i7]) + (-2.594993e-05) * (q[i8])
            + (2.799744e-03) * (q[i9]) + (-2.752438e-03) * (q[i10]) + (-2.770685e-03) * (q[i11]) + (-2.708838e-03) * (q[i12]) + (6.361470e-03) * (q[i15])
            + (-6.473874e-03) * (q[i16]) + (8.740099e-05) * (q[i19]) + (-9.883928e-05) * (q[i20]) + (4.279692e-04) * (q[i21]) + (4.093939e-04) * (q[i22])
            + (2.064399e-03) * (q[i0] * q[i0]) + (-2.060532e-03) * (q[i1] * q[i1]) + (1.145821e-03) * ((2) * q[i0] * q[i2])
            + (-1.155402e-03) * ((2) * q[i1] * q[i2]) + (-2.214427e-05) * ((3) * q[i2] * q[i2]) + (-6.231101e-04) * ((2) * q[i2] * q[i3])
            + (6.283564e-04) * ((2) * q[i2] * q[i4]) + (-2.788829e-06) * ((2) * q[i2] * q[i5]) + (-1.645044e-03) * ((2) * q[i2] * q[i6])
            + (-1.659321e-03) * ((2) * q[i2] * q[i7]) + (-3.049899e-02) * ((2) * q[i2] * q[i8]) + (-1.123049e-03) * ((2) * q[i2] * q[i9])
            + (-1.118759e-03) * ((2) * q[i2] * q[i10]) + (1.546000e-03) * ((2) * q[i2] * q[i11]) + (-1.606939e-03) * ((2) * q[i2] * q[i12])
            + (3.446050e-04) * ((2) * q[i2] * q[i15]) + (3.625418e-04) * ((2) * q[i2] * q[i16]) + (9.646155e-04) * ((2) * q[i2] * q[i19])
            + (9.561565e-04) * ((2) * q[i2] * q[i20]) + (9.184563e-06) * ((2) * q[i2] * q[i21]) + (-9.068572e-06) * ((2) * q[i2] * q[i22])
            + (1.515015e-03) * (q[i3] * q[i3]) + (-1.485448e-03) * (q[i4] * q[i4]) + (-1.879650e-05) * (q[i5] * q[i5]) + (-8.370063e-04) * (q[i6] * q[i6])
            + (8.489854e-04) * (q[i7] * q[i7]) + (1.672647e-05) * (q[i8] * q[i8]) + (1.951560e-04) * (q[i9] * q[i9]) + (-1.928276e-04) * (q[i10] * q[i10])
            + (2.309479e-03) * (q[i11] * q[i11]) + (-2.434125e-03) * (q[i12] * q[i12]) + (2.480727e-03) * (q[i15] * q[i15])
            + (-2.510068e-03) * (q[i16] * q[i16]) + (-2.042859e-04) * (q[i19] * q[i19]) + (2.057429e-04) * (q[i20] * q[i20])
            + (3.228096e-04) * (q[i21] * q[i21]) + (-3.137231e-04) * (q[i22] * q[i22]) + (3.179068e-06) * (q[i0] * q[i1]) + (-3.557220e-03) * (q[i0] * q[i3])
            + (-1.882166e-03) * (q[i0] * q[i4]) + (-1.790790e-03) * (q[i0] * q[i5]) + (-1.163352e-02) * (q[i0] * q[i6]) + (7.894988e-04) * (q[i0] * q[i7])
            + (-5.673798e-03) * (q[i0] * q[i8]) + (-1.526473e-03) * (q[i0] * q[i9]) + (1.848176e-03) * (q[i0] * q[i10]) + (1.045153e-03) * (q[i0] * q[i11])
            + (6.411273e-05) * (q[i0] * q[i12]) + (3.562394e-04) * (q[i0] * q[i15]) + (7.656795e-04) * (q[i0] * q[i16]) + (6.379012e-04) * (q[i0] * q[i19])
            + (3.685915e-05) * (q[i0] * q[i20]) + (1.027054e-04) * (q[i0] * q[i21]) + (-6.779181e-04) * (q[i0] * q[i22]) + (1.905476e-03) * (q[i1] * q[i3])
            + (3.555067e-03) * (q[i1] * q[i4]) + (1.789451e-03) * (q[i1] * q[i5]) + (7.748543e-04) * (q[i1] * q[i6]) + (-1.163635e-02) * (q[i1] * q[i7])
            + (-5.671491e-03) * (q[i1] * q[i8]) + (1.846815e-03) * (q[i1] * q[i9]) + (-1.523589e-03) * (q[i1] * q[i10]) + (-6.976181e-05) * (q[i1] * q[i11])
            + (-1.039166e-03) * (q[i1] * q[i12]) + (7.528875e-04) * (q[i1] * q[i15]) + (3.601205e-04) * (q[i1] * q[i16]) + (3.645932e-05) * (q[i1] * q[i19])
            + (6.373758e-04) * (q[i1] * q[i20]) + (6.805839e-04) * (q[i1] * q[i21]) + (-1.026114e-04) * (q[i1] * q[i22]) + (-1.073665e-05) * (q[i3] * q[i4])
            + (8.591752e-03) * (q[i3] * q[i5]) + (-3.653709e-03) * (q[i3] * q[i6]) + (1.358948e-03) * (q[i3] * q[i7]) + (1.127051e-04) * (q[i3] * q[i8])
            + (2.158290e-03) * (q[i3] * q[i9]) + (-1.252840e-03) * (q[i3] * q[i10]) + (-2.804176e-04) * (q[i3] * q[i11]) + (3.965130e-04) * (q[i3] * q[i12])
            + (4.431432e-04) * (q[i3] * q[i15]) + (-9.113398e-04) * (q[i3] * q[i16]) + (-4.642063e-04) * (q[i3] * q[i19]) + (-2.037979e-04) * (q[i3] * q[i20])
            + (-4.673228e-04) * (q[i3] * q[i21]) + (2.693041e-04) * (q[i3] * q[i22]) + (-8.593283e-03) * (q[i4] * q[i5]) + (1.356865e-03) * (q[i4] * q[i6])
            + (-3.675611e-03) * (q[i4] * q[i7]) + (1.230089e-04) * (q[i4] * q[i8]) + (-1.264543e-03) * (q[i4] * q[i9]) + (2.128636e-03) * (q[i4] * q[i10])
            + (-4.179273e-04) * (q[i4] * q[i11]) + (2.623375e-04) * (q[i4] * q[i12]) + (-9.084603e-04) * (q[i4] * q[i15]) + (4.579803e-04) * (q[i4] * q[i16])
            + (-2.047420e-04) * (q[i4] * q[i19]) + (-4.625191e-04) * (q[i4] * q[i20]) + (-2.657561e-04) * (q[i4] * q[i21]) + (4.650251e-04) * (q[i4] * q[i22])
            + (-2.200894e-03) * (q[i5] * q[i6]) + (-2.212931e-03) * (q[i5] * q[i7]) + (-1.136204e-02) * (q[i5] * q[i8]) + (6.123080e-03) * (q[i5] * q[i9])
            + (6.063427e-03) * (q[i5] * q[i10]) + (1.802504e-03) * (q[i5] * q[i11]) + (-1.910099e-03) * (q[i5] * q[i12]) + (-6.389843e-03) * (q[i5] * q[i15])
            + (-6.479207e-03) * (q[i5] * q[i16]) + (2.116533e-05) * (q[i5] * q[i19]) + (2.157559e-05) * (q[i5] * q[i20]) + (7.172603e-04) * (q[i5] * q[i21])
            + (-7.400227e-04) * (q[i5] * q[i22]) + (-1.284721e-05) * (q[i6] * q[i7]) + (8.949380e-03) * (q[i6] * q[i8]) + (4.654102e-04) * (q[i6] * q[i9])
            + (-2.886947e-04) * (q[i6] * q[i10]) + (-1.117621e-03) * (q[i6] * q[i11]) + (2.174589e-04) * (q[i6] * q[i12]) + (-6.428325e-04) * (q[i6] * q[i15])
            + (6.249175e-04) * (q[i6] * q[i16]) + (-7.928593e-04) * (q[i6] * q[i19]) + (9.333862e-05) * (q[i6] * q[i20]) + (2.855538e-05) * (q[i6] * q[i21])
            + (8.983287e-05) * (q[i6] * q[i22]) + (-8.968141e-03) * (q[i7] * q[i8]) + (2.865255e-04) * (q[i7] * q[i9]) + (-4.609410e-04) * (q[i7] * q[i10])
            + (2.363002e-04) * (q[i7] * q[i11]) + (-1.152232e-03) * (q[i7] * q[i12]) + (-6.230010e-04) * (q[i7] * q[i15]) + (6.615263e-04) * (q[i7] * q[i16])
            + (-9.679730e-05) * (q[i7] * q[i19]) + (7.850361e-04) * (q[i7] * q[i20]) + (8.112065e-05) * (q[i7] * q[i21]) + (2.716826e-05) * (q[i7] * q[i22])
            + (2.838627e-03) * (q[i8] * q[i9]) + (-2.816410e-03) * (q[i8] * q[i10]) + (4.622125e-03) * (q[i8] * q[i11]) + (4.647709e-03) * (q[i8] * q[i12])
            + (3.739850e-04) * (q[i8] * q[i15]) + (-4.028890e-04) * (q[i8] * q[i16]) + (-1.238277e-04) * (q[i8] * q[i19]) + (1.423855e-04) * (q[i8] * q[i20])
            + (4.697497e-04) * (q[i8] * q[i21]) + (4.659396e-04) * (q[i8] * q[i22]) + (-1.552921e-06) * (q[i9] * q[i10]) + (6.676637e-05) * (q[i9] * q[i11])
            + (9.903040e-05) * (q[i9] * q[i12]) + (-1.966923e-04) * (q[i9] * q[i15]) + (-1.519043e-04) * (q[i9] * q[i16]) + (-5.151900e-04) * (q[i9] * q[i19])
            + (5.072482e-05) * (q[i9] * q[i20]) + (8.193177e-06) * (q[i9] * q[i21]) + (1.973894e-05) * (q[i9] * q[i22]) + (8.644235e-05) * (q[i10] * q[i11])
            + (5.846824e-05) * (q[i10] * q[i12]) + (1.523461e-04) * (q[i10] * q[i15]) + (1.977923e-04) * (q[i10] * q[i16]) + (-5.029466e-05) * (q[i10] * q[i19])
            + (5.113233e-04) * (q[i10] * q[i20]) + (1.997862e-05) * (q[i10] * q[i21]) + (9.079683e-06) * (q[i10] * q[i22]) + (5.027884e-06) * (q[i11] * q[i12])
            + (-2.437054e-03) * (q[i11] * q[i15]) + (-6.090809e-04) * (q[i11] * q[i16]) + (1.210145e-03) * (q[i11] * q[i19])
            + (-3.686003e-04) * (q[i11] * q[i20]) + (4.179207e-04) * (q[i11] * q[i21]) + (-2.085002e-05) * (q[i11] * q[i22])
            + (-6.126193e-04) * (q[i12] * q[i15]) + (-2.427449e-03) * (q[i12] * q[i16]) + (-3.721341e-04) * (q[i12] * q[i19])
            + (1.194359e-03) * (q[i12] * q[i20]) + (1.978979e-05) * (q[i12] * q[i21]) + (-4.119226e-04) * (q[i12] * q[i22])
            + (-1.065679e-06) * (q[i15] * q[i16]) + (8.714605e-05) * (q[i15] * q[i19]) + (-9.551340e-05) * (q[i15] * q[i20])
            + (6.314654e-04) * (q[i15] * q[i21]) + (1.855685e-04) * (q[i15] * q[i22]) + (9.087750e-05) * (q[i16] * q[i19]) + (-6.702515e-05) * (q[i16] * q[i20])
            + (1.824016e-04) * (q[i16] * q[i21]) + (6.283451e-04) * (q[i16] * q[i22]) + (4.114824e-06) * (q[i19] * q[i20]) + (-2.210598e-04) * (q[i19] * q[i21])
            + (-4.203312e-04) * (q[i19] * q[i22]) + (-4.167799e-04) * (q[i20] * q[i21]) + (-2.152634e-04) * (q[i20] * q[i22])
            + (-6.924897e-07) * (q[i21] * q[i22]);
      JQ[2][i3] = (6.817190e-04) * (1) + (3.154545e-04) * ((2) * q[i3]) + (-1.050023e-01) * (q[i0]) + (1.020397e-02) * (q[i1]) + (-2.072470e-02) * (q[i2])
            + (-2.626730e-03) * (q[i4]) + (-7.588072e-04) * (q[i5]) + (-1.294811e-02) * (q[i6]) + (1.874437e-02) * (q[i7]) + (1.536486e-03) * (q[i8])
            + (-3.155387e-03) * (q[i9]) + (5.113675e-03) * (q[i10]) + (-3.877735e-03) * (q[i11]) + (-5.651667e-03) * (q[i12]) + (-1.862611e-03) * (q[i15])
            + (2.498941e-03) * (q[i16]) + (-1.422479e-03) * (q[i19]) + (3.632669e-04) * (q[i20]) + (-8.891952e-04) * (q[i21]) + (-2.091769e-04) * (q[i22])
            + (-3.329575e-03) * (q[i0] * q[i0]) + (1.952530e-03) * (q[i1] * q[i1]) + (-6.231101e-04) * (q[i2] * q[i2]) + (7.659209e-03) * ((2) * q[i0] * q[i3])
            + (-1.703064e-03) * ((2) * q[i1] * q[i3]) + (1.515015e-03) * ((2) * q[i2] * q[i3]) + (1.310312e-04) * ((3) * q[i3] * q[i3])
            + (1.318724e-03) * ((2) * q[i3] * q[i4]) + (5.198465e-04) * ((2) * q[i3] * q[i5]) + (1.017437e-02) * ((2) * q[i3] * q[i6])
            + (-3.002781e-03) * ((2) * q[i3] * q[i7]) + (4.890949e-03) * ((2) * q[i3] * q[i8]) + (1.245209e-03) * ((2) * q[i3] * q[i9])
            + (-6.931558e-04) * ((2) * q[i3] * q[i10]) + (-6.311628e-04) * ((2) * q[i3] * q[i11]) + (3.770896e-04) * ((2) * q[i3] * q[i12])
            + (7.268971e-05) * ((2) * q[i3] * q[i15]) + (-2.084037e-04) * ((2) * q[i3] * q[i16]) + (-1.649753e-04) * ((2) * q[i3] * q[i19])
            + (-3.124390e-04) * ((2) * q[i3] * q[i20]) + (-1.674498e-04) * ((2) * q[i3] * q[i21]) + (-1.204875e-04) * ((2) * q[i3] * q[i22])
            + (-1.305402e-03) * (q[i4] * q[i4]) + (9.509695e-04) * (q[i5] * q[i5]) + (-1.337203e-03) * (q[i6] * q[i6]) + (-3.771542e-04) * (q[i7] * q[i7])
            + (-3.937382e-04) * (q[i8] * q[i8]) + (4.354672e-04) * (q[i9] * q[i9]) + (-8.072043e-04) * (q[i10] * q[i10]) + (3.976655e-05) * (q[i11] * q[i11])
            + (3.180134e-04) * (q[i12] * q[i12]) + (-6.305771e-04) * (q[i15] * q[i15]) + (3.251068e-04) * (q[i16] * q[i16])
            + (-4.560208e-04) * (q[i19] * q[i19]) + (-1.282047e-04) * (q[i20] * q[i20]) + (-1.587875e-06) * (q[i21] * q[i21])
            + (1.493617e-04) * (q[i22] * q[i22]) + (-2.378489e-03) * (q[i0] * q[i1]) + (-3.557220e-03) * (q[i0] * q[i2]) + (1.336112e-02) * (q[i0] * q[i4])
            + (9.800715e-03) * (q[i0] * q[i5]) + (-1.615880e-02) * (q[i0] * q[i6]) + (3.259265e-03) * (q[i0] * q[i7]) + (-1.176956e-03) * (q[i0] * q[i8])
            + (1.243164e-02) * (q[i0] * q[i9]) + (-3.773849e-03) * (q[i0] * q[i10]) + (3.946048e-04) * (q[i0] * q[i11]) + (-9.528819e-04) * (q[i0] * q[i12])
            + (-1.674781e-03) * (q[i0] * q[i15]) + (-3.861911e-03) * (q[i0] * q[i16]) + (-7.626717e-04) * (q[i0] * q[i19]) + (4.278285e-05) * (q[i0] * q[i20])
            + (-1.361292e-04) * (q[i0] * q[i21]) + (-8.498490e-04) * (q[i0] * q[i22]) + (1.905476e-03) * (q[i1] * q[i2]) + (-1.336134e-02) * (q[i1] * q[i4])
            + (-3.748302e-03) * (q[i1] * q[i5]) + (3.136272e-03) * (q[i1] * q[i6]) + (-8.651054e-04) * (q[i1] * q[i7]) + (2.425740e-04) * (q[i1] * q[i8])
            + (-9.725343e-04) * (q[i1] * q[i9]) + (-1.391450e-04) * (q[i1] * q[i10]) + (2.653701e-04) * (q[i1] * q[i11]) + (3.474245e-05) * (q[i1] * q[i12])
            + (5.455054e-04) * (q[i1] * q[i15]) + (5.488424e-04) * (q[i1] * q[i16]) + (1.523011e-05) * (q[i1] * q[i19]) + (2.528618e-05) * (q[i1] * q[i20])
            + (6.851692e-04) * (q[i1] * q[i21]) + (2.598052e-04) * (q[i1] * q[i22]) + (-1.073665e-05) * (q[i2] * q[i4]) + (8.591752e-03) * (q[i2] * q[i5])
            + (-3.653709e-03) * (q[i2] * q[i6]) + (1.358948e-03) * (q[i2] * q[i7]) + (1.127051e-04) * (q[i2] * q[i8]) + (2.158290e-03) * (q[i2] * q[i9])
            + (-1.252840e-03) * (q[i2] * q[i10]) + (-2.804176e-04) * (q[i2] * q[i11]) + (3.965130e-04) * (q[i2] * q[i12]) + (4.431432e-04) * (q[i2] * q[i15])
            + (-9.113398e-04) * (q[i2] * q[i16]) + (-4.642063e-04) * (q[i2] * q[i19]) + (-2.037979e-04) * (q[i2] * q[i20]) + (-4.673228e-04) * (q[i2] * q[i21])
            + (2.693041e-04) * (q[i2] * q[i22]) + (1.721855e-05) * (q[i4] * q[i5]) + (-4.119390e-03) * (q[i4] * q[i6]) + (-4.095398e-03) * (q[i4] * q[i7])
            + (-2.612025e-03) * (q[i4] * q[i8]) + (-1.389606e-03) * (q[i4] * q[i9]) + (-1.383767e-03) * (q[i4] * q[i10]) + (1.854835e-04) * (q[i4] * q[i11])
            + (-2.100593e-04) * (q[i4] * q[i12]) + (-1.056834e-04) * (q[i4] * q[i15]) + (-1.009872e-04) * (q[i4] * q[i16]) + (4.109163e-04) * (q[i4] * q[i19])
            + (4.027741e-04) * (q[i4] * q[i20]) + (-1.784566e-04) * (q[i4] * q[i21]) + (1.765279e-04) * (q[i4] * q[i22]) + (-5.860166e-03) * (q[i5] * q[i6])
            + (-3.410167e-04) * (q[i5] * q[i7]) + (-5.676358e-03) * (q[i5] * q[i8]) + (-9.961027e-04) * (q[i5] * q[i9]) + (-6.296811e-04) * (q[i5] * q[i10])
            + (-6.531907e-04) * (q[i5] * q[i11]) + (-4.466833e-04) * (q[i5] * q[i12]) + (2.362346e-04) * (q[i5] * q[i15]) + (5.074723e-05) * (q[i5] * q[i16])
            + (1.175286e-04) * (q[i5] * q[i19]) + (-8.921760e-04) * (q[i5] * q[i20]) + (-2.468295e-04) * (q[i5] * q[i21]) + (-4.020164e-04) * (q[i5] * q[i22])
            + (1.879310e-03) * (q[i6] * q[i7]) + (8.413058e-04) * (q[i6] * q[i8]) + (7.022871e-04) * (q[i6] * q[i9]) + (-4.007765e-04) * (q[i6] * q[i10])
            + (-4.024529e-04) * (q[i6] * q[i11]) + (-1.035896e-03) * (q[i6] * q[i12]) + (-9.391136e-04) * (q[i6] * q[i15]) + (-1.784537e-05) * (q[i6] * q[i16])
            + (5.723820e-05) * (q[i6] * q[i19]) + (5.647931e-04) * (q[i6] * q[i20]) + (-2.184872e-05) * (q[i6] * q[i21]) + (-4.243971e-04) * (q[i6] * q[i22])
            + (2.840954e-04) * (q[i7] * q[i8]) + (-1.541094e-03) * (q[i7] * q[i9]) + (-1.745114e-03) * (q[i7] * q[i10]) + (-3.231969e-06) * (q[i7] * q[i11])
            + (-4.333243e-04) * (q[i7] * q[i12]) + (9.098021e-04) * (q[i7] * q[i15]) + (8.037877e-04) * (q[i7] * q[i16]) + (-8.243394e-04) * (q[i7] * q[i19])
            + (3.347674e-04) * (q[i7] * q[i20]) + (-5.588223e-04) * (q[i7] * q[i21]) + (4.399826e-04) * (q[i7] * q[i22]) + (-9.853283e-04) * (q[i8] * q[i9])
            + (9.389851e-04) * (q[i8] * q[i10]) + (8.302458e-04) * (q[i8] * q[i11]) + (3.707437e-04) * (q[i8] * q[i12]) + (-1.874093e-03) * (q[i8] * q[i15])
            + (1.586166e-03) * (q[i8] * q[i16]) + (-1.489504e-04) * (q[i8] * q[i19]) + (-5.408535e-05) * (q[i8] * q[i20]) + (7.900402e-05) * (q[i8] * q[i21])
            + (4.263331e-04) * (q[i8] * q[i22]) + (-4.769213e-04) * (q[i9] * q[i10]) + (3.395112e-04) * (q[i9] * q[i11]) + (3.349317e-04) * (q[i9] * q[i12])
            + (-1.930683e-04) * (q[i9] * q[i15]) + (-1.237888e-04) * (q[i9] * q[i16]) + (1.574528e-04) * (q[i9] * q[i19]) + (9.604014e-05) * (q[i9] * q[i20])
            + (1.093087e-04) * (q[i9] * q[i21]) + (-4.633238e-05) * (q[i9] * q[i22]) + (-4.832726e-04) * (q[i10] * q[i11]) + (9.182299e-05) * (q[i10] * q[i12])
            + (3.303860e-05) * (q[i10] * q[i15]) + (2.333993e-04) * (q[i10] * q[i16]) + (6.864690e-05) * (q[i10] * q[i19]) + (2.429382e-04) * (q[i10] * q[i20])
            + (-1.674627e-04) * (q[i10] * q[i21]) + (-9.695850e-05) * (q[i10] * q[i22]) + (-2.917590e-04) * (q[i11] * q[i12])
            + (-1.024490e-03) * (q[i11] * q[i15]) + (-5.020910e-05) * (q[i11] * q[i16]) + (-4.190742e-05) * (q[i11] * q[i19])
            + (1.218354e-04) * (q[i11] * q[i20]) + (-7.313836e-04) * (q[i11] * q[i21]) + (-8.641867e-05) * (q[i11] * q[i22])
            + (-2.073332e-04) * (q[i12] * q[i15]) + (-1.170318e-03) * (q[i12] * q[i16]) + (2.762868e-04) * (q[i12] * q[i19])
            + (-4.230714e-04) * (q[i12] * q[i20]) + (-3.968007e-05) * (q[i12] * q[i21]) + (2.359377e-04) * (q[i12] * q[i22])
            + (2.405502e-04) * (q[i15] * q[i16]) + (-1.154513e-04) * (q[i15] * q[i19]) + (8.314693e-05) * (q[i15] * q[i20])
            + (-3.249219e-04) * (q[i15] * q[i21]) + (-2.351795e-04) * (q[i15] * q[i22]) + (-1.910909e-04) * (q[i16] * q[i19])
            + (3.031278e-04) * (q[i16] * q[i20]) + (9.856968e-05) * (q[i16] * q[i21]) + (-2.787869e-04) * (q[i16] * q[i22]) + (1.303709e-04) * (q[i19] * q[i20])
            + (-2.283146e-04) * (q[i19] * q[i21]) + (2.080684e-04) * (q[i19] * q[i22]) + (-2.150225e-04) * (q[i20] * q[i21])
            + (3.425700e-04) * (q[i20] * q[i22]) + (4.145291e-04) * (q[i21] * q[i22]);
      JQ[2][i4] = (-7.032224e-04) * (1) + (3.037401e-04) * ((2) * q[i4]) + (1.019550e-02) * (q[i0]) + (-1.045263e-01) * (q[i1]) + (-2.060847e-02) * (q[i2])
            + (-2.626730e-03) * (q[i3]) + (-7.867142e-04) * (q[i5]) + (-1.874132e-02) * (q[i6]) + (1.288080e-02) * (q[i7]) + (-1.477931e-03) * (q[i8])
            + (-5.158458e-03) * (q[i9]) + (3.126489e-03) * (q[i10]) + (-5.590388e-03) * (q[i11]) + (-3.875277e-03) * (q[i12]) + (-2.459107e-03) * (q[i15])
            + (1.907826e-03) * (q[i16]) + (-3.897407e-04) * (q[i19]) + (1.405618e-03) * (q[i20]) + (-2.048008e-04) * (q[i21]) + (-8.828745e-04) * (q[i22])
            + (-1.948477e-03) * (q[i0] * q[i0]) + (3.350859e-03) * (q[i1] * q[i1]) + (6.283564e-04) * (q[i2] * q[i2]) + (1.318724e-03) * (q[i3] * q[i3])
            + (1.713932e-03) * ((2) * q[i0] * q[i4]) + (-7.622494e-03) * ((2) * q[i1] * q[i4]) + (-1.485448e-03) * ((2) * q[i2] * q[i4])
            + (-1.305402e-03) * ((2) * q[i3] * q[i4]) + (-1.340514e-04) * ((3) * q[i4] * q[i4]) + (-5.512622e-04) * ((2) * q[i4] * q[i5])
            + (-2.999238e-03) * ((2) * q[i4] * q[i6]) + (1.018575e-02) * ((2) * q[i4] * q[i7]) + (4.897763e-03) * ((2) * q[i4] * q[i8])
            + (-7.027872e-04) * ((2) * q[i4] * q[i9]) + (1.253167e-03) * ((2) * q[i4] * q[i10]) + (-3.640856e-04) * ((2) * q[i4] * q[i11])
            + (6.490454e-04) * ((2) * q[i4] * q[i12]) + (-2.037655e-04) * ((2) * q[i4] * q[i15]) + (6.807479e-05) * ((2) * q[i4] * q[i16])
            + (-3.064979e-04) * ((2) * q[i4] * q[i19]) + (-1.632086e-04) * ((2) * q[i4] * q[i20]) + (1.172331e-04) * ((2) * q[i4] * q[i21])
            + (1.670597e-04) * ((2) * q[i4] * q[i22]) + (-9.547658e-04) * (q[i5] * q[i5]) + (3.797123e-04) * (q[i6] * q[i6]) + (1.369226e-03) * (q[i7] * q[i7])
            + (3.832575e-04) * (q[i8] * q[i8]) + (8.160768e-04) * (q[i9] * q[i9]) + (-4.293332e-04) * (q[i10] * q[i10]) + (-3.286597e-04) * (q[i11] * q[i11])
            + (-5.807313e-05) * (q[i12] * q[i12]) + (-3.253032e-04) * (q[i15] * q[i15]) + (6.351138e-04) * (q[i16] * q[i16])
            + (1.279469e-04) * (q[i19] * q[i19]) + (4.503998e-04) * (q[i20] * q[i20]) + (-1.481153e-04) * (q[i21] * q[i21])
            + (-1.566640e-06) * (q[i22] * q[i22]) + (2.358891e-03) * (q[i0] * q[i1]) + (-1.882166e-03) * (q[i0] * q[i2]) + (1.336112e-02) * (q[i0] * q[i3])
            + (3.740858e-03) * (q[i0] * q[i5]) + (-8.506806e-04) * (q[i0] * q[i6]) + (3.139579e-03) * (q[i0] * q[i7]) + (2.537469e-04) * (q[i0] * q[i8])
            + (-1.366809e-04) * (q[i0] * q[i9]) + (-9.703830e-04) * (q[i0] * q[i10]) + (-2.479629e-05) * (q[i0] * q[i11]) + (-2.565737e-04) * (q[i0] * q[i12])
            + (5.375513e-04) * (q[i0] * q[i15]) + (5.378835e-04) * (q[i0] * q[i16]) + (2.205854e-05) * (q[i0] * q[i19]) + (9.761715e-06) * (q[i0] * q[i20])
            + (-2.597395e-04) * (q[i0] * q[i21]) + (-6.792909e-04) * (q[i0] * q[i22]) + (3.555067e-03) * (q[i1] * q[i2]) + (-1.336134e-02) * (q[i1] * q[i3])
            + (-9.843163e-03) * (q[i1] * q[i5]) + (3.240810e-03) * (q[i1] * q[i6]) + (-1.622585e-02) * (q[i1] * q[i7]) + (-1.161245e-03) * (q[i1] * q[i8])
            + (-3.814247e-03) * (q[i1] * q[i9]) + (1.230341e-02) * (q[i1] * q[i10]) + (9.170136e-04) * (q[i1] * q[i11]) + (-4.105944e-04) * (q[i1] * q[i12])
            + (-3.807639e-03) * (q[i1] * q[i15]) + (-1.680345e-03) * (q[i1] * q[i16]) + (4.173312e-05) * (q[i1] * q[i19]) + (-7.562183e-04) * (q[i1] * q[i20])
            + (8.366162e-04) * (q[i1] * q[i21]) + (1.205473e-04) * (q[i1] * q[i22]) + (-1.073665e-05) * (q[i2] * q[i3]) + (-8.593283e-03) * (q[i2] * q[i5])
            + (1.356865e-03) * (q[i2] * q[i6]) + (-3.675611e-03) * (q[i2] * q[i7]) + (1.230089e-04) * (q[i2] * q[i8]) + (-1.264543e-03) * (q[i2] * q[i9])
            + (2.128636e-03) * (q[i2] * q[i10]) + (-4.179273e-04) * (q[i2] * q[i11]) + (2.623375e-04) * (q[i2] * q[i12]) + (-9.084603e-04) * (q[i2] * q[i15])
            + (4.579803e-04) * (q[i2] * q[i16]) + (-2.047420e-04) * (q[i2] * q[i19]) + (-4.625191e-04) * (q[i2] * q[i20]) + (-2.657561e-04) * (q[i2] * q[i21])
            + (4.650251e-04) * (q[i2] * q[i22]) + (1.721855e-05) * (q[i3] * q[i5]) + (-4.119390e-03) * (q[i3] * q[i6]) + (-4.095398e-03) * (q[i3] * q[i7])
            + (-2.612025e-03) * (q[i3] * q[i8]) + (-1.389606e-03) * (q[i3] * q[i9]) + (-1.383767e-03) * (q[i3] * q[i10]) + (1.854835e-04) * (q[i3] * q[i11])
            + (-2.100593e-04) * (q[i3] * q[i12]) + (-1.056834e-04) * (q[i3] * q[i15]) + (-1.009872e-04) * (q[i3] * q[i16]) + (4.109163e-04) * (q[i3] * q[i19])
            + (4.027741e-04) * (q[i3] * q[i20]) + (-1.784566e-04) * (q[i3] * q[i21]) + (1.765279e-04) * (q[i3] * q[i22]) + (-3.586758e-04) * (q[i5] * q[i6])
            + (-5.851652e-03) * (q[i5] * q[i7]) + (-5.683570e-03) * (q[i5] * q[i8]) + (-6.389476e-04) * (q[i5] * q[i9]) + (-9.861556e-04) * (q[i5] * q[i10])
            + (4.378300e-04) * (q[i5] * q[i11]) + (6.221890e-04) * (q[i5] * q[i12]) + (5.443202e-05) * (q[i5] * q[i15]) + (2.385429e-04) * (q[i5] * q[i16])
            + (-9.016316e-04) * (q[i5] * q[i19]) + (1.081047e-04) * (q[i5] * q[i20]) + (4.077822e-04) * (q[i5] * q[i21]) + (2.557705e-04) * (q[i5] * q[i22])
            + (-1.896358e-03) * (q[i6] * q[i7]) + (-2.832570e-04) * (q[i6] * q[i8]) + (1.757509e-03) * (q[i6] * q[i9]) + (1.530174e-03) * (q[i6] * q[i10])
            + (-4.370129e-04) * (q[i6] * q[i11]) + (2.431725e-06) * (q[i6] * q[i12]) + (-7.902417e-04) * (q[i6] * q[i15]) + (-9.180732e-04) * (q[i6] * q[i16])
            + (-3.333828e-04) * (q[i6] * q[i19]) + (8.223252e-04) * (q[i6] * q[i20]) + (4.330354e-04) * (q[i6] * q[i21]) + (-5.600775e-04) * (q[i6] * q[i22])
            + (-8.515030e-04) * (q[i7] * q[i8]) + (3.995365e-04) * (q[i7] * q[i9]) + (-6.864024e-04) * (q[i7] * q[i10]) + (-1.038503e-03) * (q[i7] * q[i11])
            + (-4.192667e-04) * (q[i7] * q[i12]) + (2.146712e-05) * (q[i7] * q[i15]) + (9.376283e-04) * (q[i7] * q[i16]) + (-5.666982e-04) * (q[i7] * q[i19])
            + (-5.485476e-05) * (q[i7] * q[i20]) + (-4.266081e-04) * (q[i7] * q[i21]) + (-2.061593e-05) * (q[i7] * q[i22]) + (-9.450100e-04) * (q[i8] * q[i9])
            + (9.708365e-04) * (q[i8] * q[i10]) + (3.666690e-04) * (q[i8] * q[i11]) + (8.563020e-04) * (q[i8] * q[i12]) + (-1.573262e-03) * (q[i8] * q[i15])
            + (1.892670e-03) * (q[i8] * q[i16]) + (5.975865e-05) * (q[i8] * q[i19]) + (1.447929e-04) * (q[i8] * q[i20]) + (4.272613e-04) * (q[i8] * q[i21])
            + (8.697111e-05) * (q[i8] * q[i22]) + (4.768357e-04) * (q[i9] * q[i10]) + (8.569621e-05) * (q[i9] * q[i11]) + (-4.901619e-04) * (q[i9] * q[i12])
            + (-2.303278e-04) * (q[i9] * q[i15]) + (-3.853204e-05) * (q[i9] * q[i16]) + (-2.364868e-04) * (q[i9] * q[i19]) + (-6.932773e-05) * (q[i9] * q[i20])
            + (-9.635595e-05) * (q[i9] * q[i21]) + (-1.695343e-04) * (q[i9] * q[i22]) + (3.251140e-04) * (q[i10] * q[i11]) + (3.369470e-04) * (q[i10] * q[i12])
            + (1.181967e-04) * (q[i10] * q[i15]) + (1.877344e-04) * (q[i10] * q[i16]) + (-9.532546e-05) * (q[i10] * q[i19])
            + (-1.563696e-04) * (q[i10] * q[i20]) + (-4.528867e-05) * (q[i10] * q[i21]) + (1.107078e-04) * (q[i10] * q[i22])
            + (2.936496e-04) * (q[i11] * q[i12]) + (-1.160440e-03) * (q[i11] * q[i15]) + (-2.101134e-04) * (q[i11] * q[i16])
            + (-4.214670e-04) * (q[i11] * q[i19]) + (2.725329e-04) * (q[i11] * q[i20]) + (-2.377437e-04) * (q[i11] * q[i21])
            + (3.716635e-05) * (q[i11] * q[i22]) + (-4.993484e-05) * (q[i12] * q[i15]) + (-1.028767e-03) * (q[i12] * q[i16])
            + (1.145429e-04) * (q[i12] * q[i19]) + (-3.911072e-05) * (q[i12] * q[i20]) + (9.036042e-05) * (q[i12] * q[i21]) + (7.318424e-04) * (q[i12] * q[i22])
            + (-2.399481e-04) * (q[i15] * q[i16]) + (-3.102687e-04) * (q[i15] * q[i19]) + (1.892475e-04) * (q[i15] * q[i20])
            + (-2.721388e-04) * (q[i15] * q[i21]) + (9.849923e-05) * (q[i15] * q[i22]) + (-8.271905e-05) * (q[i16] * q[i19])
            + (1.078302e-04) * (q[i16] * q[i20]) + (-2.327941e-04) * (q[i16] * q[i21]) + (-3.290808e-04) * (q[i16] * q[i22])
            + (-1.338756e-04) * (q[i19] * q[i20]) + (3.439801e-04) * (q[i19] * q[i21]) + (-2.058941e-04) * (q[i19] * q[i22])
            + (2.046500e-04) * (q[i20] * q[i21]) + (-2.295565e-04) * (q[i20] * q[i22]) + (-4.134249e-04) * (q[i21] * q[i22]);
      JQ[2][i5] = (-2.487388e-05) * (1) + (-1.974648e-03) * ((2) * q[i5]) + (2.311095e-03) * (q[i0]) + (2.323533e-03) * (q[i1]) + (7.256947e-02) * (q[i2])
            + (-7.588072e-04) * (q[i3]) + (-7.867142e-04) * (q[i4]) + (-1.212218e-02) * (q[i6]) + (1.210702e-02) * (q[i7]) + (6.977615e-06) * (q[i8])
            + (-4.796828e-03) * (q[i9]) + (4.750550e-03) * (q[i10]) + (-4.371294e-03) * (q[i11]) + (-4.297983e-03) * (q[i12]) + (2.815028e-03) * (q[i15])
            + (-2.854398e-03) * (q[i16]) + (-5.138955e-04) * (q[i19]) + (4.998270e-04) * (q[i20]) + (1.153886e-03) * (q[i21]) + (1.139228e-03) * (q[i22])
            + (-1.714863e-03) * (q[i0] * q[i0]) + (1.726265e-03) * (q[i1] * q[i1]) + (-2.788829e-06) * (q[i2] * q[i2]) + (5.198465e-04) * (q[i3] * q[i3])
            + (-5.512622e-04) * (q[i4] * q[i4]) + (1.598485e-04) * ((2) * q[i0] * q[i5]) + (-1.496662e-04) * ((2) * q[i1] * q[i5])
            + (-1.879650e-05) * ((2) * q[i2] * q[i5]) + (9.509695e-04) * ((2) * q[i3] * q[i5]) + (-9.547658e-04) * ((2) * q[i4] * q[i5])
            + (1.624199e-05) * ((3) * q[i5] * q[i5]) + (1.201563e-04) * ((2) * q[i5] * q[i6]) + (1.129247e-04) * ((2) * q[i5] * q[i7])
            + (3.681054e-03) * ((2) * q[i5] * q[i8]) + (1.445221e-04) * ((2) * q[i5] * q[i9]) + (1.495364e-04) * ((2) * q[i5] * q[i10])
            + (2.935778e-04) * ((2) * q[i5] * q[i11]) + (-2.990441e-04) * ((2) * q[i5] * q[i12]) + (-1.269300e-03) * ((2) * q[i5] * q[i15])
            + (-1.285440e-03) * ((2) * q[i5] * q[i16]) + (-7.327898e-04) * ((2) * q[i5] * q[i19]) + (-7.266314e-04) * ((2) * q[i5] * q[i20])
            + (-3.969436e-04) * ((2) * q[i5] * q[i21]) + (3.976714e-04) * ((2) * q[i5] * q[i22]) + (5.578361e-04) * (q[i6] * q[i6])
            + (-5.615365e-04) * (q[i7] * q[i7]) + (5.006040e-06) * (q[i8] * q[i8]) + (8.902593e-04) * (q[i9] * q[i9]) + (-8.848187e-04) * (q[i10] * q[i10])
            + (1.262623e-03) * (q[i11] * q[i11]) + (-1.318934e-03) * (q[i12] * q[i12]) + (2.926691e-04) * (q[i15] * q[i15])
            + (-2.914479e-04) * (q[i16] * q[i16]) + (-2.332126e-04) * (q[i19] * q[i19]) + (2.367955e-04) * (q[i20] * q[i20])
            + (-4.019923e-04) * (q[i21] * q[i21]) + (4.060778e-04) * (q[i22] * q[i22]) + (9.988349e-06) * (q[i0] * q[i1]) + (-1.790790e-03) * (q[i0] * q[i2])
            + (9.800715e-03) * (q[i0] * q[i3]) + (3.740858e-03) * (q[i0] * q[i4]) + (4.591833e-03) * (q[i0] * q[i6]) + (-9.150038e-04) * (q[i0] * q[i7])
            + (-1.960694e-05) * (q[i0] * q[i8]) + (2.434291e-04) * (q[i0] * q[i9]) + (1.174825e-03) * (q[i0] * q[i10]) + (-1.086987e-03) * (q[i0] * q[i11])
            + (1.086708e-03) * (q[i0] * q[i12]) + (-8.253056e-04) * (q[i0] * q[i15]) + (-1.200608e-03) * (q[i0] * q[i16]) + (-1.774556e-04) * (q[i0] * q[i19])
            + (-3.916660e-04) * (q[i0] * q[i20]) + (-8.283292e-06) * (q[i0] * q[i21]) + (6.741703e-04) * (q[i0] * q[i22]) + (1.789451e-03) * (q[i1] * q[i2])
            + (-3.748302e-03) * (q[i1] * q[i3]) + (-9.843163e-03) * (q[i1] * q[i4]) + (-9.311442e-04) * (q[i1] * q[i6]) + (4.571576e-03) * (q[i1] * q[i7])
            + (8.817743e-06) * (q[i1] * q[i8]) + (1.174705e-03) * (q[i1] * q[i9]) + (2.354629e-04) * (q[i1] * q[i10]) + (-1.123989e-03) * (q[i1] * q[i11])
            + (1.038978e-03) * (q[i1] * q[i12]) + (-1.199295e-03) * (q[i1] * q[i15]) + (-8.293675e-04) * (q[i1] * q[i16]) + (-3.957794e-04) * (q[i1] * q[i19])
            + (-1.755813e-04) * (q[i1] * q[i20]) + (-6.676990e-04) * (q[i1] * q[i21]) + (1.252290e-05) * (q[i1] * q[i22]) + (8.591752e-03) * (q[i2] * q[i3])
            + (-8.593283e-03) * (q[i2] * q[i4]) + (-2.200894e-03) * (q[i2] * q[i6]) + (-2.212931e-03) * (q[i2] * q[i7]) + (-1.136204e-02) * (q[i2] * q[i8])
            + (6.123080e-03) * (q[i2] * q[i9]) + (6.063427e-03) * (q[i2] * q[i10]) + (1.802504e-03) * (q[i2] * q[i11]) + (-1.910099e-03) * (q[i2] * q[i12])
            + (-6.389843e-03) * (q[i2] * q[i15]) + (-6.479207e-03) * (q[i2] * q[i16]) + (2.116533e-05) * (q[i2] * q[i19]) + (2.157559e-05) * (q[i2] * q[i20])
            + (7.172603e-04) * (q[i2] * q[i21]) + (-7.400227e-04) * (q[i2] * q[i22]) + (1.721855e-05) * (q[i3] * q[i4]) + (-5.860166e-03) * (q[i3] * q[i6])
            + (-3.410167e-04) * (q[i3] * q[i7]) + (-5.676358e-03) * (q[i3] * q[i8]) + (-9.961027e-04) * (q[i3] * q[i9]) + (-6.296811e-04) * (q[i3] * q[i10])
            + (-6.531907e-04) * (q[i3] * q[i11]) + (-4.466833e-04) * (q[i3] * q[i12]) + (2.362346e-04) * (q[i3] * q[i15]) + (5.074723e-05) * (q[i3] * q[i16])
            + (1.175286e-04) * (q[i3] * q[i19]) + (-8.921760e-04) * (q[i3] * q[i20]) + (-2.468295e-04) * (q[i3] * q[i21]) + (-4.020164e-04) * (q[i3] * q[i22])
            + (-3.586758e-04) * (q[i4] * q[i6]) + (-5.851652e-03) * (q[i4] * q[i7]) + (-5.683570e-03) * (q[i4] * q[i8]) + (-6.389476e-04) * (q[i4] * q[i9])
            + (-9.861556e-04) * (q[i4] * q[i10]) + (4.378300e-04) * (q[i4] * q[i11]) + (6.221890e-04) * (q[i4] * q[i12]) + (5.443202e-05) * (q[i4] * q[i15])
            + (2.385429e-04) * (q[i4] * q[i16]) + (-9.016316e-04) * (q[i4] * q[i19]) + (1.081047e-04) * (q[i4] * q[i20]) + (4.077822e-04) * (q[i4] * q[i21])
            + (2.557705e-04) * (q[i4] * q[i22]) + (5.752728e-06) * (q[i6] * q[i7]) + (1.156394e-03) * (q[i6] * q[i8]) + (2.235833e-03) * (q[i6] * q[i9])
            + (-1.380465e-03) * (q[i6] * q[i10]) + (-5.280932e-04) * (q[i6] * q[i11]) + (-9.891163e-04) * (q[i6] * q[i12]) + (1.541645e-03) * (q[i6] * q[i15])
            + (2.314176e-04) * (q[i6] * q[i16]) + (1.809299e-04) * (q[i6] * q[i19]) + (-3.780468e-04) * (q[i6] * q[i20]) + (-4.964292e-05) * (q[i6] * q[i21])
            + (8.301751e-05) * (q[i6] * q[i22]) + (-1.156110e-03) * (q[i7] * q[i8]) + (1.387223e-03) * (q[i7] * q[i9]) + (-2.220273e-03) * (q[i7] * q[i10])
            + (-1.002169e-03) * (q[i7] * q[i11]) + (-5.294600e-04) * (q[i7] * q[i12]) + (-2.394354e-04) * (q[i7] * q[i15]) + (-1.548307e-03) * (q[i7] * q[i16])
            + (3.610782e-04) * (q[i7] * q[i19]) + (-1.750359e-04) * (q[i7] * q[i20]) + (7.739277e-05) * (q[i7] * q[i21]) + (-4.801370e-05) * (q[i7] * q[i22])
            + (1.313991e-03) * (q[i8] * q[i9]) + (-1.302686e-03) * (q[i8] * q[i10]) + (5.523023e-04) * (q[i8] * q[i11]) + (5.779302e-04) * (q[i8] * q[i12])
            + (1.110366e-03) * (q[i8] * q[i15]) + (-1.141076e-03) * (q[i8] * q[i16]) + (-4.558482e-04) * (q[i8] * q[i19]) + (4.501319e-04) * (q[i8] * q[i20])
            + (1.165022e-04) * (q[i8] * q[i21]) + (1.116974e-04) * (q[i8] * q[i22]) + (7.806473e-07) * (q[i9] * q[i10]) + (-3.886678e-04) * (q[i9] * q[i11])
            + (-3.133345e-04) * (q[i9] * q[i12]) + (7.187954e-04) * (q[i9] * q[i15]) + (-1.300566e-04) * (q[i9] * q[i16]) + (2.208338e-04) * (q[i9] * q[i19])
            + (-1.474719e-04) * (q[i9] * q[i20]) + (-5.418967e-04) * (q[i9] * q[i21]) + (-2.203100e-04) * (q[i9] * q[i22]) + (-3.202338e-04) * (q[i10] * q[i11])
            + (-3.828427e-04) * (q[i10] * q[i12]) + (1.317935e-04) * (q[i10] * q[i15]) + (-7.160931e-04) * (q[i10] * q[i16])
            + (1.491292e-04) * (q[i10] * q[i19]) + (-2.241152e-04) * (q[i10] * q[i20]) + (-2.214388e-04) * (q[i10] * q[i21])
            + (-5.331750e-04) * (q[i10] * q[i22]) + (1.470583e-05) * (q[i11] * q[i12]) + (-2.237288e-03) * (q[i11] * q[i15])
            + (-2.229771e-05) * (q[i11] * q[i16]) + (2.494105e-04) * (q[i11] * q[i19]) + (7.149178e-05) * (q[i11] * q[i20])
            + (-2.693622e-04) * (q[i11] * q[i21]) + (1.497267e-05) * (q[i11] * q[i22]) + (-1.129334e-05) * (q[i12] * q[i15])
            + (-2.231427e-03) * (q[i12] * q[i16]) + (6.981440e-05) * (q[i12] * q[i19]) + (2.474242e-04) * (q[i12] * q[i20])
            + (-1.618050e-05) * (q[i12] * q[i21]) + (2.787426e-04) * (q[i12] * q[i22]) + (-3.547791e-06) * (q[i15] * q[i16])
            + (-6.461589e-04) * (q[i15] * q[i19]) + (-2.655420e-04) * (q[i15] * q[i20]) + (7.849938e-04) * (q[i15] * q[i21])
            + (2.021711e-04) * (q[i15] * q[i22]) + (2.734731e-04) * (q[i16] * q[i19]) + (6.321716e-04) * (q[i16] * q[i20]) + (2.034900e-04) * (q[i16] * q[i21])
            + (7.961114e-04) * (q[i16] * q[i22]) + (4.417112e-06) * (q[i19] * q[i20]) + (6.504661e-04) * (q[i19] * q[i21]) + (-1.864600e-04) * (q[i19] * q[i22])
            + (-1.819648e-04) * (q[i20] * q[i21]) + (6.460300e-04) * (q[i20] * q[i22]) + (-3.876848e-06) * (q[i21] * q[i22]);
      JQ[2][i6] = (1.310088e-01) * (1) + (-4.170347e-04) * ((2) * q[i6]) + (5.812794e-03) * (q[i0]) + (-1.376621e-03) * (q[i1]) + (-6.440818e-04) * (q[i2])
            + (-1.294811e-02) * (q[i3]) + (-1.874132e-02) * (q[i4]) + (-1.212218e-02) * (q[i5]) + (-5.955809e-03) * (q[i7]) + (-3.563051e-05) * (q[i8])
            + (-9.249520e-03) * (q[i9]) + (4.391855e-03) * (q[i10]) + (-7.010548e-04) * (q[i11]) + (8.597632e-04) * (q[i12]) + (2.545774e-03) * (q[i15])
            + (6.446622e-03) * (q[i16]) + (-2.650868e-04) * (q[i19]) + (4.161348e-04) * (q[i20]) + (5.154993e-04) * (q[i21]) + (1.831471e-04) * (q[i22])
            + (-2.474288e-02) * (q[i0] * q[i0]) + (-2.540755e-03) * (q[i1] * q[i1]) + (-1.645044e-03) * (q[i2] * q[i2]) + (1.017437e-02) * (q[i3] * q[i3])
            + (-2.999238e-03) * (q[i4] * q[i4]) + (1.201563e-04) * (q[i5] * q[i5]) + (-4.359479e-03) * ((2) * q[i0] * q[i6])
            + (-4.147658e-04) * ((2) * q[i1] * q[i6]) + (-8.370063e-04) * ((2) * q[i2] * q[i6]) + (-1.337203e-03) * ((2) * q[i3] * q[i6])
            + (3.797123e-04) * ((2) * q[i4] * q[i6]) + (5.578361e-04) * ((2) * q[i5] * q[i6]) + (-2.034647e-03) * ((3) * q[i6] * q[i6])
            + (-1.443923e-03) * ((2) * q[i6] * q[i7]) + (1.496093e-03) * ((2) * q[i6] * q[i8]) + (-5.717403e-04) * ((2) * q[i6] * q[i9])
            + (-3.134357e-04) * ((2) * q[i6] * q[i10]) + (-5.248814e-04) * ((2) * q[i6] * q[i11]) + (-2.601745e-04) * ((2) * q[i6] * q[i12])
            + (-5.445084e-04) * ((2) * q[i6] * q[i15]) + (-2.344344e-04) * ((2) * q[i6] * q[i16]) + (-4.345726e-04) * ((2) * q[i6] * q[i19])
            + (7.992784e-05) * ((2) * q[i6] * q[i20]) + (-1.263081e-04) * ((2) * q[i6] * q[i21]) + (2.285076e-04) * ((2) * q[i6] * q[i22])
            + (-1.441506e-03) * (q[i7] * q[i7]) + (2.038677e-03) * (q[i8] * q[i8]) + (-2.425500e-03) * (q[i9] * q[i9]) + (3.637730e-04) * (q[i10] * q[i10])
            + (-4.280665e-04) * (q[i11] * q[i11]) + (-1.049009e-05) * (q[i12] * q[i12]) + (-3.336203e-05) * (q[i15] * q[i15])
            + (8.003635e-04) * (q[i16] * q[i16]) + (-1.507277e-04) * (q[i19] * q[i19]) + (1.439755e-04) * (q[i20] * q[i20]) + (9.304271e-05) * (q[i21] * q[i21])
            + (4.638544e-04) * (q[i22] * q[i22]) + (6.064606e-03) * (q[i0] * q[i1]) + (-1.163352e-02) * (q[i0] * q[i2]) + (-1.615880e-02) * (q[i0] * q[i3])
            + (-8.506806e-04) * (q[i0] * q[i4]) + (4.591833e-03) * (q[i0] * q[i5]) + (1.025195e-02) * (q[i0] * q[i7]) + (3.391187e-03) * (q[i0] * q[i8])
            + (-1.594058e-03) * (q[i0] * q[i9]) + (2.157548e-03) * (q[i0] * q[i10]) + (-1.685007e-03) * (q[i0] * q[i11]) + (-1.091461e-03) * (q[i0] * q[i12])
            + (-4.290466e-04) * (q[i0] * q[i15]) + (1.370662e-03) * (q[i0] * q[i16]) + (-1.298252e-03) * (q[i0] * q[i19]) + (9.889740e-04) * (q[i0] * q[i20])
            + (-1.969670e-04) * (q[i0] * q[i21]) + (8.597856e-06) * (q[i0] * q[i22]) + (7.748543e-04) * (q[i1] * q[i2]) + (3.136272e-03) * (q[i1] * q[i3])
            + (3.240810e-03) * (q[i1] * q[i4]) + (-9.311442e-04) * (q[i1] * q[i5]) + (-1.024657e-02) * (q[i1] * q[i7]) + (5.875240e-04) * (q[i1] * q[i8])
            + (2.119731e-05) * (q[i1] * q[i9]) + (-1.360581e-03) * (q[i1] * q[i10]) + (-7.651621e-05) * (q[i1] * q[i11]) + (4.752027e-05) * (q[i1] * q[i12])
            + (3.144774e-05) * (q[i1] * q[i15]) + (-4.277297e-04) * (q[i1] * q[i16]) + (1.368596e-04) * (q[i1] * q[i19]) + (-3.205544e-04) * (q[i1] * q[i20])
            + (-4.913701e-04) * (q[i1] * q[i21]) + (-8.847691e-06) * (q[i1] * q[i22]) + (-3.653709e-03) * (q[i2] * q[i3]) + (1.356865e-03) * (q[i2] * q[i4])
            + (-2.200894e-03) * (q[i2] * q[i5]) + (-1.284721e-05) * (q[i2] * q[i7]) + (8.949380e-03) * (q[i2] * q[i8]) + (4.654102e-04) * (q[i2] * q[i9])
            + (-2.886947e-04) * (q[i2] * q[i10]) + (-1.117621e-03) * (q[i2] * q[i11]) + (2.174589e-04) * (q[i2] * q[i12]) + (-6.428325e-04) * (q[i2] * q[i15])
            + (6.249175e-04) * (q[i2] * q[i16]) + (-7.928593e-04) * (q[i2] * q[i19]) + (9.333862e-05) * (q[i2] * q[i20]) + (2.855538e-05) * (q[i2] * q[i21])
            + (8.983287e-05) * (q[i2] * q[i22]) + (-4.119390e-03) * (q[i3] * q[i4]) + (-5.860166e-03) * (q[i3] * q[i5]) + (1.879310e-03) * (q[i3] * q[i7])
            + (8.413058e-04) * (q[i3] * q[i8]) + (7.022871e-04) * (q[i3] * q[i9]) + (-4.007765e-04) * (q[i3] * q[i10]) + (-4.024529e-04) * (q[i3] * q[i11])
            + (-1.035896e-03) * (q[i3] * q[i12]) + (-9.391136e-04) * (q[i3] * q[i15]) + (-1.784537e-05) * (q[i3] * q[i16]) + (5.723820e-05) * (q[i3] * q[i19])
            + (5.647931e-04) * (q[i3] * q[i20]) + (-2.184872e-05) * (q[i3] * q[i21]) + (-4.243971e-04) * (q[i3] * q[i22]) + (-3.586758e-04) * (q[i4] * q[i5])
            + (-1.896358e-03) * (q[i4] * q[i7]) + (-2.832570e-04) * (q[i4] * q[i8]) + (1.757509e-03) * (q[i4] * q[i9]) + (1.530174e-03) * (q[i4] * q[i10])
            + (-4.370129e-04) * (q[i4] * q[i11]) + (2.431725e-06) * (q[i4] * q[i12]) + (-7.902417e-04) * (q[i4] * q[i15]) + (-9.180732e-04) * (q[i4] * q[i16])
            + (-3.333828e-04) * (q[i4] * q[i19]) + (8.223252e-04) * (q[i4] * q[i20]) + (4.330354e-04) * (q[i4] * q[i21]) + (-5.600775e-04) * (q[i4] * q[i22])
            + (5.752728e-06) * (q[i5] * q[i7]) + (1.156394e-03) * (q[i5] * q[i8]) + (2.235833e-03) * (q[i5] * q[i9]) + (-1.380465e-03) * (q[i5] * q[i10])
            + (-5.280932e-04) * (q[i5] * q[i11]) + (-9.891163e-04) * (q[i5] * q[i12]) + (1.541645e-03) * (q[i5] * q[i15]) + (2.314176e-04) * (q[i5] * q[i16])
            + (1.809299e-04) * (q[i5] * q[i19]) + (-3.780468e-04) * (q[i5] * q[i20]) + (-4.964292e-05) * (q[i5] * q[i21]) + (8.301751e-05) * (q[i5] * q[i22])
            + (-2.277266e-03) * (q[i7] * q[i8]) + (3.240125e-04) * (q[i7] * q[i9]) + (3.225811e-04) * (q[i7] * q[i10]) + (-3.633376e-04) * (q[i7] * q[i11])
            + (3.594762e-04) * (q[i7] * q[i12]) + (5.321298e-04) * (q[i7] * q[i15]) + (5.348429e-04) * (q[i7] * q[i16]) + (3.668577e-04) * (q[i7] * q[i19])
            + (3.663251e-04) * (q[i7] * q[i20]) + (7.693029e-05) * (q[i7] * q[i21]) + (-8.031516e-05) * (q[i7] * q[i22]) + (-6.389555e-04) * (q[i8] * q[i9])
            + (-8.771073e-04) * (q[i8] * q[i10]) + (-5.018576e-04) * (q[i8] * q[i11]) + (-5.343117e-05) * (q[i8] * q[i12]) + (-3.231139e-04) * (q[i8] * q[i15])
            + (-3.754556e-04) * (q[i8] * q[i16]) + (-1.490583e-04) * (q[i8] * q[i19]) + (-1.835712e-04) * (q[i8] * q[i20]) + (-5.335340e-04) * (q[i8] * q[i21])
            + (3.416559e-05) * (q[i8] * q[i22]) + (-6.427358e-04) * (q[i9] * q[i10]) + (-1.433083e-04) * (q[i9] * q[i11]) + (-7.875434e-05) * (q[i9] * q[i12])
            + (-2.687738e-04) * (q[i9] * q[i15]) + (-4.103087e-04) * (q[i9] * q[i16]) + (1.042731e-04) * (q[i9] * q[i19]) + (-6.404165e-05) * (q[i9] * q[i20])
            + (-1.674403e-04) * (q[i9] * q[i21]) + (-2.159308e-04) * (q[i9] * q[i22]) + (-1.313475e-04) * (q[i10] * q[i11])
            + (-1.012554e-05) * (q[i10] * q[i12]) + (-5.974720e-05) * (q[i10] * q[i15]) + (2.814061e-04) * (q[i10] * q[i16])
            + (-1.477668e-04) * (q[i10] * q[i19]) + (5.322371e-05) * (q[i10] * q[i20]) + (-2.659831e-04) * (q[i10] * q[i21])
            + (-9.791022e-05) * (q[i10] * q[i22]) + (-2.782700e-04) * (q[i11] * q[i12]) + (-1.034194e-04) * (q[i11] * q[i15])
            + (5.382410e-05) * (q[i11] * q[i16]) + (-2.237864e-04) * (q[i11] * q[i19]) + (5.266348e-05) * (q[i11] * q[i20])
            + (-1.794102e-04) * (q[i11] * q[i21]) + (2.074214e-04) * (q[i11] * q[i22]) + (-7.543038e-05) * (q[i12] * q[i15])
            + (4.898942e-05) * (q[i12] * q[i16]) + (-1.779052e-04) * (q[i12] * q[i19]) + (2.987062e-04) * (q[i12] * q[i20])
            + (-3.023907e-05) * (q[i12] * q[i21]) + (-2.987766e-04) * (q[i12] * q[i22]) + (-2.468928e-05) * (q[i15] * q[i16])
            + (-2.205203e-04) * (q[i15] * q[i19]) + (9.431270e-05) * (q[i15] * q[i20]) + (3.049960e-04) * (q[i15] * q[i21])
            + (-4.658677e-05) * (q[i15] * q[i22]) + (-7.431485e-05) * (q[i16] * q[i19]) + (-1.600898e-04) * (q[i16] * q[i20])
            + (-2.786086e-04) * (q[i16] * q[i21]) + (-4.603793e-04) * (q[i16] * q[i22]) + (-4.335967e-05) * (q[i19] * q[i20])
            + (2.546358e-04) * (q[i19] * q[i21]) + (2.654591e-05) * (q[i19] * q[i22]) + (4.148696e-05) * (q[i20] * q[i21]) + (-2.644389e-04) * (q[i20] * q[i22])
            + (-1.885125e-05) * (q[i21] * q[i22]);
      JQ[2][i7] = (1.302818e-01) * (1) + (-4.007243e-04) * ((2) * q[i7]) + (1.397899e-03) * (q[i0]) + (-5.877031e-03) * (q[i1]) + (6.527983e-04) * (q[i2])
            + (1.874437e-02) * (q[i3]) + (1.288080e-02) * (q[i4]) + (1.210702e-02) * (q[i5]) + (-5.955809e-03) * (q[i6]) + (-5.774932e-05) * (q[i8])
            + (4.408551e-03) * (q[i9]) + (-9.169212e-03) * (q[i10]) + (-8.042559e-04) * (q[i11]) + (7.330325e-04) * (q[i12]) + (6.341804e-03) * (q[i15])
            + (2.538173e-03) * (q[i16]) + (4.083241e-04) * (q[i19]) + (-2.641031e-04) * (q[i20]) + (-1.823671e-04) * (q[i21]) + (-5.026373e-04) * (q[i22])
            + (-2.529714e-03) * (q[i0] * q[i0]) + (-2.469259e-02) * (q[i1] * q[i1]) + (-1.659321e-03) * (q[i2] * q[i2]) + (-3.002781e-03) * (q[i3] * q[i3])
            + (1.018575e-02) * (q[i4] * q[i4]) + (1.129247e-04) * (q[i5] * q[i5]) + (-1.443923e-03) * (q[i6] * q[i6]) + (4.180654e-04) * ((2) * q[i0] * q[i7])
            + (4.344633e-03) * ((2) * q[i1] * q[i7]) + (8.489854e-04) * ((2) * q[i2] * q[i7]) + (-3.771542e-04) * ((2) * q[i3] * q[i7])
            + (1.369226e-03) * ((2) * q[i4] * q[i7]) + (-5.615365e-04) * ((2) * q[i5] * q[i7]) + (-1.441506e-03) * ((2) * q[i6] * q[i7])
            + (-2.043729e-03) * ((3) * q[i7] * q[i7]) + (1.520306e-03) * ((2) * q[i7] * q[i8]) + (-3.131878e-04) * ((2) * q[i7] * q[i9])
            + (-5.660781e-04) * ((2) * q[i7] * q[i10]) + (2.586729e-04) * ((2) * q[i7] * q[i11]) + (5.256333e-04) * ((2) * q[i7] * q[i12])
            + (-2.316011e-04) * ((2) * q[i7] * q[i15]) + (-5.468400e-04) * ((2) * q[i7] * q[i16]) + (8.015671e-05) * ((2) * q[i7] * q[i19])
            + (-4.334767e-04) * ((2) * q[i7] * q[i20]) + (-2.268698e-04) * ((2) * q[i7] * q[i21]) + (1.228869e-04) * ((2) * q[i7] * q[i22])
            + (2.041062e-03) * (q[i8] * q[i8]) + (3.724591e-04) * (q[i9] * q[i9]) + (-2.398467e-03) * (q[i10] * q[i10]) + (-1.671712e-05) * (q[i11] * q[i11])
            + (-4.457106e-04) * (q[i12] * q[i12]) + (7.962845e-04) * (q[i15] * q[i15]) + (-3.535705e-05) * (q[i16] * q[i16])
            + (1.477554e-04) * (q[i19] * q[i19]) + (-1.466157e-04) * (q[i20] * q[i20]) + (4.622909e-04) * (q[i21] * q[i21]) + (9.348077e-05) * (q[i22] * q[i22])
            + (6.056071e-03) * (q[i0] * q[i1]) + (7.894988e-04) * (q[i0] * q[i2]) + (3.259265e-03) * (q[i0] * q[i3]) + (3.139579e-03) * (q[i0] * q[i4])
            + (-9.150038e-04) * (q[i0] * q[i5]) + (1.025195e-02) * (q[i0] * q[i6]) + (-5.919886e-04) * (q[i0] * q[i8]) + (1.357902e-03) * (q[i0] * q[i9])
            + (-2.642911e-05) * (q[i0] * q[i10]) + (4.746056e-05) * (q[i0] * q[i11]) + (-7.667034e-05) * (q[i0] * q[i12]) + (4.276688e-04) * (q[i0] * q[i15])
            + (-3.514662e-05) * (q[i0] * q[i16]) + (3.153959e-04) * (q[i0] * q[i19]) + (-1.302443e-04) * (q[i0] * q[i20]) + (-1.596513e-05) * (q[i0] * q[i21])
            + (-4.932519e-04) * (q[i0] * q[i22]) + (-1.163635e-02) * (q[i1] * q[i2]) + (-8.651054e-04) * (q[i1] * q[i3]) + (-1.622585e-02) * (q[i1] * q[i4])
            + (4.571576e-03) * (q[i1] * q[i5]) + (-1.024657e-02) * (q[i1] * q[i6]) + (-3.389789e-03) * (q[i1] * q[i8]) + (-2.176429e-03) * (q[i1] * q[i9])
            + (1.605034e-03) * (q[i1] * q[i10]) + (-1.080001e-03) * (q[i1] * q[i11]) + (-1.693575e-03) * (q[i1] * q[i12]) + (-1.361213e-03) * (q[i1] * q[i15])
            + (4.450985e-04) * (q[i1] * q[i16]) + (-9.905888e-04) * (q[i1] * q[i19]) + (1.288409e-03) * (q[i1] * q[i20]) + (1.105648e-05) * (q[i1] * q[i21])
            + (-1.934229e-04) * (q[i1] * q[i22]) + (1.358948e-03) * (q[i2] * q[i3]) + (-3.675611e-03) * (q[i2] * q[i4]) + (-2.212931e-03) * (q[i2] * q[i5])
            + (-1.284721e-05) * (q[i2] * q[i6]) + (-8.968141e-03) * (q[i2] * q[i8]) + (2.865255e-04) * (q[i2] * q[i9]) + (-4.609410e-04) * (q[i2] * q[i10])
            + (2.363002e-04) * (q[i2] * q[i11]) + (-1.152232e-03) * (q[i2] * q[i12]) + (-6.230010e-04) * (q[i2] * q[i15]) + (6.615263e-04) * (q[i2] * q[i16])
            + (-9.679730e-05) * (q[i2] * q[i19]) + (7.850361e-04) * (q[i2] * q[i20]) + (8.112065e-05) * (q[i2] * q[i21]) + (2.716826e-05) * (q[i2] * q[i22])
            + (-4.095398e-03) * (q[i3] * q[i4]) + (-3.410167e-04) * (q[i3] * q[i5]) + (1.879310e-03) * (q[i3] * q[i6]) + (2.840954e-04) * (q[i3] * q[i8])
            + (-1.541094e-03) * (q[i3] * q[i9]) + (-1.745114e-03) * (q[i3] * q[i10]) + (-3.231969e-06) * (q[i3] * q[i11]) + (-4.333243e-04) * (q[i3] * q[i12])
            + (9.098021e-04) * (q[i3] * q[i15]) + (8.037877e-04) * (q[i3] * q[i16]) + (-8.243394e-04) * (q[i3] * q[i19]) + (3.347674e-04) * (q[i3] * q[i20])
            + (-5.588223e-04) * (q[i3] * q[i21]) + (4.399826e-04) * (q[i3] * q[i22]) + (-5.851652e-03) * (q[i4] * q[i5]) + (-1.896358e-03) * (q[i4] * q[i6])
            + (-8.515030e-04) * (q[i4] * q[i8]) + (3.995365e-04) * (q[i4] * q[i9]) + (-6.864024e-04) * (q[i4] * q[i10]) + (-1.038503e-03) * (q[i4] * q[i11])
            + (-4.192667e-04) * (q[i4] * q[i12]) + (2.146712e-05) * (q[i4] * q[i15]) + (9.376283e-04) * (q[i4] * q[i16]) + (-5.666982e-04) * (q[i4] * q[i19])
            + (-5.485476e-05) * (q[i4] * q[i20]) + (-4.266081e-04) * (q[i4] * q[i21]) + (-2.061593e-05) * (q[i4] * q[i22]) + (5.752728e-06) * (q[i5] * q[i6])
            + (-1.156110e-03) * (q[i5] * q[i8]) + (1.387223e-03) * (q[i5] * q[i9]) + (-2.220273e-03) * (q[i5] * q[i10]) + (-1.002169e-03) * (q[i5] * q[i11])
            + (-5.294600e-04) * (q[i5] * q[i12]) + (-2.394354e-04) * (q[i5] * q[i15]) + (-1.548307e-03) * (q[i5] * q[i16]) + (3.610782e-04) * (q[i5] * q[i19])
            + (-1.750359e-04) * (q[i5] * q[i20]) + (7.739277e-05) * (q[i5] * q[i21]) + (-4.801370e-05) * (q[i5] * q[i22]) + (-2.277266e-03) * (q[i6] * q[i8])
            + (3.240125e-04) * (q[i6] * q[i9]) + (3.225811e-04) * (q[i6] * q[i10]) + (-3.633376e-04) * (q[i6] * q[i11]) + (3.594762e-04) * (q[i6] * q[i12])
            + (5.321298e-04) * (q[i6] * q[i15]) + (5.348429e-04) * (q[i6] * q[i16]) + (3.668577e-04) * (q[i6] * q[i19]) + (3.663251e-04) * (q[i6] * q[i20])
            + (7.693029e-05) * (q[i6] * q[i21]) + (-8.031516e-05) * (q[i6] * q[i22]) + (-8.894445e-04) * (q[i8] * q[i9]) + (-6.230244e-04) * (q[i8] * q[i10])
            + (8.977087e-05) * (q[i8] * q[i11]) + (5.125980e-04) * (q[i8] * q[i12]) + (-3.710050e-04) * (q[i8] * q[i15]) + (-3.285802e-04) * (q[i8] * q[i16])
            + (-1.816022e-04) * (q[i8] * q[i19]) + (-1.430783e-04) * (q[i8] * q[i20]) + (-3.633953e-05) * (q[i8] * q[i21]) + (5.422506e-04) * (q[i8] * q[i22])
            + (-6.463066e-04) * (q[i9] * q[i10]) + (8.946800e-06) * (q[i9] * q[i11]) + (1.347187e-04) * (q[i9] * q[i12]) + (2.835824e-04) * (q[i9] * q[i15])
            + (-5.507066e-05) * (q[i9] * q[i16]) + (5.549223e-05) * (q[i9] * q[i19]) + (-1.455974e-04) * (q[i9] * q[i20]) + (9.839484e-05) * (q[i9] * q[i21])
            + (2.637625e-04) * (q[i9] * q[i22]) + (6.824602e-05) * (q[i10] * q[i11]) + (1.358580e-04) * (q[i10] * q[i12]) + (-4.023069e-04) * (q[i10] * q[i15])
            + (-2.686026e-04) * (q[i10] * q[i16]) + (-6.344870e-05) * (q[i10] * q[i19]) + (1.039055e-04) * (q[i10] * q[i20])
            + (2.147215e-04) * (q[i10] * q[i21]) + (1.668709e-04) * (q[i10] * q[i22]) + (-2.844112e-04) * (q[i11] * q[i12])
            + (-5.275637e-05) * (q[i11] * q[i15]) + (8.334291e-05) * (q[i11] * q[i16]) + (-3.031969e-04) * (q[i11] * q[i19])
            + (1.806968e-04) * (q[i11] * q[i20]) + (-3.003335e-04) * (q[i11] * q[i21]) + (-3.141856e-05) * (q[i11] * q[i22])
            + (-5.183897e-05) * (q[i12] * q[i15]) + (1.050108e-04) * (q[i12] * q[i16]) + (-5.069996e-05) * (q[i12] * q[i19])
            + (2.194537e-04) * (q[i12] * q[i20]) + (2.042993e-04) * (q[i12] * q[i21]) + (-1.771800e-04) * (q[i12] * q[i22])
            + (-2.721343e-05) * (q[i15] * q[i16]) + (-1.581387e-04) * (q[i15] * q[i19]) + (-7.403503e-05) * (q[i15] * q[i20])
            + (4.517717e-04) * (q[i15] * q[i21]) + (2.796963e-04) * (q[i15] * q[i22]) + (9.455737e-05) * (q[i16] * q[i19]) + (-2.170218e-04) * (q[i16] * q[i20])
            + (4.784558e-05) * (q[i16] * q[i21]) + (-3.036729e-04) * (q[i16] * q[i22]) + (-4.366264e-05) * (q[i19] * q[i20])
            + (2.590795e-04) * (q[i19] * q[i21]) + (-4.059630e-05) * (q[i19] * q[i22]) + (-2.406731e-05) * (q[i20] * q[i21])
            + (-2.534018e-04) * (q[i20] * q[i22]) + (-1.582395e-05) * (q[i21] * q[i22]);
      JQ[2][i8] = (1.106335e-01) * (1) + (1.743072e-03) * ((2) * q[i8]) + (-8.777736e-04) * (q[i0]) + (8.841392e-04) * (q[i1]) + (-2.594993e-05) * (q[i2])
            + (1.536486e-03) * (q[i3]) + (-1.477931e-03) * (q[i4]) + (6.977615e-06) * (q[i5]) + (-3.563051e-05) * (q[i6]) + (-5.774932e-05) * (q[i7])
            + (1.824121e-03) * (q[i9]) + (1.798970e-03) * (q[i10]) + (-9.114369e-04) * (q[i11]) + (7.988239e-04) * (q[i12]) + (-7.749465e-03) * (q[i15])
            + (-7.820070e-03) * (q[i16]) + (-3.488025e-04) * (q[i19]) + (-3.333278e-04) * (q[i20]) + (-1.221320e-03) * (q[i21]) + (1.217945e-03) * (q[i22])
            + (2.492421e-04) * (q[i0] * q[i0]) + (2.526131e-04) * (q[i1] * q[i1]) + (-3.049899e-02) * (q[i2] * q[i2]) + (4.890949e-03) * (q[i3] * q[i3])
            + (4.897763e-03) * (q[i4] * q[i4]) + (3.681054e-03) * (q[i5] * q[i5]) + (1.496093e-03) * (q[i6] * q[i6]) + (1.520306e-03) * (q[i7] * q[i7])
            + (-1.026171e-03) * ((2) * q[i0] * q[i8]) + (1.033403e-03) * ((2) * q[i1] * q[i8]) + (1.672647e-05) * ((2) * q[i2] * q[i8])
            + (-3.937382e-04) * ((2) * q[i3] * q[i8]) + (3.832575e-04) * ((2) * q[i4] * q[i8]) + (5.006040e-06) * ((2) * q[i5] * q[i8])
            + (2.038677e-03) * ((2) * q[i6] * q[i8]) + (2.041062e-03) * ((2) * q[i7] * q[i8]) + (-3.380286e-03) * ((3) * q[i8] * q[i8])
            + (1.219217e-04) * ((2) * q[i8] * q[i9]) + (1.215441e-04) * ((2) * q[i8] * q[i10]) + (1.006271e-03) * ((2) * q[i8] * q[i11])
            + (-1.040167e-03) * ((2) * q[i8] * q[i12]) + (2.766317e-04) * ((2) * q[i8] * q[i15]) + (2.802137e-04) * ((2) * q[i8] * q[i16])
            + (1.835682e-04) * ((2) * q[i8] * q[i19]) + (1.768287e-04) * ((2) * q[i8] * q[i20]) + (1.380972e-05) * ((2) * q[i8] * q[i21])
            + (-1.355391e-05) * ((2) * q[i8] * q[i22]) + (9.753884e-04) * (q[i9] * q[i9]) + (9.696730e-04) * (q[i10] * q[i10])
            + (1.529249e-03) * (q[i11] * q[i11]) + (1.567306e-03) * (q[i12] * q[i12]) + (1.056451e-04) * (q[i15] * q[i15]) + (1.140662e-04) * (q[i16] * q[i16])
            + (-1.688085e-04) * (q[i19] * q[i19]) + (-1.628132e-04) * (q[i20] * q[i20]) + (-9.195516e-04) * (q[i21] * q[i21])
            + (-9.218395e-04) * (q[i22] * q[i22]) + (-1.745568e-04) * (q[i0] * q[i1]) + (-5.673798e-03) * (q[i0] * q[i2]) + (-1.176956e-03) * (q[i0] * q[i3])
            + (2.537469e-04) * (q[i0] * q[i4]) + (-1.960694e-05) * (q[i0] * q[i5]) + (3.391187e-03) * (q[i0] * q[i6]) + (-5.919886e-04) * (q[i0] * q[i7])
            + (1.484234e-03) * (q[i0] * q[i9]) + (-1.658021e-04) * (q[i0] * q[i10]) + (-4.435071e-04) * (q[i0] * q[i11]) + (-2.355451e-04) * (q[i0] * q[i12])
            + (-6.521984e-05) * (q[i0] * q[i15]) + (2.012094e-04) * (q[i0] * q[i16]) + (-2.381760e-04) * (q[i0] * q[i19]) + (1.675113e-04) * (q[i0] * q[i20])
            + (3.856355e-04) * (q[i0] * q[i21]) + (5.524433e-05) * (q[i0] * q[i22]) + (-5.671491e-03) * (q[i1] * q[i2]) + (2.425740e-04) * (q[i1] * q[i3])
            + (-1.161245e-03) * (q[i1] * q[i4]) + (8.817743e-06) * (q[i1] * q[i5]) + (5.875240e-04) * (q[i1] * q[i6]) + (-3.389789e-03) * (q[i1] * q[i7])
            + (1.679392e-04) * (q[i1] * q[i9]) + (-1.470540e-03) * (q[i1] * q[i10]) + (-2.058888e-04) * (q[i1] * q[i11]) + (-4.676848e-04) * (q[i1] * q[i12])
            + (-1.883926e-04) * (q[i1] * q[i15]) + (6.772655e-05) * (q[i1] * q[i16]) + (-1.747253e-04) * (q[i1] * q[i19]) + (2.498898e-04) * (q[i1] * q[i20])
            + (4.661306e-05) * (q[i1] * q[i21]) + (3.830143e-04) * (q[i1] * q[i22]) + (1.127051e-04) * (q[i2] * q[i3]) + (1.230089e-04) * (q[i2] * q[i4])
            + (-1.136204e-02) * (q[i2] * q[i5]) + (8.949380e-03) * (q[i2] * q[i6]) + (-8.968141e-03) * (q[i2] * q[i7]) + (2.838627e-03) * (q[i2] * q[i9])
            + (-2.816410e-03) * (q[i2] * q[i10]) + (4.622125e-03) * (q[i2] * q[i11]) + (4.647709e-03) * (q[i2] * q[i12]) + (3.739850e-04) * (q[i2] * q[i15])
            + (-4.028890e-04) * (q[i2] * q[i16]) + (-1.238277e-04) * (q[i2] * q[i19]) + (1.423855e-04) * (q[i2] * q[i20]) + (4.697497e-04) * (q[i2] * q[i21])
            + (4.659396e-04) * (q[i2] * q[i22]) + (-2.612025e-03) * (q[i3] * q[i4]) + (-5.676358e-03) * (q[i3] * q[i5]) + (8.413058e-04) * (q[i3] * q[i6])
            + (2.840954e-04) * (q[i3] * q[i7]) + (-9.853283e-04) * (q[i3] * q[i9]) + (9.389851e-04) * (q[i3] * q[i10]) + (8.302458e-04) * (q[i3] * q[i11])
            + (3.707437e-04) * (q[i3] * q[i12]) + (-1.874093e-03) * (q[i3] * q[i15]) + (1.586166e-03) * (q[i3] * q[i16]) + (-1.489504e-04) * (q[i3] * q[i19])
            + (-5.408535e-05) * (q[i3] * q[i20]) + (7.900402e-05) * (q[i3] * q[i21]) + (4.263331e-04) * (q[i3] * q[i22]) + (-5.683570e-03) * (q[i4] * q[i5])
            + (-2.832570e-04) * (q[i4] * q[i6]) + (-8.515030e-04) * (q[i4] * q[i7]) + (-9.450100e-04) * (q[i4] * q[i9]) + (9.708365e-04) * (q[i4] * q[i10])
            + (3.666690e-04) * (q[i4] * q[i11]) + (8.563020e-04) * (q[i4] * q[i12]) + (-1.573262e-03) * (q[i4] * q[i15]) + (1.892670e-03) * (q[i4] * q[i16])
            + (5.975865e-05) * (q[i4] * q[i19]) + (1.447929e-04) * (q[i4] * q[i20]) + (4.272613e-04) * (q[i4] * q[i21]) + (8.697111e-05) * (q[i4] * q[i22])
            + (1.156394e-03) * (q[i5] * q[i6]) + (-1.156110e-03) * (q[i5] * q[i7]) + (1.313991e-03) * (q[i5] * q[i9]) + (-1.302686e-03) * (q[i5] * q[i10])
            + (5.523023e-04) * (q[i5] * q[i11]) + (5.779302e-04) * (q[i5] * q[i12]) + (1.110366e-03) * (q[i5] * q[i15]) + (-1.141076e-03) * (q[i5] * q[i16])
            + (-4.558482e-04) * (q[i5] * q[i19]) + (4.501319e-04) * (q[i5] * q[i20]) + (1.165022e-04) * (q[i5] * q[i21]) + (1.116974e-04) * (q[i5] * q[i22])
            + (-2.277266e-03) * (q[i6] * q[i7]) + (-6.389555e-04) * (q[i6] * q[i9]) + (-8.771073e-04) * (q[i6] * q[i10]) + (-5.018576e-04) * (q[i6] * q[i11])
            + (-5.343117e-05) * (q[i6] * q[i12]) + (-3.231139e-04) * (q[i6] * q[i15]) + (-3.754556e-04) * (q[i6] * q[i16]) + (-1.490583e-04) * (q[i6] * q[i19])
            + (-1.835712e-04) * (q[i6] * q[i20]) + (-5.335340e-04) * (q[i6] * q[i21]) + (3.416559e-05) * (q[i6] * q[i22]) + (-8.894445e-04) * (q[i7] * q[i9])
            + (-6.230244e-04) * (q[i7] * q[i10]) + (8.977087e-05) * (q[i7] * q[i11]) + (5.125980e-04) * (q[i7] * q[i12]) + (-3.710050e-04) * (q[i7] * q[i15])
            + (-3.285802e-04) * (q[i7] * q[i16]) + (-1.816022e-04) * (q[i7] * q[i19]) + (-1.430783e-04) * (q[i7] * q[i20]) + (-3.633953e-05) * (q[i7] * q[i21])
            + (5.422506e-04) * (q[i7] * q[i22]) + (3.466968e-04) * (q[i9] * q[i10]) + (-5.878640e-04) * (q[i9] * q[i11]) + (-1.250041e-04) * (q[i9] * q[i12])
            + (-2.177621e-04) * (q[i9] * q[i15]) + (-5.823431e-04) * (q[i9] * q[i16]) + (4.886623e-06) * (q[i9] * q[i19]) + (6.850970e-05) * (q[i9] * q[i20])
            + (-1.462135e-04) * (q[i9] * q[i21]) + (-1.807617e-04) * (q[i9] * q[i22]) + (1.243089e-04) * (q[i10] * q[i11]) + (5.872017e-04) * (q[i10] * q[i12])
            + (-5.728400e-04) * (q[i10] * q[i15]) + (-2.213568e-04) * (q[i10] * q[i16]) + (7.077076e-05) * (q[i10] * q[i19])
            + (5.437111e-06) * (q[i10] * q[i20]) + (1.820278e-04) * (q[i10] * q[i21]) + (1.433013e-04) * (q[i10] * q[i22]) + (-2.186374e-04) * (q[i11] * q[i12])
            + (-6.202402e-04) * (q[i11] * q[i15]) + (-2.278058e-04) * (q[i11] * q[i16]) + (4.967597e-04) * (q[i11] * q[i19])
            + (1.391907e-04) * (q[i11] * q[i20]) + (7.869919e-06) * (q[i11] * q[i21]) + (-7.596144e-05) * (q[i11] * q[i22]) + (2.198356e-04) * (q[i12] * q[i15])
            + (6.076149e-04) * (q[i12] * q[i16]) + (-1.399631e-04) * (q[i12] * q[i19]) + (-4.876793e-04) * (q[i12] * q[i20])
            + (-7.232507e-05) * (q[i12] * q[i21]) + (1.357240e-06) * (q[i12] * q[i22]) + (-2.887256e-05) * (q[i15] * q[i16])
            + (2.942123e-04) * (q[i15] * q[i19]) + (1.343018e-05) * (q[i15] * q[i20]) + (-8.182625e-04) * (q[i15] * q[i21]) + (7.254698e-05) * (q[i15] * q[i22])
            + (1.246325e-05) * (q[i16] * q[i19]) + (2.942755e-04) * (q[i16] * q[i20]) + (-7.242451e-05) * (q[i16] * q[i21]) + (8.305395e-04) * (q[i16] * q[i22])
            + (-2.221943e-04) * (q[i19] * q[i20]) + (-5.539763e-04) * (q[i19] * q[i21]) + (-3.164284e-05) * (q[i19] * q[i22])
            + (3.068988e-05) * (q[i20] * q[i21]) + (5.498027e-04) * (q[i20] * q[i22]) + (-1.903642e-04) * (q[i21] * q[i22]);
      JQ[2][i9] = (5.753591e-02) * (1) + (-3.601163e-03) * ((2) * q[i9]) + (8.871379e-04) * (q[i0]) + (-8.937913e-04) * (q[i1]) + (2.799744e-03) * (q[i2])
            + (-3.155387e-03) * (q[i3]) + (-5.158458e-03) * (q[i4]) + (-4.796828e-03) * (q[i5]) + (-9.249520e-03) * (q[i6]) + (4.408551e-03) * (q[i7])
            + (1.824121e-03) * (q[i8]) + (3.150382e-03) * (q[i10]) + (5.850180e-05) * (q[i11]) + (1.022610e-03) * (q[i12]) + (7.203772e-04) * (q[i15])
            + (1.594974e-03) * (q[i16]) + (-1.511852e-04) * (q[i19]) + (-3.007639e-06) * (q[i20]) + (-1.003113e-04) * (q[i21]) + (5.577721e-04) * (q[i22])
            + (-3.616889e-03) * (q[i0] * q[i0]) + (1.274547e-04) * (q[i1] * q[i1]) + (-1.123049e-03) * (q[i2] * q[i2]) + (1.245209e-03) * (q[i3] * q[i3])
            + (-7.027872e-04) * (q[i4] * q[i4]) + (1.445221e-04) * (q[i5] * q[i5]) + (-5.717403e-04) * (q[i6] * q[i6]) + (-3.131878e-04) * (q[i7] * q[i7])
            + (1.219217e-04) * (q[i8] * q[i8]) + (-4.753033e-04) * ((2) * q[i0] * q[i9]) + (-1.276147e-04) * ((2) * q[i1] * q[i9])
            + (1.951560e-04) * ((2) * q[i2] * q[i9]) + (4.354672e-04) * ((2) * q[i3] * q[i9]) + (8.160768e-04) * ((2) * q[i4] * q[i9])
            + (8.902593e-04) * ((2) * q[i5] * q[i9]) + (-2.425500e-03) * ((2) * q[i6] * q[i9]) + (3.724591e-04) * ((2) * q[i7] * q[i9])
            + (9.753884e-04) * ((2) * q[i8] * q[i9]) + (-8.323603e-04) * ((3) * q[i9] * q[i9]) + (-2.172481e-04) * ((2) * q[i9] * q[i10])
            + (-2.931694e-05) * ((2) * q[i9] * q[i11]) + (5.262958e-05) * ((2) * q[i9] * q[i12]) + (-2.901246e-05) * ((2) * q[i9] * q[i15])
            + (-2.284468e-04) * ((2) * q[i9] * q[i16]) + (6.414339e-05) * ((2) * q[i9] * q[i19]) + (-2.279670e-06) * ((2) * q[i9] * q[i20])
            + (7.424689e-05) * ((2) * q[i9] * q[i21]) + (-1.788403e-04) * ((2) * q[i9] * q[i22]) + (-2.200224e-04) * (q[i10] * q[i10])
            + (-1.026045e-04) * (q[i11] * q[i11]) + (7.635329e-05) * (q[i12] * q[i12]) + (-1.752264e-04) * (q[i15] * q[i15])
            + (-6.107686e-05) * (q[i16] * q[i16]) + (1.884527e-05) * (q[i19] * q[i19]) + (-7.237348e-05) * (q[i20] * q[i20])
            + (4.402841e-05) * (q[i21] * q[i21]) + (5.344678e-05) * (q[i22] * q[i22]) + (9.218836e-04) * (q[i0] * q[i1]) + (-1.526473e-03) * (q[i0] * q[i2])
            + (1.243164e-02) * (q[i0] * q[i3]) + (-1.366809e-04) * (q[i0] * q[i4]) + (2.434291e-04) * (q[i0] * q[i5]) + (-1.594058e-03) * (q[i0] * q[i6])
            + (1.357902e-03) * (q[i0] * q[i7]) + (1.484234e-03) * (q[i0] * q[i8]) + (3.607942e-04) * (q[i0] * q[i10]) + (1.239044e-04) * (q[i0] * q[i11])
            + (-3.238291e-04) * (q[i0] * q[i12]) + (-2.437090e-04) * (q[i0] * q[i15]) + (3.510747e-04) * (q[i0] * q[i16]) + (-2.967470e-04) * (q[i0] * q[i19])
            + (3.917039e-04) * (q[i0] * q[i20]) + (2.764603e-05) * (q[i0] * q[i21]) + (-1.361217e-04) * (q[i0] * q[i22]) + (1.846815e-03) * (q[i1] * q[i2])
            + (-9.725343e-04) * (q[i1] * q[i3]) + (-3.814247e-03) * (q[i1] * q[i4]) + (1.174705e-03) * (q[i1] * q[i5]) + (2.119731e-05) * (q[i1] * q[i6])
            + (-2.176429e-03) * (q[i1] * q[i7]) + (1.679392e-04) * (q[i1] * q[i8]) + (-3.642427e-04) * (q[i1] * q[i10]) + (2.217855e-04) * (q[i1] * q[i11])
            + (5.530140e-04) * (q[i1] * q[i12]) + (2.875109e-04) * (q[i1] * q[i15]) + (-3.991676e-04) * (q[i1] * q[i16]) + (-1.422766e-04) * (q[i1] * q[i19])
            + (-2.535342e-04) * (q[i1] * q[i20]) + (3.084981e-05) * (q[i1] * q[i21]) + (1.378833e-04) * (q[i1] * q[i22]) + (2.158290e-03) * (q[i2] * q[i3])
            + (-1.264543e-03) * (q[i2] * q[i4]) + (6.123080e-03) * (q[i2] * q[i5]) + (4.654102e-04) * (q[i2] * q[i6]) + (2.865255e-04) * (q[i2] * q[i7])
            + (2.838627e-03) * (q[i2] * q[i8]) + (-1.552921e-06) * (q[i2] * q[i10]) + (6.676637e-05) * (q[i2] * q[i11]) + (9.903040e-05) * (q[i2] * q[i12])
            + (-1.966923e-04) * (q[i2] * q[i15]) + (-1.519043e-04) * (q[i2] * q[i16]) + (-5.151900e-04) * (q[i2] * q[i19]) + (5.072482e-05) * (q[i2] * q[i20])
            + (8.193177e-06) * (q[i2] * q[i21]) + (1.973894e-05) * (q[i2] * q[i22]) + (-1.389606e-03) * (q[i3] * q[i4]) + (-9.961027e-04) * (q[i3] * q[i5])
            + (7.022871e-04) * (q[i3] * q[i6]) + (-1.541094e-03) * (q[i3] * q[i7]) + (-9.853283e-04) * (q[i3] * q[i8]) + (-4.769213e-04) * (q[i3] * q[i10])
            + (3.395112e-04) * (q[i3] * q[i11]) + (3.349317e-04) * (q[i3] * q[i12]) + (-1.930683e-04) * (q[i3] * q[i15]) + (-1.237888e-04) * (q[i3] * q[i16])
            + (1.574528e-04) * (q[i3] * q[i19]) + (9.604014e-05) * (q[i3] * q[i20]) + (1.093087e-04) * (q[i3] * q[i21]) + (-4.633238e-05) * (q[i3] * q[i22])
            + (-6.389476e-04) * (q[i4] * q[i5]) + (1.757509e-03) * (q[i4] * q[i6]) + (3.995365e-04) * (q[i4] * q[i7]) + (-9.450100e-04) * (q[i4] * q[i8])
            + (4.768357e-04) * (q[i4] * q[i10]) + (8.569621e-05) * (q[i4] * q[i11]) + (-4.901619e-04) * (q[i4] * q[i12]) + (-2.303278e-04) * (q[i4] * q[i15])
            + (-3.853204e-05) * (q[i4] * q[i16]) + (-2.364868e-04) * (q[i4] * q[i19]) + (-6.932773e-05) * (q[i4] * q[i20]) + (-9.635595e-05) * (q[i4] * q[i21])
            + (-1.695343e-04) * (q[i4] * q[i22]) + (2.235833e-03) * (q[i5] * q[i6]) + (1.387223e-03) * (q[i5] * q[i7]) + (1.313991e-03) * (q[i5] * q[i8])
            + (7.806473e-07) * (q[i5] * q[i10]) + (-3.886678e-04) * (q[i5] * q[i11]) + (-3.133345e-04) * (q[i5] * q[i12]) + (7.187954e-04) * (q[i5] * q[i15])
            + (-1.300566e-04) * (q[i5] * q[i16]) + (2.208338e-04) * (q[i5] * q[i19]) + (-1.474719e-04) * (q[i5] * q[i20]) + (-5.418967e-04) * (q[i5] * q[i21])
            + (-2.203100e-04) * (q[i5] * q[i22]) + (3.240125e-04) * (q[i6] * q[i7]) + (-6.389555e-04) * (q[i6] * q[i8]) + (-6.427358e-04) * (q[i6] * q[i10])
            + (-1.433083e-04) * (q[i6] * q[i11]) + (-7.875434e-05) * (q[i6] * q[i12]) + (-2.687738e-04) * (q[i6] * q[i15]) + (-4.103087e-04) * (q[i6] * q[i16])
            + (1.042731e-04) * (q[i6] * q[i19]) + (-6.404165e-05) * (q[i6] * q[i20]) + (-1.674403e-04) * (q[i6] * q[i21]) + (-2.159308e-04) * (q[i6] * q[i22])
            + (-8.894445e-04) * (q[i7] * q[i8]) + (-6.463066e-04) * (q[i7] * q[i10]) + (8.946800e-06) * (q[i7] * q[i11]) + (1.347187e-04) * (q[i7] * q[i12])
            + (2.835824e-04) * (q[i7] * q[i15]) + (-5.507066e-05) * (q[i7] * q[i16]) + (5.549223e-05) * (q[i7] * q[i19]) + (-1.455974e-04) * (q[i7] * q[i20])
            + (9.839484e-05) * (q[i7] * q[i21]) + (2.637625e-04) * (q[i7] * q[i22]) + (3.466968e-04) * (q[i8] * q[i10]) + (-5.878640e-04) * (q[i8] * q[i11])
            + (-1.250041e-04) * (q[i8] * q[i12]) + (-2.177621e-04) * (q[i8] * q[i15]) + (-5.823431e-04) * (q[i8] * q[i16]) + (4.886623e-06) * (q[i8] * q[i19])
            + (6.850970e-05) * (q[i8] * q[i20]) + (-1.462135e-04) * (q[i8] * q[i21]) + (-1.807617e-04) * (q[i8] * q[i22]) + (4.886757e-05) * (q[i10] * q[i11])
            + (-4.815101e-05) * (q[i10] * q[i12]) + (6.594137e-05) * (q[i10] * q[i15]) + (6.633137e-05) * (q[i10] * q[i16])
            + (-3.746738e-05) * (q[i10] * q[i19]) + (-3.693607e-05) * (q[i10] * q[i20]) + (-6.211473e-05) * (q[i10] * q[i21])
            + (6.165970e-05) * (q[i10] * q[i22]) + (1.294106e-04) * (q[i11] * q[i12]) + (3.956050e-06) * (q[i11] * q[i15]) + (1.615459e-04) * (q[i11] * q[i16])
            + (-2.660218e-04) * (q[i11] * q[i19]) + (2.055716e-04) * (q[i11] * q[i20]) + (-5.655255e-05) * (q[i11] * q[i21])
            + (1.690360e-04) * (q[i11] * q[i22]) + (1.577575e-04) * (q[i12] * q[i15]) + (5.806691e-04) * (q[i12] * q[i16]) + (-6.097590e-05) * (q[i12] * q[i19])
            + (5.071179e-05) * (q[i12] * q[i20]) + (-7.381588e-05) * (q[i12] * q[i21]) + (-1.232102e-04) * (q[i12] * q[i22])
            + (-4.069744e-05) * (q[i15] * q[i16]) + (-5.244398e-05) * (q[i15] * q[i19]) + (8.511117e-05) * (q[i15] * q[i20])
            + (8.364882e-05) * (q[i15] * q[i21]) + (1.024389e-04) * (q[i15] * q[i22]) + (3.970330e-05) * (q[i16] * q[i19]) + (-2.986617e-05) * (q[i16] * q[i20])
            + (-4.268491e-05) * (q[i16] * q[i21]) + (-1.365421e-04) * (q[i16] * q[i22]) + (3.019187e-05) * (q[i19] * q[i20])
            + (6.170221e-06) * (q[i19] * q[i21]) + (7.599913e-05) * (q[i19] * q[i22]) + (-5.544617e-05) * (q[i20] * q[i21])
            + (-1.504658e-04) * (q[i20] * q[i22]) + (-1.829503e-05) * (q[i21] * q[i22]);
      JQ[2][i10] = (5.691600e-02) * (1) + (-3.588062e-03) * ((2) * q[i10]) + (8.983805e-04) * (q[i0]) + (-8.990074e-04) * (q[i1]) + (-2.752438e-03) * (q[i2])
            + (5.113675e-03) * (q[i3]) + (3.126489e-03) * (q[i4]) + (4.750550e-03) * (q[i5]) + (4.391855e-03) * (q[i6]) + (-9.169212e-03) * (q[i7])
            + (1.798970e-03) * (q[i8]) + (3.150382e-03) * (q[i9]) + (-9.841551e-04) * (q[i11]) + (-2.625510e-05) * (q[i12]) + (1.558823e-03) * (q[i15])
            + (7.085550e-04) * (q[i16]) + (-1.109468e-05) * (q[i19]) + (-1.568486e-04) * (q[i20]) + (-5.512975e-04) * (q[i21]) + (9.916410e-05) * (q[i22])
            + (1.348726e-04) * (q[i0] * q[i0]) + (-3.584321e-03) * (q[i1] * q[i1]) + (-1.118759e-03) * (q[i2] * q[i2]) + (-6.931558e-04) * (q[i3] * q[i3])
            + (1.253167e-03) * (q[i4] * q[i4]) + (1.495364e-04) * (q[i5] * q[i5]) + (-3.134357e-04) * (q[i6] * q[i6]) + (-5.660781e-04) * (q[i7] * q[i7])
            + (1.215441e-04) * (q[i8] * q[i8]) + (-2.172481e-04) * (q[i9] * q[i9]) + (1.221015e-04) * ((2) * q[i0] * q[i10])
            + (4.739604e-04) * ((2) * q[i1] * q[i10]) + (-1.928276e-04) * ((2) * q[i2] * q[i10]) + (-8.072043e-04) * ((2) * q[i3] * q[i10])
            + (-4.293332e-04) * ((2) * q[i4] * q[i10]) + (-8.848187e-04) * ((2) * q[i5] * q[i10]) + (3.637730e-04) * ((2) * q[i6] * q[i10])
            + (-2.398467e-03) * ((2) * q[i7] * q[i10]) + (9.696730e-04) * ((2) * q[i8] * q[i10]) + (-2.200224e-04) * ((2) * q[i9] * q[i10])
            + (-8.206650e-04) * ((3) * q[i10] * q[i10]) + (-5.578485e-05) * ((2) * q[i10] * q[i11]) + (2.749652e-05) * ((2) * q[i10] * q[i12])
            + (-2.242911e-04) * ((2) * q[i10] * q[i15]) + (-2.973345e-05) * ((2) * q[i10] * q[i16]) + (-1.276539e-06) * ((2) * q[i10] * q[i19])
            + (6.250395e-05) * ((2) * q[i10] * q[i20]) + (1.764229e-04) * ((2) * q[i10] * q[i21]) + (-7.383322e-05) * ((2) * q[i10] * q[i22])
            + (6.209638e-05) * (q[i11] * q[i11]) + (-1.057925e-04) * (q[i12] * q[i12]) + (-5.884776e-05) * (q[i15] * q[i15])
            + (-1.768509e-04) * (q[i16] * q[i16]) + (-7.063182e-05) * (q[i19] * q[i19]) + (1.950431e-05) * (q[i20] * q[i20])
            + (5.434041e-05) * (q[i21] * q[i21]) + (4.411843e-05) * (q[i22] * q[i22]) + (9.146743e-04) * (q[i0] * q[i1]) + (1.848176e-03) * (q[i0] * q[i2])
            + (-3.773849e-03) * (q[i0] * q[i3]) + (-9.703830e-04) * (q[i0] * q[i4]) + (1.174825e-03) * (q[i0] * q[i5]) + (2.157548e-03) * (q[i0] * q[i6])
            + (-2.642911e-05) * (q[i0] * q[i7]) + (-1.658021e-04) * (q[i0] * q[i8]) + (3.607942e-04) * (q[i0] * q[i9]) + (5.471964e-04) * (q[i0] * q[i11])
            + (2.231798e-04) * (q[i0] * q[i12]) + (3.978931e-04) * (q[i0] * q[i15]) + (-2.885660e-04) * (q[i0] * q[i16]) + (2.560510e-04) * (q[i0] * q[i19])
            + (1.447837e-04) * (q[i0] * q[i20]) + (1.370656e-04) * (q[i0] * q[i21]) + (3.005937e-05) * (q[i0] * q[i22]) + (-1.523589e-03) * (q[i1] * q[i2])
            + (-1.391450e-04) * (q[i1] * q[i3]) + (1.230341e-02) * (q[i1] * q[i4]) + (2.354629e-04) * (q[i1] * q[i5]) + (-1.360581e-03) * (q[i1] * q[i6])
            + (1.605034e-03) * (q[i1] * q[i7]) + (-1.470540e-03) * (q[i1] * q[i8]) + (-3.642427e-04) * (q[i1] * q[i9]) + (-3.287098e-04) * (q[i1] * q[i11])
            + (1.165157e-04) * (q[i1] * q[i12]) + (-3.468384e-04) * (q[i1] * q[i15]) + (2.473510e-04) * (q[i1] * q[i16]) + (-3.908049e-04) * (q[i1] * q[i19])
            + (2.920327e-04) * (q[i1] * q[i20]) + (-1.361306e-04) * (q[i1] * q[i21]) + (3.006953e-05) * (q[i1] * q[i22]) + (-1.252840e-03) * (q[i2] * q[i3])
            + (2.128636e-03) * (q[i2] * q[i4]) + (6.063427e-03) * (q[i2] * q[i5]) + (-2.886947e-04) * (q[i2] * q[i6]) + (-4.609410e-04) * (q[i2] * q[i7])
            + (-2.816410e-03) * (q[i2] * q[i8]) + (-1.552921e-06) * (q[i2] * q[i9]) + (8.644235e-05) * (q[i2] * q[i11]) + (5.846824e-05) * (q[i2] * q[i12])
            + (1.523461e-04) * (q[i2] * q[i15]) + (1.977923e-04) * (q[i2] * q[i16]) + (-5.029466e-05) * (q[i2] * q[i19]) + (5.113233e-04) * (q[i2] * q[i20])
            + (1.997862e-05) * (q[i2] * q[i21]) + (9.079683e-06) * (q[i2] * q[i22]) + (-1.383767e-03) * (q[i3] * q[i4]) + (-6.296811e-04) * (q[i3] * q[i5])
            + (-4.007765e-04) * (q[i3] * q[i6]) + (-1.745114e-03) * (q[i3] * q[i7]) + (9.389851e-04) * (q[i3] * q[i8]) + (-4.769213e-04) * (q[i3] * q[i9])
            + (-4.832726e-04) * (q[i3] * q[i11]) + (9.182299e-05) * (q[i3] * q[i12]) + (3.303860e-05) * (q[i3] * q[i15]) + (2.333993e-04) * (q[i3] * q[i16])
            + (6.864690e-05) * (q[i3] * q[i19]) + (2.429382e-04) * (q[i3] * q[i20]) + (-1.674627e-04) * (q[i3] * q[i21]) + (-9.695850e-05) * (q[i3] * q[i22])
            + (-9.861556e-04) * (q[i4] * q[i5]) + (1.530174e-03) * (q[i4] * q[i6]) + (-6.864024e-04) * (q[i4] * q[i7]) + (9.708365e-04) * (q[i4] * q[i8])
            + (4.768357e-04) * (q[i4] * q[i9]) + (3.251140e-04) * (q[i4] * q[i11]) + (3.369470e-04) * (q[i4] * q[i12]) + (1.181967e-04) * (q[i4] * q[i15])
            + (1.877344e-04) * (q[i4] * q[i16]) + (-9.532546e-05) * (q[i4] * q[i19]) + (-1.563696e-04) * (q[i4] * q[i20]) + (-4.528867e-05) * (q[i4] * q[i21])
            + (1.107078e-04) * (q[i4] * q[i22]) + (-1.380465e-03) * (q[i5] * q[i6]) + (-2.220273e-03) * (q[i5] * q[i7]) + (-1.302686e-03) * (q[i5] * q[i8])
            + (7.806473e-07) * (q[i5] * q[i9]) + (-3.202338e-04) * (q[i5] * q[i11]) + (-3.828427e-04) * (q[i5] * q[i12]) + (1.317935e-04) * (q[i5] * q[i15])
            + (-7.160931e-04) * (q[i5] * q[i16]) + (1.491292e-04) * (q[i5] * q[i19]) + (-2.241152e-04) * (q[i5] * q[i20]) + (-2.214388e-04) * (q[i5] * q[i21])
            + (-5.331750e-04) * (q[i5] * q[i22]) + (3.225811e-04) * (q[i6] * q[i7]) + (-8.771073e-04) * (q[i6] * q[i8]) + (-6.427358e-04) * (q[i6] * q[i9])
            + (-1.313475e-04) * (q[i6] * q[i11]) + (-1.012554e-05) * (q[i6] * q[i12]) + (-5.974720e-05) * (q[i6] * q[i15]) + (2.814061e-04) * (q[i6] * q[i16])
            + (-1.477668e-04) * (q[i6] * q[i19]) + (5.322371e-05) * (q[i6] * q[i20]) + (-2.659831e-04) * (q[i6] * q[i21]) + (-9.791022e-05) * (q[i6] * q[i22])
            + (-6.230244e-04) * (q[i7] * q[i8]) + (-6.463066e-04) * (q[i7] * q[i9]) + (6.824602e-05) * (q[i7] * q[i11]) + (1.358580e-04) * (q[i7] * q[i12])
            + (-4.023069e-04) * (q[i7] * q[i15]) + (-2.686026e-04) * (q[i7] * q[i16]) + (-6.344870e-05) * (q[i7] * q[i19]) + (1.039055e-04) * (q[i7] * q[i20])
            + (2.147215e-04) * (q[i7] * q[i21]) + (1.668709e-04) * (q[i7] * q[i22]) + (3.466968e-04) * (q[i8] * q[i9]) + (1.243089e-04) * (q[i8] * q[i11])
            + (5.872017e-04) * (q[i8] * q[i12]) + (-5.728400e-04) * (q[i8] * q[i15]) + (-2.213568e-04) * (q[i8] * q[i16]) + (7.077076e-05) * (q[i8] * q[i19])
            + (5.437111e-06) * (q[i8] * q[i20]) + (1.820278e-04) * (q[i8] * q[i21]) + (1.433013e-04) * (q[i8] * q[i22]) + (4.886757e-05) * (q[i9] * q[i11])
            + (-4.815101e-05) * (q[i9] * q[i12]) + (6.594137e-05) * (q[i9] * q[i15]) + (6.633137e-05) * (q[i9] * q[i16]) + (-3.746738e-05) * (q[i9] * q[i19])
            + (-3.693607e-05) * (q[i9] * q[i20]) + (-6.211473e-05) * (q[i9] * q[i21]) + (6.165970e-05) * (q[i9] * q[i22]) + (1.306873e-04) * (q[i11] * q[i12])
            + (-5.741643e-04) * (q[i11] * q[i15]) + (-1.549752e-04) * (q[i11] * q[i16]) + (-5.359431e-05) * (q[i11] * q[i19])
            + (6.115402e-05) * (q[i11] * q[i20]) + (-1.222902e-04) * (q[i11] * q[i21]) + (-7.195368e-05) * (q[i11] * q[i22])
            + (-1.558511e-04) * (q[i12] * q[i15]) + (-2.731546e-06) * (q[i12] * q[i16]) + (-2.062423e-04) * (q[i12] * q[i19])
            + (2.616168e-04) * (q[i12] * q[i20]) + (1.687286e-04) * (q[i12] * q[i21]) + (-5.460698e-05) * (q[i12] * q[i22])
            + (-4.238001e-05) * (q[i15] * q[i16]) + (-3.179155e-05) * (q[i15] * q[i19]) + (3.929254e-05) * (q[i15] * q[i20])
            + (1.331861e-04) * (q[i15] * q[i21]) + (4.119188e-05) * (q[i15] * q[i22]) + (8.554462e-05) * (q[i16] * q[i19]) + (-5.567756e-05) * (q[i16] * q[i20])
            + (-1.035764e-04) * (q[i16] * q[i21]) + (-8.357313e-05) * (q[i16] * q[i22]) + (3.192201e-05) * (q[i19] * q[i20])
            + (1.496858e-04) * (q[i19] * q[i21]) + (5.384536e-05) * (q[i19] * q[i22]) + (-7.447196e-05) * (q[i20] * q[i21])
            + (-4.256861e-06) * (q[i20] * q[i22]) + (-1.805391e-05) * (q[i21] * q[i22]);
      JQ[2][i11] = (-6.730134e-03) * (1) + (-3.357970e-03) * ((2) * q[i11]) + (-1.743768e-03) * (q[i0]) + (-4.216920e-04) * (q[i1]) + (-2.770685e-03) * (q[i2])
            + (-3.877735e-03) * (q[i3]) + (-5.590388e-03) * (q[i4]) + (-4.371294e-03) * (q[i5]) + (-7.010548e-04) * (q[i6]) + (-8.042559e-04) * (q[i7])
            + (-9.114369e-04) * (q[i8]) + (5.850180e-05) * (q[i9]) + (-9.841551e-04) * (q[i10]) + (-3.692658e-04) * (q[i12]) + (1.224232e-03) * (q[i15])
            + (8.886032e-04) * (q[i16]) + (-1.088796e-03) * (q[i19]) + (-4.647446e-04) * (q[i20]) + (-1.516041e-03) * (q[i21]) + (-1.487105e-04) * (q[i22])
            + (2.516890e-04) * (q[i0] * q[i0]) + (-1.337575e-04) * (q[i1] * q[i1]) + (1.546000e-03) * (q[i2] * q[i2]) + (-6.311628e-04) * (q[i3] * q[i3])
            + (-3.640856e-04) * (q[i4] * q[i4]) + (2.935778e-04) * (q[i5] * q[i5]) + (-5.248814e-04) * (q[i6] * q[i6]) + (2.586729e-04) * (q[i7] * q[i7])
            + (1.006271e-03) * (q[i8] * q[i8]) + (-2.931694e-05) * (q[i9] * q[i9]) + (-5.578485e-05) * (q[i10] * q[i10])
            + (2.362361e-04) * ((2) * q[i0] * q[i11]) + (-1.418573e-04) * ((2) * q[i1] * q[i11]) + (2.309479e-03) * ((2) * q[i2] * q[i11])
            + (3.976655e-05) * ((2) * q[i3] * q[i11]) + (-3.286597e-04) * ((2) * q[i4] * q[i11]) + (1.262623e-03) * ((2) * q[i5] * q[i11])
            + (-4.280665e-04) * ((2) * q[i6] * q[i11]) + (-1.671712e-05) * ((2) * q[i7] * q[i11]) + (1.529249e-03) * ((2) * q[i8] * q[i11])
            + (-1.026045e-04) * ((2) * q[i9] * q[i11]) + (6.209638e-05) * ((2) * q[i10] * q[i11]) + (1.094118e-03) * ((3) * q[i11] * q[i11])
            + (-8.548084e-05) * ((2) * q[i11] * q[i12]) + (-1.956376e-03) * ((2) * q[i11] * q[i15]) + (-1.667675e-04) * ((2) * q[i11] * q[i16])
            + (4.941124e-04) * ((2) * q[i11] * q[i19]) + (-2.970354e-05) * ((2) * q[i11] * q[i20]) + (-4.377671e-05) * ((2) * q[i11] * q[i21])
            + (-4.816737e-05) * ((2) * q[i11] * q[i22]) + (9.131174e-05) * (q[i12] * q[i12]) + (2.137069e-03) * (q[i15] * q[i15])
            + (-2.123403e-05) * (q[i16] * q[i16]) + (-4.597726e-04) * (q[i19] * q[i19]) + (-6.177688e-05) * (q[i20] * q[i20])
            + (-4.426272e-04) * (q[i21] * q[i21]) + (6.576752e-05) * (q[i22] * q[i22]) + (-2.862139e-04) * (q[i0] * q[i1]) + (1.045153e-03) * (q[i0] * q[i2])
            + (3.946048e-04) * (q[i0] * q[i3]) + (-2.479629e-05) * (q[i0] * q[i4]) + (-1.086987e-03) * (q[i0] * q[i5]) + (-1.685007e-03) * (q[i0] * q[i6])
            + (4.746056e-05) * (q[i0] * q[i7]) + (-4.435071e-04) * (q[i0] * q[i8]) + (1.239044e-04) * (q[i0] * q[i9]) + (5.471964e-04) * (q[i0] * q[i10])
            + (1.395562e-04) * (q[i0] * q[i12]) + (1.694093e-04) * (q[i0] * q[i15]) + (-2.069843e-04) * (q[i0] * q[i16]) + (6.329313e-04) * (q[i0] * q[i19])
            + (-2.262147e-04) * (q[i0] * q[i20]) + (1.627175e-05) * (q[i0] * q[i21]) + (8.850492e-05) * (q[i0] * q[i22]) + (-6.976181e-05) * (q[i1] * q[i2])
            + (2.653701e-04) * (q[i1] * q[i3]) + (9.170136e-04) * (q[i1] * q[i4]) + (-1.123989e-03) * (q[i1] * q[i5]) + (-7.651621e-05) * (q[i1] * q[i6])
            + (-1.080001e-03) * (q[i1] * q[i7]) + (-2.058888e-04) * (q[i1] * q[i8]) + (2.217855e-04) * (q[i1] * q[i9]) + (-3.287098e-04) * (q[i1] * q[i10])
            + (-1.408358e-04) * (q[i1] * q[i12]) + (-1.825324e-05) * (q[i1] * q[i15]) + (-1.518464e-04) * (q[i1] * q[i16]) + (-1.618743e-04) * (q[i1] * q[i19])
            + (1.495978e-04) * (q[i1] * q[i20]) + (1.855161e-04) * (q[i1] * q[i21]) + (-7.675297e-05) * (q[i1] * q[i22]) + (-2.804176e-04) * (q[i2] * q[i3])
            + (-4.179273e-04) * (q[i2] * q[i4]) + (1.802504e-03) * (q[i2] * q[i5]) + (-1.117621e-03) * (q[i2] * q[i6]) + (2.363002e-04) * (q[i2] * q[i7])
            + (4.622125e-03) * (q[i2] * q[i8]) + (6.676637e-05) * (q[i2] * q[i9]) + (8.644235e-05) * (q[i2] * q[i10]) + (5.027884e-06) * (q[i2] * q[i12])
            + (-2.437054e-03) * (q[i2] * q[i15]) + (-6.090809e-04) * (q[i2] * q[i16]) + (1.210145e-03) * (q[i2] * q[i19]) + (-3.686003e-04) * (q[i2] * q[i20])
            + (4.179207e-04) * (q[i2] * q[i21]) + (-2.085002e-05) * (q[i2] * q[i22]) + (1.854835e-04) * (q[i3] * q[i4]) + (-6.531907e-04) * (q[i3] * q[i5])
            + (-4.024529e-04) * (q[i3] * q[i6]) + (-3.231969e-06) * (q[i3] * q[i7]) + (8.302458e-04) * (q[i3] * q[i8]) + (3.395112e-04) * (q[i3] * q[i9])
            + (-4.832726e-04) * (q[i3] * q[i10]) + (-2.917590e-04) * (q[i3] * q[i12]) + (-1.024490e-03) * (q[i3] * q[i15]) + (-5.020910e-05) * (q[i3] * q[i16])
            + (-4.190742e-05) * (q[i3] * q[i19]) + (1.218354e-04) * (q[i3] * q[i20]) + (-7.313836e-04) * (q[i3] * q[i21]) + (-8.641867e-05) * (q[i3] * q[i22])
            + (4.378300e-04) * (q[i4] * q[i5]) + (-4.370129e-04) * (q[i4] * q[i6]) + (-1.038503e-03) * (q[i4] * q[i7]) + (3.666690e-04) * (q[i4] * q[i8])
            + (8.569621e-05) * (q[i4] * q[i9]) + (3.251140e-04) * (q[i4] * q[i10]) + (2.936496e-04) * (q[i4] * q[i12]) + (-1.160440e-03) * (q[i4] * q[i15])
            + (-2.101134e-04) * (q[i4] * q[i16]) + (-4.214670e-04) * (q[i4] * q[i19]) + (2.725329e-04) * (q[i4] * q[i20]) + (-2.377437e-04) * (q[i4] * q[i21])
            + (3.716635e-05) * (q[i4] * q[i22]) + (-5.280932e-04) * (q[i5] * q[i6]) + (-1.002169e-03) * (q[i5] * q[i7]) + (5.523023e-04) * (q[i5] * q[i8])
            + (-3.886678e-04) * (q[i5] * q[i9]) + (-3.202338e-04) * (q[i5] * q[i10]) + (1.470583e-05) * (q[i5] * q[i12]) + (-2.237288e-03) * (q[i5] * q[i15])
            + (-2.229771e-05) * (q[i5] * q[i16]) + (2.494105e-04) * (q[i5] * q[i19]) + (7.149178e-05) * (q[i5] * q[i20]) + (-2.693622e-04) * (q[i5] * q[i21])
            + (1.497267e-05) * (q[i5] * q[i22]) + (-3.633376e-04) * (q[i6] * q[i7]) + (-5.018576e-04) * (q[i6] * q[i8]) + (-1.433083e-04) * (q[i6] * q[i9])
            + (-1.313475e-04) * (q[i6] * q[i10]) + (-2.782700e-04) * (q[i6] * q[i12]) + (-1.034194e-04) * (q[i6] * q[i15]) + (5.382410e-05) * (q[i6] * q[i16])
            + (-2.237864e-04) * (q[i6] * q[i19]) + (5.266348e-05) * (q[i6] * q[i20]) + (-1.794102e-04) * (q[i6] * q[i21]) + (2.074214e-04) * (q[i6] * q[i22])
            + (8.977087e-05) * (q[i7] * q[i8]) + (8.946800e-06) * (q[i7] * q[i9]) + (6.824602e-05) * (q[i7] * q[i10]) + (-2.844112e-04) * (q[i7] * q[i12])
            + (-5.275637e-05) * (q[i7] * q[i15]) + (8.334291e-05) * (q[i7] * q[i16]) + (-3.031969e-04) * (q[i7] * q[i19]) + (1.806968e-04) * (q[i7] * q[i20])
            + (-3.003335e-04) * (q[i7] * q[i21]) + (-3.141856e-05) * (q[i7] * q[i22]) + (-5.878640e-04) * (q[i8] * q[i9]) + (1.243089e-04) * (q[i8] * q[i10])
            + (-2.186374e-04) * (q[i8] * q[i12]) + (-6.202402e-04) * (q[i8] * q[i15]) + (-2.278058e-04) * (q[i8] * q[i16]) + (4.967597e-04) * (q[i8] * q[i19])
            + (1.391907e-04) * (q[i8] * q[i20]) + (7.869919e-06) * (q[i8] * q[i21]) + (-7.596144e-05) * (q[i8] * q[i22]) + (4.886757e-05) * (q[i9] * q[i10])
            + (1.294106e-04) * (q[i9] * q[i12]) + (3.956050e-06) * (q[i9] * q[i15]) + (1.615459e-04) * (q[i9] * q[i16]) + (-2.660218e-04) * (q[i9] * q[i19])
            + (2.055716e-04) * (q[i9] * q[i20]) + (-5.655255e-05) * (q[i9] * q[i21]) + (1.690360e-04) * (q[i9] * q[i22]) + (1.306873e-04) * (q[i10] * q[i12])
            + (-5.741643e-04) * (q[i10] * q[i15]) + (-1.549752e-04) * (q[i10] * q[i16]) + (-5.359431e-05) * (q[i10] * q[i19])
            + (6.115402e-05) * (q[i10] * q[i20]) + (-1.222902e-04) * (q[i10] * q[i21]) + (-7.195368e-05) * (q[i10] * q[i22])
            + (1.660088e-05) * (q[i12] * q[i15]) + (1.707712e-05) * (q[i12] * q[i16]) + (-2.223878e-04) * (q[i12] * q[i19])
            + (-2.200995e-04) * (q[i12] * q[i20]) + (3.935233e-06) * (q[i12] * q[i21]) + (-4.086977e-06) * (q[i12] * q[i22])
            + (2.246258e-04) * (q[i15] * q[i16]) + (-2.731911e-04) * (q[i15] * q[i19]) + (-1.157003e-04) * (q[i15] * q[i20])
            + (5.362887e-05) * (q[i15] * q[i21]) + (7.960830e-05) * (q[i15] * q[i22]) + (1.197865e-05) * (q[i16] * q[i19]) + (7.699104e-06) * (q[i16] * q[i20])
            + (4.936564e-05) * (q[i16] * q[i21]) + (-1.514886e-04) * (q[i16] * q[i22]) + (5.760783e-05) * (q[i19] * q[i20]) + (7.847570e-05) * (q[i19] * q[i21])
            + (-6.953426e-05) * (q[i19] * q[i22]) + (-1.020223e-04) * (q[i20] * q[i21]) + (-4.386798e-05) * (q[i20] * q[i22])
            + (-2.507574e-05) * (q[i21] * q[i22]);
      JQ[2][i12] = (7.177252e-03) * (1) + (-3.266432e-03) * ((2) * q[i12]) + (-4.475553e-04) * (q[i0]) + (-1.746531e-03) * (q[i1]) + (-2.708838e-03) * (q[i2])
            + (-5.651667e-03) * (q[i3]) + (-3.875277e-03) * (q[i4]) + (-4.297983e-03) * (q[i5]) + (8.597632e-04) * (q[i6]) + (7.330325e-04) * (q[i7])
            + (7.988239e-04) * (q[i8]) + (1.022610e-03) * (q[i9]) + (-2.625510e-05) * (q[i10]) + (-3.692658e-04) * (q[i11]) + (-8.773765e-04) * (q[i15])
            + (-1.290365e-03) * (q[i16]) + (4.731398e-04) * (q[i19]) + (1.054469e-03) * (q[i20]) + (-1.320978e-04) * (q[i21]) + (-1.535922e-03) * (q[i22])
            + (1.310823e-04) * (q[i0] * q[i0]) + (-2.479423e-04) * (q[i1] * q[i1]) + (-1.606939e-03) * (q[i2] * q[i2]) + (3.770896e-04) * (q[i3] * q[i3])
            + (6.490454e-04) * (q[i4] * q[i4]) + (-2.990441e-04) * (q[i5] * q[i5]) + (-2.601745e-04) * (q[i6] * q[i6]) + (5.256333e-04) * (q[i7] * q[i7])
            + (-1.040167e-03) * (q[i8] * q[i8]) + (5.262958e-05) * (q[i9] * q[i9]) + (2.749652e-05) * (q[i10] * q[i10]) + (-8.548084e-05) * (q[i11] * q[i11])
            + (1.346939e-04) * ((2) * q[i0] * q[i12]) + (-2.415135e-04) * ((2) * q[i1] * q[i12]) + (-2.434125e-03) * ((2) * q[i2] * q[i12])
            + (3.180134e-04) * ((2) * q[i3] * q[i12]) + (-5.807313e-05) * ((2) * q[i4] * q[i12]) + (-1.318934e-03) * ((2) * q[i5] * q[i12])
            + (-1.049009e-05) * ((2) * q[i6] * q[i12]) + (-4.457106e-04) * ((2) * q[i7] * q[i12]) + (1.567306e-03) * ((2) * q[i8] * q[i12])
            + (7.635329e-05) * ((2) * q[i9] * q[i12]) + (-1.057925e-04) * ((2) * q[i10] * q[i12]) + (9.131174e-05) * ((2) * q[i11] * q[i12])
            + (-1.147698e-03) * ((3) * q[i12] * q[i12]) + (-1.695134e-04) * ((2) * q[i12] * q[i15]) + (-1.947121e-03) * ((2) * q[i12] * q[i16])
            + (-2.958783e-05) * ((2) * q[i12] * q[i19]) + (4.820150e-04) * ((2) * q[i12] * q[i20]) + (4.809768e-05) * ((2) * q[i12] * q[i21])
            + (5.052362e-05) * ((2) * q[i12] * q[i22]) + (2.019327e-05) * (q[i15] * q[i15]) + (-2.171113e-03) * (q[i16] * q[i16])
            + (6.349233e-05) * (q[i19] * q[i19]) + (4.608820e-04) * (q[i20] * q[i20]) + (-6.658309e-05) * (q[i21] * q[i21]) + (4.546156e-04) * (q[i22] * q[i22])
            + (2.970770e-04) * (q[i0] * q[i1]) + (6.411273e-05) * (q[i0] * q[i2]) + (-9.528819e-04) * (q[i0] * q[i3]) + (-2.565737e-04) * (q[i0] * q[i4])
            + (1.086708e-03) * (q[i0] * q[i5]) + (-1.091461e-03) * (q[i0] * q[i6]) + (-7.667034e-05) * (q[i0] * q[i7]) + (-2.355451e-04) * (q[i0] * q[i8])
            + (-3.238291e-04) * (q[i0] * q[i9]) + (2.231798e-04) * (q[i0] * q[i10]) + (1.395562e-04) * (q[i0] * q[i11]) + (-1.615810e-04) * (q[i0] * q[i15])
            + (-1.409281e-05) * (q[i0] * q[i16]) + (1.480147e-04) * (q[i0] * q[i19]) + (-1.575840e-04) * (q[i0] * q[i20]) + (8.082817e-05) * (q[i0] * q[i21])
            + (-1.832422e-04) * (q[i0] * q[i22]) + (-1.039166e-03) * (q[i1] * q[i2]) + (3.474245e-05) * (q[i1] * q[i3]) + (-4.105944e-04) * (q[i1] * q[i4])
            + (1.038978e-03) * (q[i1] * q[i5]) + (4.752027e-05) * (q[i1] * q[i6]) + (-1.693575e-03) * (q[i1] * q[i7]) + (-4.676848e-04) * (q[i1] * q[i8])
            + (5.530140e-04) * (q[i1] * q[i9]) + (1.165157e-04) * (q[i1] * q[i10]) + (-1.408358e-04) * (q[i1] * q[i11]) + (-2.055615e-04) * (q[i1] * q[i15])
            + (1.753839e-04) * (q[i1] * q[i16]) + (-2.282897e-04) * (q[i1] * q[i19]) + (6.312069e-04) * (q[i1] * q[i20]) + (-8.990631e-05) * (q[i1] * q[i21])
            + (-1.564901e-05) * (q[i1] * q[i22]) + (3.965130e-04) * (q[i2] * q[i3]) + (2.623375e-04) * (q[i2] * q[i4]) + (-1.910099e-03) * (q[i2] * q[i5])
            + (2.174589e-04) * (q[i2] * q[i6]) + (-1.152232e-03) * (q[i2] * q[i7]) + (4.647709e-03) * (q[i2] * q[i8]) + (9.903040e-05) * (q[i2] * q[i9])
            + (5.846824e-05) * (q[i2] * q[i10]) + (5.027884e-06) * (q[i2] * q[i11]) + (-6.126193e-04) * (q[i2] * q[i15]) + (-2.427449e-03) * (q[i2] * q[i16])
            + (-3.721341e-04) * (q[i2] * q[i19]) + (1.194359e-03) * (q[i2] * q[i20]) + (1.978979e-05) * (q[i2] * q[i21]) + (-4.119226e-04) * (q[i2] * q[i22])
            + (-2.100593e-04) * (q[i3] * q[i4]) + (-4.466833e-04) * (q[i3] * q[i5]) + (-1.035896e-03) * (q[i3] * q[i6]) + (-4.333243e-04) * (q[i3] * q[i7])
            + (3.707437e-04) * (q[i3] * q[i8]) + (3.349317e-04) * (q[i3] * q[i9]) + (9.182299e-05) * (q[i3] * q[i10]) + (-2.917590e-04) * (q[i3] * q[i11])
            + (-2.073332e-04) * (q[i3] * q[i15]) + (-1.170318e-03) * (q[i3] * q[i16]) + (2.762868e-04) * (q[i3] * q[i19]) + (-4.230714e-04) * (q[i3] * q[i20])
            + (-3.968007e-05) * (q[i3] * q[i21]) + (2.359377e-04) * (q[i3] * q[i22]) + (6.221890e-04) * (q[i4] * q[i5]) + (2.431725e-06) * (q[i4] * q[i6])
            + (-4.192667e-04) * (q[i4] * q[i7]) + (8.563020e-04) * (q[i4] * q[i8]) + (-4.901619e-04) * (q[i4] * q[i9]) + (3.369470e-04) * (q[i4] * q[i10])
            + (2.936496e-04) * (q[i4] * q[i11]) + (-4.993484e-05) * (q[i4] * q[i15]) + (-1.028767e-03) * (q[i4] * q[i16]) + (1.145429e-04) * (q[i4] * q[i19])
            + (-3.911072e-05) * (q[i4] * q[i20]) + (9.036042e-05) * (q[i4] * q[i21]) + (7.318424e-04) * (q[i4] * q[i22]) + (-9.891163e-04) * (q[i5] * q[i6])
            + (-5.294600e-04) * (q[i5] * q[i7]) + (5.779302e-04) * (q[i5] * q[i8]) + (-3.133345e-04) * (q[i5] * q[i9]) + (-3.828427e-04) * (q[i5] * q[i10])
            + (1.470583e-05) * (q[i5] * q[i11]) + (-1.129334e-05) * (q[i5] * q[i15]) + (-2.231427e-03) * (q[i5] * q[i16]) + (6.981440e-05) * (q[i5] * q[i19])
            + (2.474242e-04) * (q[i5] * q[i20]) + (-1.618050e-05) * (q[i5] * q[i21]) + (2.787426e-04) * (q[i5] * q[i22]) + (3.594762e-04) * (q[i6] * q[i7])
            + (-5.343117e-05) * (q[i6] * q[i8]) + (-7.875434e-05) * (q[i6] * q[i9]) + (-1.012554e-05) * (q[i6] * q[i10]) + (-2.782700e-04) * (q[i6] * q[i11])
            + (-7.543038e-05) * (q[i6] * q[i15]) + (4.898942e-05) * (q[i6] * q[i16]) + (-1.779052e-04) * (q[i6] * q[i19]) + (2.987062e-04) * (q[i6] * q[i20])
            + (-3.023907e-05) * (q[i6] * q[i21]) + (-2.987766e-04) * (q[i6] * q[i22]) + (5.125980e-04) * (q[i7] * q[i8]) + (1.347187e-04) * (q[i7] * q[i9])
            + (1.358580e-04) * (q[i7] * q[i10]) + (-2.844112e-04) * (q[i7] * q[i11]) + (-5.183897e-05) * (q[i7] * q[i15]) + (1.050108e-04) * (q[i7] * q[i16])
            + (-5.069996e-05) * (q[i7] * q[i19]) + (2.194537e-04) * (q[i7] * q[i20]) + (2.042993e-04) * (q[i7] * q[i21]) + (-1.771800e-04) * (q[i7] * q[i22])
            + (-1.250041e-04) * (q[i8] * q[i9]) + (5.872017e-04) * (q[i8] * q[i10]) + (-2.186374e-04) * (q[i8] * q[i11]) + (2.198356e-04) * (q[i8] * q[i15])
            + (6.076149e-04) * (q[i8] * q[i16]) + (-1.399631e-04) * (q[i8] * q[i19]) + (-4.876793e-04) * (q[i8] * q[i20]) + (-7.232507e-05) * (q[i8] * q[i21])
            + (1.357240e-06) * (q[i8] * q[i22]) + (-4.815101e-05) * (q[i9] * q[i10]) + (1.294106e-04) * (q[i9] * q[i11]) + (1.577575e-04) * (q[i9] * q[i15])
            + (5.806691e-04) * (q[i9] * q[i16]) + (-6.097590e-05) * (q[i9] * q[i19]) + (5.071179e-05) * (q[i9] * q[i20]) + (-7.381588e-05) * (q[i9] * q[i21])
            + (-1.232102e-04) * (q[i9] * q[i22]) + (1.306873e-04) * (q[i10] * q[i11]) + (-1.558511e-04) * (q[i10] * q[i15])
            + (-2.731546e-06) * (q[i10] * q[i16]) + (-2.062423e-04) * (q[i10] * q[i19]) + (2.616168e-04) * (q[i10] * q[i20])
            + (1.687286e-04) * (q[i10] * q[i21]) + (-5.460698e-05) * (q[i10] * q[i22]) + (1.660088e-05) * (q[i11] * q[i15]) + (1.707712e-05) * (q[i11] * q[i16])
            + (-2.223878e-04) * (q[i11] * q[i19]) + (-2.200995e-04) * (q[i11] * q[i20]) + (3.935233e-06) * (q[i11] * q[i21])
            + (-4.086977e-06) * (q[i11] * q[i22]) + (-2.242077e-04) * (q[i15] * q[i16]) + (-9.356121e-06) * (q[i15] * q[i19])
            + (-1.318325e-05) * (q[i15] * q[i20]) + (-1.468120e-04) * (q[i15] * q[i21]) + (4.998225e-05) * (q[i15] * q[i22])
            + (1.185705e-04) * (q[i16] * q[i19]) + (2.802190e-04) * (q[i16] * q[i20]) + (8.148644e-05) * (q[i16] * q[i21]) + (3.854881e-05) * (q[i16] * q[i22])
            + (-5.788815e-05) * (q[i19] * q[i20]) + (-4.247808e-05) * (q[i19] * q[i21]) + (-1.049382e-04) * (q[i19] * q[i22])
            + (-6.778826e-05) * (q[i20] * q[i21]) + (8.675031e-05) * (q[i20] * q[i22]) + (2.284150e-05) * (q[i21] * q[i22]);
      JQ[2][i15] = (6.601284e-03) * (1) + (-1.411100e-03) * ((2) * q[i15]) + (3.125782e-03) * (q[i0]) + (3.317269e-03) * (q[i1]) + (6.361470e-03) * (q[i2])
            + (-1.862611e-03) * (q[i3]) + (-2.459107e-03) * (q[i4]) + (2.815028e-03) * (q[i5]) + (2.545774e-03) * (q[i6]) + (6.341804e-03) * (q[i7])
            + (-7.749465e-03) * (q[i8]) + (7.203772e-04) * (q[i9]) + (1.558823e-03) * (q[i10]) + (1.224232e-03) * (q[i11]) + (-8.773765e-04) * (q[i12])
            + (5.665540e-04) * (q[i16]) + (1.849066e-04) * (q[i19]) + (3.171280e-05) * (q[i20]) + (7.157948e-04) * (q[i21]) + (-1.049503e-04) * (q[i22])
            + (2.843503e-04) * (q[i0] * q[i0]) + (3.789879e-04) * (q[i1] * q[i1]) + (3.446050e-04) * (q[i2] * q[i2]) + (7.268971e-05) * (q[i3] * q[i3])
            + (-2.037655e-04) * (q[i4] * q[i4]) + (-1.269300e-03) * (q[i5] * q[i5]) + (-5.445084e-04) * (q[i6] * q[i6]) + (-2.316011e-04) * (q[i7] * q[i7])
            + (2.766317e-04) * (q[i8] * q[i8]) + (-2.901246e-05) * (q[i9] * q[i9]) + (-2.242911e-04) * (q[i10] * q[i10]) + (-1.956376e-03) * (q[i11] * q[i11])
            + (-1.695134e-04) * (q[i12] * q[i12]) + (1.811154e-04) * ((2) * q[i0] * q[i15]) + (2.195069e-04) * ((2) * q[i1] * q[i15])
            + (2.480727e-03) * ((2) * q[i2] * q[i15]) + (-6.305771e-04) * ((2) * q[i3] * q[i15]) + (-3.253032e-04) * ((2) * q[i4] * q[i15])
            + (2.926691e-04) * ((2) * q[i5] * q[i15]) + (-3.336203e-05) * ((2) * q[i6] * q[i15]) + (7.962845e-04) * ((2) * q[i7] * q[i15])
            + (1.056451e-04) * ((2) * q[i8] * q[i15]) + (-1.752264e-04) * ((2) * q[i9] * q[i15]) + (-5.884776e-05) * ((2) * q[i10] * q[i15])
            + (2.137069e-03) * ((2) * q[i11] * q[i15]) + (2.019327e-05) * ((2) * q[i12] * q[i15]) + (2.280557e-04) * ((3) * q[i15] * q[i15])
            + (-2.599556e-05) * ((2) * q[i15] * q[i16]) + (1.298486e-04) * ((2) * q[i15] * q[i19]) + (-6.822884e-05) * ((2) * q[i15] * q[i20])
            + (-1.844805e-04) * ((2) * q[i15] * q[i21]) + (9.319952e-05) * ((2) * q[i15] * q[i22]) + (-2.785503e-05) * (q[i16] * q[i16])
            + (-2.985337e-04) * (q[i19] * q[i19]) + (-8.019728e-05) * (q[i20] * q[i20]) + (-8.546269e-05) * (q[i21] * q[i21])
            + (-5.206458e-05) * (q[i22] * q[i22]) + (-1.059418e-04) * (q[i0] * q[i1]) + (3.562394e-04) * (q[i0] * q[i2]) + (-1.674781e-03) * (q[i0] * q[i3])
            + (5.375513e-04) * (q[i0] * q[i4]) + (-8.253056e-04) * (q[i0] * q[i5]) + (-4.290466e-04) * (q[i0] * q[i6]) + (4.276688e-04) * (q[i0] * q[i7])
            + (-6.521984e-05) * (q[i0] * q[i8]) + (-2.437090e-04) * (q[i0] * q[i9]) + (3.978931e-04) * (q[i0] * q[i10]) + (1.694093e-04) * (q[i0] * q[i11])
            + (-1.615810e-04) * (q[i0] * q[i12]) + (1.832028e-04) * (q[i0] * q[i16]) + (-8.622976e-06) * (q[i0] * q[i19]) + (-1.083230e-04) * (q[i0] * q[i20])
            + (3.299489e-04) * (q[i0] * q[i21]) + (1.040241e-05) * (q[i0] * q[i22]) + (7.528875e-04) * (q[i1] * q[i2]) + (5.455054e-04) * (q[i1] * q[i3])
            + (-3.807639e-03) * (q[i1] * q[i4]) + (-1.199295e-03) * (q[i1] * q[i5]) + (3.144774e-05) * (q[i1] * q[i6]) + (-1.361213e-03) * (q[i1] * q[i7])
            + (-1.883926e-04) * (q[i1] * q[i8]) + (2.875109e-04) * (q[i1] * q[i9]) + (-3.468384e-04) * (q[i1] * q[i10]) + (-1.825324e-05) * (q[i1] * q[i11])
            + (-2.055615e-04) * (q[i1] * q[i12]) + (-1.795702e-04) * (q[i1] * q[i16]) + (1.611291e-04) * (q[i1] * q[i19]) + (-1.568737e-04) * (q[i1] * q[i20])
            + (7.180507e-04) * (q[i1] * q[i21]) + (3.390483e-05) * (q[i1] * q[i22]) + (4.431432e-04) * (q[i2] * q[i3]) + (-9.084603e-04) * (q[i2] * q[i4])
            + (-6.389843e-03) * (q[i2] * q[i5]) + (-6.428325e-04) * (q[i2] * q[i6]) + (-6.230010e-04) * (q[i2] * q[i7]) + (3.739850e-04) * (q[i2] * q[i8])
            + (-1.966923e-04) * (q[i2] * q[i9]) + (1.523461e-04) * (q[i2] * q[i10]) + (-2.437054e-03) * (q[i2] * q[i11]) + (-6.126193e-04) * (q[i2] * q[i12])
            + (-1.065679e-06) * (q[i2] * q[i16]) + (8.714605e-05) * (q[i2] * q[i19]) + (-9.551340e-05) * (q[i2] * q[i20]) + (6.314654e-04) * (q[i2] * q[i21])
            + (1.855685e-04) * (q[i2] * q[i22]) + (-1.056834e-04) * (q[i3] * q[i4]) + (2.362346e-04) * (q[i3] * q[i5]) + (-9.391136e-04) * (q[i3] * q[i6])
            + (9.098021e-04) * (q[i3] * q[i7]) + (-1.874093e-03) * (q[i3] * q[i8]) + (-1.930683e-04) * (q[i3] * q[i9]) + (3.303860e-05) * (q[i3] * q[i10])
            + (-1.024490e-03) * (q[i3] * q[i11]) + (-2.073332e-04) * (q[i3] * q[i12]) + (2.405502e-04) * (q[i3] * q[i16]) + (-1.154513e-04) * (q[i3] * q[i19])
            + (8.314693e-05) * (q[i3] * q[i20]) + (-3.249219e-04) * (q[i3] * q[i21]) + (-2.351795e-04) * (q[i3] * q[i22]) + (5.443202e-05) * (q[i4] * q[i5])
            + (-7.902417e-04) * (q[i4] * q[i6]) + (2.146712e-05) * (q[i4] * q[i7]) + (-1.573262e-03) * (q[i4] * q[i8]) + (-2.303278e-04) * (q[i4] * q[i9])
            + (1.181967e-04) * (q[i4] * q[i10]) + (-1.160440e-03) * (q[i4] * q[i11]) + (-4.993484e-05) * (q[i4] * q[i12]) + (-2.399481e-04) * (q[i4] * q[i16])
            + (-3.102687e-04) * (q[i4] * q[i19]) + (1.892475e-04) * (q[i4] * q[i20]) + (-2.721388e-04) * (q[i4] * q[i21]) + (9.849923e-05) * (q[i4] * q[i22])
            + (1.541645e-03) * (q[i5] * q[i6]) + (-2.394354e-04) * (q[i5] * q[i7]) + (1.110366e-03) * (q[i5] * q[i8]) + (7.187954e-04) * (q[i5] * q[i9])
            + (1.317935e-04) * (q[i5] * q[i10]) + (-2.237288e-03) * (q[i5] * q[i11]) + (-1.129334e-05) * (q[i5] * q[i12]) + (-3.547791e-06) * (q[i5] * q[i16])
            + (-6.461589e-04) * (q[i5] * q[i19]) + (-2.655420e-04) * (q[i5] * q[i20]) + (7.849938e-04) * (q[i5] * q[i21]) + (2.021711e-04) * (q[i5] * q[i22])
            + (5.321298e-04) * (q[i6] * q[i7]) + (-3.231139e-04) * (q[i6] * q[i8]) + (-2.687738e-04) * (q[i6] * q[i9]) + (-5.974720e-05) * (q[i6] * q[i10])
            + (-1.034194e-04) * (q[i6] * q[i11]) + (-7.543038e-05) * (q[i6] * q[i12]) + (-2.468928e-05) * (q[i6] * q[i16]) + (-2.205203e-04) * (q[i6] * q[i19])
            + (9.431270e-05) * (q[i6] * q[i20]) + (3.049960e-04) * (q[i6] * q[i21]) + (-4.658677e-05) * (q[i6] * q[i22]) + (-3.710050e-04) * (q[i7] * q[i8])
            + (2.835824e-04) * (q[i7] * q[i9]) + (-4.023069e-04) * (q[i7] * q[i10]) + (-5.275637e-05) * (q[i7] * q[i11]) + (-5.183897e-05) * (q[i7] * q[i12])
            + (-2.721343e-05) * (q[i7] * q[i16]) + (-1.581387e-04) * (q[i7] * q[i19]) + (-7.403503e-05) * (q[i7] * q[i20]) + (4.517717e-04) * (q[i7] * q[i21])
            + (2.796963e-04) * (q[i7] * q[i22]) + (-2.177621e-04) * (q[i8] * q[i9]) + (-5.728400e-04) * (q[i8] * q[i10]) + (-6.202402e-04) * (q[i8] * q[i11])
            + (2.198356e-04) * (q[i8] * q[i12]) + (-2.887256e-05) * (q[i8] * q[i16]) + (2.942123e-04) * (q[i8] * q[i19]) + (1.343018e-05) * (q[i8] * q[i20])
            + (-8.182625e-04) * (q[i8] * q[i21]) + (7.254698e-05) * (q[i8] * q[i22]) + (6.594137e-05) * (q[i9] * q[i10]) + (3.956050e-06) * (q[i9] * q[i11])
            + (1.577575e-04) * (q[i9] * q[i12]) + (-4.069744e-05) * (q[i9] * q[i16]) + (-5.244398e-05) * (q[i9] * q[i19]) + (8.511117e-05) * (q[i9] * q[i20])
            + (8.364882e-05) * (q[i9] * q[i21]) + (1.024389e-04) * (q[i9] * q[i22]) + (-5.741643e-04) * (q[i10] * q[i11]) + (-1.558511e-04) * (q[i10] * q[i12])
            + (-4.238001e-05) * (q[i10] * q[i16]) + (-3.179155e-05) * (q[i10] * q[i19]) + (3.929254e-05) * (q[i10] * q[i20])
            + (1.331861e-04) * (q[i10] * q[i21]) + (4.119188e-05) * (q[i10] * q[i22]) + (1.660088e-05) * (q[i11] * q[i12]) + (2.246258e-04) * (q[i11] * q[i16])
            + (-2.731911e-04) * (q[i11] * q[i19]) + (-1.157003e-04) * (q[i11] * q[i20]) + (5.362887e-05) * (q[i11] * q[i21])
            + (7.960830e-05) * (q[i11] * q[i22]) + (-2.242077e-04) * (q[i12] * q[i16]) + (-9.356121e-06) * (q[i12] * q[i19])
            + (-1.318325e-05) * (q[i12] * q[i20]) + (-1.468120e-04) * (q[i12] * q[i21]) + (4.998225e-05) * (q[i12] * q[i22])
            + (7.088559e-05) * (q[i16] * q[i19]) + (6.764808e-05) * (q[i16] * q[i20]) + (1.638822e-04) * (q[i16] * q[i21]) + (-1.640155e-04) * (q[i16] * q[i22])
            + (-9.041658e-06) * (q[i19] * q[i20]) + (-7.785493e-04) * (q[i19] * q[i21]) + (-6.918400e-05) * (q[i19] * q[i22])
            + (7.900249e-05) * (q[i20] * q[i21]) + (-8.887574e-06) * (q[i20] * q[i22]) + (-7.131537e-05) * (q[i21] * q[i22]);
      JQ[2][i16] = (6.513104e-03) * (1) + (-1.437953e-03) * ((2) * q[i16]) + (-3.367573e-03) * (q[i0]) + (-3.137907e-03) * (q[i1]) + (-6.473874e-03) * (q[i2])
            + (2.498941e-03) * (q[i3]) + (1.907826e-03) * (q[i4]) + (-2.854398e-03) * (q[i5]) + (6.446622e-03) * (q[i6]) + (2.538173e-03) * (q[i7])
            + (-7.820070e-03) * (q[i8]) + (1.594974e-03) * (q[i9]) + (7.085550e-04) * (q[i10]) + (8.886032e-04) * (q[i11]) + (-1.290365e-03) * (q[i12])
            + (5.665540e-04) * (q[i15]) + (3.598342e-05) * (q[i19]) + (1.660364e-04) * (q[i20]) + (1.118701e-04) * (q[i21]) + (-7.094236e-04) * (q[i22])
            + (3.868913e-04) * (q[i0] * q[i0]) + (2.860883e-04) * (q[i1] * q[i1]) + (3.625418e-04) * (q[i2] * q[i2]) + (-2.084037e-04) * (q[i3] * q[i3])
            + (6.807479e-05) * (q[i4] * q[i4]) + (-1.285440e-03) * (q[i5] * q[i5]) + (-2.344344e-04) * (q[i6] * q[i6]) + (-5.468400e-04) * (q[i7] * q[i7])
            + (2.802137e-04) * (q[i8] * q[i8]) + (-2.284468e-04) * (q[i9] * q[i9]) + (-2.973345e-05) * (q[i10] * q[i10]) + (-1.667675e-04) * (q[i11] * q[i11])
            + (-1.947121e-03) * (q[i12] * q[i12]) + (-2.599556e-05) * (q[i15] * q[i15]) + (-2.201567e-04) * ((2) * q[i0] * q[i16])
            + (-1.820508e-04) * ((2) * q[i1] * q[i16]) + (-2.510068e-03) * ((2) * q[i2] * q[i16]) + (3.251068e-04) * ((2) * q[i3] * q[i16])
            + (6.351138e-04) * ((2) * q[i4] * q[i16]) + (-2.914479e-04) * ((2) * q[i5] * q[i16]) + (8.003635e-04) * ((2) * q[i6] * q[i16])
            + (-3.535705e-05) * ((2) * q[i7] * q[i16]) + (1.140662e-04) * ((2) * q[i8] * q[i16]) + (-6.107686e-05) * ((2) * q[i9] * q[i16])
            + (-1.768509e-04) * ((2) * q[i10] * q[i16]) + (-2.123403e-05) * ((2) * q[i11] * q[i16]) + (-2.171113e-03) * ((2) * q[i12] * q[i16])
            + (-2.785503e-05) * ((2) * q[i15] * q[i16]) + (2.302472e-04) * ((3) * q[i16] * q[i16]) + (-6.792132e-05) * ((2) * q[i16] * q[i19])
            + (1.229388e-04) * ((2) * q[i16] * q[i20]) + (-9.468958e-05) * ((2) * q[i16] * q[i21]) + (1.911788e-04) * ((2) * q[i16] * q[i22])
            + (-7.985526e-05) * (q[i19] * q[i19]) + (-2.866588e-04) * (q[i20] * q[i20]) + (-5.159170e-05) * (q[i21] * q[i21])
            + (-8.863382e-05) * (q[i22] * q[i22]) + (-1.032743e-04) * (q[i0] * q[i1]) + (7.656795e-04) * (q[i0] * q[i2]) + (-3.861911e-03) * (q[i0] * q[i3])
            + (5.378835e-04) * (q[i0] * q[i4]) + (-1.200608e-03) * (q[i0] * q[i5]) + (1.370662e-03) * (q[i0] * q[i6]) + (-3.514662e-05) * (q[i0] * q[i7])
            + (2.012094e-04) * (q[i0] * q[i8]) + (3.510747e-04) * (q[i0] * q[i9]) + (-2.885660e-04) * (q[i0] * q[i10]) + (-2.069843e-04) * (q[i0] * q[i11])
            + (-1.409281e-05) * (q[i0] * q[i12]) + (1.832028e-04) * (q[i0] * q[i15]) + (1.609813e-04) * (q[i0] * q[i19]) + (-1.584302e-04) * (q[i0] * q[i20])
            + (3.315816e-05) * (q[i0] * q[i21]) + (7.259586e-04) * (q[i0] * q[i22]) + (3.601205e-04) * (q[i1] * q[i2]) + (5.488424e-04) * (q[i1] * q[i3])
            + (-1.680345e-03) * (q[i1] * q[i4]) + (-8.293675e-04) * (q[i1] * q[i5]) + (-4.277297e-04) * (q[i1] * q[i6]) + (4.450985e-04) * (q[i1] * q[i7])
            + (6.772655e-05) * (q[i1] * q[i8]) + (-3.991676e-04) * (q[i1] * q[i9]) + (2.473510e-04) * (q[i1] * q[i10]) + (-1.518464e-04) * (q[i1] * q[i11])
            + (1.753839e-04) * (q[i1] * q[i12]) + (-1.795702e-04) * (q[i1] * q[i15]) + (1.016426e-04) * (q[i1] * q[i19]) + (1.760141e-05) * (q[i1] * q[i20])
            + (7.808667e-06) * (q[i1] * q[i21]) + (3.334762e-04) * (q[i1] * q[i22]) + (-9.113398e-04) * (q[i2] * q[i3]) + (4.579803e-04) * (q[i2] * q[i4])
            + (-6.479207e-03) * (q[i2] * q[i5]) + (6.249175e-04) * (q[i2] * q[i6]) + (6.615263e-04) * (q[i2] * q[i7]) + (-4.028890e-04) * (q[i2] * q[i8])
            + (-1.519043e-04) * (q[i2] * q[i9]) + (1.977923e-04) * (q[i2] * q[i10]) + (-6.090809e-04) * (q[i2] * q[i11]) + (-2.427449e-03) * (q[i2] * q[i12])
            + (-1.065679e-06) * (q[i2] * q[i15]) + (9.087750e-05) * (q[i2] * q[i19]) + (-6.702515e-05) * (q[i2] * q[i20]) + (1.824016e-04) * (q[i2] * q[i21])
            + (6.283451e-04) * (q[i2] * q[i22]) + (-1.009872e-04) * (q[i3] * q[i4]) + (5.074723e-05) * (q[i3] * q[i5]) + (-1.784537e-05) * (q[i3] * q[i6])
            + (8.037877e-04) * (q[i3] * q[i7]) + (1.586166e-03) * (q[i3] * q[i8]) + (-1.237888e-04) * (q[i3] * q[i9]) + (2.333993e-04) * (q[i3] * q[i10])
            + (-5.020910e-05) * (q[i3] * q[i11]) + (-1.170318e-03) * (q[i3] * q[i12]) + (2.405502e-04) * (q[i3] * q[i15]) + (-1.910909e-04) * (q[i3] * q[i19])
            + (3.031278e-04) * (q[i3] * q[i20]) + (9.856968e-05) * (q[i3] * q[i21]) + (-2.787869e-04) * (q[i3] * q[i22]) + (2.385429e-04) * (q[i4] * q[i5])
            + (-9.180732e-04) * (q[i4] * q[i6]) + (9.376283e-04) * (q[i4] * q[i7]) + (1.892670e-03) * (q[i4] * q[i8]) + (-3.853204e-05) * (q[i4] * q[i9])
            + (1.877344e-04) * (q[i4] * q[i10]) + (-2.101134e-04) * (q[i4] * q[i11]) + (-1.028767e-03) * (q[i4] * q[i12]) + (-2.399481e-04) * (q[i4] * q[i15])
            + (-8.271905e-05) * (q[i4] * q[i19]) + (1.078302e-04) * (q[i4] * q[i20]) + (-2.327941e-04) * (q[i4] * q[i21]) + (-3.290808e-04) * (q[i4] * q[i22])
            + (2.314176e-04) * (q[i5] * q[i6]) + (-1.548307e-03) * (q[i5] * q[i7]) + (-1.141076e-03) * (q[i5] * q[i8]) + (-1.300566e-04) * (q[i5] * q[i9])
            + (-7.160931e-04) * (q[i5] * q[i10]) + (-2.229771e-05) * (q[i5] * q[i11]) + (-2.231427e-03) * (q[i5] * q[i12]) + (-3.547791e-06) * (q[i5] * q[i15])
            + (2.734731e-04) * (q[i5] * q[i19]) + (6.321716e-04) * (q[i5] * q[i20]) + (2.034900e-04) * (q[i5] * q[i21]) + (7.961114e-04) * (q[i5] * q[i22])
            + (5.348429e-04) * (q[i6] * q[i7]) + (-3.754556e-04) * (q[i6] * q[i8]) + (-4.103087e-04) * (q[i6] * q[i9]) + (2.814061e-04) * (q[i6] * q[i10])
            + (5.382410e-05) * (q[i6] * q[i11]) + (4.898942e-05) * (q[i6] * q[i12]) + (-2.468928e-05) * (q[i6] * q[i15]) + (-7.431485e-05) * (q[i6] * q[i19])
            + (-1.600898e-04) * (q[i6] * q[i20]) + (-2.786086e-04) * (q[i6] * q[i21]) + (-4.603793e-04) * (q[i6] * q[i22]) + (-3.285802e-04) * (q[i7] * q[i8])
            + (-5.507066e-05) * (q[i7] * q[i9]) + (-2.686026e-04) * (q[i7] * q[i10]) + (8.334291e-05) * (q[i7] * q[i11]) + (1.050108e-04) * (q[i7] * q[i12])
            + (-2.721343e-05) * (q[i7] * q[i15]) + (9.455737e-05) * (q[i7] * q[i19]) + (-2.170218e-04) * (q[i7] * q[i20]) + (4.784558e-05) * (q[i7] * q[i21])
            + (-3.036729e-04) * (q[i7] * q[i22]) + (-5.823431e-04) * (q[i8] * q[i9]) + (-2.213568e-04) * (q[i8] * q[i10]) + (-2.278058e-04) * (q[i8] * q[i11])
            + (6.076149e-04) * (q[i8] * q[i12]) + (-2.887256e-05) * (q[i8] * q[i15]) + (1.246325e-05) * (q[i8] * q[i19]) + (2.942755e-04) * (q[i8] * q[i20])
            + (-7.242451e-05) * (q[i8] * q[i21]) + (8.305395e-04) * (q[i8] * q[i22]) + (6.633137e-05) * (q[i9] * q[i10]) + (1.615459e-04) * (q[i9] * q[i11])
            + (5.806691e-04) * (q[i9] * q[i12]) + (-4.069744e-05) * (q[i9] * q[i15]) + (3.970330e-05) * (q[i9] * q[i19]) + (-2.986617e-05) * (q[i9] * q[i20])
            + (-4.268491e-05) * (q[i9] * q[i21]) + (-1.365421e-04) * (q[i9] * q[i22]) + (-1.549752e-04) * (q[i10] * q[i11])
            + (-2.731546e-06) * (q[i10] * q[i12]) + (-4.238001e-05) * (q[i10] * q[i15]) + (8.554462e-05) * (q[i10] * q[i19])
            + (-5.567756e-05) * (q[i10] * q[i20]) + (-1.035764e-04) * (q[i10] * q[i21]) + (-8.357313e-05) * (q[i10] * q[i22])
            + (1.707712e-05) * (q[i11] * q[i12]) + (2.246258e-04) * (q[i11] * q[i15]) + (1.197865e-05) * (q[i11] * q[i19]) + (7.699104e-06) * (q[i11] * q[i20])
            + (4.936564e-05) * (q[i11] * q[i21]) + (-1.514886e-04) * (q[i11] * q[i22]) + (-2.242077e-04) * (q[i12] * q[i15])
            + (1.185705e-04) * (q[i12] * q[i19]) + (2.802190e-04) * (q[i12] * q[i20]) + (8.148644e-05) * (q[i12] * q[i21]) + (3.854881e-05) * (q[i12] * q[i22])
            + (7.088559e-05) * (q[i15] * q[i19]) + (6.764808e-05) * (q[i15] * q[i20]) + (1.638822e-04) * (q[i15] * q[i21]) + (-1.640155e-04) * (q[i15] * q[i22])
            + (-9.552562e-06) * (q[i19] * q[i20]) + (9.281681e-06) * (q[i19] * q[i21]) + (-7.924950e-05) * (q[i19] * q[i22])
            + (7.053497e-05) * (q[i20] * q[i21]) + (7.728345e-04) * (q[i20] * q[i22]) + (-7.306663e-05) * (q[i21] * q[i22]);
      JQ[2][i19] = (-8.200116e-04) * (1) + (-2.374948e-04) * ((2) * q[i19]) + (2.667678e-04) * (q[i0]) + (8.714941e-04) * (q[i1]) + (8.740099e-05) * (q[i2])
            + (-1.422479e-03) * (q[i3]) + (-3.897407e-04) * (q[i4]) + (-5.138955e-04) * (q[i5]) + (-2.650868e-04) * (q[i6]) + (4.083241e-04) * (q[i7])
            + (-3.488025e-04) * (q[i8]) + (-1.511852e-04) * (q[i9]) + (-1.109468e-05) * (q[i10]) + (-1.088796e-03) * (q[i11]) + (4.731398e-04) * (q[i12])
            + (1.849066e-04) * (q[i15]) + (3.598342e-05) * (q[i16]) + (-1.484084e-04) * (q[i20]) + (1.465471e-03) * (q[i21]) + (-2.425777e-04) * (q[i22])
            + (2.950859e-04) * (q[i0] * q[i0]) + (2.170462e-04) * (q[i1] * q[i1]) + (9.646155e-04) * (q[i2] * q[i2]) + (-1.649753e-04) * (q[i3] * q[i3])
            + (-3.064979e-04) * (q[i4] * q[i4]) + (-7.327898e-04) * (q[i5] * q[i5]) + (-4.345726e-04) * (q[i6] * q[i6]) + (8.015671e-05) * (q[i7] * q[i7])
            + (1.835682e-04) * (q[i8] * q[i8]) + (6.414339e-05) * (q[i9] * q[i9]) + (-1.276539e-06) * (q[i10] * q[i10]) + (4.941124e-04) * (q[i11] * q[i11])
            + (-2.958783e-05) * (q[i12] * q[i12]) + (1.298486e-04) * (q[i15] * q[i15]) + (-6.792132e-05) * (q[i16] * q[i16])
            + (1.728568e-04) * ((2) * q[i0] * q[i19]) + (4.184498e-05) * ((2) * q[i1] * q[i19]) + (-2.042859e-04) * ((2) * q[i2] * q[i19])
            + (-4.560208e-04) * ((2) * q[i3] * q[i19]) + (1.279469e-04) * ((2) * q[i4] * q[i19]) + (-2.332126e-04) * ((2) * q[i5] * q[i19])
            + (-1.507277e-04) * ((2) * q[i6] * q[i19]) + (1.477554e-04) * ((2) * q[i7] * q[i19]) + (-1.688085e-04) * ((2) * q[i8] * q[i19])
            + (1.884527e-05) * ((2) * q[i9] * q[i19]) + (-7.063182e-05) * ((2) * q[i10] * q[i19]) + (-4.597726e-04) * ((2) * q[i11] * q[i19])
            + (6.349233e-05) * ((2) * q[i12] * q[i19]) + (-2.985337e-04) * ((2) * q[i15] * q[i19]) + (-7.985526e-05) * ((2) * q[i16] * q[i19])
            + (6.091844e-05) * ((3) * q[i19] * q[i19]) + (-4.986784e-05) * ((2) * q[i19] * q[i20]) + (-1.143215e-04) * ((2) * q[i19] * q[i21])
            + (3.806140e-05) * ((2) * q[i19] * q[i22]) + (-5.008723e-05) * (q[i20] * q[i20]) + (5.550159e-04) * (q[i21] * q[i21])
            + (-6.935476e-05) * (q[i22] * q[i22]) + (-1.413254e-04) * (q[i0] * q[i1]) + (6.379012e-04) * (q[i0] * q[i2]) + (-7.626717e-04) * (q[i0] * q[i3])
            + (2.205854e-05) * (q[i0] * q[i4]) + (-1.774556e-04) * (q[i0] * q[i5]) + (-1.298252e-03) * (q[i0] * q[i6]) + (3.153959e-04) * (q[i0] * q[i7])
            + (-2.381760e-04) * (q[i0] * q[i8]) + (-2.967470e-04) * (q[i0] * q[i9]) + (2.560510e-04) * (q[i0] * q[i10]) + (6.329313e-04) * (q[i0] * q[i11])
            + (1.480147e-04) * (q[i0] * q[i12]) + (-8.622976e-06) * (q[i0] * q[i15]) + (1.609813e-04) * (q[i0] * q[i16]) + (2.514656e-05) * (q[i0] * q[i20])
            + (-1.606318e-06) * (q[i0] * q[i21]) + (-4.005700e-04) * (q[i0] * q[i22]) + (3.645932e-05) * (q[i1] * q[i2]) + (1.523011e-05) * (q[i1] * q[i3])
            + (4.173312e-05) * (q[i1] * q[i4]) + (-3.957794e-04) * (q[i1] * q[i5]) + (1.368596e-04) * (q[i1] * q[i6]) + (-9.905888e-04) * (q[i1] * q[i7])
            + (-1.747253e-04) * (q[i1] * q[i8]) + (-1.422766e-04) * (q[i1] * q[i9]) + (-3.908049e-04) * (q[i1] * q[i10]) + (-1.618743e-04) * (q[i1] * q[i11])
            + (-2.282897e-04) * (q[i1] * q[i12]) + (1.611291e-04) * (q[i1] * q[i15]) + (1.016426e-04) * (q[i1] * q[i16]) + (-2.081722e-05) * (q[i1] * q[i20])
            + (1.391243e-04) * (q[i1] * q[i21]) + (4.763852e-05) * (q[i1] * q[i22]) + (-4.642063e-04) * (q[i2] * q[i3]) + (-2.047420e-04) * (q[i2] * q[i4])
            + (2.116533e-05) * (q[i2] * q[i5]) + (-7.928593e-04) * (q[i2] * q[i6]) + (-9.679730e-05) * (q[i2] * q[i7]) + (-1.238277e-04) * (q[i2] * q[i8])
            + (-5.151900e-04) * (q[i2] * q[i9]) + (-5.029466e-05) * (q[i2] * q[i10]) + (1.210145e-03) * (q[i2] * q[i11]) + (-3.721341e-04) * (q[i2] * q[i12])
            + (8.714605e-05) * (q[i2] * q[i15]) + (9.087750e-05) * (q[i2] * q[i16]) + (4.114824e-06) * (q[i2] * q[i20]) + (-2.210598e-04) * (q[i2] * q[i21])
            + (-4.203312e-04) * (q[i2] * q[i22]) + (4.109163e-04) * (q[i3] * q[i4]) + (1.175286e-04) * (q[i3] * q[i5]) + (5.723820e-05) * (q[i3] * q[i6])
            + (-8.243394e-04) * (q[i3] * q[i7]) + (-1.489504e-04) * (q[i3] * q[i8]) + (1.574528e-04) * (q[i3] * q[i9]) + (6.864690e-05) * (q[i3] * q[i10])
            + (-4.190742e-05) * (q[i3] * q[i11]) + (2.762868e-04) * (q[i3] * q[i12]) + (-1.154513e-04) * (q[i3] * q[i15]) + (-1.910909e-04) * (q[i3] * q[i16])
            + (1.303709e-04) * (q[i3] * q[i20]) + (-2.283146e-04) * (q[i3] * q[i21]) + (2.080684e-04) * (q[i3] * q[i22]) + (-9.016316e-04) * (q[i4] * q[i5])
            + (-3.333828e-04) * (q[i4] * q[i6]) + (-5.666982e-04) * (q[i4] * q[i7]) + (5.975865e-05) * (q[i4] * q[i8]) + (-2.364868e-04) * (q[i4] * q[i9])
            + (-9.532546e-05) * (q[i4] * q[i10]) + (-4.214670e-04) * (q[i4] * q[i11]) + (1.145429e-04) * (q[i4] * q[i12]) + (-3.102687e-04) * (q[i4] * q[i15])
            + (-8.271905e-05) * (q[i4] * q[i16]) + (-1.338756e-04) * (q[i4] * q[i20]) + (3.439801e-04) * (q[i4] * q[i21]) + (-2.058941e-04) * (q[i4] * q[i22])
            + (1.809299e-04) * (q[i5] * q[i6]) + (3.610782e-04) * (q[i5] * q[i7]) + (-4.558482e-04) * (q[i5] * q[i8]) + (2.208338e-04) * (q[i5] * q[i9])
            + (1.491292e-04) * (q[i5] * q[i10]) + (2.494105e-04) * (q[i5] * q[i11]) + (6.981440e-05) * (q[i5] * q[i12]) + (-6.461589e-04) * (q[i5] * q[i15])
            + (2.734731e-04) * (q[i5] * q[i16]) + (4.417112e-06) * (q[i5] * q[i20]) + (6.504661e-04) * (q[i5] * q[i21]) + (-1.864600e-04) * (q[i5] * q[i22])
            + (3.668577e-04) * (q[i6] * q[i7]) + (-1.490583e-04) * (q[i6] * q[i8]) + (1.042731e-04) * (q[i6] * q[i9]) + (-1.477668e-04) * (q[i6] * q[i10])
            + (-2.237864e-04) * (q[i6] * q[i11]) + (-1.779052e-04) * (q[i6] * q[i12]) + (-2.205203e-04) * (q[i6] * q[i15]) + (-7.431485e-05) * (q[i6] * q[i16])
            + (-4.335967e-05) * (q[i6] * q[i20]) + (2.546358e-04) * (q[i6] * q[i21]) + (2.654591e-05) * (q[i6] * q[i22]) + (-1.816022e-04) * (q[i7] * q[i8])
            + (5.549223e-05) * (q[i7] * q[i9]) + (-6.344870e-05) * (q[i7] * q[i10]) + (-3.031969e-04) * (q[i7] * q[i11]) + (-5.069996e-05) * (q[i7] * q[i12])
            + (-1.581387e-04) * (q[i7] * q[i15]) + (9.455737e-05) * (q[i7] * q[i16]) + (-4.366264e-05) * (q[i7] * q[i20]) + (2.590795e-04) * (q[i7] * q[i21])
            + (-4.059630e-05) * (q[i7] * q[i22]) + (4.886623e-06) * (q[i8] * q[i9]) + (7.077076e-05) * (q[i8] * q[i10]) + (4.967597e-04) * (q[i8] * q[i11])
            + (-1.399631e-04) * (q[i8] * q[i12]) + (2.942123e-04) * (q[i8] * q[i15]) + (1.246325e-05) * (q[i8] * q[i16]) + (-2.221943e-04) * (q[i8] * q[i20])
            + (-5.539763e-04) * (q[i8] * q[i21]) + (-3.164284e-05) * (q[i8] * q[i22]) + (-3.746738e-05) * (q[i9] * q[i10]) + (-2.660218e-04) * (q[i9] * q[i11])
            + (-6.097590e-05) * (q[i9] * q[i12]) + (-5.244398e-05) * (q[i9] * q[i15]) + (3.970330e-05) * (q[i9] * q[i16]) + (3.019187e-05) * (q[i9] * q[i20])
            + (6.170221e-06) * (q[i9] * q[i21]) + (7.599913e-05) * (q[i9] * q[i22]) + (-5.359431e-05) * (q[i10] * q[i11]) + (-2.062423e-04) * (q[i10] * q[i12])
            + (-3.179155e-05) * (q[i10] * q[i15]) + (8.554462e-05) * (q[i10] * q[i16]) + (3.192201e-05) * (q[i10] * q[i20]) + (1.496858e-04) * (q[i10] * q[i21])
            + (5.384536e-05) * (q[i10] * q[i22]) + (-2.223878e-04) * (q[i11] * q[i12]) + (-2.731911e-04) * (q[i11] * q[i15])
            + (1.197865e-05) * (q[i11] * q[i16]) + (5.760783e-05) * (q[i11] * q[i20]) + (7.847570e-05) * (q[i11] * q[i21]) + (-6.953426e-05) * (q[i11] * q[i22])
            + (-9.356121e-06) * (q[i12] * q[i15]) + (1.185705e-04) * (q[i12] * q[i16]) + (-5.788815e-05) * (q[i12] * q[i20])
            + (-4.247808e-05) * (q[i12] * q[i21]) + (-1.049382e-04) * (q[i12] * q[i22]) + (7.088559e-05) * (q[i15] * q[i16])
            + (-9.041658e-06) * (q[i15] * q[i20]) + (-7.785493e-04) * (q[i15] * q[i21]) + (-6.918400e-05) * (q[i15] * q[i22])
            + (-9.552562e-06) * (q[i16] * q[i20]) + (9.281681e-06) * (q[i16] * q[i21]) + (-7.924950e-05) * (q[i16] * q[i22])
            + (-2.281735e-05) * (q[i20] * q[i21]) + (2.105748e-05) * (q[i20] * q[i22]) + (7.189642e-05) * (q[i21] * q[i22]);
      JQ[2][i20] = (-7.776449e-04) * (1) + (-2.294355e-04) * ((2) * q[i20]) + (-8.810999e-04) * (q[i0]) + (-2.465857e-04) * (q[i1]) + (-9.883928e-05) * (q[i2])
            + (3.632669e-04) * (q[i3]) + (1.405618e-03) * (q[i4]) + (4.998270e-04) * (q[i5]) + (4.161348e-04) * (q[i6]) + (-2.641031e-04) * (q[i7])
            + (-3.333278e-04) * (q[i8]) + (-3.007639e-06) * (q[i9]) + (-1.568486e-04) * (q[i10]) + (-4.647446e-04) * (q[i11]) + (1.054469e-03) * (q[i12])
            + (3.171280e-05) * (q[i15]) + (1.660364e-04) * (q[i16]) + (-1.484084e-04) * (q[i19]) + (2.482349e-04) * (q[i21]) + (-1.482029e-03) * (q[i22])
            + (2.163784e-04) * (q[i0] * q[i0]) + (2.942254e-04) * (q[i1] * q[i1]) + (9.561565e-04) * (q[i2] * q[i2]) + (-3.124390e-04) * (q[i3] * q[i3])
            + (-1.632086e-04) * (q[i4] * q[i4]) + (-7.266314e-04) * (q[i5] * q[i5]) + (7.992784e-05) * (q[i6] * q[i6]) + (-4.334767e-04) * (q[i7] * q[i7])
            + (1.768287e-04) * (q[i8] * q[i8]) + (-2.279670e-06) * (q[i9] * q[i9]) + (6.250395e-05) * (q[i10] * q[i10]) + (-2.970354e-05) * (q[i11] * q[i11])
            + (4.820150e-04) * (q[i12] * q[i12]) + (-6.822884e-05) * (q[i15] * q[i15]) + (1.229388e-04) * (q[i16] * q[i16])
            + (-4.986784e-05) * (q[i19] * q[i19]) + (-4.234846e-05) * ((2) * q[i0] * q[i20]) + (-1.711052e-04) * ((2) * q[i1] * q[i20])
            + (2.057429e-04) * ((2) * q[i2] * q[i20]) + (-1.282047e-04) * ((2) * q[i3] * q[i20]) + (4.503998e-04) * ((2) * q[i4] * q[i20])
            + (2.367955e-04) * ((2) * q[i5] * q[i20]) + (1.439755e-04) * ((2) * q[i6] * q[i20]) + (-1.466157e-04) * ((2) * q[i7] * q[i20])
            + (-1.628132e-04) * ((2) * q[i8] * q[i20]) + (-7.237348e-05) * ((2) * q[i9] * q[i20]) + (1.950431e-05) * ((2) * q[i10] * q[i20])
            + (-6.177688e-05) * ((2) * q[i11] * q[i20]) + (4.608820e-04) * ((2) * q[i12] * q[i20]) + (-8.019728e-05) * ((2) * q[i15] * q[i20])
            + (-2.866588e-04) * ((2) * q[i16] * q[i20]) + (-5.008723e-05) * ((2) * q[i19] * q[i20]) + (5.911894e-05) * ((3) * q[i20] * q[i20])
            + (-3.731444e-05) * ((2) * q[i20] * q[i21]) + (1.148807e-04) * ((2) * q[i20] * q[i22]) + (-6.873703e-05) * (q[i21] * q[i21])
            + (5.637903e-04) * (q[i22] * q[i22]) + (-1.424201e-04) * (q[i0] * q[i1]) + (3.685915e-05) * (q[i0] * q[i2]) + (4.278285e-05) * (q[i0] * q[i3])
            + (9.761715e-06) * (q[i0] * q[i4]) + (-3.916660e-04) * (q[i0] * q[i5]) + (9.889740e-04) * (q[i0] * q[i6]) + (-1.302443e-04) * (q[i0] * q[i7])
            + (1.675113e-04) * (q[i0] * q[i8]) + (3.917039e-04) * (q[i0] * q[i9]) + (1.447837e-04) * (q[i0] * q[i10]) + (-2.262147e-04) * (q[i0] * q[i11])
            + (-1.575840e-04) * (q[i0] * q[i12]) + (-1.083230e-04) * (q[i0] * q[i15]) + (-1.584302e-04) * (q[i0] * q[i16]) + (2.514656e-05) * (q[i0] * q[i19])
            + (4.732295e-05) * (q[i0] * q[i21]) + (1.434421e-04) * (q[i0] * q[i22]) + (6.373758e-04) * (q[i1] * q[i2]) + (2.528618e-05) * (q[i1] * q[i3])
            + (-7.562183e-04) * (q[i1] * q[i4]) + (-1.755813e-04) * (q[i1] * q[i5]) + (-3.205544e-04) * (q[i1] * q[i6]) + (1.288409e-03) * (q[i1] * q[i7])
            + (2.498898e-04) * (q[i1] * q[i8]) + (-2.535342e-04) * (q[i1] * q[i9]) + (2.920327e-04) * (q[i1] * q[i10]) + (1.495978e-04) * (q[i1] * q[i11])
            + (6.312069e-04) * (q[i1] * q[i12]) + (-1.568737e-04) * (q[i1] * q[i15]) + (1.760141e-05) * (q[i1] * q[i16]) + (-2.081722e-05) * (q[i1] * q[i19])
            + (-3.954753e-04) * (q[i1] * q[i21]) + (-3.753872e-06) * (q[i1] * q[i22]) + (-2.037979e-04) * (q[i2] * q[i3]) + (-4.625191e-04) * (q[i2] * q[i4])
            + (2.157559e-05) * (q[i2] * q[i5]) + (9.333862e-05) * (q[i2] * q[i6]) + (7.850361e-04) * (q[i2] * q[i7]) + (1.423855e-04) * (q[i2] * q[i8])
            + (5.072482e-05) * (q[i2] * q[i9]) + (5.113233e-04) * (q[i2] * q[i10]) + (-3.686003e-04) * (q[i2] * q[i11]) + (1.194359e-03) * (q[i2] * q[i12])
            + (-9.551340e-05) * (q[i2] * q[i15]) + (-6.702515e-05) * (q[i2] * q[i16]) + (4.114824e-06) * (q[i2] * q[i19]) + (-4.167799e-04) * (q[i2] * q[i21])
            + (-2.152634e-04) * (q[i2] * q[i22]) + (4.027741e-04) * (q[i3] * q[i4]) + (-8.921760e-04) * (q[i3] * q[i5]) + (5.647931e-04) * (q[i3] * q[i6])
            + (3.347674e-04) * (q[i3] * q[i7]) + (-5.408535e-05) * (q[i3] * q[i8]) + (9.604014e-05) * (q[i3] * q[i9]) + (2.429382e-04) * (q[i3] * q[i10])
            + (1.218354e-04) * (q[i3] * q[i11]) + (-4.230714e-04) * (q[i3] * q[i12]) + (8.314693e-05) * (q[i3] * q[i15]) + (3.031278e-04) * (q[i3] * q[i16])
            + (1.303709e-04) * (q[i3] * q[i19]) + (-2.150225e-04) * (q[i3] * q[i21]) + (3.425700e-04) * (q[i3] * q[i22]) + (1.081047e-04) * (q[i4] * q[i5])
            + (8.223252e-04) * (q[i4] * q[i6]) + (-5.485476e-05) * (q[i4] * q[i7]) + (1.447929e-04) * (q[i4] * q[i8]) + (-6.932773e-05) * (q[i4] * q[i9])
            + (-1.563696e-04) * (q[i4] * q[i10]) + (2.725329e-04) * (q[i4] * q[i11]) + (-3.911072e-05) * (q[i4] * q[i12]) + (1.892475e-04) * (q[i4] * q[i15])
            + (1.078302e-04) * (q[i4] * q[i16]) + (-1.338756e-04) * (q[i4] * q[i19]) + (2.046500e-04) * (q[i4] * q[i21]) + (-2.295565e-04) * (q[i4] * q[i22])
            + (-3.780468e-04) * (q[i5] * q[i6]) + (-1.750359e-04) * (q[i5] * q[i7]) + (4.501319e-04) * (q[i5] * q[i8]) + (-1.474719e-04) * (q[i5] * q[i9])
            + (-2.241152e-04) * (q[i5] * q[i10]) + (7.149178e-05) * (q[i5] * q[i11]) + (2.474242e-04) * (q[i5] * q[i12]) + (-2.655420e-04) * (q[i5] * q[i15])
            + (6.321716e-04) * (q[i5] * q[i16]) + (4.417112e-06) * (q[i5] * q[i19]) + (-1.819648e-04) * (q[i5] * q[i21]) + (6.460300e-04) * (q[i5] * q[i22])
            + (3.663251e-04) * (q[i6] * q[i7]) + (-1.835712e-04) * (q[i6] * q[i8]) + (-6.404165e-05) * (q[i6] * q[i9]) + (5.322371e-05) * (q[i6] * q[i10])
            + (5.266348e-05) * (q[i6] * q[i11]) + (2.987062e-04) * (q[i6] * q[i12]) + (9.431270e-05) * (q[i6] * q[i15]) + (-1.600898e-04) * (q[i6] * q[i16])
            + (-4.335967e-05) * (q[i6] * q[i19]) + (4.148696e-05) * (q[i6] * q[i21]) + (-2.644389e-04) * (q[i6] * q[i22]) + (-1.430783e-04) * (q[i7] * q[i8])
            + (-1.455974e-04) * (q[i7] * q[i9]) + (1.039055e-04) * (q[i7] * q[i10]) + (1.806968e-04) * (q[i7] * q[i11]) + (2.194537e-04) * (q[i7] * q[i12])
            + (-7.403503e-05) * (q[i7] * q[i15]) + (-2.170218e-04) * (q[i7] * q[i16]) + (-4.366264e-05) * (q[i7] * q[i19]) + (-2.406731e-05) * (q[i7] * q[i21])
            + (-2.534018e-04) * (q[i7] * q[i22]) + (6.850970e-05) * (q[i8] * q[i9]) + (5.437111e-06) * (q[i8] * q[i10]) + (1.391907e-04) * (q[i8] * q[i11])
            + (-4.876793e-04) * (q[i8] * q[i12]) + (1.343018e-05) * (q[i8] * q[i15]) + (2.942755e-04) * (q[i8] * q[i16]) + (-2.221943e-04) * (q[i8] * q[i19])
            + (3.068988e-05) * (q[i8] * q[i21]) + (5.498027e-04) * (q[i8] * q[i22]) + (-3.693607e-05) * (q[i9] * q[i10]) + (2.055716e-04) * (q[i9] * q[i11])
            + (5.071179e-05) * (q[i9] * q[i12]) + (8.511117e-05) * (q[i9] * q[i15]) + (-2.986617e-05) * (q[i9] * q[i16]) + (3.019187e-05) * (q[i9] * q[i19])
            + (-5.544617e-05) * (q[i9] * q[i21]) + (-1.504658e-04) * (q[i9] * q[i22]) + (6.115402e-05) * (q[i10] * q[i11]) + (2.616168e-04) * (q[i10] * q[i12])
            + (3.929254e-05) * (q[i10] * q[i15]) + (-5.567756e-05) * (q[i10] * q[i16]) + (3.192201e-05) * (q[i10] * q[i19])
            + (-7.447196e-05) * (q[i10] * q[i21]) + (-4.256861e-06) * (q[i10] * q[i22]) + (-2.200995e-04) * (q[i11] * q[i12])
            + (-1.157003e-04) * (q[i11] * q[i15]) + (7.699104e-06) * (q[i11] * q[i16]) + (5.760783e-05) * (q[i11] * q[i19])
            + (-1.020223e-04) * (q[i11] * q[i21]) + (-4.386798e-05) * (q[i11] * q[i22]) + (-1.318325e-05) * (q[i12] * q[i15])
            + (2.802190e-04) * (q[i12] * q[i16]) + (-5.788815e-05) * (q[i12] * q[i19]) + (-6.778826e-05) * (q[i12] * q[i21])
            + (8.675031e-05) * (q[i12] * q[i22]) + (6.764808e-05) * (q[i15] * q[i16]) + (-9.041658e-06) * (q[i15] * q[i19]) + (7.900249e-05) * (q[i15] * q[i21])
            + (-8.887574e-06) * (q[i15] * q[i22]) + (-9.552562e-06) * (q[i16] * q[i19]) + (7.053497e-05) * (q[i16] * q[i21])
            + (7.728345e-04) * (q[i16] * q[i22]) + (-2.281735e-05) * (q[i19] * q[i21]) + (2.105748e-05) * (q[i19] * q[i22])
            + (7.142724e-05) * (q[i21] * q[i22]);
      JQ[2][i21] = (3.284135e-03) * (1) + (-7.242032e-04) * ((2) * q[i21]) + (6.470544e-04) * (q[i0]) + (6.795261e-04) * (q[i1]) + (4.279692e-04) * (q[i2])
            + (-8.891952e-04) * (q[i3]) + (-2.048008e-04) * (q[i4]) + (1.153886e-03) * (q[i5]) + (5.154993e-04) * (q[i6]) + (-1.823671e-04) * (q[i7])
            + (-1.221320e-03) * (q[i8]) + (-1.003113e-04) * (q[i9]) + (-5.512975e-04) * (q[i10]) + (-1.516041e-03) * (q[i11]) + (-1.320978e-04) * (q[i12])
            + (7.157948e-04) * (q[i15]) + (1.118701e-04) * (q[i16]) + (1.465471e-03) * (q[i19]) + (2.482349e-04) * (q[i20]) + (-4.247383e-04) * (q[i22])
            + (3.546135e-04) * (q[i0] * q[i0]) + (8.782667e-05) * (q[i1] * q[i1]) + (9.184563e-06) * (q[i2] * q[i2]) + (-1.674498e-04) * (q[i3] * q[i3])
            + (1.172331e-04) * (q[i4] * q[i4]) + (-3.969436e-04) * (q[i5] * q[i5]) + (-1.263081e-04) * (q[i6] * q[i6]) + (-2.268698e-04) * (q[i7] * q[i7])
            + (1.380972e-05) * (q[i8] * q[i8]) + (7.424689e-05) * (q[i9] * q[i9]) + (1.764229e-04) * (q[i10] * q[i10]) + (-4.377671e-05) * (q[i11] * q[i11])
            + (4.809768e-05) * (q[i12] * q[i12]) + (-1.844805e-04) * (q[i15] * q[i15]) + (-9.468958e-05) * (q[i16] * q[i16])
            + (-1.143215e-04) * (q[i19] * q[i19]) + (-3.731444e-05) * (q[i20] * q[i20]) + (6.811269e-05) * ((2) * q[i0] * q[i21])
            + (6.981324e-05) * ((2) * q[i1] * q[i21]) + (3.228096e-04) * ((2) * q[i2] * q[i21]) + (-1.587875e-06) * ((2) * q[i3] * q[i21])
            + (-1.481153e-04) * ((2) * q[i4] * q[i21]) + (-4.019923e-04) * ((2) * q[i5] * q[i21]) + (9.304271e-05) * ((2) * q[i6] * q[i21])
            + (4.622909e-04) * ((2) * q[i7] * q[i21]) + (-9.195516e-04) * ((2) * q[i8] * q[i21]) + (4.402841e-05) * ((2) * q[i9] * q[i21])
            + (5.434041e-05) * ((2) * q[i10] * q[i21]) + (-4.426272e-04) * ((2) * q[i11] * q[i21]) + (-6.658309e-05) * ((2) * q[i12] * q[i21])
            + (-8.546269e-05) * ((2) * q[i15] * q[i21]) + (-5.159170e-05) * ((2) * q[i16] * q[i21]) + (5.550159e-04) * ((2) * q[i19] * q[i21])
            + (-6.873703e-05) * ((2) * q[i20] * q[i21]) + (-8.845787e-05) * ((3) * q[i21] * q[i21]) + (-4.517425e-05) * ((2) * q[i21] * q[i22])
            + (4.528698e-05) * (q[i22] * q[i22]) + (1.137842e-04) * (q[i0] * q[i1]) + (1.027054e-04) * (q[i0] * q[i2]) + (-1.361292e-04) * (q[i0] * q[i3])
            + (-2.597395e-04) * (q[i0] * q[i4]) + (-8.283292e-06) * (q[i0] * q[i5]) + (-1.969670e-04) * (q[i0] * q[i6]) + (-1.596513e-05) * (q[i0] * q[i7])
            + (3.856355e-04) * (q[i0] * q[i8]) + (2.764603e-05) * (q[i0] * q[i9]) + (1.370656e-04) * (q[i0] * q[i10]) + (1.627175e-05) * (q[i0] * q[i11])
            + (8.082817e-05) * (q[i0] * q[i12]) + (3.299489e-04) * (q[i0] * q[i15]) + (3.315816e-05) * (q[i0] * q[i16]) + (-1.606318e-06) * (q[i0] * q[i19])
            + (4.732295e-05) * (q[i0] * q[i20]) + (-2.342242e-04) * (q[i0] * q[i22]) + (6.805839e-04) * (q[i1] * q[i2]) + (6.851692e-04) * (q[i1] * q[i3])
            + (8.366162e-04) * (q[i1] * q[i4]) + (-6.676990e-04) * (q[i1] * q[i5]) + (-4.913701e-04) * (q[i1] * q[i6]) + (1.105648e-05) * (q[i1] * q[i7])
            + (4.661306e-05) * (q[i1] * q[i8]) + (3.084981e-05) * (q[i1] * q[i9]) + (-1.361306e-04) * (q[i1] * q[i10]) + (1.855161e-04) * (q[i1] * q[i11])
            + (-8.990631e-05) * (q[i1] * q[i12]) + (7.180507e-04) * (q[i1] * q[i15]) + (7.808667e-06) * (q[i1] * q[i16]) + (1.391243e-04) * (q[i1] * q[i19])
            + (-3.954753e-04) * (q[i1] * q[i20]) + (2.334830e-04) * (q[i1] * q[i22]) + (-4.673228e-04) * (q[i2] * q[i3]) + (-2.657561e-04) * (q[i2] * q[i4])
            + (7.172603e-04) * (q[i2] * q[i5]) + (2.855538e-05) * (q[i2] * q[i6]) + (8.112065e-05) * (q[i2] * q[i7]) + (4.697497e-04) * (q[i2] * q[i8])
            + (8.193177e-06) * (q[i2] * q[i9]) + (1.997862e-05) * (q[i2] * q[i10]) + (4.179207e-04) * (q[i2] * q[i11]) + (1.978979e-05) * (q[i2] * q[i12])
            + (6.314654e-04) * (q[i2] * q[i15]) + (1.824016e-04) * (q[i2] * q[i16]) + (-2.210598e-04) * (q[i2] * q[i19]) + (-4.167799e-04) * (q[i2] * q[i20])
            + (-6.924897e-07) * (q[i2] * q[i22]) + (-1.784566e-04) * (q[i3] * q[i4]) + (-2.468295e-04) * (q[i3] * q[i5]) + (-2.184872e-05) * (q[i3] * q[i6])
            + (-5.588223e-04) * (q[i3] * q[i7]) + (7.900402e-05) * (q[i3] * q[i8]) + (1.093087e-04) * (q[i3] * q[i9]) + (-1.674627e-04) * (q[i3] * q[i10])
            + (-7.313836e-04) * (q[i3] * q[i11]) + (-3.968007e-05) * (q[i3] * q[i12]) + (-3.249219e-04) * (q[i3] * q[i15]) + (9.856968e-05) * (q[i3] * q[i16])
            + (-2.283146e-04) * (q[i3] * q[i19]) + (-2.150225e-04) * (q[i3] * q[i20]) + (4.145291e-04) * (q[i3] * q[i22]) + (4.077822e-04) * (q[i4] * q[i5])
            + (4.330354e-04) * (q[i4] * q[i6]) + (-4.266081e-04) * (q[i4] * q[i7]) + (4.272613e-04) * (q[i4] * q[i8]) + (-9.635595e-05) * (q[i4] * q[i9])
            + (-4.528867e-05) * (q[i4] * q[i10]) + (-2.377437e-04) * (q[i4] * q[i11]) + (9.036042e-05) * (q[i4] * q[i12]) + (-2.721388e-04) * (q[i4] * q[i15])
            + (-2.327941e-04) * (q[i4] * q[i16]) + (3.439801e-04) * (q[i4] * q[i19]) + (2.046500e-04) * (q[i4] * q[i20]) + (-4.134249e-04) * (q[i4] * q[i22])
            + (-4.964292e-05) * (q[i5] * q[i6]) + (7.739277e-05) * (q[i5] * q[i7]) + (1.165022e-04) * (q[i5] * q[i8]) + (-5.418967e-04) * (q[i5] * q[i9])
            + (-2.214388e-04) * (q[i5] * q[i10]) + (-2.693622e-04) * (q[i5] * q[i11]) + (-1.618050e-05) * (q[i5] * q[i12]) + (7.849938e-04) * (q[i5] * q[i15])
            + (2.034900e-04) * (q[i5] * q[i16]) + (6.504661e-04) * (q[i5] * q[i19]) + (-1.819648e-04) * (q[i5] * q[i20]) + (-3.876848e-06) * (q[i5] * q[i22])
            + (7.693029e-05) * (q[i6] * q[i7]) + (-5.335340e-04) * (q[i6] * q[i8]) + (-1.674403e-04) * (q[i6] * q[i9]) + (-2.659831e-04) * (q[i6] * q[i10])
            + (-1.794102e-04) * (q[i6] * q[i11]) + (-3.023907e-05) * (q[i6] * q[i12]) + (3.049960e-04) * (q[i6] * q[i15]) + (-2.786086e-04) * (q[i6] * q[i16])
            + (2.546358e-04) * (q[i6] * q[i19]) + (4.148696e-05) * (q[i6] * q[i20]) + (-1.885125e-05) * (q[i6] * q[i22]) + (-3.633953e-05) * (q[i7] * q[i8])
            + (9.839484e-05) * (q[i7] * q[i9]) + (2.147215e-04) * (q[i7] * q[i10]) + (-3.003335e-04) * (q[i7] * q[i11]) + (2.042993e-04) * (q[i7] * q[i12])
            + (4.517717e-04) * (q[i7] * q[i15]) + (4.784558e-05) * (q[i7] * q[i16]) + (2.590795e-04) * (q[i7] * q[i19]) + (-2.406731e-05) * (q[i7] * q[i20])
            + (-1.582395e-05) * (q[i7] * q[i22]) + (-1.462135e-04) * (q[i8] * q[i9]) + (1.820278e-04) * (q[i8] * q[i10]) + (7.869919e-06) * (q[i8] * q[i11])
            + (-7.232507e-05) * (q[i8] * q[i12]) + (-8.182625e-04) * (q[i8] * q[i15]) + (-7.242451e-05) * (q[i8] * q[i16]) + (-5.539763e-04) * (q[i8] * q[i19])
            + (3.068988e-05) * (q[i8] * q[i20]) + (-1.903642e-04) * (q[i8] * q[i22]) + (-6.211473e-05) * (q[i9] * q[i10]) + (-5.655255e-05) * (q[i9] * q[i11])
            + (-7.381588e-05) * (q[i9] * q[i12]) + (8.364882e-05) * (q[i9] * q[i15]) + (-4.268491e-05) * (q[i9] * q[i16]) + (6.170221e-06) * (q[i9] * q[i19])
            + (-5.544617e-05) * (q[i9] * q[i20]) + (-1.829503e-05) * (q[i9] * q[i22]) + (-1.222902e-04) * (q[i10] * q[i11]) + (1.687286e-04) * (q[i10] * q[i12])
            + (1.331861e-04) * (q[i10] * q[i15]) + (-1.035764e-04) * (q[i10] * q[i16]) + (1.496858e-04) * (q[i10] * q[i19])
            + (-7.447196e-05) * (q[i10] * q[i20]) + (-1.805391e-05) * (q[i10] * q[i22]) + (3.935233e-06) * (q[i11] * q[i12])
            + (5.362887e-05) * (q[i11] * q[i15]) + (4.936564e-05) * (q[i11] * q[i16]) + (7.847570e-05) * (q[i11] * q[i19]) + (-1.020223e-04) * (q[i11] * q[i20])
            + (-2.507574e-05) * (q[i11] * q[i22]) + (-1.468120e-04) * (q[i12] * q[i15]) + (8.148644e-05) * (q[i12] * q[i16])
            + (-4.247808e-05) * (q[i12] * q[i19]) + (-6.778826e-05) * (q[i12] * q[i20]) + (2.284150e-05) * (q[i12] * q[i22])
            + (1.638822e-04) * (q[i15] * q[i16]) + (-7.785493e-04) * (q[i15] * q[i19]) + (7.900249e-05) * (q[i15] * q[i20])
            + (-7.131537e-05) * (q[i15] * q[i22]) + (9.281681e-06) * (q[i16] * q[i19]) + (7.053497e-05) * (q[i16] * q[i20])
            + (-7.306663e-05) * (q[i16] * q[i22]) + (-2.281735e-05) * (q[i19] * q[i20]) + (7.189642e-05) * (q[i19] * q[i22])
            + (7.142724e-05) * (q[i20] * q[i22]);
      JQ[2][i22] = (-3.288472e-03) * (1) + (-7.308113e-04) * ((2) * q[i22]) + (6.914933e-04) * (q[i0]) + (6.412982e-04) * (q[i1]) + (4.093939e-04) * (q[i2])
            + (-2.091769e-04) * (q[i3]) + (-8.828745e-04) * (q[i4]) + (1.139228e-03) * (q[i5]) + (1.831471e-04) * (q[i6]) + (-5.026373e-04) * (q[i7])
            + (1.217945e-03) * (q[i8]) + (5.577721e-04) * (q[i9]) + (9.916410e-05) * (q[i10]) + (-1.487105e-04) * (q[i11]) + (-1.535922e-03) * (q[i12])
            + (-1.049503e-04) * (q[i15]) + (-7.094236e-04) * (q[i16]) + (-2.425777e-04) * (q[i19]) + (-1.482029e-03) * (q[i20]) + (-4.247383e-04) * (q[i21])
            + (-8.499914e-05) * (q[i0] * q[i0]) + (-3.527509e-04) * (q[i1] * q[i1]) + (-9.068572e-06) * (q[i2] * q[i2]) + (-1.204875e-04) * (q[i3] * q[i3])
            + (1.670597e-04) * (q[i4] * q[i4]) + (3.976714e-04) * (q[i5] * q[i5]) + (2.285076e-04) * (q[i6] * q[i6]) + (1.228869e-04) * (q[i7] * q[i7])
            + (-1.355391e-05) * (q[i8] * q[i8]) + (-1.788403e-04) * (q[i9] * q[i9]) + (-7.383322e-05) * (q[i10] * q[i10]) + (-4.816737e-05) * (q[i11] * q[i11])
            + (5.052362e-05) * (q[i12] * q[i12]) + (9.319952e-05) * (q[i15] * q[i15]) + (1.911788e-04) * (q[i16] * q[i16]) + (3.806140e-05) * (q[i19] * q[i19])
            + (1.148807e-04) * (q[i20] * q[i20]) + (-4.517425e-05) * (q[i21] * q[i21]) + (-7.112266e-05) * ((2) * q[i0] * q[i22])
            + (-6.529956e-05) * ((2) * q[i1] * q[i22]) + (-3.137231e-04) * ((2) * q[i2] * q[i22]) + (1.493617e-04) * ((2) * q[i3] * q[i22])
            + (-1.566640e-06) * ((2) * q[i4] * q[i22]) + (4.060778e-04) * ((2) * q[i5] * q[i22]) + (4.638544e-04) * ((2) * q[i6] * q[i22])
            + (9.348077e-05) * ((2) * q[i7] * q[i22]) + (-9.218395e-04) * ((2) * q[i8] * q[i22]) + (5.344678e-05) * ((2) * q[i9] * q[i22])
            + (4.411843e-05) * ((2) * q[i10] * q[i22]) + (6.576752e-05) * ((2) * q[i11] * q[i22]) + (4.546156e-04) * ((2) * q[i12] * q[i22])
            + (-5.206458e-05) * ((2) * q[i15] * q[i22]) + (-8.863382e-05) * ((2) * q[i16] * q[i22]) + (-6.935476e-05) * ((2) * q[i19] * q[i22])
            + (5.637903e-04) * ((2) * q[i20] * q[i22]) + (4.528698e-05) * ((2) * q[i21] * q[i22]) + (9.090574e-05) * ((3) * q[i22] * q[i22])
            + (-1.097233e-04) * (q[i0] * q[i1]) + (-6.779181e-04) * (q[i0] * q[i2]) + (-8.498490e-04) * (q[i0] * q[i3]) + (-6.792909e-04) * (q[i0] * q[i4])
            + (6.741703e-04) * (q[i0] * q[i5]) + (8.597856e-06) * (q[i0] * q[i6]) + (-4.932519e-04) * (q[i0] * q[i7]) + (5.524433e-05) * (q[i0] * q[i8])
            + (-1.361217e-04) * (q[i0] * q[i9]) + (3.005937e-05) * (q[i0] * q[i10]) + (8.850492e-05) * (q[i0] * q[i11]) + (-1.832422e-04) * (q[i0] * q[i12])
            + (1.040241e-05) * (q[i0] * q[i15]) + (7.259586e-04) * (q[i0] * q[i16]) + (-4.005700e-04) * (q[i0] * q[i19]) + (1.434421e-04) * (q[i0] * q[i20])
            + (-2.342242e-04) * (q[i0] * q[i21]) + (-1.026114e-04) * (q[i1] * q[i2]) + (2.598052e-04) * (q[i1] * q[i3]) + (1.205473e-04) * (q[i1] * q[i4])
            + (1.252290e-05) * (q[i1] * q[i5]) + (-8.847691e-06) * (q[i1] * q[i6]) + (-1.934229e-04) * (q[i1] * q[i7]) + (3.830143e-04) * (q[i1] * q[i8])
            + (1.378833e-04) * (q[i1] * q[i9]) + (3.006953e-05) * (q[i1] * q[i10]) + (-7.675297e-05) * (q[i1] * q[i11]) + (-1.564901e-05) * (q[i1] * q[i12])
            + (3.390483e-05) * (q[i1] * q[i15]) + (3.334762e-04) * (q[i1] * q[i16]) + (4.763852e-05) * (q[i1] * q[i19]) + (-3.753872e-06) * (q[i1] * q[i20])
            + (2.334830e-04) * (q[i1] * q[i21]) + (2.693041e-04) * (q[i2] * q[i3]) + (4.650251e-04) * (q[i2] * q[i4]) + (-7.400227e-04) * (q[i2] * q[i5])
            + (8.983287e-05) * (q[i2] * q[i6]) + (2.716826e-05) * (q[i2] * q[i7]) + (4.659396e-04) * (q[i2] * q[i8]) + (1.973894e-05) * (q[i2] * q[i9])
            + (9.079683e-06) * (q[i2] * q[i10]) + (-2.085002e-05) * (q[i2] * q[i11]) + (-4.119226e-04) * (q[i2] * q[i12]) + (1.855685e-04) * (q[i2] * q[i15])
            + (6.283451e-04) * (q[i2] * q[i16]) + (-4.203312e-04) * (q[i2] * q[i19]) + (-2.152634e-04) * (q[i2] * q[i20]) + (-6.924897e-07) * (q[i2] * q[i21])
            + (1.765279e-04) * (q[i3] * q[i4]) + (-4.020164e-04) * (q[i3] * q[i5]) + (-4.243971e-04) * (q[i3] * q[i6]) + (4.399826e-04) * (q[i3] * q[i7])
            + (4.263331e-04) * (q[i3] * q[i8]) + (-4.633238e-05) * (q[i3] * q[i9]) + (-9.695850e-05) * (q[i3] * q[i10]) + (-8.641867e-05) * (q[i3] * q[i11])
            + (2.359377e-04) * (q[i3] * q[i12]) + (-2.351795e-04) * (q[i3] * q[i15]) + (-2.787869e-04) * (q[i3] * q[i16]) + (2.080684e-04) * (q[i3] * q[i19])
            + (3.425700e-04) * (q[i3] * q[i20]) + (4.145291e-04) * (q[i3] * q[i21]) + (2.557705e-04) * (q[i4] * q[i5]) + (-5.600775e-04) * (q[i4] * q[i6])
            + (-2.061593e-05) * (q[i4] * q[i7]) + (8.697111e-05) * (q[i4] * q[i8]) + (-1.695343e-04) * (q[i4] * q[i9]) + (1.107078e-04) * (q[i4] * q[i10])
            + (3.716635e-05) * (q[i4] * q[i11]) + (7.318424e-04) * (q[i4] * q[i12]) + (9.849923e-05) * (q[i4] * q[i15]) + (-3.290808e-04) * (q[i4] * q[i16])
            + (-2.058941e-04) * (q[i4] * q[i19]) + (-2.295565e-04) * (q[i4] * q[i20]) + (-4.134249e-04) * (q[i4] * q[i21]) + (8.301751e-05) * (q[i5] * q[i6])
            + (-4.801370e-05) * (q[i5] * q[i7]) + (1.116974e-04) * (q[i5] * q[i8]) + (-2.203100e-04) * (q[i5] * q[i9]) + (-5.331750e-04) * (q[i5] * q[i10])
            + (1.497267e-05) * (q[i5] * q[i11]) + (2.787426e-04) * (q[i5] * q[i12]) + (2.021711e-04) * (q[i5] * q[i15]) + (7.961114e-04) * (q[i5] * q[i16])
            + (-1.864600e-04) * (q[i5] * q[i19]) + (6.460300e-04) * (q[i5] * q[i20]) + (-3.876848e-06) * (q[i5] * q[i21]) + (-8.031516e-05) * (q[i6] * q[i7])
            + (3.416559e-05) * (q[i6] * q[i8]) + (-2.159308e-04) * (q[i6] * q[i9]) + (-9.791022e-05) * (q[i6] * q[i10]) + (2.074214e-04) * (q[i6] * q[i11])
            + (-2.987766e-04) * (q[i6] * q[i12]) + (-4.658677e-05) * (q[i6] * q[i15]) + (-4.603793e-04) * (q[i6] * q[i16]) + (2.654591e-05) * (q[i6] * q[i19])
            + (-2.644389e-04) * (q[i6] * q[i20]) + (-1.885125e-05) * (q[i6] * q[i21]) + (5.422506e-04) * (q[i7] * q[i8]) + (2.637625e-04) * (q[i7] * q[i9])
            + (1.668709e-04) * (q[i7] * q[i10]) + (-3.141856e-05) * (q[i7] * q[i11]) + (-1.771800e-04) * (q[i7] * q[i12]) + (2.796963e-04) * (q[i7] * q[i15])
            + (-3.036729e-04) * (q[i7] * q[i16]) + (-4.059630e-05) * (q[i7] * q[i19]) + (-2.534018e-04) * (q[i7] * q[i20]) + (-1.582395e-05) * (q[i7] * q[i21])
            + (-1.807617e-04) * (q[i8] * q[i9]) + (1.433013e-04) * (q[i8] * q[i10]) + (-7.596144e-05) * (q[i8] * q[i11]) + (1.357240e-06) * (q[i8] * q[i12])
            + (7.254698e-05) * (q[i8] * q[i15]) + (8.305395e-04) * (q[i8] * q[i16]) + (-3.164284e-05) * (q[i8] * q[i19]) + (5.498027e-04) * (q[i8] * q[i20])
            + (-1.903642e-04) * (q[i8] * q[i21]) + (6.165970e-05) * (q[i9] * q[i10]) + (1.690360e-04) * (q[i9] * q[i11]) + (-1.232102e-04) * (q[i9] * q[i12])
            + (1.024389e-04) * (q[i9] * q[i15]) + (-1.365421e-04) * (q[i9] * q[i16]) + (7.599913e-05) * (q[i9] * q[i19]) + (-1.504658e-04) * (q[i9] * q[i20])
            + (-1.829503e-05) * (q[i9] * q[i21]) + (-7.195368e-05) * (q[i10] * q[i11]) + (-5.460698e-05) * (q[i10] * q[i12])
            + (4.119188e-05) * (q[i10] * q[i15]) + (-8.357313e-05) * (q[i10] * q[i16]) + (5.384536e-05) * (q[i10] * q[i19])
            + (-4.256861e-06) * (q[i10] * q[i20]) + (-1.805391e-05) * (q[i10] * q[i21]) + (-4.086977e-06) * (q[i11] * q[i12])
            + (7.960830e-05) * (q[i11] * q[i15]) + (-1.514886e-04) * (q[i11] * q[i16]) + (-6.953426e-05) * (q[i11] * q[i19])
            + (-4.386798e-05) * (q[i11] * q[i20]) + (-2.507574e-05) * (q[i11] * q[i21]) + (4.998225e-05) * (q[i12] * q[i15])
            + (3.854881e-05) * (q[i12] * q[i16]) + (-1.049382e-04) * (q[i12] * q[i19]) + (8.675031e-05) * (q[i12] * q[i20]) + (2.284150e-05) * (q[i12] * q[i21])
            + (-1.640155e-04) * (q[i15] * q[i16]) + (-6.918400e-05) * (q[i15] * q[i19]) + (-8.887574e-06) * (q[i15] * q[i20])
            + (-7.131537e-05) * (q[i15] * q[i21]) + (-7.924950e-05) * (q[i16] * q[i19]) + (7.728345e-04) * (q[i16] * q[i20])
            + (-7.306663e-05) * (q[i16] * q[i21]) + (2.105748e-05) * (q[i19] * q[i20]) + (7.189642e-05) * (q[i19] * q[i21])
            + (7.142724e-05) * (q[i20] * q[i21]);
   }

   public void getJQz(double[] q, double[][] JQ)
   {
      JQ[3][i0] = (-1.109815e-01) * (1) + (2.186380e-03) * ((2) * q[i0]) + (1.298186e-05) * (q[i1]) + (1.331273e-03) * (q[i2]) + (-7.220598e-02) * (q[i3])
            + (-3.559331e-02) * (q[i4]) + (-5.699832e-03) * (q[i5]) + (2.130645e-02) * (q[i6]) + (-4.250287e-03) * (q[i7]) + (4.216772e-03) * (q[i8])
            + (-3.284959e-03) * (q[i9]) + (2.338755e-03) * (q[i10]) + (7.367676e-04) * (q[i11]) + (-8.091329e-05) * (q[i12]) + (-9.541826e-03) * (q[i15])
            + (-9.139200e-03) * (q[i16]) + (1.184830e-03) * (q[i19]) + (-1.494506e-03) * (q[i20]) + (-1.608200e-03) * (q[i21]) + (-8.171787e-04) * (q[i22])
            + (3.091546e-03) * ((3) * q[i0] * q[i0]) + (3.516880e-04) * ((2) * q[i0] * q[i1]) + (-1.628473e-03) * ((2) * q[i0] * q[i2])
            + (-2.554699e-03) * ((2) * q[i0] * q[i3]) + (-1.489368e-03) * ((2) * q[i0] * q[i4]) + (-1.309733e-03) * ((2) * q[i0] * q[i5])
            + (-1.297379e-02) * ((2) * q[i0] * q[i6]) + (-8.947360e-04) * ((2) * q[i0] * q[i7]) + (-1.421258e-03) * ((2) * q[i0] * q[i8])
            + (-1.484376e-03) * ((2) * q[i0] * q[i9]) + (-1.081645e-03) * ((2) * q[i0] * q[i10]) + (-9.726582e-04) * ((2) * q[i0] * q[i11])
            + (2.299991e-04) * ((2) * q[i0] * q[i12]) + (2.123896e-04) * ((2) * q[i0] * q[i15]) + (-8.236463e-04) * ((2) * q[i0] * q[i16])
            + (-6.370901e-05) * ((2) * q[i0] * q[i19]) + (1.103751e-03) * ((2) * q[i0] * q[i20]) + (-6.953435e-05) * ((2) * q[i0] * q[i21])
            + (6.636358e-05) * ((2) * q[i0] * q[i22]) + (3.066759e-04) * (q[i1] * q[i1]) + (2.141207e-03) * (q[i2] * q[i2]) + (-5.565556e-03) * (q[i3] * q[i3])
            + (2.936765e-03) * (q[i4] * q[i4]) + (8.127943e-04) * (q[i5] * q[i5]) + (-1.193443e-02) * (q[i6] * q[i6]) + (7.442525e-03) * (q[i7] * q[i7])
            + (1.993809e-03) * (q[i8] * q[i8]) + (3.640691e-03) * (q[i9] * q[i9]) + (-4.561813e-04) * (q[i10] * q[i10]) + (-5.592799e-04) * (q[i11] * q[i11])
            + (1.959393e-06) * (q[i12] * q[i12]) + (-2.262076e-03) * (q[i15] * q[i15]) + (-2.350594e-03) * (q[i16] * q[i16])
            + (1.288298e-04) * (q[i19] * q[i19]) + (-3.383102e-04) * (q[i20] * q[i20]) + (-1.336771e-03) * (q[i21] * q[i21])
            + (-7.529093e-06) * (q[i22] * q[i22]) + (-2.472957e-04) * (q[i1] * q[i2]) + (-2.775416e-03) * (q[i1] * q[i3]) + (-2.774729e-03) * (q[i1] * q[i4])
            + (1.397100e-04) * (q[i1] * q[i5]) + (5.577832e-03) * (q[i1] * q[i6]) + (-5.614291e-03) * (q[i1] * q[i7]) + (1.491949e-05) * (q[i1] * q[i8])
            + (-5.789151e-04) * (q[i1] * q[i9]) + (5.807603e-04) * (q[i1] * q[i10]) + (4.939440e-04) * (q[i1] * q[i11]) + (4.886039e-04) * (q[i1] * q[i12])
            + (-1.055647e-03) * (q[i1] * q[i15]) + (1.063611e-03) * (q[i1] * q[i16]) + (8.518128e-04) * (q[i1] * q[i19]) + (-8.652112e-04) * (q[i1] * q[i20])
            + (4.364268e-04) * (q[i1] * q[i21]) + (4.455604e-04) * (q[i1] * q[i22]) + (1.754163e-03) * (q[i2] * q[i3]) + (-2.729223e-03) * (q[i2] * q[i4])
            + (5.637483e-03) * (q[i2] * q[i5]) + (-9.571732e-03) * (q[i2] * q[i6]) + (-3.346360e-03) * (q[i2] * q[i7]) + (1.531502e-03) * (q[i2] * q[i8])
            + (-1.145506e-03) * (q[i2] * q[i9]) + (-7.645814e-04) * (q[i2] * q[i10]) + (3.099205e-04) * (q[i2] * q[i11]) + (8.559853e-04) * (q[i2] * q[i12])
            + (-1.028915e-04) * (q[i2] * q[i15]) + (-4.698589e-04) * (q[i2] * q[i16]) + (-7.537860e-04) * (q[i2] * q[i19]) + (-8.232012e-04) * (q[i2] * q[i20])
            + (-1.085080e-04) * (q[i2] * q[i21]) + (-3.301443e-04) * (q[i2] * q[i22]) + (6.036308e-03) * (q[i3] * q[i4]) + (-9.588020e-03) * (q[i3] * q[i5])
            + (-1.332572e-02) * (q[i3] * q[i6]) + (-2.780377e-03) * (q[i3] * q[i7]) + (2.669265e-03) * (q[i3] * q[i8]) + (5.972616e-03) * (q[i3] * q[i9])
            + (4.885023e-04) * (q[i3] * q[i10]) + (-3.955337e-04) * (q[i3] * q[i11]) + (-1.783963e-03) * (q[i3] * q[i12]) + (3.448991e-04) * (q[i3] * q[i15])
            + (-3.403611e-03) * (q[i3] * q[i16]) + (-7.449361e-04) * (q[i3] * q[i19]) + (-1.838031e-04) * (q[i3] * q[i20]) + (4.444859e-04) * (q[i3] * q[i21])
            + (-1.090024e-03) * (q[i3] * q[i22]) + (8.595402e-04) * (q[i4] * q[i5]) + (4.970552e-04) * (q[i4] * q[i6]) + (-3.318059e-03) * (q[i4] * q[i7])
            + (2.519785e-03) * (q[i4] * q[i8]) + (4.115776e-03) * (q[i4] * q[i9]) + (2.123625e-03) * (q[i4] * q[i10]) + (-5.837825e-04) * (q[i4] * q[i11])
            + (1.205081e-04) * (q[i4] * q[i12]) + (-3.152945e-03) * (q[i4] * q[i15]) + (-1.142440e-03) * (q[i4] * q[i16]) + (1.462930e-06) * (q[i4] * q[i19])
            + (-1.045449e-04) * (q[i4] * q[i20]) + (5.186598e-04) * (q[i4] * q[i21]) + (-2.030472e-03) * (q[i4] * q[i22]) + (6.621142e-04) * (q[i5] * q[i6])
            + (2.755521e-04) * (q[i5] * q[i7]) + (2.677214e-03) * (q[i5] * q[i8]) + (9.345901e-04) * (q[i5] * q[i9]) + (8.060825e-05) * (q[i5] * q[i10])
            + (4.402940e-04) * (q[i5] * q[i11]) + (1.904859e-04) * (q[i5] * q[i12]) + (3.505028e-03) * (q[i5] * q[i15]) + (-3.198981e-03) * (q[i5] * q[i16])
            + (4.599974e-04) * (q[i5] * q[i19]) + (5.631766e-04) * (q[i5] * q[i20]) + (-9.084618e-04) * (q[i5] * q[i21]) + (-2.016757e-04) * (q[i5] * q[i22])
            + (1.005908e-02) * (q[i6] * q[i7]) + (2.764363e-03) * (q[i6] * q[i8]) + (-7.348027e-03) * (q[i6] * q[i9]) + (4.699217e-03) * (q[i6] * q[i10])
            + (2.115287e-04) * (q[i6] * q[i11]) + (4.855654e-04) * (q[i6] * q[i12]) + (5.780722e-04) * (q[i6] * q[i15]) + (9.414677e-04) * (q[i6] * q[i16])
            + (-5.449188e-04) * (q[i6] * q[i19]) + (8.165496e-04) * (q[i6] * q[i20]) + (1.794742e-03) * (q[i6] * q[i21]) + (6.902083e-04) * (q[i6] * q[i22])
            + (5.301472e-04) * (q[i7] * q[i8]) + (1.497076e-03) * (q[i7] * q[i9]) + (4.225340e-03) * (q[i7] * q[i10]) + (-6.099110e-04) * (q[i7] * q[i11])
            + (2.767898e-04) * (q[i7] * q[i12]) + (7.127504e-04) * (q[i7] * q[i15]) + (-6.352356e-04) * (q[i7] * q[i16]) + (-1.611498e-03) * (q[i7] * q[i19])
            + (-1.202997e-03) * (q[i7] * q[i20]) + (-1.377346e-04) * (q[i7] * q[i21]) + (-2.210035e-04) * (q[i7] * q[i22]) + (5.055425e-06) * (q[i8] * q[i9])
            + (-5.337817e-04) * (q[i8] * q[i10]) + (-4.324909e-04) * (q[i8] * q[i11]) + (-8.340509e-04) * (q[i8] * q[i12]) + (-1.039356e-03) * (q[i8] * q[i15])
            + (-2.300283e-03) * (q[i8] * q[i16]) + (-1.829503e-04) * (q[i8] * q[i19]) + (2.763084e-04) * (q[i8] * q[i20]) + (-3.341720e-05) * (q[i8] * q[i21])
            + (2.267855e-04) * (q[i8] * q[i22]) + (6.205401e-04) * (q[i9] * q[i10]) + (-1.484126e-04) * (q[i9] * q[i11]) + (1.019386e-04) * (q[i9] * q[i12])
            + (3.218487e-05) * (q[i9] * q[i15]) + (2.249269e-05) * (q[i9] * q[i16]) + (4.460672e-05) * (q[i9] * q[i19]) + (1.743876e-05) * (q[i9] * q[i20])
            + (1.217458e-04) * (q[i9] * q[i21]) + (1.699462e-04) * (q[i9] * q[i22]) + (3.592033e-04) * (q[i10] * q[i11]) + (2.376944e-04) * (q[i10] * q[i12])
            + (1.978951e-04) * (q[i10] * q[i15]) + (4.330330e-05) * (q[i10] * q[i16]) + (-5.757650e-04) * (q[i10] * q[i19]) + (3.177573e-04) * (q[i10] * q[i20])
            + (1.299127e-04) * (q[i10] * q[i21]) + (-1.408020e-04) * (q[i10] * q[i22]) + (5.993503e-04) * (q[i11] * q[i12]) + (1.081469e-03) * (q[i11] * q[i15])
            + (1.854720e-04) * (q[i11] * q[i16]) + (2.979992e-04) * (q[i11] * q[i19]) + (-1.001125e-03) * (q[i11] * q[i20]) + (2.566488e-04) * (q[i11] * q[i21])
            + (3.796646e-04) * (q[i11] * q[i22]) + (4.838951e-04) * (q[i12] * q[i15]) + (-1.300320e-03) * (q[i12] * q[i16]) + (3.978069e-05) * (q[i12] * q[i19])
            + (1.015993e-03) * (q[i12] * q[i20]) + (-1.147600e-04) * (q[i12] * q[i21]) + (-5.634708e-04) * (q[i12] * q[i22])
            + (3.132837e-07) * (q[i15] * q[i16]) + (1.295461e-04) * (q[i15] * q[i19]) + (3.283325e-05) * (q[i15] * q[i20]) + (-1.167079e-03) * (q[i15] * q[i21])
            + (-1.190509e-04) * (q[i15] * q[i22]) + (3.027109e-04) * (q[i16] * q[i19]) + (-3.938751e-04) * (q[i16] * q[i20])
            + (-4.024322e-04) * (q[i16] * q[i21]) + (7.784233e-04) * (q[i16] * q[i22]) + (-1.269053e-04) * (q[i19] * q[i20])
            + (-6.506993e-04) * (q[i19] * q[i21]) + (1.077289e-05) * (q[i19] * q[i22]) + (-1.982301e-04) * (q[i20] * q[i21])
            + (2.989038e-04) * (q[i20] * q[i22]) + (2.354593e-04) * (q[i21] * q[i22]);
      JQ[3][i1] = (-1.105000e-01) * (1) + (-2.156062e-03) * ((2) * q[i1]) + (1.298186e-05) * (q[i0]) + (-1.305400e-03) * (q[i2]) + (3.548523e-02) * (q[i3])
            + (7.220449e-02) * (q[i4]) + (5.617093e-03) * (q[i5]) + (-4.244224e-03) * (q[i6]) + (2.127899e-02) * (q[i7]) + (4.222046e-03) * (q[i8])
            + (2.326091e-03) * (q[i9]) + (-3.166851e-03) * (q[i10]) + (1.854969e-04) * (q[i11]) + (-6.749129e-04) * (q[i12]) + (-9.011323e-03) * (q[i15])
            + (-9.624126e-03) * (q[i16]) + (-1.508031e-03) * (q[i19]) + (1.164486e-03) * (q[i20]) + (7.894374e-04) * (q[i21]) + (1.582445e-03) * (q[i22])
            + (3.516880e-04) * (q[i0] * q[i0]) + (3.066759e-04) * ((2) * q[i0] * q[i1]) + (3.085430e-03) * ((3) * q[i1] * q[i1])
            + (-1.600676e-03) * ((2) * q[i1] * q[i2]) + (-1.489914e-03) * ((2) * q[i1] * q[i3]) + (-2.534778e-03) * ((2) * q[i1] * q[i4])
            + (-1.299480e-03) * ((2) * q[i1] * q[i5]) + (9.132872e-04) * ((2) * q[i1] * q[i6]) + (1.300026e-02) * ((2) * q[i1] * q[i7])
            + (1.391285e-03) * ((2) * q[i1] * q[i8]) + (1.083900e-03) * ((2) * q[i1] * q[i9]) + (1.475178e-03) * ((2) * q[i1] * q[i10])
            + (2.308146e-04) * ((2) * q[i1] * q[i11]) + (-9.689934e-04) * ((2) * q[i1] * q[i12]) + (8.221723e-04) * ((2) * q[i1] * q[i15])
            + (-2.117819e-04) * ((2) * q[i1] * q[i16]) + (-1.095282e-03) * ((2) * q[i1] * q[i19]) + (6.888944e-05) * ((2) * q[i1] * q[i20])
            + (7.867434e-05) * ((2) * q[i1] * q[i21]) + (-8.686967e-05) * ((2) * q[i1] * q[i22]) + (2.145444e-03) * (q[i2] * q[i2])
            + (2.957839e-03) * (q[i3] * q[i3]) + (-5.573877e-03) * (q[i4] * q[i4]) + (7.662143e-04) * (q[i5] * q[i5]) + (7.437457e-03) * (q[i6] * q[i6])
            + (-1.192737e-02) * (q[i7] * q[i7]) + (1.982408e-03) * (q[i8] * q[i8]) + (-4.503962e-04) * (q[i9] * q[i9]) + (3.589453e-03) * (q[i10] * q[i10])
            + (1.304813e-05) * (q[i11] * q[i11]) + (-6.008980e-04) * (q[i12] * q[i12]) + (-2.313700e-03) * (q[i15] * q[i15])
            + (-2.272204e-03) * (q[i16] * q[i16]) + (-3.503810e-04) * (q[i19] * q[i19]) + (1.353930e-04) * (q[i20] * q[i20])
            + (-9.860480e-06) * (q[i21] * q[i21]) + (-1.331682e-03) * (q[i22] * q[i22]) + (-2.472957e-04) * (q[i0] * q[i2]) + (-2.775416e-03) * (q[i0] * q[i3])
            + (-2.774729e-03) * (q[i0] * q[i4]) + (1.397100e-04) * (q[i0] * q[i5]) + (5.577832e-03) * (q[i0] * q[i6]) + (-5.614291e-03) * (q[i0] * q[i7])
            + (1.491949e-05) * (q[i0] * q[i8]) + (-5.789151e-04) * (q[i0] * q[i9]) + (5.807603e-04) * (q[i0] * q[i10]) + (4.939440e-04) * (q[i0] * q[i11])
            + (4.886039e-04) * (q[i0] * q[i12]) + (-1.055647e-03) * (q[i0] * q[i15]) + (1.063611e-03) * (q[i0] * q[i16]) + (8.518128e-04) * (q[i0] * q[i19])
            + (-8.652112e-04) * (q[i0] * q[i20]) + (4.364268e-04) * (q[i0] * q[i21]) + (4.455604e-04) * (q[i0] * q[i22]) + (-2.719367e-03) * (q[i2] * q[i3])
            + (1.711951e-03) * (q[i2] * q[i4]) + (5.671669e-03) * (q[i2] * q[i5]) + (3.342473e-03) * (q[i2] * q[i6]) + (9.583511e-03) * (q[i2] * q[i7])
            + (-1.557901e-03) * (q[i2] * q[i8]) + (7.736552e-04) * (q[i2] * q[i9]) + (1.135680e-03) * (q[i2] * q[i10]) + (8.498896e-04) * (q[i2] * q[i11])
            + (3.347439e-04) * (q[i2] * q[i12]) + (4.658728e-04) * (q[i2] * q[i15]) + (1.066038e-04) * (q[i2] * q[i16]) + (8.139548e-04) * (q[i2] * q[i19])
            + (7.591803e-04) * (q[i2] * q[i20]) + (-3.285522e-04) * (q[i2] * q[i21]) + (-1.144321e-04) * (q[i2] * q[i22]) + (6.036870e-03) * (q[i3] * q[i4])
            + (8.587630e-04) * (q[i3] * q[i5]) + (3.224087e-03) * (q[i3] * q[i6]) + (-5.326879e-04) * (q[i3] * q[i7]) + (-2.519406e-03) * (q[i3] * q[i8])
            + (-2.124975e-03) * (q[i3] * q[i9]) + (-4.112262e-03) * (q[i3] * q[i10]) + (7.834184e-05) * (q[i3] * q[i11]) + (-6.064450e-04) * (q[i3] * q[i12])
            + (1.111182e-03) * (q[i3] * q[i15]) + (3.177512e-03) * (q[i3] * q[i16]) + (1.178857e-04) * (q[i3] * q[i19]) + (-6.184635e-07) * (q[i3] * q[i20])
            + (-1.995427e-03) * (q[i3] * q[i21]) + (5.225465e-04) * (q[i3] * q[i22]) + (-9.516510e-03) * (q[i4] * q[i5]) + (2.779804e-03) * (q[i4] * q[i6])
            + (1.346045e-02) * (q[i4] * q[i7]) + (-2.673250e-03) * (q[i4] * q[i8]) + (-4.901910e-04) * (q[i4] * q[i9]) + (-5.909641e-03) * (q[i4] * q[i10])
            + (-1.796502e-03) * (q[i4] * q[i11]) + (-4.375266e-04) * (q[i4] * q[i12]) + (3.354312e-03) * (q[i4] * q[i15]) + (-3.226464e-04) * (q[i4] * q[i16])
            + (1.749162e-04) * (q[i4] * q[i19]) + (7.257399e-04) * (q[i4] * q[i20]) + (-1.075580e-03) * (q[i4] * q[i21]) + (4.590103e-04) * (q[i4] * q[i22])
            + (-2.830539e-04) * (q[i5] * q[i6]) + (-6.403782e-04) * (q[i5] * q[i7]) + (-2.662097e-03) * (q[i5] * q[i8]) + (-6.748973e-05) * (q[i5] * q[i9])
            + (-9.173541e-04) * (q[i5] * q[i10]) + (2.033053e-04) * (q[i5] * q[i11]) + (4.199434e-04) * (q[i5] * q[i12]) + (3.178904e-03) * (q[i5] * q[i15])
            + (-3.545824e-03) * (q[i5] * q[i16]) + (-5.702509e-04) * (q[i5] * q[i19]) + (-4.714147e-04) * (q[i5] * q[i20]) + (-1.942308e-04) * (q[i5] * q[i21])
            + (-8.835536e-04) * (q[i5] * q[i22]) + (1.008619e-02) * (q[i6] * q[i7]) + (5.266155e-04) * (q[i6] * q[i8]) + (4.249609e-03) * (q[i6] * q[i9])
            + (1.493330e-03) * (q[i6] * q[i10]) + (-2.573820e-04) * (q[i6] * q[i11]) + (6.048126e-04) * (q[i6] * q[i12]) + (-6.323197e-04) * (q[i6] * q[i15])
            + (6.999986e-04) * (q[i6] * q[i16]) + (-1.192923e-03) * (q[i6] * q[i19]) + (-1.608027e-03) * (q[i6] * q[i20]) + (2.265587e-04) * (q[i6] * q[i21])
            + (1.453181e-04) * (q[i6] * q[i22]) + (2.738656e-03) * (q[i7] * q[i8]) + (4.749150e-03) * (q[i7] * q[i9]) + (-7.327260e-03) * (q[i7] * q[i10])
            + (-4.702745e-04) * (q[i7] * q[i11]) + (-2.142420e-04) * (q[i7] * q[i12]) + (9.428895e-04) * (q[i7] * q[i15]) + (5.748451e-04) * (q[i7] * q[i16])
            + (8.248072e-04) * (q[i7] * q[i19]) + (-5.374547e-04) * (q[i7] * q[i20]) + (-6.836386e-04) * (q[i7] * q[i21]) + (-1.779180e-03) * (q[i7] * q[i22])
            + (-5.361324e-04) * (q[i8] * q[i9]) + (-3.204259e-06) * (q[i8] * q[i10]) + (8.293673e-04) * (q[i8] * q[i11]) + (4.379223e-04) * (q[i8] * q[i12])
            + (-2.278029e-03) * (q[i8] * q[i15]) + (-1.046418e-03) * (q[i8] * q[i16]) + (2.850417e-04) * (q[i8] * q[i19]) + (-1.726717e-04) * (q[i8] * q[i20])
            + (-2.283093e-04) * (q[i8] * q[i21]) + (1.446450e-05) * (q[i8] * q[i22]) + (6.272113e-04) * (q[i9] * q[i10]) + (-2.220626e-04) * (q[i9] * q[i11])
            + (-3.510685e-04) * (q[i9] * q[i12]) + (4.011463e-05) * (q[i9] * q[i15]) + (1.947114e-04) * (q[i9] * q[i16]) + (3.131939e-04) * (q[i9] * q[i19])
            + (-5.745873e-04) * (q[i9] * q[i20]) + (1.450506e-04) * (q[i9] * q[i21]) + (-1.327543e-04) * (q[i9] * q[i22]) + (-1.021766e-04) * (q[i10] * q[i11])
            + (1.458899e-04) * (q[i10] * q[i12]) + (2.651569e-05) * (q[i10] * q[i15]) + (3.412348e-05) * (q[i10] * q[i16]) + (1.504097e-05) * (q[i10] * q[i19])
            + (4.715358e-05) * (q[i10] * q[i20]) + (-1.679596e-04) * (q[i10] * q[i21]) + (-1.153232e-04) * (q[i10] * q[i22])
            + (5.996905e-04) * (q[i11] * q[i12]) + (1.303593e-03) * (q[i11] * q[i15]) + (-4.838039e-04) * (q[i11] * q[i16])
            + (-1.021959e-03) * (q[i11] * q[i19]) + (-3.769764e-05) * (q[i11] * q[i20]) + (-5.641109e-04) * (q[i11] * q[i21])
            + (-1.102424e-04) * (q[i11] * q[i22]) + (-1.792892e-04) * (q[i12] * q[i15]) + (-1.087714e-03) * (q[i12] * q[i16])
            + (9.876071e-04) * (q[i12] * q[i19]) + (-3.163449e-04) * (q[i12] * q[i20]) + (3.741781e-04) * (q[i12] * q[i21]) + (2.672405e-04) * (q[i12] * q[i22])
            + (-5.778187e-06) * (q[i15] * q[i16]) + (-4.077494e-04) * (q[i15] * q[i19]) + (3.002936e-04) * (q[i15] * q[i20])
            + (-7.800937e-04) * (q[i15] * q[i21]) + (3.931238e-04) * (q[i15] * q[i22]) + (2.687331e-05) * (q[i16] * q[i19]) + (1.281700e-04) * (q[i16] * q[i20])
            + (1.202486e-04) * (q[i16] * q[i21]) + (1.171665e-03) * (q[i16] * q[i22]) + (-1.275865e-04) * (q[i19] * q[i20])
            + (-3.065236e-04) * (q[i19] * q[i21]) + (1.984812e-04) * (q[i19] * q[i22]) + (-1.011552e-05) * (q[i20] * q[i21])
            + (6.525594e-04) * (q[i20] * q[i22]) + (2.246849e-04) * (q[i21] * q[i22]);
      JQ[3][i2] = (1.993570e-01) * (1) + (-1.236494e-06) * ((2) * q[i2]) + (1.331273e-03) * (q[i0]) + (-1.305400e-03) * (q[i1]) + (-2.337207e-02) * (q[i3])
            + (2.327494e-02) * (q[i4]) + (-7.763668e-05) * (q[i5]) + (9.364764e-03) * (q[i6]) + (9.353732e-03) * (q[i7]) + (9.573565e-03) * (q[i8])
            + (1.805297e-03) * (q[i9]) + (1.865104e-03) * (q[i10]) + (1.561177e-03) * (q[i11]) + (-1.403663e-03) * (q[i12]) + (-1.970557e-02) * (q[i15])
            + (-1.993201e-02) * (q[i16]) + (1.275394e-03) * (q[i19]) + (1.266208e-03) * (q[i20]) + (6.967254e-04) * (q[i21]) + (-7.466201e-04) * (q[i22])
            + (-1.628473e-03) * (q[i0] * q[i0]) + (-1.600676e-03) * (q[i1] * q[i1]) + (2.141207e-03) * ((2) * q[i0] * q[i2])
            + (2.145444e-03) * ((2) * q[i1] * q[i2]) + (-2.549352e-03) * ((3) * q[i2] * q[i2]) + (-6.756562e-04) * ((2) * q[i2] * q[i3])
            + (-6.928833e-04) * ((2) * q[i2] * q[i4]) + (5.646531e-03) * ((2) * q[i2] * q[i5]) + (-2.140038e-03) * ((2) * q[i2] * q[i6])
            + (2.125828e-03) * ((2) * q[i2] * q[i7]) + (-7.652074e-06) * ((2) * q[i2] * q[i8]) + (-1.039342e-03) * ((2) * q[i2] * q[i9])
            + (1.028019e-03) * ((2) * q[i2] * q[i10]) + (-1.988575e-04) * ((2) * q[i2] * q[i11]) + (-1.941948e-04) * ((2) * q[i2] * q[i12])
            + (-2.909282e-04) * ((2) * q[i2] * q[i15]) + (2.973533e-04) * ((2) * q[i2] * q[i16]) + (-5.778378e-04) * ((2) * q[i2] * q[i19])
            + (5.646483e-04) * ((2) * q[i2] * q[i20]) + (-3.514461e-05) * ((2) * q[i2] * q[i21]) + (-3.549139e-05) * ((2) * q[i2] * q[i22])
            + (-3.204087e-03) * (q[i3] * q[i3]) + (-3.201652e-03) * (q[i4] * q[i4]) + (-6.251040e-03) * (q[i5] * q[i5]) + (-5.773167e-03) * (q[i6] * q[i6])
            + (-5.751154e-03) * (q[i7] * q[i7]) + (1.203593e-03) * (q[i8] * q[i8]) + (1.087245e-03) * (q[i9] * q[i9]) + (1.063092e-03) * (q[i10] * q[i10])
            + (-6.774947e-04) * (q[i11] * q[i11]) + (-7.279767e-04) * (q[i12] * q[i12]) + (-4.514467e-03) * (q[i15] * q[i15])
            + (-4.559418e-03) * (q[i16] * q[i16]) + (-9.231063e-06) * (q[i19] * q[i19]) + (3.831992e-06) * (q[i20] * q[i20])
            + (-1.364698e-03) * (q[i21] * q[i21]) + (-1.357232e-03) * (q[i22] * q[i22]) + (-2.472957e-04) * (q[i0] * q[i1]) + (1.754163e-03) * (q[i0] * q[i3])
            + (-2.729223e-03) * (q[i0] * q[i4]) + (5.637483e-03) * (q[i0] * q[i5]) + (-9.571732e-03) * (q[i0] * q[i6]) + (-3.346360e-03) * (q[i0] * q[i7])
            + (1.531502e-03) * (q[i0] * q[i8]) + (-1.145506e-03) * (q[i0] * q[i9]) + (-7.645814e-04) * (q[i0] * q[i10]) + (3.099205e-04) * (q[i0] * q[i11])
            + (8.559853e-04) * (q[i0] * q[i12]) + (-1.028915e-04) * (q[i0] * q[i15]) + (-4.698589e-04) * (q[i0] * q[i16]) + (-7.537860e-04) * (q[i0] * q[i19])
            + (-8.232012e-04) * (q[i0] * q[i20]) + (-1.085080e-04) * (q[i0] * q[i21]) + (-3.301443e-04) * (q[i0] * q[i22]) + (-2.719367e-03) * (q[i1] * q[i3])
            + (1.711951e-03) * (q[i1] * q[i4]) + (5.671669e-03) * (q[i1] * q[i5]) + (3.342473e-03) * (q[i1] * q[i6]) + (9.583511e-03) * (q[i1] * q[i7])
            + (-1.557901e-03) * (q[i1] * q[i8]) + (7.736552e-04) * (q[i1] * q[i9]) + (1.135680e-03) * (q[i1] * q[i10]) + (8.498896e-04) * (q[i1] * q[i11])
            + (3.347439e-04) * (q[i1] * q[i12]) + (4.658728e-04) * (q[i1] * q[i15]) + (1.066038e-04) * (q[i1] * q[i16]) + (8.139548e-04) * (q[i1] * q[i19])
            + (7.591803e-04) * (q[i1] * q[i20]) + (-3.285522e-04) * (q[i1] * q[i21]) + (-1.144321e-04) * (q[i1] * q[i22]) + (-1.562891e-03) * (q[i3] * q[i4])
            + (-1.571061e-02) * (q[i3] * q[i5]) + (-4.895144e-03) * (q[i3] * q[i6]) + (-2.724645e-03) * (q[i3] * q[i7]) + (-2.313675e-03) * (q[i3] * q[i8])
            + (1.834456e-03) * (q[i3] * q[i9]) + (-1.719263e-03) * (q[i3] * q[i10]) + (-7.970426e-04) * (q[i3] * q[i11]) + (-1.430941e-03) * (q[i3] * q[i12])
            + (2.462569e-03) * (q[i3] * q[i15]) + (-9.440405e-04) * (q[i3] * q[i16]) + (3.471882e-04) * (q[i3] * q[i19]) + (3.399666e-04) * (q[i3] * q[i20])
            + (-4.242770e-04) * (q[i3] * q[i21]) + (1.596645e-04) * (q[i3] * q[i22]) + (-1.563477e-02) * (q[i4] * q[i5]) + (2.686297e-03) * (q[i4] * q[i6])
            + (4.905160e-03) * (q[i4] * q[i7]) + (2.307384e-03) * (q[i4] * q[i8]) + (1.725919e-03) * (q[i4] * q[i9]) + (-1.793558e-03) * (q[i4] * q[i10])
            + (-1.389473e-03) * (q[i4] * q[i11]) + (-8.046657e-04) * (q[i4] * q[i12]) + (9.279543e-04) * (q[i4] * q[i15]) + (-2.494502e-03) * (q[i4] * q[i16])
            + (-3.463028e-04) * (q[i4] * q[i19]) + (-3.384693e-04) * (q[i4] * q[i20]) + (1.623750e-04) * (q[i4] * q[i21]) + (-4.368106e-04) * (q[i4] * q[i22])
            + (-6.240699e-03) * (q[i5] * q[i6]) + (6.249056e-03) * (q[i5] * q[i7]) + (-1.735343e-06) * (q[i5] * q[i8]) + (2.875813e-03) * (q[i5] * q[i9])
            + (-2.851083e-03) * (q[i5] * q[i10]) + (-7.976358e-04) * (q[i5] * q[i11]) + (-8.642104e-04) * (q[i5] * q[i12]) + (5.182532e-03) * (q[i5] * q[i15])
            + (-5.230026e-03) * (q[i5] * q[i16]) + (-1.092606e-03) * (q[i5] * q[i19]) + (1.088875e-03) * (q[i5] * q[i20]) + (1.804426e-04) * (q[i5] * q[i21])
            + (1.878957e-04) * (q[i5] * q[i22]) + (9.552667e-03) * (q[i6] * q[i7]) + (-5.275663e-03) * (q[i6] * q[i8]) + (-4.297177e-03) * (q[i6] * q[i9])
            + (1.693568e-03) * (q[i6] * q[i10]) + (-9.012448e-04) * (q[i6] * q[i11]) + (-2.685933e-04) * (q[i6] * q[i12]) + (-1.495753e-04) * (q[i6] * q[i15])
            + (-7.048554e-04) * (q[i6] * q[i16]) + (1.051281e-03) * (q[i6] * q[i19]) + (2.332729e-04) * (q[i6] * q[i20]) + (-1.001613e-03) * (q[i6] * q[i21])
            + (-5.206706e-05) * (q[i6] * q[i22]) + (-5.243947e-03) * (q[i7] * q[i8]) + (1.719453e-03) * (q[i7] * q[i9]) + (-4.273448e-03) * (q[i7] * q[i10])
            + (2.739008e-04) * (q[i7] * q[i11]) + (9.051833e-04) * (q[i7] * q[i12]) + (-6.872632e-04) * (q[i7] * q[i15]) + (-1.709576e-04) * (q[i7] * q[i16])
            + (2.468784e-04) * (q[i7] * q[i19]) + (1.040407e-03) * (q[i7] * q[i20]) + (6.353159e-05) * (q[i7] * q[i21]) + (9.994085e-04) * (q[i7] * q[i22])
            + (-1.988901e-03) * (q[i8] * q[i9]) + (-1.960195e-03) * (q[i8] * q[i10]) + (-1.958002e-04) * (q[i8] * q[i11]) + (1.600792e-04) * (q[i8] * q[i12])
            + (-2.168129e-03) * (q[i8] * q[i15]) + (-2.187043e-03) * (q[i8] * q[i16]) + (-7.860911e-04) * (q[i8] * q[i19]) + (-7.718952e-04) * (q[i8] * q[i20])
            + (8.458875e-04) * (q[i8] * q[i21]) + (-8.524062e-04) * (q[i8] * q[i22]) + (-3.042158e-05) * (q[i9] * q[i10]) + (-5.052321e-04) * (q[i9] * q[i11])
            + (-5.898781e-04) * (q[i9] * q[i12]) + (2.126291e-04) * (q[i9] * q[i15]) + (-2.350800e-04) * (q[i9] * q[i16]) + (7.958861e-04) * (q[i9] * q[i19])
            + (-7.146794e-04) * (q[i9] * q[i20]) + (-2.137513e-04) * (q[i9] * q[i21]) + (3.476444e-05) * (q[i9] * q[i22]) + (5.983864e-04) * (q[i10] * q[i11])
            + (5.127200e-04) * (q[i10] * q[i12]) + (-2.269576e-04) * (q[i10] * q[i15]) + (2.115800e-04) * (q[i10] * q[i16])
            + (-7.141847e-04) * (q[i10] * q[i19]) + (7.970314e-04) * (q[i10] * q[i20]) + (-3.824529e-05) * (q[i10] * q[i21])
            + (2.139062e-04) * (q[i10] * q[i22]) + (1.333949e-03) * (q[i11] * q[i12]) + (2.093869e-03) * (q[i11] * q[i15]) + (-4.612780e-04) * (q[i11] * q[i16])
            + (-8.422558e-04) * (q[i11] * q[i19]) + (-3.052536e-04) * (q[i11] * q[i20]) + (-6.046811e-04) * (q[i11] * q[i21])
            + (-1.341003e-04) * (q[i11] * q[i22]) + (4.733281e-04) * (q[i12] * q[i15]) + (-2.107401e-03) * (q[i12] * q[i16])
            + (2.915829e-04) * (q[i12] * q[i19]) + (8.216694e-04) * (q[i12] * q[i20]) + (-1.392476e-04) * (q[i12] * q[i21])
            + (-5.861205e-04) * (q[i12] * q[i22]) + (-2.080770e-04) * (q[i15] * q[i16]) + (-1.433018e-04) * (q[i15] * q[i19])
            + (4.225078e-04) * (q[i15] * q[i20]) + (-1.630412e-03) * (q[i15] * q[i21]) + (-3.246137e-04) * (q[i15] * q[i22])
            + (4.207257e-04) * (q[i16] * q[i19]) + (-1.308262e-04) * (q[i16] * q[i20]) + (3.203257e-04) * (q[i16] * q[i21]) + (1.635022e-03) * (q[i16] * q[i22])
            + (-5.132617e-04) * (q[i19] * q[i20]) + (-1.510791e-03) * (q[i19] * q[i21]) + (-1.495248e-04) * (q[i19] * q[i22])
            + (1.482476e-04) * (q[i20] * q[i21]) + (1.505349e-03) * (q[i20] * q[i22]) + (7.342545e-05) * (q[i21] * q[i22]);
      JQ[3][i3] = (1.557260e-02) * (1) + (-2.297659e-03) * ((2) * q[i3]) + (-7.220598e-02) * (q[i0]) + (3.548523e-02) * (q[i1]) + (-2.337207e-02) * (q[i2])
            + (-5.903601e-05) * (q[i4]) + (-2.861027e-03) * (q[i5]) + (7.357195e-03) * (q[i6]) + (2.883714e-02) * (q[i7]) + (-2.371876e-03) * (q[i8])
            + (1.781216e-03) * (q[i9]) + (1.244188e-02) * (q[i10]) + (-2.343839e-05) * (q[i11]) + (8.395165e-04) * (q[i12]) + (4.887112e-03) * (q[i15])
            + (1.684864e-04) * (q[i16]) + (-2.726175e-03) * (q[i19]) + (3.994531e-03) * (q[i20]) + (4.071305e-03) * (q[i21]) + (3.704013e-04) * (q[i22])
            + (-2.554699e-03) * (q[i0] * q[i0]) + (-1.489914e-03) * (q[i1] * q[i1]) + (-6.756562e-04) * (q[i2] * q[i2])
            + (-5.565556e-03) * ((2) * q[i0] * q[i3]) + (2.957839e-03) * ((2) * q[i1] * q[i3]) + (-3.204087e-03) * ((2) * q[i2] * q[i3])
            + (1.182319e-03) * ((3) * q[i3] * q[i3]) + (1.306708e-03) * ((2) * q[i3] * q[i4]) + (8.595789e-04) * ((2) * q[i3] * q[i5])
            + (8.180236e-03) * ((2) * q[i3] * q[i6]) + (2.244172e-03) * ((2) * q[i3] * q[i7]) + (4.152577e-03) * ((2) * q[i3] * q[i8])
            + (1.871842e-03) * ((2) * q[i3] * q[i9]) + (-1.640236e-04) * ((2) * q[i3] * q[i10]) + (1.521167e-04) * ((2) * q[i3] * q[i11])
            + (-1.310267e-03) * ((2) * q[i3] * q[i12]) + (-1.179013e-03) * ((2) * q[i3] * q[i15]) + (-9.294659e-05) * ((2) * q[i3] * q[i16])
            + (1.117891e-03) * ((2) * q[i3] * q[i19]) + (-6.584318e-04) * ((2) * q[i3] * q[i20]) + (6.239800e-04) * ((2) * q[i3] * q[i21])
            + (2.862283e-04) * ((2) * q[i3] * q[i22]) + (1.289784e-03) * (q[i4] * q[i4]) + (-8.961304e-04) * (q[i5] * q[i5]) + (-9.116238e-03) * (q[i6] * q[i6])
            + (1.026232e-03) * (q[i7] * q[i7]) + (-5.386114e-04) * (q[i8] * q[i8]) + (6.405877e-04) * (q[i9] * q[i9]) + (-1.418208e-03) * (q[i10] * q[i10])
            + (-5.152622e-04) * (q[i11] * q[i11]) + (-9.127081e-04) * (q[i12] * q[i12]) + (5.518776e-05) * (q[i15] * q[i15])
            + (4.668928e-04) * (q[i16] * q[i16]) + (-4.202966e-04) * (q[i19] * q[i19]) + (-2.618625e-04) * (q[i20] * q[i20])
            + (4.875567e-04) * (q[i21] * q[i21]) + (3.564695e-06) * (q[i22] * q[i22]) + (-2.775416e-03) * (q[i0] * q[i1]) + (1.754163e-03) * (q[i0] * q[i2])
            + (6.036308e-03) * (q[i0] * q[i4]) + (-9.588020e-03) * (q[i0] * q[i5]) + (-1.332572e-02) * (q[i0] * q[i6]) + (-2.780377e-03) * (q[i0] * q[i7])
            + (2.669265e-03) * (q[i0] * q[i8]) + (5.972616e-03) * (q[i0] * q[i9]) + (4.885023e-04) * (q[i0] * q[i10]) + (-3.955337e-04) * (q[i0] * q[i11])
            + (-1.783963e-03) * (q[i0] * q[i12]) + (3.448991e-04) * (q[i0] * q[i15]) + (-3.403611e-03) * (q[i0] * q[i16]) + (-7.449361e-04) * (q[i0] * q[i19])
            + (-1.838031e-04) * (q[i0] * q[i20]) + (4.444859e-04) * (q[i0] * q[i21]) + (-1.090024e-03) * (q[i0] * q[i22]) + (-2.719367e-03) * (q[i1] * q[i2])
            + (6.036870e-03) * (q[i1] * q[i4]) + (8.587630e-04) * (q[i1] * q[i5]) + (3.224087e-03) * (q[i1] * q[i6]) + (-5.326879e-04) * (q[i1] * q[i7])
            + (-2.519406e-03) * (q[i1] * q[i8]) + (-2.124975e-03) * (q[i1] * q[i9]) + (-4.112262e-03) * (q[i1] * q[i10]) + (7.834184e-05) * (q[i1] * q[i11])
            + (-6.064450e-04) * (q[i1] * q[i12]) + (1.111182e-03) * (q[i1] * q[i15]) + (3.177512e-03) * (q[i1] * q[i16]) + (1.178857e-04) * (q[i1] * q[i19])
            + (-6.184635e-07) * (q[i1] * q[i20]) + (-1.995427e-03) * (q[i1] * q[i21]) + (5.225465e-04) * (q[i1] * q[i22]) + (-1.562891e-03) * (q[i2] * q[i4])
            + (-1.571061e-02) * (q[i2] * q[i5]) + (-4.895144e-03) * (q[i2] * q[i6]) + (-2.724645e-03) * (q[i2] * q[i7]) + (-2.313675e-03) * (q[i2] * q[i8])
            + (1.834456e-03) * (q[i2] * q[i9]) + (-1.719263e-03) * (q[i2] * q[i10]) + (-7.970426e-04) * (q[i2] * q[i11]) + (-1.430941e-03) * (q[i2] * q[i12])
            + (2.462569e-03) * (q[i2] * q[i15]) + (-9.440405e-04) * (q[i2] * q[i16]) + (3.471882e-04) * (q[i2] * q[i19]) + (3.399666e-04) * (q[i2] * q[i20])
            + (-4.242770e-04) * (q[i2] * q[i21]) + (1.596645e-04) * (q[i2] * q[i22]) + (1.959007e-03) * (q[i4] * q[i5]) + (4.694904e-03) * (q[i4] * q[i6])
            + (-4.638914e-03) * (q[i4] * q[i7]) + (-1.029825e-05) * (q[i4] * q[i8]) + (4.459834e-04) * (q[i4] * q[i9]) + (-4.160438e-04) * (q[i4] * q[i10])
            + (3.414564e-03) * (q[i4] * q[i11]) + (3.431121e-03) * (q[i4] * q[i12]) + (9.870022e-04) * (q[i4] * q[i15]) + (-9.768833e-04) * (q[i4] * q[i16])
            + (-2.270218e-03) * (q[i4] * q[i19]) + (2.276730e-03) * (q[i4] * q[i20]) + (1.190653e-03) * (q[i4] * q[i21]) + (1.195199e-03) * (q[i4] * q[i22])
            + (-6.271087e-03) * (q[i5] * q[i6]) + (-2.619309e-03) * (q[i5] * q[i7]) + (-1.224074e-02) * (q[i5] * q[i8]) + (5.931976e-04) * (q[i5] * q[i9])
            + (1.127468e-03) * (q[i5] * q[i10]) + (-1.901310e-03) * (q[i5] * q[i11]) + (-1.684910e-03) * (q[i5] * q[i12]) + (3.770076e-04) * (q[i5] * q[i15])
            + (2.227935e-03) * (q[i5] * q[i16]) + (-9.872696e-04) * (q[i5] * q[i19]) + (-2.079203e-03) * (q[i5] * q[i20]) + (1.540285e-03) * (q[i5] * q[i21])
            + (-1.739445e-03) * (q[i5] * q[i22]) + (2.388319e-03) * (q[i6] * q[i7]) + (1.302527e-03) * (q[i6] * q[i8]) + (-2.525796e-04) * (q[i6] * q[i9])
            + (1.481257e-03) * (q[i6] * q[i10]) + (-5.606625e-04) * (q[i6] * q[i11]) + (6.150328e-04) * (q[i6] * q[i12]) + (2.190919e-03) * (q[i6] * q[i15])
            + (1.475704e-03) * (q[i6] * q[i16]) + (-2.351642e-03) * (q[i6] * q[i19]) + (-1.440157e-03) * (q[i6] * q[i20]) + (-2.486855e-03) * (q[i6] * q[i21])
            + (1.476708e-04) * (q[i6] * q[i22]) + (-3.731833e-03) * (q[i7] * q[i8]) + (-1.921714e-03) * (q[i7] * q[i9]) + (-2.387680e-03) * (q[i7] * q[i10])
            + (2.047629e-03) * (q[i7] * q[i11]) + (-1.334634e-04) * (q[i7] * q[i12]) + (8.216297e-04) * (q[i7] * q[i15]) + (-9.833456e-05) * (q[i7] * q[i16])
            + (2.829038e-03) * (q[i7] * q[i19]) + (1.855864e-03) * (q[i7] * q[i20]) + (2.294031e-03) * (q[i7] * q[i21]) + (-1.392027e-03) * (q[i7] * q[i22])
            + (-3.652426e-04) * (q[i8] * q[i9]) + (6.855023e-04) * (q[i8] * q[i10]) + (1.223695e-03) * (q[i8] * q[i11]) + (6.380943e-04) * (q[i8] * q[i12])
            + (-1.681315e-03) * (q[i8] * q[i15]) + (7.726770e-04) * (q[i8] * q[i16]) + (1.835695e-03) * (q[i8] * q[i19]) + (-1.592012e-03) * (q[i8] * q[i20])
            + (4.773394e-04) * (q[i8] * q[i21]) + (-1.072421e-04) * (q[i8] * q[i22]) + (-5.715158e-04) * (q[i9] * q[i10]) + (-4.639914e-04) * (q[i9] * q[i11])
            + (-8.660618e-04) * (q[i9] * q[i12]) + (-3.666855e-04) * (q[i9] * q[i15]) + (6.472386e-04) * (q[i9] * q[i16]) + (6.488512e-04) * (q[i9] * q[i19])
            + (-4.009139e-04) * (q[i9] * q[i20]) + (-1.182874e-03) * (q[i9] * q[i21]) + (6.093263e-04) * (q[i9] * q[i22]) + (-2.395844e-04) * (q[i10] * q[i11])
            + (-3.029864e-04) * (q[i10] * q[i12]) + (-2.730054e-04) * (q[i10] * q[i15]) + (-3.540169e-04) * (q[i10] * q[i16])
            + (8.387708e-05) * (q[i10] * q[i19]) + (-6.046693e-04) * (q[i10] * q[i20]) + (2.872147e-04) * (q[i10] * q[i21])
            + (-3.128102e-04) * (q[i10] * q[i22]) + (-6.716568e-04) * (q[i11] * q[i12]) + (-9.556197e-04) * (q[i11] * q[i15])
            + (-8.723860e-04) * (q[i11] * q[i16]) + (-2.145832e-04) * (q[i11] * q[i19]) + (-1.368810e-03) * (q[i11] * q[i20])
            + (-8.578729e-04) * (q[i11] * q[i21]) + (-5.538045e-04) * (q[i11] * q[i22]) + (-2.005103e-04) * (q[i12] * q[i15])
            + (-1.512364e-03) * (q[i12] * q[i16]) + (-1.993557e-03) * (q[i12] * q[i19]) + (-3.306156e-04) * (q[i12] * q[i20])
            + (2.418505e-04) * (q[i12] * q[i21]) + (9.448845e-04) * (q[i12] * q[i22]) + (2.289200e-04) * (q[i15] * q[i16]) + (1.308130e-04) * (q[i15] * q[i19])
            + (4.418298e-04) * (q[i15] * q[i20]) + (2.398466e-04) * (q[i15] * q[i21]) + (1.323404e-04) * (q[i15] * q[i22]) + (4.027790e-06) * (q[i16] * q[i19])
            + (8.806233e-04) * (q[i16] * q[i20]) + (7.408101e-04) * (q[i16] * q[i21]) + (5.767705e-04) * (q[i16] * q[i22]) + (-1.007803e-04) * (q[i19] * q[i20])
            + (9.749011e-04) * (q[i19] * q[i21]) + (-2.244236e-06) * (q[i19] * q[i22]) + (6.515640e-04) * (q[i20] * q[i21]) + (8.760701e-04) * (q[i20] * q[i22])
            + (2.058444e-04) * (q[i21] * q[i22]);
      JQ[3][i4] = (1.556921e-02) * (1) + (2.283697e-03) * ((2) * q[i4]) + (-3.559331e-02) * (q[i0]) + (7.220449e-02) * (q[i1]) + (2.327494e-02) * (q[i2])
            + (-5.903601e-05) * (q[i3]) + (2.906511e-03) * (q[i5]) + (2.883853e-02) * (q[i6]) + (7.226012e-03) * (q[i7]) + (-2.295192e-03) * (q[i8])
            + (1.254799e-02) * (q[i9]) + (1.785154e-03) * (q[i10]) + (-8.096186e-04) * (q[i11]) + (2.268258e-05) * (q[i12]) + (1.618154e-04) * (q[i15])
            + (4.859843e-03) * (q[i16]) + (3.983855e-03) * (q[i19]) + (-2.744435e-03) * (q[i20]) + (-3.607526e-04) * (q[i21]) + (-4.038619e-03) * (q[i22])
            + (-1.489368e-03) * (q[i0] * q[i0]) + (-2.534778e-03) * (q[i1] * q[i1]) + (-6.928833e-04) * (q[i2] * q[i2]) + (1.306708e-03) * (q[i3] * q[i3])
            + (2.936765e-03) * ((2) * q[i0] * q[i4]) + (-5.573877e-03) * ((2) * q[i1] * q[i4]) + (-3.201652e-03) * ((2) * q[i2] * q[i4])
            + (1.289784e-03) * ((2) * q[i3] * q[i4]) + (1.173673e-03) * ((3) * q[i4] * q[i4]) + (8.479721e-04) * ((2) * q[i4] * q[i5])
            + (-2.258470e-03) * ((2) * q[i4] * q[i6]) + (-8.213598e-03) * ((2) * q[i4] * q[i7]) + (-4.135429e-03) * ((2) * q[i4] * q[i8])
            + (1.618644e-04) * ((2) * q[i4] * q[i9]) + (-1.866949e-03) * ((2) * q[i4] * q[i10]) + (-1.317034e-03) * ((2) * q[i4] * q[i11])
            + (1.310350e-04) * ((2) * q[i4] * q[i12]) + (9.697352e-05) * ((2) * q[i4] * q[i15]) + (1.172453e-03) * ((2) * q[i4] * q[i16])
            + (6.466786e-04) * ((2) * q[i4] * q[i19]) + (-1.116481e-03) * ((2) * q[i4] * q[i20]) + (2.800301e-04) * ((2) * q[i4] * q[i21])
            + (6.291117e-04) * ((2) * q[i4] * q[i22]) + (-8.772800e-04) * (q[i5] * q[i5]) + (1.010245e-03) * (q[i6] * q[i6]) + (-9.138727e-03) * (q[i7] * q[i7])
            + (-5.371954e-04) * (q[i8] * q[i8]) + (-1.436498e-03) * (q[i9] * q[i9]) + (5.974724e-04) * (q[i10] * q[i10]) + (-8.731155e-04) * (q[i11] * q[i11])
            + (-4.720268e-04) * (q[i12] * q[i12]) + (4.612167e-04) * (q[i15] * q[i15]) + (5.548368e-05) * (q[i16] * q[i16])
            + (-2.580627e-04) * (q[i19] * q[i19]) + (-4.215867e-04) * (q[i20] * q[i20]) + (4.514942e-06) * (q[i21] * q[i21])
            + (4.814324e-04) * (q[i22] * q[i22]) + (-2.774729e-03) * (q[i0] * q[i1]) + (-2.729223e-03) * (q[i0] * q[i2]) + (6.036308e-03) * (q[i0] * q[i3])
            + (8.595402e-04) * (q[i0] * q[i5]) + (4.970552e-04) * (q[i0] * q[i6]) + (-3.318059e-03) * (q[i0] * q[i7]) + (2.519785e-03) * (q[i0] * q[i8])
            + (4.115776e-03) * (q[i0] * q[i9]) + (2.123625e-03) * (q[i0] * q[i10]) + (-5.837825e-04) * (q[i0] * q[i11]) + (1.205081e-04) * (q[i0] * q[i12])
            + (-3.152945e-03) * (q[i0] * q[i15]) + (-1.142440e-03) * (q[i0] * q[i16]) + (1.462930e-06) * (q[i0] * q[i19]) + (-1.045449e-04) * (q[i0] * q[i20])
            + (5.186598e-04) * (q[i0] * q[i21]) + (-2.030472e-03) * (q[i0] * q[i22]) + (1.711951e-03) * (q[i1] * q[i2]) + (6.036870e-03) * (q[i1] * q[i3])
            + (-9.516510e-03) * (q[i1] * q[i5]) + (2.779804e-03) * (q[i1] * q[i6]) + (1.346045e-02) * (q[i1] * q[i7]) + (-2.673250e-03) * (q[i1] * q[i8])
            + (-4.901910e-04) * (q[i1] * q[i9]) + (-5.909641e-03) * (q[i1] * q[i10]) + (-1.796502e-03) * (q[i1] * q[i11]) + (-4.375266e-04) * (q[i1] * q[i12])
            + (3.354312e-03) * (q[i1] * q[i15]) + (-3.226464e-04) * (q[i1] * q[i16]) + (1.749162e-04) * (q[i1] * q[i19]) + (7.257399e-04) * (q[i1] * q[i20])
            + (-1.075580e-03) * (q[i1] * q[i21]) + (4.590103e-04) * (q[i1] * q[i22]) + (-1.562891e-03) * (q[i2] * q[i3]) + (-1.563477e-02) * (q[i2] * q[i5])
            + (2.686297e-03) * (q[i2] * q[i6]) + (4.905160e-03) * (q[i2] * q[i7]) + (2.307384e-03) * (q[i2] * q[i8]) + (1.725919e-03) * (q[i2] * q[i9])
            + (-1.793558e-03) * (q[i2] * q[i10]) + (-1.389473e-03) * (q[i2] * q[i11]) + (-8.046657e-04) * (q[i2] * q[i12]) + (9.279543e-04) * (q[i2] * q[i15])
            + (-2.494502e-03) * (q[i2] * q[i16]) + (-3.463028e-04) * (q[i2] * q[i19]) + (-3.384693e-04) * (q[i2] * q[i20]) + (1.623750e-04) * (q[i2] * q[i21])
            + (-4.368106e-04) * (q[i2] * q[i22]) + (1.959007e-03) * (q[i3] * q[i5]) + (4.694904e-03) * (q[i3] * q[i6]) + (-4.638914e-03) * (q[i3] * q[i7])
            + (-1.029825e-05) * (q[i3] * q[i8]) + (4.459834e-04) * (q[i3] * q[i9]) + (-4.160438e-04) * (q[i3] * q[i10]) + (3.414564e-03) * (q[i3] * q[i11])
            + (3.431121e-03) * (q[i3] * q[i12]) + (9.870022e-04) * (q[i3] * q[i15]) + (-9.768833e-04) * (q[i3] * q[i16]) + (-2.270218e-03) * (q[i3] * q[i19])
            + (2.276730e-03) * (q[i3] * q[i20]) + (1.190653e-03) * (q[i3] * q[i21]) + (1.195199e-03) * (q[i3] * q[i22]) + (2.613924e-03) * (q[i5] * q[i6])
            + (6.279395e-03) * (q[i5] * q[i7]) + (1.221304e-02) * (q[i5] * q[i8]) + (-1.123052e-03) * (q[i5] * q[i9]) + (-6.099268e-04) * (q[i5] * q[i10])
            + (-1.607223e-03) * (q[i5] * q[i11]) + (-1.913452e-03) * (q[i5] * q[i12]) + (-2.212603e-03) * (q[i5] * q[i15]) + (-3.527507e-04) * (q[i5] * q[i16])
            + (2.051015e-03) * (q[i5] * q[i19]) + (9.827736e-04) * (q[i5] * q[i20]) + (-1.737532e-03) * (q[i5] * q[i21]) + (1.528462e-03) * (q[i5] * q[i22])
            + (2.431741e-03) * (q[i6] * q[i7]) + (-3.741767e-03) * (q[i6] * q[i8]) + (-2.433371e-03) * (q[i6] * q[i9]) + (-1.892674e-03) * (q[i6] * q[i10])
            + (1.127119e-04) * (q[i6] * q[i11]) + (-2.077972e-03) * (q[i6] * q[i12]) + (-1.289673e-04) * (q[i6] * q[i15]) + (8.370493e-04) * (q[i6] * q[i16])
            + (1.840922e-03) * (q[i6] * q[i19]) + (2.832568e-03) * (q[i6] * q[i20]) + (1.400710e-03) * (q[i6] * q[i21]) + (-2.287192e-03) * (q[i6] * q[i22])
            + (1.331434e-03) * (q[i7] * q[i8]) + (1.500690e-03) * (q[i7] * q[i9]) + (-3.217646e-04) * (q[i7] * q[i10]) + (-6.173099e-04) * (q[i7] * q[i11])
            + (5.386931e-04) * (q[i7] * q[i12]) + (1.469082e-03) * (q[i7] * q[i15]) + (2.197919e-03) * (q[i7] * q[i16]) + (-1.443071e-03) * (q[i7] * q[i19])
            + (-2.349355e-03) * (q[i7] * q[i20]) + (-1.470133e-04) * (q[i7] * q[i21]) + (2.475617e-03) * (q[i7] * q[i22]) + (6.757443e-04) * (q[i8] * q[i9])
            + (-3.792716e-04) * (q[i8] * q[i10]) + (-6.633114e-04) * (q[i8] * q[i11]) + (-1.297709e-03) * (q[i8] * q[i12]) + (7.776032e-04) * (q[i8] * q[i15])
            + (-1.691448e-03) * (q[i8] * q[i16]) + (-1.577424e-03) * (q[i8] * q[i19]) + (1.827705e-03) * (q[i8] * q[i20]) + (1.110036e-04) * (q[i8] * q[i21])
            + (-4.623798e-04) * (q[i8] * q[i22]) + (-5.691556e-04) * (q[i9] * q[i10]) + (3.008557e-04) * (q[i9] * q[i11]) + (2.362237e-04) * (q[i9] * q[i12])
            + (-3.479641e-04) * (q[i9] * q[i15]) + (-2.630869e-04) * (q[i9] * q[i16]) + (-6.041358e-04) * (q[i9] * q[i19]) + (9.788540e-05) * (q[i9] * q[i20])
            + (3.185083e-04) * (q[i9] * q[i21]) + (-2.816227e-04) * (q[i9] * q[i22]) + (8.557228e-04) * (q[i10] * q[i11]) + (4.610310e-04) * (q[i10] * q[i12])
            + (6.379221e-04) * (q[i10] * q[i15]) + (-3.588534e-04) * (q[i10] * q[i16]) + (-4.020045e-04) * (q[i10] * q[i19])
            + (6.473136e-04) * (q[i10] * q[i20]) + (-6.081948e-04) * (q[i10] * q[i21]) + (1.174813e-03) * (q[i10] * q[i22])
            + (-6.650560e-04) * (q[i11] * q[i12]) + (1.505921e-03) * (q[i11] * q[i15]) + (2.099040e-04) * (q[i11] * q[i16]) + (3.381856e-04) * (q[i11] * q[i19])
            + (1.983930e-03) * (q[i11] * q[i20]) + (9.509409e-04) * (q[i11] * q[i21]) + (2.386574e-04) * (q[i11] * q[i22]) + (8.698201e-04) * (q[i12] * q[i15])
            + (9.665469e-04) * (q[i12] * q[i16]) + (1.368546e-03) * (q[i12] * q[i19]) + (2.078426e-04) * (q[i12] * q[i20]) + (-5.561184e-04) * (q[i12] * q[i21])
            + (-8.644089e-04) * (q[i12] * q[i22]) + (2.368711e-04) * (q[i15] * q[i16]) + (8.908017e-04) * (q[i15] * q[i19])
            + (-2.271586e-06) * (q[i15] * q[i20]) + (-5.666974e-04) * (q[i15] * q[i21]) + (-7.345003e-04) * (q[i15] * q[i22])
            + (4.364204e-04) * (q[i16] * q[i19]) + (1.314712e-04) * (q[i16] * q[i20]) + (-1.290802e-04) * (q[i16] * q[i21])
            + (-2.341977e-04) * (q[i16] * q[i22]) + (-9.948025e-05) * (q[i19] * q[i20]) + (-8.673770e-04) * (q[i19] * q[i21])
            + (-6.473718e-04) * (q[i19] * q[i22]) + (2.535387e-06) * (q[i20] * q[i21]) + (-9.697348e-04) * (q[i20] * q[i22])
            + (2.030181e-04) * (q[i21] * q[i22]);
      JQ[3][i5] = (-1.256203e-02) * (1) + (-2.374712e-05) * ((2) * q[i5]) + (-5.699832e-03) * (q[i0]) + (5.617093e-03) * (q[i1]) + (-7.763668e-05) * (q[i2])
            + (-2.861027e-03) * (q[i3]) + (2.906511e-03) * (q[i4]) + (2.228666e-02) * (q[i6]) + (2.211254e-02) * (q[i7]) + (4.436577e-02) * (q[i8])
            + (8.223239e-03) * (q[i9]) + (8.111889e-03) * (q[i10]) + (-3.910134e-03) * (q[i11]) + (4.062875e-03) * (q[i12]) + (-3.040717e-03) * (q[i15])
            + (-3.044449e-03) * (q[i16]) + (2.821675e-03) * (q[i19]) + (2.839825e-03) * (q[i20]) + (-1.049549e-03) * (q[i21]) + (1.032526e-03) * (q[i22])
            + (-1.309733e-03) * (q[i0] * q[i0]) + (-1.299480e-03) * (q[i1] * q[i1]) + (5.646531e-03) * (q[i2] * q[i2]) + (8.595789e-04) * (q[i3] * q[i3])
            + (8.479721e-04) * (q[i4] * q[i4]) + (8.127943e-04) * ((2) * q[i0] * q[i5]) + (7.662143e-04) * ((2) * q[i1] * q[i5])
            + (-6.251040e-03) * ((2) * q[i2] * q[i5]) + (-8.961304e-04) * ((2) * q[i3] * q[i5]) + (-8.772800e-04) * ((2) * q[i4] * q[i5])
            + (2.084497e-04) * ((3) * q[i5] * q[i5]) + (-6.685046e-04) * ((2) * q[i5] * q[i6]) + (6.582352e-04) * ((2) * q[i5] * q[i7])
            + (3.616921e-05) * ((2) * q[i5] * q[i8]) + (-1.680639e-03) * ((2) * q[i5] * q[i9]) + (1.669961e-03) * ((2) * q[i5] * q[i10])
            + (1.658584e-03) * ((2) * q[i5] * q[i11]) + (1.766628e-03) * ((2) * q[i5] * q[i12]) + (3.443046e-03) * ((2) * q[i5] * q[i15])
            + (-3.470037e-03) * ((2) * q[i5] * q[i16]) + (-3.159070e-04) * ((2) * q[i5] * q[i19]) + (3.362770e-04) * ((2) * q[i5] * q[i20])
            + (5.841197e-04) * ((2) * q[i5] * q[i21]) + (5.764229e-04) * ((2) * q[i5] * q[i22]) + (7.471263e-03) * (q[i6] * q[i6])
            + (7.479982e-03) * (q[i7] * q[i7]) + (4.209572e-03) * (q[i8] * q[i8]) + (-1.565213e-03) * (q[i9] * q[i9]) + (-1.534836e-03) * (q[i10] * q[i10])
            + (-4.852332e-04) * (q[i11] * q[i11]) + (-4.685750e-04) * (q[i12] * q[i12]) + (-2.098538e-03) * (q[i15] * q[i15])
            + (-2.123313e-03) * (q[i16] * q[i16]) + (1.300974e-03) * (q[i19] * q[i19]) + (1.304845e-03) * (q[i20] * q[i20]) + (1.230496e-04) * (q[i21] * q[i21])
            + (1.300223e-04) * (q[i22] * q[i22]) + (1.397100e-04) * (q[i0] * q[i1]) + (5.637483e-03) * (q[i0] * q[i2]) + (-9.588020e-03) * (q[i0] * q[i3])
            + (8.595402e-04) * (q[i0] * q[i4]) + (6.621142e-04) * (q[i0] * q[i6]) + (2.755521e-04) * (q[i0] * q[i7]) + (2.677214e-03) * (q[i0] * q[i8])
            + (9.345901e-04) * (q[i0] * q[i9]) + (8.060825e-05) * (q[i0] * q[i10]) + (4.402940e-04) * (q[i0] * q[i11]) + (1.904859e-04) * (q[i0] * q[i12])
            + (3.505028e-03) * (q[i0] * q[i15]) + (-3.198981e-03) * (q[i0] * q[i16]) + (4.599974e-04) * (q[i0] * q[i19]) + (5.631766e-04) * (q[i0] * q[i20])
            + (-9.084618e-04) * (q[i0] * q[i21]) + (-2.016757e-04) * (q[i0] * q[i22]) + (5.671669e-03) * (q[i1] * q[i2]) + (8.587630e-04) * (q[i1] * q[i3])
            + (-9.516510e-03) * (q[i1] * q[i4]) + (-2.830539e-04) * (q[i1] * q[i6]) + (-6.403782e-04) * (q[i1] * q[i7]) + (-2.662097e-03) * (q[i1] * q[i8])
            + (-6.748973e-05) * (q[i1] * q[i9]) + (-9.173541e-04) * (q[i1] * q[i10]) + (2.033053e-04) * (q[i1] * q[i11]) + (4.199434e-04) * (q[i1] * q[i12])
            + (3.178904e-03) * (q[i1] * q[i15]) + (-3.545824e-03) * (q[i1] * q[i16]) + (-5.702509e-04) * (q[i1] * q[i19]) + (-4.714147e-04) * (q[i1] * q[i20])
            + (-1.942308e-04) * (q[i1] * q[i21]) + (-8.835536e-04) * (q[i1] * q[i22]) + (-1.571061e-02) * (q[i2] * q[i3]) + (-1.563477e-02) * (q[i2] * q[i4])
            + (-6.240699e-03) * (q[i2] * q[i6]) + (6.249056e-03) * (q[i2] * q[i7]) + (-1.735343e-06) * (q[i2] * q[i8]) + (2.875813e-03) * (q[i2] * q[i9])
            + (-2.851083e-03) * (q[i2] * q[i10]) + (-7.976358e-04) * (q[i2] * q[i11]) + (-8.642104e-04) * (q[i2] * q[i12]) + (5.182532e-03) * (q[i2] * q[i15])
            + (-5.230026e-03) * (q[i2] * q[i16]) + (-1.092606e-03) * (q[i2] * q[i19]) + (1.088875e-03) * (q[i2] * q[i20]) + (1.804426e-04) * (q[i2] * q[i21])
            + (1.878957e-04) * (q[i2] * q[i22]) + (1.959007e-03) * (q[i3] * q[i4]) + (-6.271087e-03) * (q[i3] * q[i6]) + (-2.619309e-03) * (q[i3] * q[i7])
            + (-1.224074e-02) * (q[i3] * q[i8]) + (5.931976e-04) * (q[i3] * q[i9]) + (1.127468e-03) * (q[i3] * q[i10]) + (-1.901310e-03) * (q[i3] * q[i11])
            + (-1.684910e-03) * (q[i3] * q[i12]) + (3.770076e-04) * (q[i3] * q[i15]) + (2.227935e-03) * (q[i3] * q[i16]) + (-9.872696e-04) * (q[i3] * q[i19])
            + (-2.079203e-03) * (q[i3] * q[i20]) + (1.540285e-03) * (q[i3] * q[i21]) + (-1.739445e-03) * (q[i3] * q[i22]) + (2.613924e-03) * (q[i4] * q[i6])
            + (6.279395e-03) * (q[i4] * q[i7]) + (1.221304e-02) * (q[i4] * q[i8]) + (-1.123052e-03) * (q[i4] * q[i9]) + (-6.099268e-04) * (q[i4] * q[i10])
            + (-1.607223e-03) * (q[i4] * q[i11]) + (-1.913452e-03) * (q[i4] * q[i12]) + (-2.212603e-03) * (q[i4] * q[i15]) + (-3.527507e-04) * (q[i4] * q[i16])
            + (2.051015e-03) * (q[i4] * q[i19]) + (9.827736e-04) * (q[i4] * q[i20]) + (-1.737532e-03) * (q[i4] * q[i21]) + (1.528462e-03) * (q[i4] * q[i22])
            + (-5.953039e-03) * (q[i6] * q[i7]) + (1.560364e-03) * (q[i6] * q[i8]) + (-4.389303e-03) * (q[i6] * q[i9]) + (2.530828e-03) * (q[i6] * q[i10])
            + (1.066903e-03) * (q[i6] * q[i11]) + (-1.394643e-04) * (q[i6] * q[i12]) + (-2.992203e-03) * (q[i6] * q[i15]) + (-1.591540e-03) * (q[i6] * q[i16])
            + (2.117505e-03) * (q[i6] * q[i19]) + (-1.682309e-03) * (q[i6] * q[i20]) + (9.124768e-04) * (q[i6] * q[i21]) + (-1.664968e-04) * (q[i6] * q[i22])
            + (1.567247e-03) * (q[i7] * q[i8]) + (2.547313e-03) * (q[i7] * q[i9]) + (-4.329624e-03) * (q[i7] * q[i10]) + (1.694044e-04) * (q[i7] * q[i11])
            + (-1.035892e-03) * (q[i7] * q[i12]) + (-1.588615e-03) * (q[i7] * q[i15]) + (-3.005338e-03) * (q[i7] * q[i16]) + (-1.678776e-03) * (q[i7] * q[i19])
            + (2.085572e-03) * (q[i7] * q[i20]) + (1.582883e-04) * (q[i7] * q[i21]) + (-9.148670e-04) * (q[i7] * q[i22]) + (3.370415e-03) * (q[i8] * q[i9])
            + (3.358581e-03) * (q[i8] * q[i10]) + (-2.579714e-03) * (q[i8] * q[i11]) + (2.749042e-03) * (q[i8] * q[i12]) + (-2.099753e-03) * (q[i8] * q[i15])
            + (-2.120832e-03) * (q[i8] * q[i16]) + (3.321299e-04) * (q[i8] * q[i19]) + (3.494283e-04) * (q[i8] * q[i20]) + (-1.047609e-04) * (q[i8] * q[i21])
            + (6.351661e-05) * (q[i8] * q[i22]) + (1.795244e-03) * (q[i9] * q[i10]) + (-2.721587e-04) * (q[i9] * q[i11]) + (6.765033e-05) * (q[i9] * q[i12])
            + (6.044531e-04) * (q[i9] * q[i15]) + (-8.888490e-05) * (q[i9] * q[i16]) + (-4.683551e-04) * (q[i9] * q[i19]) + (-2.614614e-04) * (q[i9] * q[i20])
            + (5.513357e-04) * (q[i9] * q[i21]) + (-5.134596e-04) * (q[i9] * q[i22]) + (-5.485953e-05) * (q[i10] * q[i11]) + (2.937166e-04) * (q[i10] * q[i12])
            + (-7.002994e-05) * (q[i10] * q[i15]) + (6.086860e-04) * (q[i10] * q[i16]) + (-2.530374e-04) * (q[i10] * q[i19])
            + (-4.645845e-04) * (q[i10] * q[i20]) + (5.093617e-04) * (q[i10] * q[i21]) + (-5.545135e-04) * (q[i10] * q[i22])
            + (1.140986e-03) * (q[i11] * q[i12]) + (-2.709833e-03) * (q[i11] * q[i15]) + (4.243776e-04) * (q[i11] * q[i16])
            + (-1.004278e-03) * (q[i11] * q[i19]) + (-2.755955e-04) * (q[i11] * q[i20]) + (-1.054120e-03) * (q[i11] * q[i21])
            + (4.779538e-04) * (q[i11] * q[i22]) + (-4.287751e-04) * (q[i12] * q[i15]) + (2.717488e-03) * (q[i12] * q[i16]) + (2.775053e-04) * (q[i12] * q[i19])
            + (9.851661e-04) * (q[i12] * q[i20]) + (4.795968e-04) * (q[i12] * q[i21]) + (-1.037047e-03) * (q[i12] * q[i22]) + (1.007694e-03) * (q[i15] * q[i16])
            + (5.705245e-04) * (q[i15] * q[i19]) + (-3.208123e-04) * (q[i15] * q[i20]) + (1.573328e-04) * (q[i15] * q[i21])
            + (-1.446802e-04) * (q[i15] * q[i22]) + (-3.259581e-04) * (q[i16] * q[i19]) + (5.767805e-04) * (q[i16] * q[i20])
            + (1.445475e-04) * (q[i16] * q[i21]) + (-1.700077e-04) * (q[i16] * q[i22]) + (5.075397e-04) * (q[i19] * q[i20]) + (3.316346e-04) * (q[i19] * q[i21])
            + (9.389452e-05) * (q[i19] * q[i22]) + (-9.564362e-05) * (q[i20] * q[i21]) + (-3.390023e-04) * (q[i20] * q[i22])
            + (-1.336004e-04) * (q[i21] * q[i22]);
      JQ[3][i6] = (1.076226e-01) * (1) + (3.803345e-03) * ((2) * q[i6]) + (2.130645e-02) * (q[i0]) + (-4.244224e-03) * (q[i1]) + (9.364764e-03) * (q[i2])
            + (7.357195e-03) * (q[i3]) + (2.883853e-02) * (q[i4]) + (2.228666e-02) * (q[i5]) + (1.644721e-06) * (q[i7]) + (-5.708182e-03) * (q[i8])
            + (-5.801340e-03) * (q[i9]) + (-1.367266e-03) * (q[i10]) + (9.259903e-04) * (q[i11]) + (1.502845e-03) * (q[i12]) + (5.385031e-04) * (q[i15])
            + (9.371454e-03) * (q[i16]) + (-7.744288e-04) * (q[i19]) + (-2.307807e-03) * (q[i20]) + (-1.719988e-03) * (q[i21]) + (-2.084573e-04) * (q[i22])
            + (-1.297379e-02) * (q[i0] * q[i0]) + (9.132872e-04) * (q[i1] * q[i1]) + (-2.140038e-03) * (q[i2] * q[i2]) + (8.180236e-03) * (q[i3] * q[i3])
            + (-2.258470e-03) * (q[i4] * q[i4]) + (-6.685046e-04) * (q[i5] * q[i5]) + (-1.193443e-02) * ((2) * q[i0] * q[i6])
            + (7.437457e-03) * ((2) * q[i1] * q[i6]) + (-5.773167e-03) * ((2) * q[i2] * q[i6]) + (-9.116238e-03) * ((2) * q[i3] * q[i6])
            + (1.010245e-03) * ((2) * q[i4] * q[i6]) + (7.471263e-03) * ((2) * q[i5] * q[i6]) + (-4.578857e-03) * ((3) * q[i6] * q[i6])
            + (5.240790e-03) * ((2) * q[i6] * q[i7]) + (4.802060e-03) * ((2) * q[i6] * q[i8]) + (-9.200232e-04) * ((2) * q[i6] * q[i9])
            + (2.026387e-03) * ((2) * q[i6] * q[i10]) + (-1.200776e-03) * ((2) * q[i6] * q[i11]) + (5.813818e-04) * ((2) * q[i6] * q[i12])
            + (1.371895e-03) * ((2) * q[i6] * q[i15]) + (7.176423e-04) * ((2) * q[i6] * q[i16]) + (-7.310683e-04) * ((2) * q[i6] * q[i19])
            + (-8.019595e-04) * ((2) * q[i6] * q[i20]) + (-1.001196e-04) * ((2) * q[i6] * q[i21]) + (-1.661576e-03) * ((2) * q[i6] * q[i22])
            + (-5.244531e-03) * (q[i7] * q[i7]) + (-1.768166e-03) * (q[i8] * q[i8]) + (-2.235646e-03) * (q[i9] * q[i9]) + (2.415019e-04) * (q[i10] * q[i10])
            + (1.388729e-03) * (q[i11] * q[i11]) + (1.156984e-03) * (q[i12] * q[i12]) + (1.037642e-03) * (q[i15] * q[i15]) + (1.226560e-03) * (q[i16] * q[i16])
            + (5.782373e-04) * (q[i19] * q[i19]) + (3.643780e-04) * (q[i20] * q[i20]) + (6.467126e-04) * (q[i21] * q[i21]) + (7.316595e-04) * (q[i22] * q[i22])
            + (5.577832e-03) * (q[i0] * q[i1]) + (-9.571732e-03) * (q[i0] * q[i2]) + (-1.332572e-02) * (q[i0] * q[i3]) + (4.970552e-04) * (q[i0] * q[i4])
            + (6.621142e-04) * (q[i0] * q[i5]) + (1.005908e-02) * (q[i0] * q[i7]) + (2.764363e-03) * (q[i0] * q[i8]) + (-7.348027e-03) * (q[i0] * q[i9])
            + (4.699217e-03) * (q[i0] * q[i10]) + (2.115287e-04) * (q[i0] * q[i11]) + (4.855654e-04) * (q[i0] * q[i12]) + (5.780722e-04) * (q[i0] * q[i15])
            + (9.414677e-04) * (q[i0] * q[i16]) + (-5.449188e-04) * (q[i0] * q[i19]) + (8.165496e-04) * (q[i0] * q[i20]) + (1.794742e-03) * (q[i0] * q[i21])
            + (6.902083e-04) * (q[i0] * q[i22]) + (3.342473e-03) * (q[i1] * q[i2]) + (3.224087e-03) * (q[i1] * q[i3]) + (2.779804e-03) * (q[i1] * q[i4])
            + (-2.830539e-04) * (q[i1] * q[i5]) + (1.008619e-02) * (q[i1] * q[i7]) + (5.266155e-04) * (q[i1] * q[i8]) + (4.249609e-03) * (q[i1] * q[i9])
            + (1.493330e-03) * (q[i1] * q[i10]) + (-2.573820e-04) * (q[i1] * q[i11]) + (6.048126e-04) * (q[i1] * q[i12]) + (-6.323197e-04) * (q[i1] * q[i15])
            + (6.999986e-04) * (q[i1] * q[i16]) + (-1.192923e-03) * (q[i1] * q[i19]) + (-1.608027e-03) * (q[i1] * q[i20]) + (2.265587e-04) * (q[i1] * q[i21])
            + (1.453181e-04) * (q[i1] * q[i22]) + (-4.895144e-03) * (q[i2] * q[i3]) + (2.686297e-03) * (q[i2] * q[i4]) + (-6.240699e-03) * (q[i2] * q[i5])
            + (9.552667e-03) * (q[i2] * q[i7]) + (-5.275663e-03) * (q[i2] * q[i8]) + (-4.297177e-03) * (q[i2] * q[i9]) + (1.693568e-03) * (q[i2] * q[i10])
            + (-9.012448e-04) * (q[i2] * q[i11]) + (-2.685933e-04) * (q[i2] * q[i12]) + (-1.495753e-04) * (q[i2] * q[i15]) + (-7.048554e-04) * (q[i2] * q[i16])
            + (1.051281e-03) * (q[i2] * q[i19]) + (2.332729e-04) * (q[i2] * q[i20]) + (-1.001613e-03) * (q[i2] * q[i21]) + (-5.206706e-05) * (q[i2] * q[i22])
            + (4.694904e-03) * (q[i3] * q[i4]) + (-6.271087e-03) * (q[i3] * q[i5]) + (2.388319e-03) * (q[i3] * q[i7]) + (1.302527e-03) * (q[i3] * q[i8])
            + (-2.525796e-04) * (q[i3] * q[i9]) + (1.481257e-03) * (q[i3] * q[i10]) + (-5.606625e-04) * (q[i3] * q[i11]) + (6.150328e-04) * (q[i3] * q[i12])
            + (2.190919e-03) * (q[i3] * q[i15]) + (1.475704e-03) * (q[i3] * q[i16]) + (-2.351642e-03) * (q[i3] * q[i19]) + (-1.440157e-03) * (q[i3] * q[i20])
            + (-2.486855e-03) * (q[i3] * q[i21]) + (1.476708e-04) * (q[i3] * q[i22]) + (2.613924e-03) * (q[i4] * q[i5]) + (2.431741e-03) * (q[i4] * q[i7])
            + (-3.741767e-03) * (q[i4] * q[i8]) + (-2.433371e-03) * (q[i4] * q[i9]) + (-1.892674e-03) * (q[i4] * q[i10]) + (1.127119e-04) * (q[i4] * q[i11])
            + (-2.077972e-03) * (q[i4] * q[i12]) + (-1.289673e-04) * (q[i4] * q[i15]) + (8.370493e-04) * (q[i4] * q[i16]) + (1.840922e-03) * (q[i4] * q[i19])
            + (2.832568e-03) * (q[i4] * q[i20]) + (1.400710e-03) * (q[i4] * q[i21]) + (-2.287192e-03) * (q[i4] * q[i22]) + (-5.953039e-03) * (q[i5] * q[i7])
            + (1.560364e-03) * (q[i5] * q[i8]) + (-4.389303e-03) * (q[i5] * q[i9]) + (2.530828e-03) * (q[i5] * q[i10]) + (1.066903e-03) * (q[i5] * q[i11])
            + (-1.394643e-04) * (q[i5] * q[i12]) + (-2.992203e-03) * (q[i5] * q[i15]) + (-1.591540e-03) * (q[i5] * q[i16]) + (2.117505e-03) * (q[i5] * q[i19])
            + (-1.682309e-03) * (q[i5] * q[i20]) + (9.124768e-04) * (q[i5] * q[i21]) + (-1.664968e-04) * (q[i5] * q[i22]) + (-1.143098e-05) * (q[i7] * q[i8])
            + (2.577368e-03) * (q[i7] * q[i9]) + (-2.564323e-03) * (q[i7] * q[i10]) + (-8.060847e-04) * (q[i7] * q[i11]) + (-8.094762e-04) * (q[i7] * q[i12])
            + (-1.212214e-03) * (q[i7] * q[i15]) + (1.210781e-03) * (q[i7] * q[i16]) + (8.441643e-04) * (q[i7] * q[i19]) + (-8.377542e-04) * (q[i7] * q[i20])
            + (-3.850392e-04) * (q[i7] * q[i21]) + (-3.897134e-04) * (q[i7] * q[i22]) + (4.222792e-04) * (q[i8] * q[i9]) + (-3.622831e-04) * (q[i8] * q[i10])
            + (1.840413e-03) * (q[i8] * q[i11]) + (5.693488e-04) * (q[i8] * q[i12]) + (6.057846e-04) * (q[i8] * q[i15]) + (-2.253623e-03) * (q[i8] * q[i16])
            + (-2.846423e-04) * (q[i8] * q[i19]) + (1.106294e-03) * (q[i8] * q[i20]) + (-1.004595e-03) * (q[i8] * q[i21]) + (1.344199e-03) * (q[i8] * q[i22])
            + (8.809378e-04) * (q[i9] * q[i10]) + (-5.482627e-04) * (q[i9] * q[i11]) + (-6.646076e-04) * (q[i9] * q[i12]) + (5.592634e-04) * (q[i9] * q[i15])
            + (-4.138171e-04) * (q[i9] * q[i16]) + (-5.808050e-04) * (q[i9] * q[i19]) + (8.196789e-04) * (q[i9] * q[i20]) + (2.981797e-04) * (q[i9] * q[i21])
            + (-7.568454e-04) * (q[i9] * q[i22]) + (7.645532e-04) * (q[i10] * q[i11]) + (-4.659777e-05) * (q[i10] * q[i12]) + (4.928168e-05) * (q[i10] * q[i15])
            + (5.564520e-04) * (q[i10] * q[i16]) + (1.102690e-03) * (q[i10] * q[i19]) + (4.810234e-04) * (q[i10] * q[i20]) + (7.280991e-04) * (q[i10] * q[i21])
            + (1.866174e-04) * (q[i10] * q[i22]) + (-2.188075e-03) * (q[i11] * q[i12]) + (-8.049555e-05) * (q[i11] * q[i15])
            + (1.236653e-03) * (q[i11] * q[i16]) + (5.555949e-05) * (q[i11] * q[i19]) + (-6.663878e-04) * (q[i11] * q[i20]) + (8.746138e-04) * (q[i11] * q[i21])
            + (2.380237e-04) * (q[i11] * q[i22]) + (-9.200743e-04) * (q[i12] * q[i15]) + (4.833148e-04) * (q[i12] * q[i16]) + (6.442033e-04) * (q[i12] * q[i19])
            + (-7.282235e-04) * (q[i12] * q[i20]) + (1.442067e-04) * (q[i12] * q[i21]) + (2.225201e-04) * (q[i12] * q[i22]) + (7.888703e-04) * (q[i15] * q[i16])
            + (6.141175e-05) * (q[i15] * q[i19]) + (-2.685348e-04) * (q[i15] * q[i20]) + (-1.138220e-03) * (q[i15] * q[i21])
            + (5.901552e-04) * (q[i15] * q[i22]) + (8.896395e-05) * (q[i16] * q[i19]) + (6.421954e-04) * (q[i16] * q[i20]) + (2.359060e-04) * (q[i16] * q[i21])
            + (2.283668e-05) * (q[i16] * q[i22]) + (6.021812e-04) * (q[i19] * q[i20]) + (-5.092972e-04) * (q[i19] * q[i21]) + (4.299740e-04) * (q[i19] * q[i22])
            + (-4.971938e-04) * (q[i20] * q[i21]) + (-8.179393e-04) * (q[i20] * q[i22]) + (3.492630e-04) * (q[i21] * q[i22]);
      JQ[3][i7] = (-1.073752e-01) * (1) + (-3.797777e-03) * ((2) * q[i7]) + (-4.250287e-03) * (q[i0]) + (2.127899e-02) * (q[i1]) + (9.353732e-03) * (q[i2])
            + (2.883714e-02) * (q[i3]) + (7.226012e-03) * (q[i4]) + (2.211254e-02) * (q[i5]) + (1.644721e-06) * (q[i6]) + (5.638196e-03) * (q[i8])
            + (1.361736e-03) * (q[i9]) + (5.718622e-03) * (q[i10]) + (1.568874e-03) * (q[i11]) + (9.662553e-04) * (q[i12]) + (-9.219974e-03) * (q[i15])
            + (-5.866709e-04) * (q[i16]) + (2.318131e-03) * (q[i19]) + (7.661167e-04) * (q[i20]) + (-2.265384e-04) * (q[i21]) + (-1.728481e-03) * (q[i22])
            + (-8.947360e-04) * (q[i0] * q[i0]) + (1.300026e-02) * (q[i1] * q[i1]) + (2.125828e-03) * (q[i2] * q[i2]) + (2.244172e-03) * (q[i3] * q[i3])
            + (-8.213598e-03) * (q[i4] * q[i4]) + (6.582352e-04) * (q[i5] * q[i5]) + (5.240790e-03) * (q[i6] * q[i6]) + (7.442525e-03) * ((2) * q[i0] * q[i7])
            + (-1.192737e-02) * ((2) * q[i1] * q[i7]) + (-5.751154e-03) * ((2) * q[i2] * q[i7]) + (1.026232e-03) * ((2) * q[i3] * q[i7])
            + (-9.138727e-03) * ((2) * q[i4] * q[i7]) + (7.479982e-03) * ((2) * q[i5] * q[i7]) + (-5.244531e-03) * ((2) * q[i6] * q[i7])
            + (4.588625e-03) * ((3) * q[i7] * q[i7]) + (-4.807721e-03) * ((2) * q[i7] * q[i8]) + (-2.036214e-03) * ((2) * q[i7] * q[i9])
            + (9.137079e-04) * ((2) * q[i7] * q[i10]) + (5.780431e-04) * ((2) * q[i7] * q[i11]) + (-1.196945e-03) * ((2) * q[i7] * q[i12])
            + (-7.038177e-04) * ((2) * q[i7] * q[i15]) + (-1.368118e-03) * ((2) * q[i7] * q[i16]) + (8.031525e-04) * ((2) * q[i7] * q[i19])
            + (7.309260e-04) * ((2) * q[i7] * q[i20]) + (-1.652416e-03) * ((2) * q[i7] * q[i21]) + (-1.002680e-04) * ((2) * q[i7] * q[i22])
            + (1.768702e-03) * (q[i8] * q[i8]) + (-2.395308e-04) * (q[i9] * q[i9]) + (2.215410e-03) * (q[i10] * q[i10]) + (-1.134499e-03) * (q[i11] * q[i11])
            + (-1.382291e-03) * (q[i12] * q[i12]) + (-1.213451e-03) * (q[i15] * q[i15]) + (-1.050098e-03) * (q[i16] * q[i16])
            + (-3.725240e-04) * (q[i19] * q[i19]) + (-5.816897e-04) * (q[i20] * q[i20]) + (-7.341577e-04) * (q[i21] * q[i21])
            + (-6.525707e-04) * (q[i22] * q[i22]) + (-5.614291e-03) * (q[i0] * q[i1]) + (-3.346360e-03) * (q[i0] * q[i2]) + (-2.780377e-03) * (q[i0] * q[i3])
            + (-3.318059e-03) * (q[i0] * q[i4]) + (2.755521e-04) * (q[i0] * q[i5]) + (1.005908e-02) * (q[i0] * q[i6]) + (5.301472e-04) * (q[i0] * q[i8])
            + (1.497076e-03) * (q[i0] * q[i9]) + (4.225340e-03) * (q[i0] * q[i10]) + (-6.099110e-04) * (q[i0] * q[i11]) + (2.767898e-04) * (q[i0] * q[i12])
            + (7.127504e-04) * (q[i0] * q[i15]) + (-6.352356e-04) * (q[i0] * q[i16]) + (-1.611498e-03) * (q[i0] * q[i19]) + (-1.202997e-03) * (q[i0] * q[i20])
            + (-1.377346e-04) * (q[i0] * q[i21]) + (-2.210035e-04) * (q[i0] * q[i22]) + (9.583511e-03) * (q[i1] * q[i2]) + (-5.326879e-04) * (q[i1] * q[i3])
            + (1.346045e-02) * (q[i1] * q[i4]) + (-6.403782e-04) * (q[i1] * q[i5]) + (1.008619e-02) * (q[i1] * q[i6]) + (2.738656e-03) * (q[i1] * q[i8])
            + (4.749150e-03) * (q[i1] * q[i9]) + (-7.327260e-03) * (q[i1] * q[i10]) + (-4.702745e-04) * (q[i1] * q[i11]) + (-2.142420e-04) * (q[i1] * q[i12])
            + (9.428895e-04) * (q[i1] * q[i15]) + (5.748451e-04) * (q[i1] * q[i16]) + (8.248072e-04) * (q[i1] * q[i19]) + (-5.374547e-04) * (q[i1] * q[i20])
            + (-6.836386e-04) * (q[i1] * q[i21]) + (-1.779180e-03) * (q[i1] * q[i22]) + (-2.724645e-03) * (q[i2] * q[i3]) + (4.905160e-03) * (q[i2] * q[i4])
            + (6.249056e-03) * (q[i2] * q[i5]) + (9.552667e-03) * (q[i2] * q[i6]) + (-5.243947e-03) * (q[i2] * q[i8]) + (1.719453e-03) * (q[i2] * q[i9])
            + (-4.273448e-03) * (q[i2] * q[i10]) + (2.739008e-04) * (q[i2] * q[i11]) + (9.051833e-04) * (q[i2] * q[i12]) + (-6.872632e-04) * (q[i2] * q[i15])
            + (-1.709576e-04) * (q[i2] * q[i16]) + (2.468784e-04) * (q[i2] * q[i19]) + (1.040407e-03) * (q[i2] * q[i20]) + (6.353159e-05) * (q[i2] * q[i21])
            + (9.994085e-04) * (q[i2] * q[i22]) + (-4.638914e-03) * (q[i3] * q[i4]) + (-2.619309e-03) * (q[i3] * q[i5]) + (2.388319e-03) * (q[i3] * q[i6])
            + (-3.731833e-03) * (q[i3] * q[i8]) + (-1.921714e-03) * (q[i3] * q[i9]) + (-2.387680e-03) * (q[i3] * q[i10]) + (2.047629e-03) * (q[i3] * q[i11])
            + (-1.334634e-04) * (q[i3] * q[i12]) + (8.216297e-04) * (q[i3] * q[i15]) + (-9.833456e-05) * (q[i3] * q[i16]) + (2.829038e-03) * (q[i3] * q[i19])
            + (1.855864e-03) * (q[i3] * q[i20]) + (2.294031e-03) * (q[i3] * q[i21]) + (-1.392027e-03) * (q[i3] * q[i22]) + (6.279395e-03) * (q[i4] * q[i5])
            + (2.431741e-03) * (q[i4] * q[i6]) + (1.331434e-03) * (q[i4] * q[i8]) + (1.500690e-03) * (q[i4] * q[i9]) + (-3.217646e-04) * (q[i4] * q[i10])
            + (-6.173099e-04) * (q[i4] * q[i11]) + (5.386931e-04) * (q[i4] * q[i12]) + (1.469082e-03) * (q[i4] * q[i15]) + (2.197919e-03) * (q[i4] * q[i16])
            + (-1.443071e-03) * (q[i4] * q[i19]) + (-2.349355e-03) * (q[i4] * q[i20]) + (-1.470133e-04) * (q[i4] * q[i21]) + (2.475617e-03) * (q[i4] * q[i22])
            + (-5.953039e-03) * (q[i5] * q[i6]) + (1.567247e-03) * (q[i5] * q[i8]) + (2.547313e-03) * (q[i5] * q[i9]) + (-4.329624e-03) * (q[i5] * q[i10])
            + (1.694044e-04) * (q[i5] * q[i11]) + (-1.035892e-03) * (q[i5] * q[i12]) + (-1.588615e-03) * (q[i5] * q[i15]) + (-3.005338e-03) * (q[i5] * q[i16])
            + (-1.678776e-03) * (q[i5] * q[i19]) + (2.085572e-03) * (q[i5] * q[i20]) + (1.582883e-04) * (q[i5] * q[i21]) + (-9.148670e-04) * (q[i5] * q[i22])
            + (-1.143098e-05) * (q[i6] * q[i8]) + (2.577368e-03) * (q[i6] * q[i9]) + (-2.564323e-03) * (q[i6] * q[i10]) + (-8.060847e-04) * (q[i6] * q[i11])
            + (-8.094762e-04) * (q[i6] * q[i12]) + (-1.212214e-03) * (q[i6] * q[i15]) + (1.210781e-03) * (q[i6] * q[i16]) + (8.441643e-04) * (q[i6] * q[i19])
            + (-8.377542e-04) * (q[i6] * q[i20]) + (-3.850392e-04) * (q[i6] * q[i21]) + (-3.897134e-04) * (q[i6] * q[i22]) + (3.671713e-04) * (q[i8] * q[i9])
            + (-4.161039e-04) * (q[i8] * q[i10]) + (5.344024e-04) * (q[i8] * q[i11]) + (1.838683e-03) * (q[i8] * q[i12]) + (2.249842e-03) * (q[i8] * q[i15])
            + (-6.333248e-04) * (q[i8] * q[i16]) + (-1.102029e-03) * (q[i8] * q[i19]) + (2.936866e-04) * (q[i8] * q[i20]) + (1.345406e-03) * (q[i8] * q[i21])
            + (-1.009421e-03) * (q[i8] * q[i22]) + (-8.760234e-04) * (q[i9] * q[i10]) + (-4.594280e-05) * (q[i9] * q[i11]) + (7.649316e-04) * (q[i9] * q[i12])
            + (-5.560971e-04) * (q[i9] * q[i15]) + (-4.782029e-05) * (q[i9] * q[i16]) + (-4.730683e-04) * (q[i9] * q[i19]) + (-1.099627e-03) * (q[i9] * q[i20])
            + (1.866575e-04) * (q[i9] * q[i21]) + (7.260578e-04) * (q[i9] * q[i22]) + (-6.667777e-04) * (q[i10] * q[i11]) + (-5.476878e-04) * (q[i10] * q[i12])
            + (3.970188e-04) * (q[i10] * q[i15]) + (-5.558833e-04) * (q[i10] * q[i16]) + (-8.172558e-04) * (q[i10] * q[i19])
            + (5.768088e-04) * (q[i10] * q[i20]) + (-7.450826e-04) * (q[i10] * q[i21]) + (3.008438e-04) * (q[i10] * q[i22]) + (2.182957e-03) * (q[i11] * q[i12])
            + (4.914696e-04) * (q[i11] * q[i15]) + (-9.167755e-04) * (q[i11] * q[i16]) + (-7.213827e-04) * (q[i11] * q[i19])
            + (6.426055e-04) * (q[i11] * q[i20]) + (-2.241782e-04) * (q[i11] * q[i21]) + (-1.502159e-04) * (q[i11] * q[i22])
            + (1.238090e-03) * (q[i12] * q[i15]) + (-8.871230e-05) * (q[i12] * q[i16]) + (-6.713509e-04) * (q[i12] * q[i19])
            + (4.993456e-05) * (q[i12] * q[i20]) + (-2.370325e-04) * (q[i12] * q[i21]) + (-8.710689e-04) * (q[i12] * q[i22])
            + (-7.862352e-04) * (q[i15] * q[i16]) + (-6.389798e-04) * (q[i15] * q[i19]) + (-9.214458e-05) * (q[i15] * q[i20])
            + (2.972375e-05) * (q[i15] * q[i21]) + (2.369613e-04) * (q[i15] * q[i22]) + (2.711572e-04) * (q[i16] * q[i19]) + (-6.498497e-05) * (q[i16] * q[i20])
            + (5.920154e-04) * (q[i16] * q[i21]) + (-1.133731e-03) * (q[i16] * q[i22]) + (-6.020368e-04) * (q[i19] * q[i20])
            + (-8.144087e-04) * (q[i19] * q[i21]) + (-4.996268e-04) * (q[i19] * q[i22]) + (4.308491e-04) * (q[i20] * q[i21])
            + (-5.089145e-04) * (q[i20] * q[i22]) + (-3.487774e-04) * (q[i21] * q[i22]);
      JQ[3][i8] = (-2.310491e-04) * (1) + (7.957113e-05) * ((2) * q[i8]) + (4.216772e-03) * (q[i0]) + (4.222046e-03) * (q[i1]) + (9.573565e-03) * (q[i2])
            + (-2.371876e-03) * (q[i3]) + (-2.295192e-03) * (q[i4]) + (4.436577e-02) * (q[i5]) + (-5.708182e-03) * (q[i6]) + (5.638196e-03) * (q[i7])
            + (1.373567e-03) * (q[i9]) + (-1.386535e-03) * (q[i10]) + (-3.113403e-03) * (q[i11]) + (-3.324383e-03) * (q[i12]) + (1.215025e-02) * (q[i15])
            + (-1.220530e-02) * (q[i16]) + (1.431593e-03) * (q[i19]) + (-1.407848e-03) * (q[i20]) + (3.691870e-03) * (q[i21]) + (3.724218e-03) * (q[i22])
            + (-1.421258e-03) * (q[i0] * q[i0]) + (1.391285e-03) * (q[i1] * q[i1]) + (-7.652074e-06) * (q[i2] * q[i2]) + (4.152577e-03) * (q[i3] * q[i3])
            + (-4.135429e-03) * (q[i4] * q[i4]) + (3.616921e-05) * (q[i5] * q[i5]) + (4.802060e-03) * (q[i6] * q[i6]) + (-4.807721e-03) * (q[i7] * q[i7])
            + (1.993809e-03) * ((2) * q[i0] * q[i8]) + (1.982408e-03) * ((2) * q[i1] * q[i8]) + (1.203593e-03) * ((2) * q[i2] * q[i8])
            + (-5.386114e-04) * ((2) * q[i3] * q[i8]) + (-5.371954e-04) * ((2) * q[i4] * q[i8]) + (4.209572e-03) * ((2) * q[i5] * q[i8])
            + (-1.768166e-03) * ((2) * q[i6] * q[i8]) + (1.768702e-03) * ((2) * q[i7] * q[i8]) + (-1.080768e-05) * ((3) * q[i8] * q[i8])
            + (-5.393390e-04) * ((2) * q[i8] * q[i9]) + (5.418722e-04) * ((2) * q[i8] * q[i10]) + (-3.460304e-03) * ((2) * q[i8] * q[i11])
            + (-3.547921e-03) * ((2) * q[i8] * q[i12]) + (-3.006602e-03) * ((2) * q[i8] * q[i15]) + (3.066301e-03) * ((2) * q[i8] * q[i16])
            + (6.014736e-04) * ((2) * q[i8] * q[i19]) + (-6.122044e-04) * ((2) * q[i8] * q[i20]) + (9.352912e-04) * ((2) * q[i8] * q[i21])
            + (9.451885e-04) * ((2) * q[i8] * q[i22]) + (2.340922e-04) * (q[i9] * q[i9]) + (-2.270851e-04) * (q[i10] * q[i10])
            + (-2.875865e-03) * (q[i11] * q[i11]) + (2.941201e-03) * (q[i12] * q[i12]) + (-2.416944e-03) * (q[i15] * q[i15])
            + (2.450313e-03) * (q[i16] * q[i16]) + (9.640285e-04) * (q[i19] * q[i19]) + (-9.433013e-04) * (q[i20] * q[i20]) + (2.487787e-04) * (q[i21] * q[i21])
            + (-2.487447e-04) * (q[i22] * q[i22]) + (1.491949e-05) * (q[i0] * q[i1]) + (1.531502e-03) * (q[i0] * q[i2]) + (2.669265e-03) * (q[i0] * q[i3])
            + (2.519785e-03) * (q[i0] * q[i4]) + (2.677214e-03) * (q[i0] * q[i5]) + (2.764363e-03) * (q[i0] * q[i6]) + (5.301472e-04) * (q[i0] * q[i7])
            + (5.055425e-06) * (q[i0] * q[i9]) + (-5.337817e-04) * (q[i0] * q[i10]) + (-4.324909e-04) * (q[i0] * q[i11]) + (-8.340509e-04) * (q[i0] * q[i12])
            + (-1.039356e-03) * (q[i0] * q[i15]) + (-2.300283e-03) * (q[i0] * q[i16]) + (-1.829503e-04) * (q[i0] * q[i19]) + (2.763084e-04) * (q[i0] * q[i20])
            + (-3.341720e-05) * (q[i0] * q[i21]) + (2.267855e-04) * (q[i0] * q[i22]) + (-1.557901e-03) * (q[i1] * q[i2]) + (-2.519406e-03) * (q[i1] * q[i3])
            + (-2.673250e-03) * (q[i1] * q[i4]) + (-2.662097e-03) * (q[i1] * q[i5]) + (5.266155e-04) * (q[i1] * q[i6]) + (2.738656e-03) * (q[i1] * q[i7])
            + (-5.361324e-04) * (q[i1] * q[i9]) + (-3.204259e-06) * (q[i1] * q[i10]) + (8.293673e-04) * (q[i1] * q[i11]) + (4.379223e-04) * (q[i1] * q[i12])
            + (-2.278029e-03) * (q[i1] * q[i15]) + (-1.046418e-03) * (q[i1] * q[i16]) + (2.850417e-04) * (q[i1] * q[i19]) + (-1.726717e-04) * (q[i1] * q[i20])
            + (-2.283093e-04) * (q[i1] * q[i21]) + (1.446450e-05) * (q[i1] * q[i22]) + (-2.313675e-03) * (q[i2] * q[i3]) + (2.307384e-03) * (q[i2] * q[i4])
            + (-1.735343e-06) * (q[i2] * q[i5]) + (-5.275663e-03) * (q[i2] * q[i6]) + (-5.243947e-03) * (q[i2] * q[i7]) + (-1.988901e-03) * (q[i2] * q[i9])
            + (-1.960195e-03) * (q[i2] * q[i10]) + (-1.958002e-04) * (q[i2] * q[i11]) + (1.600792e-04) * (q[i2] * q[i12]) + (-2.168129e-03) * (q[i2] * q[i15])
            + (-2.187043e-03) * (q[i2] * q[i16]) + (-7.860911e-04) * (q[i2] * q[i19]) + (-7.718952e-04) * (q[i2] * q[i20]) + (8.458875e-04) * (q[i2] * q[i21])
            + (-8.524062e-04) * (q[i2] * q[i22]) + (-1.029825e-05) * (q[i3] * q[i4]) + (-1.224074e-02) * (q[i3] * q[i5]) + (1.302527e-03) * (q[i3] * q[i6])
            + (-3.731833e-03) * (q[i3] * q[i7]) + (-3.652426e-04) * (q[i3] * q[i9]) + (6.855023e-04) * (q[i3] * q[i10]) + (1.223695e-03) * (q[i3] * q[i11])
            + (6.380943e-04) * (q[i3] * q[i12]) + (-1.681315e-03) * (q[i3] * q[i15]) + (7.726770e-04) * (q[i3] * q[i16]) + (1.835695e-03) * (q[i3] * q[i19])
            + (-1.592012e-03) * (q[i3] * q[i20]) + (4.773394e-04) * (q[i3] * q[i21]) + (-1.072421e-04) * (q[i3] * q[i22]) + (1.221304e-02) * (q[i4] * q[i5])
            + (-3.741767e-03) * (q[i4] * q[i6]) + (1.331434e-03) * (q[i4] * q[i7]) + (6.757443e-04) * (q[i4] * q[i9]) + (-3.792716e-04) * (q[i4] * q[i10])
            + (-6.633114e-04) * (q[i4] * q[i11]) + (-1.297709e-03) * (q[i4] * q[i12]) + (7.776032e-04) * (q[i4] * q[i15]) + (-1.691448e-03) * (q[i4] * q[i16])
            + (-1.577424e-03) * (q[i4] * q[i19]) + (1.827705e-03) * (q[i4] * q[i20]) + (1.110036e-04) * (q[i4] * q[i21]) + (-4.623798e-04) * (q[i4] * q[i22])
            + (1.560364e-03) * (q[i5] * q[i6]) + (1.567247e-03) * (q[i5] * q[i7]) + (3.370415e-03) * (q[i5] * q[i9]) + (3.358581e-03) * (q[i5] * q[i10])
            + (-2.579714e-03) * (q[i5] * q[i11]) + (2.749042e-03) * (q[i5] * q[i12]) + (-2.099753e-03) * (q[i5] * q[i15]) + (-2.120832e-03) * (q[i5] * q[i16])
            + (3.321299e-04) * (q[i5] * q[i19]) + (3.494283e-04) * (q[i5] * q[i20]) + (-1.047609e-04) * (q[i5] * q[i21]) + (6.351661e-05) * (q[i5] * q[i22])
            + (-1.143098e-05) * (q[i6] * q[i7]) + (4.222792e-04) * (q[i6] * q[i9]) + (-3.622831e-04) * (q[i6] * q[i10]) + (1.840413e-03) * (q[i6] * q[i11])
            + (5.693488e-04) * (q[i6] * q[i12]) + (6.057846e-04) * (q[i6] * q[i15]) + (-2.253623e-03) * (q[i6] * q[i16]) + (-2.846423e-04) * (q[i6] * q[i19])
            + (1.106294e-03) * (q[i6] * q[i20]) + (-1.004595e-03) * (q[i6] * q[i21]) + (1.344199e-03) * (q[i6] * q[i22]) + (3.671713e-04) * (q[i7] * q[i9])
            + (-4.161039e-04) * (q[i7] * q[i10]) + (5.344024e-04) * (q[i7] * q[i11]) + (1.838683e-03) * (q[i7] * q[i12]) + (2.249842e-03) * (q[i7] * q[i15])
            + (-6.333248e-04) * (q[i7] * q[i16]) + (-1.102029e-03) * (q[i7] * q[i19]) + (2.936866e-04) * (q[i7] * q[i20]) + (1.345406e-03) * (q[i7] * q[i21])
            + (-1.009421e-03) * (q[i7] * q[i22]) + (-2.654165e-06) * (q[i9] * q[i10]) + (9.049542e-04) * (q[i9] * q[i11]) + (1.520027e-04) * (q[i9] * q[i12])
            + (-9.073218e-06) * (q[i9] * q[i15]) + (-1.216992e-03) * (q[i9] * q[i16]) + (1.033928e-04) * (q[i9] * q[i19]) + (5.290930e-04) * (q[i9] * q[i20])
            + (-2.171296e-04) * (q[i9] * q[i21]) + (-2.532998e-04) * (q[i9] * q[i22]) + (1.550690e-04) * (q[i10] * q[i11]) + (9.047453e-04) * (q[i10] * q[i12])
            + (1.210958e-03) * (q[i10] * q[i15]) + (7.722964e-06) * (q[i10] * q[i16]) + (-5.215299e-04) * (q[i10] * q[i19])
            + (-1.035603e-04) * (q[i10] * q[i20]) + (-2.553434e-04) * (q[i10] * q[i21]) + (-2.204404e-04) * (q[i10] * q[i22])
            + (6.327379e-06) * (q[i11] * q[i12]) + (2.458364e-03) * (q[i11] * q[i15]) + (2.508030e-04) * (q[i11] * q[i16]) + (-6.835541e-04) * (q[i11] * q[i19])
            + (1.607269e-04) * (q[i11] * q[i20]) + (-8.277569e-04) * (q[i11] * q[i21]) + (-1.381067e-04) * (q[i11] * q[i22])
            + (2.457640e-04) * (q[i12] * q[i15]) + (2.461207e-03) * (q[i12] * q[i16]) + (1.680650e-04) * (q[i12] * q[i19]) + (-6.569697e-04) * (q[i12] * q[i20])
            + (1.356376e-04) * (q[i12] * q[i21]) + (8.294795e-04) * (q[i12] * q[i22]) + (8.123149e-06) * (q[i15] * q[i16]) + (7.750505e-04) * (q[i15] * q[i19])
            + (1.638484e-04) * (q[i15] * q[i20]) + (2.465250e-03) * (q[i15] * q[i21]) + (-1.715073e-04) * (q[i15] * q[i22])
            + (-1.567777e-04) * (q[i16] * q[i19]) + (-7.861975e-04) * (q[i16] * q[i20]) + (-1.693606e-04) * (q[i16] * q[i21])
            + (2.504864e-03) * (q[i16] * q[i22]) + (5.985790e-06) * (q[i19] * q[i20]) + (1.998452e-03) * (q[i19] * q[i21]) + (-2.105038e-04) * (q[i19] * q[i22])
            + (-2.078082e-04) * (q[i20] * q[i21]) + (1.988446e-03) * (q[i20] * q[i22]) + (1.711823e-06) * (q[i21] * q[i22]);
      JQ[3][i9] = (3.732829e-02) * (1) + (-3.575926e-03) * ((2) * q[i9]) + (-3.284959e-03) * (q[i0]) + (2.326091e-03) * (q[i1]) + (1.805297e-03) * (q[i2])
            + (1.781216e-03) * (q[i3]) + (1.254799e-02) * (q[i4]) + (8.223239e-03) * (q[i5]) + (-5.801340e-03) * (q[i6]) + (1.361736e-03) * (q[i7])
            + (1.373567e-03) * (q[i8]) + (-4.851862e-06) * (q[i10]) + (1.881100e-03) * (q[i11]) + (2.079217e-03) * (q[i12]) + (-5.601694e-04) * (q[i15])
            + (3.954755e-04) * (q[i16]) + (-9.137787e-04) * (q[i19]) + (-1.001970e-03) * (q[i20]) + (-6.894929e-04) * (q[i21]) + (3.273732e-04) * (q[i22])
            + (-1.484376e-03) * (q[i0] * q[i0]) + (1.083900e-03) * (q[i1] * q[i1]) + (-1.039342e-03) * (q[i2] * q[i2]) + (1.871842e-03) * (q[i3] * q[i3])
            + (1.618644e-04) * (q[i4] * q[i4]) + (-1.680639e-03) * (q[i5] * q[i5]) + (-9.200232e-04) * (q[i6] * q[i6]) + (-2.036214e-03) * (q[i7] * q[i7])
            + (-5.393390e-04) * (q[i8] * q[i8]) + (3.640691e-03) * ((2) * q[i0] * q[i9]) + (-4.503962e-04) * ((2) * q[i1] * q[i9])
            + (1.087245e-03) * ((2) * q[i2] * q[i9]) + (6.405877e-04) * ((2) * q[i3] * q[i9]) + (-1.436498e-03) * ((2) * q[i4] * q[i9])
            + (-1.565213e-03) * ((2) * q[i5] * q[i9]) + (-2.235646e-03) * ((2) * q[i6] * q[i9]) + (-2.395308e-04) * ((2) * q[i7] * q[i9])
            + (2.340922e-04) * ((2) * q[i8] * q[i9]) + (-9.357534e-04) * ((3) * q[i9] * q[i9]) + (1.209485e-04) * ((2) * q[i9] * q[i10])
            + (1.281909e-05) * ((2) * q[i9] * q[i11]) + (9.194117e-05) * ((2) * q[i9] * q[i12]) + (4.820502e-05) * ((2) * q[i9] * q[i15])
            + (-3.311503e-04) * ((2) * q[i9] * q[i16]) + (-1.175025e-04) * ((2) * q[i9] * q[i19]) + (3.687722e-04) * ((2) * q[i9] * q[i20])
            + (1.736911e-04) * ((2) * q[i9] * q[i21]) + (-1.672486e-04) * ((2) * q[i9] * q[i22]) + (-1.185146e-04) * (q[i10] * q[i10])
            + (6.788440e-05) * (q[i11] * q[i11]) + (2.177714e-04) * (q[i12] * q[i12]) + (1.615056e-04) * (q[i15] * q[i15]) + (-3.178814e-04) * (q[i16] * q[i16])
            + (1.008393e-04) * (q[i19] * q[i19]) + (3.953414e-05) * (q[i20] * q[i20]) + (1.739053e-04) * (q[i21] * q[i21]) + (1.112554e-06) * (q[i22] * q[i22])
            + (-5.789151e-04) * (q[i0] * q[i1]) + (-1.145506e-03) * (q[i0] * q[i2]) + (5.972616e-03) * (q[i0] * q[i3]) + (4.115776e-03) * (q[i0] * q[i4])
            + (9.345901e-04) * (q[i0] * q[i5]) + (-7.348027e-03) * (q[i0] * q[i6]) + (1.497076e-03) * (q[i0] * q[i7]) + (5.055425e-06) * (q[i0] * q[i8])
            + (6.205401e-04) * (q[i0] * q[i10]) + (-1.484126e-04) * (q[i0] * q[i11]) + (1.019386e-04) * (q[i0] * q[i12]) + (3.218487e-05) * (q[i0] * q[i15])
            + (2.249269e-05) * (q[i0] * q[i16]) + (4.460672e-05) * (q[i0] * q[i19]) + (1.743876e-05) * (q[i0] * q[i20]) + (1.217458e-04) * (q[i0] * q[i21])
            + (1.699462e-04) * (q[i0] * q[i22]) + (7.736552e-04) * (q[i1] * q[i2]) + (-2.124975e-03) * (q[i1] * q[i3]) + (-4.901910e-04) * (q[i1] * q[i4])
            + (-6.748973e-05) * (q[i1] * q[i5]) + (4.249609e-03) * (q[i1] * q[i6]) + (4.749150e-03) * (q[i1] * q[i7]) + (-5.361324e-04) * (q[i1] * q[i8])
            + (6.272113e-04) * (q[i1] * q[i10]) + (-2.220626e-04) * (q[i1] * q[i11]) + (-3.510685e-04) * (q[i1] * q[i12]) + (4.011463e-05) * (q[i1] * q[i15])
            + (1.947114e-04) * (q[i1] * q[i16]) + (3.131939e-04) * (q[i1] * q[i19]) + (-5.745873e-04) * (q[i1] * q[i20]) + (1.450506e-04) * (q[i1] * q[i21])
            + (-1.327543e-04) * (q[i1] * q[i22]) + (1.834456e-03) * (q[i2] * q[i3]) + (1.725919e-03) * (q[i2] * q[i4]) + (2.875813e-03) * (q[i2] * q[i5])
            + (-4.297177e-03) * (q[i2] * q[i6]) + (1.719453e-03) * (q[i2] * q[i7]) + (-1.988901e-03) * (q[i2] * q[i8]) + (-3.042158e-05) * (q[i2] * q[i10])
            + (-5.052321e-04) * (q[i2] * q[i11]) + (-5.898781e-04) * (q[i2] * q[i12]) + (2.126291e-04) * (q[i2] * q[i15]) + (-2.350800e-04) * (q[i2] * q[i16])
            + (7.958861e-04) * (q[i2] * q[i19]) + (-7.146794e-04) * (q[i2] * q[i20]) + (-2.137513e-04) * (q[i2] * q[i21]) + (3.476444e-05) * (q[i2] * q[i22])
            + (4.459834e-04) * (q[i3] * q[i4]) + (5.931976e-04) * (q[i3] * q[i5]) + (-2.525796e-04) * (q[i3] * q[i6]) + (-1.921714e-03) * (q[i3] * q[i7])
            + (-3.652426e-04) * (q[i3] * q[i8]) + (-5.715158e-04) * (q[i3] * q[i10]) + (-4.639914e-04) * (q[i3] * q[i11]) + (-8.660618e-04) * (q[i3] * q[i12])
            + (-3.666855e-04) * (q[i3] * q[i15]) + (6.472386e-04) * (q[i3] * q[i16]) + (6.488512e-04) * (q[i3] * q[i19]) + (-4.009139e-04) * (q[i3] * q[i20])
            + (-1.182874e-03) * (q[i3] * q[i21]) + (6.093263e-04) * (q[i3] * q[i22]) + (-1.123052e-03) * (q[i4] * q[i5]) + (-2.433371e-03) * (q[i4] * q[i6])
            + (1.500690e-03) * (q[i4] * q[i7]) + (6.757443e-04) * (q[i4] * q[i8]) + (-5.691556e-04) * (q[i4] * q[i10]) + (3.008557e-04) * (q[i4] * q[i11])
            + (2.362237e-04) * (q[i4] * q[i12]) + (-3.479641e-04) * (q[i4] * q[i15]) + (-2.630869e-04) * (q[i4] * q[i16]) + (-6.041358e-04) * (q[i4] * q[i19])
            + (9.788540e-05) * (q[i4] * q[i20]) + (3.185083e-04) * (q[i4] * q[i21]) + (-2.816227e-04) * (q[i4] * q[i22]) + (-4.389303e-03) * (q[i5] * q[i6])
            + (2.547313e-03) * (q[i5] * q[i7]) + (3.370415e-03) * (q[i5] * q[i8]) + (1.795244e-03) * (q[i5] * q[i10]) + (-2.721587e-04) * (q[i5] * q[i11])
            + (6.765033e-05) * (q[i5] * q[i12]) + (6.044531e-04) * (q[i5] * q[i15]) + (-8.888490e-05) * (q[i5] * q[i16]) + (-4.683551e-04) * (q[i5] * q[i19])
            + (-2.614614e-04) * (q[i5] * q[i20]) + (5.513357e-04) * (q[i5] * q[i21]) + (-5.134596e-04) * (q[i5] * q[i22]) + (2.577368e-03) * (q[i6] * q[i7])
            + (4.222792e-04) * (q[i6] * q[i8]) + (8.809378e-04) * (q[i6] * q[i10]) + (-5.482627e-04) * (q[i6] * q[i11]) + (-6.646076e-04) * (q[i6] * q[i12])
            + (5.592634e-04) * (q[i6] * q[i15]) + (-4.138171e-04) * (q[i6] * q[i16]) + (-5.808050e-04) * (q[i6] * q[i19]) + (8.196789e-04) * (q[i6] * q[i20])
            + (2.981797e-04) * (q[i6] * q[i21]) + (-7.568454e-04) * (q[i6] * q[i22]) + (3.671713e-04) * (q[i7] * q[i8]) + (-8.760234e-04) * (q[i7] * q[i10])
            + (-4.594280e-05) * (q[i7] * q[i11]) + (7.649316e-04) * (q[i7] * q[i12]) + (-5.560971e-04) * (q[i7] * q[i15]) + (-4.782029e-05) * (q[i7] * q[i16])
            + (-4.730683e-04) * (q[i7] * q[i19]) + (-1.099627e-03) * (q[i7] * q[i20]) + (1.866575e-04) * (q[i7] * q[i21]) + (7.260578e-04) * (q[i7] * q[i22])
            + (-2.654165e-06) * (q[i8] * q[i10]) + (9.049542e-04) * (q[i8] * q[i11]) + (1.520027e-04) * (q[i8] * q[i12]) + (-9.073218e-06) * (q[i8] * q[i15])
            + (-1.216992e-03) * (q[i8] * q[i16]) + (1.033928e-04) * (q[i8] * q[i19]) + (5.290930e-04) * (q[i8] * q[i20]) + (-2.171296e-04) * (q[i8] * q[i21])
            + (-2.532998e-04) * (q[i8] * q[i22]) + (1.286065e-04) * (q[i10] * q[i11]) + (1.250044e-04) * (q[i10] * q[i12]) + (-3.606545e-04) * (q[i10] * q[i15])
            + (3.622643e-04) * (q[i10] * q[i16]) + (8.369679e-05) * (q[i10] * q[i19]) + (-8.075254e-05) * (q[i10] * q[i20]) + (3.387799e-04) * (q[i10] * q[i21])
            + (3.377193e-04) * (q[i10] * q[i22]) + (-1.895063e-04) * (q[i11] * q[i12]) + (5.412181e-04) * (q[i11] * q[i15]) + (1.706533e-04) * (q[i11] * q[i16])
            + (1.998468e-04) * (q[i11] * q[i19]) + (-1.656583e-04) * (q[i11] * q[i20]) + (4.284419e-04) * (q[i11] * q[i21]) + (1.575985e-04) * (q[i11] * q[i22])
            + (-4.133039e-05) * (q[i12] * q[i15]) + (8.252603e-04) * (q[i12] * q[i16]) + (-8.015074e-05) * (q[i12] * q[i19])
            + (-2.905708e-04) * (q[i12] * q[i20]) + (3.768899e-05) * (q[i12] * q[i21]) + (-2.169850e-04) * (q[i12] * q[i22])
            + (-2.230831e-04) * (q[i15] * q[i16]) + (-5.647189e-05) * (q[i15] * q[i19]) + (1.652257e-04) * (q[i15] * q[i20])
            + (-4.322648e-04) * (q[i15] * q[i21]) + (5.285697e-05) * (q[i15] * q[i22]) + (-3.671280e-05) * (q[i16] * q[i19])
            + (1.781724e-04) * (q[i16] * q[i20]) + (-2.725853e-05) * (q[i16] * q[i21]) + (4.789656e-05) * (q[i16] * q[i22]) + (1.186678e-04) * (q[i19] * q[i20])
            + (-3.199455e-04) * (q[i19] * q[i21]) + (1.559101e-04) * (q[i19] * q[i22]) + (-7.904078e-05) * (q[i20] * q[i21])
            + (-1.632341e-04) * (q[i20] * q[i22]) + (4.792738e-05) * (q[i21] * q[i22]);
      JQ[3][i10] = (-3.705800e-02) * (1) + (3.552622e-03) * ((2) * q[i10]) + (2.338755e-03) * (q[i0]) + (-3.166851e-03) * (q[i1]) + (1.865104e-03) * (q[i2])
            + (1.244188e-02) * (q[i3]) + (1.785154e-03) * (q[i4]) + (8.111889e-03) * (q[i5]) + (-1.367266e-03) * (q[i6]) + (5.718622e-03) * (q[i7])
            + (-1.386535e-03) * (q[i8]) + (-4.851862e-06) * (q[i9]) + (2.076850e-03) * (q[i11]) + (1.907630e-03) * (q[i12]) + (-3.640950e-04) * (q[i15])
            + (5.470199e-04) * (q[i16]) + (1.006726e-03) * (q[i19]) + (9.125165e-04) * (q[i20]) + (3.067238e-04) * (q[i21]) + (-6.889521e-04) * (q[i22])
            + (-1.081645e-03) * (q[i0] * q[i0]) + (1.475178e-03) * (q[i1] * q[i1]) + (1.028019e-03) * (q[i2] * q[i2]) + (-1.640236e-04) * (q[i3] * q[i3])
            + (-1.866949e-03) * (q[i4] * q[i4]) + (1.669961e-03) * (q[i5] * q[i5]) + (2.026387e-03) * (q[i6] * q[i6]) + (9.137079e-04) * (q[i7] * q[i7])
            + (5.418722e-04) * (q[i8] * q[i8]) + (1.209485e-04) * (q[i9] * q[i9]) + (-4.561813e-04) * ((2) * q[i0] * q[i10])
            + (3.589453e-03) * ((2) * q[i1] * q[i10]) + (1.063092e-03) * ((2) * q[i2] * q[i10]) + (-1.418208e-03) * ((2) * q[i3] * q[i10])
            + (5.974724e-04) * ((2) * q[i4] * q[i10]) + (-1.534836e-03) * ((2) * q[i5] * q[i10]) + (2.415019e-04) * ((2) * q[i6] * q[i10])
            + (2.215410e-03) * ((2) * q[i7] * q[i10]) + (-2.270851e-04) * ((2) * q[i8] * q[i10]) + (-1.185146e-04) * ((2) * q[i9] * q[i10])
            + (9.230751e-04) * ((3) * q[i10] * q[i10]) + (8.811553e-05) * ((2) * q[i10] * q[i11]) + (6.403050e-06) * ((2) * q[i10] * q[i12])
            + (3.212800e-04) * ((2) * q[i10] * q[i15]) + (-4.669194e-05) * ((2) * q[i10] * q[i16]) + (-3.655092e-04) * ((2) * q[i10] * q[i19])
            + (1.150544e-04) * ((2) * q[i10] * q[i20]) + (-1.624001e-04) * ((2) * q[i10] * q[i21]) + (1.740462e-04) * ((2) * q[i10] * q[i22])
            + (-2.069287e-04) * (q[i11] * q[i11]) + (-6.095788e-05) * (q[i12] * q[i12]) + (3.116321e-04) * (q[i15] * q[i15])
            + (-1.635291e-04) * (q[i16] * q[i16]) + (-4.304783e-05) * (q[i19] * q[i19]) + (-1.005608e-04) * (q[i20] * q[i20])
            + (-6.830081e-07) * (q[i21] * q[i21]) + (-1.751883e-04) * (q[i22] * q[i22]) + (5.807603e-04) * (q[i0] * q[i1]) + (-7.645814e-04) * (q[i0] * q[i2])
            + (4.885023e-04) * (q[i0] * q[i3]) + (2.123625e-03) * (q[i0] * q[i4]) + (8.060825e-05) * (q[i0] * q[i5]) + (4.699217e-03) * (q[i0] * q[i6])
            + (4.225340e-03) * (q[i0] * q[i7]) + (-5.337817e-04) * (q[i0] * q[i8]) + (6.205401e-04) * (q[i0] * q[i9]) + (3.592033e-04) * (q[i0] * q[i11])
            + (2.376944e-04) * (q[i0] * q[i12]) + (1.978951e-04) * (q[i0] * q[i15]) + (4.330330e-05) * (q[i0] * q[i16]) + (-5.757650e-04) * (q[i0] * q[i19])
            + (3.177573e-04) * (q[i0] * q[i20]) + (1.299127e-04) * (q[i0] * q[i21]) + (-1.408020e-04) * (q[i0] * q[i22]) + (1.135680e-03) * (q[i1] * q[i2])
            + (-4.112262e-03) * (q[i1] * q[i3]) + (-5.909641e-03) * (q[i1] * q[i4]) + (-9.173541e-04) * (q[i1] * q[i5]) + (1.493330e-03) * (q[i1] * q[i6])
            + (-7.327260e-03) * (q[i1] * q[i7]) + (-3.204259e-06) * (q[i1] * q[i8]) + (6.272113e-04) * (q[i1] * q[i9]) + (-1.021766e-04) * (q[i1] * q[i11])
            + (1.458899e-04) * (q[i1] * q[i12]) + (2.651569e-05) * (q[i1] * q[i15]) + (3.412348e-05) * (q[i1] * q[i16]) + (1.504097e-05) * (q[i1] * q[i19])
            + (4.715358e-05) * (q[i1] * q[i20]) + (-1.679596e-04) * (q[i1] * q[i21]) + (-1.153232e-04) * (q[i1] * q[i22]) + (-1.719263e-03) * (q[i2] * q[i3])
            + (-1.793558e-03) * (q[i2] * q[i4]) + (-2.851083e-03) * (q[i2] * q[i5]) + (1.693568e-03) * (q[i2] * q[i6]) + (-4.273448e-03) * (q[i2] * q[i7])
            + (-1.960195e-03) * (q[i2] * q[i8]) + (-3.042158e-05) * (q[i2] * q[i9]) + (5.983864e-04) * (q[i2] * q[i11]) + (5.127200e-04) * (q[i2] * q[i12])
            + (-2.269576e-04) * (q[i2] * q[i15]) + (2.115800e-04) * (q[i2] * q[i16]) + (-7.141847e-04) * (q[i2] * q[i19]) + (7.970314e-04) * (q[i2] * q[i20])
            + (-3.824529e-05) * (q[i2] * q[i21]) + (2.139062e-04) * (q[i2] * q[i22]) + (-4.160438e-04) * (q[i3] * q[i4]) + (1.127468e-03) * (q[i3] * q[i5])
            + (1.481257e-03) * (q[i3] * q[i6]) + (-2.387680e-03) * (q[i3] * q[i7]) + (6.855023e-04) * (q[i3] * q[i8]) + (-5.715158e-04) * (q[i3] * q[i9])
            + (-2.395844e-04) * (q[i3] * q[i11]) + (-3.029864e-04) * (q[i3] * q[i12]) + (-2.730054e-04) * (q[i3] * q[i15]) + (-3.540169e-04) * (q[i3] * q[i16])
            + (8.387708e-05) * (q[i3] * q[i19]) + (-6.046693e-04) * (q[i3] * q[i20]) + (2.872147e-04) * (q[i3] * q[i21]) + (-3.128102e-04) * (q[i3] * q[i22])
            + (-6.099268e-04) * (q[i4] * q[i5]) + (-1.892674e-03) * (q[i4] * q[i6]) + (-3.217646e-04) * (q[i4] * q[i7]) + (-3.792716e-04) * (q[i4] * q[i8])
            + (-5.691556e-04) * (q[i4] * q[i9]) + (8.557228e-04) * (q[i4] * q[i11]) + (4.610310e-04) * (q[i4] * q[i12]) + (6.379221e-04) * (q[i4] * q[i15])
            + (-3.588534e-04) * (q[i4] * q[i16]) + (-4.020045e-04) * (q[i4] * q[i19]) + (6.473136e-04) * (q[i4] * q[i20]) + (-6.081948e-04) * (q[i4] * q[i21])
            + (1.174813e-03) * (q[i4] * q[i22]) + (2.530828e-03) * (q[i5] * q[i6]) + (-4.329624e-03) * (q[i5] * q[i7]) + (3.358581e-03) * (q[i5] * q[i8])
            + (1.795244e-03) * (q[i5] * q[i9]) + (-5.485953e-05) * (q[i5] * q[i11]) + (2.937166e-04) * (q[i5] * q[i12]) + (-7.002994e-05) * (q[i5] * q[i15])
            + (6.086860e-04) * (q[i5] * q[i16]) + (-2.530374e-04) * (q[i5] * q[i19]) + (-4.645845e-04) * (q[i5] * q[i20]) + (5.093617e-04) * (q[i5] * q[i21])
            + (-5.545135e-04) * (q[i5] * q[i22]) + (-2.564323e-03) * (q[i6] * q[i7]) + (-3.622831e-04) * (q[i6] * q[i8]) + (8.809378e-04) * (q[i6] * q[i9])
            + (7.645532e-04) * (q[i6] * q[i11]) + (-4.659777e-05) * (q[i6] * q[i12]) + (4.928168e-05) * (q[i6] * q[i15]) + (5.564520e-04) * (q[i6] * q[i16])
            + (1.102690e-03) * (q[i6] * q[i19]) + (4.810234e-04) * (q[i6] * q[i20]) + (7.280991e-04) * (q[i6] * q[i21]) + (1.866174e-04) * (q[i6] * q[i22])
            + (-4.161039e-04) * (q[i7] * q[i8]) + (-8.760234e-04) * (q[i7] * q[i9]) + (-6.667777e-04) * (q[i7] * q[i11]) + (-5.476878e-04) * (q[i7] * q[i12])
            + (3.970188e-04) * (q[i7] * q[i15]) + (-5.558833e-04) * (q[i7] * q[i16]) + (-8.172558e-04) * (q[i7] * q[i19]) + (5.768088e-04) * (q[i7] * q[i20])
            + (-7.450826e-04) * (q[i7] * q[i21]) + (3.008438e-04) * (q[i7] * q[i22]) + (-2.654165e-06) * (q[i8] * q[i9]) + (1.550690e-04) * (q[i8] * q[i11])
            + (9.047453e-04) * (q[i8] * q[i12]) + (1.210958e-03) * (q[i8] * q[i15]) + (7.722964e-06) * (q[i8] * q[i16]) + (-5.215299e-04) * (q[i8] * q[i19])
            + (-1.035603e-04) * (q[i8] * q[i20]) + (-2.553434e-04) * (q[i8] * q[i21]) + (-2.204404e-04) * (q[i8] * q[i22]) + (1.286065e-04) * (q[i9] * q[i11])
            + (1.250044e-04) * (q[i9] * q[i12]) + (-3.606545e-04) * (q[i9] * q[i15]) + (3.622643e-04) * (q[i9] * q[i16]) + (8.369679e-05) * (q[i9] * q[i19])
            + (-8.075254e-05) * (q[i9] * q[i20]) + (3.387799e-04) * (q[i9] * q[i21]) + (3.377193e-04) * (q[i9] * q[i22]) + (1.848216e-04) * (q[i11] * q[i12])
            + (8.219186e-04) * (q[i11] * q[i15]) + (-3.676150e-05) * (q[i11] * q[i16]) + (-2.833224e-04) * (q[i11] * q[i19])
            + (-7.525957e-05) * (q[i11] * q[i20]) + (2.175368e-04) * (q[i11] * q[i21]) + (-3.807429e-05) * (q[i11] * q[i22])
            + (1.745728e-04) * (q[i12] * q[i15]) + (5.422234e-04) * (q[i12] * q[i16]) + (-1.673986e-04) * (q[i12] * q[i19]) + (2.034290e-04) * (q[i12] * q[i20])
            + (-1.576923e-04) * (q[i12] * q[i21]) + (-4.287859e-04) * (q[i12] * q[i22]) + (2.259030e-04) * (q[i15] * q[i16])
            + (-1.702933e-04) * (q[i15] * q[i19]) + (3.417873e-05) * (q[i15] * q[i20]) + (4.745517e-05) * (q[i15] * q[i21])
            + (-2.623439e-05) * (q[i15] * q[i22]) + (-1.618793e-04) * (q[i16] * q[i19]) + (5.807324e-05) * (q[i16] * q[i20])
            + (5.007479e-05) * (q[i16] * q[i21]) + (-4.296329e-04) * (q[i16] * q[i22]) + (-1.181828e-04) * (q[i19] * q[i20])
            + (-1.620898e-04) * (q[i19] * q[i21]) + (-7.845373e-05) * (q[i19] * q[i22]) + (1.556146e-04) * (q[i20] * q[i21])
            + (-3.133288e-04) * (q[i20] * q[i22]) + (-4.459758e-05) * (q[i21] * q[i22]);
      JQ[3][i11] = (4.345893e-02) * (1) + (5.601448e-05) * ((2) * q[i11]) + (7.367676e-04) * (q[i0]) + (1.854969e-04) * (q[i1]) + (1.561177e-03) * (q[i2])
            + (-2.343839e-05) * (q[i3]) + (-8.096186e-04) * (q[i4]) + (-3.910134e-03) * (q[i5]) + (9.259903e-04) * (q[i6]) + (1.568874e-03) * (q[i7])
            + (-3.113403e-03) * (q[i8]) + (1.881100e-03) * (q[i9]) + (2.076850e-03) * (q[i10]) + (-4.592529e-05) * (q[i12]) + (-3.116551e-04) * (q[i15])
            + (8.965799e-04) * (q[i16]) + (2.130627e-03) * (q[i19]) + (9.533648e-04) * (q[i20]) + (5.293681e-03) * (q[i21]) + (6.641784e-04) * (q[i22])
            + (-9.726582e-04) * (q[i0] * q[i0]) + (2.308146e-04) * (q[i1] * q[i1]) + (-1.988575e-04) * (q[i2] * q[i2]) + (1.521167e-04) * (q[i3] * q[i3])
            + (-1.317034e-03) * (q[i4] * q[i4]) + (1.658584e-03) * (q[i5] * q[i5]) + (-1.200776e-03) * (q[i6] * q[i6]) + (5.780431e-04) * (q[i7] * q[i7])
            + (-3.460304e-03) * (q[i8] * q[i8]) + (1.281909e-05) * (q[i9] * q[i9]) + (8.811553e-05) * (q[i10] * q[i10])
            + (-5.592799e-04) * ((2) * q[i0] * q[i11]) + (1.304813e-05) * ((2) * q[i1] * q[i11]) + (-6.774947e-04) * ((2) * q[i2] * q[i11])
            + (-5.152622e-04) * ((2) * q[i3] * q[i11]) + (-8.731155e-04) * ((2) * q[i4] * q[i11]) + (-4.852332e-04) * ((2) * q[i5] * q[i11])
            + (1.388729e-03) * ((2) * q[i6] * q[i11]) + (-1.134499e-03) * ((2) * q[i7] * q[i11]) + (-2.875865e-03) * ((2) * q[i8] * q[i11])
            + (6.788440e-05) * ((2) * q[i9] * q[i11]) + (-2.069287e-04) * ((2) * q[i10] * q[i11]) + (-1.067481e-03) * ((3) * q[i11] * q[i11])
            + (3.223710e-04) * ((2) * q[i11] * q[i12]) + (1.537403e-03) * ((2) * q[i11] * q[i15]) + (-4.745409e-05) * ((2) * q[i11] * q[i16])
            + (-9.906457e-04) * ((2) * q[i11] * q[i19]) + (5.648588e-05) * ((2) * q[i11] * q[i20]) + (-7.417927e-04) * ((2) * q[i11] * q[i21])
            + (-7.896716e-05) * ((2) * q[i11] * q[i22]) + (3.280248e-04) * (q[i12] * q[i12]) + (-6.175304e-03) * (q[i15] * q[i15])
            + (2.060527e-04) * (q[i16] * q[i16]) + (8.530024e-04) * (q[i19] * q[i19]) + (3.857706e-05) * (q[i20] * q[i20]) + (-3.345501e-04) * (q[i21] * q[i21])
            + (1.367414e-04) * (q[i22] * q[i22]) + (4.939440e-04) * (q[i0] * q[i1]) + (3.099205e-04) * (q[i0] * q[i2]) + (-3.955337e-04) * (q[i0] * q[i3])
            + (-5.837825e-04) * (q[i0] * q[i4]) + (4.402940e-04) * (q[i0] * q[i5]) + (2.115287e-04) * (q[i0] * q[i6]) + (-6.099110e-04) * (q[i0] * q[i7])
            + (-4.324909e-04) * (q[i0] * q[i8]) + (-1.484126e-04) * (q[i0] * q[i9]) + (3.592033e-04) * (q[i0] * q[i10]) + (5.993503e-04) * (q[i0] * q[i12])
            + (1.081469e-03) * (q[i0] * q[i15]) + (1.854720e-04) * (q[i0] * q[i16]) + (2.979992e-04) * (q[i0] * q[i19]) + (-1.001125e-03) * (q[i0] * q[i20])
            + (2.566488e-04) * (q[i0] * q[i21]) + (3.796646e-04) * (q[i0] * q[i22]) + (8.498896e-04) * (q[i1] * q[i2]) + (7.834184e-05) * (q[i1] * q[i3])
            + (-1.796502e-03) * (q[i1] * q[i4]) + (2.033053e-04) * (q[i1] * q[i5]) + (-2.573820e-04) * (q[i1] * q[i6]) + (-4.702745e-04) * (q[i1] * q[i7])
            + (8.293673e-04) * (q[i1] * q[i8]) + (-2.220626e-04) * (q[i1] * q[i9]) + (-1.021766e-04) * (q[i1] * q[i10]) + (5.996905e-04) * (q[i1] * q[i12])
            + (1.303593e-03) * (q[i1] * q[i15]) + (-4.838039e-04) * (q[i1] * q[i16]) + (-1.021959e-03) * (q[i1] * q[i19]) + (-3.769764e-05) * (q[i1] * q[i20])
            + (-5.641109e-04) * (q[i1] * q[i21]) + (-1.102424e-04) * (q[i1] * q[i22]) + (-7.970426e-04) * (q[i2] * q[i3]) + (-1.389473e-03) * (q[i2] * q[i4])
            + (-7.976358e-04) * (q[i2] * q[i5]) + (-9.012448e-04) * (q[i2] * q[i6]) + (2.739008e-04) * (q[i2] * q[i7]) + (-1.958002e-04) * (q[i2] * q[i8])
            + (-5.052321e-04) * (q[i2] * q[i9]) + (5.983864e-04) * (q[i2] * q[i10]) + (1.333949e-03) * (q[i2] * q[i12]) + (2.093869e-03) * (q[i2] * q[i15])
            + (-4.612780e-04) * (q[i2] * q[i16]) + (-8.422558e-04) * (q[i2] * q[i19]) + (-3.052536e-04) * (q[i2] * q[i20]) + (-6.046811e-04) * (q[i2] * q[i21])
            + (-1.341003e-04) * (q[i2] * q[i22]) + (3.414564e-03) * (q[i3] * q[i4]) + (-1.901310e-03) * (q[i3] * q[i5]) + (-5.606625e-04) * (q[i3] * q[i6])
            + (2.047629e-03) * (q[i3] * q[i7]) + (1.223695e-03) * (q[i3] * q[i8]) + (-4.639914e-04) * (q[i3] * q[i9]) + (-2.395844e-04) * (q[i3] * q[i10])
            + (-6.716568e-04) * (q[i3] * q[i12]) + (-9.556197e-04) * (q[i3] * q[i15]) + (-8.723860e-04) * (q[i3] * q[i16]) + (-2.145832e-04) * (q[i3] * q[i19])
            + (-1.368810e-03) * (q[i3] * q[i20]) + (-8.578729e-04) * (q[i3] * q[i21]) + (-5.538045e-04) * (q[i3] * q[i22]) + (-1.607223e-03) * (q[i4] * q[i5])
            + (1.127119e-04) * (q[i4] * q[i6]) + (-6.173099e-04) * (q[i4] * q[i7]) + (-6.633114e-04) * (q[i4] * q[i8]) + (3.008557e-04) * (q[i4] * q[i9])
            + (8.557228e-04) * (q[i4] * q[i10]) + (-6.650560e-04) * (q[i4] * q[i12]) + (1.505921e-03) * (q[i4] * q[i15]) + (2.099040e-04) * (q[i4] * q[i16])
            + (3.381856e-04) * (q[i4] * q[i19]) + (1.983930e-03) * (q[i4] * q[i20]) + (9.509409e-04) * (q[i4] * q[i21]) + (2.386574e-04) * (q[i4] * q[i22])
            + (1.066903e-03) * (q[i5] * q[i6]) + (1.694044e-04) * (q[i5] * q[i7]) + (-2.579714e-03) * (q[i5] * q[i8]) + (-2.721587e-04) * (q[i5] * q[i9])
            + (-5.485953e-05) * (q[i5] * q[i10]) + (1.140986e-03) * (q[i5] * q[i12]) + (-2.709833e-03) * (q[i5] * q[i15]) + (4.243776e-04) * (q[i5] * q[i16])
            + (-1.004278e-03) * (q[i5] * q[i19]) + (-2.755955e-04) * (q[i5] * q[i20]) + (-1.054120e-03) * (q[i5] * q[i21]) + (4.779538e-04) * (q[i5] * q[i22])
            + (-8.060847e-04) * (q[i6] * q[i7]) + (1.840413e-03) * (q[i6] * q[i8]) + (-5.482627e-04) * (q[i6] * q[i9]) + (7.645532e-04) * (q[i6] * q[i10])
            + (-2.188075e-03) * (q[i6] * q[i12]) + (-8.049555e-05) * (q[i6] * q[i15]) + (1.236653e-03) * (q[i6] * q[i16]) + (5.555949e-05) * (q[i6] * q[i19])
            + (-6.663878e-04) * (q[i6] * q[i20]) + (8.746138e-04) * (q[i6] * q[i21]) + (2.380237e-04) * (q[i6] * q[i22]) + (5.344024e-04) * (q[i7] * q[i8])
            + (-4.594280e-05) * (q[i7] * q[i9]) + (-6.667777e-04) * (q[i7] * q[i10]) + (2.182957e-03) * (q[i7] * q[i12]) + (4.914696e-04) * (q[i7] * q[i15])
            + (-9.167755e-04) * (q[i7] * q[i16]) + (-7.213827e-04) * (q[i7] * q[i19]) + (6.426055e-04) * (q[i7] * q[i20]) + (-2.241782e-04) * (q[i7] * q[i21])
            + (-1.502159e-04) * (q[i7] * q[i22]) + (9.049542e-04) * (q[i8] * q[i9]) + (1.550690e-04) * (q[i8] * q[i10]) + (6.327379e-06) * (q[i8] * q[i12])
            + (2.458364e-03) * (q[i8] * q[i15]) + (2.508030e-04) * (q[i8] * q[i16]) + (-6.835541e-04) * (q[i8] * q[i19]) + (1.607269e-04) * (q[i8] * q[i20])
            + (-8.277569e-04) * (q[i8] * q[i21]) + (-1.381067e-04) * (q[i8] * q[i22]) + (1.286065e-04) * (q[i9] * q[i10]) + (-1.895063e-04) * (q[i9] * q[i12])
            + (5.412181e-04) * (q[i9] * q[i15]) + (1.706533e-04) * (q[i9] * q[i16]) + (1.998468e-04) * (q[i9] * q[i19]) + (-1.656583e-04) * (q[i9] * q[i20])
            + (4.284419e-04) * (q[i9] * q[i21]) + (1.575985e-04) * (q[i9] * q[i22]) + (1.848216e-04) * (q[i10] * q[i12]) + (8.219186e-04) * (q[i10] * q[i15])
            + (-3.676150e-05) * (q[i10] * q[i16]) + (-2.833224e-04) * (q[i10] * q[i19]) + (-7.525957e-05) * (q[i10] * q[i20])
            + (2.175368e-04) * (q[i10] * q[i21]) + (-3.807429e-05) * (q[i10] * q[i22]) + (-9.738384e-05) * (q[i12] * q[i15])
            + (7.708771e-05) * (q[i12] * q[i16]) + (2.126881e-04) * (q[i12] * q[i19]) + (-2.092707e-04) * (q[i12] * q[i20]) + (2.520394e-04) * (q[i12] * q[i21])
            + (2.587395e-04) * (q[i12] * q[i22]) + (4.304277e-06) * (q[i15] * q[i16]) + (4.307798e-04) * (q[i15] * q[i19]) + (-2.607958e-04) * (q[i15] * q[i20])
            + (1.367234e-03) * (q[i15] * q[i21]) + (2.065659e-04) * (q[i15] * q[i22]) + (2.303527e-05) * (q[i16] * q[i19]) + (-4.123443e-05) * (q[i16] * q[i20])
            + (1.331680e-04) * (q[i16] * q[i21]) + (2.692128e-05) * (q[i16] * q[i22]) + (6.704706e-05) * (q[i19] * q[i20]) + (1.175299e-03) * (q[i19] * q[i21])
            + (6.339400e-05) * (q[i19] * q[i22]) + (9.226390e-05) * (q[i20] * q[i21]) + (-1.246492e-04) * (q[i20] * q[i22])
            + (3.448838e-05) * (q[i21] * q[i22]);
      JQ[3][i12] = (4.368614e-02) * (1) + (1.634584e-04) * ((2) * q[i12]) + (-8.091329e-05) * (q[i0]) + (-6.749129e-04) * (q[i1]) + (-1.403663e-03) * (q[i2])
            + (8.395165e-04) * (q[i3]) + (2.268258e-05) * (q[i4]) + (4.062875e-03) * (q[i5]) + (1.502845e-03) * (q[i6]) + (9.662553e-04) * (q[i7])
            + (-3.324383e-03) * (q[i8]) + (2.079217e-03) * (q[i9]) + (1.907630e-03) * (q[i10]) + (-4.592529e-05) * (q[i11]) + (8.579139e-04) * (q[i15])
            + (-4.133825e-04) * (q[i16]) + (9.631199e-04) * (q[i19]) + (2.098065e-03) * (q[i20]) + (-6.710139e-04) * (q[i21]) + (-5.392673e-03) * (q[i22])
            + (2.299991e-04) * (q[i0] * q[i0]) + (-9.689934e-04) * (q[i1] * q[i1]) + (-1.941948e-04) * (q[i2] * q[i2]) + (-1.310267e-03) * (q[i3] * q[i3])
            + (1.310350e-04) * (q[i4] * q[i4]) + (1.766628e-03) * (q[i5] * q[i5]) + (5.813818e-04) * (q[i6] * q[i6]) + (-1.196945e-03) * (q[i7] * q[i7])
            + (-3.547921e-03) * (q[i8] * q[i8]) + (9.194117e-05) * (q[i9] * q[i9]) + (6.403050e-06) * (q[i10] * q[i10]) + (3.223710e-04) * (q[i11] * q[i11])
            + (1.959393e-06) * ((2) * q[i0] * q[i12]) + (-6.008980e-04) * ((2) * q[i1] * q[i12]) + (-7.279767e-04) * ((2) * q[i2] * q[i12])
            + (-9.127081e-04) * ((2) * q[i3] * q[i12]) + (-4.720268e-04) * ((2) * q[i4] * q[i12]) + (-4.685750e-04) * ((2) * q[i5] * q[i12])
            + (1.156984e-03) * ((2) * q[i6] * q[i12]) + (-1.382291e-03) * ((2) * q[i7] * q[i12]) + (2.941201e-03) * ((2) * q[i8] * q[i12])
            + (2.177714e-04) * ((2) * q[i9] * q[i12]) + (-6.095788e-05) * ((2) * q[i10] * q[i12]) + (3.280248e-04) * ((2) * q[i11] * q[i12])
            + (-1.105121e-03) * ((3) * q[i12] * q[i12]) + (4.546132e-05) * ((2) * q[i12] * q[i15]) + (-1.535927e-03) * ((2) * q[i12] * q[i16])
            + (-5.515674e-05) * ((2) * q[i12] * q[i19]) + (9.596390e-04) * ((2) * q[i12] * q[i20]) + (-8.149968e-05) * ((2) * q[i12] * q[i21])
            + (-7.300253e-04) * ((2) * q[i12] * q[i22]) + (2.057077e-04) * (q[i15] * q[i15]) + (-6.247284e-03) * (q[i16] * q[i16])
            + (3.809914e-05) * (q[i19] * q[i19]) + (8.428841e-04) * (q[i20] * q[i20]) + (1.356424e-04) * (q[i21] * q[i21]) + (-3.197375e-04) * (q[i22] * q[i22])
            + (4.886039e-04) * (q[i0] * q[i1]) + (8.559853e-04) * (q[i0] * q[i2]) + (-1.783963e-03) * (q[i0] * q[i3]) + (1.205081e-04) * (q[i0] * q[i4])
            + (1.904859e-04) * (q[i0] * q[i5]) + (4.855654e-04) * (q[i0] * q[i6]) + (2.767898e-04) * (q[i0] * q[i7]) + (-8.340509e-04) * (q[i0] * q[i8])
            + (1.019386e-04) * (q[i0] * q[i9]) + (2.376944e-04) * (q[i0] * q[i10]) + (5.993503e-04) * (q[i0] * q[i11]) + (4.838951e-04) * (q[i0] * q[i15])
            + (-1.300320e-03) * (q[i0] * q[i16]) + (3.978069e-05) * (q[i0] * q[i19]) + (1.015993e-03) * (q[i0] * q[i20]) + (-1.147600e-04) * (q[i0] * q[i21])
            + (-5.634708e-04) * (q[i0] * q[i22]) + (3.347439e-04) * (q[i1] * q[i2]) + (-6.064450e-04) * (q[i1] * q[i3]) + (-4.375266e-04) * (q[i1] * q[i4])
            + (4.199434e-04) * (q[i1] * q[i5]) + (6.048126e-04) * (q[i1] * q[i6]) + (-2.142420e-04) * (q[i1] * q[i7]) + (4.379223e-04) * (q[i1] * q[i8])
            + (-3.510685e-04) * (q[i1] * q[i9]) + (1.458899e-04) * (q[i1] * q[i10]) + (5.996905e-04) * (q[i1] * q[i11]) + (-1.792892e-04) * (q[i1] * q[i15])
            + (-1.087714e-03) * (q[i1] * q[i16]) + (9.876071e-04) * (q[i1] * q[i19]) + (-3.163449e-04) * (q[i1] * q[i20]) + (3.741781e-04) * (q[i1] * q[i21])
            + (2.672405e-04) * (q[i1] * q[i22]) + (-1.430941e-03) * (q[i2] * q[i3]) + (-8.046657e-04) * (q[i2] * q[i4]) + (-8.642104e-04) * (q[i2] * q[i5])
            + (-2.685933e-04) * (q[i2] * q[i6]) + (9.051833e-04) * (q[i2] * q[i7]) + (1.600792e-04) * (q[i2] * q[i8]) + (-5.898781e-04) * (q[i2] * q[i9])
            + (5.127200e-04) * (q[i2] * q[i10]) + (1.333949e-03) * (q[i2] * q[i11]) + (4.733281e-04) * (q[i2] * q[i15]) + (-2.107401e-03) * (q[i2] * q[i16])
            + (2.915829e-04) * (q[i2] * q[i19]) + (8.216694e-04) * (q[i2] * q[i20]) + (-1.392476e-04) * (q[i2] * q[i21]) + (-5.861205e-04) * (q[i2] * q[i22])
            + (3.431121e-03) * (q[i3] * q[i4]) + (-1.684910e-03) * (q[i3] * q[i5]) + (6.150328e-04) * (q[i3] * q[i6]) + (-1.334634e-04) * (q[i3] * q[i7])
            + (6.380943e-04) * (q[i3] * q[i8]) + (-8.660618e-04) * (q[i3] * q[i9]) + (-3.029864e-04) * (q[i3] * q[i10]) + (-6.716568e-04) * (q[i3] * q[i11])
            + (-2.005103e-04) * (q[i3] * q[i15]) + (-1.512364e-03) * (q[i3] * q[i16]) + (-1.993557e-03) * (q[i3] * q[i19]) + (-3.306156e-04) * (q[i3] * q[i20])
            + (2.418505e-04) * (q[i3] * q[i21]) + (9.448845e-04) * (q[i3] * q[i22]) + (-1.913452e-03) * (q[i4] * q[i5]) + (-2.077972e-03) * (q[i4] * q[i6])
            + (5.386931e-04) * (q[i4] * q[i7]) + (-1.297709e-03) * (q[i4] * q[i8]) + (2.362237e-04) * (q[i4] * q[i9]) + (4.610310e-04) * (q[i4] * q[i10])
            + (-6.650560e-04) * (q[i4] * q[i11]) + (8.698201e-04) * (q[i4] * q[i15]) + (9.665469e-04) * (q[i4] * q[i16]) + (1.368546e-03) * (q[i4] * q[i19])
            + (2.078426e-04) * (q[i4] * q[i20]) + (-5.561184e-04) * (q[i4] * q[i21]) + (-8.644089e-04) * (q[i4] * q[i22]) + (-1.394643e-04) * (q[i5] * q[i6])
            + (-1.035892e-03) * (q[i5] * q[i7]) + (2.749042e-03) * (q[i5] * q[i8]) + (6.765033e-05) * (q[i5] * q[i9]) + (2.937166e-04) * (q[i5] * q[i10])
            + (1.140986e-03) * (q[i5] * q[i11]) + (-4.287751e-04) * (q[i5] * q[i15]) + (2.717488e-03) * (q[i5] * q[i16]) + (2.775053e-04) * (q[i5] * q[i19])
            + (9.851661e-04) * (q[i5] * q[i20]) + (4.795968e-04) * (q[i5] * q[i21]) + (-1.037047e-03) * (q[i5] * q[i22]) + (-8.094762e-04) * (q[i6] * q[i7])
            + (5.693488e-04) * (q[i6] * q[i8]) + (-6.646076e-04) * (q[i6] * q[i9]) + (-4.659777e-05) * (q[i6] * q[i10]) + (-2.188075e-03) * (q[i6] * q[i11])
            + (-9.200743e-04) * (q[i6] * q[i15]) + (4.833148e-04) * (q[i6] * q[i16]) + (6.442033e-04) * (q[i6] * q[i19]) + (-7.282235e-04) * (q[i6] * q[i20])
            + (1.442067e-04) * (q[i6] * q[i21]) + (2.225201e-04) * (q[i6] * q[i22]) + (1.838683e-03) * (q[i7] * q[i8]) + (7.649316e-04) * (q[i7] * q[i9])
            + (-5.476878e-04) * (q[i7] * q[i10]) + (2.182957e-03) * (q[i7] * q[i11]) + (1.238090e-03) * (q[i7] * q[i15]) + (-8.871230e-05) * (q[i7] * q[i16])
            + (-6.713509e-04) * (q[i7] * q[i19]) + (4.993456e-05) * (q[i7] * q[i20]) + (-2.370325e-04) * (q[i7] * q[i21]) + (-8.710689e-04) * (q[i7] * q[i22])
            + (1.520027e-04) * (q[i8] * q[i9]) + (9.047453e-04) * (q[i8] * q[i10]) + (6.327379e-06) * (q[i8] * q[i11]) + (2.457640e-04) * (q[i8] * q[i15])
            + (2.461207e-03) * (q[i8] * q[i16]) + (1.680650e-04) * (q[i8] * q[i19]) + (-6.569697e-04) * (q[i8] * q[i20]) + (1.356376e-04) * (q[i8] * q[i21])
            + (8.294795e-04) * (q[i8] * q[i22]) + (1.250044e-04) * (q[i9] * q[i10]) + (-1.895063e-04) * (q[i9] * q[i11]) + (-4.133039e-05) * (q[i9] * q[i15])
            + (8.252603e-04) * (q[i9] * q[i16]) + (-8.015074e-05) * (q[i9] * q[i19]) + (-2.905708e-04) * (q[i9] * q[i20]) + (3.768899e-05) * (q[i9] * q[i21])
            + (-2.169850e-04) * (q[i9] * q[i22]) + (1.848216e-04) * (q[i10] * q[i11]) + (1.745728e-04) * (q[i10] * q[i15]) + (5.422234e-04) * (q[i10] * q[i16])
            + (-1.673986e-04) * (q[i10] * q[i19]) + (2.034290e-04) * (q[i10] * q[i20]) + (-1.576923e-04) * (q[i10] * q[i21])
            + (-4.287859e-04) * (q[i10] * q[i22]) + (-9.738384e-05) * (q[i11] * q[i15]) + (7.708771e-05) * (q[i11] * q[i16])
            + (2.126881e-04) * (q[i11] * q[i19]) + (-2.092707e-04) * (q[i11] * q[i20]) + (2.520394e-04) * (q[i11] * q[i21]) + (2.587395e-04) * (q[i11] * q[i22])
            + (-4.345753e-06) * (q[i15] * q[i16]) + (-4.131142e-05) * (q[i15] * q[i19]) + (2.314826e-05) * (q[i15] * q[i20])
            + (-3.017616e-05) * (q[i15] * q[i21]) + (-1.395952e-04) * (q[i15] * q[i22]) + (-2.640523e-04) * (q[i16] * q[i19])
            + (4.473115e-04) * (q[i16] * q[i20]) + (-2.065151e-04) * (q[i16] * q[i21]) + (-1.411102e-03) * (q[i16] * q[i22])
            + (7.135416e-05) * (q[i19] * q[i20]) + (1.233256e-04) * (q[i19] * q[i21]) + (-9.640700e-05) * (q[i19] * q[i22])
            + (-6.256477e-05) * (q[i20] * q[i21]) + (-1.167479e-03) * (q[i20] * q[i22]) + (3.444092e-05) * (q[i21] * q[i22]);
      JQ[3][i15] = (8.570411e-03) * (1) + (3.095329e-03) * ((2) * q[i15]) + (-9.541826e-03) * (q[i0]) + (-9.011323e-03) * (q[i1]) + (-1.970557e-02) * (q[i2])
            + (4.887112e-03) * (q[i3]) + (1.618154e-04) * (q[i4]) + (-3.040717e-03) * (q[i5]) + (5.385031e-04) * (q[i6]) + (-9.219974e-03) * (q[i7])
            + (1.215025e-02) * (q[i8]) + (-5.601694e-04) * (q[i9]) + (-3.640950e-04) * (q[i10]) + (-3.116551e-04) * (q[i11]) + (8.579139e-04) * (q[i12])
            + (3.477047e-06) * (q[i16]) + (4.399609e-03) * (q[i19]) + (-1.002143e-03) * (q[i20]) + (2.999294e-03) * (q[i21]) + (4.596933e-04) * (q[i22])
            + (2.123896e-04) * (q[i0] * q[i0]) + (8.221723e-04) * (q[i1] * q[i1]) + (-2.909282e-04) * (q[i2] * q[i2]) + (-1.179013e-03) * (q[i3] * q[i3])
            + (9.697352e-05) * (q[i4] * q[i4]) + (3.443046e-03) * (q[i5] * q[i5]) + (1.371895e-03) * (q[i6] * q[i6]) + (-7.038177e-04) * (q[i7] * q[i7])
            + (-3.006602e-03) * (q[i8] * q[i8]) + (4.820502e-05) * (q[i9] * q[i9]) + (3.212800e-04) * (q[i10] * q[i10]) + (1.537403e-03) * (q[i11] * q[i11])
            + (4.546132e-05) * (q[i12] * q[i12]) + (-2.262076e-03) * ((2) * q[i0] * q[i15]) + (-2.313700e-03) * ((2) * q[i1] * q[i15])
            + (-4.514467e-03) * ((2) * q[i2] * q[i15]) + (5.518776e-05) * ((2) * q[i3] * q[i15]) + (4.612167e-04) * ((2) * q[i4] * q[i15])
            + (-2.098538e-03) * ((2) * q[i5] * q[i15]) + (1.037642e-03) * ((2) * q[i6] * q[i15]) + (-1.213451e-03) * ((2) * q[i7] * q[i15])
            + (-2.416944e-03) * ((2) * q[i8] * q[i15]) + (1.615056e-04) * ((2) * q[i9] * q[i15]) + (3.116321e-04) * ((2) * q[i10] * q[i15])
            + (-6.175304e-03) * ((2) * q[i11] * q[i15]) + (2.057077e-04) * ((2) * q[i12] * q[i15]) + (-2.868432e-04) * ((3) * q[i15] * q[i15])
            + (-2.078434e-04) * ((2) * q[i15] * q[i16]) + (6.294158e-04) * ((2) * q[i15] * q[i19]) + (1.619278e-04) * ((2) * q[i15] * q[i20])
            + (1.051880e-03) * ((2) * q[i15] * q[i21]) + (8.621996e-05) * ((2) * q[i15] * q[i22]) + (2.072165e-04) * (q[i16] * q[i16])
            + (4.075845e-04) * (q[i19] * q[i19]) + (-1.337070e-05) * (q[i20] * q[i20]) + (1.073933e-04) * (q[i21] * q[i21]) + (3.600029e-05) * (q[i22] * q[i22])
            + (-1.055647e-03) * (q[i0] * q[i1]) + (-1.028915e-04) * (q[i0] * q[i2]) + (3.448991e-04) * (q[i0] * q[i3]) + (-3.152945e-03) * (q[i0] * q[i4])
            + (3.505028e-03) * (q[i0] * q[i5]) + (5.780722e-04) * (q[i0] * q[i6]) + (7.127504e-04) * (q[i0] * q[i7]) + (-1.039356e-03) * (q[i0] * q[i8])
            + (3.218487e-05) * (q[i0] * q[i9]) + (1.978951e-04) * (q[i0] * q[i10]) + (1.081469e-03) * (q[i0] * q[i11]) + (4.838951e-04) * (q[i0] * q[i12])
            + (3.132837e-07) * (q[i0] * q[i16]) + (1.295461e-04) * (q[i0] * q[i19]) + (3.283325e-05) * (q[i0] * q[i20]) + (-1.167079e-03) * (q[i0] * q[i21])
            + (-1.190509e-04) * (q[i0] * q[i22]) + (4.658728e-04) * (q[i1] * q[i2]) + (1.111182e-03) * (q[i1] * q[i3]) + (3.354312e-03) * (q[i1] * q[i4])
            + (3.178904e-03) * (q[i1] * q[i5]) + (-6.323197e-04) * (q[i1] * q[i6]) + (9.428895e-04) * (q[i1] * q[i7]) + (-2.278029e-03) * (q[i1] * q[i8])
            + (4.011463e-05) * (q[i1] * q[i9]) + (2.651569e-05) * (q[i1] * q[i10]) + (1.303593e-03) * (q[i1] * q[i11]) + (-1.792892e-04) * (q[i1] * q[i12])
            + (-5.778187e-06) * (q[i1] * q[i16]) + (-4.077494e-04) * (q[i1] * q[i19]) + (3.002936e-04) * (q[i1] * q[i20]) + (-7.800937e-04) * (q[i1] * q[i21])
            + (3.931238e-04) * (q[i1] * q[i22]) + (2.462569e-03) * (q[i2] * q[i3]) + (9.279543e-04) * (q[i2] * q[i4]) + (5.182532e-03) * (q[i2] * q[i5])
            + (-1.495753e-04) * (q[i2] * q[i6]) + (-6.872632e-04) * (q[i2] * q[i7]) + (-2.168129e-03) * (q[i2] * q[i8]) + (2.126291e-04) * (q[i2] * q[i9])
            + (-2.269576e-04) * (q[i2] * q[i10]) + (2.093869e-03) * (q[i2] * q[i11]) + (4.733281e-04) * (q[i2] * q[i12]) + (-2.080770e-04) * (q[i2] * q[i16])
            + (-1.433018e-04) * (q[i2] * q[i19]) + (4.225078e-04) * (q[i2] * q[i20]) + (-1.630412e-03) * (q[i2] * q[i21]) + (-3.246137e-04) * (q[i2] * q[i22])
            + (9.870022e-04) * (q[i3] * q[i4]) + (3.770076e-04) * (q[i3] * q[i5]) + (2.190919e-03) * (q[i3] * q[i6]) + (8.216297e-04) * (q[i3] * q[i7])
            + (-1.681315e-03) * (q[i3] * q[i8]) + (-3.666855e-04) * (q[i3] * q[i9]) + (-2.730054e-04) * (q[i3] * q[i10]) + (-9.556197e-04) * (q[i3] * q[i11])
            + (-2.005103e-04) * (q[i3] * q[i12]) + (2.289200e-04) * (q[i3] * q[i16]) + (1.308130e-04) * (q[i3] * q[i19]) + (4.418298e-04) * (q[i3] * q[i20])
            + (2.398466e-04) * (q[i3] * q[i21]) + (1.323404e-04) * (q[i3] * q[i22]) + (-2.212603e-03) * (q[i4] * q[i5]) + (-1.289673e-04) * (q[i4] * q[i6])
            + (1.469082e-03) * (q[i4] * q[i7]) + (7.776032e-04) * (q[i4] * q[i8]) + (-3.479641e-04) * (q[i4] * q[i9]) + (6.379221e-04) * (q[i4] * q[i10])
            + (1.505921e-03) * (q[i4] * q[i11]) + (8.698201e-04) * (q[i4] * q[i12]) + (2.368711e-04) * (q[i4] * q[i16]) + (8.908017e-04) * (q[i4] * q[i19])
            + (-2.271586e-06) * (q[i4] * q[i20]) + (-5.666974e-04) * (q[i4] * q[i21]) + (-7.345003e-04) * (q[i4] * q[i22]) + (-2.992203e-03) * (q[i5] * q[i6])
            + (-1.588615e-03) * (q[i5] * q[i7]) + (-2.099753e-03) * (q[i5] * q[i8]) + (6.044531e-04) * (q[i5] * q[i9]) + (-7.002994e-05) * (q[i5] * q[i10])
            + (-2.709833e-03) * (q[i5] * q[i11]) + (-4.287751e-04) * (q[i5] * q[i12]) + (1.007694e-03) * (q[i5] * q[i16]) + (5.705245e-04) * (q[i5] * q[i19])
            + (-3.208123e-04) * (q[i5] * q[i20]) + (1.573328e-04) * (q[i5] * q[i21]) + (-1.446802e-04) * (q[i5] * q[i22]) + (-1.212214e-03) * (q[i6] * q[i7])
            + (6.057846e-04) * (q[i6] * q[i8]) + (5.592634e-04) * (q[i6] * q[i9]) + (4.928168e-05) * (q[i6] * q[i10]) + (-8.049555e-05) * (q[i6] * q[i11])
            + (-9.200743e-04) * (q[i6] * q[i12]) + (7.888703e-04) * (q[i6] * q[i16]) + (6.141175e-05) * (q[i6] * q[i19]) + (-2.685348e-04) * (q[i6] * q[i20])
            + (-1.138220e-03) * (q[i6] * q[i21]) + (5.901552e-04) * (q[i6] * q[i22]) + (2.249842e-03) * (q[i7] * q[i8]) + (-5.560971e-04) * (q[i7] * q[i9])
            + (3.970188e-04) * (q[i7] * q[i10]) + (4.914696e-04) * (q[i7] * q[i11]) + (1.238090e-03) * (q[i7] * q[i12]) + (-7.862352e-04) * (q[i7] * q[i16])
            + (-6.389798e-04) * (q[i7] * q[i19]) + (-9.214458e-05) * (q[i7] * q[i20]) + (2.972375e-05) * (q[i7] * q[i21]) + (2.369613e-04) * (q[i7] * q[i22])
            + (-9.073218e-06) * (q[i8] * q[i9]) + (1.210958e-03) * (q[i8] * q[i10]) + (2.458364e-03) * (q[i8] * q[i11]) + (2.457640e-04) * (q[i8] * q[i12])
            + (8.123149e-06) * (q[i8] * q[i16]) + (7.750505e-04) * (q[i8] * q[i19]) + (1.638484e-04) * (q[i8] * q[i20]) + (2.465250e-03) * (q[i8] * q[i21])
            + (-1.715073e-04) * (q[i8] * q[i22]) + (-3.606545e-04) * (q[i9] * q[i10]) + (5.412181e-04) * (q[i9] * q[i11]) + (-4.133039e-05) * (q[i9] * q[i12])
            + (-2.230831e-04) * (q[i9] * q[i16]) + (-5.647189e-05) * (q[i9] * q[i19]) + (1.652257e-04) * (q[i9] * q[i20]) + (-4.322648e-04) * (q[i9] * q[i21])
            + (5.285697e-05) * (q[i9] * q[i22]) + (8.219186e-04) * (q[i10] * q[i11]) + (1.745728e-04) * (q[i10] * q[i12]) + (2.259030e-04) * (q[i10] * q[i16])
            + (-1.702933e-04) * (q[i10] * q[i19]) + (3.417873e-05) * (q[i10] * q[i20]) + (4.745517e-05) * (q[i10] * q[i21])
            + (-2.623439e-05) * (q[i10] * q[i22]) + (-9.738384e-05) * (q[i11] * q[i12]) + (4.304277e-06) * (q[i11] * q[i16])
            + (4.307798e-04) * (q[i11] * q[i19]) + (-2.607958e-04) * (q[i11] * q[i20]) + (1.367234e-03) * (q[i11] * q[i21]) + (2.065659e-04) * (q[i11] * q[i22])
            + (-4.345753e-06) * (q[i12] * q[i16]) + (-4.131142e-05) * (q[i12] * q[i19]) + (2.314826e-05) * (q[i12] * q[i20])
            + (-3.017616e-05) * (q[i12] * q[i21]) + (-1.395952e-04) * (q[i12] * q[i22]) + (2.386171e-04) * (q[i16] * q[i19])
            + (-2.361487e-04) * (q[i16] * q[i20]) + (-2.004290e-04) * (q[i16] * q[i21]) + (-2.019045e-04) * (q[i16] * q[i22])
            + (3.712005e-06) * (q[i19] * q[i20]) + (4.275000e-04) * (q[i19] * q[i21]) + (4.808193e-05) * (q[i19] * q[i22]) + (-1.281918e-04) * (q[i20] * q[i21])
            + (9.076848e-05) * (q[i20] * q[i22]) + (1.820690e-04) * (q[i21] * q[i22]);
      JQ[3][i16] = (-8.773902e-03) * (1) + (-3.092488e-03) * ((2) * q[i16]) + (-9.139200e-03) * (q[i0]) + (-9.624126e-03) * (q[i1]) + (-1.993201e-02) * (q[i2])
            + (1.684864e-04) * (q[i3]) + (4.859843e-03) * (q[i4]) + (-3.044449e-03) * (q[i5]) + (9.371454e-03) * (q[i6]) + (-5.866709e-04) * (q[i7])
            + (-1.220530e-02) * (q[i8]) + (3.954755e-04) * (q[i9]) + (5.470199e-04) * (q[i10]) + (8.965799e-04) * (q[i11]) + (-4.133825e-04) * (q[i12])
            + (3.477047e-06) * (q[i15]) + (1.017425e-03) * (q[i19]) + (-4.327457e-03) * (q[i20]) + (4.671434e-04) * (q[i21]) + (3.021687e-03) * (q[i22])
            + (-8.236463e-04) * (q[i0] * q[i0]) + (-2.117819e-04) * (q[i1] * q[i1]) + (2.973533e-04) * (q[i2] * q[i2]) + (-9.294659e-05) * (q[i3] * q[i3])
            + (1.172453e-03) * (q[i4] * q[i4]) + (-3.470037e-03) * (q[i5] * q[i5]) + (7.176423e-04) * (q[i6] * q[i6]) + (-1.368118e-03) * (q[i7] * q[i7])
            + (3.066301e-03) * (q[i8] * q[i8]) + (-3.311503e-04) * (q[i9] * q[i9]) + (-4.669194e-05) * (q[i10] * q[i10]) + (-4.745409e-05) * (q[i11] * q[i11])
            + (-1.535927e-03) * (q[i12] * q[i12]) + (-2.078434e-04) * (q[i15] * q[i15]) + (-2.350594e-03) * ((2) * q[i0] * q[i16])
            + (-2.272204e-03) * ((2) * q[i1] * q[i16]) + (-4.559418e-03) * ((2) * q[i2] * q[i16]) + (4.668928e-04) * ((2) * q[i3] * q[i16])
            + (5.548368e-05) * ((2) * q[i4] * q[i16]) + (-2.123313e-03) * ((2) * q[i5] * q[i16]) + (1.226560e-03) * ((2) * q[i6] * q[i16])
            + (-1.050098e-03) * ((2) * q[i7] * q[i16]) + (2.450313e-03) * ((2) * q[i8] * q[i16]) + (-3.178814e-04) * ((2) * q[i9] * q[i16])
            + (-1.635291e-04) * ((2) * q[i10] * q[i16]) + (2.060527e-04) * ((2) * q[i11] * q[i16]) + (-6.247284e-03) * ((2) * q[i12] * q[i16])
            + (2.072165e-04) * ((2) * q[i15] * q[i16]) + (2.937789e-04) * ((3) * q[i16] * q[i16]) + (-1.587213e-04) * ((2) * q[i16] * q[i19])
            + (-6.310745e-04) * ((2) * q[i16] * q[i20]) + (8.688550e-05) * ((2) * q[i16] * q[i21]) + (1.061528e-03) * ((2) * q[i16] * q[i22])
            + (1.511783e-05) * (q[i19] * q[i19]) + (-3.944168e-04) * (q[i20] * q[i20]) + (-3.701730e-05) * (q[i21] * q[i21])
            + (-1.055802e-04) * (q[i22] * q[i22]) + (1.063611e-03) * (q[i0] * q[i1]) + (-4.698589e-04) * (q[i0] * q[i2]) + (-3.403611e-03) * (q[i0] * q[i3])
            + (-1.142440e-03) * (q[i0] * q[i4]) + (-3.198981e-03) * (q[i0] * q[i5]) + (9.414677e-04) * (q[i0] * q[i6]) + (-6.352356e-04) * (q[i0] * q[i7])
            + (-2.300283e-03) * (q[i0] * q[i8]) + (2.249269e-05) * (q[i0] * q[i9]) + (4.330330e-05) * (q[i0] * q[i10]) + (1.854720e-04) * (q[i0] * q[i11])
            + (-1.300320e-03) * (q[i0] * q[i12]) + (3.132837e-07) * (q[i0] * q[i15]) + (3.027109e-04) * (q[i0] * q[i19]) + (-3.938751e-04) * (q[i0] * q[i20])
            + (-4.024322e-04) * (q[i0] * q[i21]) + (7.784233e-04) * (q[i0] * q[i22]) + (1.066038e-04) * (q[i1] * q[i2]) + (3.177512e-03) * (q[i1] * q[i3])
            + (-3.226464e-04) * (q[i1] * q[i4]) + (-3.545824e-03) * (q[i1] * q[i5]) + (6.999986e-04) * (q[i1] * q[i6]) + (5.748451e-04) * (q[i1] * q[i7])
            + (-1.046418e-03) * (q[i1] * q[i8]) + (1.947114e-04) * (q[i1] * q[i9]) + (3.412348e-05) * (q[i1] * q[i10]) + (-4.838039e-04) * (q[i1] * q[i11])
            + (-1.087714e-03) * (q[i1] * q[i12]) + (-5.778187e-06) * (q[i1] * q[i15]) + (2.687331e-05) * (q[i1] * q[i19]) + (1.281700e-04) * (q[i1] * q[i20])
            + (1.202486e-04) * (q[i1] * q[i21]) + (1.171665e-03) * (q[i1] * q[i22]) + (-9.440405e-04) * (q[i2] * q[i3]) + (-2.494502e-03) * (q[i2] * q[i4])
            + (-5.230026e-03) * (q[i2] * q[i5]) + (-7.048554e-04) * (q[i2] * q[i6]) + (-1.709576e-04) * (q[i2] * q[i7]) + (-2.187043e-03) * (q[i2] * q[i8])
            + (-2.350800e-04) * (q[i2] * q[i9]) + (2.115800e-04) * (q[i2] * q[i10]) + (-4.612780e-04) * (q[i2] * q[i11]) + (-2.107401e-03) * (q[i2] * q[i12])
            + (-2.080770e-04) * (q[i2] * q[i15]) + (4.207257e-04) * (q[i2] * q[i19]) + (-1.308262e-04) * (q[i2] * q[i20]) + (3.203257e-04) * (q[i2] * q[i21])
            + (1.635022e-03) * (q[i2] * q[i22]) + (-9.768833e-04) * (q[i3] * q[i4]) + (2.227935e-03) * (q[i3] * q[i5]) + (1.475704e-03) * (q[i3] * q[i6])
            + (-9.833456e-05) * (q[i3] * q[i7]) + (7.726770e-04) * (q[i3] * q[i8]) + (6.472386e-04) * (q[i3] * q[i9]) + (-3.540169e-04) * (q[i3] * q[i10])
            + (-8.723860e-04) * (q[i3] * q[i11]) + (-1.512364e-03) * (q[i3] * q[i12]) + (2.289200e-04) * (q[i3] * q[i15]) + (4.027790e-06) * (q[i3] * q[i19])
            + (8.806233e-04) * (q[i3] * q[i20]) + (7.408101e-04) * (q[i3] * q[i21]) + (5.767705e-04) * (q[i3] * q[i22]) + (-3.527507e-04) * (q[i4] * q[i5])
            + (8.370493e-04) * (q[i4] * q[i6]) + (2.197919e-03) * (q[i4] * q[i7]) + (-1.691448e-03) * (q[i4] * q[i8]) + (-2.630869e-04) * (q[i4] * q[i9])
            + (-3.588534e-04) * (q[i4] * q[i10]) + (2.099040e-04) * (q[i4] * q[i11]) + (9.665469e-04) * (q[i4] * q[i12]) + (2.368711e-04) * (q[i4] * q[i15])
            + (4.364204e-04) * (q[i4] * q[i19]) + (1.314712e-04) * (q[i4] * q[i20]) + (-1.290802e-04) * (q[i4] * q[i21]) + (-2.341977e-04) * (q[i4] * q[i22])
            + (-1.591540e-03) * (q[i5] * q[i6]) + (-3.005338e-03) * (q[i5] * q[i7]) + (-2.120832e-03) * (q[i5] * q[i8]) + (-8.888490e-05) * (q[i5] * q[i9])
            + (6.086860e-04) * (q[i5] * q[i10]) + (4.243776e-04) * (q[i5] * q[i11]) + (2.717488e-03) * (q[i5] * q[i12]) + (1.007694e-03) * (q[i5] * q[i15])
            + (-3.259581e-04) * (q[i5] * q[i19]) + (5.767805e-04) * (q[i5] * q[i20]) + (1.445475e-04) * (q[i5] * q[i21]) + (-1.700077e-04) * (q[i5] * q[i22])
            + (1.210781e-03) * (q[i6] * q[i7]) + (-2.253623e-03) * (q[i6] * q[i8]) + (-4.138171e-04) * (q[i6] * q[i9]) + (5.564520e-04) * (q[i6] * q[i10])
            + (1.236653e-03) * (q[i6] * q[i11]) + (4.833148e-04) * (q[i6] * q[i12]) + (7.888703e-04) * (q[i6] * q[i15]) + (8.896395e-05) * (q[i6] * q[i19])
            + (6.421954e-04) * (q[i6] * q[i20]) + (2.359060e-04) * (q[i6] * q[i21]) + (2.283668e-05) * (q[i6] * q[i22]) + (-6.333248e-04) * (q[i7] * q[i8])
            + (-4.782029e-05) * (q[i7] * q[i9]) + (-5.558833e-04) * (q[i7] * q[i10]) + (-9.167755e-04) * (q[i7] * q[i11]) + (-8.871230e-05) * (q[i7] * q[i12])
            + (-7.862352e-04) * (q[i7] * q[i15]) + (2.711572e-04) * (q[i7] * q[i19]) + (-6.498497e-05) * (q[i7] * q[i20]) + (5.920154e-04) * (q[i7] * q[i21])
            + (-1.133731e-03) * (q[i7] * q[i22]) + (-1.216992e-03) * (q[i8] * q[i9]) + (7.722964e-06) * (q[i8] * q[i10]) + (2.508030e-04) * (q[i8] * q[i11])
            + (2.461207e-03) * (q[i8] * q[i12]) + (8.123149e-06) * (q[i8] * q[i15]) + (-1.567777e-04) * (q[i8] * q[i19]) + (-7.861975e-04) * (q[i8] * q[i20])
            + (-1.693606e-04) * (q[i8] * q[i21]) + (2.504864e-03) * (q[i8] * q[i22]) + (3.622643e-04) * (q[i9] * q[i10]) + (1.706533e-04) * (q[i9] * q[i11])
            + (8.252603e-04) * (q[i9] * q[i12]) + (-2.230831e-04) * (q[i9] * q[i15]) + (-3.671280e-05) * (q[i9] * q[i19]) + (1.781724e-04) * (q[i9] * q[i20])
            + (-2.725853e-05) * (q[i9] * q[i21]) + (4.789656e-05) * (q[i9] * q[i22]) + (-3.676150e-05) * (q[i10] * q[i11]) + (5.422234e-04) * (q[i10] * q[i12])
            + (2.259030e-04) * (q[i10] * q[i15]) + (-1.618793e-04) * (q[i10] * q[i19]) + (5.807324e-05) * (q[i10] * q[i20]) + (5.007479e-05) * (q[i10] * q[i21])
            + (-4.296329e-04) * (q[i10] * q[i22]) + (7.708771e-05) * (q[i11] * q[i12]) + (4.304277e-06) * (q[i11] * q[i15]) + (2.303527e-05) * (q[i11] * q[i19])
            + (-4.123443e-05) * (q[i11] * q[i20]) + (1.331680e-04) * (q[i11] * q[i21]) + (2.692128e-05) * (q[i11] * q[i22])
            + (-4.345753e-06) * (q[i12] * q[i15]) + (-2.640523e-04) * (q[i12] * q[i19]) + (4.473115e-04) * (q[i12] * q[i20])
            + (-2.065151e-04) * (q[i12] * q[i21]) + (-1.411102e-03) * (q[i12] * q[i22]) + (2.386171e-04) * (q[i15] * q[i19])
            + (-2.361487e-04) * (q[i15] * q[i20]) + (-2.004290e-04) * (q[i15] * q[i21]) + (-2.019045e-04) * (q[i15] * q[i22])
            + (-2.982123e-06) * (q[i19] * q[i20]) + (9.156044e-05) * (q[i19] * q[i21]) + (-1.290174e-04) * (q[i19] * q[i22])
            + (5.082644e-05) * (q[i20] * q[i21]) + (4.165926e-04) * (q[i20] * q[i22]) + (-1.806560e-04) * (q[i21] * q[i22]);
      JQ[3][i19] = (8.838663e-03) * (1) + (-6.549195e-04) * ((2) * q[i19]) + (1.184830e-03) * (q[i0]) + (-1.508031e-03) * (q[i1]) + (1.275394e-03) * (q[i2])
            + (-2.726175e-03) * (q[i3]) + (3.983855e-03) * (q[i4]) + (2.821675e-03) * (q[i5]) + (-7.744288e-04) * (q[i6]) + (2.318131e-03) * (q[i7])
            + (1.431593e-03) * (q[i8]) + (-9.137787e-04) * (q[i9]) + (1.006726e-03) * (q[i10]) + (2.130627e-03) * (q[i11]) + (9.631199e-04) * (q[i12])
            + (4.399609e-03) * (q[i15]) + (1.017425e-03) * (q[i16]) + (3.845273e-06) * (q[i20]) + (-4.411577e-03) * (q[i21]) + (-8.428399e-04) * (q[i22])
            + (-6.370901e-05) * (q[i0] * q[i0]) + (-1.095282e-03) * (q[i1] * q[i1]) + (-5.778378e-04) * (q[i2] * q[i2]) + (1.117891e-03) * (q[i3] * q[i3])
            + (6.466786e-04) * (q[i4] * q[i4]) + (-3.159070e-04) * (q[i5] * q[i5]) + (-7.310683e-04) * (q[i6] * q[i6]) + (8.031525e-04) * (q[i7] * q[i7])
            + (6.014736e-04) * (q[i8] * q[i8]) + (-1.175025e-04) * (q[i9] * q[i9]) + (-3.655092e-04) * (q[i10] * q[i10]) + (-9.906457e-04) * (q[i11] * q[i11])
            + (-5.515674e-05) * (q[i12] * q[i12]) + (6.294158e-04) * (q[i15] * q[i15]) + (-1.587213e-04) * (q[i16] * q[i16])
            + (1.288298e-04) * ((2) * q[i0] * q[i19]) + (-3.503810e-04) * ((2) * q[i1] * q[i19]) + (-9.231063e-06) * ((2) * q[i2] * q[i19])
            + (-4.202966e-04) * ((2) * q[i3] * q[i19]) + (-2.580627e-04) * ((2) * q[i4] * q[i19]) + (1.300974e-03) * ((2) * q[i5] * q[i19])
            + (5.782373e-04) * ((2) * q[i6] * q[i19]) + (-3.725240e-04) * ((2) * q[i7] * q[i19]) + (9.640285e-04) * ((2) * q[i8] * q[i19])
            + (1.008393e-04) * ((2) * q[i9] * q[i19]) + (-4.304783e-05) * ((2) * q[i10] * q[i19]) + (8.530024e-04) * ((2) * q[i11] * q[i19])
            + (3.809914e-05) * ((2) * q[i12] * q[i19]) + (4.075845e-04) * ((2) * q[i15] * q[i19]) + (1.511783e-05) * ((2) * q[i16] * q[i19])
            + (-3.517564e-04) * ((3) * q[i19] * q[i19]) + (-1.211001e-05) * ((2) * q[i19] * q[i20]) + (-1.911368e-03) * ((2) * q[i19] * q[i21])
            + (9.038099e-06) * ((2) * q[i19] * q[i22]) + (7.361915e-06) * (q[i20] * q[i20]) + (-1.211949e-03) * (q[i21] * q[i21])
            + (5.989050e-05) * (q[i22] * q[i22]) + (8.518128e-04) * (q[i0] * q[i1]) + (-7.537860e-04) * (q[i0] * q[i2]) + (-7.449361e-04) * (q[i0] * q[i3])
            + (1.462930e-06) * (q[i0] * q[i4]) + (4.599974e-04) * (q[i0] * q[i5]) + (-5.449188e-04) * (q[i0] * q[i6]) + (-1.611498e-03) * (q[i0] * q[i7])
            + (-1.829503e-04) * (q[i0] * q[i8]) + (4.460672e-05) * (q[i0] * q[i9]) + (-5.757650e-04) * (q[i0] * q[i10]) + (2.979992e-04) * (q[i0] * q[i11])
            + (3.978069e-05) * (q[i0] * q[i12]) + (1.295461e-04) * (q[i0] * q[i15]) + (3.027109e-04) * (q[i0] * q[i16]) + (-1.269053e-04) * (q[i0] * q[i20])
            + (-6.506993e-04) * (q[i0] * q[i21]) + (1.077289e-05) * (q[i0] * q[i22]) + (8.139548e-04) * (q[i1] * q[i2]) + (1.178857e-04) * (q[i1] * q[i3])
            + (1.749162e-04) * (q[i1] * q[i4]) + (-5.702509e-04) * (q[i1] * q[i5]) + (-1.192923e-03) * (q[i1] * q[i6]) + (8.248072e-04) * (q[i1] * q[i7])
            + (2.850417e-04) * (q[i1] * q[i8]) + (3.131939e-04) * (q[i1] * q[i9]) + (1.504097e-05) * (q[i1] * q[i10]) + (-1.021959e-03) * (q[i1] * q[i11])
            + (9.876071e-04) * (q[i1] * q[i12]) + (-4.077494e-04) * (q[i1] * q[i15]) + (2.687331e-05) * (q[i1] * q[i16]) + (-1.275865e-04) * (q[i1] * q[i20])
            + (-3.065236e-04) * (q[i1] * q[i21]) + (1.984812e-04) * (q[i1] * q[i22]) + (3.471882e-04) * (q[i2] * q[i3]) + (-3.463028e-04) * (q[i2] * q[i4])
            + (-1.092606e-03) * (q[i2] * q[i5]) + (1.051281e-03) * (q[i2] * q[i6]) + (2.468784e-04) * (q[i2] * q[i7]) + (-7.860911e-04) * (q[i2] * q[i8])
            + (7.958861e-04) * (q[i2] * q[i9]) + (-7.141847e-04) * (q[i2] * q[i10]) + (-8.422558e-04) * (q[i2] * q[i11]) + (2.915829e-04) * (q[i2] * q[i12])
            + (-1.433018e-04) * (q[i2] * q[i15]) + (4.207257e-04) * (q[i2] * q[i16]) + (-5.132617e-04) * (q[i2] * q[i20]) + (-1.510791e-03) * (q[i2] * q[i21])
            + (-1.495248e-04) * (q[i2] * q[i22]) + (-2.270218e-03) * (q[i3] * q[i4]) + (-9.872696e-04) * (q[i3] * q[i5]) + (-2.351642e-03) * (q[i3] * q[i6])
            + (2.829038e-03) * (q[i3] * q[i7]) + (1.835695e-03) * (q[i3] * q[i8]) + (6.488512e-04) * (q[i3] * q[i9]) + (8.387708e-05) * (q[i3] * q[i10])
            + (-2.145832e-04) * (q[i3] * q[i11]) + (-1.993557e-03) * (q[i3] * q[i12]) + (1.308130e-04) * (q[i3] * q[i15]) + (4.027790e-06) * (q[i3] * q[i16])
            + (-1.007803e-04) * (q[i3] * q[i20]) + (9.749011e-04) * (q[i3] * q[i21]) + (-2.244236e-06) * (q[i3] * q[i22]) + (2.051015e-03) * (q[i4] * q[i5])
            + (1.840922e-03) * (q[i4] * q[i6]) + (-1.443071e-03) * (q[i4] * q[i7]) + (-1.577424e-03) * (q[i4] * q[i8]) + (-6.041358e-04) * (q[i4] * q[i9])
            + (-4.020045e-04) * (q[i4] * q[i10]) + (3.381856e-04) * (q[i4] * q[i11]) + (1.368546e-03) * (q[i4] * q[i12]) + (8.908017e-04) * (q[i4] * q[i15])
            + (4.364204e-04) * (q[i4] * q[i16]) + (-9.948025e-05) * (q[i4] * q[i20]) + (-8.673770e-04) * (q[i4] * q[i21]) + (-6.473718e-04) * (q[i4] * q[i22])
            + (2.117505e-03) * (q[i5] * q[i6]) + (-1.678776e-03) * (q[i5] * q[i7]) + (3.321299e-04) * (q[i5] * q[i8]) + (-4.683551e-04) * (q[i5] * q[i9])
            + (-2.530374e-04) * (q[i5] * q[i10]) + (-1.004278e-03) * (q[i5] * q[i11]) + (2.775053e-04) * (q[i5] * q[i12]) + (5.705245e-04) * (q[i5] * q[i15])
            + (-3.259581e-04) * (q[i5] * q[i16]) + (5.075397e-04) * (q[i5] * q[i20]) + (3.316346e-04) * (q[i5] * q[i21]) + (9.389452e-05) * (q[i5] * q[i22])
            + (8.441643e-04) * (q[i6] * q[i7]) + (-2.846423e-04) * (q[i6] * q[i8]) + (-5.808050e-04) * (q[i6] * q[i9]) + (1.102690e-03) * (q[i6] * q[i10])
            + (5.555949e-05) * (q[i6] * q[i11]) + (6.442033e-04) * (q[i6] * q[i12]) + (6.141175e-05) * (q[i6] * q[i15]) + (8.896395e-05) * (q[i6] * q[i16])
            + (6.021812e-04) * (q[i6] * q[i20]) + (-5.092972e-04) * (q[i6] * q[i21]) + (4.299740e-04) * (q[i6] * q[i22]) + (-1.102029e-03) * (q[i7] * q[i8])
            + (-4.730683e-04) * (q[i7] * q[i9]) + (-8.172558e-04) * (q[i7] * q[i10]) + (-7.213827e-04) * (q[i7] * q[i11]) + (-6.713509e-04) * (q[i7] * q[i12])
            + (-6.389798e-04) * (q[i7] * q[i15]) + (2.711572e-04) * (q[i7] * q[i16]) + (-6.020368e-04) * (q[i7] * q[i20]) + (-8.144087e-04) * (q[i7] * q[i21])
            + (-4.996268e-04) * (q[i7] * q[i22]) + (1.033928e-04) * (q[i8] * q[i9]) + (-5.215299e-04) * (q[i8] * q[i10]) + (-6.835541e-04) * (q[i8] * q[i11])
            + (1.680650e-04) * (q[i8] * q[i12]) + (7.750505e-04) * (q[i8] * q[i15]) + (-1.567777e-04) * (q[i8] * q[i16]) + (5.985790e-06) * (q[i8] * q[i20])
            + (1.998452e-03) * (q[i8] * q[i21]) + (-2.105038e-04) * (q[i8] * q[i22]) + (8.369679e-05) * (q[i9] * q[i10]) + (1.998468e-04) * (q[i9] * q[i11])
            + (-8.015074e-05) * (q[i9] * q[i12]) + (-5.647189e-05) * (q[i9] * q[i15]) + (-3.671280e-05) * (q[i9] * q[i16]) + (1.186678e-04) * (q[i9] * q[i20])
            + (-3.199455e-04) * (q[i9] * q[i21]) + (1.559101e-04) * (q[i9] * q[i22]) + (-2.833224e-04) * (q[i10] * q[i11]) + (-1.673986e-04) * (q[i10] * q[i12])
            + (-1.702933e-04) * (q[i10] * q[i15]) + (-1.618793e-04) * (q[i10] * q[i16]) + (-1.181828e-04) * (q[i10] * q[i20])
            + (-1.620898e-04) * (q[i10] * q[i21]) + (-7.845373e-05) * (q[i10] * q[i22]) + (2.126881e-04) * (q[i11] * q[i12])
            + (4.307798e-04) * (q[i11] * q[i15]) + (2.303527e-05) * (q[i11] * q[i16]) + (6.704706e-05) * (q[i11] * q[i20]) + (1.175299e-03) * (q[i11] * q[i21])
            + (6.339400e-05) * (q[i11] * q[i22]) + (-4.131142e-05) * (q[i12] * q[i15]) + (-2.640523e-04) * (q[i12] * q[i16])
            + (7.135416e-05) * (q[i12] * q[i20]) + (1.233256e-04) * (q[i12] * q[i21]) + (-9.640700e-05) * (q[i12] * q[i22]) + (2.386171e-04) * (q[i15] * q[i16])
            + (3.712005e-06) * (q[i15] * q[i20]) + (4.275000e-04) * (q[i15] * q[i21]) + (4.808193e-05) * (q[i15] * q[i22]) + (-2.982123e-06) * (q[i16] * q[i20])
            + (9.156044e-05) * (q[i16] * q[i21]) + (-1.290174e-04) * (q[i16] * q[i22]) + (-1.008673e-04) * (q[i20] * q[i21])
            + (-1.018229e-04) * (q[i20] * q[i22]) + (-1.219829e-04) * (q[i21] * q[i22]);
      JQ[3][i20] = (-8.644365e-03) * (1) + (6.533052e-04) * ((2) * q[i20]) + (-1.494506e-03) * (q[i0]) + (1.164486e-03) * (q[i1]) + (1.266208e-03) * (q[i2])
            + (3.994531e-03) * (q[i3]) + (-2.744435e-03) * (q[i4]) + (2.839825e-03) * (q[i5]) + (-2.307807e-03) * (q[i6]) + (7.661167e-04) * (q[i7])
            + (-1.407848e-03) * (q[i8]) + (-1.001970e-03) * (q[i9]) + (9.125165e-04) * (q[i10]) + (9.533648e-04) * (q[i11]) + (2.098065e-03) * (q[i12])
            + (-1.002143e-03) * (q[i15]) + (-4.327457e-03) * (q[i16]) + (3.845273e-06) * (q[i19]) + (-8.348530e-04) * (q[i21]) + (-4.439633e-03) * (q[i22])
            + (1.103751e-03) * (q[i0] * q[i0]) + (6.888944e-05) * (q[i1] * q[i1]) + (5.646483e-04) * (q[i2] * q[i2]) + (-6.584318e-04) * (q[i3] * q[i3])
            + (-1.116481e-03) * (q[i4] * q[i4]) + (3.362770e-04) * (q[i5] * q[i5]) + (-8.019595e-04) * (q[i6] * q[i6]) + (7.309260e-04) * (q[i7] * q[i7])
            + (-6.122044e-04) * (q[i8] * q[i8]) + (3.687722e-04) * (q[i9] * q[i9]) + (1.150544e-04) * (q[i10] * q[i10]) + (5.648588e-05) * (q[i11] * q[i11])
            + (9.596390e-04) * (q[i12] * q[i12]) + (1.619278e-04) * (q[i15] * q[i15]) + (-6.310745e-04) * (q[i16] * q[i16])
            + (-1.211001e-05) * (q[i19] * q[i19]) + (-3.383102e-04) * ((2) * q[i0] * q[i20]) + (1.353930e-04) * ((2) * q[i1] * q[i20])
            + (3.831992e-06) * ((2) * q[i2] * q[i20]) + (-2.618625e-04) * ((2) * q[i3] * q[i20]) + (-4.215867e-04) * ((2) * q[i4] * q[i20])
            + (1.304845e-03) * ((2) * q[i5] * q[i20]) + (3.643780e-04) * ((2) * q[i6] * q[i20]) + (-5.816897e-04) * ((2) * q[i7] * q[i20])
            + (-9.433013e-04) * ((2) * q[i8] * q[i20]) + (3.953414e-05) * ((2) * q[i9] * q[i20]) + (-1.005608e-04) * ((2) * q[i10] * q[i20])
            + (3.857706e-05) * ((2) * q[i11] * q[i20]) + (8.428841e-04) * ((2) * q[i12] * q[i20]) + (-1.337070e-05) * ((2) * q[i15] * q[i20])
            + (-3.944168e-04) * ((2) * q[i16] * q[i20]) + (7.361915e-06) * ((2) * q[i19] * q[i20]) + (3.434237e-04) * ((3) * q[i20] * q[i20])
            + (1.125180e-05) * ((2) * q[i20] * q[i21]) + (-1.910928e-03) * ((2) * q[i20] * q[i22]) + (-5.956123e-05) * (q[i21] * q[i21])
            + (1.230489e-03) * (q[i22] * q[i22]) + (-8.652112e-04) * (q[i0] * q[i1]) + (-8.232012e-04) * (q[i0] * q[i2]) + (-1.838031e-04) * (q[i0] * q[i3])
            + (-1.045449e-04) * (q[i0] * q[i4]) + (5.631766e-04) * (q[i0] * q[i5]) + (8.165496e-04) * (q[i0] * q[i6]) + (-1.202997e-03) * (q[i0] * q[i7])
            + (2.763084e-04) * (q[i0] * q[i8]) + (1.743876e-05) * (q[i0] * q[i9]) + (3.177573e-04) * (q[i0] * q[i10]) + (-1.001125e-03) * (q[i0] * q[i11])
            + (1.015993e-03) * (q[i0] * q[i12]) + (3.283325e-05) * (q[i0] * q[i15]) + (-3.938751e-04) * (q[i0] * q[i16]) + (-1.269053e-04) * (q[i0] * q[i19])
            + (-1.982301e-04) * (q[i0] * q[i21]) + (2.989038e-04) * (q[i0] * q[i22]) + (7.591803e-04) * (q[i1] * q[i2]) + (-6.184635e-07) * (q[i1] * q[i3])
            + (7.257399e-04) * (q[i1] * q[i4]) + (-4.714147e-04) * (q[i1] * q[i5]) + (-1.608027e-03) * (q[i1] * q[i6]) + (-5.374547e-04) * (q[i1] * q[i7])
            + (-1.726717e-04) * (q[i1] * q[i8]) + (-5.745873e-04) * (q[i1] * q[i9]) + (4.715358e-05) * (q[i1] * q[i10]) + (-3.769764e-05) * (q[i1] * q[i11])
            + (-3.163449e-04) * (q[i1] * q[i12]) + (3.002936e-04) * (q[i1] * q[i15]) + (1.281700e-04) * (q[i1] * q[i16]) + (-1.275865e-04) * (q[i1] * q[i19])
            + (-1.011552e-05) * (q[i1] * q[i21]) + (6.525594e-04) * (q[i1] * q[i22]) + (3.399666e-04) * (q[i2] * q[i3]) + (-3.384693e-04) * (q[i2] * q[i4])
            + (1.088875e-03) * (q[i2] * q[i5]) + (2.332729e-04) * (q[i2] * q[i6]) + (1.040407e-03) * (q[i2] * q[i7]) + (-7.718952e-04) * (q[i2] * q[i8])
            + (-7.146794e-04) * (q[i2] * q[i9]) + (7.970314e-04) * (q[i2] * q[i10]) + (-3.052536e-04) * (q[i2] * q[i11]) + (8.216694e-04) * (q[i2] * q[i12])
            + (4.225078e-04) * (q[i2] * q[i15]) + (-1.308262e-04) * (q[i2] * q[i16]) + (-5.132617e-04) * (q[i2] * q[i19]) + (1.482476e-04) * (q[i2] * q[i21])
            + (1.505349e-03) * (q[i2] * q[i22]) + (2.276730e-03) * (q[i3] * q[i4]) + (-2.079203e-03) * (q[i3] * q[i5]) + (-1.440157e-03) * (q[i3] * q[i6])
            + (1.855864e-03) * (q[i3] * q[i7]) + (-1.592012e-03) * (q[i3] * q[i8]) + (-4.009139e-04) * (q[i3] * q[i9]) + (-6.046693e-04) * (q[i3] * q[i10])
            + (-1.368810e-03) * (q[i3] * q[i11]) + (-3.306156e-04) * (q[i3] * q[i12]) + (4.418298e-04) * (q[i3] * q[i15]) + (8.806233e-04) * (q[i3] * q[i16])
            + (-1.007803e-04) * (q[i3] * q[i19]) + (6.515640e-04) * (q[i3] * q[i21]) + (8.760701e-04) * (q[i3] * q[i22]) + (9.827736e-04) * (q[i4] * q[i5])
            + (2.832568e-03) * (q[i4] * q[i6]) + (-2.349355e-03) * (q[i4] * q[i7]) + (1.827705e-03) * (q[i4] * q[i8]) + (9.788540e-05) * (q[i4] * q[i9])
            + (6.473136e-04) * (q[i4] * q[i10]) + (1.983930e-03) * (q[i4] * q[i11]) + (2.078426e-04) * (q[i4] * q[i12]) + (-2.271586e-06) * (q[i4] * q[i15])
            + (1.314712e-04) * (q[i4] * q[i16]) + (-9.948025e-05) * (q[i4] * q[i19]) + (2.535387e-06) * (q[i4] * q[i21]) + (-9.697348e-04) * (q[i4] * q[i22])
            + (-1.682309e-03) * (q[i5] * q[i6]) + (2.085572e-03) * (q[i5] * q[i7]) + (3.494283e-04) * (q[i5] * q[i8]) + (-2.614614e-04) * (q[i5] * q[i9])
            + (-4.645845e-04) * (q[i5] * q[i10]) + (-2.755955e-04) * (q[i5] * q[i11]) + (9.851661e-04) * (q[i5] * q[i12]) + (-3.208123e-04) * (q[i5] * q[i15])
            + (5.767805e-04) * (q[i5] * q[i16]) + (5.075397e-04) * (q[i5] * q[i19]) + (-9.564362e-05) * (q[i5] * q[i21]) + (-3.390023e-04) * (q[i5] * q[i22])
            + (-8.377542e-04) * (q[i6] * q[i7]) + (1.106294e-03) * (q[i6] * q[i8]) + (8.196789e-04) * (q[i6] * q[i9]) + (4.810234e-04) * (q[i6] * q[i10])
            + (-6.663878e-04) * (q[i6] * q[i11]) + (-7.282235e-04) * (q[i6] * q[i12]) + (-2.685348e-04) * (q[i6] * q[i15]) + (6.421954e-04) * (q[i6] * q[i16])
            + (6.021812e-04) * (q[i6] * q[i19]) + (-4.971938e-04) * (q[i6] * q[i21]) + (-8.179393e-04) * (q[i6] * q[i22]) + (2.936866e-04) * (q[i7] * q[i8])
            + (-1.099627e-03) * (q[i7] * q[i9]) + (5.768088e-04) * (q[i7] * q[i10]) + (6.426055e-04) * (q[i7] * q[i11]) + (4.993456e-05) * (q[i7] * q[i12])
            + (-9.214458e-05) * (q[i7] * q[i15]) + (-6.498497e-05) * (q[i7] * q[i16]) + (-6.020368e-04) * (q[i7] * q[i19]) + (4.308491e-04) * (q[i7] * q[i21])
            + (-5.089145e-04) * (q[i7] * q[i22]) + (5.290930e-04) * (q[i8] * q[i9]) + (-1.035603e-04) * (q[i8] * q[i10]) + (1.607269e-04) * (q[i8] * q[i11])
            + (-6.569697e-04) * (q[i8] * q[i12]) + (1.638484e-04) * (q[i8] * q[i15]) + (-7.861975e-04) * (q[i8] * q[i16]) + (5.985790e-06) * (q[i8] * q[i19])
            + (-2.078082e-04) * (q[i8] * q[i21]) + (1.988446e-03) * (q[i8] * q[i22]) + (-8.075254e-05) * (q[i9] * q[i10]) + (-1.656583e-04) * (q[i9] * q[i11])
            + (-2.905708e-04) * (q[i9] * q[i12]) + (1.652257e-04) * (q[i9] * q[i15]) + (1.781724e-04) * (q[i9] * q[i16]) + (1.186678e-04) * (q[i9] * q[i19])
            + (-7.904078e-05) * (q[i9] * q[i21]) + (-1.632341e-04) * (q[i9] * q[i22]) + (-7.525957e-05) * (q[i10] * q[i11]) + (2.034290e-04) * (q[i10] * q[i12])
            + (3.417873e-05) * (q[i10] * q[i15]) + (5.807324e-05) * (q[i10] * q[i16]) + (-1.181828e-04) * (q[i10] * q[i19]) + (1.556146e-04) * (q[i10] * q[i21])
            + (-3.133288e-04) * (q[i10] * q[i22]) + (-2.092707e-04) * (q[i11] * q[i12]) + (-2.607958e-04) * (q[i11] * q[i15])
            + (-4.123443e-05) * (q[i11] * q[i16]) + (6.704706e-05) * (q[i11] * q[i19]) + (9.226390e-05) * (q[i11] * q[i21])
            + (-1.246492e-04) * (q[i11] * q[i22]) + (2.314826e-05) * (q[i12] * q[i15]) + (4.473115e-04) * (q[i12] * q[i16]) + (7.135416e-05) * (q[i12] * q[i19])
            + (-6.256477e-05) * (q[i12] * q[i21]) + (-1.167479e-03) * (q[i12] * q[i22]) + (-2.361487e-04) * (q[i15] * q[i16])
            + (3.712005e-06) * (q[i15] * q[i19]) + (-1.281918e-04) * (q[i15] * q[i21]) + (9.076848e-05) * (q[i15] * q[i22])
            + (-2.982123e-06) * (q[i16] * q[i19]) + (5.082644e-05) * (q[i16] * q[i21]) + (4.165926e-04) * (q[i16] * q[i22])
            + (-1.008673e-04) * (q[i19] * q[i21]) + (-1.018229e-04) * (q[i19] * q[i22]) + (1.231277e-04) * (q[i21] * q[i22]);
      JQ[3][i21] = (8.061264e-03) * (1) + (5.896429e-04) * ((2) * q[i21]) + (-1.608200e-03) * (q[i0]) + (7.894374e-04) * (q[i1]) + (6.967254e-04) * (q[i2])
            + (4.071305e-03) * (q[i3]) + (-3.607526e-04) * (q[i4]) + (-1.049549e-03) * (q[i5]) + (-1.719988e-03) * (q[i6]) + (-2.265384e-04) * (q[i7])
            + (3.691870e-03) * (q[i8]) + (-6.894929e-04) * (q[i9]) + (3.067238e-04) * (q[i10]) + (5.293681e-03) * (q[i11]) + (-6.710139e-04) * (q[i12])
            + (2.999294e-03) * (q[i15]) + (4.671434e-04) * (q[i16]) + (-4.411577e-03) * (q[i19]) + (-8.348530e-04) * (q[i20]) + (-4.130537e-06) * (q[i22])
            + (-6.953435e-05) * (q[i0] * q[i0]) + (7.867434e-05) * (q[i1] * q[i1]) + (-3.514461e-05) * (q[i2] * q[i2]) + (6.239800e-04) * (q[i3] * q[i3])
            + (2.800301e-04) * (q[i4] * q[i4]) + (5.841197e-04) * (q[i5] * q[i5]) + (-1.001196e-04) * (q[i6] * q[i6]) + (-1.652416e-03) * (q[i7] * q[i7])
            + (9.352912e-04) * (q[i8] * q[i8]) + (1.736911e-04) * (q[i9] * q[i9]) + (-1.624001e-04) * (q[i10] * q[i10]) + (-7.417927e-04) * (q[i11] * q[i11])
            + (-8.149968e-05) * (q[i12] * q[i12]) + (1.051880e-03) * (q[i15] * q[i15]) + (8.688550e-05) * (q[i16] * q[i16])
            + (-1.911368e-03) * (q[i19] * q[i19]) + (1.125180e-05) * (q[i20] * q[i20]) + (-1.336771e-03) * ((2) * q[i0] * q[i21])
            + (-9.860480e-06) * ((2) * q[i1] * q[i21]) + (-1.364698e-03) * ((2) * q[i2] * q[i21]) + (4.875567e-04) * ((2) * q[i3] * q[i21])
            + (4.514942e-06) * ((2) * q[i4] * q[i21]) + (1.230496e-04) * ((2) * q[i5] * q[i21]) + (6.467126e-04) * ((2) * q[i6] * q[i21])
            + (-7.341577e-04) * ((2) * q[i7] * q[i21]) + (2.487787e-04) * ((2) * q[i8] * q[i21]) + (1.739053e-04) * ((2) * q[i9] * q[i21])
            + (-6.830081e-07) * ((2) * q[i10] * q[i21]) + (-3.345501e-04) * ((2) * q[i11] * q[i21]) + (1.356424e-04) * ((2) * q[i12] * q[i21])
            + (1.073933e-04) * ((2) * q[i15] * q[i21]) + (-3.701730e-05) * ((2) * q[i16] * q[i21]) + (-1.211949e-03) * ((2) * q[i19] * q[i21])
            + (-5.956123e-05) * ((2) * q[i20] * q[i21]) + (-2.655636e-04) * ((3) * q[i21] * q[i21]) + (8.961552e-05) * ((2) * q[i21] * q[i22])
            + (8.952621e-05) * (q[i22] * q[i22]) + (4.364268e-04) * (q[i0] * q[i1]) + (-1.085080e-04) * (q[i0] * q[i2]) + (4.444859e-04) * (q[i0] * q[i3])
            + (5.186598e-04) * (q[i0] * q[i4]) + (-9.084618e-04) * (q[i0] * q[i5]) + (1.794742e-03) * (q[i0] * q[i6]) + (-1.377346e-04) * (q[i0] * q[i7])
            + (-3.341720e-05) * (q[i0] * q[i8]) + (1.217458e-04) * (q[i0] * q[i9]) + (1.299127e-04) * (q[i0] * q[i10]) + (2.566488e-04) * (q[i0] * q[i11])
            + (-1.147600e-04) * (q[i0] * q[i12]) + (-1.167079e-03) * (q[i0] * q[i15]) + (-4.024322e-04) * (q[i0] * q[i16]) + (-6.506993e-04) * (q[i0] * q[i19])
            + (-1.982301e-04) * (q[i0] * q[i20]) + (2.354593e-04) * (q[i0] * q[i22]) + (-3.285522e-04) * (q[i1] * q[i2]) + (-1.995427e-03) * (q[i1] * q[i3])
            + (-1.075580e-03) * (q[i1] * q[i4]) + (-1.942308e-04) * (q[i1] * q[i5]) + (2.265587e-04) * (q[i1] * q[i6]) + (-6.836386e-04) * (q[i1] * q[i7])
            + (-2.283093e-04) * (q[i1] * q[i8]) + (1.450506e-04) * (q[i1] * q[i9]) + (-1.679596e-04) * (q[i1] * q[i10]) + (-5.641109e-04) * (q[i1] * q[i11])
            + (3.741781e-04) * (q[i1] * q[i12]) + (-7.800937e-04) * (q[i1] * q[i15]) + (1.202486e-04) * (q[i1] * q[i16]) + (-3.065236e-04) * (q[i1] * q[i19])
            + (-1.011552e-05) * (q[i1] * q[i20]) + (2.246849e-04) * (q[i1] * q[i22]) + (-4.242770e-04) * (q[i2] * q[i3]) + (1.623750e-04) * (q[i2] * q[i4])
            + (1.804426e-04) * (q[i2] * q[i5]) + (-1.001613e-03) * (q[i2] * q[i6]) + (6.353159e-05) * (q[i2] * q[i7]) + (8.458875e-04) * (q[i2] * q[i8])
            + (-2.137513e-04) * (q[i2] * q[i9]) + (-3.824529e-05) * (q[i2] * q[i10]) + (-6.046811e-04) * (q[i2] * q[i11]) + (-1.392476e-04) * (q[i2] * q[i12])
            + (-1.630412e-03) * (q[i2] * q[i15]) + (3.203257e-04) * (q[i2] * q[i16]) + (-1.510791e-03) * (q[i2] * q[i19]) + (1.482476e-04) * (q[i2] * q[i20])
            + (7.342545e-05) * (q[i2] * q[i22]) + (1.190653e-03) * (q[i3] * q[i4]) + (1.540285e-03) * (q[i3] * q[i5]) + (-2.486855e-03) * (q[i3] * q[i6])
            + (2.294031e-03) * (q[i3] * q[i7]) + (4.773394e-04) * (q[i3] * q[i8]) + (-1.182874e-03) * (q[i3] * q[i9]) + (2.872147e-04) * (q[i3] * q[i10])
            + (-8.578729e-04) * (q[i3] * q[i11]) + (2.418505e-04) * (q[i3] * q[i12]) + (2.398466e-04) * (q[i3] * q[i15]) + (7.408101e-04) * (q[i3] * q[i16])
            + (9.749011e-04) * (q[i3] * q[i19]) + (6.515640e-04) * (q[i3] * q[i20]) + (2.058444e-04) * (q[i3] * q[i22]) + (-1.737532e-03) * (q[i4] * q[i5])
            + (1.400710e-03) * (q[i4] * q[i6]) + (-1.470133e-04) * (q[i4] * q[i7]) + (1.110036e-04) * (q[i4] * q[i8]) + (3.185083e-04) * (q[i4] * q[i9])
            + (-6.081948e-04) * (q[i4] * q[i10]) + (9.509409e-04) * (q[i4] * q[i11]) + (-5.561184e-04) * (q[i4] * q[i12]) + (-5.666974e-04) * (q[i4] * q[i15])
            + (-1.290802e-04) * (q[i4] * q[i16]) + (-8.673770e-04) * (q[i4] * q[i19]) + (2.535387e-06) * (q[i4] * q[i20]) + (2.030181e-04) * (q[i4] * q[i22])
            + (9.124768e-04) * (q[i5] * q[i6]) + (1.582883e-04) * (q[i5] * q[i7]) + (-1.047609e-04) * (q[i5] * q[i8]) + (5.513357e-04) * (q[i5] * q[i9])
            + (5.093617e-04) * (q[i5] * q[i10]) + (-1.054120e-03) * (q[i5] * q[i11]) + (4.795968e-04) * (q[i5] * q[i12]) + (1.573328e-04) * (q[i5] * q[i15])
            + (1.445475e-04) * (q[i5] * q[i16]) + (3.316346e-04) * (q[i5] * q[i19]) + (-9.564362e-05) * (q[i5] * q[i20]) + (-1.336004e-04) * (q[i5] * q[i22])
            + (-3.850392e-04) * (q[i6] * q[i7]) + (-1.004595e-03) * (q[i6] * q[i8]) + (2.981797e-04) * (q[i6] * q[i9]) + (7.280991e-04) * (q[i6] * q[i10])
            + (8.746138e-04) * (q[i6] * q[i11]) + (1.442067e-04) * (q[i6] * q[i12]) + (-1.138220e-03) * (q[i6] * q[i15]) + (2.359060e-04) * (q[i6] * q[i16])
            + (-5.092972e-04) * (q[i6] * q[i19]) + (-4.971938e-04) * (q[i6] * q[i20]) + (3.492630e-04) * (q[i6] * q[i22]) + (1.345406e-03) * (q[i7] * q[i8])
            + (1.866575e-04) * (q[i7] * q[i9]) + (-7.450826e-04) * (q[i7] * q[i10]) + (-2.241782e-04) * (q[i7] * q[i11]) + (-2.370325e-04) * (q[i7] * q[i12])
            + (2.972375e-05) * (q[i7] * q[i15]) + (5.920154e-04) * (q[i7] * q[i16]) + (-8.144087e-04) * (q[i7] * q[i19]) + (4.308491e-04) * (q[i7] * q[i20])
            + (-3.487774e-04) * (q[i7] * q[i22]) + (-2.171296e-04) * (q[i8] * q[i9]) + (-2.553434e-04) * (q[i8] * q[i10]) + (-8.277569e-04) * (q[i8] * q[i11])
            + (1.356376e-04) * (q[i8] * q[i12]) + (2.465250e-03) * (q[i8] * q[i15]) + (-1.693606e-04) * (q[i8] * q[i16]) + (1.998452e-03) * (q[i8] * q[i19])
            + (-2.078082e-04) * (q[i8] * q[i20]) + (1.711823e-06) * (q[i8] * q[i22]) + (3.387799e-04) * (q[i9] * q[i10]) + (4.284419e-04) * (q[i9] * q[i11])
            + (3.768899e-05) * (q[i9] * q[i12]) + (-4.322648e-04) * (q[i9] * q[i15]) + (-2.725853e-05) * (q[i9] * q[i16]) + (-3.199455e-04) * (q[i9] * q[i19])
            + (-7.904078e-05) * (q[i9] * q[i20]) + (4.792738e-05) * (q[i9] * q[i22]) + (2.175368e-04) * (q[i10] * q[i11]) + (-1.576923e-04) * (q[i10] * q[i12])
            + (4.745517e-05) * (q[i10] * q[i15]) + (5.007479e-05) * (q[i10] * q[i16]) + (-1.620898e-04) * (q[i10] * q[i19]) + (1.556146e-04) * (q[i10] * q[i20])
            + (-4.459758e-05) * (q[i10] * q[i22]) + (2.520394e-04) * (q[i11] * q[i12]) + (1.367234e-03) * (q[i11] * q[i15]) + (1.331680e-04) * (q[i11] * q[i16])
            + (1.175299e-03) * (q[i11] * q[i19]) + (9.226390e-05) * (q[i11] * q[i20]) + (3.448838e-05) * (q[i11] * q[i22]) + (-3.017616e-05) * (q[i12] * q[i15])
            + (-2.065151e-04) * (q[i12] * q[i16]) + (1.233256e-04) * (q[i12] * q[i19]) + (-6.256477e-05) * (q[i12] * q[i20])
            + (3.444092e-05) * (q[i12] * q[i22]) + (-2.004290e-04) * (q[i15] * q[i16]) + (4.275000e-04) * (q[i15] * q[i19])
            + (-1.281918e-04) * (q[i15] * q[i20]) + (1.820690e-04) * (q[i15] * q[i22]) + (9.156044e-05) * (q[i16] * q[i19]) + (5.082644e-05) * (q[i16] * q[i20])
            + (-1.806560e-04) * (q[i16] * q[i22]) + (-1.008673e-04) * (q[i19] * q[i20]) + (-1.219829e-04) * (q[i19] * q[i22])
            + (1.231277e-04) * (q[i20] * q[i22]);
      JQ[3][i22] = (8.054589e-03) * (1) + (-6.045764e-04) * ((2) * q[i22]) + (-8.171787e-04) * (q[i0]) + (1.582445e-03) * (q[i1]) + (-7.466201e-04) * (q[i2])
            + (3.704013e-04) * (q[i3]) + (-4.038619e-03) * (q[i4]) + (1.032526e-03) * (q[i5]) + (-2.084573e-04) * (q[i6]) + (-1.728481e-03) * (q[i7])
            + (3.724218e-03) * (q[i8]) + (3.273732e-04) * (q[i9]) + (-6.889521e-04) * (q[i10]) + (6.641784e-04) * (q[i11]) + (-5.392673e-03) * (q[i12])
            + (4.596933e-04) * (q[i15]) + (3.021687e-03) * (q[i16]) + (-8.428399e-04) * (q[i19]) + (-4.439633e-03) * (q[i20]) + (-4.130537e-06) * (q[i21])
            + (6.636358e-05) * (q[i0] * q[i0]) + (-8.686967e-05) * (q[i1] * q[i1]) + (-3.549139e-05) * (q[i2] * q[i2]) + (2.862283e-04) * (q[i3] * q[i3])
            + (6.291117e-04) * (q[i4] * q[i4]) + (5.764229e-04) * (q[i5] * q[i5]) + (-1.661576e-03) * (q[i6] * q[i6]) + (-1.002680e-04) * (q[i7] * q[i7])
            + (9.451885e-04) * (q[i8] * q[i8]) + (-1.672486e-04) * (q[i9] * q[i9]) + (1.740462e-04) * (q[i10] * q[i10]) + (-7.896716e-05) * (q[i11] * q[i11])
            + (-7.300253e-04) * (q[i12] * q[i12]) + (8.621996e-05) * (q[i15] * q[i15]) + (1.061528e-03) * (q[i16] * q[i16]) + (9.038099e-06) * (q[i19] * q[i19])
            + (-1.910928e-03) * (q[i20] * q[i20]) + (8.961552e-05) * (q[i21] * q[i21]) + (-7.529093e-06) * ((2) * q[i0] * q[i22])
            + (-1.331682e-03) * ((2) * q[i1] * q[i22]) + (-1.357232e-03) * ((2) * q[i2] * q[i22]) + (3.564695e-06) * ((2) * q[i3] * q[i22])
            + (4.814324e-04) * ((2) * q[i4] * q[i22]) + (1.300223e-04) * ((2) * q[i5] * q[i22]) + (7.316595e-04) * ((2) * q[i6] * q[i22])
            + (-6.525707e-04) * ((2) * q[i7] * q[i22]) + (-2.487447e-04) * ((2) * q[i8] * q[i22]) + (1.112554e-06) * ((2) * q[i9] * q[i22])
            + (-1.751883e-04) * ((2) * q[i10] * q[i22]) + (1.367414e-04) * ((2) * q[i11] * q[i22]) + (-3.197375e-04) * ((2) * q[i12] * q[i22])
            + (3.600029e-05) * ((2) * q[i15] * q[i22]) + (-1.055802e-04) * ((2) * q[i16] * q[i22]) + (5.989050e-05) * ((2) * q[i19] * q[i22])
            + (1.230489e-03) * ((2) * q[i20] * q[i22]) + (8.952621e-05) * ((2) * q[i21] * q[i22]) + (-2.655773e-04) * ((3) * q[i22] * q[i22])
            + (4.455604e-04) * (q[i0] * q[i1]) + (-3.301443e-04) * (q[i0] * q[i2]) + (-1.090024e-03) * (q[i0] * q[i3]) + (-2.030472e-03) * (q[i0] * q[i4])
            + (-2.016757e-04) * (q[i0] * q[i5]) + (6.902083e-04) * (q[i0] * q[i6]) + (-2.210035e-04) * (q[i0] * q[i7]) + (2.267855e-04) * (q[i0] * q[i8])
            + (1.699462e-04) * (q[i0] * q[i9]) + (-1.408020e-04) * (q[i0] * q[i10]) + (3.796646e-04) * (q[i0] * q[i11]) + (-5.634708e-04) * (q[i0] * q[i12])
            + (-1.190509e-04) * (q[i0] * q[i15]) + (7.784233e-04) * (q[i0] * q[i16]) + (1.077289e-05) * (q[i0] * q[i19]) + (2.989038e-04) * (q[i0] * q[i20])
            + (2.354593e-04) * (q[i0] * q[i21]) + (-1.144321e-04) * (q[i1] * q[i2]) + (5.225465e-04) * (q[i1] * q[i3]) + (4.590103e-04) * (q[i1] * q[i4])
            + (-8.835536e-04) * (q[i1] * q[i5]) + (1.453181e-04) * (q[i1] * q[i6]) + (-1.779180e-03) * (q[i1] * q[i7]) + (1.446450e-05) * (q[i1] * q[i8])
            + (-1.327543e-04) * (q[i1] * q[i9]) + (-1.153232e-04) * (q[i1] * q[i10]) + (-1.102424e-04) * (q[i1] * q[i11]) + (2.672405e-04) * (q[i1] * q[i12])
            + (3.931238e-04) * (q[i1] * q[i15]) + (1.171665e-03) * (q[i1] * q[i16]) + (1.984812e-04) * (q[i1] * q[i19]) + (6.525594e-04) * (q[i1] * q[i20])
            + (2.246849e-04) * (q[i1] * q[i21]) + (1.596645e-04) * (q[i2] * q[i3]) + (-4.368106e-04) * (q[i2] * q[i4]) + (1.878957e-04) * (q[i2] * q[i5])
            + (-5.206706e-05) * (q[i2] * q[i6]) + (9.994085e-04) * (q[i2] * q[i7]) + (-8.524062e-04) * (q[i2] * q[i8]) + (3.476444e-05) * (q[i2] * q[i9])
            + (2.139062e-04) * (q[i2] * q[i10]) + (-1.341003e-04) * (q[i2] * q[i11]) + (-5.861205e-04) * (q[i2] * q[i12]) + (-3.246137e-04) * (q[i2] * q[i15])
            + (1.635022e-03) * (q[i2] * q[i16]) + (-1.495248e-04) * (q[i2] * q[i19]) + (1.505349e-03) * (q[i2] * q[i20]) + (7.342545e-05) * (q[i2] * q[i21])
            + (1.195199e-03) * (q[i3] * q[i4]) + (-1.739445e-03) * (q[i3] * q[i5]) + (1.476708e-04) * (q[i3] * q[i6]) + (-1.392027e-03) * (q[i3] * q[i7])
            + (-1.072421e-04) * (q[i3] * q[i8]) + (6.093263e-04) * (q[i3] * q[i9]) + (-3.128102e-04) * (q[i3] * q[i10]) + (-5.538045e-04) * (q[i3] * q[i11])
            + (9.448845e-04) * (q[i3] * q[i12]) + (1.323404e-04) * (q[i3] * q[i15]) + (5.767705e-04) * (q[i3] * q[i16]) + (-2.244236e-06) * (q[i3] * q[i19])
            + (8.760701e-04) * (q[i3] * q[i20]) + (2.058444e-04) * (q[i3] * q[i21]) + (1.528462e-03) * (q[i4] * q[i5]) + (-2.287192e-03) * (q[i4] * q[i6])
            + (2.475617e-03) * (q[i4] * q[i7]) + (-4.623798e-04) * (q[i4] * q[i8]) + (-2.816227e-04) * (q[i4] * q[i9]) + (1.174813e-03) * (q[i4] * q[i10])
            + (2.386574e-04) * (q[i4] * q[i11]) + (-8.644089e-04) * (q[i4] * q[i12]) + (-7.345003e-04) * (q[i4] * q[i15]) + (-2.341977e-04) * (q[i4] * q[i16])
            + (-6.473718e-04) * (q[i4] * q[i19]) + (-9.697348e-04) * (q[i4] * q[i20]) + (2.030181e-04) * (q[i4] * q[i21]) + (-1.664968e-04) * (q[i5] * q[i6])
            + (-9.148670e-04) * (q[i5] * q[i7]) + (6.351661e-05) * (q[i5] * q[i8]) + (-5.134596e-04) * (q[i5] * q[i9]) + (-5.545135e-04) * (q[i5] * q[i10])
            + (4.779538e-04) * (q[i5] * q[i11]) + (-1.037047e-03) * (q[i5] * q[i12]) + (-1.446802e-04) * (q[i5] * q[i15]) + (-1.700077e-04) * (q[i5] * q[i16])
            + (9.389452e-05) * (q[i5] * q[i19]) + (-3.390023e-04) * (q[i5] * q[i20]) + (-1.336004e-04) * (q[i5] * q[i21]) + (-3.897134e-04) * (q[i6] * q[i7])
            + (1.344199e-03) * (q[i6] * q[i8]) + (-7.568454e-04) * (q[i6] * q[i9]) + (1.866174e-04) * (q[i6] * q[i10]) + (2.380237e-04) * (q[i6] * q[i11])
            + (2.225201e-04) * (q[i6] * q[i12]) + (5.901552e-04) * (q[i6] * q[i15]) + (2.283668e-05) * (q[i6] * q[i16]) + (4.299740e-04) * (q[i6] * q[i19])
            + (-8.179393e-04) * (q[i6] * q[i20]) + (3.492630e-04) * (q[i6] * q[i21]) + (-1.009421e-03) * (q[i7] * q[i8]) + (7.260578e-04) * (q[i7] * q[i9])
            + (3.008438e-04) * (q[i7] * q[i10]) + (-1.502159e-04) * (q[i7] * q[i11]) + (-8.710689e-04) * (q[i7] * q[i12]) + (2.369613e-04) * (q[i7] * q[i15])
            + (-1.133731e-03) * (q[i7] * q[i16]) + (-4.996268e-04) * (q[i7] * q[i19]) + (-5.089145e-04) * (q[i7] * q[i20]) + (-3.487774e-04) * (q[i7] * q[i21])
            + (-2.532998e-04) * (q[i8] * q[i9]) + (-2.204404e-04) * (q[i8] * q[i10]) + (-1.381067e-04) * (q[i8] * q[i11]) + (8.294795e-04) * (q[i8] * q[i12])
            + (-1.715073e-04) * (q[i8] * q[i15]) + (2.504864e-03) * (q[i8] * q[i16]) + (-2.105038e-04) * (q[i8] * q[i19]) + (1.988446e-03) * (q[i8] * q[i20])
            + (1.711823e-06) * (q[i8] * q[i21]) + (3.377193e-04) * (q[i9] * q[i10]) + (1.575985e-04) * (q[i9] * q[i11]) + (-2.169850e-04) * (q[i9] * q[i12])
            + (5.285697e-05) * (q[i9] * q[i15]) + (4.789656e-05) * (q[i9] * q[i16]) + (1.559101e-04) * (q[i9] * q[i19]) + (-1.632341e-04) * (q[i9] * q[i20])
            + (4.792738e-05) * (q[i9] * q[i21]) + (-3.807429e-05) * (q[i10] * q[i11]) + (-4.287859e-04) * (q[i10] * q[i12])
            + (-2.623439e-05) * (q[i10] * q[i15]) + (-4.296329e-04) * (q[i10] * q[i16]) + (-7.845373e-05) * (q[i10] * q[i19])
            + (-3.133288e-04) * (q[i10] * q[i20]) + (-4.459758e-05) * (q[i10] * q[i21]) + (2.587395e-04) * (q[i11] * q[i12])
            + (2.065659e-04) * (q[i11] * q[i15]) + (2.692128e-05) * (q[i11] * q[i16]) + (6.339400e-05) * (q[i11] * q[i19]) + (-1.246492e-04) * (q[i11] * q[i20])
            + (3.448838e-05) * (q[i11] * q[i21]) + (-1.395952e-04) * (q[i12] * q[i15]) + (-1.411102e-03) * (q[i12] * q[i16])
            + (-9.640700e-05) * (q[i12] * q[i19]) + (-1.167479e-03) * (q[i12] * q[i20]) + (3.444092e-05) * (q[i12] * q[i21])
            + (-2.019045e-04) * (q[i15] * q[i16]) + (4.808193e-05) * (q[i15] * q[i19]) + (9.076848e-05) * (q[i15] * q[i20]) + (1.820690e-04) * (q[i15] * q[i21])
            + (-1.290174e-04) * (q[i16] * q[i19]) + (4.165926e-04) * (q[i16] * q[i20]) + (-1.806560e-04) * (q[i16] * q[i21])
            + (-1.018229e-04) * (q[i19] * q[i20]) + (-1.219829e-04) * (q[i19] * q[i21]) + (1.231277e-04) * (q[i20] * q[i21]);
   }

}
