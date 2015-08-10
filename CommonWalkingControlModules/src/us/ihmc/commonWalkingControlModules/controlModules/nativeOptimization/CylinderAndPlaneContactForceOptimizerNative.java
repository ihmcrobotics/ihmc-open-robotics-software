
package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.DoubleBuffer;

import us.ihmc.utilities.exeptions.NoConvergenceException;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;


public class CylinderAndPlaneContactForceOptimizerNative
{
   private static final boolean DEBUG = false;
   public static final int nSupportVectors = 3;
   public static final int nPointsPerPlane = 4;
   public static final int nPlanes = 2;
   public static final int nCylinders = 2;
   public static final int nCylinderVectors = 8;
   public static final int nCylinderBoundedVariables = 5;
   public static final int wrenchLength = 6;
   public static final int rhoSize = nSupportVectors * nPointsPerPlane * nPlanes+nCylinderVectors*nCylinders;
   public static final int phiSize = nCylinders * nCylinderBoundedVariables;

   private static native void initialize();

   private static native ByteBuffer getCBuffer();

   private static native ByteBuffer getQrhoBuffer();
   
   private static native ByteBuffer getQphiBuffer();

   private static native ByteBuffer getcBuffer();

   private static native ByteBuffer getrhoMinBuffer();

   private static native ByteBuffer getrhoBuffer();
   
   private static native ByteBuffer getphiBuffer();
   
   private static native ByteBuffer getphiMinBuffer();
   
   private static native ByteBuffer getphiMaxBuffer();

   private static native int solveNative(double wRho, double wPhi);

   private static native double getOptValNative();

   private static final Object solveConch = new Object();

   private static final DoubleBuffer CDoubleBuffer;
   private static final DoubleBuffer QrhoDoubleBuffer;
   private static final DoubleBuffer QphiDoubleBuffer;
   private static final DoubleBuffer cDoubleBuffer;
   private static final DoubleBuffer rhoMinDoubleBuffer;
   private static final DoubleBuffer rhoDoubleBuffer;
   private static final DoubleBuffer phiDoubleBuffer;
   private static final DoubleBuffer phiMinDoubleBuffer;
   private static final DoubleBuffer phiMaxDoubleBuffer;
   
   private final BooleanYoVariable debug;

   public static DoubleBuffer setupBuffer(ByteBuffer buffer)
   {
      buffer.order(ByteOrder.nativeOrder());

      return buffer.asDoubleBuffer();
   }

   public static void setBufferToArray(DoubleBuffer buffer, double[] array)
   {
      buffer.rewind();
      buffer.put(array);
   }

   static
   {
      NativeLibraryLoader.loadLibrary("us.ihmc.commonWalkingControlModules.lib", "CylinderAndPlaneContactForceOptimizer");

      initialize();

      CDoubleBuffer = setupBuffer(getCBuffer());
      QrhoDoubleBuffer = setupBuffer(getQrhoBuffer());
      QphiDoubleBuffer = setupBuffer(getQphiBuffer());
      cDoubleBuffer = setupBuffer(getcBuffer());
      rhoMinDoubleBuffer = setupBuffer(getrhoMinBuffer());
      rhoDoubleBuffer = setupBuffer(getrhoBuffer());
      phiDoubleBuffer = setupBuffer(getphiBuffer());
      phiMinDoubleBuffer = setupBuffer(getphiMinBuffer());
      phiMaxDoubleBuffer = setupBuffer(getphiMaxBuffer());
   }

   private final double[] rho = new double[rhoSize];
   private final double[] phi = new double[phiSize];
   private final CylinderAndPlaneContactForceOptimizerNativeOutput nativeOutput;

   public CylinderAndPlaneContactForceOptimizerNative(YoVariableRegistry registry)
   {
      nativeOutput = new CylinderAndPlaneContactForceOptimizerNativeOutput();
      this.debug  = new BooleanYoVariable(this.getClass().getSimpleName()+"DebugFlag",registry);
      debug.set(DEBUG);
   }

   /**
    * dimensions
    *   nSupportVectors = 3 # support vectors are the cone approximation vectors.
    *   nPointsPerContact = 4 # points per foot/flat hand
    *   nPlanes = 2 # number of plane feet
    *   wrenchLength = 6 # universal constant for physical space
    *   nCylinders = 2 # number of cylindrical hands
    *   nCylinderVectors = 8
    *   nCylinderBoundedVariables = 5
    * end 
    * 
    * parameters
    *   
    *   C (wrenchLength, wrenchLength) diagonal psd #MomentumDotWeight
    *   
    *   Qrho (wrenchLength, nPlanes * nSupportVectors * nPointsPerContact + nCylinders*nCylinderVectors ) # ContactPointWrenchMatrix
    *   Qphi (wrenchLength, nCylinders*nCylinderBoundedVariables) # ContactPointWrenchMatrixForFullyBoundedVariables
    *   c (wrenchLength, 1) # WrenchEquationRightHandSide
    *   rhoMin (nSupportVectors * nPointsPerContact * nPlanes + nCylinders*nCylinderVectors ) positive
    *   wRho positive # GroundReactionForceRegularization
    *   phiMin (nCylinders*nCylinderBoundedVariables) negative
    *   phiMax (nCylinders*nCylinderBoundedVariables) positive
    *   wPhi positive # CylinderBoundedVectorRegularization
    *   
    * end
    * 
    * variables
    *   rho (nSupportVectors * nPointsPerContact * nPlanes + nCylinders*nCylinderVectors ) # magnitude of each ground reaction force component
    *   phi (nCylinders*nCylinderBoundedVariables) # specific to cylinders. bounded above and below
    * end
    * 
    * minimize
    *   quad(Qrho * rho + Qphi * phi -c, C) + wRho * quad(rho) + wPhi * quad(phi)
    * subject to
    *   rho >= rhoMin
    *   phiMin<=phi<=phiMax # phi is fully bounded
    * end
    */
   
   public void solve(CylinderAndPlaneContactForceOptimizerNativeInput input)
           throws NoConvergenceException
   {
      double[] C = input.getC();
      double[] Qrho = input.getQrho();
      double[] Qphi = input.getQphi();
      double[] c = input.getc();
      double[] rhoMin = input.getRhoMin();
      double[] phiMin = input.getPhiMin();
      double[] phiMax = input.getPhiMax();
      double wRho = input.getwRho();
      double wPhi = input.getwPhi();

      solve(C, Qrho, Qphi, c, rhoMin, phiMin, phiMax, wRho, wPhi);
      

      if (this.debug.getBooleanValue())
      {
         System.out.println(input.toString());
         System.out.println(this.nativeOutput.toString());
      }

   }

   public CylinderAndPlaneContactForceOptimizerNativeOutput getOutput()
   {
      return nativeOutput;
   }

   private void solve(double[] C, double[] Qrho, double[] Qphi,
                                               double[] c, double[] rhoMin, double[] phiMin, double[] phiMax,
                                                double wRho, double wPhi) throws NoConvergenceException
   {
      int numberOfIterations;
      synchronized (solveConch)
      {
         setBufferToArray(CDoubleBuffer, C);
         setBufferToArray(QrhoDoubleBuffer, Qrho);
         setBufferToArray(QphiDoubleBuffer, Qphi);
         setBufferToArray(cDoubleBuffer, c);
         setBufferToArray(rhoMinDoubleBuffer, rhoMin);
         setBufferToArray(phiMinDoubleBuffer, phiMin);
         setBufferToArray(phiMaxDoubleBuffer, phiMax);

         numberOfIterations = solveNative(wRho, wPhi);

         rhoDoubleBuffer.rewind();
         rhoDoubleBuffer.get(rho);
         
         phiDoubleBuffer.rewind();
         phiDoubleBuffer.get(phi);

         nativeOutput.setOptVal(getOptValNative());
         nativeOutput.setRho(rho);
         nativeOutput.setPhi(phi);
         nativeOutput.setNumberOfIterations(numberOfIterations);
      }

      if (numberOfIterations < 0)
      {
         throw new NoConvergenceException();
      }
   }

   public static void main(String[] args) throws NoConvergenceException
   {
      double[] C = new double[wrenchLength];    // diagonal
      double[] Qrho = new double[wrenchLength * rhoSize];
      double[] Qphi = new double[wrenchLength * phiSize];
      double[] c = new double[wrenchLength];
      double[] rhoMin = new double[rhoSize];
      double[] phiMin = new double[phiSize];
      double[] phiMax = new double[phiSize];
      double[] wRho = new double[1];
      double[] wPhi = new double[1];
      load_default_data(C, Qrho, Qphi, c, rhoMin, phiMin, phiMax, wRho, wPhi);

      CylinderAndPlaneContactForceOptimizerNative momentumOptimizerNative = new CylinderAndPlaneContactForceOptimizerNative(new YoVariableRegistry("rootRegistry"));

      long time = System.nanoTime();
      int nSolves = 10000;
      for (int i = 0; true; i++)
      {
         if (i % nSolves == 0)
         {
            double deltaTimeSeconds = (System.nanoTime() - time) / 1e9;
            double timePerSolveMillis = deltaTimeSeconds / nSolves * 1e3;
            System.out.println("10000 iterations took " + deltaTimeSeconds + " seconds. Time per solve (ms): " + timePerSolveMillis);

//          System.out.println(Arrays.toString(momentumOptimizerNative.getRho()));

            time = System.nanoTime();
         }

         momentumOptimizerNative.solve(C, Qrho, Qphi, c, rhoMin, phiMin, phiMax, wRho[0], wPhi[0]);
      }
   }

   private static void load_default_data( double[] C, double[] Qrho, double[] Qphi, double[] c,
           double[] rhoMin, double[] phiMin, double[] phiMax,  double[] wRho, double[] wPhi)
   {
      Qrho[0] = 0.20319161029830202;
      Qrho[1] = 0.8325912904724193;
      Qrho[2] = -0.8363810443482227;
      Qrho[3] = 0.04331042079065206;
      Qrho[4] = 1.5717878173906188;
      Qrho[5] = 1.5851723557337523;
      Qrho[6] = -1.497658758144655;
      Qrho[7] = -1.171028487447253;
      Qrho[8] = -1.7941311867966805;
      Qrho[9] = -0.23676062539745413;
      Qrho[10] = -1.8804951564857322;
      Qrho[11] = -0.17266710242115568;
      Qrho[12] = 0.596576190459043;
      Qrho[13] = -0.8860508694080989;
      Qrho[14] = 0.7050196079205251;
      Qrho[15] = 0.3634512696654033;
      Qrho[16] = -1.9040724704913385;
      Qrho[17] = 0.23541635196352795;
      Qrho[18] = -0.9629902123701384;
      Qrho[19] = -0.3395952119597214;
      Qrho[20] = -0.865899672914725;
      Qrho[21] = 0.7725516732519853;
      Qrho[22] = -0.23818512931704205;
      Qrho[23] = -1.372529046100147;
      Qrho[24] = 0.17859607212737894;
      Qrho[25] = 1.1212590580454682;
      Qrho[26] = -0.774545870495281;
      Qrho[27] = -1.1121684642712744;
      Qrho[28] = -0.44811496977740495;
      Qrho[29] = 1.7455345994417217;
      Qrho[30] = 1.9039816898917352;
      Qrho[31] = 0.6895347036512547;
      Qrho[32] = 1.6113364341535923;
      Qrho[33] = 1.383003485172717;
      Qrho[34] = -0.48802383468444344;
      Qrho[35] = -1.631131964513103;
      Qrho[36] = 0.6136436100941447;
      Qrho[37] = 0.2313630495538037;
      Qrho[38] = -0.5537409477496875;
      Qrho[39] = -1.0997819806406723;
      Qrho[40] = -0.3739203344950055;
      Qrho[41] = -0.12423900520332376;
      Qrho[42] = -0.923057686995755;
      Qrho[43] = -0.8328289030982696;
      Qrho[44] = -0.16925440270808823;
      Qrho[45] = 1.442135651787706;
      Qrho[46] = 0.34501161787128565;
      Qrho[47] = -0.8660485502711608;
      Qrho[48] = -0.8880899735055947;
      Qrho[49] = -0.1815116979122129;
      Qrho[50] = -1.17835862158005;
      Qrho[51] = -1.1944851558277074;
      Qrho[52] = 0.05614023926976763;
      Qrho[53] = -1.6510825248767813;
      Qrho[54] = -0.06565787059365391;
      Qrho[55] = -0.5512951504486665;
      Qrho[56] = 0.8307464872626844;
      Qrho[57] = 0.9869848924080182;
      Qrho[58] = 0.7643716874230573;
      Qrho[59] = 0.7567216550196565;
      Qrho[60] = -0.5055995034042868;
      Qrho[61] = 0.6725392189410702;
      Qrho[62] = -0.6406053441727284;
      Qrho[63] = 0.29117547947550015;
      Qrho[64] = -0.6967713677405021;
      Qrho[65] = -0.21941980294587182;
      Qrho[66] = -1.753884276680243;
      Qrho[67] = -1.0292983112626475;
      Qrho[68] = 1.8864104246942706;
      Qrho[69] = -1.077663182579704;
      Qrho[70] = 0.7659100437893209;
      Qrho[71] = 0.6019074328549583;
      Qrho[72] = 0.8957565577499285;
      Qrho[73] = -0.09964555746227477;
      Qrho[74] = 0.38665509840745127;
      Qrho[75] = -1.7321223042686946;
      Qrho[76] = -1.7097514487110663;
      Qrho[77] = -1.2040958948116867;
      Qrho[78] = -1.3925560119658358;
      Qrho[79] = -1.5995826216742213;
      Qrho[80] = -1.4828245415645833;
      Qrho[81] = 0.21311092723061398;
      Qrho[82] = -1.248740700304487;
      Qrho[83] = 1.808404972124833;
      Qrho[84] = 0.7264471152297065;
      Qrho[85] = 0.16407869343908477;
      Qrho[86] = 0.8287224032315907;
      Qrho[87] = -0.9444533161899464;
      Qrho[88] = 1.7069027370149112;
      Qrho[89] = 1.3567722311998827;
      Qrho[90] = 0.9052779937121489;
      Qrho[91] = -0.07904017565835986;
      Qrho[92] = 1.3684127435065871;
      Qrho[93] = 0.979009293697437;
      Qrho[94] = 0.6413036255984501;
      Qrho[95] = 1.6559010680237511;
      Qrho[96] = 0.5346622551502991;
      Qrho[97] = -0.5362376605895625;
      Qrho[98] = 0.2113782926017822;
      Qrho[99] = -1.2144776931994525;
      Qrho[100] = -1.2317108144255875;
      Qrho[101] = 0.9026784957312834;
      Qrho[102] = 1.1397468137245244;
      Qrho[103] = 1.8883934547350631;
      Qrho[104] = 1.4038856681660068;
      Qrho[105] = 0.17437730638329096;
      Qrho[106] = -1.6408365219077408;
      Qrho[107] = -0.04450702153554875;
      Qrho[108] = 1.7117453902485025;
      Qrho[109] = 1.1504727980139053;
      Qrho[110] = -0.05962309578364744;
      Qrho[111] = -0.1788825540764547;
      Qrho[112] = -1.1280569263625857;
      Qrho[113] = -1.2911464767927057;
      Qrho[114] = -1.7055053231225696;
      Qrho[115] = 1.56957275034837;
      Qrho[116] = 0.5607064675962357;
      Qrho[117] = -1.4266707301147146;
      Qrho[118] = -0.3434923211351708;
      Qrho[119] = -1.8035643024085055;
      Qrho[120] = -1.1625066019105454;
      Qrho[121] = 0.9228324965161532;
      Qrho[122] = 0.6044910817663975;
      Qrho[123] = -0.0840868104920891;
      Qrho[124] = -0.900877978017443;
      Qrho[125] = 0.608892500264739;
      Qrho[126] = 1.8257980452695217;
      Qrho[127] = -0.25791777529922877;
      Qrho[128] = -1.7194699796493191;
      Qrho[129] = -1.7690740487081298;
      Qrho[130] = -1.6685159248097703;
      Qrho[131] = 1.8388287490128845;
      Qrho[132] = 0.16304334474597537;
      Qrho[133] = 1.3498497306788897;
      Qrho[134] = -1.3198658230514613;
      Qrho[135] = -0.9586197090843394;
      Qrho[136] = 0.7679100474913709;
      Qrho[137] = 1.5822813125679343;
      Qrho[138] = -0.6372460621593619;
      Qrho[139] = -1.741307208038867;
      Qrho[140] = 1.456478677642575;
      Qrho[141] = -0.8365102166820959;
      Qrho[142] = 0.9643296255982503;
      Qrho[143] = -1.367865381194024;
      Qrho[144] = 0.7798537405635035;
      Qrho[145] = 1.3656784761245926;
      Qrho[146] = 0.9086083149868371;
      Qrho[147] = -0.5635699005460344;
      Qrho[148] = 0.9067590059607915;
      Qrho[149] = -1.4421315032701587;
      Qrho[150] = -0.7447235390671119;
      Qrho[151] = -0.32166897326822186;
      Qrho[152] = 1.5088481557772684;
      Qrho[153] = -1.385039165715428;
      Qrho[154] = 1.5204991609972622;
      Qrho[155] = 1.1958572768832156;
      Qrho[156] = 1.8864971883119228;
      Qrho[157] = -0.5291880667861584;
      Qrho[158] = -1.1802409243688836;
      Qrho[159] = -1.037718718661604;
      Qrho[160] = 1.3114512056856835;
      Qrho[161] = 1.8609125943756615;
      Qrho[162] = 0.7952399935216938;
      Qrho[163] = -0.07001183290468038;
      Qrho[164] = -0.8518009412754686;
      Qrho[165] = 1.3347515373726386;
      Qrho[166] = 1.4887180335977037;
      Qrho[167] = -1.6314736327976336;
      Qrho[168] = -1.1362021159208933;
      Qrho[169] = 1.327044361831466;
      Qrho[170] = 1.3932155883179842;
      Qrho[171] = -0.7413880049440107;
      Qrho[172] = -0.8828216126125747;
      Qrho[173] = -0.27673991192616;
      Qrho[174] = 0.15778600105866714;
      Qrho[175] = -1.6177327399735457;
      Qrho[176] = 1.3476485548544606;
      Qrho[177] = 0.13893948140528378;
      Qrho[178] = 1.0998712601636944;
      Qrho[179] = -1.0766549376946926;
      Qrho[180] = 1.8611734044254629;
      Qrho[181] = 1.0041092292735172;
      Qrho[182] = -0.6276245424321543;
      Qrho[183] = 1.794110587839819;
      Qrho[184] = 0.8020471158650913;
      Qrho[185] = 1.362244341944948;
      Qrho[186] = -1.8180107765765245;
      Qrho[187] = -1.7774338357932473;
      Qrho[188] = 0.9709490941985153;
      Qrho[189] = -0.7812542682064318;
      Qrho[190] = 0.0671374633729811;
      Qrho[191] = -1.374950305314906;
      Qrho[192] = 1.9118096386279388;
      Qrho[193] = 0.011004190697677885;
      Qrho[194] = 1.3160043138989015;
      Qrho[195] = -1.7038488148800144;
      Qrho[196] = -0.08433819112864738;
      Qrho[197] = -1.7508820783768964;
      Qrho[198] = 1.536965724350949;
      Qrho[199] = -0.21675928514816478;
      Qrho[200] = -1.725800326952653;
      Qrho[201] = -1.6940148707361717;
      Qrho[202] = 0.15517063201268;
      Qrho[203] = -1.697734381979077;
      Qrho[204] = -1.264910727950229;
      Qrho[205] = -0.2545716633339441;
      Qrho[206] = -0.008868675926170244;
      Qrho[207] = 0.3332476609670296;
      Qrho[208] = 0.48205072561962936;
      Qrho[209] = -0.5087540014293261;
      Qrho[210] = 0.4749463319223195;
      Qrho[211] = -1.371021366459455;
      Qrho[212] = -0.8979660982652256;
      Qrho[213] = 1.194873082385242;
      Qrho[214] = -1.3876427970939353;
      Qrho[215] = -1.106708108457053;
      Qrho[216] = -1.0280872812241797;
      Qrho[217] = -0.08197078070773234;
      Qrho[218] = -1.9970179118324083;
      Qrho[219] = -1.878754557910134;
      Qrho[220] = -0.15380739340877803;
      Qrho[221] = -1.349917260533923;
      Qrho[222] = 0.7180072150931407;
      Qrho[223] = 1.1808183487065538;
      Qrho[224] = 0.31265343495084075;
      Qrho[225] = 0.7790599086928229;
      Qrho[226] = -0.4361679370644853;
      Qrho[227] = -1.8148151880282066;
      Qrho[228] = -0.24231386948140266;
      Qrho[229] = -0.5120787511622411;
      Qrho[230] = 0.3880129688013203;
      Qrho[231] = -1.4631273212038676;
      Qrho[232] = -1.0891484131126563;
      Qrho[233] = 1.2591296661091191;
      Qrho[234] = -0.9426978934391474;
      Qrho[235] = -0.358719180371347;
      Qrho[236] = 1.7438887059831263;
      Qrho[237] = -0.8977901479165817;
      Qrho[238] = -1.4188401645857445;
      Qrho[239] = 0.8080805173258092;
      Qphi[0] = 0.2682662017650985;
      Qphi[1] = 0.44637534218638786;
      Qphi[2] = -1.8318765960257055;
      Qphi[3] = -0.3309324209710929;
      Qphi[4] = -1.9829342633313622;
      Qphi[5] = -1.013858124556442;
      Qphi[6] = 0.8242247343360254;
      Qphi[7] = -1.753837136317201;
      Qphi[8] = -0.8212260055868805;
      Qphi[9] = 1.9524510112487126;
      Qphi[10] = 1.884888920907902;
      Qphi[11] = -0.0726144452811801;
      Qphi[12] = 0.9427735461129836;
      Qphi[13] = 0.5306230967445558;
      Qphi[14] = -0.1372277142250531;
      Qphi[15] = 1.4282657305652786;
      Qphi[16] = -1.309926991335284;
      Qphi[17] = 1.3137276889764422;
      Qphi[18] = -1.8317219061667278;
      Qphi[19] = 1.4678147672511939;
      Qphi[20] = 0.703986349872991;
      Qphi[21] = -0.2163435603565258;
      Qphi[22] = 0.6862809905371079;
      Qphi[23] = -0.15852598444303245;
      Qphi[24] = 1.1200128895143409;
      Qphi[25] = -1.5462236645435308;
      Qphi[26] = 0.0326297153944215;
      Qphi[27] = 1.4859581597754916;
      Qphi[28] = 1.71011710324809;
      Qphi[29] = -1.1186546738067493;
      Qphi[30] = -0.9922787897815244;
      Qphi[31] = 1.6160498864359547;
      Qphi[32] = -0.6179306451394861;
      Qphi[33] = -1.7725097038051376;
      Qphi[34] = 0.8595466884481313;
      Qphi[35] = -0.3423245633865686;
      Qphi[36] = 0.9412967499805762;
      Qphi[37] = -0.09163346622652258;
      Qphi[38] = 0.002262217745727657;
      Qphi[39] = -0.3297523583656421;
      Qphi[40] = -0.8380604158593941;
      Qphi[41] = 1.6028434695494038;
      Qphi[42] = 0.675150311940429;
      Qphi[43] = 1.1553293733718686;
      Qphi[44] = 1.5829581243724693;
      Qphi[45] = -0.9992442304425597;
      Qphi[46] = 1.6792824558896897;
      Qphi[47] = 1.4504203490342324;
      Qphi[48] = 0.02434104849994556;
      Qphi[49] = 0.27160869657612263;
      Qphi[50] = -1.5402710478528858;
      Qphi[51] = 1.0484633622310744;
      Qphi[52] = -1.3070999712627054;
      Qphi[53] = 0.13534416402363814;
      Qphi[54] = -1.4942507790851232;
      Qphi[55] = -1.708331625671371;
      Qphi[56] = 0.436109775042258;
      Qphi[57] = -0.03518748153727991;
      Qphi[58] = 0.6992397389570906;
      Qphi[59] = 1.1634167322171374;
      c[0] = 1.9307499705822648;
      c[1] = -1.6636772756932747;
      c[2] = 0.5248484497343218;
      c[3] = 0.30789958152579144;
      c[4] = 0.602568707166812;
      c[5] = 0.17271781925751872;
      C[0] = 1.5573673875302017;
      C[1] = 1.8685546336404886;
      C[2] = 1.4520116163715753;
      C[3] = 1.5349755786303614;
      C[4] = 1.6909637037652652;
      C[5] = 1.0894949913951089;
      wRho[0] = 0.8638506377746196;
      wPhi[0] = 0.2042684414089766;
      rhoMin[0] = 0.2756197858220666;
      rhoMin[1] = 0.004251116931818011;
      rhoMin[2] = 0.4194128723232424;
      rhoMin[3] = 0.43327452487646845;
      rhoMin[4] = 1.0324889624688858;
      rhoMin[5] = 1.1404164769804863;
      rhoMin[6] = 1.6479223610064944;
      rhoMin[7] = 0.9734223776463142;
      rhoMin[8] = 1.7829091978435834;
      rhoMin[9] = 0.7901215795503316;
      rhoMin[10] = 1.489222894168885;
      rhoMin[11] = 1.1055145248347646;
      rhoMin[12] = 1.2476501715446522;
      rhoMin[13] = 0.5407839937666252;
      rhoMin[14] = 1.875190015879578;
      rhoMin[15] = 1.5393094307157957;
      rhoMin[16] = 0.29119005813981325;
      rhoMin[17] = 1.074868739889147;
      rhoMin[18] = 1.991572611111171;
      rhoMin[19] = 0.09811266501026328;
      rhoMin[20] = 0.6056396758352269;
      rhoMin[21] = 1.4816267427043326;
      rhoMin[22] = 0.07872289530522969;
      rhoMin[23] = 1.4933421819845165;
      rhoMin[24] = 1.146842559967522;
      rhoMin[25] = 1.4634113511241331;
      rhoMin[26] = 1.1016651917532665;
      rhoMin[27] = 1.8788069566023176;
      rhoMin[28] = 0.692803405800541;
      rhoMin[29] = 1.148938919872456;
      rhoMin[30] = 0.10155995800455253;
      rhoMin[31] = 1.1068656683087137;
      rhoMin[32] = 0.8387858872958742;
      rhoMin[33] = 1.966323575580403;
      rhoMin[34] = 1.8912146376740893;
      rhoMin[35] = 0.2765588297162007;
      rhoMin[36] = 0.0832312830619244;
      rhoMin[37] = 0.24135013413781436;
      rhoMin[38] = 0.3854939354396405;
      rhoMin[39] = 1.4523359886211047;
      phiMin[0] = -1.0879559070774472;
      phiMin[1] = -1.069850669070563;
      phiMin[2] = -0.9290739589250738;
      phiMin[3] = -0.013388436763032585;
      phiMin[4] = -0.7849438270889333;
      phiMin[5] = -1.997876882519387;
      phiMin[6] = -1.6405824108238947;
      phiMin[7] = -1.145721421879411;
      phiMin[8] = -0.39292592139055804;
      phiMin[9] = -1.8409388490187077;
      phiMax[0] = 0.8482944948089268;
      phiMax[1] = 1.2386545461589655;
      phiMax[2] = 0.4062153134823505;
      phiMax[3] = 0.6561314876042235;
      phiMax[4] = 0.6899069258691914;
      phiMax[5] = 0.7895037408039216;
      phiMax[6] = 0.04446377311437644;
      phiMax[7] = 1.3206941043903968;
      phiMax[8] = 0.3399800359956484;
      phiMax[9] = 1.2066005265065631;
   }
}
