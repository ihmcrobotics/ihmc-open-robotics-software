package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.DoubleBuffer;

import us.ihmc.utilities.exeptions.NoConvergenceException;
import cern.colt.Arrays;

public class ContactPointWrenchOptimizerNative
{
   public static final int NUMBER_OF_POINTS_PER_CONTACT = 4;
   public static final int NUMBER_OF_SUPPORT_VECTORS = 4;
   public static final int MAX_NUMBER_OF_CONTACTS = 2;
   public static final int WRENCH_LENGTH = 6;

   private static native void initialize();

   private static native ByteBuffer getABuffer();

   private static native ByteBuffer getWBuffer();

   private static native ByteBuffer getCBuffer();

   private static native ByteBuffer getBBuffer();

   private static native ByteBuffer getFminBuffer();

   private static native ByteBuffer getRhoBuffer();

   private static native int solveNative(double epsilon);

   private static native double getOptValNative();

   private static final Object solveConch = new Object();

   private static final DoubleBuffer aDoubleBuffer;
   private static final DoubleBuffer cDoubleBuffer;
   private static final DoubleBuffer bDoubleBuffer;
   private static final DoubleBuffer wDoubleBuffer;
   private static final DoubleBuffer rhoDoubleBuffer;
   private static final DoubleBuffer fMinDoubleBuffer;

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
      System.loadLibrary("ContactPointWrenchOptimizer");

      initialize();

      aDoubleBuffer = setupBuffer(getABuffer());
      cDoubleBuffer = setupBuffer(getCBuffer());
      bDoubleBuffer = setupBuffer(getBBuffer());
      wDoubleBuffer = setupBuffer(getWBuffer());
      rhoDoubleBuffer = setupBuffer(getRhoBuffer());
      fMinDoubleBuffer = setupBuffer(getFminBuffer());
   }

   private final double[] rho = new double[NUMBER_OF_POINTS_PER_CONTACT * NUMBER_OF_SUPPORT_VECTORS * MAX_NUMBER_OF_CONTACTS];
   private double optval = 0.0;

   public int solve(double[] a, double[] c, double[] b, double[] w, double[] fMin, double epsilon) throws NoConvergenceException
   {
      int numberOfIterations;

      synchronized (solveConch)
      {
         setBufferToArray(aDoubleBuffer, a);
         setBufferToArray(cDoubleBuffer, c);
         setBufferToArray(bDoubleBuffer, b);
         setBufferToArray(wDoubleBuffer, w);
         setBufferToArray(fMinDoubleBuffer, fMin);

         numberOfIterations = solveNative(epsilon);

         rhoDoubleBuffer.rewind();
         rhoDoubleBuffer.get(rho);

         optval = getOptValNative();
      }

      if (numberOfIterations < 0)
      {
         throw new NoConvergenceException();
      }

      return numberOfIterations;

   }

   public double[] getRho(int i)
   {
      return rho;
   }

   public double getOptval()
   {
      return optval;
   }

   public static void main(String[] args) throws NoConvergenceException
   {
      double[] a = new double[WRENCH_LENGTH * NUMBER_OF_SUPPORT_VECTORS * NUMBER_OF_POINTS_PER_CONTACT * MAX_NUMBER_OF_CONTACTS];
      double[] w = new double[WRENCH_LENGTH];
      double[] c = new double[WRENCH_LENGTH * WRENCH_LENGTH];
      double[] b = new double[MAX_NUMBER_OF_CONTACTS * NUMBER_OF_SUPPORT_VECTORS * NUMBER_OF_POINTS_PER_CONTACT * MAX_NUMBER_OF_CONTACTS];
      double[] fMin = new double[MAX_NUMBER_OF_CONTACTS];
      double[] epsilon = new double[1];

      load_default_data(a, w, c, b, fMin, epsilon);

      ContactPointWrenchOptimizerNative leeGoswamiForceOptimizerNative = new ContactPointWrenchOptimizerNative();

      long time = System.nanoTime();
      for (int i = 0; true; i++)
      {
         if (i % 10000 == 0)
         {
            System.out.println("10000 iterations took " + (System.nanoTime() - time) / 1e9 + " seconds");

            for (int j = 0; j < MAX_NUMBER_OF_CONTACTS; j++)
            {
               System.out.println(Arrays.toString(leeGoswamiForceOptimizerNative.getRho(j)));
            }

            time = System.nanoTime();
         }

         leeGoswamiForceOptimizerNative.solve(a, c, b, w, fMin, epsilon[0]);
      }
   }

   private static void load_default_data(double[] A, double[] W, double[] C, double[] B, double[] fmin, double[] epsilon)
   {
      A[0] = 0.20319161029830202;
      A[1] = 0.8325912904724193;
      A[2] = -0.8363810443482227;
      A[3] = 0.04331042079065206;
      A[4] = 1.5717878173906188;
      A[5] = 1.5851723557337523;
      A[6] = -1.497658758144655;
      A[7] = -1.171028487447253;
      A[8] = -1.7941311867966805;
      A[9] = -0.23676062539745413;
      A[10] = -1.8804951564857322;
      A[11] = -0.17266710242115568;
      A[12] = 0.596576190459043;
      A[13] = -0.8860508694080989;
      A[14] = 0.7050196079205251;
      A[15] = 0.3634512696654033;
      A[16] = -1.9040724704913385;
      A[17] = 0.23541635196352795;
      A[18] = -0.9629902123701384;
      A[19] = -0.3395952119597214;
      A[20] = -0.865899672914725;
      A[21] = 0.7725516732519853;
      A[22] = -0.23818512931704205;
      A[23] = -1.372529046100147;
      A[24] = 0.17859607212737894;
      A[25] = 1.1212590580454682;
      A[26] = -0.774545870495281;
      A[27] = -1.1121684642712744;
      A[28] = -0.44811496977740495;
      A[29] = 1.7455345994417217;
      A[30] = 1.9039816898917352;
      A[31] = 0.6895347036512547;
      A[32] = 1.6113364341535923;
      A[33] = 1.383003485172717;
      A[34] = -0.48802383468444344;
      A[35] = -1.631131964513103;
      A[36] = 0.6136436100941447;
      A[37] = 0.2313630495538037;
      A[38] = -0.5537409477496875;
      A[39] = -1.0997819806406723;
      A[40] = -0.3739203344950055;
      A[41] = -0.12423900520332376;
      A[42] = -0.923057686995755;
      A[43] = -0.8328289030982696;
      A[44] = -0.16925440270808823;
      A[45] = 1.442135651787706;
      A[46] = 0.34501161787128565;
      A[47] = -0.8660485502711608;
      A[48] = -0.8880899735055947;
      A[49] = -0.1815116979122129;
      A[50] = -1.17835862158005;
      A[51] = -1.1944851558277074;
      A[52] = 0.05614023926976763;
      A[53] = -1.6510825248767813;
      A[54] = -0.06565787059365391;
      A[55] = -0.5512951504486665;
      A[56] = 0.8307464872626844;
      A[57] = 0.9869848924080182;
      A[58] = 0.7643716874230573;
      A[59] = 0.7567216550196565;
      A[60] = -0.5055995034042868;
      A[61] = 0.6725392189410702;
      A[62] = -0.6406053441727284;
      A[63] = 0.29117547947550015;
      A[64] = -0.6967713677405021;
      A[65] = -0.21941980294587182;
      A[66] = -1.753884276680243;
      A[67] = -1.0292983112626475;
      A[68] = 1.8864104246942706;
      A[69] = -1.077663182579704;
      A[70] = 0.7659100437893209;
      A[71] = 0.6019074328549583;
      A[72] = 0.8957565577499285;
      A[73] = -0.09964555746227477;
      A[74] = 0.38665509840745127;
      A[75] = -1.7321223042686946;
      A[76] = -1.7097514487110663;
      A[77] = -1.2040958948116867;
      A[78] = -1.3925560119658358;
      A[79] = -1.5995826216742213;
      A[80] = -1.4828245415645833;
      A[81] = 0.21311092723061398;
      A[82] = -1.248740700304487;
      A[83] = 1.808404972124833;
      A[84] = 0.7264471152297065;
      A[85] = 0.16407869343908477;
      A[86] = 0.8287224032315907;
      A[87] = -0.9444533161899464;
      A[88] = 1.7069027370149112;
      A[89] = 1.3567722311998827;
      A[90] = 0.9052779937121489;
      A[91] = -0.07904017565835986;
      A[92] = 1.3684127435065871;
      A[93] = 0.979009293697437;
      A[94] = 0.6413036255984501;
      A[95] = 1.6559010680237511;
      A[96] = 0.5346622551502991;
      A[97] = -0.5362376605895625;
      A[98] = 0.2113782926017822;
      A[99] = -1.2144776931994525;
      A[100] = -1.2317108144255875;
      A[101] = 0.9026784957312834;
      A[102] = 1.1397468137245244;
      A[103] = 1.8883934547350631;
      A[104] = 1.4038856681660068;
      A[105] = 0.17437730638329096;
      A[106] = -1.6408365219077408;
      A[107] = -0.04450702153554875;
      A[108] = 1.7117453902485025;
      A[109] = 1.1504727980139053;
      A[110] = -0.05962309578364744;
      A[111] = -0.1788825540764547;
      A[112] = -1.1280569263625857;
      A[113] = -1.2911464767927057;
      A[114] = -1.7055053231225696;
      A[115] = 1.56957275034837;
      A[116] = 0.5607064675962357;
      A[117] = -1.4266707301147146;
      A[118] = -0.3434923211351708;
      A[119] = -1.8035643024085055;
      A[120] = -1.1625066019105454;
      A[121] = 0.9228324965161532;
      A[122] = 0.6044910817663975;
      A[123] = -0.0840868104920891;
      A[124] = -0.900877978017443;
      A[125] = 0.608892500264739;
      A[126] = 1.8257980452695217;
      A[127] = -0.25791777529922877;
      A[128] = -1.7194699796493191;
      A[129] = -1.7690740487081298;
      A[130] = -1.6685159248097703;
      A[131] = 1.8388287490128845;
      A[132] = 0.16304334474597537;
      A[133] = 1.3498497306788897;
      A[134] = -1.3198658230514613;
      A[135] = -0.9586197090843394;
      A[136] = 0.7679100474913709;
      A[137] = 1.5822813125679343;
      A[138] = -0.6372460621593619;
      A[139] = -1.741307208038867;
      A[140] = 1.456478677642575;
      A[141] = -0.8365102166820959;
      A[142] = 0.9643296255982503;
      A[143] = -1.367865381194024;
      A[144] = 0.7798537405635035;
      A[145] = 1.3656784761245926;
      A[146] = 0.9086083149868371;
      A[147] = -0.5635699005460344;
      A[148] = 0.9067590059607915;
      A[149] = -1.4421315032701587;
      A[150] = -0.7447235390671119;
      A[151] = -0.32166897326822186;
      A[152] = 1.5088481557772684;
      A[153] = -1.385039165715428;
      A[154] = 1.5204991609972622;
      A[155] = 1.1958572768832156;
      A[156] = 1.8864971883119228;
      A[157] = -0.5291880667861584;
      A[158] = -1.1802409243688836;
      A[159] = -1.037718718661604;
      A[160] = 1.3114512056856835;
      A[161] = 1.8609125943756615;
      A[162] = 0.7952399935216938;
      A[163] = -0.07001183290468038;
      A[164] = -0.8518009412754686;
      A[165] = 1.3347515373726386;
      A[166] = 1.4887180335977037;
      A[167] = -1.6314736327976336;
      A[168] = -1.1362021159208933;
      A[169] = 1.327044361831466;
      A[170] = 1.3932155883179842;
      A[171] = -0.7413880049440107;
      A[172] = -0.8828216126125747;
      A[173] = -0.27673991192616;
      A[174] = 0.15778600105866714;
      A[175] = -1.6177327399735457;
      A[176] = 1.3476485548544606;
      A[177] = 0.13893948140528378;
      A[178] = 1.0998712601636944;
      A[179] = -1.0766549376946926;
      A[180] = 1.8611734044254629;
      A[181] = 1.0041092292735172;
      A[182] = -0.6276245424321543;
      A[183] = 1.794110587839819;
      A[184] = 0.8020471158650913;
      A[185] = 1.362244341944948;
      A[186] = -1.8180107765765245;
      A[187] = -1.7774338357932473;
      A[188] = 0.9709490941985153;
      A[189] = -0.7812542682064318;
      A[190] = 0.0671374633729811;
      A[191] = -1.374950305314906;
      W[0] = 1.9118096386279388;
      W[1] = 0.011004190697677885;
      W[2] = 1.3160043138989015;
      W[3] = -1.7038488148800144;
      W[4] = -0.08433819112864738;
      W[5] = -1.7508820783768964;
      C[0] = 1.8842414310877373;
      C[1] = 1.445810178712959;
      C[2] = 1.0685499182618368;
      C[3] = 1.076496282315957;
      C[4] = 1.53879265800317;
      C[5] = 1.0755664045052309;
      epsilon[0] = 0.3675446360248855;
      B[0] = -0.2545716633339441;
      B[1] = -0.008868675926170244;
      B[2] = 0.3332476609670296;
      B[3] = 0.48205072561962936;
      B[4] = -0.5087540014293261;
      B[5] = 0.4749463319223195;
      B[6] = -1.371021366459455;
      B[7] = -0.8979660982652256;
      B[8] = 1.194873082385242;
      B[9] = -1.3876427970939353;
      B[10] = -1.106708108457053;
      B[11] = -1.0280872812241797;
      B[12] = -0.08197078070773234;
      B[13] = -1.9970179118324083;
      B[14] = -1.878754557910134;
      B[15] = -0.15380739340877803;
      B[16] = -1.349917260533923;
      B[17] = 0.7180072150931407;
      B[18] = 1.1808183487065538;
      B[19] = 0.31265343495084075;
      B[20] = 0.7790599086928229;
      B[21] = -0.4361679370644853;
      B[22] = -1.8148151880282066;
      B[23] = -0.24231386948140266;
      B[24] = -0.5120787511622411;
      B[25] = 0.3880129688013203;
      B[26] = -1.4631273212038676;
      B[27] = -1.0891484131126563;
      B[28] = 1.2591296661091191;
      B[29] = -0.9426978934391474;
      B[30] = -0.358719180371347;
      B[31] = 1.7438887059831263;
      B[32] = -0.8977901479165817;
      B[33] = -1.4188401645857445;
      B[34] = 0.8080805173258092;
      B[35] = 0.2682662017650985;
      B[36] = 0.44637534218638786;
      B[37] = -1.8318765960257055;
      B[38] = -0.3309324209710929;
      B[39] = -1.9829342633313622;
      B[40] = -1.013858124556442;
      B[41] = 0.8242247343360254;
      B[42] = -1.753837136317201;
      B[43] = -0.8212260055868805;
      B[44] = 1.9524510112487126;
      B[45] = 1.884888920907902;
      B[46] = -0.0726144452811801;
      B[47] = 0.9427735461129836;
      B[48] = 0.5306230967445558;
      B[49] = -0.1372277142250531;
      B[50] = 1.4282657305652786;
      B[51] = -1.309926991335284;
      B[52] = 1.3137276889764422;
      B[53] = -1.8317219061667278;
      B[54] = 1.4678147672511939;
      B[55] = 0.703986349872991;
      B[56] = -0.2163435603565258;
      B[57] = 0.6862809905371079;
      B[58] = -0.15852598444303245;
      B[59] = 1.1200128895143409;
      B[60] = -1.5462236645435308;
      B[61] = 0.0326297153944215;
      B[62] = 1.4859581597754916;
      B[63] = 1.71011710324809;
      fmin[0] = -1.1186546738067493;
      fmin[1] = -0.9922787897815244;
   }
}
