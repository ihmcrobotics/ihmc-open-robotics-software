package us.ihmc.simulationConstructionSetTools.util.environments;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;

/**
 * SCS environments defined from RDX JSON-defined environments. See RDXEnvironmentExporter to generate method
 */
public enum RDXEnvironments
{
   LOOK_AND_STEP_EASY,
   LOOK_AND_STEP_HARD;

   public CommonAvatarEnvironmentInterface getEnvironment()
   {
      CombinedTerrainObject3D terrain = new CombinedTerrainObject3D(getClass().getSimpleName());
      YoAppearanceTexture groundTexture = new YoAppearanceTexture("Textures/gridGroundProfile.png");

      Box3D ground = new Box3D(50.0, 50.0, 1.0);
      ground.getPosition().setZ(-0.5);
      terrain.addTerrainObject(new RotatableBoxTerrainObject(ground, groundTexture));

      switch (this)
      {
         case LOOK_AND_STEP_EASY -> addLookAndStepSimple(terrain);
         case LOOK_AND_STEP_HARD -> addLookAndStepHard(terrain);
      }

      terrain.addTerrainObject(terrain);
      return () -> terrain;
   }
   
   private static void addLookAndStepSimple(CombinedTerrainObject3D terrain)
   {
      AppearanceDefinition appearance = YoAppearance.DarkGray();

      Point3D position0 = new Point3D(2.9991922858880433, 0.17738506103097837, 0.07749255480474931);
      Quaternion orientation0 = new Quaternion(3.591011281523848E-4, -4.404093410555292E-4, -0.7140274385984735, 0.7001174858654047);
      Vector3D size0 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box0 = new Box3D(position0, orientation0, size0);
      terrain.addRotatableBox(box0, appearance);

      Point3D position1 = new Point3D(4.010313695756378, 0.1616056331805633, 0.07664394760851354);
      Quaternion orientation1 = new Quaternion(0.0010814627453136847, 0.0011373925186028025, -0.7128015132552289,0.7013640563057426);
      Vector3D size1 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box1 = new Box3D(position1, orientation1, size1);
      terrain.addRotatableBox(box1, appearance);

      Point3D position2 = new Point3D(1.538532083708091, 0.6741701950310802, 0.05131964350890132);
      Quaternion orientation2 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size2 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box2 = new Box3D(position2, orientation2, size2);
      terrain.addRotatableBox(box2, appearance);

      Point3D position3 = new Point3D(1.5377122418466396, 0.4871995187212327, 0.0453699739307727);
      Quaternion orientation3 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size3 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box3 = new Box3D(position3, orientation3, size3);
      terrain.addRotatableBox(box3, appearance);

      Point3D position4 = new Point3D(1.8318418085847752, 0.5832432291386377, 0.05026246658367628);
      Quaternion orientation4 = new Quaternion(0.0, 0.0, -0.7032798171527006,0.7109131443331624);
      Vector3D size4 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box4 = new Box3D(position4, orientation4, size4);
      terrain.addRotatableBox(box4, appearance);

      Point3D position5 = new Point3D(2.0226812492006916, 0.5816220959595908, 0.058535600491900285);
      Quaternion orientation5 = new Quaternion(0.0, 0.0, -0.7033244049950812,0.7108690324794822);
      Vector3D size5 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box5 = new Box3D(position5, orientation5, size5);
      terrain.addRotatableBox(box5, appearance);

      Point3D position6 = new Point3D(1.9258075527472558, 0.2916058787804267, 0.04914770991489481);
      Quaternion orientation6 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size6 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box6 = new Box3D(position6, orientation6, size6);
      terrain.addRotatableBox(box6, appearance);

      Point3D position7 = new Point3D(1.9221267562656839, 0.0971054615729663, 0.048374686104427383);
      Quaternion orientation7 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size7 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box7 = new Box3D(position7, orientation7, size7);
      terrain.addRotatableBox(box7, appearance);

      Point3D position8 = new Point3D(1.6391139250203717, 0.20003028808817114, 0.03909051303520233);
      Quaternion orientation8 = new Quaternion(4.2221998162823653E-4, -0.0021771612644779165, -0.7083346233855763,0.7058733193798512);
      Vector3D size8 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box8 = new Box3D(position8, orientation8, size8);
      terrain.addRotatableBox(box8, appearance);

      Point3D position9 = new Point3D(1.4508258759747759, 0.19187378577710404, 0.03772504674415519);
      Quaternion orientation9 = new Quaternion(0.0, 0.0, 0.7038510074598656,0.710347632710726);
      Vector3D size9 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box9 = new Box3D(position9, orientation9, size9);
      terrain.addRotatableBox(box9, appearance);

      Point3D position10 = new Point3D(1.544254053576254, -0.09368499925145309, 0.032965432002729965);
      Quaternion orientation10 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size10 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box10 = new Box3D(position10, orientation10, size10);
      terrain.addRotatableBox(box10, appearance);

      Point3D position11 = new Point3D(1.5421604510537665, -0.2847681953194701, 0.03169062655032244);
      Quaternion orientation11 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size11 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box11 = new Box3D(position11, orientation11, size11);
      terrain.addRotatableBox(box11, appearance);

      Point3D position12 = new Point3D(1.8302064340875628, -0.18772505668791095, 0.03233411894842265);
      Quaternion orientation12 = new Quaternion(0.0, 0.0, -0.7041659007794275,0.7100354809300009);
      Vector3D size12 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box12 = new Box3D(position12, orientation12, size12);
      terrain.addRotatableBox(box12, appearance);

      Point3D position13 = new Point3D(2.020076111592974, -0.19450183861517623, 0.032951829300953354);
      Quaternion orientation13 = new Quaternion(0.0, 0.0, 0.7094730567086567,0.7047325604826807);
      Vector3D size13 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box13 = new Box3D(position13, orientation13, size13);
      terrain.addRotatableBox(box13, appearance);

      Point3D position14 = new Point3D(2.3017726687557736, 0.6659211869311393, 0.050732948560174566);
      Quaternion orientation14 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size14 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box14 = new Box3D(position14, orientation14, size14);
      terrain.addRotatableBox(box14, appearance);

      Point3D position15 = new Point3D(2.3045613478518816, 0.4744061119031097, 0.04678489901068887);
      Quaternion orientation15 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size15 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box15 = new Box3D(position15, orientation15, size15);
      terrain.addRotatableBox(box15, appearance);

      Point3D position16 = new Point3D(2.214159178501207, 0.19092562262741894, 0.04546863118006792);
      Quaternion orientation16 = new Quaternion(0.0, 0.0, -0.7100452702091272,0.7041560297644605);
      Vector3D size16 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box16 = new Box3D(position16, orientation16, size16);
      terrain.addRotatableBox(box16, appearance);

      Point3D position17 = new Point3D(2.3882625449800243, 0.1854857053173351, 0.04358061639227621);
      Quaternion orientation17 = new Quaternion(0.0, 0.0, 0.7032305077721187,0.710961920878023);
      Vector3D size17 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box17 = new Box3D(position17, orientation17, size17);
      terrain.addRotatableBox(box17, appearance);

      Point3D position18 = new Point3D(2.305334951328886, -0.09669125069873875, 0.04026317060808564);
      Quaternion orientation18 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size18 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box18 = new Box3D(position18, orientation18, size18);
      terrain.addRotatableBox(box18, appearance);

      Point3D position19 = new Point3D(2.3035618801058884, -0.28517226345878166, 0.043838376674458175);
      Quaternion orientation19 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size19 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box19 = new Box3D(position19, orientation19, size19);
      terrain.addRotatableBox(box19, appearance);

      Point3D position20 = new Point3D(5.022827545101327, 0.14359272115178368, 0.06929944023803199);
      Quaternion orientation20 = new Quaternion(0.0, 0.0, -0.7146326573404541,0.6994999392870033);
      Vector3D size20 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box20 = new Box3D(position20, orientation20, size20);
      terrain.addRotatableBox(box20, appearance);

      Point3D position21 = new Point3D(6.0368497952581395, 0.10384214494564183, 0.06172802598521781);
      Quaternion orientation21 = new Quaternion(0.0056138542832086386, 0.00558201305529614, -0.7143726400604126,0.6997206992103732);
      Vector3D size21 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box21 = new Box3D(position21, orientation21, size21);
      terrain.addRotatableBox(box21, appearance);

      Point3D position22 = new Point3D(7.050523277840206, 0.08289463608396092, 0.06864253875907342);
      Quaternion orientation22 = new Quaternion(0.0, 0.0, 0.7017961773791696,0.7123777968297335);
      Vector3D size22 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box22 = new Box3D(position22, orientation22, size22);
      terrain.addRotatableBox(box22, appearance);

      Point3D position23 = new Point3D(5.023846234292196, 0.13613703474961703, 0.22602295943394107);
      Quaternion orientation23 = new Quaternion(0.0, 0.0, 0.7009087265528494,0.7132509775962896);
      Vector3D size23 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box23 = new Box3D(position23, orientation23, size23);
      terrain.addRotatableBox(box23, appearance);

      Point3D position24 = new Point3D(2.6900015009577163, 0.6436366016069017, 0.19963009876119422);
      Quaternion orientation24 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size24 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box24 = new Box3D(position24, orientation24, size24);
      terrain.addRotatableBox(box24, appearance);

      Point3D position25 = new Point3D(2.6890564650088904, 0.4543195036924751, 0.1998904919894186);
      Quaternion orientation25 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size25 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box25 = new Box3D(position25, orientation25, size25);
      terrain.addRotatableBox(box25, appearance);

      Point3D position26 = new Point3D(2.7797616190377004, 0.16496187008841534, 0.19985815206905305);
      Quaternion orientation26 = new Quaternion(0.0, 0.0, -0.7091191010590019,0.7050887181860686);
      Vector3D size26 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box26 = new Box3D(position26, orientation26, size26);
      terrain.addRotatableBox(box26, appearance);

      Point3D position27 = new Point3D(2.593463551704851, 0.1621908804922451, 0.19471440676353352);
      Quaternion orientation27 = new Quaternion(0.0, 0.0, 0.7049836505970685,0.7092235560039095);
      Vector3D size27 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box27 = new Box3D(position27, orientation27, size27);
      terrain.addRotatableBox(box27, appearance);

      Point3D position28 = new Point3D(2.684024192233144, -0.13061899475743363, 0.19603696340587146);
      Quaternion orientation28 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size28 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box28 = new Box3D(position28, orientation28, size28);
      terrain.addRotatableBox(box28, appearance);

      Point3D position29 = new Point3D(2.6860150703200567, -0.320451962389396, 0.19828815283388773);
      Quaternion orientation29 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size29 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box29 = new Box3D(position29, orientation29, size29);
      terrain.addRotatableBox(box29, appearance);

      Point3D position30 = new Point3D(3.1711383025454465, 0.5462180514407013, 0.22616424914213662);
      Quaternion orientation30 = new Quaternion(0.0, 0.0, -0.7038282851139497,0.7103701465247234);
      Vector3D size30 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box30 = new Box3D(position30, orientation30, size30);
      terrain.addRotatableBox(box30, appearance);

      Point3D position31 = new Point3D(2.9818424101161205, 0.5424769634680712, 0.22464735511398604);
      Quaternion orientation31 = new Quaternion(0.0, 0.0, 0.709774201169097,0.7044292607173344);
      Vector3D size31 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box31 = new Box3D(position31, orientation31, size31);
      terrain.addRotatableBox(box31, appearance);

      Point3D position32 = new Point3D(3.0756668786094545, 0.2573398848240338, 0.22430098450189007);
      Quaternion orientation32 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size32 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box32 = new Box3D(position32, orientation32, size32);
      terrain.addRotatableBox(box32, appearance);

      Point3D position33 = new Point3D(3.0722024139657997, 0.06783466183720784, 0.2251948202315323);
      Quaternion orientation33 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size33 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box33 = new Box3D(position33, orientation33, size33);
      terrain.addRotatableBox(box33, appearance);

      Point3D position34 = new Point3D(3.165664362999164, -0.22522204408849275, 0.22681213465404784);
      Quaternion orientation34 = new Quaternion(0.0, 0.0, -0.7083652138817587,0.7058461048714872);
      Vector3D size34 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box34 = new Box3D(position34, orientation34, size34);
      terrain.addRotatableBox(box34, appearance);

      Point3D position35 = new Point3D(2.974840053678399, -0.22519875771755868, 0.22595308610329148);
      Quaternion orientation35 = new Quaternion(0.0, 0.0, 0.7081687952141784,0.7060431697034532);
      Vector3D size35 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box35 = new Box3D(position35, orientation35, size35);
      terrain.addRotatableBox(box35, appearance);

      Point3D position36 = new Point3D(3.359147869089035, 0.550879534175364, 0.22486106332834727);
      Quaternion orientation36 = new Quaternion(0.0, 0.0, -0.7071194347376093,0.7070941274090488);
      Vector3D size36 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box36 = new Box3D(position36, orientation36, size36);
      terrain.addRotatableBox(box36, appearance);

      Point3D position37 = new Point3D(3.354472458529534, 0.15783397615504235, 0.22395977136362658);
      Quaternion orientation37 = new Quaternion(0.0, 0.0, -0.7058785319428242,0.7083329006492944);
      Vector3D size37 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box37 = new Box3D(position37, orientation37, size37);
      terrain.addRotatableBox(box37, appearance);

      Point3D position38 = new Point3D(3.350402623341776, -0.22691663331160317, 0.22640085739147484);
      Quaternion orientation38 = new Quaternion(0.0, 0.0, 0.7062242020231959,0.7079882601263245);
      Vector3D size38 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box38 = new Box3D(position38, orientation38, size38);
      terrain.addRotatableBox(box38, appearance);

      Point3D position39 = new Point3D(3.68866762247556, 0.6236265834842826, 0.2503418207870058);
      Quaternion orientation39 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size39 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box39 = new Box3D(position39, orientation39, size39);
      terrain.addRotatableBox(box39, appearance);

      Point3D position40 = new Point3D(3.7706107394082076, 0.15186184665978525, 0.2572179895864939);
      Quaternion orientation40 = new Quaternion(0.0, 0.0, -0.7053501510530733,0.708859058212144);
      Vector3D size40 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box40 = new Box3D(position40, orientation40, size40);
      terrain.addRotatableBox(box40, appearance);

      Point3D position41 = new Point3D(3.68965927337154, 0.4388934247654853, 0.24918503610056578);
      Quaternion orientation41 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size41 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box41 = new Box3D(position41, orientation41, size41);
      terrain.addRotatableBox(box41, appearance);

      Point3D position42 = new Point3D(3.588806285529547, 0.15484013654183854, 0.24678723679981535);
      Quaternion orientation42 = new Quaternion(0.006622056925947793, 0.006517902589659019, 0.7126585328208642,0.7014495569216039);
      Vector3D size42 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box42 = new Box3D(position42, orientation42, size42);
      terrain.addRotatableBox(box42, appearance);

      Point3D position43 = new Point3D(3.6964906211913506, -0.14386449366407408, 0.24588881335187482);
      Quaternion orientation43 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size43 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box43 = new Box3D(position43, orientation43, size43);
      terrain.addRotatableBox(box43, appearance);

      Point3D position44 = new Point3D(3.6860421913290207, -0.33336370841482466, 0.24832633958306163);
      Quaternion orientation44 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size44 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box44 = new Box3D(position44, orientation44, size44);
      terrain.addRotatableBox(box44, appearance);

      Point3D position45 = new Point3D(3.9745492483528864, -0.23633221329182877, 0.2477310686455436);
      Quaternion orientation45 = new Quaternion(0.0, 0.0, 0.7074123606850091,0.70680106957337);
      Vector3D size45 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box45 = new Box3D(position45, orientation45, size45);
      terrain.addRotatableBox(box45, appearance);

      Point3D position46 = new Point3D(4.1570571372942995, 0.5351500725177174, 0.24857256496015911);
      Quaternion orientation46 = new Quaternion(0.0, 0.0, -0.7031379195773428,0.7110534902891946);
      Vector3D size46 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box46 = new Box3D(position46, orientation46, size46);
      terrain.addRotatableBox(box46, appearance);

      Point3D position47 = new Point3D(3.976449906945254, 0.5313406371863271, 0.25024361307815357);
      Quaternion orientation47 = new Quaternion(0.0, 0.0, 0.7053477339779689,0.7088614633143371);
      Vector3D size47 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box47 = new Box3D(position47, orientation47, size47);
      terrain.addRotatableBox(box47, appearance);

      Point3D position48 = new Point3D(4.060087891099629, 0.2392964224398486, 0.24882388075031026);
      Quaternion orientation48 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size48 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box48 = new Box3D(position48, orientation48, size48);
      terrain.addRotatableBox(box48, appearance);

      Point3D position49 = new Point3D(4.063602710036821, 0.052570508322739594, 0.2470906830235654);
      Quaternion orientation49 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size49 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box49 = new Box3D(position49, orientation49, size49);
      terrain.addRotatableBox(box49, appearance);

      Point3D position50 = new Point3D(4.147655172749554, -0.2365996880994438, 0.2511546494352128);
      Quaternion orientation50 = new Quaternion(0.0, 0.0, -0.7069099517221287,0.7073035558769786);
      Vector3D size50 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box50 = new Box3D(position50, orientation50, size50);
      terrain.addRotatableBox(box50, appearance);

      Point3D position51 = new Point3D(4.343356717444665, -0.23511809278230003, 0.24226611243093557);
      Quaternion orientation51 = new Quaternion(-0.011094912138101922, -0.016513086855919283, 0.7006319373826693,0.7132454761206297);
      Vector3D size51 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box51 = new Box3D(position51, orientation51, size51);
      terrain.addRotatableBox(box51, appearance);

      Point3D position52 = new Point3D(4.349272116834419, 0.15033640054496056, 0.23830329314841697);
      Quaternion orientation52 = new Quaternion(0.0, 0.0, -0.711526223471408,0.7026595429598301);
      Vector3D size52 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box52 = new Box3D(position52, orientation52, size52);
      terrain.addRotatableBox(box52, appearance);

      Point3D position53 = new Point3D(4.346481880050672, 0.5267456748407474, 0.24375773924450508);
      Quaternion orientation53 = new Quaternion(0.0, 0.0, -0.706513253073592,0.7076998115241873);
      Vector3D size53 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box53 = new Box3D(position53, orientation53, size53);
      terrain.addRotatableBox(box53, appearance);

      Point3D position54 = new Point3D(4.702380013634424, 0.6079336148133885, 0.3479183895982199);
      Quaternion orientation54 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size54 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box54 = new Box3D(position54, orientation54, size54);
      terrain.addRotatableBox(box54, appearance);

      Point3D position55 = new Point3D(4.703890280086529, 0.4155963177652986, 0.34604430953711496);
      Quaternion orientation55 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size55 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box55 = new Box3D(position55, orientation55, size55);
      terrain.addRotatableBox(box55, appearance);

      Point3D position56 = new Point3D(4.792888242703473, 0.12489420821974778, 0.3479451232275179);
      Quaternion orientation56 = new Quaternion(0.0, 0.0, -0.7035865871495103,0.7106095372166822);
      Vector3D size56 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box56 = new Box3D(position56, orientation56, size56);
      terrain.addRotatableBox(box56, appearance);

      Point3D position57 = new Point3D(4.610354981743812, 0.12911728605722894, 0.3475132664494318);
      Quaternion orientation57 = new Quaternion(0.0, 0.0, -0.7132505655401294,0.7009091458646314);
      Vector3D size57 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box57 = new Box3D(position57, orientation57, size57);
      terrain.addRotatableBox(box57, appearance);

      Point3D position58 = new Point3D(4.712970930242387, -0.1602010896506567, 0.34378735053868653);
      Quaternion orientation58 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size58 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box58 = new Box3D(position58, orientation58, size58);
      terrain.addRotatableBox(box58, appearance);

      Point3D position59 = new Point3D(4.707914463288, -0.34860534277151983, 0.34697375743836595);
      Quaternion orientation59 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size59 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box59 = new Box3D(position59, orientation59, size59);
      terrain.addRotatableBox(box59, appearance);

      Point3D position60 = new Point3D(5.1727760970934185, 0.5112340340750825, 0.3478127861059397);
      Quaternion orientation60 = new Quaternion(0.0, 0.0, -0.7027188610745567,0.7114676396647134);
      Vector3D size60 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box60 = new Box3D(position60, orientation60, size60);
      terrain.addRotatableBox(box60, appearance);

      Point3D position61 = new Point3D(4.986578998410523, 0.5045728830128698, 0.35723923234878907);
      Quaternion orientation61 = new Quaternion(0.0, 0.0, 0.7072488450361675,0.7069646887893388);
      Vector3D size61 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box61 = new Box3D(position61, orientation61, size61);
      terrain.addRotatableBox(box61, appearance);

      Point3D position62 = new Point3D(5.0805297951234305, 0.21436861966465973, 0.3490365620576039);
      Quaternion orientation62 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size62 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box62 = new Box3D(position62, orientation62, size62);
      terrain.addRotatableBox(box62, appearance);

      Point3D position63 = new Point3D(5.082643679829409, 0.025622949614205873, 0.34617777218674023);
      Quaternion orientation63 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size63 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box63 = new Box3D(position63, orientation63, size63);
      terrain.addRotatableBox(box63, appearance);

      Point3D position64 = new Point3D(5.181134977089534, -0.2648254795801891, 0.34319902751110637);
      Quaternion orientation64 = new Quaternion(0.0, 0.0, -0.7087044927674657,0.705505451383057);
      Vector3D size64 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box64 = new Box3D(position64, orientation64, size64);
      terrain.addRotatableBox(box64, appearance);

      Point3D position65 = new Point3D(5.001168160091194, -0.26897331063196755, 0.3442518243348339);
      Quaternion orientation65 = new Quaternion(0.0, 0.0, 0.7085911489028206,0.7056192909045078);
      Vector3D size65 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box65 = new Box3D(position65, orientation65, size65);
      terrain.addRotatableBox(box65, appearance);

      Point3D position66 = new Point3D(5.361423516842161, -0.26268163874752193, 0.3467424072643232);
      Quaternion orientation66 = new Quaternion(0.0, 0.0, -0.702830138774019,0.7113577131309495);
      Vector3D size66 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box66 = new Box3D(position66, orientation66, size66);
      terrain.addRotatableBox(box66, appearance);

      Point3D position67 = new Point3D(5.368768670021752, 0.1171590670986501, 0.3453752030174442);
      Quaternion orientation67 = new Quaternion(0.0, 0.0, -0.7086738484704129,0.7055362332964443);
      Vector3D size67 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box67 = new Box3D(position67, orientation67, size67);
      terrain.addRotatableBox(box67, appearance);

      Point3D position68 = new Point3D(5.359430458701423, 0.5074604944131372, 0.3480032395435422);
      Quaternion orientation68 = new Quaternion(0.0, 0.0, 0.7069593494590138,0.7072541821809802);
      Vector3D size68 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box68 = new Box3D(position68, orientation68, size68);
      terrain.addRotatableBox(box68, appearance);

      Point3D position69 = new Point3D(5.75258973537161, -0.3504907819884919, 0.227418511493009);
      Quaternion orientation69 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size69 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box69 = new Box3D(position69, orientation69, size69);
      terrain.addRotatableBox(box69, appearance);

      Point3D position70 = new Point3D(5.753494342409687, -0.1684521815005659, 0.2278999120441889);
      Quaternion orientation70 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size70 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box70 = new Box3D(position70, orientation70, size70);
      terrain.addRotatableBox(box70, appearance);

      Point3D position71 = new Point3D(5.648721454782096, 0.12274996284824707, 0.22909123665228792);
      Quaternion orientation71 = new Quaternion(0.0, 0.0, -0.7107650931702626,0.7034294437473249);
      Vector3D size71 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box71 = new Box3D(position71, orientation71, size71);
      terrain.addRotatableBox(box71, appearance);

      Point3D position72 = new Point3D(5.840403719567024, 0.12495103853557084, 0.2307081992117488);
      Quaternion orientation72 = new Quaternion(0.0, 0.0, 0.705341788004333,0.7088673797649667);
      Vector3D size72 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box72 = new Box3D(position72, orientation72, size72);
      terrain.addRotatableBox(box72, appearance);

      Point3D position73 = new Point3D(5.7577226491388185, 0.4116362919569683, 0.23430691862402694);
      Quaternion orientation73 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size73 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box73 = new Box3D(position73, orientation73, size73);
      terrain.addRotatableBox(box73, appearance);

      Point3D position74 = new Point3D(5.764448795159363, 0.6034155096950473, 0.24155583755511856);
      Quaternion orientation74 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size74 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box74 = new Box3D(position74, orientation74, size74);
      terrain.addRotatableBox(box74, appearance);

      Point3D position75 = new Point3D(6.2352107198329065, -0.25880611548136095, 0.22412120969808713);
      Quaternion orientation75 = new Quaternion(0.0, 0.0, 0.7050698828375693,0.7091378288565752);
      Vector3D size75 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box75 = new Box3D(position75, orientation75, size75);
      terrain.addRotatableBox(box75, appearance);

      Point3D position76 = new Point3D(6.041424771298938, -0.2617975766406595, 0.23217475146219316);
      Quaternion orientation76 = new Quaternion(0.0, 0.0, -0.7079095834255491,0.7063030664624527);
      Vector3D size76 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box76 = new Box3D(position76, orientation76, size76);
      terrain.addRotatableBox(box76, appearance);

      Point3D position77 = new Point3D(6.124353907177331, 0.21270454422081966, 0.23404920128215498);
      Quaternion orientation77 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size77 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box77 = new Box3D(position77, orientation77, size77);
      terrain.addRotatableBox(box77, appearance);

      Point3D position78 = new Point3D(6.421349657210157, -0.2658011220542116, 0.2311280740993358);
      Quaternion orientation78 = new Quaternion(0.0, 0.0, 0.7008708717943496,0.7132881753332446);
      Vector3D size78 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box78 = new Box3D(position78, orientation78, size78);
      terrain.addRotatableBox(box78, appearance);

      Point3D position79 = new Point3D(6.2386546198104, 0.5019196938501862, 0.24330105462308782);
      Quaternion orientation79 = new Quaternion(0.0, 0.0, 0.7008772020274565,0.7132819552450227);
      Vector3D size79 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box79 = new Box3D(position79, orientation79, size79);
      terrain.addRotatableBox(box79, appearance);

      Point3D position80 = new Point3D(6.051805117872733, 0.5075380042699142, 0.2462076993245688);
      Quaternion orientation80 = new Quaternion(-0.003546939645528666, -0.003543267978614838, 0.7074639704526886,0.7067316286841134);
      Vector3D size80 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box80 = new Box3D(position80, orientation80, size80);
      terrain.addRotatableBox(box80, appearance);

      Point3D position81 = new Point3D(6.122908361470702, 0.020253416476653883, 0.2312384964958033);
      Quaternion orientation81 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size81 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box81 = new Box3D(position81, orientation81, size81);
      terrain.addRotatableBox(box81, appearance);

      Point3D position82 = new Point3D(6.409268200039842, 0.11934128728302902, 0.23604260025687943);
      Quaternion orientation82 = new Quaternion(0.0, 0.0, 0.7016890425161328,0.7124833244454162);
      Vector3D size82 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box82 = new Box3D(position82, orientation82, size82);
      terrain.addRotatableBox(box82, appearance);

      Point3D position83 = new Point3D(6.429966481857306, 0.500996096184173, 0.24551820736884225);
      Quaternion orientation83 = new Quaternion(0.0, 0.0, 0.7027244324751094,0.7114621367314882);
      Vector3D size83 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box83 = new Box3D(position83, orientation83, size83);
      terrain.addRotatableBox(box83, appearance);

      Point3D position84 = new Point3D(6.699741918264115, -0.3631569365460855, 0.21492116687566903);
      Quaternion orientation84 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size84 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box84 = new Box3D(position84, orientation84, size84);
      terrain.addRotatableBox(box84, appearance);

      Point3D position85 = new Point3D(6.698630441512664, -0.17503310876007605, 0.21724682814725804);
      Quaternion orientation85 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size85 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box85 = new Box3D(position85, orientation85, size85);
      terrain.addRotatableBox(box85, appearance);

      Point3D position86 = new Point3D(6.797501566111999, 0.11234335851675088, 0.2166656716058247);
      Quaternion orientation86 = new Quaternion(0.0, 0.0, 0.6984451166535963,0.7156636214191308);
      Vector3D size86 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box86 = new Box3D(position86, orientation86, size86);
      terrain.addRotatableBox(box86, appearance);

      Point3D position87 = new Point3D(6.61320792222252, 0.11105183808493378, 0.21603636247814803);
      Quaternion orientation87 = new Quaternion(-0.006736059144041472, 0.007390816837914501, 0.7020028580344309,0.7121039170269788);
      Vector3D size87 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box87 = new Box3D(position87, orientation87, size87);
      terrain.addRotatableBox(box87, appearance);

      Point3D position88 = new Point3D(6.7127304242037615, 0.4038501715863721, 0.21934622785676056);
      Quaternion orientation88 = new Quaternion(0.0, 0.0, -0.003814431622533768,0.9999927250292361);
      Vector3D size88 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box88 = new Box3D(position88, orientation88, size88);
      terrain.addRotatableBox(box88, appearance);

      Point3D position89 = new Point3D(6.718241475052683, 0.5961877211864752, 0.214411381592355);
      Quaternion orientation89 = new Quaternion(0.0, 0.0, -0.009312552251322236,0.9999566372451187);
      Vector3D size89 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box89 = new Box3D(position89, orientation89, size89);
      terrain.addRotatableBox(box89, appearance);

      Point3D position90 = new Point3D(6.983093459558476, -0.2800509790382789, 0.21557870456215963);
      Quaternion orientation90 = new Quaternion(0.0, 0.0, 0.703035834272026,0.7111544246712078);
      Vector3D size90 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box90 = new Box3D(position90, orientation90, size90);
      terrain.addRotatableBox(box90, appearance);

      Point3D position91 = new Point3D(6.988683516017568, 0.11099979053069783, 0.21687826978610028);
      Quaternion orientation91 = new Quaternion(0.0, 0.0, 0.6996747373431306,0.7144615188544595);
      Vector3D size91 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box91 = new Box3D(position91, orientation91, size91);
      terrain.addRotatableBox(box91, appearance);

      Point3D position92 = new Point3D(7.00489079733141, 0.5035039836470866, 0.2167604197386731);
      Quaternion orientation92 = new Quaternion(0.0, 0.0, 0.7019672015374098,0.7122092725917974);
      Vector3D size92 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box92 = new Box3D(position92, orientation92, size92);
      terrain.addRotatableBox(box92, appearance);

      Point3D position93 = new Point3D(7.345613673095441, -0.3621818158794301, 0.18851614172025807);
      Quaternion orientation93 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size93 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box93 = new Box3D(position93, orientation93, size93);
      terrain.addRotatableBox(box93, appearance);

      Point3D position94 = new Point3D(7.346728260451079, -0.17617424269482315, 0.19148708763994512);
      Quaternion orientation94 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size94 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box94 = new Box3D(position94, orientation94, size94);
      terrain.addRotatableBox(box94, appearance);

      Point3D position95 = new Point3D(7.256713781322195, 0.11746933621526329, 0.19034824545194998);
      Quaternion orientation95 = new Quaternion(0.0, 0.0, -0.7109847545862545,0.7032074222773266);
      Vector3D size95 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box95 = new Box3D(position95, orientation95, size95);
      terrain.addRotatableBox(box95, appearance);

      Point3D position96 = new Point3D(7.436520392537069, 0.1116348793188018, 0.18978383802880525);
      Quaternion orientation96 = new Quaternion(0.0, 0.0, 0.7079054853446145,0.7063071738408194);
      Vector3D size96 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box96 = new Box3D(position96, orientation96, size96);
      terrain.addRotatableBox(box96, appearance);

      Point3D position97 = new Point3D(7.356717455086338, 0.40091839642545996, 0.1894719544174172);
      Quaternion orientation97 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size97 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box97 = new Box3D(position97, orientation97, size97);
      terrain.addRotatableBox(box97, appearance);

      Point3D position98 = new Point3D(7.3566302205035035, 0.587809313874047, 0.1878985545160597);
      Quaternion orientation98 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size98 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box98 = new Box3D(position98, orientation98, size98);
      terrain.addRotatableBox(box98, appearance);

      Point3D position99 = new Point3D(7.770134742471376, -0.40055264320453654, 0.047256945955076055);
      Quaternion orientation99 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size99 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box99 = new Box3D(position99, orientation99, size99);
      terrain.addRotatableBox(box99, appearance);

      Point3D position100 = new Point3D(7.770217973789706, -0.2137607588909391, 0.04430979308496823);
      Quaternion orientation100 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size100 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box100 = new Box3D(position100, orientation100, size100);
      terrain.addRotatableBox(box100, appearance);

      Point3D position101 = new Point3D(7.869196761927994, 0.07594085472191646, 0.04358384009610045);
      Quaternion orientation101 = new Quaternion(0.0, 0.0, 0.7034682501294439,0.7107266852031212);
      Vector3D size101 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box101 = new Box3D(position101, orientation101, size101);
      terrain.addRotatableBox(box101, appearance);

      Point3D position102 = new Point3D(7.682970194739092, 0.07705257011165333, 0.047271829008987146);
      Quaternion orientation102 = new Quaternion(0.0, 0.0, 0.7034450413479779,0.7107496562102162);
      Vector3D size102 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box102 = new Box3D(position102, orientation102, size102);
      terrain.addRotatableBox(box102, appearance);

      Point3D position103 = new Point3D(7.779256902384008, 0.3659919336394692, 0.04432955446553308);
      Quaternion orientation103 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size103 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box103 = new Box3D(position103, orientation103, size103);
      terrain.addRotatableBox(box103, appearance);

      Point3D position104 = new Point3D(7.785783227158354, 0.5557054734046523, 0.045068700704124554);
      Quaternion orientation104 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size104 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box104 = new Box3D(position104, orientation104, size104);
      terrain.addRotatableBox(box104, appearance);

      Point3D position105 = new Point3D(8.252153182891886, -0.3056293119640772, 0.04285320203865197);
      Quaternion orientation105 = new Quaternion(0.0, 0.0, 0.7041815061508303,0.7100200042218157);
      Vector3D size105 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box105 = new Box3D(position105, orientation105, size105);
      terrain.addRotatableBox(box105, appearance);

      Point3D position106 = new Point3D(8.064675866921874, -0.2987549941396834, 0.046016306708178786);
      Quaternion orientation106 = new Quaternion(0.0, 0.0, 0.7060501535261506,0.7081618322853174);
      Vector3D size106 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box106 = new Box3D(position106, orientation106, size106);
      terrain.addRotatableBox(box106, appearance);

      Point3D position107 = new Point3D(8.152449598918341, -0.020698347797655257, 0.047082419865977894);
      Quaternion orientation107 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size107 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box107 = new Box3D(position107, orientation107, size107);
      terrain.addRotatableBox(box107, appearance);

      Point3D position108 = new Point3D(8.15561274022191, 0.16606239445418075, 0.04851321195122305);
      Quaternion orientation108 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size108 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box108 = new Box3D(position108, orientation108, size108);
      terrain.addRotatableBox(box108, appearance);

      Point3D position109 = new Point3D(8.06995087947249, 0.4533393922315608, 0.04470427328368063);
      Quaternion orientation109 = new Quaternion(0.0, 0.0, 0.7089663975812708,0.7052422612837637);
      Vector3D size109 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box109 = new Box3D(position109, orientation109, size109);
      terrain.addRotatableBox(box109, appearance);

      Point3D position110 = new Point3D(8.256712637978552, 0.45548448733570646, 0.0465446546756541);
      Quaternion orientation110 = new Quaternion(0.0, 0.0, 0.7057265405863627,0.708484332862771);
      Vector3D size110 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box110 = new Box3D(position110, orientation110, size110);
      terrain.addRotatableBox(box110, appearance);

      Point3D position111 = new Point3D(8.534695414460673, -0.40215227023449407, 0.0436624434311707);
      Quaternion orientation111 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size111 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box111 = new Box3D(position111, orientation111, size111);
      terrain.addRotatableBox(box111, appearance);

      Point3D position112 = new Point3D(8.540900973445865, -0.21132852297590346, 0.04400585133181643);
      Quaternion orientation112 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size112 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box112 = new Box3D(position112, orientation112, size112);
      terrain.addRotatableBox(box112, appearance);

      Point3D position113 = new Point3D(8.446166935149789, 0.07613432146555155, 0.04558588339207196);
      Quaternion orientation113 = new Quaternion(0.0, 0.0, 0.7016853567916598,0.7124869543115587);
      Vector3D size113 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box113 = new Box3D(position113, orientation113, size113);
      terrain.addRotatableBox(box113, appearance);

      Point3D position114 = new Point3D(8.629100011417771, 0.078953161543596, 0.0401527710286485);
      Quaternion orientation114 = new Quaternion(0.0, 0.0, 0.7096844738128036,0.7045196573758916);
      Vector3D size114 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box114 = new Box3D(position114, orientation114, size114);
      terrain.addRotatableBox(box114, appearance);

      Point3D position115 = new Point3D(8.544205652116847, 0.363513099370666, 0.04861356986695628);
      Quaternion orientation115 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size115 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box115 = new Box3D(position115, orientation115, size115);
      terrain.addRotatableBox(box115, appearance);

      Point3D position116 = new Point3D(8.5399389891589, 0.555300276983753, 0.04766115185873509);
      Quaternion orientation116 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size116 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box116 = new Box3D(position116, orientation116, size116);
      terrain.addRotatableBox(box116, appearance);
   }

   private static void addLookAndStepHard(CombinedTerrainObject3D terrain)
   {
      AppearanceDefinition appearance = YoAppearance.DarkGray();

      Point3D position0 = new Point3D(2.9991922858880433, 0.17738506103097837, 0.07749255480474931);
      Quaternion orientation0 = new Quaternion(3.591011281523848E-4, -4.404093410555292E-4, -0.7140274385984735,0.7001174858654047);
      Vector3D size0 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box0 = new Box3D(position0, orientation0, size0);
      terrain.addRotatableBox(box0, appearance);

      Point3D position1 = new Point3D(4.010313695756378, 0.1616056331805633, 0.07664394760851354);
      Quaternion orientation1 = new Quaternion(0.0010814627453136847, 0.0011373925186028025, -0.7128015132552289,0.7013640563057426);
      Vector3D size1 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box1 = new Box3D(position1, orientation1, size1);
      terrain.addRotatableBox(box1, appearance);

      Point3D position2 = new Point3D(1.538532083708091, 0.6741701950310802, 0.05131964350890132);
      Quaternion orientation2 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size2 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box2 = new Box3D(position2, orientation2, size2);
      terrain.addRotatableBox(box2, appearance);

      Point3D position3 = new Point3D(1.5377122418466396, 0.4871995187212327, 0.0453699739307727);
      Quaternion orientation3 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size3 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box3 = new Box3D(position3, orientation3, size3);
      terrain.addRotatableBox(box3, appearance);

      Point3D position4 = new Point3D(1.8318418085847752, 0.5832432291386377, 0.05026246658367628);
      Quaternion orientation4 = new Quaternion(0.0, 0.0, -0.7032798171527006,0.7109131443331625);
      Vector3D size4 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box4 = new Box3D(position4, orientation4, size4);
      terrain.addRotatableBox(box4, appearance);

      Point3D position5 = new Point3D(2.0226812492006916, 0.5816220959595908, 0.058535600491900285);
      Quaternion orientation5 = new Quaternion(0.0, 0.0, -0.7033244049950813,0.7108690324794821);
      Vector3D size5 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box5 = new Box3D(position5, orientation5, size5);
      terrain.addRotatableBox(box5, appearance);

      Point3D position6 = new Point3D(1.9258075527472558, 0.2916058787804267, 0.04914770991489481);
      Quaternion orientation6 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size6 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box6 = new Box3D(position6, orientation6, size6);
      terrain.addRotatableBox(box6, appearance);

      Point3D position7 = new Point3D(1.9221267562656839, 0.0971054615729663, 0.048374686104427383);
      Quaternion orientation7 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size7 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box7 = new Box3D(position7, orientation7, size7);
      terrain.addRotatableBox(box7, appearance);

      Point3D position8 = new Point3D(1.6391139250203717, 0.20003028808817114, 0.03909051303520233);
      Quaternion orientation8 = new Quaternion(4.2221998162823653E-4, -0.0021771612644779165, -0.7083346233855763,0.7058733193798512);
      Vector3D size8 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box8 = new Box3D(position8, orientation8, size8);
      terrain.addRotatableBox(box8, appearance);

      Point3D position9 = new Point3D(1.4508258759747759, 0.19187378577710404, 0.03772504674415519);
      Quaternion orientation9 = new Quaternion(0.0, 0.0, 0.7038510074598656,0.7103476327107258);
      Vector3D size9 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box9 = new Box3D(position9, orientation9, size9);
      terrain.addRotatableBox(box9, appearance);

      Point3D position10 = new Point3D(1.544254053576254, -0.09368499925145309, 0.032965432002729965);
      Quaternion orientation10 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size10 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box10 = new Box3D(position10, orientation10, size10);
      terrain.addRotatableBox(box10, appearance);

      Point3D position11 = new Point3D(1.5421604510537665, -0.2847681953194701, 0.03169062655032244);
      Quaternion orientation11 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size11 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box11 = new Box3D(position11, orientation11, size11);
      terrain.addRotatableBox(box11, appearance);

      Point3D position12 = new Point3D(1.8302064340875628, -0.18772505668791095, 0.03233411894842265);
      Quaternion orientation12 = new Quaternion(0.0, 0.0, -0.7041659007794283,0.7100354809300001);
      Vector3D size12 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box12 = new Box3D(position12, orientation12, size12);
      terrain.addRotatableBox(box12, appearance);

      Point3D position13 = new Point3D(2.020076111592974, -0.19450183861517623, 0.032951829300953354);
      Quaternion orientation13 = new Quaternion(0.0, 0.0, 0.7094730567086567,0.7047325604826807);
      Vector3D size13 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box13 = new Box3D(position13, orientation13, size13);
      terrain.addRotatableBox(box13, appearance);

      Point3D position14 = new Point3D(2.3017726687557736, 0.6659211869311393, 0.050732948560174566);
      Quaternion orientation14 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size14 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box14 = new Box3D(position14, orientation14, size14);
      terrain.addRotatableBox(box14, appearance);

      Point3D position15 = new Point3D(2.3045613478518816, 0.4744061119031097, 0.04678489901068887);
      Quaternion orientation15 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size15 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box15 = new Box3D(position15, orientation15, size15);
      terrain.addRotatableBox(box15, appearance);

      Point3D position16 = new Point3D(2.214159178501207, 0.19092562262741894, 0.04546863118006792);
      Quaternion orientation16 = new Quaternion(0.0, 0.0, -0.710045270209128,0.7041560297644597);
      Vector3D size16 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box16 = new Box3D(position16, orientation16, size16);
      terrain.addRotatableBox(box16, appearance);

      Point3D position17 = new Point3D(2.3882625449800243, 0.1854857053173351, 0.04358061639227621);
      Quaternion orientation17 = new Quaternion(0.0, 0.0, 0.7032305077721192,0.7109619208780226);
      Vector3D size17 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box17 = new Box3D(position17, orientation17, size17);
      terrain.addRotatableBox(box17, appearance);

      Point3D position18 = new Point3D(2.305334951328886, -0.09669125069873875, 0.04026317060808564);
      Quaternion orientation18 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size18 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box18 = new Box3D(position18, orientation18, size18);
      terrain.addRotatableBox(box18, appearance);

      Point3D position19 = new Point3D(2.3035618801058884, -0.28517226345878166, 0.043838376674458175);
      Quaternion orientation19 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size19 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box19 = new Box3D(position19, orientation19, size19);
      terrain.addRotatableBox(box19, appearance);

      Point3D position20 = new Point3D(5.022827545101327, 0.14359272115178368, 0.06929944023803199);
      Quaternion orientation20 = new Quaternion(0.0, 0.0, -0.7146326573404546,0.6994999392870028);
      Vector3D size20 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box20 = new Box3D(position20, orientation20, size20);
      terrain.addRotatableBox(box20, appearance);

      Point3D position21 = new Point3D(6.0368497952581395, 0.10384214494564183, 0.06172802598521781);
      Quaternion orientation21 = new Quaternion(0.005613854283208643, 0.0055820130552961455, -0.7143726400604131,0.6997206992103728);
      Vector3D size21 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box21 = new Box3D(position21, orientation21, size21);
      terrain.addRotatableBox(box21, appearance);

      Point3D position22 = new Point3D(7.050523277840206, 0.08289463608396092, 0.06864253875907342);
      Quaternion orientation22 = new Quaternion(0.0, 0.0, 0.7017961773791698,0.7123777968297335);
      Vector3D size22 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box22 = new Box3D(position22, orientation22, size22);
      terrain.addRotatableBox(box22, appearance);

      Point3D position23 = new Point3D(5.023846234292196, 0.13613703474961703, 0.22602295943394107);
      Quaternion orientation23 = new Quaternion(0.0, 0.0, 0.7009087265528491,0.71325097759629);
      Vector3D size23 = new Vector3D(1.21, 1.013, 0.155);
      Box3D box23 = new Box3D(position23, orientation23, size23);
      terrain.addRotatableBox(box23, appearance);

      Point3D position24 = new Point3D(2.6900015009577163, 0.6436366016069017, 0.19963009876119422);
      Quaternion orientation24 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size24 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box24 = new Box3D(position24, orientation24, size24);
      terrain.addRotatableBox(box24, appearance);

      Point3D position25 = new Point3D(2.6890564650088904, 0.4543195036924751, 0.1998904919894186);
      Quaternion orientation25 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size25 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box25 = new Box3D(position25, orientation25, size25);
      terrain.addRotatableBox(box25, appearance);

      Point3D position26 = new Point3D(2.7797616190377004, 0.16496187008841534, 0.24403572659127876);
      Quaternion orientation26 = new Quaternion(0.09461018209558089, 0.09407245118837755, -0.7027793486796466,0.6987849987232575);
      Vector3D size26 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box26 = new Box3D(position26, orientation26, size26);
      terrain.addRotatableBox(box26, appearance);

      Point3D position27 = new Point3D(2.593463551704851, 0.1621908804922451, 0.24443600486584452);
      Quaternion orientation27 = new Quaternion(0.09234953713579733, -0.09290494477039353, 0.698908799916006,0.7031121700183992);
      Vector3D size27 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box27 = new Box3D(position27, orientation27, size27);
      terrain.addRotatableBox(box27, appearance);

      Point3D position28 = new Point3D(2.684024192233144, -0.13061899475743363, 0.19603696340587146);
      Quaternion orientation28 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size28 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box28 = new Box3D(position28, orientation28, size28);
      terrain.addRotatableBox(box28, appearance);

      Point3D position29 = new Point3D(2.6860150703200567, -0.320451962389396, 0.19828815283388773);
      Quaternion orientation29 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size29 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box29 = new Box3D(position29, orientation29, size29);
      terrain.addRotatableBox(box29, appearance);

      Point3D position30 = new Point3D(3.1711383025454465, 0.5462180514407013, 0.28166428275520083);
      Quaternion orientation30 = new Quaternion(0.10793831101703633, 0.10894156349569274, -0.6955023910392171,0.7019668658965574);
      Vector3D size30 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box30 = new Box3D(position30, orientation30, size30);
      terrain.addRotatableBox(box30, appearance);

      Point3D position31 = new Point3D(2.9818424101161205, 0.5424769634680712, 0.28083800074621);
      Quaternion orientation31 = new Quaternion(0.11223315066187517, -0.11138798116147307, 0.7008445880063211,0.6955668918281909);
      Vector3D size31 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box31 = new Box3D(position31, orientation31, size31);
      terrain.addRotatableBox(box31, appearance);

      Point3D position32 = new Point3D(3.0756668786094545, 0.2573398848240338, 0.22430098450189007);
      Quaternion orientation32 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size32 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box32 = new Box3D(position32, orientation32, size32);
      terrain.addRotatableBox(box32, appearance);

      Point3D position33 = new Point3D(3.0722024139657997, 0.06783466183720784, 0.2251948202315323);
      Quaternion orientation33 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size33 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box33 = new Box3D(position33, orientation33, size33);
      terrain.addRotatableBox(box33, appearance);

      Point3D position34 = new Point3D(3.165664362999164, -0.22522204408849275, 0.27735751751742505);
      Quaternion orientation34 = new Quaternion(0.09437843324707647, 0.09404280191325493, -0.7020498469308203,0.6995531968124677);
      Vector3D size34 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box34 = new Box3D(position34, orientation34, size34);
      terrain.addRotatableBox(box34, appearance);

      Point3D position35 = new Point3D(2.974840053678399, -0.22519875771755868, 0.2788927721576176);
      Quaternion orientation35 = new Quaternion(0.09994070761295208, -0.09964072755297639, 0.7010812345776699,0.6989768829493477);
      Vector3D size35 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box35 = new Box3D(position35, orientation35, size35);
      terrain.addRotatableBox(box35, appearance);

      Point3D position36 = new Point3D(3.359147869089035, 0.550879534175364, 0.22486106332834727);
      Quaternion orientation36 = new Quaternion(0.0, 0.0, -0.7071194347376097,0.7070941274090483);
      Vector3D size36 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box36 = new Box3D(position36, orientation36, size36);
      terrain.addRotatableBox(box36, appearance);

      Point3D position37 = new Point3D(3.354472458529534, 0.15783397615504235, 0.22395977136362658);
      Quaternion orientation37 = new Quaternion(0.0, 0.0, -0.7058785319428242,0.7083329006492944);
      Vector3D size37 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box37 = new Box3D(position37, orientation37, size37);
      terrain.addRotatableBox(box37, appearance);

      Point3D position38 = new Point3D(3.350402623341776, -0.22691663331160317, 0.22640085739147484);
      Quaternion orientation38 = new Quaternion(0.0, 0.0, 0.7062242020231959,0.7079882601263245);
      Vector3D size38 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box38 = new Box3D(position38, orientation38, size38);
      terrain.addRotatableBox(box38, appearance);

      Point3D position39 = new Point3D(3.68866762247556, 0.6236265834842826, 0.3010889371484357);
      Quaternion orientation39 = new Quaternion(-5.839236425510448E-34, 0.14469060323807462, 8.859744206110364E-18,0.9894769473487506);
      Vector3D size39 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box39 = new Box3D(position39, orientation39, size39);
      terrain.addRotatableBox(box39, appearance);

      Point3D position40 = new Point3D(3.7706107394082076, 0.15186184665978525, 0.2940640116728258);
      Quaternion orientation40 = new Quaternion(-0.08864451472761907, -0.08908549481655237, -0.6997578049577696,0.7032388918587189);
      Vector3D size40 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box40 = new Box3D(position40, orientation40, size40);
      terrain.addRotatableBox(box40, appearance);

      Point3D position41 = new Point3D(3.68965927337154, 0.4388934247654853, 0.30028645012494665);
      Quaternion orientation41 = new Quaternion(1.9454724300576844E-34, 0.14138358070604393, 8.657247478182429E-18,0.989954889430189);
      Vector3D size41 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box41 = new Box3D(position41, orientation41, size41);
      terrain.addRotatableBox(box41, appearance);

      Point3D position42 = new Point3D(3.588806285529547, 0.15484013654183854, 0.2930951718708826);
      Quaternion orientation42 = new Quaternion(-0.08427871374180397, 0.09588259957957021, 0.7076885857849924,0.6948960289550722);
      Vector3D size42 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box42 = new Box3D(position42, orientation42, size42);
      terrain.addRotatableBox(box42, appearance);

      Point3D position43 = new Point3D(3.6964906211913506, -0.14386449366407408, 0.24588881335187482);
      Quaternion orientation43 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size43 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box43 = new Box3D(position43, orientation43, size43);
      terrain.addRotatableBox(box43, appearance);

      Point3D position44 = new Point3D(3.6860421913290207, -0.33336370841482466, 0.24832633958306163);
      Quaternion orientation44 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size44 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box44 = new Box3D(position44, orientation44, size44);
      terrain.addRotatableBox(box44, appearance);

      Point3D position45 = new Point3D(3.9745492483528864, -0.23633221329182877, 0.3000158195677145);
      Quaternion orientation45 = new Quaternion(0.09976532226249955, -0.09967911277827533, 0.7003421510403348,0.6997369694576615);
      Vector3D size45 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box45 = new Box3D(position45, orientation45, size45);
      terrain.addRotatableBox(box45, appearance);

      Point3D position46 = new Point3D(4.1570571372942995, 0.5351500725177174, 0.3009665879051755);
      Quaternion orientation46 = new Quaternion(0.0, 0.0, -0.7031379195773428,0.7110534902891948);
      Vector3D size46 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box46 = new Box3D(position46, orientation46, size46);
      terrain.addRotatableBox(box46, appearance);

      Point3D position47 = new Point3D(3.976449906945254, 0.5313406371863271, 0.30163369997832445);
      Quaternion orientation47 = new Quaternion(0.0, 0.0, 0.7053477339779695,0.7088614633143374);
      Vector3D size47 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box47 = new Box3D(position47, orientation47, size47);
      terrain.addRotatableBox(box47, appearance);

      Point3D position48 = new Point3D(4.060087891099629, 0.2392964224398486, 0.24882388075031026);
      Quaternion orientation48 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size48 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box48 = new Box3D(position48, orientation48, size48);
      terrain.addRotatableBox(box48, appearance);

      Point3D position49 = new Point3D(4.063602710036821, 0.052570508322739594, 0.2470906830235654);
      Quaternion orientation49 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size49 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box49 = new Box3D(position49, orientation49, size49);
      terrain.addRotatableBox(box49, appearance);

      Point3D position50 = new Point3D(4.147655172749554, -0.2365996880994438, 0.2989374948751531);
      Quaternion orientation50 = new Quaternion(0.09713879505240461, 0.09719288147916494, -0.7002040662118151,0.7005939365609611);
      Vector3D size50 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box50 = new Box3D(position50, orientation50, size50);
      terrain.addRotatableBox(box50, appearance);

      Point3D position51 = new Point3D(4.343356717444665, -0.23511809278230003, 0.30128052384367776);
      Quaternion orientation51 = new Quaternion(0.0844889112032617, -0.11355828784903144, 0.6956075277336033,0.7043410441716627);
      Vector3D size51 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box51 = new Box3D(position51, orientation51, size51);
      terrain.addRotatableBox(box51, appearance);

      Point3D position52 = new Point3D(4.349272116834419, 0.15033640054496056, 0.23830329314841697);
      Quaternion orientation52 = new Quaternion(0.0, 0.0, -0.7115262234714076,0.7026595429598296);
      Vector3D size52 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box52 = new Box3D(position52, orientation52, size52);
      terrain.addRotatableBox(box52, appearance);

      Point3D position53 = new Point3D(4.346481880050672, 0.5267456748407474, 0.3000392880382346);
      Quaternion orientation53 = new Quaternion(0.0, 0.0, -0.7065132530735921,0.7076998115241875);
      Vector3D size53 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box53 = new Box3D(position53, orientation53, size53);
      terrain.addRotatableBox(box53, appearance);

      Point3D position54 = new Point3D(4.702380013634424, 0.6079336148133885, 0.3797638118070186);
      Quaternion orientation54 = new Quaternion(1.6449470919030792E-33, 0.09792926395956664, 5.9964379825470255E-18,0.9951933778217867);
      Vector3D size54 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box54 = new Box3D(position54, orientation54, size54);
      terrain.addRotatableBox(box54, appearance);

      Point3D position55 = new Point3D(4.703890280086529, 0.4155963177652986, 0.37962892936597686);
      Quaternion orientation55 = new Quaternion(1.9358735945371405E-34, 0.10122575159006256, 6.1982896338026795E-18,0.9948634816973769);
      Vector3D size55 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box55 = new Box3D(position55, orientation55, size55);
      terrain.addRotatableBox(box55, appearance);

      Point3D position56 = new Point3D(4.792888242703473, 0.12489420821974778, 0.3479451232275179);
      Quaternion orientation56 = new Quaternion(0.0, 0.0, -0.7035865871495094,0.7106095372166812);
      Vector3D size56 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box56 = new Box3D(position56, orientation56, size56);
      terrain.addRotatableBox(box56, appearance);

      Point3D position57 = new Point3D(4.610354981743812, 0.12911728605722894, 0.3475132664494318);
      Quaternion orientation57 = new Quaternion(0.0, 0.0, -0.7132505655401289,0.7009091458646308);
      Vector3D size57 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box57 = new Box3D(position57, orientation57, size57);
      terrain.addRotatableBox(box57, appearance);

      Point3D position58 = new Point3D(4.712970930242387, -0.1602010896506567, 0.3912550730752458);
      Quaternion orientation58 = new Quaternion(-3.880765412410759E-34, -0.12182502599465175, -7.459631407019642E-18,0.9925515921308083);
      Vector3D size58 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box58 = new Box3D(position58, orientation58, size58);
      terrain.addRotatableBox(box58, appearance);

      Point3D position59 = new Point3D(4.707914463288, -0.34860534277151983, 0.3891170604695318);
      Quaternion orientation59 = new Quaternion(2.3291328065487197E-33, -0.1241412431733023, -7.601458804717881E-18,0.9922645573350823);
      Vector3D size59 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box59 = new Box3D(position59, orientation59, size59);
      terrain.addRotatableBox(box59, appearance);

      Point3D position60 = new Point3D(5.1727760970934185, 0.5112340340750825, 0.3967895082089883);
      Quaternion orientation60 = new Quaternion(0.09472131621407967, 0.09590058699963736, -0.6963057302396933,0.7049746660009598);
      Vector3D size60 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box60 = new Box3D(position60, orientation60, size60);
      terrain.addRotatableBox(box60, appearance);

      Point3D position61 = new Point3D(4.986578998410523, 0.5045728830128698, 0.3944322980633405);
      Quaternion orientation61 = new Quaternion(0.09774852819681804, -0.09770925509640942, 0.7004613865448716,0.700179957342037);
      Vector3D size61 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box61 = new Box3D(position61, orientation61, size61);
      terrain.addRotatableBox(box61, appearance);

      Point3D position62 = new Point3D(5.181946498191193, 0.11593862123374693, 0.3935301841017318);
      Quaternion orientation62 = new Quaternion(-0.09454705714016172, 0.09446453379644658, 0.7010687973196257,0.7004568846585553);
      Vector3D size62 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box62 = new Box3D(position62, orientation62, size62);
      terrain.addRotatableBox(box62, appearance);

      Point3D position63 = new Point3D(4.999908472643262, 0.12271227322859121, 0.3936477908269693);
      Quaternion orientation63 = new Quaternion(-0.0943438948778358, -0.0947342877428095, -0.6993100325946019,0.7022037614095209);
      Vector3D size63 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box63 = new Box3D(position63, orientation63, size63);
      terrain.addRotatableBox(box63, appearance);

      Point3D position64 = new Point3D(5.181134977089534, -0.2648254795801891, 0.34319902751110637);
      Quaternion orientation64 = new Quaternion(0.0, 0.0, -0.7087044927674649,0.7055054513830564);
      Vector3D size64 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box64 = new Box3D(position64, orientation64, size64);
      terrain.addRotatableBox(box64, appearance);

      Point3D position65 = new Point3D(5.001168160091194, -0.26897331063196755, 0.3442518243348339);
      Quaternion orientation65 = new Quaternion(0.0, 0.0, 0.7085911489028207,0.7056192909045079);
      Vector3D size65 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box65 = new Box3D(position65, orientation65, size65);
      terrain.addRotatableBox(box65, appearance);

      Point3D position66 = new Point3D(5.361423516842161, -0.26268163874752193, 0.3467424072643232);
      Quaternion orientation66 = new Quaternion(0.0, 0.0, -0.702830138774019,0.7113577131309489);
      Vector3D size66 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box66 = new Box3D(position66, orientation66, size66);
      terrain.addRotatableBox(box66, appearance);

      Point3D position67 = new Point3D(5.368768670021752, 0.1171590670986501, 0.38910279983955076);
      Quaternion orientation67 = new Quaternion(-0.09458307226381174, -0.09416431082175267, -0.7023337283279424,0.6992241836933095);
      Vector3D size67 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box67 = new Box3D(position67, orientation67, size67);
      terrain.addRotatableBox(box67, appearance);

      Point3D position68 = new Point3D(5.359430458701423, 0.5074604944131372, 0.3951426086328146);
      Quaternion orientation68 = new Quaternion(0.09529196339253196, -0.09533170427575388, 0.7005076469963307,0.7007997890784077);
      Vector3D size68 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box68 = new Box3D(position68, orientation68, size68);
      terrain.addRotatableBox(box68, appearance);

      Point3D position69 = new Point3D(5.75258973537161, -0.3504907819884919, 0.227418511493009);
      Quaternion orientation69 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size69 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box69 = new Box3D(position69, orientation69, size69);
      terrain.addRotatableBox(box69, appearance);

      Point3D position70 = new Point3D(5.753494342409687, -0.1684521815005659, 0.2278999120441889);
      Quaternion orientation70 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size70 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box70 = new Box3D(position70, orientation70, size70);
      terrain.addRotatableBox(box70, appearance);

      Point3D position71 = new Point3D(5.648721454782096, 0.12274996284824707, 0.27732476658080424);
      Quaternion orientation71 = new Quaternion(0.0, 0.0, -0.7107650931702618,0.7034294437473235);
      Vector3D size71 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box71 = new Box3D(position71, orientation71, size71);
      terrain.addRotatableBox(box71, appearance);

      Point3D position72 = new Point3D(5.840403719567024, 0.12495103853557084, 0.27650382371337645);
      Quaternion orientation72 = new Quaternion(0.0, 0.0, 0.7053417880043341,0.7088673797649665);
      Vector3D size72 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box72 = new Box3D(position72, orientation72, size72);
      terrain.addRotatableBox(box72, appearance);

      Point3D position73 = new Point3D(5.7577226491388185, 0.4116362919569683, 0.28389741291306736);
      Quaternion orientation73 = new Quaternion(0.0, 0.11262021465784755, 6.89599927000101E-18,0.993638106782555);
      Vector3D size73 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box73 = new Box3D(position73, orientation73, size73);
      terrain.addRotatableBox(box73, appearance);

      Point3D position74 = new Point3D(5.764448795159363, 0.6034155096950473, 0.28467265927325774);
      Quaternion orientation74 = new Quaternion(-1.0658594972039655E-33, 0.11109613212788319, 6.802676130403094E-18,0.9938096645868483);
      Vector3D size74 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box74 = new Box3D(position74, orientation74, size74);
      terrain.addRotatableBox(box74, appearance);

      Point3D position75 = new Point3D(6.2352107198329065, -0.25880611548136095, 0.2684214567166878);
      Quaternion orientation75 = new Quaternion(-0.09099327976522753, 0.09151827134292549, 0.6991736284516058,0.7032075556518044);
      Vector3D size75 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box75 = new Box3D(position75, orientation75, size75);
      terrain.addRotatableBox(box75, appearance);

      Point3D position76 = new Point3D(6.041424771298938, -0.2617975766406595, 0.2701155633865791);
      Quaternion orientation76 = new Quaternion(-0.09049938070702158, -0.09029400308019075, -0.7021010186557202,0.7005076835424435);
      Vector3D size76 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box76 = new Box3D(position76, orientation76, size76);
      terrain.addRotatableBox(box76, appearance);

      Point3D position77 = new Point3D(6.124353907177331, 0.21270454422081966, 0.23404920128215498);
      Quaternion orientation77 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size77 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box77 = new Box3D(position77, orientation77, size77);
      terrain.addRotatableBox(box77, appearance);

      Point3D position78 = new Point3D(6.421349657210157, -0.2658011220542116, 0.26733468034992436);
      Quaternion orientation78 = new Quaternion(-0.09224861237772161, 0.09388297766102086, 0.6947734684371486,0.7070827445042763);
      Vector3D size78 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box78 = new Box3D(position78, orientation78, size78);
      terrain.addRotatableBox(box78, appearance);

      Point3D position79 = new Point3D(6.2386546198104, 0.5019196938501862, 0.2618318096377476);
      Quaternion orientation79 = new Quaternion(-0.0924918376852269, 0.09412884116858322, 0.6947475169322426,0.7070437814862335);
      Vector3D size79 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box79 = new Box3D(position79, orientation79, size79);
      terrain.addRotatableBox(box79, appearance);

      Point3D position80 = new Point3D(6.051805117872733, 0.5075380042699142, 0.26005353871268083);
      Quaternion orientation80 = new Quaternion(-0.09694408237307763, 0.08981925984154682, 0.7007993258860914,0.7010097362319289);
      Vector3D size80 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box80 = new Box3D(position80, orientation80, size80);
      terrain.addRotatableBox(box80, appearance);

      Point3D position81 = new Point3D(6.122908361470702, 0.020253416476653883, 0.2312384964958033);
      Quaternion orientation81 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size81 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box81 = new Box3D(position81, orientation81, size81);
      terrain.addRotatableBox(box81, appearance);

      Point3D position82 = new Point3D(6.409268200039842, 0.11934128728302902, 0.23604260025687943);
      Quaternion orientation82 = new Quaternion(0.0, 0.0, 0.7016890425161331,0.7124833244454164);
      Vector3D size82 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box82 = new Box3D(position82, orientation82, size82);
      terrain.addRotatableBox(box82, appearance);

      Point3D position83 = new Point3D(6.429966481857306, 0.500996096184173, 0.26300251335578917);
      Quaternion orientation83 = new Quaternion(-0.09163052675672367, 0.092769864463877, 0.6967248198275796,0.7053879246555514);
      Vector3D size83 = new Vector3D(0.393, 0.19, 0.192);
      Box3D box83 = new Box3D(position83, orientation83, size83);
      terrain.addRotatableBox(box83, appearance);

      Point3D position84 = new Point3D(6.699741918264115, -0.3631569365460855, 0.2550180515425791);
      Quaternion orientation84 = new Quaternion(1.7479183742545392E-33, 0.1288981738241679, 7.892736799485345E-18,0.9916578345300332);
      Vector3D size84 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box84 = new Box3D(position84, orientation84, size84);
      terrain.addRotatableBox(box84, appearance);

      Point3D position85 = new Point3D(6.698630441512664, -0.17503310876007605, 0.2526101274005956);
      Quaternion orientation85 = new Quaternion(-3.884427274751654E-34, 0.1292203005277809, 7.912461371310276E-18,0.9916159104872764);
      Vector3D size85 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box85 = new Box3D(position85, orientation85, size85);
      terrain.addRotatableBox(box85, appearance);

      Point3D position86 = new Point3D(6.797501566111999, 0.11234335851675088, 0.2633885198693278);
      Quaternion orientation86 = new Quaternion(0.09819637521077755, -0.10061717351576993, 0.691507811143679,0.708555293125695);
      Vector3D size86 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box86 = new Box3D(position86, orientation86, size86);
      terrain.addRotatableBox(box86, appearance);

      Point3D position87 = new Point3D(6.61320792222252, 0.11105183808493378, 0.2609523583831561);
      Quaternion orientation87 = new Quaternion(0.09905634457071809, -0.09993018618950954, 0.6950116745649737,0.7050961428818118);
      Vector3D size87 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box87 = new Box3D(position87, orientation87, size87);
      terrain.addRotatableBox(box87, appearance);

      Point3D position88 = new Point3D(6.7127304242037615, 0.4038501715863721, 0.21934622785676056);
      Quaternion orientation88 = new Quaternion(0.0, 0.0, -0.0038144316225337672,0.9999927250292359);
      Vector3D size88 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box88 = new Box3D(position88, orientation88, size88);
      terrain.addRotatableBox(box88, appearance);

      Point3D position89 = new Point3D(6.718241475052683, 0.5961877211864752, 0.214411381592355);
      Quaternion orientation89 = new Quaternion(0.0, 0.0, -0.009312552251322237,0.999956637245119);
      Vector3D size89 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box89 = new Box3D(position89, orientation89, size89);
      terrain.addRotatableBox(box89, appearance);

      Point3D position90 = new Point3D(6.983093459558476, -0.2800509790382789, 0.21557870456215963);
      Quaternion orientation90 = new Quaternion(0.0, 0.0, 0.7030358342720265,0.7111544246712073);
      Vector3D size90 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box90 = new Box3D(position90, orientation90, size90);
      terrain.addRotatableBox(box90, appearance);

      Point3D position91 = new Point3D(6.988683516017568, 0.11099979053069783, 0.25958187507798225);
      Quaternion orientation91 = new Quaternion(0.10036878834372606, -0.10248996160407609, 0.6924383325630104,0.7070721813889761);
      Vector3D size91 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box91 = new Box3D(position91, orientation91, size91);
      terrain.addRotatableBox(box91, appearance);

      Point3D position92 = new Point3D(7.00489079733141, 0.5035039836470866, 0.2167604197386731);
      Quaternion orientation92 = new Quaternion(0.0, 0.0, 0.7019672015374099,0.7122092725917976);
      Vector3D size92 = new Vector3D(0.393001, 0.188522, 0.141535);
      Box3D box92 = new Box3D(position92, orientation92, size92);
      terrain.addRotatableBox(box92, appearance);

      Point3D position93 = new Point3D(7.345613673095441, -0.3621818158794301, 0.18851614172025807);
      Quaternion orientation93 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size93 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box93 = new Box3D(position93, orientation93, size93);
      terrain.addRotatableBox(box93, appearance);

      Point3D position94 = new Point3D(7.346728260451079, -0.17617424269482315, 0.19148708763994512);
      Quaternion orientation94 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size94 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box94 = new Box3D(position94, orientation94, size94);
      terrain.addRotatableBox(box94, appearance);

      Point3D position95 = new Point3D(7.256713781322195, 0.11746933621526329, 0.19034824545194998);
      Quaternion orientation95 = new Quaternion(0.0, 0.0, -0.7109847545862554,0.7032074222773266);
      Vector3D size95 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box95 = new Box3D(position95, orientation95, size95);
      terrain.addRotatableBox(box95, appearance);

      Point3D position96 = new Point3D(7.436520392537069, 0.1116348793188018, 0.18978383802880525);
      Quaternion orientation96 = new Quaternion(0.0, 0.0, 0.7079054853446147,0.7063071738408194);
      Vector3D size96 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box96 = new Box3D(position96, orientation96, size96);
      terrain.addRotatableBox(box96, appearance);

      Point3D position97 = new Point3D(7.356717455086338, 0.40091839642545996, 0.22230115969485045);
      Quaternion orientation97 = new Quaternion(0.0, 0.11441875733241588, 7.006128246478086E-18,0.9934326086708176);
      Vector3D size97 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box97 = new Box3D(position97, orientation97, size97);
      terrain.addRotatableBox(box97, appearance);

      Point3D position98 = new Point3D(7.3566302205035035, 0.587809313874047, 0.22224355113151553);
      Quaternion orientation98 = new Quaternion(-1.5505289284134202E-33, 0.11216790604418489, 6.8683033552035814E-18,0.9936892677561043);
      Vector3D size98 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box98 = new Box3D(position98, orientation98, size98);
      terrain.addRotatableBox(box98, appearance);

      Point3D position99 = new Point3D(7.770134742471376, -0.40055264320453654, 0.047256945955076055);
      Quaternion orientation99 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size99 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box99 = new Box3D(position99, orientation99, size99);
      terrain.addRotatableBox(box99, appearance);

      Point3D position100 = new Point3D(7.770217973789706, -0.2137607588909391, 0.04430979308496823);
      Quaternion orientation100 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size100 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box100 = new Box3D(position100, orientation100, size100);
      terrain.addRotatableBox(box100, appearance);

      Point3D position101 = new Point3D(7.869196761927994, 0.07594085472191646, 0.04358384009610045);
      Quaternion orientation101 = new Quaternion(0.0, 0.0, 0.703468250129445,0.7107266852031212);
      Vector3D size101 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box101 = new Box3D(position101, orientation101, size101);
      terrain.addRotatableBox(box101, appearance);

      Point3D position102 = new Point3D(7.682970194739092, 0.07705257011165333, 0.047271829008987146);
      Quaternion orientation102 = new Quaternion(0.0, 0.0, 0.7034450413479783,0.7107496562102167);
      Vector3D size102 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box102 = new Box3D(position102, orientation102, size102);
      terrain.addRotatableBox(box102, appearance);

      Point3D position103 = new Point3D(7.779256902384008, 0.3659919336394692, 0.04432955446553308);
      Quaternion orientation103 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size103 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box103 = new Box3D(position103, orientation103, size103);
      terrain.addRotatableBox(box103, appearance);

      Point3D position104 = new Point3D(7.785783227158354, 0.5557054734046523, 0.045068700704124554);
      Quaternion orientation104 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size104 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box104 = new Box3D(position104, orientation104, size104);
      terrain.addRotatableBox(box104, appearance);

      Point3D position105 = new Point3D(8.252153182891886, -0.3056293119640772, 0.04285320203865197);
      Quaternion orientation105 = new Quaternion(0.0, 0.0, 0.704181506150831,0.7100200042218158);
      Vector3D size105 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box105 = new Box3D(position105, orientation105, size105);
      terrain.addRotatableBox(box105, appearance);

      Point3D position106 = new Point3D(8.064675866921874, -0.2987549941396834, 0.046016306708178786);
      Quaternion orientation106 = new Quaternion(0.0, 0.0, 0.7060501535261512,0.7081618322853177);
      Vector3D size106 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box106 = new Box3D(position106, orientation106, size106);
      terrain.addRotatableBox(box106, appearance);

      Point3D position107 = new Point3D(8.152449598918341, -0.020698347797655257, 0.047082419865977894);
      Quaternion orientation107 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size107 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box107 = new Box3D(position107, orientation107, size107);
      terrain.addRotatableBox(box107, appearance);

      Point3D position108 = new Point3D(8.15561274022191, 0.16606239445418075, 0.04851321195122305);
      Quaternion orientation108 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size108 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box108 = new Box3D(position108, orientation108, size108);
      terrain.addRotatableBox(box108, appearance);

      Point3D position109 = new Point3D(8.06995087947249, 0.4533393922315608, 0.04470427328368063);
      Quaternion orientation109 = new Quaternion(0.0, 0.0, 0.7089663975812704,0.7052422612837633);
      Vector3D size109 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box109 = new Box3D(position109, orientation109, size109);
      terrain.addRotatableBox(box109, appearance);

      Point3D position110 = new Point3D(8.256712637978552, 0.45548448733570646, 0.0465446546756541);
      Quaternion orientation110 = new Quaternion(0.0, 0.0, 0.705726540586363,0.7084843328627703);
      Vector3D size110 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box110 = new Box3D(position110, orientation110, size110);
      terrain.addRotatableBox(box110, appearance);

      Point3D position111 = new Point3D(8.534695414460673, -0.40215227023449407, 0.0436624434311707);
      Quaternion orientation111 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size111 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box111 = new Box3D(position111, orientation111, size111);
      terrain.addRotatableBox(box111, appearance);

      Point3D position112 = new Point3D(8.540900973445865, -0.21132852297590346, 0.04400585133181643);
      Quaternion orientation112 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size112 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box112 = new Box3D(position112, orientation112, size112);
      terrain.addRotatableBox(box112, appearance);

      Point3D position113 = new Point3D(8.446166935149789, 0.07613432146555155, 0.04558588339207196);
      Quaternion orientation113 = new Quaternion(0.0, 0.0, 0.7016853567916603,0.7124869543115583);
      Vector3D size113 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box113 = new Box3D(position113, orientation113, size113);
      terrain.addRotatableBox(box113, appearance);

      Point3D position114 = new Point3D(8.629100011417771, 0.078953161543596, 0.0401527710286485);
      Quaternion orientation114 = new Quaternion(0.0, 0.0, 0.7096844738128042,0.7045196573758914);
      Vector3D size114 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box114 = new Box3D(position114, orientation114, size114);
      terrain.addRotatableBox(box114, appearance);

      Point3D position115 = new Point3D(8.544205652116847, 0.363513099370666, 0.04861356986695628);
      Quaternion orientation115 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size115 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box115 = new Box3D(position115, orientation115, size115);
      terrain.addRotatableBox(box115, appearance);

      Point3D position116 = new Point3D(8.5399389891589, 0.555300276983753, 0.04766115185873509);
      Quaternion orientation116 = new Quaternion(0.0, 0.0, 0.0,1.0);
      Vector3D size116 = new Vector3D(0.393, 0.192, 0.0884);
      Box3D box116 = new Box3D(position116, orientation116, size116);
      terrain.addRotatableBox(box116, appearance);

   }
}
