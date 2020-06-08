package us.ihmc.robotics;

import us.ihmc.log.LogTools;

/**
 * This is Skully the friendly debugging skull.
 */
public class Skully
{
   private static final String face =
         "  ,----._\n" +
         " )\\___/  \\\n" +
         "/__, ,__,(|\n" +
         "|.d/ \\b. _/\n" +
         " \\/''  \\||\n" +
         "  '+++'//\n" +
         "  `-.-'";

   private static final String betterFace =
      "                                     .rYrL7ri:.\n" +
      "                                 iNBBBBBBMGqMGZUu7i:. .\n" +
      "                              LOBBBBMMOOXFkOqSXGJ7iuuY7r,,\n" +
      "                          iZBBBBBBBPGME5GvvNEUNU7Y2::77...\n" +
      "                      :1BBBBBBBBBBMEMMPG8MEB1r7J7v;,.,,.\n" +
      "                  .jBBBBBBBBBBBBBBBBBMBBBX00BL.iri::i.\n" +
      "               ;GBBBBBBBBBBBBBBBBBBBBBBBMG5jjN8r.i:,r;\n" +
      "            iBBBBBBBBBBBBBBBBBBMMGOqLi&&i;jvr7OJ  :i7:.\n" +
      "          .BBBBBBBBBBBBBBBBBBBBBBBBBEqZXYSu:  ..:J2Y:,...\n" +
      "        .BBBBBBBBBBBBBBBBBBBBBBBBBBBGPOq5UUrr:iiiri     ..\n" +
      "       ,BBBBBBBBBBBBBBBBBBBBBBBBMMO0kkkU;7L7i::VJ7i.    ,.\n" +
      "       BBBBBBBBBBBBBBBBBBBBBBBBBBZM8M02JSYLU7:;Yj:,:.\n" +
      "      BBBBBBBBBBBBBBBBBBBBBBBBBBBBMZEULU1J7vi7vri  rr\n" +
      "     BBBBBBBBBBBBBBBBBBBBBBBBBBBBMGUuUuJJ77r7::,r:.iL\n" +
      "    YBBBBBBBBBBBBBBBBBBBBBBBBBBBMES22PYvu57L::iLr:.:7:\n" +
      "    BBBBBBBBBBBBBBBBBBBBBBBBBBBB0x0qS0UYqXvrLirL7::,:;.\n" +
      "   ,BBBBBBBBBBBBBBBBBBBBBBBBBBBBB8M*PBM8XurvLvu57ri.:u2:\n" +
      "   qBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBN1uLrLuv7i,:.:7kJ,\n" +
      "   BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBMN1UYLLL7r,:;r:,iU:\n" +
      "  rBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBOOkrrYurr;rr::Li:i;r.\n" +
      "  BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB27vUYv,:ijv;7L;7irjSY:\n" +
      " .BBBBBBBBBBBBBBBBBBBBBBBBBBBBB1ri;;ri:.  ....:77YL7LLYi\n" +
      " YBBBBBBBBBOr.,:.7ZBBMMMUYjF2Lii,,...            ,LkYrii\n" +
      " MBBBBBBBj.   ..   LBBMFr:::,ii.                   .71u:\n" +
      " EBBBBBq  .i:::ri:. iM07:JjU8BL:i                    rGU.\n" +
      " 2BBBBU  ,:ii:iii::. iPqBBBBFiVL:   .                 :BB:\n" +
      " :BBBq  ii,::;:i:,:1..LBBBBG   ,                       iBq\n" +
      "  BBB  .r;;ii:i:::i,:  OBBBF                            jBB\n" +
      "  1BB; .:i...:ii,ii:.  BBB1                             .BBi\n" +
      "  :BBB, ,   ..::i::,.iBBBM                              2BB2\n" +
      "   BBB. .:qMi,..,:..7Bu:YBi         ..                 OBBBi\n" +
      "   :BB:  :BB;.:,    Y:   :v                           EBBBB\n" +
      "   rBB   Y1iirv5iiNBB.    UE                         XBBBBM  ,\n" +
      "   :BB SBZUMBBBBBBBBr     ,L.                       :BBBBBr ::\n" +
      "   jBB8ZBBBBBBBBBBBO          .      7BBq57:,     r8BBBBML   :\n" +
      "   BBBBBBBBBBBBBBBB.          :      qBBBBBBBBBkBBBO07Y:.\n" +
      "  UBBBBBBBki  SBBBM            :.    YjBBMBBP718j;::,.  ..\n" +
      " .BBBBG;      BBBB8             .    .:i      Y::rLr7r7:;.\n" +
      " BBBBBji.:: .BBBBBM             7,    ::   iqJ,:7uvLrrri:.\n" +
      " BBBBki::,iUBBBBBBB            :J.    .    BB7,:LjY7;::.,\n" +
      " iBBqYirukqZBBBBBBB,      rG: .:r.         YN7:jGNL77rir,.\n" +
      "       .ivkMBBBBBBBBBBBXBBB. .vPv  ,vii752r:XOBBB02uXNFU7\n" +
      "           iBBBBBBBBBBBBBv iBBBB.:BBPrFJ;r  7B0k7ii:i..Er\n" +
      "            NBBBBBBBBBBBBYMBSBBMrBBB..S,    .UL.       :\n" +
      "            1BBBBBBBBBBBBBBL BB5:BBB  .     r;\n" +
      "            1BBBBBBBBBOBBBBZMBB.SBBr\n" +
      "            L7SBBBBBBBBBBBBBBBGYGBq,\n" +
      "            YMBBBBBBBBBBBBBErrFU.  .\n" +
      "_____________BBBBBZBBk NMr. :BX  .Br\n" +
      "|  I am a   |1BSvi1PBi iOL: 7Bi  BB\n" +
      "| a static  | BM:.:JZr rGU:  i  BBL\n" +
      "|entity, you| OBBBU i. iNUJ.   :Bk\n" +
      "|  cannot   | GBBBB .q:\n" +
      "|create me! | 8BBBM,BB7i. :  ,\n" +
      "|___________| MBBBZUBBJ55BUiBM  7i\n" +
      "          \\ | BBBBBMOqBMBBEMB8.GBu ,:,\n" +
      "           \\| .BBBBU::rk27r.iU. EBBXi.:\n" +
      "             UBBBML7YrFNjrr;.  5m.\n" +
      "             BBBBB8Z2jmBMOPYYrYBqi\n" +
      "             BBBBBBBBBBBBB2LkqBMUUi,.\n" +
      "             BBBBBBBBBBBBBk8BBBGJYi::.::\n" +
      "             BBBBBBBBBBBBBO8EG0Uji;JY;:\n" +
      "              BBBBBBBBBBBBY1FESU7ijL.\n" +
      "               BBBBBBBBBJ:7PkUuuu7r   .\n" +
      "                .YSEFS&,,,:i.:....,,..\n";

   public Skully()
   {
      throw new RuntimeException(betterFace);
   }

   public static void say(String message)
   {
      print("'" + message + "'\n" + face);
   }

   public static void say(String message, String additionalInfo)
   {
      print("'" + message + "'\n" + face + "\n" + additionalInfo);
   }

   private static void print(String output)
   {
      LogTools.warn("Skully says he discovered something mysterious:\n   " + output.replace("\n", "\n   "));
   }
}
