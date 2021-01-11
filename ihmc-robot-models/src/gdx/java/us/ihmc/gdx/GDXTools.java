package us.ihmc.gdx;

import com.badlogic.gdx.Application;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.math.Matrix4;
import org.apache.logging.log4j.Level;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.log.LogTools;

public class GDXTools
{
   public static void syncLogLevelWithLogTools()
   {
      Level log4jLevel = LogTools.getLevel();
      int gdxLogLevel = 2;
      switch (log4jLevel.getStandardLevel())
      {
         case OFF:
            gdxLogLevel = Application.LOG_NONE;
            break;
         case FATAL:
         case ERROR:
            gdxLogLevel = Application.LOG_ERROR;
            break;
         case WARN:
         case INFO:
            gdxLogLevel = Application.LOG_INFO;
            break;
         case DEBUG:
         case TRACE:
            gdxLogLevel = Application.LOG_DEBUG;
            break;
      }
      Gdx.app.setLogLevel(gdxLogLevel);
   }

   public static void convertEuclidAffineToGDXAffine(AffineTransform euclidAffine, Matrix4 gdxAffineToPack)
   {
      gdxAffineToPack.val[Matrix4.M00] = (float) euclidAffine.getM00();
      gdxAffineToPack.val[Matrix4.M01] = (float) euclidAffine.getM01();
      gdxAffineToPack.val[Matrix4.M02] = (float) euclidAffine.getM02();
      gdxAffineToPack.val[Matrix4.M10] = (float) euclidAffine.getM10();
      gdxAffineToPack.val[Matrix4.M11] = (float) euclidAffine.getM11();
      gdxAffineToPack.val[Matrix4.M12] = (float) euclidAffine.getM12();
      gdxAffineToPack.val[Matrix4.M20] = (float) euclidAffine.getM20();
      gdxAffineToPack.val[Matrix4.M21] = (float) euclidAffine.getM21();
      gdxAffineToPack.val[Matrix4.M22] = (float) euclidAffine.getM22();
      gdxAffineToPack.val[Matrix4.M03] = (float) euclidAffine.getM03();
      gdxAffineToPack.val[Matrix4.M13] = (float) euclidAffine.getM13();
      gdxAffineToPack.val[Matrix4.M23] = (float) euclidAffine.getM23();
   }
}
