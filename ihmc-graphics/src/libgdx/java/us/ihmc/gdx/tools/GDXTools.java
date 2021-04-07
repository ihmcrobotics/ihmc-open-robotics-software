package us.ihmc.gdx.tools;

import com.badlogic.gdx.Application;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.math.Vector3;
import org.apache.logging.log4j.Level;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
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

   public static void toGDX(AffineTransform euclidAffine, Matrix4 gdxAffineToPack)
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

   public static void toEuclid(Matrix4 gdxAffine, AffineTransform euclidAffine)
   {
      euclidAffine.getLinearTransform().setM00(gdxAffine.val[Matrix4.M00]);
      euclidAffine.getLinearTransform().setM01(gdxAffine.val[Matrix4.M01]);
      euclidAffine.getLinearTransform().setM02(gdxAffine.val[Matrix4.M02]);
      euclidAffine.getLinearTransform().setM10(gdxAffine.val[Matrix4.M10]);
      euclidAffine.getLinearTransform().setM11(gdxAffine.val[Matrix4.M11]);
      euclidAffine.getLinearTransform().setM12(gdxAffine.val[Matrix4.M12]);
      euclidAffine.getLinearTransform().setM20(gdxAffine.val[Matrix4.M20]);
      euclidAffine.getLinearTransform().setM21(gdxAffine.val[Matrix4.M21]);
      euclidAffine.getLinearTransform().setM22(gdxAffine.val[Matrix4.M22]);
      euclidAffine.getTranslation().setX(gdxAffine.val[Matrix4.M03]);
      euclidAffine.getTranslation().setY(gdxAffine.val[Matrix4.M13]);
      euclidAffine.getTranslation().setZ(gdxAffine.val[Matrix4.M23]);
   }

   public static void toEuclid(Matrix4 gdxAffine, RotationMatrix euclidRotationMatrix)
   {
      euclidRotationMatrix.set(gdxAffine.val[Matrix4.M00],
                               gdxAffine.val[Matrix4.M01],
                               gdxAffine.val[Matrix4.M02],
                               gdxAffine.val[Matrix4.M10],
                               gdxAffine.val[Matrix4.M11],
                               gdxAffine.val[Matrix4.M12],
                               gdxAffine.val[Matrix4.M20],
                               gdxAffine.val[Matrix4.M21],
                               gdxAffine.val[Matrix4.M22]);
   }

   public static void toGDX(RigidBodyTransform rigidBodyTransform, Matrix4 gdxAffineToPack)
   {
      gdxAffineToPack.val[Matrix4.M00] = (float) rigidBodyTransform.getM00();
      gdxAffineToPack.val[Matrix4.M01] = (float) rigidBodyTransform.getM01();
      gdxAffineToPack.val[Matrix4.M02] = (float) rigidBodyTransform.getM02();
      gdxAffineToPack.val[Matrix4.M10] = (float) rigidBodyTransform.getM10();
      gdxAffineToPack.val[Matrix4.M11] = (float) rigidBodyTransform.getM11();
      gdxAffineToPack.val[Matrix4.M12] = (float) rigidBodyTransform.getM12();
      gdxAffineToPack.val[Matrix4.M20] = (float) rigidBodyTransform.getM20();
      gdxAffineToPack.val[Matrix4.M21] = (float) rigidBodyTransform.getM21();
      gdxAffineToPack.val[Matrix4.M22] = (float) rigidBodyTransform.getM22();
      gdxAffineToPack.val[Matrix4.M03] = (float) rigidBodyTransform.getM03();
      gdxAffineToPack.val[Matrix4.M13] = (float) rigidBodyTransform.getM13();
      gdxAffineToPack.val[Matrix4.M23] = (float) rigidBodyTransform.getM23();
   }

   public static void toEuclid(Matrix4 gdxAffine, RigidBodyTransform rigidBodyTransform)
   {
      rigidBodyTransform.getRotation().set(gdxAffine.val[Matrix4.M00],
                                           gdxAffine.val[Matrix4.M01],
                                           gdxAffine.val[Matrix4.M02],
                                           gdxAffine.val[Matrix4.M10],
                                           gdxAffine.val[Matrix4.M11],
                                           gdxAffine.val[Matrix4.M12],
                                           gdxAffine.val[Matrix4.M20],
                                           gdxAffine.val[Matrix4.M21],
                                           gdxAffine.val[Matrix4.M22]);
      rigidBodyTransform.getTranslation().setX(gdxAffine.val[Matrix4.M03]);
      rigidBodyTransform.getTranslation().setY(gdxAffine.val[Matrix4.M13]);
      rigidBodyTransform.getTranslation().setZ(gdxAffine.val[Matrix4.M23]);
   }

   public static void toGDX(RotationMatrix euclidRotationMatrix, Matrix4 gdxRotationMatrix)
   {
      gdxRotationMatrix.val[Matrix4.M00] = (float) euclidRotationMatrix.getM00();
      gdxRotationMatrix.val[Matrix4.M01] = (float) euclidRotationMatrix.getM01();
      gdxRotationMatrix.val[Matrix4.M02] = (float) euclidRotationMatrix.getM02();
      gdxRotationMatrix.val[Matrix4.M10] = (float) euclidRotationMatrix.getM10();
      gdxRotationMatrix.val[Matrix4.M11] = (float) euclidRotationMatrix.getM11();
      gdxRotationMatrix.val[Matrix4.M12] = (float) euclidRotationMatrix.getM12();
      gdxRotationMatrix.val[Matrix4.M20] = (float) euclidRotationMatrix.getM20();
      gdxRotationMatrix.val[Matrix4.M21] = (float) euclidRotationMatrix.getM21();
      gdxRotationMatrix.val[Matrix4.M22] = (float) euclidRotationMatrix.getM22();
   }

   public static Vector3 toGDX(Tuple3DReadOnly euclidTuple)
   {
      return new Vector3(euclidTuple.getX32(), euclidTuple.getY32(), euclidTuple.getZ32());
   }

   public static Vector2 toGDX(Tuple2DReadOnly euclidTuple)
   {
      return new Vector2(euclidTuple.getX32(), euclidTuple.getY32());
   }

   public static void toGDX(Tuple3DReadOnly euclidTuple, Vector3 gdxVector3)
   {
      gdxVector3.set(euclidTuple.getX32(), euclidTuple.getY32(), euclidTuple.getZ32());
   }

   public static void toEuclid(Vector3 gdxVector3, Vector3DBasics euclidVector3D32)
   {
      euclidVector3D32.set(gdxVector3.x, gdxVector3.y, gdxVector3.z);
   }

   public static void toEuclid(Vector3 gdxVector3, Point3DBasics euclidPoint3D32)
   {
      euclidPoint3D32.set(gdxVector3.x, gdxVector3.y, gdxVector3.z);
   }

   public static void toEuclid(Matrix4 gdxAffine, Point3DBasics euclidPoint)
   {
      euclidPoint.set(gdxAffine.val[Matrix4.M03],
                      gdxAffine.val[Matrix4.M13],
                      gdxAffine.val[Matrix4.M23]);
   }

   public static void toGDX(Point3DBasics euclidPoint, Matrix4 gdxAffine)
   {
      gdxAffine.setTranslation(euclidPoint.getX32(), euclidPoint.getY32(), euclidPoint.getZ32());
   }

   public static void toGDX(javafx.scene.paint.Color javaFXColor, Color gdxColor)
   {
      gdxColor.set((float) javaFXColor.getRed(), (float) javaFXColor.getGreen(), (float) javaFXColor.getBlue(), (float) javaFXColor.getOpacity());
   }

   public static Color toGDX(javafx.scene.paint.Color javaFXColor)
   {
      return new Color((float) javaFXColor.getRed(), (float) javaFXColor.getGreen(), (float) javaFXColor.getBlue(), (float) javaFXColor.getOpacity());
   }

   /**
    * JavaFX colors are immutable so we have to return a new one and there isn't a garbage free option
    * unless we look em up.
    */
   public static javafx.scene.paint.Color toJavaFX(Color gdxColor)
   {
      return javafx.scene.paint.Color.color(gdxColor.r, gdxColor.g, gdxColor.b, gdxColor.a);
   }
}
