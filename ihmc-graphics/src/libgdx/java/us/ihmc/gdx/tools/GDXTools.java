package us.ihmc.gdx.tools;

import com.badlogic.gdx.Application;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.math.Vector3;
import org.apache.logging.log4j.Level;
import org.lwjgl.openvr.HmdMatrix34;
import org.lwjgl.openvr.HmdMatrix44;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.log.LogTools;

import java.nio.FloatBuffer;

public class GDXTools
{
   public static void syncLogLevelWithLogTools()
   {
      Gdx.app.setLogLevel(toGDX(LogTools.getLevel()));
   }

   public static int toGDX(Level log4jLevel)
   {
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
      return gdxLogLevel;
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
      euclidAffine.getLinearTransform().normalize();
      euclidAffine.getTranslation().setX(gdxAffine.val[Matrix4.M03]);
      euclidAffine.getTranslation().setY(gdxAffine.val[Matrix4.M13]);
      euclidAffine.getTranslation().setZ(gdxAffine.val[Matrix4.M23]);
   }

   public static void toEuclid(Matrix4 gdxAffine, RotationMatrix euclidRotationMatrix)
   {
      euclidRotationMatrix.setAndNormalize(gdxAffine.val[Matrix4.M00],
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

   public static void toGDX(HmdMatrix44 openVRProjectionMatrix, Matrix4 gdxProjectionMatrixToPack)
   {
      FloatBuffer openVRValueBuffer = openVRProjectionMatrix.m();
      gdxProjectionMatrixToPack.val[0] = openVRValueBuffer.get(0);
      gdxProjectionMatrixToPack.val[1] = openVRValueBuffer.get(4);
      gdxProjectionMatrixToPack.val[2] = openVRValueBuffer.get(8);
      gdxProjectionMatrixToPack.val[3] = openVRValueBuffer.get(12);
      gdxProjectionMatrixToPack.val[4] = openVRValueBuffer.get(1);
      gdxProjectionMatrixToPack.val[5] = openVRValueBuffer.get(5);
      gdxProjectionMatrixToPack.val[6] = openVRValueBuffer.get(9);
      gdxProjectionMatrixToPack.val[7] = openVRValueBuffer.get(13);
      gdxProjectionMatrixToPack.val[8] = openVRValueBuffer.get(2);
      gdxProjectionMatrixToPack.val[9] = openVRValueBuffer.get(6);
      gdxProjectionMatrixToPack.val[10] = openVRValueBuffer.get(10);
      gdxProjectionMatrixToPack.val[11] = openVRValueBuffer.get(14);
      gdxProjectionMatrixToPack.val[12] = openVRValueBuffer.get(3);
      gdxProjectionMatrixToPack.val[13] = openVRValueBuffer.get(7);
      gdxProjectionMatrixToPack.val[14] = openVRValueBuffer.get(11);
      gdxProjectionMatrixToPack.val[15] = openVRValueBuffer.get(15);
   }

   public static void toGDX(HmdMatrix34 openVRRigidBodyTransform, Matrix4 gdxAffineToPack)
   {
      FloatBuffer openVRValueBuffer = openVRRigidBodyTransform.m();
      gdxAffineToPack.val[0] = openVRValueBuffer.get(0);
      gdxAffineToPack.val[1] = openVRValueBuffer.get(4);
      gdxAffineToPack.val[2] = openVRValueBuffer.get(8);
      gdxAffineToPack.val[3] = 0;
      gdxAffineToPack.val[4] = openVRValueBuffer.get(1);
      gdxAffineToPack.val[5] = openVRValueBuffer.get(5);
      gdxAffineToPack.val[6] = openVRValueBuffer.get(9);
      gdxAffineToPack.val[7] = 0;
      gdxAffineToPack.val[8] = openVRValueBuffer.get(2);
      gdxAffineToPack.val[9] = openVRValueBuffer.get(6);
      gdxAffineToPack.val[10] = openVRValueBuffer.get(10);
      gdxAffineToPack.val[11] = 0;
      gdxAffineToPack.val[12] = openVRValueBuffer.get(3);
      gdxAffineToPack.val[13] = openVRValueBuffer.get(7);
      gdxAffineToPack.val[14] = openVRValueBuffer.get(11);
      gdxAffineToPack.val[15] = 1;
   }

   public static void toEuclid(HmdMatrix34 openVRRigidBodyTransform, RigidBodyTransform rigidBodyTransformToPack)
   {
      FloatBuffer openVRValueBuffer = openVRRigidBodyTransform.m();
      rigidBodyTransformToPack.getRotation().setAndNormalize(openVRValueBuffer.get(0),
                                                             openVRValueBuffer.get(1),
                                                             openVRValueBuffer.get(2),
                                                             openVRValueBuffer.get(4),
                                                             openVRValueBuffer.get(5),
                                                             openVRValueBuffer.get(6),
                                                             openVRValueBuffer.get(8),
                                                             openVRValueBuffer.get(9),
                                                             openVRValueBuffer.get(10));
      rigidBodyTransformToPack.getTranslation().setX(openVRValueBuffer.get(3));
      rigidBodyTransformToPack.getTranslation().setY(openVRValueBuffer.get(7));
      rigidBodyTransformToPack.getTranslation().setZ(openVRValueBuffer.get(11));
   }

   public static void toEuclid(HmdMatrix34 openVRRigidBodyTransform, Pose3DBasics poseToPack)
   {
      FloatBuffer openVRValueBuffer = openVRRigidBodyTransform.m();
      poseToPack.getOrientation().setRotationMatrix(openVRValueBuffer.get(0),
                                                    openVRValueBuffer.get(1),
                                                    openVRValueBuffer.get(2),
                                                    openVRValueBuffer.get(4),
                                                    openVRValueBuffer.get(5),
                                                    openVRValueBuffer.get(6),
                                                    openVRValueBuffer.get(8),
                                                    openVRValueBuffer.get(9),
                                                    openVRValueBuffer.get(10));
      poseToPack.getOrientation().normalize();
      poseToPack.getPosition().setX(openVRValueBuffer.get(3));
      poseToPack.getPosition().setY(openVRValueBuffer.get(7));
      poseToPack.getPosition().setZ(openVRValueBuffer.get(11));
   }

   public static void toEuclid(HmdMatrix34 openVRAffineTransform, AffineTransform euclidAffine)
   {
      FloatBuffer openVRValueBuffer = openVRAffineTransform.m();
      euclidAffine.getLinearTransform().setM00(openVRValueBuffer.get(0));
      euclidAffine.getLinearTransform().setM01(openVRValueBuffer.get(1));
      euclidAffine.getLinearTransform().setM02(openVRValueBuffer.get(2));
      euclidAffine.getLinearTransform().setM10(openVRValueBuffer.get(4));
      euclidAffine.getLinearTransform().setM11(openVRValueBuffer.get(5));
      euclidAffine.getLinearTransform().setM12(openVRValueBuffer.get(6));
      euclidAffine.getLinearTransform().setM20(openVRValueBuffer.get(8));
      euclidAffine.getLinearTransform().setM21(openVRValueBuffer.get(9));
      euclidAffine.getLinearTransform().setM22(openVRValueBuffer.get(10));
      euclidAffine.getLinearTransform().normalize();
      euclidAffine.getTranslation().setX(openVRValueBuffer.get(3));
      euclidAffine.getTranslation().setY(openVRValueBuffer.get(7));
      euclidAffine.getTranslation().setZ(openVRValueBuffer.get(11));
   }

   public static void toEuclid(Matrix4 gdxAffine, RigidBodyTransform rigidBodyTransform)
   {
      rigidBodyTransform.getRotation().setAndNormalize(gdxAffine.val[Matrix4.M00],
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

   public static void toGDX(Point3DReadOnly euclidPoint, Matrix4 gdxAffine)
   {
      gdxAffine.setTranslation(euclidPoint.getX32(), euclidPoint.getY32(), euclidPoint.getZ32());
   }

   public static void toGDX(Pose3DReadOnly euclidPose, RigidBodyTransform tempTransform, Matrix4 gdxAffine)
   {
      euclidPose.get(tempTransform);
      toGDX(tempTransform, gdxAffine);
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

   public static Color toGDX(AppearanceDefinition appearanceDefinition)
   {
      Color gdxColor = new Color(1.0f, 1.0f, 1.0f, 1.0f);
      toGDX(appearanceDefinition, gdxColor);
      return gdxColor;
   }

   public static void toGDX(AppearanceDefinition appearanceDefinition, Color gdxColor)
   {
      gdxColor.r = appearanceDefinition.getColor().getX();
      gdxColor.g = appearanceDefinition.getColor().getY();
      gdxColor.b = appearanceDefinition.getColor().getZ();
      gdxColor.a = 1.0f - (float) appearanceDefinition.getTransparency();
   }
}
