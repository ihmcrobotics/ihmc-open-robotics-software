package us.ihmc.rdx.tools;

import com.badlogic.gdx.Application;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.VertexAttribute;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import com.badlogic.gdx.graphics.g3d.model.data.ModelMesh;
import com.badlogic.gdx.graphics.g3d.model.data.ModelMeshPart;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.math.Vector3;
import net.mgsx.gltf.scene3d.attributes.PBRColorAttribute;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.logging.log4j.Level;
import org.lwjgl.opengl.GL41;
import org.lwjgl.opengl.GL43;
import org.lwjgl.opengl.GLDebugMessageCallback;
import org.lwjgl.opengl.KHRDebug;
import org.lwjgl.openvr.HmdMatrix34;
import org.lwjgl.openvr.HmdMatrix44;
import org.lwjgl.openvr.HmdVector3;
import org.lwjgl.system.Callback;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.visual.ColorDefinition;

import java.nio.FloatBuffer;

import static com.badlogic.gdx.graphics.profiling.GLInterceptor.resolveErrorNumber;
import static org.lwjgl.glfw.GLFW.glfwGetVersionString;
import static org.lwjgl.system.APIUtil.apiUnknownToken;
import static org.lwjgl.system.MemoryUtil.NULL;

public class LibGDXTools
{
   public static boolean ENABLE_OPENGL_DEBUGGER = Boolean.parseBoolean(System.getProperty("enable.opengl.debugger", "false"));

   public static void syncLogLevelWithLogTools()
   {
      Gdx.app.setLogLevel(toLibGDX(LogTools.getLevel()));
   }

   public static int toLibGDX(Level log4jLevel)
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

   public static void toLibGDX(AffineTransform euclidAffine, Matrix4 gdxAffineToPack)
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

   public static void toLibGDX(RigidBodyTransform rigidBodyTransform, Matrix4 gdxAffineToPack)
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

   public static void toLibGDX(HmdMatrix44 openVRProjectionMatrix, Matrix4 gdxProjectionMatrixToPack)
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

   public static void toLibGDX(HmdMatrix34 openVRRigidBodyTransform, Matrix4 gdxAffineToPack)
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
      toEuclidUnsafe(openVRRigidBodyTransform, rigidBodyTransformToPack);
      if (!rigidBodyTransformToPack.getRotation().isIdentity())
         rigidBodyTransformToPack.getRotation().normalize();
   }

   public static void toEuclidUnsafe(HmdMatrix34 openVRRigidBodyTransform, RigidBodyTransform rigidBodyTransformToPack)
   {
      FloatBuffer openVRValueBuffer = openVRRigidBodyTransform.m();
      rigidBodyTransformToPack.getRotation().setUnsafe(openVRValueBuffer.get(0),
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

   public static void toEuclid(HmdVector3 OpenVR3DVector, Vector3D vector3D)
   {
      FloatBuffer openVRValueBuffer = OpenVR3DVector.v();
      vector3D.set(openVRValueBuffer.get(0), openVRValueBuffer.get(1), openVRValueBuffer.get(2));
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

   public static void toLibGDX(RotationMatrix euclidRotationMatrix, Matrix4 gdxRotationMatrix)
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

   public static void toLibGDX(QuaternionReadOnly euclidQuaternion, Quaternion gdxQuaternion)
   {
      gdxQuaternion.x = euclidQuaternion.getX32();
      gdxQuaternion.y = euclidQuaternion.getY32();
      gdxQuaternion.z = euclidQuaternion.getZ32();
      gdxQuaternion.w = euclidQuaternion.getS32();
   }

   public static void toEuclid(Quaternion gdxQuaternion, us.ihmc.euclid.tuple4D.Quaternion euclidQuaternion)
   {
      euclidQuaternion.set(gdxQuaternion.x, gdxQuaternion.y, gdxQuaternion.z, gdxQuaternion.w);
   }

   public static Vector3 toLibGDX(Tuple3DReadOnly euclidTuple)
   {
      return new Vector3(euclidTuple.getX32(), euclidTuple.getY32(), euclidTuple.getZ32());
   }

   public static Vector2 toLibGDX(Tuple2DReadOnly euclidTuple)
   {
      return new Vector2(euclidTuple.getX32(), euclidTuple.getY32());
   }

   public static void toLibGDX(Tuple3DReadOnly euclidTuple, Vector3 gdxVector3)
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

   public static void toLibGDX(Point3DReadOnly euclidPoint, Matrix4 gdxAffine)
   {
      gdxAffine.setTranslation(euclidPoint.getX32(), euclidPoint.getY32(), euclidPoint.getZ32());
   }

   /**
    * Converts the euclid pose to a rigid body transform and a Matrix4 gdxAffine
    * @param euclidPose input pose. Not modified.
    * @param tempTransform temporary rigid body transform. Modified.
    * @param gdxAffine Matrix4 representation. Modified.
    */
   public static void toLibGDX(RigidBodyTransformReadOnly euclidPose, RigidBodyTransform tempTransform, Matrix4 gdxAffine)
   {
      tempTransform.set(euclidPose);
      toLibGDX(tempTransform, gdxAffine);
   }

   /**
    * Setting the position to NaN will mean it doesn't get shown.
    */
   public static void hideGraphic(ModelInstance modelInstance)
   {
      modelInstance.transform.setTranslation(Float.NaN, Float.NaN, Float.NaN);
   }

   public static void toLibGDX(javafx.scene.paint.Color javaFXColor, Color gdxColor)
   {
      gdxColor.set((float) javaFXColor.getRed(), (float) javaFXColor.getGreen(), (float) javaFXColor.getBlue(), (float) javaFXColor.getOpacity());
   }

   public static Color toLibGDX(javafx.scene.paint.Color javaFXColor)
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

   /**
    * @param hue 0.0 to 360.0
    * @param saturation 0.0 to 1.0
    * @param value 0.0 to 1.0
    */
   public static Color hueSaturationValue(double hue, double saturation, double value)
   {
      return new Color().fromHsv((float) hue, (float) saturation, (float) value);
   }

   public static Color toLibGDX(AppearanceDefinition appearanceDefinition)
   {
      Color gdxColor = new Color(1.0f, 1.0f, 1.0f, 1.0f);
      toLibGDX(appearanceDefinition, gdxColor);
      return gdxColor;
   }

   public static Color toLibGDX(ColorDefinition colorDefinition)
   {
      Color gdxColor = new Color(1.0f, 1.0f, 1.0f, 1.0f);
      toLibGDX(colorDefinition, gdxColor);
      return gdxColor;
   }

   public static void toLibGDX(ColorDefinition colorDefinition, Color gdxColor)
   {
      gdxColor.r = (float) colorDefinition.getRed();
      gdxColor.g = (float) colorDefinition.getGreen();
      gdxColor.b = (float) colorDefinition.getBlue();
      gdxColor.a = (float) colorDefinition.getAlpha();
   }

   public static ColorDefinition toColorDefinition(Color gdxColor)
   {
      ColorDefinition colorDefinition = new ColorDefinition();
      toColorDefinition(gdxColor, colorDefinition);
      return colorDefinition;
   }

   public static void toColorDefinition(Color gdxColor, ColorDefinition colorDefinition)
   {
      colorDefinition.setRed(gdxColor.r);
      colorDefinition.setGreen(gdxColor.g);
      colorDefinition.setBlue(gdxColor.b);
      colorDefinition.setAlpha(gdxColor.a);
   }

   public static void toLibGDX(Vector3 bulletColor, Color gdxColor)
   {
      gdxColor.r = bulletColor.x;
      gdxColor.g = bulletColor.y;
      gdxColor.b = bulletColor.z;
      gdxColor.a = 1.0f;
   }

   public static Color toLibGDX(double red, double green, double blue, double alpha)
   {
      return new Color((float) red, (float) green, (float) blue, (float) alpha);
   }

   public static void toLibGDX(float[] imColor, Color gdxColor)
   {
      gdxColor.set(imColor[0], imColor[1], imColor[2], imColor[3]);
   }

   public static void toLibGDX(AppearanceDefinition appearanceDefinition, Color gdxColor)
   {
      gdxColor.r = appearanceDefinition.getColor().getX();
      gdxColor.g = appearanceDefinition.getColor().getY();
      gdxColor.b = appearanceDefinition.getColor().getZ();
      gdxColor.a = 1.0f - (float) appearanceDefinition.getTransparency();
   }

   /**
    * @param opacity 1.0 for full visibility and 0.0 for invisible
    */
   public static void setOpacity(ModelInstance modelInstance, float opacity)
   {
      for (Material material : modelInstance.materials)
      {
         setOpacity(material, opacity);
      }
   }

   /**
    * @param opacity 1.0 for full visibility and 0.0 for invisible
    */
   public static void setOpacity(Model model, float opacity)
   {
      for (Material material : model.materials)
      {
         setOpacity(material, opacity);
      }
   }

   /**
    * @param opacity 1.0 for full visibility and 0.0 for invisible
    */
   public static void setOpacity(Material material, float opacity)
   {
      if (opacity < 1.0f)
      {
         material.set(new BlendingAttribute(true, opacity));
      }
      else
      {
         material.remove(BlendingAttribute.Type);
      }
   }

   public static void setDiffuseColor(ModelInstance modelInstance, Color color)
   {
      for (Material material : modelInstance.materials)
      {
         material.set(PBRColorAttribute.createBaseColorFactor(color));
      }
   }

   public static long countVertices(ModelInstance modelInstance)
   {
      return countVertices(modelInstance.model);
   }

   public static long countVertices(Model model)
   {
      long numberOfVertices = 0;
      for (int i = 0; i < model.meshes.size; i++)
      {
         numberOfVertices += model.meshes.get(i).getNumVertices();
      }
      return numberOfVertices;
   }

   public static long countVertices(ModelData modelData)
   {
      long numberOfVertices = 0;
      for (int i = 0; i < modelData.meshes.size; i++)
      {
         ModelMesh modelMesh = modelData.meshes.get(i);
         long floatsPerVertex = calculateFloatsPerVertex(modelMesh);
         numberOfVertices += modelMesh.vertices.length / floatsPerVertex;
      }
      return numberOfVertices;
   }

   public static int calculateFloatsPerVertex(ModelMesh modelMesh)
   {
      int vertexSize = 0;
      for (int j = 0; j < modelMesh.attributes.length; j++) {
         VertexAttribute attribute = modelMesh.attributes[j];
         vertexSize += attribute.getSizeInBytes();
      }
      return vertexSize / Float.BYTES;
   }

   public static ModelMeshPart findModelMeshPart(ModelData modelData, String meshPartId)
   {
      for (ModelMesh mesh : modelData.meshes)
         for (ModelMeshPart part : mesh.parts)
            if (part.id.equals(meshPartId))
               return part;
      return null;
   }

   public static ModelMesh findMeshContainingPart(ModelData modelData, String meshPartId)
   {
      for (ModelMesh mesh : modelData.meshes)
         for (ModelMeshPart part : mesh.parts)
            if (part.id.equals(meshPartId))
               return mesh;
      return null;
   }

   public static void setFloatVertexPosition(float[] vertices, int floatsPerVertex, int vertexIndex, Tuple3DReadOnly position)
   {
      vertices[floatsPerVertex * vertexIndex]     = position.getX32();
      vertices[floatsPerVertex * vertexIndex + 1] = position.getY32();
      vertices[floatsPerVertex * vertexIndex + 2] = position.getZ32();
   }

   public static void printShaderLog(String shaderPath, ShaderProgram shaderProgram)
   {
      for (String line : shaderProgram.getLog().split("\n"))
      {
         if (line.isEmpty())
            continue;

         if (line.contains("error"))
            LogTools.error(line);
         else
            LogTools.info(line);
      }
   }

   public static void printGLVersion()
   {
      String glfwVersionString = glfwGetVersionString();
      LogTools.info("Using GLFW {}", glfwVersionString);
      String openGLVersion = GL41.glGetString(GL41.GL_VERSION);
      String openGLVendor = GL41.glGetString(GL41.GL_VENDOR);
      String openGLRenderer = GL41.glGetString(GL41.GL_RENDERER);
      LogTools.info("Using OpenGL {} {} {}", openGLVersion, openGLRenderer, openGLVendor);
   }

   public static GLProfiler createGLProfiler()
   {
      GLProfiler glProfiler = new GLProfiler(Gdx.graphics);
      glProfiler.enable();
      glProfiler.setListener(error ->
      {
         int i = 0;
         int checkIndex = 0;
         int ihmcIndex = 0;
         String glMethodName = null;
         try
         {
            final StackTraceElement[] stack = Thread.currentThread().getStackTrace();
            for (; i < stack.length; i++)
            {
               if (stack[i].getMethodName().equals("check"))
               {
                  if (i + 1 < stack.length)
                  {
                     final StackTraceElement glMethod = stack[i + 1];
                     glMethodName = glMethod.getMethodName();
                     checkIndex = i + 1;
                  }
               }
               if (glMethodName != null && stack[i].getClassName().contains("us.ihmc."))
               {
                  ihmcIndex = i;
                  break;
               }
            }
         }
         catch (Exception ignored)
         {
         }

         if (glMethodName != null)
         {
            LogTools.error(ihmcIndex, "GLProfiler: Error " + resolveErrorNumber(error) + " from " + glMethodName);
         }
         else
         {
            LogTools.error(ihmcIndex, "GLProfiler: Error " + resolveErrorNumber(error) + " at: " + new Exception());
            // This will capture current stack trace for logging, if possible
         }
         new Throwable().printStackTrace();
      });
      return glProfiler;
   }

   /**
    * TODO: Add support for all of the drivers.
    * See org.lwjgl.opengl.GLUtil#setupDebugMessageCallback(java.io.PrintStream)
    */
   public static Callback setupDebugMessageCallback(int minimumDebugLevel)
   {
      LogTools.info("Using KHR_debug for OpenGL debugging");
      GLDebugMessageCallback callback = GLDebugMessageCallback.create((source, type, id, severity, length, message, userParam) ->
      {
         if (getDebugSeverityLevel(severity) >= getDebugSeverityLevel(minimumDebugLevel))
         {
            String messageString =
                  "[" + getDebugSeverity(severity) + "] ID: " + String.format("0x%X", id) + " Source: " + getDebugSource(source) + " Type: " + getDebugType(type)
                  + " " + GLDebugMessageCallback.getMessage(length, message);
            switch (severity)
            {
               case GL43.GL_DEBUG_SEVERITY_HIGH:
                  LogTools.fatal(messageString);
                  break;
               case GL43.GL_DEBUG_SEVERITY_MEDIUM:
                  LogTools.error(messageString);
                  break;
               case GL43.GL_DEBUG_SEVERITY_LOW:
                  LogTools.warn(messageString);
                  break;
               case GL43.GL_DEBUG_SEVERITY_NOTIFICATION:
               default:
                  LogTools.info(messageString);
                  break;
            }
         }
      });
      KHRDebug.glDebugMessageCallback(callback, NULL);
      GL41.glEnable(GL43.GL_DEBUG_OUTPUT);
      return callback;
   }

   private static int getDebugSeverityLevel(int severity)
   {
      switch (severity)
      {
         case GL43.GL_DEBUG_SEVERITY_HIGH:
            return 5;
         case GL43.GL_DEBUG_SEVERITY_MEDIUM:
            return 4;
         case GL43.GL_DEBUG_SEVERITY_LOW:
            return 3;
         case GL43.GL_DEBUG_SEVERITY_NOTIFICATION:
         default:
            return 2;
      }
   }

   private static String getDebugSeverity(int severity)
   {
      switch (severity)
      {
         case GL43.GL_DEBUG_SEVERITY_HIGH:
            return "HIGH";
         case GL43.GL_DEBUG_SEVERITY_MEDIUM:
            return "MEDIUM";
         case GL43.GL_DEBUG_SEVERITY_LOW:
            return "LOW";
         case GL43.GL_DEBUG_SEVERITY_NOTIFICATION:
            return "NOTIFICATION";
         default:
            return apiUnknownToken(severity);
      }
   }

   private static String getDebugSource(int source)
   {
      switch (source)
      {
         case GL43.GL_DEBUG_SOURCE_API:
            return "API";
         case GL43.GL_DEBUG_SOURCE_WINDOW_SYSTEM:
            return "WINDOW SYSTEM";
         case GL43.GL_DEBUG_SOURCE_SHADER_COMPILER:
            return "SHADER COMPILER";
         case GL43.GL_DEBUG_SOURCE_THIRD_PARTY:
            return "THIRD PARTY";
         case GL43.GL_DEBUG_SOURCE_APPLICATION:
            return "APPLICATION";
         case GL43.GL_DEBUG_SOURCE_OTHER:
            return "OTHER";
         default:
            return apiUnknownToken(source);
      }
   }

   private static String getDebugType(int type)
   {
      switch (type)
      {
         case GL43.GL_DEBUG_TYPE_ERROR:
            return "ERROR";
         case GL43.GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
            return "DEPRECATED BEHAVIOR";
         case GL43.GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
            return "UNDEFINED BEHAVIOR";
         case GL43.GL_DEBUG_TYPE_PORTABILITY:
            return "PORTABILITY";
         case GL43.GL_DEBUG_TYPE_PERFORMANCE:
            return "PERFORMANCE";
         case GL43.GL_DEBUG_TYPE_OTHER:
            return "OTHER";
         case GL43.GL_DEBUG_TYPE_MARKER:
            return "MARKER";
         default:
            return apiUnknownToken(type);
      }
   }

   public static Pair<String, String> loadCombinedShader(String pathForLoadingFromClasspath)
   {
      String combinedString = Gdx.files.classpath(pathForLoadingFromClasspath).readString();

      // Support loading from CRLF checkouts
      combinedString = combinedString.replaceAll("\\r\\n", "\n");

      String vertexMacro = "#type vertex\n";
      int vertexBegin = combinedString.indexOf(vertexMacro);
      String fragmentMacro = "#type fragment\n";
      int fragmentBegin = combinedString.indexOf(fragmentMacro);

      String vertexShader = combinedString.substring(vertexBegin + vertexMacro.length() - 1, fragmentBegin).trim();
      String fragmentShader = combinedString.substring(fragmentBegin + fragmentMacro.length() - 1).trim();
      return Pair.of(vertexShader, fragmentShader);
   }
}
