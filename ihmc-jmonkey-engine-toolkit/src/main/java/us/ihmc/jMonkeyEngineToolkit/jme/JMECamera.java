package us.ihmc.jMonkeyEngineToolkit.jme;

import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.jMonkeyEngineToolkit.CameraAdapter;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraController;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEDataTypeUtils;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEGeometryUtils;

/**
 * JMECamera overloads camera and implements the following assumptions
 * 
 * - Z-UP frame
 * - Horizontal FoV
 * 
 *
 */
public class JMECamera extends Camera implements CameraAdapter
{
   public static final Quaternion convertLookingUpToLookingForward = new Quaternion();
   private static final Quaternion convertLookingForwardToLookingUp;
   static
   {
      convertLookingUpToLookingForward.fromRotationMatrix(0f, 0f, 1f, 1f, 0f, 0f, 0f, 1f, 0f);
      convertLookingForwardToLookingUp = convertLookingUpToLookingForward.inverse();
   }
   private final QuaternionBasics cameraRotation = new us.ihmc.euclid.tuple4D.Quaternion();
   private final Vector3D cameraPosition = new Vector3D();

   private CameraController cameraController;

   private final RigidBodyTransform cameraTransform = new RigidBodyTransform();

   private float horizontalFoVInRadians = FastMath.PI / 4.0f;
   private float clipDistanceNear = 0.15f;
   private float clipDistanceFar = 1000.0f;
   private boolean frustrumNeedsUpdating = false;

   private int previousWindowWidth = 0;
   private int previousWindowHeight = 0;

   public JMECamera(Camera camera)
   {
      super(camera.getWidth(), camera.getHeight());
      copyFrom(camera);
      updateFrustrum();
   }

   public void setHorizontalFoVInRadians(float fovHorizontalInRadians)
   {
      if (this.horizontalFoVInRadians != fovHorizontalInRadians)
      {
         this.horizontalFoVInRadians = fovHorizontalInRadians;
         frustrumNeedsUpdating = true;
      }
   }

   public void setClipDistanceNear(float clipDistanceNear)
   {
      if (this.clipDistanceNear != clipDistanceNear)
      {
         this.clipDistanceNear = clipDistanceNear;
         frustrumNeedsUpdating = true;
      }
   }

   public void setClipDistanceFar(float clipDistanceFar)
   {
      if (this.clipDistanceFar != clipDistanceFar)
      {
         this.clipDistanceFar = clipDistanceFar;
         frustrumNeedsUpdating = true;
      }
   }

   @Deprecated
   public void setFrustumPerspective(float fovY, float aspect, float near, float far)
   {
      throw new RuntimeException("Use setFovHorizontalInRadians, setClipDistanceNear and setClipDistanceFar.");
   }

   private void updateFrustrum()
   {
      int windowWidth = getWidth();
      int windowHeight = getHeight();

      if (windowWidth != previousWindowWidth || windowHeight != previousWindowHeight || frustrumNeedsUpdating)
      {
         float aspect = ((float) getInternalHeight()) / ((float) getInternalWidth());

         float w = FastMath.tan(horizontalFoVInRadians * .5f) * clipDistanceNear;
         float h = w * aspect;
         frustumLeft = -w;
         frustumRight = w;
         frustumBottom = -h;
         frustumTop = h;
         frustumNear = clipDistanceNear;
         frustumFar = clipDistanceFar;

         onFrustumChange();
         previousWindowWidth = windowWidth;
         previousWindowHeight = windowHeight;

         frustrumNeedsUpdating = false;
      }

   }

   public float getNearClip()
   {
      return clipDistanceNear;
   }

   public float getFarClip()
   {
      return clipDistanceFar;
   }

   public float getHorizontalFovInRadians()
   {
      return horizontalFoVInRadians;
   }

   public CameraController getController()
   {
      return cameraController;
   }

   public void setLocationInZUpCoordinates(Tuple3DBasics cameraPosition)
   {
      setLocationInZUpCoordinates(JMEDataTypeUtils.vecMathTuple3dToJMEVector3f(cameraPosition));
   }

   public void setLocationInZUpCoordinates(Vector3f location)
   {
      Vector3f tempVector = new Vector3f(location);
      JMEGeometryUtils.transformFromZupToJMECoordinates(tempVector);
      setLocation(tempVector);
   }

   public void setRotationInZUpcoordinates(QuaternionReadOnly rotation)
   {
      setRotationInZUpcoordinates(JMEDataTypeUtils.vecMathQuat4dToJMEQuaternion(rotation));
   }

   public void setRotationInZUpcoordinates(Quaternion rotation)
   {
      Quaternion tempQuat = new Quaternion(rotation);
      tempQuat.multLocal(convertLookingUpToLookingForward);
      JMEGeometryUtils.transformFromZupToJMECoordinates(tempQuat);
      setRotation(tempQuat);
   }

   /**
    * @deprecated Use setLocationInZUpCoordinates
    */
   @Deprecated
   public void setLocation(Vector3f location)
   {
      super.setLocation(location);
   }

   /**
    * @deprecated Use setRotationInZUpcoordinates
    */
   @Deprecated
   public void setRotation(Quaternion rotation)
   {
      super.setRotation(rotation);
   }

   public void updateCamera()
   {
      if (cameraController != null)
      {
         cameraController.computeTransform(cameraTransform);
         cameraTransform.get(cameraRotation, cameraPosition);

         setLocationInZUpCoordinates(cameraPosition);
         setRotationInZUpcoordinates(cameraRotation);

         setHorizontalFoVInRadians((float) cameraController.getHorizontalFieldOfViewInRadians());
         setClipDistanceNear((float) cameraController.getClipNear());
         setClipDistanceFar((float) cameraController.getClipFar());
      }

      updateFrustrum();
   }

   public void setCameraController(CameraController cameraController)
   {
      this.cameraController = cameraController;
   }

   public QuaternionBasics getCameraRotation()
   {
      Quaternion quat = new Quaternion(getRotation());
      JMEGeometryUtils.transformFromJMECoordinatesToZup(quat);
      quat.multLocal(convertLookingForwardToLookingUp);

      return new us.ihmc.euclid.tuple4D.Quaternion(quat.getX(), quat.getY(), quat.getZ(), quat.getW());
   }

   public Point3D getCameraPosition()
   {
      Vector3f position = new Vector3f(getLocation());
      JMEGeometryUtils.transformFromJMECoordinatesToZup(position);
      return new Point3D(position.getX(), position.getY(), position.getZ());
   }

   @Override
   public void resize(int width, int height, boolean fixAspect)
   {
      super.resize(width, height, false);

      if (fixAspect /* && !parallelProjection */)
      {

         frustumRight = frustumTop * ((float) getInternalWidth() / getInternalHeight());
         frustumLeft = -frustumRight;
         onFrustumChange();
      }
   }

   public int getInternalWidth()
   {
      return (int) (width * (viewPortRight - viewPortLeft));
   }

   public int getInternalHeight()
   {
      return (int) (height * (viewPortTop - viewPortBottom));
   }

   private boolean alreadyClosing = false;

   public void closeAndDispose()
   {
      if (alreadyClosing)
         return;
      alreadyClosing = true;

      if (cameraController != null)
      {
         cameraController.closeAndDispose();
         cameraController = null;
      }
   }
}
