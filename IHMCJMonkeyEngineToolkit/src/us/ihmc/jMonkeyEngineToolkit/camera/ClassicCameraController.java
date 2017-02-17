package us.ihmc.jMonkeyEngineToolkit.camera;

import java.util.ArrayList;

import javax.swing.JFrame;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.input.SelectedListener;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.robotics.geometry.shapes.Sphere3d;
import us.ihmc.tools.inputDevices.keyboard.Key;
import us.ihmc.tools.inputDevices.keyboard.KeyListener;
import us.ihmc.tools.inputDevices.keyboard.ModifierKeyInterface;
import us.ihmc.tools.inputDevices.mouse.MouseButton;
import us.ihmc.tools.inputDevices.mouse.MouseListener;
import us.ihmc.tools.inputDevices.mouse3DJoystick.Mouse3DListener;

public class ClassicCameraController implements TrackingDollyCameraController, KeyListener, MouseListener, Mouse3DListener, SelectedListener
{
   public static final double MIN_FIELD_OF_VIEW = 0.001;
   public static final double MAX_FIELD_OF_VIEW = 2.0;

   private static final double MIN_CAMERA_POSITION_TO_FIX_DISTANCE = 0.1; // 0.8;

   public final static double CAMERA_START_X = 0.0;
   public final static double CAMERA_START_Y = -6.0;
   public final static double CAMERA_START_Z = 1.0;

   private double fieldOfView = CameraConfiguration.DEFAULT_FIELD_OF_VIEW;
   private double clipDistanceNear = CameraConfiguration.DEFAULT_CLIP_DISTANCE_NEAR;
   private double clipDistanceFar = CameraConfiguration.DEFAULT_CLIP_DISTANCE_FAR;

   private double camX, camY, camZ, fixX, fixY, fixZ;

   private double zoom_factor = 1.0;
   private double rotate_factor = 1.0;
   private double rotate_camera_factor = 1.0;

   private boolean isMounted = false;
   private CameraMountInterface cameraMount;

   private boolean isTracking = true, isTrackingX = true, isTrackingY = true, isTrackingZ = false;
   private boolean isDolly = false, isDollyX = true, isDollyY = true, isDollyZ = false;
   private double trackDX = 0.0, trackDY = 0.0, trackDZ = 0.0;
   private double dollyDX = 2.0, dollyDY = 12.0, dollyDZ = 0.0;

   private ViewportAdapter viewportAdapter;
   private JFrame jFrame;

   // Flying
   private boolean fly = true;
   private boolean forward = false;
   private boolean backward = false;
   private boolean left = false;
   private boolean right = false;
   private boolean up = false;
   private boolean down = false;

   private ArrayList<Point3D> storedCameraPositions = new ArrayList<Point3D>(0);
   private ArrayList<Point3D> storedFixPositions = new ArrayList<Point3D>(0);
   private int storedPositionIndex = 0;
   private boolean transitioning = false;

   private static double transitionTime = 500;
   private double camXSpeed;
   private double camYSpeed;
   private double camZSpeed;
   private double fixXSpeed;
   private double fixYSpeed;
   private double fixZSpeed;
   private long lastTransitionTime;

   private ArrayList<Point3D> keyFrameCamPos = new ArrayList<Point3D>(0);
   private ArrayList<Point3D> keyFrameFixPos = new ArrayList<Point3D>(0);
   private ArrayList<Integer> keyFrameTimes = new ArrayList<Integer>(0);
   
   private Graphics3DNode fixPointNode = new Graphics3DNode("cameraFixPoint", new Graphics3DObject(new Sphere3d(0.01), YoAppearance.RGBColor(1.0, 0.0, 0.0, 0.5)));

   private boolean toggleCameraKeyPoints = false;
   private int cameraKeyPointIndex;
   private ArrayList<Integer> cameraKeyPoints = new ArrayList<Integer>(0);

   private CameraTrackingAndDollyPositionHolder cameraTrackAndDollyVariablesHolder;

   private Graphics3DAdapter graphics3dAdapter;

   public static ClassicCameraController createClassicCameraControllerAndAddListeners(ViewportAdapter viewportAdapter, CameraTrackingAndDollyPositionHolder cameraTrackAndDollyVariablesHolder, Graphics3DAdapter graphics3dAdapter)
   {
      return createClassicCameraControllerAndAddListeners(viewportAdapter, cameraTrackAndDollyVariablesHolder, graphics3dAdapter, null);
   }
   
   public static ClassicCameraController createClassicCameraControllerAndAddListeners(ViewportAdapter viewportAdapter, CameraTrackingAndDollyPositionHolder cameraTrackAndDollyVariablesHolder, Graphics3DAdapter graphics3dAdapter, JFrame jFrame)
   {
      ClassicCameraController classicCameraController = new ClassicCameraController(graphics3dAdapter, viewportAdapter, cameraTrackAndDollyVariablesHolder, jFrame);
      graphics3dAdapter.addKeyListener(classicCameraController);
      graphics3dAdapter.addMouseListener(classicCameraController);
      graphics3dAdapter.addMouse3DListener(classicCameraController);
      graphics3dAdapter.addSelectedListener(classicCameraController);

      return classicCameraController;
   }

   public ClassicCameraController(Graphics3DAdapter graphics3dAdapter, ViewportAdapter viewportAdapter, CameraTrackingAndDollyPositionHolder cameraTrackAndDollyVariablesHolder)
   {
      this(graphics3dAdapter, viewportAdapter, cameraTrackAndDollyVariablesHolder, null);
   }
   
   public ClassicCameraController(Graphics3DAdapter graphics3dAdapter, ViewportAdapter viewportAdapter, CameraTrackingAndDollyPositionHolder cameraTrackAndDollyVariablesHolder, JFrame jFrame)
   {
      if (graphics3dAdapter == null) throw new RuntimeException("graphics3dAdapter == null");
      this.graphics3dAdapter = graphics3dAdapter;
      this.viewportAdapter = viewportAdapter;
      this.jFrame = jFrame;

      this.camX = CAMERA_START_X;
      this.camY = CAMERA_START_Y;
      this.camZ = CAMERA_START_Z;
      this.fixX = 0.0;
      this.fixY = 0.0;
      this.fixZ = 0.6;

      this.cameraTrackAndDollyVariablesHolder = cameraTrackAndDollyVariablesHolder;

      // Don't do this stuff by default
      setTracking(false);
      setDolly(false);
   }

   public void setCameraMount(CameraMountInterface mount)
   {
      this.cameraMount = mount;
   }

   public CameraMountInterface getCameraMount()
   {
      return cameraMount;
   }

   public boolean isMounted()
   {
      return isMounted;
   }

   @Override
   public CameraTrackingAndDollyPositionHolder getCameraTrackAndDollyVariablesHolder()
   {
      return cameraTrackAndDollyVariablesHolder;
   }

   @Override
   public void setConfiguration(CameraConfiguration config, CameraMountList mountList)
   {
      if (config == null)
         return;

      this.isMounted = config.isCameraMounted();
      if (isMounted && (mountList != null))
      {
         this.cameraMount = mountList.getCameraMount(config.getCameraMountName());
      }

      this.camX = config.camX;
      this.camY = config.camY;
      this.camZ = config.camZ;
      this.fixX = config.fixX;
      this.fixY = config.fixY;
      this.fixZ = config.fixZ;

      this.isTracking = config.isTracking;
      this.isTrackingX = config.isTrackingX;
      this.isTrackingY = config.isTrackingY;
      this.isTrackingZ = config.isTrackingZ;
      this.isDolly = config.isDolly;
      this.isDollyX = config.isDollyX;
      this.isDollyY = config.isDollyY;
      this.isDollyZ = config.isDollyZ;

      this.trackDX = config.trackDX;
      this.trackDY = config.trackDY;
      this.trackDZ = config.trackDZ;
      this.dollyDX = config.dollyDX;
      this.dollyDY = config.dollyDY;
      this.dollyDZ = config.dollyDZ;

      setFieldOfView(config.fieldOfView);
      this.clipDistanceFar = config.clipDistanceFar;
      this.clipDistanceNear = config.clipDistanceNear;

      // this.update();
   }

   @Override
   public boolean isTracking()
   {
      return isTracking;
   }

   @Override
   public boolean isTrackingX()
   {
      return isTrackingX;
   }

   @Override
   public boolean isTrackingY()
   {
      return isTrackingY;
   }

   @Override
   public boolean isTrackingZ()
   {
      return isTrackingZ;
   }

   @Override
   public boolean isDolly()
   {
      return isDolly;
   }

   @Override
   public boolean isDollyX()
   {
      return isDollyX;
   }

   @Override
   public boolean isDollyY()
   {
      return isDollyY;
   }

   @Override
   public boolean isDollyZ()
   {
      return isDollyZ;
   }

   @Override
   public void setTracking(boolean track, boolean trackX, boolean trackY, boolean trackZ)
   {
      setTracking(track);
      setTrackingX(trackX);
      setTrackingY(trackY);
      setTrackingZ(trackZ);
   }

   @Override
   public void setDolly(boolean dolly, boolean dollyX, boolean dollyY, boolean dollyZ)
   {
      setDolly(dolly);
      setDollyX(dollyX);
      setDollyY(dollyY);
      setDollyZ(dollyZ);
   }

   @Override
   public void setTrackingOffsets(double dx, double dy, double dz)
   {
      trackDX = dx;
      trackDY = dy;
      trackDZ = dz;
   }

   @Override
   public void setDollyOffsets(double dx, double dy, double dz)
   {
      dollyDX = dx;
      dollyDY = dy;
      dollyDZ = dz;
   }

   @Override
   public void setTracking(boolean track)
   {
      isTracking = track;
   }

   @Override
   public void setTrackingX(boolean trackX)
   {
      isTrackingX = trackX;
   }

   @Override
   public void setTrackingY(boolean trackY)
   {
      isTrackingY = trackY;
   }

   @Override
   public void setTrackingZ(boolean trackZ)
   {
      isTrackingZ = trackZ;
   }

   @Override
   public void setDolly(boolean dolly)
   {
      isDolly = dolly;
   }

   @Override
   public void setDollyX(boolean dollyX)
   {
      isDollyX = dollyX;
   }

   @Override
   public void setDollyY(boolean dollyY)
   {
      isDollyY = dollyY;
   }

   @Override
   public void setDollyZ(boolean dollyZ)
   {
      isDollyZ = dollyZ;
   }

   @Override
   public double getTrackingXOffset()
   {
      return trackDX;
   }

   @Override
   public double getTrackingYOffset()
   {
      return trackDY;
   }

   @Override
   public double getTrackingZOffset()
   {
      return trackDZ;
   }

   @Override
   public double getDollyXOffset()
   {
      return dollyDX;
   }

   @Override
   public double getDollyYOffset()
   {
      return dollyDY;
   }

   @Override
   public double getDollyZOffset()
   {
      return dollyDZ;
   }

   @Override
   public void setTrackingXOffset(double dx)
   {
      trackDX = dx;
   }

   @Override
   public void setTrackingYOffset(double dy)
   {
      trackDY = dy;
   }

   @Override
   public void setTrackingZOffset(double dz)
   {
      trackDZ = dz;
   }

   @Override
   public void setDollyXOffset(double dx)
   {
      dollyDX = dx;
   }

   @Override
   public void setDollyYOffset(double dy)
   {
      dollyDY = dy;
   }

   @Override
   public void setDollyZOffset(double dz)
   {
      dollyDZ = dz;
   }

   @Override
   public void update()
   {
      if (graphics3dAdapter.getContextManager().getCurrentViewport() != viewportAdapter)
      {
         forward = false;
         backward = false;
         left = false;
         right = false;
         up = false;
         down = false;
      }

      if (isTracking)
      {
         if (isTrackingX)
         {
            double trackX = cameraTrackAndDollyVariablesHolder.getTrackingX();
            if (!Double.isNaN(trackX))
               fixX = trackX + trackDX;
         }

         if (isTrackingY)
         {
            double trackY = cameraTrackAndDollyVariablesHolder.getTrackingY();
            if (!Double.isNaN(trackY))
               fixY = trackY + trackDY;
         }

         if (isTrackingZ)
         {
            double trackZ = cameraTrackAndDollyVariablesHolder.getTrackingZ();
            if (!Double.isNaN(trackZ))
               fixZ = trackZ + trackDZ;
         }
      }

      if (isDolly)
      {
         double dollyX = cameraTrackAndDollyVariablesHolder.getDollyX();
         if (isDollyX)
         {
            if (!Double.isNaN(dollyX))
               camX = dollyX + dollyDX;
         }

         if (isDollyY)
         {
            double dollyY = cameraTrackAndDollyVariablesHolder.getDollyY();
            if (!Double.isNaN(dollyY))
               camY = dollyY + dollyDY;
         }

         if (isDollyZ)
         {
            double dollyZ = cameraTrackAndDollyVariablesHolder.getDollyZ();
            if (!Double.isNaN(dollyZ))
               camZ = dollyZ + dollyDZ;
         }
      }

      double fieldOfView = cameraTrackAndDollyVariablesHolder.getFieldOfView();
      if (!Double.isNaN(fieldOfView))
         setFieldOfView(fieldOfView);

      // Flying
      if (fly && !isTracking && !isDolly && !transitioning)
      {
         if (forward)
         {
            moveCameraForward(-0.5);
         }

         if (backward)
         {
            moveCameraForward(0.5);
         }

         if (left)
         {
            pan(20, 0);
         }

         if (right)
         {
            pan(-20, 0);
         }

         if (up)
         {
            pan(00, 20);
         }

         if (down)
         {
            pan(00, -20);
         }
      }

      // End Flying

      if (transitioning && !isTracking && !isDolly)
      {
         int numberOfDimensionsThatHaveTransitioned = 0;
         double elapsedTransitionTime = System.currentTimeMillis() - lastTransitionTime;
         lastTransitionTime = System.currentTimeMillis();

         if (Math.abs(camX - storedCameraPositions.get(storedPositionIndex).getX()) <= Math.abs(camXSpeed * elapsedTransitionTime))
         {
            camX = storedCameraPositions.get(storedPositionIndex).getX();
            numberOfDimensionsThatHaveTransitioned++;
         }
         else
         {
            camX += camXSpeed * elapsedTransitionTime;
         }

         if (Math.abs(camY - storedCameraPositions.get(storedPositionIndex).getY()) <= Math.abs(camYSpeed * elapsedTransitionTime))
         {
            camY = storedCameraPositions.get(storedPositionIndex).getY();
            numberOfDimensionsThatHaveTransitioned++;
         }
         else
         {
            camY += camYSpeed * elapsedTransitionTime;
         }

         if (Math.abs(camZ - storedCameraPositions.get(storedPositionIndex).getZ()) <= Math.abs(camZSpeed * elapsedTransitionTime))
         {
            camZ = storedCameraPositions.get(storedPositionIndex).getZ();
            numberOfDimensionsThatHaveTransitioned++;
         }
         else
         {
            camZ += camZSpeed * elapsedTransitionTime;
         }

         if (Math.abs(fixX - storedFixPositions.get(storedPositionIndex).getX()) <= Math.abs(fixXSpeed * elapsedTransitionTime))
         {
            fixX = storedFixPositions.get(storedPositionIndex).getX();
            numberOfDimensionsThatHaveTransitioned++;
         }
         else
         {
            fixX += fixXSpeed * elapsedTransitionTime;
         }

         if (Math.abs(fixY - storedFixPositions.get(storedPositionIndex).getY()) <= Math.abs(fixYSpeed * elapsedTransitionTime))
         {
            fixY = storedFixPositions.get(storedPositionIndex).getY();
            numberOfDimensionsThatHaveTransitioned++;
         }
         else
         {
            fixY += fixYSpeed * elapsedTransitionTime;
         }

         if (Math.abs(fixZ - storedFixPositions.get(storedPositionIndex).getZ()) <= Math.abs(fixZSpeed * elapsedTransitionTime))
         {
            fixZ = storedFixPositions.get(storedPositionIndex).getZ();
            numberOfDimensionsThatHaveTransitioned++;
         }
         else
         {
            fixZ += fixZSpeed * elapsedTransitionTime;
         }

         if (numberOfDimensionsThatHaveTransitioned == 6)
         {
            transitioning = false;
         }
      }
   }

   public void addKeyFrame(int time)
   {
      addKeyFrame(keyFrameCamPos.size(), time);
   }

   public void addKeyFrame(int i, int time)
   {
      keyFrameCamPos.add(i, new Point3D(camX, camY, camZ));
      keyFrameFixPos.add(i, new Point3D(fixX, fixY, fixZ));
      keyFrameTimes.add(i, time);
   }

   public int removeKeyFrameByTime(int time)
   {
      for (int i = 0; i < keyFrameTimes.size(); i++)
      {
         if (keyFrameTimes.get(i) == time)
         {
            removeKeyFrameByIndex(i);

            return i;
         }
      }

      return -1;
   }

   public void removeKeyFrameByIndex(int i)
   {
      if ((i >= 0) && (i < keyFrameTimes.size()))
      {
         keyFrameTimes.remove(i);
         keyFrameCamPos.remove(i);
         keyFrameFixPos.remove(i);
      }
   }

   @Override
   public void setKeyFrameTime(int time)
   {
      for (int i = keyFrameTimes.size() - 1; i >= 0; i--)
      {
         if (time >= keyFrameTimes.get(i))
         {
            if (keyFrameTimes.size() > i + 1)
            {
               double elapsedTime = time - keyFrameTimes.get(i);
               double totalTime = keyFrameTimes.get(i + 1) - keyFrameTimes.get(i);
               camX = keyFrameCamPos.get(i).getX() + (keyFrameCamPos.get(i + 1).getX() - keyFrameCamPos.get(i).getX()) * elapsedTime / totalTime;
               camY = keyFrameCamPos.get(i).getY() + (keyFrameCamPos.get(i + 1).getY() - keyFrameCamPos.get(i).getY()) * elapsedTime / totalTime;
               camZ = keyFrameCamPos.get(i).getZ() + (keyFrameCamPos.get(i + 1).getZ() - keyFrameCamPos.get(i).getZ()) * elapsedTime / totalTime;

               fixX = keyFrameFixPos.get(i).getX() + (keyFrameFixPos.get(i + 1).getX() - keyFrameFixPos.get(i).getX()) * elapsedTime / totalTime;
               fixY = keyFrameFixPos.get(i).getY() + (keyFrameFixPos.get(i + 1).getY() - keyFrameFixPos.get(i).getY()) * elapsedTime / totalTime;
               fixZ = keyFrameFixPos.get(i).getZ() + (keyFrameFixPos.get(i + 1).getZ() - keyFrameFixPos.get(i).getZ()) * elapsedTime / totalTime;
            }

            break;
         }
      }
   }

   public void gotoKey(int index)
   {
      if ((index >= 0) && (index < keyFrameCamPos.size()))
      {
         storedPositionIndex = index;
         camX = keyFrameCamPos.get(index).getX();
         camY = keyFrameCamPos.get(index).getY();
         camZ = keyFrameCamPos.get(index).getZ();

         fixX = keyFrameFixPos.get(index).getX();
         fixY = keyFrameFixPos.get(index).getY();
         fixZ = keyFrameFixPos.get(index).getZ();
      }
   }

   public ArrayList<Integer> getCameraKeyPoints()
   {
      return cameraKeyPoints;
   }

   @Override
   public double getFixX()
   {
      return this.fixX;
   }

   @Override
   public double getFixY()
   {
      return this.fixY;
   }

   @Override
   public double getFixZ()
   {
      return this.fixZ;
   }

   @Override
   public double getCamX()
   {
      return this.camX;
   }

   @Override
   public double getCamY()
   {
      return this.camY;
   }

   @Override
   public double getCamZ()
   {
      return this.camZ;
   }

   @Override
   public void setFixX(double fx)
   {
      this.fixX = fx;
   }

   @Override
   public void setFixY(double fy)
   {
      this.fixY = fy;
   }

   @Override
   public void setFixZ(double fz)
   {
      this.fixZ = fz;
   }

   @Override
   public void setCamX(double cx)
   {
      this.camX = cx;
   }

   @Override
   public void setCamY(double cy)
   {
      this.camY = cy;
   }

   @Override
   public void setCamZ(double cz)
   {
      this.camZ = cz;
   }

   @Override
   public void setFixPosition(double fx, double fy, double fz)
   {
      this.fixX = fx;
      this.fixY = fy;
      this.fixZ = fz;
   }

   @Override
   public void setCameraPosition(double cx, double cy, double cz)
   {
      this.camX = cx;
      this.camY = cy;
      this.camZ = cz;
   }

   private Vector3D v3d = new Vector3D();
   private RigidBodyTransform t3d = new RigidBodyTransform();
   private Vector3D rotVector = new Vector3D();
   private AxisAngle rotAxisAngle4d = new AxisAngle();

   public void doMouseDraggedLeft(double dx, double dy)
   {
      // Rotate around fix point:

      double delX0 = camX - fixX, delY0 = camY - fixY, delZ0 = camZ - fixZ;
      v3d.set(delX0, delY0, delZ0);

      // double offsetDistance = v3d.length();

      t3d.setRotationYawAndZeroTranslation(-dx * rotate_factor);
      t3d.transform(v3d);

      if (!isDolly || (!isDollyX && !isDollyY))
      {
         camX = v3d.getX() + fixX;
         camY = v3d.getY() + fixY;
      }

      delX0 = camX - fixX;
      delY0 = camY - fixY;
      delZ0 = camZ - fixZ;

      // v3d.set(delX0, delY0, delZ0);
      rotVector.cross(new Vector3D(0.0, 0.0, -1.0), v3d);
      rotAxisAngle4d.set(rotVector, dy * rotate_factor / 4.0);

      t3d.setRotationAndZeroTranslation(rotAxisAngle4d);
      t3d.transform(v3d);

      if ((v3d.getX() * delX0 > 0.0) && (v3d.getY() * delY0 > 0.0))
      {
         if (!isDolly || (!isDollyX && !isDollyY))
         {
            camX = v3d.getX() + fixX;
            camY = v3d.getY() + fixY;
         }

         if (!isDolly || !isDollyZ)
         {
            camZ = v3d.getZ() + fixZ;

            /*
             * double factor = elevate_factor * Math.abs(offsetDistance);
             * //camZ-fixZ); if (factor < elevate_factor) factor =
             * elevate_factor; //camZ = v3d.z + fixZ + dy * elevate_factor; camZ
             * = v3d.z + fixZ + dy * factor;
             */
         }
      }

      // transformChanged(currXform);
   }
   
   public void rotateAroundFix(double dx, double dy)
   {
      double distanceFromCameraToFix = Math.sqrt(Math.pow(camX - fixX, 2) + Math.pow(camY - fixY, 2) + Math.pow(camZ - fixZ, 2));
      
      if (distanceFromCameraToFix > 1.0)
      {
         dx /= distanceFromCameraToFix;
         dy /= distanceFromCameraToFix;
      }
      
      double delX0 = camX - fixX, delY0 = camY - fixY, delZ0 = camZ - fixZ;
      v3d.set(delX0, delY0, delZ0);

      t3d.setRotationYawAndZeroTranslation(-dx * rotate_factor);
      t3d.transform(v3d);

      if (!isDolly || (!isDollyX && !isDollyY))
      {
         camX = v3d.getX() + fixX;
         camY = v3d.getY() + fixY;
      }

      delX0 = camX - fixX;
      delY0 = camY - fixY;
      delZ0 = camZ - fixZ;

      rotVector.cross(new Vector3D(0.0, 0.0, -1.0), v3d);
      rotAxisAngle4d.set(rotVector, dy * rotate_factor / 4.0);

      t3d.setRotationAndZeroTranslation(rotAxisAngle4d);
      t3d.transform(v3d);

      if ((v3d.getX() * delX0 > 0.0) && (v3d.getY() * delY0 > 0.0))
      {
         if (!isDolly || (!isDollyX && !isDollyY))
         {
            camX = v3d.getX() + fixX;
            camY = v3d.getY() + fixY;
         }

         if (!isDolly || !isDollyZ)
         {
            camZ = v3d.getZ() + fixZ;
         }
      }
   }

   public void doMouseDraggedRight(double dx, double dy)
   {
      // Elevate up and down
      double delX0 = camX - fixX, delY0 = camY - fixY, delZ0 = camZ - fixZ;
      v3d.set(delX0, delY0, delZ0);

      // double offsetDistance = v3d.length();

      t3d.setRotationYawAndZeroTranslation(-dx * rotate_camera_factor);
      t3d.transform(v3d);

      if (!isTracking || (!isTrackingX && !isTrackingY))
      {
         fixX = camX - v3d.getX();
         fixY = camY - v3d.getY();
      }

      delX0 = camX - fixX;
      delY0 = camY - fixY;
      delZ0 = camZ - fixZ;

      // v3d.set(delX0, delY0, delZ0);

      rotVector.set(-1.0, 0.0, 0.0);
      rotVector.cross(new Vector3D(0.0, 0.0, -1.0), v3d);
      rotAxisAngle4d.set(rotVector, dy * rotate_camera_factor / 4.0);

      t3d.setRotationAndZeroTranslation(rotAxisAngle4d);
      t3d.transform(v3d);

      if ((v3d.getX() * delX0 > 0.0) && (v3d.getY() * delY0 > 0.0))
      {
         if (!isTracking || (!isTrackingX && !isTrackingY))
         {
            fixX = camX - v3d.getX();
            fixY = camY - v3d.getY();
         }

         if (!isTracking || !isTrackingZ)
         {
            fixZ = camZ - v3d.getZ();

            /*
             * double factor = elevate_camera_factor * offsetDistance;
             * //Math.abs(camZ-fixZ); if (factor < elevate_camera_factor) factor
             * = elevate_camera_factor; fixZ = camZ - v3d.z + dy factor; //fixZ
             * = camZ - v3d.z + dy * elevate_factor;
             */
         }
      }

      // transformChanged(currXform);
   }

   @Override
   public void setFieldOfView(double fov)
   {
      fieldOfView = fov;

      if (fieldOfView < MIN_FIELD_OF_VIEW)
         fieldOfView = MIN_FIELD_OF_VIEW;
      if (fieldOfView > MAX_FIELD_OF_VIEW)
         fieldOfView = MAX_FIELD_OF_VIEW;
   }

   public void doMouseDraggedMiddle(double dx, double dy)
   {
      // Zooms in and out

      if ((this.isMounted) && (viewportAdapter != null))
      {
         cameraMount.zoom(dy * 0.1);
      }

      else
      {
         Vector3D v3d = new Vector3D(camX - fixX, camY - fixY, camZ - fixZ);

         Vector3D offsetVec = new Vector3D(v3d);

         // offsetVec.normalize();
         offsetVec.scale(dy * zoom_factor);

         // if (offsetVec.length() < v3d.length())
         // {
         if (!isDolly || (!isDollyX && !isDollyY))
         {
            camX += offsetVec.getX();
            camY += offsetVec.getY();
         }

         if (!isDolly || !isDollyZ)
            camZ += offsetVec.getZ();

         // }

         v3d.set(camX - fixX, camY - fixY, camZ - fixZ);

         if (v3d.length() < MIN_CAMERA_POSITION_TO_FIX_DISTANCE)
         {
            v3d.normalize();
            v3d.scale(MIN_CAMERA_POSITION_TO_FIX_DISTANCE);
            camX = v3d.getX() + fixX;
            camY = v3d.getY() + fixY;
            camZ = v3d.getZ() + fixZ;
         }
      }

      // transformChanged(currXform);
   }

   private void moveCameraForward(double distance)
   {
      double angleXY = Math.atan2(camY - fixY, camX - fixX);
      double angleZ = Math.atan2(camZ - fixZ, Math.hypot(camY - fixY, camX - fixX));

      double distXY = distance * Math.cos(angleZ);

      camX += distXY * Math.cos(angleXY);
      camY += distXY * Math.sin(angleXY);
      camZ += distance * Math.sin(angleZ);

      if (Math.sqrt(Math.pow(camX - fixX, 2) + Math.pow(camY - fixY, 2) + Math.pow(camY - fixY, 2)) < 1)
      {
         fixX += distXY * Math.cos(angleXY);
         fixY += distXY * Math.sin(angleXY);
         fixZ += distance * Math.sin(angleZ);
      }

      // Vector3d v3d = new Vector3d(camX - fixX, camY - fixY, camZ - fixZ);
      //
      // Vector3d offsetVec = new Vector3d(v3d);
      //
      //// offsetVec.normalize();
      // offsetVec.scale(distance * zoom_factor);
      //
      //// if (offsetVec.length() < v3d.length())
      //// {
      // if (!isDolly || (!isDollyX &&!isDollyY))
      // {
      // camX += offsetVec.x;
      // camY += offsetVec.y;
      // }
      //
      // if (!isDolly ||!isDollyZ)
      // camZ += offsetVec.z;
   }

   public void pan(double dx, double dy)
   {
      double distanceFromCameraToFix = Math.sqrt(Math.pow(camX - fixX, 2) + Math.pow(camY - fixY, 2) + Math.pow(camZ - fixZ, 2));
      dx *= distanceFromCameraToFix / viewportAdapter.getPhysicalWidth() * .00023;
      dy *= distanceFromCameraToFix / viewportAdapter.getPhysicalHeight() * .00007;
      double theta = Math.PI / 2 + Math.atan2((camZ - fixZ), Math.hypot(camX - fixX, camY - fixY));
      if (!isTracking || !isTrackingZ)
      {
         camZ += dy * Math.sin(theta);
         fixZ += dy * Math.sin(theta);
      }

      double d = dy * Math.cos(theta);
      theta = Math.atan2(camY - fixY, camX - fixX);

      if (!isTracking || !isTrackingY)
      {
         camY += d * Math.sin(theta);
         fixY += d * Math.sin(theta);
      }

      if (!isTracking || !isTrackingX)
      {
         camX += d * Math.cos(theta);
         fixX += d * Math.cos(theta);
      }

      theta = Math.PI / 2 + Math.atan2(camY - fixY, camX - fixX);

      if (!isTracking || !isTrackingY)
      {
         camY -= dx * Math.sin(theta);
         fixY -= dx * Math.sin(theta);
      }

      if (!isTracking || !isTrackingX)
      {
         camX -= dx * Math.cos(theta);
         fixX -= dx * Math.cos(theta);
      }
   }
   
   public void translateFix(double dx, double dy, double dz)
   {
//      double zTiltAngle = Math.PI / 2 + Math.atan2(camZ - fixZ, Math.hypot(camX - fixX, camY - fixY));

//      double ky = dy * Math.sin(zTiltAngle);
//      double kz = dz * Math.cos(zTiltAngle);
      
      double yTiltAngle = Math.atan2(camY - fixY, camX - fixX);
      
      if (!isTracking || !isTrackingZ)
      {
         camZ += dz;
         fixZ += dz;
      }

      if (!isTracking || !isTrackingY)
      {
         camY += dy * Math.sin(yTiltAngle);
         fixY += dy * Math.sin(yTiltAngle);
      }

      if (!isTracking || !isTrackingX)
      {
         camX += dy * Math.cos(yTiltAngle);
         fixX += dy * Math.cos(yTiltAngle);
      }

      double xTiltAngle = yTiltAngle + Math.PI / 2;

      if (!isTracking || !isTrackingY)
      {
         camY -= dx * Math.sin(xTiltAngle);
         fixY -= dx * Math.sin(xTiltAngle);
      }

      if (!isTracking || !isTrackingX)
      {
         camX -= dx * Math.cos(xTiltAngle);
         fixX -= dx * Math.cos(xTiltAngle);
      }
   }

   private void initTransition()
   {
      camXSpeed = -(camX - storedCameraPositions.get(storedPositionIndex).getX()) / transitionTime;
      camYSpeed = -(camY - storedCameraPositions.get(storedPositionIndex).getY()) / transitionTime;
      camZSpeed = -(camZ - storedCameraPositions.get(storedPositionIndex).getZ()) / transitionTime;

      fixXSpeed = -(fixX - storedFixPositions.get(storedPositionIndex).getX()) / transitionTime;
      fixYSpeed = -(fixY - storedFixPositions.get(storedPositionIndex).getY()) / transitionTime;
      fixZSpeed = -(fixZ - storedFixPositions.get(storedPositionIndex).getZ()) / transitionTime;

      transitioning = true;
      lastTransitionTime = System.currentTimeMillis();
   }

   @Override
   public void toggleCameraKeyMode()
   {
      setUseCameraKeyPoints(!useKeyCameraPoints());
   }

   public boolean getCameraKeyMode()
   {
      return toggleCameraKeyPoints;
   }

   @Override
   public void setUseCameraKeyPoints(boolean use)
   {
      toggleCameraKeyPoints = use;
   }

   @Override
   public boolean useKeyCameraPoints()
   {
      return toggleCameraKeyPoints;
   }

   @Override
   public boolean setCameraKeyPoint(int time)
   {
      boolean added = false;
      for (int i = 0; i < cameraKeyPoints.size(); i++)
      {
         if (cameraKeyPoints.get(i) == time)
         {
            addKeyFrame(removeKeyFrameByTime(time), time);

            return false;
         }

         if (cameraKeyPoints.get(i) > time)
         {
            cameraKeyPoints.add(time);
            addKeyFrame(1, time);

            return true;
         }
      }

      if (!added)
      {
         cameraKeyPoints.add(time);
         addKeyFrame(time);
      }

      return true;
   }

   @Override
   public void nextCameraKeyPoint(int time)
   {
      // int closestLesserTime = Integer.MIN_VALUE;
      // int index = -1;
      // for (int i = 0; i < cameraKeyPoints.size(); i++)
      // {
      // if (cameraKeyPoints.get(i) < time && cameraKeyPoints.get(i) > closestLesserTime)
      // {
      // index = i;
      // closestLesserTime = cameraKeyPoints.get(i);
      // }
      // }
      // if (index != -1)
      // {
      // camera.initTransition(index);
      // }
      cameraKeyPointIndex++;

      if (cameraKeyPointIndex >= cameraKeyPoints.size())
      {
         cameraKeyPointIndex = 0;
      }

      toggleCameraKeyPoints = false;
      gotoKey(cameraKeyPointIndex);
   }

   @Override
   public void previousCameraKeyPoint(int time)
   {
      cameraKeyPointIndex--;

      if (cameraKeyPointIndex < 0)
      {
         cameraKeyPointIndex = cameraKeyPoints.size() - 1;
      }

      toggleCameraKeyPoints = false;
      gotoKey(cameraKeyPointIndex);
   }

   @Override
   public void removeCameraKeyPoint(int time)
   {
      cameraKeyPoints.remove(cameraKeyPointIndex);
      removeKeyFrameByIndex(cameraKeyPointIndex);
   }

   @Override
   public double getTrackXVar()
   {
      return cameraTrackAndDollyVariablesHolder.getTrackingX();
   }

   @Override
   public double getTrackYVar()
   {
      return cameraTrackAndDollyVariablesHolder.getTrackingY();
   }

   @Override
   public double getTrackZVar()
   {
      return cameraTrackAndDollyVariablesHolder.getTrackingZ();
   }

   @Override
   public double getDollyXVar()
   {
      return cameraTrackAndDollyVariablesHolder.getDollyX();
   }

   @Override
   public double getDollyYVar()
   {
      return cameraTrackAndDollyVariablesHolder.getDollyY();
   }

   @Override
   public double getDollyZVar()
   {
      return cameraTrackAndDollyVariablesHolder.getDollyZ();
   }

   public void nextStoredPosition()
   {
      if (storedCameraPositions.size() > 0)
      {
         storedPositionIndex++;

         if (storedPositionIndex >= storedCameraPositions.size())
         {
            storedPositionIndex = 0;
         }

         initTransition();
      }
   }

   public void previousStoredPosition()
   {
      if (storedCameraPositions.size() > 0)
      {
         storedPositionIndex--;

         if (storedPositionIndex < 0)
         {
            storedPositionIndex = storedCameraPositions.size() - 1;
         }

         initTransition();
      }
   }

   public void storePosition()
   {
      storedCameraPositions.add(new Point3D(getCamX(), getCamY(), getCamZ()));
      storedFixPositions.add(new Point3D(getFixX(), getFixY(), getFixZ()));
   }

   @Override
   public void reset()
   {
      
   }

   private RotationMatrix rotationMatrix = new RotationMatrix();
   private Vector3D positionOffset = new Vector3D();

   private Vector3D zAxis = new Vector3D(), yAxis = new Vector3D(), xAxis = new Vector3D();

   @Override
   public void computeTransform(RigidBodyTransform currXform)
   {
      update();
      CameraMountInterface cameraMount = getCameraMount();
      if (isMounted() && (cameraMount != null))
      {
         cameraMount.getTransformToCamera(currXform);

         return;
      }

      positionOffset.set(getCamX(), getCamY(), getCamZ());
      xAxis.set(getFixX(), getFixY(), getFixZ());
      
      fixPointNode.translateTo(getFixX(), getFixY(), getFixZ());

      xAxis.sub(positionOffset);
      xAxis.normalize();
      zAxis.set(0.0, 0.0, 1.0);
      yAxis.cross(zAxis, xAxis);
      zAxis.cross(xAxis, yAxis);

      rotationMatrix.setColumns(xAxis, yAxis, zAxis);

      currXform.setRotationAndZeroTranslation(rotationMatrix);
      currXform.setTranslation(positionOffset);
      currXform.normalizeRotationPart();
   }
   
   private boolean shouldAcceptDeviceInput()
   {
      if (alreadyClosing || graphics3dAdapter.getContextManager().getCurrentViewport() != viewportAdapter)
         return false;
      
      if (jFrame != null && !jFrame.isActive())
         return false;
      
      return true;
   }

   @Override
   public void keyPressed(Key key)
   {
      if (shouldAcceptDeviceInput())
      {
         switch (key)
         {
         case W:
            forward = true;

            break;

         case S:
            backward = true;

            break;

         case A:
            left = true;

            break;

         case D:
            right = true;

            break;

         case Q:
            up = true;

            break;

         case Z:
            down = true;

            break;
            
         default:
            
            break;
         }
      }
   }

   @Override
   public void keyReleased(Key key)
   {
      if (shouldAcceptDeviceInput())
      {
         switch (key)
         {
         case W:
            forward = false;

            break;

         case S:
            backward = false;

            break;

         case A:
            left = false;

            break;

         case D:
            right = false;

            break;

         case Q:
            up = false;

            break;

         case Z:
            down = false;

            break;

         case RIGHT:
            nextStoredPosition();

            break;

         case LEFT:
            previousStoredPosition();

            break;

         case K:
            storePosition();

            break;
         default:

            break;
         }
      }
   }

   @Override
   public void selected(Graphics3DNode graphics3dNode, ModifierKeyInterface modifierKeyInterface, Point3DReadOnly location, Point3DReadOnly cameraLocation,
                        QuaternionReadOnly cameraRotation)
   {
      if (shouldAcceptDeviceInput())
      {
         if (modifierKeyInterface.isKeyPressed(Key.SHIFT))
         {
            if (!isTracking() || !isTrackingX())
               setFixX(location.getX());
            if (!isTracking() || !isTrackingY())
               setFixY(location.getY());
            if (!isTracking() || !isTrackingZ())
               setFixZ(location.getZ());
         }
      }
   }

   @Override
   public void mouseDragged(MouseButton mouseButton, double dx, double dy)
   {
      if (shouldAcceptDeviceInput())
      {
         switch (mouseButton)
         {
         case LEFT:
            doMouseDraggedLeft(dx, dy);

            break;

         case RIGHT:
            doMouseDraggedRight(dx, dy);

            break;

         case MIDDLE:
            doMouseDraggedMiddle(dx, dy);

            break;

         case LEFTRIGHT:
            pan(dx, dy);

            break;
         }
      }
   }
   
   private double rotateGain = 0.15;
   private double translateGain = 0.05;

   @Override
   public void mouseDragged(double dx, double dy, double dz, double drx, double dry, double drz)
   {
      if (shouldAcceptDeviceInput())
      {
         //      doMouseDraggedRight(drz, drx);
         //      doMouseDraggedMiddle(0.0, dz);
         //      moveCameraForward(dy);

         rotateAroundFix(-drz * rotateGain, -drx * rotateGain);
         translateFix(-dx * translateGain, -dy * translateGain, dz * translateGain);
      }
   }

   @Override
   public double getClipNear()
   {
      if (isMounted)
      {
         return cameraMount.getClipDistanceNear();
      }
      else
      {
         return clipDistanceNear;
      }
   }

   @Override
   public double getClipFar()
   {
      if (isMounted)
      {
         return cameraMount.getClipDistanceFar();
      }
      else
      {
         return clipDistanceFar;
      }
   }

   @Override
   public double getHorizontalFieldOfViewInRadians()
   {
      if (isMounted)
      {
         return cameraMount.getFieldOfView();
      }
      else
      {
         return fieldOfView;
      }
   }

   @Override
   public void setClipDistanceNear(double near)
   {
      clipDistanceNear = near;
   }

   @Override
   public void setClipDistanceFar(double far)
   {
      clipDistanceFar = far;
   }

   @Override
   public void copyPositionTrackingDollyConfiguration(TrackingDollyCameraController otherCamera)
   {
      setTracking(otherCamera.isTracking(), otherCamera.isTrackingX(), otherCamera.isTrackingY(), otherCamera.isTrackingZ());
      setDolly(otherCamera.isDolly(), otherCamera.isDollyX(), otherCamera.isDollyY(), otherCamera.isDollyZ());

      setCameraPosition(otherCamera.getCamX(), otherCamera.getCamY(), otherCamera.getCamZ());

      setFixPosition(otherCamera.getFixX(), otherCamera.getFixY(), otherCamera.getFixZ());

      setDollyOffsets(otherCamera.getDollyXOffset(), otherCamera.getDollyYOffset(), otherCamera.getDollyZOffset());
      setTrackingOffsets(otherCamera.getTrackingXOffset(), otherCamera.getTrackingYOffset(), otherCamera.getTrackingZOffset());

      if (otherCamera instanceof ClassicCameraController)
      {
         ClassicCameraController classicOtherCamera = (ClassicCameraController) otherCamera;

         keyFrameCamPos = classicOtherCamera.keyFrameCamPos;
         keyFrameFixPos = classicOtherCamera.keyFrameFixPos;
         keyFrameTimes = classicOtherCamera.keyFrameTimes;

         toggleCameraKeyPoints = classicOtherCamera.toggleCameraKeyPoints;
         cameraKeyPointIndex = classicOtherCamera.cameraKeyPointIndex;
         cameraKeyPoints = classicOtherCamera.cameraKeyPoints;

         System.out.println("Copying camera keys");
      }
   }

   private boolean alreadyClosing = false;
   
   @Override
   public void closeAndDispose()
   {
      if (alreadyClosing) return;
      alreadyClosing = true;
      
      this.cameraMount = null;
      this.viewportAdapter = null;
      if (cameraTrackAndDollyVariablesHolder != null)
      {
         cameraTrackAndDollyVariablesHolder.closeAndDispose();
         cameraTrackAndDollyVariablesHolder = null;
      }
      graphics3dAdapter = null;
   }

   public Graphics3DNode getFixPointNode()
   {
      return fixPointNode;
   }
}
