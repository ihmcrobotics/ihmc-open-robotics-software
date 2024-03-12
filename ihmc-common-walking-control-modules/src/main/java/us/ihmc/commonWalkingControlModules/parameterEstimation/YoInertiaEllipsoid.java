package us.ihmc.commonWalkingControlModules.parameterEstimation;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.SingularValueDecomposition3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoComposite.YoColorRGBASingleDefinition;
import us.ihmc.scs2.definition.yoComposite.YoQuaternionDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicEllipsoid3DDefinition;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoInertiaEllipsoid
{
   private final YoVector3D radii;
   private final YoInteger rgba;
   private final YoColorRGBASingleDefinition color;
   private final RigidBodyReadOnly rigidBody;
   private final YoGraphicEllipsoid3DDefinition ellipsoidDefinition;

   /**
    * Creates an ellipsoid visual which corresponds to the inertia of a rigid body. The color and size are backed by YoVariables and can be updated.
    *
    * @param rigidBody - The rigid body for which the inertia ellipsoid should be created.
    * @param registry  - The registry for the ellipsoid.
    */
   public YoInertiaEllipsoid(RigidBodyReadOnly rigidBody, YoRegistry registry)
   {
      this.rigidBody = rigidBody;
      radii = new YoVector3D(rigidBody.getName() + "_Radii", registry);
      rgba = new YoInteger(rigidBody.getName() + "_RGBAInteger", registry);
      color = new YoColorRGBASingleDefinition(rgba.getName());

      // Initialize to some color
      int[] rgbColor = InertiaVisualizationTools.getRGBForCoolwarmColorMap(0.2);
      int opacityAlpha = (int) (0.5 * 255);
      int rgbInt = ColorDefinitions.toRGBA(rgbColor[0], rgbColor[1], rgbColor[2], opacityAlpha);
      this.setColor(rgbInt);

      /* Set up the inertial properties of the YoInertiaEllipsoid and the Ellipsoid3DDefinition */
      RigidBodyDefinition rigidBodyDefinition = convertRigidBodyToRigidBodyDefinition(rigidBody);
      ellipsoidDefinition = convertRobotMassPropertiesToInertiaEllipsoid(rigidBodyDefinition);
   }

   /**
    * Updates the color, size, and location of the ellipsoid.
    * <p>
    * NOTE: Right now only the color update is implemented.
    * </p>
    *
    * @param scale - The scale of the color map. Should be between 0 and 1.
    */
   public void update(double scale)
   {
      int[] rgbColor = InertiaVisualizationTools.getRGBForCoolwarmColorMap(scale);
      int opacityAlpha = (int) (0.6 * 255);
      int rgbInt = ColorDefinitions.toRGBA(rgbColor[0], rgbColor[1], rgbColor[2], opacityAlpha);
      this.setColor(rgbInt);
   }

   /**
    * Converts a RigidBodyDefinition into a YoGraphicEllipsoid3DDefinition. Copies functionality from a similar method in YoGraphicTools. Exposes more of the
    * method so that the color of the inertial ellipsoids can be set individually.
    *
    * @param rigidBodyDefinition - The rigid body definition to convert to an ellipsoid definition.
    */
   private YoGraphicEllipsoid3DDefinition convertRobotMassPropertiesToInertiaEllipsoid(RigidBodyDefinition rigidBodyDefinition)
   {
      // It is up to the user to handle when null is returned outside this method
      if (rigidBodyDefinition.getInertiaPose() == null)
         return null;
      if (rigidBodyDefinition.getMomentOfInertia() == null)
         return null;

      SingularValueDecomposition3D svd = new SingularValueDecomposition3D();
      if (!svd.decompose(rigidBodyDefinition.getMomentOfInertia()))
         return null;

      if (rigidBody.isRootBody())
         return null;

      ReferenceFrame referenceFrame = rigidBody.getParentJoint().getFrameAfterJoint();

      RigidBodyTransform ellipsoidPose = new RigidBodyTransform(rigidBodyDefinition.getInertiaPose());
      ellipsoidPose.appendOrientation(svd.getU());
      Vector3D radii = computeInertiaEllipsoidRadii(svd.getW(), rigidBodyDefinition.getMass());
      setRadii(radii);
      YoGraphicEllipsoid3DDefinition ellipsoid = convertEllipsoid3DDefinition(referenceFrame, ellipsoidPose, new Ellipsoid3DDefinition(this.getRadii()));
      ellipsoid.setName(rigidBody.getName() + " inertia");
      ellipsoid.setColor(color);

      return ellipsoid;
   }

   private static YoGraphicEllipsoid3DDefinition convertEllipsoid3DDefinition(ReferenceFrame referenceFrame,
                                                                              RigidBodyTransformReadOnly originPose,
                                                                              Ellipsoid3DDefinition geometryDefinition)
   {
      YoGraphicEllipsoid3DDefinition ellipsoid = new YoGraphicEllipsoid3DDefinition();

      YoTuple3DDefinition position = new YoTuple3DDefinition();
      position.setX(originPose.getTranslationX());
      position.setY(originPose.getTranslationY());
      position.setZ(originPose.getTranslationZ());
      position.setReferenceFrame(referenceFrame.getName());
      ellipsoid.setPosition(position);

      Quaternion orientation = new Quaternion(originPose.getRotation());
      YoQuaternionDefinition yoOrientation = new YoQuaternionDefinition();
      yoOrientation.setX(orientation.getX());
      yoOrientation.setY(orientation.getY());
      yoOrientation.setZ(orientation.getZ());
      yoOrientation.setS(orientation.getS());
      yoOrientation.setReferenceFrame(referenceFrame.getName());
      ellipsoid.setOrientation(yoOrientation);

      YoTuple3DDefinition radii = new YoTuple3DDefinition();
      radii.setX(geometryDefinition.getRadiusX());
      radii.setY(geometryDefinition.getRadiusY());
      radii.setZ(geometryDefinition.getRadiusZ());
      radii.setReferenceFrame(referenceFrame.getName());
      ellipsoid.setRadii(radii);

      return ellipsoid;
   }

   private static RigidBodyDefinition convertRigidBodyToRigidBodyDefinition(RigidBodyReadOnly rigidBody)
   {
      RigidBodyDefinition rigidBodyDefinition = new RigidBodyDefinition(rigidBody.getName());
      rigidBodyDefinition.setMass(rigidBody.getInertia().getMass());
      rigidBodyDefinition.setMomentOfInertia(rigidBody.getInertia().getMomentOfInertia());
      RigidBodyTransform inertiaPose = new RigidBodyTransform();
      inertiaPose.set(rigidBody.getBodyFixedFrame().getTransformToParent());
      rigidBodyDefinition.setInertiaPose(inertiaPose);
      return rigidBodyDefinition;
   }

   private static Vector3D computeInertiaEllipsoidRadii(Vector3DReadOnly principalMomentsOfInertia, double mass)
   {
      double Ixx = principalMomentsOfInertia.getX();
      double Iyy = principalMomentsOfInertia.getY();
      double Izz = principalMomentsOfInertia.getZ();

      // http://en.wikipedia.org/wiki/Ellipsoid#Mass_properties
      Vector3D radii = new Vector3D();
      radii.setX(Math.sqrt(5.0 / 2.0 * (Iyy + Izz - Ixx) / mass));
      radii.setY(Math.sqrt(5.0 / 2.0 * (Izz + Ixx - Iyy) / mass));
      radii.setZ(Math.sqrt(5.0 / 2.0 * (Ixx + Iyy - Izz) / mass));

      return radii;
   }

   private static double calculateEllipsoidDensity(RigidBodyDefinition link)
   {
      double mass = link.getMass();
      MomentOfInertiaDefinition momentOfInertia = link.getMomentOfInertia();
      double Ixx = momentOfInertia.getIxx();
      double Iyy = momentOfInertia.getIyy();
      double Izz = momentOfInertia.getIzz();

      // Compute the three semi-axes of an ellipsoid and use them to compute volume (to finally compute density)
      double semiAxisX = Math.sqrt(5.0 * (-Ixx + Iyy + Izz) / (2.0 * mass));
      double semiAxisY = Math.sqrt(5.0 * (Ixx - Iyy + Izz) / (2.0 * mass));
      double semiAxisZ = Math.sqrt(5.0 * (Ixx + Iyy - Izz) / (2.0 * mass));

      double volume = (4.0 / 3.0) * Math.PI * semiAxisX * semiAxisY * semiAxisZ;
      if (Double.isNaN(volume))
      {
         LogTools.warn(
               "Ellipsoid volume is NaN for " + link.getName() + ", meaning something is wrong with the inertia (likely in the URDF). Setting density to 0.");
         return 0.0;
      }

      return mass / volume;
   }

   public void setRadii(Vector3D radii)
   {
      this.radii.set(radii);
   }

   public YoVector3D getRadii()
   {
      return radii;
   }

   public void setColor(int rgba)
   {
      // The 'color' is defined by this YoInteger. We do not change 'color' directly.
      this.rgba.set(rgba);
   }

   public YoInteger getColorInteger()
   {
      return rgba;
   }

   public RigidBodyReadOnly getRigidBody()
   {
      return rigidBody;
   }

   public YoGraphicEllipsoid3DDefinition getEllipsoidDefinition()
   {
      return ellipsoidDefinition;
   }
}
