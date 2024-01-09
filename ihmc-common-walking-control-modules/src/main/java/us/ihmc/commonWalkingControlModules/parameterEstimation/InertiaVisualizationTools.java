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
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoComposite.YoColorRGBASingleDefinition;
import us.ihmc.scs2.definition.yoComposite.YoQuaternionDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicEllipsoid3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

//TODO Relocate this, maybe the ellipsoid stuff to scs2 and the colormap stuff to open robotics
public class InertiaVisualizationTools
{
   public static ArrayList<YoInertiaEllipsoid> createYoInertiaEllipsoids(RigidBodyReadOnly rootBody, YoRegistry registry)
   {
      ArrayList<YoInertiaEllipsoid> yoInertiaEllipsoids = new ArrayList<>();
      RigidBodyReadOnly[] bodies = rootBody.subtreeArray();
      for (int i = 0; i < bodies.length; i++)
      {
         if (bodies[i].isRootBody())
            continue;

         YoInertiaEllipsoid yoInertiaEllipsoid = new YoInertiaEllipsoid(bodies[i].getName(), registry);
         yoInertiaEllipsoids.add(yoInertiaEllipsoid);
      }
      return yoInertiaEllipsoids;
   }

   public static YoGraphicDefinition getInertiaEllipsoidGroup(RigidBodyReadOnly rootBody, ArrayList<YoInertiaEllipsoid> yoInertiaEllipsoids)
   {
      YoGraphicGroupDefinition ellipsoidGroup = new YoGraphicGroupDefinition("Inertia Ellipsoids");

      RigidBodyReadOnly[] bodies = rootBody.subtreeArray();
      for (int i = 0; i < bodies.length; i++)
      {
         if (bodies[i].isRootBody())
            continue;

         // TODO: move this to a method or change all methods to work with RigidBodyBasics
         RigidBodyDefinition rigidBodyDefinition = new RigidBodyDefinition(bodies[i].getName());
         rigidBodyDefinition.setMass(bodies[i].getInertia().getMass());
         rigidBodyDefinition.setMomentOfInertia(bodies[i].getInertia().getMomentOfInertia());
         RigidBodyTransform inertiaPose = new RigidBodyTransform();
         inertiaPose.set(bodies[i].getBodyFixedFrame().getTransformToParent());
         rigidBodyDefinition.setInertiaPose(inertiaPose);

         //Initialize to the middle of the colormap
         int[] rgbColor = getRGBForViridisColorMap(0.5);
         int opacityAlpha = (int) (0.5 * 255);
         int rgbInt = ColorDefinitions.toRGBA(rgbColor[0], rgbColor[1], rgbColor[2], opacityAlpha);
         yoInertiaEllipsoids.get(i).setColor(rgbInt);

         /* Create the ellipsoid */
         YoGraphicEllipsoid3DDefinition ellipsoid = convertRobotMassPropertiesToInertiaEllipsoids(rootBody,
                                                                                                  rigidBodyDefinition,
                                                                                                  yoInertiaEllipsoids.get(i));
         if (ellipsoid == null)
            continue;

         ellipsoidGroup.addChild(ellipsoid);
      }

      return ellipsoidGroup;
   }

   //TODO: this is a testing method which can be called in the simulation constructor, should remove eventually
   public static ArrayList<YoGraphicEllipsoid3DDefinition> getInertiaEllipsoidListTesting(RigidBodyReadOnly rootBody,
                                                                                          RobotDefinition robotDefinition,
                                                                                          boolean colorBasedOnDensity)
   {
      ArrayList<YoGraphicEllipsoid3DDefinition> ellipsoidList = new ArrayList<>();
      List<RigidBodyDefinition> allRigidBodies = robotDefinition.getAllRigidBodies();
      allRigidBodies.remove(0); // We remove the elevator

      if (colorBasedOnDensity)
      {
         // Get the maximum density to normalize the color map
         double maxLinkDensity = 0.0;
         for (RigidBodyDefinition rigidBodyDefinition : allRigidBodies)
         {
            maxLinkDensity = Math.max(maxLinkDensity, calculateEllipsoidDensity(rigidBodyDefinition));
         }

         for (RigidBodyDefinition rigidBodyDefinition : allRigidBodies)
         {
            /* Calculate the color of the inertia ellipsoid */
            double scale = calculateEllipsoidDensity(rigidBodyDefinition) / maxLinkDensity;
            // Change from linear scale to emphasize lower densities (pelvis is disproportionately dense)
            int[] rgbColor = getRGBForViridisColorMap(Math.pow(1 - scale, 2.5)); // purple is denser
            //                     int[] rgbColor = getRGBForViridisColorMap(Math.pow(scale, 0.4 * 255)); // yellow is denser
            int opacityAlpha = (int) ((Math.pow(scale, 0.4) + 1.0) * 0.5 * 255);
            int rgbInt = ColorDefinitions.toRGBA(rgbColor[0], rgbColor[1], rgbColor[2], opacityAlpha);
            //TODO: verify the input is correct
            YoColorRGBASingleDefinition color = new YoColorRGBASingleDefinition(Integer.toString(rgbInt));

            /* Create the ellipsoid */
            YoGraphicEllipsoid3DDefinition ellipsoid = convertRobotMassPropertiesToInertiaEllipsoidsTesting(rootBody, rigidBodyDefinition, color);
            if (ellipsoid == null)
               continue;

            ellipsoidList.add(ellipsoid);
         }
      }
      else
      {
         // Else, just use one color for all ellipsoids
         for (RigidBodyDefinition rigidBodyDefinition : allRigidBodies)
         {
            int rgbInt = ColorDefinitions.toRGBA(40, 150, 75, 153);
            //TODO: verify the input is correct
            YoColorRGBASingleDefinition color = new YoColorRGBASingleDefinition(Integer.toString(rgbInt));

            /* Create the ellipsoid */
            YoGraphicEllipsoid3DDefinition ellipsoid = convertRobotMassPropertiesToInertiaEllipsoidsTesting(rootBody, rigidBodyDefinition, color);
            if (ellipsoid == null)
               continue;

            ellipsoidList.add(ellipsoid);
         }
      }
      return ellipsoidList;
   }

   public static YoGraphicDefinition getEllipsoidGraphicGroup(ArrayList<YoGraphicEllipsoid3DDefinition> ellipsoidList)
   {
      YoGraphicGroupDefinition inertiaEllipsoids = new YoGraphicGroupDefinition("Inertia Ellipsoids");
      for (YoGraphicEllipsoid3DDefinition ellipsoid : ellipsoidList)
      {
         inertiaEllipsoids.addChild(ellipsoid);
      }
      return inertiaEllipsoids;
   }

   public static void updateEllipsoid(YoInertiaEllipsoid inertiaEllipsoid, double scale)
   {
      //FIXME: nothing updates in simulation
      int[] rgbColor = getRGBForViridisColorMap(scale);
      int opacityAlpha = (int) (0.6 * 255);
      int rgbInt = ColorDefinitions.toRGBA(rgbColor[0], rgbColor[1], rgbColor[2], opacityAlpha);
      inertiaEllipsoid.setColor(rgbInt);
   }

   /**
    * Copies functionality from a simimlar method in YoGraphicTools. Exposes more of the method so that the color of the inertial ellipsoids can be set
    * individually.
    */
   private static YoGraphicEllipsoid3DDefinition convertRobotMassPropertiesToInertiaEllipsoids(RigidBodyReadOnly rootBody,
                                                                                               RigidBodyDefinition rigidBodyDefinition,
                                                                                               YoInertiaEllipsoid yoEllipsoid)
   {
      // It is up to the user to handle when null is returned outside this method
      if (rigidBodyDefinition.getInertiaPose() == null)
         return null;
      if (rigidBodyDefinition.getMomentOfInertia() == null)
         return null;

      SingularValueDecomposition3D svd = new SingularValueDecomposition3D();
      if (!svd.decompose(rigidBodyDefinition.getMomentOfInertia()))
         return null;

      RigidBodyReadOnly rigidBody = MultiBodySystemTools.findRigidBody(rootBody, rigidBodyDefinition.getName());

      if (rigidBody.isRootBody())
         return null;

      ReferenceFrame referenceFrame = rigidBody.getParentJoint().getFrameAfterJoint();

      RigidBodyTransform ellipsoidPose = new RigidBodyTransform(rigidBodyDefinition.getInertiaPose());
      ellipsoidPose.appendOrientation(svd.getU());
      Vector3D radii = computeInertiaEllipsoidRadii(svd.getW(), rigidBodyDefinition.getMass());
      yoEllipsoid.setRadii(radii);
      YoGraphicEllipsoid3DDefinition ellipsoid = convertEllipsoid3DDefinition(referenceFrame, ellipsoidPose, new Ellipsoid3DDefinition(yoEllipsoid.getRadii()));
      ellipsoid.setName(rigidBody.getName() + " inertia");
      ellipsoid.setColor(yoEllipsoid.getColor());

      return ellipsoid;
   }

   /**
    * Copies functionality from a simimlar method in YoGraphicTools. Exposes more of the method so that the color of the inertial ellipsoids can be set
    * individually.
    */
   private static YoGraphicEllipsoid3DDefinition convertRobotMassPropertiesToInertiaEllipsoidsTesting(RigidBodyReadOnly rootBody,
                                                                                                      RigidBodyDefinition rigidBodyDefinition,
                                                                                                      YoColorRGBASingleDefinition color)
   {
      // It is up to the user to handle when null is returned outside this method
      if (rigidBodyDefinition.getInertiaPose() == null)
         return null;
      if (rigidBodyDefinition.getMomentOfInertia() == null)
         return null;

      SingularValueDecomposition3D svd = new SingularValueDecomposition3D();
      if (!svd.decompose(rigidBodyDefinition.getMomentOfInertia()))
         return null;

      RigidBodyReadOnly rigidBody = MultiBodySystemTools.findRigidBody(rootBody, rigidBodyDefinition.getName());

      if (rigidBody.isRootBody())
         return null;

      ReferenceFrame referenceFrame = rigidBody.getParentJoint().getFrameAfterJoint();

      RigidBodyTransform ellipsoidPose = new RigidBodyTransform(rigidBodyDefinition.getInertiaPose());
      ellipsoidPose.appendOrientation(svd.getU());
      Vector3D radii = computeInertiaEllipsoidRadii(svd.getW(), rigidBodyDefinition.getMass());
      YoGraphicEllipsoid3DDefinition ellipsoid = convertEllipsoid3DDefinition(referenceFrame, ellipsoidPose, new Ellipsoid3DDefinition(radii));
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

   /**
    * Returns the radii of an ellipsoid given the inertia parameters, assuming a uniform mass
    * distribution.
    *
    * @param principalMomentsOfInertia principal moments of inertia {Ixx, Iyy, Izz}
    * @param mass                      mass of the link
    * @return the three radii of the inertia ellipsoid
    */
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

   /**
    * Takes a scale between 0 and 1 and returns the RGB values for the Viridis colormap at that scale.
    * <p>
    * NOTE: This method uses coefficients from a polynomial fit of the RGB values of the Viridis colormap from Python. The coefficients were computed externally
    * in a Python script. For a higher or lower order fit, the coefficients would need to be regenerated.
    * </p>
    *
    * @param scale Number between 0 and 1, determines the location on the colormap.
    * @return RGB values for the colormap at the given scale.
    */
   private static int[] getRGBForViridisColorMap(double scale)
   {
      double[] redCoefficients = {0.2774879, -0.11255063, 2.10607102, -14.14404302, 23.72334937, -10.90841729};
      double[] greenCoefficients = {0.00247317, 1.49535097, -1.31431592, 1.28943779, -0.223256, -0.35398068};
      double[] blueCoefficients = {0.31371651, 2.2855777, -9.80527933, 22.74666121, -25.84324129, 10.38676565};

      if (scale < 0 || scale > 1)
      {
         throw new IllegalArgumentException("Value must be between 0 and 1");
      }

      int red = evaluateRGBPolynomial(redCoefficients, scale);
      int green = evaluateRGBPolynomial(greenCoefficients, scale);
      int blue = evaluateRGBPolynomial(blueCoefficients, scale);

      return new int[] {red, green, blue};
   }

   /**
    * Helper method for {@link #getRGBForViridisColorMap(double)}. Evaluates a polynomial for one color (R, G, or B) with the given
    * coefficients at the given scale.
    *
    * @param colorCoefficients Coefficients of the polynomial, in order of increasing degree.
    * @param scale             Scale at which to evaluate the polynomial.
    * @return Value of the polynomial at the given scale.
    */
   private static int evaluateRGBPolynomial(double[] colorCoefficients, double scale)
   {
      double rgbValue = 0.0;
      for (int i = 0; i < colorCoefficients.length; i++)
      {
         rgbValue += colorCoefficients[i] * Math.pow(scale, i);
      }
      return (int) Math.round(rgbValue * 255);  // Scale to 0-255 range
   }
}
