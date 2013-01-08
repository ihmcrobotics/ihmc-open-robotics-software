package us.ihmc.graphics3DAdapter.graphics;

import java.net.URL;
import java.util.ArrayList;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.HeightMap;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DAddArcTorusInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DAddConeInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DAddCoordinateSystemInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DAddCubeInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DAddCylinderInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DAddEllipsoidInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DAddExtrudedPolygonInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DAddHeightMapInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DAddHemiEllipsoidInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DAddMeshDataInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DAddModelFileInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DAddPolygonInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DAddPyramidCubeInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DAddSphereInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DAddTeaPotInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DAddTextInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DAddTruncatedConeInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DAddWedgeInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DIdentityInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DRotateInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DRotateMatrixInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DScaleInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.Graphics3DTranslateInstruction;
import us.ihmc.graphics3DAdapter.input.ModifierKeyInterface;
import us.ihmc.graphics3DAdapter.input.SelectedListener;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;
import us.ihmc.utilities.InertiaTools;


public class Graphics3DObject
{
   private enum Axis
   {
      X, Y, Z
   }
   
   public static final Axis X = Axis.X;
   public static final Axis Y = Axis.Y;
   public static final Axis Z = Axis.Z;
   
   private ArrayList<Graphics3DPrimitiveInstruction> graphics3DInstructions;
   
   private final ArrayList<SelectedListener> selectedListeners = new ArrayList<SelectedListener>();

   public Graphics3DObject(ArrayList<Graphics3DPrimitiveInstruction> graphics3DInstructions)
   {
      this();
      this.graphics3DInstructions = graphics3DInstructions;
   }

   /**
    * Default no-arg constructor.  This creates a new empty Graphics3DObject component.
    */
   public Graphics3DObject()
   {
      graphics3DInstructions = new ArrayList<Graphics3DPrimitiveInstruction>();
   }

   public ArrayList<Graphics3DPrimitiveInstruction> getGraphics3DInstructions()
   {
      return graphics3DInstructions;
   }
   
   /**
    * Merge this with the specified Graphics3DObject.
    *
    * @param graphics3DObject Graphics3DObject to combine with.
    */
   public void combine(Graphics3DObject Graphics3DObject)
   {
      this.identity();
      this.graphics3DInstructions.addAll(Graphics3DObject.getGraphics3DInstructions());
   }
   
   public void addInstruction(Graphics3DPrimitiveInstruction instruction)
   {
      this.graphics3DInstructions.add(instruction);
   }

   /**
    * Translates from the current position by the specified distances.  Graphic
    * components added after translation will appear in the new location.  The coordinate
    * system for these translations is based on those that preceded it.  Each new
    * has its coordinates reset to the parent joint's origin.  {@link #identity Identity}
    * resets back to the joint origin.
    *
    * @param tx distance translated in the x direction
    * @param ty distance translated in the y direction
    * @param tz distance translated in the z direction
    */
   public void translate(double tx, double ty, double tz)
   {
      graphics3DInstructions.add(new Graphics3DTranslateInstruction(tx, ty, tz));
   }

   /**
    * Translates from the current position by the specified distances.  Graphic
    * components added after translation will appear in the new location.  The coordinate
    * system for these translations is based on those that preceded it.  Each new
    * has its coordinates reset to the parent joint's origin.  {@link #identity Identity}
    * resets back to the joint origin.
    *
    * @param translation Vector3d representing the translation.
    */
   public void translate(Vector3d translation)
   {
      graphics3DInstructions.add(new Graphics3DTranslateInstruction(translation));
   }

   /**
    * Rotates the coordinate system counter clockwise around the specified axis by the given
    * angle in radians.  This does not rotate existing graphics, instead it rotates a "cursor"
    * when another object is added it will be centered on the origin of the current system
    * as described by the translations and rotations applied since its creation at the joint
    * origin.
    *
    * @param rotAng the angle to rotate around the specified axis in radians.
    * @param rotAxis Axis around which to rotate. Either Link.X, Link.Y or Link.Z
    */
   public void rotate(double rotAng, int rotAxis)
   {
      Vector3d axis = new Vector3d();
      if(rotAxis == 0)
         axis.setX(1.0);
      else if(rotAxis == 1)
         axis.setY(1.0);
      else if(rotAxis == 2)
         axis.setZ(1.0);
      
      rotate(rotAng, axis);
   }
   
   public void rotate(double rotAng, Axis rotAxis)
   {
      Vector3d axis = new Vector3d();
      if(rotAxis == Axis.X)
         axis.setX(1.0);
      else if(rotAxis == Axis.Y)
         axis.setY(1.0);
      else if(rotAxis == Axis.Z)
         axis.setZ(1.0);
      
      rotate(rotAng, axis);
   }

   /**
    * Rotates the coordinate system counter clockwise around the specified axis by the given
    * angle in radians.  This does not rotate existing graphics, instead it rotates a "cursor"
    * When another object is added it will be centered on the origin of the current system
    * as described by the translations and rotations applied since its creation at the joint
    * origin.
    *
    * @param rotAng the angle to rotate around the specified axis in radians.
    * @param rotAxis Vector3d describing the axis of rotation.
    */
   public void rotate(double rotAng, Vector3d rotAxis)
   {
      graphics3DInstructions.add(new Graphics3DRotateInstruction(rotAng, rotAxis));
   }

   /**
    * Rotates the coordinate system as described by the given rotation matrix.
    * This does not rotate existing graphics, instead it rotates a "cursor".
    * When another object is added it will be centered on the origin of the current system
    * as described by the translations and rotations applied since its creation at the joint
    * origin.
    *
    * @param rot Matrix3d describing the rotation to be applied.
    */
   public void rotate(Matrix3d rot)
   {
      graphics3DInstructions.add(new Graphics3DRotateMatrixInstruction(rot));
   }

   /**
    * Scales the coordinate system by the specified scale factor. This does not scale existing
    * graphics, instead it scales the "current" coordinate system.  When another object is added
    * it will be uniformly scaled by the specified factor.
    *
    * @param scaleFactor Factor by which the coordinate system is scaled.  For example, 0.5 would
    * reduce future objects size by 50% whereas 2 would double it.
    * @return 
    */
   public Graphics3DScaleInstruction scale(double scaleFactor)
   {
      return scale(new Vector3d(scaleFactor, scaleFactor, scaleFactor));
   }

   /**
    * Scales the coordinate system by the specified scale factor. This does not scale existing
    * graphics, instead it scales the "current" coordinate system.  When another object is added
    * it will be uniformly scaled by the specified factor.  The components of the vector indicate
    * scale factors in each dimension.
    *
    * @param scaleFactors Vector3d describing the scaling factors in each dimension.
    * @return 
    */
   public Graphics3DScaleInstruction scale(Vector3d scaleFactors)
   {
      Graphics3DScaleInstruction graphics3DScale = new Graphics3DScaleInstruction(scaleFactors);
      graphics3DInstructions.add(graphics3DScale);
      return graphics3DScale;
   }

   /**
    * Resets the coordinate system to the joint origin.  This clears all rotations, translations,
    * and scale factors.
    */
   public void identity()
   {
      graphics3DInstructions.add(new Graphics3DIdentityInstruction());
   }

   /**
    * Adds the specified 3DS Max file to the center of the current coordinate system
    * with a default appearance.  3DS Max is a 3D modeling program that allows the creation
    * of detailed models and animations.  This function only imports the model allowing the use
    * of more complicated and detailed system representations in simulations.
    *
    * @param fileURL URL pointing to the desired 3ds file.
    */
   public void addModelFile(URL fileURL)
   {
      addModelFile(fileURL, null);
   }

   /**
    * Adds the specified 3DS Max file to the center of the current coordinate system
    * with the given appearance.  3DS Max is a 3D modeling program that allows the creation
    * of detailed models and animations.  This function only imports the model allowing the use
    * of more complicated and detailed system representations in simulations.
    *
    * @param fileURL URL pointing to the desired 3ds file.
    * @param yoAppearanceDefinition Appearance to use with the 3ds model once imported.
    */
   public void addModelFile(URL fileURL, AppearanceDefinition yoAppearanceDefinition)
   {
      if (fileURL == null)
      {
         System.err.println("fileURL == null in addModelFile");

         return;
      }

      String fileName = fileURL.getFile();

      // System.out.println("File name: " + fileName + " " + fileName.length());

      if ((fileName == null) || (fileName.equals("")))
      {
         System.out.println("Null File Name in add3DSFile");

         return;
      }

      addModelFile(fileName, yoAppearanceDefinition);
   }

   /**
    * Adds the specified 3DS Max file to the center of the current coordinate system
    * with a default appearance.  3DS Max is a 3D modeling program that allows the creation
    * of detailed models and animations.  This function only imports the model allowing the use
    * of more complicated and detailed system representations in simulations.
    *
    * @param fileName File path of the desired 3ds file.
    */
   public void addModelFile(String fileName)
   {
      addModelFile(fileName, null);
   }

   /**
    * Adds the specified 3DS Max file to the center of the current coordinate system
    * with the given appearance.  3DS Max is a 3D modeling program that allows the creation
    * of detailed models and animations.  This function only imports the model allowing the use
    * of more complicated and detailed system representations in simulations.
    *
    * @param fileName File path to the desired 3ds file.
    * @param app Appearance to use with the model once imported.
    */
   /**
   * @param fileName
   * @param app
   */
   public void addModelFile(String fileName, AppearanceDefinition app)
   {
      graphics3DInstructions.add(new Graphics3DAddModelFileInstruction(fileName, app));
   }

   public void addCoordinateSystem(double length)
   {
      addCoordinateSystem(length, YoAppearance.Black());
   }

   public void addCoordinateSystem(double length, AppearanceDefinition arrowAppearance)
   {
      addCoordinateSystem(length, YoAppearance.Red(), YoAppearance.White(), YoAppearance.Blue(), arrowAppearance);
   }

   /**
    * Creates a graphical representation of the x, y, and z axis of the current coordinate
    * system centered at its origin.  In the image below red, white and blue represent the
    * x, y and z axies respectively.<br /><br />
    * <img src="doc-files/LinkGraphics.addCoordinateSystem.jpg">
    *
    * @param length the length in meters of each axis arrow.
    */
   public void addCoordinateSystem(double length, AppearanceDefinition xAxisAppearance, AppearanceDefinition yAxisAppearance, AppearanceDefinition zAxisAppearance, AppearanceDefinition arrowAppearance)
   {
      graphics3DInstructions.add(new Graphics3DAddCoordinateSystemInstruction(length));
   }

   /**
    * Adds a solid black cube with the specified dimensions centered around the origin of the
    * current coordinate system.  All lengths are in meters.</ br></ br>
    * The image below demonstrates a 0.25 x 0.25 x 0.25 cube generated by the following code:<br /><br />
    * {@code linkGraphics.addCoordinateSystem(0.5);}<br />
    * {@code linkGraphics.addCube(0.25, 0.25, 0.25);}<br /><br />
    *
    * As is show by the graphical representation the cube is centered on the coordinate system.
    * Again, x, y and z are red, white and blue.
    * <br /><br /><img src="doc-files/LinkGraphics.addCube1.jpg">
    *
    * @param lx length of the cube in the x direction.
    * @param ly length of the cube in the y direction.
    * @param lz length of the cube in the z direction.
    */
   public void addCube(double lx, double ly, double lz)
   {
      addCube(lx, ly, lz, YoAppearance.Black());
   }

   /**
    * Adds a solid cube with the given dimensions and appearance centered on the origin of the
    * current coordinate system.  All lengths are in meters.</ br></ br>
    * The image below demonstrates a yellow 0.1 x 0.35 x 0.2 cube generated by the following code:<br /><br />
    * {@code linkGraphics.addCoordinateSystem(0.5);}<br />
    * {@code linkGraphics.addCube(0.1, 0.35, 0.2, YoAppearance.Yellow());}<br /><br />
    *
    * As is show by the graphical representation the cube is centered on the coordinate system.
    * Again, x, y and z are red, white and blue.
    * <br /><br /><img src="doc-files/LinkGraphics.addCube2.jpg">
    *
    *
    * @param lx length of the cube in the x direction.
    * @param ly length of the cube in the y direction.
    * @param lz length of the cube in the z direction.
    * @param cubeApp Appearance of the cube.  See {@link YoAppearance YoAppearance} for implementations.
    */
   public void addCube(double lx, double ly, double lz, AppearanceDefinition cubeApp)
   {
      graphics3DInstructions.add(new Graphics3DAddCubeInstruction(lx, ly, lz, cubeApp));
   }

   /**
    * Adds a solid wedge with the given dimensions centered on the origin of the current
    * coordinate system.  The peak of the wedge is directly above the far edge of the cube
    * in the x direction.</ br></ br>
    * The image below demonstrates a 0.25 x 0.25 x 0.25 wedge generated by the following code:<br /><br />
    * {@code linkGraphics.addCoordinateSystem(0.5);}<br />
    * {@code linkGraphics.addWedge(0.25, 0.25, 0.25);}<br /><br />
    *
    * As is show by the graphical representation the wedge is centered on the coordinate system.
    * Again, x, y and z are red, white and blue.
    * <br /><br /><img src="doc-files/LinkGraphics.addWedge1.jpg">
    *
    * @param lx length of the wedge in the x direction.
    * @param ly length of the wedge in the y direction.
    * @param lz length of the wedge in the z direction.
    */
   public void addWedge(double lx, double ly, double lz)
   {
      addWedge(lx, ly, lz, YoAppearance.Black());
   }

   /**
    *
    * Adds a solid wedge with the given dimensions and appearance centered on the origin of the current
    * coordinate system.  The peak of the wedge is directly above the far edge of the cube
    * in the x direction.</ br></ br>
    * The image below demonstrates a green 0.35 x 0.3 x 0.1 wedge generated by the following code:<br /><br />
    * {@code linkGraphics.addCoordinateSystem(0.5);}<br />
    * {@code linkGraphics.addWedge(0.35, 0.3, 0.1, YoAppearance.GREEN());}<br /><br />
    *
    * As is show by the graphical representation the wedge is centered on the coordinate system.
    * Again, x, y and z are red, white and blue.
    * <br /><br /><img src="doc-files/LinkGraphics.addWedge2.jpg">
    *
    * @param lx length of the wedge in the x direction.
    * @param ly length of the wedge in the y direction.
    * @param lz length of the wedge in the z direction.
    * @param wedgeApp Appearance of the wedge.  See {@link YoAppearance YoAppearance} for implementations.
    */
   public void addWedge(double lx, double ly, double lz, AppearanceDefinition wedgeApp)
   {
         graphics3DInstructions.add(new Graphics3DAddWedgeInstruction(lx, ly, lz, wedgeApp));
   }

   /**
    * Adds a solid sphere with the given radius centered on the origin of the current coordinate system.
    * </ br></ br>
    * The image below demonstrates a sphere with a 0.25 meter radius generated by the following code:<br /><br />
    * {@code linkGraphics.addCoordinateSystem(0.5);}<br />
    * {@code linkGraphics.addSphere(0.25);}<br /><br />
    *
    * As is show by the graphical representation the sphere is centered on the coordinate system.
    * Again, x, y and z are red, white and blue.
    * <br /><br /><img src="doc-files/LinkGraphics.addSphere1.jpg">
    *
    * @param radius radius of the new sphere in meters.
    */
   public void addSphere(double radius)
   {
      addSphere(radius, YoAppearance.Black());
   }

   /**
    * Adds a solid sphere with the given radius and appearance centered on the origin of the current coordinate system.
    * </ br></ br>
    * The image below demonstrates a blue sphere with a 0.15 meter radius generated by the following code:<br /><br />
    * {@code linkGraphics.addCoordinateSystem(0.5);}<br />
    * {@code linkGraphics.addSphere(0.15, YoAppearance.Blue());}<br /><br />
    *
    * As is show by the graphical representation the sphere is centered on the coordinate system.
    * Again, x, y and z are red, white and blue.
    * <br /><br /><img src="doc-files/LinkGraphics.addSphere2.jpg">
    *
    * @param radius radius of the new sphere in meters.
    * @param sphereApp Appearance to be used with the new sphere.  See {@link YoAppearance YoAppearance} for implementations.
    */
   public Graphics3DAddSphereInstruction addSphere(double radius, AppearanceDefinition sphereApp)
   {
      Graphics3DAddSphereInstruction instruction = new Graphics3DAddSphereInstruction(radius, sphereApp);
      graphics3DInstructions.add(instruction);
      return instruction;
   }

   public Graphics3DAddMeshDataInstruction addMeshData(MeshDataHolder meshData, AppearanceDefinition meshAppearance)
   {
      Graphics3DAddMeshDataInstruction instruction = new Graphics3DAddMeshDataInstruction(meshData, meshAppearance);
      graphics3DInstructions.add(instruction);
      return instruction;
   }
   
   /**
    * Adds a solid ellipsoid with the given radii centered on the origin of the current coordinate system.
    * </ br></ br>
    * The image below demonstrates an ellipsoid with radii of 0.3, 0.2 and 0.1 in the x, y and z directions respectively:<br /><br />
    * {@code linkGraphics.addCoordinateSystem(0.5);}<br />
    * {@code linkGraphics.addEllipsoid(0.3, 0.2, 0.1);}<br /><br />
    *
    * As is show by the graphical representation the ellipsoid is centered on the coordinate system.
    * Again, x, y and z are red, white and blue.
    * <br /><br /><img src="doc-files/LinkGraphics.addEllipsoid1.jpg">
    *
    * @param xRad x direction radius in meters
    * @param yRad y direction radius in meters
    * @param zRad z direction radius in meters
    */
   public void addEllipsoid(double xRadius, double yRadius, double zRadius)
   {
      addEllipsoid(xRadius, yRadius, zRadius, YoAppearance.Black());
   }

   /**
    * Adds a solid ellipsoid with the given radii and appearance centered on the origin of the current coordinate system.
    * </ br></ br>
    * The image below demonstrates a red ellipsoid with radii of 0.2, 0.2 and 0.1 in the x, y and z directions respectively:<br /><br />
    * {@code linkGraphics.addCoordinateSystem(0.5);}<br />
    * {@code linkGraphics.addEllipsoid(0.2, 0.2, 0.1, YoAppearance.Red());}<br /><br />
    *
    * As is show by the graphical representation the ellipsoid is centered on the coordinate system.
    * Again, x, y and z are red, white and blue.
    * <br /><br /><img src="doc-files/LinkGraphics.addEllipsoid2.jpg">
    *
    * @param xRad x direction radius in meters
    * @param yRad y direction radius in meters
    * @param zRad z direction radius in meters
    * @param ellipsoidApp Appearance to be used with the new ellipsoid.  See {@link YoAppearance YoAppearance} for implementations.
    */
   public Graphics3DAddEllipsoidInstruction addEllipsoid(double xRadius, double yRadius, double zRadius, AppearanceDefinition ellipsoidApp)
   {
         Graphics3DAddEllipsoidInstruction instruction = new Graphics3DAddEllipsoidInstruction(xRadius, yRadius, zRadius, ellipsoidApp);
         graphics3DInstructions.add(instruction);
         return instruction;
   }

   /**
    * Adds a soild cylinder with the given radius and height centered on the origin of the current coordinate system.
    * </ br></ br>
    * The image below demonstrates a cylinder with radius of 0.2 and a height of 0.4 as described by the following code:<br /><br />
    * {@code linkGraphics.addCoordinateSystem(0.5);}<br />
    * {@code linkGraphics.addCylinder(0.4, 0.2);}<br /><br />
    *
    * As is show by the graphical representation the cylinder is centered on the coordinate system.
    * Again, x, y and z are red, white and blue.
    * <br /><br /><img src="doc-files/LinkGraphics.addCylinder1.jpg">
    *
    * @param height cylinder height in meters.
    * @param radius cylinder radius in meters.
    */
   public void addCylinder(double height, double radius)
   {
      addCylinder(height, radius, YoAppearance.Black());
   }

   /**
    * Adds a soild cylinder with the given radius, height and appearance centered on the origin of the current coordinate system.
    * </ br></ br>
    * The image below demonstrates a maroon cylinder with radius of 0.3 and a height of 0.1 as described by the following code:<br /><br />
    * {@code linkGraphics.addCoordinateSystem(0.5);}<br />
    * {@code linkGraphics.addCylinder(0.1, 0.3, YoAppearance.Maroon());}<br /><br />
    *
    * As is show by the graphical representation the cylinder is centered on the coordinate system.
    * Again, x, y and z are red, white and blue.
    * <br /><br /><img src="doc-files/LinkGraphics.addCylinder2.jpg">
    *
    * @param height cylinder height in meters.
    * @param radius cylinder radius in meters.
    * @param cylApp Appearance to be used with the new cylinder.  See {@link YoAppearance YoAppearance} for implementations.
    */
   public Graphics3DAddCylinderInstruction addCylinder(double height, double radius, AppearanceDefinition cylApp)
   {
      Graphics3DAddCylinderInstruction instruction = new Graphics3DAddCylinderInstruction(height, radius, cylApp);
      graphics3DInstructions.add(instruction);
      return instruction;
   }

   /**
    * Adds a cone with the given height and radius centered on the origin of the current coordinate system.
    * </ br></ br>
    * The image below demonstrates a cone with radius of 0.15 and a height of 0.35 as described by the following code:<br /><br />
    * {@code linkGraphics.addCoordinateSystem(0.5);}<br />
    * {@code linkGraphics.addCone(0.35, 0.15);}<br /><br />
    *
    * As is show by the graphical representation the cone is centered on the coordinate system.
    * Again, x, y and z are red, white and blue.
    * <br /><br /><img src="doc-files/LinkGraphics.addCone1.jpg">
    *
    * @param height cone height in meters.
    * @param radius cone radius in meters.
    */
   public void addCone(double height, double radius)
   {
      addCone(height, radius, YoAppearance.Black());
   }

   /**
    * Adds a cone with the given height, radius and appearance centered on the origin of the current coordinate system.
    * </ br></ br>
    * The image below demonstrates a dark green cone with radius of 0.4 and a height of 0.2 as described by the following code:<br /><br />
    * {@code linkGraphics.addCoordinateSystem(0.5);}<br />
    * {@code linkGraphics.addCone(0.2, 0.4, YoAppearance.DarkGreen());}<br /><br />
    *
    * As is show by the graphical representation the cone is centered on the coordinate system.
    * Again, x, y and z are red, white and blue.
    * <br /><br /><img src="doc-files/LinkGraphics.addCone2.jpg">
    *
    * @param height cone height in meters.
    * @param radius cone radius in meters.
    * @param coneApp Appearance to be used with the new cone.  See {@link YoAppearance YoAppearance} for implementations.
    */
   public void addCone(double height, double radius, AppearanceDefinition coneApp)
   {
      graphics3DInstructions.add(new Graphics3DAddConeInstruction(height, radius, coneApp));
   }

   /**
    * Adds a truncated cone with the given height, base width x, base width y, top width x, and top width y centered
    * on the origin of the current coordinate system.
    * </ br></ br>
    * The image below demonstrates a truncated cone with a height of 0.3, a x base width of 0.25, a y base width of 0.2,
    * a x top width of 0.15, and a y top width of 0.1:<br /><br />
    * {@code linkGraphics.addCoordinateSystem(0.5);}<br />
    * {@code linkGraphics.addGenTruncatedCone(0.3, 0.25, 0.2, 0.15, 0.1);}<br /><br />
    *
    * As is show by the graphical representation the truncated cone is centered on the coordinate system.
    * Again, x, y and z are red, white and blue.
    * <br /><br /><img src="doc-files/LinkGraphics.addGenTruncatedCone1.jpg">
    *
    * @param height in meters
    * @param bx x direction width of the base in meters
    * @param by y direction width of the base in meters
    * @param tx x direction width of the top in meters
    * @param ty y direction width of the top in meters
    */
   public void addGenTruncatedCone(double height, double bx, double by, double tx, double ty)
   {
      addGenTruncatedCone(height, bx, by, tx, ty, YoAppearance.Black());
   }

   /**
    * Adds a truncated cone with the given height, base width x, base width y, top width x,
    * top width y, and appearance centered on the origin of the current coordinate system.
    * </ br></ br>
    * The image below demonstrates a navy blue truncated cone with a height of 0.25, a x
    * base width of 0.15, a y base width of 0.15, a x top width of 0.05, and a y top width
    * of 0.1:<br /><br />
    *
    * {@code linkGraphics.addCoordinateSystem(0.5);}<br />
    * {@code linkGraphics.addGenTruncatedCone(0.25, 0.15, 0.15, 0.05, 0.1, YoAppearance.Navy());}<br /><br />
    *
    * As is show by the graphical representation the truncated cone is centered on the coordinate system.
    * Again, x, y and z are red, white and blue.
    * <br /><br /><img src="doc-files/LinkGraphics.addGenTruncatedCone2.jpg">
    *
    * @param height in meters
    * @param bx x direction width of the base in meters
    * @param by y direction width of the base in meters
    * @param tx x direction width of the top in meters
    * @param ty y direction width of the top in meters
    * @param coneApp Appearance to be used with the new truncated cone.  See {@link YoAppearance YoAppearance} for implementations.
    */
   public void addGenTruncatedCone(double height, double bx, double by, double tx, double ty, AppearanceDefinition coneApp)
   {
         graphics3DInstructions.add(new Graphics3DAddTruncatedConeInstruction(height, bx, by, tx, ty, coneApp));
   }

   /**
    * Adds a hemi ellipsoid with the given x, y and z radii centered on the current coordinate system.  Hemi ellipsoids
    * are essentially cut in half, in this case the missing half is below the xy plane.
    * </ br></ br>
    * The image below demonstrates a hemi ellipsoid with x, y and z radii of 0.25, 0.15 and 0.35 respectively:<br /><br />
    *
    * {@code linkGraphics.addCoordinateSystem(0.5);}<br />
    * {@code linkGraphics.addHemiEllipsoid(0.25, 0.15, 0.35);}<br /><br />
    *
    * As is show by the graphical representation the hemi ellipsoid is centered on the coordinate system.
    * Again, x, y and z are red, white and blue.
    * <br /><br /><img src="doc-files/LinkGraphics.addHemiEllipsoid1.jpg">
    *
    * @param xRad radius of the ellipsoid in the x direction.
    * @param yRad radius of the ellipsoid in the y direction.
    * @param zRad radius of the ellipsoid in the z direction.
    */
   public void addHemiEllipsoid(double xRad, double yRad, double zRad)
   {
      addHemiEllipsoid(xRad, yRad, zRad, YoAppearance.Black());
   }

   /**
    * Adds a hemi ellipsoid with the given appearance and x, y and z radii centered on the current coordinate system.  Hemi ellipsoids
    * are essentially cut in half, in this case the missing half is below the xy plane.
    * </ br></ br>
    * The image below demonstrates a dark red hemi ellipsoid with x, y and z radii of 0.15, 0.2 and 0.4 respectively:<br /><br />
    *
    * {@code linkGraphics.addCoordinateSystem(0.5);}<br />
    * {@code linkGraphics.addHemiEllipsoid(0.15, 0.2, 0.4, YoAppearance.DarkRed());}<br /><br />
    *
    * As is show by the graphical representation the hemi ellipsoid is centered on the coordinate system.
    * Again, x, y and z are red, white and blue.
    * <br /><br /><img src="doc-files/LinkGraphics.addHemiEllipsoid2.jpg">
    *
    * @param xRad radius of the ellipsoid in the x direction.
    * @param yRad radius of the ellipsoid in the y direction.
    * @param zRad radius of the ellipsoid in the z direction.
    * @param hEApp Appearance to be used with the new hemi ellipsoid.  See {@link YoAppearance YoAppearance} for implementations.
    */
   public void addHemiEllipsoid(double xRad, double yRad, double zRad, AppearanceDefinition hEApp)
   {
         graphics3DInstructions.add(new Graphics3DAddHemiEllipsoidInstruction(xRad, yRad, zRad, hEApp));
   }

   /**
    * Creates an ArcTorus centered on the current coordinate system.  An ArcTorus is a toroid shape beginning at the start
    * angle and wrapping around to the end angle.  Ensure that the angles don't overlap as the shape will appear inverted; the
    * start angle must be smaller than the end angle.  All angles are measured in radians.  The shape is also defined by its
    * major and minor radii.
    * </ br></ br>
    * The image below demonstrates a arctorus beginning at 3/2 pi and ending at 5/2 pi with major and minor radii of
    * 0.25 and 0.15 respectively:<br /><br />
    *
    * {@code linkGraphics.addCoordinateSystem(0.5);}<br />
    * {@code linkGraphics.addArcTorus(3*Math.PI/2, 5*Math.PI/2, 0.25, 0.15);}<br /><br />
    *
    * As is show by the graphical representation the arctorus is centered on the coordinate system.  Note that the ends
    * of the torus have no texture or surface, they are transparent.
    * Again, x, y and z are red, white and blue.
    * <br /><br /><img src="doc-files/LinkGraphics.addArcTorus1.jpg">
    *
    * @param startAngle Angle in radians at which the torus begins.
    * @param endAngle Angle in radians at which the torus ends
    * @param majorRadius Distance from the origin to the center of the torus
    * @param minorRadius Distance from the center of the torus to the walls on either side.
    */
   public void addArcTorus(double startAngle, double endAngle, double majorRadius, double minorRadius)
   {
      addArcTorus(startAngle, endAngle, majorRadius, minorRadius, YoAppearance.Black());
   }

   /**
    * Creates an ArcTorus centered on the current coordinate system with the specified appearance.  An ArcTorus is a toroid
    * shape beginning at the start angle and wrapping around to the end angle.  Ensure that the angles don't overlap as the
    * shape will appear inverted; the start angle must be smaller than the end angle.  All angles are measured in radians.
    * The shape is also defined by its major and minor radii.  The minor radius is from the origin to the inner wall of the
    * torus while the major is from the origin to the outer wall.
    * </ br></ br>
    * The image below demonstrates an aqua arctorus beginning at 0 and ending at 3/2 pi with major and minor radii of
    * 0.30 and 0.05 respectively:<br /><br />
    *
    * {@code linkGraphics.addCoordinateSystem(0.5);}<br />
    * {@code linkGraphics.addArcTorus(0, 3*Math.PI/2, 0.30, 0.05);}<br /><br />
    *
    * As is show by the graphical representation the arctorus is centered on the coordinate system.  Note that the ends
    * of the torus have no texture or surface, they are transparent.
    * Again, x, y and z are red, white and blue.
    * <br /><br /><img src="doc-files/LinkGraphics.addArcTorus2.jpg">
    *
    * @param startAngle Angle in radians at which the torus begins.
    * @param endAngle Angle in radians at which the torus ends.
    * @param majorRadius Distance from the origin to the center of the torus.
    * @param minorRadius Distance from the center of the torus to the walls on either side.
    * @param arcTorusApp Appearance to be used with the new arctorus.  See {@link YoAppearance YoAppearance} for implementations.
    */
   public Graphics3DAddArcTorusInstruction addArcTorus(double startAngle, double endAngle, double majorRadius, double minorRadius, AppearanceDefinition arcTorusApp)
   {
      Graphics3DAddArcTorusInstruction instruction = new Graphics3DAddArcTorusInstruction(startAngle, endAngle, majorRadius, minorRadius, arcTorusApp);
      graphics3DInstructions.add(instruction);
      return instruction;
   }

   /**
    * Creates a pyramid cube centered on the origin of the current coordinate system.  A pyramid cube is nothing more than
    * a standard cube of the given length, width and height with a square base pyramid of the specified height on the top and
    * bottom.
    * </ br></ br>
    * The image below demonstrates a pyramid cube beginning with dimensions of 0.2 by 0.2 by 0.2 and a pyramid height of 0.2:<br /><br />
    *
    * {@code linkGraphics.addCoordinateSystem(0.5);}<br />
    * {@code linkGraphics.addPyramidCube(0.2, 0.2, 0.2, 0.2);}<br /><br />
    *
    * As is show by the graphical representation the pyramid cube is centered on the coordinate system.
    * Again, x, y and z are red, white and blue.
    * <br /><br /><img src="doc-files/LinkGraphics.addPyramidCube1.jpg">
    *
    * @param lx Length in meters of the cube. (x direction)
    * @param ly Width in meters of the cube. (y direction)
    * @param lz Height of the cube in meters. (z direction)
    * @param lh Height of the pyramids in meters.
    */
   public void addPyramidCube(double lx, double ly, double lz, double lh)
   {
      addPyramidCube(lx, ly, lz, lh, YoAppearance.Black());
   }

   /**
    * Creates a pyramid cube with the specified appearance centered on the origin of the current coordinate system.
    * A pyramid cube is nothing more than a standard cube of the given length, width and height with a square base
    * pyramid of the specified height on the top and bottom.
    * </ br></ br>
    * The image below demonstrates a alluminum pyramid cube beginning with dimensions of 0.4 by 0.2 by 0.1 and a pyramid height of 0.3:<br /><br />
    *
    * {@code linkGraphics.addCoordinateSystem(0.5);}<br />
    * {@code linkGraphics.addPyramidCube(0.2, 0.2, 0.2, 0.2);}<br /><br />
    *
    * As is show by the graphical representation the pyramid cube is centered on the coordinate system.
    * Again, x, y and z are red, white and blue.
    * <br /><br /><img src="doc-files/LinkGraphics.addPyramidCube2.jpg">
    *
    * @param lx Length in meters of the cube. (x direction)
    * @param ly Width in meters of the cube. (y direction)
    * @param lz Height of the cube in meters. (z direction)
    * @param lh Height of the pyramids in meters.
    * @param cubeApp Appearance to be used with the new pyramid cube.  See {@link YoAppearance YoAppearance} for implementations.
    */
   public void addPyramidCube(double lx, double ly, double lz, double lh, AppearanceDefinition cubeApp)
   {
      graphics3DInstructions.add(new Graphics3DAddPyramidCubeInstruction(lx, ly, lz, lh, cubeApp));
   }
   
   public void addPolygon(ArrayList<Point3d> polygonPoints)
   {
      addPolygon(polygonPoints, YoAppearance.Black());
   }
   
   /**
    * Creates a polygon centered at the current coordinate system with the given vertices.
    * The points this shape is composed of must be coplanar and the order matters.  Randomly
    * inserting points will produce unpredictable results, clockwise direction determines the
    * side that is drawn.
    *
    * @param polygonPoints ArrayList containing the points.
    * @param appearance Appearance to be used with the new polygon.  See {@link YoAppearance YoAppearance} for implementations.
    */
   public void addPolygon(ArrayList<Point3d> polygonPoints, AppearanceDefinition yoAppearance)
   {
      graphics3DInstructions.add(new Graphics3DAddPolygonInstruction(polygonPoints, yoAppearance));
   }

   /**
    * Creates a polygon centered at the current coordinate system with the given vertices.
    * The points this shape is composed of must be coplanar and in a logical order.  Randomly
    * inserting points will produce unpredictable results, clockwise direction determines the
    * side that is drawn.
    *
    * @param polygonPoint Array containing Point3d's to be used when generating the shape.
    */
   public Graphics3DAddPolygonInstruction addPolygon(Point3d[] polygonPoint)
   {
      return addPolygon(polygonPoint, YoAppearance.Black());
   }

   /**
    * Creates a polygon centered at the current coordinate system with the given vertices.
    * The points this shape is composed of must be coplanar and the order matters.  Randomly
    * inserting points will produce unpredictable results, clockwise direction determines the
    * side that is drawn.
    *
    * @param polygonPoints Array containing the points
    * @param appearance Appearance to be used with the new polygon.  See {@link YoAppopearance YoAppearance} for implementations.
    */
   public Graphics3DAddPolygonInstruction addPolygon(Point3d[] polygonPoints, AppearanceDefinition yoAppearance)
   {
      Graphics3DAddPolygonInstruction graphics3DAddPolygonDouble = new Graphics3DAddPolygonInstruction(polygonPoints, yoAppearance);
      graphics3DInstructions.add(graphics3DAddPolygonDouble);
      return graphics3DAddPolygonDouble;
   }
   
   
   public void addExtrudedPolygon(List<Point2d> polygonPoints, double height)
   {
      addExtrudedPolygon(polygonPoints, height, YoAppearance.Black());
   }
   
   public void addExtrudedPolygon(List<Point2d> polygonPoints, double height, AppearanceDefinition appearance)
   {
      Graphics3DInstruction instruction = new Graphics3DAddExtrudedPolygonInstruction(polygonPoints, height, appearance);
      graphics3DInstructions.add(instruction);
   }

   public Graphics3DInstruction addText(String text, AppearanceDefinition yoAppearance)
   {
      Graphics3DInstruction instruction = new Graphics3DAddTextInstruction(text, yoAppearance);
      graphics3DInstructions.add(instruction);
      return instruction;
   }
   
   public void createInertiaEllipsoid(Matrix3d momentOfInertia, Vector3d comOffset, double mass, AppearanceDefinition appearance)
   {
      Vector3d principalMomentsOfInertia = new Vector3d(momentOfInertia.m00, momentOfInertia.m11, momentOfInertia.m22);
      Vector3d ellipsoidRadii = InertiaTools.getInertiaEllipsoidRadii(principalMomentsOfInertia, mass);

      this.addEllipsoid(ellipsoidRadii.x, ellipsoidRadii.y, ellipsoidRadii.z, appearance);
      this.identity();
   }

   
   public Graphics3DAddTeaPotInstruction addTeaPot(AppearanceDefinition appearance)
   {
      Graphics3DAddTeaPotInstruction graphics3DAddTeaPot = new Graphics3DAddTeaPotInstruction(appearance);
      graphics3DInstructions.add(graphics3DAddTeaPot);
      return graphics3DAddTeaPot;
   }

   public Graphics3DInstruction addHeightMap(HeightMap heightMap, int xPointsPerSide, int yPointsPerSide, AppearanceDefinition appearance)
   {
      return addHeightMap(heightMap, xPointsPerSide, yPointsPerSide, appearance, null);
   }
   
   public Graphics3DInstruction addHeightMap(HeightMap heightMap, int xPointsPerSide, int yPointsPerSide, AppearanceDefinition appearance, Transform3D transform)
   {
      Graphics3DAddHeightMapInstruction instruction = new Graphics3DAddHeightMapInstruction(heightMap, xPointsPerSide, yPointsPerSide, appearance, transform);
      graphics3DInstructions.add(instruction);
      return instruction;
   }
   
   public void notifySelectedListeners(Graphics3DNode graphics3dNode, ModifierKeyInterface modifierKeyHolder, Point3d location, Point3d cameraPosition, Quat4d cameraRotation)
   {
      for(SelectedListener selectedListener : selectedListeners)
      {
         selectedListener.selected(graphics3dNode, modifierKeyHolder, location, cameraPosition, cameraRotation);
      }
   }
   
   public void registerSelectedListener(SelectedListener selectedListener)
   {
      selectedListeners.add(selectedListener);
   }
}
