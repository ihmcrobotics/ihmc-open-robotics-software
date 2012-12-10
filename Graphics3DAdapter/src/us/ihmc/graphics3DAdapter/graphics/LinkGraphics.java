package us.ihmc.graphics3DAdapter.graphics;

import java.net.URL;
import java.util.ArrayList;
import java.util.List;

import javax.media.j3d.Appearance;
import javax.media.j3d.Transform3D;
import javax.vecmath.Color3f;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.HeightMap;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsAddArcTorus;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsAddCone;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsAddCoordinateSystem;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsAddCube;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsAddCylinder;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsAddEllipsoid;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsAddExtrudedPolygon;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsAddHeightMap;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsAddHemiEllipsoid;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsAddMeshData;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsAddModelFile;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsAddPolygonDouble;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsAddPyramidCube;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsAddSphere;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsAddTeaPot;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsAddText;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsAddTruncatedCone;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsAddVRMLFile;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsAddWedge;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsIdentity;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsPrimitiveInstruction;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsRotate;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsRotateMatrix;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsScale;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsTranslate;
import us.ihmc.utilities.InertiaTools;

/**
 * <p>Title: SimulationConstructionSet</p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2000</p>
 *
 * <p>Company: Yobotics, Inc.</p>
 *
 * @author not attributable
 * @version 1.0
 */
public class LinkGraphics
{
   private enum Axis
   {
      X, Y, Z
   }
   
   public static final Axis X = Axis.X;
   public static final Axis Y = Axis.Y;
   public static final Axis Z = Axis.Z;
   
   private ArrayList<LinkGraphicsPrimitiveInstruction> linkGraphicsInstructions;

   public LinkGraphics(ArrayList<LinkGraphicsPrimitiveInstruction> linkGraphicsInstructions)
   {
      this();
      this.linkGraphicsInstructions = linkGraphicsInstructions;
   }

   /**
    * Default no-arg constructor.  This creates a new empty LinkGraphics component.
    */
   public LinkGraphics()
   {
      linkGraphicsInstructions = new ArrayList<LinkGraphicsPrimitiveInstruction>();
   }

   /**
    * gets a color from an appearance
    */
   public Color3f getColor(Appearance app)
   {
      Color3f color = new Color3f();
      if (app.getMaterial() != null)
         app.getMaterial().getAmbientColor(color);
      else
         return null;

      return color;
   }

   public ArrayList<LinkGraphicsPrimitiveInstruction> getLinkGraphicsInstructions()
   {
      return linkGraphicsInstructions;
   }
   
   /**
    * Merge this with the specified LinkGraphics.
    *
    * @param linkGraphics LinkGraphics to combine with.
    */
   public void combine(LinkGraphics linkGraphics)
   {
      this.identity();
      this.linkGraphicsInstructions.addAll(linkGraphics.getLinkGraphicsInstructions());
   }
   
   public void addInstruction(LinkGraphicsPrimitiveInstruction instruction)
   {
      this.linkGraphicsInstructions.add(instruction);
   }

   /*
    * public void removeAllGraphics() { for(int i=0; i<linkBG.numChildren();
    * i++) { //Node child = linkBG.getChild(i); linkBG.removeChild(i); }
    * 
    * this.numShapes = 0; this.lastGroup = linkBG; }
    */

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
      linkGraphicsInstructions.add(new LinkGraphicsTranslate(tx, ty, tz));
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
      linkGraphicsInstructions.add(new LinkGraphicsTranslate(translation));
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
      linkGraphicsInstructions.add(new LinkGraphicsRotate(rotAng, rotAxis));
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
      linkGraphicsInstructions.add(new LinkGraphicsRotateMatrix(rot));
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
   public LinkGraphicsScale scale(double scaleFactor)
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
   public LinkGraphicsScale scale(Vector3d scaleFactors)
   {
      LinkGraphicsScale linkGraphicsScale = new LinkGraphicsScale(scaleFactors);
      linkGraphicsInstructions.add(linkGraphicsScale);
      return linkGraphicsScale;
   }

   /**
    * Resets the coordinate system to the joint origin.  This clears all rotations, translations,
    * and scale factors.
    */
   public void identity()
   {
      linkGraphicsInstructions.add(new LinkGraphicsIdentity());
   }


   /*
    * protected void addTransformGroup(TransformGroup transGroup) {
    * this.lastGroup.addChild(transGroup); this.numShapes++; }
    */

   /**
    * Adds the specified VRML file to the center of the current coordinate system
    * using the default appearance.  VRML, or Virtual Reality Modeling Language, is
    * a standard file format describing 3d objects.  For more information google
    * VRML and read about it.
    *
    * @param fileURL URL describing the location of the VMRL file.
    */
   public void addVRMLFile(URL fileURL)
   {
      addVRMLFile(fileURL, null);

      // linkGraphicsInstructions.add(new LinkGraphicsAddVRMLFile(fileURL.getPath()));
   }

   /**
    * Adds the specified VRML file to the center of the current coordinate system
    * using the provided appearance.  VRML or Virtual Reality Modeling Language is
    * a standard file format describing 3d objects.  For more information google
    * VRML and read about it.
    *
    * @param fileUrl URL describing the location of the VRML file.
    * @param yoAppearanceDefinition Appearance to use in the creation of this shape.
    */
   public void addVRMLFile(URL fileUrl, YoAppearanceDefinition yoAppearanceDefinition)
   {
      
      linkGraphicsInstructions.add(new LinkGraphicsAddVRMLFile(fileUrl.getPath(), yoAppearanceDefinition));

   }

   /**
    * Adds the specified VRML file to the center of the current coordinate system
    * using the provided appearance.  VRML or Virtual Reality Modeling Language is
    * a standard file format describing 3d objects.  For more information google
    * VRML and read about it.
    *
    * @param fileName Path to the desired file.
    * @param app Appearance to use with the VRML file.
    */
   public void addVRMLFile(String fileName, YoAppearanceDefinition app)
   {
      
         linkGraphicsInstructions.add(new LinkGraphicsAddVRMLFile(fileName, app));
   }

   /**
    * Adds the specified VRML file to the center of the current coordinate system
    * using the default appearance.  VRML, or Virtual Reality Modeling Language, is
    * a standard file format describing 3d objects.  For more information google
    * VRML and read about it.
    *
    * @param fileName String
    */
   public void addVRMLFile(String fileName)
   {
      addVRMLFile(fileName, null);

      // linkGraphicsInstructions.add(new LinkGraphicsAddVRMLFile(fileName));
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

      // linkGraphicsInstructions.add(new LinkGraphicsAdd3DSFile(fileURL.getPath()));
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
   public void addModelFile(URL fileURL, YoAppearanceDefinition yoAppearanceDefinition)
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
   public void addModelFile(String fileName, YoAppearanceDefinition app)
   {
     
         linkGraphicsInstructions.add(new LinkGraphicsAddModelFile(fileName, app));
   }

   public void addCoordinateSystem(double length)
   {
      addCoordinateSystem(length, YoAppearance.Black());
   }

   public void addCoordinateSystem(double length, YoAppearanceDefinition arrowAppearance)
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
   public void addCoordinateSystem(double length, YoAppearanceDefinition xAxisAppearance, YoAppearanceDefinition yAxisAppearance, YoAppearanceDefinition zAxisAppearance, YoAppearanceDefinition arrowAppearance)
   {
     
      linkGraphicsInstructions.add(new LinkGraphicsAddCoordinateSystem(length));
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

      // linkGraphicsInstructions.add(new LinkGraphicsAddCube(lx, ly, lz));
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
   public void addCube(double lx, double ly, double lz, YoAppearanceDefinition cubeApp)
   {
     
         linkGraphicsInstructions.add(new LinkGraphicsAddCube(lx, ly, lz, cubeApp));
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

      // linkGraphicsInstructions.add(new LinkGraphicsAddWedge(lx, ly, lz));
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
   public void addWedge(double lx, double ly, double lz, YoAppearanceDefinition wedgeApp)
   {
         linkGraphicsInstructions.add(new LinkGraphicsAddWedge(lx, ly, lz, wedgeApp));
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

      //    linkGraphicsInstructions.add(new LinkGraphicsAddSphere(radius));
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
   public LinkGraphicsAddSphere addSphere(double radius, YoAppearanceDefinition sphereApp)
   {
      LinkGraphicsAddSphere instruction = new LinkGraphicsAddSphere(radius, sphereApp);
      linkGraphicsInstructions.add(instruction);
      return instruction;
   }

   public LinkGraphicsAddMeshData addMeshData(MeshDataHolder meshData, YoAppearanceDefinition meshAppearance)
   {
      LinkGraphicsAddMeshData instruction = new LinkGraphicsAddMeshData(meshData, meshAppearance);
      linkGraphicsInstructions.add(instruction);
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
   public void addEllipsoid(double xRad, double yRad, double zRad)
   {
      addEllipsoid(xRad, yRad, zRad, YoAppearance.Black());

      // linkGraphicsInstructions.add(new LinkGraphicsAddEllipsoid(xRad, yRad, zRad));
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
   public LinkGraphicsAddEllipsoid addEllipsoid(double xRad, double yRad, double zRad, YoAppearanceDefinition ellipsoidApp)
   {
         LinkGraphicsAddEllipsoid instruction = new LinkGraphicsAddEllipsoid(xRad, yRad, zRad, ellipsoidApp);
         linkGraphicsInstructions.add(instruction);
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

      // linkGraphicsInstructions.add(new LinkGraphicsAddCylinder(height, radius));
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
   public LinkGraphicsAddCylinder addCylinder(double height, double radius, YoAppearanceDefinition cylApp)
   {
      
         LinkGraphicsAddCylinder instruction = new LinkGraphicsAddCylinder(height, radius, cylApp);
         linkGraphicsInstructions.add(instruction);
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

      // linkGraphicsInstructions.add(new LinkGraphicsAddCone(height, radius));
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
   public void addCone(double height, double radius, YoAppearanceDefinition coneApp)
   {
      
         linkGraphicsInstructions.add(new LinkGraphicsAddCone(height, radius, coneApp));
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

      // linkGraphicsInstructions.add(new LinkGraphicsAddTruncatedCone(height, bx, by, tx, ty));
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
   public void addGenTruncatedCone(double height, double bx, double by, double tx, double ty, YoAppearanceDefinition coneApp)
   {
         linkGraphicsInstructions.add(new LinkGraphicsAddTruncatedCone(height, bx, by, tx, ty, coneApp));
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

      // linkGraphicsInstructions.add(new LinkGraphicsAddHemiEllipsoid(xRad, yRad, zRad));
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
   public void addHemiEllipsoid(double xRad, double yRad, double zRad, YoAppearanceDefinition hEApp)
   {
         linkGraphicsInstructions.add(new LinkGraphicsAddHemiEllipsoid(xRad, yRad, zRad, hEApp));
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

      // addCylinder(1.0f, 0.2f);
      // linkGraphicsInstructions.add(new LinkGraphicsAddArcTorus(startAngle, endAngle, majorRadius, minorRadius));
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
   public LinkGraphicsAddArcTorus addArcTorus(double startAngle, double endAngle, double majorRadius, double minorRadius, YoAppearanceDefinition arcTorusApp)
   {

         LinkGraphicsAddArcTorus instruction = new LinkGraphicsAddArcTorus(startAngle, endAngle, majorRadius, minorRadius, arcTorusApp);
         linkGraphicsInstructions.add(instruction);
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

      // linkGraphicsInstructions.add(new LinkGraphicsAddPyramidCube(lx, ly, lz, lh));
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
   public void addPyramidCube(double lx, double ly, double lz, double lh, YoAppearanceDefinition cubeApp)
   {
      linkGraphicsInstructions.add(new LinkGraphicsAddPyramidCube(lx, ly, lz, lh, cubeApp));
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
   public void addPolygon(ArrayList<Point3d> polygonPoints, YoAppearanceDefinition yoAppearance)
   {
      linkGraphicsInstructions.add(new LinkGraphicsAddPolygonDouble(polygonPoints, yoAppearance));
   }

   /**
    * Creates a polygon centered at the current coordinate system with the given vertices.
    * The points this shape is composed of must be coplanar and in a logical order.  Randomly
    * inserting points will produce unpredictable results, clockwise direction determines the
    * side that is drawn.
    *
    * @param polygonPoint Array containing Point3d's to be used when generating the shape.
    */
   public void addPolygon(Point3d[] polygonPoint)
   {
      addPolygon(polygonPoint, YoAppearance.Black());

      // linkGraphicsInstructions.add(new LinkGraphicsAddPolygonDouble(polygonPoint));
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
   public void addPolygon(Point3d[] polygonPoints, YoAppearanceDefinition yoAppearance)
   {
      linkGraphicsInstructions.add(new LinkGraphicsAddPolygonDouble(polygonPoints, yoAppearance));
   }
   
   
   public void addExtrudedPolygon(List<Point2d> polygonPoints, double height)
   {
      addExtrudedPolygon(polygonPoints, height, YoAppearance.Black());
   }
   
   public void addExtrudedPolygon(List<Point2d> polygonPoints, double height, YoAppearanceDefinition appearance)
   {
      LinkGraphicsInstruction instruction = new LinkGraphicsAddExtrudedPolygon(polygonPoints, height, appearance);
      linkGraphicsInstructions.add(instruction);
   }

   public LinkGraphicsInstruction addText(String text, YoAppearanceDefinition yoAppearance)
   {
      LinkGraphicsInstruction instruction = new LinkGraphicsAddText(text, yoAppearance);
      linkGraphicsInstructions.add(instruction);
      return instruction;
      
   }
   
   public void createInertiaEllipsoid(Matrix3d momentOfInertia, Vector3d comOffset, double mass, YoAppearanceDefinition appearance)
   {
      Vector3d principalMomentsOfInertia = new Vector3d(momentOfInertia.m00, momentOfInertia.m11, momentOfInertia.m22);
      Vector3d ellipsoidRadii = InertiaTools.getInertiaEllipsoidRadii(principalMomentsOfInertia, mass);

      this.addEllipsoid(ellipsoidRadii.x, ellipsoidRadii.y, ellipsoidRadii.z, appearance);

      this.identity();
   }

   
   public LinkGraphicsAddTeaPot addTeaPot(YoAppearanceDefinition appearance)
   {
      LinkGraphicsAddTeaPot linkGraphicsAddTeaPot = new LinkGraphicsAddTeaPot(appearance);
      linkGraphicsInstructions.add(linkGraphicsAddTeaPot);
      return linkGraphicsAddTeaPot;
   }

   public LinkGraphicsInstruction addHeightMap(HeightMap heightMap, int xPointsPerSide, int yPointsPerSide, YoAppearanceDefinition appearance)
   {
      return addHeightMap(heightMap, xPointsPerSide, yPointsPerSide, appearance, null);
   }
   
   public LinkGraphicsInstruction addHeightMap(HeightMap heightMap, int xPointsPerSide, int yPointsPerSide, YoAppearanceDefinition appearance, Transform3D transform)
   {
      LinkGraphicsAddHeightMap instruction = new LinkGraphicsAddHeightMap(heightMap, xPointsPerSide, yPointsPerSide, appearance, transform);
      linkGraphicsInstructions.add(instruction);
      return instruction;
   }


   
//   /*
//    * J3D transition stuff
//    */
//   private J3DLinkGraphics j3dLinkGraphics = null;
//   public SharedGroup getSharedGroup()
//   {
//      if(j3dLinkGraphics == null)
//         j3dLinkGraphics = new J3DLinkGraphics(this);
//      
//      return j3dLinkGraphics.getSharedGroup();
//   }
//   
//   public void updateGraphics()
//   {
//      if(j3dLinkGraphics != null)
//         j3dLinkGraphics.updateGraphics();
//   }
}
