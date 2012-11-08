package com.yobotics.simulationconstructionset;

import java.io.FileNotFoundException;
import java.net.URL;
import java.util.ArrayList;
import java.util.Enumeration;

import javax.media.j3d.Appearance;
import javax.media.j3d.BranchGroup;
import javax.media.j3d.Geometry;
import javax.media.j3d.Group;
import javax.media.j3d.Leaf;
import javax.media.j3d.Light;
import javax.media.j3d.Material;
import javax.media.j3d.Node;
import javax.media.j3d.PolygonAttributes;
import javax.media.j3d.Shape3D;
import javax.media.j3d.SharedGroup;
import javax.media.j3d.Transform3D;
import javax.media.j3d.TransformGroup;
import javax.vecmath.AxisAngle4d;
import javax.vecmath.Color3f;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3d;

import org.j3d.renderer.java3d.loaders.STLLoader;

import us.ihmc.utilities.InertiaTools;
import us.ihmc.utilities.math.geometry.GeometryGenerator;

import com.eteks.sweethome3d.j3d.DAELoader;
import com.mnstarfire.loaders3d.Loader3DS;
import com.sun.j3d.loaders.Loader;
import com.sun.j3d.utils.picking.PickTool;
import com.yobotics.simulationconstructionset.robotdefinition.AppearanceDefinition;
import com.yobotics.simulationconstructionset.robotdefinition.LinkGraphicsDefinition;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsAddArcTorus;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsAddCone;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsAddCoordinateSystem;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsAddCube;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsAddCylinder;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsAddEllipsoid;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsAddHemiEllipsoid;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsAddModelFile;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsAddPolygonDouble;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsAddPolygonFloat;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsAddPyramidCube;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsAddSphere;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsAddTruncatedCone;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsAddVRMLFile;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsAddWedge;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsIdentity;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsInstruction;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsRotate;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsRotateDefinedAxis;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsRotateMatrix;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsScale;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsTranslate;

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
   // private BranchGroup linkBG;
   private SharedGroup sharedGroup;
   private int numShapes;
   private Group lastGroup;
   private LinkGraphicsDefinition linkGraphicsDefinition;

   public LinkGraphics(LinkGraphicsDefinition linkGraphicsDefinition)
   {
      this();
      setUpGraphicsFromDefinition(linkGraphicsDefinition);
   }

   private void setUpGraphicsFromDefinition(LinkGraphicsDefinition graphicsDefinition)
   {
      // LinkGraphics this = new LinkGraphics();

      for (LinkGraphicsInstruction instruction : graphicsDefinition.getInstructions())
      {
         if (instruction instanceof LinkGraphicsAddModelFile)
         {
            // Appearance app = new Appearance();

            if (((LinkGraphicsAddModelFile) instruction).getAppearance() != null)
            {
               Color3f color = ((LinkGraphicsAddModelFile) instruction).getAppearance().getColor();
               Appearance app = new Appearance();
               Material tmp = new Material();
               tmp.setAmbientColor(color);
               app.setMaterial(tmp); // YoAppearance.Color(new Color(color.x, color.y, color.z));
               this.addModelFile(((LinkGraphicsAddModelFile) instruction).getFileName(), app);
            }
            else
               this.addModelFile(((LinkGraphicsAddModelFile) instruction).getFileName());

         }
         else if (instruction instanceof LinkGraphicsAddArcTorus)
         {
            if (((LinkGraphicsAddArcTorus) instruction).getAppearance() != null)
            {
               Color3f color = ((LinkGraphicsAddArcTorus) instruction).getAppearance().getColor();
               Appearance app = new Appearance();
               Material tmp = new Material();
               tmp.setAmbientColor(color);
               app.setMaterial(tmp);
               this.addArcTorus(((LinkGraphicsAddArcTorus) instruction).getStartAngle(), ((LinkGraphicsAddArcTorus) instruction).getEndAngle(),
                     ((LinkGraphicsAddArcTorus) instruction).getMajorRadius(), ((LinkGraphicsAddArcTorus) instruction).getMinorRadius(), app);
            }
            else
               this.addArcTorus(((LinkGraphicsAddArcTorus) instruction).getStartAngle(), ((LinkGraphicsAddArcTorus) instruction).getEndAngle(),
                     ((LinkGraphicsAddArcTorus) instruction).getMajorRadius(), ((LinkGraphicsAddArcTorus) instruction).getMinorRadius());

         }
         else if (instruction instanceof LinkGraphicsAddCone)
         {
            if (((LinkGraphicsAddCone) instruction).getAppearance() != null)
            {
               Color3f color = ((LinkGraphicsAddCone) instruction).getAppearance().getColor();
               Appearance app = new Appearance();
               Material tmp = new Material();
               tmp.setAmbientColor(color);
               app.setMaterial(tmp);

               this.addCone(((LinkGraphicsAddCone) instruction).getHeight(), ((LinkGraphicsAddCone) instruction).getRadius(), app);
            }
            else
               this.addCone(((LinkGraphicsAddCone) instruction).getHeight(), ((LinkGraphicsAddCone) instruction).getRadius());
         }
         else if (instruction instanceof LinkGraphicsAddCylinder)
         {
            if (((LinkGraphicsAddCylinder) instruction).getAppearance() != null)
            {
               Color3f color = ((LinkGraphicsAddCylinder) instruction).getAppearance().getColor();
               Appearance app = new Appearance();
               Material tmp = new Material();
               tmp.setAmbientColor(color);
               app.setMaterial(tmp);
               this.addCylinder(((LinkGraphicsAddCylinder) instruction).getHeight(), ((LinkGraphicsAddCylinder) instruction).getRadius(), app);

            }
            else
               this.addCylinder(((LinkGraphicsAddCylinder) instruction).getHeight(), ((LinkGraphicsAddCylinder) instruction).getRadius());
         }
         else if (instruction instanceof LinkGraphicsAddCoordinateSystem)
         {
            this.addCoordinateSystem(((LinkGraphicsAddCoordinateSystem) instruction).getLength());
         }
         else if (instruction instanceof LinkGraphicsAddCube)
         {
            if (((LinkGraphicsAddCube) instruction).getAppearance() != null)
            {
               Color3f color = ((LinkGraphicsAddCube) instruction).getAppearance().getColor();
               Appearance app = new Appearance();
               Material tmp = new Material();
               tmp.setAmbientColor(color);
               app.setMaterial(tmp);

               this.addCube(((LinkGraphicsAddCube) instruction).getX(), ((LinkGraphicsAddCube) instruction).getY(), ((LinkGraphicsAddCube) instruction).getZ(),
                     app);
            }
            else
               this.addCube(((LinkGraphicsAddCube) instruction).getX(), ((LinkGraphicsAddCube) instruction).getY(), ((LinkGraphicsAddCube) instruction).getZ());
         }
         else if (instruction instanceof LinkGraphicsAddEllipsoid)
         {
            if (((LinkGraphicsAddEllipsoid) instruction).getAppearance() != null)
            {
               Color3f color = ((LinkGraphicsAddEllipsoid) instruction).getAppearance().getColor();
               Appearance app = new Appearance();
               Material tmp = new Material();
               tmp.setAmbientColor(color);
               app.setMaterial(tmp);
               this.addEllipsoid(((LinkGraphicsAddEllipsoid) instruction).getXRad(), ((LinkGraphicsAddEllipsoid) instruction).getYRad(),
                     ((LinkGraphicsAddEllipsoid) instruction).getZRad(), app);
            }
            else
               this.addEllipsoid(((LinkGraphicsAddEllipsoid) instruction).getXRad(), ((LinkGraphicsAddEllipsoid) instruction).getYRad(),
                     ((LinkGraphicsAddEllipsoid) instruction).getZRad());
         }
         else if (instruction instanceof LinkGraphicsAddHemiEllipsoid)
         {
            if (((LinkGraphicsAddHemiEllipsoid) instruction).getAppearance() != null)
            {
               Color3f color = ((LinkGraphicsAddHemiEllipsoid) instruction).getAppearance().getColor();
               Appearance app = new Appearance();
               Material tmp = new Material();
               tmp.setAmbientColor(color);
               app.setMaterial(tmp);
               this.addHemiEllipsoid(((LinkGraphicsAddHemiEllipsoid) instruction).getXRad(), ((LinkGraphicsAddHemiEllipsoid) instruction).getYRad(),
                     ((LinkGraphicsAddHemiEllipsoid) instruction).getZRad(), app);
            }
            else
               this.addHemiEllipsoid(((LinkGraphicsAddHemiEllipsoid) instruction).getXRad(), ((LinkGraphicsAddHemiEllipsoid) instruction).getYRad(),
                     ((LinkGraphicsAddHemiEllipsoid) instruction).getZRad());

         }
         else if (instruction instanceof LinkGraphicsAddPyramidCube)
         {
            if (((LinkGraphicsAddPyramidCube) instruction).getAppearance() != null)
            {
               Color3f color = ((LinkGraphicsAddPyramidCube) instruction).getAppearance().getColor();
               Appearance app = new Appearance();
               Material tmp = new Material();
               tmp.setAmbientColor(color);
               app.setMaterial(tmp);
               this.addPyramidCube(((LinkGraphicsAddPyramidCube) instruction).getX(), ((LinkGraphicsAddPyramidCube) instruction).getY(),
                     ((LinkGraphicsAddPyramidCube) instruction).getZ(), ((LinkGraphicsAddPyramidCube) instruction).getHeight(), app);
            }
            else
               this.addPyramidCube(((LinkGraphicsAddPyramidCube) instruction).getX(), ((LinkGraphicsAddPyramidCube) instruction).getY(),
                     ((LinkGraphicsAddPyramidCube) instruction).getZ(), ((LinkGraphicsAddPyramidCube) instruction).getHeight());
         }
         else if (instruction instanceof LinkGraphicsAddSphere)
         {
            if (((LinkGraphicsAddSphere) instruction).getAppearance() != null)
            {
               Color3f color = ((LinkGraphicsAddSphere) instruction).getAppearance().getColor();
               Appearance app = new Appearance();
               Material tmp = new Material();
               tmp.setAmbientColor(color);
               app.setMaterial(tmp);
               this.addSphere(((LinkGraphicsAddSphere) instruction).getRadius(), app);
            }
            else
               this.addSphere(((LinkGraphicsAddSphere) instruction).getRadius());
         }
         else if (instruction instanceof LinkGraphicsAddVRMLFile)
         {
            if (((LinkGraphicsAddVRMLFile) instruction).getAppearance() != null)
            {
               Color3f color = ((LinkGraphicsAddVRMLFile) instruction).getAppearance().getColor();
               Appearance app = new Appearance();
               Material tmp = new Material();
               tmp.setAmbientColor(color);
               app.setMaterial(tmp);
               this.addVRMLFile(((LinkGraphicsAddVRMLFile) instruction).getFileName(), app);
            }
            else
               this.addVRMLFile(((LinkGraphicsAddVRMLFile) instruction).getFileName());
         }
         else if (instruction instanceof LinkGraphicsAddWedge)
         {
            if (((LinkGraphicsAddWedge) instruction).getAppearance() != null)
            {
               Color3f color = ((LinkGraphicsAddWedge) instruction).getAppearance().getColor();
               Appearance app = new Appearance();
               Material tmp = new Material();
               tmp.setAmbientColor(color);
               app.setMaterial(tmp);
               this.addWedge(((LinkGraphicsAddWedge) instruction).getX(), ((LinkGraphicsAddWedge) instruction).getY(),
                     ((LinkGraphicsAddWedge) instruction).getZ(), app);
            }
            else
               this.addWedge(((LinkGraphicsAddWedge) instruction).getX(), ((LinkGraphicsAddWedge) instruction).getY(),
                     ((LinkGraphicsAddWedge) instruction).getZ());
         }
         else if (instruction instanceof LinkGraphicsAddTruncatedCone)
         {
            if (((LinkGraphicsAddTruncatedCone) instruction).getAppearance() != null)
            {
               Color3f color = ((LinkGraphicsAddTruncatedCone) instruction).getAppearance().getColor();
               Appearance app = new Appearance();
               Material tmp = new Material();
               tmp.setAmbientColor(color);
               app.setMaterial(tmp);
               this.addGenTruncatedCone(((LinkGraphicsAddTruncatedCone) instruction).getHeight(), ((LinkGraphicsAddTruncatedCone) instruction).getBX(),
                     ((LinkGraphicsAddTruncatedCone) instruction).getBY(), ((LinkGraphicsAddTruncatedCone) instruction).getTX(),
                     ((LinkGraphicsAddTruncatedCone) instruction).getTY(), app);
            }
            else
               this.addGenTruncatedCone(((LinkGraphicsAddTruncatedCone) instruction).getHeight(), ((LinkGraphicsAddTruncatedCone) instruction).getBX(),
                     ((LinkGraphicsAddTruncatedCone) instruction).getBY(), ((LinkGraphicsAddTruncatedCone) instruction).getTX(),
                     ((LinkGraphicsAddTruncatedCone) instruction).getTY());
         }
         else if (instruction instanceof LinkGraphicsIdentity)
         {
            this.identity();
         }
         else if (instruction instanceof LinkGraphicsRotate)
         {
            this.rotate(((LinkGraphicsRotate) instruction).getAngle(), ((LinkGraphicsRotate) instruction).getAxis());
         }
         else if (instruction instanceof LinkGraphicsRotateDefinedAxis)
         {
            this.rotate(((LinkGraphicsRotateDefinedAxis) instruction).getAngle(), ((LinkGraphicsRotateDefinedAxis) instruction).getAxis());
         }
         else if (instruction instanceof LinkGraphicsRotateMatrix)
         {
            this.rotate(((LinkGraphicsRotateMatrix) instruction).getRotationMatrix());
         }
         else if (instruction instanceof LinkGraphicsScale)
         {
            this.scale(((LinkGraphicsScale) instruction).getScaleFactor());
         }
         else if (instruction instanceof LinkGraphicsTranslate)
         {
            this.translate(((LinkGraphicsTranslate) instruction).getTranslation());
         }

      }

      // return this;
   }

   /**
    * Default no-arg constructor.  This creates a new empty LinkGraphics component.
    */
   public LinkGraphics()
   {
      // this.linkBG = new BranchGroup();
      this.sharedGroup = new SharedGroup();

      sharedGroup.setCapability(BranchGroup.ALLOW_DETACH);

      this.numShapes = 0;
      this.lastGroup = this.sharedGroup;

      linkGraphicsDefinition = new LinkGraphicsDefinition();
   }

   public void setSharedGroup(SharedGroup group)
   {
      // this.linkBG = new BranchGroup();
      this.sharedGroup = group;

      sharedGroup.setCapability(BranchGroup.ALLOW_DETACH);

      this.numShapes = 0;
      this.lastGroup = this.sharedGroup;
   }

   public LinkGraphicsDefinition getLinkGraphicsDefinition()
   {
      return linkGraphicsDefinition;
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

   /**
    * Merge this with the specified LinkGraphics.
    *
    * @param linkGraphics LinkGraphics to combine with.
    */
   public void combine(LinkGraphics linkGraphics)
   {
      // Add the links graphics:
      javax.media.j3d.Link link3d = new javax.media.j3d.Link();
      link3d.setSharedGroup(linkGraphics.sharedGroup);

      this.sharedGroup.addChild(link3d);
      this.linkGraphicsDefinition.combineGraphicsInstructions(linkGraphics.getLinkGraphicsDefinition());

      // this.linkBG.addChild(link.linkBG);
   }

   /*
    * public void removeAllGraphics() { for(int i=0; i<linkBG.numChildren();
    * i++) { //Node child = linkBG.getChild(i); linkBG.removeChild(i); }
    * 
    * this.numShapes = 0; this.lastGroup = linkBG; }
    */

   /**
    * Configures the passed in node based on its type.  If the node is a Shape3D or a
    * Light configure its appearance, otherwise it is probably a group. In that case
    * recurse through the group and enable its components.
    *
    * @param node Node to enable.
    * @param app Appearance to use if the node is a Shape3D.  If node is a group this appearance is passed down the tree.
    * @param level current level of the recursion, used only for debugging
    */
   private static void recursiveSetEnabling(Node node, Appearance app, int level)
   {
      // Stuff to do to every node no matter what its type:

      // node.setCapability(Node.ALLOW_PICKABLE_READ);
      // node.setCapability(Node.ALLOW_PICKABLE_WRITE);
      // System.out.print(level + ": ");
      // Stuff to do to a node if it is a leaf:
      if (node instanceof Leaf)
      {
         // System.out.println("node is a leaf");
         Leaf leaf = (Leaf) node;

         if (leaf instanceof Shape3D)
         {
            // System.out.println("leaf is a Shape3D");
            Shape3D shape = (Shape3D) leaf;

            if (app != null)
               shape.setAppearance(app);

            // System.out.println(shape);
            PickTool.setCapabilities(shape, PickTool.INTERSECT_FULL);
         }

         else if (leaf instanceof Light)
         {
            // System.out.println("leaf is a light");
            @SuppressWarnings("unused")
            Light light = (Light) leaf;
         }

         // else System.out.println("leaf is NOT a Shape3D");
      }

      // Stuff to do to a node if its a Group:
      else if (node instanceof Group)
      {
         Group group = (Group) node;

         // System.out.println("node is a group with " + group.numChildren() + "children");

         Enumeration<?> e = group.getAllChildren();

         // if (group instanceof BranchGroup) System.out.println("Group is a BranchGroup!");
         // else if (group instanceof TransformGroup) System.out.println("Group is a TransformGroup!");
         // else System.out.println("Group is a neither a BranchGroup or a TransformGroup!");

         while (e.hasMoreElements())
         {
            Node child = (Node) e.nextElement();

            recursiveSetEnabling(child, app, level + 1);
         }
      }

      // Stuff to do if neither a leaf or a group (not sure if possible?)
      else
      {
         System.out.println("node is neither a group nor a leaf");
      }
   }

   /**
    * Creates a TransformGroup based on the specified translation in the x, y and z directions.
    *
    * @param tx distance in the x direction
    * @param ty distance in the y direction
    * @param tz distance in the z direction
    * @return TransformGroup incorporating the desired translation
    */
   private static TransformGroup translateTransformGroup(double tx, double ty, double tz)
   {
      Transform3D t1 = new Transform3D();

      Vector3d vec = new Vector3d(tx, ty, tz);
      t1.set(vec);
      TransformGroup transGroup = new TransformGroup(t1);

      return transGroup;
   }

   /**
    * Creates a TransformGroup based on the specified rotation axis and angle.
    *
    * @param rotAng angle, in radians, to rotate around the axis
    * @param rotAxis axis (X = 0, Y = 1, or Z = 2) to rotate around.
    * If the number is not one of these the z axis will be selected
    * @return TransformGroup incorporating the desired rotation
    */
   private static TransformGroup rotateTransformGroup(double rotAng, int rotAxis)
   {
      Transform3D t1 = new Transform3D();

      if (rotAxis == Link.X)
         t1.rotX(rotAng);
      else if (rotAxis == Link.Y)
         t1.rotY(rotAng);

      // else if (rotAxis == Link.Z)
      else
         t1.rotZ(rotAng);

      // else return;

      TransformGroup transGroup = new TransformGroup(t1);

      return transGroup;
   }

   /**
    * Creates a TransformGroup based on the specified rotation axis and angle.
    *
    * @param rotAng angle, in radians, to rotate around the axis
    * @param rotAxis Vector3d representing the desired rotation axis
    * @return TransformGroup incorporating the desired rotation
    */
   private static TransformGroup rotateTransformGroup(double rotAng, Vector3d rotAxis)
   {
      Transform3D t1 = new Transform3D();
      AxisAngle4d axisAngle4d = new AxisAngle4d(rotAxis, rotAng);
      t1.setRotation(axisAngle4d);

      TransformGroup transGroup = new TransformGroup(t1);

      return transGroup;
   }

   /**
    * Creates a TransformGroup based on the specified rotation matrix.
    *
    * @param rotMatrix Matrix3d containing the rotation
    * @return TransformGroup incorporating the desired rotation
    */
   private static TransformGroup rotateTransformGroup(Matrix3d rotMatrix)
   {
      Transform3D t1 = new Transform3D(rotMatrix, new Vector3d(), 1.0);
      TransformGroup transGroup = new TransformGroup(t1);

      return transGroup;
   }

   /**
    * Creates a TransformGroup which scales the coordinate system by the specified factor
    *
    * @param scaleFactor factor to scale the system by
    * @return TransformGroup incorporating the desired scale
    */
   private static TransformGroup scaleTransformGroup(double scaleFactor)
   {
      Transform3D t1 = new Transform3D();

      t1.setScale(scaleFactor);

      TransformGroup transGroup = new TransformGroup(t1);

      return transGroup;
   }

   /**
    * Creates a TransformGroup which scales the coordinate system by the specified factors
    * on each axis.
    *
    * @param scaleFactors Vector3d representing the scale factor for each axis.  The x component
    * scales the x axis, the y scales y axis and so on.
    * @return TransformGroup incorporating the desired scale
    */
   private static TransformGroup scaleTransformGroup(Vector3d scaleFactors)
   {
      Transform3D t1 = new Transform3D();

      t1.setScale(scaleFactors);

      TransformGroup transGroup = new TransformGroup(t1);

      return transGroup;
   }

   /**
    * Adds a Shape3D with the specified geometry and appearance as a child of the given Group.
    *
    * @param geometry Geometry to use with the new shape
    * @param appearance Appearance for the new shape.  For implementations see {@link YoAppearance YoAppearance}
    * @param group The Group to which the shape will be added.
    */
   private static void addShapeToGroup(Geometry geometry, Appearance appearance, Group group)
   {
      Shape3D linkShape = new Shape3D();

      linkShape.setAppearance(appearance);
      linkShape.setGeometry(geometry);

      PickTool.setCapabilities(linkShape, PickTool.INTERSECT_FULL);
      group.addChild(linkShape);
   }

   /**
    * Adds the specified Shape3D as a child of the given group.
    *
    * @param shape Shape3D to be added
    * @param group Group to which the shape is added
    */
   private static void addShapeToGroup(Shape3D shape, Group group)
   {
      PickTool.setCapabilities(shape, PickTool.INTERSECT_FULL);
      group.addChild(shape);
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
      TransformGroup transGroup = translateTransformGroup(tx, ty, tz);
      lastGroup.addChild(transGroup);
      lastGroup = transGroup;

      linkGraphicsDefinition.addInstruction(new LinkGraphicsTranslate(tx, ty, tz));
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
      TransformGroup transGroup = translateTransformGroup(translation.x, translation.y, translation.z);
      lastGroup.addChild(transGroup);
      lastGroup = transGroup;
      linkGraphicsDefinition.addInstruction(new LinkGraphicsTranslate(translation));
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
      TransformGroup transGroup = rotateTransformGroup(rotAng, rotAxis);

      lastGroup.addChild(transGroup);
      lastGroup = transGroup;
      linkGraphicsDefinition.addInstruction(new LinkGraphicsRotateDefinedAxis(rotAng, rotAxis));
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
      TransformGroup transGroup = rotateTransformGroup(rotAng, rotAxis);

      lastGroup.addChild(transGroup);
      lastGroup = transGroup;
      linkGraphicsDefinition.addInstruction(new LinkGraphicsRotate(rotAng, rotAxis));
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
      if (rot == null)
         return;
      TransformGroup transGroup = rotateTransformGroup(rot);

      lastGroup.addChild(transGroup);
      lastGroup = transGroup;
      linkGraphicsDefinition.addInstruction(new LinkGraphicsRotateMatrix(rot));
   }

   /**
    * Scales the coordinate system by the specified scale factor. This does not scale existing
    * graphics, instead it scales the "current" coordinate system.  When another object is added
    * it will be uniformly scaled by the specified factor.
    *
    * @param scaleFactor Factor by which the coordinate system is scaled.  For example, 0.5 would
    * reduce future objects size by 50% whereas 2 would double it.
    */
   public void scale(double scaleFactor)
   {
      TransformGroup transGroup = scaleTransformGroup(scaleFactor);

      lastGroup.addChild(transGroup);
      lastGroup = transGroup;
      linkGraphicsDefinition.addInstruction(new LinkGraphicsScale(scaleFactor));
   }

   /**
    * Scales the coordinate system by the specified scale factor. This does not scale existing
    * graphics, instead it scales the "current" coordinate system.  When another object is added
    * it will be uniformly scaled by the specified factor.  The components of the vector indicate
    * scale factors in each dimension.
    *
    * @param scaleFactors Vector3d describing the scaling factors in each dimension.
    */
   public void scale(Vector3d scaleFactors)
   {
      TransformGroup transGroup = scaleTransformGroup(scaleFactors);

      lastGroup.addChild(transGroup);
      lastGroup = transGroup;
      linkGraphicsDefinition.addInstruction(new LinkGraphicsScale(scaleFactors));
   }

   /**
    * Resets the coordinate system to the joint origin.  This clears all rotations, translations,
    * and scale factors.
    */
   public void identity()
   {
      this.lastGroup = this.sharedGroup;
      linkGraphicsDefinition.addInstruction(new LinkGraphicsIdentity());
   }

   /**
    * Adds the given geometry with the specified appearance to the origin
    * of the current coordinate system.  The {@link YoAppearance YoAppearance}
    * and {@link GeometryGenerator GeometryGenerator} classes provide a number of basic appearances
    * and shapes respectively.  For more detailed information see the Java3d API.
    *
    * @param geometry Geometry of the new shape.
    * @param appearance Appearance of the new shape.
    */
   public Shape3D addShape(Geometry geometry, Appearance appearance)
   {
      Shape3D linkShape = new Shape3D();

      linkShape.setAppearance(appearance);
      linkShape.setGeometry(geometry);

      addShape(linkShape);

      linkShape.setCapability(Shape3D.ALLOW_APPEARANCE_WRITE);
      linkShape.setCapability(Shape3D.ALLOW_APPEARANCE_OVERRIDE_WRITE);

      return linkShape;

      // linkGraphicsDefinition.addInstruction();
   }

   /**
    * Adds the specified shape to the origin of the current coordinate system.  For
    * more information see the Java3d API.
    *
    * @param shape Shape3D to be added.
    */
   public void addShape(Shape3D shape)
   {
      addShapeToGroup(shape, this.lastGroup);
      this.numShapes++;

      // linkGraphicsDefinition.addInstruction();
   }

   /**
    * Adds the specified BranchGroup to the origin of the current coordinate system.
    * This group may contain a number of sub groups and graphics objects.  For
    * more information see the Java3d API.
    *
    * @param branchGroup BranchGroup to be added.
    */
   public void addBranchGroup(BranchGroup branchGroup)
   {
      this.lastGroup.addChild(branchGroup);
      this.numShapes++;

      // linkGraphicsDefinition.addInstruction();
   }

   /**
    * Adds the specified group to the origin of the current coordinate system.  This
    * group may contain a number of sub groups and graphics objects.  For more information
    * see the Java3d API.
    *
    * @param group Group to be added.
    */
   public void addGroup(Group group)
   {
      this.lastGroup.addChild(group);
      this.numShapes++;

      // linkGraphicsDefinition.addInstruction();
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

      // linkGraphicsDefinition.addInstruction(new LinkGraphicsAddVRMLFile(fileURL.getPath()));
   }

   /**
    * Adds the specified VRML file to the center of the current coordinate system
    * using the provided appearance.  VRML or Virtual Reality Modeling Language is
    * a standard file format describing 3d objects.  For more information google
    * VRML and read about it.
    *
    * @param fileURL URL describing the location of the VRML file.
    * @param app Appearance to use in the creation of this shape.
    */
   public void addVRMLFile(URL fileURL, Appearance app)
   {
      // Use with new xj3d stuff
      // int flag = org.web3d.j3d.loaders.VRML97Loader.LOAD_ALL; flag &= ~org.web3d.j3d.loaders.VRML97Loader.LOAD_BEHAVIOR_NODES; // Static Loads only
      // org.web3d.j3d.loaders.VRML97Loader loader = new org.web3d.j3d.loaders.VRML97Loader(flag);

      com.sun.j3d.loaders.vrml97.VrmlLoader loader = new com.sun.j3d.loaders.vrml97.VrmlLoader(); // Use with old x3d.jar

      com.sun.j3d.loaders.Scene model = null;

      try
      {
         model = loader.load(fileURL);
      }
      catch (Exception e)
      {
         System.out.println("VRML file not loaded");
      }

      BranchGroup vrmlGroup = model.getSceneGroup();

      // TransformGroup viewGroups = model.getViewGroups();
      // this.addTranfromGroup(viewGroups);
      // System.out.println(vrmlGroup);

      if (app != null)
         recursiveSetEnabling(vrmlGroup, app, 0);

      this.addBranchGroup(vrmlGroup);
      if (app != null)
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddVRMLFile(fileURL.getPath(), new AppearanceDefinition(getColor(app))));
      else
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddVRMLFile(fileURL.getPath()));

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
   public void addVRMLFile(String fileName, Appearance app)
   {
      // Use with new xj3d stuff
      // int flag = org.web3d.j3d.loaders.VRML97Loader.LOAD_ALL; flag &= ~org.web3d.j3d.loaders.VRML97Loader.LOAD_BEHAVIOR_NODES; // Static Loads only
      // org.web3d.j3d.loaders.VRML97Loader loader = new org.web3d.j3d.loaders.VRML97Loader(flag);

      com.sun.j3d.loaders.vrml97.VrmlLoader loader = new com.sun.j3d.loaders.vrml97.VrmlLoader(); // Use with old x3d.jar

      com.sun.j3d.loaders.Scene model = null;

      try
      {
         model = loader.load(fileName);
      }
      catch (Exception e)
      {
         System.out.println("VRML file " + fileName + " not loaded");

         return;
      }

      BranchGroup vrmlGroup = model.getSceneGroup();

      // TransformGroup viewGroups = model.getViewGroups();
      // this.addTranfromGroup(viewGroups);
      // System.out.println(vrmlGroup);

      if (app != null)
         recursiveSetEnabling(vrmlGroup, app, 0);

      this.addBranchGroup(vrmlGroup);
      if (app != null)
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddVRMLFile(fileName, new AppearanceDefinition(getColor(app))));
      else
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddVRMLFile(fileName));
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

      // linkGraphicsDefinition.addInstruction(new LinkGraphicsAddVRMLFile(fileName));
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

      // linkGraphicsDefinition.addInstruction(new LinkGraphicsAdd3DSFile(fileURL.getPath()));
   }

   /**
    * Adds the specified 3DS Max file to the center of the current coordinate system
    * with the given appearance.  3DS Max is a 3D modeling program that allows the creation
    * of detailed models and animations.  This function only imports the model allowing the use
    * of more complicated and detailed system representations in simulations.
    *
    * @param fileURL URL pointing to the desired 3ds file.
    * @param app Appearance to use with the 3ds model once imported.
    */
   public void addModelFile(URL fileURL, Appearance app)
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

      addModelFile(fileName, app);

      //    if (app != null)
      //       linkGraphicsDefinition.addInstruction(new LinkGraphicsAdd3DSFile(fileURL.getPath(), new AppearanceDefinition(getColor(app))));
      //    else
      //       linkGraphicsDefinition.addInstruction(new LinkGraphicsAdd3DSFile(fileURL.getPath()));
   }

   /*
    * public void add3DSFile2(URL fileURL, Appearance app) { Loader3DS loader =
    * new Loader3DS(); loader.setTextureLightingOn(); // turns on modulate mode
    * for textures (lighting)
    * 
    * 
    * String URLBase = fileURL.toString(); System.out.println(URLBase);
    * //URLBase = URLBase.substring(URLBase.lastIndexOf("/"));
    * 
    * URLBase = URLBase.substring(0, URLBase.lastIndexOf("/"));
    * 
    * System.out.println(URLBase); loader.setURLBase(URLBase);
    * 
    * com.sun.j3d.loaders.Scene scene = null;
    * 
    * try{scene = loader.load(fileURL);} catch(FileNotFoundException
    * e){System.err.println("File Not Found in add3DSFile: " + fileURL + "  " +
    * e);return;}
    * 
    * BranchGroup branchGroup = scene.getSceneGroup();
    * 
    * if (app != null) recursiveSetEnabling(branchGroup, app, 0);
    * this.addBranchGroup(scene.getSceneGroup()); }
    */

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

      // linkGraphicsDefinition.addInstruction(new LinkGraphicsAdd3DSFile(fileName));
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
   public void addModelFile(String fileName, Appearance app)
   {
      //Determine filename
      
      ModelFileType modelFileType = ModelFileType.getFileType(fileName);
      Loader loader;
      switch (modelFileType)
      {
      case COLLADA:
         loader = new DAELoader();
         break;
      case _STL:
         loader = new STLLoader();
         fileName = "file://" + fileName;
         break;
      case _3DS:
         Loader3DS loader3ds = new Loader3DS();
         loader3ds.setTextureLightingOn(); // turns on modulate mode for textures (lighting)
         loader = loader3ds;
         break;
      default:
         throw new RuntimeException("Unkown filetype: " + modelFileType);
      }
      
      com.sun.j3d.loaders.Scene scene = null;

      try
      {
         scene = loader.load(fileName);
      }
      catch (FileNotFoundException e)
      {
         System.err.println("File Not Found in addModelFile: " + fileName + "  " + e);

         return;
      }

      // Rotate to make sure z up corresponds to 3d studio max:
      TransformGroup transGroup = rotateTransformGroup(Math.PI / 2.0, Link.X);

      BranchGroup branchGroup = scene.getSceneGroup();
      transGroup.addChild(branchGroup);

      if (app != null)
         recursiveSetEnabling(branchGroup, app, 0);

      // this.addBranchGroup(branchGroup);

      this.addGroup(transGroup);
      if (app != null)
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddModelFile(fileName, new AppearanceDefinition(getColor(app))));
      else
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddModelFile(fileName));
   }

   public void addCoordinateSystem(double length)
   {
      addCoordinateSystem(length, YoAppearance.Black());
   }

   public void addCoordinateSystem(double length, Appearance arrowAppearance)
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
   public void addCoordinateSystem(double length, Appearance xAxisAppearance, Appearance yAxisAppearance, Appearance zAxisAppearance, Appearance arrowAppearance)
   {
      Geometry bar = GeometryGenerator.Cylinder(length / 32.0, length, 15);
      Geometry arrow = GeometryGenerator.Cone(length / 10.0, length / 15.0, 15);

      BranchGroup base = new BranchGroup();

      TransformGroup yAxisGroup = rotateTransformGroup(-Math.PI / 2.0, Link.X);
      TransformGroup xAxisGroup = rotateTransformGroup(Math.PI / 2.0, Link.Y);

      addShapeToGroup(bar, xAxisAppearance, xAxisGroup);
      addShapeToGroup(bar, yAxisAppearance, yAxisGroup);
      addShapeToGroup(bar, zAxisAppearance, base);

      base.addChild(xAxisGroup);
      base.addChild(yAxisGroup);

      TransformGroup transZGroup1 = translateTransformGroup(0.0, 0.0, length);
      TransformGroup transZGroup2 = translateTransformGroup(0.0, 0.0, length);
      TransformGroup transZGroup3 = translateTransformGroup(0.0, 0.0, length);

      addShapeToGroup(arrow, arrowAppearance, transZGroup1);
      addShapeToGroup(arrow, arrowAppearance, transZGroup2);
      addShapeToGroup(arrow, arrowAppearance, transZGroup3);

      base.addChild(transZGroup1);
      xAxisGroup.addChild(transZGroup2);
      yAxisGroup.addChild(transZGroup3);

      addBranchGroup(base);
      linkGraphicsDefinition.addInstruction(new LinkGraphicsAddCoordinateSystem(length));
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

      // linkGraphicsDefinition.addInstruction(new LinkGraphicsAddCube(lx, ly, lz));
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
   public void addCube(double lx, double ly, double lz, Appearance cubeApp)
   {
      Geometry cubeGeom = GeometryGenerator.Cube(lx, ly, lz);
      addShape(cubeGeom, cubeApp);

      if (cubeApp != null)
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddCube(lx, ly, lz, new AppearanceDefinition(getColor(cubeApp))));
      else
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddCube(lx, ly, lz));
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

      // linkGraphicsDefinition.addInstruction(new LinkGraphicsAddWedge(lx, ly, lz));
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
   public void addWedge(double lx, double ly, double lz, Appearance wedgeApp)
   {
      Geometry wedgeGeom = GeometryGenerator.Wedge(lx, ly, lz);
      addShape(wedgeGeom, wedgeApp);
      if (wedgeApp != null)
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddWedge(lx, ly, lz, new AppearanceDefinition(getColor(wedgeApp))));
      else
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddWedge(lx, ly, lz));
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

      //    linkGraphicsDefinition.addInstruction(new LinkGraphicsAddSphere(radius));
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
   public void addSphere(double radius, Appearance sphereApp)
   {
      Geometry sphereGeom = GeometryGenerator.Sphere(radius, 15, 15);
      addShape(sphereGeom, sphereApp);
      if (sphereApp != null)
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddSphere(radius, new AppearanceDefinition(getColor(sphereApp))));
      else
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddSphere(radius));
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

      // linkGraphicsDefinition.addInstruction(new LinkGraphicsAddEllipsoid(xRad, yRad, zRad));
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
   public void addEllipsoid(double xRad, double yRad, double zRad, Appearance ellipsoidApp)
   {
      Geometry ellipsoidGeom = GeometryGenerator.Ellipsoid(xRad, yRad, zRad, 15, 15);
      addShape(ellipsoidGeom, ellipsoidApp);
      if (ellipsoidApp != null)
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddEllipsoid(xRad, yRad, zRad, new AppearanceDefinition(getColor(ellipsoidApp))));
      else
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddEllipsoid(xRad, yRad, zRad));
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

      // linkGraphicsDefinition.addInstruction(new LinkGraphicsAddCylinder(height, radius));
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
   public void addCylinder(double height, double radius, Appearance cylApp)
   {
      Geometry cylGeom = GeometryGenerator.Cylinder(radius, height, 15);
      addShape(cylGeom, cylApp);
      if (cylApp != null)
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddCylinder(height, radius, new AppearanceDefinition(getColor(cylApp))));
      else
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddCylinder(height, radius));
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

      // linkGraphicsDefinition.addInstruction(new LinkGraphicsAddCone(height, radius));
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
   public void addCone(double height, double radius, Appearance coneApp)
   {
      Geometry coneGeom = GeometryGenerator.Cone(height, radius, 15);
      addShape(coneGeom, coneApp);
      if (coneApp != null)
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddCone(height, radius, new AppearanceDefinition(getColor(coneApp))));
      else
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddCone(height, radius));
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

      // linkGraphicsDefinition.addInstruction(new LinkGraphicsAddTruncatedCone(height, bx, by, tx, ty));
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
   public void addGenTruncatedCone(double height, double bx, double by, double tx, double ty, Appearance coneApp)
   {
      Geometry genTruncatedConeGeom = GeometryGenerator.GenTruncatedCone(height, bx, by, tx, ty, 15);
      addShape(genTruncatedConeGeom, coneApp);
      if (coneApp != null)
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddTruncatedCone(height, bx, by, tx, ty, new AppearanceDefinition(getColor(coneApp))));
      else
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddTruncatedCone(height, bx, by, tx, ty));
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

      // linkGraphicsDefinition.addInstruction(new LinkGraphicsAddHemiEllipsoid(xRad, yRad, zRad));
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
   public void addHemiEllipsoid(double xRad, double yRad, double zRad, Appearance hEApp)
   {
      Geometry hEGeom = GeometryGenerator.HemiEllipsoid(xRad, yRad, zRad, 16, 16);
      addShape(hEGeom, hEApp);
      if (hEApp != null)
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddHemiEllipsoid(xRad, yRad, zRad, new AppearanceDefinition(getColor(hEApp))));
      else
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddHemiEllipsoid(xRad, yRad, zRad));
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
      // linkGraphicsDefinition.addInstruction(new LinkGraphicsAddArcTorus(startAngle, endAngle, majorRadius, minorRadius));
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
   public void addArcTorus(double startAngle, double endAngle, double majorRadius, double minorRadius, Appearance arcTorusApp)
   {
      // System.out.println("start: " + startAngle + ", end: " + endAngle + ", MR: " + majorRadius + ", mr: " + minorRadius);
      Geometry arcTorusGeom = GeometryGenerator.ArcTorus(startAngle, endAngle, majorRadius, minorRadius, 15);

      // Geometry arcTorusGeom = GeometryGenerator.Sphere(0.2f,15,15);
      addShape(arcTorusGeom, arcTorusApp);
      if (arcTorusApp != null)
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddArcTorus(startAngle, endAngle, majorRadius, minorRadius, new AppearanceDefinition(
               getColor(arcTorusApp))));
      else
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddArcTorus(startAngle, endAngle, majorRadius, minorRadius));
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

      // linkGraphicsDefinition.addInstruction(new LinkGraphicsAddPyramidCube(lx, ly, lz, lh));
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
   public void addPyramidCube(double lx, double ly, double lz, double lh, Appearance cubeApp)
   {
      Geometry cubeGeom = GeometryGenerator.PyramidCube(lx, ly, lz, lh);
      addShape(cubeGeom, cubeApp);
      if (cubeApp != null)
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddPyramidCube(lx, ly, lz, lh, new AppearanceDefinition(getColor(cubeApp))));
      else
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddPyramidCube(lx, ly, lz, lh));
   }

   /**
    * Creates a polygon centered at the current coordinate system with the given vertices.
    * The points this shape is composed of must be coplanar and the order matters.  Randomly
    * inserting points will produce unpredictable results, clockwise direction determines the
    * side that is drawn.
    *
    * @param polygonPoint Point3f array containing the desired points.
    */
   public Shape3D addPolygon(Point3f[] polygonPoint)
   {
      return addPolygon(polygonPoint, YoAppearance.Black());

      // linkGraphicsDefinition.addInstruction(new LinkGraphicsAddPolygonFloat(polygonPoint));
   }

   /**
    * Creates a polygon centered at the current coordinate system with the given vertices.
    * The points this shape is composed of must be coplanar and the order matters.  Randomly
    * inserting points will produce unpredictable results, clockwise direction determines the
    * side that is drawn.
    *
    * @param polygonPoints Point3f array containing the desired points.
    * @param appearance Appearance to be used with the new polygon.  See {@link YoAppearance YoAppearance} for implementations.
    */
   public Shape3D addPolygon(Point3f[] polygonPoints, Appearance appearance)
   {
      Geometry geometry = GeometryGenerator.Polygon(polygonPoints);
      Shape3D shape = addShape(geometry, appearance);
      if (appearance != null)
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddPolygonFloat(polygonPoints, new AppearanceDefinition(getColor(appearance))));
      else
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddPolygonFloat(polygonPoints));

      return shape;
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
   public void addPolygon(ArrayList<Point3d> polygonPoints, Appearance appearance)
   {
      PolygonAttributes polyAttributes = new PolygonAttributes();
      polyAttributes.setCullFace(PolygonAttributes.CULL_NONE);

      // polyAttributes.setCullFace(PolygonAttributes.CULL_BACK);
      appearance.setPolygonAttributes(polyAttributes);

      Geometry geometry = GeometryGenerator.Polygon(polygonPoints);
      addShape(geometry, appearance);
      if (appearance != null)
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddPolygonDouble(polygonPoints, new AppearanceDefinition(getColor(appearance))));
      else
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddPolygonDouble(polygonPoints));
   }

   /**
    * Creates a polygon centered at the current coordinate system with the given vertices.
    * The points this shape is composed of must be coplanar and in a logical order.  Randomly
    * inserting points will produce unpredictable results, clockwise direction determines the
    * side that is drawn.
    *
    * @param polygonPoint Array containing Point3d's to be used when generating the shape.
    */
   public Shape3D addPolygon(Point3d[] polygonPoint)
   {
      return addPolygon(polygonPoint, YoAppearance.Black());

      // linkGraphicsDefinition.addInstruction(new LinkGraphicsAddPolygonDouble(polygonPoint));
   }

   /**
    * Creates a polygon centered at the current coordinate system with the given vertices.
    * The points this shape is composed of must be coplanar and the order matters.  Randomly
    * inserting points will produce unpredictable results, clockwise direction determines the
    * side that is drawn.
    *
    * @param polygonPoints Array containing the points
    * @param appearance Appearance to be used with the new polygon.  See {@link YoAppearance YoAppearance} for implementations.
    */
   public Shape3D addPolygon(Point3d[] polygonPoints, Appearance appearance)
   {
      PolygonAttributes polyAttributes = new PolygonAttributes();
      polyAttributes.setCullFace(PolygonAttributes.CULL_NONE);

      // polyAttributes.setCullFace(PolygonAttributes.CULL_BACK);
      appearance.setPolygonAttributes(polyAttributes);

      Geometry geometry = GeometryGenerator.Polygon(polygonPoints);
      Shape3D shape = addShape(geometry, appearance);
      if (appearance != null)
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddPolygonDouble(polygonPoints, new AppearanceDefinition(getColor(appearance))));
      else
         linkGraphicsDefinition.addInstruction(new LinkGraphicsAddPolygonDouble(polygonPoints));

      return shape;
   }

   public void createInertiaEllipsoid(Link link, Appearance appearance)
   {
      //    LinkGraphics linkGraphics = link.getLinkGraphics();
      this.identity();
      Vector3d comOffSet = new Vector3d();
      link.getComOffset(comOffSet);
      this.translate(comOffSet);
      Matrix3d momentOfInertia = new Matrix3d();
      link.getMomentOfInertia(momentOfInertia);
      double mass = link.getMass();

      Vector3d principalMomentsOfInertia = new Vector3d(momentOfInertia.m00, momentOfInertia.m11, momentOfInertia.m22);

      Vector3d ellipsoidRadii = InertiaTools.getInertiaEllipsoidRadii(principalMomentsOfInertia, mass);

      this.addEllipsoid(ellipsoidRadii.x, ellipsoidRadii.y, ellipsoidRadii.z, appearance);

      comOffSet.scale(-1.0);
      this.translate(comOffSet); // translate back
   }

   /*
    * public String getName() { return this.name; }
    */

   /*
    * public int getNumShapes() { return this.numShapes; }
    */

   /*
    * public BranchGroup getBranchGroup() { return linkBG; }
    */

   /**
    * Retrieves the shared group containing all of the graphical elements for this link.  This can be used to
    * make a copy or add the link graphically to another component.
    *
    * @return SharedGroup for this link.
    */
   public SharedGroup getSharedGroup()
   {
      return sharedGroup;
   }

}
