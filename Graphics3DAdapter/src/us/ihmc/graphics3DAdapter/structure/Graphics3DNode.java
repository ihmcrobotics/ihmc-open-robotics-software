package us.ihmc.graphics3DAdapter.structure;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.input.ModifierKeyInterface;
import us.ihmc.graphics3DAdapter.input.SelectedListener;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.math.geometry.RotationFunctions;

public class Graphics3DNode
{
   private static final Graphics3DNodeType DEFAULT_NODE_TYPE = Graphics3DNodeType.JOINT;
   private final String name;
   private final Graphics3DNodeType nodeType;
   private final Transform3D transform = new Transform3D();

   private Graphics3DObject graphicsObject;
   private boolean hasGraphicsObjectChanged = false;

   private final ArrayList<Graphics3DNode> childeren = new ArrayList<Graphics3DNode>();
   private final ArrayList<SelectedListener> selectedListeners = new ArrayList<SelectedListener>();

   public Graphics3DNode(String name, Graphics3DNodeType nodeType, Graphics3DObject graphicsObject)
   {
      this.name = name;
      this.nodeType = nodeType;

      if (graphicsObject != null)
      {
         setGraphicsObject(graphicsObject);
      }
   }

   public Graphics3DNode(String name, Graphics3DObject graphicsObject)
   {
      this(name, DEFAULT_NODE_TYPE, graphicsObject);
   }

   public Graphics3DNode(String name, Graphics3DNodeType nodeType)
   {
      this(name, nodeType, null);
   }

   public Graphics3DNode(String name)
   {
      this(name, DEFAULT_NODE_TYPE, null);
   }

   public synchronized Transform3D getTransform()
   {
      return transform;
   }

   public synchronized void setTransform(Transform3D transform)
   {
      assert(!RotationFunctions.isNaNorInf(transform));
      this.transform.set(transform);
   }

   public void translate(double distance, Axis axis)
   {
      if (axis == Axis.X)
      {
         translate(distance, 0, 0);
      }
      else if (axis == Axis.Y)
      {
         translate(0, distance, 0);
      }
      else if (axis == Axis.Z)
      {
         translate(0, 0, distance);
      }
   }

   public void translate(double x, double y, double z)
   {
      Transform3D translator = new Transform3D();
      translator.set(new Vector3d(x, y, z));

      transform.mul(translator);
   }

   public void rotate(double angle, Axis axis)
   {
      Transform3D rotator = new Transform3D();

      if (axis == Axis.X)
      {
         rotator.rotX(angle);
      }
      else if (axis == Axis.Y)
      {
         rotator.rotY(angle);
      }
      else if (axis == Axis.Z)
      {
         rotator.rotZ(angle);
      }

      transform.mul(rotator);
   }

   public Vector3d getTranslation()
   {
      Vector3d translation = new Vector3d();
      getTransform().get(translation);

      return translation;
   }

   public void addChild(Graphics3DNode child)
   {
      synchronized (childeren)
      {
         childeren.add(child);
      }
   }

   public List<Graphics3DNode> getChildrenNodes()
   {
      synchronized (childeren)
      {
         return Collections.unmodifiableList(childeren);
      }
   }

   public Graphics3DObject getGraphics3DObject()
   {
      return graphicsObject;
   }

   public synchronized Graphics3DObject getGraphicsObjectAndResetHasGraphicsObjectChanged()
   {
      Graphics3DObject ret = graphicsObject;
      setHasGraphicsObjectChanged(false);

      return ret;
   }

   public synchronized void setHasGraphicsObjectChanged(boolean hasGraphicsObjectChanged)
   {
      this.hasGraphicsObjectChanged = hasGraphicsObjectChanged;
   }

   public boolean getHasGraphicsObjectChanged()
   {
      return hasGraphicsObjectChanged;
   }

   public void setGraphicsObject(Graphics3DObject graphicsObject)
   {
      this.graphicsObject = graphicsObject;
      setHasGraphicsObjectChanged(true);
   }

   public String getName()
   {
      return name;
   }

   public Graphics3DNodeType getNodeType()
   {
      return nodeType;
   }

   public void notifySelectedListeners(ModifierKeyInterface modifierKeys, Point3d location, Point3d cameraPosition, Quat4d cameraRotation)
   {
      for (SelectedListener selectedListener : selectedListeners)
      {
         selectedListener.selected(this, modifierKeys, location, cameraPosition, cameraRotation);
      }

      graphicsObject.notifySelectedListeners(this, modifierKeys, location, cameraPosition, cameraRotation);
   }

   public void addSelectedListener(SelectedListener selectedListener)
   {
      selectedListeners.add(selectedListener);
   }
}
