package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import java.util.ArrayList;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;

/**
 * Created with IntelliJ IDEA.
 * User: pneuhaus
 * Date: 5/14/13
 * Time: 1:36 PM
 * To change this template use File | Settings | File Templates.
 */
public class ScriptObject
{
   private long timeStamp;
   private Object scriptObject;

   public ScriptObject(long timeStamp, Object scriptObject)
   {
      this.timeStamp = timeStamp;
      this.scriptObject = scriptObject;
   }

   public long getTimeStamp()
   {
      return timeStamp;
   }

   public void setTimeStamp(long timeStamp)
   {
      this.timeStamp = timeStamp;
   }

   public Object getScriptObject()
   {
      return scriptObject;
   }

   @SuppressWarnings("rawtypes")
   public void applyTransform(RigidBodyTransform transform3D)
   {
      if (scriptObject instanceof TransformableDataObject)
      {
         this.scriptObject = ((TransformableDataObject) scriptObject).transform(transform3D);
      }
   }

   public static String getListInfo(ArrayList<ScriptObject> listOfScriptObjects)
   {
      String ret = "";
      for (ScriptObject scriptObject : listOfScriptObjects)
      {
         ret = ret + scriptObject.toString() + "\n";
      }

      return ret;
   }

   public String toString()
   {
      return (this.getScriptObject().toString());
   }
}
