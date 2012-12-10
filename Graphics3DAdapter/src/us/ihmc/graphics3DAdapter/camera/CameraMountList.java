package us.ihmc.graphics3DAdapter.camera;


import java.util.ArrayList;

import us.ihmc.graphics3DAdapter.camera.TransformToScreenHolder;


public class CameraMountList implements java.io.Serializable
{
   private static final long serialVersionUID = 7819849315544602348L;
   private ArrayList<TransformToScreenHolder> mounts = new ArrayList<TransformToScreenHolder>();

   public CameraMountList()
   {
   }

   public void addCameraMount(TransformToScreenHolder mount)
   {
      mounts.add(mount);
   }

   public void addCameraMounts(ArrayList<TransformToScreenHolder> mountArrayList)
   {
      mounts.addAll(mountArrayList);
   }


   public TransformToScreenHolder getCameraMount(String name)
   {
      for (int i = 0; i < mounts.size(); i++)
      {
         TransformToScreenHolder mount = mounts.get(i);

         if (mount.getName().equals(name))
         {
            return mount;
         }
      }

      return null;
   }
}

