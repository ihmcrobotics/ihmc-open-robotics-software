package us.ihmc.jMonkeyEngineToolkit.camera;


import java.util.ArrayList;


public class CameraMountList implements java.io.Serializable
{
   private static final long serialVersionUID = 7819849315544602348L;
   private ArrayList<CameraMountInterface> mounts = new ArrayList<CameraMountInterface>();

   public CameraMountList()
   {
   }

   public void addCameraMount(CameraMountInterface mount)
   {
      mounts.add(mount);
   }

   public void addCameraMounts(ArrayList<CameraMountInterface> mountArrayList)
   {
      mounts.addAll(mountArrayList);
   }


   public CameraMountInterface getCameraMount(String name)
   {
      for (int i = 0; i < mounts.size(); i++)
      {
         CameraMountInterface mount = mounts.get(i);

         if (mount.getName().equals(name))
         {
            return mount;
         }
      }

      return null;
   }

   public ArrayList<CameraMountInterface> getCameraMountList()
   {
      return mounts;
   }
}

