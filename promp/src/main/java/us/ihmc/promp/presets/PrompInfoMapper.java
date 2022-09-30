package us.ihmc.promp.presets;

import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.Loader;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.annotation.Name;
import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.Info;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(value = {
      @Platform(
            value = {"linux", "windows", "macosx"},
            compiler = "cpp11",
            define = {"UNIQUE_PTR_NAMESPACE std", "SHARED_PTR_NAMESPACE std"},
            include = {"promp/trajectory.hpp",
                       "promp/trajectory_group.hpp",
                       "promp/promp.hpp",},
            link = "promp"),
      @Platform(
            value = "linux",
            includepath = "../include",
            linkpath = "../lib"),
      @Platform(
            value = "windows",
            includepath = "../../include",
            linkpath = "../Release")
      },
      target = "us.ihmc.promp",
      global = "us.ihmc.promp.global.promp"
)
public class PrompInfoMapper implements InfoMapper
{
   static
   {
      Loader.checkVersion("org.bytedeco", "promp");
   }

   public void map(InfoMap infoMap)
   {
      infoMap.put(new Info("promp::TrajectoryGroup::normalize_length").javaText("public native long normalize_length();"));
      // For some reason, normalize_length(size_t) isn't treated as a method when javacpp generates the C++ jni code
      // It's referenced like: ptr->normalize_length;
      // Which obviously can't compile, so for now, we skip it
      infoMap.put(new Info("promp::TrajectoryGroup::normalize_length(size_t)").skip());
      infoMap.put(new Info("Eigen::MatrixXd").pointerTypes("EigenMatrixXd"));
      infoMap.put(new Info("Eigen::VectorXd").pointerTypes("EigenVectorXd"));
      infoMap.put(new Info("std::tuple<int,Eigen::VectorXd,Eigen::MatrixXd>").pointerTypes("IntVectorMatrixTuple").define());
      infoMap.put(new Info("std::pair<Eigen::VectorXd,Eigen::VectorXd>").pointerTypes("VectorVectorPair").define());
      infoMap.put(new Info("std::vector<std::string>").pointerTypes("StringVector").define());
      infoMap.put(new Info("std::vector<size_t>").pointerTypes("SizeTVector").define());
      infoMap.put(new Info("std::vector<promp::Trajectory>").pointerTypes("TrajectoryVector").define());
      infoMap.put(new Info("PROMPEXPORT").cppTypes().annotations());
   }

   @Name("Eigen::MatrixXd")
   public static class EigenMatrixXd extends Pointer
   {
      static
      {
         Loader.load();
      }

      public EigenMatrixXd(int rows, int cols)
      {
         allocate(rows, cols);
      }

      private native void allocate(int rows, int cols);

      // Calls data() on the C++ side
      public native DoublePointer data();

      // Calls rows() on the C++ side
      public native long rows();

      // Calls cols() on the C++ side
      public native long cols();

      // Calls coeff(Index rowId, Index colId) on the C++ side
      public native double coeff(int rowId, int colId);
   }

   @Name("Eigen::VectorXd")
   public static class EigenVectorXd extends Pointer
   {
      static
      {
         Loader.load();
      }

      public EigenVectorXd(int length)
      {
         allocate(length);
      }

      private native void allocate(int length);

      // Calls data() on the C++ side
      public native DoublePointer data();

      // Calls size() on the C++ side
      public native long size();
   }
}
