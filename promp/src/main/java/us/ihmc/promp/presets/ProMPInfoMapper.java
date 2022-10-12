package us.ihmc.promp.presets;

import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.Loader;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.annotation.*;
import org.bytedeco.javacpp.tools.Info;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(value = {
      @Platform(
            compiler = "cpp17",
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
            value = "windows-x86_64",
            includepath = {"..\\include", "C:\\Program Files (x86)\\Eigen3\\include\\eigen3"},
            linkpath = "..\\")
      },
      target = "us.ihmc.promp",
      global = "us.ihmc.promp.global.promp"
)
public class ProMPInfoMapper implements InfoMapper
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
      infoMap.put(new Info("PROMPEXPORT", "PROMPCALL").cppTypes().annotations());
   }

   /*
    *    Eigen matrix example
    *
    *    Eigen::MatrixXd m(2,2);
    *    m(0,0) = 3;
    *
    *    m(0,0) is a functor which returns a Scalar& which has an operator= overload;
    *    so we can directly assign a value, ie 3
    *
    *    See: Eigen/src/Core/DenseCoeffsBase.h:362
    */

   @Name("Eigen::DenseBase<Eigen::MatrixXd>::Scalar")
   public static class EigenDoubleScalar extends Pointer
   {
      public EigenDoubleScalar(double value)
      {
         allocate(value);
      }

      private native void allocate(double value);

      public native @Name("operator =") void put(double value);
   }

   @Name("Eigen::MatrixXd")
   public static class EigenMatrixXd extends Pointer
   {
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

      public native @Name("operator ()") @ByRef EigenDoubleScalar apply(int rowId, int colId);

      public void debugPrint(String name)
      {
         System.out.println(name);

         for (int row = 0; row < rows(); row++)
         {
            for (int col = 0; col < cols(); col++)
            {
               System.out.print(coeff(row, col) + " ");
            }
            System.out.println();
         }
      }
   }

   @Name("Eigen::VectorXd")
   public static class EigenVectorXd extends Pointer
   {
      public EigenVectorXd(int length)
      {
         allocate(length);
      }

      private native void allocate(int length);

      // Calls data() on the C++ side
      public native DoublePointer data();

      // Calls size() on the C++ side
      public native long size();

      public native @Name("operator ()") @ByRef EigenDoubleScalar apply(int id);
   }
}
