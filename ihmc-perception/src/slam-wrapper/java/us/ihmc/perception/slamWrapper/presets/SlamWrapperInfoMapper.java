package us.ihmc.perception.slamWrapper.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;


@Properties(value = {
        @Platform(
                compiler = "cpp17",
                define = {"UNIQUE_PTR_NAMESPACE std", "SHARED_PTR_NAMESPACE std"},
                include = {"FactorGraphExternal.h"},
                link = {"slam-wrapper"},
                preload = "jniSlamWrapper"
        ),
        @Platform(
                value = "linux",
                includepath = "../include",
                linkpath = "../lib",
                preload = {
                        "tbb",
                        "boost_filesystem",
                        "boost_chrono",
                        "boost_timer",
                        "boost_serialization",
                        "gtsam",
                        "slam-wrapper",
                        "jniSlamWrapper"
                }),
        @Platform(
                value = "windows",
                includepath = {
                        "..\\include",
                        "C:\\Program Files (x86)\\Eigen3\\include\\eigen3",
                        "C:\\Program Files\\GTSAM\\include",
                        "C:\\local\\boost_1_74_0"
                },
                linkpath = {
                        "..\\lib",
                        "C:\\Program Files\\GTSAM\\lib",
                        "C:\\local\\boost_1_74_0\\lib64-msvc-14.2"
                },
                link = {"slam-wrapper", "gtsam"},
                preload = {"gtsam", "slam-wrapper", "jniSlamWrapper"})
},
        target = "us.ihmc.perception.slamWrapper.SlamWrapper"
)
public class SlamWrapperInfoMapper implements InfoMapper
{
   public void map(InfoMap infoMap)
   {
   }
}
